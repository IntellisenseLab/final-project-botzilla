import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
from ultralytics import YOLO
import os

# --------------------------------------------------------------------------
# Path to trained weights — adjust if needed
# --------------------------------------------------------------------------
_DEFAULT_WEIGHTS = os.path.join(
    os.path.dirname(__file__),
    '..', '..', '..', '..', '..', '..',     # walk up to repo root
    'runs', 'detect', 'train4', 'weights', 'best.pt'
)

class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_node')

        # --- Parameters (override at launch with --ros-args -p weights:=<path>) ---
        self.declare_parameter('weights', _DEFAULT_WEIGHTS)
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('show_window', True)   # set False on Pi (no display)

        weights    = self.get_parameter('weights').value
        self.conf  = self.get_parameter('confidence').value
        self.show  = self.get_parameter('show_window').value

        # --- Load YOLO model ---
        weights = os.path.realpath(weights)
        if not os.path.isfile(weights):
            self.get_logger().error(f'Weights file not found: {weights}')
            raise FileNotFoundError(weights)

        self.model = YOLO(weights)
        self.get_logger().info(f'YOLO model loaded from: {weights}')

        # --- CV Bridge ---
        self.bridge = cv_bridge.CvBridge()

        # --- Subscriber: RGB frames from Kinect or Webcam bridge ---
        self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # --- Publishers ---
        # Annotated image for rqt / rviz2 debugging
        self.pub_annotated = self.create_publisher(Image, '/perception/yolo_image', 10)
        # Detected cube centre in pixel coords (x, y) + confidence (z)
        self.pub_target = self.create_publisher(Point, '/perception/cube_target', 10)

        self.get_logger().info('YOLO Perception Node ready — waiting for frames on /camera/rgb/image_raw ...')

    # ------------------------------------------------------------------
    def image_callback(self, msg: Image):
        try:
            # 1. ROS Image → OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. Run inference
            results = self.model.predict(source=cv_image, conf=self.conf, verbose=False)

            # 3. Pick the highest-confidence detection (if any)
            best_box   = None
            best_conf  = 0.0

            for result in results:
                for box in result.boxes:
                    c = float(box.conf[0])
                    if c > best_conf:
                        best_conf = c
                        best_box  = box

            if best_box is not None:
                x1, y1, x2, y2 = map(int, best_box.xyxy[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # Publish pixel-space target (z = confidence)
                target_msg = Point()
                target_msg.x = cx
                target_msg.y = cy
                target_msg.z = best_conf
                self.pub_target.publish(target_msg)

                self.get_logger().debug(
                    f'Cube detected @ ({cx:.0f}, {cy:.0f})  conf={best_conf:.2f}'
                )

            # 4. Publish annotated image
            annotated = results[0].plot()
            ros_annotated = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            ros_annotated.header = msg.header
            self.pub_annotated.publish(ros_annotated)

            # 5. Optional local debug window (disable on Pi)
            if self.show:
                cv2.imshow('YOLO Debug View', annotated)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'image_callback error: {e}')

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.show:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()