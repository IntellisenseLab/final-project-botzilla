import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
import numpy as np
import os
from ultralytics import YOLO

current_dir = os.path.dirname(__file__)

file_path = os.path.abspath(
    os.path.join(
        current_dir,
        "..", "..",
        "datasets",
        "runs",
        "detect",
        "train",
        "weights",
        "best.pt"
    )
)

# Kinect minimum sensing range (objects closer become 0 or invalid)
KINECT_MIN_RANGE_M = 0.55

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = cv_bridge.CvBridge()
        self.latest_depth = None  # Raw float32 depth frame (480x640), values in mm

        # Subscribe to the Kinect RGB stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Subscribe to the Kinect Depth stream
        self.subscription_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # Publish annotated image for debugging in rqt_image_view
        self.publisher_annotated = self.create_publisher(Image, '/perception/yolo_image', 10)

        # Publish cube position to brain_node: x=normalized horizontal, z=distance in meters
        self.cube_pub = self.create_publisher(Point, 'detected_cube', 10)

        # Load YOLO model
        self.model = YOLO(file_path)

        self.get_logger().info('YOLO Perception Node Initialized. Waiting for video stream...')

    def depth_callback(self, msg):
        """Store the latest depth frame for use in image_callback."""
        try:
            # mono8 encoding from kinect_bridge: depth scaled to 0-255 (lossy)
            # Use passthrough to keep raw bytes for index-based access
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth = depth_image
        except Exception as e:
            self.get_logger().error(f'Depth decode error: {e}')

    def get_depth_at(self, cx, cy, depth_img):
        """
        Sample the depth around a bounding box center in a small patch to avoid noise.
        Returns distance in meters, or None if invalid.
        """
        h, w = depth_img.shape[:2]
        # Clamp coordinates to image bounds
        cx = max(2, min(cx, w - 3))
        cy = max(2, min(cy, h - 3))

        # Sample a 5x5 patch and take the median non-zero value
        patch = depth_img[cy - 2:cy + 3, cx - 2:cx + 3].flatten().astype(np.float32)
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None

        # kinect_bridge encodes depth as mono8: values 0-255 scale.
        # To map back to meters: the Kinect raw depth is ~0-4000 mm, scaled to uint8 as (raw/4000*255)
        # Reverse: meters = (median_value / 255.0) * 4.0
        raw_val = np.median(valid)
        distance_m = (raw_val / 255.0) * 4.0
        return distance_m

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_h, img_w = cv_image.shape[:2]
            img_center_x = img_w / 2.0

            # Run YOLO inference
            results = self.model.predict(source=cv_image, conf=0.5, verbose=False)

            annotated_image = results[0].plot()

            best_cube = None  # (distance_m, normalized_x)
            best_dist = float('inf')

            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Normalized horizontal offset: -1.0 (far left) to +1.0 (far right)
                    norm_x = (cx - img_center_x) / img_center_x

                    # Get depth at bounding box center
                    dist_m = None
                    if self.latest_depth is not None:
                        dist_m = self.get_depth_at(cx, cy, self.latest_depth)

                    # If depth is invalid/too close, mark the cube as "captured" range
                    if dist_m is None or dist_m < KINECT_MIN_RANGE_M:
                        dist_m = 0.0  # Signal to brain: cube is in blind spot (already captured vicinity)

                    # Pick the closest cube
                    if dist_m < best_dist or (dist_m == 0.0 and best_cube is None):
                        best_dist = dist_m
                        best_cube = (dist_m, norm_x)

                    # Draw depth overlay on annotated image
                    label = f"{dist_m:.2f}m" if dist_m > 0 else "< 0.55m"
                    cv2.putText(annotated_image, label, (cx - 20, int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

            # Publish the best (closest) detected cube
            if best_cube is not None:
                cube_msg = Point()
                cube_msg.x = best_cube[1]    # normalized horizontal offset
                cube_msg.y = 0.0             # unused
                cube_msg.z = best_cube[0]    # distance in meters (0.0 = blind-spot/captured range)
                self.cube_pub.publish(cube_msg)

            # Publish annotated image for debugging
            ros_annotated = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            self.publisher_annotated.publish(ros_annotated)

            cv2.imshow("YOLO Debug View", annotated_image)
            cv2.waitKey(1)

        except cv_bridge.CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()