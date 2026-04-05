import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import cv_bridge
import cv2
# from ultralytics import YOLO # Coworker will uncomment this

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        self.bridge = cv_bridge.CvBridge()
        
        # Subscribe to the Kinect RGB stream using Sensor Data QoS (Best Effort) to match the bridge
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Optional: Publisher for viewing the YOLO bounding boxes in RQT
        self.publisher_annotated = self.create_publisher(Image, '/perception/yolo_image', 10)
        
        # Load Model (Coworker will add path to weights here)
        # self.model = YOLO('best.pt') 
        
        self.get_logger().info('YOLO Perception Node Initialized. Waiting for video stream...')

    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image message to OpenCV format (BGR for YOLO/OpenCV)
            # Note: Kinect sends raw RGB, so we decode it into BGR for OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # --- COWORKER ADDS YOLO INFERENCE HERE ---
            # results = self.model.predict(source=cv_image, conf=0.5, show=False)
            # annotated_image = results[0].plot()
            # 
            # (Optional) Extract bounding box center and publish goal coordinates
            # -----------------------------------------
            
            # 2. (Optional) Publish the annotated image back to ROS for debugging
            # ros_annotated = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            # self.publisher_annotated.publish(ros_annotated)
            
            # Placeholder display (can be removed later)
            cv2.imshow("YOLO Debug View", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

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