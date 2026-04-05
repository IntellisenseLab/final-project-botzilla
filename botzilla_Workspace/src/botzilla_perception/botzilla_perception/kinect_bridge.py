import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv_bridge
import freenect
import cv2
import numpy as np

class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.bridge = cv_bridge.CvBridge()
        
        # Timer to capture frames at roughly 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)
        self.get_logger().info('Custom Kinect ROS 2 Bridge Started!')

    def timer_callback(self):
        try:
            # 1. Fetch RGB ONLY if a node (rqt or YOLO) is subscribing to it
            if self.publisher_rgb.get_subscription_count() > 0:
                rgb_frame, _ = freenect.sync_get_video()
                if rgb_frame is not None:
                    rgb_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                    ros_rgb = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
                    self.publisher_rgb.publish(ros_rgb)

            # 2. Fetch Depth ONLY if a node is subscribing to it
            if self.publisher_depth.get_subscription_count() > 0:
                depth_frame, _ = freenect.sync_get_depth()
                if depth_frame is not None:
                    depth_8bit = depth_frame.astype(np.uint8)
                    ros_depth = self.bridge.cv2_to_imgmsg(depth_8bit, encoding="mono8")
                    self.publisher_depth.publish(ros_depth)

        except TypeError:
            pass
        except Exception as e:
            self.get_logger().error(f'Kinect read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = KinectBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Kinect bridge cleanly...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()