import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import freenect
import numpy as np
import threading

class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        
        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', qos_profile_sensor_data)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        
        # Shared memory variables
        self.latest_rgb = None
        self.latest_depth = None
        self.new_rgb_available = False
        self.new_depth_available = False

        # Start the background camera thread
        self.kinect_thread = threading.Thread(target=self.run_camera_loop, daemon=True)
        self.kinect_thread.start()

        # ROS Timer running independently at 30Hz (0.033s)
        self.timer = self.create_timer(1.0/30.0, self.publish_frames)
        self.get_logger().info('Decoupled 30FPS Kinect Bridge Started!')

    # --- CAMERA THREAD (Producer) ---
    def video_cb(self, dev, data, timestamp):
        # Just grab the bytes and flag it as ready. Absolutely nothing else.
        self.latest_rgb = data.tobytes()
        self.new_rgb_available = True

    def depth_cb(self, dev, data, timestamp):
        self.latest_depth = data.astype(np.uint8).tobytes()
        self.new_depth_available = True

    def run_camera_loop(self):
        try:
            # This locks this thread and runs at the hardware's max speed (30hz)
            freenect.runloop(video=self.video_cb, depth=self.depth_cb)
        except Exception as e:
            self.get_logger().error(f"Camera thread error: {e}")

    # --- ROS THREAD (Consumer) ---
    def publish_frames(self):
        # Publish RGB
        if self.new_rgb_available and self.publisher_rgb.get_subscription_count() > 0:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_color_optical_frame"
            msg.height, msg.width, msg.step = 480, 640, 640 * 3
            msg.encoding = "rgb8"
            msg.data = self.latest_rgb
            self.publisher_rgb.publish(msg)
            self.new_rgb_available = False # Wait for next frame
            
        # Publish Depth
        if self.new_depth_available and self.publisher_depth.get_subscription_count() > 0:
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_depth_optical_frame"
            msg.height, msg.width, msg.step = 480, 640, 640
            msg.encoding = "mono8"
            msg.data = self.latest_depth
            self.publisher_depth.publish(msg)
            self.new_depth_available = False # Wait for next frame

def main(args=None):
    rclpy.init(args=args)
    node = KinectBridge()
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