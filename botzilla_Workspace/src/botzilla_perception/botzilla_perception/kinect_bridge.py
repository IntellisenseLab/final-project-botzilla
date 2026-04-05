import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import freenect
import numpy as np

class KinectBridge(Node):
    def __init__(self):
        super().__init__('kinect_bridge')
        
        # Optimization 1: Use Sensor QoS (Best Effort) instead of Reliable. 
        # This prevents lag by dropping old frames instead of queueing them in memory.
        self.publisher_rgb = self.create_publisher(Image, '/camera/rgb/image_raw', qos_profile_sensor_data)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        
        # Optimization 2: Fast polling. 
        # sync_get_video will naturally cap us at 30hz at the hardware level.
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info('Ultra-Fast Kinect ROS 2 Bridge Started!')

    def timer_callback(self):
        try:
            # ---> RGB STREAM
            if self.publisher_rgb.get_subscription_count() > 0:
                rgb_frame, _ = freenect.sync_get_video()
                if rgb_frame is not None:
                    # Optimization 3: Completely bypass cv_bridge and OpenCV. 
                    # Freenect gives us raw RGB natively. Wrap the bytes instantly.
                    msg_rgb = Image()
                    msg_rgb.header.stamp = self.get_clock().now().to_msg()
                    msg_rgb.header.frame_id = "camera_rgb_frame"
                    msg_rgb.height = 480
                    msg_rgb.width = 640
                    msg_rgb.encoding = "rgb8"
                    msg_rgb.is_bigendian = 0
                    msg_rgb.step = 640 * 3
                    msg_rgb.data = rgb_frame.tobytes() # C-level memory dump (100x faster)
                    
                    self.publisher_rgb.publish(msg_rgb)

            # ---> DEPTH STREAM
            if self.publisher_depth.get_subscription_count() > 0:
                depth_frame, _ = freenect.sync_get_depth()
                if depth_frame is not None:
                    depth_8bit = depth_frame.astype(np.uint8)
                    
                    msg_depth = Image()
                    msg_depth.header.stamp = self.get_clock().now().to_msg()
                    msg_depth.header.frame_id = "camera_depth_frame"
                    msg_depth.height = 480
                    msg_depth.width = 640
                    msg_depth.encoding = "mono8"
                    msg_depth.is_bigendian = 0
                    msg_depth.step = 640
                    msg_depth.data = depth_8bit.tobytes()
                    
                    self.publisher_depth.publish(msg_depth)

        except TypeError:
            # Ignore initialization gaps
            pass
        except Exception as e:
            self.get_logger().error(f'Kinect read error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = KinectBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down cleanly...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()