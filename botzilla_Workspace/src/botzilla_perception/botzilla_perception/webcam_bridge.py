import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np

class WebcamBridge(Node):
    """
    Webcam-based drop-in replacement for KinectBridge.
    Publishes to the same /camera/rgb/image_raw topic so the YOLO node
    works identically without any Kinect hardware.
    """

    def __init__(self):
        super().__init__('kinect_bridge')  # Same node name as real bridge

        self.publisher_rgb = self.create_publisher(
            Image, '/camera/rgb/image_raw', qos_profile_sensor_data
        )

        # Open the default webcam (index 0)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Could not open webcam! Check camera index.')
            raise RuntimeError('Webcam not found')

        self.get_logger().info('Webcam Bridge started — publishing to /camera/rgb/image_raw at 30 Hz')

        # Timer at 30 Hz to match Kinect rate
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from webcam.')
            return

        # OpenCV gives BGR; convert to RGB to match Kinect output
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb_frame.shape

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'
        msg.height = h
        msg.width = w
        msg.encoding = 'rgb8'
        msg.step = w * 3
        msg.data = rgb_frame.tobytes()
        self.publisher_rgb.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamBridge()
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
