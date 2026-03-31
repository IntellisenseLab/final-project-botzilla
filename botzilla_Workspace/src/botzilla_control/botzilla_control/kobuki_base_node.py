import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# Import your hardware driver
from .KobukiDriver import Kobuki 

class KobukiBaseNode(Node):
    def __init__(self):
        super().__init__('kobuki_base_node')
        
        # Initialize the hardware connection
        self.get_logger().info('Connecting to Kobuki hardware...')
        self.robot = Kobuki()
        self.robot.play_on_sound()

        # Subscribe to standard ROS velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('Kobuki Base Node started. Listening to /cmd_vel...')

    def cmd_vel_callback(self, msg):
        """
        Translates ROS 2 Twist messages into Kobuki hardware commands.
        """
        linear_x = msg.linear.x   # Forward/Backward speed (m/s)
        angular_z = msg.angular.z # Turning speed (rad/s)
        
        wheel_base = 0.230 # 23 cm wheel separation for Kobuki
        
        # Calculate left and right wheel speeds in mm/s
        left_wheel_speed = (linear_x - (angular_z * wheel_base / 2.0)) * 1000.0
        right_wheel_speed = (linear_x + (angular_z * wheel_base / 2.0)) * 1000.0
        
        # Determine the 'rotate' flag based on your driver's logic
        # 1 means pure rotation, 0 means forward/arc 
        rotate_flag = 1 if linear_x == 0.0 and angular_z != 0.0 else 0
        
        # Send to hardware
        self.robot.move(int(left_wheel_speed), int(right_wheel_speed), rotate_flag)
        
        self.get_logger().debug(f'Moving -> L: {left_wheel_speed:.1f}, R: {right_wheel_speed:.1f}, Rot: {rotate_flag}')

def main(args=None):
    rclpy.init(args=args)
    node = KobukiBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kobuki node...')
        # Stop motors safely (0 left, 0 right, 0 rotate flag)
        node.robot.move(0, 0, 0) 
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()