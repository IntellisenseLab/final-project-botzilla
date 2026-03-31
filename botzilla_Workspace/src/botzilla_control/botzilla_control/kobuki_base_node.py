import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

# Import your hardware driver (make sure KobukiDriver.py is accessible)
# from .KobukiDriver import Kobuki 

class KobukiBaseNode(Node):
    def __init__(self):
        super().__init__('kobuki_base_node')
        
        # Initialize the hardware connection
        self.get_logger().info('Connecting to Kobuki hardware...')
        # self.robot = Kobuki()
        # self.robot.play_on_sound()

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
        
        # NOTE: You will need to map these to the exact function your KobukiDriver uses.
        # Below is an example of mapping to left/right wheel speeds in mm/s:
        
        wheel_base = 0.230 # 23 cm wheel separation for Kobuki
        
        left_wheel_speed = (linear_x - (angular_z * wheel_base / 2.0)) * 1000.0
        right_wheel_speed = (linear_x + (angular_z * wheel_base / 2.0)) * 1000.0
        
        # Send to hardware (uncomment and adjust based on your driver's exact move method)
        # self.robot.move(int(left_wheel_speed), int(right_wheel_speed))
        
        self.get_logger().debug(f'Moving -> L: {left_wheel_speed:.1f}, R: {right_wheel_speed:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = KobukiBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Kobuki node...')
        # node.robot.move(0, 0) # Stop motors safely
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()