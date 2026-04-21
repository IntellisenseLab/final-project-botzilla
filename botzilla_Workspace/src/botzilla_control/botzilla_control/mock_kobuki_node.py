import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MockKobukiNode(Node):
    """
    Software-only mock for kobuki_base_node.
    Listens on /cmd_vel and logs commands instead of driving hardware.
    Use this on your laptop to test control logic without the robot.
    """

    def __init__(self):
        super().__init__('kobuki_base_node')  # Same node name as real driver

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info('[MOCK] Kobuki Base Node started. Listening on /cmd_vel ...')

    def cmd_vel_callback(self, msg):
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        wheel_base = 0.230  # 23 cm — same as real node
        left  = (linear_x - angular_z * wheel_base / 2.0) * 1000.0
        right = (linear_x + angular_z * wheel_base / 2.0) * 1000.0

        self.get_logger().info(
            f'[MOCK] CMD_VEL → linear={linear_x:.3f} m/s  angular={angular_z:.3f} rad/s'
            f'  |  L={left:.1f} mm/s  R={right:.1f} mm/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MockKobukiNode()
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
