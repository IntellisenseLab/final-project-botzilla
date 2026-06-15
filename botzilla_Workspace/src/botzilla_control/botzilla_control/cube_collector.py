import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import time

# --- TUNABLE PARAMETERS ---
ALIGNMENT_THRESHOLD = 0.08
CAPTURE_BLIND_SPOT_M = 0.55
CAPTURE_TIME_S = 2.5
CAPTURE_SPEED = 0.12
KP_ANGULAR = 0.8
APPROACH_SPEED = 0.15
CUBE_LOST_TIMEOUT_S = 1.5

class CubeCollector(Node):
    def __init__(self):
        super().__init__('cube_collector')

        self.state = "SEARCHING"
        self.target_cube = None
        self._phase_timer = None
        self._cube_last_seen = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Point, 'detected_cube', self.cube_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Cube Collector Node Started. SEARCHING for cubes...")

    def cube_callback(self, msg):
        self.target_cube = msg
        self._cube_last_seen = self.get_clock().now()
        if self.state == "SEARCHING":
            self.get_logger().info(f"Cube detected! Transitioning to TARGETING.")
            self.state = "TARGETING"

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now()

        if self.state == "SEARCHING":
            msg.angular.z = 0.4

        elif self.state == "TARGETING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost.")
            elif self.target_cube:
                error_x = self.target_cube.x
                if abs(error_x) > ALIGNMENT_THRESHOLD:
                    msg.angular.z = -KP_ANGULAR * error_x
                else:
                    self._transition("APPROACHING", "Aligned. Moving in.")

        elif self.state == "APPROACHING":
            if self._cube_timed_out(now):
                self._transition("SEARCHING", "Cube lost.")
            elif self.target_cube:
                msg.angular.z = -KP_ANGULAR * 0.5 * self.target_cube.x
                if self.target_cube.z == 0.0:
                    self._transition("CAPTURING", "In blind spot. Firming capture.")
                else:
                    msg.linear.x = APPROACH_SPEED

        elif self.state == "CAPTURING":
            elapsed = self._elapsed_since(now)
            if elapsed < CAPTURE_TIME_S:
                msg.linear.x = CAPTURE_SPEED
            else:
                self._transition("DONE", "Cube collected successfully! Stopping.")

        elif self.state == "DONE":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def _transition(self, new_state, reason=""):
        self.get_logger().info(f"[{self.state}] -> [{new_state}] | {reason}")
        self.state = new_state
        self._phase_timer = self.get_clock().now()

    def _elapsed_since(self, now):
        if self._phase_timer is None: return 0.0
        return (now - self._phase_timer).nanoseconds * 1e-9

    def _cube_timed_out(self, now):
        return (now - self._cube_last_seen).nanoseconds * 1e-9 > CUBE_LOST_TIMEOUT_S

def main(args=None):
    rclpy.init(args=args)
    node = CubeCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
