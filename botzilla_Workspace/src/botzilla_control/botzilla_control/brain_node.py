import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class BotzillaBrain(Node):
    def __init__(self):
        super().__init__('botzilla_brain')
        self.state = "SEARCHING"
        self.target_cube = None # Will store Point(x, y, z)
        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Point, 'detected_cube', self.cube_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("BotZilla Brain initialized in SEARCHING state.")

    def cube_callback(self, msg):
        self.target_cube = msg
        if self.state == "SEARCHING":
            self.get_logger().info("Cube Detected! Transitioning to TARGETING.")
            self.state = "TARGETING"

    def control_loop(self):
        msg = Twist()
        
        if self.state == "SEARCHING":
            msg.angular.z = 0.5 # Rotate to find cube
            
        elif self.state == "TARGETING":
            if not self.target_cube:
                self.state = "SEARCHING"
                return
            
            # center_x estimation: assuming 0.0 is center, -1.0 left, 1.0 right
            if abs(self.target_cube.x) > 0.05: 
                msg.angular.z = -0.3 if self.target_cube.x > 0 else 0.3
                self.get_logger().info(f"Aligning... current x: {self.target_cube.x}")
            else:
                self.get_logger().info("Target Aligned. Approaching...")
                self.state = "APPROACHING"

        elif self.state == "APPROACHING":
            if self.target_cube.z > 0.25: # z is distance in meters
                msg.linear.x = 0.15
            else:
                msg.linear.x = 0.0
                self.state = "PICKING"
                self.get_logger().info("Goal Reached. Ready to pick.")

        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = BotzillaBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()