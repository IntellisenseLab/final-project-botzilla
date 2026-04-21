import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Odometry
import math

class PerceptionSimulator(Node):
    def __init__(self):
        super().__init__('perception_simulator')
        self.cube_pos = (1.0, 0.0) # Coordinates from your botzilla_arena.world
        self.publisher_ = self.create_publisher(Point, 'detected_cube', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.get_logger().info("Perception Simulator Started. Tracking Cube at (1, 0)")

    def odom_callback(self, msg):
        # Current robot position from Odometry
        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        
        # Calculate distance (Z)
        dist = math.sqrt((self.cube_pos[0] - rx)**2 + (self.cube_pos[1] - ry)**2)
        
        # Simulating X alignment (Simple difference for now)
        # In a real cam, this would be horizontal pixel offset
        angle_to_cube = math.atan2(self.cube_pos[1] - ry, self.cube_pos[0] - rx)
        
        # Publish the simulated "Camera" data
        out = Point()
        out.x = angle_to_cube # Simplified: 0 means aligned
        out.y = 0.0
        out.z = dist
        self.publisher_.publish(out)

def main():
    rclpy.init()
    node = PerceptionSimulator()
    rclpy.spin(node)
    rclpy.shutdown()