import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy


import numpy as np
from ddrobot_nav.smoothing import smooth_path



class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(Path, '/trajectory_path', 10)

        # Example waypoints
        waypoints = [(0,0), (1,0.2), (2,-0.2), (3,0.3), (4,0)]
        qos = QoSProfile( depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        path, _, _, _ = smooth_path(waypoints, n_samples=400)
        self.path = path
        self.path_pub = self.create_publisher(Path, '/trajectory_path',qos)

        self.timer = self.create_timer(1.0, self.publish_path)
        
	
    def publish_path(self):
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()

        for p in self.path:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.pub.publish(msg)
        self.get_logger().info("Published trajectory")

def main():
    rclpy.init()
    node = TrajectoryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

