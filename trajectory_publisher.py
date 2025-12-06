# trajectory_publisher.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from smoothing import smooth_path  # copy smoothing.py into this package or pip install prototype

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.pub = self.create_publisher(Path, 'planned_path', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        # example waypoints
        self.waypoints = [(0,0),(1,0.2),(2,-0.2),(3,0.3),(4,0)]
        # Create path once
        path_pts, s, csx, csy = smooth_path(self.waypoints, n_samples=400)
        self.path_pts = path_pts

    def timer_callback(self):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for pt in self.path_pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(pt[0])
            ps.pose.position.y = float(pt[1])
            ps.pose.position.z = 0.0
            # orientation left zero
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)
        self.get_logger().info(f'Published path with {len(msg.poses)} points')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

