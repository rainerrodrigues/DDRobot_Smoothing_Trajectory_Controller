# controller_node.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.sub_path = self.create_subscription(Path, 'planned_path', self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.odom_cb, 50)
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pts = None
        self.odom = None
        # controller params
        self.lookahead = 0.6
        self.v_cmd = 0.35
        self.slowdown_gain = 2.0
        self.timer = self.create_timer(0.02, self.timer_cb)

    def path_cb(self, msg):
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        self.path_pts = np.array(pts)
        self.get_logger().info(f'Received path with {len(pts)} points')

    def odom_cb(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        # quaternion to yaw
        q = pose.orientation
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)
        self.odom = (x, y, yaw)

    def find_goal(self, pos):
        if self.path_pts is None:
            return None, None
        dists = np.hypot(self.path_pts[:,0] - pos[0], self.path_pts[:,1] - pos[1])
        idx = int(np.argmin(dists))
        L = 0.0
        goal = self.path_pts[-1]
        for j in range(idx, len(self.path_pts)-1):
            L += np.hypot(self.path_pts[j+1,0]-self.path_pts[j,0], self.path_pts[j+1,1]-self.path_pts[j,1])
            if L >= self.lookahead:
                goal = self.path_pts[j+1]; break
        return goal, idx

    def timer_cb(self):
        if self.odom is None or self.path_pts is None:
            return
        x,y,theta = self.odom
        goal, idx = self.find_goal((x,y))
        if goal is None:
            return
        dx = goal[0] - x
        dy = goal[1] - y
        # transform to robot frame
        xr =  math.cos(-theta)*dx - math.sin(-theta)*dy
        yr =  math.sin(-theta)*dx + math.cos(-theta)*dy
        dist_goal = max(1e-6, math.hypot(xr, yr))
        curvature = 2.0 * yr / (dist_goal**2)
        v = self.v_cmd / (1.0 + self.slowdown_gain * abs(curvature))
        omega = v * curvature
        # publish Twist
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

