import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import math,csv

from ddrobot_nav.controller import PurePursuitController

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.path = None
        self.create_subscription(Path,'/trajectory_path',self.path_cb,10)

        #path = [
           # [0.0, 0.0],
           # [1.0, 0.0],
          #  [1.5, 0.5],
         #   [2.0, 1.0],
        #]
        # Setting parameter for ros param set
        self.declare_parameter('lookahead', 1.0)
        self.declare_parameter('v_cmd', 0.6)
        self.declare_parameter('slowdown_gain', 0.8)
        self.error_log = []



        self.controller = PurePursuitController(path=[],lookahead=self.get_parameter('lookahead').value,v_cmd=self.get_parameter('v_cmd').value, slowdown_gain=self.get_parameter('slowdown_gain').value)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def on_param_change(self, params):
    for p in params:
        if p.name == 'lookahead':
            self.controller.lookahead = p.value
        elif p.name == 'v_cmd':
            self.controller.v_cmd = p.value
        elif p.name == 'slowdown_gain':
            self.controller.slowdown_gain = p.value
    return rclpy.parameter.SetParametersResult(successful=True)
    
    self.add_on_set_parameters_callback(self.on_param_change)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z)
        )

        v, omega = self.controller.compute_control((x, y, yaw))

        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        error = math.hypot(tx - x, ty - y)
        self.error_log.append((self.get_clock().now().nanoseconds * 1e-9, error))

        self.cmd_pub.publish(cmd)
        
     def path_cb(self, msg):
     	self.path = [(p.pose.position.x, p.pose.position.y)
                 for p in msg.poses]
    	self.controller.path = self.path
    	


    def destroy_node(self):
    	with open('/tmp/tracking_error.csv', 'w', newline='') as f:
    	    writer = csv.writer(f)
    	    writer.writerow(['time', 'error'])
    	    writer.writerows(self.error_log)
    	super().destroy_node()



def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


