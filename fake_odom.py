#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math


class FakeOdom(Node):

    def __init__(self):
        super().__init__('fake_odom')

        # Odometry publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Simple forward motion
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.2     # m/s
        self.dt = 0.05   # seconds

        self.get_logger().info('Fake odometry node started')

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # --- Integrate simple motion ---
        self.x += self.v * self.dt * math.cos(self.theta)
        self.y += self.v * self.dt * math.sin(self.theta)

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Quaternion from yaw
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)

        # --- TF transform ---
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FakeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

