#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class SimpleSimulator:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.wheelbase = 0.33

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v = 0.0
        self.delta = 0.0

    def step(self):
        if abs(self.v) > 1e-3:
            omega = self.v * math.tan(self.delta) / self.wheelbase
        else:
            omega = 0.0

        self.theta += omega * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt

        return self.x, self.y, self.theta, self.v, omega


class GymBridge(Node):
    def __init__(self):
        super().__init__('simple_sim_bridge')

        self.sim = SimpleSimulator()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            '/drive',
            self.drive_callback,
            10
        )

        self.tf_br = TransformBroadcaster(self)

        self.timer = self.create_timer(0.02, self.update)

    def drive_callback(self, msg):
        self.sim.v = msg.drive.speed
        self.sim.delta = msg.drive.steering_angle

    def update(self):
        x, y, theta, v, omega = self.sim.step()
        now = self.get_clock().now().to_msg()

        # -------- Odometry (odom -> base_link) --------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y

        q = quaternion_from_euler(0.0, 0.0, theta)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # -------- TF (odom -> base_link) --------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_br.sendTransform(tf)

        # -------- Fake LaserScan (for TF testing only) --------
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = 'laser'
        scan.angle_min = -2.35
        scan.angle_max = 2.35
        scan.angle_increment = 4.7 / 1080
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = [10.0] * 1080  # ⚠️ fake

        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = GymBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
