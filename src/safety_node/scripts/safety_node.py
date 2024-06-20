#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.0
        self.brake_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

    def odom_callback(self, odom_msg):
        # Update current speed from the odometry message
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # Calculate Time-To-Collision (TTC)
        ttc_threshold = 1.0  # threshold in seconds
        need_to_brake = False

        for i, distance in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= distance <= scan_msg.range_max:
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                relative_speed = self.speed * np.cos(angle)
                if relative_speed > 0:
                    ttc = distance / relative_speed
                    if ttc < ttc_threshold:
                        need_to_brake = True
                        break

        if need_to_brake:
            # Publish drive/brake message
            brake_msg = AckermannDriveStamped()
            brake_msg.drive = AckermannDrive()
            brake_msg.drive.speed = 0.0
            self.brake_publisher.publish(brake_msg)
            self.get_logger().info('Emergency brake applied!')

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()