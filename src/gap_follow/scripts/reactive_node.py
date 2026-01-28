#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):

    def __init__(self):
        super().__init__('reactive_follow_gap')

        # Topics
        self.scan_topic = '/scan'
        self.drive_topic = '/drive'

        # ===== CAR + LIDAR TUNING (FOR YOUR SETUP) =====
        self.max_range = 4.0                 # meters (X2 reliable range)
        self.bubble_radius = 0.45            # meters (1/10 car safety)
        self.max_steering_angle = 0.34       # rad (~19.5 deg)
        self.steering_gain = 0.9
        self.front_fov_deg = 70              # +- degrees
        self.prev_steering = 0.0

        # ROS interfaces
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.lidar_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.drive_topic,
            10
        )

        self.get_logger().info("Follow-the-Gap initialized (YDLIDAR X2, 1/10 RC)")

    # ------------------------------------------------
    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges)
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_range
        ranges = np.clip(ranges, 0.0, self.max_range)
        return ranges

    # ------------------------------------------------
    def find_max_gap(self, ranges):
        gaps = []
        start = None

        for i, r in enumerate(ranges):
            if r > 0 and start is None:
                start = i
            elif r == 0 and start is not None:
                gaps.append((start, i - 1))
                start = None

        if start is not None:
            gaps.append((start, len(ranges) - 1))

        if not gaps:
            return None, None

        return max(gaps, key=lambda g: g[1] - g[0])

    # ------------------------------------------------
    def find_best_point(self, start, end, ranges):
        segment = ranges[start:end + 1]
        return start + np.argmax(segment)

    # ------------------------------------------------
    def lidar_callback(self, data):
        ranges = self.preprocess_lidar(data.ranges)

        # ===== FRONT-ONLY FOV =====
        front_angle = np.deg2rad(self.front_fov_deg)

        start_i = int((-front_angle - data.angle_min) / data.angle_increment)
        end_i   = int(( front_angle - data.angle_min) / data.angle_increment)

        start_i = max(start_i, 0)
        end_i   = min(end_i, len(ranges) - 1)

        ranges = ranges[start_i:end_i]

        # ===== FIND CLOSEST OBSTACLE =====
        closest_idx = np.argmin(ranges)
        closest_dist = ranges[closest_idx]

        # ===== DISTANCE-AWARE BUBBLE =====
        if closest_dist > 0.0:
            bubble_angle = int(
                self.bubble_radius /
                (closest_dist * data.angle_increment)
            )
            bubble_start = max(0, closest_idx - bubble_angle)
            bubble_end   = min(len(ranges) - 1, closest_idx + bubble_angle)
            ranges[bubble_start:bubble_end] = 0.0

        # ===== FIND GAP =====
        gap_start, gap_end = self.find_max_gap(ranges)
        if gap_start is None:
            return

        best_idx = self.find_best_point(gap_start, gap_end, ranges)

        # ===== STEERING =====
        angle = (
            data.angle_min +
            (best_idx + start_i) * data.angle_increment
        ) * self.steering_gain

        angle = np.clip(angle,
                         -self.max_steering_angle,
                          self.max_steering_angle)

        # Steering smoothing (ESC & traction safety)
        angle = 0.7 * self.prev_steering + 0.3 * angle
        self.prev_steering = angle

        # ===== SPEED CONTROL (CRITICAL FOR 4300KV) =====
        min_dist = np.min(ranges)

        if min_dist < 0.8:
            speed = 0.7
        elif min_dist < 1.2:
            speed = 1.1
        else:
            speed = 1.6

        # ===== PUBLISH =====
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed

        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
