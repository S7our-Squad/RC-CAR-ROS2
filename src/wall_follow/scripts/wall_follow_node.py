import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # Create subscribers and publishers
        self.lidar_subscriber = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # Set PID gains
        self.kp = 1.0
        self.kd = 0.1
        self.ki = 0.01

        # Store history for PID
        self.integral = 0.0
        self.prev_error = 0.0

    def get_range(self, range_data, angle, angle_min, angle_increment):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        # Calculate the index in the range array for the given angle
        index = int((angle - angle_min) / angle_increment)
        if index < 0 or index >= len(range_data) or np.isnan(range_data[index]) or np.isinf(range_data[index]):
            return float('inf')
        return range_data[index]

    def get_error(self, range_data, dist, angle_min, angle_increment):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        theta = np.deg2rad(45)  # 45 degrees in radians
        a = self.get_range(range_data, theta, angle_min, angle_increment)
        b = self.get_range(range_data, 0, angle_min, angle_increment)
        alpha = np.arctan((a * np.cos(theta) - b) / (a * np.sin(theta)))
        D_t = b * np.cos(alpha)
        error = D_t - dist
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        self.integral += error
        derivative = error - self.prev_error
        angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_publisher.publish(drive_msg)

    def scan_callback(self, scan_msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        desired_distance = 1.0  # Desired distance to the wall in meters
        velocity = 1.0  # Desired velocity of the car

        error = self.get_error(scan_msg.ranges, desired_distance, scan_msg.angle_min, scan_msg.angle_increment)
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()