#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class WallFollowNode(Node):
    def __init__(self):
        super().__init__('wall_follow')
        
        # Subscribe to LiDAR scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publish velocity commands
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Parameters
        self.declare_parameter('target_distance', 0.5)   # meters from wall
        self.declare_parameter('linear_velocity', 0.3)
        self.declare_parameter('kp', 1.0)               # Proportional gain
        self.declare_parameter('kd', 0.1)               # Derivative gain
        self.declare_parameter('wall_side', 'left')     # left or right
        
        self.target_distance = self.get_parameter('target_distance').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.wall_side = self.get_parameter('wall_side').value
        
        self.prev_error = 0.0
        self.get_logger().info(f'Wall Follow Node started (wall: {self.wall_side})')
    
    def scan_callback(self, msg: LaserScan):
        """
        Process LiDAR scan and maintain distance from wall.
        """
        try:
            ranges = np.array(msg.ranges)
            
            # Replace bad values
            ranges = np.nan_to_num(ranges, nan=msg.range_max,
                                  posinf=msg.range_max,
                                  neginf=msg.range_max)
            
            # Get wall distance based on side
            if self.wall_side.lower() == 'left':
                # Left side scan: indices 0-90 degrees from left
                wall_range_indices = range(0, int(len(ranges) * 0.25))
            else:  # right
                # Right side scan: last 90 degrees
                wall_range_indices = range(int(len(ranges) * 0.75), len(ranges))
            
            wall_ranges = ranges[wall_range_indices]
            wall_distance = np.min(wall_ranges[wall_ranges > 0.1])
            
            # Calculate error and derivative
            error = self.target_distance - wall_distance
            error_derivative = error - self.prev_error
            self.prev_error = error
            
            # PID control
            steering = self.kp * error + self.kd * error_derivative
            steering = np.clip(steering, -1.5, 1.5)  # Limit steering
            
            # Check for obstacles in front
            front_ranges = ranges[int(len(ranges)*0.35):int(len(ranges)*0.65)]
            front_obstacle = np.min(front_ranges)
            
            # Generate command
            cmd = Twist()
            
            if front_obstacle < 0.3:  # Obstacle too close ahead
                cmd.linear.x = 0.0
                cmd.angular.z = steering * 2.0  # Turn more sharply
            else:
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = steering
            
            self.velocity_publisher.publish(cmd)
            
            self.get_logger().debug(
                f'Wall dist: {wall_distance:.2f}m, Front: {front_obstacle:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follow_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
