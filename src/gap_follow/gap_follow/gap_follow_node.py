#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class GapFollowNode(Node):
    def __init__(self):
        super().__init__('gap_follow')
        
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
        self.declare_parameter('linear_velocity', 0.5)
        self.declare_parameter('gap_threshold', 0.5)  # meters
        self.declare_parameter('min_gap_width', 5)    # degrees
        self.declare_parameter('max_angular_velocity', 1.5)
        
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.gap_threshold = self.get_parameter('gap_threshold').value
        self.min_gap_width = self.get_parameter('min_gap_width').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        self.get_logger().info('Gap Follow Node started')
        self.get_logger().info(f'Target gap threshold: {self.gap_threshold}m')
    
    def scan_callback(self, msg: LaserScan):
        """
        Process LiDAR scan and find largest gap.
        """
        try:
            # Convert ranges to numpy for easier processing
            ranges = np.array(msg.ranges)
            
            # Replace infinities with max range
            ranges = np.nan_to_num(ranges, nan=msg.range_max, 
                                  posinf=msg.range_max, 
                                  neginf=msg.range_max)
            
            # Find gaps (free space where range > threshold)
            gaps = self.find_gaps(ranges, msg)
            
            if gaps:
                # Get largest gap
                largest_gap = max(gaps, key=lambda x: x[1] - x[0])
                gap_center = (largest_gap[0] + largest_gap[1]) / 2
                
                # Calculate angle to gap center
                gap_angle = (msg.angle_min + 
                           gap_center * msg.angle_increment)
                
                # Generate movement command toward gap
                cmd = Twist()
                cmd.linear.x = self.linear_velocity
                cmd.angular.z = self.saturate_angular_velocity(
                    gap_angle * 0.5, 
                    self.max_angular_velocity
                )
                
                self.velocity_publisher.publish(cmd)
                
                self.get_logger().debug(
                    f'Gap found at {math.degrees(gap_angle):.2f}°'
                )
            else:
                # No gap found - stop
                self.get_logger().warn('No gap found - stopping')
                cmd = Twist()
                self.velocity_publisher.publish(cmd)
                
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')
    
    def find_gaps(self, ranges, msg):
        """
        Find all gaps in range data.
        A gap is a continuous region where range > gap_threshold.
        Returns list of (start_index, end_index) tuples.
        """
        gaps = []
        gap_start = None
        
        for i, r in enumerate(ranges):
            if r > self.gap_threshold:  # Free space
                if gap_start is None:
                    gap_start = i
            else:  # Obstacle
                if gap_start is not None:
                    # Check gap width
                    gap_width = i - gap_start
                    gap_width_degrees = gap_width * math.degrees(msg.angle_increment)
                    
                    if gap_width_degrees >= self.min_gap_width:
                        gaps.append((gap_start, i))
                    gap_start = None
        
        # Handle gap that extends to end of scan
        if gap_start is not None:
            gap_width = len(ranges) - gap_start
            gap_width_degrees = gap_width * math.degrees(msg.angle_increment)
            
            if gap_width_degrees >= self.min_gap_width:
                gaps.append((gap_start, len(ranges)))
        
        return gaps
    
    def saturate_angular_velocity(self, angular_vel, max_vel):
        """Limit angular velocity to maximum value."""
        if angular_vel > max_vel:
            return max_vel
        elif angular_vel < -max_vel:
            return -max_vel
        return angular_vel


def main(args=None):
    rclpy.init(args=args)
    gap_follow_node = GapFollowNode()
    
    try:
        rclpy.spin(gap_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        gap_follow_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
