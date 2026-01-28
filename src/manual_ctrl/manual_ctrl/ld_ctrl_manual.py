#!/usr/bin/env python3

"""
LiDAR Control Node for Manual Mode
Separate node that monitors LiDAR and provides feedback to manual driver.
Publishes warnings and obstacle distances.
Does NOT control the robot - only provides information.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LdCtrlManualNode(Node):
    def __init__(self):
        super().__init__('ld_ctrl_manual')
        
        # Subscribe to LiDAR scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Parameters
        self.declare_parameter('warning_threshold', 1.0)    # meters
        self.declare_parameter('danger_threshold', 0.5)     # meters
        self.declare_parameter('log_interval', 10)          # updates before logging
        
        self.warning_threshold = self.get_parameter('warning_threshold').value
        self.danger_threshold = self.get_parameter('danger_threshold').value
        self.log_interval = self.get_parameter('log_interval').value
        
        self.update_count = 0
        
        self.get_logger().info('LiDAR Control Manual Node started')
        self.get_logger().info(f'Warning threshold: {self.warning_threshold}m')
        self.get_logger().info(f'Danger threshold: {self.danger_threshold}m')
    
    def scan_callback(self, msg: LaserScan):
        """
        Process LiDAR scan and provide feedback to manual driver.
        """
        try:
            ranges = np.array(msg.ranges)
            
            # Replace bad values
            ranges = np.nan_to_num(ranges, nan=msg.range_max,
                                  posinf=msg.range_max,
                                  neginf=msg.range_max)
            
            # Get distances in different sectors
            left_start = 0
            left_end = int(len(ranges) * 0.25)
            front_start = int(len(ranges) * 0.35)
            front_end = int(len(ranges) * 0.65)
            right_start = int(len(ranges) * 0.75)
            right_end = len(ranges)
            
            left_dist = np.min(ranges[left_start:left_end]) if left_end > left_start else msg.range_max
            front_dist = np.min(ranges[front_start:front_end]) if front_end > front_start else msg.range_max
            right_dist = np.min(ranges[right_start:right_end]) if right_end > right_start else msg.range_max
            
            all_min = np.min(ranges[ranges > 0.05])
            
            # Increment counter
            self.update_count += 1
            
            # Log only every N updates to avoid spam
            if self.update_count >= self.log_interval:
                self.update_count = 0
                
                # Check danger levels
                if all_min < self.danger_threshold:
                    self.get_logger().error(
                        f'🚨 DANGER! Obstacle very close: {all_min:.2f}m\n'
                        f'   Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m'
                    )
                elif all_min < self.warning_threshold:
                    self.get_logger().warn(
                        f'⚠️  WARNING! Obstacle approaching: {all_min:.2f}m\n'
                        f'   Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m'
                    )
                else:
                    self.get_logger().info(
                        f'✓ Clear. Front: {front_dist:.2f}m | Left: {left_dist:.2f}m | Right: {right_dist:.2f}m'
                    )
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    ld_ctrl_manual_node = LdCtrlManualNode()
    
    try:
        rclpy.spin(ld_ctrl_manual_node)
    except KeyboardInterrupt:
        pass
    finally:
        ld_ctrl_manual_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
