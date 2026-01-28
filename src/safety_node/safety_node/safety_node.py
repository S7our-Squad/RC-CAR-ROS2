#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        
        # Subscribe to LiDAR scan
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Subscribe to commanded velocity
        self.cmd_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        
        # Publish safe velocity (after safety checks)
        self.safe_velocity_publisher = self.create_publisher(
            Twist,
            '/safe_cmd_vel',
            10
        )
        
        # Parameters
        self.declare_parameter('collision_threshold', 0.3)  # 30cm
        self.declare_parameter('warning_threshold', 0.5)    # 50cm
        self.declare_parameter('front_angle_range', 90)     # degrees
        
        self.collision_threshold = self.get_parameter('collision_threshold').value
        self.warning_threshold = self.get_parameter('warning_threshold').value
        self.front_angle_range = self.get_parameter('front_angle_range').value
        
        self.last_cmd = Twist()
        self.emergency_stop = False
        
        self.get_logger().info('Safety Node started')
        self.get_logger().info(f'Collision threshold: {self.collision_threshold}m')
    
    def cmd_callback(self, msg: Twist):
        """Store last velocity command."""
        self.last_cmd = msg
    
    def scan_callback(self, msg: LaserScan):
        """
        Monitor LiDAR scan for obstacles.
        Prevent collisions by issuing emergency stop if needed.
        """
        try:
            ranges = np.array(msg.ranges)
            
            # Replace bad values
            ranges = np.nan_to_num(ranges, nan=msg.range_max,
                                  posinf=msg.range_max,
                                  neginf=msg.range_max)
            
            # Check front sector for collisions
            angle_range_rad = np.radians(self.front_angle_range / 2)
            front_start_idx = int(len(ranges) * 
                                 (msg.angle_min + angle_range_rad) / 
                                 (msg.angle_max - msg.angle_min))
            front_end_idx = int(len(ranges) * 
                               (msg.angle_max - angle_range_rad) / 
                               (msg.angle_max - msg.angle_min))
            
            front_ranges = ranges[max(0, front_start_idx):min(len(ranges), front_end_idx)]
            min_front_distance = np.min(front_ranges) if len(front_ranges) > 0 else msg.range_max
            
            # Check all sectors
            min_distance = np.min(ranges[ranges > 0.05])  # Ignore very close noise
            
            # Decision logic
            if min_distance < self.collision_threshold or min_front_distance < self.collision_threshold:
                # COLLISION IMMINENT - HARD STOP
                self.emergency_stop = True
                safe_cmd = Twist()  # All zeros = stop
                self.get_logger().error(
                    f'⚠️ EMERGENCY STOP! Min dist: {min_distance:.2f}m'
                )
            elif min_distance < self.warning_threshold:
                # WARNING - reduce speed but allow some movement
                self.emergency_stop = False
                safe_cmd = Twist()
                safe_cmd.linear.x = max(0, self.last_cmd.linear.x * 0.3)  # 30% speed
                safe_cmd.angular.z = self.last_cmd.angular.z * 0.5  # 50% turn
                self.get_logger().warn(
                    f'⚠️ WARNING! Min dist: {min_distance:.2f}m - reducing speed'
                )
            else:
                # SAFE - allow normal command
                self.emergency_stop = False
                safe_cmd = self.last_cmd
            
            self.safe_velocity_publisher.publish(safe_cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')
            # Failsafe: stop on error
            safe_cmd = Twist()
            self.safe_velocity_publisher.publish(safe_cmd)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
