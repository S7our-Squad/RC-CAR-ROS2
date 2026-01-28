import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

class LaneControllerNode(Node):
    def __init__(self):
        super().__init__('lane_controller_node')
        
        # PID parameters
        self.kp = 1.5      # Proportional gain
        self.ki = 0.01     # Integral gain  
        self.kd = 0.5      # Derivative gain
        self.integral = 0.0
        self.prev_error = 0.0
        self.dt = 0.1
        
        # Subscribe to lane detection
        self.lane_sub = self.create_subscription(
            Float32MultiArray, "/lane_detection/lane_info", 
            self.lane_callback, 10
        )
        
        # Publish Ackermann commands (your format)
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped, "/drive_ackermann", 10
        )
        
        # Control timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.current_lane = None
        
        self.get_logger().info('Lane Controller (Ackermann) started')

    def lane_callback(self, msg):
        """Process lane detection data"""
        if len(msg.data) > 1 and msg.data[0] > 0:
            # Average all detected lanes
            positions = msg.data[1:]
            self.current_lane = sum(positions) / len(positions)
        else:
            self.current_lane = None

    def control_loop(self):
        """PID control loop - outputs Ackermann commands"""
        if self.current_lane is None:
            # No lanes - send zero steering command
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.ackermann_pub.publish(msg)
            return
        
        # Error = distance from center (0.5 = center of image)
        error = self.current_lane - 0.5
        
        # PID calculation
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        
        # Steering angle (-1.57 to 1.57 radians ~ ±90 degrees)
        steering_angle = (self.kp * error + 
                         self.ki * self.integral + 
                         self.kd * derivative)
        
        # Limit steering angle
        steering_angle = max(-1.57, min(1.57, steering_angle))
        
        # Publish Ackermann command
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.3          # Forward speed (m/s)
        msg.drive.steering_angle = steering_angle  # Radians
        msg.drive.steering_angle_velocity = 1.0    # Max turn rate
        
        self.ackermann_pub.publish(msg)
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    node = LaneControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
