#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped # Import the specific message

class SimpleCarController(Node):
    def __init__(self):
        super().__init__('ackermann_car_controller')
        
        # New: Publisher for the combined Ackermann drive command
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'ackermann_cmd', 10) 
        
        # Original: Joystick subscriber
        self.joystick_sub = self.create_subscription(Joy, 'driftpilot_joy/joy_ramped', self.joy_callback, 10)
        
        self.current_speed = 0.0      # Corresponds to speed (m/s)
        self.current_steering = 0.0   # Corresponds to steering_angle (rad)
        
        # Define maximum movement values for scaling
        self.MAX_SPEED = 1             # Max linear speed in meters/second
        self.MAX_STEERING_ANGLE = 0.27  # Max steering angle in radians (approx 30 degrees)
        
        self.timer = self.create_timer(1.0 / 50.0, self.publish_commands) # 50Hz update

    def joy_callback(self, msg):
        # Safety check: Ensure the message has enough axes to read
        # Checking for index 4 requires a length of at least 5.
        if len(msg.axes) >= 5: 
            
            # --- Input Mapping ---
            # Axis 0: Left/Right Stick X-axis (for Steering)
            # Axis 4: Trigger/Stick (for Throttle/Speed) - Used based on your original code
                        
            raw_steering_input = msg.axes[2]
            raw_throttle_input = msg.axes[1] 
            
            self.MAX_SPEED = 1             # Max linear speed in meters/second
            self.MAX_STEERING_ANGLE = 0.52 # Max steering angle in radians (approx 30 degrees)

            # 1. Calculate Speed (Linear): Maps [-1.0, 1.0] input to [-MAX_SPEED, MAX_SPEED]
            # Assumes stick up/down maps to forward/backward speed
            self.current_speed = (raw_throttle_input) * self.MAX_SPEED

            # 2. Calculate Steering Angle (Angular): Maps [-1.0, 1.0] input to [-MAX_STEERING_ANGLE, MAX_STEERING_ANGLE]
            self.current_steering = raw_steering_input * self.MAX_STEERING_ANGLE
        else:
            # Emergency stop/neutral if the joystick data is invalid
            self.current_speed = 0.0
            self.current_steering = 0.0

    def publish_commands(self):
        # 1. Create a new AckermannDriveStamped message
        drive_msg = AckermannDriveStamped()
        
        # 2. Fill the Header
        # The 'Stamped' version requires a header for timestamping and frame_id
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link' # Replace with your robot's base frame
        
        # 3. Fill the Drive command fields
        drive_msg.drive.speed = self.current_speed             # Set linear speed (m/s)
        drive_msg.drive.steering_angle = self.current_steering # Set steering angle (rad)
        
        # 4. Publish the command
        self.drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCarController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class PS4Controller(Node):
    def __init__(self, rate):
        super().__init__('joystick_ramped')

        self.subscription = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        self.publisher = self.create_publisher(
            Joy, 'driftpilot_joy/joy_ramped', 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_joy)

        # ---------- Joy state ----------
        self.target_joy = Joy()
        self.target_joy.axes = [0.] * 8
        self.target_joy.buttons = [0] * 11

        # ---------- Axis mapping ----------
        self.THROTTLE_AXIS = 1
        self.STEERING_AXIS = 2

        # ---------- Throttle ramp ----------
        self.throttle_target = 0.0
        self.throttle_output = 0.0
        self.THROTTLE_RAMP_RATE = 0.04   # per cycle @ 50 Hz

    def joy_callback(self, msg):
        self.target_joy.buttons = msg.buttons
        self.target_joy.axes = msg.axes

        if len(msg.axes) > self.THROTTLE_AXIS:
            self.throttle_target = msg.axes[self.THROTTLE_AXIS]
        else:
            self.throttle_target = 0.0

    def ramp(self, target, current, rate):
        delta = target - current
        if delta > rate:
            delta = rate
        elif delta < -rate:
            delta = -rate
        return current + delta

    def publish_joy(self):
        # ---- Apply ramp ONLY to throttle ----
        self.throttle_output = self.ramp(
            self.throttle_target,
            self.throttle_output,
            self.THROTTLE_RAMP_RATE
        )

        self.target_joy.axes[self.THROTTLE_AXIS] = self.throttle_output
        self.publisher.publish(self.target_joy)


def main(args=None):
    rclpy.init(args=args)
    joystick = PS4Controller(rate=50)
    rclpy.spin(joystick)
    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
