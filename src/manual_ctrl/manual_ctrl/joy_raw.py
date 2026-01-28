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
