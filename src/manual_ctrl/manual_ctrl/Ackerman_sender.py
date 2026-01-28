#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import json


WINDOWS_IP = "192.168.1.100"   # 🔁 CHANGE THIS
WINDOWS_PORT = 5005


class SimpleCarController(Node):
    def __init__(self):
        super().__init__('ackermann_socket_controller')

        # Joystick subscriber (unchanged)
        self.joystick_sub = self.create_subscription(
            Joy,
            'driftpilot_joy/joy_ramped',
            self.joy_callback,
            10
        )

        # Ackermann state
        self.current_speed = 0.0
        self.current_steering = 0.0

        self.MAX_SPEED = 1.0
        self.MAX_STEERING_ANGLE = 0.52

        # Socket setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((WINDOWS_IP, WINDOWS_PORT))
        self.get_logger().info("Connected to Windows socket receiver")

        # 50 Hz update (same as before)
        self.timer = self.create_timer(1.0 / 50.0, self.send_commands)

    def joy_callback(self, msg: Joy):
        if len(msg.axes) >= 5:
            raw_steering_input = msg.axes[2]
            raw_throttle_input = msg.axes[1]

            self.current_speed = raw_throttle_input * self.MAX_SPEED
            self.current_steering = raw_steering_input * self.MAX_STEERING_ANGLE
        else:
            self.current_speed = 0.0
            self.current_steering = 0.0

    def send_commands(self):
        ackermann_dict = {
            "speed": float(self.current_speed),
            "steering_angle": float(self.current_steering),
            "steering_angle_velocity": 0.0,
            "acceleration": 0.0,
            "jerk": 0.0
        }

        payload = json.dumps(ackermann_dict).encode("utf-8")
        self.sock.sendall(payload + b"\n")  # newline-delimited packets


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCarController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()