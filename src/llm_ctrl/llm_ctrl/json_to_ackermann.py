import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
from rclpy.qos import QoSProfile

import socket
import threading
import json

HOST = "0.0.0.0"
PORT = 5000

class LlmAckermannNode(Node):
    def __init__(self):
        super().__init__("llm_ackermann_node")

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(
            AckermannDriveStamped,
            "ackermann_cmd",
            qos,
        )

        # Safety limits
        self.MAX_SPEED = 3.0
        self.MAX_STEER = 0.6
        self.MAX_ACCEL = 2.0
        self.MAX_JERK = 5.0

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((HOST, PORT))

        threading.Thread(target=self.udp_loop, daemon=True).start()
        self.get_logger().info("LLM Ackermann node listening on UDP 5000")

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def udp_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(1024)
                payload = json.loads(data.decode("utf-8"))
                self.publish_command(payload)
            except json.JSONDecodeError:
                self.get_logger().warn("Invalid JSON received")
            except Exception as e:
                self.get_logger().error(f"UDP error: {e}")

    def publish_command(self, drive_params: dict):
        msg = AckermannDriveStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Extract with defaults for safety
        msg.drive.speed = float(drive_params.get("speed", 0.0))
        msg.drive.steering_angle = float(drive_params.get("steering_angle", 0.0))
        msg.drive.steering_angle_velocity = float(
            drive_params.get("steering_angle_velocity", 0.0)
        )
        msg.drive.acceleration = float(drive_params.get("acceleration", 0.0))
        msg.drive.jerk = float(drive_params.get("jerk", 0.0))

        # Safety clamping
        msg.drive.speed = self.clamp(msg.drive.speed, -self.MAX_SPEED, self.MAX_SPEED)
        msg.drive.steering_angle = self.clamp(
            msg.drive.steering_angle, -self.MAX_STEER, self.MAX_STEER
        )
        msg.drive.acceleration = self.clamp(
            msg.drive.acceleration, -self.MAX_ACCEL, self.MAX_ACCEL
        )
        msg.drive.jerk = self.clamp(msg.drive.jerk, -self.MAX_JERK, self.MAX_JERK)

        #self.pub.publish(msg)
'''
        self.get_logger().info(
            f"Published: speed={msg.drive.speed:.2f}, "
            f"steer={msg.drive.steering_angle:.2f}"
        )
'''
        self.last_cmd = msg
        self.last_update_time = self.get_clock.now()

    def timer_callback:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9

    # 🚨 Watchdog: stop car if no UDP update
        if dt > self.TIMEOUT_SEC:
            self.last_cmd.drive.speed = 0.0
            self.last_cmd.drive.steering_angle = 0.0

        self.last_cmd.header.stamp = now.to_msg()
        self.pub.publish(self.last_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = LlmAckermannNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
