import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import time
import threading
import smbus

# ================= PCA9685 CONFIG =================
I2C_ADDR = 0x40
ESC_CH = 0
SERVO_CH = 1

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

# ================= SERVO / ESC PARAMS =================
SERVO_CENTER_US = 1500
SERVO_RANGE_US = 200
MAX_STEERING_RAD = 0.27 # 15.5 deg

ESC_NEUTRAL_US = 1000
ESC_FORWARD_MAX_US = 1600
ESC_CALIB_MAX_US = 2000
MAX_SPEED_MPS = 1
STAGE_1 = 1100
STAGE_2 = 1300
STAGE_3 = 1500
PWM_FREQ = 50  # Hz

# =====================================================

class RCCarPWMDriver(Node):
    def __init__(self):
        super().__init__('rc_car_pwm_driver')

        self.bus = smbus.SMBus(1)

        # 🔧 FIX: init PCA9685 properly
        self._init_pca9685()
        self.set_pwm_freq(PWM_FREQ)

        # Safe startup
        self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
        self.set_pwm_us(SERVO_CH, SERVO_CENTER_US)

        self.calibrating = True
        self.state = 0

        self.get_logger().info("Starting ESC calibration...")
        threading.Thread(target=self._calibrate_esc, daemon=True).start()

        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            'ackermann_cmd',
            self.drive_callback,
            10
        )

    # ================= PCA9685 INIT =================
    def _init_pca9685(self):
        # Reset
        self.bus.write_byte_data(I2C_ADDR, MODE1, 0x00)
        time.sleep(0.01)

        # Auto-increment + wake
        self.bus.write_byte_data(I2C_ADDR, MODE1, 0x20)
        time.sleep(0.01)

        self.get_logger().info("PCA9685 initialized")

    # ================= PCA9685 LOW LEVEL =================
    def write(self, reg, val):
        self.bus.write_byte_data(I2C_ADDR, reg, val)

    def set_pwm_freq(self, freq):
        prescale = int(25000000 / (4096 * freq) - 1)
        oldmode = self.bus.read_byte_data(I2C_ADDR, MODE1)

        self.write(MODE1, (oldmode & 0x7F) | 0x10)  # sleep
        self.write(PRESCALE, prescale)
        self.write(MODE1, oldmode)
        time.sleep(0.005)
        self.write(MODE1, oldmode | 0x80)

        self.get_logger().info(f"PCA9685 PWM freq set to {freq} Hz")

    def set_pwm_us(self, channel, us):
        ticks = int(us * 4096 / 20000)
        base = LED0_ON_L + 4 * channel

        self.write(base, 0)                 # ON_L
        self.write(base + 1, 0)             # ON_H
        self.write(base + 2, ticks & 0xFF)  # OFF_L
        self.write(base + 3, ticks >> 8)    # OFF_H

    # ================= ESC CALIBRATION =================
    def _calibrate_esc(self):
        try:
            

            self.state = 1
            self.get_logger().info("ESC CAL: NEUTRAL")
            self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
            time.sleep(2)

            self.calibrating = False
            self.get_logger().info("✓ ESC calibration complete")

        except Exception as e:
            self.get_logger().error(f"ESC calibration error: {e}")
            self.calibrating = False
    """SERVO_CENTER_US = 1500 
    SERVO_RANGE_US = 500
    MAX_STEERING_RAD = 0.27 # 15.5 deg

    ESC_NEUTRAL_US = 1000
    ESC_FORWARD_MAX_US = 1600
    ESC_CALIB_MAX_US = 2000
    MAX_SPEED_MPS = 1"""
    # ================= MAPPING =================
    def map_range(self, value, in_min, in_max, out_min, out_max):
        #value = np.clip(value, in_min, in_max)
        return (value - in_min) * (out_max - out_min)/(in_max-in_min) + out_min

    def convert_speed_to_pwm(self, speed_mps):
        #speed_mps = np.clip(speed_mps, 0.0, MAX_SPEED_MPS)
        if(abs(speed_mps/MAX_SPEED_MPS) <= 0.1):
            return int(self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            ESC_NEUTRAL_US, STAGE_1
        ))
        elif(abs(speed_mps/MAX_SPEED_MPS) <= 0.45):
            return int(self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            STAGE_1, STAGE_2
        ))
        elif (abs(speed_mps/MAX_SPEED_MPS) <= 0.75) :
            return int(self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            STAGE_2, STAGE_3
        ))
        else:
            return int(self.map_range(
            speed_mps,
            0.0, MAX_SPEED_MPS,
            STAGE_3, ESC_FORWARD_MAX_US
        ))


    def convert_steering_to_pwm(self, angle_rad):
        return int(self.map_range(
            angle_rad,
            -MAX_STEERING_RAD, MAX_STEERING_RAD,
            SERVO_CENTER_US - SERVO_RANGE_US,
            SERVO_CENTER_US + SERVO_RANGE_US
        ))

    # ================= ROS CALLBACK =================
    def drive_callback(self, msg):
        if self.calibrating:
            return

        speed_pwm = self.convert_speed_to_pwm(msg.drive.speed)
        steer_pwm = self.convert_steering_to_pwm(msg.drive.steering_angle)

        self.set_pwm_us(ESC_CH, speed_pwm)
        #self.get_logger().info(f"speed-pwm:{speed_pwm}...")
        self.set_pwm_us(SERVO_CH, steer_pwm)
        self.get_logger().info(f"steer:{steer_pwm}..speed:{speed_pwm}")

    def on_shutdown(self):
        self.set_pwm_us(ESC_CH, ESC_NEUTRAL_US)
        self.set_pwm_us(SERVO_CH, SERVO_CENTER_US)
        time.sleep(0.5)

# ================= MAIN =================
def main(args=None):
    rclpy.init(args=args)
    node = RCCarPWMDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
