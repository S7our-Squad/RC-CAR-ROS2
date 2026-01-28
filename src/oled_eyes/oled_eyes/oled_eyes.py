"""#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Joy
import time
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw
import random


class OLEDEyes(Node):
    def __init__(self):
        super().__init__('oled_eyes_node')

        # --- OLED Setup ---
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = SSD1306_I2C(128, 64, i2c, addr=0x3c)
        self.display.fill(0)
        self.display.show()
        self.is_open = True

        # --- Eye parameters ---
        self.center_x = 64
        self.center_y = 32
        self.eye_w = 40
        self.eye_h = 25
        self.min_w = 15
        self.min_h = 10
        self.border = 6
        

        self.target_x = self.center_x
        self.target_y = self.center_y
        self.last_cmd_time = time.time()
        self.random_mode = False
        self.steering = 0.0
        self.throttle = 0.0
        self.current_scale = 1

        # ROS2 subscription
        #self.create_subscription(AckermannDrive, '/ackermann_cmd', self.callback, 10)
        #Joy Subscription
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Timer for updates
        self.timer = self.create_timer(0.05, self.update_display)
        self.blink_timer = self.create_timer(2.0, self.blink)
        self.get_logger().info("OLED Eyes Node Started ✅")
        self.reopener = None
        self.random_time = self.create_timer(3, self.rand)
    def joy_callback(self, msg: Joy):
        # Typical joystick mapping (adjust if needed for your controller)
        steering_axis = -msg.axes[2]   # Left stick horizontal
        throttle_axis = msg.axes[1]   # Left stick vertical

        # Normalize throttle: Only allow forward response
        self.steering = steering_axis  # -1 to +1
        self.throttle = max(0.0, throttle_axis)  # treat backward as 0

        self.last_cmd_time = time.time()
        self.random_mode = False


    def draw_eye(self, x, y, w, h):
        img = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(img)
        x0 = int(x - w / 2)
        y0 = int(y - h / 2)
        x1 = int(x + w / 2)
        y1 = int(y + h / 2)
        draw.rounded_rectangle([x0, y0, x1, y1], radius=self.border, fill=255)
        self.display.image(img)
        self.display.show()

    def rand(self):
        if self.random_mode is True:
            x = [0, 64, 128]
            y = [0, 32, 64]
            rand_x = random.choice(x)
            rand_y = random.choice(y)
            self.target_x = rand_x
            self.target_y = rand_y

    def update_display(self):
        
        self.get_logger().info("In update display\n")
        self.draw_eye(self.center_x, self.center_y, self.eye_w, self.eye_h)
        if self.throttle ==0 and self.steering == 0:
            now = time.time()
            flag = True
            while(time.time()-now<3):
                pass 
            # Random motion every 2–3 sec if idle
            self.random_mode =False

            while(self.throttle ==0 and self.steering == 0):
                # Smooth transition
                self.center_x += (self.target_x - self.center_x) * 0.15
                self.center_y += (self.target_y - self.center_y) * 0.1
                self.draw_eye(self.center_x, self.center_y, self.eye_w, self.eye_h)
                if not self.random_mode:
                    self.get_logger().info("Entering random motion mode")
                    self.random_mode = True
                    target_x = self.target_x
                    target_y = self.target_y
                else:
                    target_x = 64
                    target_y = 32
        else:
            target_x = 64
            target_y = 32

        half_w = self.eye_w / 2
        if self.steering != 0:
            target_x = self.center_x + (128 - self.eye_w) * (abs(self.steering) / self.steering)
        else:
            if self.random_mode is False:
                target_x = 64

        half_w = self.eye_w / 2
        self.center_x = max(half_w, min(self.center_x, 128 - half_w))

        # Smooth transition
        self.center_x += (target_x - self.center_x) * 0.15
        self.center_y += (target_y - self.center_y) * 0.1

        # Shrink based on throttle
        
# Called every frame/update
        target_scale = 1.0 if self.throttle == 0 else (0.5 - self.throttle / 2)

# Smooth factor: smaller = slower, larger = faster transition
        smooth_speed = 0.1  

# Linear interpolation toward target scale
        self.current_scale = (
        self.current_scale +
        (target_scale - self.current_scale) * smooth_speed
        )

        self.eye_w = 40 * self.current_scale
        self.eye_h = 25 * self.current_scale


        self.draw_eye(self.center_x, self.center_y, self.eye_w, self.eye_h)

    def shutdown(self):
        """"""Clean shutdown - clear the OLED display""""""
        self.get_logger().info("Shutting down OLED Eyes... 👋")
        self.display.fill(0)
        self.display.show()
        self.get_logger().info("OLED cleared ✅")

    def blink(self):
        if self.is_open:
            self.display.fill(0)
            self.display.show()
            self.is_open = False
        else:
            self.draw_eye(self.center_x, self.center_y, self.eye_w, self.eye_h)
            self.is_open = True


def main(args=None):
    rclpy.init(args=args)
    node = OLEDEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()  # ← Clear display on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()"""
    
    
    
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import time
import board
import busio
from adafruit_ssd1306 import SSD1306_I2C
from PIL import Image, ImageDraw
import random


class OLEDEyes(Node):
    def __init__(self):
        super().__init__('oled_eyes_node')

        # ---------- OLED ----------
        i2c = busio.I2C(board.SCL, board.SDA)
        self.display = SSD1306_I2C(128, 64, i2c, addr=0x3c)
        self.display.fill(0)
        self.display.show()

        # ---------- EYE PARAMS ----------
        self.center_x = 64
        self.center_y = 32
        self.eye_w = 40
        self.eye_h = 25
        self.border = 6

        self.current_scale = 1.0
        self.target_x = 64
        self.target_y = 32

        # ---------- INPUT STATE ----------
        self.steering = 0.0
        self.throttle = 0.0

        self.prev_steering = 0.0
        self.prev_throttle = 0.0

        self.last_change_time = time.time()
        self.random_mode = False

        self.DEADZONE = 0.05
        self.IDLE_TIME = 3.0

        # ---------- ROS ----------
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.create_timer(0.05, self.update_display)
        self.create_timer(3.0, self.random_target)
        self.create_timer(2.0, self.blink)

        self.is_open = True
        self.get_logger().info("OLED Eyes Node Started ✅")

    # ================= CALLBACKS =================

    def joy_callback(self, msg: Joy):
        steering = -msg.axes[2]
        throttle = max(0.0, msg.axes[1])

        steering_changed = abs(steering - self.prev_steering) > self.DEADZONE
        throttle_changed = abs(throttle - self.prev_throttle) > self.DEADZONE

        if steering_changed or throttle_changed:
            self.last_change_time = time.time()
            self.random_mode = False

        self.steering = steering
        self.throttle = throttle

        self.prev_steering = steering
        self.prev_throttle = throttle

    def random_target(self):
        if time.time() - self.last_change_time > self.IDLE_TIME and self.steering == 0 and self.throttle == 0:
            if not self.random_mode:
                self.get_logger().info("Entering RANDOM mode 👀")
            self.random_mode = True

            self.target_x = random.randint(20, 108)
            self.target_y = random.randint(16, 48)

    # ================= DRAW =================

    def draw_eye(self):
        img = Image.new('1', (128, 64))
        draw = ImageDraw.Draw(img)

        x0 = int(self.center_x - self.eye_w / 2)
        y0 = int(self.center_y - self.eye_h / 2)
        x1 = int(self.center_x + self.eye_w / 2)
        y1 = int(self.center_y + self.eye_h / 2)

        draw.rounded_rectangle(
            [x0, y0, x1, y1],
            radius=self.border,
            fill=255
        )

        self.display.image(img)
        self.display.show()

    # ================= UPDATE LOOP =================

    def update_display(self):
        # ---- TARGET SELECTION ----
        if not self.random_mode:
            if abs(self.steering) > self.DEADZONE:
                self.target_x = 64 + (128 - self.eye_w) * self.steering
            else:
                self.target_x = 64

            self.target_y = 32

        # ---- SMOOTH MOTION ----
        self.center_x += (self.target_x - self.center_x) * 0.15
        self.center_y += (self.target_y - self.center_y) * 0.10

        self.center_x = max(10, min(118, self.center_x))
        self.center_y = max(10, min(54, self.center_y))

        # ---- THROTTLE → SQUINT ----
        target_scale = 1.0 if self.throttle == 0 else 0.5
        self.current_scale += (target_scale - self.current_scale) * 0.1

        self.eye_w = 40 * self.current_scale
        self.eye_h = 25 * self.current_scale

        self.draw_eye()

    # ================= BLINK =================

    def blink(self):
        if self.is_open:
            self.display.fill(0)
            self.display.show()
        else:
            self.draw_eye()

        self.is_open = not self.is_open

    # ================= SHUTDOWN =================

    def shutdown(self):
        self.display.fill(0)
        self.display.show()


def main(args=None):
    rclpy.init(args=args)
    node = OLEDEyes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

