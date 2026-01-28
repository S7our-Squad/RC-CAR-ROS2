self.last_cmd = AckermannDriveStamped()
self.last_cmd.header.frame_id = "base_link"
self.last_update_time = self.get_clock().now()

self.CONTROL_RATE = 20.0  # Hz
self.TIMEOUT_SEC = 0.3
self.timer = self.create_timer(
    1.0 / self.CONTROL_RATE,
    self.timer_callback
)

