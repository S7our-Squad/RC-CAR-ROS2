#!/usr/bin/env python3

import json
import socket
import time
from typing import Dict, Any
from multiprocessing import shared_memory

import ollama  # pip install ollama
import math

# ================= CONFIG =================
SHM_NAME = "shm"
SHM_SIZE = 512

REMOTE_IP = "172.20.10.9"   # RC car IP
REMOTE_PORT = 5000          # RC car port

MAX_STEERING_RAD = 0.27

# ================= PROMPT =================
SYSTEM_PROMPT = """
You are a command-to-control translator for an RC car with Ackermann steering.

You receive a short natural language command from the user.
You MUST respond with ONLY a JSON object (no text, no explanation).

The JSON schema is strictly:
{
  "speed": float,
  "steering_angle": float,
  "steering_angle_velocity": float,
  "acceleration": float,
  "jerk": float
}

Rules:
- If the user says "set velocity to X", set speed = X (float).
- If no speed is mentioned, keep speed = 0.0.
- If the user says "stop", set speed = 0.0 and steering_angle = 0.0.
- If the user says left/right, do NOT change speed.
- Steering angle is given in DEGREES by the user.
- Convert to radians.
- Clamp steering angle between -0.27 and +0.27 radians.
- acceleration, jerk, steering_angle_velocity = 0.0 unless specified.

Output ONLY valid JSON.
"""

RC_KNOWLEDGE = """
RC Car Commands:
- forward/backward → speed ±
- left/right → steering
- stop → speed=0 steering=0
- Max speed: 3.0 m/s
"""

# =====================================================
class AckermannLLMBridge:
    def __init__(self):
        # -------- Shared Memory --------
        self.shm = shared_memory.SharedMemory(name=SHM_NAME)
        self.shm_buf = self.shm.buf

        # -------- Socket --------
        self.sock = None
        self.connect_socket()

        print("🚗 Ackermann LLM Bridge Ready")
        print(f"← SHM : {SHM_NAME}")
        print(f"→ UDP : {REMOTE_IP}:{REMOTE_PORT}\n")

    # ================= SOCKET =================
    def connect_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"✅ UDP socket ready ({REMOTE_IP}:{REMOTE_PORT})")
        except Exception as e:
            print(f"❌ Socket error: {e}")
            self.sock = None

    def send_json_to_socket(self, data: Dict[str, Any]):
        if not self.sock:
            self.connect_socket()
            if not self.sock:
                return

        try:
            msg = json.dumps(data).encode("utf-8")
            self.sock.sendto(msg, (REMOTE_IP, REMOTE_PORT))
            print(f"📤 Sent: {msg.decode()}")
        except Exception as e:
            print(f"❌ Send failed: {e}")
            self.sock.close()
            self.sock = None

    # ================= SHARED MEMORY =================
    def read_from_shared_memory(self) -> str:
        raw = bytes(self.shm_buf[:SHM_SIZE])
        raw = raw.split(b"\x00", 1)[0]
        return raw.decode("utf-8", errors="ignore").strip()

    # ================= LLM =================
    def parse_drive_command(self, command: str) -> Dict[str, Any]:
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT + "\n\n" + RC_KNOWLEDGE},
            {"role": "user", "content": command},
        ]

        response = ollama.chat(
            model="llama3.2:latest",
            messages=messages,
            format={
                "type": "object",
                "properties": {
                    "speed": {"type": "number"},
                    "steering_angle": {"type": "number"},
                    "steering_angle_velocity": {"type": "number"},
                    "acceleration": {"type": "number"},
                    "jerk": {"type": "number"},
                },
                "required": [
                    "speed",
                    "steering_angle",
                    "steering_angle_velocity",
                    "acceleration",
                    "jerk",
                ],
            },
        )

        data = json.loads(response["message"]["content"])

        # ---------- Post-processing ----------
        steering = data.get("steering_angle", 0.0)

        # If steering looks like degrees → convert
        if abs(steering) > 1.5:
            steering = math.radians(steering)

        # Clamp steering
        steering = max(-MAX_STEERING_RAD, min(MAX_STEERING_RAD, steering))
        data["steering_angle"] = steering

        # Defaults
        defaults = {
            "speed": 0.0,
            "steering_angle": 0.0,
            "steering_angle_velocity": 0.0,
            "acceleration": 0.0,
            "jerk": 0.0,
        }
        defaults.update(data)

        print(f"🤖 LLM Output: {defaults}")
        return defaults

    # ================= RUN =================
    def run(self):
        last_cmd = ""

        while True:
            try:
                cmd = self.read_from_shared_memory()

                if cmd and cmd != last_cmd:
                    print(f"📥 SHM Command: '{cmd}'")

                    control = self.parse_drive_command(cmd)
                    self.send_json_to_socket(control)

                    last_cmd = cmd

                time.sleep(0.1)  # 10 Hz

            except KeyboardInterrupt:
                print("\n👋 Exiting")
                break
            except Exception as e:
                print(f"⚠️ Error: {e}")
                time.sleep(0.2)


# ================= MAIN =================
if __name__ == "__main__":
    bridge = AckermannLLMBridge()
    try:
        bridge.run()
    finally:
        if bridge.sock:
            bridge.sock.close()
