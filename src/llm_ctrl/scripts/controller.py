import json
from typing import Dict, Any
import ollama  # pip install ollama
import socket
from multiprocessing import shared_memory
import threading
import time

SHM_NAME = "shm"
SHM_SIZE = 512
REMOTE_IP = "172.20.10.9"  # Change to your RC car's IP
REMOTE_PORT = 5000  # Change if needed

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
- If no steering command is mentioned, steering_angle = 0.0.
- If the user says "turn left/right", set steering_angle to a small angle:
  - "small left/right": +/-0.2
  - "left/right": +/-0.4
  - "sharp left/right": +/-1
- acceleration, jerk, steering_angle_velocity can be 0.0 unless explicitly mentioned.

Output ONLY valid JSON, no markdown, no comments, no surrounding text.
"""

RC_KNOWLEDGE = """
RC Car Commands:
- "forward/backward" → speed positive/negative
- "left/right turn" → steering_angle -0.4/+0.4  
- "stop" → speed=0, steering=0
- "drift left" → steering=-0.6, speed=1.5
- Max speed: 3.0 m/s
"""

class AckermannLLMBridge:
    def __init__(self):
        # Connect to shared memory
        self.shm = shared_memory.SharedMemory(name=SHM_NAME)
        self.shm_buf = self.shm.buf
        
        # Socket connection
        self.sock = None
        self.connect_socket()
        print("🚗 Ackermann LLM Bridge Ready")
        print(f"← Reading from SHM: {SHM_NAME}")
        print(f"→ Sending JSON to: {REMOTE_IP}:{REMOTE_PORT}\n")

    def connect_socket(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.connect((REMOTE_IP, REMOTE_PORT))
            print(f"✅ Connected to {REMOTE_IP}:{REMOTE_PORT}")
        except Exception as e:
            print(f"❌ Socket connect failed: {e}")
            self.sock = None

    def read_from_shared_memory(self) -> str:
        """Read null-terminated string from shared memory"""
        raw_bytes = bytes(self.shm_buf[:SHM_SIZE])
        if b"\x00" in raw_bytes:
            raw_bytes = raw_bytes.split(b"\x00", 1)[0]
        return raw_bytes.decode("utf-8", errors="ignore").strip()

    def parse_drive_command(self, command: str) -> Dict[str, Any]:
        messages = [
        {"role": "system", "content": SYSTEM_PROMPT + "\n\nKnowledge: " + RC_KNOWLEDGE},
        {"role": "user", "content": command},
        ]
        response = ollama.chat(
            model="llama3.2:latest",messages=messages,
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

        content = response["message"]["content"]
        data = json.loads(content)
        print(data)
        defaults = {
            "speed": 0.0,
            "steering_angle": 0.0,
            "steering_angle_velocity": 0.0,
            "acceleration": 0.0,
            "jerk": 0.0,
        }
        defaults.update(data)
        return defaults

    def send_json_to_socket(self, data: Dict[str, Any]):
        """Send JSON command over socket"""
        if not self.sock:
            self.connect_socket()
            if not self.sock:
                return
                
        try:
            json_str = json.dumps(data)
            self.sock.sendall(json_str.encode("utf-8") + b"\n")
            print(f"📤 Sent: {json_str}")
        except Exception as e:
            print(f"❌ Socket send failed: {e}")
            self.sock.close()
            self.sock = None

    def run(self):
        last_command = ""
        while True:
            try:
                # Read from shared memory
                command = self.read_from_shared_memory()
                # print(type(command))
                if command and command != last_command:
                    print(f"📥 SHM Command: '{command}'")
                    
                    # Parse with LLM
                    result = self.parse_drive_command(command)
                    
                    # Send JSON over socket
                    self.send_json_to_socket(result)
                    
                    last_command = command
                
                time.sleep(0.1)  # 10Hz polling
                
            except KeyboardInterrupt:
                print("\n👋 Shutting down...")
                break
            except Exception as e:
                print(f"⚠️  Error: {e}")
                time.sleep(0.1)

if __name__ == "__main__":
    bridge = AckermannLLMBridge()
    try:
        bridge.run()
    finally:
        if bridge.sock:
            bridge.sock.close()
