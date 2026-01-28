import socket
import json
import serial
import time

# ================= SOCKET CONFIG =================
HOST = "0.0.0.0"
PORT = 5005

# ================= ARDUINO SERIAL CONFIG =================
SERIAL_PORT = "COM6"       # Windows: COMx | Linux: /dev/ttyUSB0
BAUDRATE = 115200

# ================= SETUP SERIAL =================
arduino = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
time.sleep(2)  # wait for Arduino reset
print("Arduino connected")

# ================= SETUP SOCKET =================
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind((HOST, PORT))
sock.listen(1)

print("Waiting for ROS sender...")
conn, addr = sock.accept()
print(f"Connected from {addr}")

buffer = ""

# ================= MAIN LOOP =================
while True:
    data = conn.recv(1024).decode()
    if not data:
        break

    buffer += data
    while "\n" in buffer:
        line, buffer = buffer.split("\n", 1)

        ackermann = json.loads(line)

        # Extract values
        speed = ackermann["speed"]
        steering = ackermann["steering_angle"]

        # Send to Arduino (CSV format)
        serial_msg = f"{speed:.3f},{steering:.3f}\n"
        arduino.write(serial_msg.encode())

        print("Sent to Arduino:", serial_msg.strip())