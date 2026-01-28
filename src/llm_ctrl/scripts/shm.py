#!/usr/bin/env python3
import time
from multiprocessing import shared_memory

SHM_NAME = "shm"
SHM_SIZE = 512

def main():
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME)
        print(f"✅ Attached to shared memory: {SHM_NAME}")
    except FileNotFoundError:
        print("❌ Shared memory not found. Start writer first.")
        return

    try:
        while True:
            buf = bytes(shm.buf[:SHM_SIZE])

            # Raw buffer (hex for debugging)
            print("\n--- RAW BUFFER (hex) ---")
            print(buf.hex())

            # Decoded text (null-terminated)
            text = buf.split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
            print("\n--- DECODED TEXT ---")
            print(f"'{text}'")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n🛑 Exiting reader")

    finally:
        shm.close()

if __name__ == "__main__":
    main()
