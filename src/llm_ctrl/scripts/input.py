#!/usr/bin/env python3
import threading
import time
import json
import os
import msvcrt
import numpy as np
import sounddevice as sd
from faster_whisper import WhisperModel
from scipy.signal import butter, filtfilt
from multiprocessing import shared_memory
import sys
# # sys.path.insert(0, './')
# import nvidia_cubalas
# import nvidia_cudnn


# ================== WINDOWS OPTIMIZATION ==================
os.environ["OMP_NUM_THREADS"] = "4"
os.environ["CT2_FAST_NATIVE_THREADING"] = "1"

# ================== CONFIG ==================
SAMPLE_RATE = 16000
BLOCK_DURATION = 0.1
CHANNELS = 1
FRAMES_PER_BLOCK = int(SAMPLE_RATE * BLOCK_DURATION)

SHM_NAME = "shm"
SHM_SIZE = 512

# ==========================================================
class TextInputApp:
    def __init__(self):
        # ---------- Shared Memory ----------
        try:
            self.shm = shared_memory.SharedMemory(name=SHM_NAME)
            print(f"🔗 Attached to shared memory: {SHM_NAME}")
        except FileNotFoundError:
            self.shm = shared_memory.SharedMemory(
                create=True, size=SHM_SIZE, name=SHM_NAME
            )
            print(f"🆕 Created shared memory: {SHM_NAME}")

        self.shm_buf = self.shm.buf
        self.shm_lock = threading.Lock()

        # ---------- Audio ----------
        sd.default.samplerate = SAMPLE_RATE
        sd.default.channels = CHANNELS
        model = "medium.en"
        device="cpu"
        print(f"->loading model...{model} ..with device {device}")
        self.model = WhisperModel(model, device=device, compute_type="int8")

        # ---------- State ----------
        self.recording = threading.Event()
        self.stop_event = threading.Event()
        self.audio_data = []
        self.audio_lock = threading.Lock()
        self.input_mode = 0  # 1 = voice, 2 = keyboard
        self.current_text = ""

        print(f"📱 Shared Memory Name : {SHM_NAME}")
        print(f"📱 Max Text Size     : {SHM_SIZE} bytes")

    # ================== SHARED MEMORY ==================
    
    """SHM_NAME = "text_input_shm"
    SHM_SIZE = 512"""
    def write_to_shared_memory(self, text: str):
        with self.shm_lock:
            # self.shm_buf[:] = b"\x00" * SHM_SIZE
            
            
            self.shm_buf[:SHM_SIZE] = b"0"*(SHM_SIZE)

            data = text.encode("utf-8")[: SHM_SIZE - 1]
            if len(data) < SHM_SIZE:
                data += b"\x00"*(SHM_SIZE - len(data))
            self.shm_buf[: SHM_SIZE] = data

    # ================== AUDIO ==================
    def preprocess_audio(self, audio):
        b, a = butter(4, 300, btype="high", fs=SAMPLE_RATE)
        return filtfilt(b, a, audio)

    def audio_callback(self, indata, frames, time_, status):
        if status:
            print(status)
        if self.recording.is_set():
            with self.audio_lock:
                self.audio_data.extend(indata[:, 0].astype(np.float32))

    def audio_thread(self):
        with sd.InputStream(
            channels=CHANNELS,
            samplerate=SAMPLE_RATE,
            blocksize=FRAMES_PER_BLOCK,
            callback=self.audio_callback,
        ):
            while not self.stop_event.is_set():
                time.sleep(0.1)

    def process_recording(self):
        with self.audio_lock:
            if len(self.audio_data) < SAMPLE_RATE * 0.5:
                print("❌ Audio too short")
                return
            audio = np.array(self.audio_data[-SAMPLE_RATE * 3:], dtype=np.float32)

        #audio = self.preprocess_audio(audio)
        audio = self.preprocess_audio(audio).astype(np.float32)
        audio = np.clip(audio, -1.0, 1.0)
        audio /= max(np.max(np.abs(audio)), 1e-6)

        print("🔮 Transcribing...")
        segments, _ = self.model.transcribe(audio, language="en", vad_filter=True)
        text = " ".join(seg.text.strip() for seg in segments).strip()

        if text:
            print(f"✅ Voice → Shared Memory: '{text}'")
            self.write_to_shared_memory(text)
        else:
            print("❌ No speech detected")

    # ================== INPUT ==================
    def initial_setup(self):
        print("\n=== INPUT MODE ===")
        print("1 → 🎤 Voice")
        print("2 → ⌨️ Keyboard")

        while self.input_mode == 0:
            if msvcrt.kbhit():
                k = msvcrt.getch().decode(errors="ignore")
                if k == "1":
                    self.input_mode = 1
                    print("🎤 Voice mode selected")
                elif k == "2":
                    self.input_mode = 2
                    print("⌨️ Keyboard mode selected")
            time.sleep(0.01)

    def keyboard_thread(self):
        print("ESC = toggle mode | Ctrl+C = quit")

        while not self.stop_event.is_set():
            if msvcrt.kbhit():
                key = msvcrt.getch()

                # ESC
                if key == b"\x1b":
                    self.input_mode = 3 - self.input_mode
                    self.current_text = ""
                    print(
                        f"\n🔄 Switched to {'VOICE' if self.input_mode==1 else 'KEYBOARD'}"
                    )

                elif self.input_mode == 1:  # VOICE
                    if key == b" ":
                        print("\n🎤 Recording... ENTER to stop")
                        self.audio_data.clear()
                        self.recording.set()
                    elif key == b"\r" and self.recording.is_set():
                        self.recording.clear()
                        self.process_recording()

                else:  # KEYBOARD
                    if key == b"\r":
                        if self.current_text.strip():
                            print(f"\n✅ Text → Shared Memory: '{self.current_text}'")
                            self.write_to_shared_memory(self.current_text)
                            self.current_text = ""
                    elif key == b"\x08":
                        self.current_text = self.current_text[:-1]
                        print("\b \b", end="", flush=True)
                    else:
                        char = key.decode(errors="ignore")
                        if char.isprintable():
                            self.current_text += char
                            print(char, end="", flush=True)

            time.sleep(0.01)

    # ================== RUN ==================
    def run(self):
        self.initial_setup()

        threading.Thread(target=self.keyboard_thread, daemon=True).start()
        threading.Thread(target=self.audio_thread, daemon=True).start()

        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n🛑 Exiting...")
        finally:
            self.shm.close()
            try:
                self.shm.unlink()
                print("🧹 Shared memory unlinked")
            except FileNotFoundError:
                pass
                


# ================== MAIN ==================
if __name__ == "__main__":
    TextInputApp().run()
