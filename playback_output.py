import serial
import numpy as np
import sounddevice as sd
import threading
import queue

PORT = "COM3"  # adjust
BAUD = 115200
NSAM = 160
RATE = 8000

audio_q = queue.Queue()


def capture():
    with serial.Serial(PORT, BAUD, timeout=5) as ser:
        print("Capturing — press Ctrl+C to stop")
        buf = bytearray()
        while True:
            buf += ser.read(512)
            # Find frame markers
            while True:
                idx = buf.find(b"\xff\xfe")
                if idx == -1:
                    buf = buf[-1:]  # keep last byte in case split marker
                    break
                frame_start = idx + 2
                frame_end = frame_start + NSAM * 2
                if len(buf) < frame_end:
                    break
                frame = buf[frame_start:frame_end]
                buf = buf[frame_end:]
                samples = (
                    np.frombuffer(frame, dtype=np.int16).astype(np.float32) / 32768.0
                )
                audio_q.put(samples)


def play():
    print("Playing...")
    with sd.OutputStream(samplerate=RATE, channels=1, dtype="float32") as stream:
        while True:
            samples = audio_q.get()
            stream.write(samples)


t = threading.Thread(target=capture, daemon=True)
t.start()
play()
