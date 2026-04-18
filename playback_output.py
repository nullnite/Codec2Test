import serial
import numpy as np
import sounddevice
import queue
import threading
import time

PORT = "COM3"
BAUD = 921600
NSAM = 160
RATE = 8000
FRAME_LEN = NSAM * 2  # 320 bytes
HEADER = 6  # 0xFF 0xFE seq len_lo len_hi chk
MARKER = b"\xff\xfe"

audio_q = queue.Queue(maxsize=2000)
stats = {
    "frames": 0,
    "underruns": 0,
    "overruns": 0,
    "bad_len": 0,
    "bad_chk": 0,
    "bad_seq": 0,
}
expected_seq = None


def audio_callback(outdata, frames, time_info, status):
    try:
        data = audio_q.get_nowait()
        outdata[:, 0] = data[:frames]
    except queue.Empty:
        outdata[:] = 0
        stats["underruns"] += 1


def parse_frames(buf):
    global expected_seq
    frames = []
    while True:
        idx = buf.find(MARKER)
        if idx == -1:
            buf = buf[-(HEADER + FRAME_LEN) :]  # keep tail in case marker is split
            break

        if len(buf) < idx + HEADER + FRAME_LEN:
            buf = buf[idx:]  # keep from marker onwards
            break

        seq = buf[idx + 2]
        ln = buf[idx + 3] | (buf[idx + 4] << 8)
        chk = buf[idx + 5]

        if ln != FRAME_LEN:
            stats["bad_len"] += 1
            buf = buf[idx + 1 :]
            continue

        payload = buf[idx + HEADER : idx + HEADER + FRAME_LEN]

        computed = 0
        for b in payload:
            computed ^= b
        if computed != chk:
            stats["bad_chk"] += 1
            buf = buf[idx + 1 :]
            continue

        if expected_seq is not None and seq != expected_seq:
            stats["bad_seq"] += 1
        expected_seq = (seq + 1) & 0xFF

        samples = (
            np.frombuffer(bytes(payload), dtype=np.int16).astype(np.float32) / 32768.0
        )
        frames.append(samples)
        stats["frames"] += 1
        buf = buf[idx + HEADER + FRAME_LEN :]

    return buf, frames


def capture():
    with serial.Serial(PORT, BAUD, timeout=5) as ser:
        print(f"Capturing on {PORT} at {BAUD}...")
        buf = bytearray()
        total_bytes = 0
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            buf += chunk
            total_bytes += len(chunk)
            buf, frames = parse_frames(buf)
            # if frames:
            #     print(
            #         f"Got {len(frames)} frames, total_bytes={total_bytes}, "
            #         f"bad_len={stats['bad_len']} bad_chk={stats['bad_chk']}"
            #     )
            for f in frames:
                try:
                    audio_q.put_nowait(f)
                except queue.Full:
                    audio_q.get_nowait()
                    audio_q.put_nowait(f)
                    stats["overruns"] += 1


def play():
    print("Buffering...")
    deadline = time.time() + 30
    while audio_q.qsize() < 60 and time.time() < deadline:
        time.sleep(0.1)
        if audio_q.qsize() > 0:
            print(
                f"  buffering: {audio_q.qsize()} frames, "
                f"bad_len={stats['bad_len']} bad_chk={stats['bad_chk']} bad_seq={stats['bad_seq']}"
            )

    print(f"Starting with {audio_q.qsize()} frames buffered")
    print(
        f"Parse stats: frames={stats['frames']} bad_len={stats['bad_len']} "
        f"bad_chk={stats['bad_chk']} bad_seq={stats['bad_seq']}"
    )

    # List available devices so we can diagnose
    print(sounddevice.query_devices())

    try:
        with sounddevice.OutputStream(
            samplerate=RATE,
            channels=1,
            dtype="float32",
            blocksize=NSAM,
            callback=audio_callback,
            latency="high",
        ) as stream:
            print(
                f"Stream opened: {stream.samplerate}Hz "
                f"latency={stream.latency:.3f}s"
            )
            last = stats.copy()
            while True:
                time.sleep(2)
                d = {k: stats[k] - last[k] for k in stats}
                last = stats.copy()
                print(
                    f"Queue:{audio_q.qsize():3d}  "
                    f"Frames:{d['frames']}  "
                    f"Underruns:{d['underruns']}  "
                    f"Overruns:{d['overruns']}  "
                    f"bad_len:{d['bad_len']}  "
                    f"bad_chk:{d['bad_chk']}  "
                    f"bad_seq:{d['bad_seq']}"
                )
    except Exception as e:
        print(f"Stream error: {e}")
        import traceback

        traceback.print_exc()


threading.Thread(target=capture, daemon=True).start()

try:
    play()
except KeyboardInterrupt:
    print("Done")
