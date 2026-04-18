import serial
import sys
import time

PORT = "COM3"
BAUD = 921600
OUTFILE = "rx_capture.raw"

FRAME_LEN = 160 * 2  # 320 bytes
HEADER = 6
MARKER = b"\xff\xfe"
TARGET_FRAMES = 500  # 10 s × 50 frames/s

stats = {"frames": 0, "bad_len": 0, "bad_chk": 0, "bad_seq": 0}
expected_seq = None


def parse_frames(buf, outfile):
    global expected_seq
    while True:
        idx = buf.find(MARKER)
        if idx == -1:
            buf = buf[-(HEADER + FRAME_LEN) :]
            break

        if len(buf) < idx + HEADER + FRAME_LEN:
            buf = buf[idx:]
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
            missed = (seq - expected_seq) & 0xFF
            stats["bad_seq"] += missed
            print(f"  SEQ GAP: expected {expected_seq} got {seq} ({missed} missed)")
        expected_seq = (seq + 1) & 0xFF

        outfile.write(payload)
        outfile.flush()
        stats["frames"] += 1
        buf = buf[idx + HEADER + FRAME_LEN :]

        if stats["frames"] >= TARGET_FRAMES:
            break

    return buf


def main():
    print(
        f"Capturing {TARGET_FRAMES} frames ({TARGET_FRAMES / 50:.0f}s of audio) to {OUTFILE}..."
    )
    print(f"Port: {PORT} at {BAUD} baud")
    print(f"Waiting as long as needed...\n")

    t_start = time.time()
    t_report = t_start + 2

    with serial.Serial(PORT, BAUD, timeout=5) as ser, open(OUTFILE, "wb") as outfile:
        buf = bytearray()

        while stats["frames"] < TARGET_FRAMES:
            chunk = ser.read(ser.in_waiting or 1)
            buf += chunk
            buf = parse_frames(buf, outfile)

            if time.time() > t_report:
                elapsed = time.time() - t_start
                remaining = TARGET_FRAMES - stats["frames"]
                rate = stats["frames"] / elapsed if elapsed > 0 else 0
                eta = f"{remaining / rate:.0f}s" if rate > 0 else "?"
                print(
                    f"  {elapsed:.0f}s elapsed  "
                    f"frames={stats['frames']}/{TARGET_FRAMES}  "
                    f"({rate:.1f}/s)  "
                    f"ETA={eta}  "
                    f"bad_len={stats['bad_len']}  "
                    f"bad_chk={stats['bad_chk']}  "
                    f"bad_seq={stats['bad_seq']}"
                )
                t_report += 2

    elapsed = time.time() - t_start
    audio_s = TARGET_FRAMES / 50
    print(f"\nDone. Captured {audio_s:.0f}s of audio in {elapsed:.1f}s real time.")
    print(
        f"  bad_len={stats['bad_len']}  "
        f"bad_chk={stats['bad_chk']}  "
        f"bad_seq={stats['bad_seq']}"
    )
    print(f"  Saved to {OUTFILE} — play with:")
    print(f"  aplay -f S16_LE -r 8000 -c 1 {OUTFILE}")
    print(
        f"  sox -r 8000 -e signed -b 16 -c 1 {OUTFILE} {OUTFILE.replace('.raw', '.wav')}"
    )


if __name__ == "__main__":
    main()
