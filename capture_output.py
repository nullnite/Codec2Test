import serial

PORT = "COM3"  # adjust
BAUD = 115200

with serial.Serial(PORT, BAUD, timeout=15) as ser:
    print("Waiting for device...")

    while True:
        line = ser.readline()
        print(line.decode(errors="replace").strip())
        if b"BEGIN_C2" in line:
            break

    buf = bytearray()
    while True:
        chunk = ser.read(64)
        if b"END_C2" in chunk:
            buf += chunk[: chunk.index(b"END_C2")]
            break
        buf += chunk

    buf = buf.rstrip(b"\r\n")

    with open("device_output_c2.bit", "wb") as f:
        f.write(buf)

    print(f"Saved {len(buf)} bytes ({len(buf)//6} frames)")
