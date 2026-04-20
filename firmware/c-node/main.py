import serial
import hmac
import hashlib
import struct
import time

# ---------- CONFIG ----------
START = 0x7E
END = 0x7F
TAG_SIZE = 32

KEY = b"roborebound-demo-key"

SERIAL_PORT = "/dev/serial0"   # change if needed
BAUD = 115200

# ---------- PAYLOAD FORMAT ----------
# MUST match STM32 struct EXACTLY
PAYLOAD_FMT = "<I B i i i B B h h h h h h h B h B B"

# ---------- SERIAL HELPERS ----------
def read_exact(ser, n):
    buf = b""
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

# ---------- HMAC ----------
def verify_hmac(payload, rx_tag):
    calc = hmac.new(KEY, payload, hashlib.sha256).digest()
    return hmac.compare_digest(calc, rx_tag)

# ---------- DECODE ----------
def decode_payload(payload):
    fields = struct.unpack(PAYLOAD_FMT, payload)

    return {
        "timestamp": fields[0],

        "gps_valid": fields[1],
        "lat": fields[2] / 1e7,
        "lon": fields[3] / 1e7,
        "alt": fields[4] / 1000.0,
        "sats": fields[5],

        "imu_valid": fields[6],
        "ax": fields[7] / 1000.0,
        "ay": fields[8] / 1000.0,
        "az": fields[9] / 1000.0,
        "gx": fields[10] / 100.0,
        "gy": fields[11] / 100.0,
        "gz": fields[12] / 100.0,
        "temp": fields[13] / 100.0,

        "vision_valid": fields[14],
        "line_error": fields[15],
        "obstacle": fields[16],
        "confidence": fields[17],
    }

# ---------- PRINT ----------
def print_data(d):
    print(f"\nTime: {d['timestamp']} ms")

    if d["imu_valid"]:
        print(f"IMU: ax={d['ax']:.2f} ay={d['ay']:.2f} az={d['az']:.2f} g | "
              f"gx={d['gx']:.2f} gy={d['gy']:.2f} gz={d['gz']:.2f} dps | "
              f"T={d['temp']:.2f} C")

    if d["gps_valid"]:
        print(f"GPS: lat={d['lat']:.6f} lon={d['lon']:.6f} alt={d['alt']:.2f} m sats={d['sats']}")
    else:
        print("GPS: no fix")

    if d["vision_valid"]:
        print(f"VISION: line={d['line_error']} obstacle={d['obstacle']} conf={d['confidence']}")
    else:
        print("VISION: invalid")

# ---------- MAIN ----------
def main():
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    print(f"Listening on {SERIAL_PORT} @ {BAUD}")

    while True:
        b = ser.read(1)
        if not b:
            continue

        if b[0] != START:
            continue

        # length
        length_bytes = read_exact(ser, 2)
        if not length_bytes:
            continue

        payload_len = (length_bytes[0] << 8) | length_bytes[1]

        payload = read_exact(ser, payload_len)
        tag = read_exact(ser, TAG_SIZE)
        end_byte = read_exact(ser, 1)

        if not payload or not tag or not end_byte:
            print("Frame read error")
            continue

        if end_byte[0] != END:
            print("Bad frame end")
            continue

        match = verify_hmac(payload, tag)
        print(f"\nHMAC match = {match}")

        if not match:
            continue

        try:
            data = decode_payload(payload)
            print_data(data)
        except Exception as e:
            print("Decode error:", e)

        time.sleep(0.05)

# ---------- RUN ----------
if __name__ == "__main__":
    main()