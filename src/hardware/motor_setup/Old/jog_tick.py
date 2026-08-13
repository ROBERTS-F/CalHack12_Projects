#!/usr/bin/env python3
import time
import argparse
import serial

# ---- Protocol constants (Feetech SCS/SMS/STS) ----
PING       = 0x01
READ_DATA  = 0x02
WRITE_DATA = 0x03
REG_WRITE  = 0x04
ACTION     = 0x05

# ---- Control table addresses ----
ADDR_ID               = 0x05
ADDR_TORQUE_ENABLE    = 0x28   # 1 = on, 0 = off
ADDR_GOAL_POSITION    = 0x2A   # 2 bytes: little-endian
ADDR_PRESENT_POSITION = 0x38   # 2 bytes: little-endian

def checksum(data_bytes):
    return (~sum(data_bytes)) & 0xFF  # sum over [ID..last_param], then invert

def packet(id_, instruction, params):
    body = [id_, 2 + len(params), instruction] + params
    return bytearray([0xFF, 0xFF] + body + [checksum(body)])

def write_byte(ser, id_, addr, val):
    ser.write(packet(id_, WRITE_DATA, [addr, val & 0xFF]))
    time.sleep(0.02)

def write_word(ser, id_, addr, value):
    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    ser.write(packet(id_, WRITE_DATA, [addr, lo, hi]))
    time.sleep(0.02)

def read_bytes(ser, id_, addr, length):
    ser.write(packet(id_, READ_DATA, [addr, length]))
    time.sleep(0.02)
    return ser.read(ser.in_waiting or 1)

def decode_present_position(status_bytes):
    b = list(status_bytes)
    if len(b) < 6 or b[0] != 0xFF or b[1] != 0xFF:
        return None
    data_len = b[3] - 2  # LEN includes ERR + DATA + CHK; data count = LEN-2
    if data_len <= 0 or len(b) < 5 + data_len:
        return None
    data0 = b[5:5+data_len]
    if len(data0) >= 2:
        return data0[0] | (data0[1] << 8)
    return None

def enable_torque(ser, id_, on=True):
    write_byte(ser, id_, ADDR_TORQUE_ENABLE, 1 if on else 0)

def read_present_position(ser, id_):
    resp = read_bytes(ser, id_, ADDR_PRESENT_POSITION, 2)
    return decode_present_position(resp), list(resp)

def move_to_ticks(ser, id_, ticks):
    write_word(ser, id_, ADDR_GOAL_POSITION, int(ticks))

def main():
    ap = argparse.ArgumentParser(description="Move a Feetech bus servo to a tick.")
    ap.add_argument("--port", default="COM4", help="Serial port (e.g., COM4 or /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Baud rate (default 1000000)")
    ap.add_argument("--id",   type=int, default=1, help="Servo ID (default 1)")
    ap.add_argument("--pos",  type=int, required=True, help="Goal position in ticks (e.g., 1200)")
    ap.add_argument("--monitor", type=float, default=2.0,
                    help="Seconds to print present position after the command (default 2.0; set 0 to skip)")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Opened {args.port} @ {args.baud} bps for ID {args.id}")
        enable_torque(ser, args.id, True)

        cur, raw = read_present_position(ser, args.id)
        print(f"Present before: {cur} ticks | raw={raw}")

        print(f"Moving to {args.pos} ticks ...")
        move_to_ticks(ser, args.id, args.pos)

        if args.monitor > 0:
            t0 = time.time()
            while time.time() - t0 < args.monitor:
                pos, _ = read_present_position(ser, args.id)
                if pos is not None:
                    print(f"  pos={pos} ticks")
                time.sleep(0.1)

        print("Done.")

if __name__ == "__main__":
    main()
