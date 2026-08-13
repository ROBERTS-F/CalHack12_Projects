#!/usr/bin/env python3
import time, argparse, serial

# ---- Feetech protocol ----
READ_DATA  = 0x02
ADDR_PRESENT_POS = 0x38   # 2 bytes

def checksum(bs): 
    return (~sum(bs)) & 0xFF

def pkt(sid, inst, params):
    body = [sid, 2 + len(params), inst] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def read_pos(ser, sid, delay=0.04):
    ser.reset_input_buffer()
    ser.write(pkt(sid, READ_DATA, [ADDR_PRESENT_POS, 2])); ser.flush()
    time.sleep(delay)
    b = list(ser.read(ser.in_waiting or 1))
    if len(b) >= 8 and b[0]==0xFF and b[1]==0xFF and b[2]==sid:
        return b[5] | (b[6] << 8)
    return None

def main():
    ap = argparse.ArgumentParser(description="Read Feetech servo positions.")
    ap.add_argument("--port", default="COM5")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True, help="Servo IDs, e.g. 1 2 3")
    ap.add_argument("--interval", type=float, default=0.2, help="Time between reads (s)")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Reading positions on {args.port}...")
        while True:
            line = []
            for sid in args.ids:
                pos = read_pos(ser, sid)
                line.append(f"ID {sid}: {pos if pos is not None else 'None'}")
            print(" | ".join(line))
            time.sleep(args.interval)

if __name__ == "__main__":
    main()
