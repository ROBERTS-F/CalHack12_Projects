#!/usr/bin/env python3
import time, argparse, serial

# ---- Feetech protocol ----
READ_DATA  = 0x02
WRITE_DATA = 0x03

# ---- Control table ----
ADDR_TORQUE_ENABLE = 0x28   # 1 = on, 0 = off
ADDR_GOAL_POSITION = 0x2A   # 2 bytes (little endian)
ADDR_PRESENT_POS   = 0x38   # 2 bytes

def checksum(bs): return (~sum(bs)) & 0xFF
def pkt(sid, inst, params):
    body = [sid, 2 + len(params), inst] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def torque_on(ser, sid, on=True):
    ser.write(pkt(sid, WRITE_DATA, [ADDR_TORQUE_ENABLE, 1 if on else 0])); ser.flush(); time.sleep(0.01)

def write_pos(ser, sid, pos_ticks):
    lo, hi = pos_ticks & 0xFF, (pos_ticks >> 8) & 0xFF
    ser.write(pkt(sid, WRITE_DATA, [ADDR_GOAL_POSITION, lo, hi])); ser.flush()

def read_pos(ser, sid, delay=0.04):
    ser.reset_input_buffer()
    ser.write(pkt(sid, READ_DATA, [ADDR_PRESENT_POS, 2])); ser.flush()
    time.sleep(delay)
    b = list(ser.read(ser.in_waiting or 1))
    if len(b) >= 8 and b[0]==0xFF and b[1]==0xFF and b[2]==sid:
        return b[5] | (b[6] << 8)
    return None

def main():
    ap = argparse.ArgumentParser(description="Move multiple Feetech servos (position-only writes) and print feedback.")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids",  nargs="+", type=int, required=True, help="Servo IDs, e.g. 1 2")
    ap.add_argument("--pos",  nargs="+", type=int, required=True, help="Target ticks, e.g. 2000 1000")
    ap.add_argument("--monitor", type=float, default=3.0, help="Seconds to print live positions")
    ap.add_argument("--stagger_ms", type=int, default=0, help="Optional small stagger between writes (0..5 ms)")
    args = ap.parse_args()

    if len(args.ids) != len(args.pos):
        raise SystemExit("ids and pos must have the same length")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        # 1) torque on all (same as jog_tick)
        for sid in args.ids:
            torque_on(ser, sid, True)

        # 2) send simple position-only WRITE to each (fast = “software sync”)
        for i, (sid, p) in enumerate(zip(args.ids, args.pos)):
            write_pos(ser, sid, int(p))
            if args.stagger_ms > 0 and i < len(args.ids) - 1:
                time.sleep(args.stagger_ms / 1000.0)

        print(f"Sent: {dict(zip(args.ids, args.pos))}")

        # 3) monitor present positions
        if args.monitor > 0:
            t0 = time.time()
            while time.time() - t0 < args.monitor:
                line = []
                for sid in args.ids:
                    rp = read_pos(ser, sid)
                    line.append(f"ID {sid}: {rp if rp is not None else 'None'}")
                print(" | ".join(line))
                time.sleep(0.15)

if __name__ == "__main__":
    main()
