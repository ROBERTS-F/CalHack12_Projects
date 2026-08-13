#!/usr/bin/env python3
import time, math, argparse, serial

# Feetech protocol
PING, READ, WRITE, REG_WRITE, ACTION = 0x01, 0x02, 0x03, 0x04, 0x05
BROADCAST = 0xFE

# Control table
ADDR_TORQUE_ENABLE = 0x28
ADDR_GOAL_POS      = 0x2A
ADDR_PRESENT_POS   = 0x38

def cs(d): return (~sum(d)) & 0xFF
def pkt(sid, inst, params):
    body = [sid, 2 + len(params), inst] + list(params)
    return bytearray([0xFF, 0xFF] + body + [cs(body)])

def torque_on(ser, sid, on=True):
    ser.write(pkt(sid, WRITE, [ADDR_TORQUE_ENABLE, 1 if on else 0]))
    ser.flush()
    time.sleep(0.01)

def reg_write_pos_only(ser, sid, pos):
    pL, pH = pos & 0xFF, (pos >> 8) & 0xFF
    ser.write(pkt(sid, REG_WRITE, [ADDR_GOAL_POS, pL, pH]))
    ser.flush()

def reg_write_block(ser, sid, pos, t_ms=0, speed=0):
    pL, pH = pos & 0xFF, (pos >> 8) & 0xFF
    tL, tH = t_ms & 0xFF, (t_ms >> 8) & 0xFF
    sL, sH = speed & 0xFF, (speed >> 8) & 0xFF
    ser.write(pkt(sid, REG_WRITE, [ADDR_GOAL_POS, pL, pH, tL, tH, sL, sH]))
    ser.flush()

def action(ser):
    ser.write(pkt(BROADCAST, ACTION, []))
    ser.flush()

def read_pos(ser, sid, delay=0.05):
    """Returns current position in ticks, or None if no reply."""
    ser.reset_input_buffer()
    ser.write(pkt(sid, READ, [ADDR_PRESENT_POS, 2]))
    ser.flush()
    time.sleep(delay)
    b = list(ser.read(ser.in_waiting or 1))
    if len(b) >= 8 and b[0]==0xFF and b[1]==0xFF and b[2]==sid:
        return b[5] | (b[6]<<8)
    return None

def sync_move(ser, ids, positions, block=False, t_ms=0, speed=0):
    """Stage each goal with REG_WRITE, then ACTION for simultaneous start."""
    for sid in ids:
        torque_on(ser, sid, True)
    for sid, pos in zip(ids, positions):
        if block:
            reg_write_block(ser, sid, pos, t_ms, speed)
        else:
            reg_write_pos_only(ser, sid, pos)
    action(ser)

def main():
    ap = argparse.ArgumentParser(description="Simultaneous move for multiple Feetech servos with position feedback.")
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--ids", nargs="+", type=int, required=True, help="Servo IDs (e.g. 1 2 3)")
    ap.add_argument("--pos", nargs="+", type=int, required=True, help="Target positions in ticks (e.g. 1200 900 1500)")
    ap.add_argument("--block", action="store_true", help="Use block write (pos,time,speed)")
    ap.add_argument("--time_ms", type=int, default=0)
    ap.add_argument("--speed", type=int, default=0)
    ap.add_argument("--monitor", type=float, default=3.0,
                    help="Seconds to print live current positions after move (default 3.0)")
    args = ap.parse_args()

    if len(args.ids) != len(args.pos):
        raise SystemExit("❌ ids and pos must have the same length")

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Opened {args.port} @ {args.baud} bps")
        sync_move(ser, args.ids, args.pos, block=args.block, t_ms=args.time_ms, speed=args.speed)
        print(f"Sent simultaneous move → {dict(zip(args.ids, args.pos))}")

        if args.monitor > 0:
            t0 = time.time()
            while time.time() - t0 < args.monitor:
                readings = []
                for sid in args.ids:
                    pos = read_pos(ser, sid)
                    if pos is not None:
                        readings.append(f"ID {sid}: {pos:4d}")
                print(" | ".join(readings))
                time.sleep(0.15)
        print("✅ Done.")

if __name__ == "__main__":
    main()
