#!/usr/bin/env python3
import time, math, argparse, serial

# ---- Feetech protocol ----
PING, READ_DATA, WRITE_DATA, REG_WRITE, ACTION = 0x01, 0x02, 0x03, 0x04, 0x05
BROADCAST = 0xFE

# ---- Common control table ----
ADDR_TORQUE_ENABLE    = 0x28
ADDR_GOAL_POS         = 0x2A   # pos(L,H), block may include time/speed after
ADDR_PRESENT_POSITION = 0x38   # 2 bytes

# ---------------- Low-level helpers ----------------
def cs(d): return (~sum(d)) & 0xFF

def make_packet(sid, inst, params):
    body = [sid, 2 + len(params), inst] + list(params)
    return bytearray([0xFF, 0xFF] + body + [cs(body)])

def read_exact(ser, n, timeout_s=0.2):
    """Read exactly n bytes or return fewer if timeout."""
    out = bytearray()
    t0 = time.time()
    while len(out) < n and (time.time() - t0) < timeout_s:
        chunk = ser.read(n - len(out))
        if chunk:
            out.extend(chunk)
        else:
            time.sleep(0.001)
    return bytes(out)

def read_status_packet(ser, timeout_s=0.15):
    """
    Assemble one Feetech status packet:
      FF FF | ID | LEN | ERR | ...DATA... | CHK
    Return bytes or b"" if not enough data.
    """
    # seek header
    t0 = time.time()
    while (time.time() - t0) < timeout_s:
        b = ser.read(1)
        if not b: continue
        if b[0] != 0xFF:
            continue
        # got one FF, check next
        b2 = ser.read(1)
        if not b2: 
            continue
        if b2[0] != 0xFF:
            continue
        # header OK: read ID+LEN
        rest = read_exact(ser, 2, timeout_s)
        if len(rest) < 2:
            return b""
        sid, length = rest[0], rest[1]
        # length covers ERR..CHK inclusive; total remaining = length
        payload = read_exact(ser, length, timeout_s)
        if len(payload) < length: 
            return b""
        pkt = bytes([0xFF,0xFF,sid,length]) + payload
        # (Optional) verify checksum
        body = [sid, length] + list(payload[:-1])  # without final CHK
        if ((~sum(body)) & 0xFF) != payload[-1]:
            # bad checksum; return anyway so caller can debug
            return pkt
        return pkt
    return b""

def decode_present_position(pkt, expect_id=None):
    """
    pkt: FF FF ID LEN ERR D0 D1 CHK
    returns int position or None.
    """
    if len(pkt) < 7 or pkt[0]!=0xFF or pkt[1]!=0xFF:
        return None
    sid, ln = pkt[2], pkt[3]
    if expect_id is not None and sid != expect_id:
        return None
    # ln includes ERR..CHK => data length = ln - 2
    if 4 + ln != len(pkt):
        return None
    if ln < 3:  # ERR + at least 2 data + CHK => 1+2+1=4 => ln>=4; some firmwares ln=5 etc.
        return None
    err = pkt[4]
    # next two should be position
    if ln < 4:
        return None
    pos_lo = pkt[5] if len(pkt) > 5 else 0
    pos_hi = pkt[6] if len(pkt) > 6 else 0
    return (pos_lo | (pos_hi << 8))

# ---------------- Bus ops ----------------
def torque_on(ser, sid, on=True):
    ser.write(make_packet(sid, WRITE_DATA, [ADDR_TORQUE_ENABLE, 1 if on else 0]))
    ser.flush()
    time.sleep(0.01)

def write_goal_pos_only(ser, sid, ticks):
    pL, pH = ticks & 0xFF, (ticks >> 8) & 0xFF
    ser.write(make_packet(sid, WRITE_DATA, [ADDR_GOAL_POS, pL, pH]))
    ser.flush()

def write_goal_block(ser, sid, ticks, time_ms=0, speed=0):
    # For firmwares that support [pos(2), time(2), speed(2)]
    pL, pH = ticks & 0xFF, (ticks >> 8) & 0xFF
    tL, tH = time_ms & 0xFF, (time_ms >> 8) & 0xFF
    sL, sH = speed & 0xFF, (speed >> 8) & 0xFF
    ser.write(make_packet(sid, WRITE_DATA, [ADDR_GOAL_POS, pL, pH, tL, tH, sL, sH]))
    ser.flush()

def read_present_pos(ser, sid, delay=0.03):
    ser.reset_input_buffer()
    ser.write(make_packet(sid, READ_DATA, [ADDR_PRESENT_POSITION, 2]))
    ser.flush()
    pkt = read_status_packet(ser, timeout_s=max(0.05, delay))
    pos = decode_present_position(pkt, expect_id=sid)
    return pos, list(pkt) if pkt else []

# ---------------- CLI “move + monitor” ----------------
def main():
    ap = argparse.ArgumentParser(description="Feetech: move to ticks and print live position.")
    ap.add_argument("--port", required=True, help="COM port (e.g., COM4)")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Baud (default 1000000)")
    ap.add_argument("--id",   type=int, required=True, help="Servo ID")
    ap.add_argument("--pos",  type=int, required=True, help="Goal position (ticks)")
    ap.add_argument("--monitor", type=float, default=2.0, help="Seconds to print Present Position")
    ap.add_argument("--block", action="store_true", help="Use block write [pos,time,speed] instead of pos-only")
    ap.add_argument("--time_ms", type=int, default=0, help="Block field: time in ms")
    ap.add_argument("--speed", type=int, default=0, help="Block field: speed units (0=max)")
    ap.add_argument("--retry115200", action="store_true", help="If read fails, retry once at 115200")
    args = ap.parse_args()

    def run_once(baud):
        with serial.Serial(args.port, baud, timeout=0.1) as ser:
            print(f"Opened {args.port} @ {baud} (ID={args.id})")
            torque_on(ser, args.id, True)

            pos0, raw0 = read_present_pos(ser, args.id)
            print(f"Present before: {pos0} | raw={raw0}")

            if args.block:
                write_goal_block(ser, args.id, args.pos, args.time_ms, args.speed)
                print(f"Sent BLOCK → pos={args.pos}, time={args.time_ms}, speed={args.speed}")
            else:
                write_goal_pos_only(ser, args.id, args.pos)
                print(f"Sent POS-ONLY → pos={args.pos}")

            if args.monitor > 0:
                t0 = time.time()
                while time.time() - t0 < args.monitor:
                    pos, raw = read_present_pos(ser, args.id)
                    print(f"  present={pos} | raw={raw}")
                    time.sleep(0.1)
            return True

    ok = run_once(args.baud)
    # If you’re seeing bogus reads (e.g., 65532) or no motion at 1M, some SMS/SC models are at 115200.
    if args.retry115200 and not ok and args.baud != 115200:
        print("Retrying at 115200…")
        run_once(115200)

if __name__ == "__main__":
    main()
