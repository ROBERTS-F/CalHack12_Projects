#!/usr/bin/env python3
import time, serial, argparse

# ---- Feetech protocol ----
PING, READ, WRITE = 0x01, 0x02, 0x03
ADDR_ID           = 0x05
ADDR_TORQUE_EN    = 0x28

# Lock candidates used by different Feetech firmwares
LOCK_ADDR_CANDIDATES = [0x30, 0x37, 0x2F]  # try all

def cs(d): return (~sum(d)) & 0xFF
def pkt(i, inst, params):
    body = [i, 2 + len(params), inst] + list(params)
    return bytearray([0xFF, 0xFF] + body + [cs(body)])

def wbyte(ser, sid, addr, val, delay=0.02):
    ser.write(pkt(sid, WRITE, [addr, val & 0xFF])); ser.flush(); time.sleep(delay)

def rbytes(ser, sid, addr, n, delay=0.04):
    ser.write(pkt(sid, READ, [addr, n])); ser.flush(); time.sleep(delay)
    b = list(ser.read(ser.in_waiting or 1))
    # expect: FF FF ID LEN ERR DATA.. CHK
    if len(b) >= 7 and b[0]==0xFF and b[1]==0xFF and b[2]==sid:
        # single byte read is at b[5]
        return b[5:5+n]
    return None

def ping(ser, sid, delay=0.02):
    ser.write(pkt(sid, PING, [])); ser.flush(); time.sleep(delay)
    b = list(ser.read(ser.in_waiting or 1))
    return (len(b) >= 4 and b[0]==0xFF and b[1]==0xFF)

def find_servo(port, bauds=(1_000_000, 115200), ids=range(1, 254)):
    """Return (baud, id) of the first responding servo, or (None, None)."""
    for baud in bauds:
        try:
            with serial.Serial(port, baud, timeout=0.1) as ser:
                for sid in ids:
                    ser.reset_input_buffer()
                    if ping(ser, sid):
                        return baud, sid
        except Exception:
            pass
    return None, None

def unlock_any(ser, sid):
    for a in LOCK_ADDR_CANDIDATES:
        try:
            wbyte(ser, sid, a, 0)  # unlock
        except Exception:
            pass

def relock_any(ser, sid):
    for a in LOCK_ADDR_CANDIDATES:
        try:
            wbyte(ser, sid, a, 1)  # relock
        except Exception:
            pass

def set_id(port, target_new_id, old_id=None, baud_hint=1_000_000):
    # Step 0: detect baud/ID if not provided
    baud = baud_hint
    sid  = old_id
    if sid is None:
        fb, fi = find_servo(port, bauds=(baud_hint, 115200))
        if fb is None:
            print("❌ No servo found on the bus (check power, wiring, and adapter).")
            return False
        baud, sid = fb, fi
        print(f"Detected servo: ID={sid} @ {baud} bps")

    with serial.Serial(port, baud, timeout=0.1) as ser:
        # Torque OFF (some firmwares require RAM quiet to update EEPROM)
        wbyte(ser, sid, ADDR_TORQUE_EN, 0)
        time.sleep(0.05)

        # Unlock (try all known lock addresses)
        unlock_any(ser, sid)
        time.sleep(0.05)

        # Write new ID
        wbyte(ser, sid, ADDR_ID, target_new_id)
        print(f"Wrote ID {sid} → {target_new_id}")
        time.sleep(0.08)

        # Re-lock using the NEW id (device should now answer as new id)
        relock_any(ser, target_new_id)
        time.sleep(0.05)

        # Optional: torque ON again
        wbyte(ser, target_new_id, ADDR_TORQUE_EN, 1)

        # Verify via read (may be None on some adapters)
        rb = rbytes(ser, target_new_id, ADDR_ID, 1)
        print(f"Read-back at new ID: {rb}")

    # Final: power-cycle recommended for confidence test
    print("🔌 Power-cycle the servo, then verify by pinging the NEW ID.")
    return True

if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Persistently set Feetech servo ID (handles EEPROM lock).")
    ap.add_argument("--port", required=True, help="e.g., COM4")
    ap.add_argument("--new",  type=int, required=True, help="New ID (1..253)")
    ap.add_argument("--old",  type=int, help="Current ID (if unknown, tool will search)")
    ap.add_argument("--baud", type=int, default=1_000_000, help="Hint baud (also tries 115200 when searching)")
    args = ap.parse_args()
    ok = set_id(args.port, args.new, old_id=args.old, baud_hint=args.baud)
    if not ok: exit(1)
