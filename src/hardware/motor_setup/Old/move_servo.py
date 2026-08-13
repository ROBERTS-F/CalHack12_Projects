import time
import serial

# --------- EDIT THESE IF NEEDED ----------
PORT = "COM4"       # your port
BAUD = 1_000_000    # common default for Feetech bus servos
ID   = 1            # the servo ID you just set
# -----------------------------------------

# Feetech protocol constants (SCS/SMS/STS family)
PING       = 0x01
READ_DATA  = 0x02
WRITE_DATA = 0x03
REG_WRITE  = 0x04
ACTION     = 0x05

# Common control table addresses (Feetech bus servos)
ADDR_ID               = 0x05
ADDR_TORQUE_ENABLE    = 0x28   # 1 = on, 0 = off
ADDR_GOAL_POSITION    = 0x2A   # 2 bytes: little-endian
ADDR_PRESENT_POSITION = 0x38   # 2 bytes: little-endian

def checksum(data_bytes):
    # checksum is the bitwise NOT of the sum of bytes [ID..last param]
    return (~sum(data_bytes)) & 0xFF

def packet(id_, instruction, params):
    body = [id_, 2 + len(params), instruction] + params  # length = inst(1)+params+n + checksum(1), but spec uses "length" excluding 0xFF,0xFF and including instruction+params+checksum
    # For Feetech, LENGTH often = len([instruction] + params + [checksum])  -> we fill actual checksum after
    cs = checksum(body)
    return bytearray([0xFF, 0xFF] + body + [cs])

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
    # Expect a status packet: FF FF ID LEN ERR <data...> CHK
    b = list(status_bytes)
    if len(b) < 6 or b[0] != 0xFF or b[1] != 0xFF:
        return None
    data_len = b[3] - 2  # LEN includes ERR + DATA + CHK; data count = LEN-2
    if data_len <= 0 or len(b) < 5 + data_len:
        return None
    # data starts at index 5 (after FF FF ID LEN ERR)
    data0 = b[5:5+data_len]
    if len(data0) >= 2:
        return data0[0] | (data0[1] << 8)
    return None

def enable_torque(ser, id_, on=True):
    write_byte(ser, id_, ADDR_TORQUE_ENABLE, 1 if on else 0)

def read_present_position(ser, id_):
    resp = read_bytes(ser, id_, ADDR_PRESENT_POSITION, 2)
    pos = decode_present_position(resp)
    return pos, list(resp)

def move_to_ticks(ser, id_, ticks):
    # Typical Feetech range is around 0..1000/4095 depending on model.
    write_word(ser, id_, ADDR_GOAL_POSITION, int(ticks))

def main():
    with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
        print(f"Opened {PORT} @ {BAUD} bps for ID {ID}")
        # Turn torque on (some firmwares default to on already)
        enable_torque(ser, ID, True)

        # Read current ticks
        cur, raw = read_present_position(ser, ID)
        print(f"Present position: {cur} ticks | raw={raw}")

        # Simple demo: move between three positions in ticks.
        # Start conservative; we’ll see what range your servo reports.
        demo_positions = []
        if cur is not None:
            demo_positions = [max(cur-100, 0), cur, cur+100]
        else:
            demo_positions = [300, 500, 700]  # fallback guesses

        for tgt in demo_positions:
            print(f"\nMoving to {tgt} ticks ...")
            move_to_ticks(ser, ID, tgt)

            # poll until it settles (or timeout)
            t0 = time.time()
            while time.time() - t0 < 3.0:
                pos, _ = read_present_position(ser, ID)
                if pos is not None:
                    print(f"  pos={pos} ticks")
                time.sleep(0.1)

        print("\nDone. If it didn’t move, try smaller steps (+/- 20), or try a wider range (e.g., 200 ↔ 800).")

if __name__ == "__main__":
    main()
