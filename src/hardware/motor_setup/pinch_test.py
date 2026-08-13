#!/usr/bin/env python3
import time, argparse, serial, sys

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

# ---- Cross-platform non-blocking key reader ----
class KeyReader:
    def __init__(self):
        self._is_windows = (sys.platform.startswith('win'))
        if not self._is_windows:
            import termios, tty
            self._termios = termios
            self._tty = tty
            self._fd = sys.stdin.fileno()
            self._old = self._termios.tcgetattr(self._fd)

    def __enter__(self):
        if self._is_windows:
            return self
        # put stdin in raw, non-canonical mode
        new = self._termios.tcgetattr(self._fd)
        new[3] = new[3] & ~(self._termios.ICANON | self._termios.ECHO)
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, new)
        return self

    def __exit__(self, exc_type, exc, tb):
        if not self._is_windows:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    def getch(self):
        """Return a single lowercased char if available, else None."""
        if self._is_windows:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                # Handle Ctrl+C
                if ch in (b'\x03',):
                    raise KeyboardInterrupt
                try:
                    return ch.decode(errors='ignore').lower()
                except Exception:
                    return None
            return None
        else:
            import select
            r, _, _ = select.select([sys.stdin], [], [], 0)
            if r:
                ch = sys.stdin.read(1)
                if ch == '\x03':  # Ctrl+C
                    raise KeyboardInterrupt
                return ch.lower()
            return None

def main():
    ap = argparse.ArgumentParser(description="Keyboard-toggle Feetech gripper (ID=6).")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--id",   type=int, default=6, help="Gripper servo ID (default 6)")
    ap.add_argument("--open_ticks",  type=int, default=2500, help="Ticks for OPEN state")
    ap.add_argument("--close_ticks", type=int, default=2000, help="Ticks for CLOSED state")
    ap.add_argument("--poll", type=float, default=0.15, help="Seconds between status prints")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, KeyReader() as kr:
        # Power up/enable torque
        torque_on(ser, args.id, True)

        # Start CLOSED by default; press keys to change
        is_open = False
        last_sent_ticks = None

        print("\nControls: [o]=open  [c]=close  [space]=toggle  [q]=quit  (Ctrl+C also works)\n")
        try:
            while True:
                ch = kr.getch()
                if ch:
                    if ch == 'q':
                        break
                    elif ch == 'o':
                        is_open = True
                    elif ch == 'c':
                        is_open = False
                    elif ch == ' ':
                        is_open = not is_open

                desired_ticks = args.open_ticks if is_open else args.close_ticks
                if desired_ticks != last_sent_ticks:
                    write_pos(ser, args.id, desired_ticks)
                    last_sent_ticks = desired_ticks
                    print(f"[CMD] Gripper {'OPEN ' if is_open else 'CLOSED'} -> {desired_ticks} ticks")

                rp = read_pos(ser, args.id)  # live feedback
                print(f"ID {args.id} | target={desired_ticks} | present={rp if rp is not None else 'None'} | state={'OPEN' if is_open else 'CLOSED'}")
                time.sleep(args.poll)

        except KeyboardInterrupt:
            pass

        # Optionally torque off on exit (comment out if you want to keep holding)
        # torque_on(ser, args.id, False)
        print("Exiting.")

if __name__ == "__main__":
    main()
s