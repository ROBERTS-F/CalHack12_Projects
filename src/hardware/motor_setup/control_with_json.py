#!/usr/bin/env python3
import time, argparse, serial, sys, os, json
from datetime import datetime, timezone

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
    def __init__(self, enabled=True):
        self.enabled = enabled
        if not enabled:
            self._is_windows = None
            return
        self._is_windows = (sys.platform.startswith('win'))
        if not self._is_windows:
            import termios, tty
            self._termios = termios
            self._tty = tty
            self._fd = sys.stdin.fileno()
            self._old = self._termios.tcgetattr(self._fd)

    def __enter__(self):
        if not self.enabled:
            return self
        if self._is_windows:
            return self
        new = self._termios.tcgetattr(self._fd)
        new[3] = new[3] & ~(self._termios.ICANON | self._termios.ECHO)
        self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, new)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and not self._is_windows:
            self._termios.tcsetattr(self._fd, self._termios.TCSADRAIN, self._old)

    def getch(self):
        if not self.enabled:
            return None
        if self._is_windows:
            import msvcrt
            if msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x03',):  # Ctrl+C
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
                if ch == '\x03':
                    raise KeyboardInterrupt
                return ch.lower()
            return None

# ---- JSON status parsing ----
def _parse_iso_z(s):
    # Accept ISO with 'Z'
    try:
        if s.endswith('Z'):
            return datetime.fromisoformat(s.replace('Z', '+00:00'))
        return datetime.fromisoformat(s)
    except Exception:
        return None

def grip_from_json_file(path):
    """
    Returns tuple: (grip_state: 'open'|'close'|None, source: 'ws'|'http'|None, extra_info: dict)
    Chooses the newer of ws.lastMessageText.data.gripState and http.lastBody.gripState.
    """
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
    except Exception as e:
        return (None, None, {"error": f"read/parse error: {e}"})

    best_state, best_src, best_time = None, None, None

    # Try ws.lastMessageText
    try:
        ws = data.get("ws") or {}
        last_text = ws.get("lastMessageText")
        last_at   = _parse_iso_z(ws.get("lastAt", "")) or None
        if isinstance(last_text, str):
            j = json.loads(last_text)  # nested JSON string
            # expect {"type":"sample","data":{"gripState":"open","gripStrength":...,"t":...}}
            state = (j.get("data") or {}).get("gripState")
            if isinstance(state, str):
                state_norm = state.strip().lower()
                if state_norm in ("open", "close", "closed"):
                    best_state, best_src, best_time = state_norm, "ws", last_at
    except Exception:
        pass

    # Try http.lastBody.gripState
    try:
        http = data.get("http") or {}
        body = http.get("lastBody") or {}
        state = body.get("gripState")
        last_at = _parse_iso_z(http.get("lastAt", "")) or None
        if isinstance(state, str):
            state_norm = state.strip().lower()
            if state_norm in ("open", "close", "closed"):
                if best_time is None or (last_at and best_time and last_at > best_time) or (best_time is None and last_at):
                    best_state, best_src, best_time = state_norm, "http", last_at
    except Exception:
        pass

    # Normalize "closed" -> "close"
    if best_state == "closed":
        best_state = "close"

    return (best_state, best_src, {"timestamp": best_time.isoformat() if isinstance(best_time, datetime) else None})

def main():
    ap = argparse.ArgumentParser(description="JSON-driven Feetech gripper control (ID=6).")
    ap.add_argument("--port", required=True, help="Serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--id",   type=int, default=6, help="Gripper servo ID (default 6)")
    ap.add_argument("--open_ticks",  type=int, default=2500, help="Ticks for OPEN state")
    ap.add_argument("--close_ticks", type=int, default=2000, help="Ticks for CLOSED state")
    ap.add_argument("--poll", type=float, default=0.20, help="Seconds between loops")
    ap.add_argument("--status_json", type=str, required=True, help="Path to status.json (with http/ws fields)")
    ap.add_argument("--keyboard", action="store_true", help="Allow keyboard overrides (o/c/space/q)")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, KeyReader(enabled=args.keyboard) as kr:
        torque_on(ser, args.id, True)

        last_sent_ticks = None
        is_open = None  # unknown until first JSON read

        print("\nMode: JSON-driven. Controls (if --keyboard set): [o]=open  [c]=close  [space]=toggle  [q]=quit\n")
        try:
            while True:
                # 1) Optional keyboard override
                if args.keyboard:
                    ch = kr.getch()
                    if ch:
                        if ch == 'q':
                            break
                        elif ch == 'o':
                            is_open = True
                        elif ch == 'c':
                            is_open = False
                        elif ch == ' ':
                            is_open = not bool(is_open)

                # 2) Read JSON status
                state, source, meta = grip_from_json_file(args.status_json)
                if state in ("open", "close"):
                    is_open = (state == "open")

                # 3) Command if changed / needed
                desired_ticks = args.open_ticks if is_open else args.close_ticks if is_open is not None else None
                if desired_ticks is not None and desired_ticks != last_sent_ticks:
                    write_pos(ser, args.id, desired_ticks)
                    last_sent_ticks = desired_ticks
                    print(f"[CMD] From {'keyboard' if args.keyboard and kr.getch else 'JSON'}: "
                          f"{'OPEN ' if is_open else 'CLOSED'} -> {desired_ticks} ticks "
                          f"(source={source}, ts={meta.get('timestamp')})")

                # 4) Live feedback
                rp = read_pos(ser, args.id)
                state_str = "UNKNOWN"
                if is_open is True: state_str = "OPEN"
                elif is_open is False: state_str = "CLOSED"
                print(f"ID {args.id} | target={desired_ticks if desired_ticks is not None else 'None'} "
                      f"| present={rp if rp is not None else 'None'} | state={state_str} | src={source}")

                time.sleep(args.poll)

        except KeyboardInterrupt:
            pass

        print("Exiting.")

if __name__ == "__main__":
    main()
