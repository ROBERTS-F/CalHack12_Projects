#!/usr/bin/env python3
# so100_control.py — Use your IK to command STS3215 motors (Feetech protocol 1.0) over serial.

import time
import argparse
import numpy as np

# ---- Import your IK (prefer local file; fallback to the LeRobot path) ----
try:
    from so100_ik import Robot, RobotKinematics, RobotUtils
except ImportError:
    from lerobot.common.kinematics.kinematics import Robot, RobotKinematics, RobotUtils  # type: ignore

# ===================== STS3215 CALIBRATION =====================
# STS3215: 12-bit absolute (0..4095) per 360°.
TICKS_PER_REV = 4096
TICKS_PER_RAD = TICKS_PER_REV / (2 * np.pi)  # ≈ 651.898 ticks/rad

# Per-joint direction to match your mechanical positive direction (+1 or −1)
DIRECTION = np.array([+1, -1, -1, -1, -1], dtype=float)

# Per-joint zero offsets in ticks (where mech angle = 0 rad should be).
# Start with mid-scale; later replace with your measured centers per joint.
OFFSET_TICKS = np.array([2048, 2048, 2048, 2048, 2048], dtype=float)

# Absolute bounds
MIN_TICKS = np.zeros(5, dtype=int)
MAX_TICKS = np.ones(5, dtype=int) * (TICKS_PER_REV - 1)  # 4095

# Streaming profile (interpolation)
PROFILE_STEPS   = 60     # number of setpoints
PROFILE_SECONDS = 1.5    # total move time
# ========================================================================

# ----------------- Feetech Protocol 1.0 helpers -----------------
READ_DATA  = 0x02
WRITE_DATA = 0x03

ADDR_TORQUE_ENABLE = 0x28   # 1 byte
ADDR_GOAL_POSITION = 0x2A   # 2 bytes (LE)
ADDR_PRESENT_POS   = 0x38   # 2 bytes (LE)

def checksum(bs): return (~sum(bs)) & 0xFF

def pkt(sid, inst, params):
    # 0xFF 0xFF, ID, LENGTH, INSTRUCTION, PARAMS..., CHECKSUM
    body = [sid, 2 + len(params), inst] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])

def torque_on(ser, sid, on=True):
    ser.write(pkt(sid, WRITE_DATA, [ADDR_TORQUE_ENABLE, 1 if on else 0]))
    ser.flush()
    time.sleep(0.002)

def write_pos(ser, sid, pos_ticks):
    lo, hi = pos_ticks & 0xFF, (pos_ticks >> 8) & 0xFF
    ser.write(pkt(sid, WRITE_DATA, [ADDR_GOAL_POSITION, lo, hi]))
    ser.flush()

def read_present_pos(ser, sid, retries=2):
    """
    Try to read present position (2 bytes) with READ_DATA.
    Returns int ticks or None if no response.
    Status packet: 0xFF 0xFF ID LEN ERR PARAMS... CHKSUM
    """
    req = pkt(sid, READ_DATA, [ADDR_PRESENT_POS, 2])
    for _ in range(retries + 1):
        ser.reset_input_buffer()
        ser.write(req); ser.flush()
        time.sleep(0.005)
        hdr = ser.read(4)  # 0xFF 0xFF ID LEN
        if len(hdr) < 4 or hdr[0] != 0xFF or hdr[1] != 0xFF:
            continue
        _id, ln = hdr[2], hdr[3]
        rest = ser.read(ln)  # ERR + PARAMS + CHKSUM
        if len(rest) != ln:
            continue
        err = rest[0]
        if err != 0:
            # Non-zero error code; still try to parse params
            pass
        if ln < 3:
            continue
        pos_lo, pos_hi = rest[1], rest[2]
        ticks = (pos_hi << 8) | pos_lo
        return int(ticks)
    return None

# ----------------- Conversions & interpolation ------------------
def wrap4096(x_ticks: np.ndarray) -> np.ndarray:
    """Wrap arbitrary tick values to [0, 4095]."""
    return (np.round(x_ticks) % TICKS_PER_REV).astype(int)

def mech_rad_to_ticks(q_mech: np.ndarray) -> np.ndarray:
    """Convert mech radians -> absolute ticks (per-joint direction & offset), wrapped to [0,4095]."""
    raw = OFFSET_TICKS + DIRECTION * (q_mech * TICKS_PER_RAD)
    wrapped = wrap4096(raw)
    return np.clip(wrapped, MIN_TICKS, MAX_TICKS)

def ticks_to_mech_rad(ticks: np.ndarray) -> np.ndarray:
    """Convert absolute ticks -> mech radians."""
    delta_ticks = (ticks.astype(float) - OFFSET_TICKS)
    return (delta_ticks / TICKS_PER_RAD) * DIRECTION

def lerp_joint_space(q_start: np.ndarray, q_goal: np.ndarray, steps: int) -> np.ndarray:
    """Joint-space linear interpolation in radians."""
    alphas = np.linspace(0.0, 1.0, steps)
    return (1.0 - alphas)[:, None] * q_start[None, :] + alphas[:, None] * q_goal[None, :]

def _shortest_tick_delta(curr: int, goal: int) -> int:
    d = (goal - curr) % TICKS_PER_REV
    if d > TICKS_PER_REV / 2:
        d -= TICKS_PER_REV
    return int(d)

def step_toward_goal_ticks(curr_ticks: np.ndarray, goal_ticks: np.ndarray, frac: float) -> np.ndarray:
    """Interpolate along shortest arc in tick space (per joint)."""
    outs = []
    for c, g in zip(curr_ticks.astype(float), goal_ticks.astype(int)):
        d = _shortest_tick_delta(int(c), int(g))
        outs.append((c + frac * d) % TICKS_PER_REV)
    return np.array(outs, dtype=float)

# --------------------------------- Main ----------------------------------
def main():
    p = argparse.ArgumentParser(description="SO100 IK → STS3215 motor command (serial).")
    p.add_argument("--x", type=float, help="Target X [m] in WORLD frame")
    p.add_argument("--y", type=float, help="Target Y [m] in WORLD frame")
    p.add_argument("--z", type=float, help="Target Z [m] in WORLD frame")
    p.add_argument("--port", type=str, required=True, help="Serial port (e.g., COM5 or /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (115200 or 1000000 are common)")
    p.add_argument("--ids", type=int, nargs="+", default=[1,2,3,4,5], help="Servo IDs to command (order=joint order)")
    p.add_argument("--k", type=float, default=0.35, help="IK step size")
    p.add_argument("--iters", type=int, default=250, help="IK iterations per interp step")
    p.add_argument("--dry", action="store_true", help="Solve IK only; do not command motors")
    p.add_argument("--single_test", type=int, default=None, help="Bypass IK and send a fixed goal (ticks) to this ID")
    p.add_argument("--test_ticks", type=int, default=3000, help="Ticks goal for --single_test")
    args = p.parse_args()

    # 0) Single-motor bring-up path (no IK), to verify bus/baud/ID quickly.
    if args.single_test is not None:
        import serial
        sid = args.single_test
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        time.sleep(0.1)
        print(f"[TEST] Enabling torque on ID {sid} @ {args.port} {args.baud} ...")
        torque_on(ser, sid, True)
        print(f"[TEST] Writing GOAL {args.test_ticks} ticks to ID {sid}")
        write_pos(ser, sid, int(args.test_ticks))
        ser.close()
        return

    # 1) Build goal transform (position-only here). Require x,y,z unless --dry.
    if not args.dry:
        if args.x is None or args.y is None or args.z is None:
            raise SystemExit("Please provide --x --y --z (meters) or use --dry.")
    T_goal = np.eye(4)
    if args.x is not None and args.y is not None and args.z is not None:
        T_goal[:3, 3] = np.array([args.x, args.y, args.z], dtype=float)

    # 2) Solve IK using your kinematics
    robot = Robot("so100")
    ik    = RobotKinematics()
    q0_dh = np.zeros(5)
    if args.x is not None:
        q_dh  = ik.inverse_kinematics(robot, q0_dh, T_goal, use_orientation=False, k=args.k, n_iter=args.iters)
        q_mech_goal = robot.from_dh_to_mech(q_dh)
    else:
        q_mech_goal = np.zeros(5)  # dry path without target

    # Sanity check FK error (if target was provided)
    if args.x is not None:
        T_chk = ik.forward_kinematics(robot, q_dh)
        pos_err = np.linalg.norm(RobotUtils.calc_lin_err(T_chk, T_goal))
        print("\n[IK] Target (m):     ", np.round(T_goal[:3, 3], 4))
        print("[IK] DH (deg):       ", np.round(np.rad2deg(q_dh), 3))
        print("[IK] MECH (deg):     ", np.round(np.rad2deg(q_mech_goal), 3))
        print("[IK] Position error: ", np.round(pos_err, 6), "m")
    else:
        print("\n[IK] Dry run without target (no motion)")

    if args.dry:
        print("[DRY] No motor commands sent.")
        return

    # 3) Open serial and enable torque on the selected IDs
    import serial  # local import so --dry works without pyserial
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(0.2)
    try:
        # Gate joint count by number of IDs provided
        dof = min(len(args.ids), 5)
        ids = args.ids[:dof]
        q_mech_goal = q_mech_goal[:dof]

        print(f"[SERIAL] Port={args.port} Baud={args.baud} IDs={ids}")
        for sid in ids:
            torque_on(ser, sid, True)

        # 4) Read present ticks if possible; else assume zeros mech
        present_ticks = []
        for sid in ids:
            t = read_present_pos(ser, sid)
            present_ticks.append(2048 if t is None else t)
        present_ticks = np.array(present_ticks, dtype=float)

        # 5A) Preferred: shortest-path interpolation in tick space from present to goal
        goal_ticks = mech_rad_to_ticks(q_mech_goal)
        dt = PROFILE_SECONDS / max(1, PROFILE_STEPS - 1)
        print(f"[MOVE] Streaming {PROFILE_STEPS} steps over {PROFILE_SECONDS:.2f}s (tick-space shortest path)")
        for i in range(PROFILE_STEPS):
            alpha = i / (PROFILE_STEPS - 1) if PROFILE_STEPS > 1 else 1.0
            tgt_ticks = step_toward_goal_ticks(present_ticks, goal_ticks, alpha)
            for sid, t in zip(ids, tgt_ticks):
                write_pos(ser, sid, int(t))
            time.sleep(dt)

        print("[DONE] Target commanded.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
