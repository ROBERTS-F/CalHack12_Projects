# CalHack12_Projects — Spectacles → SO-101 Teleop

Teleoperate a physical [LeRobot SO-101](https://github.com/TheRobotStudio/SO-ARM100) robot arm using hand tracking from **Snap Spectacles**. The Lens streams hand pose / pinch data to a small relay server; Python processes on the host machine turn that into inverse-kinematics solutions and Feetech STS3215 serial commands for the real arm. A MuJoCo model of the arm (built from its own CAD) is available as a digital twin for visualization.

> Built at CalHacks 12. This README documents the repo as committed — see [Known limitations](#known-limitations--todos) for the gap between the demo and what's fully automated in code.

## How it works

```
Snap Spectacles Lens                 (hand/arm pose + pinch, on-device)
        │  WebSocket / HTTP  (via ngrok tunnel)
        ▼
Node relay  (src/vision/saving_json_input.js)
        │  writes latest sample, throttled ~20/s
        ▼
status.json  (src/vision/data/status.json)
        │
        ├─ polled @ 5 Hz ────────────► control_with_json.py ──► Feetech serial ──► gripper (servo ID 6)
        │                                                                          [closed loop, automatic]
        │
        └─ (no live reader yet) ────► so100_control.py --x --y --z ──► so100_ik.py (DH + damped
                                       [manual / CLI-triggered]          least-squares IK) ──► Feetech
                                                                          serial ──► arm joints 1–5

mujoco_models/so_arm100.xml ──► viewer_web.py / so100_xml_test.py   (MuJoCo digital twin,
                                                                      standalone — not fed by status.json)
```

- The **gripper** path is fully closed-loop: a pinch on the Spectacles reaches the physical gripper with no manual step.
- The **arm-position** path (moving the end effector to an x/y/z target) uses the same IK and serial stack, but is invoked from the command line rather than polled continuously from `status.json`. See [Known limitations](#known-limitations--todos).

## Repo structure

```
src/
  vision/            Node relay server — receives Spectacles data, persists status.json
    saving_json_input.js   production relay (throttled disk writes, status snapshot)
    web_test.js             earlier/minimal version of the relay
    data/status.json        latest tracking sample (gripState, gripStrength, timestamp)

  control/           Robot kinematics (pure Python, no hardware dependency)
    so100_ik.py       DH model + inverse-kinematics solver for the SO-100
    so100_control.py  IK solution → Feetech tick commands over serial

  hardware/
    motor_setup/
      find_motors_port.py   scan serial ports, ping servo IDs to find the bus
      set_motors_id.py      assign/reassign Feetech servo IDs
      read_ticks.py         dump present position of a servo
      sync_move_simple.py   send raw tick targets to a set of servo IDs
      control_with_json.py  polls status.json, drives the gripper (servo ID 6) automatically
      pinch_test.py          manual keyboard-jog gripper tester
      Old/                   earlier iterations of the above, kept for reference

  simulation/
    viewer_web.py       headless MuJoCo render, served as MJPEG over Flask
    so100_xml_test.py   one-shot PNG snapshot of the MuJoCo model

mujoco_models/
  so_arm100.xml        MJCF model of the arm, built from its own CAD
  meshes/               STL meshes for every printed and off-the-shelf part

requirements.txt      Python dependencies
Dockerfile / docker-compose.yml / .devcontainer/   containerized dev environment
```

## Prerequisites

**Hardware**
- A LeRobot SO-101 (the successor to the SO-ARM100 — see the naming note below) with 6× Feetech STS3215 smart servos, connected over USB serial.
- Snap Spectacles, with a Lens configured to POST/WebSocket hand-tracking + pinch data to this relay (the Lens project itself is not part of this repo).

**Software**
- Python 3.11 (matches the `Dockerfile`), with `pip install -r requirements.txt`
- `pyserial` for the hardware scripts (add it to your environment if not already present — it's imported directly in `so100_control.py` and the `hardware/motor_setup/` scripts)
- Node.js with `npm install` inside `src/vision/` (Express, `ws`, `cors`)
- [ngrok](https://ngrok.com/) (or any tunnel) to expose the relay to the Spectacles
- `MUJOCO_GL=egl` or `osmesa` for headless MuJoCo rendering (set automatically by `viewer_web.py`/`so100_xml_test.py`; `osmesa` is the default in the Dockerfile)

Alternatively, use the provided `Dockerfile` / `.devcontainer` for a preconfigured environment (MuJoCo's GL backends, build tools, and serial/udev access are already set up there).

## Running it

1. **Find and configure the servo bus** (one-time bring-up):
   ```
   python src/hardware/motor_setup/find_motors_port.py
   python src/hardware/motor_setup/set_motors_id.py --port <serial-port> ...
   ```

2. **Start the gripper bridge** — polls `status.json` and drives the gripper automatically:
   ```
   python src/hardware/motor_setup/control_with_json.py \
     --port /dev/tty.usbmodemXXXXX \
     --id 6 \
     --status_json src/vision/data/status.json \
     --open_ticks 2500 --close_ticks 2000
   ```

3. **Start the relay server:**
   ```
   cd src/vision
   npm install
   node saving_json_input.js
   ```

4. **Expose it to the Spectacles:**
   ```
   ngrok http 3000
   ```
   Point the Spectacles Lens at the resulting `https://*.ngrok.app` URL. Pinch open/close now drives the physical gripper in real time.

5. **Move the arm to a target position** (currently manual — see [Known limitations](#known-limitations--todos)):
   ```
   python src/control/so100_control.py --x 0.20 --y 0.00 --z 0.15 --port /dev/tty.usbmodemXXXXX
   ```
   Add `--dry` to solve and print the IK solution without sending any motor commands.

6. **Watch the digital twin (optional):**
   ```
   python src/simulation/viewer_web.py
   ```
   then open `http://localhost:5000` in a browser.

## Kinematics

`so100_ik.py` implements a 5-DOF DH model of the SO-100 arm and solves inverse kinematics numerically: forward kinematics via the standard DH product-of-transforms, an analytic Jacobian, and a damped least-squares pseudo-inverse to stay stable near singularities. Rather than solving straight to a distant goal, it interpolates a chain of intermediate poses (SLERP for orientation, linear for position, roughly one waypoint per centimeter of travel) and re-solves at each step — this is what keeps the arm from diverging on large reaches.

Solved joint angles come back in DH convention and are converted to the arm's mechanical convention via a measured **14.45°** offset before being mapped to Feetech ticks (`so100_control.py`): 12-bit absolute encoders (4096 ticks/rev), per-joint direction and zero-offset calibration, and shortest-arc interpolation in tick space so a move near the 0/4095 wraparound doesn't snap the joint the long way around.

## Known limitations / TODOs

- **Arm position isn't polled live yet.** `status.json` currently only carries `gripState`/`gripStrength` — no hand position — and nothing in the repo continuously feeds tracked position into `so100_control.py` the way `control_with_json.py` does for the gripper. Closing this loop would mean extending the relay/status schema with a position sample and adding a poller analogous to `control_with_json.py` that calls the IK solver and streams ticks per update.
- **Per-joint tick calibration is a placeholder.** `OFFSET_TICKS` in `so100_control.py` is mid-scale (2048) for every joint; replace with each joint's measured zero-position center for accurate absolute positioning.
- **SO-100 vs. SO-101 naming.** The physical arm this repo targets is the SO-101, but the DH table, mesh model, and code (`so100_ik.py`, `so100_control.py`, `mujoco_models/so_arm100.xml`) all carry over SO-100 naming and geometry from the original build. Double-check link lengths and the 14.45° mechanical offset against your SO-101 before trusting IK output — the two arms share most of their mechanical design but aren't guaranteed identical.
- **`mujoco_models/so_arm100.xml`** has a typo in one mesh path (`meshea/Camera-Mount-v6.stl` instead of `meshes/`) that will fail to load until fixed.
- **The digital twin is standalone.** `viewer_web.py` / `so100_xml_test.py` render one fixed demo pose; they aren't wired to `status.json`, IK output, or servo readback.
- **No auth on the relay.** `saving_json_input.js` binds `0.0.0.0` and, once tunneled with ngrok, accepts unauthenticated requests on `POST /ingest` and the WebSocket. Fine for a private demo; add auth before exposing it more broadly.
