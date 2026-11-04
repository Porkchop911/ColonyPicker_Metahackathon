# Rig

The physical setup, its addresses, its quirks, and the rules that keep measurements valid.
Everything here was true on 2026-10-31 on the `tipcam-external` branch; the open hardware problems are
in [STATUS.md](STATUS.md).

---

## 1. Inventory

| Part | What it is | Role |
|---|---|---|
| Hiwonder ROSOrin Pro | mecanum base + 5-DoF arm, ROS 2 (Jazzy-ready), pre-existing control stack | the robot |
| Deptrum Aurora 930 | 640×400 **fixed-focus** RGB-D, eye-in-hand on `link4` | tip pixel, target dot, its own pose from the ruling |
| UGREEN webcam #1, 4K | index `1` (`rig_io.OVERHEAD`) | looking straight down at the sheet; grid + ink dots + landing |
| UGREEN webcam #2, 1080p | index `0` (`rig_io.SIDE`) | low beside the sheet; measures tip height above the paper |
| 5 mm ruled notebook sheet | the calibration target | numbered crossings give a metric frame for both cameras — no printed markers needed |
| 96-well plate | pitch `PITCH_MM = 9.0` (`tools/plate_rig.py`) | the real workpiece; also a second metric reference for camera poses |
| Philips Hue Play bars, lamps **12 and 13** | on bridge `<HUE_BRIDGE_IP>` | top/side light, set before opening the cameras |
| Steel needle in the gripper | the picking tool | position varies per grasp → measured every session, never assumed |
| Red cube | physical reference for cross-camera matching (`"red cube"`, `RED_AREA = (25, 120)`) | anchors `match_frames()` tie votes |
| PC: Windows, RTX 3090 | Python 3.12 venv | SAM 3.1 (~0.3-0.9 s per 1080p image), all pipeline logic |

Servos (bus ids): **1 pan, 2 shoulder, 3 elbow, 4 wrist, 5 roll, 10 gripper**. Pulses 0-1000 = 240°,
0.24°/pulse, 500 = centre. Arm joints are ±2.09 rad in the URDF; the gripper is *not* 0.24°/pulse
(see [ARCHITECTURE.md §3](ARCHITECTURE.md)).

## 2. Network and hosts

| Thing | Address / path |
|---|---|
| Robot SSH | `ubuntu@<ROBOT_IP>` (`tools/rig_io.py:22`) |
| Bridge launch line | `source ~/hwr/deploy/env.zsh; python3 ~/colony/servo_bridge.py` (`tools/rig_io.py:24`) |
| Bridge file on robot | `~/colony/servo_bridge.py` (deploy: `scp robot/servo_bridge.py ubuntu@<ROBOT_IP>:colony/servo_bridge.py`) |
| Driver file on robot | `~/hwr/ros2_ws/src/hwr_driver/hwr_driver/driver_node.py` (symlink install); restart with `sudo systemctl restart hwr` |
| Wheel gateway (pre-existing) | `POST localhost:8090/drive` **on the robot**, capped at 0.5 s per pulse |
| Hue bridge | `<HUE_BRIDGE_IP>` (Hue bridge, found by MAC prefix `vendor` in ARP), **plain HTTP v1 API**, `timeout=5` |
| Arm camera USB device | `3251:1930`, reset with `sudo -n usbreset 3251:1930` |

Trusted-LAN design throughout: no TLS, no auth on the Hue API, SSH key auth assumed for the robot,
and a standing **NOPASSWD sudo** grant for `usbreset`.

## 3. Environment setup (PC)

```bash
uv venv .venv --python 3.12
uv pip install --python .venv/Scripts/python.exe torch torchvision \
    --index-url https://download.pytorch.org/whl/cu128      # CUDA 12.8
uv pip install --python .venv/Scripts/python.exe -r requirements.txt
hf auth login                                               # facebook/sam3.1 weights are gated
scp robot/servo_bridge.py ubuntu@<ROBOT_IP>:colony/servo_bridge.py
python tools/hue.py pair                                    # press the bridge link button first
```

`requirements.txt` pins almost nothing (`numpy<2` is the only bound) and installs SAM as a mutable git
dependency (`sam3 @ git+https://github.com/facebookresearch/sam3.git`). torch/torchvision are
deliberately absent from the file — install them from the CUDA index first.

**Python version trap:** the repo contains both `cpython-312` and `cpython-314` bytecode in
`tools/__pycache__/`. Use the 3.12 hackathon venv; SAM/torch/`triton-windows` may not build on 3.14.

## 4. Cameras: the rules that make measurements valid

1. **Lock lamps before opening the camera.** Exposure locks at open, so a lamp change after opening
   silently invalidates the run. `arm_touch.auto_light()` sets lamps 12/13 at the start of every tool.
2. **Backend order is MSMF first**, DSHOW fallback (`tools/rig_io.py` — the old README had this
   reversed; the code is authoritative). Reason: "DSHOW: 5 fps at 1080 on the 4K UGREEN, MSMF 30".
3. **Exposure targets are measured, not guessed.** Overhead targets a scene median of `140.0`
   (accept 100-180); the side camera targets a 90th percentile of `185.0` (accept 160-210) —
   "dots found at 183, lost at 223". Cache `evidence/descend/exposure.json`; a cached step is reused
   only if its brightness miss is ≤ 25.0, otherwise steps `-3..-10` are searched, 8 frames each
   ("the 4K UGREEN lags ~4 frames behind").
4. **Never trust a single snap.** `tip_triangulate.snap_arm(n=6, frozen_s=3.0)` treats byte-identical
   repeats as stale and restarts the driver once. The robot-side bridge refuses frames older than
   `MAX_FRAME_AGE = 1.0` s. If a session spans a camera-driver restart it "keeps reporting the old
   frame as stale for good (live 2026-10-27: 598 s)" — call `rig_io.reset_bridge()`.
5. **Arm-camera dropouts are usually the USB hub.** Physical fix: put the Aurora on a direct port.
   Software recovery: `rig_io.restart_arm_camera()` (kills `aurora930_node`, `usbreset 3251:1930`; the
   platform launch respawns it after ~8 s).
6. **`sensor_supervisor_node` fights the arm camera** — it restarts it every 16 s. `preflight.py`
   stops it (SIGINT then SIGKILL) and the driver patch keeps `joint_states` populated so a partial
   servo response cannot disconnect the camera subtree in `robot_state_publisher`.

## 5. Lighting

- `tools/hue.py pair` → writes `config/hue.local.json` (`{"username": ..., "clientkey": ...}`) via
  `POST /api {"devicetype": "hwr_colony#pc", "generateclientkey": true}`. **This file is gitignored
  and holds live credentials** — treat it as a secret; rotate by re-pairing if the repo is shared.
- `tools/hue.py list`, `tools/hue.py set 12 on --bri 254 --ct 250` (bri 1-254, ct 153-500 mired).
  `set_light()` reads state back after `sleep(0.1*transition_ds + 0.3)` "so callers can confirm before
  taking a picture".
- Autotune lives in `arm_touch.auto_light()` / `usable()`; the arm-camera lamp sweep is
  `LIGHTS = [(0,254),(0,180),(120,120),(254,0),(254,254),(0,0)]`. Per-day lamp state is cached
  (`evidence/tip/lamps.json`, `model_jab` writes `lamps.json` in its `--out` dir).

## 6. Sheet, plate, and cross-camera tie

- The ruling is the metric frame: `SQUARE_MM = 5.0`, crossings are numbered and matched by neighbour
  walk + regrown homography (`paper_grid.number()`); Hough was abandoned ("~15k phantom lines per
  family").
- Whole-square ambiguity is resolved by `tip_motion.lattice_offset()` (≤ 1.5 mm) or by ink dots
  anchored on the red cube (`model_jab.match_frames()`, 90°-multiple rotation).
- Ink is detected as teal: `inRange(hsv, (45,35,50), (100,255,255))` (`arm_calibrate.TEAL =
  ((40,25,60),(110,255,255))` for the inked needle), and marks are scored by **difference image**
  (`new_mark` / `ink_dot`), never by absolute detection.
- ArUco (`tools/markers.py`, `DICT_4X4_50`, `SIDE_MM = 30.0`, ids `SHEET_IDS = (8,19,24,32)`,
  `HOLDER_IDS = (9,45,46,6)`, `CHASSIS_ID = 5`) is optional fallback hardware. The marker spec in
  `docs/history/plan_speedup_2026-10-29.md` is stale and contradicts the code.

## 7. The needle and roll

- The needle is placed **by hand** in the gripper; its tip in the camera frame changes with every
  grasp and is re-measured (`tip_triangulate.measure() -> tip_arm_cam_mm`; `model_jab` caches the last
  one in `<out>/tip_cam.json`).
- Roll (servo 5) orientation matters: `--roll 1000` gives a near-vertical needle and is the tuned
  configuration for `fit-tip`/`hover`/`dot`; the default in code is `START[5] = 502`.
  Per-roll state is stored separately (`evidence/tip/dot_offsets_<roll>.json`, hover memory keys get
  `@<roll>`).
- **A true 180° roll is impossible in position mode**: 0-1000 pulses cover 240°, so a 90° flip is 375
  pulses and 180° exceeds the usable range; LX-16A motor mode was tried and is a dead end (the RRC
  firmware does not forward undocumented subcommands). Options left: re-index the servo horn, or
  replace the RRC board. See `docs/history/hiwonder_support_request.md`.
- Servo 5 rotation winds the gripper cable. Inspect slack by hand in teach mode before any 90° flip,
  and run flips with the tip clear of the paper.

## 8. Physical limits and habits

| Limit | Value | Source |
|---|---|---|
| Wheel speed clamp | `max_wheel_rps = 4.0`, watchdog zeros motors 0.4 s after the last nonzero command | `robot/hwr_driver_node.py` |
| Drive pulse cap | 0.5 s per `/drive` command | gateway (used by `tip_motion.drive`) |
| Joint pulse sanity | bridge rejects anything outside 0-1000 or an id outside `{1,2,3,4,5,10}` | `robot/servo_bridge.py` |
| Elbow read-back anomaly | `arm_to_start.py` clamps read-back to 0-1000 ("the elbow read -54") | `tools/arm_to_start.py` |
| Shoulder lag abort | hold position if any joint lags > 25 pulses, or the shoulder > 40 | `tools/arm_to_start.py`, `tools/arm_park.py` |
| Sweep floor | a measured tip below `MIN_HEIGHT_MM = 15.0` ends the sweep and the arm goes back up | `tools/arm_calibrate.py:34` |
| Blind zone | below `HOVER_MM = 3.0` mm the arm camera cannot see the tip | `tools/aim_touch.py` |
| Reach | the plate's usable area is set by `SIDE_UP_MM = (110, 170)` and `AIM_HOVER_MM = 60.0`; 60 mm hover put the tip over the robot body | `tools/arm_calibrate.py` |

House rules the project follows: always return to START before a new goal, always lift the elbow
(`LIFT = {2: 40}`) after a touch, never re-base a command on read-back, and never chain moves without
a settle. The 2026-10-31 handover records a standing operator rule of **"no e-stop / never suggest
stopping"** — stop cooperatively via `evidence/descend/stop.flag` or an action that is between moves.
