# Architecture

How the pieces fit: a Windows PC doing vision and control logic, a ROS 2 robot doing motion, and a
set of JSON caches in `evidence/` carrying calibration state between runs.

Read this before running anything that moves the arm. Operational details (current servo state,
what is broken today) are in [STATUS.md](STATUS.md); every script is listed in [TOOLS.md](TOOLS.md);
hardware and setup in [RIG.md](RIG.md).

---

## 1. The two machines

```
   PC (Windows, RTX 3090, Python 3.12 venv)                 Robot (Hiwonder ROSOrin Pro, ROS 2)
 ┌───────────────────────────────────────────┐          ┌────────────────────────────────────────┐
 │ tools/*.py  (all the pipeline logic)      │          │  robot/hwr_driver_node.py (deployed as │
 │   ├─ rig_io.py  ── SSH one long session ──┼─stdio──► │  ~/hwr/.../driver_node.py) — sole owner │
 │   │      JSON lines, no port, no server   │          │  of the RRC board: servos, wheels, IMU │
 │   │                                       │          │      ▲ topics: servo_cmd servo_torque  │
 │   ├─ OpenCV  ◄── 2 UGREEN webcams (USB)   │          │      │         servo_raw  wheel_speeds │
 │   └─ SAM 3.1  ◄── torch/CUDA              │          │      │         joint_states  imu  beep │
 │                                           │          │  camera/color/image_raw (Deptrum eye-in-│
 │ evidence/**.json  calibration + run logs  │          │  hand) ──► bridge ──► PC              │
 └───────────────────────────────────────────┘          └────────────────────────────────────────┘
```

- The PC reaches the robot **only** through `tools/rig_io.py`, which spawns one long-lived SSH
  session running `robot/servo_bridge.py` on the robot. There is no HTTP server or socket for
  servos. Wheel/strafe motion goes through the platform's pre-existing HTTP gateway instead
  (`curl POST localhost:8090/drive`, run on the robot over SSH — see `tools/tip_motion.py`).
- `robot/servo_bridge.py` is a thin shim: stdin JSON line in, one JSON reply out, translated into
  ROS 2 topics on the driver side.

## 2. Three planes of data

| Plane | Path | Notes |
|---|---|---|
| **Motion** | `rig_io.servo()` → SSH → `servo_bridge` → `servo_cmd` → driver → bus servos | commands are validated twice (bridge and driver) |
| **Vision** | arm camera: `{"snap": true}` → base64 JPEG; UGREENs: OpenCV `VideoCapture` on the PC | arm frames are refused if older than `MAX_FRAME_AGE = 1.0` s |
| **Model state** | JSON caches under `evidence/tip/`, `evidence/descend/` | these are the calibration; delete them and the arm model is nominal again |

### Servo bridge protocol (`robot/servo_bridge.py`, exact)

| Command | Reply | Notes |
|---|---|---|
| `{"set": {"1": 500, "10": 680}, "ms": 300}` | `{"ok": true}` | `ms` optional, default 500 → `ServoCommand(duration_ms, ids, positions)` on `servo_cmd` |
| `{"torque": {"3": false}}` | `{"ok": true}` | `false` = limp (teach by hand); restored only by a later position command |
| `{"read": true}` | `{"pulses": {"1": 500, ...}, "age": s}` | 5 Hz `joint_states`, retained last-good value per joint |
| `{"snap": true}` | `{"jpeg": base64, "age": s}` | raises `stale camera frame (n.n s old)` beyond 1.0 s |
| `{"raw": [43, 5], "reply": true}` | `{"ok": true}` | **EXPERIMENTAL**, verbatim bus bytes, bypasses pulse validation, replies logged only |
| anything else | `{"error": "ValueError: expected set, torque, read or snap"}` | the bridge keeps serving after errors |

Startup handshake: the bridge waits up to 10 s for `joint_states` and a matched subscription before
printing `{"ready": bool, "camera": bool}` — a command published before that match is lost.

Valid servo ids: `SERVO_IDS = {1, 2, 3, 4, 5, 10}` = pan, shoulder, elbow, wrist, roll, gripper.
Pulses must be `0 <= v <= 1000`. `PULSE_RAD = radians(0.24)` (1000 pulses = 240°, 500 = joint zero).

## 3. Frames and units

| Frame | Origin | Used by |
|---|---|---|
| **sheet frame** | the 5 mm ruled notebook paper, crossings numbered `(u, v)` | all measurement: `tools/paper_grid.py`, `tools/tip_triangulate.py`, `tools/plate_rig.py`. Unit: mm |
| **base_link frame** | robot base, arm geometry from `robot/hwr.urdf` | `tools/hwr_kinematics.py`, `tools/coarse_aim.py`, `tools/model_jab.py`. Unit: metres / radians |
| **camera optical frames** | x right, y down, z forward | tip detection, `pixel_to_plane`, `camera_pose` |

Pulse conventions (the part that bites):

- `tools/hwr_kinematics.py:pulses_to_joints()` — `pan = (pulses[1]-500)*0.24°`, but
  **`shoulder/elbow/wrist = (500 - pulses[2..4])*0.24°`** (joints 2-4 are flipped).
- `robot/hwr_driver_node.py` publishes `joint_states` as `position = (pulse - 500) * RAD_PER_PULSE`
  for all joints, and `robot/servo_bridge.py` inverts it as `500 + position/PULSE_RAD`.
- ⚠ **Open question, unresolved in code:** read literally, `joint_states`/TF angles for joints 2-4
  are the negative of the vendored kinematics' URDF angles. Also the gripper: kinematics uses
  `GRIPPER_RAD_PER_PULSE = (-0.70 + 0.147)/(598 - 486) = -0.0049375 rad/pulse` while
  `joint_states` reports the gripper at the generic 0.24°/pulse. Until a hardware check settles it,
  trust the **pulse numbers** (every tool works in pulses) and treat TF angles as unverified.
- Gripper calibration anchors (2026-10-16 live readbacks): jaws fully open stall at pulse **598**
  (`r_joint = -0.70`), a 3 cm cube at pulse **486** (`r_joint = -0.147`). Two-point linear fit;
  pulse 200 is extrapolation.

Arm geometry as coded in `tools/hwr_kinematics.py` (`ArmKinematics`, metres):
`pan_x=0.0466`, `shoulder_z=0.1907`, `upper=0.08513`, `forearm=0.085147`, `tool=0.05577+0.08`,
`camera_along=0.069`, `camera_up=0.042`, `joint_limit=2.09` rad. IK rejects `z < 0`, the keep-out
box guess `x < 0.13 and z < 0.12`, and any `|angle| > 2.09`; **elbow-up branch only**.
`camera_along`/`camera_up` are approximations, not URDF-derived (~2 mm discrepancy acknowledged).

## 4. The canonical pipeline (what actually runs today)

Order as used by the newest working path (`tools/arm_calibrate.py hover`/`dot`, 2026-10-31):

1. **Rig health** — `tools/preflight.py`, then `tools/plate_rig.py` (both rig cameras posed in one
   sheet frame from the ruling + the 96-well plate) and `tools/side_aim.py` (side-pose sanity).
2. **Tip offset for this grasp** — the needle position in the gripper varies, so the tip in the arm
   camera frame is measured, never assumed: `tools/tip_triangulate.py` (`measure() ->
   tip_arm_cam_mm`), or `tools/tip_motion.py` when cross-camera pairing fails.
3. **Arm model** — `tools/arm_calibrate.py collect` → `fit-arm` → `fit-tie` (re-tie after the
   cameras or robot move) → `fit-tip --roll <R>` → `aim` (A/B validation) → `repeat` /
   `repeat-report` (gate 0 scatter test).
4. **Coarse placement** — `tools/coarse_aim.py --calib evidence/tip/arm_model.json --sheet X Y --go`:
   model-only first move, tip `HOVER_MM = 15.0` above the target. Model-only placing is 5-10 mm off;
   the last millimetres come from the closed loop.
5. **Closed-loop approach + ink** — `tools/arm_calibrate.py hover` / `dot --roll 1000`: anchor at
   START (gated to 15 mm), find the printed crossing, descend `HOVER_STEPS_MM = (40, 25, 15, 10)`
   with per-goal memory, then `TOUCH_STEPS_MM = (8, 6, 4, 3, 2, 1, 0, -1, -2, -3)` pressing into the
   paper.
6. **Contact decision** — side-view stall rules: `CONTACT_ROW = 0.4` (a 1 mm step whose side-view tip
   row falls below 40 % of the rows/mm above 2 mm) and `CONTACT_MM = 0.3` (a commanded mm that moves
   the tip less than 0.3 mm). `DOT_STOP_AT_CONTACT = False` — it keeps pressing to negative mm.
7. **Score the result** — overhead difference image → `new_mark` / `ink_dot` → measured mark minus
   aimed crossing written to `evidence/tip/dot_offsets*.json`; the mean offset is subtracted from the
   next run's aim.
8. **Offline verification** — `tools/regress.py` (replays 5 recorded runs against the current code),
   `tools/replay_run.py` (single run), `tools/teleop_report.py` (deadband/backlash/mm-per-pulse from
   teleop logs), `tools/aim_sim.py` (controller changes before touching arm constants).

### Competing implementations (do not confuse them)

| Family | Status | Why it matters |
|---|---|---|
| `arm_calibrate.py hover/dot` | **current live path** | model move + measured correction + per-goal memory |
| `tip_descend.py` | superseded as a path, **but it is the shared motion/IO layer** | `read/send/move/commanded/go_start/START/LIFT` are imported by almost every tool |
| `dot_servo.py` | superseded, still cited as the right pattern | tracks one chosen dot instead of re-acquiring |
| `model_jab.py` (+ `jab.py`, `aim_touch.py`) | `model_jab` is the current placer used by the dashboard; `jab.py` superseded | model-led, criticised in `docs/history/accuracy_analysis_2026-10-30.md` |
| `upright_*.py`, `needle_grasp.py` | abandoned experiments | upright grasp / auto needle pickup; both marked "EXPERIMENTAL, not working" |

`tip_descend.commanded()` encodes a real trap: never re-base commands on read-back, because
"servo 2 reads ~5 below where it holds … re-sending read-back sank the arm ~5 pulses per command".

## 5. Calibration caches (the state that makes runs reproducible)

| File | Written by | Read by | Invalidate when |
|---|---|---|---|
| `evidence/tip/arm_model.json` | `arm_calibrate fit-arm` / `fit-tie` / `fit-tip` (`tip_by_roll`) | `coarse_aim --calib`, `arm_calibrate hover/dot` | cameras or robot base moved; new grasp |

`arm_model.json` is not applied wholesale. At runtime only the **arm terms** are used, and only through
`coarse_aim.py --calib evidence/tip/arm_model.json`: joint zero offsets (shoulder **+19**, elbow
**−40** pulses), one direction offset per joint (pan **4.8**, shoulder **−5.7**, elbow **−4.1**, wrist
**−1.3**), and the approach-side terms (final approach from **20 pulses above**). The fitted base
yaw/xyz and the fitted `tip_cam` are stored but **ignored at runtime** — the tip offset actually used
comes from the measured `tip_by_roll` / per-session tip. If the file is missing, placement falls back to
nominal URDF (1.91 mm class error at 60 mm instead of 0.95 mm).
| `evidence/tip/hover_memory.json` | `arm_calibrate hover/dot` | same goal next run | goal changed, arm re-homed |
| `evidence/tip/printed_crossings.json` | `arm_calibrate local_crossing` | `arm_calibrate` | sheet moved |
| `evidence/tip/ugreen_rig.json`, `ugreen4k_lens.json`, `ugreen_lens.json` | `tip_triangulate --recalibrate`, `plate_rig` | `tip_triangulate`, `model_jab`, `side_aim`, `preflight` | UGREEN moved or re-mounted |
| `evidence/tip/top_pose_cache.json`, `side_pose_cache.json` | `plate_rig` | `plate_rig --check`, `arm_calibrate` | camera moved |
| `evidence/tip/arm_calibration_data*.json` | `arm_calibrate collect` | `fit*` | stamped files are never overwritten (append-only history) |
| `evidence/tip/arm_offsets.json` | legacy `fit` | legacy path | — |
| `evidence/tip/dot_offsets.json`, `dot_offsets_<roll>.json` | `arm_calibrate dot` scoring | next run's aim correction | roll changed (per-roll file) |
| `evidence/tip/side_pose_cache.json`, `lamps.json` | `arm_calibrate`, `model_jab` | same-day runs | new day |
| `evidence/descend/exposure.json` | `rig_io.Camera._lock_exposure` | `rig_io` | lamp or scene change; safe to delete (it is a cache only) |
| `evidence/descend/ugreen_dots.json` | `tip_triangulate`, `model_jab` | `side_aim`, `preflight --sam`, `d1_retie_forensics` | **per-run semantics:** "this run's day's set: replays load it, not the latest" |
| `evidence/descend/regress_baseline.json` | `regress --save` | `regress` | deliberately, when a behaviour change is accepted |
| `config/hue.local.json` | `hue pair` | `hue` | gitignored — a credential, see [RIG.md](RIG.md) |

## 6. Measurement design (why it is built this way)

- **Everything is metric because the paper is metric.** The 5 mm ruling is the calibration target for
  both cameras; no ArUco markers are needed in the normal path (`tools/markers.py` is optional and
  its spec in `docs/history/plan_speedup_2026-10-29.md` is stale).
- **Absolute accuracy comes from the cameras, repeatability from the model.** The arm model is fit
  against externally triangulated tip positions; the model then does fast big moves and the cameras
  correct the last millimetres.
- **Contact is inferred, not sensed.** There is no force sensor. Contact is a *stall signature* in
  image measurements (side-view tip row stops falling, or a commanded mm produces < 0.3 mm of motion).
- **The mobile base is a measurement device, not a correction axis.** Strafing is used to *triangulate*
  (`tip_motion.py`); it is deliberately not used to correct placement — the same pulse produced 0,
  −3.3, 0, 0, 0, +18 mm (`docs/history/rca_touch_dot.md` §3) and the gateway refused some commands
  ("drive outside bounded no-reverse envelope", `model_jab.py:906`).
- **Exposure is part of the measurement.** Camera exposure is locked before every run, and lamps are
  set **before** opening the UGREEN because exposure locks at open. `rig_io` searches manual exposure
  steps `range(-3, -11, -1)` against a brightness target (median 140 overhead, 90th percentile 185
  for the side view).
- **Stale frames are treated as corruption.** `snap_arm()` repeats byte-identical frames to detect a
  stalled driver and restarts it once; the bridge refuses frames older than 1 s; `rig_io.reset_bridge()`
  exists because a session spanning a camera-driver restart "keeps reporting the old frame as stale
  for good (live 2026-10-27: 598 s)".

## 7. Safety model

| Layer | Enforcement |
|---|---|
| Bridge | id whitelist (`1,2,3,4,5,10`), pulse range 0-1000, rejects empty `set` |
| Driver | `Watchdog(timeout=0.4)` auto-zeros motors 0.4 s after the last nonzero `wheel_speeds`; `validate_wheel_rps(msg.rps, max_wheel_rps=4.0)` → `stop_all_motors()` on violation; `_on_servo_cmd` rejects bad ids/pulses; startup and shutdown stop all motors and servos |
| Tools | `arm_to_start.py` / `arm_park.py` abort if any joint lags > 25 pulses (shoulder > 40); `tip_descend` aborts on two failed measurements or a height `SURPRISE_MM = 4.0` below prediction, always lifts; `arm_calibrate` ends a sweep below `MIN_HEIGHT_MM = 15.0`; `roll_flip.py` never auto-reverses |
| Physical | torque-off teaching (`tools/teach.py`) is the highest-risk operation in the repo: the shoulder drops when limp, and torque is restored only by a position command |

`dashboard.py` Stop is cooperative: it writes `evidence/descend/stop.flag` and the run aborts **at its
next move**. Actions other than a run cannot be stopped midway.

## 8. Known architectural weaknesses

1. **No pulse↔URDF sign verification.** See §3. Needs one hardware check (known joint2 pulse vs TF vs `fk()`).
2. **`{"raw": [...]}` bypasses all pulse validation** and writes arbitrary bytes to the servo bus.
   Used by `roll_flip.py probe-raw`. Motor mode on the LX-16A was a dead end: "the RRC firmware does
   not forward undocumented subcommands" (2026-10-31).
3. **One SSH session, blocking reads, no timeout or keepalive.** `rig_io._line()` can block a caller
   thread indefinitely; `stderr=subprocess.DEVNULL` hides SSH auth/host-key failures behind a bare
   "servo bridge not ready". Teleop's live view and mover share one session under an `RLock`.
4. **A 1414-line tool (`arm_calibrate.py`) is the pipeline head** and a 1223-line tool
   (`model_jab.py`) the placer; both carry dozens of magic constants. The tuning history is in
   `docs/history/`, not in code comments.
5. **Vendored code is not editable here** (`hwr_kinematics.py` verbatim from
   `Porkchop911/dozorin-private @ 62eb5b4`, `hwr_driver_node.py` from `highwonder_rework`). Changes
   must be made in the source repo and re-vendored.
6. **`arm_to_start.py`, `arm_park.py`, `teach.py` have no `if __name__ == "__main__"` guard** —
   importing them executes them. Never import these three.
