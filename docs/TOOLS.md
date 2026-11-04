# Tools reference

Every script in `tools/` and `robot/`, what it is for, how to invoke it, and whether it is the
current path or a superseded experiment. Verified against the code at `tipcam-external` tip `b66b291`.

Interpreter: `D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe` (Python 3.12).
Below, `<py>` means that interpreter, run from the repo root.

Legend — **CURRENT** in the live path · **LIBRARY** imported by other tools · **SUPERSEDED** replaced,
kept for the technique · **EXPERIMENTAL/ABANDONED** does not work, do not build on it ·
**OFFLINE** safe with no robot attached.

---

## 0. Start here

| Need | Run |
|---|---|
| Check the rig before moving anything | `<py> tools/preflight.py --sam` |
| Park the arm safely / home it | `<py> tools/arm_park.py evidence/descend` · `<py> tools/arm_to_start.py evidence/descend` |
| Drive the whole demo from a browser | `<py> tools/dashboard.py` → http://localhost:8000 |
| Recalibrate the arm model | `<py> tools/arm_calibrate.py collect` → `fit-arm` |
| Ink a printed crossing (the current success case) | `<py> tools/arm_calibrate.py dot --roll 1000` |
| Verify a code change before putting it on the robot | `<py> tools/regress.py --out evidence/regress_out` |

⚠ `tools/arm_to_start.py`, `tools/arm_park.py`, `tools/teach.py` have **no `__main__` guard** and take
a bare positional output dir — importing them runs them and moves the arm.

---

## 1. Foundation libraries (no CLI of their own, imported everywhere)

### `tools/rig_io.py` — CURRENT · PC-side I/O hub
Capture + SSH. Everything else depends on it.
- `Camera(index=OVERHEAD, settle_s=1.0, paper_target=None)`, `.frame(average=3, settle=0.3)`, `.close()`.
- `servo(*commands) -> list[dict]`, `reset_bridge()`, `restart_arm_camera()`.
- `ROBOT = "ubuntu@<ROBOT_IP>"`; `BRIDGE = "source ~/hwr/deploy/env.zsh; python3 ~/colony/servo_bridge.py"`.
- `OVERHEAD = 1` (4K UGREEN looking down), `SIDE = 0` (1080p UGREEN beside the sheet),
  `EXPOSURE_TARGET = 140.0` (overhead median), `SIDE_PAPER = 185.0` (side 90th pct; "dots found at
  183, lost at 223"), cache `evidence/descend/exposure.json`.
- Backend order is **MSMF first**, DSHOW fallback ("DSHOW: 5 fps at 1080 on the 4K UGREEN, MSMF 30").
  Note: the old README said the reverse; the code is authoritative.
- Exposure lock: cache accepted if `miss(cached) <= 25.0`, else search `range(-3, -11, -1)` reading
  8 frames per candidate ("the 4K UGREEN lags ~4 frames behind").
- A daemon reader keeps the last 8 `(monotonic, image)` frames; `frame()` averages the recent ones
  and raises `RuntimeError("UGREEN stopped delivering frames")` after a 5 s bound.

### `tools/paper_grid.py` — CURRENT · library + single-image sanity check
5 mm ruling → numbered crossings → camera intrinsics/pose in the sheet frame. `SQUARE_MM = 5.0`.
- `<py> tools/paper_grid.py frame.png` → prints `N grid crossings`, `rms … px  f …  dist …`, the
  camera pose in mm, and writes `frame_grid.jpg`.
- Exports: `ruling_mask()`, `crossings(mask)`, `number(points, seed, patch=120.0)`,
  `grid_points(image, seed=None)`, `calibrate(obj, img, size)`.
- Imports: `cv2`, `numpy` only. Fully **OFFLINE**.

### `tools/tip_triangulate.py` — CURRENT · library + one-shot measurement
Needle tip in 3D from the fixed UGREEN and the eye-in-hand arm camera, both self-located on the
ruling. "Read-only: snaps both cameras, never moves the arm."
- `<py> tools/tip_triangulate.py` · `--near 1062 440` (UGREEN tip guess in px) ·
  `--out evidence/tip` (writes `<stamp>_ugreen.jpg`, `<stamp>_arm.png`, `<stamp>_tip.json`) ·
  `--recalibrate` (rebuild the UGREEN rig).
- `measure() -> {tip_sheet_mm, height_mm, miss_mm, ink_score, valid_pairings, shift_mm,
  tip_arm_cam_mm, arm_tip_px, ugreen_tip_px, arm_pose}`.
- `snap_arm(n=6, frozen_s=3.0)` — stall-proof: byte-identical repeats are treated as stale and the
  driver is restarted once.
- Other exports used across the repo: `arm_tip()`, `arm_tip_on_line()`, `_needle_blob()`,
  `ugreen_tip()`, `ray()`, `on_sheet()`, `meet()`, `ugreen_rig()`, `side_marks()`, `side_rig()`,
  `arm_candidates()`.
- Caches: `evidence/tip/ugreen_rig.json`, `evidence/tip/ugreen4k_lens.json` (4K overhead),
  `evidence/tip/ugreen_lens.json` (the side camera, f fixed 1148). `ROTATION_DEG = 180`,
  `MAX_MISS_MM = 3.0`.

### `tools/tip_descend.py` — LIBRARY (its descent path is superseded)
**This is the repo's servo-command layer.** Closed-loop descent steered only by measured tip
positions (no contact detector, no model heights).
- `<py> tools/tip_descend.py --dot 35.3 8.3 [--near 1154 440] [--no-touch] [--pose P1..P6] [--tip-cam X Y Z]`
- Imported functions: `read()`, `send()`, `move(target, ms=700, settle=0.6)`, `commanded(start=None)`,
  `go_start()`, `POSE_HELP`, `SERVOS`, `START`, `LIFT`.
- `START = {1:497, 2:423, 3:235, 4:269, 5:502, 10:662}`, `JOINTS = (1,2,3)`, `PROBE = 6`,
  `MAX_STEP = 8`, `ALIGN_MM = 0.8`, `FINAL_MM = 2.0`, `TOUCH_MM = -0.5`, `SURPRISE_MM = 4.0`,
  `LIFT = {2: 40}`. Broyden-updated Jacobian; aborts on two failed measurements or a surprise drop;
  always lifts. Writes `evidence/descend/<stamp>/{NNN}_ugreen.jpg`, `{NNN}_arm.png`, `log.json`.
- `commanded()` deliberately never re-bases on read-back (see §4 of ARCHITECTURE.md).

### `tools/arm_touch.py` — LIBRARY (standalone path superseded)
Touch a dot with the arm camera alone: hold the dot `AIM_BEYOND = 24.0` px past the tip, push along
the tip ray, feel contact as the tip pixel leaving its free-air spot.
- Used exports: `auto_light()` / `usable()` (lamp autotune — drives Hue 12/13 on every run),
  `unit_ray()`, `sheet_xy()`, `dots_near()`, `constellation_shift()`.
- Standalone: `<py> tools/arm_touch.py [--out evidence/arm_touch] [--steps 200] [--dot U V] [--pose P1..P6]`.
- `MAX_STEP = 10`, `PUSH_FAR_MM = 6.0`, `PUSH_NEAR_MM = 1.5`, `SLOW_BELOW_MM = 262.0`,
  `BACKLASH = {1: 2, 3: 3}`, `FINGER_ROW = 340`, `ALIGN_PX = [6, 10]`, `LIFT = {2: 40}`.

### `tools/hwr_kinematics.py` + `robot/hwr.urdf` — VENDORED, do not edit
FK/IK and arm geometry. Verbatim from `Porkchop911/dozorin-private @ 62eb5b4`. Full interface in
[ARCHITECTURE.md §3](ARCHITECTURE.md).

---

## 2. Calibration and placement

### `tools/arm_calibrate.py` — **CURRENT pipeline head** (1414 lines)
Calibrate the arm model against the external cameras, then hover and ink a printed grid crossing.

```
<py> tools/arm_calibrate.py collect                     # ~40 poses, robot on, arm at START (~3 min)
<py> tools/arm_calibrate.py fit-arm evidence/tip/arm_calibration_data_*.json
<py> tools/arm_calibrate.py fit-tie <runs>              # keep arm terms, refit only the session tie
<py> tools/arm_calibrate.py fit-tip --roll 1000         # per-roll tip-in-camera fit
<py> tools/arm_calibrate.py aim                         # live A/B: nominal vs calibrated at 60 mm
<py> tools/arm_calibrate.py hover [--roll R]            # hover over the printed crossing
<py> tools/arm_calibrate.py dot  [--roll R]            # hover, then press and mark it  (CURRENT goal)
<py> tools/arm_calibrate.py selftest                    # the fit on synthetic data
<py> tools/arm_calibrate.py repeat                      # gate 0: return-to-START scatter (~10 min)
<py> tools/arm_calibrate.py repeat-report evidence/tip/arm_repeat_<stamp>.json
```
`cmd` choices are exactly those eleven; `--roll` defaults to `START[5] = 502` ("1000: needle
near-vertical"); trailing positional `data` defaults to `evidence/tip/arm_calibration_data.json`.

Key constants: `START = {1:497, 2:420, 3:190, 4:269, 5:502}`, `MIN_HEIGHT_MM = 15.0`,
`MISS_MM = 5.0`, `ANCHOR_EVERY = 20`, `USE_SIGMA = 1.5`, `HOVER_STEPS_MM = (40, 25, 15, 10)`,
`HOVER_TRIES = 3`, `HOVER_GOAL = (-5.0, 45.0)`, `HOVER_GOAL_ROLL = (-3.2, 40.9)`,
`TOUCH_STEPS_MM = (8, 6, 4, 3, 2, 1, 0, -1, -2, -3)`, `TOUCH_GOOD_MM = 1.0`,
`DOT_STOP_AT_CONTACT = False`, `CONTACT_ROW = 0.4`, `CONTACT_MM = 0.3`, `AIM_HOVER_MM = 60.0`,
`SIDE_UP_MM = (110, 170)`, `REPEAT_AWAY = 20`, `REPEAT_DWELL = 1.6`.
Outputs/caches are tabulated in [ARCHITECTURE.md §5](ARCHITECTURE.md).

### `tools/coarse_aim.py` — CURRENT · fast first move
Model-only placement (URDF + fitted arm model), tip 15 mm above the target, then it reports the real
miss. "Model-only placing on this arm was ~5-10 mm off … `aim_touch.py` closes the last millimetres."
- `<py> tools/coarse_aim.py --out DIR [--calib evidence/tip/arm_model.json] [--pose '{"1":497,...}']
  [--dot U V | --sheet X Y] [--go] [--lam 202.5] [--hover 15.0] [--joints 1 3 4] [--ms 1500]`
- Guards: `MAX_MOVE = 150` pulses/joint, `TILT_WARN_DEG = 5.0`, abort if `residual > 1.0`,
  `APPROACH = -1`. Writes `DIR/<stamp>/log.json`, `before_arm.png`, `after_arm.png`.
- Exports reused by calibration: `arm_pulses()`, `camera_in_base()`, `sheet_from_base()`,
  `tip_in_base()`, `solve(pose, tip_cam, goal, joints, iters=30)`, and the `CALIB` global — all 25
  callers only see calibration when `coarse_aim.CALIB` is set.

### `tools/model_jab.py` — CURRENT placer used by the dashboard (1223 lines)
"One model move, image-measured corrections, one fluid jab" — now model move + iterative correction
+ a watched wrist arc for contact. Criticised as model-led in
`docs/history/accuracy_analysis_2026-10-30.md`.
- `<py> tools/model_jab.py --out DIR [--fwd 10] [--up 10] [--press 0.0] [--lam 206] [--plan-only]
  [--rightmost] [--dot U V] [--red U V] [--strafe] [--along MM] [--lamp-test] [--stop-file PATH] [--jab]`
- **There is no `--calib` flag here** — adding it is listed as future work in the 2026-10-31 handover.
- Selected constants: `SAM_THRESHOLD = 0.4`, `SAM_AREA = (40, 300)`, `RED_AREA = (25, 120)`,
  `SAM_BLACK_FRAC = 0.6`, `MATCH_MM = 3.0`, `PLACE_JOINTS = (1, 3, 4)`, `PLACE_TOL_MM = 1.0`,
  `SIDE_TOL_MM = 1.2`, `PAN_BACKLASH = 5`, `PAN_SIGMA_MM = 4.0`, `ROLL_MM = 0.42`,
  `ROLL_AIM_MM = 0.6`, `CONTACT_EXTRA = 6`, `CONTACT_LAG = 6`, `TRI_STOP_MM = 0.5`,
  `WRIST_STEP = 3`, `GAIN = 0.35`, `ARC_TILT = 15`, `PLACE_FLOOR_MM = 5.0`, `LIFT_ELBOW = 40`.
- Day caches inside `--out DIR`: `lamps.json`, `tip_cam.json` (last run's tip in the camera frame),
  `ugreen_dots.json`. Contact arc drags ~0.76 mm per 1-pulse step.

### `tools/aim_touch.py` — SUPERSEDED path, live as the touch library (643 lines)
Put the tip on a dot with the arm camera only, tip depth from a touch-off (`--lam 206.0`).
- `<py> tools/aim_touch.py --out DIR [--lam 202.5] [--dot U V] [--hover] [--pose P1..P6] [--steps 150]
  [--joints 1 3 4] [--no-strafe] [--no-ugreen]`
- `JOINTS = (1, 3, 4)` (servo 2 held), `J0` mm/pulse matrix, `DEADBAND = 2`, `BACKLASH = 5`,
  `ALIGN_MM = 0.5`, `SAFE_MM = 8.0`, `APPROACH_MM = 15.0`, `HOVER_MM = 3.0` (blind below),
  `FLOOR_MM = -0.5`, plus the UGREEN handoff thresholds (`UG_START_MM = 8.0`, `UG_OK_MM = 1.0`,
  `UG_MIN_PAIRS = 4`).
- Exports in use: `touch()`, `lock()`, `moved_needle()`, `needle_bottom()`, `on_paper()`,
  `ug_black()`, `ugreen_view()`, `neighbours()`.

### `tools/aim_sim.py` — OFFLINE · controller harness, still valid
Tests `aim_touch.touch()` against a simulated plant with deadband, backlash and Jacobian error.
- `<py> tools/aim_sim.py [--db 2] [--bl 5] [--jerr 0.1] [--runs 30]`; asserts the ideal plant lands
  (`np.median(ideal) < 2.5`). "Re-run with the Step 1 numbers before changing DEADBAND/BACKLASH on
  the arm." Writes nothing.

### `tools/side_aim.py` — CURRENT diagnostic (live window)
Shows whether the side UGREEN's pose still solves from notebook dots (`OK: N of M dots, rms px,
height mm, tilt deg`; good is `s['dots'] >= 6`). No flags, `q` quits. Writes scratch
`evidence/descend/side_aim_top.png`, `side_aim_crop.png`; reads `evidence/descend/ugreen_dots.json`.

### `tools/plate_rig.py` — CURRENT camera-rig tool (331 lines)
Both rig cameras' poses in **one sheet frame** from the ruling plus the 96-well plate — no ink dots
needed.
- `<py> tools/plate_rig.py` (grab both, solve, write caches) · `--check` (do cached poses still fit) ·
  `--images TOP SIDE` (**OFFLINE** solve from saved frames).
- `PITCH_MM = 9.0`, `SNAP_PX = 1.5`, `WELLS = {"top": (41, 20, 400, 3.0), "side": (21, 30, 50, 6.0)}`,
  `CHECK_MIN = 150`, `TOP_PLATE_FIX = ([-0.00737, -0.01127, 0.00117], [1.58, 1.47, 0.46])` ("one view;
  remeasure if the overhead moves far"). Rejects: overhead `< 150` crossings, plate tilt `> 2.0°` or
  height outside 5-25 mm; side `< 100` crossings or height outside 80-250 mm (caches left untouched).

### `tools/tip_motion.py` — CURRENT as a library; standalone use is a one-off measurement
Tip in 3D from base strafes only (UGREEN rays + the arm camera's own measured displacement).
- `<py> tools/tip_motion.py --near 1142 204` (required) · `--moves "0,0.05,0.2;0,-0.05,0.2"`
  (`vx,vy[,seconds]` per `/drive` pulse) · `--from evidence/tip_motion/<run>` (re-analyse, no motion)
  · `--home <run dir>` (strafe back to frame 0 of a saved run).
- Exports: `arm_view(image) -> (R, t, blobs)`, `lattice_offset(ref, other)` (whole-square shift,
  ≤ 1.5 mm), `drive(vx, vy, seconds=0.3)`, `return_home(..., tol_mm=2.0)`.
- Physics note kept in the code: treating strafes as pure translation put the tip 7 mm too high
  (63.4 mm vs ~56 mm on tape); per-frame rotation is kept, with 4 candidate 90° rotations.

---

## 3. Perception and detection

### `tools/sam_detect.py` — CURRENT · one image, text-prompted SAM 3.1
- `<py> tools/sam_detect.py IMAGE "black dot" [--threshold 0.5] [--out overlay.jpg]`
  (default overlay `<image>_sam.jpg`). Prints `load X s, infer Y s, N x 'prompt'`.
- `detect(predictor, image, prompt, threshold=0.5) -> [{score, box, centroid, area, mask}]`.
- `load()` carries two workarounds that must not be removed: `decoder.sdpa_kernel = lambda *a, **k:
  contextlib.nullcontext()` (SAM forces FLASH_ATTENTION, "No available kernel" on Windows torch) and a
  patched `predictor.model.init_state` (upstream passes `offload_state_to_cpu`, missing in sam3
  `f147e8`-era builds). An image is run as a one-frame video. First call downloads gated
  `facebook/sam3.1` weights (`hf auth login`).

### `tools/sam_live.py` — SUPERSEDED demo (`dashboard.py` is the live path)
- `<py> tools/sam_live.py ["black dot"]`; type a new prompt + Enter, `t 0.3` sets threshold, `s`
  saves, `q` quits. Writes `evidence/sam_live/{stamp}_frame.jpg` and `{stamp}_{prompt}.jpg`. ~1-3 fps,
  no inter-frame tracking.

### `tools/markers.py` — OPTIONAL, partially superseded
ArUco calibration aids. `README` and both handovers say the 5 mm ruling is the calibration target and
markers are optional; `preflight.py` reports them as "optional: the dots are the fallback".
- `<py> tools/markers.py draw` · `sheet evidence/markers_a4.png` · `check frame.jpg` · `selftest`
  (**OFFLINE**).
- `DICT = DICT_4X4_50`, `SIDE_MM = 30.0` (6×6 5 mm squares), `SHEET_IDS = (8, 19, 24, 32)`,
  `HOLDER_IDS = (9, 45, 46, 6)`, `CHASSIS_ID = 5`.
- ⚠ `docs/history/plan_speedup_2026-10-29.md` describes an obsolete marker spec ("20 mm; ids 0-3
  notebook, 10-13 holder, 20 chassis") that no longer matches the code.

### `tools/d1_retie_forensics.py` — SUPERSEDED one-off diagnostic
Answers "why did Touch 1's `along` flip ±5-13 mm between 1-2 pulse moves?" (AGREED-plan item D1, now
closed). `<py> tools/d1_retie_forensics.py [--round 2] [--frames 12-63]`; the run it inspects is
hardcoded (`RUN = evidence/descend/20261030T103024`); writes scratch images to
`evidence/regress_out/d1_*.png`. Needs the GPU for SAM, never the robot.

---

## 4. Motion, teaching, safety

### `tools/preflight.py` — CURRENT preflight (172 lines)
"Fixes what it can (watchdog, stalled arm camera), reports the rest, one line per check." Exit 1 if
any check fails. `<py> tools/preflight.py [--sam] [--start] [--no-robot]`.
- Checks: robot reachable; `sensor_supervisor_node` stopped (it "restarts the arm camera every 16 s");
  arm camera ≥ 6/8 snaps, else `restart_arm_camera()` + 30 s wait; overhead median 100-180 (target
  140); side 90th pct 160-210 (target 185); overhead grid ≥ 200 crossings; ArUco ids (optional);
  teal inked tip (`inRange(hsv, (45,35,50), (100,255,255))`). `--sam` adds SAM checks against
  `evidence/descend/ugreen_dots.json` and side height `80 <= |z| <= 250`.
- "The dashboard holds both UGREENs: stop it first." Temp frame `evidence/descend/_preflight.png`.
- ⚠ Its start pose uses roll 502; the 2026-10-31 handover forbids any roll command until the gripper
  cable is unwound. See [STATUS.md](STATUS.md).

### `tools/arm_to_start.py` / `tools/arm_park.py` — CURRENT, tiny, dangerous if imported
- `<py> tools/arm_to_start.py OUT_DIR` → `{1:497, 2:420, 3:190, 4:269, 5:502}`, writes
  `{OUT_DIR}/arm_at_start.jpg`.
- `<py> tools/arm_park.py OUT_DIR` → `{1:505, 2:900, 3:0, 4:205, 5:510}` (fold up, needle clear of the
  paper), writes `{OUT_DIR}/arm_parked.jpg`.
- Both: 12 interpolated steps at `ms=700` + 1.0 s settle, read-back clamped to 0-1000 ("the elbow read
  -54"), abort holding position if any joint lags > 25 pulses or the shoulder lags > 40.

### `tools/teleop.py` — CURRENT for grasp work (237 lines)
Keyboard piloting over the live arm-camera view, recording every step.
- `<py> tools/teleop.py --out DIR [--lam 202.5] [--pose 6 ints]`. Keys: `a/d` pan, `t/g` shoulder,
  `w/s` elbow, `r/f` wrist, `q/e` roll; `1 2 5 0` = 1/2/5/10-pulse steps (default 5 — "1-2 pulse steps
  sit inside the servo deadband"); space = MARK, `l` = LIFT (elbow +40), `x` quit; left-click the view
  to pick a target dot (`SEARCH_PX = 60`, match ≥ 0.5).
- Writes `OUT/{stamp}/log.json` (phases `go-start, light, start, move, mark, target, job-error`),
  `NNN_arm.png`, `NNN_mark.png`. Calls `arm_touch.auto_light()` (moves Hue 12/13) and `go_start()` —
  **the arm moves on launch**. Moves run on a worker thread sharing one SSH session.

### `tools/teleop_report.py` — CURRENT, OFFLINE analysis of teleop logs
Per-servo deadband, backlash, and image-Jacobian column (mm/pulse).
- `<py> tools/teleop_report.py RUN/log.json [RUN2/log.json ...] [--per-run] [--json OUT]` ·
  `--selftest` (asserts a synthetic model: `deadband == 3`, `max_unmoved == 2`, `reversals == 2`,
  `backlash == 7`, `jac == [1.2, 0, 0]`). `NOISE_MM = 0.3`. Read-back `moved` is treated as unreliable
  ("servo 2 reads ~5 low") and only reported as a side column.

### `tools/teach.py` — NEW one-off, **highest physical risk in the repo**
Servos go limp, the arm is guided by hand, every readback is logged.
- `<py> tools/teach.py OUT_DIR [--joints 1,2,3,4,5,10] [--seconds 120]` → `teach.jsonl` (every 0.2 s)
  + `keyframes.json` (poses held ≥ 1 s, every joint within 3 pulses). Ends after `--seconds`, or when
  `OUT_DIR/stop` exists.
- `{"torque": {j: False}}` after a 3 s countdown — "hold the arm before it starts (the shoulder drops
  when limp)". Torque is restored by a **position command** at `ms=1200` twice, because the board's
  "torque on" (0x0B) left the arm limp (2026-10-30).

### `tools/roll_flip.py` — NEW one-off probe/diagnostic (2026-10-31)
ONE defined move that rotates servo 5 by 90° (quarter turn), and reversed returns.
- `<py> tools/roll_flip.py probe|flip|unflip|park-high|probe-raw [--ms N] [--dry-run]`;
  `probe-raw` takes `--subs 0x2A,0x2C,0x1E`, `--payload 0x1D,5,1,0,100,0`, `--reply`, `--watch 8.0`.
- `ROLL = 5`, `HIGH, NORMAL = 875, 500`, `STEP = 100` pulses (24°), `FLIP_MS = 1200`, `TOL = 8` (~2°),
  `LAG_MAX = 40`, `MODE_READ_SUB = 0x2C`, `MODE_REPLY_WAIT_S = 2.0`.
- Key finding recorded in its comments: "375 pulses × 0.24 deg/pulse = 90 deg … A TRUE 180 deg
  rotation … is therefore IMPOSSIBLE in position mode alone". Motor mode (LX-16A cmd 29) is a **dead
  end**: "the RRC firmware does not forward undocumented subcommands" — only re-indexing the servo
  horn or replacing the RRC board remain. Motor-mode replies are read with
  `ssh ubuntu@<ROBOT_IP> "grep -a servo_raw ~/hwr/state/mode_session.log | tail"`.
- Safety text is explicit: the gripper cable winds 90° (inspect slack by hand in teach mode first), the
  needle sweeps a half-turn (keep the tip clear of the paper), lag abort holds position and raises,
  "nothing is auto-reversed, by design". Writes no evidence files.

---

## 5. Verification and reporting

### `tools/regress.py` — CURRENT gate before code goes on the robot
- `<py> tools/regress.py --out DIR [--save]`. Baseline `evidence/descend/regress_baseline.json`
  (rows: 170928→6, 172244→4, 152609→0, 171326→5, 103024→20). `OWN_ABORTS = ("RuntimeError",
  "OutOfFrames", "KeyboardInterrupt")`; any other exception, or divergence earlier than the baseline
  row, is a FAIL (exit 1). 900 s timeout per replay.

### `tools/replay_run.py` — CURRENT, OFFLINE replay of a recorded run
"Saved camera frames in, commands recorded, no robot, no lamps … SAM 3.1 runs for real."
- `<py> tools/replay_run.py evidence/descend/20261029T125405 --out DIR [-- model_jab args]` ·
  `--compare LOG.json` (compare only).
- Monkey-patches `mj.read/send/drive/Camera/tt.snap_arm/at.auto_light` and `mj.time`. Run dirs before
  2026-10-29 13:00 saved only some UGREEN frames → replay aborts at the first hover (`OutOfFrames`).
  Prints `DIVERGED at row N` and the largest field difference over the `COMPARE` field list.

### `tools/dashboard.py` — CURRENT demo front end (416 lines)
- `<py> tools/dashboard.py [--port 8000] [--lan]` → http://localhost:8000 (`--lan` binds 0.0.0.0).
  Stdlib `ThreadingHTTPServer`, MJPEG `/stream/{overhead,side,arm}`, buttons POST
  `/action/{start,run,stop,park,preflight}`, `/target` (JSON `{x, y}` as 0-1 fractions), `/state`.
- Spawns `tools/model_jab.py --out evidence/descend --lam 190 --jab --stop-file
  evidence/descend/stop.flag` (plus `--red`/`--dot` from the clicked 4K pixel), `arm_to_start.py`,
  `arm_park.py`, `preflight.py --sam` (stdout → `evidence/descend/preflight.txt`).
- During a run it closes both UGREENs and shows the run's saved frames instead. Detection prompts:
  `"small red dot"`, `"dot"`, `"red cube"`, `"thin metal needle"`, plus the teal tip by HSV.

---

## 6. Superseded or abandoned (kept for the technique, not to run)

| Tool | What it was | Evidence it is dead |
|---|---|---|
| `tools/jab.py` | practise a learnt fwd+down shoulder/elbow/wrist jab in air, `--live JAB_JSON` to replay it | only cited in `docs/history/critique_2026-10-28.md`; `docs/history/handover.md` states the jab was removed from `model_jab.py` on 2026-10-28 |
| `tools/dot_servo.py` | lock one chosen dot in the arm camera and close in (`AIM_BEYOND = 6.0` px past the tip so the dot stays visible) | absent from the 2026-10-31 handover; still quoted as "already tracks one chosen dot instead of re-acquiring targets" |
| `tools/needle_grasp.py` | pick the needle up from the tripod tube by SAM masks → planes → 3D axis, then 30 mm blind (`APPROACH_MM = 30.0`); positional args `look|grasp|calibrate|offline` | handover: "EXPERIMENTAL, not working"; the tripod moved, needle is placed by hand |
| `tools/upright_dot_touch.py` | touch with the needle's underside (level gripper, tip hidden from the arm camera); SAM prompts `'thin metal needle'`, `'pen'`, `'needle'`, `'metal rod'` | handover: abandoned, back to the angled needle |
| `tools/upright_side_touch.py` | side-camera-only touch, contact = Theil median-of-pairwise-slope stall (`docstring` literally says "EXPERIMENTAL") | as above |
| `tools/upright_touch_session.py` | persistent upright session loading SAM/cameras/side pose once, adaptive gain | as above |
| `tools/arm_view.py` | live arm-camera window via `{"snap"}`; `s` saves to `evidence/view/{stamp}.jpg`, `q` quits; calls `reset_bridge()` on a stale reply | CURRENT-but-incidental; superseded for demos by `dashboard.py` |
| `tools/hue.py` | Philips Hue over the bridge's local API — `pair`, `list`, `set LIGHT on|off [--bri 1..254] [--ct 153..500 mired]` | CURRENT (indirectly): `arm_touch.auto_light()` sets lamps 12/13 every run. Needs the LAN, never the robot. See [RIG.md](RIG.md) |

---

## 7. Robot-side files

| File | Role |
|---|---|
| `robot/servo_bridge.py` | NEW (built during the event). ROS 2 node `colony_servo_bridge`, stdio JSON-lines ↔ `servo_cmd` / `servo_torque` / `servo_raw` / `read` / `snap`. Full protocol in [ARCHITECTURE.md §2](ARCHITECTURE.md). Deploy: `scp robot/servo_bridge.py ubuntu@<ROBOT_IP>:colony/servo_bridge.py` |
| `robot/hwr_driver_node.py` | **VENDORED, lightly patched.** rclpy node `hwr_driver`, the one process that owns the RRC board. Deployed as `~/hwr/ros2_ws/src/hwr_driver/hwr_driver/driver_node.py`, restarted with `sudo systemctl restart hwr`. Patches: `servo_torque` subscription (teach mode), `servo_raw` probe (replies logged only), joint-state robustness (seed 0.0, retain last good per joint, refuse shape mismatch) so "a partial servo response must not disconnect the articulated camera subtree" |
| `robot/hwr.urdf` | **VENDORED** vendor CAD export, de-tilted ~0.5°. Joints 1-5 revolute, all ±2.09 rad, `effort=10 velocity=6`. Eye-in-hand camera hangs off `link4` |

## 8. Hardware-required vs offline-safe

**Needs the live robot:** `rig_io.py`, `arm_calibrate.py` (`collect/aim/hover/dot/repeat`),
`coarse_aim.py --go`, `model_jab.py`, `tip_descend.py`, `dot_servo.py`, `arm_touch.py`,
`aim_touch.py`, `tip_motion.py`, `teleop.py`, `teach.py` (torque off), `roll_flip.py`,
`arm_to_start.py`, `arm_park.py`, `dashboard.py`, `arm_view.py`, `side_aim.py`, `preflight.py`
(without `--no-robot`).

**Safe with no robot attached:** `sam_detect.py`, `sam_live.py` (PC camera + GPU), `markers.py`,
`paper_grid.py`, `teleop_report.py`, `regress.py`, `replay_run.py`, `aim_sim.py`,
`d1_retie_forensics.py`, `plate_rig.py --check` / `--images`, `preflight.py --no-robot --sam`,
`arm_calibrate.py selftest`, `hue.py` (LAN only, never the robot).
