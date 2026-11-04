# Working in this repo (read this first)

**Start at `docs/colony_demo_writeup_2026-11-04.md`** (working demo, how to run it, limits); rig details in `docs/new_layout_2026-11-04.md`. Their numbers replace older ones here. Valid wells now: A10 A12 C10 C11 C12.

Robot needle placing with two fixed cameras. The current work is the **colony-picking demo**: colony from the agar
-> its own well -> A12 (disinfect) -> A10 (rinse), `tools/colony_cycle.py` (uses `colony_pick.py`, `well_tour.py`,
`bg_refresh.py`). Runbook: `docs/demo_runbook_2026-11-03.md`. Wells-only history: `docs/writeup_well_servo_2026-11-01.md`.
Older docs in `docs/` are history; trust the newest dated one.

## Safety rules (the arm is real; breaking these has damaged the needle)

1. **Never fold the arm.** No `arm_park.py`, no PARK, no pose with shoulder > ~600 or wrist < ~300: it hits the
   needle. If the arm reads folded or not where `evidence/tip/well_servo_state.json` says: **send nothing, ask the user.**
2. **Never command servo 5 (roll) or 10 (gripper).** Only joints 1-4 (pan, shoulder, elbow, wrist), values 0-1000.
3. **One driver.** Before moving the arm, check no other process drives it (look for running `well_servo`, `pose_gui`,
   `arm_calibrate`, `colony_*`, `well_tour`, `bg_refresh` Python processes). `Servos.send` also refuses while another
   live process tree holds `evidence/tip/arm.lock`. Never hand the user a run command while your own run is moving.
4. **Dry-run first.** Any new or edited motion code runs `--dry` (simulated arm) before the robot.
5. **Moves go through `well_reach.jgo`** (small interpolated pieces). Never send one big raw move.
6. Do not run `highwonder_rework/deploy/sync.ps1`. No e-stop exists; do not add one.
7. If the user's answer about safety is unclear, ask again. Do not proceed.

## Pythons

| For | Use |
|---|---|
| Anything that moves the robot or opens cameras through `arm_calibrate.cameras()` (needs `sam3`) | `D:\robot\highwonder_rework\.worktrees\hackathon\.venv\Scripts\python.exe` |
| Offline: `--dry`, tests, `plate_rig.py --images ...` | plain `python` works |

Run everything from the repo root (paths like `evidence/tip/...` are relative).

## Map

| Path | What |
|---|---|
| `tools/well_servo.py` | **current tool.** Camera-guided placing per well. Constants at the top (poses, heights, gates, band geometry) |
| `tools/test_well_servo.py` | offline replay test of the tip measurement on saved frames. Run after any detector change |
| `tools/plate_rig.py` | camera poses + plate pose from the paper grid; `well("E12")` = well centre in sheet mm (z down, negative = up) |
| `tools/well_reach.py` | shared pieces: `Servos` (robot I/O), `jgo` (safe moves), `Eyes` (cameras), `needle_ends`, `PARK` |
| `tools/rig_io.py` | camera capture and the SSH servo bridge (robot `ubuntu@<ROBOT_IP>`) |
| `tools/arm_calibrate.py` | `cameras()` (lamp + camera open + pose check), `in_well`; the rest is older work |
| `tools/pose_gui.py` | small window to set servos 1-4 by hand, read back, save poses (for teaching) |
| `evidence/tip/` | caches (`*_pose_cache.json`), state (`well_servo_state.json`), results (`well_servo.json`), logs (`well_servo_run*.log`), frames (`well2_<tag>_<n>_<label>.raw.{top,side}.jpg`; `_miss` = not measured) |
| other `tools/*.py` | older approaches (model_jab, tip_descend, dot_servo, aim_touch, well_visit...). Do not use for wells |

## Procedure: a well run

1. **Look, don't move.** Read the servos and take both frames:
   ```
   python -c "import sys; sys.path.insert(0,'tools'); import well_reach as wr; print(wr.Servos().read())"
   ```
   Arm must be near the state file and unfolded (rule 1).
2. **Rig changed?** (paper, plate, mat, cameras moved) -> `python tools/plate_rig.py` (full solve; add
   `--images TOP SIDE` for saved frames). Then draw well labels on the overhead frame and check A1 is the printed A1.
   If rows are mirrored, flip them (multiply the plate matrix by diag(1,-1,1,1) with translation 7*9 mm in y).
3. **Background pose** (`BG_POSE` in `well_servo.py`) must keep the needle **out of both camera views**. If the robot
   base moved, check a frame at that pose by eye; raise the shoulder (keep shoulder+elbow+wrist = 1157) if the needle shows.
4. `python tools/test_well_servo.py` -> `ok`, and `python tools/well_servo.py --dry --wells E12 D12` -> all in.
5. Live, one or few wells, in the background, watched:
   ```
   <venv python> -u tools/well_servo.py --wells E12 D12 > evidence/tip/well_servo_runN.log 2>&1
   ```
6. Read the log: `grep -v Warn evidence/tip/well_servo_runN.log | grep -v "^  .*: xy"`. Results in
   `evidence/tip/well_servo.json` (`went_in`, `gate_xy_mm`, `seen_in`, `photo`).
7. A well counts only with `went_in` true **and** `seen_in` equal to the well **and** a photo where the needle is in it.

## When a run fails

| Log says | Look at | Usual cause |
|---|---|---|
| `tip not measured` at `_start` / `_find` | newest `well2_*_start_miss.jpg` | needle not near the expectation; background ghost; start pose moved with the robot base |
| `probe of joint k: tip not measured` | `*_probe*_miss.jpg` | expectation wrong (bad start), or the band/side line lost |
| `not within G mm at H mm` | the step lines `W_H i: xy ... z ...` | xy drifting while descending, or a lost measurement |
| `refusing {...}: tip predicted X mm over the rim` | step lines | step would dip under the floor; normally halved automatically |
| `arm not where ... says` | read-back vs state file | arm moved by hand / reboot: rule 1, ask the user |

Snapshots (`*.jpg` without `.raw`) show both views cropped: green circle = expected tip, red cross = measured.
To debug a measurement, copy the `.raw.top/.raw.side` frames and the run's `_bg000.raw.*` frames and replay
`well_servo.marker_tip` on them (see `tools/test_well_servo.py` for the stub). **Add the failing frames to the test
before fixing.**

## Key numbers

| | |
|---|---|
| needle vertical | shoulder + elbow + wrist = 1157 pulses |
| lower teal band -> tip | 15.3 mm; upper band 28.5 mm above the lower |
| well pitch / plate top | 9 mm / ~16 mm above the paper |
| backlash | 3-10 mm at the tip; every move ends from +8 pulses above |
| shoulder sag | read-back ~5-16 pulses under the command: never re-send a read-back as a command |
| lamp | `evidence/tip/lamps.json` [110, 153]; 254 washes the plate out in the overhead |

## Working style the user expects

Be brief. Do the work, report numbers. No streaming of every reading while waiting. Do what the user asks
(e.g. a full joint sweep) without arguing. Never suggest stopping for the day. No buttons in end-of-turn boxes.
