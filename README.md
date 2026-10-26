# ColonyPicker — Meta hackathon

A Hiwonder ROSOrin Pro (mecanum base, 5-DoF arm, eye-in-hand RGB-D camera) turned into a microbial
colony picker: find single colonies on a sourdough-starter plate, light them three ways, guide a
steel needle onto each one with two cameras, and photograph the plate to confirm the pick.

## Pre-existing vs. built during the event

**Pre-existing (not in this repo):** the robot's own control stack — ROS 2 servo/motor driver,
camera driver, HTTP gateway (`/drive`), environment script — written before the event, used here as
the platform this code runs on. Also an earlier needle-touch prototype and a colony-picker step-test
plan (2026-10-24); their lessons are cited in comments, their code is not included.

**Built during the event (everything in this repo):** camera calibration on the 5 mm grid sheet,
two-camera and robot-motion triangulation of the needle tip, closed-loop strafing, Hue lighting
control, SAM 3.1 detection, the dot-servo descent, the PC/robot I/O (`tools/rig_io.py`,
`robot/servo_bridge.py`), plan and evidence.

**Open-source dependencies:** SAM 3.1 (Meta, `facebookresearch/sam3`, `facebook/sam3.1` weights),
PyTorch, OpenCV, NumPy, Philips Hue local API.

## Rig

| Part | Role |
|---|---|
| Arm camera (Deptrum Aurora 930, 640x400, fixed focus) | eye-in-hand: tip pixel, target dot, its own pose from the grid |
| UGREEN 1080p webcam on the PC | fixed second view: tip ray, sheet calibration |
| 5 mm grid notebook sheet | calibration target for both cameras (no printed markers needed) |
| Hue Play bars (lights 12, 13) | top/side light; set before opening the UGREEN (exposure locks at open) |
| Steel needle in the gripper | the picking tool; position varies per grasp, so it is measured, not assumed |
| RTX 3090 on the PC | SAM 3.1 (~0.3-0.9 s per 1080p image) |

## Tools

| File | What |
|---|---|
| `tools/rig_io.py` | UGREEN capture (locked exposure, draining reader; DSHOW with MSMF fallback) and the SSH client for the servo bridge |
| `robot/servo_bridge.py` | robot side: `set` / `read` / `snap` as JSON lines; refuses camera frames older than 1 s |
| `tools/paper_grid.py` | 5 mm ruling -> numbered crossings -> camera intrinsics/pose in the sheet frame |
| `tools/tip_triangulate.py` | tip in 3D from both cameras; arm/UGREEN tip detection; stale-frame-proof arm snapshots |
| `tools/tip_motion.py` | tip in 3D from robot strafes (UGREEN rays + arm-measured motion incl. yaw); always returns to the start |
| `tools/tip_descend.py` | closed-loop descent onto a dot from measured tip positions |
| `tools/dot_servo.py` | lock a dot in the arm camera, close in, UGREEN height readouts (in progress) |
| `tools/sam_detect.py`, `tools/sam_live.py` | SAM 3.1 text-prompted segmentation, one image / live UGREEN stream |
| `tools/hue.py` | Hue bridge pairing and lamp control (local API) |
| `tools/arm_view.py` | live arm-camera window |

## Results so far (2026-10-26, see `docs/plan.md` and `evidence/`)

- Tip height by two-camera triangulation: 75.2 mm, ruler ~7.5 cm; repeats within 0.25 mm.
- UGREEN calibration checked on an upright tape: 20-80 mm marks land within ~1 mm.
- Strafe triangulation with per-frame yaw: 57-59 mm vs tape ~56 mm (pure translation gave 63.4).
- Found and fixed: arm-camera USB drop-outs (hub), stale frames passed off as fresh, lamps blinding
  the arm camera.

## Setup

```
uv venv .venv --python 3.12
uv pip install --python .venv/Scripts/python.exe torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv/Scripts/python.exe -r requirements.txt
hf auth login                      # facebook/sam3.1 is gated
scp robot/servo_bridge.py ubuntu@<robot>:colony/servo_bridge.py
python tools/hue.py pair           # press the bridge link button first
```
