# Handover: colony picker, 2026-10-27

## Where it stands

| Milestone | State |
|---|---|
| 1. Touch dots on paper (needle placed by hand) | **The needle touches the paper reliably, steered from the arm camera only.** Touching a *chosen* dot is not yet confirmed. |
| 2. Pick up the needle, verify the tip | Not started. Tip measurement tools exist (see below). |
| 3. Colony detection with light control | SAM 3.1 runs live on the UGREEN; Hue lamps are automated. Not tried on a plate. |
| 4. Pick and transfer | Not started. |
| 5. Classify | Not started. No Meta/Llama model call yet. |

## How a touch works now (arm camera only)

- The needle is rigid to the arm camera, so its tip sits on one pixel in free air.
- **Landing point:** where the tip pixel's ray meets the paper (grid pose per frame). This holds whatever the tip's depth, so aim = put the target dot on that point.
- **Tip depth along its ray:** ~202.5 mm, from a touch-off (descend until contact, h = 0 at the contact frame). Used only for slowing down and the press depth.
- **Contact signal:** the paper pushes the tip back along the needle line. The needle shadow meeting the tip shows it visually, ~1 mm before contact.

## Tools (all in `tools/`)

| Tool | Use |
|---|---|
| `teleop.py --out DIR` | **Start here.** Live arm view, click a target (template-tracked), servo panel with −/+ per servo, step sizes, MARK, LIFT; every step logged with the image, pulses, height and target offset. |
| `aim_touch.py --out DIR` | Automatic: descend to 15 mm, pick the dot nearest the landing point, align (pan/elbow/wrist), press to −2 mm, lift. Has a live overlay window. |
| `arm_touch.py` | Lamp auto-selection (`auto_light`), dot detection, pattern tracking helpers; an older touch-off loop. |
| `tip_triangulate.py` | Arm tip finder (`arm_tip`, `arm_tip_on_line`); stall-proof `snap_arm`. |
| `rig_io.py` | UGREEN capture, robot bridge (thread-safe), `restart_arm_camera`, `reset_bridge`. |
| `robot/servo_bridge.py` | Robot side; deployed at `~/colony/servo_bridge.py`. |

Run with the worktree venv: `D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe`. This repo has no venv of its own.

## Robot start-up checklist

1. Robot on and the stack up (~1 min).
2. Stop the sensor supervisor: `kill -INT <pid of sensor_supervisor_node>`. It restarts the camera every 16 s; the user ordered it off. It comes back on every boot.
3. Reset the e-stop; it latches at startup and drops servo commands silently: `ros2 service call /estop std_srvs/srv/SetBool "{data: false}"`.
4. If the arm camera publishes no frames: `tools/rig_io.restart_arm_camera()` (driver restart + USB reset). The tools also do this themselves on a stall.
5. Move to the start pose with **all** joints commanded: `{1:497, 2:423, 3:235, 4:269, 5:502, 10:662}`. Gripper 662 holds the needle.

## Known problems (read before driving the arm)

| Problem | What to do |
|---|---|
| **Never build a servo command from read-back.** Servo 2 reads back ~5 pulses below where it holds, and the reading lags a step. Re-sending read-back sank the arm ~5 pulses per command; that was most of the "servo 2 sag". | `teleop.py` keeps a commanded pose. **Still to fix:** the lift/abort paths in `aim_touch.py` and `arm_touch.py`, and `tip_descend.move` callers, re-base on `read()`. |
| Servo deadband and backlash: 1–3 pulse steps often do nothing, then jump. A direction change swallows ~5 pulses. The shoulder sticks ~15 pulses upward under load, then jumps. | Steps ≥ 5; expect the first click after a reversal to do nothing. |
| Servo 2 struggles when the arm reaches low and far. | The base was driven ~4 cm closer (forward only: `/drive` has no reverse). Prefer elbow and wrist for height and reach. |
| The arm camera driver stalls every few minutes (topic up, no frames, no kernel error). | Auto-restart in `snap_arm`; a session that missed a restart never recovers, so the bridge is reopened (`reset_bridge`). |
| Lamp 12 sits in the arm view and darkens the auto exposure. | `auto_light()` prefers lamp 13 alone. |
| Teleop's target tracker can lock onto the needle or shadow (target offset stops changing). | Re-click the dot. Fix: reject template matches along the needle line. |
| SAM 3.1 point tracking loses a 12 px dot after the prompted frame. The tracker forces flash attention, which Windows torch lacks. | Patched out in `sam_detect.load()`. Use SAM for colonies (bigger), not for ink dots. |

## Next steps

1. Fix read-back re-basing in all lift/abort paths (commanded pose only).
2. Teleop: block template matches on the needle line.
3. Aimed touches on 3–5 dots. Confirm each landing: lift, then compare before/after frames at the target.
4. Milestone 3: plate under the lamps, SAM 3.1 colony detection, light sweep.
5. Add the Meta/Llama model call for pick/classify decisions (the pitch depends on it).

## Evidence

Today's run images and logs (contact frames, teleop logs) are in the session scratchpad, not in the repo. Copy them in under a new `evidence/` subfolder once the user OKs creating it.
