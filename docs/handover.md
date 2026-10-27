# Handover: colony picker, 2026-10-27 (evening)

## Where it stands

| Milestone | State |
|---|---|
| 1. Touch dots on paper (needle placed by hand) | **THE JAB works and landed ~4-5 mm from a black dot, fully automatic** (`jab.py --live`: UGREEN finds the dot, strafe + elbow/wrist place the tip, one fluid shoulder/elbow/wrist jab). Not yet on the dot. |
| 2. Pick up the needle, verify the tip | Not started. Tip measurement tools exist (see below). |
| 3. Colony detection with light control | SAM 3.1 runs live on the UGREEN; Hue lamps are automated. Not tried on a plate. |
| 4. Pick and transfer | Not started. |
| 5. Classify | Not started. No Meta/Llama model call yet. |

## THE JAB (user's definition)

From an aimed pose about 10 mm up, the tip moves **10 mm forward and 10 mm down in one fluid move of several
servos at once**: the dot lies where the tip's camera ray meets the paper, ahead of the tip and not hidden.

- Learnt in the air (`jab.py`, 3 one-joint trials, 4 practices): **shoulder -30, elbow +14, wrist +30** from
  start (2: 298, 3: 256, 4: 373) gave 9.5/10.3 and 8.0/10.8 mm (forward/down). Per 10 pulses: shoulder -10
  = 9 mm back, 16.5 down; elbow -10 = 8 back, 10 down; wrist +10 = 7 forward, 5.6 up.
- **The shoulder is essential**: elbow and wrist alone work almost exactly against each other (10/10 needs
  about (-60, +75) pulses). The user's own teleop drops all used servo 2.
- From 10 mm up the same jab travels ~16 mm forward (not 10): the live jab missed by 4.4 mm because the
  tip was placed 10 mm behind the dot. Next run: `--fwd 16`.

## How it is measured

| Quantity | From | Why |
|---|---|---|
| Height | arm camera grid pose, tip depth 206 mm along the tip pixel's ray (median of the last 3 frames) | reliable; one bad fit once read 0.3 mm for ~20 |
| Forward / sideways | **UGREEN**: tip found as what moved (darker now than before, long and thin), needle axis from the same blob | the arm grid's y jitters 2-3 mm and slips by whole 5 mm squares |
| Black dots | UGREEN `ug_black`: grey < 0.12 x paper, round, paper round it, inside the sheet's x extent | at dusk binder rings read as dark as ink; the crossing hull cut the dots off |

## Tools (all in `tools/`)

| Tool | Use |
|---|---|
| `jab.py --out DIR --start S E W` | Learns THE JAB in the air (trials, practices, writes `jab.json`). `--dq S E W` repeats one command; `--live JAB_JSON [--fwd 16]` does the real thing: dot, place (strafe sideways, elbow/wrist forward/height, damped), jab, miss, lift. `--no-shoulder` = elbow/wrist only (cannot make 10/10). |
| `teleop.py --out DIR [--pose ...]` | Live arm view, servo panel, MARK, LIFT. Drives to the start pose (or `--pose`) first. |
| `aim_touch.py` | Older automatic aim (arm camera + UGREEN finish). Superseded by `jab.py --live`; its helpers (`ug_black`, `moved_needle`, `lock`, `neighbours`) are used by `jab.py`. |
| `aim_sim.py` | Offline simulator of the `aim_touch` loop. |
| `teleop_report.py LOG` | Deadband, backlash and mm/pulse per servo from a teleop log. |
| `tip_descend.py` | `read`, `send`, `move`, `commanded()`, `go_start()`: commands never re-based on read-back. |
| `tip_triangulate.py`, `rig_io.py`, `arm_touch.py`, `tip_motion.py` | Tip finders, stall-proof `snap_arm`, camera/bridge I/O, `auto_light`, base strafe `drive`. |

Run with the worktree venv: `D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe`.

## Robot start-up checklist

1. Robot on and the stack up (~1 min). **No e-stop any more** (removed from the stack, deployed 2026-10-27).
2. Stop the sensor supervisor if it runs: `kill -INT <pid of sensor_supervisor_node>`. It restarts the
   camera every 16 s and wrecks teleop latency. It comes back on every stack restart.
3. If the arm camera publishes no frames: `tools/rig_io.restart_arm_camera()`; the tools also do this on a stall.

## Known problems (read before driving the arm)

| Problem | What to do |
|---|---|
| **Servo 2 would not rise** at the end of the day (285-300 commanded, stays 279; down still works), even unloaded, cool to the touch, robot on power. | Unresolved. Read its temperature/voltage with the driver stopped (`hwr_core/rrc/board.py` `servo_temp`, `servo_vin`). The jab needs it. |
| Never build a servo command from read-back (servo 2 reads ~5 low; re-sending sinks it). | All tools command from fixed values (`--start`, `commanded()`). |
| Elbow lags its command 11-14 pulses under load; the shoulder sticks when asked to rise under load. | Returns to start come from a fixed side (PRELOAD) with the elbow up first; placing uses elbow/wrist, the shoulder only moves down (in the jab). |
| Strafe speed varies 8-51 mm/s between runs (mecanum slip). | Measured by the UGREEN each time and relearnt as it goes. |
| Pan barely moves the tip sideways in the UGREEN view (~0.3 mm/pulse). | Sideways by strafe. |
| Lamp `auto_light` rejects views with little grid. | `aim_touch` falls back to lamp 13 at 254. |

## Next steps

1. Get servo 2 rising again (temperature/voltage readout, then decide).
2. `jab.py --live jab.json --fwd 16`: should close the 4-5 mm miss. Then 5 dots in a row.
3. Relearn the jab from the 10 mm placing height rather than 24 mm, so `--fwd` needs no correction.
4. Milestone 3: plate under the lamps, SAM 3.1 colony detection, light sweep.
5. Add the Meta/Llama model call for pick/classify decisions (the pitch depends on it).

## Evidence

Run images and logs are in the session scratchpad, not in the repo.
