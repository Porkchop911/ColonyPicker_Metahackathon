# Handover: colony picker, 2026-10-29 (afternoon)

**Current plan: `docs/plan_2026-10-28.md`** (after `docs/rca_touch_dot.md`). The jab below was removed
from `model_jab.py` on 2026-10-28; the final contact is the watched wrist arc.

## Where it stands

| Milestone | State |
|---|---|
| 1. Touch dots on paper (needle placed by hand) | **First automatic touch on a dot: teal ink mark 1.0 mm from the dot centre** (dot 1, `model_jab.py --jab`, 2026-10-28). Dot 2: placed to 0.63 mm (UGREEN) but the descent stopped in the air. |
| 2. Pick up the needle, verify the tip | Not started. Tip measurement tools exist (see below). |
| 3. Colony detection with light control | SAM 3.1 runs live on the UGREEN; Hue lamps are automated. Not tried on a plate. |
| 4. Pick and transfer | Not started. |
| 5. Classify | Not started. No Meta/Llama model call yet. |

## Session 2026-10-29 (live): new cameras, side contact, 135 s touch

Details and every run: `docs/plan_speedup_2026-10-29.md` §9.

| Item | State |
|---|---|
| Cameras | **overhead = 4K UGREEN, index 1** (`rig_io.OVERHEAD`, lens `evidence/tip/ugreen4k_lens.json`, MSMF 1080p 31 fps); **side = old 1080p UGREEN, index 0** (`rig_io.SIDE`), low beside the sheet. `ugreen_dots.json` is in 4K-view pixels: `--dot/--red` take 4K-view pixels |
| Best run | `20261029T125405`: touch in 135 s, stopped by the side camera (`side-still`); ink ~1.7 mm (low confidence, on an old mark). `20261029T120925`: ink 1.49 mm, clean round blot |
| Before each run | re-ink the tip (one touch spends it); `python tools/arm_to_start.py <dir>` (start pose, elbow 190: `model_jab` lowers it measured to ~160); stop `sensor_supervisor_node` after any stack restart (by pid, see memory: `pgrep -f` over ssh matches its own shell) |
| Run | `<hackathon venv python> tools/model_jab.py --out evidence/descend --lam 190 --jab` (no 590 s tool timeout: runs take 135-500 s) |
| After | `python tools/arm_park.py <dir>` folds the arm up and back (needle clear of the paper) before power-off |
| Open | 12 mm sideways start error in 2 of 5 runs (ties exact; differs only in pan 531-533 vs 520-524); side camera not metric (grid and dots not resolvable at its angle: raise it / tilt 15-20 deg down); next touch on a fresh dot to judge accuracy |

## Session 2026-10-28 (live): what changed in `model_jab.py`

The needle is held at an angle on purpose: the final contact is a **wrist turn**, not the jab.
Run: `model_jab.py --out evidence/descend --lam 190 --jab [--dot U V]` from the start pose
`{1:497, 2:420, 3:225, 4:269, 5:502}` (servo 2 commanded 431: it settles ~11 short when rising).

| Finding (live) | Change |
|---|---|
| Servo 2 rises, but settles a fixed ~11 pulses short (17-26 at full reach); 26-30 C, 12.2 V | command +11 when rising; start-pose return in steps 380 -> 431 |
| Dots 100+ mm beyond the arc's reach; shoulder cap 100/move | the move splits to respect `SHOULDER_MAX`, measured between parts |
| Pan sticks then jumps 13-20 mm at reach; noisy gain flipped sign, ping-pong | steps <= 2, the gain sign is never flipped, no relearn on reversals; pan only beyond 6 mm |
| **Roll (servo 5) moves the tip 0.26 mm/pulse sideways**, deadband 1-2, backlash ~0.5 mm | roll closes the last 6 mm; the arm-camera tip pixel is re-found after each roll step (20 px guard) |
| Sideways steps happened at 5.6 mm; along sat at -57 mm while sideways stepped | sideways only at >= 8 mm and once along <= 10 mm |
| First move lands 6-13 mm low; along swung +-12 at GAIN 0.6 | `FIRST_MARGIN_MM` 15, `GAIN` 0.35, `PLACE_ITERS` 30 |
| The arm tie aimed 8-12 mm beside dot 2 (one dot in the arm view) | one-dot ties checked against the model carry; **UGREEN checks the arc's landing at hover**, sideways error re-aims placing (up to 3 rounds, from hover) |
| Arm-camera height read 0 with the tip above the paper; wrist read-back lag 2 and 3 both fired in the air | 1-pulse wrist steps past the height stop; **contact = the UGREEN teal tip still for 2 steps**; lag >= 6 is only a backstop |
| UGREEN tip: "darker than before" fails on the teal tip | `teal_tip`: saturated-teal blob, rotated-rectangle elongation, lowest blob |
| The inked tip drips while hovering (2 round drips on the sheet) | drips are round and ignored by `teal_tip`; marks are judged by a before/after saturation diff |

Afternoon (red dots, `--red U V`):

| Finding (live) | Change |
|---|---|
| Dot 2 touched by the "still" rule, mark 2.35 mm off; then dot 2 was under teal ink and SAM saw 2 black dots | a missing dot is filled from `ugreen_dots.json` if the others sit within 15 px |
| SAM "red dot" finds none; "small red dot" finds all 5 | `--red U V` targets the nearest red dot |
| Model arc travel off -5 / +7 mm along (red 1 landed 10 mm past) | **practice arc**: wrist down to 4 mm in the air, UGREEN teal tip travel extrapolated to the paper, re-aims along |
| Full UGREEN re-aims swung -12/+10/-8 mm (pan) | `UG_GAIN` 0.5; unconverged rounds no longer touch (unless the last check is under 5 mm) |
| Inked black dot: the tie matched 0 dots, placing swung +-20 mm | red dots are tie points in both views (matched 5) |
| Red 2: practice predicted 1.1 mm, tip landed ~5 mm short and slid through the dot | open: practice arc lower (~2 mm), stop contact at the first stalled frame |

Roll/elbow/wrist per-pulse data: `evidence/roll_tri/20261028T085059_spike/log.jsonl`. Gate decided: roll
qualifies, **gate 1 mm**. Untested live: the "still" contact rule (replayed offline on saved frames: it fires
after the tip stops) and the UGREEN re-aim with roll after the rename fix.

## THE JAB (user's definition, removed 2026-10-28)

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
| Servo 2 settles ~11 pulses short when rising (17-26 at full reach), 2-3 when falling; temperature and voltage normal (2026-10-28). | Command past the target when rising. |
| Never build a servo command from read-back (servo 2 reads ~5 low; re-sending sinks it). | All tools command from fixed values (`--start`, `commanded()`). |
| Elbow lags its command 11-14 pulses under load; the shoulder sticks when asked to rise under load. | Returns to start come from a fixed side (PRELOAD) with the elbow up first; placing uses elbow/wrist, the shoulder only moves down (in the jab). |
| Strafe speed varies 8-51 mm/s between runs (mecanum slip). | Measured by the UGREEN each time and relearnt as it goes. |
| Pan at reach: stick-slip, 0 mm then 13-20 mm jumps; strafe went the wrong way twice (2026-10-28). | Pan beyond 6 mm, roll below; no strafe. |
| Lamp `auto_light` rejects views with little grid. | `aim_touch` falls back to lamp 13 at 254. |

## Next steps

1. Dot 2 again with the "still" contact rule; then the gate: 2 in a row, 4 of 5 (1 mm, ink mark vs dot).
2. If the tip stops in the air before the paper (no mark), lower `CONTACT_STILL_PX` or require 3 steps.
3. Blot the tip after inking, so it does not drip while placing.
4. Milestone 3: plate under the lamps, SAM 3.1 colony detection, light sweep.
5. Add the Meta/Llama model call for pick/classify decisions (the pitch depends on it).

## Evidence

Run images and logs: `evidence/descend/2026102808*` - `20261028T1003*` (ink before/after: `teal_*_touch*.jpg`), `evidence/roll_tri/20261028*`.
