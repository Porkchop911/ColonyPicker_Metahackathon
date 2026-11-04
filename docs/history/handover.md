# Handover: colony picker, 2026-10-30 (morning)

**Current plan: `docs/plan_2026-10-28.md`** (after `docs/rca_touch_dot.md`); today's goals: `docs/plan_2026-10-30.md`.
The jab below was removed from `model_jab.py` on 2026-10-28; the final contact is the watched wrist arc.

## Session 2026-10-30: start here

**Branch `tipcam-external`, nothing committed today.** Arm at the start pose, angled needle in the gripper (655),
tip inked. Report on the last touch (pictures): https://claude.ai/artifact/MwcLxv8yfKkLgE6PajEsUs

### Result
Touch 1 on the upper dot (overhead px 1177.9, 569.5), run `evidence/descend/20261030T103024`: **ink mark 3.6 mm off**
(3.5 mm short along the needle, towards the robot; 0.9 mm sideways), clean blot, no drag. In-run estimates were all better
(practice arc 0.7, overhead tip end 2.0, arm camera 1.9 mm). Remaining dots: centre (1173, 613), right (1261, 621);
the left dot (1080, 605) carries the user's hand-guided touch blot.

### Next, in order — AGREED plan: `docs/plan_speed_accuracy_2026-10-30_AGREED.md`

Merged from the two agent plans of 2026-10-30 (`_meta`, `_OPUS`; review `plan_review_2026-10-30_meta.md`).
Targets: touch **≤ 120 s** start → lift; ink mark **≤ 1.5 mm**, then the gate (**1 mm, 2 in a row, 4 of 5**);
accuracy is measured on the mark, split (along, across) — Touch 1's in-run estimates said 0.8/0.1 mm against
the real 3.6 mm. One writer on `tools/model_jab.py` at a time; nothing committed until the user says so.

| # | Do | Who | Pass gate |
|---|---|---|---|
| 1 | **Regress hygiene.** Replay loads each run's own `ugreen_dots.json` (saved into the run folder at tie time; parent file as fallback); add `20261030T103024` to `tools/regress.py`; re-baseline | meta | 5/5, no crash-type errors |
| 2 | **Diagnostics (offline).** D1: replay round 2 of 103024 from saved arm frames (grid pose, dots, tie pairing, along per frame → grid-slip vs tie-alias). D2: `tools/mark_bias.py` — every historical mark as (along, across), mean/std. D3: overhead ray tilt at the dots, mm/mm. D4: `pose0` 15.5 s stall cause | meta | four outputs into the AGREED plan's results log |
| 3 | **S1a + S2.** Reject impossible re-ties: > 4 mm beyond model prediction after a ≤ 3-pulse move → re-snap, median of ≤ 3; a 0-dot carry never drives a > 2 mm move; `place-skip` consumes the median. Settle sized by jump (400/800/1500 ms; keep 1500 when servo 2 rises) | one writer | replay 103024 iterations drop; regress 5/5 |
| 4 | **5th dot on the sheet** (side pose needs 5; Touch 1's failed, so no triangulated height) | user | `tools/side_aim.py`: ≥ 5 dots agree |
| 5 | **Live touch 2**, centre dot (1173, 613): `arm_to_start.py`, thin re-ink (last 2–3 mm only — Touch 1's film ended ~8 px above the point; ~1.5 mm of its error), `model_jab.py --jab --dot 1173 613 --along -3.9` (skips the practice arc) | user + driver | time, mark (along, across), contact rule fired → handover table |
| 6 | **A1 (+ A2).** Practice arc `PRACTICE_H` 4.0 → 2.0, teal tip every step, quadratic along(h) at h = 0 (fallback < 3 samples; degrade to arm height without tri-height). If D2 std < 0.7 mm: `MARK_BIAS_MM` at the arc landing, refit after every touch | one writer | regress 5/5; practice prediction vs tip end < 0.7 mm |
| 7 | **Conditional speed.** S1b (overhead places for rnd ≥ 1) only if D3 ≤ ~0.3 mm/mm and ≤ 120 s still needs it; S4 descent steps 6/3/1; S5 per D4 | one writer | regress 5/5 |
| 8 | **Live touches 3–5** (right dot 1261, 621; red dots), fresh dot + thin ink each, mark measured every time | user + driver | ≤ 120 s, ≤ 1.5 mm each; then the 1 mm gate |

Deferred unless ≤ 90 s is ever required: learned Jacobian / persistent daemon (plan_speedup R3/R6).
`--along` retires as the default aim if step 6 lands (stays as manual override).
`--along -3.9`, not +3.5: `--along` *replaces* the UGREEN fix (`model_jab.py` `ug_fix[0] = args.along * fwd`), and touch 1's
fix was already -7.4 mm along fwd (+fwd = away from the robot) when the mark landed 3.5 mm short: -7.4 + 3.5. ±1 mm (the sheet
frame re-fits every retie), valid near the upper/centre dots only; confirm the sign with D2's mark tool on 103024 first.

Progress: **step 1 DONE — regress 5/5** against the re-saved `evidence/descend/regress_baseline.json` (details in the AGREED plan's results log); step 2 diagnostics (D1–D4) in progress. Step 1 shipped: per-run dots seeded; `tools/replay_run.py` prefers the run's own file;
`tie-inputs` added to the compare skip set) — regress re-verification running. Already in the tree:
`side_teal` guards (`SIDE_TRACK_PX=30`, `SIDE_WINDOW_PX=90`: Touch 1's false "side-still" at y 1079 killed,
replay-verified) and `place-skip` (`SKIP_TOL_MM=1.0`: the 14 no-op 1-pulse iterations of Touch 1 round 2).
Baseline segments for step timing: startup 47 s (`pose0` snap 15.5 s; failed side-rig ~14 s), placing 117 s
(29 reties × 4.2 s; round-2 flips are measurement, not motion), practice arcs ~25 s, descent 25 s.

### Changed today
| File | Change |
|---|---|
| `tools/model_jab.py` | arm-view dots: "dot" fallback at dark 0.85 when "black dot" finds none (sunlit view: tie had 0 dots); overhead `teal_tip`: blob must sit on paper (median gray under it > 110: the keyboard box's teal line art took the tip twice) and on the final arc only within 60 px of the last tip; `CONTACT_EXTRA` 3 -> 6 (touch stopped in the air at 3); `tie-inputs` log row |
| `tools/teach.py` (new) | hand-guided teaching: servos limp, readback logged, keyframes. Torque back on = a position command (the board's "torque on" left them limp) |
| `robot/hwr_driver_node.py` (new, vendored) | robot driver + `servo_torque` topic; deployed to `~/hwr/.../driver_node.py` (backup `.bak_20261030`). **Never run `highwonder_rework/deploy/sync.ps1`: the local repo still has the e-stop and is behind the robot** |
| `robot/servo_bridge.py` | `{"torque": {...}}` command (deployed) |
| `tools/teleop.py` | gripper (servo 10) on the panel |
| `tools/needle_grasp.py`, `tools/upright_dot_touch.py`, `tools/upright_side_touch.py` | EXPERIMENTAL, not working (below) |

### Abandoned today (user: back to the angled needle)
| Try | Outcome |
|---|---|
| Underside touch (needle hanging, tip hidden from the arm camera; overhead aims, side detects contact) | `upright_dot_touch.py` aimed onto the dot, then pressed 18 mm: its in-air rate came from steps already stalled. Aim moves at low height let the tip sink ~20 px/step (slack). SAM reads the inked needle as "pen" (0.79), not "needle" |
| Needle pick-up from a Falcon tube on a tripod | hand-taught path `evidence/teach_20261030/path.json`; replay missed (elbow sags 11 pulses under its command). Look-from-camera axis: views from the folded pose have no baseline (axis 151 mm off); calibration went limp unattended and crashed the tripod (fixed: no unattended limp). The tripod moved since; user: needle placed by hand is fine |

### Incidents
- Servo commands accepted but no joint moved for ~1 min (readback frozen at shoulder 231), then recovered by itself. Cause unknown.
- `sudo systemctl restart hwr` moved the parked arm (shoulder 900 -> 725): restart only with the arm supported.
- The UGREENs hang if a browser tab holds them: close it (use the dashboard instead).
- `tools/regress.py` 1/4: the replays need the `ugreen_dots.json` of their day (found 2, expected 4), not today's changes.

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
| Side camera | 143 mm up, tilted 39 deg. Its pose is solved at every tie from the dots it shares with the overhead (`tt.side_marks` + `tt.side_rig`: SAM "dot" on a paper crop, every 4-point correspondence, f fixed 1148); the grid cannot place it (numbering slips at that slant). Dots triangulate onto the paper within -1.1..+0.8 mm. `model_jab` logs `tri_height_mm` (tip over the paper from overhead + side, mm) at every wrist and contact step. Re-aim with `tools/side_aim.py` (OK at >= 6 of 7 dots) |
| Demo | `<venv python> tools/dashboard.py [--lan]` -> http://localhost:8000: overhead, side and arm streams with SAM 3.1 detections, click a dot to target it, Arm to start / Run touch / Stop / Park. It holds both UGREENs: run touches from it, not from a second terminal |
| After | `python tools/arm_park.py <dir>` folds the arm up and back (needle clear of the paper) before power-off |
| Open | 12 mm sideways start error in 2 of 5 runs (ties exact; differs only in pan 531-533 vs 520-524); side camera not metric (grid and dots not resolvable at its angle: raise it / tilt 15-20 deg down); next touch on a fresh dot to judge accuracy |

## Tomorrow: start here (written 2026-10-29 22:05, battery empty)

**Branch `tipcam-external`** (not merged). Arm last seen lifted over the sheet, upright needle in the gripper with teal ink
(the battery died; on power-up the arm may have sagged: park it first, `tools/arm_park.py`).

### What was being tested: the upright grasp, teal tip down (user's test)
The needle hangs nearly vertical below the gripper; the arm camera sees only its back end, so the usable tip comes from
the overhead + side UGREENs (code on this branch: `tip_cam` from external triangulation, boot-time measurement before
any motion, `black stick` SAM fallback). Results:

| Finding | Detail |
|---|---|
| The tip must start over the **paper** | over the wood the pale teal reads the wood's hue (H 12-22); over the sheet both UGREENs find it |
| Both UGREENs see the teal tip there | 21:13: tip 40.6/41.1 mm up from two side poses, rays meet within 0.08-0.73 mm |
| Reach | the arm is near its forward limit: to move the tip over the sheet the needle was tilted (wrist), pose `{2: 409, 3: 180-195, 4: 356}` |
| **Slack settle drop** | the first step down after moving up drops the tip ~10 mm (9-11 mm in 5 runs) whatever its size; afterwards the start pose itself sits lower |
| 3.5x step response | model 3 mm steps moved the long needle's tip 8-11 mm; 1-pulse steps sit in the servo deadband (false 'still') |
| Ink matters both ways | dry tip: no overhead detection (hue 20-29); thick fresh bulb: overhead and side pick points ~5 mm apart (rays miss 5-6 mm) and drips confuse |
| **Touches happened** (side-only descent, `tools/upright_side_touch.py`) | try 5: contact detected late, ~5-8 mm teal streak (dragged); try 6: tip reached the same spot (end of that streak), little extra drag; both at side-view y ~365-385 after ~20 real moves |
| Why contact is hard here | the side camera alone cannot tell descending from sliding toward it (both read 'down'); SAM's tip end flips +-6 px between the ink bulb and the metal |

Contact rule now in `tools/upright_side_touch.py` (untested live, try 7 died with the battery): progress along the in-air
direction, median of pairwise slopes over the last 6 real moves (>= 2 joint pulses each), contact under 0.4 x the in-air
slope; offline on try 6 it stays quiet through the flips (min 2.0 vs threshold 1.65). Real contact data is still needed.

### Next steps, in order
1. Robot on -> `tools/preflight.py --sam` (stops the watchdog; the side pose may need the arm out of view).
2. Re-ink thin (a film, no bulb), wipe drips under the tip.
3. Run `<venv python> tools/upright_side_touch.py <existing scratch folder>` and **watch it**: it must stop on its own at
   first contact; if it passes side y ~385, stop it (Ctrl-C) and lift (`send({3: elbow+20})`).
4. Add the overhead sideways check (fresh thin ink): side slowing + overhead lateral motion = contact.
5. Then decide the holder: the needle's presentation must put the sharp end over the paper and in both UGREENs' view.

Other state: `tools/upright_touch_session.py` (persistent session: SAM/cameras loaded once, retries) uses the side pose
from `evidence/tip/side_rig_frame.jpg` (the side camera without the needle in front; camera unmoved all evening, pose
re-solved 131-136 mm up). `tip_triangulate.side_rig(..., min_dots=)` added. The main touch path (`model_jab.py`, angled
needle, arm camera sees the tip) is unchanged on `cameras-side-contact-replay` apart from this branch's additions.

## Before the next robot session (2026-10-29 night)

| Step | Command |
|---|---|
| Robot on, then preflight (stops the watchdog, checks/restarts the arm camera, cameras, dots, side pose, ink, markers) | `<venv python> tools/preflight.py --sam --start` (or the dashboard's Preflight button) |
| Before code goes onto the robot | `<venv python> tools/regress.py --out <scratch dir>` (4 recorded runs; FAIL on any Python error) |
| Markers (optional, hand-drawn on the 5 mm grid) | `python tools/markers.py draw`: ids 8 and 19 on the notebook, id 5 on top of the robot base |
| Arm model check (~3 min, robot on, arm at start, inked tip) | `<venv python> tools/arm_calibrate.py collect` then `fit`: measures the model's error; offsets only kept if known to 1.5 pulses (self-test: ~+-4-5 pulses with 40 poses, so expect none) |

Offline findings: practice-arc landing predictions vs ink marks 1.2 / 1.9 mm (2 touches): a camera-driven final
correction (R4b) would land ~1-2 mm, about today's level. Frame writes 3 ms (no gain from background writing).

## Session 2026-10-29 evening: plan section 10 cheap wins, red dots

- Implemented (plan 10.8 "cheap wins"): dots-first tip depth (cube attempt removed), validated + lazy side rig,
  UGREEN round stop rule (`rounds-stopped`), readback-settled waits (`settle`). Side camera exposure now locks on the
  paper (`rig_io.SIDE_PAPER`): a median lock put its paper at 223/255 and SAM lost the dots (the 0-dot side-rig failures).
- Descent stops on the triangulated height (`contact by=tri-height`); far targets strafe first (`reach-strafe`,
  > 30 mm sideways); overhead black dots by SAM "dot" minus teal (small black dot went blind at dusk); an implausible
  practice arc (> 15 mm, long curved arcs) is ignored for the hover check's sideways fix.
- Red dot 1 (1133, 479): touched 170928, tip on the dot at contact (~1 mm by eye, dry tip, no mark).
- Red dot 2 (1302, 359): 5 tries, no touch: argmin crash on an empty arm view (fixed), 32 mm not strafed (threshold
  35 -> 30), 0 overhead dots (prompt fixed), practice arc 17.5 mm (now ignored), last: start tie failed (cube/dots not
  agreeing; base drifted from strafe-backs?). Red dot 3 (904, 563) not tried.
- Other agent (Muse, .pi/): ran 3 live runs in parallel, changed side_rig (one-to-one matching) and placing (reversal
  hold, roll deadband). Its `evidence/replay_new/`, `evidence/live_run_stdout*.txt`, `nul` are untracked leftovers.
- Arm parked {2: 900, 3: 0}. Nothing committed since ab37796.

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
