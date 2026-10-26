# Plan and log (written during the event)

References to plan/20, needle_touch.py, servo_shell.py and hwr_core point to the pre-existing robot repo, which is not part of this submission; their roles are taken here by tools/rig_io.py and robot/servo_bridge.py.


Pitch: a Hiwonder ROSOrin Pro picks single colonies from a sourdough-starter plate. Local vision
(SAM 3.1 detects, DINOv3 embeds), Meta API models (Llama) choose and explain, Hue lamps light the
plate three ways, a steel needle touches each colony, and photos confirm each pick.
Make-or-break arm test: `plan/20-colony-step-test.md`. Needle history: `plan/20-needle-touch-handover.md`.

## Scope decisions (user, 2026-10-26)

| Decision | Consequence |
|---|---|
| **No selective or differential media.** Plates are flour + agar, slightly acidified. | No free labels from growth, and no acid halos (they need an indicator: CaCO3 or a pH dye). Plates contaminate and age fast, and that is fine. |
| **Anything that grows is a legitimate target.** Purity does not matter: detection and pickability do. | A pick counts as a success when the plate photo shows the colony was touched and the destination spot grows. The species does not matter. |
| The starter outcompetes everything else because it is the inoculum | The first 2-3 days of a plate are mostly starter organisms |
| Hue lamps (top, side, back), NVIDIA GPU on the PC, workspace = worktree `.worktrees/hackathon`, branch `hackathon` | – |

## Architecture (all on the PC except the servo shell)

```
Hue bridge (local REST) ─ set_lamps(top|side|back)
UGREEN overhead (PC, DSHOW) ─┐
arm camera (servo_shell snap)┴─ capture_plate(lighting) ─▶ SAM 3.1 masks ─▶ per-colony features
                                                              (area, circularity, gloss = top vs side,
                                                               translucency = backlight) + DINOv3 embedding
                                                                         │
             Llama (Meta API, OpenAI-compatible): choose picks + explain uncertain ones ◀── candidate table + crops
                                                                         │
 pick(colony_id, dest_slot): overhead pixel -> base frame (plate homography) -> arm-camera servo (needle_touch.py)
                             -> ArmContact descent -> lift -> touch destination slot
                                                                         │
 verify_pick: before/after overhead diff at the colony (now) + growth at the destination slot (24-48 h)
```

Agent tools (the `hwr_agent` mission loop is already OpenAI-style; point `LM_STUDIO_URL`/`HWR_AGENT_MODEL`
at the Meta endpoint): `set_lamps`, `capture_plate`, `detect_colonies`, `classify`, `choose_picks`,
`pick`, `verify_pick`.

Reused: `tools/needle_touch.py` (`Arm` fixed-pitch IK + readback, `Camera` drain thread, `ArmContact`),
`tools/servo_shell.py`, `hwr_core.arm` kinematics, the `depth_probe` / `needle_check` / `pose_move` bench tools
(still uncommitted in the main checkout).

## Milestones

Each step climbs the same dry-run ladder and moves up one rung only once the rung below passes:

| Rung | What runs | What it proves |
|---|---|---|
| D0 offline | code on recorded frames, no robot (selftest style, like `tools/needle_contact_selftest.py`) | the logic |
| D1 air | the arm runs the full motion but stops at hover + 20 mm; nothing is touched | the motion and the camera reads |
| D2 proxy | real contact on paper, a blank plate, or a dummy needle | the contact |
| D3 real | the real target | the result |

Every rung leaves a CSV and frames under `evidence/phase21/<milestone>/`, with rates and denominators.

### 1 — Touch dots on paper, needle placed in the gripper by hand

| Step | Rung | Pass |
|---|---|---|
| 1.1 Rig recovered: estop cleared, readback fresh, needle signature at hover back to (341, 226) +-3 px | D0/D1 | `needle_check` matches at hover 3/3 |
| 1.2 Step test S0-S2 (`plan/20-colony-step-test.md`): single-pulse yield, deadband, backlash, hold drift | D1 | GO / CONDITIONAL / NO-GO verdict recorded |
| 1.3 Dot identity: a candidate that disagrees with the Jacobian prediction is rejected; tested on the run 17/18 frames | D0 | the run-18 blob jump is rejected |
| 1.4 Whole touch sequence on a dot, stopping at hover + 20 mm | D1 | aim converges <= 1 px on 5/5 dots |
| 1.5 Run 19: descend until `ArmContact` fires, on a bounded descent (`FLOOR_Z` raised) | D2 | first paper contact = the dot, <= 1 mm off, on 4/5 dots (ink mark measured on the overhead photo) |

### 2 — The robot picks up the needle itself and tracks the tip all the way

| Step | Rung | Pass |
|---|---|---|
| 2.1 Needle holder at a fixed, known spot (upright in a foam block or tube); grasp pose from IK | D0 | IK reaches the grasp and lift poses with margin |
| 2.2 Grasp approach to 20 mm above the needle head, then back | D1 | readback within tolerance; holder in view on the arm camera |
| 2.3 Grasp and lift; measure how far the needle sticks out (arm-camera signature + overhead) | D2 | protrusion measured, repeatable to +-1 mm over 5 grasps |
| 2.4 Tip checked at every stage of every move (signature present, tip pixel stable); stop on loss | D1 | a needle pulled out by hand mid-move is caught within one stage |
| 2.5 Return the needle to the holder | D2 | 5/5 returns, needle upright |
| 2.6 Milestone 1's dot touches with a self-grasped needle | D2 | same pass as 1.5 |

### 3 — Colony detection with light control

| Step | Rung | Pass |
|---|---|---|
| 3.0 Plates: flour-agar, starter spread at 10^-4..10^-6; re-plate every 2 days | – | a plate with single colonies always available |
| 3.1 Hue bridge: `set_lamps(top|side|back, level)`, state read back | D0 | each lamp switches on, off and dims on command |
| 3.2 `capture_plate`: UGREEN with exposure locked per lighting state, one frame per lamp state | D1 | 3 frames, the same pixel grid, settings logged |
| 3.3 SAM 3.1 on printed dots and photos of plates | D0 | every dot found, no false hits on the paper ruling |
| 3.4 SAM 3.1 on real plates; colony features (area, circularity, gloss, translucency) | D3 | >= 90 % of hand-counted colonies found on 3 plates |
| 3.5 Plate homography: overhead pixel -> arm base frame, checked with a dot touch | D2 | predicted vs touched <= 2 mm (arm-camera servoing closes the rest) |

### 4 — Pick and transfer to a fresh plate

| Step | Rung | Pass |
|---|---|---|
| 4.1 `pick(colony_id, dest_slot)` on a plate photo, motion planned only | D0 | targets, hover points and destination slots drawn on the photo |
| 4.2 Full pick and transfer stopping at hover + 20 mm over the colony and the slot | D1 | aim <= 1 px at both |
| 4.3 Pick on a blank agar plate with dots drawn on the lid's underside or on paper under the dish | D2 | contact detected on agar, dent visible in the before/after photo |
| 4.4 Real colonies -> fresh plate, `verify_pick` diff | D3 | the colony is visibly touched on 4/5; the slot grows within 48 h on >= 3/5 |

### 5 — Classify

| Step | Rung | Pass |
|---|---|---|
| 5.1 Ground truth for the labels (open question 1) | – | >= 30 labelled colonies |
| 5.2 DINOv3 embeddings + features -> clusters / linear probe | D0 | held-out accuracy reported with its denominator |
| 5.3 `choose_picks`: Llama gets the candidate table + crops, returns picks + reasons as JSON | D0 | valid JSON; reasons name the features that drove each pick |
| 5.4 Agent loop end to end: lamps -> capture -> detect -> classify -> choose -> pick -> verify | D3 | one full plate run |

### Demo

Video of 5.4, write-up, numbers with denominators from every milestone.

## Bench safety (from `evidence/phase20/e0/INCIDENT-20261026.md`)

- No servo command unless `{"read":true}` returns fresh pulses and the depth feed is live.
- Staged moves: one articulation at a time, <= ~200 pulses per joint, >= 2.5 s per stage; refuse a stage when
  the nearest depth return is < ~200 mm.
- Cartesian moves go through `needle_touch.Arm` (IK + pan preload), never raw multi-joint jumps.

## Risks

| Risk | Mitigation |
|---|---|
| 0 of 18 dot touches so far; the step test has never run | Step 1.2 is the gate. On NO-GO: the plate moves under a fixed arm, or a "detect, rank, guide by hand" demo |
| Bacterial colonies are ~0.5-1.5 mm, below the 1 mm placement target | Target the larger colonies (yeast-like, 2-4 mm) first |
| Contamination and moulds overgrow plates within days | Fresh plates every 2 days; mould is just another detectable class |
| The needle carries material between picks | No sterilization needed for the demo (purity is out of scope); wipe it between picks for cleaner destination spots |
| The pitch says "selective plates" and "acid halos" | Reword: "fresh plates", and gloss/translucency instead of halos |

## Open questions

1. Ground truth for "yeast vs bacteria": hand labels from shape and gloss, or a catalase test (3 % H2O2 from
   the pharmacy: yeast bubbles, lactic acid bacteria do not). The robot could dab a pick into a drop of H2O2
   in view of the overhead camera.
2. Starter type and age, incubation spot and temperature, how agar is sterilized.

## Session 2026-10-26 (live): tip triangulation works; first closed-loop descent aborted safely

| Item | Result |
|---|---|
| SAM 3.1 on the PC | `.venv` (Py 3.12, torch 2.11 cu128, `sam3` from GitHub f1647e8, triton-windows). `tools/sam_detect.py`: dot sheet 3/3 black, 5/5 red, 0.3-0.9 s/image on the 3090. Upstream bug worked around (init_state kwarg). Misses the dots in the 640x400 arm image |
| Arm camera focus | fixed; no focus control in driver/SDK, max 640x400. Needle 5-15 cm from the lens is blurred; black needle bands invisible to it (UGREEN sees them) |
| Sheet as calibration target | `tools/paper_grid.py`: 5 mm ruling -> crossings -> lattice numbering (patch walk + homography regrowth; Hough failed on the dense grid). UGREEN 1649 crossings 1.7 px rms, f ~1147, 186 mm above the sheet; arm camera 950 crossings 0.5 px |
| Tip in 3D | `tools/tip_triangulate.py`: both camera poses from the grid, frames tied by black-dot pairing + ink check, tip = meeting point of the two rays. **75.2 mm above the paper, user measured ~7.5 cm.** Repeats within 0.25 mm; ray miss 0.4-0.6 mm |
| Grasp model (this grasp) | tip in the arm-camera frame (4.15, 24.94, 236.11) mm at roll 500 (6 runs, spread <= 0.4 mm). Unchanged after an 11 mm strafe (pure translation). Roll: tip 23.6 mm off the roll axis, ~0.1 mm/pulse sideways = fine adjust without moving the camera |
| Strafe | /drive vy 0.05 x 0.3 s moved 3.4 mm (15 nominal), back-and-forth did not cancel: precise to measure, not to command |
| Contact incident | shoulder 396 -> 361 on the user's 14 cm estimate pushed the needle into the paper and shifted it in the jaws. Rule since: no downward move without a measured tip height |
| `tools/tip_descend.py` | run 1-2 refused (logging bug; fresh calibration miss 1.97 > 1.5). Run 3 (`--tip-cam 4.15 24.94 236.11 --dot 35.3 8.3`): start 74.3 mm above, 24.6 mm from the dot; lost the UGREEN tip during the pan +-6 Jacobian probe (search radius 90 px centred on the previous probe) -> abort -> lift. Lift reached shoulder 423 of 456 (33 short, inside the 35 tolerance): battery low (user: charging) |

Next: centre the UGREEN tip search on the start pose for every probe (or widen to ~150 px with dots masked); tighten the lift check (expect it to reach, not "within 35"); rerun the descent on a charged battery. Arm left at {1:497, 2:423, 3:235, 4:269, 5:502, 10:657}, estop off, needle held.

## Session 2026-10-26 night (live): camera faults found; strafe triangulation fixed; dot servo not run

| Item | Result |
|---|---|
| Reboot + carry | robot carried to the charger and back; needle shifted in the jaws (tip offset now ~(4, 58, 209) mm in the arm-camera frame, was (4, 25, 236)). Arm came up with servo 2 at its back stop (933) and elbow 3 limp; staged return elbow-first was blocked at 180, shoulder-first from 933 worked |
| Hue | `tools/hue.py`, bridge 192.168.0.70 (local API, key in config/hue.local.json, git-excluded). Rig lamps = lights 12, 13 (Hue Play). Placed behind the sheet they blinded the arm camera (auto exposure -> black); placed as side/top, every setting works. Rule: set the lamps, then open the UGREEN (its exposure locks at open). At night the UGREEN view is lit almost only by the lamps: both 254/ct 250 -> UGREEN 1713 crossings, arm 1084 inliers |
| UGREEN lens | fixed to the ruler-confirmed calibration (`evidence/phase21/tip/ugreen_lens.json`); a moved UGREEN only re-solves its pose. Tape check: projected 20-80 mm heights land on the tape marks within ~1 mm |
| Strafe triangulation | `tools/tip_motion.py`: pure-translation model gave 63.4 mm vs tape ~56 mm; the robot yaws up to 0.7 deg per strafe. With each frame's measured rotation and the arm tip ray as constraints: 57.3-59.1 mm, ray error 0.18 mm. Strafes always return to the start (closed loop, measured by the arm camera, self-correcting direction); forward steps are not used (cannot be undone) |
| Camera fault 1 | USB hub 1-2.1 connection: camera dropped off the bus (`bulk_transfer failed -4`) from 20:47 after the carry; fixed by re-seating the hub (user) |
| Camera fault 2 | `sensor_supervisor_node` kept killing/respawning the camera every 16 s after the hub was fixed (it stopped seeing frames after its own restart). User: shut it off -> killed (SIGINT; SIGTERM was ignored). Camera then 13.5 Hz, max gap 0.15 s. **It stays off only until the next stack restart; while off, lidar dropouts trigger no estop.** Root cause in the supervisor not found |
| servo_shell | deployed to the robot (old copy `~/hwr/tools/servo_shell.py.bak`): colour image BEST_EFFORT, `{"snap"}` refuses frames older than 1 s, `--no-depth` flag (no effect, can go). PC `snap_arm` refuses repeated (stale) frames |
| UGREEN backend | DSHOW stopped opening the UGREEN (after re-enumeration); `Camera` falls back to MSMF, which misreports exposure (always -6): lock the step whose scene median is closest to 140 |
| UGREEN tip | fixed threshold broke with exposure; now relative to local paper, restricted to the sheet hull, nearest blob to the prediction: stable (742, 180) |
| `tools/dot_servo.py` | user's approach: hold the dot 6 px past the tip in the arm camera, close in, UGREEN height readouts. Read-only checks only, never moved. Open: which UGREEN dot is the target is ambiguous (two unknowns fit every dot exactly; heights 43/63/152 mm). Planned fix (written, not applied): add the arm camera's own grid height as a third constraint so the leftover miss picks the dot |

Stop: battery empty. Arm at {1:502, 2:412, 3:235, 4:268, 5:503, 10:658}; it goes limp when power drops.
