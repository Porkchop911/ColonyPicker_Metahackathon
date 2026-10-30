# Plan, 2026-10-30: servo state -> tip position (calibrated model; a learnt residual only if earned)

Written against branch `tipcam-external` @ 433822d (where `tools/arm_calibrate.py`, the side camera and
`docs/plan_speed_accuracy_2026-10-30_AGREED.md` live; `main` does not have them yet). Takes in
`docs/accuracy_analysis_2026-10-30.md` (meta agent). Report for the user:
https://claude.ai/artifact/5gzBWTWiEyE8sv2uiUCUCE

**Question:** train a small MLP from servo states to tip positions? **Answer:** not as a plain
pulses -> tip net. The arm's errors depend on how a pose was reached (pan backlash, shoulder sag and
stick, elbow lag; `docs/rca_touch_dot.md` §3), so a net of the pulses alone learns the mean and keeps the
scatter. ~200 poses fit the 19 physical parameters of step 3 (A-D); they do not fit a net. A learnt residual on top of the
calibrated model comes in only if the data asks for it (step 4).

**When:** gate 0 rides along on the next robot session (cheap; its floor feeds AGREED's error budget
too). Steps 2-6 only if placing still dominates touch time after AGREED steps 3 (re-tie filter) and 6
(arc fit + mark bias); otherwise their value is robustness (the 8-of-49 "placing did not converge"). The AGREED baseline's 3.6 mm
mark is ~2.0 mm arc extrapolation and ~1.5 mm ink-vs-tip; the placing flips were measurement, not motion.
A better arm model attacks neither. What it buys is **speed**: the first model move is 5-10 mm off today,
so placing needs many correction rounds. AGREED defers "learned Jacobians"; this plan keeps that.

**Not touched:** the descent and contact stay measured, closed loop. The model only places the hover
(>= 10 mm); nothing goes low open-loop (no-open-loop-low-poses rule).

Review: meta agent, 2026-10-30, "ready with fixes"; its four must-fixes and the gaps it named are
folded in below.

## Step 0: work in the right tree

This doc sits on `claude/mlp-servo-tip-mapping-61ee3b`, whose `tools/` predates `tipcam-external` (no
`arm_calibrate.py`, `regress.py`, `replay_run.py`, side camera). All code here is written on
`tipcam-external` (or on `main` once that branch and the AGREED step-3 work are merged), never on
this branch. This doc moves there with the first code commit.

## 0. What exists

| Piece | State on `tipcam-external` | Used here |
|---|---|---|
| `tools/arm_calibrate.py` | collect (~40 random poses, nearest-next order) + LM fit of shoulder/elbow offsets, `tip_cam`, base yaw + xyz; selftest. **Never run**: no `evidence/tip/arm_calibration_data.json`; nothing reads `arm_offsets.json` | extended in steps 2-3 |
| Labels | overhead teal tip + side SAM needle tip, triangulated (`tt.meet`), kept if rays miss <= 3 mm | as is: no grid-square ambiguity, unlike the arm camera's grid pose |
| Fit | own Levenberg-Marquardt, numeric Jacobian (no scipy in the venv), `sigma()` from `J^T J` | reused; new terms are more columns |
| `evidence/descend/*/log.json` | 32 pose + tip pairs, each in its own run's frame | not usable for a fit |

`arm_calibrate.py` already found that a pan offset equals a turned base and a wrist offset equals a
different tip: both stay 0. New terms follow the same rule (`USE_SIGMA`: a term known worse than its
threshold is written as 0).

## 1. Gate 0: the repeatability floor (robot, ~10 min + 30 min analysis)

No model or controller beats this, so it decides whether steps 2-6 run. The meta agent's experiments 1-3.

- Static noise: arm still, 30 frames per camera (30 side SAM runs, ~1 min; 100 is not needed for a
  sigma) -> sigma of the triangulated tip.
- One joint at a time; the other joints stay put. Per joint (pan, elbow, wrist), from 1 hover pose
  (START, tip ~45-70 mm up): 10 arrivals from +20 pulses, 10 from -20, alternating so drift does not line up
  with side. Servo 2: 5 + 5. That is 70 arrivals at ~4 s = ~5 min; a second and third pose only if
  time allows.
- Each arrival logs commanded and read-back pulses, previous position, dwell time.
- Report per joint: same-side scatter (mm), side gap in tip mm **and** in pulses (via the nominal
  Jacobian), each with a bootstrap 90% interval, not a point distance. The gap is "direction-dependent
  offset": gear dead zone plus direction-dependent sag, cable drag and servo deadband; gate 0 does not
  split those, and step 3 does not need it to.
- Dwell: arrivals at 1.6 s, plus 5 per joint at 0.4 s (AGREED S2's shortest settle) -> how much the
  tip still creeps between 0.4 and 1.6 s.

| Same-side scatter | Means | Next |
|---|---|---|
| <= 0.5 mm | repeatable; error is mostly bias | steps 2-6 |
| 0.5-1 mm | fine for placing, tight for the 1 mm mark gate | steps 2-6, note the floor in the AGREED error budget |
| > 1 mm | the 1 mm gate cannot come from any model | stop; backlash discipline or mechanics (stiffer needle, braced base) |

A large two-side gap means every final approach to the hover must come from one side (step 5).

Script: a `repeat` subcommand in `arm_calibrate.py` (same camera setup as `collect`), not a new file.

## 2. Collection: extend `arm_calibrate.collect` (robot, ~30 min + ~1 h script)

| Part | Today | Change |
|---|---|---|
| Poses | 40 | ~200; same nearest-next order (keeps neighbours close) |
| Ranges | pan +-35, shoulder 0..+60, elbow 0..+120, wrist +-(25 + 0.3 elbow) | keep; add servo 2 at two set levels so sag is excited |
| Approach | not recorded | **set, not inherited**: per pose and per joint an approach sign drawn at random (balanced, independent of the target pose and of the path). The joint first goes to target - sign x 20 pulses (20 > the widest gap gate 0 finds, so the zone is always fully crossed), then to the target. Nearest-next order alone would tie the sign to the pose and let backlash soak up sag or gain error |
| Logged per joint | – | approach sign, previous position, reversal size, dwell |
| Dwell | 1.6 s | the dwell the controller will use (AGREED S2 settle for that jump size); if gate 0 shows creep between 0.4 and 1.6 s, also log dwell and test it as an input |
| Pulses stored | read-back only (`now = read()`) | commanded **and** read-back (servo 2 settles 11-16 short rising; placing commands, so the model must map commands) |
| Label check | rays miss <= 3 mm | also hand-mark the tip point in ~20 kept frames (spread over the pose range): the label bias (the ink film ends ~8 px above the point) measured, not assumed |
| Drift | none | START repeated every 20 poses; a > 2 mm jump stops the run, data so far kept |
| Height | sweep ends when the measured tip is < 15 mm | also skip poses the nominal model puts < 25 mm up (model is 5-10 mm off) |
| Frames | not kept | keep overhead + side frame per pose, so labels can be re-measured offline |

## 3. Fit: more terms in `arm_calibrate.model` (offline, 2-3 h)

Each term stays only if held-out error falls and its sigma passes its own threshold.

| # | Term | Params | Kept if sigma <= |
|---|---|---|---|
| A | today's model: base yaw + xyz, `tip_cam`, shoulder/elbow offsets | 9 | 1.5 pulses (`USE_SIGMA`, offsets) |
| B | joint gains (pulse -> deg, nominal 0.24) for pan, shoulder, elbow, wrist | 4 | 2 % of nominal |
| C | gravity sag: shoulder, elbow deflect by `c_j * torque_j(q)`, torque from the nominal chain (link-midpoint masses) | 2 | 30 % of its own value |
| D | direction-dependent offset: joint j sits at `q_j + b_j * approach_j` | 4 | 1 pulse |

19 parameters in all. Gates pass in these units because a term below them moves the tip < ~0.5 mm.

**Not greedy:** B, C and D interact (gains change the torques sag uses; both change what D must explain),
so admission order would decide the result. Instead fit all 8 subsets of {B, C, D} on top of A
(cheap: 8 fits) and keep the smallest one within 0.2 mm held-out RMS of the best.

**Commanded vs read-back is a stage, not a swap.** Fitting FK straight from commanded pulses folds servo
steady-state error (servo 2 rests ~16 short rising) into "kinematics", and that error depends on
direction, so it collides with D. Two stages instead: (1) commanded -> read-back per joint, as a function
of approach sign and load (a small table or linear fit; the data has both columns); (2) FK + B, C, D from
read-back. D then holds only what happens after the read-back sensor (gear slack past the encoder, if
read-back is motor-side). At use, (1) predicts the read-back for a command, (2) the tip. One-stage
"FK from commanded" is fitted too, as the comparison.

Validation: 5-fold cross-validation over blocks of the pose path (neighbours never split across
train/test), drift anchors held out; the folds also split the approach signs evenly. Report RMS and 95th
percentile for each subset and the one-stage comparison against the nominal model.
`selftest` gets synthetic sag and backlash with known values; recovered within 3 sigma, as today.

| Result | Next |
|---|---|
| held-out RMS <= 2 mm, within ~0.5 mm of the gate-0 floor | step 5; no residual |
| <= 2 mm but >= 1 mm above the floor, residual shows structure vs pan/elbow/wrist | step 4 |
| > 3 mm | stop; keep the nominal model, look at labels and the base tie |

## 4. Learnt residual (only if step 3 sends it here; offline, 1-2 h)

- Gaussian process (RBF + white noise), not an MLP: ~200 points fit trivially, and its variance lets
  placing refuse a move where it is unsure. Written in numpy (~30 lines, no sklearn in the venv).
- Input: read-back pulses, approach signs, the calibrated model's tip.
  Output: a 3-D tip correction added to the calibrated model.
- Same folds as step 3; kept only if >= 0.5 mm better held-out RMS.
- Off where the variance is high, which includes everything below 25 mm.

## 5. Use it (one writer on `model_jab.py`, per AGREED)

- `coarse_aim.py` and `model_jab.py` take `--calib PATH` (the fitted JSON); nominal stays the default
  until step 6 passes. `tip_in_base` applies the terms around the vendored `hwr_kinematics`, which stays
  untouched.
- **`--calib` applies only the arm's own terms:** shoulder/elbow offsets, gains, sag, direction offsets
  and the commanded -> read-back stage. The fitted base yaw/xyz and `tip_cam` are written to the JSON
  for the record and **ignored at runtime**: the base is tied fresh each run (`sheet_from_base`) and
  `tip_cam` comes from the session's own triangulation, as today. One tip estimate, not two.
- The last move into the hover comes from the fitted approach side (one overshoot move when needed),
  so the backlash term holds.
- Gated by `tools/regress.py`: with `--calib` off, 5/5 identical as usual. With it on, the solved moves
  change on purpose, so recorded runs diverge: the gate is no crash, and every divergence is explained by
  a changed model move.

## 6. Live check (robot, ~40 min)

**Interleaved A/B, at the real hover height.** 10 dots, nominal and `--calib` alternating dot by dot
(order flipped every pair), both with the same AGREED code. Comparing against an older baseline would
credit AGREED's gains to the model. The first-move error is measured at the hover `model_jab` uses
(10 mm up), not at the 25-70 mm training heights: the envelope gap (poses trained >= 25 mm, used at
10 mm; joint angles similar, so the terms probably carry over) is measured, not assumed.

| Measure | Today | Target |
|---|---|---|
| Tip error after the first move to the 10 mm hover | 5-10 mm | <= 2 mm, and no worse than at training heights |
| Placing iterations to tolerance | up to 30 (round 2 of 103024: 20) | median <= 3 |
| Runs ending "placing did not converge" | 8 of 49 (RCA) | 0 of 10 |
| Touch time (AGREED target <= 120 s) | 223 s baseline | report the placing share |

The ink-mark gate (1 mm, 2 in a row, 4 of 5) is AGREED's and is not claimed here.

## Risks

| Risk | Handling |
|---|---|
| Servo 2 refuses to rise or slips mid-run | drift anchor every 20 poses; stop, power-cycle |
| Calibrated at 25-70 mm, used near the paper | used only for the >= 10 mm hover; residual off where unsure |
| Needle re-seated | `tip_cam` refit from 20 poses, the rest kept |
| Too many terms for the data | all subsets compared, per-term sigma gates, held-out error decides |
| Side SAM tip lost at some poses | pose skipped (as today); report the skip rate |
| Label bias depends on the pose (needle angle vs side camera) and leaks into gains or sag | ~20 hand-marked frames; refit with corrected labels if the bias trends with pose |
| Model fitted at one dwell, used at another | dwell matched to the controller, or logged and tested |

## Order

| Step | Robot | Time | Stop if |
|---|---|---|---|
| 0 port | no | – | code goes on `tipcam-external` / merged `main` only |
| 1 gate 0 | yes, on the next robot session | 10 min + 30 min analysis | same-side scatter > 1 mm |
| — | AGREED steps 3, 6 land; placing still dominates touch time? | | no: stop here, keep gate 0's numbers |
| 2 collect | yes | 1.5 h | anchor jumps twice |
| 3 fit | no | 2-3 h | held-out RMS > 3 mm |
| 4 residual | no | 1-2 h | not >= 0.5 mm better |
| 5-6 use, check | yes | 1.5 h | `--calib` no better than nominal in the A/B |

## Results

**Gate 0, 2026-10-30, `evidence/tip/arm_repeat_20261030T141229.json` (run 6; runs 1-5 were measurement fixes):
GO.** START, tip 84 mm up; 20-pulse excursions; overhead teal tip + side SAM needle, rays meet to 0.7-4.6 mm.

| Joint | Scatter from + side | Scatter from - side | Side gap | Read-back after return, + / - | 0.4 s vs 1.6 s dwell |
|---|---|---|---|---|---|
| still (camera noise) | 0.20 mm | | | | |
| pan (1) | 0.28 mm | **2.61 mm** (stick-slip) | 15.2 mm | 498 / 495-496 | 0.22 mm |
| elbow (3) | 0.32 mm | 0.05 mm | 5.8 mm | 188 / 179 | 0.21 mm |
| wrist (4) | 0.36 mm | 0.07 mm | 2.7 mm | 269 / 266 | 0.19 mm |
| shoulder (2) | 0.12 mm | 0.63 mm | 8.3 mm | 417 / 407-409 | 0.47 mm |

- From the + side every joint returns to the same point, (44.9, 18.0, -84.1) mm, within 0.4 mm. A fixed final
  approach from + is repeatable to well under the 1 mm gate.
- From the - side elbow, wrist and shoulder stop short, and **read-back shows it** (9, 3, 9 pulses): this is
  servo position error before the read-back, so step 3's two-stage model (command -> read-back -> tip) can carry it.
  Pan's 15 mm gap with only 2-3 pulses of read-back difference is slack after the read-back.
- Settle 0.4 s is as good as 1.6 s (<= 0.5 mm): AGREED S2's short settles cost nothing here.
- Pulses-per-mm in `repeat-report` use a guessed tip offset (`TIP_GUESS`); the mm numbers are measured.
