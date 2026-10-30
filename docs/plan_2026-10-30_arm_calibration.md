# Plan, 2026-10-30: servo state -> tip position (calibrated model; a learnt residual only if earned)

Written against branch `tipcam-external` @ 433822d (where `tools/arm_calibrate.py`, the side camera and
`docs/plan_speed_accuracy_2026-10-30_AGREED.md` live; `main` does not have them yet). Takes in
`docs/accuracy_analysis_2026-10-30.md` (meta agent). Report for the user:
https://claude.ai/artifact/5gzBWTWiEyE8sv2uiUCUCE

**Question:** train a small MLP from servo states to tip positions? **Answer:** not as a plain
pulses -> tip net. The arm's errors depend on how a pose was reached (pan backlash, shoulder sag and
stick, elbow lag; `docs/rca_touch_dot.md` §3), so a net of the pulses alone learns the mean and keeps the
scatter. ~200 poses fit ~28 physical parameters; they do not fit a net. A learnt residual on top of the
calibrated model comes in only if the data asks for it (step 4).

**When:** after AGREED steps 3 (re-tie filter) and 6 (arc fit + mark bias). The AGREED baseline's 3.6 mm
mark is ~2.0 mm arc extrapolation and ~1.5 mm ink-vs-tip; the placing flips were measurement, not motion.
A better arm model attacks neither. What it buys is **speed**: the first model move is 5-10 mm off today,
so placing needs many correction rounds. AGREED defers "learned Jacobians"; this plan keeps that.

**Not touched:** the descent and contact stay measured, closed loop. The model only places the hover
(>= 10 mm); nothing goes low open-loop (no-open-loop-low-poses rule).

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

## 1. Gate 0: the repeatability floor (robot, ~15 min + 30 min analysis)

No model or controller beats this, so it decides whether steps 2-6 run. The meta agent's experiments 1-3.

- Static noise: arm still, 100 frames per camera -> sigma of the triangulated tip.
- Same-side return: 3 hover poses (tip 30-50 mm up), each reached 10x from +30 pulses away per joint
  (pan, elbow, wrist; a separate 5x block for servo 2) -> scatter.
- Other side: the same from -30 pulses -> the distance between the two clusters = backlash per joint.
- Each arrival logs commanded and read-back pulses, settle 1.6 s as in `collect`.

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
| Approach | not recorded | per joint +1/-1; half the poses reached via a 15-pulse overshoot from the other side |
| Pulses stored | read-back only (`now = read()`) | commanded **and** read-back (servo 2 settles 11-16 short rising; placing commands, so the model must map commands) |
| Drift | none | START repeated every 20 poses; a > 2 mm jump stops the run, data so far kept |
| Height | sweep ends when the measured tip is < 15 mm | also skip poses the nominal model puts < 25 mm up (model is 5-10 mm off) |
| Frames | not kept | keep overhead + side frame per pose, so labels can be re-measured offline |

## 3. Fit: more terms in `arm_calibrate.model` (offline, 2-3 h)

Added one at a time; each stays only if held-out error falls and its sigma passes `USE_SIGMA`.

| # | Term | Params | Note |
|---|---|---|---|
| A | today's model: base yaw + xyz, `tip_cam`, shoulder/elbow offsets | 9 | baseline |
| B | joint gains (pulse -> deg, nominal 0.24) for pan, shoulder, elbow, wrist | 4 | |
| C | gravity sag: shoulder, elbow deflect by `c_j * torque_j(q)`, torque from the nominal chain (link-midpoint masses) | 2 | needs B's poses at two servo-2 levels |
| D | backlash: joint j sits at `q_j + b_j * approach_j` | 4 | gate 0 gives the starting values |
| E | input commanded vs read-back (same model, refit) | 0 | keep whichever predicts better on held-out poses |

Validation: 5-fold cross-validation over blocks of the pose path (neighbours never split across
train/test), drift anchors held out. Report RMS and 95th percentile for A..E against the nominal model.
`selftest` gets synthetic sag and backlash with known values; recovered within 3 sigma, as today.

| Result | Next |
|---|---|
| held-out RMS <= 2 mm, within ~0.5 mm of the gate-0 floor | step 5; no residual |
| <= 2 mm but >= 1 mm above the floor, residual shows structure vs pan/elbow/wrist | step 4 |
| > 3 mm | stop; keep the nominal model, look at labels and the base tie |

## 4. Learnt residual (only if step 3 sends it here; offline, 1-2 h)

- Gaussian process (RBF + white noise), not an MLP: ~200 points fit trivially, and its variance lets
  placing refuse a move where it is unsure. Written in numpy (~30 lines, no sklearn in the venv).
- Input: pulses (commanded or read-back, per step 3E), approach signs, the calibrated model's tip.
  Output: a 3-D tip correction added to the calibrated model.
- Same folds as step 3; kept only if >= 0.5 mm better held-out RMS.
- Off where the variance is high, which includes everything below 25 mm.

## 5. Use it (one writer on `model_jab.py`, per AGREED)

- `coarse_aim.py` and `model_jab.py` take `--calib PATH` (the fitted JSON); nominal stays the default
  until step 6 passes. `tip_in_base` applies the terms around the vendored `hwr_kinematics`, which stays
  untouched.
- Base yaw/xyz are per session: still tied fresh from a frame each run (`sheet_from_base`), not reused.
- The last move into the hover comes from the fitted approach side (one overshoot move when needed),
  so the backlash term holds.
- Gated by `tools/regress.py` 5/5, like every AGREED code step.

## 6. Live check (robot, ~30 min)

| Measure | Today | Target |
|---|---|---|
| Tip error after the first move to the hover | 5-10 mm | <= 2 mm |
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
| Too many terms for the data | one term at a time, `USE_SIGMA`, held-out error decides |
| Side SAM tip lost at some poses | pose skipped (as today); report the skip rate |

## Order

| Step | Robot | Time | Stop if |
|---|---|---|---|
| — | AGREED steps 3, 6 land first | | |
| 1 gate 0 | yes | 45 min | same-side scatter > 1 mm |
| 2 collect | yes | 1.5 h | anchor jumps twice |
| 3 fit | no | 2-3 h | held-out RMS > 3 mm |
| 4 residual | no | 1-2 h | not >= 0.5 mm better |
| 5-6 use, check | yes | 1.5 h | no fewer placing iterations than today |
