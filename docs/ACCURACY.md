# Why the tip misses: an error budget

Everything the repo currently knows about the sources of position error, ranked by how many millimetres each one
is worth. Written 2026-11-01 from the evidence on disk plus a fresh re-analysis of it. It is a diagnosis, not a
plan — `docs/history/plan_2026-10-30_arm_calibration.md` is still the plan of record.

**How to read the evidence column.**
**M** = measured in a run whose file is committed in `evidence/`, reproducible with a command below.
**P** = published statistic in a doc or code comment, traceable to a run but not re-derived here.
**A** = asserted: stated in a doc, no artifact. Treat as folklore until measured.

## 1. The short version

| Quantity | Today | Gate | Evidence |
|---|---|---|---|
| Tip seen by two cameras, static scene | **0.20-0.56 mm** 1-sigma scatter | — | M `arm_repeat_20261030T141229.report.json static_mm 0.2046`, `…T135951.report.json static_mm 0.5588` |
| Tip position predicted by the calibrated arm model | **1.20 mm** median, **2.34 mm** p90 (9 poses, fit data) | 1 mm | M `arm_model.json tie_median_mm / tie_p90_mm`; re-derived by refit |
| Same model on data it has never seen | **1.67 mm** median, **3.90 mm** p90 (worst 66 mm) | 1 mm | M 5-fold cross-validation re-run, §5 |
| Hover over a crossing, 10-40 mm up, corrected | **0.5-0.95 mm** median | — | M `arm_aim_20261030T212403.json` calibrated rows; `arm_hover_*` trails |
| Where the needle actually puts ink | **1.26 mm** and **2.71 mm** off the printed crossing | 1-2 mm | M `evidence/tip/dot_offsets.json` |
| Tip depth above the paper (`lam`) | a constant, 190 / 202.5 / 206 in different tools | should be measured | M `tools/dashboard.py:190`, `tools/coarse_aim.py:94`, `tools/model_jab.py:180` |

The gap between row 1 and row 5 is the whole problem: the cameras are good to half a millimetre, the model is
good to one, and the needle lands two millimetres off. Everything between row 1 and row 5 is where the error is
made.

## 2. Ranked error budget

Values are the millimetres that channel is worth **at the tip**, in the configuration the rig actually uses.

| # | Channel | Worth | Class | Reducible? |
|---|---|---|---|---|
| 1 | Actuator direction dependence: backlash, hysteresis, creep, sag | **2.7-15.2 mm** between approach directions; **0.1-2.6 mm** of pure scatter | M | Partly. Direction discipline is already in the code; the residual is the plant. |
| 2 | Tip depth `lam` treated as a constant | **16 mm** of disagreement between tools today; a few mm each time it is stale | M | Yes — `dot` already measures it and throws the number away. |
| 3 | Vision **bias** (not noise): the two views disagree with each other | **0.6-2.6 mm** median ray miss in the calibration data itself | M | Yes, with a measured reference plate. |
| 4 | Calibration model instability across sessions | **25-66 mm** when a model meets data from another session | M | Yes — refit per session, which the tools already insist on. |
| 5 | Sequence defects: blind final descent, open-loop target carry, no contact stop | **~2-3 mm** at the moment that matters | M | Yes, mostly software. |
| 6 | Contact mechanics: needle bend, ink skid, drag past contact | **~2 mm** systematic mark offset | M | Partly — softer stop, shorter/stiffer needle. |
| 7 | Sheet frame and ruling assumptions | **~2 mm** near the far edge; **5 mm** jumps if a square is miscounted | M/P | Yes, locally. |
| 8 | Geometry never verified: camera position in the model, pulse↔URDF signs | **1.6-3.5 mm per 0.5-1°** of camera-pose error | A | Yes — one hardware check closes it. |
| 9 | Lighting and exposure drift through the afternoon | shifts detection thresholds, no numbers | P | Partly, with the existing auto-exposure. |

## 3. Measurement is precise, not accurate

Three numbers that must be read together.

**Precision is genuinely good.** 30 still frames of a stationary arm at START: `static_mm` 0.2046 mm
(`evidence/tip/arm_repeat_20261030T141229.report.json`) and 0.5588 mm (`…T135951.report.json`). On 2026-10-26,
the same tip measured repeatedly while parked 75 mm above the paper gave heights 75.05, 75.19, 75.29 mm and ray
misses 0.28-1.12 mm (`evidence/tip/15*_tip.json`).

**But the calibration data is noisier than the static test suggests.** Every pose in every calibration run carries
`miss_mm` — the distance between the two cameras' rays to the same tip. Re-derived from the committed JSON:

| calibration run | poses | median ray miss | p90 | max | gate |
|---|---|---|---|---|---|
| `arm_calibration_data_20261030T151228.json` | 41 | **1.49 mm** | 2.03 | 4.41 | 5.0 |
| `arm_calibration_data_20261030T152658.json` | 83 | **2.63 mm** | 3.40 | 4.71 | 5.0 |
| `arm_calibration_data_20261030T211239.json` | 10 | **0.59 mm** | 0.97 | 1.12 | 5.0 |

`MISS_MM = 5.0` (`tools/arm_calibrate.py:164`) is the acceptance gate, and `collect()` also filters poses whose
two views disagree with the model. Under that gate the 152658 fit sees rays 2.63 mm apart **median** — before any
arm error exists at all. A model fitted on data whose measurement error is 2.6 mm cannot honestly claim 1.2 mm
accuracy, and the difference between "the arm is off" and "the measurement is off" is invisible in these numbers.

**And the filter selects on measurement luck.** The clean run (211239, median 0.59 mm) is the run whose poses came
from `plate_rig.py` — i.e. from `evidence/tip/side_pose_cache.json`, the same lens estimate whose crossings were
already matched with 0.8 mm tolerance. Run 152658, which measured its poses live, has 4.4× the ray disagreement at
the median. The repo has never measured vision error against a reference it did not also fit. The closest thing to
a truth check is `tools/tip_triangulate.py:108` ("fixed, dots triangulate onto the paper within -1.1..+0.8 mm and
the height matched the tape") and the 4K lens note in `evidence/tip/ugreen4k_lens.json`: **"Not yet checked on the
tape"**.

Consequence: treat the arm model's sub-2 mm numbers as *agreement between the model and a measurement of unknown
bias*, not as accuracy.

## 4. The arm does not arrive at the same place from the two sides

This is the largest measured channel and the best-documented one.

**Side gaps at START** (`arm_repeat_20261030T141229.report.json`, 20 pulses away, return to START, ±90 %
interval):

| joint | away side | gap between the two return paths | in pulses | return-leg scatter | creep (0.4 s dwell vs 1.6 s) |
|---|---|---|---|---|---|
| 1 pan | +1 | **15.18 mm** | 13.6 | 0.28 / 2.61 mm | 0.22 mm |
| 2 shoulder | +1 | **8.28 mm** | 6.5 | 0.12 / 0.63 mm | 0.47 mm |
| 3 elbow | −1 | **5.83 mm** | 4.2 | 0.32 / 0.05 mm | 0.21 mm |
| 4 wrist | −1 | **2.74 mm** | 2.4 | 0.36 / 0.07 mm | 0.19 mm |

`static_mm 0.2046`, verdict **"go: calibrate"**.

Earlier the same day, the same test with different dwell/counts (`…T135951.report.json`) gave `static_mm 0.5588`,
pan return scatter **29.62 mm** on one side and 0.55 mm on the other, and verdict **"stop: no model meets the 1 mm
gate"**. The verdict flipped within three hours because a single joint locked into its bad direction. That is the
practical meaning of hysteresis here: the same script returns opposite advice depending on which side the joint
happens to be on when you start.

**Read-back confirms the mechanism, directionally.** Across 212 hover attempts, the commanded pulse is compared to
what the servo reports:

| joint | approaching from below (raising) | approaching from above (falling) |
|---|---|---|
| 1 pan | +1, always | −1 to −2 |
| 2 shoulder | **−11 to −13 short** (68 of 92 rows) | −1 to −3 short |
| 3 elbow | **−11 short** | −1 to −3 short |
| 4 wrist | 0 to −3 | 0 to −3 |

The shoulder loses 11-13 pulses when it has to lift a load, and 1-3 when it falls. The correction in
`arm_model.json` is `b2 = −5.74` pulses — a compromise between two states, not a model of either.

**The fine axis has no agreed size.** Pan is the only joint that moves the tip sideways without a big arc, and the
repo quotes its scale at least six different ways, all from real runs: **1.114 mm/pulse** (the gate-0 repeat report's
nominal model at START, `evidence/tip/arm_repeat_20261030T141229.report.json`), **~0.8** ("One pan pulse is ~0.8 mm
sideways out here", `tools/arm_calibrate.py:1206`), **1.2-1.8** (`tools/model_jab.py:50`), **~2.2** (`tools/model_jab.py:53`
"half a pan pulse"), **2.95** (`tools/aim_touch.py:35`, Jacobian row), **~4** (`tools/model_jab.py:84` "pan ~4 mm/pulse
with slack"), and **−0.28** in one jab run (`tools/jab.py:200`). A 5× spread on the only axis available for lateral
fine work. On top of it sit `PAN_STEP = 2`, `PAN_BACKLASH = 5` and `PAN_SIGMA_MM = 4.0`
(`tools/model_jab.py:54-78`) — so the smallest commandable sideways step is 2 pulses (1.6-8 mm depending which scale
you believe) with a ~4 mm spread around it. Nothing below roughly 4 mm can be commanded sideways reliably, which is
why the whole fine strategy in `docs/history/plan_speed_accuracy_2026-10-30_AGREED.md` is "get XY to within 4 mm at
40 mm up and then finish by going straight down".

### Which side the code approaches from, and which side the measurements say to use

Gate 0 judges each joint on its **better** side and records that side as `approach` in the report
(`repeat_stats`, `tools/arm_calibrate.py:741-748`); the verdict compares each joint's *best* side against fixed
thresholds (`:746-749`): `go: calibrate` if the worst best-side scatter is ≤ 0.5 mm, `go, tight for a 1 mm mark` if
≤ 1.0 mm, otherwise `stop: no model meets the 1 mm gate`. Run `20261030T141229` chose sides **1: +1, 2: +1, 3: −1,
4: −1** (worst best-side scatter 0.279 mm). `drop_mm(joint, side)` (`:653`) says what those sides mean physically at
START, in millimetres of tip height for the away pose (`+ = lower`):

| joint | away side +1 | away side −1 |
|---|---|---|
| 1 pan | no height change | no height change |
| 2 shoulder | **22.8 mm higher** | 21.7 mm lower |
| 3 elbow | 20.7 mm higher | **19.1 mm lower** |
| 4 wrist | 13.6 mm higher | **12.0 mm lower** |

So gate 0 wanted the pan and shoulder to fall into place from above, and the elbow and wrist to be lifted into
place from below. Note that a consistent read-back shortfall is not itself a problem — the model absorbs a bias;
scatter is what gate 0 measures. That is why the elbow, which loses 11 pulses when it lifts, was still judged
better on the lifting side (0.05 mm scatter vs 0.32 mm).

The live code does not do this. Every approach in production uses **one sign for all four joints**:
`coarse_aim.APPROACH = -1` (`tools/coarse_aim.py:39`, "from 20 pulses above (gate 0's repeatable side)") and the
away move is `planned[k] - APPROACH * 20` (`tools/coarse_aim.py:156`; the same expression at
`tools/arm_calibrate.py:1040`, `:1162`, `:1368`) = **target + 20 pulses on every joint**, i.e. from above for all
four — the side gate 0 rejected for the elbow and the wrist.

The fitted direction offsets inherit the same single-sign assumption. `arm_pulses` applies them as
`out[k] += terms["b"][str(k)] * sign.get(k, APPROACH)` (`tools/coarse_aim.py:42-48`), and `arm_model.json` stores
`"approach": -1` — one value for four joints — while the data they were fitted on drew those signs **at random, per
pose and per joint** (`approach_plan`, `tools/arm_calibrate.py:437-444`, deliberate: "nearest-next order alone would
tie it to the pose and let backlash soak up sag or gain error"). A single `b_j` is therefore an average of two
directions in the fit and is then applied in one direction only, and not the direction gate 0 measured as the
repeatable one for two of the four joints.

One more trap for whoever reads this next: the word `approach` means opposite things in the two places it appears.
In `arm_repeat_*.report.json` it is the **away side** (`away = START + side × 20`). In `arm_calibration_data_*.json`
it is the **sign used in `arm_pulses`**, where `approach: +1` means the away pose was *below* the target. Keep that
straight before re-deriving anything from these files.

**Dwell matters, and it is measured.** Waiting 1.6 s instead of 0.4 s after a move changes the landing by
0.19-0.47 mm (§4 table). `REPEAT_DWELL = 1.6` / `SHORT_DWELL = 0.4` (`tools/arm_calibrate.py:637-638`),
`REPEAT_AWAY = 20` pulses chosen as "> the widest gap seen, 5 pulses" (`:636`). The code is consistent about this
in calibration and inconsistent in production: `tip_descend.py:83` dwells **0.6 s** with a 35-pulse read-back
tolerance, `coarse_aim.py:159` dwells 0.3 s at the approach-from-above pose, `arm_calibrate.place()` dwells 2.5 s
(`tools/arm_calibrate.py:1190-1195`, plus the +15-pulse take-up on joints 2 and 3 first).

## 5. The calibration works — on data it was fitted to, in the session it was fitted in

**It helps, measured.** Twelve aimed hovers at 60 mm, six crossings, both models, each anchored at START
(`evidence/tip/arm_aim_20261030T212403.json`): calibrated **0.95 mm** median xy (worst 1.72), nominal **1.91 mm**
median xy (worst 4.18). `arm_model.json` says the same thing as a fit residual: nominal 6.70 mm median →
calibrated 1.20 mm median on the tie run.

**It does not travel.** Fitting each calibration run independently, in that run's own sheet frame:

| source run | poses | d2 (shoulder zero) | d3 (elbow zero) | b1 | b2 | b3 | b4 | in-sample median | p90 | worst row |
|---|---|---|---|---|---|---|---|---|---|---|
| 151228 | 39 | **−0.7** | **−28.8** | 5.05 | −4.67 | −4.28 | −1.07 | 1.26 mm | 3.62 | **112 mm** |
| 152658 (the shipped model) | 78 | **+19.4** | **−39.7** | 4.82 | −5.74 | −4.14 | −1.32 | 1.39 mm | 3.66 | **65 mm** |
| 211239 | 9 | **−32.1** | **+22.4** | 4.88 | −4.22 | −3.78 | −1.70 | 0.86 mm | 1.42 | 2.0 mm |
| run8 | 10 | +226.6 | +74.0 | 5.30 | −9.82 | −0.62 | −4.58 | 2.74 mm | 4.99 | 7.7 mm |

**The mismatch is measurable inside the file itself.** `fit_arm` is called two ways: the 10-30 `fit` fitted all 13
parameters on run 152658 (78 poses), and `fit-tie` (`tie_only`, `tools/arm_calibrate.py:797`) later kept those arm
terms and refit the tie (tip, yaw, base, z) from run 211239 (9 poses) — into the same file. Evaluating the shipped
`session` vector through the repo's own predictor (`arm_model(p, cmd pulses, approach signs)`):

| scored against | n | median residual | p90 | worst | rows over 5 mm |
|---|---|---|---|---|---|
| 211239, the run the tie came from | 9 | **1.20 mm** (= the published `tie_median_mm`) | 2.34 | 2.8 | 0 |
| 152658, the run the file still names in `runs` and whose `median_mm`/`p90_mm` it publishes | 78 | **65.68 mm** | 129.2 | 136.8 | **78 of 78** |

Cross-session refits tell the same story: fitting each run in its own frame and applying the shipped arm terms to a
different run's data gives residuals of 25-31 mm median. Fitting run 211239 independently gives d2 = **−32.1**,
d3 = **+22.4** where the shipped file says **+19.4** and **−39.7** — 51 and 62 pulses apart. `fit_tip` refuses
outright (`tools/arm_calibrate.py:1007` docstring: "tip_cam_mm is fit from this run's grasp and the sheet frame is
fit from its crossings — a model is only usable in the session it was fitted in"). `docs/STATUS.md` records the
rule: calibrate, use, and refit on the same day, at the same roll.

**The headline numbers in the file describe a fit that has been overwritten.** `median_mm 1.391` / `p90_mm 3.663`
come from 78 poses in run 152658 — in-sample, and 152658 contains a row whose residual is **65 mm** and another with
a 4.7 mm ray miss (`miss_mm`, gate 5.0). The model in the file no longer reproduces them (§5 above). Held-out
performance for 152658 (5-fold cross-validation re-run on the committed data, fit ~63 poses / score ~15, five folds):
pooled **median 1.67 mm, p90 3.90 mm, worst 66.0 mm**, per-fold test medians 0.87 / 1.15 / 1.46 / 1.54 / 2.11.
`fit_arm`'s own docstring quotes "1.8 mm median / 4.0 mm p90" for held-out prediction
(`tools/arm_calibrate.py:799`), which matches. So the honest statement about the current model is: *about 2 mm,
sometimes 4 mm, occasionally catastrophically wrong.*

**The two arm terms are only weakly identified.** Across folds of the same run: `d2` ranges 13.4-22.4 pulses,
`d3` ranges −49.0 to −31.0, while the direction terms stay put (`b1` 4.8-4.9, `b2` −6.5 to −5.2, `b3` −4.8 to −2.2,
`b4` −1.2 to −1.5). The direction terms are real; the zero offsets trade against the sheet frame they are fitted
with. Two different runs put `d3` at −39.7 and +22.4 — 62 pulses, 15°, of disagreement about where the elbow zero
is.

**And it is not used where it matters.** `tools/model_jab.py` has no `--calib` flag and never sets `coarse_aim.CALIB`;
`tools/dashboard.py:190` launches it as `--out evidence/descend --lam 190 --jab`. `coarse_aim.py` *does* support
`--calib` (`:101`) and changes its approach strategy when it is supplied (approach from 20 pulses above, per-joint
direction offsets, `:44-50`). The production placing path is the nominal URDF. `tools/regress.py` never loads
`arm_model.json` either, so the 5-run regression protects code paths, not model quality.

## 6. Tip depth is assumed, not measured — and the assumption is inconsistent

Height is not measured directly by either camera. It is `lam × the unit ray from the tip camera`:
`tip_cam = args.lam * at.unit_ray(free)` (`tools/model_jab.py:368`), `tip_cam = args.lam * ray`
(`tools/coarse_aim.py:120`). Whatever `lam` you pass becomes the tip's distance from the camera, in millimetres,
for every subsequent calculation including the paper plane and the descent schedule.

| where | value |
|---|---|
| `coarse_aim.py:94` default | 202.5 |
| `model_jab.py:180` default | 206.0 |
| `jab.py:57` default | 206.0 |
| `aim_touch.py:77` default | 206.0 |
| `dashboard.py:190` (what the dashboard actually runs) | **190** |
| `replay_run.py:101` (what replays use) | **190** |
| measured in the RCA | 202.5 → 206 after reopening the gripper → **188** after an angled re-mount |

`model_jab.py:537` says it out loud: "the start depth --lam holds until the dots tie triangulates it below". Until
the dots tie succeeds — which needs a side-camera crossing match — the whole chain runs on the constant. A 16 mm
spread on the constant that sets height means a 16 mm possible bias in every descent, and the descent is planned
against the paper plane at `tip_z = 0`.

The tool that would fix this already exists and already computes the number: `dot` measures the UGREEN distance
and writes it into the hover trail as `lam_ugreen_mm`. No committed trail has that field — the only 4K measurement
on disk is `evidence/tip/arm_hover_20261030T214136.json`, where the needle tip sits at z = −54.6 mm and the 4K
sheet at z = +64.0 mm, i.e. **118.6 mm** of tip-to-sheet distance. Grepping every committed `evidence/descend` and
`evidence/tip` JSON for a recorded `lam` returns nothing. The single quantity that decides whether the needle hits
the paper is never logged.

## 7. Contact and ink

**Contact detection is switched off.** `DOT_STOP_AT_CONTACT = False` (`tools/arm_calibrate.py:923`). The machinery
is there and calibrated: `CONTACT_ROW = 0.4` (:924, "a 1 mm step whose side-view tip row falls < 40 % of the
rows/mm seen above 2 mm: on the paper") and `CONTACT_MM = 0.3` (:925, "a commanded mm down that moves the tip less
than this: it is on the paper"). Because it is off, `dot` executes the full `TOUCH_STEPS_MM` ladder —
`(8, 6, 4, 3, 2, 1, 0, −1, −2, −3)` (:917) — and ends 3 mm *below* the measured paper plane.

**The paper is not where the model says it is.** From the committed hover rows, goal height minus measured tip
height (positive = tip above where it was commanded):

| commanded height | median z error | n |
|---|---|---|
| +15 mm | +0.11 | 29 |
| +10 mm | **−0.21** | 22 |
| +1 mm | **−0.91** | 6 |
| 0 mm | **−0.97** | 6 |
| −2 mm | **+0.85** | 3 |
| −3 mm | **+1.32** | 3 |

Above the paper the model is right to ~0.2 mm at 15 mm, and off by ~1 mm at 1 mm. Below the paper the sign flips:
the tip is *above* its commanded depth because the paper holds the needle up. In the one run with a 4K measurement
(214136) the sheet was 64 mm up in UGREEN's frame while the arm model called +3 mm contact, and the run stopped at
−3 mm because `h <= TOUCH_STEPS_MM[-1] or blind >= 3` (:1247). The "paper plane" the descent is planned against is
the plane fitted from the overhead crossings, not the plane the needle meets.

**The ink moves.** `dot` compensates with the previous session's average mark offset from `dot_offsets.json`, which
holds two measurements: **1.26 mm** and **2.71 mm** off the printed crossing. The correction is applied to the aim
before descending, so a systematic 2 mm skid is subtracted — but only if it is the same skid, and the average has
n = 2. `CONTACT_EXTRA = 6` pulses with "the drag is ~0.76 mm per pulse past contact" (`tools/model_jab.py:92`) and
a contact approach ~72° from vertical all say the last centimetre is not a rigid-body problem.

**Depth is only trusted above 15 mm.** `MIN_HEIGHT_MM = 15.0` (`tools/arm_calibrate.py:34`) rejects any measurement
closer than that, and `TOUCH_XY_MM = 3.0` comments "readings near the paper scatter 1-2 mm; 2.0 aborted at the
paper twice" (:918). Every measurement the final approach depends on is in the region the code declares unreliable.

## 8. The sheet frame and the ruling

- `paper_grid.SQUARE_MM = 5.0` (`tools/paper_grid.py:17`) — the graph-paper pitch is *assumed from the sheet*, and
  `local_crossing` (`tools/arm_calibrate.py:944`) is explicit that "the global lattice was ~2 mm off the ruling
  near the sheet's far edge (fits 0.8 px overall; paper not flat there, 2026-10-30)".
- Lattice indexing has historically been the biggest single error in the chain: `tools/arm_calibrate.py:41` ("the
  overhead numbers the grid from whichever crossing it locks onto, so its sheet frame moves by whole squares
  between runs") — whole-square errors are 5, 10, 20 mm. `cameras()` brute-forces shifts up to ±100 mm to remove
  them.
- Two hardcoded corrections carry their own history: `TOP_PLATE_FIX = (1.58, 1.47, 0.46)` with "Without it the
  chain was 2.25 mm off" (`tools/plate_rig.py`, measured from one view), and `chain_off_mm: [0.5, 0.5]` in
  `evidence/tip/side_pose_cache.json`.
- Focal length moves the ruler. `tools/tip_triangulate.py:43` records single-view calibration drifting
  **1097-1163 px between frames with the ray miss going 0.4 → 2.4 mm**, and `:108` records that freeing the side
  focal length on 6 dots moves it "to 1056 or 1262" while keeping it fixed puts dots on the paper within
  −1.1..+0.8 mm. The cached side lens is f = 1148.01 with rms 0.90 px over 210 crossings
  (`evidence/tip/side_pose_cache.json`); the overhead is f = 1608.47, 1.81 px over 1332 crossings, marked
  "Not yet checked on the tape" (`evidence/tip/ugreen4k_lens.json`).
- Crossings are the acceptance criterion for lighting: `LAMP_CROSSINGS = 600` ("ruling crossings the overhead must
  still see under a setting", `tools/arm_calibrate.py:474`) and `auto_lamps` picks the first lamp setting that finds
  the inked tip in 5 snaps with the rays within 1 mm (`:479-484`, with the note "2026-10-30: both lamps had been
  switched off and the tip was lost at 40 mm"). At dusk the UGREEN paper threshold fell 180 → 65 and one lamp went
  dark (`docs/history/rca_touch_dot.md` §6) — but nothing in the evidence ties a lamp setting to a crossing count
  for a session that also has an accuracy number attached to it.

## 9. Geometry that has never been checked

These are not errors yet; they are mm-scale unknowns with no measurement attached.

1. **Camera placement in the model.** `tools/hwr_kinematics.py:82` uses `camera_along = 0.069`, `camera_up = 0.042`.
   The URDF puts `camera_link` at link4 offset `(-0.013858, -0.019126, 0.051830)` m (from
   `robot/hwr.urdf:170-182`). The model's camera is ~17 mm further along the arm than the URDF's, and the URDF's
   lateral offsets are not represented at all. The tip sits ~180-200 mm from the camera, so **0.5° of camera-pose
   error is 1.6 mm of tip error** and 1° is 3.5 mm. The tip estimate is carried by the camera pose at every move.
2. **Pulse↔URDF signs.** `robot/hwr_driver_node.py:191` publishes `(pulse − 500) × 0.24°` for every joint including
   2, 3 and 4, and `robot/servo_bridge.py` passes commanded pulses straight through (`:72-77`) and converts
   read-back the same way (`:50`) — so commands and read-back live in the same, consistent pulse domain. The flip
   for joints 2-4 exists **only inside the kinematics** (`tools/hwr_kinematics.py:23-26`). Nothing verifies that the
   flip is right: `docs/ARCHITECTURE.md` §5 lists "no pulse↔URDF sign check" as a standing gap. If the flip is
   wrong, shoulder and elbow predictions are mirror-imaged and the corrected-pulse path moves the arm the wrong
   way. One photo of a known joint-2 pulse against `fk()` settles it.
3. **The roll sign is a guess.** `tools/hwr_kinematics.py:50` states "shortcut: joint5 = +roll sign not yet checked
   on hardware; verify with one wrist-roll photo". A wrong sign makes roll corrections diverge, and roll is the
   repo's only fine axis (0.26-0.42 mm/pulse).
4. **The needle does not sit on the roll axis.** From the fitted tips in `arm_model.json`: at roll 502 the tip is
   179.5 mm from the arm origin, at roll 1000 it is 197.3 mm, and those two points are 101.5 mm apart. A rigid
   rotation about joint5 would keep the radius fixed; the observed geometry implies the tip sits **~59 mm off the
   roll axis** *and* the two fits disagree about the tip by 18 mm. Both cannot be true in one session. At
   `tool0 → roll_joint = 55.77 mm` (`robot/hwr.urdf:145-147`) plus a ~144 mm needle, a 59 mm lateral offset is a
   bent needle, a crooked clamp, or a bad fit — it has not been separated.
5. **The gripper's hold is not characterised.** The RCA recorded 202.5 → 206 mm of tip depth across a gripper
   reopen and 188 mm after a re-mount, and `fit_arm`'s docstring records "with elbow 0-40 and wrist ±25 the elbow
   offset traded off against the tip". `tip_cam_mm` is explicitly "record only: per grasp"
   (`tools/arm_calibrate.py:849`). Nothing measures grasp repeatability with a fixed command.
6. **`hwr_kinematics.py` is unimported dead code.** `robot/README.md` documents it as "the offline FK helper" but
   no tool imports it. The live kinematics are `tip_triangulate.ArmKinematics`. Two implementations of the same arm
   can drift; only one is tested by use.

## 10. Sequence defects that manufacture error

Independent of any physical error budget, the current sequences create error deliberately:

1. **The last centimetre is open loop.** Below 2 mm the code goes "blind" (`blind_ok=h <= 2.0`, `:1221`), a blind
   step is recorded with error 0.0 (`:1246`), and the run ends when `blind >= 3`. 35 of 77 committed touch steps
   have no tip measurement at all.
2. **The target is carried through the model, not re-measured.** `evidence/descend/20261029T115803/log.json` has 36
   `retie` phases, every one `matched=0` — the goal was propagated by FK for the entire 524 s run. 164912 has 39
   with 34 carry fallbacks; 103024 has 30 with 13. `coarse_aim.py:97` documents why it matters: "right after the
   first move, it read about 23 mm across all runs".
3. **The 4K fallback never fires when it is needed.** In the two runs where a `place` cycle failed, the recorded
   `ugreen_fix` phase reports `matched_in` 4/3 and 2/0 — the last-resort 4K UGREEN measurement found the crossing
   in 0 of 3 attempts.
4. **Corrections do not converge.** `evidence/tip/arm_hover_20261030T235134.json`: at +15 mm errors 2.2, 0.67,
   0.85 mm; at +10 mm 2.26, 1.67, 0.74 mm; touch steps +6/+3/+2 give 2.49→2.12→2.17, 1.56→2.31→1.48, 2.0→1.7→1.68.
   The correction sometimes makes it worse, then the descent proceeds anyway.
5. **Stale state is reused.** `hover_memory.json` reuses a previous session's corrections and start poses
   (`evidence/tip/arm_hover_20261030T213200.json` carries `corr_mm` forward into a new session's first attempt at
   each height). `side_pose_cache.json` and `printed_crossings.json` are never re-fit per session. Wet ink lasts
   minutes; cached corrections last days.
6. **The abort comes after the damage.** `evidence/tip/arm_hover_20261030T232627.json` records a +15 mm step with a
   **98.45 mm** xy error (tip at z = −30.2 mm, i.e. under the paper) before stopping. Rows are appended before the
   sanity check.

## 11. Contradictions in the record

Worth knowing before quoting any number from `docs/history/`:

| Claim | Where | Problem |
|---|---|---|
| "one pan pulse is ~0.8 mm" | `tools/arm_calibrate.py` dot path | vs 1.114 / 2.2 / 2.95 mm elsewhere |
| "1.8 mm median / 4.0 mm p90" | `fit_arm` docstring, `docs/STATUS.md` | held-out on one run, in-sample figures quoted as accuracy elsewhere |
| "gate 0 verdict GO" | `docs/history/plan_2026-10-30_arm_calibration.md` | the GO rests on pan's *best* side (0.28 mm) while its worst side is 2.61 mm; the same test earlier that day said stop |
| "coarse_aim.py and model_jab.py take --calib" | `docs/history/plan_2026-10-30_arm_calibration.md:137` | true only on the separate `claude/mlp-servo-tip-mapping` branch; `model_jab.py` has no such flag at HEAD |
| "1635 `*_arm.png`, 107 runs / 14 contacts" | `docs/handover.md`, `docs/history/rca_touch_dot.md` | unauditable (session scratchpad) and inflated by the 1.6 GB `.claude/worktrees/` copies |
| "5 mm short and dragged" | `docs/history/rca_touch_dot.md` | the strongest model-free datum in the repo, and the only one that was never quantified |
| 1 mm vs 2 mm vs "1 mm, 2 in a row" | `docs/history/needle_on_dot.md`, `plan_2026-10-28.md`, `plan_speed_accuracy_2026-10-30_AGREED.md` | three different acceptance definitions are still in the tree |

## 12. What to measure next

Ordered by millimetres closed per hour of arm time. Each is a measurement, not a fix.

1. **Log `lam` every run** (zero arm time). `dot` already computes `lam_ugreen_mm`; write it into the session
   header and refuse to descend if it disagrees with `--lam` by more than ~5 mm. Closes channel 6, costs nothing.
2. **Measure vision accuracy against a reference that was not fitted** (10 min, no arm motion). Tape a known
   distance, or use the ruling itself, and compare against `tip_sheet_mm` from both cameras. Until this is done, no
   sub-2 mm claim in this repo means what it appears to mean. Closes channel 3.
3. **One hardware check on pulse↔URDF signs** (5 min, one joint, one photo). Closes channel 8.2 and 8.3 and is a
   prerequisite for trusting every Jacobian in the repo.
4. **Repeat the gate-0 test 20 times in one session** (≈15 min). One session's verdict is a coin flip on which side
   the joint started. This is the difference between "calibrate" and "stop" in the repo's own output.
5. **Grasp repeatability**: clamp, measure `lam`, reopen the gripper to the same pulse, measure again, ×5. This is
   the number that decides whether 1 mm marks are reachable at all.
6. **Contact-sweep A/B**: flip `DOT_STOP_AT_CONTACT` at 40 mm above (the agreed plan's step 1, never run) and
   compare the ink offset in `dot_offsets.json` with the stop on and off. The current ladder drives 3 mm past the
   plane on every attempt.
7. **Per-session lamp/crossing record**: crossings counted at the accepted lamp setting, per session. Nobody has
   correlated the documented dusk collapse (threshold 180 → 65) with an accuracy number.

## Reproducing the numbers in this file

```bash
# static vision scatter, side gaps, mm/pulse, creep
python - <<'PY'
import json
for s in ("20261030T141229", "20261030T135951"):
    d = json.load(open(f"evidence/tip/arm_repeat_{s}.report.json", encoding="utf-8"))
    print(s, d["static_mm"], {k: (v["gap_mm"], v["scatter_mm"]) for k, v in d.items()
                              if isinstance(v, dict)})
PY

# ray-miss medians in the calibration data (the vision-accuracy question)
python - <<'PY'
import json, statistics as st
from pathlib import Path
for p in Path("evidence/tip").glob("arm_calibration_data_*.json"):
    m = [r["miss_mm"] for r in json.load(open(p, encoding="utf-8")) if isinstance(r, dict) and "miss_mm" in r]
    if m: print(p.name, len(m), round(st.median(m), 2), round(sorted(m)[-max(1, len(m)//10)], 2), round(max(m), 2))
PY

# read-back shortfall by approach direction
python - <<'PY'
import json, statistics as st
rows = json.load(open("evidence/tip/arm_repeat_20261030T141229.json", encoding="utf-8"))
by = {}
for r in rows:
    if isinstance(r, dict) and r.get("read"):
        for j in r["read"]:
            d = r["read"][j] - r["cmd"][j]
            if d: by.setdefault((int(j), r.get("side")), []).append(d)
for k in sorted(by): print(k, "median", st.median(by[k]), "n", len(by[k]))
PY

# the A/B that shows calibration working
python - <<'PY'
import json, statistics as st
rows = json.load(open("evidence/tip/arm_aim_20261030T212403.json", encoding="utf-8"))
for m in ("calibrated", "nominal"):
    v = [row["miss_mm"] for row in rows if row["model"] == m]
    print(m, "median", round(st.median(v), 2), "max", round(max(v), 2))
PY

# where the ink actually lands
python -c "import json; print(json.load(open('evidence/tip/dot_offsets.json', encoding='utf-8')))"
```

`tools/regress.py` and `tools/replay_run.py` replay the descend runs offline; nothing in this file required the arm
to be powered.
