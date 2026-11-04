# Plan: speed and accuracy of the dot touch (2026-10-30)

Basis: run `evidence/descend/20261030T103024` (upper dot, 223 s, ink mark 3.6 mm off) and the 20 touch runs of
2026-10-28/29 (100-412 s). Line numbers: `tools/model_jab.py` in the main checkout as of 10:45 (uncommitted edits
by the `.pi` agent included: side tracker guard, `place-skip`, `CONTACT_EXTRA` 6).

**Targets:** touch in **<= 120 s** from start pose to lift; ink mark **<= 1.5 mm**, then the gate (1 mm, 2 in a row, 4 of 5).

**One writer on `model_jab.py`.** The `.pi` agent is editing it. Every step below goes in one at a time, after
`tools/regress.py` passes (it is 0/4 right now: the replays need their day's `ugreen_dots.json`).

## 1. Where the time goes (run 103024)

| Phase | s | Per unit | Cause |
|---|---|---|---|
| Start pose (`pose0`) | 32 | snap 15.5 s | arm camera froze; `snap_arm` waited out stalls |
| First tie | 15 | | side pose failed (0 dots agree), SAM 4 s |
| Placing round 1 | 36 | 8 x 4.3 s | |
| Hover check + practice arc | 13 | | |
| **Placing round 2** | **81** | **20 x 4.1 s** | along error flipped -12.6 / +9.9 / -6.8 mm under 1-2 pulse moves |
| Descent + contact | 25 | 11 x 2.1 s | 3-pulse steps, arm view each step (no triangulation) |
| Contact view, lift | 5 | | |

Per placing iteration: settle 1.6 s (`settle(..., 1500, 2.3)` at l. 986: 1.5 s minimum even for a 1-pulse move),
snap 0.5, grid 0.2, **SAM 1.75 s** (retie, l. 691).

**The flips are measurement, not motion.** Round 2, it 8 -> 9: command +1/+1 pulse, along 2.3 -> -12.6 mm. It 11 -> 12
(both ties on 2 dots): -1.5 -> +9.9 mm. Of 28 iterations, 9 had a tie on 0 dots (carry). Real placing needed ~6.

## 2. Where the error goes

| Part | mm | Evidence |
|---|---|---|
| Practice-arc extrapolation | ~2.0 along | arc stopped 3.9 mm up, extrapolated straight; overhead tip end at contact 2.0 mm short of it. Arc 1 of the same run said 9.2 mm |
| Mark vs tip end | ~1.5 along | mark centre 1.5 mm further toward the robot than the overhead tip end |
| Sideways | 0.9 | within roll's reach; not the problem |

Ink marks so far: 1.0 (10-28 dot 1), 2.35 (10-28 dot 2), 1.49 (10-29 120925), ~1.7 (10-29 125405), 3.6 (10-30).
Direction is not logged for most: step D2 measures it.

## 3. Diagnostics first (offline, no robot)

| # | What | Output | Decides |
|---|---|---|---|
| D1 | Replay round 2 of 103024 from its saved arm frames (`NNN_arm.png`): per frame the grid pose (rot, t), the arm dots SAM finds, the tie's dot pairing, tip in sheet mm | table: frame, matched, pairing, tip, target, along | whether the flips are **grid slips** (rot/t jumps by a 5 mm square, carry rows) or **tie aliases** (2-dot pairing swaps). Picks S1a's rule |
| D2 | `tools/mark_bias.py` (new, small): for each touch run with before/after overhead frames, mark centre (saturation diff, as the judge), dot, needle axis from the hover frame -> mark - dot as (along, across) | table over all runs with marks + mean/std along | whether a constant along bias exists (A2) |
| D3 | Overhead camera's ray tilt at the dots (`ugreen4k_lens.json` + rig): mm of horizontal shift per mm of height error | one number | whether the overhead can place along without the arm tie (S1b) |
| D4 | Why `pose0` snap took 15.5 s: `snap_arm` freeze log, compare with 10-29 runs | cause | S5 |

## 4. Speed

| # | Change | Where | Saves | Risk / check |
|---|---|---|---|---|
| S1a | **Reject impossible re-ties.** After a move of <= 3 pulses per joint, a measured along/height change > 4 mm beyond the model's prediction is a bad measure: snap again (no move), take the median of up to 3 measures. A 0-dot carry never drives a move > 2 mm along | `retie` l. 680 / place loop l. 941-998 | ~10 iterations x 4 s in 103024 | D1 first. Replay 103024 + regress: iteration count, no new crash |
| S1b | **Final placing by the overhead.** After the first hover check, along and sideways come from the UGREEN teal tip (ray at the arm camera's height, as the hover check does, l. 1008-1018); the arm camera gives height only (grid pose, no SAM). Factor the hover-check measure into one function, reuse it | place loop for `rnd >= 1` | ~2.8 s per iteration (0.5 s UGREEN vs 1.75 s SAM + tie), and no tie flips | needs D3 (parallax under ~0.3 mm/mm). Teal tip on paper only (guard exists). Fallback: arm re-tie if the teal tip is lost |
| S2 | **Settle by move size.** `settle(..., 1500, 2.3)` -> `ms = 400` if every jump <= 3 pulses, 800 if <= 15, else 1500. Readback still decides (2-pulse rule) | l. 986 | ~1 s x 20 | servo 2 rises short: keep 1500 when servo 2 moves up |
| S3 | **`place-skip` live** (`.pi` agent, done offline): stop placing when along/height are within 1 mm and only sideways is out | l. 952 | 14 iterations in 103024 by its count | regress |
| S4 | **Descent in bigger steps high up:** `WRIST_STEP` 6 while height > 5 mm, then 3, then 1-pulse contact steps | l. 1128 | ~6 s | 6 pulses = ~2 mm height: still 2+ readings above 1 mm |
| S5 | **Warm arm camera before `pose0`:** preflight leaves it streaming; `model_jab` restarts it once if the first frame takes > 3 s, rather than waiting out stalls | start, `snap_arm` | ~13 s | D4 |
| S6 | **5th dot on the sheet** (side pose needs 5 dots): triangulated height replaces the arm view each descent step (exists, l. 1133) and gives the tri-height stop | sheet, by hand | ~0.7 s per step, plus A4 | `tools/side_aim.py` must report >= 5 dots agree |

Expected, all in: placing round 2 ~6 x 1.5 s, round 1 ~6 x 2.5 s, descent ~15 s, start ~20 s -> **~100-120 s**.

## 5. Accuracy

| # | Change | Where | Fixes | Check |
|---|---|---|---|---|
| A1 | **Practice arc lower and curved.** `PRACTICE_H` 4.0 -> 2.0; log the UGREEN teal tip at every practice step (not only the last); fit along(h) with a quadratic through the samples, read it at h = 0. Fall back to the straight line with < 3 samples | l. 64, 1038-1072 | the ~2 mm extrapolation error (half the height left, curvature fitted) | arm-camera height read 0 in the air before: stop also on side tri-height <= 1.5 mm when available. Live: practice prediction vs overhead tip end at contact, < 0.7 mm |
| A2 | **Mark bias.** If D2 shows a consistent along offset (std < 0.7 mm), add `MARK_BIAS_MM` along to the practice-arc landing (not via `--along`, which skips the practice arc). Refit after every touch | after l. 1067 | the ~1.5 mm mark vs tip end | D2 table; the next 3 touches |
| A3 | **Ink only the last 2-3 mm**, blot once | procedure | the needle's side printing instead of its point (likely cause of A2's offset) | mark round and small |
| A4 | **Contact by tri-height** (needs S6), else side-still on the guarded tracker (`.pi` agent's `SIDE_TRACK_PX`) | l. 1141 | stop at the first contact: no drag (0.76 mm per extra pulse) | `contact by=tri-height` in the log |
| A5 | **Fresh dot each touch**; no touches near old ink (SAM and `teal_tip` both take old ink) | procedure | | |

## 6. Order of work

| Step | Do | Pass |
|---|---|---|
| 1 | Fix `regress.py` replays to load their own day's `ugreen_dots.json` (it fails 0/4 on that alone) | 4/4 on today's code |
| 2 | D1, D2, D3, D4 (offline) | the four tables above |
| 3 | S2 + S1a (+ S3 already in) | replay 103024: same target, fewer iterations; regress 4/4 |
| 4 | Add the 5th dot (S6) | `side_aim.py` >= 5 dots |
| 5 | A1 (+ A2 if D2 supports it) | regress 4/4 |
| 6 | **Live touch 2** (centre dot 1173, 613), re-inked thin | time, mark, practice prediction vs tip end, logged |
| 7 | S1b if D3 passed, S4, S5 | regress |
| 8 | **Live touches 3-5** (right dot 1261, 621 and red dots) | <= 120 s, mark <= 1.5 mm; then the 1 mm gate |

Each live touch: write the mark (along, across, mm), time per phase, contact rule fired, into the handover table.
