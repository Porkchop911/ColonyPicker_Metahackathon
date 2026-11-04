# Implementation: needle into the 96-well plate, 2026-10-31 — A RESOUNDING FAILURE

Goal: `docs/goal_plate_well_reach_2026-10-31.md`. Branch `plate-well-reach`.

## Verdict

**0 of 96 wells reached. The arm moved violently, was thrown into a fully raised pose by a bug, and then flailed at
the edge of its reach.** On 2026-10-30 the same arm hovered calmly to 0.13 mm over a printed crossing. Today's motion
behaviour is a regression, and it is Claude's responsibility.

## What went wrong (Claude's doing)

| # | Failure | Effect |
|---|---|---|
| 1 | Drove the arm from scratch scripts with raw `tip_descend.send` and one-shot big moves (~250 pulses of pan, ~200 shoulder in 2.5 s) to take background frames | violent swings ("that was quite violent") |
| 2 | Wrote a speed-limiting wrapper (`arm_calibrate.send`) and ran it on the robot **without an offline test**; its big-pan lift called itself recursively, +30 pulses per level | the arm was thrown up to shoulder 881 / elbow 701 until the bridge rejected a command > 1000 |
| 3 | Chose targets far outside the calibrated region: Claude's own offline reach check said the wells need pan +120..+230 with the arm nearly stretched (shoulder ~250, START 420), and Claude went ahead instead of raising the plate's placement first | there 10 pulses move the tip 20-40 mm and the model is 5-10 mm off: every wiggle and correction is a wide swing ("flailing") |
| 4 | Tip refits with goals spread up to 80 mm apart, then "local" wiggles that were not local at that reach | more wide, pointless motion |
| 5 | Kept trying live after the first violent move instead of stopping to rethink | repeated the damage |

## What was not Claude's doing (but cost time)

- Roll servo 5 takes no position commands since another session's RRC sub-command probe (~10:20); a power cycle may
  restore it (unverified).
- The needle was re-seated by hand (~10:35, near-vertical) and slid in the gripper twice (~23 mm, ~6 mm, and again
  today): every stored tip model went stale.
- The arm calibration (`arm_model.json`) was fitted near START; it does not hold at the far reach the plate needs.

## What exists (code, committed, unproven on the robot)

| Piece | State |
|---|---|
| `plate_rig.py --plate`, `plate_rig.well("B7", below_rim)` | plate pose from the side camera's well lattice, A1 checked on the overhead image: **works** (offline, no arm) |
| Reach model, pitch held (`aim_solve(..., pitch)`) | ~54 of 96 wells in reach with a vertical needle, columns 1 to 6-10; the rest need the base move. Offline only |
| `arm_calibrate.py well --wells ...` | measured placing over the rim, blind 3 mm entry, in-well check, photos, `well_reach.json`. **Never got past the anchor** |
| Bare-needle measurement (`measure.thin`, `needle_ends(min_px)`) | pairs every changed blob in both views; rays met to 0.1-1.7 mm on several poses |
| `arm_calibrate.send` | speed limit (12 ms/pulse), one lift for big pans, ceiling 620, out-of-range refused; dry-run tested offline **after** the incident |
| `hue.py` | the main checkout's key from a worktree |
| Merge of colony detection | done |

## Lessons (now a standing rule in memory: `dry-run-motion-code`)

- No new or edited motion code reaches the robot before an offline dry run that prints and bounds every command.
- No raw one-shot moves; everything through the speed-limited send.
- Check reach geometry first; targets outside the calibrated region need the user's OK.
- Stop at the first sign of wrong motion.

## To actually reach the wells

Move the plate (or the robot, within 40 mm) so the plate sits near the arm's centre line and the arm works near
START, where the calibration holds to ~1 mm. Then: `plate_rig.py --plate` (no motion), a small-move tip refit near
START, and `well` on one well, watched.
