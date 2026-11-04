# AGREED plan: speed and accuracy of the dot touch — 2026-10-30

Merged from `plan_speed_accuracy_2026-10-30_meta.md` and `plan_speed_accuracy_2026-10-30_OPUS.md`
(review: `plan_review_2026-10-30_meta.md`). User approved starting this plan 2026-10-30.
Both source plans agreed on the baseline, the targets, and most fixes; differences were sequencing
and are settled here. **One writer on `tools/model_jab.py` at a time**; nothing is committed until
the user says so; every code step is gated by `tools/regress.py`.

**Targets:** touch **≤ 120 s** start pose → lift; ink mark **≤ 1.5 mm**, then the gate
(**1 mm, 2 in a row, 4 of 5**). Accuracy is measured on the ink mark (overhead, after lift), split
(along, across) — in-run estimates on Touch 1 said 0.8/0.1 mm against a real 3.6 mm mark.

## Baseline (run `evidence/descend/20261030T103024`, upper dot, 223 s, mark 3.6 mm)

| Segment | s | Cause |
|---|---|---|
| Startup → first move | ~47 | `pose0` snap 15.5 s (arm camera stall); failed side-rig solve ~14 s |
| Placing rounds 1+2 | 117 | 29 reties × ~4.2 s (settle 1.6 + SAM 1.75 + snap 0.5 + grid 0.2). Round 2: 20 iterations; along flipped −12.6/+9.9 mm after 1–2 pulse moves; 9/30 reties on 0 dots (carry), 4 on 1 dot — **the flips are measurement, not motion** |
| Practice arcs | ~25 | extrapolated from 3.9 mm up; the two arcs disagreed by 9.3 mm; ~2.0 mm of the mark error |
| Descent + contact | 25–27 | 11 side looks × 2.1 s, no triangulation (side pose failed: needs 5 dots, sheet has 4) |
| Mark vs tip end | ~1.5 mm | ink film ends ~8 px above the point; prints from the needle's side |

Already implemented 2026-10-30 (uncommitted, both agents' work): `side_teal` guards
(`SIDE_TRACK_PX=30`, `SIDE_WINDOW_PX=90`), `place-skip` (`SKIP_TOL_MM=1.0`), `teal_tip` `on_paper` +
`near=`/`TIP_JUMP_PX=60`, `CONTACT_EXTRA=6`.

## Steps

| # | Do | Who | Pass gate |
|---|---|---|---|
| 1 | **Regress hygiene.** `tools/replay_run.py` loads the run's own day's `ugreen_dots.json` (saved into each run folder at tie time; parent file as fallback). Add `20261030T103024` to `tools/regress.py` RUNS. Re-baseline (`--save`) | meta | 5/5: divergence at-or-after baseline, no crash-type errors |
| 2 | **Diagnostics (offline).** D1: replay round 2 of 103024 from saved arm frames — per frame grid pose, SAM dots, tie pairing, tip/target/along; decide grid-slip vs tie-alias (record which prompt found the dots). D2: `tools/mark_bias.py` — every historical mark as (along, across), mean/std. D3: overhead ray tilt at the dots, mm/mm. D4: `pose0` stall cause vs 2026-10-29 runs | meta (parallelisable) | four outputs written into this doc's results section |
| 3 | **S1a + S2.** Reject impossible re-ties: after a ≤3-pulse/joint move, a measured along/height change > 4 mm beyond model prediction → re-snap, median of up to 3 measures; a 0-dot carry never drives a > 2 mm move; `place-skip` consumes the median. Settle sized by jump (`settle` ms 400/800/1500; keep 1500 when servo 2 rises) | one writer | replay 103024: iteration count drops materially; regress 5/5 |
| 4 | **5th dot on the sheet** | user | `tools/side_aim.py` reports ≥ 5 dots agree |
| 5 | **Live touch 2** — centre dot (1173, 613): `arm_to_start.py`, thin re-ink (last 2–3 mm), `model_jab.py --jab --dot 1173 613 --along -3.9`; measure the mark with D2's tool | user + driver | time, (along, across) mark, contact rule fired → handover table |
| 6 | **A1 (+ A2).** Practice arc `PRACTICE_H` 4.0 → 2.0, teal tip logged every step, quadratic along(h) read at h = 0 (straight-line fallback < 3 samples; degrade to arm height when tri-height unavailable). If D2 std < 0.7 mm: `MARK_BIAS_MM` applied at the arc landing, refit after every touch | one writer | regress 5/5; live practice prediction vs tip end < 0.7 mm |
| 7 | **Conditional speed:** S1b (overhead places along/sideways for rnd ≥ 1, arm gives height) only if D3 ≤ ~0.3 mm/mm *and* the ≤ 120 s budget still needs it after steps 3–6; S4 descent steps 6/3/1; S5 per D4 | one writer | regress 5/5 |
| 8 | **Live touches 3–5** (right dot 1261, 621; red dots), fresh dot each touch, thin ink, mark measured every time | user + driver | ≤ 120 s and ≤ 1.5 mm each; then the 1 mm gate |

Explicitly deferred (revisit only if ≤ 90 s is ever required): model re-solve replacement / learned
Jacobians, persistent daemon (plan_speedup R3/R6). Retired if step 6 lands: `--along` as the default
aim mechanism (stays as a manual override).

## Rules carried over

Never command from readback (servo 2 settles ~11–16 pulses short rising). Close browser tabs holding
the UGREENs before a run. `tools/arm_park.py` after the session. Never run
`highwonder_rework/deploy/sync.ps1`. Venv:
`D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe`. Fresh dot each touch;
no touches near old ink. `evidence/regress_out/` is scratch and may be deleted.

## Results log

_(filled in as steps land: step, date, gate evidence)_

- Step 1 (2026-10-30): DONE. Per-run `ugreen_dots.json` seeded into the 5 run folders from each run's `frames` row; `tools/replay_run.py` prefers the run's own file; `tools/model_jab.py` tie writes the validated dots into its run folder; `tie-inputs`/`tip-cam` (pure diagnostics the 10-29 recordings predate) added to the compare skip set; `20261030T103024` added to `tools/regress.py`. **Regress 5/5 OK** against the re-saved `evidence/descend/regress_baseline.json` (170928 row 6 own-abort `tip -0.1 mm up` — fake-arm artifact; 172244 row 4: replay needs no reach-strafe from the recorded pose — open-loop artifact; 152609 row 0 = old baseline; 171326 row 5 reach-tied early; 103024 row 20, where the new `place-skip` first fires). Control: pristine-HEAD code aborts the same replays at the tie (`SAM found 2 black dots`), so today's changes strictly improve replay fidelity; no crash-type errors anywhere.
- Step 2 (2026-10-30, Opus, took over from meta):
  - **D1 — tie aliases, not grid slips.** `tools/d1_retie_forensics.py` on 103024's 64 arm frames: today's 4 dots sit 10.6–15.4 mm apart (UG inter-dot vectors); the winning and runner-up dot pairings differ by exactly those spacings (d_off 10.6 / 15.4 / 19.8 / 30 mm), mostly with 1 dot matched (cube tolerance 25 mm cannot separate them). The grid numbering also slips 5–75 mm between frames, but the re-tie compensates when the pairing is right. The ±10–15 mm along flips are the pairing switching. (The script's frame↔iteration mapping is off after frame 14; the alias pattern holds across all frames.)
  - **D2 — 2 clean marks, same direction.** Teal diff (after-lift vs hover overhead frame) near the dot: 120925 mark (1329.8, 616.6) vs dot (1331.0, 623.1) = 6.6 px ≈ 1.5 mm; 103024 (1172.7, 555.1) vs (1177.9, 569.5) = 15.3 px ≈ 3.6 mm. Both towards the robot (−y in the image), both after a practice arc that predicted ≤ 0.3 mm. 125405 shows no new teal (on old ink), 170928 dry tip, 102003 no mark. Mean ~2.5 mm, spread ~2 mm: **A2 not supported yet** (rule: std < 0.7 mm); the bias direction is consistent.
  - **D3 — overhead is ~45° oblique at the dots: 0.97–1.04 mm horizontal per mm of height** (camera 195 mm up; `tt.ugreen_rig` on 103024's hover frame, rays at all 4 dots). **S1b dropped** (gate 0.3). Consequence for accuracy: the hover check and the practice-arc low point put the overhead tip ray at the *arm camera's* height; a 1 mm height error there is a 1 mm along error. A candidate cause of the practice arc's short bias; A1 should read the low point at the side-camera triangulated height when available.
  - **D4 — `pose0` stall is intermittent.** Snap at `pose0`: 0.45–1.1 s normally, 14.7–16.1 s in 3 of 25 runs (202603, 202811, 103024): an arm-camera freeze and restart. The other ~16 s before `pose0` is model loading (the deferred daemon). S5: low value; the preflight warm-up covers most of it.
- Step 3 (2026-10-30, Opus, in the worktree `claude/upper-dot-touch-handover-6a04db`, not live, not committed):
  - S1a: `retie(..., small=True)` after a placing move of ≤ 3 pulses: a tie whose target lies > `TIE_JUMP_MM` 5.0 from the carry is rejected (`retie-rejected`), the carry is used and does not update `base_target`.
  - Deadband: `MIN_STEP` 2: a placing correction (it > 0) whose biggest jump is 1 pulse is raised to 2 pulses on the biggest joint, the same direction (`place-min-step`).
  - S2: placing move time by size: 400 ms (≤ 3 pulses), 800 (≤ 15), 1500 (larger, or servo 2 rising); settle cap = move + 0.8 s.
  - `tools/regress.py` calls `replay_run.py` next to itself (a worktree's code can replay the main checkout's recordings).
  - Regress **5/5 OK**, same divergence rows as the baseline (6, 4, 0, 5, 20). Replays are open-loop: `place-min-step` fired once (103024), `retie-rejected` never fired, so S1a is untested until a live run; the time saving cannot be shown by replay.
- Step 5 (2026-10-30 11:58, Opus): **live touch 2**, centre dot (1173, 613), run `evidence/descend/20261030T115758`, commit d419c79 code, `--along -3.9`, side pose failed (4 dots), tip not re-inked after touch 1.
  - **Time 133 s** (touch 1: 223 s): `pose0` 17 s, tie to first move 18 s, placing 19 iterations 58 s (~3 s each; 2 `place-min-step`, 0 `retie-rejected`), hover check, no practice arc (`--along`), descent + contact 35 s.
  - **Accuracy: the tip came down ~9 mm short of the dot** (overhead at contact: tip end (1164.5, 574) vs dot (1172.5, 613.1), 39 px towards the robot, beside the upper dot); arm camera at contact: 5.0 mm short of the shifted target. **No ink mark** (dry tip). Contact by `side-still`, arm height −0.94 mm.
  - **`--along -3.9` was wrong (Opus's derivation).** It assumed the uncorrected arc lands at the same error on every dot; here the arc landed ~5 mm short with no correction, where touch 1's numbers implied ~4 mm past. The arc's along error changes by ~9 mm between neighbouring dots (cf. 2026-10-28: −5/+7 mm). A constant `--along` cannot work; the along landing has to be measured each touch (practice arc, A1). `--along` should not be used for touches.
- Step 6 (2026-10-30, Opus): **A1** in `model_jab.py`: `PRACTICE_H` 4.0 → 2.0; the overhead teal tip is sampled at every practice step (`teal_tip(near=)`), and `arc_landing()` fits height and sheet xy as quadratics in the wrist pulse (the arc is a circle about the wrist: smooth in angle, not in height) and reads xy where height reaches 0; falls back to the old straight line with < 3 samples, no zero crossing, or a > 3 mm disagreement. Both are logged (`practice-samples`: `landing_mm`, `straight_mm`). Self-check on a synthetic 45 mm arc: noise-free fit 0.02 mm vs line 1.1 mm; with 0.3 mm sample noise both ~1.2 mm RMS (fit: bias 0.1, sd 1.3; line: bias 1.1, sd 0.6), so most of the gain is the lower stop. Regress 5/5 (the practice arc is not reached by any replay: first live use is touch 3). **A2 not applied** (D2: 2 marks).
- Step 8a (2026-10-30 12:15, Opus): **live touch 3**, right dot (1260.2, 621.4), run `evidence/descend/20261030T121508`, commit c592597, re-inked, no `--along`. **110 s.**
  - **Mark: an 8 mm streak, 5.5–13 mm short of the dot** (away from the robot, +y in the overhead image; centre 9.2 mm). The overhead tip end stopped at (1263, 677), 13 mm short, at wrist 235 (side y froze there too); the arm camera still read 2.4 mm, and 4 more steps (to 227) pressed the needle on: the streak.
  - **The practice arc never ran.** It ran only with the hover sideways miss ≤ `UG_TOL_MM` 1.0; the miss read 1.29–1.44 mm, which pan/roll do not act on (`SIDE_TOL_MM` 1.2), so 3 rounds changed nothing, `rounds-stopped` fired, and the touch used the uncorrected model arc. Touches 2 and 3 show that arc landing 9–13 mm short on today's dots.
  - Fixes (commit after this entry): `PRACTICE_GATE_MM` 2.0 for the practice arc; `PRACTICE_H` 3.0 (2.0 would touch with this arm-height bias) and the practice stops as `practice-touched` when the overhead tip stays within `STILL_UG_PX` 2 px (the landing measured); descent stops on `wrist-still`: the side tip still (< 1.5 px) on 2 steps running (overhead only without a side reading), no extra pulses. On touch 3 that stops at wrist 232 (one step after contact) instead of 227. Regress 5/5; the replay of 121508 diverges at its reach strafe, so the new paths are untested offline.
