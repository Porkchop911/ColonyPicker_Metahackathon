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
