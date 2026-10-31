# Speed + accuracy improvement plan — 2026-10-30

Scope: `tools/model_jab.py` dot touches (overhead-aimed, angled needle). Goal: **≤1 mm mark error at
≤120 s per touch**, measured on the ink mark (overhead pixels after lift), not on in-run estimates.
Baseline is Touch 1, run `evidence/descend/20261030T103024` (`--jab --dot 1176 570`): **223 s, mark
3.6 mm off** (3.5 mm short along the needle, 0.9 mm sideways). In-run estimates said 0.8 / 0.1 mm —
every estimator was optimistic, so all acceptance below is on the measured mark.

## Where the 223 s went (log.json, wait_s totals)

| Segment | s | Waits inside |
|---|---|---|
| Startup → first move | 48 | light→pose0 31.6 serial init; failed side-rig solve 14.5 |
| Place loop 1 (9 iters) | 36 | retie 4.2 s each: sleep 1.6 + SAM 1.75 + snap 0.5 + grid 0.2 |
| Practice-arc gap | 13 | |
| Place loop 2 (20 iters) | 81 | 14 iterations commanded identical 1-pulse nudges (side already 0.08–0.15 mm) |
| Practice-arc gap | 12 | |
| Descent (11 side looks) | 27 | ~2.1 s/step: 3 cameras + SAM per step |
| Contact + touch + lift | 5 | |
| **Total** | **223** | sleep 65.0 · SAM 57.8 · snap 46.4 · grid 11.9 · ugreen 9.7 · side 6.5 |

## Where the 3.6 mm went

1. **~2.0 mm final aim.** Practice arc 2 left the tip 2.0 mm short along; the arc is extrapolated from
   3.9 mm up and is volatile (arc 1, same setup: 9.2 mm past). The UGREEN correction after the arc
   moved along only via the place loop, which then oscillated ±2–10 mm without converging.
2. **~1.5 mm mark vs tip end.** `contact_ugreen_zoom.png`: the teal film ends ~8 px above the point;
   the mark is a clean round blot printed from the inked side of the needle, not its point.
3. **Contact was luck.** `side-still` fired from `side_teal` locked on frame-edge ink (y 1079,
   +69 px/step) while the real tip had left the frame at wrist 181; final wrist 161 < arc landing 163,
   triangulated height −0.34 mm. The overhead `still` rule never fired — `teal_tip` had taken the
   keyboard box's teal line art at (1862, 650).

Accuracy ceiling note: the sideways axis is already good (0.9 mm; `PAN_SIGMA_MM = 4.0` scatter is
absorbed by measuring after every move). The fight is along-axis aim and knowing when contact happened.

---

## Status: done 2026-10-30 (uncommitted, in working tree)

| Change | Effect |
|---|---|
| `side_teal` guards: `SIDE_TRACK_PX=30`, `SIDE_WINDOW_PX=90`, bottom-row reject | False `side-still` contact eliminated; replay-verified on all 13 side frames of the baseline run (tracker defers to SAM during fast descent — 50–83 px/step is real motion) |
| `SKIP_TOL_MM=1.0` place-skip | Kills the 14 no-op iterations: **−55 s** when along/height are placed and only sideways is out |
| `teal_tip` guards: `on_paper` + `near=`/`TIP_JUMP_PX=60` (user, today) | Overhead tip can no longer jump to keyboard-box art mid-arc |
| `CONTACT_EXTRA` 3→6 (user, today) | Backstop wrist travel past paper height; still rules decide |

Projected baseline with only these: **~150–160 s**, contact honest. Not yet re-measured live.

## Phase 1 — finish Touch 1's lessons live (no new code)

Ordered; each is a gate for the next.

1. **5th dot on the sheet.** The side pose needs 5 dots (`side camera: only 0/4 dots agree` twice in
   the baseline run). With a side pose, descent height is triangulated (`TRI_STOP_MM=0.5`) instead of
   inferred from side pixels, and contact stops being luck. *Acceptance: `side-rig` succeeds at tie
   time and descent logs `tri_height_mm` on every step.*
2. **Ink only the last 2–3 mm of the needle.** Removes the ~1.5 mm mark-vs-tip term at the source.
   *Acceptance: post-touch zoom shows the film ending at the point.*
3. **Touches 2/3** — centre (1173, 613), right (1261, 621): `arm_to_start.py`, then
   `model_jab.py --jab --dot X Y --along 3.5`, re-ink between. `--along` cancels the ~2 mm short final
   aim and skips both practice arcs (−25 s). *Accepted risk: the arc measurement that caught round 1's
   9.2 mm miss is skipped; the post-lift mark measurement (step 4) is the check.*
   *Acceptance per touch: mark ≤1.5 mm (target 1.0), time ≤160 s, clean blot (no drag streak).*
4. **Measure every mark from `after_touch_clear_o.jpg`-style overhead frames** (dot centre vs blot
   centroid, split along/sideways using the needle direction). Record in the run folder. This is the
   only accuracy number that counts; if `--along 3.5` overshoots systematically, adjust the constant
   from these two data points, not from in-run estimates.

## Phase 2 — speed: attack the retie (offline code, replay-gated)

The remaining big pool is per-iteration cost: 30 reties × 4.2 s ≈ 126 s of a touch. Two moves, in order:

1. **Condition-based settle instead of fixed sleep** (`sleep` = 65 s total, 1.6 s per retie).
   Poll the arm view until the tip pixel is stable across 2 frames (plan_speedup R2). Expected
   −0.8–1.2 s/iteration → **−25–35 s/touch**. Risk: servo 2's slow settle (reads ~11 pulses short
   rising) — stability must be judged on the tip pixel, not readback. *Gate: replay on
   `20261029T152609` (long placing) diverges no earlier than baseline and raises nothing new.*
2. **Skip SAM in reties after small moves** (plan_speedup S6). When the commanded move is ≤2 pulses,
   track the existing dots by colour/grid within a window instead of a full SAM re-solve
   (SAM = 57.8 s total, 1.75 s per retie). Fall back to SAM if the window finds <2 dots.
   Expected **−20–30 s/touch**. *Gate: same replay; plus a forced-fallback test (window emptied)
   still completes the tie.*
3. **Descent step cost** (27 s / 11 steps): `side_look` already prefers colour (~14 ms) over SAM;
   with the Phase 1 side pose working, drop the per-step UGREEN+arm snaps to every 2nd step
   (side frame + triangulation carries height). Expected **−8–10 s**. *Gate: replay of
   `20261029T170928` (tri-height stop) still stops on `tri-height`, not a backstop.*

Projected after Phase 2: **~90–110 s/touch**.

Do **not** revive: per-move model re-solves replaced by learned Jacobians (R3) or the daemon (R6)
until Phases 1–2 are measured — iteration *count* is now bounded by place-skip; iteration *cost* is
the lever. Revisit only if <90 s is required.

## Phase 3 — accuracy: close the along axis (needs Phase 1 data)

1. **Replace practice-arc extrapolation with a measured landing.** The arc's low point is read at
   3.3–3.9 mm up and extrapolated; Touch 1's two arcs disagreed by 9.3 mm. Instead: after the final
   UGREEN round, descend in the *overhead* view with `teal_tip(near=)` tracking (now guarded) and stop
   the along correction when the tip's overhead pixel crosses the dot's row — the overhead camera sees
   the tip and the dot in one frame, no extrapolation. Keep the arc only as the height donor.
   *Acceptance: 3 consecutive touches, along error ≤1 mm each, no `--along` fudge.*
2. **Contact by triangulation first.** With the side pose live: stop at `tri_height_mm ≤ 0.5`
   (`TRI_STOP_MM`), overhead `still` as confirmation, `CONTACT_EXTRA=6` demoted to backstop and
   reduced once the still rules have fired correctly 3 times running. Removes both failure modes seen
   (stopped in air at extra=3; false side-still).
3. **Along fudge retirement.** `--along` stays available as a manual override, but the default path
   must hit ≤1 mm without it on the centre dot before it is used on new dots. If Phase 3.1 lands,
   delete the practice-arc correction path rather than leaving two aim mechanisms.

## Regress hygiene (do alongside Phase 1, 15 min)

`tools/regress.py` currently reports 1/4 for an environmental reason: `tools/replay_run.py:54` copies
the *latest* `evidence/descend/ugreen_dots.json` (today's tight cluster) into replays of 2026-10-29
runs (yesterday's layout) → all four abort at the tie (`found 1–2 black dots, expected 4`). Fix:
prefer a `ugreen_dots.json` saved inside the run folder, fall back to the parent file; save the current
dots into each new run folder at tie time. Then re-baseline (`--save`) once, with today's
`model_jab.py`, and add `20261030T103024` to `RUNS` — it is the only recording that exercises the
`side_teal` guards and place-skip. *Acceptance: 5/5 OK (divergence at-or-after baseline, no crashes).*

Note: `evidence/regress_out/` from the 2026-10-30 verification runs can be deleted; it is replay
scratch, and `--out` must be an existing folder or every replay crashes before reaching code.

## Order, gates, totals

| Phase | Needs robot? | Expected result | Gate to proceed |
|---|---|---|---|
| 1. Live lessons | Yes (user at robot) | ≤1.5 mm, ~150 s on dots 2/3 | Both marks measured, side-rig OK |
| Regress hygiene | No | 5/5 | — (do before Phase 2) |
| 2. Retie cost | No (replay-gated) | ~90–110 s | Replays clean, then 1 live touch ≤120 s |
| 3. Along axis | Mixed | ≤1 mm without `--along` | 3 consecutive ≤1 mm touches |

Hard constraints carried over: never command from readback (servo 2 settles ~11 pulses short rising);
close browser tabs holding the UGREENs before a run; `tools/arm_park.py` after; do not run
`highwonder_rework/deploy/sync.ps1`; venv is
`D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe`; nothing is committed
until the user says so.

---

## Review (Claude Opus, 2026-10-30 ~11:00)

Checked against `evidence/descend/20261030T103024/log.json` and `tools/model_jab.py` as of 10:45.
Companion plan: `docs/plan_speed_accuracy_2026-10-30_OPUS.md`.

**Right:** time breakdown, 3.6 mm split, regress root cause (`replay_run.py:54` copies the latest
`ugreen_dots.json`), 5th dot first, phased gates.

### Serious

| # | Claim | Problem | Fix |
|---|---|---|---|
| 1 | Place-skip: −55 s, kills 14 no-op iterations | Would not have fired here. For `it > 0`, `aim == want`, so the skip's along equals the logged `along_mm`: round 2 sat at **2.1–2.3 mm** (it 5–8, 17–19), above `SKIP_TOL_MM` 1.0 and the loop exit 1.5. Those iterations were not sideways-only: they commanded 1-pulse elbow/wrist nudges (GAIN 0.35) inside the servo deadband, so nothing moved | Minimum step of 2 pulses, or the along tolerance set to what one pulse moves. Replay 103024 and count iterations before claiming the saving |
| 2 | Touches 2/3 with `--along 3.5` cancel the ~2 mm short aim | `--along` **replaces** `ug_fix` (l. 754: `ug_fix[0] = args.along * fwd`), it does not add to it. This run's `ugreen-fix` round 0 was already ~7.4 mm along toward the robot (`fix_mm [0.28, 7.37]`; along ≈ −err_y from the place rows) and the mark still landed 3.5 mm short. `--along 3.5` aims 3.5 mm forward instead: on the same dot ~**7 mm past** | Do not run live. Either the implied value (about −3.9 mm, sign to be confirmed by replay) or keep the practice arc and add a measured mark bias after it |

### Weaker

| # | Claim | Problem |
|---|---|---|
| 3 | 2.1: tip-pixel polling instead of fixed sleep | The wait is already readback-settled (`settle()`); the 1.6 s is its fixed 1500 ms floor (l. 986) even for 1-pulse moves. Polling the camera costs 0.5 s/frame. Cheaper: floor by move size (400 ms for <= 3 pulses), keep 1500 when servo 2 rises |
| 4 | The ±10 mm along jumps are not addressed | Round 2 along: 2.3 → −12.6 (it 8 → 9, +1/+1 pulse, 0-dot carry), −1.5 → +9.9 (it 11 → 12, both ties on 2 dots), −6.8, +6.6. Biggest cost and accuracy risk. 2.2 (skip SAM) does not remove them and leans on the carry, where the 0-dot jumps came from. Diagnose first, offline, from the saved arm frames: grid slip vs tie alias |
| 5 | 3.1: correct along during the overhead-watched descent | The wrist arc cannot correct along on the way down; that needs elbow moves near the paper (open-loop low poses jammed the needle). Parallax hides the tip-on-dot until contact. Safer: practice arc to 2 mm with a curve fitted through every step's overhead tip, read at h = 0 |

Minor: the 31.6 s start is not serial init: `pose0` logged a 15.5 s arm-camera snap. Descent is not
"SAM per step": `side_look` uses colour and falls back to SAM only when it loses the tip.
