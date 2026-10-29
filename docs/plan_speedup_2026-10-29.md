# Plan: make touch runs battery-cheap — compute on the desktop, not on the clock

Date: 2026-10-29 (completed revision; supersedes the truncated first draft of the same date).
Scope: `tools/model_jab.py` (the live touch script, ~720 lines) and its full stack:
`tools/rig_io.py`, `tools/sam_detect.py`, `tools/tip_triangulate.py`, `tools/tip_motion.py`,
`tools/tip_descend.py`, `tools/paper_grid.py`, `robot/servo_bridge.py`.

Agreed requirements (2026-10-29, two questionnaires): idle RTX 3090 desktop available during
runs; priorities (a) cutting waits, (b) learning parameters from past logs, (c) offline
rehearsal, (d) faster perception; deliverable is this plan doc first; target **60–120 s per
touch**; behaviour change allowed up to *waits + fewer rounds where the repeat is proven
information-free*; no predictive-plunge / closed-loop-descent redesign. No hard constraints
were selected, so robot-side bridge changes are in scope. Safety gates (floor checks,
shoulder-slip abort, UGREEN landing check, tie quality gates) are **not** to be weakened —
they are the reason the slow runs still touched correctly; the plan speeds up how their
inputs are produced, never skips them.

---

## 1. Where the time goes (measured, not guessed)

Reference runs `evidence/descend/20261028T130933/log.json` (313 s, 89 arm views, touched) and
`evidence/descend/20261028T124320/log.json` (138 s, 31 arm views, touched); 68 run logs in
`evidence/descend/*/log.json` total (66 with ≥2 rows; duration median ~92 s, max 313 s;
only 14 `touch` rows vs 46 `abort` rows across all logs — most battery is spent on runs
that never touch).

### 1.1 Segment decomposition (seconds from `log.json` timestamps)

| Segment | 130933 | 124320 | 183012 (291 s, aborted) | 125017 (266 s, touched) | What's in it |
|---|---|---|---|---|---|
| `light` → `start` (start-up) | 60 | 55 | 39 | 46 | lamp test, `sam.load()`, first `snap_arm(3)`, needle SAM, elbow nudge, tie-band loop (up to 16 `view()`s), UGREEN tie (4–5 SAM prompts), cube centres ×4, two `triangulate()` attempts |
| `start` → first `hover` (placing) | 58 | 39 | 98 | 40 | `PLACE_ITERS` (≤30) iterations: model solve + `send(ms=1500)` + `sleep(2.3)` + `view()` (3-frame snap + grid PnP) + `retie()` (2 SAM prompts + cube SAM) + `read()` |
| `hover` → `touch`/`abort` (check + practice + descent) | 193 | 42 | 152 | 178 | per `UG_ROUNDS` (≤6) round: UGREEN check (`ugcam.frame()`), practice arc (≈5 wrist steps × (`send`+`sleep(0.9)`+`view()`) + fly-back + `view()` + `retie()`), then final wrist descent (3-pulse steps, `view()` each) + contact loop (1-pulse steps, `ugcam.frame()` each) |
| `touch` → `lift` | 2 | 2 | 2 | 2 | final `retie`, lift `sleep(2.2)` — negligible, not a target |

Across all 68 logs, time attributed to the phase *preceding* each timestamp gap is dominated
by: `place-plan` 1994 s, `light` 1703 s (start-up, 66 runs ⇒ ~26 s/run average before any
placing), `tip-depth-failed` 1090 s (61 occurrences ⇒ ~18 s per failed attempt), `place`
860 s, `ugreen-check` 324 s, `wrist` 241 s. These six buckets are ~90% of all logged time.

### 1.2 Measured unit costs (desktop RTX 3090, 2026-10-29, on saved frames)

- SAM 3.1: `load()` **8.7 s** (paid unconditionally at `tools/model_jab.py:143`, even for
  `--plan-only`). `detect()` (`tools/sam_detect.py:31-38`: `start_session` → `add_prompt` →
  `close_session` per call, image re-encoded as a one-frame video every time): **0.33–0.75 s**
  per prompt on saved frames (`black dot` 0.75/0.41 s, `small red dot` 0.36/0.39 s,
  `red cube` 0.33 s); identical repeats cost the same 0.34–0.36 s — zero caching benefit today.
- `retie()` = `arm_dots` (2 prompts: `black dot` + `small red dot`, `tools/model_jab.py:255-262`)
  + `cube_centre` (1 prompt, `tools/model_jab.py:264-272`) ≈ 1.1–1.5 s GPU, called after
  **every** move (772 `retie` rows across logs). Observed `place-plan`→`retie` gap:
  **median 4.4 s, p90 4.7 s, n=477** — so ~3 s of each gap is *not* SAM: it is
  `send(ms=1500)` + `sleep(2.3)` (`tools/model_jab.py:568-569` in current numbering: sleep 2.3 s),
  `snap_arm(3)` (3 distinct JPEGs over SSH, `sleep(0.12)` between, `tools/tip_triangulate.py:291-321`),
  grid detection + PnP (`tools/tip_motion.py:45-60` via `tools/paper_grid.py`), and `read()`.
- `view()` (`tools/model_jab.py:210-221`): one `snap_arm(3)` + `tm.arm_view` (grid_points +
  solvePnPRansac + blob extraction); silently retries up to 3 frames on `RuntimeError`
  (`sleep(0.3)` between) — a slow or failing grid pose is invisible in the log (no `note()`
  on retry). 1635 `*_arm.png` files across evidence ⇒ ~24 views per run average.
- `ugcam.frame()` (`tools/rig_io.py:70-80`): mean of 3 frames captured ≥0.3 s after the call,
  blocks up to **5 s** then raises `UGREEN stopped delivering frames`. Every call pays at
  least the 0.3 s settle + frame cadence (~8 calls in `model_jab.py`: lines 279, 284, 310,
  591, 624, 670, 676, 689 in current numbering). No timestamp/freshness is returned, so a
  caller cannot tell a 0.3 s-old frame from a 4 s-old one.
- `Camera.__init__` (`tools/rig_io.py:31-46`): 3 s settle drain + exposure sweep
  (8 steps × 8 reads on MSMF, `tools/rig_io.py:48-61`) — several seconds before the reader
  thread even starts; paid once per run at `tools/model_jab.py:242`.
- `servo()` (`tools/rig_io.py:112-139`): one long-lived SSH session (kept deliberately:
  sshd `MaxStartups`); each command is a stdin/stdout round-trip. `snap` replies carry a
  base64 JPEG (~100–300 KB at q90) decoded desktop-side.
- Bridge (`robot/servo_bridge.py`): `{"snap": true}` re-encodes JPEG q90 on the Jetson
  **per request** (`robot/servo_bridge.py:79-88`), refuses frames older than
  `MAX_FRAME_AGE = 1.0` s, and at start-up waits up to 10 s for `joint_states` + publisher
  match (`robot/servo_bridge.py:56-60`). There is no "give me N frames" or "give me the
  cached frame" command: `snap_arm(3)` = 3 SSH round-trips + 3 Jetson encodes.
- Sleeps in `tools/model_jab.py` (current numbering): 141:1.5, 220:0.3 (view retry),
  281:1.3, 303:1.3 (tie band), 469:1.2 (roll), 496:1.4 (pan), 568:2.3 (place move),
  621:0.9 (practice/wrist step), 626:1.1 (fly-back), 661:0.9 (wrist), 674:0.6 (contact),
  713:2.2 (lift). Plus `send(..., ms=...)` motion durations (400–1500 ms) that the sleeps
  sit *on top of*, and `tm.drive()`'s `sleep(seconds + 1.2)` (`tools/tip_motion.py:37`)
  per strafe pulse.
- Fixed start-up waste: the `"cube"` `triangulate()` attempt fails in most runs
  (61 `tip-depth-failed` rows; in both reference runs `UGREEN: nothing long and thin moved
  on the sheet`) after ~18 s average, then the work is redone via `"dots"`. Note the
  failure is **conditional**: `triangulate(..., "cube", ug_pre)` uses `am.moved_needle`
  only when `ug_before is not None`, else `sam_needle_tip` (`tools/model_jab.py:181-203`);
  the plan must not assume the cube path is always useless — it must measure *why* it
  fails (moved-needle found nothing vs SAM found nothing vs ray gap >25 mm) before gating it.

### 1.3 Battery arithmetic

- 313 s run → 120 s needs **−193 s**; → 60 s needs −253 s. 138 s run → 120 s needs −18 s.
- The median run (~92 s) already meets 120 s *when it touches*; the battery problem is
  (i) the long tail (5 runs >190 s) and (ii) 46/66 runs aborting after spending their
  start-up + placing budget. **Any work item that converts an abort into a touch, or
  aborts 30 s earlier with the same information, is worth as much as a pure speed-up.**
  The first draft of this plan ignored (ii); §3 W8 and the acceptance gates in §5 fix that.

---

## 2. Design principles

1. **Never re-measure what did not move.** Sheet dots, red dots, cube, UGREEN rig and the
   notebook's dot layout (`evidence/descend/ugreen_dots.json`) are constant within a run.
2. **Overlap, don't just delete, waits.** A settle sleep and a camera capture for the
   *previous* pose can overlap the next move's planning; a sleep that exists because a
   servo needs `ms` to travel must become a deadline checked against motion-complete,
   not a fixed tax added after it.
3. **Classical first, SAM as fallback** — only where a classical detector already exists
   and is proven on saved frames (`tm.arm_view` blobs, `teal_tip`, `moved_needle`).
   SAM stays the authority for the initial tie and for any frame the classical path
   rejects. This is *not* a SAM→classical swap (see §6 non-goals).
4. **Every optimisation ships behind a replay gate** (W5) with golden masters from the
   two reference runs *plus* one aborted long run (183012) — an optimisation that only
   preserves the happy path is not accepted.
5. **Log the wait, not just the result.** Every `note()` row gets a `wait_s` breakdown
   (move / sleep / snap / grid / sam / ugreen) so the next plan iteration argues from
   data. This is a prerequisite for W6 and costs nothing at runtime.

---

## 3. Work items

Ordered by (saving × certainty) / risk. Each item: change, files, expected saving
(against §1.1 segments), acceptance test, rollback.

### W1 — Cache everything that does not move; SAM session reuse

**Change.**
- `ug_tie`, `red_px`, `ug_px`/`ug_mm`, `cube_u`: computed once at tie time
  (`tools/model_jab.py:310-365` region) and stored in a run-level `TieCache`; `retie()`
  must not re-derive them. (Today `retie` correctly reuses `ug_tie`/`cube_u` closures,
  but `arm_dots` + `cube_centre` re-run SAM on the *arm* image every call, and
  `cube_centre` is re-run even when the cube is out of view and the `except` branch
  immediately discards it for the model carry — memoise the last good cube centre and
  the "cube not in this view" verdict per view-pose bucket.)
- `arm_dots`: try the classical blob set first — `tm.arm_view` already returns blob
  sheet-points for the same image (`tools/tip_motion.py:45-60`, blackhat >40, area
  3–200, needle masked). The tie needs *pixel* centres matched against `ug_tie` by
  `match_frames` (`tools/model_jab.py:85-99`); use classical pixel centroids
  (recompute centroids in px before `on_sheet`) and fall back to the two SAM prompts
  only if `match_frames` returns `matched < 2` or the classical count is 0. SAM stays
  the fallback, not the default.
- `tools/sam_detect.py`: add `detect_many(predictor, image, prompts)` — one
  `start_session`, all prompts, one `close_session`; and an optional in-process
  memo keyed by (image bytes hash, prompt, threshold). Keep `detect()` as a thin
  wrapper so other tools (`tools/aim_touch.py`, `tools/dot_servo.py`) are untouched.
- `sam_needle_tip` after roll steps (`tools/model_jab.py:169-179, 473`): keep SAM
  (no proven classical equivalent at roll extremes) but route through `detect_many`.

**Files:** `tools/model_jab.py` (`arm_dots`, `cube_centre`, `retie`, tie block),
`tools/sam_detect.py`, `tools/tip_motion.py` (expose px centroids from `arm_view`).

**Saving:** retie SAM 1.1–1.5 s → ~0.2 s classical (PnP already paid in `view()`);
×~12 reties/run median (772/66) ⇒ **~12–18 s/run**; start-up tie −1–2 s. On 130933
(21+ reties): ~25 s.

**Acceptance:** replay (W5) of 130933 + 124320 + 183012: identical `matched` on every
`retie` row, `target_mm` within 0.5 mm of golden on every `hover` row, zero new
`roll-tip-rejected`. Live A/B: median `place-plan`→`retie` gap ≤2.5 s (from 4.4 s).

**Rollback:** `--sam-tie` flag restores SAM-first `arm_dots`; cache is per-run, no
persisted state to corrupt.

### W2 — Cheaper `view()`: fewer round-trips, snap overlapped with settle

**Change.**
- Bridge: add `{"snap": {"n": 3, "max_age": 1.0}}` → returns N **distinct** cached
  frames (the bridge already holds the latest `Image` in `state["image"]`,
  `robot/servo_bridge.py:48-49`; it must buffer the last ~8 frames with stamps and
  **not** re-encode: encode each frame once on arrival, cache the JPEG bytes).
  Keep `{"snap": true}` for compatibility with `tools/tip_descend.py:106` and others.
- `snap_arm(n)`: one bridge call for n frames; keep the byte-distinctness check and
  the stall/restart logic (`tools/tip_triangulate.py:301-319`) but base "distinct"
  on bridge frame stamps, not JPEG bytes (re-encode of the same frame must not look fresh).
- `view()`: pick n from data — replay grid-pose repeatability at n=1,2,3 on saved
  `*_arm.png` sequences (W5 provides this); default n=2 if pose σ ≤0.3 mm / 0.1°,
  else n=3. (Current code already calls `snap_arm(3)`, not 6 — the first draft's
  "6→3" was wrong; the win here is round-trips + encode, est. 0.3–0.6 s/view, plus
  the retry loop becoming visible: log `view-retry` rows instead of silent `sleep(0.3)`.)
- Overlap: in the place/roll/pan branches, issue `send()` then start `snap_arm`
  immediately; the bridge's distinct-frame logic already rejects pre-move frames
  *only if* stamps are compared against the send time — pass `after=<monotonic>`
  in the snap request so frames captured before the move are not counted.
  The fixed sleeps (1.2/1.4/2.3 s) become `max(0, deadline − elapsed)` where the
  deadline is `ms + settle_margin` measured from the send, with `settle_margin`
  fitted in W6 (start: 0.4 s roll/pan, 0.8 s place).

**Files:** `robot/servo_bridge.py` (frame buffer, encode-once, `snap{n,after}`),
`tools/tip_triangulate.py` (`snap_arm`), `tools/model_jab.py` (`view`, sleeps at
469/496/568), `tools/rig_io.py` (no change; session reused).

**Saving:** ~0.5–1.0 s × ~24 views/run ⇒ **12–24 s/run**; on 130933 (89 views):
45–90 s. This is the single biggest lever for the long tail.

**Acceptance:** replay: grid pose from n-frame mean within golden tolerance on all
reference frames; no `grid pose failed 3 frames running` on replays. Live: per-view
wall time (new `wait_s` logging) median ≤1.2 s; zero stale-frame touches (bridge
stamp check). Robot-side deploy is a file copy + bridge restart; old `{"snap": true}`
path remains for rollback.

### W3 — Start-up: no paying for failure twice (60/55 s → ≤25 s)

**Change.**
- Lazy SAM: move `predictor = sam.load()` (`tools/model_jab.py:143`) behind first
  actual SAM use (tie block), and start it in a background thread at `light` time
  so the 8.7 s overlaps lamp test + camera open + first snap. `--plan-only` without
  a tie must never load SAM. Thread-safety: `sam.load()` only builds the predictor;
  all `detect` calls stay on the main thread (predictor is not assumed thread-safe).
- Tip-depth order: log *why* `"cube"` fails for one week of runs (add `how` sub-reason:
  `moved_needle` empty / SAM empty / ray gap / λ out of range — the four exits in
  `triangulate`, `tools/model_jab.py:181-203`). Then: if the cube attempt's cheap
  pre-check (needle visible as moved region in `ug_pre`→`ug0` diff above a pixel
  threshold) fails, skip straight to `"dots"`. Do **not** delete the cube path:
  it is the only depth source when the arm view has <2 dots.
- Tie-band loop (`tools/model_jab.py:295-305`, up to 16 iterations): cap at 6 with
  the existing step sizing (steps are 3–12 pulses ≈ 4.5–18 mm; 16 iterations means
  oscillation, not approach — log `tie-band-oscillation` and abort early rather
  than burning 16 views). Early abort here saves up to ~40 s on a doomed run.
- `Camera()` settle: reduce `settle_s` 3.0 → 1.0 and skip the 8-step exposure sweep
  when the previous run's locked exposure is cached in `evidence/descend/.exposure`
  (same lamps, same room; validate median brightness ∈ [120,160] on the first
  frame, else run the sweep). Cache is advisory only.
- `restart_arm_camera` 30 s allowance (`tools/tip_triangulate.py:311` region):
  emit a `note(phase="camera-stall")` at 5 s so a dead run is visible on the
  console immediately (today it looks identical to a slow run for 30 s).

**Files:** `tools/model_jab.py` (start-up block 131-365), `tools/rig_io.py`
(`Camera.__init__`, exposure cache), `tools/tip_triangulate.py` (stall note via
callback — `tip_triangulate` must not import `model_jab`'s `note`; pass an
optional `on_stall` callable).

**Saving:** **25–35 s/run** typical (lazy/overlapped SAM 8.7 s, cube-fail skip
~15 s when it applies, tie-band cap variable). 130933: 60 → ~25 s.

**Acceptance:** replay cannot test start-up (no saved frames before `light`);
accept on live runs: `light`→`start` ≤25 s median over 5 runs, zero runs paying
both tip-depth paths in full without the pre-check firing, `--plan-only` wall
time ≤10 s to first `place-plan`.

### W4 — Practice arc: once per run unless the pose actually changed

**Change.** Cache `(wrist_land_step_direction, travel slope px/mm and mm/mm from
the practice arc, hover pose cmd snapshot)` after the first successful practice
arc (`tools/model_jab.py:613-637`). In later rounds, reuse the slope to compute
`gap_fix` from the UGREEN check alone; re-run the arc only if (a) pan/elbow/shoulder
pulses since the arc exceed thresholds fitted in W6 (start: pan >4 or elbow >6
pulses), or (b) the UGREEN check's implied landing moved >2 mm from the cached
prediction, or (c) `ROLL` changed since (roll moves the tip px the arc measures).
Also fix the known bookkeeping hazard this item exposes: today a practice-round
`gap_fix` is applied to `ug_fix`/`target` *after* the arc
(`tools/model_jab.py:644-646`) while the subsequent touch still uses the `land`
computed before the arc — when the arc is cached, `land`/`w_end` must be
recomputed from the post-fix target before descent, every round, cached or not.
(This is correctness, not speed; it is in scope because W4 changes when `land`
is allowed to be stale.)

**Files:** `tools/model_jab.py` (round loop 425-651, practice block, descent entry).

**Saving:** arc ≈ 5 steps × (0.9 s sleep + view ~1.5 s) + fly-back + view + retie
≈ 15–18 s; 130933 ran it in most of 6 rounds ⇒ **up to 75 s** there; typical
multi-round run: 15–50 s.

**Acceptance:** replay 130933: rounds 2+ produce `gap_fix` within 0.7 mm of the
golden full-arc values using the cached slope, or the re-run trigger fires and
says why (`practice-reason` field). Live: touches after a cached-slope round
land within the existing 5 mm `UG_TOUCH_MAX_MM` gate at the same rate as
full-arc rounds over ≥10 runs.

### W5 — Offline rehearsal harness (prerequisite for W1/W2/W4 acceptance)

**Change.** Build `tools/replay_run.py`:
- Input: a run directory (`NNN_arm.png` sequence, `sam_*.png`, `*_ugreen.jpg`,
  `contact_*_ugreen.jpg`, `dots.png`, `log.json` — 130933 has 115 files).
- It monkeypatches `rig_io.servo` (snap replies served from the saved `NNN_arm.png`
  sequence in `view()` order; `read` replies from `log.json` `pose`/`readback`
  rows), `Camera.frame` (saved UGREEN jpgs in call order), and records every
  `send()` instead of executing it. SAM runs for real on the 3090 against the
  saved `sam_*.png` / arm frames (SAM is deterministic enough at fixed threshold;
  where it is not, the harness reports divergence instead of hiding it).
- Output: a replayed `log.json` + a diff report vs the golden `log.json`:
  per-phase row presence, `matched`, `target_mm`, `tip_mm`, `gap_fix`,
  `w_end` trajectories; pass/fail against tolerances in a checked-in
  `tools/replay_tolerances.json`.
- Golden masters: 130933 (long, touched), 124320 (short, touched), 183012
  (long, **aborted** — the replay must reproduce the abort, not "fix" it).
- Also used standalone for W2's n-frame study and W6's parameter fits.

**Files:** new `tools/replay_run.py`, `tools/replay_tolerances.json`; small
seam in `tools/model_jab.py`: wrap the `servo`/`Camera` imports so the harness
can substitute them without editing the script (`import rig_io` + call via
module, or an env-var-selected backend). This seam is the only production-code
change in W5 and must be behaviour-identical by default.

**Saving:** indirect but decisive: turns each W1–W4/W6 live trial (~2–5 min
battery + user attendance) into a desktop run; prevents shipping a "speed-up"
that changes touch outcomes. Also the only way to test the abort path without
burning a battery cycle on a known-doomed run.

**Acceptance:** harness reproduces all three golden runs within tolerance on
unmodified code (this is the harness's own acceptance test, run in CI-style
before any W-item merges).

### W6 — Learn parameters from the 68 logs (fewer rounds, not just faster rounds)

**Change.** Offline script `tools/mine_logs.py` over `evidence/descend/*/log.json`:
- Fit per-joint response tables from existing rows: pan gain (`pan-step`
  `mm_per_pulse`, 144 rows; today relearned per run from scratch and sign-flips
  poisoned it, `tools/model_jab.py:488-508`), roll `ROLL_MM` (currently a hand
  constant 0.42 with documented drift 0.26→0.45, `tools/model_jab.py:67`),
  elbow mm/pulse (~1.5 assumed at line 300), strafe `mm_per_s`, wrist
  mm-height/pulse from `wrist` rows (133).
- Output: `evidence/descend/learned_params.json` (checked in, versioned) with
  per-pose-bucket (pan/elbow range) medians + IQRs; `model_jab.py` loads it as
  the *prior* and still relearns online, but starts from the fitted value and
  needs one confirming step instead of two exploring steps.
- Fit the W2 settle margins and W4 re-run thresholds from the same data
  (view-gap distributions conditioned on move size).
- Explicitly *not* fitted: anything that changes where the tip lands without a
  confirming measurement (that's the excluded predictive redesign).

**Saving:** 1–2 fewer placing iterations + 1 fewer UGREEN round on a typical
run ⇒ **15–40 s/run**, and fewer `PLACE_ITERS` exhaustions (a top abort cause:
46 aborts). Compounds with W1/W2 (each saved iteration is worth more once
iterations are cheap — and vice versa; §4 orders accordingly).

**Acceptance:** replay: with learned priors, 124320/130933 converge in ≤
golden iterations and land within golden `gap_fix` tolerance; 183012 still
aborts but ≥20 s earlier or converts — either is reported, neither is hidden.
Params file regenerated by a single documented command; online relearning
still overrides a prior that the live steps contradict (existing sign/gain
guards stay).

### W7 — Perception pipeline: keep the 3090 busy instead of the robot waiting

**Change.**
- Async SAM worker: a desktop-side process/thread pool that owns the predictor;
  `detect_many` requests are queued. The only place this helps without changing
  behaviour is *speculative* work whose result is checked before use: e.g.
  pre-detecting arm dots on the frame `view()` is about to return while PnP
  finishes, and pre-running the UGREEN `teal_tip` SAM fallback while the
  classical `teal_tip` runs. Results are used only if the input frame hash
  matches what the main thread later asks for — no stale-result path exists.
- UGREEN `frame()` returns (image, capture_stamp); callers in the contact loop
  (`tools/model_jab.py:670-688`) use the stamp to reject a frame older than
  the last wrist send (today a 5 s block or a stale mean can be read as
  "tip did not move" ⇒ false `still` ⇒ the known streak failure mode).
  This is a *correctness* fix with a speed side-effect (no repeated contact
  steps taken on stale frames).
- Classical tracking between SAM anchors in the contact loop: `teal_tip` is
  already classical and fast; keep it, but when it returns None, fall back to
  template/`moved_needle` on the last UGREEN frame before paying a SAM call.

**Files:** `tools/sam_detect.py` (worker), `tools/rig_io.py` (`frame` stamps),
`tools/model_jab.py` (contact loop, `teal_tip` fallback order).

**Saving:** 5–15 s/run (speculative dot detection overlaps PnP; contact loop
stops paying 0.6 s sleep + SAM on None frames). Modest — this item is last
and optional; its stamp fix should be pulled forward if contact streaks recur.

### W8 — Fail fast: spend battery on runs that can touch (new in this revision)

**Change.**
- Abort-budget guards, all logged with `phase="early-abort"` + reason:
  tie-band iterations >6 (W3), `matched < 2` after the tie block (already
  raises — keep), placing iterations without `|err|` improvement for 5
  consecutive iterations (the logs show 30-iteration placing grinds),
  UGREEN rounds where `gap_fix` oscillates sign twice at similar magnitude
  (full fixes swung −12/+10/−8 mm historically).
- Each guard's threshold is fitted/validated by replaying the 46 aborted
  golden logs (W5 extended: replay must reproduce ≥90% of historical aborts
  at or before their historical abort time, and must **not** early-abort any
  of the 14 historical touches).
- Report per run at exit: `run-summary` row with segment times (from the W-§2.5
  `wait_s` logging) so the user sees *why* a run died in one line.

**Saving:** converts the long tail: a 291 s abort (183012) that is recognisably
doomed at 90 s saves ~200 s of battery for a fresh attempt. Across the 46
historical aborts this is the largest *expected* battery saving in the plan,
larger than any single W1–W4 item — the first draft missed it entirely.

---

## 4. Order and dependencies

1. **§2.5 wait logging + W5 replay harness** first (1–2 sessions): nothing else
   can be accepted without them.
2. **W1** (cache + classical-first tie) — biggest safe saving, replay-gated.
3. **W3** (start-up) — independent of W1, live-gated (no replay possible).
4. **W2** (bridge + view) — needs a robot-side deploy window; do after W1 so
   view-cost attribution is clean.
5. **W4** (practice once) + its `land` recomputation fix.
6. **W6** (learned priors) — needs the wait logging to validate iteration savings.
7. **W8** (fail fast) — needs W5's abort corpus; thresholds from W6 fits.
8. **W7** (async/speculative) — last, optional, pull the frame-stamp fix
   forward independently if streaks recur.

Projected totals (reference runs, W1–W4 only): 130933: 313 → ~150–175 s;
124320: 138 → ~75–85 s. Adding W6+W8: 130933-class runs → ~110–130 s when
they touch, and doomed runs die at ≤100 s instead of 200–300 s. The 60 s
end of the target is **not** reachable within the agreed change-risk limit
(waits + proven-redundant rounds only); reaching it would need the excluded
predictive placing (fewer confirming views), which is §6.

## 5. Acceptance for the plan as a whole

- Median touched-run time over the next 10 live `--jab` runs ≤120 s, with
  touch accuracy unchanged (every touch inside the existing UGREEN
  `UG_TOUCH_MAX_MM = 5 mm` predicted-miss gate; ink-mark miss measured from
  `contact_ugreen.jpg` no worse than the historical 14-touch distribution).
- No historical-touch replay (W5 corpus) changes outcome under any merged item.
- ≥90% of historical aborts replay-abort no later than they did historically.
- Start-up (`light`→`start`) median ≤25 s over 5 consecutive runs.
- Every merged item has its flag/rollback path exercised once on replay.

## 6. Non-goals (explicitly excluded by the agreed change-risk limit)

- No closed-loop servoing during descent; the wrist arc stays open-loop with
  per-step measurement. No predictive plunge.
- No SAM→classical detector *replacement*: classical paths are first-attempt
  with SAM fallback and SAM-verified ties; SAM remains the tie authority.
- No weakening of safety gates: `floor_ok` −3 mm placing floor, shoulder-slip
  abort, `UG_MAX_MM`/`UG_TOUCH_MAX_MM` landing gates, `matched ≥2` tie gate,
  `MAX_FRAME_AGE` staleness refusal (its *implementation* moves to stamps in
  W2, its semantics stay).
- No changes to `tools/aim_touch.py` / `tools/dot_servo.py` behaviour (they
  share `sam_detect.detect`, which stays compatible).
- The 0.5–1 mm touch-accuracy problem (post-touchdown slide, see
  `docs/critique_rca_plan_2026-10-28.md`) is **not** solved by this plan;
  speed must not be bought by removing the contact steps that detect it.
  W7's frame stamps make that detection more reliable, nothing more.

## 7. Risks and open questions

- **Classical arm dots (W1):** `tm.arm_view` blobs include ink marks, holes and
  binding, not just tie dots; `match_frames` must reject them geometrically.
  If replay shows `matched` dropping on >5% of frames, W1 ships with SAM-first
  retained and only the cube memoisation + `detect_many` kept.
- **Bridge frame buffer (W2):** the Jetson is shared with the platform stack;
  buffering 8 JPEGs (~2 MB) is fine, but encode-on-arrival adds steady CPU.
  Measure bridge CPU before/after; if >10% of a core, encode on demand but
  keep the stamp semantics.
- **SAM determinism (W5):** if `detect` on identical input is not repeatable
  enough for golden masters, the harness must record SAM *outputs* (masks are
  already saved as `sam_*.png` inputs, not outputs) — then replay tests
  everything except SAM itself, and SAM changes get live A/B instead. Decide
  after the first harness run; do not assume either way.
- **Exposure cache (W3):** lamps/room change between days; the brightness
  validation gate is mandatory, and a failed validation must fall back to the
  full sweep silently (log it, don't abort).
- **Learned priors (W6):** the needle bends and gains drift (documented
  `ROLL_MM` 0.26→0.45 after a bend). Priors must carry a fitted date and the
  online relearn must be able to override within one step; a stale prior that
  costs an extra iteration is acceptable, one that causes a wrong-direction
  move is not — the existing sign guards are the protection and stay.
- Open question for the user: is a *second* battery/user-attention budget
  (e.g. max wall time per attempt, after which the run should early-abort
  and ask) wanted as a hard cap in W8, or are the fitted guards enough?
  The plan implements fitted guards; a hard cap is a one-line addition if wanted.

## 8. What was wrong with the first draft of this plan (kept for the record)

- It ended mid-sentence in W5 (`- Feed a record` …) and had no §4–§5 content
  beyond a projection paragraph — W5's spec, W6/W7 detail, ordering and
  plan-level acceptance were missing.
- It claimed `snap_arm(6)` was the current cost; the code calls `snap_arm(3)`
  (`tools/model_jab.py:210,214`). The real W2 win is round-trips + Jetson
  re-encoding + stamp-based freshness, not frame count.
- It claimed the plan targets the median run; in fact the median (~92 s)
  already meets 120 s and the battery is eaten by the long tail and the
  46 aborts — hence W8, which the draft lacked entirely.
- It treated the `"cube"` tip-depth failure as pure waste; the failure is
  conditional on needle visibility, so W3 now requires measuring the exit
  reason before gating it.
- It did not state that 60 s is unreachable within the agreed risk limit.

## 9. Adjustments after review, implemented and tested (2026-10-29)

Each item was measured before it was built; items the data did not support were dropped.

| Item | Measured | Decision |
|---|---|---|
| W0 camera switch (new) | overhead = 4K UGREEN, index 1; old 1080p UGREEN is now the low side camera, index 0 | `rig_io.OVERHEAD = 1`, all `Camera(0)` -> `Camera()`; MSMF first (DSHOW gave 5 fps at 1080, MSMF 31); exposure sweep reads 8 frames per step (4 locked one step too dark, median 71 -> 117-136) |
| W0 lens | 4K at 1080: f 1608.5, 1.81 px rms, 1332 crossings (4K run: 3217.6/2) | `evidence/tip/ugreen4k_lens.json`, `tip_triangulate.LENS` points at it. Tape check still to do |
| W0 dots | `"black dot"` finds only binder holes; `"small black dot"` finds the 4 ink dots, 62-180 px; ink/paper 0.07-0.52 overlaps red 0.36-0.50 | `model_jab`: prompt `"small black dot"`, `SAM_AREA` (40, 300), `SAM_BLACK_FRAC` 0.6 (the red prompt separates red); `ugreen_dots.json` re-written in 4K pixels |
| W1 SAM session reuse | 3 prompts on one retie frame: 1.05 s separate, 0.88 s in one session, identical masks | dropped: 0.17 s per retie. SAM stays primary (no classical-first) |
| W2 bridge batch snap | `snap_arm(3)` 0.53-0.71 s live, one snap 0.18 s | dropped: at most ~0.3 s per view, not worth a robot deploy. The overlapped settle would average a moving arm |
| W3 start-up | SAM load overlapped with lamps + UGREEN open: camera+SAM ready in 8.5 s (serial ~16 s); `Camera` settle 3 -> 1 s | done. Tie-band cap dropped: no log data on its iterations |
| W4 practice arc | 7 rounds in 4 runs re-ran the arc with no arm move in between (124 s): a halved along fix (~1.1 mm) sits under placing's 1.5 mm along tolerance, so placing exits at once | done as a cache keyed on the exact pose: same pose -> same landing, no information lost (`practice-arc-cached` row) |
| W8 fail fast | every no-progress rule (norm/side/max error, 4-15 steps) that catches aborts also kills 1-8 touched runs; fix-sign oscillation occurs in touched runs (124320, 125017) | dropped: the logs do not separate doomed runs |
| W5/W6/W7 | not started | |

Tested live (no robot motion): start-up + UGREEN tie on the 4K view: rig 430 inliers at 1.05 px, 3 red dots, cube, 3 of 4 black dots within 1.7 px of
`ugreen_dots.json` (the far dot at (721, 320) is ~63 px and SAM misses it in some frames: the run fills it from the file).

Open:
- Far black dots: 5 of 5 later frames found 4/4 black + 3/3 red on the full frame, same with a crop around the dots, so no crop. The one miss came with a brighter exposure lock (median 136 vs 117-123).
- `--dot U V` / `--red U V` take UGREEN pixels: use 4K-view pixels now (`ugreen_dots.json`).
- Root cause of the no-move rounds: `UG_GAIN` 0.5 halves fixes below the 1.5 mm placing tolerance. Not changed (a gain change is a behaviour change).
- The teal tip finder and the hover check are untested on the 4K view (no needle in view today).

### 9.1 Live runs 2026-10-29 (4K overhead + side camera)

| Run | Result | Time | Notes |
|---|---|---|---|
| `20261029T113753` | **touch**: tip on dot 4 in the overhead zoom, arm camera miss (-0.83, 1.4) mm | 181 s (start 43 s) | `practice-arc-cached` fired in round 3. Tip not inked: no ink mark, the side teal finder failed |
| `20261029T114242` | abort `tip 4.7 mm up while placing` after 5 rounds | | practice arcs swung -13 / +11 mm along: no ink, so the overhead tip came from the SAM fallback |

Side camera (`rig_io.SIDE = 0`): SAM 3.1 "thin metal needle" finds the needle in every side frame (0.74-0.87). Tip y over
six 3-pulse wrist steps 659 -> 692 px (~7 px per step), then +4, +2, +1 px on the 1-pulse contact steps: the tip meeting
the paper. `model_jab` logs `side_tip_px`/`side_down_px` per step, saves `side_*.jpg`, stops on 2 steps under
`SIDE_STILL_PX` 1.5 px.
| `20261029T120925` | **touch, ink mark 1.49 mm from the dot centre** (-0.03, -1.49 mm), round blot, no streak | 412 s | inked tip. 2 rounds; practice arcs predicted 2.5 then 0.3 mm. Side tip y +8 px per 3-pulse step in the air, 4, 0, 5 px on the contact steps |

Fixes from these runs: `teal_tip` used S >= paper + 20, but the 4K UGREEN reads the ink H~56 S~62 on warm paper (H 16, S 76),
so every overhead tip came from the SAM fallback; now hue 45-100, S > 35 (finds the tip in the hover and contact frames of
120925). Side camera raised by the user: the grid still does not solve (squares ~5 px at the far side) and SAM sees only
the binder holes, not the ink dots, so it stays a pixel-only contact sensor for now.
| `20261029T121904` | abort: arm camera stalled | 320 s | `sensor_supervisor_node` was running (restarts the camera every 16 s); stopped by pid (SIGINT, then SIGKILL), not respawned; snaps then 0.46-0.54 s |
| `20261029T123956` | abort: UGREEN check did not converge in 6 rounds | 233 s | first check 12.4 mm sideways (as 115803), worked off at gain 0.5 in 5 rounds; then the practice arc read -11.8 mm: the tip had no ink left (spent on 120925), so the overhead tip came from the SAM fallback |

The 12 mm runs (115803, 123956): ties replayed offline are exact (all matched dots 0.1-0.6 mm), the start tip pixel, tip depth and
roll are the same as in the good runs; they differ in pan (531-533 vs 520-524 before the first hover). Root cause open.
Changes: `UG_GAIN` 0.5 -> 0.8 (each UGREEN fix was realised ~1:1 in both runs); `teal_tip` returns None when the tip's end runs
into old ink (a re-touched dot's mark hid it). Re-ink the tip before every run: one touch spends the ink.
| `20261029T125405` | **touch in 135 s** (start 47 s, first hover 78 s), stopped by `side-still` | 135 s | re-inked tip, gain 0.8: 2 rounds, practice arcs 1.2 then 0.2 mm, first check 0.4 mm. New ink only at the edge of 120925's mark: ~1.7 mm (+1.63, -0.59) from the dot centre, 10 px of new ink, low confidence. Side tip: +9, +8, +10 px per 3-pulse step, then 7, 1, 0 px on the contact steps |

### 9.2 Offline work after the robot was parked (2026-10-29 evening)

| Item | Result |
|---|---|
| Wait logging (2.5) | every `log.json` row carries `wait_s` {sleep, snap, grid, sam, ugreen, side} since the previous row; `pose0` row logs the first arm read |
| Frame recording | every UGREEN frame is saved as `ug_NNN.jpg` in call order (only some were before) |
| W5 replay harness | `tools/replay_run.py RUN --out DIR`: model_jab unchanged, fake arm (holds commands), saved frames, sleeps skipped, real SAM; compares row by row. Run 125405 replays with **0.0 difference** in every compared field for 33 rows (older runs end at the first hover: their UGREEN frames were not all saved). `--compare LOG` compares only. A replay is open-loop: after the first changed command, later rows are not meaningful |
| 12 mm start error | **cause: the model carry of the target.** At the first hover the arm-view dots (3 agreeing, one solution) put the target 16.9 / 19.2 mm from the carry in 123956 / 115803 and 0.0-0.9 mm in the good runs; the arm tip pixel is fine (start, roll and hover pixels agree to 1-3 mm). `retie` with the cube out of view takes a dot only within 8 mm of the carry, so a drifted carry refuses every true dot |
| Fix | `vote_offset` (dots agreeing at the tie's rotation) is **logged only** (`retie-vote` rows): right after the first move it read ~23 mm in every run, good ones included (2 dots, once 3), and the replay cannot tell whether that is the vote or the carry. Decide from the next live runs' `retie-vote` rows |
| W3 cube depth skip | dropped: the failing cube attempt is one classical check; the time was the band loop after it (10-11 views, 18-20 s, elbow 270 -> ~160 in steps of 12). `arm_to_start.py` now leaves the elbow at 190: ~3-4 views, ~12 s less |
