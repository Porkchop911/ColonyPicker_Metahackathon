# Critical evaluation: `docs/rca_touch_dot.md` + `docs/plan_2026-10-28.md`

**Verdict:** The RCA is a good *diagnosis narrative* but a weak *evidence document* — its central insight is right and code-corroborated, its statistics are unauditable and internally inconsistent. The plan correctly inherits the RCA's direction (watch the descent, measure first contact, offline first) but is **not executable as written**: its core loop (0.5 mm / 0.2 mm corrected steps judged to ≤1 mm) exceeds the measured resolution of every actuator and sensor it proposes to use, on the RCA's own numbers. Several RCA fixes are silently dropped, and the schedule underestimates the two hardest steps by roughly an order of magnitude.

---

## RCA — what holds up

Verified against code:

- **Blind final centimetre (RC1) is real.** `tools/aim_touch.py:59` `HIDE_MM = 8.0` with the exact anecdote in the RCA; `tools/aim_touch.py:65` "below this the needle hides the dot: descend blind." In `tools/model_jab.py`, every UGREEN read is a discrete `ugcam.frame()` between moves (`tools/model_jab.py:229`, `tools/model_jab.py:234`, `tools/model_jab.py:260`, `tools/model_jab.py:458`, `tools/model_jab.py:490`, `tools/model_jab.py:530`), never mid-motion. First-contact vs end-pose was genuinely the wrong success metric, and the user's drag observation correctly falsifies the two "on the dot" runs.
- **`back_px` false stops, `moved_needle`, hand-made `ug_black`** — all confirmed (`tools/aim_touch.py:67` `STOP_PX = 8.0  # …the dot beside the tip faked 5 px`; `tools/aim_touch.py:232`, `tools/aim_touch.py:290`). The process self-criticism (RC5) is consistent with code: only `aim_touch` has a simulator (`tools/aim_sim.py`), `jab.py`/`model_jab.py` have none, no replay/stub code exists anywhere.
- **Servo/gateway facts** — 0.5 s strafe cap enforced (`tools/tip_motion.py:37`, `tools/model_jab.py:382`); commanded-pose-only discipline present.

## RCA — where it fails its own standards

1. **The headline numbers don't add up and can't be audited.** "About 95 recorded runs" vs the table's 21 + 37 + 49 = **107** (contacts 3 + 5 + 6 = 14 do match). Worse, none of it is in the repo: `evidence/` contains only 2026-10-26 material, the named runs `mjab/20261027T224605` exist only as text in the RCA itself, and run outputs went to caller-supplied `--out` dirs. A document whose RC5 condemns unlogged trial-and-error rests its own failure tallies (target lost 6, tie failed 5, grid pose 4…) on logs a reader cannot inspect. These counts should be treated as unverifiable, not wrong — but the RCA never flags that.
2. **The ranking is asserted, not derived.** RC1 is ranked #1 "by how much they explain," but by the RCA's own table most runs died *before* the final centimetre (RC4/RC5 failures). RC1 explains why the few contacts were false successes; it does not explain the modal failure. And RC1 vs RC2 are confounded: blindness doesn't cause early touchdown — servo-2 sag (RC2) does; blindness only hides it. No error budget or ablation separates "trajectory was wrong" from "trajectory was fine but unwatched," yet the plan's spending follows this ranking.
3. **It describes dead code as current behaviour.** The "8 waypoints" jab in `tools/model_jab.py` is unreachable after the `return` at `tools/model_jab.py:500`; the live final motion is the wrist-arc loop (`tools/model_jab.py:479-489`) which *does* measure height each step from the arm camera. The RCA's table is historical, but it never pins claims to run IDs/commits, so "every contact was made by a motion no sensor watched" is strictly true of the drag runs and not quite of the current code.
4. **"SAM 3.1 was required" is oversimplified.** `model_jab`'s SAM path still applies hand-made filters on top (`SAM_AREA`, `SAM_BLACK_FRAC` at `tools/model_jab.py:41-43`), and SAM is the component that *fails* on the needle (`tools/model_jab.py:146`) and on dots above 30 mm (`tools/model_jab.py:39`, `tools/model_jab.py:244`). The RCA simultaneously blames hand-made detectors and documents SAM's failures without reconciling them — and the plan inherits this as an absolute rule (see below).

## Plan — fatal problems

### 1. The specified descent cannot physically make 0.5 mm / 0.2 mm steps (Step 5 vs RC2)

The plan's own source numbers: elbow ≈1.5 mm/pulse (`tools/model_jab.py:250`), pan ≈2.2–4 mm/pulse with `SIDE_TOL_MM = 1.2` = *half a pan pulse* (`tools/model_jab.py:52`), deadband 3–5 pulses, backlash ~5 pulses, a pan reversal "jumps ~20 mm" (`tools/model_jab.py:362`), strafe non-repeatable (0 / −3.3 / +18 mm for the same pulse). `docs/needle_on_dot.md` already concluded from the simulator that **≤1 mm is out of reach with pan as the sideways axis**. Step 5 says "correct sideways and along before the next step" — with what? Pan can't, strafe can't repeatably, and reversals trigger the backlash the RCA documents. The one fine axis that exists in the record — wrist roll, ~0.1 mm/pulse sideways (`docs/plan.md`) — is mentioned only in passing ("wrist roll on the angled needle") with no Jacobian, no calibration step, no numbers. Meanwhile Step 5 explicitly *replaces* the wrist turn — currently the only semi-closed-loop final motion in the code — with this unachievable stepped descent. **As specified, the plan's gate is unreachable; this is the single biggest flaw.**

### 2. Rule 4 (SAM for every detection) contradicts the real-time loop (Rules 1, Steps 3–5)

SAM here is file-in, session-per-call: `tools/sam_detect.py:31-38` starts and closes a session per image, and `model_jab` writes a PNG per detection. No latency figure is measured or budgeted anywhere, yet the plan wants SAM tip-tracking on a 5–10 Hz stream, per-step correction, and stop-on-contact. That loop as specified will run at SAM latency, not camera latency — likely blowing the 5-minute timebox (Rule 7) on step count alone. Rule 4 also bans exactly the techniques Step 3 relies on (motion difference, shadow meeting, read-back lag) and the only UGREEN tip finder that works (`moved_needle`, needed *because* SAM misses the needle). Step 2's check "SAM finds the needle in this view" is a hope with no fallback if it fails — and yesterday's evidence says it fails.

### 3. The contact signal lost its independence — and its best candidate

RCA §9.1 asked for a contact signal *independent of the cameras* (conductive path first). The plan drops the conductive option entirely and judges first contact with the same side camera that controls the descent — correlated errors, and the "ground truth" in Step 3 (frame where tip visibly touches, side camera) is circular for the leading candidate (tip-meets-shadow, same camera). Ten *manual teleop* descents also don't transfer cleanly: different speed, different compliance. If a foil/conductive test is impractical, that should be argued, not silently omitted.

### 4. The ToF role is physically shaky

A VL53L-class sensor "pointing down beside the needle" with a ~25° cone: at the 2–10 mm heights where it must act, the cone is a few mm wide, may contain the needle itself, and several VL53 variants are at/below their minimum range there. Yet Rule 2 makes it a *hard floor* ("stop below 2 mm measured") — a few-mm-noise sensor (the plan's own words) enforcing a 2 mm threshold will false-stop or false-pass. On the wrist it also adds load/cabling to the arm whose servo 2 is the known-weak joint. The plan hedges ("not the final millimetre") while simultaneously giving it a safety-critical interlock; pick one.

### 5. Measurement error consumes the whole tolerance

Gate: first contact ≤1 mm. Judged by cameras whose cross-check criterion in Step 2 is "agree to ≤1 mm." If the judge's own uncertainty is ~1 mm, passes and failures at the gate are indistinguishable from measurement noise. No error budget exists anywhere in the plan (side-camera mm/px at grazing angle, triangulation, tie error — and the RCA's own warning that the oblique-view cube centre was 17 mm off is directly relevant to a low side camera).

### 6. Sequencing and dropped RCA items

- **Step 0 gates on the thing Step 1 diagnoses.** "Servo 2 rises 423→443 within 3 pulses" is a *pass criterion* before the diagnosis step that would explain a failure — and a pass at the folded start pose says little about behaviour at reach (Step 1.4 partly covers this, but with no numeric pass/fail for temp/voltage/lag). A power cycle already "fixed" it once; no repeat/soak test distinguishes fixed from intermittent.
- Step 1 restarts the stack — which per the handover resurrects `sensor_supervisor_node`, invalidating Step 0's first check. Not mentioned.
- RCA fixes with no plan home: re-measuring tip depth after every grasp change (Step 0's "pushed by hand" is qualitative), lighting/exposure control for the new camera (dusk was RC6; the shadow cue *depends* on light direction), stop-on-loss-of-tracking during descent.
- **Step 4 is not 60 minutes.** `tools/model_jab.py` is a 563-line `main()` with all logic in nested closures over live I/O; stubbing robot I/O for replay is a refactor, not an addition. And "today's logs are the first test set" assumes logs that aren't in the repo (see RCA point 1 above).
- **Target-within-20 mm (Rule 5) is a diagnostic scaffold, not a solution.** Moving the notebook per dot may pass the gate while proving nothing about colonies on a plate, which can't be moved to the arm. The plan never labels it as such, so a passed gate risks being over-read — exactly the metric error the RCA confesses to.

### 7. Minor

- Gate statistics (2-in-a-row, then 4/5) are weak evidence of reliability — acceptable for a hackathon, but shouldn't be reported as more.
- Step 6 says "ask before creating `evidence/`" — it already exists.

## What to change first

1. **Reinstate an independent contact sensor** (conductive foil) as Step 3's primary; demote camera/ToF to corroboration. This also fixes the circular ground truth.
2. **Before Step 5, run a fine-axis spike:** quantify wrist-roll (and any elbow/wrist combination) in mm/pulse at the actual descent pose, with deadband/backlash, the way `docs/needle_on_dot.md` Step 1 prescribed. If no axis resolves ≤0.3 mm, the 1 mm gate needs re-negotiating *before* the day is spent.
3. **Measure SAM latency once** and design the loop as: fast classical tracker during motion, SAM as periodic re-anchor — i.e., soften Rule 4 to "SAM anchors, tracking interpolates," which is what the working code already does.
4. **Write an error budget** for the first-contact measurement; require judge uncertainty ≤ ~0.3 mm before gating on 1 mm.
5. **Swap Steps 0/1 for servo 2**, add numeric thresholds and a repeat test; re-kill the supervisor after any stack restart.
6. **Commit the run logs** (or at least the run table) into `evidence/` — the RCA's counts, and tomorrow's gate, otherwise remain unverifiable.

The plan's instincts — servo first, watch the descent, offline before live, one change per run, timebox — are the right corrective to yesterday. But as costed and specified, Steps 2–5 each carry an unexamined assumption (SAM speed, ToF close-range behaviour, sub-pulse actuation, 1 mm judging) that yesterday's data already contradicts.
