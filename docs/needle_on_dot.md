# Needle on the dot: how to proceed (2026-10-27)

Status: the needle reaches the paper reliably (arm camera only), but **no touch has landed on the chosen
dot yet**. This is the procedure to get there, built on the literature review in this session (sources at
the end). Goal = plan step 1.5: **first contact on the chosen dot, <= 1 mm off, on 4/5 dots**.

## Principles (what the literature says, applied here)

| # | Principle | Source | Here |
|---|---|---|---|
| 1 | Close the loop on what the camera sees, tool and target together; calibration errors then cancel | Hager et al. 1995 (endpoint closed-loop), Navab et al. 2000 | Keep aiming the landing point (tip-pixel ray meets the sheet) onto the dot. Do not switch to model-based IK |
| 2 | Learn the image Jacobian online, but only from data you trust, and only near the goal | Jägersand (Broyden + trust region); Broyden degrades far from the goal | Update only on steps that really moved, never on predicted targets, never across a reversal |
| 3 | Hobby servos: deadband and backlash outweigh calibration error | hobby-arm reports (Parallax, Instructables) | Steps of 0 or >= deadband; always finish each joint from the same direction |
| 4 | Final millimetres need a second view or a sensor, not a prediction | Hager (two cameras), micromanipulation contact detection | UGREEN checks tip vs dot at hover; stop on contact, do not press -2 mm |
| 5 | Calibrate the tool against the camera with a mark you can measure | QPix pin calibration (pierce tape) | Every touch leaves a mark; measure mark vs dot; a steady offset is corrected like a pin offset |

## Step 0 — find out why the touches miss (no robot needed, ~30 min)

Go through the aim_touch / teleop logs and `*_marked.png` frames from today's runs (session scratchpad on
the PC). For each failed touch note: final `err_mm` at contact, whether `seen` was true in the last steps,
and where the needle actually hit (from the contact frame or the paper). Sort each miss into one bucket:

| Pattern | Likely cause | Fixed in |
|---|---|---|
| Misses all in the same direction, similar size | landing point bias: needle not along the measured ray (bent, re-grasped), tip pixel `free` moved, or the descent is not along the ray | Step 5 (offset correction) |
| Misses scattered, 1-4 mm, direction varies | deadband / backlash / read-back lag, Jacobian drifted | Steps 2-3 |
| Hit a neighbouring dot / err jumps by ~5 mm | wrong dot snapped (`SNAP_MM` 4 with dots 5 mm apart) or tracker on the needle | Step 2e |
| err small at the last `seen` step, large at contact | the unseen final descent drifts sideways | Steps 2d, 4 |
| Arm sinks over the run, servo 2 lower each step | `cmd = dict(read())` re-sends read-back | Step 2a |

Write the bucket counts down; they tell you which fixes matter first.

## Step 1 — measure the servos from the image (teleop, ~20 min)

Use `tools/teleop.py` at ~15 mm height over a clicked dot. Measure motion from the **target offset in the
image** (`target_mm` in the log), not from read-back.

For pan (1), elbow (3), wrist (4), one at a time:
1. Step size 1: press + until `target_mm` changes. Pulses needed = **deadband**.
2. Step size 5: press + ten times, then - ten times. mm per pulse on the way out = the joint's **Jacobian
   column** (x, y, height). Pulses at the start of the way back that change nothing = **backlash**.
3. Repeat once to check it holds.

Expect deadband 3-5 and backlash ~5 pulses (handover). Put the numbers in a table in this file and in
`aim_touch.py` as `DEADBAND`, `BACKLASH` and a re-seeded `J0`.

## Step 2 — code changes in `tools/aim_touch.py`

a. **Commanded pose only.** Start `cmd` from the known start pose (or the teleop end pose passed on the
   command line), never from `read()`: lines 140, 169 and `lift()` (130-133). `lift` raises `cmd[3]` by
   40 and sends `cmd`. Same fix in `arm_touch.py` `lift` and the `tip_descend.move` callers. Read-back
   is only a sanity check (abort if it is > 35 off).
b. **Movement = commanded step, confirmed by the image.** Use `dq` (minus any backlash take-up, see c)
   as `moved` for the Broyden update, and update only if the image error changed by more than the noise
   (~0.2 mm) and no joint reversed direction in this step. Drop the read-back `moved` (line 184), which lags.
c. **Steps the servo can do.** Per joint: |raw| < DEADBAND/2 -> 0; otherwise at least DEADBAND, at most
   MAX_STEP. On a reversal add BACKLASH[j] to the command and leave it out of `moved`.
d. **Fixed final direction.** Pick a final direction per joint (e.g. pan +, elbow -, wrist +). Before the
   descent, any joint whose last move was the other way overshoots by BACKLASH + 5 and comes back. The
   final descent moves only elbow+wrist in the xy-neutral combination from the Jacobian (current solve
   with err = 0); if that combination reverses a joint, stop and re-align instead.
e. **Lock the target identity.** Pick the dot once at ~15 mm; then track it by its offset from the
   landing point and accept a re-detection only within 2 mm (half the dot spacing), not `SNAP_MM` 4.
   A jump >= 2 mm = target lost: lift and restart, do not continue on a guess.
f. **Tighter alignment, Jacobian guard.** Align at SAFE_MM to <= 0.5 mm (not `XY_OK_MM` 2.0). If the
   error grows two steps in a row, reset the Jacobian to the Step 1 seed (Broyden drift guard).

Optional but cheap: run the changed loop first against a small offline simulator (pinhole camera, the
Step 1 Jacobian, injected deadband, backlash, read-back lag, hidden dot below ~3 mm), so each change is
seen to work before it reaches the arm.

## Step 3 — dry run at hover, nothing touched (plan rung D1)

`aim_touch.py` with the descent stopped at **3 mm** above the paper, on 5 dots.

Pass: alignment <= 0.5 mm on 5/5 in the arm camera, and in the log the last steps have `seen = true`.
Also note the lowest height where the dot is still re-found: below that the touch is blind.

**Second-view check (UGREEN):** at the 3 mm hover, grab a UGREEN frame. Find the tip (`ugreen_tip`) and
the dot, both on the sheet (UGREEN is calibrated to the same sheet frame), and compare the tip-to-dot
offset with what the arm camera claims. Pass: they agree within 1 mm. If they disagree by a consistent
amount, the arm camera's landing point is biased: go to Step 5.

## Step 4 — contact, gently (plan rung D2)

From the 3 mm hover: descend 0.5 mm per step along the xy-neutral direction, no xy corrections once the
dot is unseen. Stop on the first of:
- needle push-back along the needle line (`back_px`, threshold from the touch-off run), or the shadow
  meeting the tip;
- a sideways jump of the tip pixel between steps (the "V-curve" contact sign used in micro-injection);
- model height <= -0.5 mm (safety floor; replaces the -2 mm press).

Then lift with the commanded pose (Step 2a).

## Step 5 — verify every touch and measure the miss (QPix style)

Give each touch a mark that shows on the overhead photo: dab the needle tip with a coloured marker
(red on black dots), or let it pierce thin paper/tape and image it with the back lamp. After the lift:
- UGREEN photo of the sheet; find the mark and its dot in sheet mm; **miss = mark - dot**.
- Log a row per touch: dot, final arm-camera error, UGREEN hover error, miss (x, y), contact signal.

Over 5 touches:
- **Mean miss** = landing-point bias. If |mean| > 0.5 mm, add it as a fixed offset to the aim point,
  like a pin offset in commercial pickers. Re-measure after every new needle grasp.
- **Scatter** (std) = repeatability. If > 1 mm, go back to Steps 1-2 (deadband/backlash/Jacobian).

## Step 6 — gate

5 dots, the offset from Step 5 applied. **Pass: <= 1 mm on 4/5.** Save the log, contact frames,
overhead before/after photos and the miss table under `evidence/needle_on_dot/<timestamp>/`.
Then milestone 1 is done and work moves on to plates (agar height plane) and self-grasp.

## Fallback: supervised teleop

If the automatic loop is not ready, get the same result by hand with `teleop.py` and the same rules:
step size >= 5, every joint finished in its fixed direction, align at ~8 mm, pause at 3 mm for the UGREEN
check, descend with elbow/wrist only, mark + measure each touch (Step 5). It gives a real accuracy number
and touch poses to replay, and its logs feed Step 1.

## Sources

- Hager, Chang, Morse (1995), robot hand-eye coordination based on stereo vision; Kim, Rizzi, Hager,
  Koditschek (1995), "A 'Robust' Convergent Visual Servoing System": https://repository.upenn.edu/ese_papers/654
- Navab et al. (CVPR 2000), uncalibrated needle placement by visual servoing:
  https://mlanthology.org/cvpr/2000/navab2000cvpr-visual
- Jägersand / Broyden image-Jacobian estimation and its limits: https://arxiv.org/pdf/2106.06083
- Logan, Dudash, Negrón (2026), low-cost 5-DoF colony picking, learned IK (~1 mm):
  https://arxiv.org/abs/2603.20465
- QPix pin-to-camera calibration (pierce tape):
  https://www.moleculardevices.com/sites/default/files/en/assets/user-guide/bpd/qpix-pin-calibration.pdf
- Contact detection in micro-injection (lateral "V-curve"):
  https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/8846379
- Hobby-servo deadband and backlash: https://instructables.com/Servo-Deadband-Correction,
  https://forums.parallax.com/discussion/comment/1318164
