# Write-up: camera-guided needle into the 96-well plate, 2026-10-31 / 11-01

Branch `well-reach-v2`. Tool `tools/well_servo.py`. Goal: `docs/goal_plate_well_reach_2026-10-31.md`
(needle tip into the wells, ~20 wells, photo proof). Previous attempt: `docs/implementation_plate_well_reach_2026-10-31_FAILURE.md`.

## Result

| | |
|---|---|
| Wells entered, camera-guided, with photos | **2**: F6 (old plate place, 1.67 mm off centre 2 mm over the rim) and E12 (new place, 0.94 mm) |
| Overhead check after entry | both seen in the right well (`which_well`) |
| Live runs of `well_servo.py` | 16, most of them 0 wells |
| Goal (~20 wells) | **not met** |

Proof: `evidence/tip/servo_F6_proof.jpg`, `evidence/tip/servo_E12_proof.jpg`, `evidence/tip/well_servo.json`
(new plate place), `evidence/tip/well_servo_oldplate.json` (before the plate moved), logs `evidence/tip/well_servo_run*.log`.

## What works (keep)

- **Plate and cameras from the paper grid** (`tools/plate_rig.py`). After every move of the paper / plate: full solve,
  then check A1 by eye (the solver can mirror the rows; fix by flipping rows). Rays of a hand-picked tip met at 0.11 mm.
- **Vertical needle**: vertical when shoulder + elbow + wrist = 1157 pulses.
- **Tip by the teal bands**: the lower band is 15.3 mm above the tip, the upper band 28.5 mm above the lower.
  Overhead band ray cut with the plane of the side camera and the side needle line; tip on the axis through both bands.
  Repeat frames agree to ~0.1 mm; on saved frames it agrees with hand-picked / triangulated tips to ~1 mm.
- **Controls near the plate**: pan (sideways), shoulder (mostly height), wrist (reach); elbow held. Every move ends
  from 8 pulses above (backlash). Typical Jacobian (mm/pulse, columns pan / shoulder / wrist) in `J_PRIOR`.
- **Offline replay test** `tools/test_well_servo.py` on saved frames (started late; should have come first).

## What is wrong with the last sessions

These are process failures more than code failures. Most of the 16 live runs failed on something an offline check
would have caught.

### 1. Tuning by live run
Each change was checked by the next live run of the whole pipeline (~5-15 minutes, arm moving). A failure deep in one
well (lost tip) then cascaded into every later well, so one run showed one bug. A replay test on saved frames
existed only from run 8 on, and covered only the case just fixed.
**Instead:** collect frames over the working area first (a grid of poses at safe height), build the detector against
them offline, add each live failure's frames to `test_well_servo.py` before fixing it.

### 2. Too many fragile perception heuristics, layered
"Lowest point in both views", free needle-end pairing, hue thresholds, window sizes, fallbacks between them. Each was
added to fix one run and broke another:
- free-tip pairs of junk (robot wheel, plate rim, arm cable) meet within 0.1-1 mm, as well as the real tip does: the
  ray-meet distance does not tell the needle from junk;
- the background ghost of the needle (needle visible in the background frame) was picked as the tip, twice;
- two teal bands: the upper one was taken for the lower one (tip 28 mm wrong in height);
- the lower band merges with the shaft when the hue threshold is widened.
**Instead:** one measurement (bands + side line), with geometric checks (band on the side line, partner band at the
known spacing), and a clean background frame with the needle out of both views (check it by eye after any change).

### 3. The rig changed and nothing checked it at run start
Between runs the plate moved (closer to the robot), the robot base (mecanum wheels) moved ~80 mm, a black mat was
added, the lamp went from full to 110. Each broke a constant: the start pose `OVER_PLATE`, the background pose
`BG_POSE` (needle ghost now in view), the start search window, the camera-check threshold (`CHECK_MIN`), exposure (the
overhead at lamp 254 washed the plate out, so the E12 photo is weak).
**Instead:** a start-of-session check: both camera frames, plate labels drawn, the background frame shown with the
needle out of view, the needle found at the start pose. Five minutes, no motion beyond the background pose.

### 4. Safety: the arm driven into the needle
After a break the arm stood folded (near the old PARK). The user wrote "this is not park pose any more"; Claude read it
as approval and drove it toward the background pose. The move jammed the needle. The user had said three times that
PARK / folding hits the remounted needle. PARK now has wrist 500 (`well_reach.PARK`, `arm_park.py`).
**Rule now:** if the arm is not where the last commanded state says, or is folded (shoulder > ~600 or wrist < ~300),
send nothing and ask; an ambiguous answer about safety means ask again.

### 5. The controller made errors that lasted
- The Jacobian was probed per well (±12 pulses); one bad probe reading (shoulder at half its value) made the next
  step drop 15 mm. Now columns far from `J_PRIOR` fall back to it.
- Long descents (80 mm) drifted sideways (xy error 4 -> 16 mm): steps were nearly all height. Now xy is corrected
  first while it is more than two gates off.
- A lift that lost the tip went on by prediction; the next well started from a position 30 mm wrong, and all later
  wells failed. Now each well starts from the last *measured* tip, else a search.
- Floor refusals aborted the well instead of taking a smaller step. Now halved up to 4 times.

### 6. Avoidable slips
- Ran with the system Python instead of the robot venv (no `sam3`): one run lost.
- Taught (hand-guided) poses were replayed into wells without a camera check first; they landed 1-2 wells off.
- Streamed every reading to the user while waiting (token waste; the user had to say stop).
- Disagreed with the user about an elbow range sweep instead of doing what was asked.

## What to do next

1. Start-of-session check (point 3) before any well run.
2. Record a frame set over the plate at 30, 15, 8, 4 mm at several wells, with the needle position confirmed by eye on
   a few; make the detector pass all of them offline.
3. Keep the plate where the arm is mid-reach, not stretched (with the arm stretched, shoulder and elbow only move the
   tip up and down).
4. Run wells one at a time until 3 in a row go in; then batches.
5. Proof: overhead `which_well` plus a photo where the well is visible (lamp ~110, not 254).

## Run log (new plate place, 2026-10-31 23:30 onward)

| Run | Wells | In | What stopped it | Fixed in |
|---|---|---|---|---|
| 8 | C4.. | 0 | system Python; camera check 150 crossings under the mat; arm folded after the break (then the jam) | venv path; `CHECK_MIN` 30; PARK wrist 500 |
| 9 | E12.. | 0 | start pose now 80 mm off (base moved); side line lost near the plate (horizontal edge fused with the needle) | free start search; vertical opening |
| 10 | E12.. | 0 | background pose left the needle's ghost in the side view; probes took it for the tip | `BG_POSE` shoulder 560 |
| 11 | E12.. | **1 (E12)** | D12 floor refusal aborted; D11 bad shoulder probe -> 15 mm drop | step halving; `J_PRIOR` check; lamp 110 |
| 12 | D12.. | 0 | start search picked the robot's wheel | search only over the plate |
| 13 | D12.. | 0 | band blurred over the wood high up (lamp 110) | free-tip fallback (later removed) |
| 14 | D12.. | 0 | descent drifted sideways 4 -> 16 mm | xy first |
| 15 | D12.. | 0 | free-tip fallback accepted a junk pair; every later well started 30 mm wrong | bands only, hue from 22 when none found, band on the side line |
| 16 | D12..B12 | 0 | D12 shoulder probe lost; D11 not within 3 mm at 30 mm; E11 reached 4 mm but not within 2.5 mm; then the pan probe lost at every later well (same cascade as run 15). Ended at {597, 153, 173, 922}: stretched, needle ~18 deg off vertical | open: point 1-2 of "What to do next" |
