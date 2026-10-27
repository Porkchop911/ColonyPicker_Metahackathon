# Root cause analysis: why the needle does not reliably touch a chosen dot (2026-10-27)

## 1. What was asked, what happened

**Goal:** the needle tip makes its *first* contact with the paper on a chosen black ink dot on the
notebook (about 2-3 mm across), automatically, twice in a row.

**Outcome after one day:** about 95 recorded runs across four tools (`aim_touch.py`, `jab.py`,
`model_jab.py`, plus teleop sessions), 14 of them ended in a contact with the paper, and **none is a
verified clean first contact on the dot.**

| Tool | Runs with a log | Contact | Main failure |
|---|---|---|---|
| `aim_touch.py` (arm-camera servoing, UGREEN finish) | 21 | 3 | target lost (6), UGREEN lost tip/dot (5), grid pose failed (2) |
| `jab.py` (learnt fluid jab, UGREEN placing) | 37 | 5 | placing sank to the paper or below (6), tip/dots lost (7), learning only (10) |
| `model_jab.py` (URDF model, SAM 3.1, both cameras) | 49 | 6 | placing did not converge (8), camera tie failed (5), placing move refused (5), code errors (2) |

The two runs reported as "on the dot" (`mjab/20261027T224605`, `225117`) were judged from a UGREEN photo
taken **after** the move had ended. The user watched one of the final attempts and saw what the photo
cannot show: **the tip touched down about 5 mm short and dragged across the paper to the dot.** So the
success criterion I used (where the tip ends) was the wrong one; the goal is where it *first* touches.

The causes below are ranked by how much they explain. Each has the evidence from the logs.

## 2. Root cause 1: the final centimetre is executed blind

**Every contact today was made by a motion that no sensor watched while it happened.**

| Version | Final motion | Watched during the motion? | Result |
|---|---|---|---|
| `aim_touch` | elbow/wrist steps down, "blind below 3 mm" by design | no: the dot hides under the tip in the arm camera | landed 10-25 mm off, or pressed |
| `jab.py` live | one fluid move (shoulder -30, elbow +14, wrist +30) | no | 4.4 mm off; one 16 mm forward instead of 10 |
| `model_jab` jab | one fluid model move, then 8 waypoints | no | 3 mm short; 0.4-1.1 mm "miss" at the end but touched down ~5 mm early and dragged |
| `model_jab` angled needle | 8 waypoints | no | 9 mm short, pressed 6.7 mm into the paper |

Why it is blind:

- **The arm camera cannot see the target at contact.** The needle is rigid to the camera and points
  along the camera's view, so when the tip is over the dot, the dot sits exactly behind the tip in the
  image (`aim_touch` "hidden" logic, `HIDE_MM`). Measured: the dot vanished from detection at 6-8 mm and
  below in every arm-camera run.
- **The UGREEN sees the tip and the dot together, but my code only read it between moves.** The
  camera can be read at any time (user); not reading it during the final motion was my design choice,
  not a limitation. Its tip finder needs motion between two frames (`moved_needle`) or a SAM mask; SAM
  3.1 does not segment the thin needle at the UGREEN's image edge ("thin metal stick", "needle", "pin":
  0 masks). Read continuously during the descent, the motion itself would have shown the tip.
- **No contact sensor.** Push-back of the tip along the needle line (`back_px`) worked with the
  straight needle, but faked 5-9 px from a dot beside the tip (two false stops), and the tip finder
  behind it fails on the angled needle. Servo load (read-back lag) was added late and is noisy (8
  pulses of "load" on a reversal with no contact).

**Consequence:** the end pose can be measured, the path cannot. A path that touches early and drags
looks the same as a clean touch in every number I logged.

## 3. Root cause 2: the arm does not go where it is commanded

The plan, the model and the jab all assume commanded pulses become joint angles. Today's measurements
say otherwise, by 3-25 mm at the tip:

| Joint | Measured behaviour | Evidence | Tip effect |
|---|---|---|---|
| **Servo 2 (shoulder)** | sags under load and **refused to rise** (commanded 285-300, stayed 279; only down worked) until a power cycle; slips mid-run | read-back 272 for 280 commanded; one 1-pulse correction dropped the tip 10.8 -> 1.9 mm (the hard hit on the paper); 76 mm reach-back runs | 5-15 mm drops, the early touchdowns |
| **Servo 3 (elbow)** | lags its command 11-14 pulses under load, read-back a step late | `jab.py` probes: commanded 300 read 289, 315 read 301 | 5-20 mm height error per move |
| **Servo 1 (pan)** | stick-slip and slack: 1 pulse = 1-4 mm at the tip, a reversal swallows 0-5 pulses then jumps | 513 <-> 520 oscillation (17 mm per 7 pulses); 541 <-> 548; "+3 moved 16 mm, -5 moved 3.8 mm" | sideways cannot be set finer than ~2 mm |
| **Servo 4 (wrist)** | follows, but elbow and wrist work against each other along the needle | `jab.py` trials: elbow -10 = 8 mm back / 10 down, wrist +10 = 7.6 forward / 7.6 up; 10/10 forward/down needs (-60, +75) | the jab needs the weak shoulder |
| **Base strafe** | not repeatable: the same 0.15 s pulse moved 0, -3.3, 0, 0, 0 and once +18 mm; speeds 6-51 mm/s between runs | strafe probes, `model_jab` logs | sideways corrections overshoot or do nothing |

Why it matters for the jab: a fluid move that is a straight line in the model is a curve in reality,
because the shoulder and elbow fall behind (sag, lag) and the pan sticks. The tip comes down before it
goes forward, touches the paper short, and the rest of the move drags it.

Contributing: the **gripper does not hold the needle rigidly** under contact. The tip depth was
202.5 mm in the morning, 206 mm after the gripper reopened, 188 mm after the angled re-mount; each hard
press can shift it again, and every height and every model move depend on it.

## 4. Root cause 3: the targets were where the arm is weakest

The nearest black dot was usually 45-126 mm from the needle's natural landing point at the start pose:
76 mm back toward the robot, 100-126 mm to the side. Reaching there needs big shoulder moves (-80 pulses
planned) at full stretch, exactly where servo 2 sags and stops rising (handover: "Servo 2 struggles when
the arm reaches low and far"). The one target that was close to the natural reach (the dot ~1 cm from
the hover at 22:46) is the one that came closest.

## 5. Root cause 4: measurement fragility outside the final centimetre

Even before contact, a large share of runs died on sensing, not on motion:

| Failure | Runs | Cause |
|---|---|---|
| Target lost by the arm camera | 6 (`aim_touch`) + 3 (`jab.py` probes) | prediction window vs. 3-25 mm motion errors (root cause 2) |
| UGREEN lost tip or dot | 8 | needle found only as "what moved"; small moves invisible; dusk; outliers 35-130 mm |
| Two-camera tie failed | 5 | SAM finds no dots in the arm view above ~30 mm; the cube leaves the view at the tie height; the cube centre is 17 mm off from the oblique view |
| Grid pose failed / wrong | 4 + one at -96 mm | little grid in view after big moves; one pose fit put the tip 96 mm under the paper |
| Target carried wrongly between frames | 1 visible (50 mm jump) | `lattice_offset` matched the wrong mark pattern |
| Wrong object as "black dot" | 1 | a mark on the white UGREEN box passed the hand-made dot filter; the needle was driven to the box |

## 6. Root cause 5: how I worked (process)

These are my failures, not the hardware's, and they multiplied the cost of everything above.

1. **Live trial and error instead of offline tests.** About 50 code changes went straight onto the
   arm. Only `aim_touch` had a simulator; `jab.py` and `model_jab.py` had none, and the recorded frames
   were never replayed through the new code. Bugs that a replay would have caught went to the arm:
   a sign error on the strafe direction (twice), a wrong seed sign for the jab bias, the shoulder
   solved but not sent or checked, a `NameError`, a `TypeError` in logging, a 0.6 s strafe the gateway
   rejects (<= 0.5 s per pulse), "rightmost" taken from the wrong camera.
2. **Safety checks added after the damage.** Contact/floor checks ran only in the descend phase, so an
   align step drove the tip to -6 mm; the floor check used the model, not the measurement, so the
   placing move went to 1.9 mm; a probe used the model's height scale already known to be 5x off and
   dropped 13 mm into the paper. The needle hit the paper hard at least four times.
3. **The wrong success measure.** Final-pose miss from the arm camera and an after-the-fact UGREEN photo
   instead of first-contact position. I reported "touched, 0.75 mm" when it had dragged.
4. **Hand-made detectors where SAM 3.1 was required** (until the user insisted), which picked the box
   mark and missed dots at dusk.
5. **Changing several things per run**, so a failed run rarely told which change was wrong, and
   abandoning working pieces (UGREEN finish, learnt jab) for new approaches mid-way.

## 7. Root cause 6: environment and setup

- Light fell through the afternoon: thresholds tuned at noon failed at dusk (UGREEN paper 180 -> 65).
- The sensor supervisor restarted the arm camera every 16 s and came back with every stack restart
  (teleop latency, stalls).
- The e-stop latched at every boot and dropped servo commands silently (removed).
- Two battery shutdowns; servo 2 lost its ability to rise until a power cycle.
- The base drifted from today's strafes, so the start view changed between runs.

## 8. Why it fails, in one paragraph

The last centimetre before contact is executed open-loop by an arm whose joints land 3-25 mm away from
where they are commanded, worst at the shoulder and worst at the long reach the chosen dots required.
No sensor watched the tip and the dot during that centimetre: the arm camera has the dot hidden behind
the tip, the UGREEN could have but my code read it only between moves, and the contact detector was
unreliable. So the tip touches
down early and drags, and the end-pose measurement I used hid it. Around that, fragile detection and
untested code changes made most runs fail before they got there.

## 9. What would fix it (in order)

1. **Measure first contact, not the end pose.** A contact signal independent of the cameras: a
   conductive path (needle wired, a thin foil under the sheet, contact closes the circuit), or servo
   current/load read at high rate during the descent. Stop on the first contact and record *where* it
   happened.
2. **Watch the last centimetre.** Put the UGREEN (or a second cheap camera) low and to the side of the
   target, so the tip, its shadow and the dot are visible during the descent. Descend in small measured
   steps (0.5 mm) with this view correcting sideways and along, not as one move.
3. **Fix or work around servo 2.** Read its temperature, voltage and load with the driver stopped
   (`hwr_core/rrc/board.py` `servo_temp`, `servo_vin`); check its torque limit; reduce its load
   (counterweight, shorter reach). Until then, no motion that relies on the shoulder rising.
4. **Put the target where the arm is strong.** Place the notebook so the target dot lies within about
   20 mm of the needle's natural landing point at the start pose; no 76-126 mm reaches.
5. **Hold the needle rigidly** (a clamp or a collar in the gripper) and re-measure the tip depth after
   every grasp change, triangulated from both cameras.
6. **Process:** a replay harness that runs every new version on recorded frames first; one change per
   live run; a fixed run table (attempt, change, first-contact position, miss); safety checks on
   measured values only, before any new motion is tried.

## 10. What works and can be kept

- SAM 3.1 two-camera calibration on the black dots and the red cube (3 dots matched, 180 deg).
- URDF model for the large placing move (91 -> 2.3 mm sideways in one move), with measured corrections.
- Tip depth triangulated from both cameras (rays agree to 0.9 mm).
- Hover over the target to ~1 mm, measured, repeatedly (22:22, 22:46, 22:51, 23:13).
- Commanded-pose tracking (no re-basing on read-back), e-stop removed, gateway limits understood.
