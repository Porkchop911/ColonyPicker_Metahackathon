# Support request to Hiwonder — ROSOrin Pro arm/servo behaviour

**Product:** Hiwonder ROSOrin Pro (mecanum base, 5-DoF arm, servos 1–5 + gripper 10, RRC board)
**Use:** precision needle positioning (target: first contact within 1–2 mm at the tip)
**Status:** draft — fill in the bracketed fields from the robot before sending.
**Evidence base:** `docs/rca_touch_dot.md`, `docs/handover.md`, `docs/plan_2026-10-28.md` in this repo.

---

Subject: ROSOrin Pro — shoulder servo will not rise under load, large deadband/backlash,
undocumented torque/telemetry behaviour, and stack behaviours that move or stall the arm

Hello Hiwonder support,

We are using a ROSOrin Pro for a research prototype and have hit several arm behaviours we
cannot explain or work around from the published interface. Everything below was measured on
the robot (commanded vs. read-back pulses, tip motion measured by external cameras); our own
software bugs have been separated out and are not part of this request. We would appreciate
answers to the numbered questions, and any firmware/driver updates that address them.

Robot details:

- Serial / order number: [ ]
- Firmware / stack version: [ `cat ~/hwr/...` — record actual version ]
- Battery during tests: 12.2 V, servos 26–30 °C (normal), unless stated otherwise
- Servo IDs: 1 = pan/base, 2 = shoulder, 3 = elbow, 4 = wrist, 5 = roll, 10 = gripper
- Pulse scale: 0–1000 pulses = 240°, centre 500 (0.24°/pulse)

## 1. Servo 2 (shoulder) will not rise under load — highest priority

Measured behaviour, repeatable over 2026-10-27…30:

- Servo 2 settles a fixed amount short when **rising**: ~11 pulses short near the start pose,
  17–26 pulses short at full reach. Falling, it settles only 2–3 pulses short.
- At long reach it **refused to rise at all**: commanded 285→300, read-back stayed at 279; only
  downward moves worked. A power cycle restored rising.
- Read-back sits ~5 pulses below the held position and lags one step behind, so re-sending
  read-back as a command sinks the joint ~5 pulses per command (we now never do this).
- Once, a 1-pulse correction on servo 2 dropped the tip from 10.8 mm to 1.9 mm above the paper
  (a stored slip releasing).
- A related incident (2026-10-30): all servo commands were accepted but **no joint moved for
  ~1 minute** (read-back frozen, shoulder at 231), then it recovered by itself. Cause unknown.

Questions:

1. Is this the expected torque limit of the shoulder servo at reach, a firmware current/torque
   limit, or a sign of a failing servo? What is the rated holding/stall torque, and the
   recommended maximum reach/load for continuous positioning?
2. Is there a supported way to read per-servo **temperature, input voltage, load/current, and
   torque-limit setting**? We found references to `servo_temp` / `servo_vin` in
   `hwr_core/rrc/board.py`; are these the supported calls, and what are the safe limits?
3. Why would rising be blocked entirely (down still working) until a power cycle — thermal
   cut-out, over-current latch, or something else? Is there a status flag we can poll instead
   of discovering it from missed motion?
4. Is the ~11–26 pulse rise-shortfall a known compliance/backlash figure for this arm, and is
   there a factory calibration or compensation procedure?

## 2. Deadband, backlash, and stick-slip on all arm joints

Measured (tip motion by external camera, `docs/needle_on_dot.md`, `docs/rca_touch_dot.md`):

| Joint | Deadband | Backlash on reversal | Other |
|---|---|---|---|
| 1 (pan) | 2 pulses (2-pulse steps: no motion, every time) | ~5 pulses swallowed, then a jump | stick-slip: 1 pulse = 1–4 mm at the tip; "+3 pulses moved 16 mm, −5 moved 3.8 mm" |
| 3 (elbow) | 1-pulse steps sit in the deadband | — | lags command by 11–14 pulses under load, read-back a step late |
| 5 (roll) | 1–2 pulses | ~0.5 mm at the tip | 0.26–0.42 mm/pulse at the tip |

Questions:

5. What deadband and backlash values does Hiwonder specify for these bus servos, and are ours
   (above) within specification or a warranty case?
6. Can deadband be reduced, or backlash compensated, through servo registers or the RRC
   firmware? If registers exist, please share the register map / protocol commands.
7. Is there a recommended motion discipline (approach direction, minimum step, speed/duration
   settings) that minimises the pan stick-slip jumps?

## 3. Servo protocol / RRC firmware limits we probed

- The servos appear to speak the LX-16A-family bus protocol (e.g. `SERVO_OR_MOTOR_MODE`,
  cmd 29, exists at the silicon level), but the RRC (STM32) firmware seems to forward only a
  fixed subcommand vocabulary: raw payload probes for motor mode (writes `[29, 5, 1, 0, 100, 0]`
  and variants; mode-read guesses) produced no reply and no motion
  (`tools/roll_flip.py`, probe of 2026-10-31). In position mode a servo can reach at most
  ±120° from centre, so a true 180° re-orientation of servo 5 is impossible without motor mode
  or mechanically re-indexing the horn.

Questions:

8. Please confirm which bus-servo commands the RRC firmware forwards, and provide the full
   supported command/protocol list for the RRC board. Is motor (continuous-rotation) mode
   reachable at all through the stock firmware?
9. Is there newer RRC firmware, or a documented raw-passthrough, that exposes more of the
   servo protocol (mode, torque on/off, telemetry reads)?

## 4. Torque on/off semantics

For hand-teaching we added a `servo_torque` path: position 0 = limp, 1 = hold
(`robot/hwr_driver_node.py:143-147`, deployed 2026-10-30). Observed: after going limp, the
board's "torque on" command (0x0B) **left the servos limp**; only a fresh position command
re-enabled holding torque.

Questions:

10. What is the correct command sequence to disable and re-enable torque on these servos?
    Is 0x0B the right command, and does re-enabling torque hold the *current* (hand-moved)
    position or snap back to the last commanded position?

## 5. Stack behaviours that move or stall the arm

These are in the shipped ROS 2 stack as deployed on our robot:

- **`sensor_supervisor_node` restarts the arm camera (Deptrum Aurora 930) every ~16 s** and
  returns after every stack restart. It wrecked camera latency and aborted runs (arm-camera
  stalls of ~10–15 s; our snapshot path now refuses frames older than 1 s). We stop it by PID
  after every restart.
- **`sudo systemctl restart hwr` moved the parked arm** (shoulder 900 → 725) on restart —
  a safety concern with a needle in the gripper. Restarting is only safe with the arm supported.
- (Historical, already removed by us:) the e-stop latched at every boot and **silently dropped
  servo commands**.

Questions:

11. How is `sensor_supervisor_node` meant to be disabled permanently, and why does it restart
    a healthy camera on a ~16 s cycle? Is there a fixed version?
12. On stack/driver start-up, what servo commands does the driver send, and how do we make
    start-up/restart leave the arm exactly where it is (no motion on restart)?
13. Is the Aurora 930's USB drop-out under hub load a known issue, with a recommended
    hub/power arrangement?

## 6. What we have already ruled out (our side)

- Commanding from read-back (sinks servo 2 ~5 pulses/command) — fixed in our tools; all
  commands now build on the commanded pose.
- Battery: failures above occurred at 12.2 V / 26–30 °C, not only at low charge (two earlier
  battery shutdowns did separately cost servo 2 its ability to rise until a power cycle —
  see question 3).
- Our camera/vision errors are tracked separately (`docs/rca_touch_dot.md` §5) and are not
  part of this request.

## 7. What would help most, in order

1. A way to read servo temperature/voltage/load and any fault latch (Q2, Q3), so "will not
   rise" is diagnosable instead of silent.
2. Confirmation of whether the servo 2 rise-shortfall and refusal are spec, configuration,
   or a failing unit — and an RMA/replacement path if the latter (Q1, Q4).
3. The RRC forwarded-command list and any firmware with motor-mode / fuller telemetry (Q8, Q9).
4. Correct torque off/on sequence (Q10).
5. Permanent opt-out for `sensor_supervisor_node` and no-motion-on-restart behaviour (Q11, Q12).

We can supply logs on request: commanded vs. read-back traces, run logs under `evidence/`,
and the probe tool (`tools/roll_flip.py`) used for §3. Short test scripts against a driver
stopped/started as you advise are no problem.

Thank you,
[ name / organisation / contact ]
