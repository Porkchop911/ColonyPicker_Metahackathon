# Second opinion (Meta AI), 2026-10-31: camera-guided needle into wells

Asked with the request in this session (setup, measured errors, failures, plan a-c). Verdict, condensed:

- Plan direction right; first fix three things: a 25 cm needle is the main error amplifier (1 pulse ~ 1 mm at the
  tip), the tip must be optically unambiguous (durable markers, not a learned detector), and the final approach must
  never reverse a joint (backlash, not gain, makes the 2-5 mm limit cycle).
- 1 mm as absolute accuracy: not realistic now. As a plane-corrected image measurement at <= 2 mm over the rim,
  with every joint arriving from its own fixed direction: plausible after shortening the needle.
- Parallax: at ~45 deg the overhead shifts the tip by ~ its height (10 mm up = 10 mm off). Use the overhead ray cut
  with the plane at the tip's height (height from the side camera), compare with the well centre; the correction
  vanishes at the rim.
- Detection: lighting first (lock exposure, cross-polariser, dark card so the needle reads dark, backlight the side
  view); two marker sleeves (~15 and ~45 mm above the tip, heat-shrink or fluorescent + UV), fit the needle line and
  extrapolate the tip; hard gates: shaft line, epipolar, frame-to-frame continuity; rise and reacquire on loss.
  SAM not first. Background difference is invalid while the arm's shadow is in frame.
- Control: wrist frozen in the final phase; pan/shoulder/elbow, steps <= 2 mm down to ~0.5 mm, gain ~0.4, settle +
  median of frames, no integral action, no reversal (overshoot -> retract 10-15 mm and re-approach); align at
  <= 2 mm over the rim, then ~4 mm straight down.
- Mechanics outrank software: needle 80-100 mm in a rigid clamp near the wrist axis, near-vertical; bolt the base,
  corner-fixture the plate; use the 40 mm base move toward the plate.
- Audit first: taught poses replaying ~10 mm off (something moves between runs?), the static measurement noise
  floor, per-pose repeatability (20x) and hysteresis per approach direction (10x each), the tip detector.

Open question back: what limits the needle length (collisions at ~90 mm?) and its diameter.
