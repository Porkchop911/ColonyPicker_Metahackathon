# Arm movement accuracy — deep analysis (2026-10-30)

Status: reasoning only, no code changed. Companion to
`docs/plan_speed_accuracy_2026-10-30_AGREED.md` (the agreed speed+accuracy plan).
Question addressed: how to improve arm movement **substantially** in terms of accuracy,
beyond tuning the constants of the current placer.

## Bottom line

The largest accuracy gain will not come from tuning `GAIN`, `PAN_BACKLASH`, settle times or tolerance constants. The current placer in `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py` is still fundamentally **model-led**: `D:/robot/ColonyPicker_Metahackathon/tools/coarse_aim.py:59-75` solves inverse kinematics against the URDF-style model, `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:401-402` reads the tip through one rigid `tip_cam`, and the arm-move branch at `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:948-985` corrects mainly along `fwd` with fixed `GAIN = 0.35` from `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:88`. That architecture can remove repeatable model bias, but its ceiling is set by backlash state, gravity sag, pan scatter, tip-depth error and arc/contact behaviour.

The substantial improvement is to make the live loop **measure-led**:

1. lock the target once in sheet coordinates;
2. estimate tip state continuously in sheet coordinates;
3. learn the local pulse-to-tip Jacobian by probing and Broyden/RLS updates;
4. move all relevant joints together with a damped, backlash-aware controller;
5. approach the final point from one canonical direction;
6. treat the final wrist arc and ink contact as a separate measured landing problem.

The repo already contains most of the primitives: `D:/robot/ColonyPicker_Metahackathon/tools/tip_descend.py:71-96` has a sheet-frame `Probe.state()` returning `(x, y, height)`, and `D:/robot/ColonyPicker_Metahackathon/tools/dot_servo.py:81-121` already tracks one chosen dot instead of re-acquiring targets. The direction should be to promote that measured-Jacobian style into the main placing/descent path, not to keep improving the model-only path indefinitely.

## Why the current placer has an accuracy ceiling

### 1. Pan is too coarse and too variable to be a final-accuracy actuator

`D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:75` records the problem directly: `PAN_SIGMA_MM = 4.0`, with realised pan response spanning roughly `-0.8..19.5 mm/pulse` over logged steps. Roll is finer — `ROLL_MM = 0.42` at `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:72` — but has `ROLL_DEADBAND = 1.5` pulses at `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:73`, i.e. about `0.63 mm` of command can disappear before travel starts.

That means a 1 mm-class final correction cannot be allocated naively across whichever joint the model chooses. Near the target, pan should normally be frozen unless the sideways error is large. Fine correction should be allocated preferentially to roll for small across-error and elbow/wrist for along/height, with pan treated as a coarse, high-uncertainty actuator.

### 2. Sequential single-axis correction fights coupling

The present place loop separates concerns: roll handles small sideways error, pan handles large sideways error, strafe handles some sideways cases, and the arm move handles mainly along/height. But real arm moves are coupled: an elbow/wrist correction changes across as well as along; a pan correction changes height and along; roll changes the tip ray geometry. Correcting one component at a time can therefore chase its own tail, especially when each branch has different deadbands, reversal rules and gains.

A multivariable step is more principled: compute one joint increment from the full measured error vector using a local Jacobian, with joint-specific costs. That is the difference between "fix along, then fix across, then discover along moved" and "choose the least-bad combined move under the current actuator uncertainties".

### 3. The model does not contain the physical error states that matter

`D:/robot/ColonyPicker_Metahackathon/tools/hwr_kinematics.py:142-154` builds the camera pose from the kinematic chain only. The physical plant also contains:

- gear backlash and dead zone;
- direction-dependent hysteresis;
- shoulder/elbow deflection under gravity;
- servo holding error under load;
- needle bending and mount flex;
- camera bracket flex;
- base/chassis movement;
- paper height variation;
- thermal and supply-voltage drift.

Those effects are configuration-dependent. This matches the recent configuration-dependent calibration literature: local POE models interpolated over shoulder/elbow configuration reduced maximum positioning error by more than 50% to sub-millimetre levels on industrial robots in arXiv:2510.19962. The lesson is not "copy that exact model"; it is that **constant offsets are the wrong model class** for an arm whose realised gain already varies strongly with reach.

### 4. The existing joint calibration is not in the live path

`D:/robot/ColonyPicker_Metahackathon/tools/arm_calibrate.py` fits shoulder/elbow offsets and writes `D:/robot/ColonyPicker_Metahackathon/evidence/tip/arm_offsets.json`, but nothing else in `tools/`, `docs/` or `robot/` reads that file. So calibration currently improves knowledge, not motion. Wiring it in would help the first model move, but by itself it will not solve backlash state, sag during the arc, or contact drag.

### 5. Hover accuracy is not mark accuracy

Even a perfect hover estimate does not guarantee a perfect ink mark. The final sequence in `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:1128-1233` rotates the wrist in 3-pulse steps, then takes up to `CONTACT_EXTRA = 6` one-pulse steps from `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:92`. The code itself notes that each pulse past contact drags the tip about `0.76 mm`. So the mark error contains at least three separate terms:

\[
e_{\text{mark}} = e_{\text{place}} + e_{\text{arc landing}} + e_{\text{contact/drag}} + e_{\text{ink centroid}}
\]

Improving only `e_place` will stall if the arc extrapolation or post-contact drag dominates.

## The highest-value redesign: measured-Jacobian visual servoing in sheet coordinates

### State first, commands second

Maintain an explicit estimated state:

- target position in sheet coordinates, fixed after the initial tie;
- tip position `(x, y, h)` in the same sheet coordinates;
- covariance/uncertainty for both;
- per-joint last direction and backlash state;
- current local Jacobian estimate and its confidence.

The target should not be re-identified from scratch after every move. Cross-camera dot pairing is the fragile operation; same-camera lattice/blob motion is the robust one. `D:/robot/ColonyPicker_Metahackathon/tools/tip_motion.py:60-73` already implements `lattice_offset()` for whole-square sheet shifts, and the probe in `D:/robot/ColonyPicker_Metahackathon/tools/tip_descend.py:71-96` avoids dot pairing by cutting the UGREEN tip ray at the arm-camera grid height.

A robust estimator would therefore use:

- UGREEN for absolute target identity and periodic re-anchoring;
- arm-camera lattice motion for relative tip displacement between views;
- side/overhead triangulation for height and tip depth when available;
- innovation gating: a sudden multi-mm "target jump" is treated as a bad retie unless independently confirmed.

That directly addresses the observed retie flips: the controller should servo against a stable state estimate, not against whichever dot pairing won the latest frame.

### Probe the Jacobian, then update it online

At a new configuration, probe each candidate joint with small ± moves and measure the actual tip displacement in sheet coordinates. The probe size should be chosen from sensing noise: large enough that tip motion is at least ~5× the static measurement noise, small enough that the local-linear approximation holds.

Then update the Jacobian after ordinary moves using a guarded Broyden or recursive-least-squares rule:

- accept the update only if the commanded move was large enough to measure;
- reject outliers where the observed motion contradicts all plausible geometry;
- decay confidence when configuration changes a lot;
- re-probe when predicted-vs-realised error exceeds a threshold, e.g. 30–40%.

This is exactly the uncalibrated visual-servoing pattern in the literature: dynamic Broyden Jacobian estimation avoids needing an accurate robot/camera model, and newer work combines Broyden estimation with Kalman-style robust estimation and predictive control. For this robot, plain damped least squares plus a trust region is likely enough; full MPC would be premature.

### One MIMO move, with actuator-aware allocation

Replace separate along/across decisions with:

\[
\Delta q = (J^T W J + \lambda^2 D)^{-1} J^T W e
\]

where:

- `e` is the sheet-frame error `(along, across, height)`;
- `W` reflects measurement confidence;
- `D` penalises expensive or unreliable joints;
- pan gets a high cost near the target;
- joints in a backlash dead zone get temporarily reduced effective gain;
- reversal of a loaded joint costs more than continuing in the same direction;
- shoulder motion is penalised when it risks slip or large sag change.

Step size should be governed by a trust region learned from the last move: if the arm realised only 40% of prediction, shrink; if it realised 90–110%, grow cautiously. That replaces the fixed `GAIN = 0.35` compromise at `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:88`, which is simultaneously too timid in well-behaved configurations and too optimistic where the model is wrong.

### Stop on predicted mark uncertainty, not on a point estimate

The loop should stop when the **predicted landing distribution**, not just the point estimate, is inside tolerance. In practice: propagate tip covariance, Jacobian uncertainty, arc-landing uncertainty and backlash state into a predicted mark error; require e.g. the 95% landing radius to be below the gate before descending. If uncertainty is too high but the point estimate looks good, the correct action is another measurement or a Jacobian probe — not a hopeful jab.

## Backlash and hysteresis: treat direction as state

For low-cost geared servos, position is not a function of command alone; it is a function of command **and approach history**. The literature on dynamic dead-zone modelling makes the same point formally: backlash width can be asymmetric and position-dependent, and compensation needs the dead-zone state, not just a constant pulse offset.

Practical rules for this arm:

1. **Canonical final approach.** Always enter the final tolerance box from the same sheet-frame direction and from above. If the tip overshoots, do not immediately reverse one pulse; back out along the approach path far enough to re-take the gear slack, then re-approach.

2. **Per-joint backlash maps.** Measure dead zone by commanding ± steps at multiple configurations and observing actual tip/joint response with the cameras. Fit `b_j(q, direction, load-sign)` rather than one global `PAN_BACKLASH = 5` from `D:/robot/ColonyPicker_Metahackathon/tools/model_jab.py:54`.

3. **Take-up moves are part of the plan.** A reversal is not `+N` pulses; it is "move `-b_j` pulses to cross the dead zone, then move the desired amount, all while expecting no tip motion during the first part". The planner should know that the first pulses after reversal are intentionally non-productive.

4. **Freeze coarse joints late.** Inside a few mm, pan reversals should be almost forbidden. If across-error is too large for roll at that point, back out and re-plan the coarse pose rather than dithering pan around the target.

5. **Order targets to preserve approach history.** When touching multiple dots, visiting them in an order that minimises pan reversals and large configuration changes is an accuracy feature, not just a speed feature.

## Compliance and gravity: feedforward for the path, feedback for the point

Visual feedback can correct static sag at the measured pose, so compliance is not mainly a hover problem. It matters most:

- during the wrist arc, where configuration changes continuously;
- at contact, where reaction force adds deflection;
- when the controller predicts the result of a move before making it.

The right model is a lumped deflection law such as:

\[
\delta q \approx K(q)^{-1} \tau_g(q) + \delta_0 + \text{backlash state}
\]

You do not need true link masses and stiffnesses separately; identify the lumped products from camera-measured tip error over a designed pose grid. Add a configuration-dependent residual on top — Fourier terms in shoulder/elbow angles, local affine patches over the sheet workspace, or a small Gaussian-process residual — and validate it on held-out poses. The unified gravitational/elasto-geometrical calibration literature (IEEE 11020762; ScienceDirect S0736584523000340) supports identifying gravity terms from data rather than trusting CAD parameters.

Two cautions:

- Fit only what the measurements can observe. `D:/robot/ColonyPicker_Metahackathon/tools/arm_calibrate.py` already discovered that pan offset trades against base yaw and wrist offset against tip position. More parameters without better excitation will produce a prettier fit and a worse arm.
- Keep the residual model interpretable at first: joint zeros, tip vector, camera extrinsics, two or three gravity-compliance coefficients, backlash widths. Add flexible residual terms only after those are stable.

Hand-eye calibration belongs in the same bundle. The simultaneous nonlinear AX=XB formulation is the most noise-robust of the classical options (arXiv:2311.12655), and in this codebase the real requirement is broader: jointly estimate camera extrinsics, `tip_cam` including depth, joint zeros and sheet/base transforms by minimising reprojection and ray-meet errors over many poses. In particular, **tip depth must be re-measured during placing**, not only tip direction after roll; at oblique viewing angles a depth error projects directly into sheet XY error.

## The final arc deserves its own controller

The practice arc should not be one low measurement plus linear extrapolation. At the final XY, after placing converges:

1. step down the actual wrist arc through several heights, e.g. 12, 8, 4, 2 mm;
2. triangulate the tip at each height from overhead + side views;
3. fit the landing point where the fitted arc meets the local paper plane, with uncertainty;
4. compensate the along-bias from that fit before the real descent;
5. descend under side/overhead tracking, correcting across/height continuously;
6. stop on triangulated height plus tip-stillness, then retract along the needle/vertical before any lateral motion.

Local paper plane matters: the grid pose assumes a plane, but paper curl or tilt means the relevant height is height above the paper **at that dot**, not above the global fitted plane. The practice-arc fit should therefore estimate local contact height as well as landing XY.

Needle mechanics also matter. Keep the needle as normal to the paper as possible, shorten and stiffen the exposed needle, minimise lateral motion after first contact, and learn a mark-bias term as a function of approach angle and press depth. Bevel/tissue needle-deflection literature is only partly transferable to paper, but the shared lesson holds: once contact starts, lateral force bends the tip and the mark is written by the bent, dragged tip — not by the hover tip point.

## What the servo bridge can and cannot support

`D:/robot/ColonyPicker_Metahackathon/robot/servo_bridge.py:69-80` exposes position targets with a duration, readback, camera snapshots, and on/off torque only. There is no velocity/acceleration profile control, no current/load telemetry and no following-error stream. So from the PC side, trajectory quality must be synthesised from sequenced setpoints:

- split long moves into synchronised intermediate setpoints approximating an S-curve;
- slow down geometrically as error shrinks;
- dwell until optical motion, not just readback, has settled;
- identify the dominant flex/overshoot frequency from step responses and consider simple input shaping if a clean resonance exists.

Also validate what readback actually measures. If `joint_states` originates motor-side of the gearbox, readback can agree perfectly while the output shaft sits anywhere inside the backlash band. The cameras are currently the only true output-side sensors; if a hardware change ever becomes acceptable, output-shaft encoders plus the existing cameras (dual-loop: joint loop inside, visual tip loop outside) would be the single biggest mechanical-accuracy upgrade, followed by a stiffer/shorter needle mount and locking the chassis during contact.

## Experiments that decide where the millimetres are

Before choosing between calibration, control redesign and mechanics, decompose the error experimentally:

1. **Static sensing noise:** hold still, capture 100 frames per camera; get σ for tip, target, height.
2. **Same-command repeatability:** move away and return to the same final pose from the same approach direction 20×; measure tip scatter. This is the floor any controller must live with.
3. **Hysteresis width:** repeat (2) from opposite approach histories; the separation between the two clusters is the backlash/compliance state effect.
4. **Jacobian validity:** random small moves per joint across the workspace; report predicted-vs-realised gain by joint, configuration and direction.
5. **Sag map:** hold a pose grid; compare model tip vs measured tip as a function of shoulder/elbow.
6. **Arc-only scatter:** from verified hovers, run practice arcs onto a surrogate surface; separate landing bias/scatter from placing error and from ink-centroid measurement.
7. **A/B controller test:** current placer vs measured-Jacobian placer on the same dots, judged on the physical ink mark split into along/across, with time and iteration count recorded.

Decision rules:

- If same-direction return scatter is already ≳0.5 mm, no calibration will reliably deliver 1 mm marks; prioritise backlash discipline, output sensing or mechanics.
- If scatter is small but bias is large and pose-dependent, calibration + residual learning will pay off quickly.
- If hover is accurate but marks are off, stop optimising placing; the money is in arc fitting, contact stop and drag.
- If marks are good in one sheet region and bad in another, fit a spatial bias field and check paper-plane/base-registration assumptions before touching joint models.

For repeated colony work, that bias field becomes run-to-run learning: store commanded pose, approach history, predicted landing and measured mark error per target, then feed the local average error forward into the next run through the inverse measured Jacobian. That is the point-to-point form of iterative learning control; the indirect-ILC literature maps workspace errors back into motor-pulse compensation in exactly this way (SAGE, doi:10.1177/09544062261489107).

## Recommended priority order

1. **Measured-Jacobian MIMO placing with a locked target and stable state estimator** — biggest software win; removes model bias and retie injection from the loop.
2. **Backlash-state motion discipline: canonical approach, take-up moves, pan frozen late** — converts random hysteresis into repeatable, learnable bias.
3. **Apply and extend calibration: joint zeros, tip depth, hand-eye, gravity/compliance lumped terms, then a validated configuration-dependent residual** — improves first-move accuracy, move prediction and the arc.
4. **Rebuild the final descent around multi-height arc fitting, continuous triangulation and drag-free retract** — attacks the hover-to-mark gap directly.
5. **Run-to-run spatial bias learning over the sheet** — cheap accuracy once 1–4 make errors repeatable.
6. **Only if the measured repeatability floor still blocks the gate: output-shaft encoders, stiffer tool, braced base** — hardware closes the loop that software can only observe intermittently.

The governing principle: **first make the error repeatable, then make it small.** Backlash discipline and approach consistency create repeatability; measured Jacobians and calibration then remove the repeatable part; learning squeezes the residue. Trying to calibrate while every approach arrives with a different gear state is fitting noise.

## Literature anchors

- Uncalibrated visual servoing / dynamic Broyden Jacobian estimation: https://ieeexplore.ieee.org/document/803299 and https://link.springer.com/chapter/10.1007/978-981-99-5203-8_32
- IBVS interaction matrix and `v = −λ L⁺ e` control law: ViSP IBVS tutorial (fetched 2026-10-30)
- Hand-eye AX=XB, simultaneous nonlinear solve most robust: https://arxiv.org/abs/2311.12655
- Configuration-dependent kinematics calibration, >50% max-error reduction: https://arxiv.org/pdf/2510.19962
- Unified gravitational + elasto-geometrical calibration: https://ieeexplore.ieee.org/document/11020762
- Elasto-geometrical error and gravity model calibration: https://www.sciencedirect.com/science/article/pii/S0736584523000340
- Dynamic dead-zone / backlash modelling for rotary servo actuators: https://www.sciencedirect.com/science/article/pii/S0957415826000838
- Indirect iterative learning control mapping workspace error to pulse compensation: https://journals.sagepub.com/doi/abs/10.1177/09544062261489107
- Uncalibrated visual predictive control using Broyden estimation: https://ieeexplore.ieee.org/document/7844288
- Model Predictive Control for Uncalibrated Visual Servoing (Springer): https://link.springer.com/chapter/10.1007/978-981-99-6886-2_38
- Data-Driven Model Predictive Control for Uncalibrated Visual Servoing (MDPI Symmetry 16(1):48): https://www.mdpi.com/2073-8994/16/1/48

Caveat on sources: two full texts (Hardware-X low-cost-servo backlash test stand and an MDPI visual-servoing article) returned HTTP 403 during the 2026-10-30 session, so the claims above rest on the fetched abstracts, the ViSP/arXiv pages, and the search-result summaries — not on those two full texts.
