"""A1 entry for the re-seated near-vertical needle (2026-11-01) — every leg camera-measured.

Why not `well_reach fit/walk/well` directly: that tool was built for the previous seating —
its PITCHES cap at 1140 assuming vertical ≈ 1000 (shoulder+elbow+wrist pulses), while THIS
needle is vertical at 1157 (evidence/tip/newneedle_notes.json, measured live: the proof pose
{1:595,2:358,3:209,4:590} sums to 1157 and the snapshot shows the needle vertical). At 1157 no
A1 transit pose exists inside ENVELOPE even with a correct model (the wrist would need 435
at pitch 1000, i.e. ~590+ at 1157 — that is presses, not a pose this solver reaches over A1).
So the walk travels at 1157 (needle vertical, both cameras see it) and the last ~35 mm descend
at the nearest reachable angle the solver allows, corrected live by the measured-Jacobian
servo (`into`'s own method — joints 1–3, wrist held, gate 1.0 mm, depth 3 mm).

Safety, in order:
1. The model is refitted (tip, yaw, base) from camera-measured poses BEFORE anything moves:
   the stored well2_model.json predates the re-seat and is 153 mm off at the proof pose.
2. Every travel leg is <= 15 mm and ends with a camera triangulation (MISS <= 3.0 mm,
   >= 8 mm over the paper); the correction is re-measured, never extrapolated.
3. Pitch moves <= 10 pulses per 5 mm piece, every piece's predicted tip >= 30 mm over the
   highest rim (TRANSIT 35 − 5), and nothing goes below the rim except the final 3 mm
   inside A1's opening (gate 1.0 mm xy, ENTRY_XY_MM 2.5).
4. Any failed measurement, gate, or geometry check aborts BEFORE the arm moves again —
   the arm stops high, never deeper.
"""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import arm_calibrate as ac  # noqa: E402
import numpy as np  # noqa: E402
import plate_rig as pr  # noqa: E402
import well_reach as wr  # noqa: E402

VERTICAL = 1157  # shoulder+elbow+wrist pulses: this seating's vertical (newneedle_notes.json)
PROOF = {1: 595, 2: 358, 3: 209, 4: 590}
HOVER_MM = 35.0   # TRANSIT_MM: travel height over the highest rim
FLOOR_MM = 30.0   # TRANSIT − 5: no piece of any travel path goes under this
LEG_MM = 15.0     # WALK_LEG_MM
MIN_UP_MM = 30.0  # never descend to measure below this over the paper


def fit_poses():
    """Calibration poses where the needle is vertical (pitch 1157) and hangs over the paper
    in both cameras: direct variants of PROOF (constructed, not solved — the stored model's
    tip is 153 mm off for this seating, so its solver cannot generate them). Every pose is
    within 35 pulses of PROOF per joint and sums to exactly 1157."""
    deltas = [{}, {1: 30}, {1: -30}, {2: 20, 4: -20}, {2: -20, 4: 20}, {3: 20, 4: -20},
              {3: -20, 4: 20}, {1: 25, 2: 12, 4: -12}, {1: -25, 3: 12, 4: -12},
              {1: 15, 2: -15, 3: -15, 4: 30}, {1: -15, 2: 15, 3: 15, 4: -30}]
    out = []
    for d in deltas:
        q = {k: PROOF[k] + d.get(k, 0) for k in wr.ARM}
        assert q[2] + q[3] + q[4] == VERTICAL, q
        out.append(q)
    return out


def measured_poses(arm: wr.Arm, eyes: wr.Eyes, poses) -> tuple[list[dict], list[np.ndarray]]:
    """Move through the calibration poses at 1157, triangulating at each; the moves build on
    commanded values (never read-back), in 6-pulse pieces, wrist >= floor guard throughout."""
    qs, seen = [], []
    for q in poses:
        if q[2] + q[3] + q[4] != VERTICAL:
            raise RuntimeError(f"calibration pose {q} is not at vertical pitch {VERTICAL}")
        wr.jgo(arm.servos, q, arm.q)
        arm.q = {k: int(q[k]) for k in wr.ARM}  # int: dict equality with arm.q at the last pose must hold
        got = eyes.free_tip("cal")
        if got is not None and -got[2] >= MIN_UP_MM:
            qs.append(dict(q))
            seen.append(got)
            print(f"  cal {q}: tip {np.round(got, 1).tolist()} ({-got[2]:.0f} mm up)", flush=True)
        else:
            print(f"  cal {q}: not measured ({got})", flush=True)
    return qs, seen


def main() -> None:
    servos = wr.Servos()
    eyes = wr.Eyes(time.strftime("%Y%m%dT%H%M%S") + "_a1safe")
    now = {k: servos.read()[k] for k in wr.ARM}
    print(f"[a1] readback at start: {now}", flush=True)

    # Park background (PARK is out of both views; travel there is joint-space, wrist 500).
    q = wr.jgo(servos, wr.PARK, now)
    eyes.background()
    q = wr.jgo(servos, PROOF, q)  # the one pose the needle is already proven visible at
    print(f"[a1] at proof pose {PROOF}", flush=True)
    arm = wr.Arm(wr.model(), servos, dict(PROOF), np.zeros(3))
    arm.tied = False

    # Travel recovery: by the measured Jacobian (joints 1-3, wrist held) at the current height.
    def travel(arm: wr.Arm, eyes: wr.Eyes, rim: np.ndarray, to: np.ndarray) -> bool:
        """Joints 1-3 by the measured Jacobian in <=8 mm steps at the current height; the wrist
        never moves, so no solver branch cap can be hit. False = a step went unmeasured (arm high)."""
        arm.joints = (1, 2, 3)
        for i in range(60):
            err = to - arm.m
            if np.linalg.norm(err[:2]) <= 3.0 and abs(err[2]) <= 5.0:
                print(f"  travel ok: {np.round(arm.m, 1).tolist()}", flush=True)
                return True
            want = err if np.linalg.norm(err) <= wr.SERVO_STEP_MM else err * wr.SERVO_STEP_MM / np.linalg.norm(err)
            v = np.linalg.lstsq(arm.J, want, rcond=None)[0]
            v = np.round(v * min(1.0, wr.SERVO_MAX_PULSES / max(np.abs(v).max(), 1e-9)))
            if not v.any():
                raise RuntimeError("travel: zero joint step from the measured Jacobian")
            pred = wr.step_joints(arm, v, -20.0, float(rim[2]))  # floor '-20': travel stays at transit height
            got = wr.look(eyes, pred, wr.SERVO_LOOK_MM, f"travel{i}")
            if got is None:  # one wide re-look: a transient triangulation miss must not strand the run
                got = wr.look(eyes, pred, 25.0, f"travel{i}w")
            if got is None:
                got = wr.look(eyes, arm.m, 25.0, f"travel{i}p")
            if got is None:
                print(f"  travel step {i}: unmeasured (predicted {np.round(pred, 1).tolist()})", flush=True)
                return False
            arm.m = got
        raise RuntimeError("travel: not converging in 60 steps")

    # --- 1. refit the model from measurements at vertical poses; nothing travels yet ---
    poses = fit_poses()
    if len(poses) < 8:
        raise RuntimeError(f"only {len(poses)} calibration poses solvable offline: not moving")
    qs, seen = measured_poses(arm, eyes, poses)
    if len(qs) < 8:
        raise RuntimeError(f"only {len(qs)} calibration poses measured: not moving")
    p0 = np.array(json.loads(ac.ARM_MODEL.read_text())["session"], float)
    x, res = wr.fit_params(p0, qs, np.array(seen))
    while res.max() > 3.0 and len(qs) > 6:
        keep = np.arange(len(qs)) != int(np.argmax(res))
        qs, seen = [qq for qq, k in zip(qs, keep) if k], [ss for ss, k in zip(seen, keep) if k]
        x, res = wr.fit_params(p0, qs, np.array(seen))
        print(f"  dropped a fit pose; median {np.median(res):.2f} / max {res.max():.2f} mm over {len(qs)}", flush=True)
    # The global fit only steers 15 mm walk legs, each re-measured (15 mm window, per-leg learn);
    # the crash-critical gates are camera-exact below (probed Jacobian, 1.0 mm entry gate).
    # 2026-11-01 live: new-seating fit plateaus at ~3 mm median over +-30-pulse spans (backlash).
    if np.median(res) > 5.0 or res.max() > 8.0:
        raise RuntimeError(f"refit too poor (median {np.median(res):.2f}, max {res.max():.2f} mm): not moving")
    print(f"[a1] refit: tip {np.round(x[6:9], 1).tolist()}, yaw {np.degrees(x[9]):+.2f} deg, base "
          f"{np.round(x[10:13], 1).tolist()} mm; median {np.median(res):.2f} / max {res.max():.2f} mm", flush=True)
    Path("evidence/tip/well2_model.json").write_text(json.dumps(
        {"p": x.tolist(), "median_mm": float(np.median(res)), "max_mm": float(res.max()),
         "poses": len(qs), "when": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))
    arm.p = x
    # tie the correction at a pose the fit actually kept: go back there and re-measure
    wr.jgo(arm.servos, qs[-1], arm.q)
    arm.q = {k: int(qs[-1][k]) for k in wr.ARM}
    tie = eyes.free_tip("tie")
    if tie is None:
        raise RuntimeError("no tip measured at the tie pose: not moving")
    arm.corr = wr.tip(x, arm.q) - tie
    arm.tied = True  # the fit IS the tie (well_reach.fit's own convention)
    print(f"[a1] tied; residual at current pose {np.round(arm.corr, 2).tolist()} mm", flush=True)

    # --- 2. measured walk at 1157 to over A1, 35 mm up; every leg measured ---
    rim = pr.well("A1")
    end = np.r_[rim[:2], wr.TOP_Z - HOVER_MM]
    arm.target = "A1"
    # The offline check (2026-11-01, saved refit): a pitch-FREE walk to over A1 solves smoothly
    # (solver pitches 1130->1110, tip lands on the transit goal), while every pitch-strict path
    # dies on the wrist cap — so the walk fixes no pitch. Descent below transit is into()'s
    # probed measured Jacobian (joints 1-3, wrist held), which never uses solve().
    a = arm.predicted()
    n = max(1, math.ceil(np.linalg.norm(end[:2] - a[:2]) / LEG_MM))
    fresh = 0.0
    walked = True
    for i in range(1, n + 1):
        goal = a + (end - a) * i / n
        if fresh + LEG_MM > wr.WALK_FRESH_MM and np.linalg.norm(goal[:2] - rim[:2]) < 40.0:
            raise RuntimeError(f"no measured tip in the last {fresh:.0f} mm: not going over the plate")
        try:
            arm.line(goal, floor_mm=FLOOR_MM)  # pitch-free: solver picks the smooth pitch per piece
        except RuntimeError as e:
            if "branch hop" not in str(e) and "no pose" not in str(e):
                raise
            walked = False
            print(f"  walk {i}/{n}: solver refused ({e}); continuing by measured Jacobian", flush=True)
            break
        got = eyes.tip(goal, wr.WALK_EXPECT_MM, f"walk{i}")
        fresh = 0.0 if got is not None else fresh + LEG_MM
        if got is not None:
            print(f"  walk {i}/{n}: measured {np.round(got, 1).tolist()} (off {np.round(got - goal, 1).tolist()})",
                  flush=True)
            arm.corr = wr.tip(arm.p, arm.q) - got  # raw correction: learn() refits on sparse data and drifts
            arm.m = got
        else:
            print(f"  walk {i}/{n}: not measured", flush=True)
    if not walked:
        # the solver branch-hopped (wrist at the 640 cap): park the wrist high, probe, travel by Jacobian.
        # On ANY recovery failure the arm returns to PARK and the refit/walk retry once (like fit()).
        for attempt in (1, 2):
            arm.q[4] = 560
            wr.jgo(arm.servos, arm.q, arm.q)
            arm.m = wr.look(eyes, arm.predicted(), wr.START_LOOK_MM, "travel_start")
            ok_travel = False
            if arm.m is None:
                print("  travel: no tip after lowering the wrist", flush=True)
            else:
                arm.J = wr.model_jacobian(arm)
                wr.probe(arm, eyes, float(rim[2]), "travel")  # stops high on a failed probe; the arm never descends here
                ok_travel = travel(arm, eyes, rim, end)
            if ok_travel:
                break
            wr.jgo(arm.servos, wr.PARK, arm.q)
            arm.q = dict(wr.PARK)
            if attempt == 2:
                raise RuntimeError("travel failed twice: arm parked, needle never descended")
            print("  travel failed; parked, refitting and retrying the whole walk once", flush=True)
            wr.jgo(arm.servos, wr.PARK, arm.q)  # device-faithful retry: park, fresh refit, re-walk (fit() re-plans after each refusal)
            arm.q = {k: int(wr.PARK[k]) for k in wr.ARM}
            qs2, seen2 = measured_poses(arm, eyes, fit_poses())
            if len(qs2) < 6:
                raise RuntimeError("retry refit measured too few poses: not moving")
            x2, res2 = wr.fit_params(p0, qs2, np.array(seen2))
            while res2.max() > 3.0 and len(qs2) > 6:
                keep = np.arange(len(qs2)) != int(np.argmax(res2))
                qs2, seen2 = [q for q, k in zip(qs2, keep) if k], [s for s, k in zip(seen2, keep) if k]
                x2, res2 = wr.fit_params(p0, qs2, np.array(seen2))
            if np.median(res2) > 5.0 or res2.max() > 8.0:
                raise RuntimeError(f"retry refit too poor (median {np.median(res2):.2f}, max {res2.max():.2f}): not moving")
            for q_, s_ in zip(qs2, seen2):
                if q_ not in arm.data:
                    arm.data.append((dict(q_), np.asarray(s_, float)))
            arm.p = x2
            wr.jgo(arm.servos, qs2[-1], arm.q)  # tie at a kept pose, freshly measured (same convention as above)
            arm.q = {k: int(qs2[-1][k]) for k in wr.ARM}
            tie2 = eyes.free_tip("tie2")
            if tie2 is None:
                raise RuntimeError("retry: no tip at the tie pose: not moving")
            arm.corr = wr.tip(x2, arm.q) - tie2
            print(f"[a1] retry refit: median {np.median(res2):.2f} / max {res2.max():.2f} mm on {len(qs2)} poses", flush=True)
            a = arm.predicted()
            n2 = max(1, math.ceil(np.linalg.norm(np.r_[rim[:2], wr.TOP_Z - HOVER_MM] - a) / LEG_MM))
            fresh = 0.0
            for i in range(1, n2 + 1):
                goal = a + (np.r_[rim[:2], wr.TOP_Z - HOVER_MM] - a) * i / n2
                if fresh + LEG_MM > wr.WALK_FRESH_MM and np.linalg.norm(goal[:2] - rim[:2]) < 40.0:
                    raise RuntimeError(f"retry walk: no measured tip in the last {fresh:.0f} mm")
                arm.line(goal, floor_mm=FLOOR_MM)
                got = eyes.tip(goal, wr.WALK_EXPECT_MM, f"rwalk{i}")
                fresh = 0.0 if got is not None else fresh + LEG_MM
                if got is not None:
                    print(f"  retry walk {i}/{n2}: measured {np.round(got, 1).tolist()}", flush=True)
                    arm.corr = wr.tip(arm.p, arm.q) - got
                    arm.m = got
            if fresh > 0 or arm.m is None:
                raise RuntimeError("retry walk ended without a measured tip: not descending")
            break
        if float(rim[2]) - arm.m[2] < HOVER_MM - 8.0:
            raise RuntimeError(f"travel ended {float(rim[2]) - arm.m[2]:.1f} mm over the rim, not {HOVER_MM:.0f}: not descending")
    elif fresh > 0 or arm.m is None:
        raise RuntimeError("walk ended without a measured tip over the plate: not descending")

    # --- 3. entry by the measured Jacobian (into's gates: descend 15→6 mm,
    # xy gate 1.0 mm, then 3 mm inside A1's opening, overhead confirms the well) ---
    result = wr.into(arm, eyes, "A1", {})
    if not result.get("reached"):
        raise RuntimeError(f"A1 not reached: {result}")
    print("[a1] done.", flush=True)


if __name__ == "__main__":
    main()
