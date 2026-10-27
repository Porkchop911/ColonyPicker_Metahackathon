#!/usr/bin/env python3
"""Fast first move from the URDF arm model: put the needle tip almost over a target, no servo loop.

One arm frame gives the camera pose in the sheet frame (grid PnP); the vendored arm model
(`hwr_kinematics`, from the robot stack's URDF) gives the same camera in base_link at the commanded
pulses. Together they tie base_link to the sheet (this frame's own grid numbering). The tip sits LAM mm
out along its free-air pixel ray in the camera frame, so the model carries it to any pose. Pan / elbow /
wrist (--joints; servo 2 is held: it sags low and far) are solved by damped Gauss-Newton on the model
so the tip lands HOVER mm above the target; --go sends them and measures where the tip's landing point
really ended up. Model-only placing on this arm was ~5-10 mm off (cube stacking, dozorin plan/16),
so this only replaces the slow approach: aim_touch.py closes the last millimetres.

    python tools/coarse_aim.py --out DIR [--pose '{"1":497,...}'] [--dot U V | --sheet X Y] [--go]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_touch as at  # noqa: E402
import hwr_kinematics as hk  # noqa: E402
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402

START_POSE = {1: 497, 2: 423, 3: 235, 4: 269, 5: 502, 10: 662}  # handover 2026-10-27, all joints commanded
JOINTS = (1, 3, 4)   # pan, elbow, wrist
HOVER_MM = 15.0      # stop this far above the target: aim_touch.py's APPROACH_MM
MAX_MOVE = 150       # pulses per joint in one go; a bigger jump means the model or the pose is off
TILT_WARN_DEG = 5.0  # sheet normal vs base z: more = model, pose or camera convention is wrong
ARM = hk.ArmKinematics()


def camera_in_base(pose: dict) -> tuple[np.ndarray, np.ndarray]:
    """Model camera pose (R, origin mm): optical frame -> base_link, at these (float) pulses."""
    rot, origin = ARM.camera_pose(hk.pulses_to_joints(pose))
    return rot, origin * 1000.0


def sheet_from_base(rot: np.ndarray, t: np.ndarray, pose: dict) -> tuple[np.ndarray, np.ndarray]:
    """(A, b) with sheet = A @ base + b (mm), from one frame's grid pose (sheet -> camera: cam = rot @
    sheet + t) and the model camera at the pulses that frame was taken at."""
    r_cam, o_cam = camera_in_base(pose)
    return rot.T @ r_cam.T, -rot.T @ (r_cam.T @ o_cam + t)


def tip_in_base(pose: dict, tip_cam: np.ndarray) -> np.ndarray:
    """Needle tip in base_link (mm): the camera-frame tip (rigid to the camera) carried by the model."""
    r_cam, o_cam = camera_in_base(pose)
    return r_cam @ tip_cam + o_cam


def solve(pose: dict, tip_cam: np.ndarray, goal: np.ndarray, joints=JOINTS, iters: int = 30):
    """Float pulses for `joints` putting the tip at `goal` (base mm); others held. (pose, residual mm)."""
    q = {k: float(v) for k, v in pose.items()}
    for _ in range(iters):
        err = goal - tip_in_base(q, tip_cam)
        if np.linalg.norm(err) < 0.05:
            break
        jac = np.zeros((3, len(joints)))
        for i, k in enumerate(joints):  # central differences, 1 pulse: the model is smooth
            up, down = dict(q), dict(q)
            up[k] += 1.0
            down[k] -= 1.0
            jac[:, i] = (tip_in_base(up, tip_cam) - tip_in_base(down, tip_cam)) / 2.0
        step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-3 * np.eye(3), err)  # damped: safe near singular
        for i, k in enumerate(joints):
            q[k] += float(np.clip(step[i], -40.0, 40.0))
    return q, float(np.linalg.norm(goal - tip_in_base(q, tip_cam)))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--pose", type=json.loads, help="commanded pulses the arm is at now (default: start pose)")
    ap.add_argument("--lam", type=float, default=202.5, help="tip depth along its pixel ray, mm")
    ap.add_argument("--dot", type=float, nargs=2, help="target px in this arm frame (default: dot nearest the landing point)")
    ap.add_argument("--sheet", type=float, nargs=2, help="target in this frame's sheet mm")
    ap.add_argument("--hover", type=float, default=HOVER_MM)
    ap.add_argument("--joints", type=int, nargs="+", default=JOINTS)
    ap.add_argument("--go", action="store_true", help="send the solved pose (default: plan only)")
    ap.add_argument("--ms", type=int, default=1500)
    args = ap.parse_args()
    pose = {int(k): int(v) for k, v in (args.pose or START_POSE).items()}
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log: list = []

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, (np.ndarray, list, tuple)) else v) for k, v in row.items()}, flush=True)

    note(phase="light", lamps=at.auto_light())
    img = tt.snap_arm(3)
    cv2.imwrite(str(out / "before_arm.png"), img)
    free = np.array(tt.arm_tip(img))
    ray = at.unit_ray(free)
    tip_cam = args.lam * ray
    rot, t, blobs = tm.arm_view(img)
    a, b = sheet_from_base(rot, t, pose)
    # sanity: the sheet must lie flat in base_link; a tilt here means the pose passed in, the model
    # or the camera axes disagree with the real arm, and the solved move would be wrong
    normal = a.T @ np.array([0.0, 0.0, 1.0])  # sheet +z (into the paper) in base
    tilt = float(np.degrees(np.arccos(min(1.0, abs(normal[2])))))
    sheet_z = float((a.T @ (np.zeros(3) - b))[2])  # sheet origin height in base, mm
    foot = at.sheet_xy(free, rot, t)
    if args.sheet:
        target = np.array(args.sheet)
    elif args.dot:
        target = at.sheet_xy(args.dot, rot, t)
    else:
        pts = np.array([at.sheet_xy(f, rot, t) for f in at.dots_near(img, free)]).reshape(-1, 2)
        if not len(pts):
            raise SystemExit("no dot found near the landing point; pass --dot or --sheet")
        target = pts[np.argmin(np.linalg.norm(pts - foot, axis=1))]
    goal_sheet = np.array([*target, -args.hover])  # sheet z is negative above the paper
    goal = a.T @ (goal_sheet - b)
    new, residual = solve(pose, tip_cam, goal, tuple(args.joints))
    planned = {k: int(round(v)) for k, v in new.items()}
    jumps = {k: planned[k] - pose[k] for k in args.joints}
    note(phase="plan", pose=pose, tip_px=free, sheet_tilt_deg=tilt, sheet_z_base_mm=sheet_z,
         landing_mm=foot, target_mm=target, goal_base_mm=goal, residual_mm=residual, planned=planned, jumps=jumps)
    if tilt > TILT_WARN_DEG:
        raise SystemExit(f"sheet tilted {tilt:.1f} deg in the model: check --pose and the camera axes before moving")
    if residual > 1.0:
        raise SystemExit(f"model cannot reach the goal with joints {args.joints} ({residual:.1f} mm left); try adding 2")
    if max(abs(v) for v in jumps.values()) > MAX_MOVE:
        raise SystemExit(f"jump > {MAX_MOVE} pulses: start closer, or check the pose")
    if not args.go:
        print("plan only; add --go to move. Next commanded pose:", json.dumps(planned))
        return
    from rig_io import servo  # robot I/O only when moving
    servo({"set": {str(k): v for k, v in planned.items()}, "ms": args.ms})
    time.sleep(args.ms / 1000 + 0.8)
    img = tt.snap_arm(3)
    cv2.imwrite(str(out / "after_arm.png"), img)
    rot, t, blobs2 = tm.arm_view(img)
    # the target in the new frame's numbering, from the images only (ink marks matched across both
    # frames): carrying it through the model would hide exactly the model error measured here
    shift, hits = tm.lattice_offset(blobs, blobs2)
    target_now = target - shift
    landing = at.sheet_xy(free, rot, t)
    height = float(-(rot.T @ (tip_cam - t))[2])
    note(phase="moved", commanded=planned, landing_mm=landing, target_mm=target_now, marks_matched=hits,
         model_miss_mm=target_now - landing, height_mm=height)
    if hits < 3:
        print("too few ink marks matched across the move: the miss above is not trustworthy")
    print("commanded pose now:", json.dumps(planned), "- pass it to aim_touch / teleop as the start")


if __name__ == "__main__":
    main()
