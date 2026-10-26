#!/usr/bin/env python3
"""Closed-loop descent of the needle tip onto an ink dot, steered and stopped by measured tip
positions only (no contact detector, no model-based heights).

Per step: the arm camera's grid pose + this grasp's tip offset (tools/tip_triangulate.py) gives the
tip HEIGHT - invariant to how the two grid numberings line up. The UGREEN's tip ray, cut at that
height, gives the tip's sheet XY in the UGREEN frame, where the target dot is known. So the descent
needs no dot pairing, which fails at the end anyway when the needle hides the dot from the arm.

Steering: Jacobian of (x, y, height) over pan/shoulder/elbow from +-PROBE central differences,
Broyden-updated after every move; <= MAX_STEP pulses per joint per move. Phases: align at the start
height, halve the height down to FINAL_MM holding XY, one touch step aimed at TOUCH_MM, lift.
Aborts (and lifts) on two failed measurements in a row or a height more than SURPRISE_MM below the
prediction.

    .venv/Scripts/python tools/tip_descend.py --dot 35.3 8.3 [--near 1154 440] [--no-touch]
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
import paper_grid as pg  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import Camera, servo  # noqa: E402

JOINTS = (1, 2, 3)  # pan, shoulder, elbow; wrist and roll hold (the tip offset is calibrated at this roll)
PROBE = 6
MAX_STEP = 8
ALIGN_MM = 0.8
FINAL_MM = 2.0
TOUCH_MM = -0.5
SURPRISE_MM = 4.0
LIFT = {2: 40}  # shoulder +40 pulses raises the arm (live: 361 -> 416 lifted the needle off the paper)


def read() -> dict:
    return {int(k): v for k, v in servo({"read": True})[0]["pulses"].items()}


def move(target: dict, ms: int = 700) -> dict:
    """Command bus pulses, wait, verify readback within 35 (sag), resend once."""
    for _ in range(2):
        servo({"set": {str(k): int(v) for k, v in target.items()}, "ms": ms})
        time.sleep(ms / 1000 + 0.6)
        now = read()
        if max(abs(now[k] - v) for k, v in target.items()) <= 35:
            return now
    raise RuntimeError(f"arm did not reach {target}: readback {now}")


class Probe:
    def __init__(self, cam: Camera, rig: dict, tip_cam: np.ndarray, near: tuple[float, float], out: Path):
        self.cam, self.rig, self.tip_cam, self.near, self.out = cam, rig, tip_cam, near, out
        k, d, r, t = (np.array(rig[x]) for x in ("k", "d", "r", "t"))
        self.ug = (k, d, r, t)
        dots = np.array([[x, y, 0.0] for x, y in rig["dots"]])
        self.dot_px = cv2.projectPoints(dots, r, t, k, d)[0][:, 0].tolist()
        self.n = 0

    def state(self) -> np.ndarray:
        """(x, y, height) of the tip in mm, UGREEN sheet frame."""
        self.n += 1
        ug, arm = self.cam.frame(), tt.snap_arm()
        cv2.imwrite(str(self.out / f"{self.n:03d}_ugreen.jpg"), ug)
        cv2.imwrite(str(self.out / f"{self.n:03d}_arm.png"), arm)
        obj, px, _ = pg.grid_points(arm, seed=(arm.shape[1] / 2, arm.shape[0] * 0.75))
        ok, r, t, inliers = cv2.solvePnPRansac(obj, px, tt.ARM_K, tt.ARM_D, reprojectionError=2.0)
        if not ok or len(inliers) < 100:
            raise RuntimeError(f"arm grid pose failed ({0 if inliers is None else len(inliers)} inliers)")
        rot, _ = cv2.Rodrigues(r)
        height = -(rot.T @ (self.tip_cam - t.ravel()))[2]
        pu = tt.ugreen_tip(ug, self.near, radius=90, exclude=self.dot_px)
        c, d = tt.ray(pu, *self.ug)
        xy = (c + d * ((-height - c[2]) / d[2]))[:2]
        self.near = pu
        return np.array([xy[0], xy[1], height])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dot", type=float, nargs=2, required=True, help="target dot, UGREEN sheet mm (rig dots)")
    ap.add_argument("--near", type=float, nargs=2, default=(1154, 440), help="UGREEN tip guess px")
    ap.add_argument("--no-touch", action="store_true", help="stop at FINAL_MM above the dot")
    ap.add_argument("--tip-cam", type=float, nargs=3, default=None,
                    help="this grasp's tip offset in the arm-camera frame (mean of earlier triangulations)")
    args = ap.parse_args()
    out = Path("evidence/descend") / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    log = []

    def note(**row):
        row["t"] = round(time.time(), 2)
        log.append(row)
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, np.ndarray) else v) for k, v in row.items()}, flush=True)

    rig = json.loads(tt.RIG.read_text())
    # this grasp's tip offset, from one full two-camera triangulation
    cam = Camera(0)
    first = tt.measure(cam.frame(), tt.snap_arm(), rig, tuple(args.near))
    fresh = np.array(first["tip_arm_cam_mm"])
    tip_cam = np.array(args.tip_cam) if args.tip_cam else fresh
    note(phase="calibrate", tip_cam=tip_cam, fresh=fresh, miss=first["miss_mm"], height=first["height_mm"], ink=first["ink_score"])
    if args.tip_cam is None and first["miss_mm"] > 1.5:
        raise SystemExit("tip offset not trustworthy (miss > 1.5 mm); not moving")
    if np.linalg.norm(fresh - tip_cam) > 3.0:
        raise SystemExit(f"fresh tip offset {fresh} is {np.linalg.norm(fresh - tip_cam):.1f} mm from --tip-cam: needle slipped?")
    probe = Probe(cam, rig, tip_cam, tuple(first["ugreen_tip_px"]), out)
    dot = np.array(args.dot)

    def measure() -> np.ndarray:
        for attempt in range(2):
            try:
                return probe.state()
            except RuntimeError as error:
                note(phase="measure-failed", error=str(error))
        raise RuntimeError("two measurements in a row failed")

    def lift(reason: str) -> None:
        now = read()
        move({k: now[k] + v for k, v in LIFT.items()}, ms=1500)
        note(phase="lift", reason=reason, pulses=read())

    pose = read()
    try:
        s = measure()
        note(phase="start", pulses=pose, tip=s, error_xy=dot - s[:2])
        # Jacobian by central differences, returning to the base pose after each joint
        jac = np.zeros((3, len(JOINTS)))
        for col, j in enumerate(JOINTS):
            plus = move({**{k: pose[k] for k in JOINTS}, j: pose[j] + PROBE})
            sp = measure()
            minus = move({**{k: pose[k] for k in JOINTS}, j: pose[j] - PROBE})
            sm = measure()
            jac[:, col] = (sp - sm) / (plus[j] - minus[j])
            move({k: pose[k] for k in JOINTS})
        note(phase="jacobian", J_mm_per_pulse=jac)
        s = measure()
        goal_h, phase = s[2], "align"
        for step in range(40):
            err_xy = dot - s[:2]
            if phase == "align" and np.linalg.norm(err_xy) <= ALIGN_MM:
                phase = "descend"
            if phase == "descend":
                if s[2] <= FINAL_MM + 0.7 and np.linalg.norm(err_xy) <= ALIGN_MM:
                    if args.no_touch:
                        note(phase="done", tip=s, error_xy=err_xy)
                        break
                    phase = "touch"
                goal_h = max(FINAL_MM, s[2] / 2)
            if phase == "touch":
                goal_h = TOUCH_MM
            want = np.array([*dot, goal_h]) - s
            raw = np.linalg.lstsq(jac, want, rcond=None)[0]
            raw *= min(1.0, MAX_STEP / max(np.abs(raw).max(), 1e-9))
            dq = np.round(raw)
            if not dq.any():  # under one pulse everywhere: take one pulse on the joint that wants most
                dq[np.argmax(np.abs(raw))] = np.sign(raw[np.argmax(np.abs(raw))])
            predicted = s + jac @ dq
            if phase != "touch" and predicted[2] < goal_h - 0.5:
                dq *= max(0.0, (s[2] - goal_h) / max(s[2] - predicted[2], 1e-9))
                dq = np.round(dq)
                predicted = s + jac @ dq
            target = {k: pose[k] + int(d) for k, d in zip(JOINTS, dq)}
            pose = move({**{k: pose[k] for k in JOINTS}, **target})
            new = measure()
            note(phase=phase, step=step, dq=dq, pulses=pose, tip=new, predicted=predicted, error_xy=dot - new[:2])
            if new[2] < predicted[2] - SURPRISE_MM:
                lift(f"height {new[2]:.1f} mm, {predicted[2] - new[2]:.1f} mm below prediction")
                return
            moved = new - s
            if np.linalg.norm(dq) > 0:  # Broyden: keep the Jacobian honest as the pose changes
                jac += np.outer(moved - jac @ dq, dq) / (dq @ dq)
            s = new
            if phase == "touch":
                note(phase="touched", tip=s, error_xy=dot - s[:2])
                lift("touch done")
                return
        else:
            lift("step budget used")
    except Exception as error:  # any failure ends with the needle raised
        note(phase="abort", error=repr(error))
        lift("abort")
        raise


if __name__ == "__main__":
    main()
