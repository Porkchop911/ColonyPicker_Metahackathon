#!/usr/bin/env python3
"""Lock the needle onto an ink dot in the arm camera and close in, with UGREEN height readouts.

Arm camera (eye-in-hand): the target dot is held AIM_BEYOND px past the tip along the needle, so it
stays visible just beyond the point instead of hiding behind it (a lesson from the pre-event needle prototype). The dot is picked
once (the black dot nearest the tip) and then only tracked; a lost dot is never re-acquired.

UGREEN height, no cross-camera translation needed: the arm camera sees the dot along one ray and
the tip along another; their directions in the sheet frame follow from the arm camera's measured
rotation and the fixed 180 deg between the two grid numberings (tip_motion.py, 2026-10-26), and its
height above the sheet from its own grid pose. So the camera sits at a known point on the ray back
from the dot, and the tip lies lam along its tip ray: lam is fitted to the UGREEN's ray to the tip,
and the leftover miss picks which UGREEN dot is the target (with two free unknowns every dot fitted
exactly and the pick flipped between dots, 43/63/152 mm). Height = -tip z.

Control: state (dot u - aim u, dot v - aim v, height) over pan/shoulder/elbow, Jacobian from +-PROBE
pulses, Broyden-updated; <= MAX_STEP pulses per joint per move. Halve the height down to FINAL_MM
holding the dot on the aim point, then one touch step, then lift.

    .venv/Scripts/python tools/dot_servo.py --near 852 168 [--no-touch]
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
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import Camera  # noqa: E402
from tip_descend import move, read  # noqa: E402

JOINTS = (1, 2, 3)
PROBE = 5
MAX_STEP = 6
AIM_BEYOND = 6.0
FINAL_MM = 3.0
TOUCH_MM = -0.5
ALIGN_PX = 1.5
TRACK_PX = 20.0
SURPRISE_MM = 4.0
LIFT = {2: 40}
ROT180 = np.diag([-1.0, -1.0, 1.0])  # arm grid numbering -> UGREEN numbering for this rig


def arm_dots(image: np.ndarray) -> np.ndarray:
    """Black ink dots in the arm image (not the needle, not the spiral binding chain)."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    dark = ((hat > 50) & (gray < 110)).astype(np.uint8)
    dark[cv2.dilate(tt._needle_blob(image).astype(np.uint8), np.ones((7, 7), np.uint8)) > 0] = 0
    count, _, stats, centres = cv2.connectedComponentsWithStats(dark)
    pts = np.array([centres[i] for i in range(1, count) if 3 <= stats[i, cv2.CC_STAT_AREA] <= 80
                    and max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) <= 12]).reshape(-1, 2)
    alone = np.array([np.sum(np.linalg.norm(pts - p, axis=1) < 20) == 1 for p in pts], bool)
    return pts[alone] if len(pts) else pts


def unit_ray(px) -> np.ndarray:
    n = cv2.undistortPoints(np.array([[px]], np.float64), tt.ARM_K, tt.ARM_D)[0, 0]
    v = np.array([n[0], n[1], 1.0])
    return v / np.linalg.norm(v)


class Sensor:
    def __init__(self, cam: Camera, rig: dict, ug_near, out: Path):
        self.cam, self.rig, self.ug_near, self.out = cam, rig, tuple(ug_near), out
        self.ku, self.du, self.ru, self.tu = (np.array(rig[x]) for x in ("k", "d", "r", "t"))
        self.black = [np.array([*d, 0.0]) for d in rig["black"]]
        self.dot_px = cv2.projectPoints(np.array([[x, y, 0.0] for x, y in rig["dots"]]), self.ru, self.tu,
                                        self.ku, self.du)[0][:, 0].tolist()
        self.dot = None  # tracked arm-image dot pixel
        self.n = 0

    def read(self) -> dict:
        self.n += 1
        arm, ug = tt.snap_arm(3), self.cam.frame(average=1, settle=0.2)
        cv2.imwrite(str(self.out / f"{self.n:03d}_arm.png"), arm)
        cv2.imwrite(str(self.out / f"{self.n:03d}_ugreen.jpg"), ug)
        tip, axis = tt.arm_tip(arm, with_axis=True)
        aim = np.array(tip) + AIM_BEYOND * axis
        dots = arm_dots(arm)
        if self.dot is None:  # pick once: the black dot nearest the tip
            if not len(dots):
                raise RuntimeError("no black dot in the arm view")
            self.dot = dots[np.argmin(np.linalg.norm(dots - np.array(tip), axis=1))]
        else:
            gaps = np.linalg.norm(dots - self.dot, axis=1) if len(dots) else np.array([np.inf])
            if gaps.min() > TRACK_PX:
                raise RuntimeError(f"target dot lost (nearest candidate {gaps.min():.0f} px from {np.round(self.dot, 1).tolist()})")
            self.dot = dots[np.argmin(gaps)]
        rot, t, _ = tm.arm_view(arm)
        cam_z = float((-rot.T @ t)[2])  # arm camera height (negative = above the sheet), numbering-free
        d_dot = ROT180 @ rot.T @ unit_ray(self.dot)
        d_tip = ROT180 @ rot.T @ unit_ray(tip)
        pu = tt.ugreen_tip(ug, self.ug_near, radius=90, exclude=self.dot_px, sheet=self.rig["sheet"])
        self.ug_near = pu
        cu, du = tt.ray(pu, self.ku, self.du, self.ru, self.tu)
        m = np.eye(3) - np.outer(du, du)
        best = None
        for D in self.black:
            # the arm camera sits on the ray back from this dot, at its own measured height
            rho = cam_z / -d_dot[2]
            C = D - rho * d_dot
            lam = float(np.linalg.lstsq((m @ d_tip)[:, None], m @ (cu - C), rcond=None)[0][0])
            T = C + lam * d_tip
            res = float(np.linalg.norm(m @ (T - cu)))
            if rho > 0 and lam > 0 and (best is None or res < best[0]):
                best = (res, T, D)
        if best is None:
            raise RuntimeError("no UGREEN dot fits the arm geometry")
        res, T, D = best
        return {"err": self.dot - aim, "height": float(-T[2]), "fit_mm": res, "tip_px": tip,
                "dot_px": self.dot.copy(), "ugreen_px": pu, "dot_sheet": D[:2], "tip_sheet": T}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--near", type=float, nargs=2, required=True, help="UGREEN tip guess px")
    ap.add_argument("--no-touch", action="store_true")
    args = ap.parse_args()
    out = Path("evidence/dot_servo") / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    log = []

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, (np.ndarray, list, tuple)) else (round(v, 2) if isinstance(v, float) else v))
               for k, v in row.items()}, flush=True)

    cam = Camera(0)
    rig = tt.ugreen_rig(cam.frame())
    sensor = Sensor(cam, rig, args.near, out)

    def measure() -> dict:
        for _ in range(2):
            try:
                return sensor.read()
            except RuntimeError as error:
                note(phase="measure-failed", error=str(error))
                if "lost" in str(error):
                    break  # never re-acquire: locking onto a neighbouring blob mid-approach steers the needle off
        raise RuntimeError("measurement failed")

    def state(r: dict) -> np.ndarray:
        return np.array([r["err"][0], r["err"][1], r["height"]])

    def lift(reason: str) -> None:
        now = read()
        move({k: now[k] + v for k, v in LIFT.items()}, ms=1500)
        note(phase="lift", reason=reason, pulses=read())

    pose = read()
    try:
        r = measure()
        note(phase="start", pulses=pose, height=r["height"], err=r["err"], fit=r["fit_mm"], dot_px=r["dot_px"],
             tip_px=r["tip_px"], dot_sheet=r["dot_sheet"])
        jac = np.zeros((3, 3))
        for col, j in enumerate(JOINTS):
            plus = move({**{k: pose[k] for k in JOINTS}, j: pose[j] + PROBE})
            sp = state(measure())
            minus = move({**{k: pose[k] for k in JOINTS}, j: pose[j] - PROBE})
            sm = state(measure())
            jac[:, col] = (sp - sm) / (plus[j] - minus[j])
            move({k: pose[k] for k in JOINTS})
        note(phase="jacobian", J_per_pulse=jac)
        r = measure()
        s = state(r)
        phase = "align"
        for step in range(40):
            aligned = np.hypot(*s[:2]) <= ALIGN_PX
            if phase == "align" and aligned:
                phase = "close"
            if phase == "close" and s[2] <= FINAL_MM + 0.7 and aligned:
                if args.no_touch:
                    note(phase="done", height=s[2], err=s[:2])
                    return
                phase = "touch"
            goal_h = s[2] if phase == "align" else (TOUCH_MM if phase == "touch" else max(FINAL_MM, s[2] * 0.6))
            want = np.array([0.0, 0.0, goal_h]) - s
            raw = np.linalg.lstsq(jac, want, rcond=None)[0]
            raw *= min(1.0, MAX_STEP / max(np.abs(raw).max(), 1e-9))
            dq = np.round(raw)
            if not dq.any():
                dq[np.argmax(np.abs(raw))] = np.sign(raw[np.argmax(np.abs(raw))])
            predicted = s + jac @ dq
            if phase != "touch" and predicted[2] < goal_h - 0.5:
                dq = np.round(dq * max(0.0, (s[2] - goal_h) / max(s[2] - predicted[2], 1e-9)))
                predicted = s + jac @ dq
            pose = move({k: pose[k] + int(d) for k, d in zip(JOINTS, dq)})
            r = measure()
            new = state(r)
            note(phase=phase, step=step, dq=dq, height=new[2], predicted_h=predicted[2], err=new[:2],
                 fit=r["fit_mm"], pulses=pose)
            if new[2] < predicted[2] - SURPRISE_MM:
                lift(f"height {new[2]:.1f} mm, {predicted[2] - new[2]:.1f} below prediction")
                return
            if dq.any():
                jac += np.outer((new - s) - jac @ dq, dq) / (dq @ dq)
            s = new
            if phase == "touch":
                note(phase="touched", height=s[2], err=s[:2], tip_sheet=r["tip_sheet"], dot_sheet=r["dot_sheet"])
                lift("touch done")
                return
        lift("step budget used")
    except Exception as error:  # any failure ends with the needle raised
        note(phase="abort", error=repr(error))
        lift("abort")
        raise


if __name__ == "__main__":
    main()
