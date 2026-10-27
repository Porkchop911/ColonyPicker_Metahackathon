#!/usr/bin/env python3
"""Put the held needle's tip on an ink dot, arm camera only, with the tip depth known from a touch-off.

The needle is rigid to the arm camera: its tip sits LAM mm out along the free-air tip pixel's ray
(touch-off by arm_touch / plunge, live 2026-10-27: 202.5 mm, h = 0 at the contact frame). Each frame's
grid pose gives the tip's landing point (its pixel ray meets the paper there, whatever the true depth)
and the target dot, both in that frame's numbering; the target is predicted from its offset to the
landing point and snapped to the nearest dot. State (dot x - land x, dot y - land y, tip
height) over pan / elbow / wrist (--joints); Jacobian seeded from a probed run, updated only from moves the readback
confirms and where the target dot was re-found. Descend only while the xy error is small;
press to TOUCH_MM below the paper, then lift with the elbow and check it lifted.

    python tools/aim_touch.py --out DIR [--lam 202.5] [--dot U V]
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
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import servo  # noqa: E402
from tip_descend import move, read  # noqa: E402

JOINTS = (1, 3, 4)  # pan, elbow, wrist: servo 2 sagged and stopped following upward moves at low reach (live 2026-10-27)
# (x, y, height) mm per (pan, elbow, wrist) pulse, probed live 2026-10-27 aim run 5
J0 = np.array([[2.95, -0.11, -0.13], [-0.11, 0.47, 0.89], [0.0, 0.33, 0.42]])
MAX_STEP = 8
XY_OK_MM = 2.0     # descend only while the tip is over the dot this closely
DESCEND_MM = 3.0   # height step while above 6 mm, then 1 mm
TOUCH_MM = -2.0    # planned overtravel into the soft notebook: with the target right under the tip the
                   # dot sits beside the tip in the image and faked a 5 px "push" with no touch (live 2026-10-27)
APPROACH_MM = 15.0  # come down to this before picking the target
SAFE_MM = 8.0      # align at or above this height, never skimming the paper
SNAP_MM = 4.0      # predicted target within this of a detected dot: take the dot (ink dots are >= 5 mm apart)
STOP_PX = 8.0      # a push this strong stops the descent whatever the model says
LIFT_ELBOW = 40    # elbow + raises the camera here (servo 2 sags when the arm reaches low)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--lam", type=float, default=202.5, help="tip depth along its pixel ray, mm")
    ap.add_argument("--dot", type=float, nargs=2, help="target dot px (default: nearest the tip's foot)")
    ap.add_argument("--steps", type=int, default=60)
    ap.add_argument("--joints", type=int, nargs=3, default=JOINTS, help="servos driving (x, y, height)")
    ap.add_argument("--press", type=float, default=TOUCH_MM, help="target tip height at the touch, mm (negative = into the paper)")
    args = ap.parse_args()
    joints = tuple(args.joints)
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log, n = [], [0]

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, (np.ndarray, list, tuple)) else (round(v, 2) if isinstance(v, float) else v))
               for k, v in row.items()}, flush=True)

    note(phase="light", lamps=at.auto_light())
    first = tt.snap_arm(3)
    free, axis = tt.arm_tip(first, with_axis=True)
    free, axis = np.array(free), np.array(axis)
    ray = at.unit_ray(free)
    offset = [None]  # target - tip foot, sheet mm: carried frame to frame, no numbering link needed

    def sense(expect=None) -> dict:
        """Tip back-shift, tip in this frame's sheet frame, and the target's sheet xy in the same frame.
        The target is predicted from the tip (`expect`: offset from the tip foot) and snapped to the
        nearest dot within SNAP_MM; linking grid numberings across frames slipped by whole squares
        (live 2026-10-27: a 2-pulse move "moved" the target 15 mm). Hidden under the needle: prediction."""
        for _ in range(3):  # a frame whose grid pose does not solve is retaken
            n[0] += 1
            img = tt.snap_arm(2)
            cv2.imwrite(str(out / f"{n[0]:03d}_arm.png"), img)
            try:
                rot, t, _ = tm.arm_view(img)
                break
            except RuntimeError:
                time.sleep(0.3)
        else:
            raise RuntimeError("grid pose failed 3 frames running")
        try:
            back = float(-(np.array(tt.arm_tip_on_line(img, free, axis)) - free) @ axis)
        except RuntimeError:  # at contact the shadow runs on from the tip and the walk finds no end
            back = float("nan")
        tip = rot.T @ (args.lam * ray - t)  # sheet frame, z negative above the sheet (height only)
        # where the tip will land: its pixel ray meets the paper there whatever the true depth; the point
        # straight under the depth-model tip missed by "quite a lot" (live 2026-10-27)
        foot = at.sheet_xy(free, rot, t)
        found = at.dots_near(img, free)
        pts = np.array([at.sheet_xy(f, rot, t) for f in found]).reshape(-1, 2)
        goal, seen = None, False
        if expect is not None:
            goal = foot + expect
            if len(pts):
                gaps = np.linalg.norm(pts - goal, axis=1)
                if gaps.min() <= SNAP_MM:
                    goal, seen = pts[np.argmin(gaps)], True
        show(img, rot, t, np.array([*foot, tip[2]]), goal)
        return {"img": img, "back": back, "tip": tip, "foot": foot, "rot": rot, "t": t, "found": found, "pts": pts,
                "goal": goal, "seen": seen}

    def show(img, rot, t, tip, goal) -> None:
        """Arm view with the target dot (red ring) and the tip's landing point (green cross) marked."""
        view = img.copy()
        marks = [((tip[0], tip[1], 0.0), (0, 200, 0))] + ([((goal[0], goal[1], 0.0), (0, 0, 255))] if goal is not None else [])
        for xyz, colour in marks:
            px = cv2.projectPoints(np.array([xyz], float), cv2.Rodrigues(rot)[0], t, tt.ARM_K, tt.ARM_D)[0][0, 0]
            centre = tuple(int(round(v)) for v in px)
            if colour == (0, 0, 255):
                cv2.circle(view, centre, 9, colour, 2)
            else:
                cv2.drawMarker(view, centre, colour, cv2.MARKER_CROSS, 14, 2)
        cv2.putText(view, f"h {-tip[2]:.1f} mm" + ("" if goal is None else f"  err {np.hypot(*(goal - tip[:2])):.1f} mm"),
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imwrite(str(out / f"{n[0]:03d}_marked.png"), view)
        cv2.imshow("aim_touch: red = target, green = where the tip lands", cv2.resize(view, None, fx=2, fy=2))
        cv2.waitKey(1)

    def lift(reason: str) -> None:
        now = read()
        servo({"set": {str(k): v for k, v in {**now, 3: now[3] + LIFT_ELBOW}.items()}, "ms": 1500})
        time.sleep(2.2)
        after = read()
        note(phase="lift", reason=reason, lifted=after[3] - now[3] >= 20, pulses=after)

    try:
        s = sense()
        # come down to APPROACH_MM first: from high up the landing point lies far ahead and swings with
        # every step, and the target picked there was 57 mm off and never re-found (live 2026-10-27)
        jac, cmd = J0.copy(), dict(read())
        for _ in range(20):
            h = -s["tip"][2]
            if h <= APPROACH_MM + 3:
                break
            dh = max(APPROACH_MM - h, -15.0)
            raw = np.linalg.lstsq(jac[2:3, 1:], [dh], rcond=None)[0]  # elbow + wrist only, xy ignored
            for k, d in zip(joints[1:], np.clip(np.round(raw), -MAX_STEP, MAX_STEP).astype(int)):
                cmd[k] += int(d)
            move(cmd, ms=300, settle=0.6)
            s = sense()
            note(phase="approach", height_mm=-s["tip"][2])
        # target: the dot nearest the tip's foot on the sheet (or the one asked for); first frame = reference
        if args.dot:
            i = int(np.argmin(np.linalg.norm(s["found"] - np.array(args.dot), axis=1)))
        else:
            i = int(np.argmin(np.linalg.norm(s["pts"] - s["foot"], axis=1)))
        dot = s["found"][i]
        s["goal"] = s["pts"][i]

        def state(r: dict) -> np.ndarray:
            return np.array([*(r["goal"] - r["foot"]), -r["tip"][2]])

        x = state(s)
        offset[0] = x[:2]
        note(phase="start", tip_px=free, dot_px=dot, err_mm=x[:2], height_mm=x[2])
        # seeded, not probed: probes move the target ~24 mm relative to the landing point, out of any
        # snap window (live 2026-10-27); refined below from steps where the dot was really re-found
        x = state(sense(offset[0]))
        cmd = dict(read())
        for step in range(args.steps):  # noqa: B007
            close = np.hypot(*x[:2]) <= XY_OK_MM
            if not close:  # align at a safe height: climb first if the probes left the tip low
                dh = max(0.0, SAFE_MM - x[2])
            else:
                dh = max(args.press - x[2], -(DESCEND_MM if x[2] > 6 else 1.0))
            raw = np.linalg.lstsq(jac, np.array([-x[0], -x[1], dh]), rcond=None)[0]
            # clip each joint on its own: scaling the whole step to the biggest one pushed pan into its deadband
            dq = np.clip(np.round(raw), -MAX_STEP, MAX_STEP).astype(int)
            before = read()
            for k, d in zip(joints, dq):
                cmd[k] += int(d)
            now = move(cmd, ms=300, settle=0.6)
            moved = np.array([now[k] - before[k] for k in joints], float)
            r = sense(x[:2] + jac[:2] @ moved)  # where the target should now be, relative to the tip
            new = state(r)
            note(phase="descend" if close else "align", step=step, dq=dq, moved=moved, err_mm=new[:2],
                 height_mm=new[2], back_px=r["back"], seen=r["seen"], pulses=now)
            # a touch counts only at the end of a descend step: run 4 "touched" on an align step, 1.9 mm off
            pressed = close and new[2] <= args.press + 0.3 and np.hypot(*new[:2]) <= XY_OK_MM
            if pressed or r["back"] > STOP_PX:
                cv2.imwrite(str(out / "contact_arm.png"), r["img"])
                note(phase="touch" if pressed else "push-stop", err_mm=new[:2], height_mm=new[2], start_dot_px=dot)
                lift("contact")
                return
            if moved.any() and r["seen"]:  # a predicted (unseen) target would only confirm the model
                jac += np.outer((new - x) - jac @ moved, moved) / (moved @ moved)
            x = new
        lift("step budget used, no contact")
    except BaseException as error:  # any failure, Ctrl-C included, ends with the needle raised
        note(phase="abort", error=repr(error))
        lift("abort")
        raise


if __name__ == "__main__":
    main()
