#!/usr/bin/env python3
"""Touch an ink dot with the held needle using the arm camera alone: lock, push along the ray, feel.

The needle is rigid to the arm camera, so in free air its tip stays on one pixel whatever the arm
does, at an unknown depth along that pixel's ray (the grasp changes between runs). Hold the target
dot just beyond the tip in the image and the dot sits on the tip's ray; then moving the camera along
that ray brings the tip onto the dot without ever knowing the depth. Contact is felt: the paper
pushes the needle, and the tip pixel leaves its free-air spot.

Control: pan centres the dot in u, elbow in v (fixed gains refined from real moves); once aligned the
shoulder steps down (10 pulses while the paper is far along the tip ray, from the grid, then 2) with
the elbow fed forward. The target is followed as part of the whole dot pattern.

Light: lamp settings are tried in turn and the first one where the tip is found below the image top
and dots are visible wins (a lamp in the arm's view darkens the auto exposure and floods the needle
mask).

    python tools/arm_touch.py [--out DIR] [--steps 80]
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
import hue  # noqa: E402
import paper_grid as pg  # noqa: E402
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from tip_descend import POSE_HELP, commanded, move, read  # noqa: E402

JOINTS = (1, 2, 3)
MAX_STEP = 10       # pulses per joint per align move
PUSH_FAR_MM = 6.0   # camera approach per push step while the paper is far along the tip ray
PUSH_NEAR_MM = 1.5  # ... and near it
# (dot v px, paper mm along the tip ray) per (shoulder, elbow) pulse, live 2026-10-27 run 13: elbow +6 ->
# v +13.6 px, paper +7 mm; shoulder -10 & elbow +10 -> v +8.4 px, paper +1.5 mm (servo 2: higher = arm up)
SE_TO_VP = np.array([[1.46, 2.3], [1.15, 1.3]])
SLOW_BELOW_MM = 262.0  # paper distance along the tip ray: yesterday's tip sat ~237 mm out, + 25
GAIN_PAN_U = 6.0    # px of dot u per pan pulse (live 2026-10-27: 1.6-10, backlash)
GAIN_ELBOW_V = 2.5  # px of dot v per elbow pulse (2.0-2.9)
SHOULDER_V = 1.46   # px of dot v per shoulder pulse (pattern-shift prediction only)
BACKLASH = {1: 2, 3: 3}  # pulses lost on a direction reversal (elbow: +3 x3 commanded, then a 7-11 jump)
FINGER_ROW = 340    # blobs below this row are the gripper's own edges, not ink
AIM_BEYOND = 24.0   # px: dot clear of the tip even after a ~20 px elbow overshoot; contact lands ~14 mm short
ALIGN_LO = np.array([-6.0, -30.0])  # dot - aim, px: above the aim (v < 0, away from the tip) is the safe side:
ALIGN_HI = np.array([6.0, 10.0])    # every push drops the dot ~12 px there, and re-centring it undid the push
ALIGN_PX = np.array([6.0, 10.0])  # u, v: half the smallest real move (pan ~10 px with backlash; elbow sticks, then jumps ~8 pulses = ~20 px)
LIFT = {2: 40}
# (light 12 bri, light 13 bri), 0 = off; first usable wins
LIGHTS = [(0, 254), (0, 180), (120, 120), (254, 0), (254, 254), (0, 0)]  # 12 sits in the arm view: its glare darkens the auto exposure


def unit_ray(px) -> np.ndarray:
    n = cv2.undistortPoints(np.array([[px]], np.float64), tt.ARM_K, tt.ARM_D)[0, 0]
    v = np.array([n[0], n[1], 1.0])
    return v / np.linalg.norm(v)


def constellation_shift(before: np.ndarray, after: np.ndarray, expect=(0.0, 0.0), tol: float = 8.0) -> tuple[np.ndarray, int]:  # 8: a 10-pulse pan also rotates the view ~5 px
    """Image shift that lays the most `before` dots onto `after` dots (within `tol` px). The dots move
    together when the arm moves, so a 30 px pan snap that leaves the target nearer a neighbour's old
    spot still overlays the whole pattern. Blobs fixed to the gripper never move in the image and
    out-voted a real 60 px pan shift (live 2026-10-27), so after a clear move (`expect`, from the servo
    readback) a near-zero shift is refused; the gains are too loose (backlash) to predict more."""
    best = (np.zeros(2), 0)
    for shift in (after[None, :, :] - before[:, None, :]).reshape(-1, 2):
        if np.linalg.norm(expect) > 20 and np.linalg.norm(shift) < 10:
            continue  # the arm clearly moved: a pattern that stayed put is the gripper's own blobs
        hits = int((np.linalg.norm(before[:, None, :] + shift - after[None, :, :], axis=2).min(axis=1) <= tol).sum())
        if hits > best[1]:
            best = (shift, hits)
    return best


def sheet_xy(px, rot: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Where an arm-image pixel lands on the sheet, in that frame's own grid numbering (mm)."""
    return tt.on_sheet(px, tt.ARM_K, tt.ARM_D, cv2.Rodrigues(rot)[0], t)[:2]


def dots_near(image: np.ndarray, tip) -> np.ndarray:
    """Isolated black ink dots, with only the needle itself (3 px margin) masked out."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    dark = ((hat > 40) & (gray < 120)).astype(np.uint8)
    dark[cv2.dilate(tt._needle_blob(image).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0] = 0
    count, _, stats, centres = cv2.connectedComponentsWithStats(dark)
    h, w = gray.shape
    paper = np.median(gray[h // 2:, w // 4:3 * w // 4])
    # ink sits on paper: the lamp, box, floor and cable gave "dots" too (live 2026-10-27)
    on_paper = [np.median(gray[max(0, int(y) - 10):int(y) + 11, max(0, int(x) - 10):int(x) + 11]) >= 0.85 * paper
                for x, y in centres]
    pts = np.array([centres[i] for i in range(1, count) if 3 <= stats[i, cv2.CC_STAT_AREA] <= 80
                    and centres[i][1] < FINGER_ROW and on_paper[i]
                    and max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) <= 12]).reshape(-1, 2)
    alone = np.array([np.sum(np.linalg.norm(pts - p, axis=1) < 20) == 1 for p in pts], bool)
    return pts[alone] if len(pts) else pts


def usable(image: np.ndarray) -> bool:
    """Tip found, dots visible, and a solid grid: daylight alone gave 170 crossings and a camera
    height off by 2x (266 vs 133 mm), lamp light 700-1200."""
    try:
        tip = tt.arm_tip(image)
        obj, _, _ = pg.grid_points(image, seed=(image.shape[1] / 2, image.shape[0] * 0.75))
    except (RuntimeError, ValueError, TypeError):
        return False
    paper = np.median(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)[150:330, 200:420])
    return tip[1] > 60 and len(dots_near(image, tip)) >= 2 and len(obj) >= 500 and paper >= 120


def auto_light() -> tuple[int, int]:
    for a, b in LIGHTS:
        hue.set_light("12", a > 0, bri=a or None, ct=250)
        hue.set_light("13", b > 0, bri=b or None, ct=250)
        time.sleep(1.5)  # arm camera auto exposure
        if usable(tt.snap_arm(3)):
            return a, b
    raise RuntimeError("no lamp setting gives a usable arm view")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=Path("evidence/arm_touch"))
    ap.add_argument("--steps", type=int, default=200)
    ap.add_argument("--dot", type=float, nargs=2, help="target dot px (default: nearest beyond the tip)")
    ap.add_argument("--pose", type=int, nargs=6, help=POSE_HELP)
    args = ap.parse_args()
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log, n = [], [0]

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, (np.ndarray, list, tuple)) else (round(v, 2) if isinstance(v, float) else v))
               for k, v in row.items()}, flush=True)

    note(phase="light", lamps=auto_light())
    first = tt.snap_arm(4)
    cv2.imwrite(str(out / "000_arm.png"), first)
    tip0, axis = tt.arm_tip(first, with_axis=True)
    tip0 = np.array(tip0)
    aim = tip0 + AIM_BEYOND * axis
    dots = dots_near(first, tip0)
    if args.dot:
        dot = dots[np.argmin(np.linalg.norm(dots - np.array(args.dot), axis=1))]
    else:  # the dot nearest the aim point, ahead of the tip (up the needle axis)
        ahead = dots[(dots - tip0) @ axis > 0]
        if not len(ahead):
            raise RuntimeError("no dot ahead of the needle tip")
        dot = ahead[np.argmin(np.linalg.norm(ahead - aim, axis=1))]
    rot0, t0, ref = tm.arm_view(first)
    home = -rot0.T @ t0
    target = sheet_xy(dot, rot0, t0)
    note(phase="start", tip_px=tip0, axis=axis, dot_px=dot, dot_sheet=target, cam_mm=home)

    pattern = [dots]  # previous frame's dots, for constellation tracking
    occluded = [0]    # frames running the target has been hidden behind the needle

    def sense(expect=(0.0, 0.0)) -> dict:
        """Tip (required), target dot (constellation-tracked), grid pose if it solves."""
        nonlocal dot
        for attempt in range(3):  # auto exposure can swing for a frame after a move
            n[0] += 1
            img = tt.snap_arm(2)
            cv2.imwrite(str(out / f"{n[0]:03d}_arm.png"), img)
            try:
                tip = np.array(tt.arm_tip(img))
                break
            except RuntimeError as error:
                note(phase="frame-retry", frame=n[0], error=str(error))
                time.sleep(0.5)
        else:
            raise RuntimeError("needle tip not found 3 frames running")
        found = dots_near(img, tip)
        # follow the whole dot pattern, not the nearest blob: a pan backlash snap moved everything ~30 px,
        # past the gap to a neighbour (live 2026-10-27); the grid link slipped whole squares
        shift, hits = (constellation_shift(pattern[0], found, expect) if len(found)
                       else (np.zeros(2), 0))
        if hits < 3:
            raise RuntimeError(f"dot pattern lost: {hits} of {len(pattern[0])} dots matched")
        gaps = np.linalg.norm(found - (dot + shift), axis=1)
        if gaps.min() > 12.0:  # a dot beside the needle loses part of its blob to the mask and its centre shifts
            spot = np.round(dot + shift).astype(int)
            hidden = cv2.dilate(tt._needle_blob(img).astype(np.uint8), np.ones((15, 15), np.uint8))
            inside = 0 <= spot[1] < hidden.shape[0] and 0 <= spot[0] < hidden.shape[1] and hidden[spot[1], spot[0]]
            if not inside or occluded[0] >= 2:
                raise RuntimeError(f"target dot missing at {np.round(dot + shift, 1).tolist()} (nearest {gaps.min():.1f} px)")
            occluded[0] += 1  # behind the needle: carry it with the pattern for up to 2 frames
            note(phase="occluded", frame=n[0], dot_px=dot + shift)
            dot = dot + shift
        else:
            occluded[0] = 0
            dot = found[np.argmin(gaps)]
        pattern[0] = found
        out_row = {"tip": tip, "dot": dot.copy(), "cam": None, "ray": None, "paper_mm": None}
        try:
            rot, t, blobs = tm.arm_view(img)
        except RuntimeError:
            return out_row
        o, _ = tm.lattice_offset(ref, blobs)
        r = unit_ray(tip)
        out_row.update(cam=-rot.T @ t + np.array([*o, 0.0]), ray=rot.T @ r,  # tip ray in the sheet frame
                       paper_mm=float(rot[:, 2] @ t / (rot[:, 2] @ r)))  # camera -> paper along the tip ray
        return out_row

    def lift(reason: str) -> None:
        now = commanded()
        move({k: now[k] + v for k, v in LIFT.items()}, ms=1500)
        note(phase="lift", reason=reason, pulses=read())

    pose = read()
    try:
        # no probing phase: per-joint gains from today's runs, refined from every real move (servo deadband
        # made +-3-pulse probes read zero motion and the Jacobian NaN, live 2026-10-27)
        gain = {"u": GAIN_PAN_U, "v": GAIN_ELBOW_V}
        firsts = [sense() for _ in range(3)]  # free-air tip spot and its jitter
        tips = np.array([f["tip"] for f in firsts])
        free = np.median(tips, axis=0)
        back = -(tips - free) @ axis  # paper pushes the needle back toward the fingers
        contact_px = max(1.5, 4 * float(np.abs(back - np.median(back)).max()))
        note(phase="free-air", tip_free=free, contact_px=contact_px, paper_mm=firsts[-1]["paper_mm"])
        prev = firsts[-1]
        cmd = commanded(args.pose)  # sub-deadband steps accumulate here until the servo moves
        heading = {1: 0, 3: 0}  # last commanded direction per joint, for backlash take-up
        over = 0
        for step in range(args.steps):
            err = prev["dot"] - aim
            inside = (err >= ALIGN_LO) & (err <= ALIGN_HI)
            aligned = bool(inside.all())
            dq = {1: 0, 2: 0, 3: 0}
            if aligned:  # push: shoulder down (user's call) with the elbow solved so v holds and the camera
                # closes PUSH mm on the paper; elbow-only feed-forward cancelled the approach (run 13: -10/+10
                # gave +1.5 mm)
                far = prev["paper_mm"] is not None and prev["paper_mm"] > SLOW_BELOW_MM
                ds, de = np.linalg.solve(SE_TO_VP, [-err[1], -(PUSH_FAR_MM if far else PUSH_NEAR_MM)])
                dq[2], dq[3] = int(round(ds)), int(round(de))
            else:  # inside the tolerance leave that axis alone: chasing sub-step errors made the pan hunt
                if not inside[0]:
                    dq[1] = int(np.clip(np.round(-err[0] / gain["u"]), -MAX_STEP, MAX_STEP)) or int(-np.sign(err[0]))
                if not inside[1]:
                    dq[3] = int(np.clip(np.round(-err[1] / gain["v"]), -MAX_STEP, MAX_STEP)) or int(-np.sign(err[1]))
            for k, d in dq.items():
                cmd[k] += d
                if k in heading and d:
                    if heading[k] and np.sign(d) != heading[k]:
                        cmd[k] += int(np.sign(d)) * BACKLASH[k]  # take up the slack in the same move
                    heading[k] = int(np.sign(d))
            now = move(cmd, ms=250, settle=0.6)  # 0.25 s caught the arm mid-move: double-exposed frames
            moved = {k: now[k] - pose[k] for k in JOINTS}
            pose = now
            # where the dots should have gone, from the servo motion actually read back
            expect = (gain["u"] * moved[1], gain["v"] * moved[3] + SHOULDER_V * moved[2])
            new = sense(expect)
            d_err = new["dot"] - prev["dot"]
            if moved[1] and not moved[2] and not moved[3] and abs(d_err[0]) > 2:  # pan-only move: refine u gain
                gain["u"] = float(np.clip(d_err[0] / moved[1], 1.0, 15.0))
            if moved[3] and not moved[1] and not moved[2] and abs(d_err[1]) > 2:  # elbow-only move: refine v gain
                gain["v"] = float(np.clip(d_err[1] / moved[3], 0.5, 8.0))
            shift = float(-(new["tip"] - free) @ axis)
            over = over + 1 if shift > contact_px else 0
            note(phase="push" if aligned else "align", step=step, dq=list(dq.values()), moved=list(moved.values()),
                 err=new["dot"] - aim, tip_back_px=shift, paper_mm=new["paper_mm"], gain=gain, pulses=pose)
            if over >= 2:  # two frames running: one noisy frame is not a contact (live 2026-10-27)
                note(phase="contact", tip_px=new["tip"], dot_px=new["dot"], shift_px=shift,
                     lam_mm=prev["paper_mm"], paper_mm=new["paper_mm"])  # tip depth: paper distance just before contact
                cv2.imwrite(str(out / "contact_arm.png"), cv2.imread(str(out / f"{n[0]:03d}_arm.png")))
                lift("contact")
                return
            prev = new
        lift("step budget used, no contact")
    except BaseException as error:  # any failure, Ctrl-C included, ends with the needle raised
        note(phase="abort", error=repr(error))
        lift("abort")
        raise


if __name__ == "__main__":
    main()
