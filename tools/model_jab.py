#!/usr/bin/env python3
"""THE JAB placed and solved by the arm model: one model move, image-measured corrections, one fluid jab.

1. One arm frame ties base_link to that frame's sheet (coarse_aim.sheet_from_base); the target is the
   black dot nearest the landing point. 2. Pan/elbow/wrist put the tip --fwd mm behind and --up mm above
   it (model IK, servo 2 held). 3. A new frame measures the tip against the dot (the dot carried across
   frames by the ink marks, lattice_offset), and the model corrects the remainder, up to PLACE_ITERS times.
   Moves are planned relative to the model tip, so the model's absolute error (~5-10 mm) mostly drops out.
4. Final contact (--jab): the wrist turns toward the model arc's landing (--press mm into the paper) in
   WRIST_STEP pulses, arm-camera height read each step, stop at 0.3 mm. 5. The miss is measured from the
   image, then the elbow lifts. Every planned pose is checked against the paper first.

    python tools/model_jab.py --out DIR [--fwd 10] [--up 10] [--press 0.0] [--lam 206] [--plan-only]
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
import aim_touch as am  # noqa: E402
import arm_touch as at  # noqa: E402
import paper_grid as pg  # noqa: E402
import coarse_aim as ca  # noqa: E402
import tip_motion as tm  # noqa: E402
import sam_detect as sam  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import Camera  # noqa: E402
from tip_motion import drive  # noqa: E402
from tip_descend import read, send  # noqa: E402

TIE_MAX_MM = 30.0         # ... and lowered to at most this, where SAM still finds the dots in the arm view
START_MIN_MM = 15.0       # the tip is lifted to at least this before the cameras are tied and anything swings
SAM_THRESHOLD = 0.4       # SAM 3.1 "black dot" score floor
SAM_AREA = (60, 300)      # px: ink dots 81-131 in the UGREEN; red dots 44-59, binder holes 441-819 (live 2026-10-27)
RED_AREA = (25, 120)      # px: red dots 44-59 in the UGREEN
SAM_BLACK_FRAC = 0.08     # darkest pixel / paper: black 0-1, red 12-25 of 65 at dusk
CUBE_MM = 30.0            # the red cube's edge (~6 grid squares in the arm view); its centre sits at half this height
CUBE_TOL_MM = 25.0        # the cube mask centre is rough from the oblique arm camera: 17 mm off (live 2026-10-27)
MATCH_MM = 3.0            # a dot seen by both cameras agrees to this after the frame fit
PLACE_JOINTS = (1, 3, 4)  # pan, elbow, wrist: servo 2 held while placing (it sticks rising under load)
PLACE_ITERS = 30        # along swings +-12 mm near the elbow/wrist deadband ate 20 (live 2026-10-28); pan converges one way at 1.2-1.8 mm/pulse: 36 -> 9.6 mm took 7 steps (live 2026-10-28)
ALONG_OK_MM = 30.0        # the jab covers any along distance up to this from the hover
PLACE_TOL_MM = 1.0        # along the needle and in height
SIDE_TOL_MM = 1.2         # sideways: half a pan pulse (~2.2 mm/pulse here); the jab cannot correct sideways
PAN_BACKLASH = 5          # pulses a pan reversal swallows: +3 moved 16 mm sideways, -5 only 3.8 (live 2026-10-27)
FIRST_MARGIN_MM = 15.0    # the first (big) model move aims this much higher: it came down 6-13 mm too low (live 2026-10-28)
# strafe only where pan cannot align by itself (user): a pan reversal, or pan out of reach; one measured
# pulse at a time, its length not trusted (0 to 18 mm for the same 0.15 s)
STRAFE_BUDGET_S = 0.6     # total strafing per run (user: you can't strafe that much); undone at the end
ALONG_FIRST_MM = 10.0     # along error over this is corrected before any sideways step (arm moves shift sideways too)
SIDE_MIN_MM = 8.0         # pan, roll and strafe only with the tip at least this high
UG_ROUNDS = 6             # placing rounds re-aimed by the UGREEN's landing check
UG_MAX_MM = 15.0          # a larger UGREEN correction is a misfound tip, not a fix
UG_TOL_MM = 1.0           # the UGREEN's landing error accepted
PRACTICE_H = 4.0          # mm: the practice arc stops this high; the extrapolation to the paper covers the rest
UG_TOUCH_MAX_MM = 5.0     # unconverged after the rounds: touch only if the last check is within this (it predicts the mark)
UG_GAIN = 0.5             # share of the UGREEN landing error applied per round
ROLL_REACH_MM = 6.0       # sideways under this is closed by roll, over it by pan: pan sticks, then jumps 13-19 mm
ROLL_MM = 0.26            # mm sideways per roll pulse at the descent pose, deadband 1-2, backlash ~0.5 mm (live 2026-10-28)
ROLL_PX_MAX = 20.0        # arm-camera tip pixel shift accepted per roll step (one step read 11 mm sideways, live 2026-10-28)
ROLL_MAX = 40             # roll pulses from the start pose: the needle's angle in the jaw limits the useful range
PAN_STEP = 2              # sideways past the strafe budget: small pan steps, never reversing
STRAFE_V = 0.05           # m/s base strafe for sideways after the first move (pan ~4 mm/pulse with slack)
SHOULDER_SLIP = 3         # read-back pulses below its start: the shoulder is giving way
PLACE_FLOOR_MM = 5.0      # any measured height under this while placing stops the run
SHOULDER_MAX = 100        # pulses per move, downward only (it does not follow upward under load)
GAIN = 0.35               # corrections after the first move: small elbow/wrist moves go ~2.5x the model at reach, 0.6 swung along -9/+14/-10 (live 2026-10-28)
SHEET_MARGIN_PX = 6       # a target dot lies this far inside the grid-crossings outline
BLACK_GRAY = 80           # arm camera: black ink below this (aim_touch.BLACK_GRAY)
CONTACT_EXTRA = 10        # wrist pulses past the arm camera's paper height: it read -0.1 mm with the tip still above (live 2026-10-28)
CONTACT_LAG = 6           # wrist read-back this far behind: backstop only; 2 and 3 fired in the air (live 2026-10-28), the tip standing still decides
CONTACT_STILL_PX = 1.0    # the UGREEN teal tip moved less than this on 2 running 1-pulse steps: resting on the paper
WRIST_STEP = 3            # pulses per measured wrist step on the final arc
LIFT_ELBOW = 40


def match_frames(src: np.ndarray, dst: np.ndarray, cube_src, cube_dst) -> tuple[np.ndarray, np.ndarray, int]:
    """Rotation (a multiple of 90 deg: both are grid numberings of one sheet) and offset taking `src` sheet mm
    onto `dst` sheet mm. The red cube anchors the offset for each rotation (to CUBE_TOL_MM); the dots seen in
    both views pick the rotation (most within MATCH_MM, anchored near the cube) and refine the offset."""
    best = None
    for k in range(4):
        c, s_ = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)
        r = np.array([[c, -s_], [s_, c]])
        anchor = cube_dst - r @ cube_src
        for p in src:
            for q in dst:
                off = q - r @ p
                if np.linalg.norm(off - anchor) > CUBE_TOL_MM:
                    continue  # this pairing disagrees with the cube
                d = np.linalg.norm((src @ r.T + off)[:, None] - dst[None], axis=2).min(1)
                score = (int((d <= MATCH_MM).sum()), -float(d[d <= MATCH_MM].sum()))
                if best is None or score > best[0]:
                    best = (score, r, off)
    if best is None or best[0][0] < 1:
        raise RuntimeError("no dot seen by both cameras agrees with the cube: frames not tied")
    (n, _), r, off = best
    m = np.linalg.norm((src @ r.T + off)[:, None] - dst[None], axis=2).argmin(1)
    ok = np.linalg.norm(src @ r.T + off - dst[m], axis=1) <= MATCH_MM
    off = (dst[m[ok]] - src[ok] @ r.T).mean(0)  # refined on all matched dots
    return r, off, n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fwd", type=float, default=10.0, help="tip placed this far behind the dot, mm")
    ap.add_argument("--up", type=float, default=10.0, help="tip placed this far above the paper, mm")
    ap.add_argument("--press", type=float, default=0.0, help="the wrist arc ends this far into the paper, mm")
    ap.add_argument("--lam", type=float, default=206.0, help="tip depth along its pixel ray, mm")
    ap.add_argument("--plan-only", action="store_true", help="plan the placing move, do not move")
    ap.add_argument("--rightmost", action="store_true", help="target the notebook's black dot furthest right as the robot sees it (leftmost in the UGREEN)")
    ap.add_argument("--dot", type=float, nargs=2, metavar=("U", "V"), help="target the black dot nearest this UGREEN pixel")
    ap.add_argument("--red", type=float, nargs=2, metavar=("U", "V"), help="target the red dot nearest this UGREEN pixel")
    ap.add_argument("--strafe", action="store_true", help="allow base strafes where pan cannot align")
    ap.add_argument("--jab", action="store_true", help="do the jab onto the paper (default: stop at hover; user: not yet)")
    args = ap.parse_args()
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log, n = [], [0]

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, (np.ndarray, list, tuple)) else v) for k, v in row.items()}, flush=True)

    try:
        note(phase="light", lamps=at.auto_light())
    except RuntimeError as error:  # its "usable" test needs the old tip finder; lamp 13 alone worked every run
        at.hue.set_light("12", False)
        at.hue.set_light("13", True, bri=254, ct=250)
        time.sleep(1.5)
        note(phase="light", lamps=[0, 254], fallback=str(error))
    predictor = sam.load()

    def teal_tip(img: np.ndarray):
        """Lowest pixel of the teal-inked tip in a UGREEN frame (the long thin saturated-teal blob; the paper reads
        S~90 there and the round drips are not long), or None."""
        m = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (75, 130, 50), (105, 255, 255))
        count, lab, st, _ = cv2.connectedComponentsWithStats(m)
        def long_(j):  # rotated rectangle: the angled tip's upright box was only 2.7:1 (live 2026-10-28)
            (_, _), (w, h), _ = cv2.minAreaRect(np.column_stack(np.nonzero(lab == j)[::-1]).astype(np.float32))
            return max(w, h) >= 3 * max(min(w, h), 1.0)
        keep = [j for j in range(1, count) if 30 <= st[j, 4] < 5000 and long_(j)]
        if not keep:
            return None
        # the needle enters from the image top, so its tip is the lowest end; a teal glint near the top took "biggest"
        vs, us = np.nonzero(lab == max(keep, key=lambda j: st[j, 1] + st[j, 3]))
        return np.array([us[vs == vs.max()].mean(), vs.max()], float)

    def sam_needle_tip(img: np.ndarray, name: str, far_up: bool) -> np.ndarray:
        """The needle tip by SAM 3.1 ("thin metal stick": "needle" missed the angled needle at 2 of 3 rolls):
        the far end of the best mask, topmost pixel in the arm view, bottommost in the UGREEN."""
        path = out / f"sam_{name}.png"
        cv2.imwrite(str(path), img)
        found = sam.detect(predictor, str(path), "thin metal stick", 0.15)
        if not found:
            raise RuntimeError(f"SAM 3.1 found no needle in the {name} view")
        vs, us = np.nonzero(max(found, key=lambda d: d["score"])["mask"])
        k = int(np.argmin(vs) if far_up else np.argmax(vs))
        return np.array([us[k], vs[k]], float)

    def triangulate(ug_img, rig, rot, t, rot_u, off_u, tip_cam, how: str, ug_before=None) -> np.ndarray:
        """Tip in the camera frame from both cameras: the arm ray to the tip meets the UGREEN ray to it
        (carried into the arm sheet frame by the tie). A new or angled grasp moves the tip along its ray."""
        uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
        try:
            # SAM misses the thin needle at the UGREEN's image edge; what moved since `ug_before` finds it
            ug_tip_px = (np.array(am.moved_needle(ug_img, ug_before, rig["sheet"])) if ug_before is not None
                         else sam_needle_tip(ug_img, f"ugreen_tip_{how}", far_up=False))
            o_u, e_u = tt.ray(ug_tip_px, uk, ud, ur, ut)
            r3 = np.eye(3)
            r3[:2, :2] = rot_u
            o_a, e_a = r3 @ o_u + np.array([*off_u, 0.0]), r3 @ e_u
            c_a, d_a = -rot.T @ t.ravel(), rot.T @ at.unit_ray(free)
            m_ = np.array([[d_a @ d_a, -d_a @ e_a], [d_a @ e_a, -e_a @ e_a]])
            s_, u_ = np.linalg.solve(m_, [(o_a - c_a) @ d_a, (o_a - c_a) @ e_a])
            gap_ = float(np.linalg.norm(c_a + s_ * d_a - (o_a + u_ * e_a)))
            lam = s_ * np.linalg.norm(d_a)
            note(phase="tip-depth", how=how, lam_mm=lam, rays_miss_mm=gap_, ugreen_tip_px=ug_tip_px)
            if gap_ <= (25.0 if how == "cube" else 6.0) and 120 < lam < 450:
                return lam * at.unit_ray(free)
        except (RuntimeError, np.linalg.LinAlgError) as error:
            note(phase="tip-depth-failed", how=how, error=str(error))
        return tip_cam

    img = tt.snap_arm(3)
    free = sam_needle_tip(img, "arm_tip", far_up=True)  # tip pixel at this roll; the angled needle moves with roll
    tip_cam = args.lam * at.unit_ray(free)  # the tip in the camera frame: rigid to the camera; depth set below
    cmd = {k: int(v) for k, v in read().items()}  # the one read-back; commands are built on `cmd` after this

    def view(pose: dict):
        """(frame, sheet->camera rot, t, ink blobs, sheet-from-base (A, b)) at these pulses."""
        for _ in range(3):
            n[0] += 1
            im = tt.snap_arm(3)
            cv2.imwrite(str(out / f"{n[0]:03d}_arm.png"), im)
            try:
                rot, t, blobs = tm.arm_view(im)
                return im, rot, t, blobs, ca.sheet_from_base(rot, t, pose)
            except RuntimeError:
                time.sleep(0.3)
        raise RuntimeError("grid pose failed 3 frames running")

    def tip_sheet(rot, t) -> np.ndarray:
        return rot.T @ (tip_cam - t)  # sheet frame, z negative above the paper

    def wrist_land(pose: dict, a, b):
        """Turning only the wrist from `pose`: (wrist pulse where the model tip reaches the paper + PRESS,
        landing xy on this frame's sheet), or None. The final contact is this arc (user: a wrist turn
        replaces the jab, which touched down 5-9 mm off and pressed up to 6.7 mm)."""
        z = lambda q: float((a @ ca.tip_in_base(q, tip_cam) + b)[2])  # noqa: E731  sheet z, + into the paper
        step = 1 if z({**pose, 4: pose[4] + 1}) > z(pose) else -1
        for k in range(1, 200):
            q = {**pose, 4: pose[4] + step * k}
            if z(q) >= args.press:
                return pose[4] + step * k, (a @ ca.tip_in_base(q, tip_cam) + b)[:2], step
        return None

    def floor_ok(pose: dict, a, b, floor: float) -> bool:
        """The model tip at `pose` stays above `floor` (sheet mm, positive = into the paper)."""
        return float((a @ ca.tip_in_base(pose, tip_cam) + b)[2]) <= floor

    ugcam = Camera(0)

    def sam_find(img: np.ndarray, name: str, prompt: str, area, dark: float, thresh: float = SAM_THRESHOLD) -> np.ndarray:
        """Pixel centres of SAM 3.1 masks for `prompt` within `area` px whose darkest pixel is below
        `dark` x the paper's brightness."""
        path = out / f"sam_{name}.png"
        cv2.imwrite(str(path), img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        paper = float(np.median(gray[gray > np.percentile(gray, 40)]))
        keep = [d["centroid"] for d in sam.detect(predictor, str(path), prompt, thresh)
                if d["centroid"] is not None and area[0] <= d["area"] <= area[1] and gray[d["mask"]].min() <= dark * paper]
        return np.array(keep, float).reshape(-1, 2)

    def arm_dots(img: np.ndarray, name: str) -> np.ndarray:
        """Tie points in the arm view: black dots plus red dots (a touched black dot is under teal ink: the tie
        matched 0 dots and placing swung +-20 mm, live 2026-10-28). A mask found by both prompts counts once."""
        pts = list(sam_find(img, name, "black dot", (20, 300), 0.4, 0.3))  # 0.4 missed the one dot in view
        for q in sam_find(img, name + "_red", "small red dot", (10, 80), 1.0, 0.3):
            if all(np.linalg.norm(q - p_) > 6.0 for p_ in pts):
                pts.append(q)
        return np.array(pts, float).reshape(-1, 2)

    def cube_centre(img: np.ndarray, name: str, cam_rays) -> np.ndarray:
        """The red cube (user's orientation mark): SAM 3.1's biggest "red cube" mask, its centre placed
        on the cube's mid-height in that camera's sheet frame."""
        c = sam_find(img, name + "_cube", "red cube", (500, 10 ** 7), 1.0)
        if not len(c):
            raise RuntimeError(f"SAM 3.1 found no red cube in the {name} view")
        o, d = cam_rays(c[0])
        return (o + d * ((-CUBE_MM / 2 - o[2]) / d[2]))[:2]

    hold = [False]
    roll_sign, roll_last, arm_tip_px = [1], [None], [free]  # roll pulse direction shrinking +across; (|across| before, step) of the last roll
    strafed, pan_gain, first_pan = [0.0], [0.0], [0, 0.0]  # net strafe (s, +vy); d(side err)/d(pan pulse)
    try:
        # rough depth first (cube-only tie, 180 deg as every tie today): the band loop below must not
        # lower the tip by a wrong (too short) depth into the paper
        ug_pre = ugcam.frame()
        send({3: cmd[3] + 8}, ms=800)  # a nudge up (elbow +) so the UGREEN finds the needle as what moved
        time.sleep(1.3)
        cmd = {**cmd, 3: cmd[3] + 8}
        im0, rot0, t0, _, _ = view(cmd)
        ug0 = ugcam.frame()
        rig0 = tt.ugreen_rig(ug0)
        k0, d0, r0, tr0 = (np.array(rig0[q]) for q in ("k", "d", "r", "t"))
        cu0 = cube_centre(ug0, "ugreen0", lambda q: tt.ray(q, k0, d0, r0, tr0))
        ca0 = cube_centre(im0, "arm0", lambda q: (-rot0.T @ t0.ravel(), rot0.T @ at.unit_ray(q)))
        a0_, b0_ = ca.sheet_from_base(rot0, t0, cmd)
        cube_base = a0_.T @ (np.array([*ca0, -CUBE_MM / 2]) - b0_)  # carried by the model when it leaves the view
        r180 = -np.eye(2)
        tip_cam = triangulate(ug0, rig0, rot0, t0, r180, ca0 - r180 @ cu0, tip_cam, "cube", ug_pre)
        # into the TIE band before the cameras are tied: under 15 mm a swing can drag on the paper (one
        # start was 3 mm up), over 30 mm SAM finds no dots in the arm view (they are too small at 65 mm)
        for _ in range(16):
            im, rot, t, blobs, (a, b) = view(cmd)
            h = -tip_sheet(rot, t)[2]
            if START_MIN_MM <= h <= TIE_MAX_MM:
                break
            step = 8 if h < START_MIN_MM else -int(np.clip((h - 22.0) / 1.5, 3, 12))  # elbow: + raises, ~1.5 mm/pulse
            cmd = {**cmd, 3: cmd[3] + step}
            send({3: cmd[3]}, ms=800)
            time.sleep(1.3)
        else:
            raise RuntimeError(f"could not bring the tip to {START_MIN_MM}-{TIE_MAX_MM} mm before starting")
        # coordinate system, re-initialised with both cameras every start (user): the UGREEN calibrates on
        # the sheet, the arm frame ties base_link to its own sheet frame (above); the two sheet frames are
        # matched on the black dots SAM 3.1 finds in both views. Targets come from the UGREEN (all 3 black
        # dots always in view), carried into the arm frame
        ug_img = ugcam.frame()
        rig = tt.ugreen_rig(ug_img)
        uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
        # UGREEN: ink reads 0.00-0.01 of the paper, red 0.18+; the brighter arm camera reads ink at 0.18-0.27,
        # so there any dot-sized mask is a candidate and the geometry decides
        ug_px = sam_find(ug_img, "ugreen", "black dot", SAM_AREA, SAM_BLACK_FRAC)
        arm_px = arm_dots(im, "arm")
        ug_mm = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in ug_px]).reshape(-1, 2)
        arm_mm = np.array([at.sheet_xy(q, rot, t) for q in arm_px]).reshape(-1, 2)
        known_file = args.out / "ugreen_dots.json"  # the notebook's 3 black dots, from the last good run
        if len(ug_px) > 3 and known_file.exists():  # a 4th "dot" once: the needle tip or a press dent
            known = np.array(json.loads(known_file.read_text()))
            ug_px = ug_px[[np.linalg.norm(known - q, axis=1).min() <= 15.0 for q in ug_px]]
        if 0 < len(ug_px) < 3 and known_file.exists():  # a touched dot is covered by teal ink: fill it from the last good set
            known = np.array(json.loads(known_file.read_text()))
            near = [np.linalg.norm(ug_px - q, axis=1).min() for q in known]
            if sorted(near)[len(ug_px) - 1] <= 15.0:  # the dots still seen sit where they were: same camera, same sheet
                ug_px = np.array([q if near[j] > 15.0 else ug_px[np.argmin(np.linalg.norm(ug_px - q, axis=1))]
                                  for j, q in enumerate(known)])
                note(phase="dots-filled", from_file=int(3 - sum(n <= 15.0 for n in near)))
                ug_mm = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in ug_px]).reshape(-1, 2)
        if len(ug_px) != 3:
            raise RuntimeError(f"SAM 3.1 found {len(ug_px)} black dots on the notebook in the UGREEN, expected 3")
        known_file.write_text(json.dumps(ug_px.tolist()))
        red_px = sam_find(ug_img, "ugreen_red", "small red dot", RED_AREA, 1.0)  # "red dot" finds none, "small red dot" all 5
        ug_tie = np.vstack([ug_mm, np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in red_px]).reshape(-1, 2)])
        cube_u = cube_centre(ug_img, "ugreen", lambda q: tt.ray(q, uk, ud, ur, ut))
        try:
            cube_a = cube_centre(im, "arm", lambda q: (-rot.T @ t.ravel(), rot.T @ at.unit_ray(q)))
        except RuntimeError:  # out of the steeper view at the tie height: from the start frame, via the model
            cube_a = (a @ cube_base + b)[:2]
            note(phase="cube-carried", cube_mm=cube_a)
        rot_u, off_u, matched = match_frames(ug_tie, arm_mm, cube_u, cube_a)
        i = int(np.argmin(ug_px[:, 0])) if args.rightmost else 0  # the robot's right = the UGREEN's left (user)
        if args.dot:
            i = int(np.argmin(np.linalg.norm(ug_px - np.array(args.dot), axis=1)))
        elif not args.rightmost:  # nearest to the landing point
            land = at.sheet_xy(free, rot, t)
            i = int(np.argmin(np.linalg.norm(ug_mm @ rot_u.T + off_u - land, axis=1)))
        tgt_px, tgt_mm = ug_px[i], ug_mm[i]
        if args.red:  # target a red dot (user)
            if not len(red_px):
                raise RuntimeError("SAM 3.1 found no red dot on the notebook in the UGREEN")
            tgt_px = red_px[int(np.argmin(np.linalg.norm(red_px - np.array(args.red), axis=1)))]
            if np.linalg.norm(tgt_px - np.array(args.red)) > 25.0:
                raise RuntimeError(f"no red dot near {args.red}: nearest {tgt_px.tolist()}")
            tgt_mm = tt.on_sheet(tgt_px, uk, ud, ur, ut)[:2]
        target = tgt_mm @ rot_u.T + off_u  # this arm frame's sheet mm
        base_target = [a.T @ (np.array([*target, 0.0]) - b)]  # the target in base_link (3D): the model carries it

        ug_fix = [np.zeros(2)]  # arm-frame offset of the target the UGREEN measured at hover

        def retie(im_, rot_, t_) -> np.ndarray:
            """The target in a new arm frame by tying the cameras again (SAM 3.1) after every move: carrying it
            by matched ink marks failed (2 marks once) or slipped a whole pattern (~50 mm, live 2026-10-27).
            The UGREEN dots do not move."""
            arm_px_ = arm_dots(im_, "arm_retie")
            arm_mm_ = np.array([at.sheet_xy(q, rot_, t_) for q in arm_px_]).reshape(-1, 2)
            try:
                cube_a_ = cube_centre(im_, "arm_retie", lambda q: (-rot_.T @ t_.ravel(), rot_.T @ at.unit_ray(q)))
                r_, o_, n_ = match_frames(ug_tie, arm_mm_, cube_u, cube_a_)
                got = tgt_mm @ r_.T + o_
                if n_ < 2 and base_target[0] is not None:  # one dot + cube jumped the target 17 mm (live 2026-10-28): the carry unless it agrees
                    a_n, b_n = ca.sheet_from_base(rot_, t_, cmd)
                    prior = (a_n @ base_target[0] + b_n)[:2]
                    if np.linalg.norm(got - prior) > 8.0:
                        got = prior
            except RuntimeError:
                # cube out of view after a big move: the rotation stays (grid numbering keeps its orientation),
                # the model's carry of the target picks which dot is which (dots >= 25 mm apart)
                a_n, b_n = ca.sheet_from_base(rot_, t_, cmd)
                prior = (a_n @ base_target[0] + b_n)[:2] if base_target[0] is not None else None
                cands = [q - rot_u @ u_ + rot_u @ tgt_mm for u_ in ug_tie for q in arm_mm_]
                if prior is None:
                    raise RuntimeError("re-tie failed: no cube and no prior for the target")
                # no dots either (too small from high up after the first move): the model's carry, tied at the next view
                best = min(cands, key=lambda c: np.linalg.norm(c - prior)) if cands else prior + 1e3
                got = best if np.linalg.norm(best - prior) <= 8.0 else prior
                n_ = int(np.linalg.norm(best - prior) <= 8.0)
            note(phase="retie", matched=n_)
            a_n, b_n = ca.sheet_from_base(rot_, t_, cmd)
            base_target[0] = a_n.T @ (np.array([*got, 0.0]) - b_n)  # 3D, in base, for the next carry
            return got + ug_fix[0]  # the UGREEN's correction rides on every re-tie
        mark = ug_img.copy()
        for k_, q in enumerate(ug_px):
            cv2.circle(mark, (int(q[0]), int(q[1])), 16, (0, 255, 255) if k_ == i else (0, 0, 255), 3)
        cv2.imwrite(str(out / "dots.png"), mark)  # UGREEN: red = black dots (SAM 3.1), yellow = target
        # tip depth for this grasp, triangulated: the arm ray to the tip meets the UGREEN ray to it (the UGREEN
        # ray carried into this arm frame by the tie); a new or angled grasp moves the tip along its ray
        tip_cam = triangulate(ug_img, rig, rot, t, rot_u, off_u, tip_cam, "dots", ug0)
        note(phase="frames", ugreen_black_px=ug_px, arm_black_px=arm_px, matched=matched, rot_deg=float(
             np.degrees(np.arctan2(rot_u[1, 0], rot_u[0, 0]))), target_ugreen_px=tgt_px, target_mm=target)
        def arm_fwd(pose, a_):
            """Horizontal forward of the arm's vertical plane (pan axis -> tip), on the sheet. Elbow, wrist
            and shoulder move the tip only in this plane; forward along the camera ray instead pushed every
            correction sideways, 5.7 -> 12 -> 18 mm (live 2026-10-27)."""
            tb = ca.tip_in_base(pose, tip_cam)
            d = a_ @ np.array([tb[0], tb[1], 0.0]) / np.hypot(tb[0], tb[1])
            return d[:2] / np.linalg.norm(d[:2])

        fwd = arm_fwd(cmd, a)
        note(phase="start", pose=cmd, target_mm=target, tip_mm=tip_sheet(rot, t), fwd=fwd)

        # 2./3. place: the tip at target - fwd * FWD, UP above the paper; relative model moves from here
        shoulder0 = read()[2]
        roll0 = cmd[5]
        pan_dir, strafe_mm_s = 0, 25.0  # mm/s sideways per +vy second; sign and size learnt from each pulse
        strafed[0] = 0.0
        for rnd in range(UG_ROUNDS):  # placing, then the UGREEN's check of the landing; its error re-aims placing
            gap_fix = None
            for it in range(1 if rnd else 0, PLACE_ITERS + 1):  # re-aims start from hover: a fresh first move left the arc out of reach
                tip = tip_sheet(rot, t)
                land = wrist_land(cmd, a, b)
                if land is None:
                    raise RuntimeError("the wrist arc does not reach the paper from this pose")
                offset = land[1] - tip_sheet(rot, t)[:2]  # model: where the arc lands, relative to the tip now
                want = np.array([*(target - offset), -args.up])  # the arc's landing on the dot
                aim = want - np.array([0.0, 0.0, FIRST_MARGIN_MM if it == 0 else 0.0])
                err = want - tip
                side = np.array([-fwd[1], fwd[0]])
                along, across = float(err[:2] @ fwd), float(err[:2] @ side)
                note(phase="place", it=it, err_mm=err, along_mm=along, side_mm=across, height_mm=-tip[2])
                # the jab goes from the measured tip to the dot whatever the along distance; along corrections went the
                # wrong way (8.8 -> 47 mm, the shoulder may not rise), so placing needs only sideways and height
                if abs(across) <= SIDE_TOL_MM and abs(err[2]) <= 4.0 and abs(along) <= 1.5:  # the arc lands where aimed
                    break
                if it == PLACE_ITERS:
                    raise RuntimeError(f"placing left {np.linalg.norm(err):.1f} mm after {PLACE_ITERS} corrections")
                if roll_last[0] is not None and abs(across) > roll_last[0][0] + 0.3:  # the last roll step went the wrong way
                    roll_sign[0] = -roll_sign[0]
                roll_last[0] = None
                sideways_ok = -tip[2] >= SIDE_MIN_MM and abs(along) <= ALONG_FIRST_MM  # along first: it sat at -57 mm for 10 sideways steps; rule: no sideways motion under 8 mm, height first (pan swung 20 mm at 5.6 mm, live 2026-10-28)
                if it > 0 and sideways_ok and SIDE_TOL_MM < abs(across) <= ROLL_REACH_MM:
                    # the last sideways mm by roll (0.26 mm/pulse); the arm camera does not roll, so the tip pixel is
                    # found again after each step and the tip re-placed on its ray at the same depth
                    d = roll_sign[0] * int(np.sign(across)) * int(np.clip(round(abs(across) / ROLL_MM), 1, 8))
                    if abs(cmd[5] + d - roll0) > ROLL_MAX:
                        raise RuntimeError(f"roll {cmd[5] + d} beyond {ROLL_MAX} pulses from {roll0}")
                    send({5: cmd[5] + d}, ms=600)
                    time.sleep(1.2)
                    cmd = {**cmd, 5: cmd[5] + d}
                    im, rot, t, blobs2, (a, b) = view(cmd)
                    px_ = sam_needle_tip(im, f"arm_tip_roll{it}", far_up=True)
                    if np.linalg.norm(px_ - arm_tip_px[0]) <= ROLL_PX_MAX:  # 8 pulses move it ~2 mm; a jump is SAM's mask end elsewhere
                        tip_cam, arm_tip_px[0] = np.linalg.norm(tip_cam) * at.unit_ray(px_), px_
                    else:
                        note(phase="roll-tip-rejected", px=px_, kept=arm_tip_px[0])
                    shift, hits = tm.lattice_offset(blobs, blobs2)
                    target, blobs = retie(im, rot, t), blobs2
                    roll_last[0] = (abs(across), d)
                    note(phase="roll-step", it=it, roll=cmd[5], step=d, side_before_mm=across)
                    continue
                if it == 1 and first_pan[0]:
                    pan_gain[0] = (across - first_pan[1]) / first_pan[0]
                    note(phase="pan-gain", mm_per_pulse=pan_gain[0])
                left = STRAFE_BUDGET_S - abs(strafed[0])
                step_dir = -int(np.sign(across * pan_gain[0])) if pan_gain[0] else 0  # pan step shrinking the error
                reverse = bool(step_dir) and bool(pan_dir) and step_dir != pan_dir  # a reversal jumps ~20 mm
                if it > 0 and sideways_ok and abs(across) > ROLL_REACH_MM and step_dir and (not reverse or left < 0.15 or not args.strafe):
                    # pan first (user); a reversal takes up the slack in the same move when strafing is not available
                    # (the gateway refused: "drive outside bounded no-reverse envelope", live 2026-10-27)
                    # sized from the measured gain, no automatic slack take-up: 2 + 5 pulses jumped 17 mm across the dot
                    # and pan oscillated 513 <-> 520 (live 2026-10-27); here it moved ~2.4 mm per pulse
                    d = step_dir * int(np.clip(round(abs(across) / max(abs(pan_gain[0]), 0.5)), 1, PAN_STEP))  # +4 jumped 23 mm (live 2026-10-28)
                    before_side = across
                    send({1: cmd[1] + d}, ms=800)
                    time.sleep(1.4)
                    cmd, pan_dir = {**cmd, 1: cmd[1] + d}, step_dir
                    im, rot, t, blobs2, (a, b) = view(cmd)
                    shift, hits = tm.lattice_offset(blobs, blobs2)
                    target, blobs = retie(im, rot, t), blobs2
                    new_side = float(((np.array([*(target - args.fwd * fwd), -args.up]) - tip_sheet(rot, t))[:2]) @ side)
                    # relearnt from every pan step but reversals: backlash eats them, the sign flipped and pan
                    # ping-ponged 503 <-> 507 twelve times (live 2026-10-28)
                    # the sign is the mechanism's and stays; backlash noise flipped it and pan ping-ponged 514 <-> 518 (live 2026-10-28)
                    if abs(new_side - before_side) > 0.5 and not reverse and np.sign(new_side - before_side) == np.sign(d * pan_gain[0]):
                        pan_gain[0] = (new_side - before_side) / d
                    note(phase="pan-step", it=it, pan=cmd[1], step=d, side_after_mm=new_side, mm_per_pulse=pan_gain[0])
                    continue
                if args.strafe and it > 0 and sideways_ok and abs(across) > SIDE_TOL_MM:  # sideways by strafe: pan flipped +-11 mm (live 2026-10-27)
                    sec = float(np.clip(GAIN * across / strafe_mm_s, -min(left, 0.5), min(left, 0.5)))  # gateway: <= 0.5 s a pulse
                    sec = sec if abs(sec) >= 0.15 else 0.15 * np.sign(sec)  # -0.1 s moved nothing 6 times running
                    drive(0.0, STRAFE_V * np.sign(sec), abs(sec))
                    strafed[0] += sec
                    im, rot, t, blobs2, (a, b) = view(cmd)
                    shift, hits = tm.lattice_offset(blobs, blobs2)
                    target, blobs = retie(im, rot, t), blobs2
                    tip = tip_sheet(rot, t)
                    new_across = float(((np.array([*(target - args.fwd * fwd), -args.up]) - tip)[:2]) @ side)
                    seen = across - new_across  # sideways the tip moved toward the target
                    if abs(seen) > 0.3:  # a contradicting sign replaces the guess: averaging kept it wrong twice
                        got = seen / sec
                        strafe_mm_s = got if np.sign(got) != np.sign(strafe_mm_s) else 0.5 * strafe_mm_s + 0.5 * got
                    note(phase="strafe", it=it, sec=sec, side_before_mm=across, side_after_mm=new_across, mm_per_s=strafe_mm_s)
                    err = want - tip  # along and height corrected below in the same iteration (a `continue` starved them)
                    along = float(err[:2] @ fwd)
                    if abs(along) <= PLACE_TOL_MM and abs(err[2]) <= PLACE_TOL_MM:
                        continue
                # pan only for the first, big move; then shoulder/elbow/wrist: elbow and wrist alone wanted -57/+90
                # pulses for 10 mm along the needle (they work against each other there, jab.py trials)
                joints = (1, 2, 3, 4) if it == 0 else (2, 3, 4)  # a dot 47 mm further out needs the shoulder too
                corr = aim - tip
                if it > 0:
                    corr[:2] = (corr[:2] @ fwd) * fwd  # along and height by the arm; sideways is pan/strafe's
                goal = ca.tip_in_base(cmd, tip_cam) + a.T @ (corr * (1.0 if it == 0 else GAIN))  # relative: model offset drops out
                new, residual = ca.solve(cmd, tip_cam, goal, joints)
                if 2 in joints and new[2] > cmd[2] + 0.5:  # the shoulder does not follow upward under load: the tip
                    joints = tuple(k for k in joints if k != 2)  # came down to 0.2 mm when it was told to rise 27
                    new, residual = ca.solve(cmd, tip_cam, goal, joints)
                planned = {**cmd, **{k: int(round(new[k])) for k in joints}}
                dpan = planned[1] - cmd[1]
                if dpan and pan_dir and np.sign(dpan) != pan_dir:
                    planned[1] += int(np.sign(dpan)) * PAN_BACKLASH  # take up the slack in the same move
                pan_dir = int(np.sign(dpan)) or pan_dir
                if it == 0:
                    first_pan[:] = [dpan, across]  # measured after the move: sideways error change per pan pulse
                jumps = {k: planned[k] - cmd[k] for k in joints}  # the shoulder was solved but not checked or sent once
                note(phase="place-plan", it=it, planned=planned, jumps=jumps, residual_mm=residual)
                # elbow/wrist cannot move purely along the needle (it is not quite in the arm's plane): the
                # sideways remainder (3.4 mm once) is left to the next strafe
                big = max(abs(v) for v in jumps.values())
                if big == 0:  # under one pulse everywhere: as close as the servos go (along 1.6 mm once, live 2026-10-28)
                    break
                part = min(ca.MAX_MOVE / big, SHOULDER_MAX / max(-jumps.get(2, 0), 1))
                if part < 1.0 and residual <= 1.0:  # far target (76 mm back once, 104 out): go part of the way, measure, go on
                    planned = {**cmd, **{k: int(round(cmd[k] + part * (planned[k] - cmd[k]))) for k in joints}}
                    jumps = {k: planned[k] - cmd[k] for k in joints}
                    note(phase="place-split", it=it, fraction=part, planned=planned)
                if residual > (1.0 if it == 0 else 5.0) or max(abs(v) for v in jumps.values()) > ca.MAX_MOVE:
                    raise RuntimeError(f"placing move not trusted: residual {residual:.1f} mm, jumps {jumps}")
                if not floor_ok(planned, a, b, -3.0):
                    raise RuntimeError("placing move would take the tip within 3 mm of the paper")
                if args.plan_only:
                    return
                send({k: planned[k] for k in joints}, ms=1500)
                time.sleep(2.3)
                cmd = planned
                im, rot, t, blobs2, (a, b) = view(cmd)
                # the shoulder gave way once under load (272 for 280) and the needle hit the paper hard: stop on a
                # slip or on any low reading while placing (live 2026-10-27)
                now = read()
                if 2 not in joints and now[2] < shoulder0 - SHOULDER_SLIP:
                    raise RuntimeError(f"shoulder slipped to {now[2]} (was {shoulder0})")
                if -tip_sheet(rot, t)[2] < PLACE_FLOOR_MM:
                    raise RuntimeError(f"tip {-tip_sheet(rot, t)[2]:.1f} mm up while placing")
                shoulder0 = now[2] if 2 in joints else shoulder0
                shift, hits = tm.lattice_offset(blobs, blobs2)  # the dot in the new frame's numbering
                target, blobs = retie(im, rot, t), blobs2
                fwd = arm_fwd(cmd, a)  # pan may have turned the plane

            tip = tip_sheet(rot, t)
            land = wrist_land(cmd, a, b)
            note(phase="hover", tip_mm=tip, target_mm=target, wrist_land=land[0], landing_mm=land[1],
                 land_err_mm=target - (land[1] - tip_sheet(rot, t)[:2] + tip[:2]))
            # second opinion from the UGREEN, independent of the arm frame and the camera tie: the tip found as
            # what moved since the start frame, against the target dot's pixel
            ug_now = ugcam.frame()
            cv2.imwrite(str(out / "hover_ugreen.jpg"), ug_now)
            try:
                tip_px = teal_tip(ug_now)  # the inked tip by colour: "darker than before" took a point 53 mm off (live 2026-10-28)
                if tip_px is None:
                    tip_px = np.array(am.moved_needle(ug_now, ug_img, rig["sheet"]))
                o, d = tt.ray(tip_px, uk, ud, ur, ut)
                tip_ug = (o + d * ((tip[2] - o[2]) / d[2]))[:2]  # at the arm camera's height
                gap = tgt_mm - tip_ug
                mark = ug_now.copy()
                cv2.circle(mark, tuple(int(v) for v in tgt_px), 16, (0, 255, 255), 3)
                cv2.drawMarker(mark, tuple(int(v) for v in tip_px), (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
                cv2.imwrite(str(out / "hover_check.jpg"), mark)  # yellow ring = target dot, red cross = tip
                landing_ug = tip_ug + rot_u.T @ (land[1] - tip_sheet(rot, t)[:2])  # where the wrist arc lands, UGREEN sheet
                gap_fix = rot_u @ (tgt_mm - landing_ug)  # dot - landing, in this arm sheet frame
                side_ = np.array([-fwd[1], fwd[0]])
                gap_fix = (gap_fix @ side_) * side_  # sideways only: along it carries the tip-height parallax, and a 2.2 mm
                # along re-aim drove the wrist out of the arc's reach (live 2026-10-28); the arm camera's along held 0.5 mm on dot 1
                note(phase="ugreen-check", tip_px=tip_px, dot_px=tgt_px, dot_minus_tip_mm=gap, dot_minus_landing_mm=gap_fix,
                     horizontal_mm=float(np.linalg.norm(gap)), expected_mm=args.fwd)
            except RuntimeError as error:
                note(phase="ugreen-check-failed", error=str(error))
            if args.jab and gap_fix is not None and np.linalg.norm(gap_fix) <= UG_TOL_MM:
                # practice arc in the air: the model's arc travel was off -5 and +7 mm along (live 2026-10-28); the
                # UGREEN's teal tip travel down to PRACTICE_H, extrapolated to the paper, is the landing
                w0, h0, ug0 = cmd[4], -tip[2], tip_ug
                h = h0
                while h > PRACTICE_H and (land[0] - cmd[4]) * land[2] > 0:
                    cmd = {**cmd, 4: cmd[4] + land[2] * WRIST_STEP}
                    send({4: cmd[4]}, ms=400)
                    time.sleep(0.9)
                    im, rot, t, blobs2, (a, b) = view(cmd)
                    h = -tip_sheet(rot, t)[2]
                low_px = teal_tip(ugcam.frame())
                send({4: w0}, ms=600)
                time.sleep(1.1)
                cmd = {**cmd, 4: w0}
                im, rot, t, blobs, (a, b) = view(cmd)
                target = retie(im, rot, t)
                if low_px is not None and h0 - h > 2.0:
                    o, d = tt.ray(low_px, uk, ud, ur, ut)
                    low = (o + d * ((-h - o[2]) / d[2]))[:2]
                    seen = low + (low - ug0) * h / (h0 - h)
                    gap_fix = rot_u @ (tgt_mm - seen)
                    note(phase="practice-arc", low_mm=h, low_px=low_px, dot_minus_landing_mm=gap_fix)
                else:
                    note(phase="practice-arc-failed", low_mm=h, low_px=low_px)
            if gap_fix is not None and np.linalg.norm(gap_fix) > UG_MAX_MM:
                raise RuntimeError(f"UGREEN landing check {np.linalg.norm(gap_fix):.1f} mm off: tip misfound or tie wrong, see hover_check.jpg")
            if gap_fix is None or np.linalg.norm(gap_fix) <= UG_TOL_MM:
                break
            gap_fix = UG_GAIN * gap_fix  # full fixes swung -12/+10/-8 mm at 40 mm reach (pan stick-slip, live 2026-10-28)
            ug_fix[0] = ug_fix[0] + gap_fix  # the arm tie aimed 12.5 mm beside dot 2 (one dot in view, live 2026-10-28)
            target = target + gap_fix
            note(phase="ugreen-fix", round=rnd, fix_mm=gap_fix, total_mm=ug_fix[0])
        else:  # unconverged: the touch after 5 rounds landed 8.6 mm off, as the last check said (live 2026-10-28)
            if np.linalg.norm(gap_fix / UG_GAIN) > UG_TOUCH_MAX_MM:
                raise RuntimeError(f"UGREEN landing check did not converge in {UG_ROUNDS} rounds")
            note(phase="touch-unconverged", predicted_miss_mm=gap_fix / UG_GAIN)
        if not args.jab:  # the paper is not to be touched yet (user, 2026-10-27)
            hold[0] = True
            return
        # 4. final contact: the wrist turns toward the arc's landing in WRIST_STEP pulses, height measured each
        # step; stop at the paper or at the planned wrist value
        w_end, _, wdir = land
        while (w_end - cmd[4]) * wdir > 0:
            nxt = cmd[4] + wdir * min(WRIST_STEP, abs(w_end - cmd[4]))
            send({4: nxt}, ms=400)
            time.sleep(0.9)
            cmd = {**cmd, 4: nxt}
            im, rot, t, blobs2, (a, b) = view(cmd)
            h = -tip_sheet(rot, t)[2]
            note(phase="wrist", wrist=nxt, height_mm=h)
            if h <= 0.3:
                break
        # the arm-camera height is not a contact signal: 1-pulse steps on until the tip stops in the UGREEN or
        # the wrist stalls; the first contact leaves a teal ink mark, the judge
        prev, contact, stills = teal_tip(ugcam.frame()), None, 0
        for extra in range(1, CONTACT_EXTRA + 1):
            nxt = cmd[4] + wdir
            send({4: nxt}, ms=200)
            time.sleep(0.6)
            cmd = {**cmd, 4: nxt}
            ug_f = ugcam.frame()
            now_px, lag = teal_tip(ug_f), abs(read()[4] - nxt)
            moved = float(np.linalg.norm(now_px - prev)) if now_px is not None and prev is not None else None
            note(phase="contact-step", wrist=nxt, extra=extra, tip_px=now_px, moved_px=moved, lag=lag)
            cv2.imwrite(str(out / f"contact_{extra:02d}_ugreen.jpg"), ug_f)
            stills = stills + 1 if moved is not None and moved < CONTACT_STILL_PX else 0  # 2 running: the tip px jitters ~1 px
            if lag >= CONTACT_LAG or stills >= 2:
                contact = "stall" if lag >= CONTACT_LAG else "still"
                break
            prev = now_px
        note(phase="contact", by=contact, wrist=cmd[4])
        im, rot, t, blobs2, (a, b) = view(cmd)
        cv2.imwrite(str(out / "contact_arm.png"), im)
        ug_contact = ugcam.frame()
        cv2.imwrite(str(out / "contact_ugreen.jpg"), ug_contact)
        cu, cv_ = (int(v) for v in tgt_px)
        crop = ug_contact[max(0, cv_ - 120):cv_ + 120, max(0, cu - 160):cu + 160].copy()
        cv2.circle(crop, (min(cu, 160), min(cv_, 120)), 22, (0, 255, 255), 2)
        cv2.imwrite(str(out / "contact_ugreen_zoom.png"), cv2.resize(crop, None, fx=2, fy=2))
        target_now = retie(im, rot, t)
        tip = tip_sheet(rot, t)
        note(phase="touch", by="wrist", miss_mm=target_now - tip[:2], height_mm=-tip[2], wrist=cmd[4], readback=read())
        return
    except BaseException as error:
        note(phase="abort", error=repr(error))
        raise
    finally:
        if hold[0]:  # stay at hover for the user to look
            note(phase="holding", pose=cmd)
            return
        if abs(strafed[0]) >= 0.05:  # put the base back where it started
            drive(0.0, -STRAFE_V * np.sign(strafed[0]), min(abs(strafed[0]), 0.5))
            if abs(strafed[0]) > 0.5:
                drive(0.0, -STRAFE_V * np.sign(strafed[0]), abs(strafed[0]) - 0.5)
            note(phase="strafe-back", seconds=-strafed[0])
        if not args.plan_only:
            send({3: cmd[3] + LIFT_ELBOW}, ms=1500)  # elbow only: re-sending servo 2 read-back sinks it
            time.sleep(2.2)
            note(phase="lift", readback=read())


if __name__ == "__main__":
    main()
