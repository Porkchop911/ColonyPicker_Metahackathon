#!/usr/bin/env python3
"""Put the held needle's tip on an ink dot, arm camera only, with the tip depth known from a touch-off.

The needle is rigid to the arm camera: its tip sits LAM mm out along the free-air tip pixel's ray
(touch-off by arm_touch / plunge, live 2026-10-27: 202.5 mm; 206.0 after the gripper reopened, h = 0 at the contact frame). Each frame's
grid pose gives the tip's landing point (its pixel ray meets the paper there, whatever the true depth)
and the target dot, both in that frame's numbering; the target is predicted from its offset to the
landing point and locked to a dot within LOCK_MM. State (dot x - land x, dot y - land y, tip
height) over pan / elbow / wrist (--joints). Procedure (docs/needle_on_dot.md, Steps 2-4): align at SAFE_MM
to ALIGN_MM, finish every joint from its final direction, descend with elbow + wrist only in the
combination that holds y, blind below HOVER_MM, stop on push-back or at FLOOR_MM, lift.

    python tools/aim_touch.py --out DIR [--lam 202.5] [--dot U V] [--hover] [--pose P1 P2 P3 P4 P5 P10]
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
from rig_io import Camera  # noqa: E402
from tip_descend import POSE_HELP, commanded, go_start, move, read, send  # noqa: E402

JOINTS = (1, 3, 4)  # pan, elbow, wrist: servo 2 sagged and stopped following upward moves at low reach (live 2026-10-27)
# (x, y, height) mm per (pan, elbow, wrist) pulse, probed live 2026-10-27 aim run 5
J0 = np.array([[2.95, -0.11, -0.13], [-0.11, 0.47, 0.89], [0.0, 0.33, 0.42]])
MAX_STEP = 8
# shortcut: seeds from the 2026-10-27 teleop logs, one value for all joints; replace with the Step 1 numbers
DEADBAND = 2       # pulses; smaller align steps are dropped (2-pulse steps after a reversal never moved)
BACKLASH = 5       # pulses a reversal swallows (first 5-pulse click after a reversal: 0 moved)
ALIGN_MM = 0.5     # xy alignment at SAFE_MM before any descent
REALIGN_MM = 1.0   # a seen error this large on the way down: back to aligning
NOISE_MM = 0.2     # image error changes below this are not motion: no Jacobian update
# shortcut: LOAD_PULSES is a guess (read-back was never logged at a contact); calibrate from the `load` column
LOAD_PULSES = 6    # elbow/wrist lagging their command this much beyond the free-air lag = pressing on something
FREE_STEPS = 3     # first align steps, high above the paper: the free-air lag baseline
BLACK_GRAY = 80    # black ink reads below this in gray; red dots and holes lighter (as in ugreen_rig)
PAIR_MM = 3.0      # pair the UGREEN target once the arm camera has the tip this close to it
UG_PAIR_MM = 100.0  # the UGREEN goes for the nearest black dot within this (any black dot will do)
BLACK_NEAR_MM = 20.0  # arm camera: prefer a black target only this close to the landing point
UG_BLACK_FRAC = 0.12  # UGREEN: black ink reads 0-2, red 22-47 of a ~180 paper in daylight (live 2026-10-27)
UG_START_MM = 8.0  # the UGREEN takes over from this height above the paper
UG_OK_MM = 1.0     # descend only while the UGREEN tip-to-dot error is under this
UG_DROP_MM = 0.3   # height per descent step under UGREEN control (1 mm above 3 mm)
UG_PROBE_MM = 2.0  # a probe repeats until the UGREEN sees the tip move this much (0.4 mm was too small to use)
UG_LOW_MM = 5.0    # an error over UG_BACK_MM below this height: climb back to UG_START_MM first
UG_BACK_MM = 3.0
UG_GO_MM = 1.5     # drop (with the correction in the same step) while the UGREEN error is under this   # ... and a black dot lies this close to the UGREEN tip (dots are >= 14 mm apart)
UG_MIN_PAIRS = 4   # moves seen by the UGREEN before its Jacobian is trusted
HIDE_MM = 8.0      # a dot this close to the landing point sits under the needle tip, which dots_near masks
                   # (live 2026-10-27: tracked 48 -> 11 mm, then "lost" at 6 mm): unseen there = hidden
MAX_LOCK_MM = 12.0  # widest window after a long move; two dots inside it = ambiguous, not seen
LOCK_MM = 6.0      # re-detection window, under half the dot spacing (nearest detected dots 14-30 mm apart, 2026-10-27 frames)
APPROACH_MM = 15.0  # come down to this before picking the target
SAFE_MM = 8.0      # align at or above this height, never skimming the paper
HOVER_MM = 3.0     # below this the needle hides the dot: descend blind, no xy corrections
FLOOR_MM = -0.5    # model height safety floor; the -2 mm press is gone
STOP_PX = 8.0      # push-back this strong = contact: the dot beside the tip faked 5 px with no touch (live 2026-10-27)
STRAFE_V = 0.05     # m/s base strafe for x trims below one pan step
STRAFE_S, STRAFE_MIN_S = 0.15, 0.1  # pulse length, halved after an overshoot (tip_motion used >= 0.1 s)
STRAFE_PULSES = 8
LIFT_ELBOW = 40    # elbow + raises the camera here (servo 2 sags when the arm reaches low)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--lam", type=float, default=206.0, help="tip depth along its pixel ray, mm")
    ap.add_argument("--dot", type=float, nargs=2, help="target dot px (default: nearest the tip's foot)")
    ap.add_argument("--steps", type=int, default=150)
    ap.add_argument("--joints", type=int, nargs=3, default=JOINTS, help="servos driving (x, y, height)")
    ap.add_argument("--pose", type=int, nargs=6, help=POSE_HELP)
    ap.add_argument("--no-strafe", action="store_true", help="x by pan only, the base stays put")
    ap.add_argument("--no-ugreen", action="store_true", help="arm camera only, no overhead check on the last stretch")
    ap.add_argument("--hover", action="store_true", help="dry run: stop at HOVER_MM, grab a UGREEN frame, lift")
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

    note(phase="go-start", pulses=go_start(args.pose))
    try:
        note(phase="light", lamps=at.auto_light())
    except RuntimeError as error:  # its "usable" test wants most of the grid in view; the touch needs less
        at.hue.set_light("12", False)
        at.hue.set_light("13", True, bri=254, ct=250)  # the setting auto_light chose in every run on 2026-10-27
        time.sleep(1.5)
        note(phase="light", lamps=[0, 254], fallback=str(error))
    first = tt.snap_arm(3)
    free, axis = tt.arm_tip(first, with_axis=True)
    free, axis = np.array(free), np.array(axis)
    ray = at.unit_ray(free)
    offset = [None]  # target - tip foot, sheet mm: carried frame to frame, no numbering link needed
    vecs = [None]    # other dots - target, sheet mm, from the last frame the target itself was found

    def sense(expects=(), windows=None) -> dict:
        """Tip back-shift, tip in this frame's sheet frame, and the target's sheet xy in the same frame.
        The target is predicted from the tip (`expects`: candidate offsets from the tip foot) and locked
        to a dot within LOCK_MM of one; linking grid numberings across frames slipped by whole squares
        (live 2026-10-27: a 2-pulse move "moved" the target 15 mm). No dot, or the candidates on
        different dots: from the neighbour dots (the target hides under the tip once aligned), else
        unseen, goal = the first candidate."""
        for _ in range(3):  # a frame whose grid pose does not solve is retaken
            n[0] += 1
            img = tt.snap_arm(2)
            cv2.imwrite(str(out / f"{n[0]:03d}_arm.png"), img)
            try:
                rot, t, _ = tm.arm_view(img)
                if -(rot.T @ (args.lam * ray - t))[2] > -20:  # a pose putting the tip 96 mm under the paper
                    break                                      # was taken as real (live 2026-10-27)
            except RuntimeError:
                pass
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
        goal, seen = (foot + expects[0] if len(expects) else None), False
        i = lock(pts - foot, expects, windows)
        if i is not None:
            goal, seen = pts[i], True
            vecs[0] = np.delete(pts, i, axis=0) - goal
        elif vecs[0] is not None and goal is not None and (est := neighbours(pts, goal, vecs[0])) is not None:
            goal, seen = est, True
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
        before, now = read(), commanded()
        send({**now, 3: now[3] + LIFT_ELBOW}, ms=1500)
        time.sleep(2.2)
        after = read()
        note(phase="lift", reason=reason, lifted=after[3] - before[3] >= 20, pulses=after)

    try:
        s = sense()
        # come down to APPROACH_MM first: from high up the landing point lies far ahead and swings with
        # every step, and the target picked there was 57 mm off and never re-found (live 2026-10-27)
        jac, cmd = J0.copy(), commanded(args.pose)
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
        else:  # black ink only when there is one (user, 2026-10-27): red reads lighter
            gray = cv2.cvtColor(s["img"], cv2.COLOR_BGR2GRAY)
            black = [int(gray[int(v), int(u)] < BLACK_GRAY) for u, v in s["found"]]
            gaps = np.linalg.norm(s["pts"] - s["foot"], axis=1)
            # a far black dot was lost on the way (68 mm, live 2026-10-27): only near ones; the UGREEN
            # finish moves on to the nearest black dot anyway
            near_black = [g if b and g <= BLACK_NEAR_MM else np.inf for g, b in zip(gaps, black)]
            i = int(np.argmin(near_black)) if np.isfinite(min(near_black, default=np.inf)) else int(np.argmin(gaps))
        dot = s["found"][i]
        s["goal"] = s["pts"][i]

        def state(r: dict) -> np.ndarray:
            return np.array([*(r["goal"] - r["foot"]), -r["tip"][2]])

        x = state(s)
        offset[0] = x[:2]
        note(phase="start", tip_px=free, dot_px=dot, err_mm=x[:2], height_mm=x[2])
        # seeded, not probed: probes move the target ~24 mm relative to the landing point, out of any
        # snap window (live 2026-10-27); refined in touch() from steps where the dot was really re-found
        ug = {} if args.no_ugreen else ugreen_view(note, out)
        reason, r = touch(sense, state, note, jac, joints, offset[0], args.steps, args.hover,
                          None if args.no_strafe else tm.drive, ug.get("measure"))
        if reason == "contact":
            cv2.imwrite(str(out / "contact_arm.png"), r["img"])
        if reason == "hover":  # second view: tip vs dot on the sheet, compared offline (Step 3)
            try:
                cv2.imwrite(str(out / "hover_ugreen.jpg"), (ug.get("cam") or Camera(0)).frame())
            except Exception as error:  # noqa: BLE001 - the dry run still counts without the second view
                note(phase="ugreen-failed", error=repr(error))
        lift(reason)
    except BaseException as error:  # any failure, Ctrl-C included, ends with the needle raised
        note(phase="abort", error=repr(error))
        lift("abort")
        raise


def moved_needle(img: np.ndarray, before: np.ndarray, sheet, with_axis: bool = False):
    """Lowest pixel of the needle in a UGREEN frame, found as what moved since `before` (taken before
    an arm move): the biggest long, thin component of the frame difference. Shape rules alone took the
    sheet's edge shadow or a binder hole; a "dark now" test failed at dusk; the shadow moves too but as
    a soft blob (live 2026-10-27)."""
    # not masked to the sheet hull: from grid crossings it began 260 px down at dusk, above it the needle
    keep = np.zeros(img.shape[:2], np.uint8)
    cv2.fillConvexPoly(keep, np.asarray(sheet, np.int32), 1)
    g, g0 = (cv2.GaussianBlur(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), (3, 3), 0) for f in (img, before))
    paper = np.median(g[keep > 0])
    # darker now than before = where the needle is now (the absolute difference held both positions)
    mask = ((g0.astype(np.int16) - g) > max(8.0, 0.12 * paper)).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)))
    score = [stats[i, cv2.CC_STAT_AREA] if max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
             >= 3 * min(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) else 0 for i in range(1, count)]
    if not max(score, default=0):
        raise RuntimeError("UGREEN: nothing long and thin moved on the sheet")
    vs, us = np.nonzero(labels == 1 + int(np.argmax(score)))
    tip = (float(us[vs == vs.max()].mean()), float(vs.max()))
    if not with_axis:
        return tip
    pts = np.column_stack([us, vs]).astype(float)
    axis = np.linalg.svd(pts - pts.mean(0), full_matrices=False)[2][0]
    return tip, axis if axis @ (np.array(tip) - pts.mean(0)) > 0 else -axis  # unit, shaft -> tip


def needle_bottom(img: np.ndarray, sheet) -> tuple[float, float]:
    """Lowest pixel of the needle in a UGREEN frame: the biggest long, thin dark blob on the sheet that
    touches the sheet's edge."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    keep = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(keep, np.asarray(sheet, np.int32), 1)
    dark = ((gray < 0.55 * np.median(gray[keep > 0])) & (keep > 0)).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    if count < 2:
        raise RuntimeError("UGREEN: no dark blob on the sheet")
    # it reaches in over the sheet edge and is long and thin; the biggest blob anywhere was a binder hole,
    # and "touches the image top" failed once the base had moved and it entered over the side (live 2026-10-27)
    edge = cv2.dilate(keep, np.ones((3, 3), np.uint8)) - cv2.erode(keep, np.ones((9, 9), np.uint8))
    score = []
    for i in range(1, count):
        w, h = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
        touches = bool(edge[labels == i].any())
        score.append(stats[i, cv2.CC_STAT_AREA] if touches and max(w, h) >= 3 * min(w, h) else 0)
    if not max(score, default=0):
        raise RuntimeError("UGREEN: no needle reaching over the sheet edge")
    vs, us = np.nonzero(labels == 1 + int(np.argmax(score)))
    return float(us[vs == vs.max()].mean()), float(vs.max())


def on_paper(gray: np.ndarray, centre, paper: float) -> bool:
    """A blob with paper round it: the binder rings read as dark as ink at dusk but sit on the desk,
    and the sheet hull from grid crossings cut the dots off once the light fell (live 2026-10-27)."""
    u, v = int(centre[0]), int(centre[1])
    ring = gray[max(0, v - 20):v + 21, max(0, u - 20):u + 21]
    return float(np.median(ring)) >= 0.8 * paper


def ug_black(img: np.ndarray, rig: dict) -> tuple[np.ndarray, np.ndarray]:
    """Black ink dots in a UGREEN frame: (sheet mm, px). ugreen_rig's shape test without its isolation
    filter, which dropped the target black dot beside a red one (live 2026-10-27)."""
    k, d, rr, tr = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((21, 21), np.uint8))
    count, labels, stats, centres = cv2.connectedComponentsWithStats((hat > 25).astype(np.uint8))
    keep = np.zeros(gray.shape, np.uint8)
    cv2.fillConvexPoly(keep, np.asarray(rig["sheet"], np.int32), 1)
    paper = float(np.median(gray[keep > 0]))
    xs = np.asarray(rig["sheet"])[:, 0]  # within the sheet's left-right extent: the box beside it is white too
    mm, px = [], []
    for i in range(1, count):
        w, h, area = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        if 30 <= area <= 400 and max(w, h) < 30 and max(w, h) < 2.2 * min(w, h) and area > 0.4 * w * h \
                and gray[labels == i].min() < UG_BLACK_FRAC * paper and on_paper(gray, centres[i], paper)                 and xs.min() <= centres[i][0] <= xs.max():
            mm.append(tt.on_sheet(centres[i], k, d, rr, tr)[:2])
            px.append(centres[i])
    return np.array(mm).reshape(-1, 2), np.array(px).reshape(-1, 2)


def ugreen_view(note, out: Path) -> dict:
    """The overhead UGREEN as a second view of tip and target: {"cam", "measure"}; {} if it is not
    usable. measure(h, x_arm) -> target - tip on the sheet, mm, UGREEN sheet frame, or None. The
    target dot is paired once the arm camera has the tip within PAIR_MM of it: the nearest black dot
    to the tip's point on the sheet."""
    try:
        cam = Camera(0)
        first = cam.frame()
        rig = tt.ugreen_rig(first)
    except Exception as error:  # noqa: BLE001 - no second view: the arm camera alone, as before
        note(phase="ugreen-off", error=repr(error))
        return {}
    k, d, rr, tr = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
    dots_px = cv2.projectPoints(np.array([[*q, 0.0] for q in rig["dots"]]), rr, tr, k, d)[0][:, 0]
    black, black_px = ug_black(first, rig)
    dots_px = np.vstack([dots_px, black_px])  # never take a dot for the needle point
    st = {"near": None, "dot": None, "n": 0, "before": first}
    note(phase="ugreen-on", rms=rig["rms"], black=len(black))

    def measure(h: float, x_arm=None) -> np.ndarray | None:
        img = cam.frame()
        st["n"] += 1
        cv2.imwrite(str(out / f"ug_{st['n']:03d}.jpg"), img)
        try:  # first look: what moved since the frame before the last arm move
            pu = tt.ugreen_tip(img, st["near"] or moved_needle(img, st["before"], rig["sheet"]), radius=90,
                               exclude=dots_px, sheet=rig["sheet"])
        except RuntimeError as error:
            note(phase="ugreen-no-tip", error=str(error))
            st["before"] = img
            return None
        st["near"] = pu
        c, ray_ = tt.ray(pu, k, d, rr, tr)
        tip = (c + ray_ * ((-h - c[2]) / ray_[2]))[:2]  # the tip at its height, on the sheet
        if st["dot"] is None:
            # this frame's black dots too: the needle or its shadow hid one in the first frame (live 2026-10-27)
            seen_now = np.vstack([black, ug_black(img, rig)[0]])
            gaps = np.linalg.norm(seen_now - tip, axis=1) if len(seen_now) else np.array([np.inf])
            near_arm = x_arm is None or np.hypot(*x_arm) <= PAIR_MM  # None: final stretch, pair now
            if not near_arm or gaps.min() > UG_PAIR_MM:
                if near_arm:
                    note(phase="ugreen-unpaired", gap_mm=float(gaps.min()), tip_mm=tip, height_mm=h)
                return None
            st["dot"] = seen_now[int(np.argmin(gaps))]
            note(phase="ugreen-paired", dot_mm=st["dot"], tip_px=pu, gap_mm=float(gaps.min()))
        return st["dot"] - tip

    return {"cam": cam, "measure": measure}


def neighbours(pts: np.ndarray, guess: np.ndarray, vecs: np.ndarray) -> np.ndarray | None:
    """Target from the visible neighbour dots, which move rigidly with the sheet: each dot found within
    LOCK_MM of guess + its stored vector votes target = dot - vector; the median of the votes."""
    votes = [p - v for v in vecs for p in pts if np.linalg.norm(p - guess - v) <= LOCK_MM]
    return np.median(votes, axis=0) if votes else None


def lock(rel: np.ndarray, expects, windows=None) -> int | None:
    """Index of the dot (offsets from the tip foot, mm) within LOCK_MM of the first expected offset that
    has one, most likely first; None if none has, or two dots share that window. Demanding all
    candidates agree dropped the target whenever a long pan step put "not moved" on a neighbour (simulator)."""
    for e, w in zip(expects, windows or [LOCK_MM] * len(expects)):
        near = np.flatnonzero(np.linalg.norm(rel - e, axis=1) <= w) if len(rel) else []
        if len(near):
            return int(near[0]) if len(near) == 1 else None
    return None


def touch(sense, state, note, jac: np.ndarray, joints: tuple, start: np.ndarray, steps: int,
          hover: bool, strafe=None, ugreen=None) -> tuple[str, dict]:
    """Align at SAFE_MM, finish each joint from its final direction, trim x by strafing the base
    (`strafe(vx, vy, seconds)`, None = pan only), descend until contact, trim again at HOVER_MM before
    the blind part. Returns (reason, last sense). Raises when the target is lost: the caller lifts."""
    cmd, heading = commanded(), {j: 0 for j in joints}
    side = [1.0]  # which vy sign reduces x; learnt from the first pulse that went the wrong way

    def trim(x: np.ndarray, r: dict, aim: float = 0.0) -> tuple[np.ndarray, dict]:
        """x below one pan step: strafe pulses toward x = aim, measured after each. Mecanum slip makes the
        pulse length unpredictable (0.05 m/s x 0.3 s moved 3.4 mm, plan.md), so halve it after an overshoot."""
        seconds, last = STRAFE_S, None
        for _ in range(STRAFE_PULSES):
            e = x[0] - aim
            if strafe is None or abs(e) <= ALIGN_MM:
                break
            if last is not None and np.sign(e) == np.sign(last) and abs(e) > abs(last) + NOISE_MM:
                side[0] = -side[0]
            elif last is not None and np.sign(e) != np.sign(last):
                seconds = max(seconds / 2, STRAFE_MIN_S)
            last = e
            strafe(0.0, -STRAFE_V * side[0] * np.sign(e), seconds)
            r = sense([x[:2]])
            if not r["seen"]:  # hidden under the tip: a strafe can't be measured, stop trimming
                note(phase="strafe-blind", err_mm=x[:2])
                break
            x = state(r)
            note(phase="strafe", seconds=seconds, side=side[0], err_mm=x[:2], height_mm=x[2])
        return x, r

    def step(raw) -> tuple[np.ndarray, bool]:
        """Command an align step: pan 0 or DEADBAND..MAX_STEP pulses, elbow/wrist rounded; backlash taken up on a
        reversal and not counted as motion. Returns (pulses meant to move, any reversal)."""
        dq, flipped = np.zeros(len(joints)), False
        for i, j in enumerate(joints):
            if i == 0:  # pan alone: 0 or DEADBAND..MAX_STEP
                d = 0 if abs(raw[i]) < DEADBAND / 2 else int(np.sign(raw[i]) * min(max(round(abs(raw[i])), DEADBAND), MAX_STEP))
            else:  # elbow/wrist work as a pair: zeroing one drove the tip 5 mm down per step (live 2026-10-27)
                d = int(np.clip(round(raw[i]), -MAX_STEP, MAX_STEP))
            if d:
                take = BACKLASH if heading[j] and np.sign(d) != heading[j] else 0
                flipped |= bool(take)
                cmd[j] += d + int(np.sign(d)) * take
                heading[j], dq[i] = int(np.sign(d)), d
        move(cmd, ms=300, settle=0.6)
        return dq, flipped

    def solve(x: np.ndarray, dh: float) -> np.ndarray:
        """Pan for x; elbow + wrist for y and the height change dh, scaled as a pair. A joint-wise clip of
        the full 3x3 solve broke the elbow/wrist balance and the pan step that offset it (simulator)."""
        pan = -x[0] / jac[0, 0]
        ew = np.linalg.solve(jac[1:, 1:], np.array([-x[1], dh]) - jac[1:, 0] * pan)
        return np.array([pan, *(ew * min(1.0, MAX_STEP / max(np.abs(ew).max(), 1e-9)))])

    def align_raw(x: np.ndarray, seen: bool = True) -> np.ndarray:
        # never lower the needle toward a dot that is not confirmed (live 2026-10-27: 15 -> 7.7 mm unseen)
        return solve(x, float(np.clip(SAFE_MM - x[2], 0.0 if not seen else -1.0, 1.0)))

    def tol() -> np.ndarray:
        """xy tolerance: ALIGN_MM, but x no finer than half a pan step (pan moves the landing point
        ~3 mm per pulse: J0, teleop 2026-10-27), or the descent re-aligns forever (simulator)."""
        return np.array([max(ALIGN_MM, abs(jac[0, 0]) * DEADBAND / 2), ALIGN_MM])

    def done(x: np.ndarray) -> bool:
        """Aligned, or no step the servos can take would help."""
        stuck = all(abs(v) < DEADBAND / 2 for v in align_raw(x))
        return (bool(np.all(np.abs(x[:2]) <= tol())) or stuck) and abs(x[2] - SAFE_MM) <= 1.5

    def neutral(dh: float) -> np.ndarray:
        """Elbow + wrist pulses that change the height by dh and hold y (pan still)."""
        raw = np.linalg.solve(jac[1:, 1:], [0.0, dh])
        return raw * min(1.0, MAX_STEP / np.abs(raw).max())

    def finish(x: np.ndarray, r: dict, n: int) -> tuple[str, dict]:
        """Last stretch steered by the UGREEN, which sees tip and dot together: strafe for one sheet
        direction, the elbow/wrist pair for the other, each column learnt from probes big enough to see.
        The height is held in closed loop on every step (arm-camera grid pose) and its row of the
        Jacobian is updated from each pair move: open-loop corrections sank the tip 4 mm per step
        (live 2026-10-27). No pan, no re-aligning on arm-camera sightings."""
        st = {"x": x, "r": r, "goal": max(x[2], UG_START_MM)}

        def look():
            e = ugreen(st["x"][2])
            if e is None:
                raise RuntimeError("UGREEN lost the tip or the dot")
            return e

        def apply(sec: float, ymm: float) -> None:
            """Strafe `sec` (signed), move arm-camera y by `ymm` with the height brought to the goal; re-sense."""
            if strafe is not None and abs(sec) >= STRAFE_MIN_S / 2:
                strafe(0.0, STRAFE_V * np.sign(sec), float(np.clip(abs(sec), STRAFE_MIN_S, 0.3)))
            h0 = st["x"][2]
            d = np.linalg.solve(jac[1:, 1:], [ymm, float(np.clip(st["goal"] - h0, -1.0, 1.0))])
            d = np.round(d * min(1.0, MAX_STEP / max(np.abs(d).max(), 1e-9)))
            if d.any():
                for k, v in zip(joints[1:], d):
                    cmd[k] += int(v)
                move(cmd, ms=300, settle=0.6)
            st["r"] = sense([st["x"][:2]])
            st["x"] = state(st["r"])
            if d.any():  # height row from the measured change (y is the UGREEN's business)
                row = jac[2, 1:] + (st["x"][2] - h0 - jac[2, 1:] @ d) * d / (d @ d)
                # keep the elbow/wrist block invertible: one update made it singular (simulator)
                if abs(np.linalg.det(np.vstack([jac[1, 1:], row]))) > 0.3 * abs(np.linalg.det(J0[1:, 1:])):
                    jac[2, 1:] = row

        def probe(sec: float, ymm: float) -> np.ndarray:
            """UGREEN error change per unit command, repeating the command until it moved > UG_PROBE_MM."""
            e0, total = look(), 0.0
            for _ in range(3):
                apply(sec, ymm)
                total += 1
                e1 = look()
                if np.hypot(*(e1 - e0)) > UG_PROBE_MM:
                    break
            return (e1 - e0) / (total * (sec or ymm))

        apply(0.0, 4.0)  # a nudge so the UGREEN finds the needle as the thing that moved
        apply(0.0, 4.0)  # (one 2 mm nudge left nothing visible, live 2026-10-27)
        jug = np.column_stack([probe(0.3, 0.0), probe(0.0, 3.0)])  # per strafe second, per y mm
        cond = float(np.linalg.cond(jug / np.linalg.norm(jug, axis=0)))  # columns differ in units: compare directions
        note(phase="ugreen-probe", jac=jug, cond=cond, height_mm=st["x"][2])
        if strafe is None or not np.isfinite(cond) or cond > 10:
            raise RuntimeError(f"UGREEN probes did not move the tip usefully: {jug.round(2).tolist()}")
        for m in range(80):
            e, x = look(), st["x"]
            act = "correct"
            if np.hypot(*e) > UG_BACK_MM and x[2] < UG_LOW_MM:  # never chase a big error low over the paper
                st["goal"], act = UG_START_MM, "back-up"
            elif np.hypot(*e) <= UG_GO_MM:  # close enough: drop, correcting in the same step
                dh = -(1.0 if x[2] > HOVER_MM else UG_DROP_MM)
                if x[2] + dh < FLOOR_MM:  # the floor is checked before the step, not after it
                    note(phase="touch", by="floor-ahead", ug_err_mm=e, height_mm=x[2])
                    return "contact", st["r"]
                st["goal"], act = x[2] + dh, "drop"
            # separate correct and drop steps churned: each drop added ~0.7 mm (simulator)
            sec, ymm = np.linalg.solve(jug, -e) if np.hypot(*e) > UG_OK_MM / 3 else (0.0, 0.0)
            apply(float(np.clip(sec, -0.3, 0.3)), float(np.clip(ymm, -3.0, 3.0)))
            r = st["r"]
            note(phase="ugreen-" + act, step=n + m, ug_err_mm=e, height_mm=st["x"][2], goal_mm=st["goal"],
                 back_px=r["back"], readback=read())
            if r["back"] >= STOP_PX or (np.isnan(r["back"]) and st["x"][2] <= HOVER_MM):
                note(phase="touch", by="push", ug_err_mm=look(), height_mm=st["x"][2])
                return "contact", r
        return "UGREEN step budget used, no contact", st["r"]

    r = sense([start])
    x = state(r)
    if ugreen is not None:  # the overhead view steers it all: arm-camera alignment lost far dots (live 2026-10-27)
        return finish(x, r, 0)
    phase, lost, grew, seen_before, fine, low_trim = "align", 0, 0, r["seen"], {}, False
    lags = []  # elbow/wrist commanded - read-back per step; the first FREE_STEPS are the free-air baseline
    # UGREEN: target - tip per move, to learn d(UGREEN error)/d(pulses) during alignment and steer the
    # last stretch with it, where the arm camera cannot see the dot under the tip
    ug_prev, ug_dq, ug_de, loaded = None, [], [], 0

    def ug_jac() -> np.ndarray | None:
        if len(ug_dq) < UG_MIN_PAIRS or np.linalg.matrix_rank(np.array(ug_dq)) < 3:
            return None
        return np.linalg.lstsq(np.array(ug_dq), np.array(ug_de), rcond=None)[0].T  # 2 x 3
    for n in range(steps):
        if phase == "align":
            if done(x):
                # finish each joint from its final direction (pan +, elbow/wrist the way the descent
                # moves them), so the backlash sits the same way for every touch
                final = dict(zip(joints, [1, *np.sign(neutral(-1.0)).astype(int)]))
                expects = [x[:2]]
                for i, j in enumerate(joints):
                    if ugreen is None and heading[j] != final[j]:  # the UGREEN finish needs no backlash order:
                        # these 5-pulse pan moves sent the needle ~15 mm away right before the drop
                        for d in (-final[j] * 5, final[j] * 5):
                            step([d if k == j else 0 for k in joints])
                        # a joint never moved here (heading 0) had its slack either way: it may end up
                        # to BACKLASH pulses off; pan at ~3 mm/pulse lost the dot on that (simulator)
                        expects += [x[:2] + jac[:2, i] * final[j] * k for k in range(1, BACKLASH + 1)]
                r = sense(expects)
                if not r["seen"] and np.hypot(*x[:2]) > HIDE_MM:
                    raise RuntimeError("target lost after the final-direction moves")
                x, r = trim(state(r), r)
                note(phase="aligned", step=n, err_mm=x[:2], height_mm=x[2], heading=dict(heading))
                if done(x):
                    phase, fine = "descend", {j: float(cmd[j]) for j in joints}
                    if ugreen is not None:
                        return finish(x, r, n)
                continue
            dq, flipped = step(align_raw(x, seen_before or np.hypot(*x[:2]) <= HIDE_MM))
        else:
            if x[2] <= HOVER_MM + 0.25 and not low_trim:  # last look at the dot
                # aim off by the x the blind elbow/wrist descent will add (it holds y only): ~0.8 mm (simulator)
                drift = float(jac[0, 1:] @ np.linalg.solve(jac[1:, 1:], [0.0, -x[2]]))
                x, r = trim(x, r, aim=-drift)
                low_trim = True
                note(phase="hover", step=n, err_mm=x[:2], height_mm=x[2])
            if hover and x[2] <= HOVER_MM + 0.25:
                return "hover", r
            if x[2] > HOVER_MM + 0.25:  # dot still in view: keep correcting xy on the way down
                dq, flipped = step(solve(x, -1.0))
                fine = {j: float(cmd[j]) for j in joints}
            else:  # blind: the y-holding elbow/wrist pair only, summed in float (rounding walked y off)
                e, jug = (ugreen(x[2], x[:2]) if ugreen else None), ug_jac()
                if e is not None and jug is not None and np.hypot(*e) > ALIGN_MM:
                    # the UGREEN sees tip and dot together: correct xy, hold the height
                    raw = np.linalg.lstsq(np.vstack([jug, jac[2:3]]), [-e[0], -e[1], 0.0], rcond=None)[0]
                    note(phase="ugreen-correct", step=n, ug_err_mm=e, raw=raw)
                    step(raw)
                    fine = {j: float(cmd[j]) for j in joints}
                elif e is not None:
                    note(phase="ugreen-check", step=n, ug_err_mm=e, jac_ready=jug is not None)
                dpulse = neutral(-0.5)
                for j, d in zip(joints[1:], dpulse):
                    fine[j] += d
                    cmd[j] = int(round(fine[j]))
                move(cmd, ms=300, settle=0.6)
                dq, flipped = np.array([0.0, *dpulse]), False
        expects = [x[:2] + jac[:2] @ dq, x[:2]]  # moved as commanded, or not at all (deadband, backlash)
        # the window grows with the predicted move: starts 45-52 mm off lost the dot with a fixed 6 mm
        # window (live 2026-10-27); two dots inside it count as ambiguous
        moved_mm = float(np.linalg.norm(jac[:2] @ dq))
        r = sense(expects, [min(LOCK_MM + 0.5 * moved_mm, MAX_LOCK_MM), LOCK_MM])
        new = state(r)
        if ugreen and new[2] > HOVER_MM:
            e = ugreen(new[2], new[:2])
            if e is not None and ug_prev is not None and dq.any() and not flipped:
                ug_dq.append(dq.tolist())
                ug_de.append((e - ug_prev).tolist())
            ug_prev = e
        # force feedback: pressing on the paper makes elbow/wrist lag their command (servo 2 already reads
        # ~5 low in free air, so compare with this run's free-air lag, not with zero)
        now = read()
        lags.append([cmd[j] - now[j] for j in joints[1:]])
        load = float(np.abs(np.array(lags[-1]) - np.median(lags[:FREE_STEPS], axis=0)).max())
        # a reversal step takes up backlash in the command and the read-back lags a step: 8 pulses of
        # "load" with no contact (live 2026-10-27); count only non-reversal steps, two in a row
        loaded = loaded + 1 if load >= LOAD_PULSES and not flipped else 0
        note(phase=phase, step=n, dq=dq, err_mm=new[:2], height_mm=new[2], back_px=r["back"], seen=r["seen"], load=load,
             readback=now,
             pulses=dict(cmd))
        # in every phase: an align step drove the tip to -6 mm unchecked (live 2026-10-27)
        pushed = (r["back"] >= STOP_PX or (np.isnan(r["back"]) and new[2] <= HOVER_MM)
                  or (len(lags) > FREE_STEPS and loaded >= 2))
        if pushed or new[2] <= FLOOR_MM:
            note(phase="touch" if phase == "descend" else "unplanned-contact", by="push" if pushed else "floor",
                 err_mm=new[:2], height_mm=new[2])
            return ("contact" if phase == "descend" else "unplanned contact while aligning"), r
        if not r["seen"] and np.hypot(*new[:2]) > HIDE_MM and (phase == "align" or new[2] > HOVER_MM):
            lost += 1
            if lost >= 2:
                raise RuntimeError(f"target lost: no dot within {LOCK_MM} mm of the prediction twice running")
        else:
            lost = 0
        if phase == "descend" and r["seen"] and np.any(np.abs(new[:2]) > tol() + REALIGN_MM - ALIGN_MM):
            phase = "align"  # climbs back to SAFE_MM first
            note(phase="realign", step=n, err_mm=new[:2])
        if r["seen"] and seen_before and not flipped and dq.any() and np.linalg.norm(new - x) > NOISE_MM:
            jac += np.outer((new - x) - jac @ dq, dq) / (dq @ dq)
        if phase == "align":
            grew = grew + 1 if np.hypot(*new[:2]) > np.hypot(*x[:2]) + NOISE_MM else 0
            if grew >= 2:  # Broyden drifted: back to the seed
                jac[:], grew = J0, 0
                note(phase="jacobian-reset", step=n)
        seen_before, x = r["seen"], new
    return "step budget used, no contact", r


if __name__ == "__main__":
    main()
