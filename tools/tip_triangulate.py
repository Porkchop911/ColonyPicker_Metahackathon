#!/usr/bin/env python3
"""Needle tip in 3D from two cameras that both locate themselves on the 5 mm grid sheet.

UGREEN (fixed, PC) and the arm camera (eye-in-hand) each get their pose from the ruling
(tools/paper_grid.py). Each numbers the grid from its own crossing, so the arm frame is mapped onto
the UGREEN frame by a fixed rotation plus a whole number of squares. The shift comes from pairing
a black ink dot seen by the arm camera with each black dot the UGREEN sees; the pairing under which
every other UGREEN dot in the arm's view lands on ink breaks ties. The tip is where the two rays meet; their
miss distance is a self-check (it cannot pick the pairing: a shift only moves the miss along one
direction, so wrong pairings also met within 0.3 mm live). Refused when the miss exceeds MAX_MISS_MM.

Live 2026-10-26: tip 75.2 mm above the paper, rays 0.55 mm apart; user measured ~7.5 cm.

    .venv/Scripts/python tools/tip_triangulate.py            # one measurement at the current pose
    .venv/Scripts/python tools/tip_triangulate.py --out DIR  # also keep both frames + a JSON record
    .venv/Scripts/python tools/tip_triangulate.py --recalibrate   # after moving the UGREEN or the sheet
Read-only: snaps both cameras, never moves the arm.
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paper_grid as pg  # noqa: E402
from rig_io import Camera, reset_bridge, restart_arm_camera, servo  # noqa: E402

# arm colour camera intrinsics, read from the robot camera_info topic on 2026-10-26: 640x400
ARM_K = np.array([[418.6042, 0.0, 324.1325], [0.0, 419.4625, 184.2325], [0.0, 0.0, 1.0]])
ARM_D = np.array([0.01029, -0.04893, 0.00039, -0.00087, 0.02712])
# shortcut: the two grid numberings differ by 180 deg because the UGREEN faces the robot (2026-10-26
# rig). Search all four 90-deg steps if the UGREEN is ever moved to the robot's side.
ROTATION_DEG = 180
MAX_MISS_MM = 3.0
# the UGREEN and the sheet stay put, so one calibration is reused: per-frame single-view calibration
# moved the focal length 1097-1163 px between frames and the ray miss with it (0.4 -> 2.4 mm)
RIG = Path("evidence/tip/ugreen_rig.json")
# the UGREEN's lens is hardware: calibrated once (the ruler-confirmed daylight frame) and reused, so a
# moved UGREEN only re-solves its pose. Single-view self-calibration drifted f by ~1 % per view.
LENS = Path("evidence/tip/ugreen4k_lens.json")  # the 4K overhead since 2026-10-29; ugreen_lens.json is the old 1080p, now the side camera


def ray(px, k, d, r, t) -> tuple[np.ndarray, np.ndarray]:
    """Camera centre and unit ray through pixel `px`, in the sheet frame of (r, t)."""
    rot, _ = cv2.Rodrigues(np.asarray(r, np.float64))
    n = cv2.undistortPoints(np.array([[px]], np.float64), k, d)[0, 0]
    v = rot.T @ np.array([n[0], n[1], 1.0])
    return (-rot.T @ np.asarray(t, np.float64).reshape(3, 1)).ravel(), v / np.linalg.norm(v)


def on_sheet(px, k, d, r, t) -> np.ndarray:
    c, v = ray(px, k, d, r, t)
    return c + v * (-c[2] / v[2])


def meet(a: tuple, b: tuple) -> tuple[np.ndarray, float]:
    """Midpoint of the closest approach of two rays, and their miss distance."""
    (c1, d1), (c2, d2) = a, b
    w = c1 - c2
    p, q, r, s, u = d1 @ d1, d1 @ d2, d2 @ d2, d1 @ w, d2 @ w
    den = p * r - q * q
    m1, m2 = c1 + (q * u - r * s) / den * d1, c2 + (p * u - q * s) / den * d2
    return (m1 + m2) / 2, float(np.linalg.norm(m1 - m2))


def _on_ruled_sheet(points: np.ndarray, crossings: np.ndarray) -> np.ndarray:
    """Which image points lie inside the hull of the found grid crossings: drops the spiral binding,
    the margin line beside it, punched holes at the edge and anything off the sheet."""
    hull = cv2.convexHull(crossings.astype(np.float32))
    return np.array([cv2.pointPolygonTest(hull, (float(u), float(v)), False) > 0 for u, v in points], bool)


def ugreen_rig(image: np.ndarray) -> dict:
    """UGREEN intrinsics + pose from the ruling, and the sheet position of each ink dot it sees."""
    obj, px, _ = pg.grid_points(image)
    if LENS.exists():
        lens = json.loads(LENS.read_text())
        k, d = np.array(lens["k"]), np.array(lens["d"])
        ok, r, t, inl = cv2.solvePnPRansac(obj, px, k, d, reprojectionError=3.0)
        proj, _ = cv2.projectPoints(obj[inl[:, 0]], r, t, k, d)
        rms, used = float(np.sqrt(np.mean(np.sum((proj[:, 0] - px[inl[:, 0]]) ** 2, 1)))), int(len(inl))
    else:
        rms, k, d, r, t, used = pg.calibrate(obj, px, (image.shape[1], image.shape[0]))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((21, 21), np.uint8))
    count, labels, stats, centres = cv2.connectedComponentsWithStats((hat > 25).astype(np.uint8))
    dots = []
    for i in range(1, count):
        w, h, area = stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT], stats[i, cv2.CC_STAT_AREA]
        # ink dots are small and round; the margin line breaks into long thin pieces, holes are big.
        # Colour does not separate them: red ink reads grey under this camera's cast (sat 4-26).
        if 30 <= area <= 400 and max(w, h) < 30 and max(w, h) < 2.2 * min(w, h) and area > 0.4 * w * h \
                and hat[labels == i].max() > 50:
            dots.append(on_sheet(centres[i], k, d, r, t)[:2].tolist() + [float(gray[labels == i].min())])
    # the margin line also passes the shape test, as a chain of pieces ~7 mm apart; drawn dots are
    # isolated (>= 25 mm apart on the test sheet)
    pts = np.array(dots).reshape(-1, 3)
    far = np.array([np.sum(np.linalg.norm(pts[:, :2] - p[:2], axis=1) < 12.0) == 1 for p in pts], bool)
    pts = pts[far] if len(pts) else pts
    return {"k": k.tolist(), "d": d.ravel().tolist(), "r": r.ravel().tolist(), "t": t.ravel().tolist(),
            "rms": rms, "crossings": used, "dots": pts[:, :2].tolist(),
            "sheet": cv2.convexHull(px.astype(np.float32))[:, 0].tolist(),
            "black": pts[pts[:, 2] < 80, :2].tolist()}  # black ink reads < 80; red and holes stay lighter


def arm_candidates(image: np.ndarray, rig: dict) -> tuple[list[tuple[np.ndarray, np.ndarray, list]], int]:
    """Arm-camera poses in the UGREEN sheet frame, one per (arm black dot, UGREEN black dot) pairing:
    grid PnP in the arm's own numbering, then the fixed rotation and the whole-square shift that puts
    the paired dots on top of each other. Each candidate carries an ink score: UGREEN dots in this
    view that land on ink, minus twice those that land on blank paper. Returns (candidates, inliers)."""
    obj, px, _ = pg.grid_points(image, seed=(image.shape[1] / 2, image.shape[0] * 0.75))
    ok, r, t, inliers = cv2.solvePnPRansac(obj, px, ARM_K, ARM_D, reprojectionError=2.0)
    if not ok:
        raise RuntimeError("arm camera: grid pose failed")
    rot, _ = cv2.Rodrigues(r)
    c, s = np.cos(np.radians(ROTATION_DEG)), np.sin(np.radians(ROTATION_DEG))
    m = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])  # arm numbering -> UGREEN numbering
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    dark = ((hat > 50) & (gray < 110)).astype(np.uint8)
    dark[cv2.dilate(_needle_blob(image).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0] = 0  # needle is dark too
    count, _, stats, centres = cv2.connectedComponentsWithStats(dark)
    blacks = np.array([centres[i] for i in range(1, count) if 3 <= stats[i, cv2.CC_STAT_AREA] <= 80
                       and max(stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]) <= 12]).reshape(-1, 2)
    # drawn dots are isolated; the spiral loops pass the darkness test as a chain ~10 px apart
    alone = np.array([np.sum(np.linalg.norm(blacks - b, axis=1) < 20) == 1 for b in blacks], bool)
    blacks = blacks[alone & _on_ruled_sheet(blacks, px)] if len(blacks) else blacks
    ink = cv2.dilate(hat, np.ones((3, 3), np.uint8))
    ink[cv2.dilate(_needle_blob(image).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0] = 0
    every = np.array([[x, y, 0.0] for x, y in rig["dots"]])
    out = []
    for dot in blacks:
        mine = m @ on_sheet(dot, ARM_K, ARM_D, r, t)
        for theirs in rig["black"]:
            shift = np.round((np.array([*theirs, 0.0]) - mine) / pg.SQUARE_MM) * pg.SQUARE_MM
            shift[2] = 0.0
            rot_u = rot @ m.T
            r_u, _ = cv2.Rodrigues(rot_u)
            t_u = t.ravel() - rot_u @ shift
            uv = cv2.projectPoints(every, r_u, t_u, ARM_K, ARM_D)[0][:, 0]
            inside = (uv[:, 0] > 2) & (uv[:, 0] < image.shape[1] - 3) & (uv[:, 1] > 2) & (uv[:, 1] < image.shape[0] - 3)
            hits = np.array([ink[int(round(v)), int(round(u))] > 30 for u, v in uv[inside]], bool)
            # one UGREEN dot landing on blank paper outweighs two that land on ink
            out.append((r_u.ravel(), t_u, shift[:2].tolist(), int(hits.sum() - 2 * (~hits).sum())))
    if not out:
        raise RuntimeError(f"arm camera: {len(blacks)} black dots here, {len(rig['black'])} from the UGREEN: nothing to pair")
    return out, int(len(inliers))


def _needle_blob(image: np.ndarray, frac: float = 0.8) -> np.ndarray:
    """Fingers + needle in the arm image: the dark blob at the bottom centre (boolean mask). Blue
    channel, relative threshold: the blue ruling is bright in blue, so it never joins the needle."""
    b = cv2.GaussianBlur(image[:, :, 0], (3, 3), 0)
    h, w = b.shape
    paper = np.median(b[h // 2:, w // 4:3 * w // 4])  # around the needle: the sheet, not the floor beyond it
    dark = (b < frac * paper).astype(np.uint8)
    dark[:, :int(w * 0.3)] = 0  # fingers and needle sit in the middle; the floor at the edges joined them
    dark[:, int(w * 0.75):] = 0
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    finger = labels[h - 3, int(w * 0.4):int(w * 0.6)]
    if not (finger > 0).any():
        raise RuntimeError("arm camera: no fingers/needle at the bottom centre")
    return labels == np.bincount(finger[finger > 0]).argmax()


def arm_tip(image: np.ndarray, with_axis: bool = False):
    """Sub-pixel needle tip; see _arm_tip. Lamp glare can pale the steel past the 0.8 mask (live
    2026-10-27), so looser masks are tried before giving up."""
    for frac in (0.8, 0.85, 0.9):
        try:
            return _arm_tip(image, with_axis, frac)
        except RuntimeError as error:
            last = error
    raise last


def _arm_tip(image: np.ndarray, with_axis: bool, frac: float):
    """Sub-pixel needle tip in the arm image. The needle line is fitted on the thin part of the blob;
    the last ~70 px toward the tip are lighter (lamp highlights) and drop out of the mask, so the tip
    is found by walking up that line on local contrast (centre vs +-6 px either side, blue channel):
    the last half-contrast crossing with no needle in the 6 px beyond. `with_axis` also returns the
    unit image direction from the fingers toward the tip."""
    b = cv2.GaussianBlur(image[:, :, 0], (3, 3), 0).astype(np.float32)
    blob = _needle_blob(image, frac)
    vs, us = np.nonzero(blob)
    top = vs.min()
    rows = []  # the thin needle only: from the top until the blob widens into the fingers
    for v in range(top, blob.shape[0]):
        if blob[v].sum() > 10:
            break
        rows.append(v)
    near = np.isin(vs, rows)
    if near.sum() < 15:
        raise RuntimeError("arm camera: needle too short in the mask")
    vx, vy, x0, y0 = cv2.fitLine(np.stack([us[near], vs[near]], 1).astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).ravel()
    axis = np.array([vx, vy]) * (1 if vy < 0 else -1)  # pointing up the image, toward the tip
    tip = _walk(b, np.array([x0, y0]) + (top - y0) / axis[1] * axis - 20 * axis, axis)
    if with_axis:
        return (float(tip[0]), float(tip[1])), axis.astype(float)
    return float(tip[0]), float(tip[1])


def arm_tip_on_line(image: np.ndarray, tip: tuple, axis) -> tuple[float, float]:
    """Tip found by the contrast walk alone, along a needle line already measured in free air. The
    needle is rigid to the camera, so its line holds; near the paper the sheet edge and floor enter
    the image and joined the finger mask (live 2026-10-27), which this never builds."""
    b = cv2.GaussianBlur(image[:, :, 0], (3, 3), 0).astype(np.float32)
    axis = np.asarray(axis, float)
    found = _walk(b, np.asarray(tip, float) - 18 * axis, axis)  # short: behind it the floor can show
    return float(found[0]), float(found[1])


def _walk(b: np.ndarray, start: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """Walk up the needle line from `start` (on the needle) on local contrast; return the tip."""
    side = np.array([-axis[1], axis[0]])
    step = 0.25
    h, w = b.shape

    def pix(p):
        return cv2.getRectSubPix(b, (1, 1), tuple(np.float32(p)))[0, 0]

    c = []
    for k in range(int(170 / step)):
        p = start + k * step * axis
        if not (8 <= p[0] < w - 8 and 8 <= p[1] < h - 8):
            break
        c.append((pix(p + 6 * side) + pix(p - 6 * side)) / 2 - pix(p))
    c = np.array(c)
    on = c > 0.5 * np.median(c[:int(15 / step)])
    gap = int(6 / step)
    last = next((i for i in range(len(on)) if on[i] and not on[i + 1:i + 1 + gap].any()), None)
    if last is None or last + 2 >= len(c):
        raise RuntimeError("arm camera: needle runs out of the image")
    half = np.median(c[max(0, last - int(8 / step)):last - int(2 / step)]) / 2
    k = last + 1
    while k < len(c) - 1 and c[k] > half:
        k += 1
    frac = (c[k - 1] - half) / (c[k - 1] - c[k]) if c[k - 1] != c[k] else 0.0
    return start + (k - 1 + frac) * step * axis


def ugreen_tip(image: np.ndarray, near: tuple[float, float], radius: int = 70,
               exclude: tuple = (), sheet: np.ndarray | None = None) -> tuple[float, float]:
    """Needle point in the UGREEN image near `near`.

    Dark = below 0.55 x the paper's median in the window (a fixed threshold broke when the exposure
    changed). `sheet`: hull of the grid crossings; outside it (the floor above the sheet edge) is
    ignored, since the floor merged with the needle and dragged the 'tip' 230 px away (2026-10-26).
    Of the dark blobs, the one whose lowest point is nearest `near` wins.
    shortcut: assumes the needle enters from the image top (robot opposite the UGREEN), so the
    tip is the blob's lowest pixel; generalise with an axis fit if the camera moves round.
    `exclude`: pixels of known ink dots, blanked so a dot under the tip is not taken for it."""
    u0, v0 = int(near[0]), int(near[1])
    x0, y0 = max(u0 - radius, 0), max(v0 - radius, 0)
    roi = cv2.cvtColor(image[y0:v0 + radius, x0:u0 + radius], cv2.COLOR_BGR2GRAY)
    keep = np.ones(roi.shape, np.uint8)
    if sheet is not None:
        full = np.zeros(image.shape[:2], np.uint8)
        cv2.fillConvexPoly(full, np.asarray(sheet, np.int32), 1)
        keep = full[y0:v0 + radius, x0:u0 + radius]
    for u, v in exclude:
        cv2.circle(keep, (int(round(u - x0)), int(round(v - y0))), 9, 0, -1)
    paper = np.median(roi[keep > 0]) if keep.any() else np.median(roi)
    dark = ((roi < 0.55 * paper) & (keep > 0)).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(dark)
    best = None
    for i in range(1, count):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        vs, us = np.nonzero(labels == i)
        low = vs >= vs.max() - 1
        point = np.array([us[low].mean() + x0, vs.max() + y0], float)
        gap = np.hypot(*(point - np.array(near)))
        if best is None or gap < best[0]:
            best = (gap, point)
    if best is None:
        raise RuntimeError("UGREEN: no dark needle point near the predicted tip")
    return float(best[1][0]), float(best[1][1])


_last_jpeg: list[bytes] = []


def snap_arm(n: int = 6, frozen_s: float = 3.0) -> np.ndarray:
    """Mean of `n` distinct arm-camera frames, all newer than the previous call's.

    The camera sometimes repeats one frame for ~10-30 s while the topic still looks fresh (age
    ~0.05 s; live 2026-10-26 night: 6 bit-identical measurements in a row, strafe frames that showed
    no move). Live frames never repeat byte for byte (sensor noise), so a repeat is a stale frame."""
    frames, seen = [], set(_last_jpeg)
    deadline = time.monotonic() + frozen_s
    restarted = False
    while len(frames) < n:
        try:
            reply = servo({"snap": True})[0]
            jpeg = base64.b64decode(reply["jpeg"])
            fresh = jpeg not in seen
        except RuntimeError:  # 'no camera frame yet' / 'stale camera frame'
            fresh = False
        if not fresh:
            if time.monotonic() > deadline:
                if restarted:
                    raise RuntimeError("arm camera stalled, a driver restart did not bring it back")
                restart_arm_camera()  # driver stall: restart once, then allow it ~30 s to come back
                reset_bridge()  # the old session never sees the respawned driver's frames
                restarted, deadline = True, time.monotonic() + 30.0
            time.sleep(0.3)
            continue
        seen.add(jpeg)
        deadline = time.monotonic() + frozen_s
        frames.append(cv2.imdecode(np.frombuffer(jpeg, np.uint8), 1).astype(np.float32))
        time.sleep(0.12)
    _last_jpeg[:] = [jpeg]
    return np.mean(frames, 0).astype(np.uint8)


def measure(ug_image: np.ndarray, arm_image: np.ndarray, rig: dict, ug_near: tuple[float, float]) -> dict:
    ku, du, ru, tu = (np.array(rig[x]) for x in ("k", "d", "r", "t"))
    candidates, inliers = arm_candidates(arm_image, rig)
    pa, pu = arm_tip(arm_image), ugreen_tip(ug_image, ug_near)
    seen = ray(pu, ku, du, ru, tu)
    tries = [(meet(seen, ray(pa, ARM_K, ARM_D, r, t)), r, t, shift, score) for r, t, shift, score in candidates]
    # physically possible: the rays meet and the tip is above the paper. That alone left one pairing
    # under room light (2026-10-26 night), where the ink scores did not separate them; ink breaks ties.
    valid = [x for x in tries if x[0][1] <= MAX_MISS_MM and -x[0][0][2] > 0]
    if not valid:
        raise RuntimeError(f"no dot pairing makes the tip rays meet above the paper: misses "
                           f"{sorted(round(x[0][1], 1) for x in tries)[:4]} mm")
    valid.sort(key=lambda x: (-x[4], x[0][1]))
    (tip, miss), ra, ta, shift, score = valid[0]
    if len(valid) > 1 and valid[1][4] == score and abs(valid[1][0][0][2] - tip[2]) > 1.0:
        raise RuntimeError(f"{len(valid)} dot pairings fit equally (ink score {score}) and disagree on the tip")
    rot, _ = cv2.Rodrigues(ra)
    in_cam = rot @ tip + ta  # the tip in the arm-camera frame: fixed for one grasp
    return {"tip_sheet_mm": np.round(tip, 2).tolist(), "height_mm": round(float(-tip[2]), 2),
            "miss_mm": round(miss, 2), "ink_score": score, "valid_pairings": len(valid),
            "shift_mm": shift,
            "tip_arm_cam_mm": np.round(in_cam, 2).tolist(), "grid_inliers": inliers,
            "arm_tip_px": np.round(pa, 2).tolist(), "ugreen_tip_px": np.round(pu, 2).tolist(),
            "arm_pose": {"r": np.round(ra, 5).tolist(), "t": np.round(ta, 2).tolist()}}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--near", type=float, nargs=2, default=(1062, 440), help="UGREEN tip guess px")
    ap.add_argument("--out", type=Path, default=None, help="keep frames + JSON here")
    ap.add_argument("--recalibrate", action="store_true", help="redo the UGREEN rig (camera or sheet moved)")
    args = ap.parse_args()
    ug = Camera().frame()
    if args.recalibrate or not RIG.exists():
        rig = ugreen_rig(ug)
        RIG.parent.mkdir(parents=True, exist_ok=True)
        RIG.write_text(json.dumps(rig))
    rig = json.loads(RIG.read_text())
    print(f"UGREEN: {rig['crossings']} crossings, rms {rig['rms']:.2f} px, {len(rig['dots'])} dots")
    pulses = servo({"read": True})[0]["pulses"]
    arm = snap_arm()
    result = {"pulses": pulses, **measure(ug, arm, rig, tuple(args.near))}
    print(json.dumps(result, indent=1))
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S")
        cv2.imwrite(str(args.out / f"{stamp}_ugreen.jpg"), ug)
        cv2.imwrite(str(args.out / f"{stamp}_arm.png"), arm)
        (args.out / f"{stamp}_tip.json").write_text(json.dumps({"rig": rig, **result}, indent=1))


if __name__ == "__main__":
    main()
