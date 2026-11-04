#!/usr/bin/env python3
"""Both rig cameras' poses in one sheet frame, from the 5 mm ruling and the 96-well plate (no ink dots needed).

The overhead looks at the sheet at ~45 deg since 2026-10-30 (colony imaging): its grid pose comes from crossings numbered
on undistorted points, then regrown with the camera model (a homography walk on raw pixels slipped rows there). The side
camera sees too little ruling to number on its own, so it is tied through the plate: both cameras number the 8 x 12 wells
(9 mm pitch, SBS), the plate pose chains the side into the overhead's sheet frame (a few mm off), and the side's own
crossings snapped to that frame's 5 mm lattice give the final pose. The snap is ambiguous by whole squares; the black mat,
seen flat on the paper by both cameras, picks the candidate.

    <hackathon venv python> tools/plate_rig.py            # grab both cameras, solve, write both pose caches
    <hackathon venv python> tools/plate_rig.py --check    # only report how well the cached poses still fit
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paper_grid as pg  # noqa: E402

TOP_LENS = Path("evidence/tip/ugreen4k_lens.json")
TOP_CACHE = Path("evidence/tip/top_pose_cache.json")
SIDE_CACHE = Path("evidence/tip/side_pose_cache.json")  # also holds the side lens (k, d) from the old 1080p UGREEN fit
PLATE_CACHE = Path("evidence/tip/plate_pose_cache.json")  # plate frame -> sheet frame (4x4), for well targets
PITCH_MM = 9.0  # SBS 96-well spacing
SNAP_PX = 1.5  # a crossing this close to a projected lattice point counts as fitting
# well blobs (blackhat kernel, threshold, min area px, max aspect): far wells in the low side view are flat ellipses
WELLS = {"top": (41, 20, 400, 3.0), "side": (21, 30, 50, 6.0)}  # shortcut: tuned on the 2026-10-30 views; retune if moved
# the overhead's plate pose vs the one the verified side pose implies (plate frame, overhead on its +x side): the deep
# wells' dark centroids sit off the openings at 46 deg. Without it the chain was 2.25 mm off, near the whole-square tie,
# and one solve came out a square wrong (2026-10-30, checked against the black mat's edge in both views)
TOP_PLATE_FIX = ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])  # 2026-11-03 near-vertical overhead, wells found by rim: no bias; was ([-0.00737, -0.01127, 0.00117], [1.58, 1.47, 0.46])  # shortcut: one view; remeasure if the overhead moves far
CHECK_MIN = 30  # cached top pose: fewer crossings than this on the lattice -> camera moved, rerun (150 before the black mat;
#                with it and the lamp on, an unmoved camera fits 43-71, 2026-10-31)


def undistort(px: np.ndarray, k: np.ndarray, d: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(np.asarray(px, np.float64).reshape(-1, 1, 2), k, d, P=k)[:, 0]


def on_plane(und: np.ndarray, k: np.ndarray, r, t) -> np.ndarray:
    """Undistorted pixels -> (x, y) on the sheet plane z = 0."""
    rot = cv2.Rodrigues(np.asarray(r, float))[0]
    c = -rot.T @ np.asarray(t, float).ravel()
    dirs = rot.T @ (np.linalg.inv(k) @ np.vstack([und.T, np.ones(len(und))]))
    return (c[:, None] + dirs * (-c[2] / dirs[2]))[:2].T


def snap(und: np.ndarray, k: np.ndarray, r, t, off=(0.0, 0.0)) -> tuple[np.ndarray, np.ndarray]:
    """Each crossing's nearest 5 mm lattice point (sheet mm, z = 0) with the frame moved by `off`, and its reprojection
    error (px) under (r, t) with the same move."""
    q = on_plane(und, k, r, t) + np.asarray(off)
    obj = np.column_stack([np.round(q / pg.SQUARE_MM) * pg.SQUARE_MM, np.zeros(len(q))])
    back = obj - np.r_[off, 0.0]
    err = np.linalg.norm(cv2.projectPoints(back, np.asarray(r, float), np.asarray(t, float), k, None)[0][:, 0] - und,
                         axis=1)
    return obj, err


def regrow(und: np.ndarray, k: np.ndarray, r, t, rounds: int = 40) -> tuple[np.ndarray, np.ndarray, int, float]:
    """Relabel every crossing by the pose, refit on the ones that fit, until the set stops growing."""
    r, t = np.array(r, float).reshape(3, 1), np.array(t, float).reshape(3, 1)  # copies: solvePnP writes into its guess
    for i in range(rounds):  # loose first, so a start pose a little off grows over the whole sheet, not one patch
        obj, err = snap(und, k, r, t)
        good = err < max(SNAP_PX, 4.0 - 0.5 * i)
        if good.sum() < 6:
            break
        _, r, t = cv2.solvePnP(obj[good], und[good], k, None, r, t, useExtrinsicGuess=True)
    obj, err = snap(und, k, r, t)
    good = err < SNAP_PX
    return r, t, int(good.sum()), float(np.sqrt(np.mean(err[good] ** 2))) if good.any() else 99.0


def ruling_crossings(img: np.ndarray) -> np.ndarray:
    """Ruling crossings in full-frame px; the ruling filters are sized for 1080p."""
    px_scale = img.shape[1] // 1920
    small = cv2.resize(img, None, fx=1 / px_scale, fy=1 / px_scale, interpolation=cv2.INTER_AREA)
    return pg.crossings(pg.ruling_mask(small)) * px_scale


def grid_pose(img: np.ndarray, k: np.ndarray, d: np.ndarray, plate: tuple) -> tuple[np.ndarray, np.ndarray, int, float]:
    """Overhead pose on the ruling. The plate lies flat on the sheet, so its pose gives the sheet's tilt and the camera
    height; the sheet's turn about the vertical and the plate height are searched, the best snap regrown. (Numbering
    the ruling alone, as paper_grid does, found 20-40 consistent crossings of ~450 at this tilt, seed by seed.)"""
    und = undistort(ruling_crossings(img), k, d)
    cam_plate = homog(*plate[:2])
    starts = []
    for h in np.arange(8.0, 20.1, 1.0):  # plate top above the paper, mm
        for yaw in np.radians(np.arange(-45.0, 45.0, 0.5)):
            c, s_ = np.cos(yaw), np.sin(yaw)
            plate_sheet = np.eye(4)  # sheet z points down (into the paper), plate z up: a half turn about x
            plate_sheet[:3, :3] = np.diag([1.0, -1.0, -1.0]) @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
            plate_sheet[2, 3] = -h
            m = cam_plate @ plate_sheet
            r, t = cv2.Rodrigues(m[:3, :3])[0], m[:3, 3].reshape(3, 1)
            starts.append((int((snap(und, k, r, t)[1] < 3.0).sum()), r, t))
    # the best start alone stalled on a patch (~110 of ~550) in one frame of three: regrow the best ten
    return max((regrow(und, k, r, t) for _, r, t in sorted(starts, key=lambda x: -x[0])[:10]), key=lambda g: g[2])


def wells(img: np.ndarray, name: str) -> np.ndarray:
    ksz, th, min_area, aspect = WELLS[name]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if name == "top":  # 4K, near straight down: wells are clean circles, the black-hat merges them (2026-11-03)
        found = cv2.HoughCircles(cv2.medianBlur(gray, 7), cv2.HOUGH_GRADIENT, 1, 38, param1=60, param2=18,
                                 minRadius=15, maxRadius=26)
        keep = [] if found is None else found[0, :, :2]
        return np.array([c for c in keep if gray[int(c[1]), int(c[0])] > np.percentile(gray, 60)], np.float64)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((ksz, ksz), np.uint8))
    n, _, st, cen = cv2.connectedComponentsWithStats((hat > th).astype(np.uint8))
    keep = [cen[j] for j in range(1, n) if min_area <= st[j, 4] <= 3000 and max(st[j, 2:4]) < aspect * min(st[j, 2:4])]
    # wells sit in white plastic: drop blobs on a dark surround (the keyboard drawing on the box behind is a lattice too)
    bright = cv2.dilate((gray > np.percentile(gray, 60)).astype(np.uint8), np.ones((ksz, ksz), np.uint8))
    return np.array([c for c in keep if bright[int(c[1]), int(c[0])]], np.float64)


def number_lattice(points: np.ndarray, seed: np.ndarray, patch: float, tol: float) -> tuple[np.ndarray, np.ndarray]:
    """paper_grid.number for the wells: the two steps are the shortest non-parallel neighbour vectors (the 'most
    horizontal / most vertical' pick takes a diagonal on the plate), one blob per lattice site."""
    start = int(np.argmin(np.linalg.norm(points - seed, axis=1)))
    steps = points[np.argsort(np.linalg.norm(points - points[start], axis=1))[1:9]] - points[start]
    lens = np.linalg.norm(steps, axis=1)
    a = steps[np.argmin(lens)]
    ang = np.abs(np.degrees(np.arctan2(steps[:, 1], steps[:, 0]) - np.arctan2(a[1], a[0]))) % 180
    skew = (ang > 35) & (ang < 145)
    b = steps[skew][np.argmin(lens[skew])]
    ij, queue = {start: (0, 0)}, [start]
    while queue:
        m0 = queue.pop()
        i, j = ij[m0]
        for step, di, dj in ((a, 1, 0), (-a, -1, 0), (b, 0, 1), (-b, 0, -1)):
            dist = np.linalg.norm(points - (points[m0] + step), axis=1)
            m = int(np.argmin(dist))
            if dist[m] > tol * np.linalg.norm(step) or m in ij or np.linalg.norm(points[m] - points[start]) > patch:
                continue
            ij[m] = (i + di, j + dj)
            queue.append(m)
    labelled = np.array(list(ij))
    lattice = np.array([ij[m] for m in labelled], np.float32)
    for _ in range(30):
        hom, _ = cv2.findHomography(lattice, points[labelled].astype(np.float32), cv2.RANSAC, 3.0)
        guess = cv2.perspectiveTransform(points[None].astype(np.float32), np.linalg.inv(hom))[0]
        rounded = np.round(guess)
        gap = np.linalg.norm(guess - rounded, axis=1)
        inside = np.all((rounded >= lattice.min(0) - 2) & (rounded <= lattice.max(0) + 2), axis=1)
        site = {}
        for q in np.nonzero((gap < 0.3) & inside)[0]:
            key = tuple(rounded[q].astype(int))
            if key not in site or gap[q] < gap[site[key]]:
                site[key] = q
        nxt = np.array(sorted(site.values()))
        if np.array_equal(nxt, labelled):
            break
        labelled, lattice = nxt, rounded[nxt].astype(np.float32)
    return lattice.astype(np.int32), points[labelled]


def plate_window(ij: np.ndarray, px: np.ndarray, rows: int = 8, cols: int = 12) -> tuple[np.ndarray, np.ndarray]:
    """The labels in the basis where the wells fill a cols x rows rectangle (any unimodular change of the walk's basis),
    the best-filled window kept: (column 0..11, row 0..7) per well."""
    mats = [np.array([[a, b], [c, d]]) for a in (-1, 0, 1) for b in (-1, 0, 1) for c in (-1, 0, 1) for d in (-1, 0, 1)
            if abs(a * d - b * c) == 1]
    best = None
    for m in mats:
        q = ij @ m.T
        for w, h in ((cols, rows), (rows, cols)):
            xs = range(min(q[:, 0].min(), q[:, 0].max() - w + 1), max(q[:, 0].min(), q[:, 0].max() - w + 1) + 1)
            ys = range(min(q[:, 1].min(), q[:, 1].max() - h + 1), max(q[:, 1].min(), q[:, 1].max() - h + 1) + 1)
            for x0 in xs:
                for y0 in ys:
                    inside = (q[:, 0] >= x0) & (q[:, 0] < x0 + w) & (q[:, 1] >= y0) & (q[:, 1] < y0 + h)
                    if best is None or inside.sum() > best[0]:
                        best = (int(inside.sum()), q[inside] - np.array([x0, y0]), inside, w == rows)
    _, q, inside, swap = best
    return (q[:, ::-1] if swap else q), px[inside]


def whole_plate(r, t, img: np.ndarray, k: np.ndarray, d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The lattice numbering can sit a whole well off when a column went unseen (side view 2026-11-03: 78 of 96, one
    column out, 9 mm; digits on the margin matched blobs too, so blob counts tie): keep the shift whose 96 points are
    darkest against the plate. shortcut: 1 grey level between the two best shifts, always look at the drawn labels."""
    cell = np.array([[c * PITCH_MM, w * PITCH_MM, 0.0] for c in range(12) for w in range(8)])
    rot = cv2.Rodrigues(r)[0]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((WELLS["side"][0],) * 2, np.uint8)).astype(float)

    def dark(shift) -> float:
        u = cv2.projectPoints(cell, r, t + rot @ shift, k, d)[0][:, 0].astype(int)
        inside = (u[:, 0] >= 0) & (u[:, 0] < img.shape[1]) & (u[:, 1] >= 0) & (u[:, 1] < img.shape[0])
        return float(hat[u[inside, 1], u[inside, 0]].sum())
    shifts = [np.array([[dx * PITCH_MM], [dy * PITCH_MM], [0.0]]) for dx in (-2, -1, 0, 1, 2) for dy in (-1, 0, 1)]
    return r, t + rot @ max(shifts, key=dark)


def plate_poses(img: np.ndarray, k: np.ndarray, d: np.ndarray, name: str) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Camera poses in the plate frame (wells' top plane z = 0, x along the 12, y along the 8, 9 mm pitch), one per
    labelling of the rectangle that puts the camera above the plate: the two 180-degree turns. (r, t, inliers)."""
    blobs = wells(img, name)
    und = undistort(blobs, k, d)
    # seeds: the blobs with the most blobs around them (96 wells are the densest patch); keep the fullest window
    px_scale = img.shape[1] / 1920  # the window sizes were set on 1080p frames
    crowd = (np.linalg.norm(und[:, None] - und[None], axis=2) < 60.0 * px_scale).sum(axis=1)
    best = None
    for s in np.argsort(-crowd)[:10]:
        try:
            q, px = plate_window(*number_lattice(und, und[s], patch=400.0 * px_scale, tol=0.4))
        except (ValueError, cv2.error):
            continue
        if best is None or len(q) > len(best[0]):
            best = (q, px)
    if best is None:
        return []
    q, px = best
    out = []
    for fx, fy in ((False, False), (True, False), (False, True), (True, True)):
        cell = np.column_stack([11 - q[:, 0] if fx else q[:, 0], 7 - q[:, 1] if fy else q[:, 1]])
        obj = np.column_stack([cell * PITCH_MM, np.zeros(len(cell))]).astype(np.float64)
        ok, r, t, inl = cv2.solvePnPRansac(obj, px, k, None, reprojectionError=3.0 * px_scale, iterationsCount=2000)
        if not ok or inl is None:
            continue
        r, t = cv2.solvePnP(obj[inl[:, 0]], px[inl[:, 0]], k, None, r, t, useExtrinsicGuess=True)[1:]
        if (-cv2.Rodrigues(r)[0].T @ t.ravel())[2] > 0:  # camera above the wells (plate z points to the camera side)
            out.append((*(whole_plate(r, t, img, k, d) if name == "side" else (r, t)), len(inl)))
    print(f"{name}: {len(blobs)} blobs, {len(q)} wells numbered, poses {[n for *_, n in out]} inliers", flush=True)
    return out


def homog(r, t) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = cv2.Rodrigues(np.asarray(r, float))[0]
    m[:3, 3] = np.asarray(t, float).ravel()
    return m


def dark_on_sheet(img: np.ndarray, k, d, r, t, grid: np.ndarray) -> np.ndarray:
    """The black mat (under half the paper's brightness; wood stays above) sampled at sheet points `grid` (Nx3); -1 off the image."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    dark = cv2.morphologyEx((gray < 0.45 * np.percentile(gray, 90)).astype(np.uint8), cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    rot, t = cv2.Rodrigues(np.asarray(r, float))[0], np.asarray(t, float).ravel()
    front = (grid @ rot.T + t)[:, 2] > 50.0  # projectPoints also maps points behind the camera
    px = np.full((len(grid), 2), -1)
    px[front] = np.round(cv2.projectPoints(grid[front], np.asarray(r, float), t, k, d)[0][:, 0]).astype(int)
    seen = front & (px[:, 0] >= 0) & (px[:, 0] < gray.shape[1]) & (px[:, 1] >= 0) & (px[:, 1] < gray.shape[0])
    out = np.full(len(grid), -1)
    out[seen] = dark[px[seen, 1], px[seen, 0]]
    return out


def flat_on_sheet(sheet_plate: np.ndarray, rt, tt_) -> np.ndarray:
    """One near-vertical view fixes the plate's place well but its tilt poorly (2.2 deg on a flat plate, 3.8 mm across
    it, 2026-11-03): slide the wells along the camera rays onto one flat plane and refit the plate in that plane."""
    cell = np.array([[c * PITCH_MM, w * PITCH_MM, 0.0] for c in range(12) for w in range(8)])
    pts = (sheet_plate[:3, :3] @ cell.T).T + sheet_plate[:3, 3]
    cam = -cv2.Rodrigues(rt)[0].T @ np.ravel(tt_)
    z = pts[:, 2].mean()
    flat = cam + (pts - cam) * ((z - cam[2]) / (pts[:, 2] - cam[2]))[:, None]
    u, _, vt = np.linalg.svd((cell[:, :2] - cell[:, :2].mean(0)).T @ (flat[:, :2] - flat[:, :2].mean(0)))
    q = (u @ vt).T  # no det fix: the plate frame may be a reflection, see well()
    out = np.eye(4)
    out[:2, :2], out[2, 2] = q, np.sign(sheet_plate[2, 2])
    out[:2, 3], out[2, 3] = flat[:, :2].mean(0) - q @ cell[:, :2].mean(0), z
    return out


def solve(top_img: np.ndarray, side_img: np.ndarray) -> tuple[dict, dict, dict]:
    lens = json.loads(TOP_LENS.read_text())
    kt, dt = np.array(lens["k"]), np.array(lens["d"])
    kt[:2] *= top_img.shape[1] / 1920  # the lens was calibrated on 1080p frames, same field of view as 4K
    old_side = json.loads(SIDE_CACHE.read_text())
    ks, ds = np.array(old_side["k"]), np.array(old_side["d"])
    top_plate = plate_poses(top_img, kt, dt, "top")
    side_plate = plate_poses(side_img, ks, ds, "side")
    if not top_plate or not side_plate:
        raise RuntimeError("plate: no pose in one of the views (is the 96-well plate in both?)")
    # the labelling with the overhead on the plate's +x side, the one TOP_PLATE_FIX was measured in
    plate = next(p for p in top_plate if (-cv2.Rodrigues(p[0])[0].T @ p[1].ravel())[0] > 0)
    rt, tt_, n_top, rms_top = grid_pose(top_img, kt, dt, plate)
    print(f"overhead: {n_top} crossings on the lattice, {rms_top:.2f} px", flush=True)
    sheet_plate = np.linalg.inv(homog(rt, tt_)) @ homog(*plate[:2]) @ homog(*TOP_PLATE_FIX)
    tilt = np.degrees(np.arccos(abs(sheet_plate[2, 2])))
    sheet_plate = flat_on_sheet(sheet_plate, rt, tt_)
    print(f"plate: {tilt:.2f} deg off the sheet, top {-sheet_plate[2, 3]:.1f} mm above it", flush=True)
    if n_top < CHECK_MIN or tilt > 4.0 or not 5.0 < -sheet_plate[2, 3] < 25.0:
        raise RuntimeError(f"overhead: {n_top} crossings, plate {tilt:.1f} deg off the sheet: no consistent pose")
    side_und = undistort(ruling_crossings(side_img), ks, ds)
    centre = sheet_plate[:3, :3] @ np.array([5.5 * PITCH_MM, 3.5 * PITCH_MM, 0.0]) + sheet_plate[:3, 3]
    xs, ys = np.meshgrid(np.arange(-150, 150, 2.0), np.arange(-150, 150, 2.0))  # the mat lies beside the plate
    grid = np.column_stack([xs.ravel() + centre[0], ys.ravel() + centre[1], np.zeros(xs.size)])
    top_dark = dark_on_sheet(top_img, kt, dt, rt, tt_, grid)
    turns = []
    for r, t, _ in side_plate:  # the plate's two turns
        chain = homog(r, t) @ np.linalg.inv(sheet_plate)
        rs, ts = cv2.Rodrigues(chain[:3, :3])[0], chain[:3, 3].reshape(3, 1)
        # the snap fixes the frame only to whole squares: the one nearest the (bias-corrected) plate chain
        fit = max(((snap(side_und, ks, rs, ts, (dx, dy))[1] < SNAP_PX).sum(), dx, dy)
                  for dx in np.arange(-2.5, 2.5, 0.25) for dy in np.arange(-2.5, 2.5, 0.25))
        off = (float(fit[1]), float(fit[2]))
        obj, err = snap(side_und, ks, rs, ts, off)
        good = err < SNAP_PX
        if good.sum() < 6:
            continue
        _, r1, t1 = cv2.solvePnP(obj[good], side_und[good], ks, None, rs.copy(), ts.copy(), useExtrinsicGuess=True)
        r1, t1, n, rms = regrow(side_und, ks, r1, t1)
        side_dark = dark_on_sheet(side_img, ks, ds, r1, t1, grid)
        both = (top_dark >= 0) & (side_dark >= 0)
        iou = ((top_dark == 1) & (side_dark == 1) & both).sum() / max((((top_dark == 1) | (side_dark == 1)) & both).sum(), 1)
        print(f"side, plate turn {len(turns) + 1}: chain off by {np.round(off, 2).tolist()} mm, {n} crossings, "
              f"{rms:.2f} px, mat overlap {iou:.2f}", flush=True)
        turns.append((iou, n, r1, t1, rms, off))
    if not turns:
        raise RuntimeError("side: no plate turn snapped onto the ruling")
    iou, n, rs, ts, rms_side, off = max(turns, key=lambda c: c[0])  # the wrong turn sees the mat elsewhere: 0.2 vs 0.6
    if max(abs(v) for v in off) > 1.5:
        print(f"WARNING: the plate chain was {np.round(off, 2).tolist()} mm off (the fixed overhead bias no longer "
              f"cancels?): near 2.5 the side frame is a whole square out; check a measured tip", flush=True)
    centre = -cv2.Rodrigues(rs)[0].T @ ts.ravel()
    top = {"k": kt.tolist(), "d": dt.ravel().tolist(), "r": rt.ravel().tolist(), "t": tt_.ravel().tolist(),
           "crossings": n_top, "rms": rms_top, "source": "plate_rig.py grid pose"}
    side = {"k": ks.tolist(), "d": ds.ravel().tolist(), "r": rs.ravel().tolist(), "t": ts.ravel().tolist(),
            "crossings": n, "rms": rms_side, "mat_overlap": float(iou), "chain_off_mm": list(off),
            "centre_mm": centre.tolist(),
            "source": "plate_rig.py: plate chain + side crossings"}
    print(f"side: {n} crossings, {rms_side:.2f} px, mat overlap {iou:.2f}, camera {-centre[2]:.0f} mm up", flush=True)
    if n < 50 or not 80.0 < -centre[2] < 250.0:  # 50: the black mat (2026-10-31) hides most side crossings
        raise RuntimeError(f"side: {n} crossings, camera {-centre[2]:.0f} mm up: implausible, "
                           f"caches left as they were")
    # seen from above the printed plate is a mirror image of the labelling above: printed A1 is cell (11, 0) here
    sheet_plate = sheet_plate @ np.array([[-1.0, 0, 0, 11 * PITCH_MM], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    plate_out = {"sheet_plate": sheet_plate.tolist(), "source": "plate_rig.py: overhead plate pose + TOP_PLATE_FIX"}
    return top, side, plate_out


def well(name: str, below_rim_mm: float = 0.0) -> np.ndarray:
    """Sheet mm (z down) of a well's centre, `below_rim_mm` under its rim, e.g. well("B7", 3.0). Plate frame: x along the
    12 columns, y along the 8 rows (A1 at the origin), z up (a reflection when A1 was picked on a mirrored corner)."""
    m = np.array(json.loads(PLATE_CACHE.read_text())["sheet_plate"])
    row, col = "ABCDEFGH".index(name[0].upper()), int(name[1:]) - 1
    return m[:3, :3] @ np.array([col * PITCH_MM, row * PITCH_MM, -below_rim_mm]) + m[:3, 3]


def plate_from_side(top_img: np.ndarray, side_img: np.ndarray, a1_px=None) -> dict:
    """Plate pose in the sheet frame without renumbering the overhead: both cached camera poses regrown (creep only),
    the plate from the side camera's well lattice (under the warm colony lamp the overhead's far wells read white and
    its well numbering failed, 2026-10-31). The lattice is symmetric, so the A1 corner is the one nearest the previous
    cache's A1, or the corner whose projection is nearest `a1_px` (overhead pixel of well A1, picked by eye)."""
    for cache, img in ((TOP_CACHE, top_img), (SIDE_CACHE, side_img)):
        c = json.loads(cache.read_text())
        k = np.array(c["k"])
        und = undistort(ruling_crossings(img), k, np.array(c["d"]))
        r, t, n, rms = regrow(und, k, np.array(c["r"]), np.array(c["t"]))
        print(f"{cache.name}: {n} crossings, {rms:.2f} px", flush=True)
        if n < CHECK_MIN:
            raise RuntimeError(f"{cache.name}: {n} crossings fit: camera moved too far, run the full solve")
        cache.write_text(json.dumps({**c, "r": r.ravel().tolist(), "t": t.ravel().tolist(), "crossings": n, "rms": rms}))
    top, side = json.loads(TOP_CACHE.read_text()), json.loads(SIDE_CACHE.read_text())
    poses = plate_poses(side_img, np.array(side["k"]), np.array(side["d"]), "side")
    if not poses:
        raise RuntimeError("side: no plate pose (is the 96-well plate in view?)")
    r, t, _ = max(poses, key=lambda p: p[2])
    m = np.linalg.inv(homog(side["r"], side["t"])) @ homog(r, t)  # plate -> sheet
    # the four ways to put A1 on a corner keeping columns on the 12-well side: reflections of the plate frame
    cands = []
    for fx in (1, -1):
        for fy in (1, -1):
            f = np.diag([fx, fy, 1.0, 1.0])
            f[:2, 3] = [(fx < 0) * 11 * PITCH_MM, (fy < 0) * 7 * PITCH_MM]
            cands.append(m @ f)
    if a1_px is not None:
        px = [cv2.projectPoints(c_[:3, 3][None], np.array(top["r"]), np.array(top["t"]), np.array(top["k"]),
                                np.array(top["d"]))[0][0, 0] for c_ in cands]
        best = cands[int(np.argmin([np.linalg.norm(p - np.asarray(a1_px)) for p in px]))]
    elif PLATE_CACHE.exists():
        old = np.array(json.loads(PLATE_CACHE.read_text())["sheet_plate"])[:3, 3]
        best = min(cands, key=lambda c_: np.linalg.norm(c_[:3, 3] - old))
    else:
        raise RuntimeError("no previous plate cache: pass --a1 U V (overhead pixel of well A1)")
    print(f"plate: A1 at sheet {np.round(best[:3, 3], 1).tolist()} mm (top {-best[2, 3]:.1f} mm above the paper)", flush=True)
    return {"sheet_plate": best.tolist(), "source": "plate_rig.py --plate: side well lattice, A1 corner chosen"}


def check(top_img: np.ndarray) -> int:
    """Crossings the cached overhead pose still puts on the lattice (a moved camera drops this to a few)."""
    c = json.loads(TOP_CACHE.read_text())
    k, d = np.array(c["k"]), np.array(c["d"])
    und = undistort(ruling_crossings(top_img), k, d)
    return int((snap(und, k, np.array(c["r"]), np.array(c["t"]))[1] < SNAP_PX).sum())


def refresh(top_img: np.ndarray) -> int:
    """Follow a small overhead creep (3.4 px between two runs, 2026-10-30) without changing the frame: regrow from the
    cached pose, so every crossing keeps its lattice label, and rewrite the cache. Returns the crossings that fit; the
    cache is left alone when that stays under CHECK_MIN (moved too far: rerun the full solve and refit the tie)."""
    c = json.loads(TOP_CACHE.read_text())
    k, d = np.array(c["k"]), np.array(c["d"])
    und = undistort(ruling_crossings(top_img), k, d)
    r, t, n, rms = regrow(und, k, np.array(c["r"]), np.array(c["t"]))
    if n >= CHECK_MIN:
        TOP_CACHE.write_text(json.dumps({**c, "r": r.ravel().tolist(), "t": t.ravel().tolist(), "crossings": n, "rms": rms,
                                         "refreshed": True}))
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--plate", action="store_true", help="only the plate pose (cameras regrown from their caches)")
    ap.add_argument("--a1", nargs=2, type=float, metavar=("U", "V"), help="with --plate: overhead pixel of well A1")
    ap.add_argument("--images", nargs=2, metavar=("TOP", "SIDE"), help="solve from saved frames instead of the cameras")
    args = ap.parse_args()
    if args.images:
        top_img, side_img = (cv2.imread(p) for p in args.images)
    else:
        from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
        cams = Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER)
        top_img, side_img = (c.frame() for c in cams)
        for c in cams:
            c.close()
    if args.check:
        print(f"cached overhead pose: {check(top_img)} crossings on the lattice (>= {CHECK_MIN} ok)")
        return
    if args.plate:
        PLATE_CACHE.write_text(json.dumps(plate_from_side(top_img, side_img, args.a1)))
        print(f"wrote {PLATE_CACHE}")
        return
    top, side, plate = solve(top_img, side_img)
    TOP_CACHE.write_text(json.dumps(top))
    SIDE_CACHE.write_text(json.dumps(side))
    PLATE_CACHE.write_text(json.dumps(plate))
    print(f"wrote {TOP_CACHE}, {SIDE_CACHE}, {PLATE_CACHE}")


if __name__ == "__main__":
    main()
