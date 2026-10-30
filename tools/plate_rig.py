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
PITCH_MM = 9.0  # SBS 96-well spacing
SNAP_PX = 1.5  # a crossing this close to a projected lattice point counts as fitting
# well blobs (blackhat kernel, threshold, min area px, max aspect): far wells in the low side view are flat ellipses
WELLS = {"top": (41, 20, 400, 3.0), "side": (21, 30, 50, 6.0)}  # shortcut: tuned on the 2026-10-30 views; retune if moved
# the overhead's plate pose vs the one the verified side pose implies (plate frame, overhead on its +x side): the deep
# wells' dark centroids sit off the openings at 46 deg. Without it the chain was 2.25 mm off, near the whole-square tie,
# and one solve came out a square wrong (2026-10-30, checked against the black mat's edge in both views)
TOP_PLATE_FIX = ([-0.00737, -0.01127, 0.00117], [1.58, 1.47, 0.46])  # shortcut: one view; remeasure if the overhead moves far
CHECK_MIN = 150  # cached top pose: fewer crossings than this on the lattice -> camera moved, rerun


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


def grid_pose(img: np.ndarray, k: np.ndarray, d: np.ndarray, plate: tuple) -> tuple[np.ndarray, np.ndarray, int, float]:
    """Overhead pose on the ruling. The plate lies flat on the sheet, so its pose gives the sheet's tilt and the camera
    height; the sheet's turn about the vertical and the plate height are searched, the best snap regrown. (Numbering
    the ruling alone, as paper_grid does, found 20-40 consistent crossings of ~450 at this tilt, seed by seed.)"""
    und = undistort(pg.crossings(pg.ruling_mask(img)), k, d)
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


def plate_poses(img: np.ndarray, k: np.ndarray, d: np.ndarray, name: str) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Camera poses in the plate frame (wells' top plane z = 0, x along the 12, y along the 8, 9 mm pitch), one per
    labelling of the rectangle that puts the camera above the plate: the two 180-degree turns. (r, t, inliers)."""
    blobs = wells(img, name)
    und = undistort(blobs, k, d)
    # seeds: the blobs with the most blobs around them (96 wells are the densest patch); keep the fullest window
    crowd = (np.linalg.norm(und[:, None] - und[None], axis=2) < 60.0).sum(axis=1)
    best = None
    for s in np.argsort(-crowd)[:10]:
        try:
            q, px = plate_window(*number_lattice(und, und[s], patch=400.0, tol=0.4))
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
        ok, r, t, inl = cv2.solvePnPRansac(obj, px, k, None, reprojectionError=3.0, iterationsCount=2000)
        if not ok or inl is None:
            continue
        r, t = cv2.solvePnP(obj[inl[:, 0]], px[inl[:, 0]], k, None, r, t, useExtrinsicGuess=True)[1:]
        if (-cv2.Rodrigues(r)[0].T @ t.ravel())[2] > 0:  # camera above the wells (plate z points to the camera side)
            out.append((r, t, len(inl)))
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


def solve(top_img: np.ndarray, side_img: np.ndarray) -> tuple[dict, dict]:
    lens = json.loads(TOP_LENS.read_text())
    kt, dt = np.array(lens["k"]), np.array(lens["d"])
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
    print(f"plate: {tilt:.2f} deg off the sheet, top {-sheet_plate[2, 3]:.1f} mm above it", flush=True)
    if n_top < CHECK_MIN or tilt > 2.0 or not 5.0 < -sheet_plate[2, 3] < 25.0:
        raise RuntimeError(f"overhead: {n_top} crossings, plate {tilt:.1f} deg off the sheet: no consistent pose")
    side_und = undistort(pg.crossings(pg.ruling_mask(side_img)), ks, ds)
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
    if n < 100 or not 80.0 < -centre[2] < 250.0:
        raise RuntimeError(f"side: {n} crossings, camera {-centre[2]:.0f} mm up: implausible, "
                           f"caches left as they were")
    return top, side


def check(top_img: np.ndarray) -> int:
    """Crossings the cached overhead pose still puts on the lattice (a moved camera drops this to a few)."""
    c = json.loads(TOP_CACHE.read_text())
    k, d = np.array(c["k"]), np.array(c["d"])
    und = undistort(pg.crossings(pg.ruling_mask(top_img)), k, d)
    return int((snap(und, k, np.array(c["r"]), np.array(c["t"]))[1] < SNAP_PX).sum())


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--check", action="store_true")
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
    top, side = solve(top_img, side_img)
    TOP_CACHE.write_text(json.dumps(top))
    SIDE_CACHE.write_text(json.dumps(side))
    print(f"wrote {TOP_CACHE}, {SIDE_CACHE}")


if __name__ == "__main__":
    main()
