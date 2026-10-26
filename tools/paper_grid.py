#!/usr/bin/env python3
"""Metric paper frame from the 5 mm ruling of the grid sheet: grid lines -> numbered intersections.

Each camera that sees the sheet gets (paper mm, image px) pairs; cv2.calibrateCamera / solvePnP then
gives its pose in the paper frame (x, y on the sheet, z up from it). The origin and axis direction
are arbitrary per image (the ruling repeats); the ink dots fix them across cameras.

    .venv/Scripts/python tools/paper_grid.py IMAGE      # calibrate from one view, print K, pose, residual
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

SQUARE_MM = 5.0


def ruling_mask(image: np.ndarray) -> np.ndarray:
    """Thin dark lines on bright paper, restricted to the large connected net (drops wood grain)."""
    red = image[:, :, 2].astype(np.float32)  # blue-grey ruling is darkest in red
    hat = cv2.morphologyEx(red, cv2.MORPH_BLACKHAT, cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9)))
    mask = (hat > 6).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    keep = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] > 0.02 * mask.size * 0.05]
    return np.isin(labels, keep).astype(np.uint8) * 255


def _family(mask: np.ndarray, angles: np.ndarray, length: int = 31) -> np.ndarray:
    """Pixels on long straight strokes at one of `angles` (deg from the image x axis): the mask
    opened with a line kernel per angle."""
    out = np.zeros_like(mask)
    c = length // 2
    for a in angles:
        kernel = np.zeros((length, length), np.uint8)
        dx, dy = c * np.cos(np.radians(a)), c * np.sin(np.radians(a))
        cv2.line(kernel, (int(round(c - dx)), int(round(c - dy))), (int(round(c + dx)), int(round(c + dy))), 1)
        out |= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return out


def crossings(mask: np.ndarray) -> np.ndarray:
    """Sub-pixel centres where a near-horizontal and a steep ruling stroke overlap (Nx2)."""
    rows = cv2.dilate(_family(mask, np.arange(-20, 21, 4)), np.ones((3, 3), np.uint8))
    cols = cv2.dilate(_family(mask, np.arange(50, 131, 4)), np.ones((3, 3), np.uint8))
    count, _, stats, centres = cv2.connectedComponentsWithStats((rows & cols), connectivity=8)
    area = stats[1:, cv2.CC_STAT_AREA]
    return centres[1:][(area >= 4) & (area <= 200)].astype(np.float32)


def number(points: np.ndarray, seed: np.ndarray, patch: float = 120.0) -> tuple[np.ndarray, np.ndarray]:
    """Integer lattice (i, j) per crossing. A neighbour walk numbers a small patch around `seed`
    (short walks cannot slip); its homography then labels every crossing by rounding, and the fit is
    regrown until the labelled set stops changing. Returns (ij, px) for labelled crossings.

    Hough lines were tried first and failed on this dense grid: a slightly tilted line collects votes
    from every ruling line it crosses (~15k phantom lines per family). A whole-sheet walk slipped by
    one row behind dots and the needle."""
    start = int(np.argmin(np.linalg.norm(points - seed, axis=1)))
    near = np.argsort(np.linalg.norm(points - points[start], axis=1))[1:9]
    steps = points[near] - points[start]
    a = steps[np.argmax(np.abs(steps[:, 0]) - np.abs(steps[:, 1]))]  # most horizontal neighbour
    b = steps[np.argmax(np.abs(steps[:, 1]) - np.abs(steps[:, 0]))]  # most vertical neighbour
    a = a if a[0] > 0 else -a
    b = b if b[1] > 0 else -b
    ij = {start: (0, 0)}
    queue = [start]
    while queue:
        k = queue.pop()
        i, j = ij[k]
        for d, di, dj in ((a, 1, 0), (-a, -1, 0), (b, 0, 1), (-b, 0, -1)):
            dist = np.linalg.norm(points - (points[k] + d), axis=1)
            m = int(np.argmin(dist))
            if dist[m] > 0.25 * np.linalg.norm(d) or m in ij or np.linalg.norm(points[m] - points[start]) > patch:
                continue
            ij[m] = (i + di, j + dj)
            queue.append(m)
    keys = np.array(list(ij))
    if len(keys) < 6:
        raise ValueError(f"only {len(keys)} crossings around the seed")
    lattice = np.array([ij[k] for k in keys], np.float32)
    labelled = keys
    for _ in range(8):
        h, _ = cv2.findHomography(lattice, points[labelled], cv2.RANSAC, 3.0)
        guess = cv2.perspectiveTransform(points[None], np.linalg.inv(h))[0]
        rounded = np.round(guess)
        close = np.linalg.norm(guess - rounded, axis=1) < 0.2
        # grow outward gradually: only crossings within the fitted region plus a margin of 6 squares
        span = (lattice.min(0) - 6, lattice.max(0) + 6)
        inside = np.all((rounded >= span[0]) & (rounded <= span[1]), axis=1)
        nxt = np.nonzero(close & inside)[0]
        if len(nxt) == len(labelled) and np.array_equal(nxt, labelled):
            break
        labelled, lattice = nxt, rounded[nxt].astype(np.float32)
    return lattice.astype(np.int32), points[labelled]


def grid_points(image: np.ndarray, seed: tuple[float, float] | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(paper mm Nx3, image px Nx2, ruling mask). x runs along the near-horizontal ruling, y along
    the steep one, z = 0 on the sheet. Mis-labelled crossings are trimmed later by calibrate()."""
    mask = ruling_mask(image)
    pts = crossings(mask)
    h, w = image.shape[:2]
    first = [seed] if seed is not None else []
    # a seed on a dot, the needle or a blurred patch numbers too little: try a few, keep the largest
    best = None
    for sd in first + [(w / 2, h / 2), (w / 2, h * 0.75), (w * 0.35, h * 0.6), (w * 0.65, h * 0.6), (w / 2, h * 0.35)]:
        try:
            ij, px = number(pts, np.array(sd, np.float32))
        except (ValueError, cv2.error):
            continue
        if best is None or len(ij) > len(best[0]):
            best = (ij, px)
    if best is None:
        raise RuntimeError("paper grid: no seed numbered enough crossings")
    ij, px = best
    obj = np.column_stack([ij[:, 0] * SQUARE_MM, ij[:, 1] * SQUARE_MM, np.zeros(len(ij))]).astype(np.float32)
    return obj, px, mask


def calibrate(obj: np.ndarray, img: np.ndarray, size: tuple[int, int]):
    """Single-view calibration: principal point fixed at the centre, no tangential, k1 k2 only."""
    w, h = size
    f0 = 0.5 * w / np.tan(np.radians(45))  # 90 deg horizontal FOV as the start
    k = np.array([[f0, 0, w / 2], [0, f0, h / 2], [0, 0, 1]], np.float64)
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT | cv2.CALIB_FIX_ASPECT_RATIO
             | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K3)
    for _ in range(3):  # refit without crossings off by > 3x the residual (numbering slips)
        rms, k, dist, rvecs, tvecs = cv2.calibrateCamera([obj], [img], size, k, None, flags=flags)
        proj, _ = cv2.projectPoints(obj, rvecs[0], tvecs[0], k, dist)
        err = np.linalg.norm(proj[:, 0] - img, axis=1)
        good = err < max(3 * rms, 1.0)
        if good.all():
            break
        obj, img = obj[good], img[good]
    return rms, k, dist, rvecs[0], tvecs[0], len(obj)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image")
    args = ap.parse_args()
    image = cv2.imread(args.image)
    obj, img, mask = grid_points(image)
    print(f"{len(obj)} grid crossings")
    rms, k, dist, rvec, tvec, used = calibrate(obj, img, (image.shape[1], image.shape[0]))
    print(f"{used} used after outlier trim")
    r, _ = cv2.Rodrigues(rvec)
    centre = (-r.T @ tvec).ravel()
    print(f"rms {rms:.2f} px  f {k[0, 0]:.1f}  dist {np.round(dist.ravel(), 4).tolist()}")
    print(f"camera in paper frame (mm): {np.round(centre, 1).tolist()}")
    overlay = image.copy()
    proj, _ = cv2.projectPoints(obj, rvec, tvec, k, dist)
    for p, q in zip(img, proj[:, 0]):
        cv2.circle(overlay, tuple(int(c) for c in p), 3, (0, 0, 255), 1)
        cv2.circle(overlay, tuple(int(c) for c in q), 1, (0, 255, 0), -1)
    out = args.image.rsplit(".", 1)[0] + "_grid.jpg"
    cv2.imwrite(out, overlay)
    print(f"overlay {out}")


if __name__ == "__main__":
    main()
