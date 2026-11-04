#!/usr/bin/env python3
"""ArUco markers for calibration (plan 12): the camera tie, the needle holder and the base's pose. SAM 3.1 stays the
detector for targets (dots, colonies); markers only replace the dot/cube matching that ties the cameras.

    python tools/markers.py draw                               the cell patterns to colour in on the 5 mm grid
    python tools/markers.py sheet evidence/markers_a4.png      the same as an A4 page (if a printer turns up)
    python tools/markers.py check IMAGE                        markers found in a camera frame

Hand-drawn on the notebook's 5 mm grid (no printer): a marker is 6 x 6 squares (black border + 4 x 4 code) = 30 mm,
coloured in with a black marker pen, with at least one blank square all round. The ids are the ones with the fewest
black code cells (4-6 of 16). Notebook: SHEET_IDS (flat on the paper); holder: HOLDER_IDS; robot chassis: CHASSIS_ID.
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
SIDE_MM = 30.0           # 6 x 5 mm notebook squares, hand-drawn: ~170 px in the 4K overhead at 1080p, ~90 px in the arm view
SHEET_IDS = (8, 19, 24, 32)   # 4-5 black code cells each: least to colour in
HOLDER_IDS = (9, 45, 46, 6)
CHASSIS_ID = 5
_detector = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())


def detect(img: np.ndarray) -> dict[int, np.ndarray]:
    """{marker id: its 4 corner pixels (4, 2), in ArUco order}."""
    gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = _detector.detectMarkers(gray)
    return {} if ids is None else {int(i): c.reshape(4, 2).astype(float) for i, c in zip(ids.ravel(), corners)}


def on_paper(found: dict[int, np.ndarray], to_sheet) -> dict[int, np.ndarray]:
    """Sheet-marker corners carried onto the paper (z = 0) in one camera's sheet frame; `to_sheet(px) -> (x, y)`."""
    return {i: np.array([to_sheet(q) for q in c]) for i, c in found.items() if i in SHEET_IDS}


def tie(src: dict[int, np.ndarray], dst: dict[int, np.ndarray]) -> tuple[np.ndarray, np.ndarray, int, float]:
    """Rotation and offset taking `src` sheet mm onto `dst` sheet mm from the markers both saw (corner to corner,
    rigid least squares); (r 2x2, offset, markers used, rms mm). Two grid numberings of one sheet differ by a multiple
    of 90 deg: within 3 deg of one, the rotation is snapped to it and only the offset refit."""
    common = sorted(set(src) & set(dst))
    if not common:
        raise RuntimeError("no marker seen by both cameras")
    a = np.vstack([src[i] for i in common])
    b = np.vstack([dst[i] for i in common])
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    r = (u @ np.diag([1.0, np.sign(np.linalg.det(u @ vt))]) @ vt).T
    ang = np.degrees(np.arctan2(r[1, 0], r[0, 0]))
    snap = 90.0 * round(ang / 90.0)
    if abs(ang - snap) <= 3.0:
        c, s = np.cos(np.radians(snap)), np.sin(np.radians(snap))
        r = np.array([[c, -s], [s, c]])
    off = cb - r @ ca
    rms = float(np.sqrt(np.mean(np.sum((a @ r.T + off - b) ** 2, 1))))
    return r, off, len(common), rms


def pose(corners: np.ndarray, k: np.ndarray, d: np.ndarray, side_mm: float = SIDE_MM) -> tuple[np.ndarray, np.ndarray]:
    """A marker's pose in the camera frame from its printed size (rvec, tvec): the chassis marker is not on the paper."""
    h = side_mm / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    ok, r, t = cv2.solvePnP(obj, corners.astype(np.float64), k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise RuntimeError("marker pose failed")
    return r, t


def in_sheet(corners: np.ndarray, k, d, r_cam, t_cam, side_mm: float = SIDE_MM) -> tuple[np.ndarray, float]:
    """A marker's centre (x, y, z mm) and heading (deg) in a camera's sheet frame (r_cam, t_cam: sheet -> camera):
    for the chassis marker, where the base stands and which way it faces."""
    rm, tm = pose(corners, k, d, side_mm)
    rot_c = cv2.Rodrigues(np.asarray(r_cam, np.float64))[0]
    t_c = np.asarray(t_cam, np.float64).reshape(3, 1)
    centre = (rot_c.T @ (tm.reshape(3, 1) - t_c)).ravel()
    x_axis = rot_c.T @ cv2.Rodrigues(rm)[0] @ np.array([1.0, 0.0, 0.0])
    return centre, float(np.degrees(np.arctan2(x_axis[1], x_axis[0])))


def patterns(ids) -> str:
    """Each marker as 6 rows of 6 cells, # = colour black, . = leave white: one cell per 5 mm notebook square."""
    out = []
    for mid in ids:
        m = cv2.aruco.generateImageMarker(DICT, mid, 6, borderBits=1)
        out.append(f"id {mid}:\n" + "\n".join("  " + " ".join("#" if v == 0 else "." for v in row) for row in m))
    return "\n\n".join(out)


def make_sheet(path: str, dpi: int = 300) -> None:
    """A4 portrait: the 4 sheet markers, the 4 holder markers, the chassis marker, a 50 mm check line, labels."""
    mm = dpi / 25.4
    page = np.full((int(297 * mm), int(210 * mm)), 255, np.uint8)
    side = int(round(SIDE_MM * mm))

    def put(mid: int, x_mm: float, y_mm: float, label: str) -> None:
        img = cv2.aruco.generateImageMarker(DICT, mid, side, borderBits=1)
        x, y = int(x_mm * mm), int(y_mm * mm)
        page[y:y + side, x:x + side] = img
        cv2.putText(page, f"{label}  id {mid}", (x, y + side + int(5 * mm)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
        cv2.rectangle(page, (x - int(6 * mm), y - int(6 * mm)), (x + side + int(6 * mm), y + side + int(9 * mm)), 180, 1)
    cv2.putText(page, "Colony picker markers - print at 100 % (no fit to page). Cut along the grey boxes.",
                (int(12 * mm), int(14 * mm)), cv2.FONT_HERSHEY_SIMPLEX, 1.3, 0, 3)
    cv2.line(page, (int(12 * mm), int(24 * mm)), (int(62 * mm), int(24 * mm)), 0, 4)
    cv2.putText(page, "this line must measure 50 mm", (int(66 * mm), int(26 * mm)), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    for n, mid in enumerate(SHEET_IDS):
        put(mid, 20 + 45 * n, 40, "notebook")
    for n, mid in enumerate(HOLDER_IDS):
        put(mid, 20 + 45 * n, 100, "holder")
    put(CHASSIS_ID, 20, 160, "robot chassis (top, facing up)")
    cv2.imwrite(path, page, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    from PIL import Image
    Image.open(path).save(path, dpi=(dpi, dpi))  # the print size comes from the DPI tag


def _self_check() -> None:
    """Render a marker into a synthetic camera view, find it, and recover its pose and a known tie."""
    k = np.array([[1150.0, 0, 960], [0, 1150.0, 540], [0, 0, 1]])
    d = np.zeros(5)
    side = 200
    tile = cv2.aruco.generateImageMarker(DICT, CHASSIS_ID, side, borderBits=1)
    tile = cv2.copyMakeBorder(tile, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
    h = SIDE_MM / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
    flip = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]  # the marker frame's z points at the camera (OpenCV convention)
    rv = cv2.Rodrigues(flip @ cv2.Rodrigues(np.array([0.3, -0.2, 0.4]))[0])[0].ravel()
    tv = np.array([10.0, -5.0, 250.0])
    px = cv2.projectPoints(obj, rv, tv, k, d)[0].reshape(4, 2)
    src = np.float32([[40, 40], [240, 40], [240, 240], [40, 240]])
    view = cv2.warpPerspective(tile, cv2.getPerspectiveTransform(src, px.astype(np.float32)), (1920, 1080), borderValue=255)
    found = detect(view)
    assert CHASSIS_ID in found, found
    r, t = pose(found[CHASSIS_ID], k, d)
    assert np.linalg.norm(t.ravel() - tv) < 2.0 and np.linalg.norm(r.ravel() - rv) < 0.03, (r.ravel(), t.ravel())
    a = {0: np.array([[0, 0], [20, 0], [20, 20], [0, 20.0]]), 1: np.array([[100, 50], [120, 50], [120, 70], [100, 70.0]])}
    rr = np.array([[-1.0, 0], [0, -1.0]])
    b = {i: c @ rr.T + [7.0, -3.0] + 0.2 for i, c in a.items()}
    r2, off, n, rms = tie(a, b)
    assert n == 2 and np.allclose(r2, rr) and np.linalg.norm(off - [7.2, -2.8]) < 1e-6 and rms < 1e-6, (r2, off, rms)
    print("markers self-check ok: pose within", np.round(np.linalg.norm(t.ravel() - tv), 2), "mm, tie exact")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("sheet").add_argument("path")
    sub.add_parser("check").add_argument("image")
    sub.add_parser("selftest")
    sub.add_parser("draw")
    args = ap.parse_args()
    if args.cmd == "sheet":
        make_sheet(args.path)
        print(f"wrote {args.path}: print at 100 %, the line must measure 50 mm")
    elif args.cmd == "check":
        found = detect(cv2.imread(args.image))
        print({i: np.round(c.mean(0), 1).tolist() for i, c in sorted(found.items())} or "no markers")
    elif args.cmd == "draw":
        print("notebook (draw them straight on the sheet):\n" + patterns(SHEET_IDS))
        print("\nrobot chassis (draw on a cut-out piece of the grid paper, tape it on top of the base):\n" + patterns([CHASSIS_ID]))
        print("\nneedle holder (later):\n" + patterns(HOLDER_IDS))
    else:
        _self_check()


if __name__ == "__main__":
    sys.exit(main())
