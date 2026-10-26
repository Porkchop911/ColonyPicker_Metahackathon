#!/usr/bin/env python3
"""Needle tip in 3D by moving the robot: UGREEN-only triangulation over a known pure translation.

Strafing (mecanum, no turn) moves camera and needle together, so the tip moves exactly as far as the
arm camera. The arm camera measures that move itself: grid pose per frame, with the frames tied by
matching the dark blobs (ink dots, holes, binding) it sees in both - same camera, same light, small
move, so unlike cross-camera dot pairing this match is unambiguous. The UGREEN's rays to the tip at
each robot position, with those displacements, triangulate the tip; the arm camera's ray in the
first frame then gives the tip offset in the arm-camera frame. No pairing across the two cameras.

    .venv/Scripts/python tools/tip_motion.py --near 1142 204      # strafe left twice, then measure
    .venv/Scripts/python tools/tip_motion.py --near 882 136 --from evidence/tip_motion/<run>  # re-analyse
Moves the robot base (gateway /drive, 0.3 s pulses); the arm does not move.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import paper_grid as pg  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import ROBOT, Camera  # noqa: E402

DRIVE = ("set -a; source ~/hwr/config/hwr.env; set +a; curl -s -X POST localhost:8090/drive "
         "-H \"Authorization: Bearer $HWR_GATEWAY_TOKEN\" -H 'Content-Type: application/json' -d '{body}'")


def drive(vx: float, vy: float, seconds: float = 0.3) -> None:
    """One bounded /drive pulse sent on the robot (the token stays there); gateway caps it at 0.5 s."""
    body = json.dumps({"vx": vx, "vy": vy, "wz": 0.0, "duration": seconds})
    out = subprocess.run(["ssh", ROBOT, DRIVE.replace("{body}", body)], capture_output=True, text=True, timeout=30)
    if '"accepted":true' not in out.stdout:
        raise RuntimeError(f"drive refused: {out.stdout} {out.stderr}")
    time.sleep(seconds + 1.2)  # stop, settle


def arm_view(image: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(R, t, blob sheet points) in this frame's own grid numbering."""
    obj, px, _ = pg.grid_points(image, seed=(image.shape[1] / 2, image.shape[0] * 0.75))
    ok, r, t, inliers = cv2.solvePnPRansac(obj, px, tt.ARM_K, tt.ARM_D, reprojectionError=2.0)
    if not ok or len(inliers) < 100:
        raise RuntimeError("arm grid pose failed")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hat = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, np.ones((15, 15), np.uint8))
    dark = (hat > 40).astype(np.uint8)
    dark[cv2.dilate(tt._needle_blob(image).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0] = 0
    count, _, stats, centres = cv2.connectedComponentsWithStats(dark)
    blobs = [tt.on_sheet(centres[i], tt.ARM_K, tt.ARM_D, r, t)[:2] for i in range(1, count)
             if 3 <= stats[i, cv2.CC_STAT_AREA] <= 200]
    rot, _ = cv2.Rodrigues(r)
    return rot, t.ravel(), np.array(blobs).reshape(-1, 2)


def lattice_offset(ref: np.ndarray, other: np.ndarray) -> tuple[np.ndarray, int]:
    """Whole-square shift taking `other`'s numbering onto `ref`'s: the one that lands most blobs on
    blobs (<= 1.5 mm). Both sets come from the same camera, so they are the same physical marks."""
    best = (np.zeros(2), -1)
    if not len(ref) or not len(other):
        return best
    for dx in range(-20, 21):
        for dy in range(-20, 21):
            o = np.array([dx, dy]) * pg.SQUARE_MM
            gaps = np.linalg.norm(ref[:, None, :] - (other[None, :, :] + o), axis=2).min(axis=0)
            hits = int((gaps <= 1.5).sum())
            if hits > best[1]:
                best = (o, hits)
    return best


def centre(ref_blobs: np.ndarray, image: np.ndarray) -> np.ndarray:
    """Arm-camera centre in the numbering of the frame whose blobs are `ref_blobs`."""
    rot, t, blobs = arm_view(image)
    o, _ = lattice_offset(ref_blobs, blobs)
    return -rot.T @ t + np.array([*o, 0.0])


def return_home(ref_blobs: np.ndarray, home: np.ndarray, left: np.ndarray, tol_mm: float = 2.0) -> dict:
    """Strafe back until the arm camera is within `tol_mm` of `home` along the strafe axis (user rule:
    every strafe series ends where it started). `left`: unit sheet direction the camera moves for +vy.
    Mecanum slip makes pulse length unpredictable, so it measures after every pulse and shortens the
    pulse after an overshoot. A forward offset cannot be undone here (/drive never reverses)."""
    seconds, last, sign = 0.3, None, 1.0
    for _ in range(14):
        off = centre(ref_blobs, tt.snap_arm()) - home
        lateral = float(off[:2] @ left)
        if abs(lateral) <= tol_mm:
            return {"lateral_mm": round(lateral, 2), "forward_mm": round(float(off[:2] @ np.array([-left[1], left[0]])), 2)}
        if last is not None:
            if np.sign(lateral) == np.sign(last) and abs(lateral) > abs(last) + 0.5:
                sign = -sign  # the pulse pushed it further away: `left` points the other way
            elif np.sign(lateral) != np.sign(last):
                seconds = max(seconds / 2, 0.1)  # overshot: shorter pulses
        last = lateral
        drive(0.0, (-0.05 if lateral > 0 else 0.05) * sign, seconds)
    raise RuntimeError(f"could not return home: still {lateral:.1f} mm off along the strafe axis")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--near", type=float, nargs=2, required=True, help="UGREEN tip guess px at the start")
    ap.add_argument("--from", dest="source", type=Path, default=None, help="re-analyse a saved run, no motion")
    ap.add_argument("--moves", default="0,0.05,0.2;0,0.05,0.2;0,-0.05,0.2;0,-0.05,0.2",
                    help="vx,vy[,seconds] per /drive pulse, ';'-separated; strafes only by default (a forward step cannot be undone)")
    ap.add_argument("--home", type=Path, default=None,
                    help="only strafe back to frame 0 of this saved run (strafe axis from its frames 0->1)")
    args = ap.parse_args()
    out = args.source or Path("evidence/tip_motion") / time.strftime("%Y%m%dT%H%M%S")
    rig = json.loads(tt.RIG.read_text())
    ku, du, ru, tu = (np.array(rig[x]) for x in ("k", "d", "r", "t"))
    dot_px = cv2.projectPoints(np.array([[x, y, 0.0] for x, y in rig["dots"]]), ru, tu, ku, du)[0][:, 0].tolist()
    near, frames = tuple(args.near), []
    if args.home:
        ref = arm_view(cv2.imread(str(args.home / "0_arm.png")))
        home = -ref[0].T @ ref[1]
        step = centre(ref[2], cv2.imread(str(args.home / "1_arm.png"))) - home
        print("returned:", return_home(ref[2], home, step[:2] / np.linalg.norm(step[:2])))
        return
    if args.source is None:
        out.mkdir(parents=True, exist_ok=True)
        cam = Camera(0)
        for n, move in enumerate([None] + args.moves.split(";")):
            if move:
                drive(*map(float, move.split(",")))  # vx, vy[, seconds]
            cv2.imwrite(str(out / f"{n}_ugreen.jpg"), cam.frame())
            cv2.imwrite(str(out / f"{n}_arm.png"), tt.snap_arm())
        ref = arm_view(cv2.imread(str(out / "0_arm.png")))
        home = -ref[0].T @ ref[1]
        step = centre(ref[2], cv2.imread(str(out / "1_arm.png"))) - home
        back = return_home(ref[2], home, step[:2] / np.linalg.norm(step[:2]))
        (out / "returned.json").write_text(json.dumps(back))
        print("returned:", back, flush=True)
    for n in range(len(list(out.glob("*_ugreen.jpg")))):
        ug, arm = cv2.imread(str(out / f"{n}_ugreen.jpg")), cv2.imread(str(out / f"{n}_arm.png"))
        pu = tt.ugreen_tip(ug, near, radius=160, exclude=dot_px)
        h, w = ug.shape[:2]
        if not (15 < pu[0] < w - 15 and 15 < pu[1] < h - 15):
            print(f"frame {n}: UGREEN tip {np.round(pu, 1).tolist()} at the image edge - tip left the view, frame dropped")
            continue
        near = pu
        rot, t, blobs = arm_view(arm)
        frames.append({"ugreen_px": pu, "arm_px": tt.arm_tip(arm), "R": rot, "t": t, "blobs": blobs, "n": n})
        print(f"frame {n}: ugreen tip {np.round(pu, 1).tolist()}, {len(blobs)} arm blobs", flush=True)
    if len(frames) < 2:
        raise SystemExit("fewer than two usable frames")

    # every frame's arm pose in frame 0's numbering
    for f in frames:
        o, hits = lattice_offset(frames[0]["blobs"], f["blobs"])
        f["C"] = -f["R"].T @ f["t"] + np.array([*o, 0.0])
        f["offset_hits"] = hits
        n = cv2.undistortPoints(np.array([[f["arm_px"]]], np.float64), tt.ARM_K, tt.ARM_D)[0, 0]
        f["v"] = np.array([n[0], n[1], 1.0]) / np.linalg.norm([n[0], n[1], 1.0])
        f["ray"] = tt.ray(f["ugreen_px"], ku, du, ru, tu)
    # Unknowns: the tip's distance lam along the arm camera's tip ray (fixed for one grasp) and the
    # in-plane shift s between the two grid numberings. Each frame's measured rotation is kept: the
    # robot yaws up to 0.7 deg per strafe, and treating strafes as pure translation put the tip
    # 7 mm too high (63.4 vs ~56 mm on a tape, 2026-10-26 night).
    fits = []
    for q in range(4):
        c, s = np.cos(q * np.pi / 2), np.sin(q * np.pi / 2)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])  # arm numbering -> UGREEN numbering
        rows, rhs = [], []
        for f in frames:
            cu, du = f["ray"]
            m = np.eye(3) - np.outer(du, du)
            rows.append(np.hstack([(m @ rz @ f["R"].T @ f["v"])[:, None], m[:, :2]]))
            rhs.append(m @ (cu - rz @ f["C"]))
        x = np.linalg.lstsq(np.vstack(rows), np.concatenate(rhs), rcond=None)[0]
        lam, shift = x[0], np.array([x[1], x[2], 0.0])
        tips = np.array([rz @ (f["C"] + lam * f["R"].T @ f["v"]) + shift for f in frames])
        res = [np.linalg.norm((np.eye(3) - np.outer(f["ray"][1], f["ray"][1])) @ (tp - f["ray"][0]))
               for tp, f in zip(tips, frames)]
        heights = -tips[:, 2]
        camera_h = -np.array([(rz @ f["C"])[2] for f in frames])
        # physics: the tip is in front of the lens, above the paper and below the camera
        possible = lam > 0 and (heights > 0).all() and (heights < camera_h).all()
        fits.append({"rotation_deg": q * 90, "lam": float(lam), "tips": tips, "rms": float(np.sqrt(np.mean(np.square(res)))),
                     "possible": bool(possible)})
    ok = [f for f in fits if f["possible"]]
    if not ok:
        raise SystemExit(f"no physically possible solution: {[(f['rotation_deg'], round(f['rms'], 2)) for f in fits]}")
    best = min(ok, key=lambda f: f["rms"])
    result = {"frames": [f["n"] for f in frames], "height_mm": round(float(-best["tips"][0][2]), 2),
              "heights_mm": np.round(-best["tips"][:, 2], 2).tolist(), "ray_rms_mm": round(best["rms"], 3),
              "rotation_deg": best["rotation_deg"],
              "tip_arm_cam_mm": np.round(best["lam"] * frames[0]["v"], 2).tolist(),
              "rejected": [(f["rotation_deg"], round(f["rms"], 2), f["possible"]) for f in fits if f is not best],
              "moves_mm": [np.round(f["C"] - frames[0]["C"], 2).tolist() for f in frames],
              "offset_hits": [f["offset_hits"] for f in frames]}
    (out / "result.json").write_text(json.dumps(result, indent=1))
    print(json.dumps(result, indent=1))


if __name__ == "__main__":
    main()
