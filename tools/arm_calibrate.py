#!/usr/bin/env python3
"""Arm calibration against the external cameras (plan 12 L2): joint zero offsets from tip positions triangulated by the
overhead and side UGREENs over ~40 poses, so the arm model's 5-10 mm absolute error stops costing placing iterations.

    <hackathon venv python> tools/arm_calibrate.py collect      robot on, arm at the start pose, inked tip (~3 min)
    <hackathon venv python> tools/arm_calibrate.py fit [DATA]   fit and write evidence/tip/arm_offsets.json
    <hackathon venv python> tools/arm_calibrate.py selftest     the fit on synthetic data with known offsets

Unknowns: shoulder and elbow zero offsets (pulses, kept), the tip in the camera frame (changes with every grasp) and
the base on the desk: yaw + position in the overhead's sheet frame (changes with every strafe). A pan offset is the same
as a turned base, and a wrist offset the same as a different tip: neither is identifiable here, so both stay 0.
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
import coarse_aim as ca  # noqa: E402

OUT = Path("evidence/tip/arm_offsets.json")
DATA = Path("evidence/tip/arm_calibration_data.json")
START = {1: 497, 2: 420, 3: 190, 4: 269, 5: 502}  # tools/arm_to_start.py: tip ~45-70 mm up
MIN_HEIGHT_MM = 15.0  # a measured tip lower than this ends the sweep (the arm goes back up)


def model(p: np.ndarray, pulses: list[dict]) -> np.ndarray:
    """Tip in the overhead's sheet frame (mm) at these pulses: p = (d2, d3, tip_cam xyz, yaw, base xyz)."""
    d2, d3, tx, ty, tz, yaw, bx, by, bz = p
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # the base stands on the desk: yaw only
    out = []
    for q in pulses:
        tip = ca.tip_in_base({**q, 2: q[2] + d2, 3: q[3] + d3}, np.array([tx, ty, tz]))
        out.append(rot @ tip + [bx, by, bz])
    return np.array(out)


def fit(pulses: list[dict], seen: np.ndarray, p0: np.ndarray | None = None, iters: int = 60) -> tuple[np.ndarray, float]:
    """Levenberg-Marquardt on the 3D tip residuals (numeric Jacobian; no scipy in the venv); (params, rms mm)."""
    p = np.array([0.0, 0.0, 0.0, 25.0, 200.0, 0.0, 0.0, 0.0, 0.0]) if p0 is None else p0.astype(float)
    if p0 is None:  # base position and yaw from the data: centroid offset, then the best of 8 yaws
        best = None
        for yaw in np.linspace(-np.pi, np.pi, 8, endpoint=False):
            p[5] = yaw
            p[6:9] = 0.0
            p[6:9] = (seen - model(p, pulses)).mean(0)
            e = np.linalg.norm(model(p, pulses) - seen)
            best = (e, p.copy()) if best is None or e < best[0] else best
        p = best[1]
    lam = 1e-2
    res = (model(p, pulses) - seen).ravel()
    for _ in range(iters):
        jac = np.empty((res.size, p.size))
        for i in range(p.size):
            dp = np.zeros_like(p)
            dp[i] = 1e-3 if i == 5 else 1e-2
            jac[:, i] = ((model(p + dp, pulses) - seen).ravel() - res) / dp[i]
        h = jac.T @ jac
        step = np.linalg.solve(h + lam * np.diag(np.diag(h) + 1e-9), -jac.T @ res)
        new = (model(p + step, pulses) - seen).ravel()
        if new @ new < res @ res:
            p, res, lam = p + step, new, lam * 0.3
            if np.linalg.norm(step) < 1e-6:
                break
        else:
            lam *= 10
    return p, float(np.sqrt(np.mean(np.sum(res.reshape(-1, 3) ** 2, 1))))


def sigma(p: np.ndarray, pulses: list[dict], seen: np.ndarray) -> np.ndarray:
    """1-sigma of each parameter from the fit's Jacobian and residual scatter: an offset that trades off against the
    tip comes out wide, and is not used (see USE_SIGMA)."""
    res = (model(p, pulses) - seen).ravel()
    jac = np.empty((res.size, p.size))
    for i in range(p.size):
        dp = np.zeros_like(p)
        dp[i] = 1e-3 if i == 5 else 1e-2
        jac[:, i] = ((model(p + dp, pulses) - seen).ravel() - res) / dp[i]
    var = res @ res / max(1, res.size - p.size)
    return np.sqrt(np.diag(np.linalg.pinv(jac.T @ jac)) * var)


USE_SIGMA = 1.5  # pulses: an offset known worse than this is written as 0


def poses(n: int = 40, seed: int = 0) -> list[dict]:
    """Poses around the start, only ones that lift or swing (elbow + raises, shoulder + rises), in nearest-next order."""
    # wide ranges: with elbow 0-40 and wrist +-25 the elbow offset traded off against the tip (10 pulses moved the tip
    # < 1 mm once the tip refit); elbow 0-120, shoulder 0-60, wrist +-60 recovered 5.0/-3.8 for a true 6/-4. The wrist
    # swing grows with the elbow lift, so the low poses keep the tip off the paper
    rng = np.random.default_rng(seed)
    out = [dict(START)]
    for _ in range(n - 1):
        el = int(rng.integers(0, 121))
        wr = 25 + int(0.3 * el)
        out.append({**START, 1: START[1] + int(rng.integers(-35, 36)), 2: START[2] + int(rng.integers(0, 61)),
                    3: START[3] + el, 4: START[4] + int(rng.integers(-wr, wr + 1))})
    order, left = [out[0]], out[1:]
    while left:
        j = int(np.argmin([sum(abs(order[-1][k] - q[k]) for k in (1, 2, 3, 4)) for q in left]))
        order.append(left.pop(j))
    return order


def selftest() -> None:
    truth = np.array([6.0, -4.0, 5.0, 28.0, 195.0, 1.2, 40.0, -90.0, -35.0])
    qs = poses(40)
    rng = np.random.default_rng(1)
    seen = model(truth, qs) + rng.normal(0.0, 0.8, (len(qs), 3))  # ~1 mm triangulation noise
    p, rms = fit(qs, seen)
    sd = sigma(p, qs, seen)
    assert all(abs(p[i] - truth[i]) <= 3 * sd[i] + 0.5 for i in (0, 1)), (p[:2], sd[:2], truth[:2])
    print(f"arm_calibrate self-check ok: offsets {np.round(p[:2], 2)} +- {np.round(sd[:2], 2)} (true {truth[:2]}), "
          f"rms {rms:.2f} mm; used: {[bool(sd[i] <= USE_SIGMA) for i in (0, 1)]}")


def collect() -> None:
    """Drive the poses, triangulate the inked tip at each (overhead teal tip + side SAM tip), save the data."""
    import base64  # noqa: F401  (robot I/O only here)
    import markers  # noqa: F401
    import model_jab as mj
    import sam_detect as sam
    import tip_triangulate as tt
    from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
    from tip_descend import read, send
    import dashboard as dash
    predictor = sam.load()
    det = lambda path, q, th: sam.detect(predictor, path, q, th)  # noqa: E731
    top, side = Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER)
    scratch = Path("evidence/tip/_cal.png")
    img = top.frame()
    rig = tt.ugreen_rig(img)
    uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
    cv2.imwrite(str(scratch), img)
    red = [d["centroid"] for d in det(str(scratch), "small red dot", mj.SAM_THRESHOLD)
           if d["centroid"] and mj.RED_AREA[0] <= d["area"] <= mj.RED_AREA[1]]
    known = json.loads(Path("evidence/descend/ugreen_dots.json").read_text())
    sheet = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in known + red])
    srig = tt.side_rig(tt.side_marks(side.frame(), det, scratch), sheet)
    sk = tuple(np.array(srig[q]) for q in ("k", "d", "r", "t"))
    print(f"side pose: {srig['dots']} dots, {srig['rms']:.1f} px", flush=True)
    rows = []
    try:
        for i, q in enumerate(poses()):
            send(q, ms=1200)
            time.sleep(1.6)
            now = read()
            ug_px = dash.teal_tip(top.frame())
            sf = side.frame()
            cv2.imwrite(str(scratch), sf)
            found = det(str(scratch), "thin metal needle", 0.15)
            if ug_px is None or not found:
                print(f"{i:2d} tip not found (overhead {ug_px is not None}, side {bool(found)})", flush=True)
                continue
            vs, us = np.nonzero(max(found, key=lambda d: d["score"])["mask"])
            k = int(np.argmax(vs))
            p3, miss = tt.meet(tt.ray(np.array(ug_px), uk, ud, ur, ut), tt.ray(np.array([us[k], vs[k]], float), *sk))
            print(f"{i:2d} pulses {now} tip {np.round(p3, 1)} miss {miss:.1f} mm", flush=True)
            if miss <= 3.0:
                rows.append({"pulses": {str(j): int(now[j]) for j in (1, 2, 3, 4, 5)}, "tip_mm": p3.tolist(), "miss_mm": miss})
            if -p3[2] < MIN_HEIGHT_MM:
                print(f"tip {-p3[2]:.1f} mm up: ending the sweep", flush=True)
                break
    finally:
        send({3: START[3] + 40}, ms=1200)  # elbow up, away from the paper
        time.sleep(1.5)
        top.close()
        side.close()
        scratch.unlink(missing_ok=True)
        DATA.write_text(json.dumps(rows, indent=1))
        print(f"{len(rows)} poses saved to {DATA}", flush=True)


def fit_file(path: Path) -> None:
    rows = json.loads(path.read_text())
    qs = [{int(k): v for k, v in r["pulses"].items()} for r in rows]
    seen = np.array([r["tip_mm"] for r in rows])
    p, rms = fit(qs, seen)
    sd = sigma(p, qs, seen)
    _, rms0 = fit(qs, seen, p0=np.r_[0.0, 0.0, p[2:]], iters=0)
    res = np.linalg.norm(model(p, qs) - seen, axis=1)
    use = [float(p[i]) if sd[i] <= USE_SIGMA else 0.0 for i in (0, 1)]
    OUT.write_text(json.dumps({"shoulder_offset_pulses": use[0], "elbow_offset_pulses": use[1],
                               "fitted": p[:2].tolist(), "sigma_pulses": sd[:2].tolist(),
                               "rms_mm": rms, "rms_without_offsets_mm": rms0, "poses": len(rows),
                               "max_residual_mm": float(res.max()), "tip_cam_mm": p[2:5].tolist()}, indent=1))
    print(f"offsets shoulder {p[0]:+.1f} +- {sd[0]:.1f}, elbow {p[1]:+.1f} +- {sd[1]:.1f} pulses (used {use}); rms {rms:.2f} mm (without offsets {rms0:.2f}), "
          f"max {res.max():.2f} mm over {len(rows)} poses -> {OUT}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=("collect", "fit", "selftest"))
    ap.add_argument("data", nargs="?", type=Path, default=DATA)
    args = ap.parse_args()
    {"collect": collect, "selftest": selftest, "fit": lambda: fit_file(args.data)}[args.cmd]()


if __name__ == "__main__":
    main()
