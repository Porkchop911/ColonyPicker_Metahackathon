#!/usr/bin/env python3
"""Arm calibration against the external cameras (plan 12 L2): joint zero offsets from tip positions triangulated by the
overhead and side UGREENs over ~40 poses, so the arm model's 5-10 mm absolute error stops costing placing iterations.

    <hackathon venv python> tools/arm_calibrate.py collect      robot on, arm at the start pose, inked tip (~3 min)
    <hackathon venv python> tools/arm_calibrate.py fit [DATA]   fit and write evidence/tip/arm_offsets.json
    <hackathon venv python> tools/arm_calibrate.py selftest     the fit on synthetic data with known offsets
    <hackathon venv python> tools/arm_calibrate.py repeat       gate 0: return-to-START scatter per joint and side (~10 min)
    <hackathon venv python> tools/arm_calibrate.py repeat-report FILE   its numbers and the go/stop verdict

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
    # gate 0 stats: each side's cluster 1 mm off centre along x (a 2 mm gap), 0.3 mm noise per axis
    rows = [{"static": True, "tip_mm": rng.normal(0.0, 0.1, 3).tolist(), "miss_mm": 0.5} for _ in range(STATIC_N)]
    for j, s, dwell in repeat_plan():
        tip = np.array([s * 1.0, 0.0, -50.0]) + rng.normal(0.0, 0.3, 3) + (s * 0.5 if dwell == SHORT_DWELL else 0.0)
        rows.append({"joint": j, "side": s, "dwell_s": dwell, "tip_mm": tip.tolist(), "miss_mm": 0.5})
    stats = repeat_stats(rows)
    for j in REPEAT_JOINTS:
        r = stats[str(j)]
        assert r["gap_90"][0] - 0.3 <= 2.0 <= r["gap_90"][1] + 0.3, (j, r)
        assert 0.3 <= r["creep_mm"] <= 1.0, (j, r)
    assert 0.3 <= stats["worst_scatter_mm"] <= 1.0 and stats["verdict"].startswith("go"), stats
    print(f"gate-0 stats self-check ok: gap {stats['1']['gap_mm']:.2f} mm (true 2.0), worst scatter "
          f"{stats['worst_scatter_mm']:.2f} mm (true 0.52), verdict '{stats['verdict']}'")


SCRATCH = Path("evidence/tip/_cal.png")
SIDE_CACHE = Path("evidence/tip/side_pose_cache.json")  # the last side pose; refit when < 80 % of dots still fit
SIDE_UP_MM = (110.0, 170.0)  # plausible side-camera height above the paper
MISS_MM = 5.0  # overhead and side tip rays must meet this close (thick ink: the two cameras see points ~3 mm apart)


def cameras():
    """Overhead + side cameras with their poses in the overhead's sheet frame, and SAM: (top, side, det, urig, srig)."""
    import base64  # noqa: F401  (robot I/O only here)
    import markers  # noqa: F401
    import model_jab as mj
    import sam_detect as sam
    import tip_triangulate as tt
    from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
    predictor = sam.load()
    det = lambda path, q, th: sam.detect(predictor, path, q, th)  # noqa: E731
    top, side = Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER)
    img = top.frame()
    rig = tt.ugreen_rig(img)
    urig = tuple(np.array(rig[q]) for q in ("k", "d", "r", "t"))
    cv2.imwrite(str(SCRATCH), img)
    red = [d["centroid"] for d in det(str(SCRATCH), "small red dot", mj.SAM_THRESHOLD)
           if d["centroid"] and mj.RED_AREA[0] <= d["area"] <= mj.RED_AREA[1]]
    # the sheet's black dots as SAM finds them now (the saved list goes stale when dots are added: 4 -> 12, 2026-10-30)
    black = [d["centroid"] for d in det(str(SCRATCH), "black dot", mj.SAM_THRESHOLD)
             if d["centroid"] and mj.SAM_AREA[0] <= d["area"] <= mj.SAM_AREA[1]]
    known = black if len(black) >= 5 else json.loads(Path("evidence/descend/ugreen_dots.json").read_text())
    sheet = np.array([tt.on_sheet(q, *urig)[:2] for q in known + red])
    marks = tt.side_marks(side.frame(), det, SCRATCH)

    def agree(s: dict) -> int:
        """Sheet dots that project onto a side mark (12 px) under side pose `s`."""
        proj = cv2.projectPoints(np.column_stack([sheet, np.zeros(len(sheet))]), np.array(s["r"]), np.array(s["t"]),
                                 np.array(s["k"]), np.array(s["d"]))[0][:, 0]
        return int((np.linalg.norm(proj[:, None] - marks[None], axis=2).min(axis=1) <= 12.0).sum())

    def refine(s: dict) -> dict:
        """Re-solve the pose on every dot that lands on a mark (12 px), 3 rounds: a subset pose -> all dots."""
        k, d = np.array(s["k"]), np.array(s["d"])
        r, t = np.array(s["r"], float).reshape(3, 1), np.array(s["t"], float).reshape(3, 1)
        obj = np.column_stack([sheet, np.zeros(len(sheet))]).astype(np.float64)
        for _ in range(3):
            gap = np.linalg.norm(cv2.projectPoints(obj, r, t, k, d)[0][:, 0][:, None] - marks[None], axis=2)
            hit = gap.min(axis=1) <= 12.0
            if hit.sum() < 5:
                break
            _, r, t = cv2.solvePnP(obj[hit], marks[gap[hit].argmin(axis=1)].astype(np.float64), k, d, r, t,
                                   useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        proj = cv2.projectPoints(obj, r, t, k, d)[0][:, 0]
        gap = np.linalg.norm(proj[:, None] - marks[None], axis=2).min(axis=1)
        centre = -cv2.Rodrigues(r)[0].T @ t.ravel()
        return {**s, "r": r.ravel().tolist(), "t": t.ravel().tolist(), "dots": int((gap <= 12.0).sum()),
                "rms": float(np.sqrt(np.mean(gap[gap <= 12.0] ** 2))), "centre_mm": centre.tolist()}

    def plausible(s: dict) -> bool:
        """The side camera sits ~140 mm above the paper (tape, full 12-dot fit 138 mm); a 10-dot fit at 286 mm put the
        tip 95 mm under the paper (2026-10-30)."""
        return SIDE_UP_MM[0] <= -s["centre_mm"][2] <= SIDE_UP_MM[1]

    # side_rig tries every 4 marks x 4 dots: > 15 min with 12 dots. So: random 6-dot subsets (~15 s each), each refined on
    # all dots, until one fits 80 % (one farthest-first subset alone fit 8/12 and put the tip 1.8 mm up instead of ~50).
    # Cached: the camera does not move.
    srig = json.loads(SIDE_CACHE.read_text()) if SIDE_CACHE.exists() else None
    if srig is None or agree(srig) < 0.8 * len(sheet) or not plausible(srig):
        rng, best = np.random.default_rng(), None
        for attempt in range(20):
            pick = rng.choice(len(sheet), min(6, len(sheet)), replace=False)
            try:
                s = refine(tt.side_rig(marks, sheet[pick]))
            except RuntimeError:
                continue
            print(f"side pose try {attempt}: {s['dots']}/{len(sheet)} dots, {s['rms']:.1f} px, "
                  f"camera {-s['centre_mm'][2]:.0f} mm up", flush=True)
            if not plausible(s):
                continue
            best = s if best is None or (s["dots"], -s["rms"]) > (best["dots"], -best["rms"]) else best
            if best["dots"] >= 0.8 * len(sheet):
                break
        if best is None or best["dots"] < 0.8 * len(sheet):
            raise RuntimeError(f"side pose: at best {best and best['dots']}/{len(sheet)} dots fit")
        srig = best
        SIDE_CACHE.write_text(json.dumps(srig))
    print(f"side pose: {srig['dots']} dots, {srig['rms']:.1f} px; now {agree(srig)}/{len(sheet)} dots on a mark", flush=True)
    return top, side, det, urig, tuple(np.array(srig[q]) for q in ("k", "d", "r", "t"))


TEAL = ((40, 25, 60), (110, 255, 255))  # HSV range of the tip ink (upright_touch_session)


def measure(top, side, det, urig, srig):
    """Inked tip in the sheet frame (mm, z down): side SAM needle end, then the overhead teal blob whose ray best meets
    the side ray (old ink on the paper does not line up; upright_touch_session.measure). (p3, miss mm), or None."""
    import tip_triangulate as tt
    sf = side.frame()
    cv2.imwrite(str(SCRATCH), sf)
    found = det(str(SCRATCH), "thin metal needle", 0.15)
    if not found:
        print("tip not found: no needle in the side view", flush=True)
        return None
    vs, us = np.nonzero(max(found, key=lambda d: d["score"])["mask"])
    k = int(np.argmax(vs))
    spx = np.array([us[k], vs[k]], float)
    # SAM's needle mask stops where a thick ink bulb begins: the bulb's lowest teal pixel within 60 px is the real end
    xs, ys = int(spx[0]), int(spx[1])
    wx, wy = max(0, xs - 60), max(0, ys - 30)
    n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.inRange(cv2.cvtColor(sf[wy:ys + 60, wx:xs + 60], cv2.COLOR_BGR2HSV), *TEAL))
    near = [j for j in range(1, n) if st[j, 4] >= 6 and st[j, 1] + wy <= ys + 5]  # touching SAM's end, not a drip below
    if near:
        ty, tx = np.nonzero(lab == max(near, key=lambda j: st[j, 4]))
        spx = np.array([tx[ty == ty.max()].mean() + wx, ty.max() + wy], float)
    side_ray = tt.ray(spx, *srig)
    tf = top.frame()
    # only blobs over the paper: the cyan line art on the box at the overhead's right edge met the side ray to 0.01 mm
    # 110 mm from the tip (2026-10-30). Paper = convex hull of the biggest bright area (as tt.side_marks finds it).
    gray = cv2.cvtColor(tf, cv2.COLOR_BGR2GRAY)
    _, plab, pst, _ = cv2.connectedComponentsWithStats((gray > 0.8 * np.percentile(gray, 90)).astype(np.uint8))
    pys, pxs = np.nonzero(plab == 1 + int(np.argmax(pst[1:, 4])))
    paper = cv2.convexHull(np.column_stack([pxs, pys]).astype(np.int32))
    n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.inRange(cv2.cvtColor(tf, cv2.COLOR_BGR2HSV), *TEAL))
    best = None
    for j in range(1, n):
        if st[j, 4] < 6:
            continue
        vs, us = np.nonzero(lab == j)
        px = np.array([us[vs == vs.max()].mean(), vs.max()])
        if cv2.pointPolygonTest(paper, (float(px[0]), float(px[1])), False) < 0:
            continue
        got = tt.meet(tt.ray(px, *urig), side_ray)
        best = got if best is None or got[1] < best[1] else best
    if best is None:
        print("tip not found: no teal in the overhead view", flush=True)
    return best


def collect() -> None:
    """Drive the poses, triangulate the inked tip at each (overhead teal tip + side SAM tip), save the data."""
    from tip_descend import read, send
    top, side, det, urig, srig = cameras()
    rows = []
    try:
        for i, q in enumerate(poses()):
            send(q, ms=1200)
            time.sleep(1.6)
            now = read()
            got = measure(top, side, det, urig, srig)
            if got is None:
                continue
            p3, miss = got
            print(f"{i:2d} pulses {now} tip {np.round(p3, 1)} miss {miss:.1f} mm", flush=True)
            if miss <= MISS_MM:
                rows.append({"pulses": {str(j): int(now[j]) for j in (1, 2, 3, 4, 5)}, "tip_mm": p3.tolist(), "miss_mm": miss})
            if -p3[2] < MIN_HEIGHT_MM:
                print(f"tip {-p3[2]:.1f} mm up: ending the sweep", flush=True)
                break
    finally:
        send({3: START[3] + 40}, ms=1200)  # elbow up, away from the paper
        time.sleep(1.5)
        top.close()
        side.close()
        SCRATCH.unlink(missing_ok=True)
        DATA.write_text(json.dumps(rows, indent=1))
        print(f"{len(rows)} poses saved to {DATA}", flush=True)


# Gate 0 (docs/plan_2026-10-30_arm_calibration.md): how well the arm returns to one pose, per joint and approach side
REPEAT_JOINTS = {1: 10, 3: 10, 4: 10, 2: 5}  # arrivals per side; servo 2 fewer (it sticks rising, slow to test)
REPEAT_AWAY = 20      # pulses: the joint leaves this far to one side, then returns (> the widest gap seen, 5 pulses)
REPEAT_DWELL = 1.6    # s after the move, as in collect
SHORT_DWELL = 0.4     # s: AGREED S2's shortest settle; creep between the two is measured
SHORT_N = 5           # short-dwell arrivals per joint
STATIC_N = 30         # still measurements: the cameras' own noise
TIP_GUESS = np.array([0.0, 25.0, 200.0])  # tip in the camera frame, mm (fit's start value): only for the height guard


def repeat_plan() -> list[tuple[int, int, float]]:
    """(joint, side, dwell) arrivals: sides alternate so drift does not line up with a side; short dwells last."""
    plan = []
    for j, n in REPEAT_JOINTS.items():
        plan += [(j, s, REPEAT_DWELL) for _ in range(n) for s in (1, -1)]
        plan += [(j, (1, -1)[i % 2], SHORT_DWELL) for i in range(SHORT_N)]
    return plan


def drop_mm(j: int, side: int) -> float:
    """How far the nominal model lowers the tip at START with joint j REPEAT_AWAY pulses to `side` (mm, + = down)."""
    away = {**START, j: START[j] + side * REPEAT_AWAY}
    return float(ca.tip_in_base(START, TIP_GUESS)[2] - ca.tip_in_base(away, TIP_GUESS)[2])


def repeat() -> None:
    """Gate 0 on the robot: still frames, then each joint leaves START to either side and returns, tip measured each time."""
    from tip_descend import read, send
    out = Path(f"evidence/tip/arm_repeat_{time.strftime('%Y%m%dT%H%M%S')}.json")
    top, side, det, urig, srig = cameras()
    rows = []
    try:
        send(START, ms=2500)
        time.sleep(2.0)
        for i in range(STATIC_N):
            got = measure(top, side, det, urig, srig)
            if got is not None:
                rows.append({"static": True, "tip_mm": got[0].tolist(), "miss_mm": got[1], "t": time.time()})
        heights = [-r["tip_mm"][2] for r in rows]
        if not heights:
            raise RuntimeError("tip not found at START: ink the tip, check the lamps")
        height = float(np.median(heights))
        print(f"START: tip {height:.1f} mm up, {len(heights)}/{STATIC_N} still frames", flush=True)
        # the far side must keep the tip >= MIN_HEIGHT_MM even if the model's drop is 1.5x off (no low open-loop poses)
        unsafe = {(j, s) for j in REPEAT_JOINTS for s in (1, -1) if height - 1.5 * max(drop_mm(j, s), 0.0) < MIN_HEIGHT_MM}
        for j, s in sorted(unsafe):
            print(f"skipping joint {j} side {s:+d}: the model drops the tip {drop_mm(j, s):.1f} mm", flush=True)
        for i, (j, s, dwell) in enumerate(repeat_plan()):
            if (j, s) in unsafe:
                continue
            away = {**START, j: START[j] + s * REPEAT_AWAY}
            send(away, ms=800)
            time.sleep(0.8)
            prev = read()
            send(START, ms=800)
            time.sleep(0.8 + dwell)
            now = read()
            got = measure(top, side, det, urig, srig)
            if got is None:
                continue
            p3, miss = got
            print(f"{i:3d} joint {j} side {s:+d} dwell {dwell} tip {np.round(p3, 1)} miss {miss:.1f} mm", flush=True)
            rows.append({"joint": j, "side": s, "dwell_s": dwell, "cmd": {str(k): v for k, v in START.items()},
                         "read": {str(k): int(now[k]) for k in START}, "prev": {str(k): int(prev[k]) for k in START},
                         "tip_mm": p3.tolist(), "miss_mm": miss, "t": time.time()})
            if -p3[2] < MIN_HEIGHT_MM:
                print(f"tip {-p3[2]:.1f} mm up: stopping", flush=True)
                break
    finally:
        send({3: START[3] + 40}, ms=1200)  # elbow up, away from the paper
        time.sleep(1.5)
        top.close()
        side.close()
        SCRATCH.unlink(missing_ok=True)
        out.write_text(json.dumps(rows, indent=1))
        print(f"{len(rows)} rows saved to {out}; next: arm_calibrate.py repeat-report {out}", flush=True)


def scatter(p: np.ndarray) -> float:
    """RMS 3D distance to the centroid (mm)."""
    return float(np.sqrt(np.mean(np.sum((p - p.mean(0)) ** 2, 1)))) if len(p) > 1 else float("nan")


def repeat_stats(rows: list[dict], boots: int = 1000, seed: int = 0) -> dict:
    """Per joint: same-side scatter, side gap (mm, pulses via the nominal model, bootstrap 90 %), short-dwell creep."""
    rng = np.random.default_rng(seed)
    ok = [r for r in rows if r["miss_mm"] <= MISS_MM]
    out = {"static_mm": scatter(np.array([r["tip_mm"] for r in ok if r.get("static")]))}
    for j in REPEAT_JOINTS:
        arm = {s: np.array([r["tip_mm"] for r in ok if r.get("joint") == j and r["side"] == s and r["dwell_s"] == REPEAT_DWELL])
               for s in (1, -1)}
        res = {"n": {str(s): len(arm[s]) for s in arm}, "scatter_mm": {str(s): scatter(arm[s]) for s in arm}}
        if all(len(arm[s]) >= 3 for s in arm):
            gap = lambda a, b: float(np.linalg.norm(a.mean(0) - b.mean(0)))  # noqa: E731
            boot = [gap(arm[1][rng.integers(0, len(arm[1]), len(arm[1]))], arm[-1][rng.integers(0, len(arm[-1]), len(arm[-1]))])
                    for _ in range(boots)]
            per_pulse = float(np.linalg.norm(ca.tip_in_base({**START, j: START[j] + 1}, TIP_GUESS) - ca.tip_in_base(START, TIP_GUESS)))
            g = gap(arm[1], arm[-1])
            res.update(gap_mm=g, gap_90=[float(np.percentile(boot, 5)), float(np.percentile(boot, 95))],
                       gap_pulses=g / per_pulse, mm_per_pulse=per_pulse)
            short = [r for r in ok if r.get("joint") == j and r["dwell_s"] == SHORT_DWELL]
            if short:
                res["creep_mm"] = float(np.mean([np.linalg.norm(np.array(r["tip_mm"]) - arm[r["side"]].mean(0)) for r in short]))
        out[str(j)] = res
    # judged on each joint's better side: placing always makes its last approach from that side (plan step 5)
    for j in REPEAT_JOINTS:
        sc = out[str(j)]["scatter_mm"]
        out[str(j)]["approach"] = None if all(np.isnan(v) for v in sc.values()) else int(min(sc, key=lambda s: np.nan_to_num(sc[s], nan=np.inf)))
    worst = np.nanmax([np.nanmin(list(out[str(j)]["scatter_mm"].values()) + [np.inf]) for j in REPEAT_JOINTS] + [np.nan])
    worst = np.nan if np.isinf(worst) else worst
    out["worst_scatter_mm"] = float(worst)
    out["verdict"] = ("no data" if np.isnan(worst) else "go: calibrate" if worst <= 0.5 else
                      "go, tight for a 1 mm mark" if worst <= 1.0 else "stop: no model meets the 1 mm gate")
    return out


def repeat_report(path: Path) -> None:
    stats = repeat_stats(json.loads(path.read_text()))
    print(f"still: {stats['static_mm']:.2f} mm RMS")
    for j in REPEAT_JOINTS:
        r = stats[str(j)]
        line = f"joint {j}: n {r['n']}, scatter +{r['scatter_mm']['1']:.2f} / -{r['scatter_mm']['-1']:.2f} mm"
        if "gap_mm" in r:
            line += (f", side gap {r['gap_mm']:.2f} mm [{r['gap_90'][0]:.2f}, {r['gap_90'][1]:.2f}]"
                     f" = {r['gap_pulses']:.1f} pulses at {r['mm_per_pulse']:.2f} mm/pulse")
        if "creep_mm" in r:
            line += f", {SHORT_DWELL} s dwell off by {r['creep_mm']:.2f} mm"
        print(line)
    print(f"approach sides {[(j, stats[str(j)]['approach']) for j in REPEAT_JOINTS]}; worst scatter on those sides "
          f"{stats['worst_scatter_mm']:.2f} mm -> {stats['verdict']}")
    path.with_suffix(".report.json").write_text(json.dumps(stats, indent=1))


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
    ap.add_argument("cmd", choices=("collect", "fit", "selftest", "repeat", "repeat-report"))
    ap.add_argument("data", nargs="?", type=Path, default=DATA)
    args = ap.parse_args()
    {"collect": collect, "selftest": selftest, "fit": lambda: fit_file(args.data), "repeat": repeat,
     "repeat-report": lambda: repeat_report(args.data)}[args.cmd]()


if __name__ == "__main__":
    main()
