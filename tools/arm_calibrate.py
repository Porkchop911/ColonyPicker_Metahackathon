#!/usr/bin/env python3
"""Arm calibration against the external cameras (plan 12 L2): joint zero offsets from tip positions triangulated by the
overhead and side UGREENs over ~40 poses, so the arm model's 5-10 mm absolute error stops costing placing iterations.

    <hackathon venv python> tools/arm_calibrate.py collect      robot on, arm at the start pose, inked tip (~3 min)
    <hackathon venv python> tools/arm_calibrate.py fit [DATA]   fit and write evidence/tip/arm_offsets.json
    <hackathon venv python> tools/arm_calibrate.py selftest     the fit on synthetic data with known offsets
    <hackathon venv python> tools/arm_calibrate.py repeat       gate 0: return-to-START scatter per joint and side (~10 min)
    <hackathon venv python> tools/arm_calibrate.py repeat-report FILE   its numbers and the go/stop verdict
    <hackathon venv python> tools/arm_calibrate.py fit-arm RUN...      arm terms (offsets, approach) -> arm_model.json
    <hackathon venv python> tools/arm_calibrate.py aim          live A/B: nominal vs calibrated on grid crossings at 60 mm

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
    # the base stands on the desk: yaw only, then base z up -> sheet z down (heights are -z; yaw alone could not fit
    # live data: offsets of hundreds of pulses, 2026-10-30)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])
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
    # fit_arm: synthetic run (0.8 mm noise): direction terms to 1 pulse; offsets to 4 (they trade against the tip, as above)
    truth = np.array([6.0, -4.0, 5.0, -6.0, -4.0, -1.0, 3.0, -20.0, 170.0, 1.5, 40.0, -30.0, -10.0])
    qs = [{**START, 1: START[1] + int(rng.integers(-35, 36)), 2: START[2] + int(rng.integers(-40, 21)),
           3: START[3] + int(rng.integers(-50, 21)), 4: START[4] + int(rng.integers(-40, 41))} for _ in range(60)]
    signs = [{k: int(rng.choice((-1, 1))) for k in (1, 2, 3, 4)} for _ in qs]
    tips = arm_model(truth, qs, signs) + rng.normal(0.0, 0.8, (len(qs), 3))
    tmp, tmp_out = Path("evidence/tip/_selftest_run.json"), Path("evidence/tip/_selftest_model.json")
    tmp.write_text(json.dumps([{"cmd": {str(k): v for k, v in q.items()}, "approach": {str(k): v for k, v in sg.items()},
                                "tip_mm": t.tolist()} for q, sg, t in zip(qs, signs, tips)]))
    try:
        got = fit_arm([tmp], out_path=tmp_out)
    finally:
        tmp.unlink(missing_ok=True)
        tmp_out.unlink(missing_ok=True)
    fitted = np.r_[got["d2"], got["d3"], [got["b"][str(k)] for k in (1, 2, 3, 4)]]
    assert np.all(np.abs(fitted - truth[:6]) <= [4, 4, 1, 1, 1, 1]), (fitted, truth[:6])
    print(f"fit_arm self-check ok: offsets {fitted[:2].round(1)} (true {truth[:2]}), direction {fitted[2:].round(1)} "
          f"(true {truth[2:6]})")
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
    import paper_grid as pg
    import sam_detect as sam
    import tip_triangulate as tt
    from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
    predictor = sam.load()
    det = lambda path, q, th: sam.detect(predictor, path, q, th)  # noqa: E731
    top, side = Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER)
    img = top.frame()
    import plate_rig as prig
    if prig.TOP_CACHE.exists():  # tilted overhead since 2026-10-30: both poses from plate_rig.py, no ink dots
        fits = prig.check(img)
        if fits < prig.CHECK_MIN:
            fits = prig.refresh(img)
            print(f"overhead crept: pose regrown from the cache, {fits} crossings", flush=True)
        if fits < prig.CHECK_MIN:
            raise RuntimeError(f"cached overhead pose fits {fits} crossings (< {prig.CHECK_MIN}): camera moved? "
                               f"rerun tools/plate_rig.py")
        rigs = [json.loads(p.read_text()) for p in (prig.TOP_CACHE, prig.SIDE_CACHE)]
        print(f"cached poses: overhead {fits} crossings on the lattice, side camera "
              f"{-rigs[1]['centre_mm'][2]:.0f} mm up", flush=True)
        return top, side, det, *(tuple(np.array(r[q]) for q in ("k", "d", "r", "t")) for r in rigs)
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
    # Cached: the camera does not move. Never refit over a cache: the regular 2x4 dot block lets a shifted pose fit as
    # well (11 dots, 0.46 px, centre 47 mm off) and a refit at 14:47 broke every measurement (2026-10-30).
    srig = json.loads(SIDE_CACHE.read_text()) if SIDE_CACHE.exists() else None
    if srig is not None:
        # the overhead numbers the grid from whichever crossing it locks onto, so its sheet frame moves by whole squares
        # between runs (cache fit 0/12 dots with neither camera moved, 2026-10-30): shift this run's overhead frame back
        base = sheet.copy()
        shifts = [np.array([i, j]) * pg.SQUARE_MM for i in range(-20, 21) for j in range(-20, 21)]
        hits = []
        for s in shifts:
            sheet[:] = base + s
            hits.append(agree(srig))
        best = shifts[int(np.argmax(hits))]
        sheet[:] = base + best
        if max(hits) < 0.6 * len(sheet):
            raise RuntimeError(f"side pose cache fits only {max(hits)}/{len(sheet)} dots at any grid shift: camera or sheet "
                               f"moved? delete {SIDE_CACHE} to refit, then check a measured tip by eye")
        k, d, r, t = urig  # cam = R sheet + t; sheet' = sheet + s  ->  t' = t - R s
        urig = (k, d, r, t - (cv2.Rodrigues(np.array(r, float))[0] @ np.r_[best, 0.0]).reshape(np.shape(t)))
        print(f"overhead grid shifted by {best.round(1).tolist()} mm to the cached side pose", flush=True)
    if srig is None:
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


TIP_MIN_PX = 200  # overhead ink blob: the tip is 800-1700 px; a 60 px tint at the plate's corner met junk (2026-10-30)
TEAL = ((40, 25, 60), (110, 255, 255))  # HSV range of the tip ink (upright_touch_session)


def paper_hull(img: np.ndarray) -> np.ndarray:
    """Convex hull of the biggest bright area: the paper (as tt.side_marks finds it)."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, lab, st, _ = cv2.connectedComponentsWithStats((gray > 0.8 * np.percentile(gray, 90)).astype(np.uint8))
    ys, xs = np.nonzero(lab == 1 + int(np.argmax(st[1:, 4])))
    return cv2.convexHull(np.column_stack([xs, ys]).astype(np.int32))


def teal_ends(img: np.ndarray, on_paper: bool, min_px: int = 6) -> list[tuple[np.ndarray, int]]:
    """(lowest pixel, area) of each teal blob, over the paper (`on_paper`) or off it."""
    paper = paper_hull(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # under the warm colony lamp the ink reads hue 22-30 against the paper's 15 (2026-10-30): floor relative to the paper
    lo = (min(TEAL[0][0], int(np.median(hsv[..., 0][gray > np.percentile(gray, 70)])) + 8), *TEAL[0][1:])
    n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.inRange(hsv, lo, TEAL[1]))
    out = []
    for j in range(1, n):
        if st[j, 4] < min_px:
            continue
        vs, us = np.nonzero(lab == j)
        px = np.array([us[vs == vs.max()].mean(), vs.max()])
        if (cv2.pointPolygonTest(paper, (float(px[0]), float(px[1])), False) >= 0) == on_paper:
            out.append((px, int(st[j, 4])))
    return out


BG_LIFT = {2: 25, 3: 45, 4: 25}  # pulses over START that take the needle out of the side camera's view (2026-10-30)


def side_background(side) -> np.ndarray:
    """Side frame with the needle lifted out of view (arm at START + BG_LIFT), for needle_end."""
    from tip_descend import send
    send({**START, **{k: START[k] + v for k, v in BG_LIFT.items()}}, ms=1500)
    time.sleep(2.0)
    return side.frame()


def needle_end(frame: np.ndarray, bg: np.ndarray) -> np.ndarray | None:
    """The needle's lowest pixel in the side view by difference against `bg`: of the changed blobs >= 1500 px not
    touching the left edge (robot body), the one reaching lowest. SAM 3.1 returned nothing on ~half the frames at
    START; teal colour fails there (the box behind reads teal too); this matched SAM's end to 1-10 px (2026-10-30)."""
    blur = lambda img: cv2.GaussianBlur(img, (5, 5), 0).astype(np.int16)  # noqa: E731
    d = cv2.morphologyEx((np.abs(blur(frame) - blur(bg)).max(axis=2) > 30).astype(np.uint8), cv2.MORPH_OPEN,
                         np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(d)
    keep = [j for j in range(1, n) if st[j, 4] >= 1500 and st[j, 0] > 0]
    if not keep:
        return None
    vs, us = np.nonzero(lab == max(keep, key=lambda j: st[j, 1] + st[j, 3]))
    return np.array([us[vs == vs.max()].mean(), vs.max()], float)


def measure(top, side, det, urig, srig, bg: np.ndarray | None = None):
    """Inked tip in the sheet frame (mm, z down) from SAM's needle end in the side view and an overhead teal blob whose
    rays meet. SAM returns nothing on ~half the frames at START (2026-10-30): callers retry on fresh frames.
    Overhead: teal blobs over the paper only. Of the blobs that meet to MISS_MM the biggest wins (old ink spots ~170 px
    lined up too, the tip is ~1500 px); none -> the closest, which the caller rejects by its miss. (p3, miss), or None."""
    import tip_triangulate as tt
    sf = side.frame()
    end = None if bg is None else needle_end(sf, bg)
    found = []
    if end is None:
        cv2.imwrite(str(SCRATCH), sf)
        # second prompt as upright_touch_session; a teal-colour fallback paired box art with the tip (y 13-50 mm off)
        found = det(str(SCRATCH), "thin metal needle", 0.15) or det(str(SCRATCH), "black stick", 0.3)
    # the ink bulb itself, where it reads teal (relative to the paper): in the plate_rig layout the lifted needle stays in
    # the side view and the background difference picked a blob by a dish (2026-10-30); rays must still meet below
    inked = [(px, a) for px, a in teal_ends(sf, on_paper=False) + teal_ends(sf, on_paper=True) if a >= 30]
    if end is not None or inked:
        sides = inked or [(end, 0)]  # the difference end only without ink: it paired a plate corner (2026-10-30)
    elif found:
        vs, us = np.nonzero(max(found, key=lambda d: d["score"])["mask"])
        k = int(np.argmax(vs))
        spx = np.array([us[k], vs[k]], float)
        # SAM's mask stops where a thick ink bulb begins: the bulb's lowest teal pixel is the real end. The bulb touches
        # SAM's end (6 px) and ends within 20 px below it: paper ink touching the end in the image dragged it 59 px down
        xs, ys = int(spx[0]), int(spx[1])
        wx, wy = max(0, xs - 60), max(0, ys - 30)
        n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.inRange(cv2.cvtColor(sf[wy:ys + 60, wx:xs + 60], cv2.COLOR_BGR2HSV), *TEAL))
        near = [j for j in range(1, n) if st[j, 4] >= 6 and st[j, 1] + st[j, 3] + wy <= ys + 20
                and np.min(np.hypot(*(np.nonzero(lab == j)[::-1] + np.array([[wx], [wy]]) - spx[:, None]))) <= 6.0]
        if near:
            ty, tx = np.nonzero(lab == max(near, key=lambda j: st[j, 4]))
            spx = np.array([tx[ty == ty.max()].mean() + wx, ty.max() + wy], float)
        sides = [(spx, 0)]
    else:
        print("tip not found: no needle in the side view", flush=True)
        return None
    # overhead: only over the paper: the cyan line art on the box at its right edge met the side ray to 0.01 mm 110 mm
    # from the tip (2026-10-30)
    tops = [(px, a) for px, a in teal_ends(top.frame(), on_paper=True) if a >= TIP_MIN_PX]
    if not tops:
        print("tip not found: no teal in the overhead view", flush=True)
        return None
    cands = [(*tt.meet(tt.ray(tpx, *urig), tt.ray(spx, *srig)), area + s_area, tpx, spx) for spx, s_area in sides
             for tpx, area in tops]
    fit = [c for c in cands if c[1] <= MISS_MM]
    p3, miss, _, tpx, spx = max(fit, key=lambda c: c[2]) if fit else min(cands, key=lambda c: c[1])
    measure.px = (tpx.tolist(), spx.tolist())  # the pixels behind the last result, for a fit in image space
    return p3, miss


COLLECT_N = 200     # poses (plan step 2)
ANCHOR_EVERY = 20   # START again from +REPEAT_AWAY (gate 0's repeatable side; from below pan scatters 2.6 mm) every this many poses
DRIFT_MM = 2.0      # an anchor this far from the first one twice in a row ends the run
SAFE_MM = 25.0      # a pose or its pre-move predicted lower than this (measured START height + measured slopes) is skipped
PROBE = 10          # pulses per joint around START for the measured height slopes
LOCAL_MM = 40.0     # a pose or pre-move the plane fitted to the measured poses puts lower than this is skipped


def approach_plan(qs: list[dict], seed: int = 1) -> list[tuple[dict, dict, dict]]:
    """(pre-move, target, approach sign per joint) for each pose. The sign is drawn per pose and joint, balanced and
    independent of the pose (nearest-next order alone would tie it to the pose and let backlash soak up sag or gain
    error; meta review 2026-10-30): the joint first goes REPEAT_AWAY pulses to the other side, then to the target."""
    rng = np.random.default_rng(seed)
    out = []
    for q in qs:
        sign = {k: int(rng.choice((-1, 1))) for k in (1, 2, 3, 4)}
        out.append(({**q, **{k: q[k] - sign[k] * REPEAT_AWAY for k in sign}}, q, sign))
    return out


def low_plan(n: int, rise, height: float, seed: int = 0) -> list[tuple[dict, dict, dict]]:
    """approach_plan for n poses between SAFE_MM (1.5x model margin, pre-move included) and START height, nearest-next
    order. `rise(q)`: the model's tip rise from START (mm), `height`: START's measured tip height. The side camera's
    frame ends just above START (tip ~85 mm up): poses() lifted the tip out of it, 55 of 63 unmeasured (2026-10-30).
    Wide joint ranges, kept by the height band; the band also sits nearer the 10 mm hover where the model is used."""
    rng = np.random.default_rng(seed)
    safe = lambda q: height + 1.5 * min(rise(q), 0.0) >= SAFE_MM  # noqa: E731
    out = [dict(START)]
    for _ in range(100 * n):
        if len(out) >= n:
            break
        # elbow -100 took the tip 16 mm under the paper plane (2026-10-30): ranges kept to what measured safe
        q = {**START, 1: START[1] + int(rng.integers(-35, 36)), 2: START[2] + int(rng.integers(-40, 21)),
             3: START[3] + int(rng.integers(-50, 21)), 4: START[4] + int(rng.integers(-40, 41))}
        if rise(q) <= 3.0 and safe(q):
            out.append(q)
    order, left = [out[0]], out[1:]
    while left:
        j = int(np.argmin([sum(abs(order[-1][k] - q[k]) for k in (1, 2, 3, 4)) for q in left]))
        order.append(left.pop(j))
    return [(pre, q, s) for pre, q, s in approach_plan(order, seed + 1) if safe(pre)]


def collect() -> None:
    """Drive COLLECT_N poses, each reached from a random side per joint, triangulate the inked tip at each (overhead
    teal tip + side SAM tip); command, read-back, previous position, approach and dwell logged; START re-measured
    every ANCHOR_EVERY poses."""
    from tip_descend import read, send
    top, side, det, urig, srig = cameras()
    bg = side_background(side)
    rows, anchor0, drifted = [], None, 0
    # both camera poses in this run's sheet frame: a fit in image space reprojects the model tip into each
    rows.append({"rig": {"top": [np.asarray(v).tolist() for v in urig], "side": [np.asarray(v).tolist() for v in srig]}})
    try:
        send(START, ms=2500)
        time.sleep(2.0)
        def height_at(q: dict) -> float:
            """Measured tip height (mm) at q, approached from PROBE pulses above on every joint; side SAM misses ~1 in 5."""
            send({**q, **{k: q[k] + PROBE for k in (1, 2, 3, 4)}}, ms=800)
            time.sleep(0.8)
            send(q, ms=800)
            time.sleep(0.8 + REPEAT_DWELL)
            got = next((g for g in (measure(top, side, det, urig, srig, bg) for _ in range(5)) if g is not None), None)
            if got is None:
                raise RuntimeError("tip not found: ink the tip, check the lamps")
            return float(-got[0][2])

        # the guard's height slopes are measured, not modelled: TIP_GUESS put a wrist -19 pose ~70 mm up, it measured 24
        # and the sweep hit 11.5 mm on pose 8 (2026-10-30). Linear over the ranges low_plan draws; MIN_HEIGHT_MM still stops it.
        height = height_at(START)
        slope = {k: (height_at({**START, k: START[k] + PROBE}) - height_at({**START, k: START[k] - PROBE})) / (2 * PROBE)
                 for k in (1, 2, 3, 4)}
        rise = lambda q: sum(slope[k] * (q[k] - START[k]) for k in slope)  # noqa: E731
        low = lambda q: height + 1.5 * min(rise(q), 0.0) < SAFE_MM  # noqa: E731
        seen_q, seen_h = [], []  # read-back pulses and measured heights so far, seeded with earlier runs' poses
        for f in sorted(DATA.parent.glob("arm_calibration_data*.json")):  # heights do not depend on the grid numbering
            for r in json.loads(f.read_text()):
                if "pulses" in r:
                    seen_q.append([r["pulses"][str(k)] for k in (1, 2, 3, 4)])
                    seen_h.append(-r["tip_mm"][2])
        print(f"height fit seeded with {len(seen_q)} earlier poses", flush=True)

        def local(q: dict) -> float:
            """Height (mm) at q from a plane fitted to the poses measured so far (>= 8), else the START slopes: the START
            slopes were 2-6x off a few poses away (elbow 0.39 measured, 2.2 mm/pulse real, 2026-10-30)."""
            if len(seen_q) < 8:
                return height + rise(q)
            a = np.column_stack([np.ones(len(seen_q)), np.array(seen_q, float) - [START[k] for k in (1, 2, 3, 4)]])
            coef = np.linalg.lstsq(a, np.array(seen_h), rcond=None)[0]
            return float(coef @ np.r_[1.0, [q[k] - START[k] for k in (1, 2, 3, 4)]])
        plan = low_plan(COLLECT_N, rise, height, seed=int(time.time()) % 100000)  # new poses each run
        print(f"START tip {height:.1f} mm up; slopes mm/pulse {{{', '.join(f'{k}: {v:+.2f}' for k, v in slope.items())}}}; "
              f"{len(plan)} poses", flush=True)
        for i, (pre, q, sign) in enumerate(plan):
            if i % ANCHOR_EVERY == 0:
                send({**START, **{k: START[k] + REPEAT_AWAY for k in (1, 2, 3, 4)}}, ms=1000)
                time.sleep(1.0)
                send(START, ms=800)
                time.sleep(0.8 + REPEAT_DWELL)
                got = next((g for g in (measure(top, side, det, urig, srig, bg) for _ in range(5)) if g is not None), None)
                if got is not None and got[1] <= MISS_MM:
                    anchor0 = got[0] if anchor0 is None else anchor0
                    off = float(np.linalg.norm(got[0] - anchor0))
                    drifted = drifted + 1 if off > DRIFT_MM else 0
                    print(f"anchor {i}: {off:.2f} mm from the first", flush=True)
                    rows.append({"anchor": True, "cmd": {str(k): v for k, v in START.items()},
                                 "read": {str(k): int(v) for k, v in read().items()}, "tip_mm": got[0].tolist(),
                                 "miss_mm": got[1], "t": time.time()})
                    if drifted >= 2:
                        print("anchor moved twice: stopping", flush=True)
                        break
            if low(pre) or low(q) or min(local(pre), local(q)) < LOCAL_MM:
                print(f"{i:3d} skipped: predicted below {SAFE_MM:.0f} mm (slopes) or {LOCAL_MM:.0f} mm (fit "
                      f"{min(local(pre), local(q)):.0f})", flush=True)
                continue
            send(pre, ms=1000)
            time.sleep(1.0)
            prev = read()
            send(q, ms=800)
            time.sleep(0.8 + REPEAT_DWELL)
            now = read()
            got = next((g for g in (measure(top, side, det, urig, srig, bg) for _ in range(5)) if g is not None), None)
            if got is None:
                continue
            p3, miss = got
            print(f"{i:3d} cmd {[q[k] for k in (1, 2, 3, 4)]} read {[now[k] for k in (1, 2, 3, 4)]} "
                  f"tip {np.round(p3, 1)} miss {miss:.1f} mm", flush=True)
            if miss <= MISS_MM:
                seen_q.append([now[k] for k in (1, 2, 3, 4)])
                seen_h.append(float(-p3[2]))
                rows.append({"pulses": {str(j): int(now[j]) for j in (1, 2, 3, 4, 5)},  # read-back: what fit uses
                             "top_px": measure.px[0], "side_px": measure.px[1],
                             "cmd": {str(j): int(q[j]) for j in (1, 2, 3, 4, 5)},
                             "prev": {str(j): int(prev[j]) for j in (1, 2, 3, 4, 5)},
                             "approach": {str(k): v for k, v in sign.items()}, "dwell_s": REPEAT_DWELL,
                             "tip_mm": p3.tolist(), "miss_mm": miss, "t": time.time()})
            if miss <= MISS_MM and -p3[2] < MIN_HEIGHT_MM:  # a bad pair (41 mm miss) read 1.7 mm and ended run 9
                print(f"tip {-p3[2]:.1f} mm up: ending the sweep", flush=True)
                break
    finally:
        send({3: START[3] + 40}, ms=1200)  # elbow up, away from the paper
        time.sleep(1.5)
        top.close()
        side.close()
        SCRATCH.unlink(missing_ok=True)
        out = DATA.with_name(f"arm_calibration_data_{time.strftime('%Y%m%dT%H%M%S')}.json")  # never overwrite a run
        out.write_text(json.dumps(rows, indent=1))
        print(f"{len(rows)} poses saved to {out}", flush=True)


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
    bg = side_background(side)
    rows = []
    try:
        send(START, ms=2500)
        time.sleep(2.0)
        for i in range(STATIC_N):
            got = measure(top, side, det, urig, srig, bg)
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
            got = measure(top, side, det, urig, srig, bg)
            if got is None:
                continue
            p3, miss = got
            print(f"{i:3d} joint {j} side {s:+d} dwell {dwell} tip {np.round(p3, 1)} miss {miss:.1f} mm", flush=True)
            rows.append({"joint": j, "side": s, "dwell_s": dwell, "cmd": {str(k): v for k, v in START.items()},
                         "read": {str(k): int(now[k]) for k in START}, "prev": {str(k): int(prev[k]) for k in START},
                         "tip_mm": p3.tolist(), "miss_mm": miss, "t": time.time()})
            if miss <= MISS_MM and -p3[2] < MIN_HEIGHT_MM:
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
    rows = [r for r in json.loads(path.read_text()) if "pulses" in r]
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


ARM_MODEL = Path("evidence/tip/arm_model.json")
def arm_model(p: np.ndarray, qs: list[dict], signs: list[dict]) -> np.ndarray:
    """Tip in the sheet frame (mm): p = (d2, d3, b1-b4, tip_cam xyz, yaw, base xyz); arm terms as arm_pulses."""
    d2, d3, b1, b2, b3, b4, tx, ty, tz, yaw, bx, by, bz = p
    terms = {"d2": d2, "d3": d3, "b": {"1": b1, "2": b2, "3": b3, "4": b4}}
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])  # as model()
    return np.array([rot @ ca.tip_in_base(ca.arm_pulses(q, terms, sg), np.array([tx, ty, tz])) + [bx, by, bz]
                     for q, sg in zip(qs, signs)])


def fit_arm(paths: list[Path], key: str = "cmd", out_path: Path = ARM_MODEL, tie_only: bool = False) -> dict:
    """Fit arm_model to collected runs and write the arm's own terms to ARM_MODEL. Live 2026-10-30 (run 11, 78 poses,
    5-fold blocks held out): commanded pulses + approach terms 1.8 mm median / 4.0 mm p90; without approach terms no
    model got under ~9 mm. The tip, base and grid frame are per run and only recorded. Several starts (geometry
    first), then the worst 10 % dropped and refit. `tie_only`: keep ARM_MODEL's arm terms, refit only this session's
    tie (tip, yaw, base), e.g. after the cameras or the robot moved."""
    rows = [r for path in paths for r in json.loads(path.read_text()) if "approach" in r and "cmd" in r]
    qs = [{int(k): v for k, v in r[key].items()} for r in rows]
    signs = [{int(k): v for k, v in r["approach"].items()} for r in rows]
    seen = np.array([r["tip_mm"] for r in rows])

    def lm(p, idx, iters, free):
        lam, e = 1e-2, (arm_model(p, [qs[i] for i in idx], [signs[i] for i in idx]) - seen[idx]).ravel()
        for _ in range(iters):
            jac = np.zeros((e.size, p.size))
            for i in np.nonzero(free)[0]:
                dp = np.zeros(p.size)
                dp[i] = 1e-3 if i == 9 else 1e-2
                jac[:, i] = ((arm_model(p + dp, [qs[j] for j in idx], [signs[j] for j in idx]) - seen[idx]).ravel() - e) / dp[i]
            h = jac.T @ jac + np.diag(~free * 1.0)
            step = np.linalg.solve(h + lam * np.diag(np.diag(h) + 1e-9), -jac.T @ e)
            step[~free] = 0.0
            new = (arm_model(p + step, [qs[j] for j in idx], [signs[j] for j in idx]) - seen[idx]).ravel()
            if new @ new < e @ e:
                p, e, lam = p + step, new, lam * 0.3
                if np.linalg.norm(step) < 1e-6:
                    break
            else:
                lam *= 10
        return p, np.linalg.norm(e.reshape(-1, 3), axis=1)

    idx = np.arange(len(rows))
    geometry = np.r_[np.zeros(6, bool), np.ones(7, bool)]
    saved = json.loads(out_path.read_text()) if tie_only else None
    arm0 = np.r_[saved["d2"], saved["d3"], [saved["b"][str(k)] for k in (1, 2, 3, 4)]] if tie_only else np.zeros(6)
    best = None
    for t0 in ([0, 25, 200], [70, -10, 175], [0, 60, 250], [-40, 0, 150], [40, 40, 180]):
        for yaw in np.linspace(-np.pi, np.pi, 12, endpoint=False):
            p0 = np.r_[arm0, t0, yaw, 0.0, 0.0, 0.0]
            p0[10:] = (seen - arm_model(p0, qs, signs)).mean(0)
            p, r = lm(p0, idx, 40, geometry)
            best = (np.median(r), p) if best is None or np.median(r) < best[0] else best
    nominal0 = np.r_[np.zeros(6), best[1][6:]]
    nominal, rn = lm(nominal0, idx, 300, geometry)  # the nominal URDF tied to the sheet by the same data: A/B baseline
    nominal, _ = lm(nominal, idx[rn <= np.percentile(rn, 90)], 300, geometry)
    free = geometry if tie_only else np.ones(13, bool)
    p, r = lm(best[1], idx, 300, free)
    p, _ = lm(p, idx[r <= np.percentile(r, 90)], 300, free)
    r = np.linalg.norm(arm_model(p, qs, signs) - seen, axis=1)
    rn = np.linalg.norm(arm_model(nominal, qs, signs) - seen, axis=1)
    out = {"d2": float(p[0]), "d3": float(p[1]), "b": {str(k): float(p[1 + k]) for k in (1, 2, 3, 4)},
           "input": key, "approach": ca.APPROACH, "poses": len(rows), "median_mm": float(np.median(r)),
           "p90_mm": float(np.percentile(r, 90)), "tip_cam_mm": p[6:9].tolist(),  # record only: per grasp
           # this session's tie to the cached side pose's sheet frame (tip, yaw, base): `aim` uses it, --calib does not
           "session": p.tolist(), "nominal_session": nominal.tolist(), "nominal_median_mm": float(np.median(rn)),
           "runs": [str(x) for x in paths]}
    if tie_only:  # the arm terms and their provenance stay; only the session tie is new
        out = {**saved, "session": out["session"], "nominal_session": out["nominal_session"],
               "tie_median_mm": out["median_mm"], "tie_p90_mm": out["p90_mm"], "tie_poses": len(rows),
               "nominal_median_mm": out["nominal_median_mm"], "tie_runs": out["runs"], "tip_cam_mm": out["tip_cam_mm"]}
    out_path.write_text(json.dumps(out, indent=1))
    print(f"arm model ({key}, {len(rows)} poses): d2 {p[0]:+.1f} d3 {p[1]:+.1f} pulses, b {np.round(p[2:6], 1).tolist()}; "
          f"tip {np.median(r):.2f} mm median, {np.percentile(r, 90):.2f} p90 (in sample; nominal {np.median(rn):.2f}) "
          f"-> {out_path}")
    return out


AIM_HOVER_MM = 60.0  # A/B hover: mid-data (36-91 mm); 40 mm needed shoulder -60 pulses, outside the sampled range
AIM_REACH_MM = 30.0  # crossings this far around the START footprint (15 until the dishes covered it, 2026-10-30)


def aim_solve(p: np.ndarray, goal: np.ndarray, start: dict) -> tuple[dict, float]:
    """Commanded pulses (joints 1-4) putting arm_model(p)'s tip at `goal` (sheet mm), approached from above: damped
    Gauss-Newton as coarse_aim.solve. (pose, residual mm)."""
    q = {k: float(v) for k, v in start.items()}
    up = {k: ca.APPROACH for k in (1, 2, 3, 4)}

    def tip(x: dict) -> np.ndarray:
        return arm_model(p, [x], [up])[0]

    for _ in range(40):
        err = goal - tip(q)
        if np.linalg.norm(err) < 0.05:
            break
        jac = np.column_stack([(tip({**q, k: q[k] + 1.0}) - tip({**q, k: q[k] - 1.0})) / 2.0 for k in (1, 2, 3, 4)])
        step = jac.T @ np.linalg.solve(jac @ jac.T + 1e-2 * np.eye(3), err)
        for i, k in enumerate((1, 2, 3, 4)):
            q[k] += float(np.clip(step[i], -20.0, 20.0))
    return {k: int(round(v)) for k, v in q.items()}, float(np.linalg.norm(goal - tip(q)))


def aim(n: int = 10) -> None:
    """Plan step 6, live A/B: grid crossings around the START footprint at AIM_HOVER_MM, the nominal URDF (tied to the
    sheet by the same data) and the calibrated model in turn, order flipped every pair; each move from 20 pulses above,
    the tip measured by both cameras. Reports |measured - goal| for each."""
    from tip_descend import read, send
    model_json = json.loads(ARM_MODEL.read_text())
    models = {"nominal": np.array(model_json["nominal_session"]), "calibrated": np.array(model_json["session"])}
    top, side, det, urig, srig = cameras()
    bg = side_background(side)
    up = {k: ca.APPROACH for k in (1, 2, 3, 4)}
    # re-anchor: the START tip moved 3 mm between the tie run and the A/B (base or needle slip, 2026-10-30); shift both
    # models' base so they predict the START measured now (translation only, the arm terms untouched)
    send({**START, **{k: START[k] + REPEAT_AWAY for k in (1, 2, 3, 4)}}, ms=1000)
    time.sleep(1.0)
    send(START, ms=800)
    time.sleep(0.8 + REPEAT_DWELL)
    got = next((g for g in (measure(top, side, det, urig, srig, bg) for _ in range(5)) if g is not None), None)
    if got is None or got[1] > MISS_MM:
        raise RuntimeError("START tip not measured: cannot anchor the A/B")
    for name, p in models.items():
        delta = got[0] - arm_model(p, [START], [up])[0]
        p[10:13] += delta
        print(f"{name}: anchored at START, base moved {np.round(delta, 1).tolist()} mm", flush=True)
    start_xy = arm_model(models["calibrated"], [START], [up])[0][:2]
    rng = np.random.default_rng(int(time.time()) % 100000)
    cross = [np.round(start_xy / 5.0) * 5.0 + 5.0 * np.array([i, j]) for i in range(-8, 9) for j in range(-8, 9)]
    # the tip must not be seen over a dish: the ink is not found against the agar (2026-10-30). Paper or the black mat
    # behind the hover point are fine; the crossings themselves need not be visible for the A/B (the mat covers them)
    img = top.frame()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    p90 = np.percentile(gray, 90)
    paper = cv2.erode(((gray > 0.8 * p90) | (gray < 0.45 * p90)).astype(np.uint8), np.ones((15, 15), np.uint8))

    def on_paper(xyz: np.ndarray) -> bool:
        u, v = cv2.projectPoints(np.array([xyz], float), np.asarray(urig[2], float), np.asarray(urig[3], float),
                                 np.asarray(urig[0], float), np.asarray(urig[1], float))[0][0, 0]
        return 0 <= u < gray.shape[1] and 0 <= v < gray.shape[0] and bool(paper[int(v), int(u)])

    cross = [c for c in cross if np.linalg.norm(c - start_xy) <= AIM_REACH_MM and on_paper(np.r_[c, -AIM_HOVER_MM])]
    print(f"{len(cross)} crossings within {AIM_REACH_MM:.0f} mm of START seen over paper or mat", flush=True)
    targets = [cross[i] for i in rng.choice(len(cross), min(n, len(cross)), replace=False)]
    out = Path(f"evidence/tip/arm_aim_{time.strftime('%Y%m%dT%H%M%S')}.json")
    rows = []
    try:
        for i, xy in enumerate(targets):
            goal = np.array([xy[0], xy[1], -AIM_HOVER_MM])  # sheet z is negative above the paper
            for name in (("nominal", "calibrated") if i % 2 == 0 else ("calibrated", "nominal")):
                q, resid = aim_solve(models[name], goal, START)
                if resid > 1.0 or max(abs(q[k] - START[k]) for k in (1, 2, 3, 4)) > 60:
                    print(f"{i} {name}: no solution near START ({resid:.1f} mm)", flush=True)
                    continue
                send({**q, **{k: q[k] - ca.APPROACH * REPEAT_AWAY for k in (1, 2, 3, 4)}}, ms=1000)
                time.sleep(1.0)
                send(q, ms=800)
                time.sleep(0.8 + REPEAT_DWELL)
                got = next((g for g in (measure(top, side, det, urig, srig, bg) for _ in range(5)) if g is not None), None)
                if got is None or got[1] > MISS_MM:
                    print(f"{i} {name}: tip not measured", flush=True)
                    continue
                err = got[0] - goal
                print(f"{i} {name}: goal {goal.round(1).tolist()} cmd {[q[k] for k in (1, 2, 3, 4)]} "
                      f"error xy {np.linalg.norm(err[:2]):.2f} mm, z {err[2]:+.2f} mm", flush=True)
                rows.append({"target": i, "model": name, "goal_mm": goal.tolist(), "cmd": {str(k): v for k, v in q.items()},
                             "read": {str(k): int(v) for k, v in read().items()}, "tip_mm": got[0].tolist(),
                             "miss_mm": got[1], "error_mm": err.tolist()})
                send({**START, 3: START[3] + 20}, ms=1000)  # lift between attempts
                time.sleep(1.2)
    finally:
        send({3: START[3] + 40}, ms=1200)
        time.sleep(1.5)
        top.close()
        side.close()
        SCRATCH.unlink(missing_ok=True)
        out.write_text(json.dumps(rows, indent=1))
        for name in models:
            e = np.array([r["error_mm"] for r in rows if r["model"] == name]).reshape(-1, 3)
            if len(e):
                xy = np.linalg.norm(e[:, :2], axis=1)
                print(f"{name}: {len(e)} moves, xy error median {np.median(xy):.2f} mm, max {xy.max():.2f}; "
                      f"z median {np.median(e[:, 2]):+.2f} mm", flush=True)
        print(f"saved {out}", flush=True)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=("collect", "fit", "fit-arm", "fit-tie", "aim", "selftest", "repeat", "repeat-report"))
    ap.add_argument("data", nargs="*", type=Path, default=[DATA])
    args = ap.parse_args()
    {"collect": collect, "selftest": selftest, "fit": lambda: fit_file(args.data[0]), "repeat": repeat,
     "fit-arm": lambda: fit_arm(args.data), "fit-tie": lambda: fit_arm(args.data, tie_only=True), "aim": aim,
     "repeat-report": lambda: repeat_report(args.data[0])}[args.cmd]()


if __name__ == "__main__":
    main()
