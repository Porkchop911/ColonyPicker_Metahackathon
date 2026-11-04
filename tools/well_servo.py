#!/usr/bin/env python3
"""Needle into wells by the cameras alone, with the remounted vertical needle (2026-10-31): no arm model, no taught
pose. Pan, shoulder and wrist move the tip (the elbow holds; the needle starts vertical and tilts a few degrees), their
effect on the tip measured by probing (the Jacobian J, mm per pulse), refitted on every measurement over a well.

Per well: over its centre at each of HEIGHTS_MM over the rim, steps of <= STEP_MM by J at GAIN, each move ending with
every joint coming from AWAY pulses above (one approach side: backlash), each step measured by both cameras (rays
cut near the predicted tip only); at the last height within GATE_MM: straight down by J to DEPTH_MM under the rim,
photos, back up the same way, up to the first height, on to the next well. Every move is checked first: the tip as
J predicts it stays over the floor, and goes under the rim only within ENTRY_XY_MM of the target's centre.

    <venv python> tools/well_servo.py --dry                 # simulated arm and cameras, every command checked
    <venv python> tools/well_servo.py --wells F6 E6         # robot
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plate_rig as pr  # noqa: E402
import well_reach as wr  # noqa: E402

MARKER_L_MM = 15.3     # lower teal band's centroid to tip (bands re-edged black, plate moved: eye-picked tip, rays 0.11 mm)
BAND_GAP_MM = 28.5     # lower to upper band along the needle (same frame); runs 7 took the upper band for the lower one
VERTICAL_SUM = 1157    # shoulder + elbow + wrist with the needle vertical in the side view (sweep, 2026-10-31)
# pan (sideways), shoulder (height), wrist (reach: tilting the needle swings the tip out); the elbow holds. Out by the
# plate the arm is nearly stretched and shoulder and elbow both move the tip almost only up and down (probe over F6:
# -0.06 / +0.03 mm per pulse sideways), so the vertical-holding pair could not correct reach (live, 2026-10-31)
J3 = (1, 2, 4)
AWAY = 8
BG_POSE = {1: 499, 2: 560, 3: 179, 4: 418}  # raised again, vertical (sum 1157): after the base moved, 458 left the
#                                              needle's ghost in the side view by the wells and probes took it for the tip
OVER_PLATE = {1: 601, 2: 330, 3: 173, 4: 660}  # since the plate moved closer: ~26 mm over C4 (eye-picked tip)  # background frames here: the needle vertical, up over the robot's front, far
#                                     from the plate in both views (PARK folds the arm into the remounted needle)
PROBE = 12
# mm per pulse of pan, shoulder, wrist (columns) by the E12 probe, live 2026-11-01; probes over D12 and D11 agreed within
# 0.15 but for one shoulder probe at half (0.42 down): a column further than J_TOL from this is taken as a misreading
J_PRIOR = np.array([[-0.36, 0.33, 0.48], [-0.84, -0.11, -0.18], [0.01, -0.87, -0.48]])
J_TOL = 0.3
HEIGHTS_MM = (30.0, 15.0, 8.0, 4.0, 2.0)
GATES_MM = (3.0, 2.5, 2.5, 2.5, 2.5)  # xy over the centre to go lower, per height: live the servo circled 2-3.4 mm
#                                       around F6's centre at 3-4 mm up; a well is ~6.4 mm across
Z_OK_MM = 1.5
STEP_MM = 5.0
GAIN = 0.5
STEPS = 40  # 20 ran out crossing 80 mm to the first well (base moved); F6 starts 40 mm over the first height: 10 steps of <= 5 mm at half gain ran out (dry run)
DEPTH_MM = 3.0
ENTRY_XY_MM = 2.5
LOOK_MM = 12.0         # a measurement is looked for this close to where J put the tip (8: D6's band sat 51 px off)
START_LOOK_MM = 40.0
BG_MAX_S = 600.0
ORDER = ["C4", "B4", "A4", "A3", "B3", "C3", "D3", "D4", "D5", "C5", "B5", "A5", "A6", "B6", "C6", "D6", "E6",
         "E5", "E4", "E3", "E2", "D2", "C2", "B2", "A2"]  # a snake out from C4, under the start pose
STATE = Path("evidence/tip/well_servo_state.json")  # last commanded pose (start from it, not the sagging read-back)
OUT = Path("evidence/tip/well_servo.json")


def teal_blobs(img: np.ndarray, bg: np.ndarray, hue_lo: int = 55) -> list[np.ndarray]:
    """Centroids of teal blobs that changed against the parked background (the backdrop's ink dots are teal too)."""
    import cv2
    m = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (hue_lo, 28, 50), (100, 255, 255))  # bands over the plate: sat 39-54
    ch = (np.abs(cv2.GaussianBlur(img, (5, 5), 0).astype(np.int16)
                 - cv2.GaussianBlur(bg, (5, 5), 0).astype(np.int16)).max(axis=2) > 25).astype(np.uint8) * 255
    n, _, st, cen = cv2.connectedComponentsWithStats(cv2.morphologyEx(cv2.bitwise_and(m, ch), cv2.MORPH_OPEN,
                                                                      np.ones((3, 3), np.uint8)))
    return [cen[j] for j in range(1, n) if st[j, 4] >= 15]


def side_needle(img: np.ndarray, bg: np.ndarray, near: np.ndarray, r_px: float, v_cut: int):
    """Two points on the needle's line in the side view: the tall thin changed blob, above the plate's outline (v_cut:
    lower down the needle merges with the arm's shadow on the plate into one wide blob, live F6), whose fitted line
    passes nearest `near` (within r_px). Over the white backdrop the needle is a clean dark line."""
    import cv2
    d = (np.abs(cv2.GaussianBlur(img, (5, 5), 0).astype(np.int16)
                - cv2.GaussianBlur(bg, (5, 5), 0).astype(np.int16)).max(axis=2) > 25).astype(np.uint8)
    d[max(v_cut, 0):] = 0
    # only a window around the prediction: above it the moving cable merges with the needle into one wide blob (live)
    u0, u1 = int(max(near[0] - r_px, 0)), int(near[0] + r_px)
    v0 = int(max(near[1] - 200, 0))  # 120 left too little needle over the plate's outline 6 mm up (live E12)
    keep = np.zeros_like(d)
    keep[v0:, u0:u1] = 1
    d = d * keep
    # a vertical opening: a changed horizontal edge by the plate's far rim fused with the needle into one wide blob (E12)
    n, lab, st, _ = cv2.connectedComponentsWithStats(cv2.morphologyEx(d, cv2.MORPH_OPEN, np.ones((9, 3), np.uint8)))
    best = None
    for j in range(1, n):
        if st[j, 4] < 150 or st[j, 3] < 3 * st[j, 2]:
            continue
        vs, us = np.nonzero(lab == j)
        a, b = np.polyfit(vs, us, 1)
        dist = abs(a * near[1] + b - near[0]) / np.hypot(1.0, a)
        if dist <= r_px and (best is None or dist < best[0]):
            best = (dist, np.array([a * vs.min() + b, vs.min()], float), np.array([a * vs.max() + b, vs.max()], float))
    return None if best is None else best[1:]


def marker_tip(eyes, expect: np.ndarray, within: float, label: str) -> np.ndarray | None:
    """The tip by the teal band. Overhead: the teal blob nearest where the band should be (the expected tip MARKER_L_MM
    higher). Side: the needle as a line (its teal reads ambiguous, the side image is teal-cast: ~30 blobs along it).
    The band = the overhead's ray through it cut with the plane of the side camera and the needle's line; the tip
    MARKER_L_MM under it; xy from the overhead's ray through the needle's end cut at that height. Near the plate the
    side view loses the tip against the plate's outline (rays 9 mm apart over F6 18 mm up, 2026-10-31)."""
    import cv2
    import arm_calibrate as ac
    import tip_triangulate as tt
    eyes.n += 1
    sf, tf = eyes.side.frame(), eyes.top.frame()
    cv2.imwrite(f"evidence/tip/well2_{eyes.tag}_{eyes.n:03d}_{label}.raw.side.jpg", sf)
    cv2.imwrite(f"evidence/tip/well2_{eyes.tag}_{eyes.n:03d}_{label}.raw.top.jpg", tf)
    proj = lambda p, rig: cv2.projectPoints(np.asarray(p, float).reshape(1, 3), rig[2], rig[3], rig[0], rig[1])[0][0, 0]  # noqa: E731
    band_exp = expect - [0, 0, MARKER_L_MM]
    r_px = max(40.0, 5.0 * within)
    want = proj(band_exp, eyes.urig)
    v_cut = int(min(proj(pr.well(w), eyes.srig)[1] for w in ("A1", "A12", "H1", "H12")) - 6)  # the plate's far rim
    line = side_needle(sf, eyes.bg_side, proj(band_exp, eyes.srig), r_px, v_cut)
    if line is None:
        eyes.snapshot(f"{eyes.n:03d}_{label}_miss", expect, False)
        return None
    c1, d1 = tt.ray(line[0], *eyes.srig)
    _, d2 = tt.ray(line[1], *eyes.srig)
    nrm = np.cross(d1, d2)

    def on_plane(px):  # overhead pixel -> 3D on the plane of the side camera and the needle's line
        c0, d0 = tt.ray(px, *eyes.urig)
        return c0 + d0 * (nrm @ (c1 - c0)) / (nrm @ d0)

    # a band lies on the side view's needle line, within its extent (+-40 px): wider hues let in other changed blobs
    on_line = lambda p: line[0][1] - 40 <= proj(p, eyes.srig)[1] <= line[1][1] + 40  # noqa: E731
    near = lambda p, q: abs(q[2] - p[2] - BAND_GAP_MM) <= 5.0 and np.linalg.norm(q[:2] - p[:2]) <= 4.0  # q under p  # noqa: E731
    # hue from 22 only when none is found: high up over the dark wood the out-of-focus band reads hue 24-30 (live D12),
    # but at 20 the lower band merged with the shaft (centroid 10 px off)
    for hue_lo in (55, 22):
        bands = [(b, p) for b in teal_blobs(tf, ac.measure.top_bg, hue_lo) for p in [on_plane(b)] if on_line(p)]
        # the lower band: none of the other bands BAND_GAP_MM under it (that one is the upper band)
        lower = [(b, p) for b, p in bands if np.linalg.norm(b - want) <= r_px and not any(near(p, q) for _, q in bands)]
        if bands:  # retried only with no band at all (a far upper band must not open the way for junk)
            break
    if not lower:
        eyes.snapshot(f"{eyes.n:03d}_{label}_miss", expect, False)
        return None
    band_px, band = min(lower, key=lambda bp: np.linalg.norm(bp[0] - want))
    upper = [q for _, q in bands if near(q, band)]
    # the tip on the needle's axis through both bands (the overhead's traced end overshot 26 px onto the plate's rim);
    # with the upper band unseen, straight down (the needle is held vertical)
    axis = band - min(upper, key=lambda q: abs(band[2] - q[2] - BAND_GAP_MM)) if upper else np.array([0.0, 0.0, 1.0])
    tip = band + axis / np.linalg.norm(axis) * MARKER_L_MM
    ac.measure.px = (band_px.tolist(), line[1].tolist())
    ok = np.linalg.norm(tip - expect) <= within
    eyes.snapshot(f"{eyes.n:03d}_{label}" + ("" if ok else "_miss"), tip if ok else expect, ok)
    return tip if ok else None


def sane(J: np.ndarray, what: str) -> np.ndarray:
    """J with any column further than J_TOL from J_PRIOR's replaced by the prior's."""
    bad = np.linalg.norm(J - J_PRIOR, axis=0) > J_TOL
    if bad.any():
        print(f"  {what}: joint(s) {[J3[i] for i in np.nonzero(bad)[0]]} off the prior, the prior's taken", flush=True)
    return np.where(bad, J_PRIOR, J)


def find_tip(eyes, label: str) -> np.ndarray | None:
    """The tip anywhere over the plate: every pair of needle ends meeting within MISS_MM, by how well they meet, the
    first the teal bands confirm; else the first over 25 mm up (there the band blurs over the wood, but the ends meet
    well). Lowest-in-both took the robot's wheel, then a changed patch by the plate's rim (live D12, D11)."""
    import cv2
    import arm_calibrate as ac
    import tip_triangulate as tt
    eyes.n += 1
    sf, tf = eyes.side.frame(), eyes.top.frame()
    cv2.imwrite(f"evidence/tip/well2_{eyes.tag}_{eyes.n:03d}_{label}.raw.side.jpg", sf)
    cv2.imwrite(f"evidence/tip/well2_{eyes.tag}_{eyes.n:03d}_{label}.raw.top.jpg", tf)
    centre = (pr.well("A1") + pr.well("H12")) / 2 - [0, 0, 60.0]
    top_rim = min(pr.well(w)[2] for w in ("A1", "A12", "H1", "H12"))
    cands = [tt.meet(tt.ray(t, *eyes.urig), tt.ray(s_, *eyes.srig))
             for s_ in wr.needle_ends(sf, eyes.bg_side) for t in wr.needle_ends(tf, ac.measure.top_bg)]
    cands = sorted((c for c in cands if c[1] <= wr.MISS_MM and np.linalg.norm(c[0] - centre) <= 100.0
                    and c[0][2] < top_rim - 3.0), key=lambda c: c[1])
    for p, _ in cands[:6]:
        got = marker_tip(eyes, p, 6.0, f"{label}_band")
        if got is not None:
            return got
    return None


def full(q: dict) -> dict:
    """All four arm joints as whole pulses."""
    return {k: int(round(q[k])) for k in (1, 2, 3, 4)}


class Rig:
    def __init__(self, servos, eyes, q3: dict):
        self.servos, self.eyes, self.q = servos, eyes, dict(q3)
        self.m = None                 # tip as last measured at self.q
        self.J = None                 # 3 x 3, mm per pulse of joints 1-3
        self.local: list = []         # (joints, tip) measured over the current well
        self.bg_at = None
        self.seen = False             # self.m measured (not a lift's prediction)

    def save(self) -> None:
        if isinstance(self.servos, wr.Servos):
            STATE.write_text(json.dumps({str(k): v for k, v in full(self.q).items()}))

    def go(self, q3: dict, floor_mm: float | None = None, rim_z: float | None = None, target: str | None = None,
           approach: bool = True) -> None:
        """To q3 (wrist following), ending from AWAY above; with floor_mm, every piece's J-predicted tip checked."""
        q3 = {k: int(round(v)) for k, v in q3.items()}
        legs = ([{**q3, **{k: q3[k] + AWAY for k in J3}}] if approach else []) + [q3]
        if floor_mm is not None:
            for leg in legs:
                for f in np.linspace(0.0, 1.0, 6)[1:]:
                    d = np.array([self.q[k] + (leg[k] - self.q[k]) * f - self.q[k] for k in J3])
                    p = self.m + self.J @ d
                    if rim_z - p[2] < floor_mm - 0.5:
                        raise RuntimeError(f"refusing {leg}: tip predicted {rim_z - p[2]:.1f} mm over the rim")
                    if rim_z - p[2] < 0 and (target is None or np.linalg.norm(p[:2] - pr.well(target)[:2]) > ENTRY_XY_MM):
                        raise RuntimeError(f"refusing {leg}: under the rim away from the target")
        for leg in legs:
            assert all(0 <= v <= 1000 for v in full(leg).values()), full(leg)
            wr.jgo(self.servos, full(leg), full(self.q))
            self.q = dict(leg)
        self.save()

    def background(self) -> None:
        if self.bg_at is not None and self.servos.now() - self.bg_at < BG_MAX_S:
            return
        # never PARK with this mount: folding up and back hits the needle (user, 2026-10-31)
        back = dict(self.q)
        wr.jgo(self.servos, full(BG_POSE), full(self.q))
        self.eyes.background()
        self.bg_at = self.servos.now()
        wr.jgo(self.servos, full({**back, **{k: back[k] + AWAY for k in J3}}), full(BG_POSE))
        wr.jgo(self.servos, full(back), full({**back, **{k: back[k] + AWAY for k in J3}}))

    def look(self, expect, within, label):
        for _ in range(2):
            got = (self.eyes.tip(np.asarray(expect, float), within, label) if isinstance(self.eyes, Fake)
                   else marker_tip(self.eyes, np.asarray(expect, float), within, label))
            if got is not None:
                return got
        return None

    def probe(self, rim_z: float, label: str) -> None:
        """J by +-PROBE pulses on each of joints 1-3 (from above each time, back after each), central differences."""
        m0, q0 = self.m.copy(), dict(self.q)
        cols = []
        for k in J3:
            for _ in range(2):  # a lost measurement repeats the joint's probe once
                got = []
                for sg in (1, -1):
                    self.go({**q0, k: q0[k] + sg * PROBE})
                    got.append(self.look(m0, 25.0, f"{label}_probe{k}{'+' if sg > 0 else '-'}"))
                self.go(q0)
                if got[0] is not None and got[1] is not None:
                    break
            else:
                raise RuntimeError(f"probe of joint {k}: tip not measured")
            cols.append((got[0] - got[1]) / (2 * PROBE))
        self.J = sane(np.column_stack(cols), "probe")
        again = self.look(m0, 6.0, f"{label}_probe0")
        self.m = m0 if again is None else again
        self.local = [(np.array([q0[k] for k in J3], float), self.m)]
        print("  probed, mm per pulse: " + ", ".join(f"joint {k} {np.round(self.J[:, i], 2).tolist()}"
                                                  for i, k in enumerate(J3)), flush=True)

    def refit(self) -> None:
        """J by least squares over the measurements at this well, once they span all three joints."""
        if len(self.local) < 5:
            return
        qs = np.array([q for q, _ in self.local])
        if np.linalg.matrix_rank(qs - qs.mean(0), tol=1.0) < 3:
            return
        sol = np.linalg.lstsq(np.column_stack([qs, np.ones(len(qs))]), np.array([m for _, m in self.local]), rcond=None)[0]
        self.J = sane(sol[:3].T, "refit")

    def step_to(self, goal: np.ndarray, gate: float, floor: float, rim_z: float, label: str) -> bool:
        for it in range(STEPS):
            err = goal - self.m
            print(f"  {label} {it}: xy {np.linalg.norm(err[:2]):.2f} z {err[2]:+.2f} mm "
                  f"({rim_z - self.m[2]:.1f} mm up)", flush=True)
            if np.linalg.norm(err[:2]) <= gate and abs(err[2]) <= Z_OK_MM:
                return True
            if np.linalg.norm(err[:2]) > 2 * gate:  # xy first: down-steps' sideways error outgrew the correction (D12, 4 -> 16 mm)
                err = np.r_[err[:2], 0.0]
            want = GAIN * err if np.linalg.norm(err) > 1.5 else err  # the last mm whole: half of it is under a pulse
            want = want if np.linalg.norm(want) <= STEP_MM else want * STEP_MM / np.linalg.norm(want)
            v = np.round(np.linalg.lstsq(self.J, want, rcond=None)[0])
            if not v.any():
                i = int(np.argmax(np.abs(self.J.T @ err)))
                v[i] = np.sign(self.J[:, i] @ err)
            for _ in range(4):  # refused (the tip predicted under the floor on the way): a smaller step
                try:
                    self.go({**self.q, **{k: self.q[k] + v[i] for i, k in enumerate(J3)}}, floor, rim_z)
                    break
                except RuntimeError as e:
                    if not str(e).startswith("refusing") or not np.round(v / 2).any():
                        raise
                    print(f"  {e}: half the step", flush=True)
                    v = np.round(v / 2)
            pred = self.m + self.J @ v
            got = self.look(pred, LOOK_MM, f"{label}_{it}")
            if got is None:
                got = self.look(goal, 25.0, f"{label}_{it}w")
            if got is None:
                return False
            self.m, self.seen = got, True
            self.local.append((np.array([self.q[k] for k in J3], float), got))
            self.refit()
        return False

    def lift(self, rim_z: float, to_mm: float, label: str) -> None:
        """Up to to_mm over the rim by J in steps; a lost tip goes on by the prediction (up is the safe way)."""
        for it in range(12):
            if rim_z - self.m[2] >= to_mm - 2.0:
                return
            want = np.r_[0.0, 0.0, -min(STEP_MM * 2, to_mm - (rim_z - self.m[2]))]
            v = np.round(np.linalg.lstsq(self.J, want, rcond=None)[0])
            pred = self.m + self.J @ v
            self.go({**self.q, **{k: self.q[k] + v[i] for i, k in enumerate(J3)}}, rim_z - self.m[2] - 5.0, rim_z)
            got = self.look(pred, 12.0, f"{label}_lift{it}")
            self.m, self.seen = (pred, False) if got is None else (got, True)


def visit(rig: Rig, w: str) -> dict:
    rim = pr.well(w)
    rz = float(rim[2])
    if isinstance(rig.eyes, Fake):
        rig.eyes.target = w
    rig.background()
    if rig.m is None:  # coming from elsewhere (the background pose): to over the plate first
        rig.go(OVER_PLATE)
    # from the last well's measured tip; else searched anywhere over the plate (the robot base moves between sessions,
    # 80 mm after a break; a lift that lost the tip went on by prediction and left it 30 mm off, live D12)
    got = rig.look(rig.m, 6.0 if rig.seen else 15.0, f"{w}_start") if rig.m is not None else None
    if isinstance(rig.eyes, Fake):
        got = rig.eyes.free_tip(f"{w}_free")
    elif got is None:
        got = find_tip(rig.eyes, f"{w}_find")
    rig.m, rig.seen = got, got is not None
    if rig.m is None:
        raise RuntimeError("tip not measured")
    rig.probe(rz, w)  # every well: F6's Jacobian mispredicted D6, two rows over, by ~8-10 mm (live)
    rig.local = [(np.array([rig.q[k] for k in J3], float), rig.m)]
    rig.lift(rz, HEIGHTS_MM[0], w)  # never cross over at less than the first height
    res = {"steps": []}
    for h, gate in zip(HEIGHTS_MM, GATES_MM):
        floor = max(h - 5.0, 0.5)  # 5 under: each move's +8 on the wrist dips the tip ~3.5 mm on the way (live E6)
        if not rig.step_to(rim - [0, 0, h], gate, floor, rz, f"{w}_{h:.0f}"):
            rig.lift(rz, HEIGHTS_MM[0], w)
            return {"went_in": False, "reason": f"not within {gate} mm at {h:.0f} mm"}
        res["steps"].append({"h": h, "tip": rig.m.tolist(), "q": full(rig.q)})
    gate_xy = float(np.linalg.norm((rim - rig.m)[:2]))
    hover_q, hover_m = dict(rig.q), rig.m.copy()
    v_in = np.round(np.linalg.lstsq(rig.J, rim + [0, 0, DEPTH_MM] - rig.m, rcond=None)[0])
    rig.go({**rig.q, **{k: rig.q[k] + v_in[i] for i, k in enumerate(J3)}}, -DEPTH_MM - 1.0, rz, target=w,
           approach=False)
    seen = rig.eyes.which_well(rz, w)
    photo = rig.eyes.photos(f"servo_{w}")
    rig.go(hover_q, -DEPTH_MM - 1.0, rz, target=w, approach=False)
    rig.m = hover_m
    rig.lift(rz, HEIGHTS_MM[0], w)
    print(f"{w}: in, {gate_xy:.2f} mm off the centre 2 mm over the rim; overhead: {seen}", flush=True)
    return {**res, "went_in": True, "gate_xy_mm": gate_xy, "seen_in": None if seen is None else seen[0], "photo": photo}


class Fake:
    """--dry: a linear arm (slopes like the old probe) with 1.5 mm of backlash per joint direction change, cameras with
    a 1 mm bias and 0.3 mm noise, one measurement in eight lost."""
    JT = np.array([[-0.32, -0.06, 0.45], [-0.87, 0.03, -0.12], [0.0, -0.55, -0.15]])  # pan, shoulder, wrist

    def __init__(self, q):
        self.q, self.n, self.rng, self.target, self.last = dict(q), 0, np.random.default_rng(2), ORDER[0], None
        self.anchor = None

    def true(self):
        q3 = np.array([self.q[k] for k in J3], float)
        if self.anchor is None:  # the first measured pose sits 70 mm over F6, as live
            self.anchor = (q3.copy(), pr.well("F6") - [0, 0, 70.0])
        return self.anchor[1] + self.JT @ (q3 - self.anchor[0])

    def read(self):
        return {**self.q, 5: 497, 10: 664}

    def send(self, q, ms):
        assert all(0 <= v <= 1000 for v in q.values()) and all(abs(q[k] - self.q[k]) <= wr.STEP for k in q), (self.q, q)
        self.q.update(q)
        self.n += 1

    def dwell(self, s):
        pass

    def now(self):
        return float(self.n)

    def background(self):
        pass

    def tip(self, expect, within, label):
        if self.rng.random() < 0.125:
            return None
        t = self.true() + [1.0, -0.5, 0.0] + self.rng.normal(0, 0.3, 3)
        return t if np.linalg.norm(t - expect) <= within else None

    def free_tip(self, label):
        return self.true() + self.rng.normal(0, 0.3, 3)

    def which_well(self, rz, target):
        t = self.true()
        if rz - t[2] > -1.0:
            return None
        w = min((f"{r}{c}" for r in "ABCDEFGH" for c in range(1, 13)), key=lambda x: np.linalg.norm(pr.well(x)[:2] - t[:2]))
        return w, float(np.linalg.norm(pr.well(w)[:2] - t[:2]))

    def photos(self, stem):
        return f"(dry) {stem}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--wells", nargs="*", default=ORDER)
    args = ap.parse_args()
    if args.dry:
        start = {1: 595, 2: 358, 3: 209, 4: 590}
        servos = eyes = Fake(full(start))
    else:
        saved = {int(k): v for k, v in json.loads(STATE.read_text()).items()}
        start = {k: saved[k] for k in (1, 2, 3, 4)}
        servos, eyes = wr.Servos(), wr.Eyes(time.strftime("%Y%m%dT%H%M%S"))
        r = servos.read()
        off = {k: r[k] - full(start)[k] for k in (1, 2, 3, 4) if abs(r[k] - full(start)[k]) > 25}  # backlash + sag ~16
        if off:
            raise RuntimeError(f"arm not where {STATE} says: {off}")
        servos.td._sent.update(full(start))
    rig = Rig(servos, eyes, start)
    results = json.loads(OUT.read_text()) if OUT.exists() and not args.dry else {}
    try:
        for w in args.wells:
            try:
                results[w] = {"well": w, **visit(rig, w)}
            except RuntimeError as e:
                results[w] = {"well": w, "went_in": False, "reason": str(e)}
                print(f"{w}: stopped ({e})", flush=True)
                if rig.m is None:
                    break
                rig.lift(float(pr.well(w)[2]), HEIGHTS_MM[0], w)
            if not results[w]["went_in"]:
                print(f"{w}: not in ({results[w]['reason']})", flush=True)
            if not args.dry:
                OUT.write_text(json.dumps(results, indent=1))
    finally:
        rig.save()
    got = [w for w in args.wells if results.get(w, {}).get("went_in")]
    print(f"{'dry run, ' + str(servos.n) + ' commands checked: ' if args.dry else ''}in {len(got)}/{len(args.wells)}: {got}")


if __name__ == "__main__":
    main()
