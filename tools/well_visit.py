#!/usr/bin/env python3
"""Needle into wells from poses taught with the arm powered (tools/pose_gui.py, 2026-10-31), the cameras checking.

Limp hand-taught poses replayed one to two wells off (the read-back under a hand is not where the powered arm holds:
H1 pan 612 limp vs 594 powered); powered ones replay where they were taught, if approached as the window does
(every changed joint from AWAY pulses above). Per well:

1. hover pose: taught, or interpolated over the taught wells (bilinear in column / row per joint, exact at each),
   plus the joint correction the last interpolated well ended with; lifted (~5 cm up) -> AWAY above -> hover;
2. the tip measured by both cameras at the hover; the in-well tip predicted as that plus a measured Jacobian (probed at
   the first well, +-12 pulses ~5 cm up) times the hover-to-in step. At a taught well the prediction must be the
   centre, so its miss is the cameras' bias near the plate (averaged, subtracted for the others). An interpolated well
   whose prediction misses the centre by more than GATE_MM is corrected sideways at constant height, each step
   measured;
3. down by the taught (or interpolated) hover-to-in joint step plus that correction: a real, taught move, not a
   Jacobian guess; photos, the overhead's word on the well; back up the same way.

    <venv python> tools/well_visit.py --dry           # simulated arm and cameras, every command checked
    <venv python> tools/well_visit.py                 # robot, ORDER
    <venv python> tools/well_visit.py --wells H1 A1   # robot, these wells
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plate_rig as pr  # noqa: E402
import well_reach as wr  # noqa: E402

ARM = (1, 2, 3, 4)
POWERED = Path("evidence/tip/powered_poses.json")  # pose_gui saves: name -> {"cmd": {servo: pulses}, "read": ...}
# well -> (pose name in the well, pose name hovering over it), as saved 2026-10-31
TAUGHT_NAMES = {"H1": ("H1 inside", "H1 hover close"), "A1": ("A1 down", "A1 Hover"),
                "D7": ("D7 dow2n", "D7 Hover"), "A12": ("A12 down", "A12 hover")}
# taught first (they give the cameras' bias and the Jacobian), then interpolated ones near and between them
ORDER = ["D7", "H1", "A1", "A12", "G1", "H2", "B1", "C1", "D1", "E1", "F1", "A2", "B2", "H6", "D6", "E7",
         "C7", "B7", "A7", "D9", "C11", "B12"]
LIFT = {2: 40, 3: 40}  # pulses over a hover pose: ~5 cm up (shoulder and elbow both raise the tip)
DEEPER_SHOULDER = 4    # extra shoulder pulses down for an interpolated well
HOVER_SHOULDER = 15    # shoulder pulses over the in-well pose: ~1 cm up; the entry is this straight back down
AWAY = 8               # every joint ends a move into a hover pose coming from this many pulses above (as pose_gui)
GATE_MM = 1.5          # an interpolated well goes in with the bias-corrected tip this close to the centre (xy)
GAIN = 0.6             # of the measured error per correction step: at 1.0 the H1 servo swung 2-5 mm either side
STEP_MM = 4.0          # largest correction per step
STEPS = 8
CARRY_MAX = 20         # pulses: the largest joint correction carried to the next well
OUT = Path("evidence/tip/well_visit.json")
# the taught poses need shoulder down to ~250 (A12) and wrist up to 690: well_reach's model envelope stopped short
wr.ENVELOPE = {1: (470, 740), 2: (200, 620), 3: (0, 450), 4: (200, 720)}


def taught() -> dict[str, tuple[tuple, tuple]]:
    saved = json.loads(POWERED.read_text())
    return {w: tuple(tuple(saved[n]["cmd"][str(k)] for k in ARM) for n in names) for w, names in TAUGHT_NAMES.items()}


def colrow(w: str) -> tuple[int, int]:
    return int(w[1:]) - 1, "ABCDEFGH".index(w[0])


def features(w: str) -> np.ndarray:
    c, r = colrow(w)
    return np.array([1.0, c, r, c * r])


def pose(w: str) -> tuple[dict, dict, bool]:
    """(in-well pose, hover pose, taught?) for well `w`. In-well: taught, or interpolated (exact at the taught wells).
    Hover: the in-well pose with the shoulder HOVER_SHOULDER higher, for every well: the in-well tip is predicted from
    the hover by the Jacobian times that step, and over a taught hover-to-in step (A1: 120 pulses) the Jacobian's
    error alone was ~5 mm (dry run)."""
    t = taught()
    if w in t:
        qi = dict(zip(ARM, t[w][0]))
    else:
        x = np.array([features(v) for v in t])
        coef = np.linalg.lstsq(x, np.array([p[0] for p in t.values()], float), rcond=None)[0]
        q = np.round(features(w) @ coef).astype(int)
        q[2] = max(q[2], 0)  # the elbow's commandable end
        qi = dict(zip(ARM, q.tolist()))
    return qi, {**qi, 2: qi[2] + HOVER_SHOULDER}, w in t


def lifted(q: dict) -> dict:
    return {k: q[k] + LIFT.get(k, 0) for k in ARM}


def above(q: dict) -> dict:
    return {k: min(q[k] + AWAY, 1000) for k in ARM}


def solve(arm: wr.Arm, want: np.ndarray) -> np.ndarray:
    """Pulses of joints 1-4 moving the tip by `want` (min-norm over the measured 3 x 4 Jacobian); without the elbow when
    that would take it under 0: by H1 the elbow sits at its end."""
    v = np.linalg.lstsq(arm.J, want, rcond=None)[0]
    if arm.q[3] + v[2] < 0:
        sub = np.linalg.lstsq(arm.J[:, [0, 1, 3]], want, rcond=None)[0]
        v = np.array([sub[0], sub[1], 0.0, sub[2]])
    return v


def visit(arm: wr.Arm, eyes, w: str, carry: np.ndarray, bias: list) -> tuple[dict, np.ndarray]:
    """One well (steps 1-3 of the module doc). Returns (result, the joint correction it ended with); taught wells
    append the cameras' bias to `bias`."""
    qi, qh, is_taught = pose(w)
    rim = pr.well(w)
    rz = float(rim[2])
    if isinstance(eyes, Fake):
        eyes.well = w
    start = dict(qh) if is_taught else {k: qh[k] + int(carry[i]) for i, k in enumerate(ARM)}
    start[3] = max(start[3], 0)
    arm.q = wr.jgo(arm.servos, lifted(start), arm.q)
    arm.q = wr.jgo(arm.servos, above(start), arm.q)
    arm.q = wr.jgo(arm.servos, start, arm.q)
    res = {"taught": is_taught, "hover": start, "steps": []}
    arm.m = wr.look(eyes, rim - [0, 0, 6.0], 30.0, f"{w}_hover")
    if arm.m is None and is_taught:  # a taught well goes in anyway: it was put there by hand, powered
        res["reason_no_bias"] = "tip not measured at the hover"
    elif arm.m is None:
        return {**res, "went_in": False, "reason": "tip not measured at the hover"}, carry
    if arm.m is not None and arm.J is None:  # the Jacobian first: the bias needs it
        hq = dict(arm.q)
        arm.q = wr.jgo(arm.servos, lifted(arm.q), arm.q)  # probe ~5 cm up, clear of the plate
        arm.m = eyes.free_tip(f"{w}_lifted")
        if arm.m is None:
            return {**res, "went_in": False, "reason": "tip not measured lifted (for the probe)"}, carry
        wr.probe(arm, eyes, rz, w)
        arm.q = wr.jgo(arm.servos, above(hq), arm.q)
        arm.q = wr.jgo(arm.servos, hq, arm.q)
        arm.m = wr.look(eyes, rim - [0, 0, 6.0], 30.0, f"{w}_hover2")
        if arm.m is None and not is_taught:
            return {**res, "went_in": False, "reason": "tip not measured back at the hover"}, carry
    # where the tip will be in the well: this hover reading plus the Jacobian times the taught hover-to-in step. A hover
    # pose need not be straight over the centre (A1's step is shoulder -70, wrist +50), so the bias is taken there
    step_in = np.array([qi[k] - qh[k] for k in ARM], float)
    b = np.mean(bias, axis=0) if bias else np.zeros(2)
    if arm.m is not None and arm.J is not None:
        res["hover_tip"], res["hover_up_mm"] = arm.m.tolist(), float(rz - arm.m[2])
        if is_taught:
            bias.append((arm.m + arm.J @ step_in - rim)[:2])
            print(f"  {w} taught: cameras + Jacobian put the in-well tip {np.round(bias[-1], 1).tolist()} mm off the "
                  f"centre (bias), hover {rz - arm.m[2]:.1f} mm up", flush=True)
    if not is_taught:
        ok = False
        for it in range(STEPS + 1):
            err = np.r_[rim[:2] - (arm.m + arm.J @ step_in)[:2] + b, 0.0]  # sideways only, at the hover's height
            res["steps"].append({"q": dict(arm.q), "tip": arm.m.tolist(), "err_xy": err[:2].tolist()})
            print(f"  {w} {it}: in-well tip predicted {np.linalg.norm(err):.2f} mm off the centre, hover "
                  f"{rz - arm.m[2]:.1f} mm up", flush=True)
            if np.linalg.norm(err) <= GATE_MM:
                ok = True
                break
            if it == STEPS:
                break
            want = GAIN * err
            want = want if np.linalg.norm(want) <= STEP_MM else want * STEP_MM / np.linalg.norm(want)
            v = np.round(solve(arm, want))
            if not v.any():
                i = int(np.argmax(np.abs(arm.J.T @ err)))
                v[i] = np.sign(arm.J[:, i] @ err)
            pred = wr.step_joints(arm, v, min(3.0, rz - arm.m[2] - 2.0), rz)
            got = wr.look(eyes, pred, 10.0, f"{w}_{it}")
            got = wr.look(eyes, rim - [0, 0, 6.0], 20.0, f"{w}_{it}w") if got is None else got
            if got is None:
                break
            arm.m = got
        if not ok:
            arm.q = wr.jgo(arm.servos, lifted(arm.q), arm.q)
            return {**res, "went_in": False, "reason": f"not within {GATE_MM} mm of the centre"}, carry
        carry = np.clip([arm.q[k] - qh[k] for k in ARM], -CARRY_MAX, CARRY_MAX)
    hover = dict(arm.q)
    down = {k: hover[k] + qi[k] - qh[k] for k in ARM}  # the hover-to-in step, from where the hover ended
    if not is_taught:  # interpolated in-well poses ended ~0.4 mm under the rim in the dry run: ~3 mm deeper
        down[2] -= DEEPER_SHOULDER
    down[3] = max(down[3], 0)
    arm.q = wr.jgo(arm.servos, down, arm.q)
    seen = eyes.which_well(rz, w)
    photo = eyes.photos(f"visit_{w}")
    arm.q = wr.jgo(arm.servos, hover, arm.q)  # straight back out
    arm.q = wr.jgo(arm.servos, lifted(arm.q), arm.q)
    print(f"{w}: in ({'taught' if is_taught else 'interpolated'}), overhead sees the needle in {seen}", flush=True)
    return {**res, "went_in": True, "down": down, "seen_in": None if seen is None else seen[0],
            "seen_off_mm": None if seen is None else seen[1], "photo": photo}, carry


class Fake:
    """--dry: servos and cameras. The true tip is linear in the joints near the visited well, exact at its taught pose
    (interpolated wells' true poses sit 8 / -10 / 6 pulses off), the cameras read BIAS_TRUE off plus 0.4 mm noise."""
    JT = np.array([[-0.33, 0.53, 0.87, 0.60], [-0.65, -0.25, -0.40, -0.30], [0.0, -0.70, -0.73, -0.50]])  # joints 1-4
    BIAS_TRUE = np.array([1.5, -1.0, 0.0])

    def __init__(self, q: dict):
        self.q, self.n, self.rng, self.parking, self.bg_at = dict(q), 0, np.random.default_rng(1), False, None
        self.well = ORDER[0]

    def true(self) -> np.ndarray:
        qi, _, is_taught = pose(self.well)
        qt = np.array([qi[k] for k in ARM], float) + (0 if is_taught else np.array([8, -10, 6, 0]))
        return pr.well(self.well) + [0, 0, 3.0] + self.JT @ (np.array([self.q[k] for k in ARM]) - qt)

    def read(self):
        return {**self.q, 5: 497, 10: 657}

    def send(self, q, ms):
        assert all(0 <= q[k] <= 1000 for k in ARM) and all(abs(q[k] - self.q[k]) <= wr.STEP for k in ARM), (self.q, q)
        self.q, self.n = dict(q), self.n + 1

    def dwell(self, s):
        pass

    def now(self):
        return float(self.n)

    def background(self):
        pass

    def tip(self, expect, within, label):
        t = self.true() + self.BIAS_TRUE + self.rng.normal(0, 0.4, 3)
        return t if np.linalg.norm(t - expect) <= within else None

    def free_tip(self, label):
        return self.true() + self.BIAS_TRUE + self.rng.normal(0, 0.4, 3)

    def which_well(self, rz, target):
        t = self.true()
        w = min((f"{r}{c}" for r in "ABCDEFGH" for c in range(1, 13)), key=lambda x: np.linalg.norm(pr.well(x)[:2] - t[:2]))
        return (w, float(np.linalg.norm(pr.well(w)[:2] - t[:2]))) if rz - t[2] < -1.0 else None

    def photos(self, stem):
        return f"(dry) {stem}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--wells", nargs="*", default=ORDER)
    args = ap.parse_args()
    if args.dry:
        servos = eyes = Fake(wr.PARK)
    else:
        servos = wr.Servos()
        eyes = wr.Eyes(time.strftime("%Y%m%dT%H%M%S"))
    now = {k: min(max(servos.read()[k], 0), 1000) for k in ARM}
    if not args.dry:
        servos.td._sent.update(now)
    arm = wr.Arm(wr.model(), servos, now, np.zeros(3))
    arm.J, arm.joints = None, ARM
    arm.q = wr.jgo(servos, wr.PARK, now)  # background with the arm out of both views
    eyes.background()
    results = json.loads(OUT.read_text()) if OUT.exists() and not args.dry else {}
    carry, bias = np.zeros(4), []
    try:
        for w in args.wells:
            try:
                results[w], carry = visit(arm, eyes, w, carry, bias)
            except RuntimeError as e:  # a refused move ends this well, up and on
                results[w] = {"went_in": False, "reason": f"refused: {e}"}
                arm.q = wr.jgo(servos, lifted(arm.q), arm.q)
            if not results[w]["went_in"]:
                print(f"{w}: not in ({results[w]['reason']})", flush=True)
            if not args.dry:
                OUT.write_text(json.dumps(results, indent=1, default=lambda o: np.asarray(o).tolist()))
    finally:
        arm.q = wr.jgo(servos, lifted(arm.q), arm.q) if arm.q[2] < 600 else arm.q
        wr.jgo(servos, wr.PARK, arm.q)
    got = [w for w in args.wells if results.get(w, {}).get("went_in")]
    print(f"cameras' bias near the plate: {np.round(np.mean(bias, axis=0), 2).tolist() if bias else None} mm "
          f"(from {len(bias)} taught wells)")
    print(f"{'dry run, ' + str(servos.n) + ' commands checked: ' if args.dry else ''}in {len(got)}/{len(args.wells)}: {got}")


if __name__ == "__main__":
    main()
