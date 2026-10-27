#!/usr/bin/env python3
"""Offline check of aim_touch.touch(), no robot: servos with deadband and backlash, a true Jacobian off
from J0 by up to --jerr, ink dots 14 x 20 mm apart around the target, the target hidden below 2 mm.
Prints contact rate and landing miss; asserts the ideal plant (no deadband, no backlash, exact J0) lands.
Re-run with the Step 1 numbers (docs/needle_on_dot.md) before changing DEADBAND/BACKLASH on the arm.

    python tools/aim_sim.py [--db 2] [--bl 5] [--jerr 0.1] [--runs 30]
"""
from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import aim_touch as at  # noqa: E402

JOINTS = (1, 3, 4)
START = {1: 497, 2: 423, 3: 235, 4: 269, 5: 502, 10: 662}


class Arm:
    """State (dot - landing point x, y, height) = start + J_true @ output pulses moved."""

    def __init__(self, rng, db: int, bl: int, jscale: np.ndarray, start_err):
        self.rng, self.db, self.bl = rng, db, bl
        self.J = at.J0 * jscale
        self.cmd = dict(START)
        self.motor = {j: float(START[j]) for j in JOINTS}
        self.out, self.base = dict(self.motor), dict(self.motor)
        self.x0 = np.array([*start_err, at.SAFE_MM + 4])
        self.shift = np.zeros(3)  # base strafe so far
        self.strafe_gain = rng.choice([-1, 1]) * rng.uniform(0.1, 0.35)  # 0.05 m/s x 0.3 s moved 3.4 of 15 mm
        a = rng.uniform(0, 2 * np.pi)
        self.ug_rot = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) * rng.choice([1, -1], 2)
        self.ug_slope = rng.normal(0, 0.3, 2)

    def ugreen(self, h, x_arm=None):
        """Overhead view: target - tip in its own sheet axes (rotated), plus the landing-point offset that
        grows with height, plus noise."""
        x = self.x()
        return self.ug_rot @ x[:2] + self.ug_slope * x[2] + self.rng.normal(0, 0.15, 2)

    def strafe(self, vx, vy, seconds):
        self.shift += [self.strafe_gain * vy * seconds * 1000 * self.rng.uniform(0.6, 1.4), self.rng.normal(0, 0.2), 0]

    def move(self, cmd, ms=0, settle=0):
        self.cmd.update(cmd)
        for j in JOINTS:
            if abs(cmd[j] - self.motor[j]) >= self.db:  # deadband: a small change does nothing
                self.motor[j] = float(cmd[j])
            m, o = self.motor[j], self.out[j]  # backlash: play of bl pulses between motor and output
            if m > o + self.bl / 2:
                self.out[j] = m - self.bl / 2
            elif m < o - self.bl / 2:
                self.out[j] = m + self.bl / 2
        return dict(self.cmd)

    def x(self) -> np.ndarray:
        return self.x0 + self.shift + self.J @ np.array([self.out[j] - self.base[j] for j in JOINTS])

    def sense(self, expects=(), windows=None):
        x = self.x()
        grid = np.array([[i * 14.0, k * 20.0] for i in range(-2, 3) for k in range(-2, 3)])
        rel = x[:2] + grid + self.rng.normal(0, 0.1, grid.shape)
        if x[2] < 2.0:  # hidden under the needle
            rel = rel[np.linalg.norm(grid, axis=1) > 0]
        i = at.lock(rel, expects, windows) if len(expects) else 12
        goal = rel[i] if i is not None else (expects[0] if len(expects) else None)
        return {"goal": goal, "foot": np.zeros(2), "tip": np.array([0, 0, -x[2]]), "img": None,
                "back": 10.0 if x[2] <= 0 else self.rng.normal(0, 0.8), "seen": i is not None}


def run(seed: int, db: int, bl: int, jerr: float, strafe: bool = True, ugreen: bool = False, far: float = 6):
    rng = np.random.default_rng(seed)
    err, jscale = rng.uniform(-far, far, 2), rng.uniform(1 - jerr, 1 + jerr, (3, 3))
    arm = Arm(rng, db, bl, jscale, err)
    at.move, at.commanded, at.read = arm.move, lambda *a: dict(arm.cmd), lambda: dict(arm.cmd)
    state = lambda r: np.array([*(r["goal"] - r["foot"]), -r["tip"][2]])  # noqa: E731
    try:
        reason, _ = at.touch(arm.sense, state, lambda **row: None, at.J0.copy(), JOINTS, arm.sense()["goal"], 150, False,
                              arm.strafe if strafe else None, arm.ugreen if ugreen else None)
    except RuntimeError as error:
        return f"lost: {error}"[:40], None
    return reason, arm.x()


def batch(db: int, bl: int, jerr: float, runs: int, strafe: bool = True, ugreen: bool = False, far: float = 6) -> list[float]:
    at.DEADBAND, at.BACKLASH = max(db, 1), bl  # the controller told the plant's true numbers
    why, misses = collections.Counter(), []
    for k in range(runs):
        reason, x = run(k, db, bl, jerr, strafe, ugreen, far)
        why[reason] += 1
        if reason == "contact":
            misses.append(float(np.hypot(*x[:2])))
    m = np.array(misses)
    print(f"{'ugreen' if ugreen else 'strafe' if strafe else 'pan only'}{f' far {far:g}' if far != 6 else ''} db {db} bl {bl} J +-{jerr:.0%}: contact {len(m)}/{runs}, miss median {np.median(m) if len(m) else np.nan:.2f} mm,"
          f" <= 1 mm {int((m <= 1).sum())}, <= 1.5 mm {int((m <= 1.5).sum())} | {dict(why)}")
    return misses


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--db", type=int, default=2)
    ap.add_argument("--bl", type=int, default=5)
    ap.add_argument("--jerr", type=float, default=0.1)
    ap.add_argument("--runs", type=int, default=30)
    args = ap.parse_args()
    ideal = batch(1, 0, 0.0, 10)
    # ideal plant: every run lands; the miss floor is half a pan pulse (~1.5 mm) plus the blind descent
    assert len(ideal) == 10 and np.median(ideal) < 2.5, "ideal plant must land every run, near the pan floor"
    batch(args.db, args.bl, args.jerr, args.runs, strafe=False)
    batch(args.db, args.bl, args.jerr, args.runs)
    batch(args.db, args.bl, args.jerr, args.runs, ugreen=True)
    batch(args.db, args.bl, args.jerr, args.runs, ugreen=True, far=40)
