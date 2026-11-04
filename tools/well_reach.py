#!/usr/bin/env python3
"""Needle tip into the wells of the 96-well plate: measured placing over each rim, a short entry, the overhead's word
on which well holds the needle, photos, evidence/tip/well_reach.json.

Why not arm_calibrate `well` (2026-10-31, 0 of 96, docs/implementation_plate_well_reach_2026-10-31_FAILURE.md): it
jumped to the plate in one move from a model fitted near START (5-10 mm off out there), wiggled 20 pulses on every
joint at full reach, and never had a tip measurement checked over the plate. Here:

- One motion gate, Arm.go: joints 1-4 only (never roll 5 or the gripper), inside ENVELOPE, at most STEP pulses per
  joint per command at MS_PER_PULSE, and no waypoint whose predicted tip (model minus the measured correction) is under
  the floor the caller states: TRANSIT_MM over the plate for every sideways move, only the entry goes below the rim.
- Every move is a straight tip line cut into CART_MM pieces, so the tip path is calm and predictable.
- The model is never trusted far from a measurement: `walk` goes out from where the arm is in measured legs, carrying
  the measured correction, and refuses the plate without a measurement in the last WALK_FRESH_MM.
- `--dry` runs the same code on a simulated arm whose true tip is the model plus a smooth error, and checks every
  command (steps, envelope, true tip height, entry inside the well).

    <venv python> tools/well_reach.py --dry ring          # offline: every reachable well, outer ring first
    <venv python> tools/well_reach.py home                # robot: hold where it is, then slowly to START
    <venv python> tools/well_reach.py fit                 # robot: this grasp's tip and base from ~12 measured poses
    <venv python> tools/well_reach.py walk                # robot: tie to the cameras by START, then to TRANSIT_MM over H1
    <venv python> tools/well_reach.py well H1 A1          # robot: into these wells
    <venv python> tools/well_reach.py ring                # robot: every arm-reachable well, outer ring first
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_calibrate as ac  # noqa: E402
import coarse_aim as ca  # noqa: E402
import plate_rig as pr  # noqa: E402

ARM = (1, 2, 3, 4)
STEP = 6              # pulses per joint per command
MS_PER_PULSE = 25     # half the speed arm_calibrate.send allows (12): 6 pulses in 250 ms
MIN_MS = 250
DWELL_S = 1.2         # after the last piece of a move, before frames
CART_MM = 5.0         # tip path pieces
J3 = (1, 2, 3)        # the joints the measured Jacobian moves over the plate (the wrist holds)
PROBE = 12            # pulses per joint for the measured Jacobian (6 was mostly backlash)
SERVO_MAX_PULSES = 12  # largest joint step per servo move (a near-singular J asked for one moving the tip 30 mm)
SERVO_STEP_MM = 8.0   # largest tip step per servo move
SERVO_TRIES = 15      # servo moves per height (8 ran out at 15 mm over H1, live)
SERVO_LOOK_MM = 10.0  # a servo step's tip looked for this close to where the Jacobian put it
START_LOOK_MM = 40.0  # a well started with no measured tip looks this wide around its transit point (a static junk
#                       pair sat 50 mm off H1, 2026-10-31)
PITCH_STEP = 10       # needle-angle turn per piece (shoulder+elbow+wrist pulses)
BRANCH_JUMP = 30      # pulses on one joint for one piece: more is a solver branch change, refused
# whole pulses reach the plate on a ~0.4-0.5 mm lattice (best of 16 roundings, 2026-10-31): the placing floor there
ROUND_MM = 0.75
SMOOTH = 15           # pulses per joint per piece a well's planned path may need (well_pitch)
TRANSIT_MM = 35.0     # over the highest rim for every sideways move (dishes and plate are ~15-17 mm tall)
HEIGHTS_MM = (15.0, 6.0)  # measured placing over the rim; the last one is the gate (at 3 mm the tip pick took a well shadow, live H1)
GATE_MM = 1.0         # the user's accuracy gate, xy over the well centre at the last height
Z_OK_MM = 1.5
TRIES = 5             # placings per height (a lost measurement spends one)
PASS_MM = 3.0         # xy good enough to go lower at the heights above the gate
DEPTH_MM = 3.0        # below the rim (wells ~10 mm deep): a short entry from the gate pose, xy held
ENTRY_XY_MM = 2.5     # any waypoint below the rim must be this close to the target well's centre (wells ~5 mm wide)
EXPECT_MM = 8.0       # a measured tip farther than this from where it was sent is a junk pair
WALK_EXPECT_MM = 15.0  # walk legs, and the first measurement over each well (the model is extended as it goes)
WALK_LEG_MM = 15.0    # measured every this far on the walk: fitted by START only, the model was 13 mm further off per
#                       25 mm leg out there and an 8 mm gate threw the real tip away (2026-10-31)
WALK_FRESH_MM = 60.0  # the plate is entered only with a measurement this close behind
BG_MAX_S = 600.0      # the parked background is retaken when older than this (daylight drifts)
MISS_MM = 3.0         # overhead and side rays meet at least this close (by START ~1 mm; 2.5 mm at H1, 2026-10-31)
PITCHES = range(820, 1141, 10)  # shoulder+elbow+wrist sums tried; 1000 ~ vertical needle (near START only ~860-940 lift)
# joint limits for everything this tool sends: the full dry run's range with the live-fitted tip (walk from START, all
# 96 wells; pan 497-706, shoulder 332-617, elbow 37-244, wrist 256-542, 2026-10-31) plus ~20 pulses, shoulder at
# arm_calibrate's 620 ceiling
ENVELOPE = {1: (470, 740), 2: (300, 620), 3: (0, 450), 4: (200, 640)}  # elbow to 450: 300 pinned the solver (the
# first model's poses reached 511)
# ... widened (pan +15, elbow to 0-300, wrist 200-640) when learning moved the model: the wrist ran into 560 and the
# solver hopped branch (dry run, 2026-10-31)
FIT_MODEL = Path("evidence/tip/well2_model.json")  # arm_model's session with this grasp's tip, yaw and base (`fit`)
# after the power cycle the roll read 496 and the needle hung ~35 deg tilted, 80 mm from the stored grasp tip
# (2026-10-31): the tip and base are refitted from measured poses before any walk
PARK = {1: 505, 2: 900, 3: 0, 4: 500}  # arm_park.py: folded up and back, out of both cameras' views (power-on frames)
FIT_LIFT = 20  # shoulder pulses over START for the fit poses (~30 mm up: the tip hung 24 mm up at START)
FIT_DELTAS = [{}, {4: 10}, {4: -10}, {1: 20}, {1: 40}, {1: 40, 3: 15}, {1: 40, 2: 10}, {1: 60}, {1: 60, 4: 15},
              {1: 20, 3: -10}, {1: 60, 3: 20, 4: -10}, {2: 10, 3: 20}]  # joint steps from START + FIT_LIFT
FIT_LOW_MM = 12.0  # a fit pose measured lower than this ends the fit
FIT_AWAY = 8  # pulses: the approach overshoot (> the widest backlash gap seen, 5)
FIT_KEEP = 12  # the fit's poses stay in the learnt data
LEARN_KEEP = 60  # ... with this many of the latest measurements
STATE = Path("evidence/tip/well2_state.json")  # last commanded pose, correction, per-well corrections
REACH = ac.WELL_REACH  # evidence/tip/well_reach.json: the deliverable, one entry per well
UP = {k: ca.APPROACH for k in ARM}
NAMES = [f"{r}{c}" for r in "ABCDEFGH" for c in range(1, 13)]
TOP_Z = min(pr.well(w)[2] for w in NAMES)  # the highest rim (sheet z points down)


def model() -> np.ndarray:
    if FIT_MODEL.exists():  # this grasp's tip and base, fitted live (`fit`)
        return np.array(json.loads(FIT_MODEL.read_text())["p"], float)
    m = json.loads(ac.ARM_MODEL.read_text())
    p = np.array(m["session"], float)
    p[6:9] = m["tip_by_roll"][ac.GRASP_TIP]  # the re-seated near-vertical needle
    return p


def tip(p: np.ndarray, q: dict) -> np.ndarray:
    return ac.arm_model(p, [q], [UP])[0]


def inside(q: dict) -> bool:
    return all(ENVELOPE[k][0] <= q[k] <= ENVELOPE[k][1] for k in ARM)


def solve(p: np.ndarray, goal: np.ndarray, q0: dict, pitch: int | None = None,
          strict: bool = False) -> tuple[dict | None, int | None]:
    """Joints 1-4 for a tip at `goal` (model frame) with the needle angle at `pitch`, else the nearest pitch that works
    (default: nearest vertical); of the solutions found from q0, from aim_solve on q0 and on START (one start alone
    missed wells its neighbours had), the one closest to q0, so a path never jumps branch. `strict`: only `pitch`.
    (None, None) when none."""
    for s in [pitch or 1000] + ([] if strict else sorted(PITCHES, key=lambda s: abs(s - (pitch or 1000)))):
        seeds = [{k: q0[k] for k in ARM}] + [ac.aim_solve(p, goal, {k: st[k] for k in ARM}, s)[0] for st in (q0, ac.START)]
        sols = [q for q in (polish(p, goal, c, s) for c in seeds) if q is not None and inside(q)]
        if sols:
            return min(sols, key=lambda q: max(abs(q[k] - q0[k]) for k in ARM)), s
    return None, None


def polish(p: np.ndarray, goal: np.ndarray, q: dict, s: int) -> dict | None:
    """aim_solve's pull to the start pose leaves 0.5-0.9 mm, and rounding each joint alone up to ~1.5 mm at full reach
    (1 shoulder pulse ~1.1 mm there): Newton on tip + pitch (4 x 4, no pull), then the best of the 16 whole-pulse
    roundings. None when the tip stays more than ROUND_MM off.""" 
    x = np.array([q[k] for k in ARM], float)
    f = lambda v: np.r_[tip(p, dict(zip(ARM, v))) - goal, v[1] + v[2] + v[3] - s]  # noqa: E731
    for _ in range(15):
        jac = np.column_stack([(f(x + e) - f(x - e)) / 2.0 for e in np.eye(4)])
        x += np.clip(np.linalg.lstsq(jac, -f(x), rcond=None)[0], -5.0, 5.0)
    best = min((np.floor(x) + np.array(c) for c in np.ndindex(2, 2, 2, 2)),
               key=lambda v: np.linalg.norm(tip(p, dict(zip(ARM, v))) - goal))
    out = {k: int(v) for k, v in zip(ARM, best)}
    return out if np.linalg.norm(tip(p, out) - goal) <= ROUND_MM else None


def ring_order() -> list[str]:
    """Outer ring first (goal requirement 19), each ring walked round from its H-row, column-1 corner."""
    out = []
    for k in range(4):
        r0, r1, c0, c1 = k, 7 - k, k, 11 - k
        cells = [(r, c0) for r in range(r1, r0 - 1, -1)] + [(r0, c) for c in range(c0 + 1, c1 + 1)]
        cells += [(r, c1) for r in range(r0 + 1, r1 + 1)] + [(r1, c) for c in range(c1 - 1, c0, -1)]
        out += [f"{'ABCDEFGH'[r]}{c + 1}" for r, c in cells]
    return out


def well_pitch(p: np.ndarray, rim: np.ndarray, corr: np.ndarray, q0: dict, prefer: int = 1000) -> int | None:
    """The needle angle for a well: nearest vertical whose whole path, TRANSIT_MM down to the entry in CART_MM pieces,
    solves at that one angle with <= SMOOTH pulses per piece (near full stretch some angles sit by a singular pose and
    the solution hops: 68 pulses for one piece in the dry run, 2026-10-31). None: no such angle (base move)."""
    hs = np.arange(TRANSIT_MM, -DEPTH_MM - 0.1, -CART_MM).tolist() + [-DEPTH_MM]
    for s in sorted(PITCHES, key=lambda s: abs(s - prefer)):
        q, ok = dict(q0), True
        for h in hs:
            got, gs = solve(p, rim - [0, 0, h] + corr, q, s, strict=True)
            if got is None or (h != hs[0] and max(abs(got[k] - q[k]) for k in ARM) > SMOOTH):
                ok = False
                break
            q = got
        if ok:
            return s
    return None


def reachable(p: np.ndarray, name: str) -> tuple[bool, str]:
    """The well has a smooth path at one needle angle inside ENVELOPE (model only; the live run measures)."""
    if well_pitch(p, pr.well(name), np.zeros(3), ac.START) is None:
        return False, "no smooth path at any needle angle inside the envelope: needs the base move"
    return True, ""


class Arm:
    """The one way commands reach the servos. `corr` (sheet mm) is the model's measured error: the tip is commanded at
    goal + corr, so the model predicts the real tip at tip(q) - corr."""

    def __init__(self, p: np.ndarray, servos, q: dict, corr: np.ndarray):
        self.p, self.servos, self.q, self.corr = p, servos, {k: int(q[k]) for k in ARM}, np.asarray(corr, float)
        self.target: str | None = None  # the well an entry may go into
        self.tied = False  # corr holds a checked tie to the cameras (set by walk, kept in STATE)
        self.data: list[tuple[dict, np.ndarray]] = []  # (commanded pose, measured tip) the model is fitted to
        self.pitch_ref: int | None = None  # the needle angle every well prefers (set where the walk ends)
        self.m: np.ndarray | None = None  # the tip as last measured at self.q
        self.J: np.ndarray | None = None  # measured d tip / d pulse of self.joints there (refit_local)
        self.joints = J3  # the joints the measured Jacobian moves (well_visit adds the wrist)
        self.local: list[tuple[np.ndarray, np.ndarray]] = []  # (joints 1-3, measured tip) at the current well
        self.sent, self.max_step = 0, 0

    def predicted(self, q: dict | None = None) -> np.ndarray:
        return tip(self.p, q or self.q) - self.corr

    def go(self, q: dict, floor_mm: float) -> None:
        """Joint-space pieces of <= STEP pulses; every piece's predicted tip stays >= floor_mm over the highest rim, and
        below the rim only near the target well's centre."""
        q = {k: int(q[k]) for k in ARM}
        if not inside(q):
            raise RuntimeError(f"refusing {q}: outside {ENVELOPE}")
        q0, n = dict(self.q), max(1, math.ceil(max(abs(q[k] - self.q[k]) for k in ARM) / STEP))
        for i in range(1, n + 1):
            w = {k: round(q0[k] + (q[k] - q0[k]) * i / n) for k in ARM}
            t = self.predicted(w)
            if TOP_Z - t[2] < floor_mm - 0.5:
                raise RuntimeError(f"refusing {w}: predicted tip {TOP_Z - t[2]:.1f} mm over the rim, floor {floor_mm}")
            if TOP_Z - t[2] < 0 and (self.target is None or np.linalg.norm(t[:2] - pr.well(self.target)[:2]) > ENTRY_XY_MM):
                raise RuntimeError(f"refusing {w}: below the rim away from the target well")
            step = max(abs(w[k] - self.q[k]) for k in ARM)
            self.servos.send(w, max(MIN_MS, MS_PER_PULSE * step))
            self.q, self.sent, self.max_step = w, self.sent + 1, max(self.max_step, step)
        self.servos.dwell(DWELL_S)

    def learn(self, m: np.ndarray) -> None:
        """Calibrate where the arm works: the tip measured at the current pose joins the data, tip, yaw and base are
        refitted on all of it from the current values, and corr becomes the refit's residual here (the model fitted by
        START alone was 27 mm off 35 mm over H1, 2026-10-31). The fit poses plus the last LEARN_KEEP measurements."""
        self.data = self.data[:FIT_KEEP] + self.data[FIT_KEEP:][-LEARN_KEEP + 1:] + [(dict(self.q), np.asarray(m, float))]
        x, res = fit_params(self.p, [d[0] for d in self.data], np.array([d[1] for d in self.data]), seed=False)
        self.p = x
        self.corr = tip(self.p, self.q) - np.asarray(m, float)
        print(f"  learnt: {len(self.data)} poses, {np.median(res):.2f} mm median / {res.max():.2f} max; here "
              f"{np.linalg.norm(self.corr):.2f} mm off the refit", flush=True)

    def background(self, eyes) -> None:
        """Background frames with the arm parked out of both views, then back: a background with the needle a leg
        away kept its ghost, and the tip pick took the ghost's end (tie off by 9 mm, 2026-10-31); the parked one has
        no needle at all (23 of 24 fit poses measured). Retaken when older than BG_MAX_S. Every piece of the way
        there and back keeps the predicted tip at or above where it starts."""
        if eyes.bg_at is not None and self.servos.now() - eyes.bg_at < BG_MAX_S:
            return
        back = dict(self.q)
        n = max(1, math.ceil(max(abs(PARK[k] - back[k]) for k in ARM) / STEP))
        h0 = TOP_Z - self.predicted()[2]
        for i in range(1, n + 1):
            w = {k: back[k] + (PARK[k] - back[k]) * i / n for k in ARM}
            if TOP_Z - self.predicted(w)[2] < h0 - 1.0:
                raise RuntimeError(f"the way to PARK dips to {TOP_Z - self.predicted(w)[2]:.0f} mm over the rim at {w}")
        self.servos.parking = True
        try:
            jgo(self.servos, PARK, back)
            eyes.background()
            eyes.bg_at = self.servos.now()
            jgo(self.servos, back, PARK)
        finally:
            self.servos.parking = False

    def line(self, goal: np.ndarray, floor_mm: float, pitch: int | None = None) -> int:
        """Straight tip line from the predicted tip to `goal` (real sheet mm) in CART_MM pieces, the needle angle
        moving evenly from the current pitch to `pitch`. Returns the pitch used at the end."""
        a = self.predicted()
        floor_mm = min(floor_mm, TOP_Z - a[2] - 1.0)  # a line starting under its floor (a rise) must not dip below its start
        s0 = self.q[2] + self.q[3] + self.q[4]
        s1 = s0 if pitch is None else pitch
        # the needle angle turns <= PITCH_STEP per piece: a 40-pulse turn inside one 5 mm piece dipped the tip 1.3 mm
        # between the solved ends (dry run, 2026-10-31)
        n = max(1, math.ceil(np.linalg.norm(goal - a) / CART_MM), math.ceil(abs(s1 - s0) / PITCH_STEP))
        s = s0
        for i in range(1, n + 1):
            s = self.piece(a + (goal - a) * i / n, round(s0 + (s1 - s0) * i / n), floor_mm)
        return s

    def piece(self, point: np.ndarray, want: int, floor_mm: float, depth: int = 0) -> int:
        """One tip step to `point` (real sheet mm) at needle angle `want`. A solution more than BRANCH_JUMP pulses away is
        bisected (the stretched arm needs 30+ pulses for 5 mm, dry run 2026-10-31); one that stays that far after 3
        halvings is a solver branch hop and refused."""
        q, got = solve(self.p, point + self.corr, self.q, want)
        if q is None:
            raise RuntimeError(f"no pose for {np.round(point, 1).tolist()} inside the envelope")
        if max(abs(q[k] - self.q[k]) for k in ARM) > BRANCH_JUMP:
            if depth >= 3:
                raise RuntimeError(f"refusing {q}: {max(abs(q[k] - self.q[k]) for k in ARM)} pulses from {self.q} "
                                   f"for a {CART_MM / 8:.1f} mm step (a solver branch hop)")
            here, s = self.predicted(), self.q[2] + self.q[3] + self.q[4]
            self.piece((here + point) / 2, round((s + want) / 2), floor_mm, depth + 1)
            return self.piece(point, want, floor_mm, depth + 1)
        self.go(q, floor_mm)
        return got


ARM_LOCK = 'evidence/tip/arm.lock'


def _drives_arm(pid):
    """pid is alive and runs one of our tools (a dead holder's pid was briefly 'alive' and blocked a run, 2026-11-03)."""
    import psutil
    try:
        return any('tools' in a for a in psutil.Process(pid).cmdline())
    except (psutil.Error, OSError):
        return False


def claim_arm():
    """One driver: the lock file holds the driving process tree's root pid (env ARM_DRIVER, inherited by children).
    Raises while another live tree holds it. Two colony_cycle runs drove the arm at once, 2026-11-02 23:31."""
    import os
    import psutil
    me = os.environ.setdefault('ARM_DRIVER', str(os.getpid()))
    try:
        held = open(ARM_LOCK).read().strip()
    except OSError:
        held = ''
    if held and held != me and held.isdigit() and _drives_arm(int(held)):
        raise RuntimeError(f'arm is driven by pid {held} ({ARM_LOCK}): one driver only')
    open(ARM_LOCK, 'w').write(me)


class Servos:
    """The robot, through tip_descend's bridge session."""

    def __init__(self):
        import tip_descend as td
        self.td = td

    def read(self) -> dict:
        return self.again(self.td.read)

    def send(self, q: dict, ms: int) -> None:
        claim_arm()
        self.again(lambda: self.td.send(q, ms))
        time.sleep(ms / 1000)

    @staticmethod
    def again(call):
        """One reconnect on a dropped bridge: the robot's WiFi reassociated mid-run (2026-10-31); commands are absolute
        positions, so a resend is safe."""
        import rig_io
        try:
            return call()
        except RuntimeError as e:
            if "bridge" not in str(e):
                raise
            print(f"bridge dropped ({e}): reconnecting", flush=True)
            rig_io.reset_bridge()
            time.sleep(3.0)
            return call()

    def dwell(self, s: float) -> None:
        time.sleep(s)

    def now(self) -> float:
        return time.time()


def needle_ends(frame, bg, min_px=300, low=12, gap=4, reach=80):
    """Each changed blob's end along its own long axis (the needle points down the image), traced past the 3x3
    opening into the raw difference: the tapered tip lost ~25 px to the opening (5 mm in the side view)."""
    raw = np.abs(cv2.GaussianBlur(frame, (3, 3), 0).astype(np.int16) - cv2.GaussianBlur(bg, (3, 3), 0).astype(np.int16)).max(axis=2)
    d = cv2.morphologyEx((np.abs(cv2.GaussianBlur(frame, (5, 5), 0).astype(np.int16) - cv2.GaussianBlur(bg, (5, 5), 0).astype(np.int16)).max(axis=2) > 30).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(d)
    out = []
    for j in range(1, n):
        if st[j, 4] < min_px:
            continue
        vs, us = np.nonzero(lab == j)
        pts = np.column_stack([us, vs]).astype(float)
        c = pts.mean(0)
        w, v = np.linalg.eigh(np.cov((pts - c).T))
        ax = v[:, -1] if v[1, -1] > 0 else -v[:, -1]  # long axis, pointing down the image
        end = np.array([us[vs == vs.max()].mean(), vs.max()], float)
        if w[-1] > 9 * max(w[0], 1e-6):  # elongated: trace on
            k, miss, best = 0, 0, end
            while k < reach and miss <= gap:
                k += 1
                u, vv = (end + ax * k).round().astype(int)
                if not (1 <= u < raw.shape[1] - 1 and 1 <= vv < raw.shape[0] - 1):
                    break
                if raw[vv - 1:vv + 2, u - 1:u + 2].max() > low:
                    best, miss = end + ax * k, 0
                else:
                    miss += 1
            end = best
        out.append(end)
    return out


class Eyes:
    """Both cameras; the bare needle by difference against a background pair taken with the tip lifted away."""

    def __init__(self, tag: str):
        self.top, self.side, self.det, self.urig, self.srig = ac.cameras()
        ac.measure.thin = True
        self.bg_side, self.tag, self.n, self.bg_at = None, tag, 0, None

    def background(self) -> None:
        self.bg_side, ac.measure.top_bg = self.side.frame(), self.top.frame()
        cv2.imwrite(f"evidence/tip/well2_{self.tag}_bg{self.n:03d}.raw.side.jpg", self.bg_side)
        cv2.imwrite(f"evidence/tip/well2_{self.tag}_bg{self.n:03d}.raw.top.jpg", ac.measure.top_bg)

    def tip(self, expect: np.ndarray, within: float, label: str) -> np.ndarray | None:
        """The tip near `expect` (sheet mm, within `within`), as free_tip picks it."""
        return self.free_tip(label, expect, within)

    def free_tip(self, label: str, expect: np.ndarray | None = None, within: float = math.inf) -> np.ndarray | None:
        """Every changed blob's lowest pixel in both views against the background; of the pairs meeting within MISS_MM,
        over 8 mm up (a shadow on the paper meets too) and within `within` of `expect`, the one lowest in both images:
        the tip is the arm's lowest point in both views, and the closest-meeting pair was a changed patch on the robot
        body in 5 of 12 fit poses (2026-10-31). Retried on 3 frame pairs; the frames are saved for a look."""
        import tip_triangulate as tt
        self.n += 1
        for _ in range(3):
            sf, tf = self.side.frame(), self.top.frame()
            cv2.imwrite(f"evidence/tip/well2_{self.tag}_{self.n:03d}_{label}.raw.side.jpg", sf)
            cv2.imwrite(f"evidence/tip/well2_{self.tag}_{self.n:03d}_{label}.raw.top.jpg", tf)
            sides, tops = needle_ends(sf, self.bg_side), needle_ends(tf, ac.measure.top_bg)
            cands = [(*tt.meet(tt.ray(t_, *self.urig), tt.ray(s_, *self.srig)), t_, s_) for s_ in sides for t_ in tops]
            cands = [c for c in cands if c[1] <= MISS_MM and -c[0][2] >= 8.0
                     and (expect is None or np.linalg.norm(c[0] - expect) <= within)]
            if cands:
                p3, _, tpx, spx = max(cands, key=lambda c: c[2][1] + c[3][1])
                ac.measure.px = (tpx.tolist(), spx.tolist())
                self.snapshot(f"{self.n:03d}_{label}", p3 if expect is None else expect, True)
                return p3
        if expect is not None:
            ac.measure.px = None
            self.snapshot(f"{self.n:03d}_{label}_miss", expect, False)
        return None

    def snapshot(self, label: str, expect: np.ndarray, ok: bool) -> None:
        """Top and side crops around where the tip was expected, the found tip marked: for a look by eye."""
        crops = []
        for cam, rig, i in ((self.top, self.urig, 0), (self.side, self.srig, 1)):
            img = cam.frame()
            e = cv2.projectPoints(np.asarray(expect, float).reshape(1, 3), rig[2], rig[3], rig[0], rig[1])[0][0, 0]
            cv2.circle(img, tuple(int(v) for v in e), 30, (0, 255, 0), 2)
            if ok and getattr(ac.measure, "px", None):
                cv2.drawMarker(img, tuple(int(v) for v in ac.measure.px[i]), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
            x0, y0 = int(np.clip(e[0] - 300, 0, img.shape[1] - 600)), int(np.clip(e[1] - 300, 0, img.shape[0] - 600))
            crops.append(img[y0:y0 + 600, x0:x0 + 600])
        cv2.imwrite(f"evidence/tip/well2_{self.tag}_{label}.jpg", np.hstack(crops))

    def which_well(self, rim_z: float, target: str) -> tuple[str, float] | None:
        near = cv2.projectPoints(pr.well(target).reshape(1, 3), self.urig[2], self.urig[3], self.urig[0],
                                 self.urig[1])[0][0, 0]
        seen = ac.in_well(self.top.frame(), self.urig, rim_z, near)
        return None if seen is None else (seen[0], seen[1])

    def photos(self, stem: str) -> str:
        path = f"evidence/tip/well2_{self.tag}_{stem}"
        cv2.imwrite(path + ".top.jpg", self.top.frame())
        cv2.imwrite(path + ".side.jpg", self.side.frame())
        return path + ".top.jpg"


def walk(arm: Arm, eyes, to: str = "H1") -> None:
    """From the arm's pose to TRANSIT_MM over `to` in WALK_LEG_MM legs, measuring at each leg end."""
    rim = pr.well(to)
    end = np.r_[rim[:2], TOP_Z - TRANSIT_MM]
    # no climb on the spot: by START the folded arm rises only through a near-singular pose (37 pulses per 0.6 mm in
    # the dry run, 2026-10-31); the legs climb to TRANSIT_MM on the way out
    a = arm.predicted()
    n = max(1, math.ceil(np.linalg.norm(end[:2] - a[:2]) / WALK_LEG_MM))
    # the needle angle turns evenly from START's to the one over `to`: held at START's (879) the arm ran into a
    # 33-pulse-per-5-mm corner halfway (dry run, 2026-10-31)
    s0 = arm.q[2] + arm.q[3] + arm.q[4]
    s1 = solve(arm.p, end + arm.corr, arm.q)[1]
    if s1 is None:
        raise RuntimeError(f"no pose {TRANSIT_MM:.0f} mm over {to}")
    if not arm.tied:
        raise RuntimeError("no fit to the cameras in STATE: run `fit` first")
    since = 0.0
    for i in range(1, n + 1):
        goal = a + (end - a) * i / n
        if since + WALK_LEG_MM > WALK_FRESH_MM and np.linalg.norm(goal[:2] - rim[:2]) < 40.0:
            raise RuntimeError(f"no measured tip in the last {since:.0f} mm: not going over the plate")
        arm.background(eyes)
        arm.line(goal, floor_mm=TRANSIT_MM - 5.0, pitch=round(s0 + (s1 - s0) * i / n))
        got = eyes.tip(goal, WALK_EXPECT_MM, f"walk{i}")
        since = 0.0 if got is not None else since + WALK_LEG_MM
        if got is not None:
            print(f"walk {i}/{n}: off {np.round(got - goal, 1).tolist()} mm", flush=True)
            arm.learn(got)
        else:
            print(f"walk {i}/{n}: tip not measured", flush=True)
    if since > 0:
        raise RuntimeError("walk ended without a measured tip over the plate")
    arm.m = got
    arm.pitch_ref = arm.q[2] + arm.q[3] + arm.q[4]


def model_jacobian(arm: Arm) -> np.ndarray:
    """d tip / d pulse of arm.joints by the model at the arm's pose (3 x n): only the first guess for a probe column
    that measured nothing."""
    return np.column_stack([(tip(arm.p, {**arm.q, k: arm.q[k] + 1}) - tip(arm.p, {**arm.q, k: arm.q[k] - 1})) / 2.0
                            for k in arm.joints])


def step_joints(arm: Arm, v: np.ndarray, floor_mm: float, rim_z: float, target: str | None = None,
                approach: bool = True) -> np.ndarray:
    """arm.joints (1-3, or 1-4 with the wrist) by `v` pulses in STEP pieces, ending (`approach`) with every joint coming from
    FIT_AWAY pulses above, as the fit did: 6-pulse probes from either side moved the tip 5 vs 12 mm (backlash, live
    2026-10-31). Every piece's tip as the measured Jacobian predicts it stays >= floor_mm over `rim_z`, and below
    the rim only within ENTRY_XY_MM of `target`'s centre. Returns the predicted tip."""
    v = np.round(np.asarray(v, float))
    q0 = dict(arm.q)
    q = {**q0, **{k: q0[k] + int(v[i]) for i, k in enumerate(arm.joints)}}
    legs = ([{**q, **{k: q[k] - ca.APPROACH * FIT_AWAY for k in arm.joints}}] if approach else []) + [q]
    for leg in legs:
        if not inside(leg):
            raise RuntimeError(f"refusing {leg}: outside {ENVELOPE}")
    pieces, at = [], dict(q0)
    for leg in legs:
        n = max(1, math.ceil(max(abs(leg[k] - at[k]) for k in ARM) / STEP))
        pieces += [{k: round(at[k] + (leg[k] - at[k]) * i / n) for k in ARM} for i in range(1, n + 1)]
        at = leg
    for w in pieces:
        pred = arm.m + arm.J @ np.array([w[k] - q0[k] for k in arm.joints], float)
        if rim_z - pred[2] < floor_mm - 0.5:
            raise RuntimeError(f"refusing {w}: tip predicted {rim_z - pred[2]:.1f} mm over the rim, floor {floor_mm}")
        if rim_z - pred[2] < 0 and (target is None or np.linalg.norm(pred[:2] - pr.well(target)[:2]) > ENTRY_XY_MM):
            raise RuntimeError(f"refusing {w}: below the rim away from the target well")
    for w in pieces:
        step = max(abs(w[k] - arm.q[k]) for k in ARM)
        arm.servos.send(w, max(MIN_MS, MS_PER_PULSE * step))
        arm.q, arm.sent = w, arm.sent + 1
    arm.servos.dwell(DWELL_S)
    return arm.m + arm.J @ v


def look(eyes, expect: np.ndarray, within: float, label: str) -> np.ndarray | None:
    return next((g for g in (eyes.tip(expect, within, label) for _ in range(2)) if g is not None), None)


def probe(arm: Arm, eyes, rim_z: float, label: str) -> None:
    """The local Jacobian measured: each of arm.joints +-PROBE pulses from here and back; a column that
    measured nothing keeps the model's. Over the plate the model's slopes were off too (corrections overshot ~2x)."""
    jac, m0 = model_jacobian(arm), arm.m.copy()
    arm.J = jac.copy()
    floor = min(TRANSIT_MM - 15.0, rim_z - m0[2] - 10.0)  # probes stay within 10 mm under where they start
    arm.local = [(np.array([arm.q[k] for k in arm.joints], float), m0)]
    for i, k in enumerate(arm.joints):
        got = []
        for sg in (1, -1):
            v = np.zeros(len(arm.joints))
            v[i] = sg * PROBE
            pred = step_joints(arm, v, floor, rim_z)
            got.append(look(eyes, pred, EXPECT_MM, f"{label}_probe{k}{'+' if sg > 0 else '-'}"))
            if got[-1] is not None:
                arm.local.append((np.array([arm.q[k] for k in arm.joints], float), got[-1]))
            step_joints(arm, -v, floor, rim_z)
        if got[0] is not None and got[1] is not None:
            jac[:, i] = (got[0] - got[1]) / (2 * PROBE)
    arm.J, arm.m = jac, m0
    print("  probed, mm per pulse: " + ", ".join(f"joint {k} {np.round(jac[:, i], 2).tolist()}"
                                                  for i, k in enumerate(arm.joints)), flush=True)


def refit_local(arm: Arm) -> None:
    """J by least squares on every tip measured at this well (tip = a + J q): rank-1 (Broyden) updates from small,
    noisy steps drifted J, and the blind entry then went 3.6 mm sideways onto the rim (dry run C1). Needs 5 poses
    spanning all three joints; the probes give that."""
    if len(arm.local) < 5:
        return
    qs = np.array([p for p, _ in arm.local])
    if np.linalg.matrix_rank(qs - qs.mean(0), tol=1.0) < 3:
        return
    sol = np.linalg.lstsq(np.column_stack([qs, np.ones(len(qs))]), np.array([m for _, m in arm.local]), rcond=None)[0]
    arm.J = sol[:3].T


def servo(arm: Arm, eyes, goal: np.ndarray, gate: float, floor_mm: float, rim_z: float, label: str) -> bool:
    """Steps of <= SERVO_STEP_MM by the measured Jacobian toward `goal`, each measured (gate EXPECT_MM around where the
    Jacobian put it) and the Jacobian refitted on every tip measured at this well. True once xy is within `gate`."""
    for it in range(SERVO_TRIES):
        err = goal - arm.m
        print(f"  {label} {it}: xy {np.linalg.norm(err[:2]):.2f} z {err[2]:+.2f} mm", flush=True)
        if np.linalg.norm(err[:2]) <= gate and abs(err[2]) <= (GATE_MM if gate <= GATE_MM else Z_OK_MM):
            return True
        want = err if np.linalg.norm(err) <= SERVO_STEP_MM else err * SERVO_STEP_MM / np.linalg.norm(err)
        v = np.linalg.lstsq(arm.J, want, rcond=None)[0]
        v = np.round(v * min(1.0, SERVO_MAX_PULSES / max(np.abs(v).max(), 1e-9)))  # a near-singular J asked 30 mm
        if not v.any():  # under one pulse: the joint that moves the tip most along the error
            i = int(np.argmax(np.abs(arm.J.T @ err)))
            v[i] = np.sign(arm.J[:, i] @ err)
        pred = step_joints(arm, v, floor_mm, rim_z)
        got = look(eyes, pred, SERVO_LOOK_MM, f"{label}_{it}")
        if got is None:
            return False
        arm.local.append((np.array([arm.q[k] for k in J3], float), got))
        refit_local(arm)
        arm.m = got
    return False


def lift(arm: Arm, eyes, goal: np.ndarray, rim_z: float, name: str) -> None:
    """Back up to `goal` (transit) in measured steps; up is the safe way, so a lost measurement goes on by the
    Jacobian's prediction (a lift that stopped at a lost tip left the next well's probes by the rim, dry run)."""
    for it in range(12):
        err = goal - arm.m
        if abs(err[2]) <= 3.0 and np.linalg.norm(err[:2]) <= 3.0:
            return
        want = err if np.linalg.norm(err) <= SERVO_STEP_MM else err * SERVO_STEP_MM / np.linalg.norm(err)
        v = np.linalg.lstsq(arm.J, want, rcond=None)[0]
        v = np.round(v * min(1.0, SERVO_MAX_PULSES / max(np.abs(v).max(), 1e-9)))
        pred = step_joints(arm, v, rim_z - arm.m[2] - 1.0, rim_z)
        got = look(eyes, pred, SERVO_LOOK_MM, f"{name}_lift{it}")
        if got is not None:
            arm.local.append((np.array([arm.q[k] for k in J3], float), got))
            refit_local(arm)
        arm.m = pred if got is None else got
    raise RuntimeError(f"{name}: no lift back to {TRANSIT_MM:.0f} mm in 12 steps")


def into(arm: Arm, eyes, name: str, memory: dict) -> dict:
    """Over well `name` at TRANSIT_MM, then down to each of HEIGHTS_MM by the measured Jacobian (servo), the last one to
    the GATE_MM; the entry DEPTH_MM below the rim by the Jacobian, the overhead's word on which well, photos, back out
    and up. Returns its well_reach entry."""
    t0, rim = time.time(), pr.well(name)
    over = lambda h: rim - [0, 0, h]  # noqa: E731  real sheet mm, h over this well's rim
    rz = float(rim[2])
    arm.background(eyes)
    arm.m = look(eyes, arm.m if arm.m is not None else over(TRANSIT_MM), EXPECT_MM if arm.m is not None else START_LOOK_MM,
                 f"{name}_start")
    if arm.m is None:
        raise RuntimeError("tip not measured where the arm is: not moving on")
    # settle into the approach-from-above backlash state every later step ends in, and measure there: reached from the
    # other side the start sat ~10 mm off that state, and the first step "jumped" by it (live H1, 2026-10-31)
    arm.J = model_jacobian(arm)
    step_joints(arm, np.zeros(3), rz - arm.m[2] - 10.0, rz)
    arm.m = look(eyes, over(TRANSIT_MM), START_LOOK_MM, f"{name}_settled")
    if arm.m is None:
        raise RuntimeError("tip not measured after settling: not moving on")
    probe(arm, eyes, rz, name)  # every well: a Jacobian carried from H1 by Broyden alone predicted a 60 mm drop at B1
    if not servo(arm, eyes, over(TRANSIT_MM), PASS_MM, TRANSIT_MM - 8.0, rz, f"{name}_{TRANSIT_MM:.0f}"):
        return {"reached": False, "reason": f"not over it at {TRANSIT_MM:.0f} mm"}
    for h in HEIGHTS_MM:
        # floors: 3 mm under each height, but never under 1.5 mm over the rim (at h - 3 = 0 the dry run's true tip
        # reached rim height 2.5 mm off C1's centre)
        if not servo(arm, eyes, over(h), GATE_MM if h == HEIGHTS_MM[-1] else PASS_MM, max(h - 3.0, 1.5), rz,
                     f"{name}_{h:.0f}"):
            got = look(eyes, arm.m, SERVO_LOOK_MM, f"{name}_up")
            arm.m = arm.m if got is None else got
            lift(arm, eyes, over(TRANSIT_MM), rz, name)
            return {"reached": False, "reason": f"not within the gate at {h:.0f} mm"}
    gate_xy = float(np.linalg.norm((over(HEIGHTS_MM[-1]) - arm.m)[:2]))
    m_over = arm.m.copy()
    v_in = np.round(np.linalg.lstsq(arm.J, over(-DEPTH_MM) - arm.m, rcond=None)[0])
    # straight down: shoulder and elbow fall (the from-above direction) anyway; the +8 overshoot would be a 13 mm
    # sideways detour whose way back down crossed the rim off-centre (dry run)
    step_joints(arm, v_in, -DEPTH_MM - 1.0, rz, target=name, approach=False)
    seen = eyes.which_well(rz, name)
    photo = eyes.photos(name)
    step_joints(arm, -v_in, -DEPTH_MM - 1.0, rz, target=name, approach=False)  # straight back out
    arm.m = m_over
    lift(arm, eyes, over(TRANSIT_MM), rz, name)
    ok = seen is not None and seen[0] == name
    print(f"{name}: {'REACHED' if ok else 'NOT reached'} (overhead sees the needle in {seen}), gate {gate_xy:.2f} mm, "
          f"{time.time() - t0:.0f} s", flush=True)
    return {"reached": ok, "gate_xy_mm": gate_xy, "seen_in": None if seen is None else seen[0],
            "seen_off_centre_mm": None if seen is None else seen[1], "photo": photo,
            "reason": "" if ok else "overhead did not see the needle in this well", "seconds": round(time.time() - t0)}


STEP_FAST = 12      # fast=True: pieces for moves over FAST_FROM pulses (transits): 12 in 250 ms, still half the limit
FAST_FROM = 60      # smaller moves (descents into wells, corrections near wells and agar) keep STEP: fast low moves once dislodged the needle


def jgo(servos, q: dict, now: dict, dwell: float = DWELL_S, fast: bool = False) -> dict:
    """Joint pieces of <= STEP pulses at MS_PER_PULSE with no model (park and fit only); returns q. `dwell`: pause after
    the move (frames need DWELL_S; a move straight into another needs little)."""
    assert all(0 <= q[k] <= 1000 for k in ARM) and q[2] <= 920, q
    span = max(abs(q[k] - now[k]) for k in ARM)
    step = STEP_FAST if fast and span > FAST_FROM else STEP
    n = max(1, math.ceil(span / step))
    for i in range(1, n + 1):
        servos.send({k: round(now[k] + (q[k] - now[k]) * i / n) for k in ARM}, max(MIN_MS, MS_PER_PULSE * STEP))
    servos.dwell(dwell)
    return dict(q)


def fit_params(p: np.ndarray, qs: list[dict], seen: np.ndarray, seed: bool = True) -> tuple[np.ndarray, np.ndarray]:
    """Tip (camera frame), yaw and base of arm_model fitted to measured tips (arm terms held): damped Gauss-Newton from
    p, the tip seeded by inverting the first pose (as arm_calibrate.fit_tip) unless `seed` is False. (params, residual
    mm per pose)."""
    up = [UP] * len(qs)
    c, s_ = np.cos(p[9]), np.sin(p[9])
    rot = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]) @ np.diag([1.0, -1.0, -1.0])
    terms = {"d2": p[0], "d3": p[1], "b": {str(k): p[1 + k] for k in (1, 2, 3, 4)}}
    r_cam, o_cam = ca.camera_in_base(ca.arm_pulses(qs[0], terms, UP))
    x = p.copy()
    if seed:
        x[6:9] = r_cam.T @ (rot.T @ (seen[0] - p[10:13]) - o_cam)
    free, lam = np.arange(6, 13), 1e-2
    err = lambda v: (ac.arm_model(v, qs, up) - seen).ravel()  # noqa: E731
    e = err(x)
    for _ in range(200):
        cols = []
        for i in free:
            d = np.eye(13)[i] * (1e-3 if i == 9 else 1e-2)
            cols.append((err(x + d) - e) / d[i])
        jac = np.column_stack(cols)
        h = jac.T @ jac
        step = np.linalg.solve(h + lam * np.diag(np.diag(h) + 1e-9), -jac.T @ e)
        new = x.copy()
        new[free] += step
        en = err(new)
        if en @ en < e @ e:
            x, e, lam = new, en, lam * 0.3
            if np.linalg.norm(step) < 1e-5:
                break
        else:
            lam *= 10
    return x, np.linalg.norm(e.reshape(-1, 3), axis=1)


def fit(servos, eyes) -> None:
    """Background with the arm parked, then FIT_DELTAS around START + FIT_LIFT, each measured with no expectation;
    tip, yaw and base fitted (the worst pose dropped while one is over 3 mm, down to 6), saved to FIT_MODEL. Ends at
    START + FIT_LIFT with a fresh STATE, tied (the fit is the tie).""" 
    now = {k: servos.read()[k] for k in ARM}
    if max(abs(now[k] - ac.START[k]) for k in ARM) > 25:  # START, or the last fit's centre (shoulder +20)
        raise RuntimeError(f"arm at {now}, not START: run `home` first")
    if STATE.exists():  # build on what was commanded: servo 2 reads ~5 under where it holds
        held = {int(k): v for k, v in json.loads(STATE.read_text())["q"].items()}
        now = held if max(abs(held[k] - now[k]) for k in ARM) <= 12 else now
    q = jgo(servos, PARK, now)
    eyes.background()
    centre = {**ac.START, 2: ac.START[2] + FIT_LIFT}
    q = jgo(servos, centre, q)
    qs, seen = [], []
    try:
        for i, d in enumerate(FIT_DELTAS):
            target = {k: centre[k] + d.get(k, 0) for k in ARM}
            # every joint ends coming from above, the approach arm_model's terms assume (ca.APPROACH): from either side
            # the fit left 4-9 mm on some poses (backlash, 2026-10-31)
            q = jgo(servos, {k: target[k] - ca.APPROACH * FIT_AWAY for k in ARM}, q)
            q = jgo(servos, target, q)
            got = eyes.free_tip(f"fit{i}")
            if got is None:
                print(f"fit {i} {target}: tip not measured", flush=True)
                continue
            print(f"fit {i} {target}: tip {np.round(got, 1).tolist()} ({-got[2]:.0f} mm up)", flush=True)
            if -got[2] < FIT_LOW_MM:
                raise RuntimeError(f"tip {-got[2]:.0f} mm up at {target}: stopping the fit")
            qs.append(target)
            seen.append(got)
    finally:
        q = jgo(servos, centre, q)
        # the fit ties the model to today's camera frame itself; a separate tie near the robot measured nothing there
        # (the tip projects onto the silver robot body in the overhead, 2026-10-31)
        STATE.write_text(json.dumps({"q": {str(k): v for k, v in q.items()}, "corr": [0.0, 0.0, 0.0], "tied": True,
                                     "memory": {}}))
    Path(f"evidence/tip/well2_fit_{time.strftime('%Y%m%dT%H%M%S')}.json").write_text(json.dumps(
        [{"cmd": {str(k): v for k, v in qq.items()}, "tip_mm": t.tolist()} for qq, t in zip(qs, seen)], indent=1))
    if len(qs) < 6:
        raise RuntimeError(f"only {len(qs)} poses measured: look at evidence/tip/well2_*_fit*.jpg")
    p0 = np.array(json.loads(ac.ARM_MODEL.read_text())["session"], float)
    seen = np.array(seen)
    x, res = fit_params(p0, qs, seen)
    while res.max() > 3.0 and len(qs) > 6:
        keep = np.arange(len(qs)) != int(np.argmax(res))
        print(f"dropping fit pose {qs[int(np.argmax(res))]} ({res.max():.1f} mm)", flush=True)
        qs, seen = [qq for qq, k in zip(qs, keep) if k], seen[keep]
        x, res = fit_params(p0, qs, seen)
    print(f"fit: tip {np.round(x[6:9], 1).tolist()} (camera frame), yaw {np.degrees(x[9]):+.2f} deg, base "
          f"{np.round(x[10:13], 1).tolist()} mm; per pose {np.round(res, 1).tolist()} mm, median {np.median(res):.2f}",
          flush=True)
    FIT_MODEL.write_text(json.dumps({"p": x.tolist(), "median_mm": float(np.median(res)), "max_mm": float(res.max()),
                                     "poses": len(qs), "when": time.strftime("%Y-%m-%dT%H:%M:%S")}, indent=1))


def home(servos) -> None:
    """After power-on: hold every servo where it reads (roll and gripper too: a hold at the read value moves nothing, and
    the gripper keeps the needle), then joints 1-4 to START in STEP-pulse pieces at MS_PER_PULSE. Forgets STATE."""
    now = servos.read()
    print(f"read {now}", flush=True)
    if any(not 0 <= now[k] <= 1000 for k in (5, 10)) or any(not -100 <= now[k] <= 1100 for k in ARM):
        raise RuntimeError(f"a servo reads far outside 0-1000 ({now}): not moving")
    # the park pose reads the elbow past its end (-54..-66, arm_park.py): clamp into 0-1000 as arm_to_start.py does
    q = {k: min(max(now[k], 0), 1000) for k in ARM}
    servos.send({**q, 5: now[5], 10: now[10]}, max(1000, MS_PER_PULSE * max(abs(q[k] - now[k]) for k in ARM)))
    n = max(1, math.ceil(max(abs(ac.START[k] - q[k]) for k in ARM) / STEP))
    for i in range(1, n + 1):
        w = {k: round(q[k] + (ac.START[k] - q[k]) * i / n) for k in ARM}
        servos.send(w, max(MIN_MS, MS_PER_PULSE * STEP))
    servos.dwell(DWELL_S)
    STATE.unlink(missing_ok=True)
    print(f"at START, read {servos.read()}", flush=True)


def run(cmd: str, names: list[str], dry: bool) -> None:
    if cmd == "home":
        return home(Servos())
    if cmd == "fit":
        return fit(Servos(), Eyes(time.strftime("%Y%m%dT%H%M%S")))
    p = model()
    order = ring_order() if cmd == "ring" else names
    state = json.loads(STATE.read_text()) if STATE.exists() and not dry else {}
    if dry:
        servos, eyes, q, corr = Sim(p), None, {k: ac.START[k] for k in ARM}, np.zeros(3)
        eyes = servos
    else:
        servos = Servos()
        now = servos.read()
        q, corr = ({int(k): v for k, v in state["q"].items()}, np.array(state["corr"])) if state else (dict(ac.START), np.zeros(3))
        # servo 2 reads ~5 under where it holds by START; stretched toward the plate shoulder and elbow sagged 15 / 13
        off = {k: now[k] - q[k] for k in ARM if abs(now[k] - q[k]) > 25}
        if off:
            raise RuntimeError(f"arm is not where {STATE if state else 'START'} says (read-back off by {off}): not moving")
        servos.td._sent.update({k: q[k] for k in ARM})
        eyes = Eyes(time.strftime("%Y%m%dT%H%M%S"))
    if state.get("p"):
        p = np.array(state["p"], float)
    arm = Arm(p, servos, q, corr)
    arm.tied = bool(state.get("tied")) or dry  # the dry run simulates the fit's poses below
    arm.pitch_ref = state.get("pitch_ref")
    arm.m = None if state.get("m") is None else np.array(state["m"])
    arm.J = None if state.get("J") is None else np.array(state["J"])
    if dry:  # the fit's poses, measured on the simulated arm
        arm.data = [(qq, servos.true(qq)) for qq in ({k: ac.START[k] + (k == 2) * FIT_LIFT + d.get(k, 0) for k in ARM}
                                                     for d in FIT_DELTAS)]
    elif state.get("data"):
        arm.data = [({int(k): v for k, v in qq.items()}, np.array(m)) for qq, m in state["data"]]
    else:
        last = sorted(Path("evidence/tip").glob("well2_fit_*.json"))[-1]
        arm.data = [({int(k): v for k, v in r["cmd"].items()}, np.array(r["tip_mm"])) for r in json.loads(last.read_text())]
    memory = {k: v for k, v in state.get("memory", {}).items()}
    results = json.loads(REACH.read_text()) if REACH.exists() and not dry else {}
    for w in NAMES:
        ok, why = reachable(p, w)
        if not ok and not (results.get(w) or {}).get("reached"):
            results[w] = {"well": w, "reached": False, "reason": why}

    def save() -> None:
        if dry:
            return
        STATE.write_text(json.dumps({"q": {str(k): v for k, v in arm.q.items()}, "corr": arm.corr.tolist(), "tied": arm.tied,
                                     "memory": memory, "p": arm.p.tolist(), "pitch_ref": arm.pitch_ref,
                                     "m": None if arm.m is None else arm.m.tolist(),
                                     "J": None if arm.J is None else arm.J.tolist(),
                                     "data": [({str(k): v for k, v in qq.items()}, m.tolist()) for qq, m in arm.data]}))
        REACH.write_text(json.dumps({w: results.get(w, {"well": w, "reached": False, "reason": "not tried yet"})
                                     for w in NAMES}, indent=1))

    try:
        if cmd == "walk" or dry:
            walk(arm, eyes, order[0] if order else "H1")
            save()
        if cmd in ("well", "ring"):
            if not arm.tied:
                raise RuntimeError("no tie to the cameras: run `walk` first")
            refused = 0
            for w in order:
                if not reachable(p, w)[0]:
                    continue
                try:
                    results[w] = {"well": w, **into(arm, eyes, w, memory)}
                    refused = 0
                except RuntimeError as e:  # a refused move ends this well (into lifts first); two in a row end the run
                    refused += 1
                    print(f"{w}: {e}", flush=True)
                    results[w] = {"well": w, "reached": False, "reason": f"refused: {e}"}
                    if refused >= 2:
                        raise
                save()
    finally:
        save()
        if dry:
            servos.report(results, order)


class Sim:
    """Offline robot and cameras: the true tip is the model plus a smooth error (a moved camera frame of ~22 mm,
    plus ~10 mm more at the plate), measurements add 0.3 mm noise and fail one time in eight. Asserts the motion rules on every command."""

    def __init__(self, p: np.ndarray):
        self.p, self.q, self.rng = p, {k: ac.START[k] for k in ARM}, np.random.default_rng(0)
        self.low, self.n, self.t, self.lo, self.hi = math.inf, 0, 0.0, dict(self.q), dict(self.q)
        self.target_xy = None
        self.parking, self.bg_at = False, None

    def true(self, q: dict | None = None) -> np.ndarray:
        q = q or self.q
        bias = np.array([2.5, -1.5, 1.5] if FIT_MODEL.exists() else [18.0, -12.0, 5.0]) \
            + 0.12 * (q[1] - 497) * np.array([-0.8, 0.1, -0.4]) + 0.05 * (q[2] - 420) * np.array([0.5, 0.5, -1.0])
        return tip(self.p, q) + bias

    def read(self) -> dict:
        return dict(self.q)

    def send(self, q: dict, ms: int) -> None:
        assert set(q) == set(ARM), q
        assert all(abs(q[k] - self.q[k]) <= STEP for k in ARM), (self.q, q)
        assert inside(q) or self.parking, q
        assert ms >= MIN_MS
        self.q, self.n, self.t = dict(q), self.n + 1, self.t + ms / 1000
        self.lo = {k: min(self.lo[k], q[k]) for k in ARM}
        self.hi = {k: max(self.hi[k], q[k]) for k in ARM}
        t = self.true()
        h = TOP_Z - t[2]
        if h < 0:  # really below the rims: only inside a well, near its centre
            near = min(NAMES, key=lambda w: np.linalg.norm(pr.well(w)[:2] - t[:2]))
            assert np.linalg.norm(pr.well(near)[:2] - t[:2]) <= ENTRY_XY_MM, (near, t)
        if pr.well("A1")[0] - 10 < t[0] < pr.well("A12")[0] + 10 and pr.well("A1")[1] - 10 < t[1] < pr.well("H1")[1] + 10:
            self.low = min(self.low, h) if h >= 0 else self.low

    def dwell(self, s: float) -> None:
        self.t += s

    def now(self) -> float:
        return self.t * 1.0

    # Eyes
    def background(self) -> None:
        self.t += 1.0

    def tip(self, expect, within, label) -> np.ndarray | None:
        self.t += 2.0
        t = self.true() + self.rng.normal(0, 0.3, 3)
        if self.rng.random() < 0.125 or np.linalg.norm(t - expect) > within:
            return None
        return t

    def which_well(self, rim_z: float, target: str):
        t = self.true()
        if rim_z - t[2] > -1.0:
            return None
        near = min(NAMES, key=lambda w: np.linalg.norm(pr.well(w)[:2] - t[:2]))
        return near, float(np.linalg.norm(pr.well(near)[:2] - t[:2]))

    def photos(self, stem: str) -> str:
        return f"(dry) {stem}"

    def report(self, results: dict, order: list[str]) -> None:
        got = [w for w in NAMES if results.get(w, {}).get("reached")]
        print(f"dry run: {self.n} commands, ~{self.t / 60:.0f} min; lowest true tip off-entry over the plate "
              f"{self.low:.1f} mm over the highest rim; joints {self.lo} .. {self.hi}")
        print(f"reached {len(got)}/96: {got}")
        for w in NAMES:
            r = results.get(w, {})
            if not r.get("reached"):
                print(f"  {w}: {r.get('reason', 'not tried')}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=("home", "fit", "walk", "well", "ring"))
    ap.add_argument("wells", nargs="*")
    ap.add_argument("--dry", action="store_true", help="simulated arm and cameras, no robot")
    args = ap.parse_args()
    run(args.cmd, [w.upper() for w in args.wells], args.dry)


if __name__ == "__main__":
    main()
