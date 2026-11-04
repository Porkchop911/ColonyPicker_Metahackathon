#!/usr/bin/env python3
"""Raster-scan the plate by direct servo steps and camera-measured tip positions.

Run it with the hackathon venv python, for example:
    python tools/raster_scan.py --points 100

Method (no arm model, no IK solver):
- Start from the hand-set pose; lift servo 2 until the cameras measure the tip
  clearly above the plate rim (height_mm >= CLEAR_MM).
- Inner raster: step servo 1 across its range. At each stop, descend servo 2
  in small measured steps until the camera-measured tip height reaches the
  touch band (TOUCH_MM over the rim plane, i.e. at/just inside the opening).
  Record pose, measured tip (sheet mm), height over rim, nearest well and
  off-centre distance, and save the overhead picture.
- Lift back to clearance, step servo 1, repeat. At the end of a row, shift
  servos 3 and 4 to a new row pose and sweep servo 1 back the other way.

Heights are camera measurements, never servo inference: z from triangulating
the needle ends in both cameras (tip_triangulate.meet); sheet z points down,
paper at z=0, so height over the rim = rim_z - tip_z at the nearest well.
A descent step whose measured height is already below the rim plane away
from a well centre aborts that point (never push blind into plastic).

Outputs:
- evidence/tip/raster_100.json : one record per touch point
- evidence/tip/raster_100.csv  : same, flat
- evidence/tip/raster_frames/  : overhead frame per touch point
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import arm_calibrate as ac  # noqa: E402
import plate_rig as pr  # noqa: E402
import tip_triangulate as tt  # noqa: E402
import well_reach as wr  # noqa: E402

STEP_PULSES = 6          # one joint command piece (well_reach.jgo)
DESCEND_STEP = 4         # servo-2 pulses per measured descent step (far from the plate)
CLEAR_MM = 12.0          # tip this high over the rim counts as clear
TOUCH_MM = 0.5           # descend until measured height <= this (over rim)
MAX_DESCEND = 90         # servo-2 pulses per point descent budget
LIFT_MARGIN = 8          # extra servo-2 pulses up past first clear reading
LIVE_BAND_MM = 45.0      # candidates this far from the previous fix are ghosts
SERVO1_STEP = 8          # raster step along servo 1
SERVO1_RANGE = (520, 720)  # pan range covering the plate (well_reach ENVELOPE)
ROW_SHIFTS = [(0, 0), (14, -14), (-14, 14), (28, -28), (-28, 28),
              (42, -42), (-42, 42), (56, -56), (-56, 56), (70, -70)]
NAMES = [f"{r}{c}" for r in "ABCDEFGH" for c in range(1, 13)]
WELLS = {w: pr.well(w) for w in NAMES}
OUT_JSON = Path("evidence/tip/raster_100.json")
OUT_CSV = Path("evidence/tip/raster_100.csv")
FRAMES = Path("evidence/tip/raster_frames")


LAST_TIP: np.ndarray | None = None


def measure_tip(eyes: wr.Eyes) -> tuple[np.ndarray, float] | None:
    """Triangulated needle tip in sheet mm and the ray miss distance (mm).

    First fix picks the candidate lowest in both images (the tip, as
    Eyes.free_tip does); later fixes pick the candidate nearest the previous
    fix, which rejects the background ghost at the pose the background
    frames were taken from.
    """
    global LAST_TIP
    for _ in range(3):
        sf, tf = eyes.side.frame(), eyes.top.frame()
        sides = wr.needle_ends(sf, eyes.bg_side)
        tops = ac.needle_ends(tf, ac.measure.top_bg)
        cands = []
        for s in sides:
            for t in tops:
                p3, miss = tt.meet(tt.ray(t, *eyes.urig), tt.ray(s, *eyes.srig))
                if miss <= wr.MISS_MM and -p3[2] >= 0.0:
                    if LAST_TIP is not None and np.linalg.norm(p3 - LAST_TIP) > LIVE_BAND_MM:
                        continue  # background/lighting ghost, not the needle that moved
                    cands.append((p3, miss, t, s))
        if cands:
            if LAST_TIP is None:
                p3, miss, _, _ = max(cands, key=lambda c: c[2][1] + c[3][1])
            else:
                p3, miss, _, _ = min(cands, key=lambda c: float(np.linalg.norm(c[0] - LAST_TIP)))
            LAST_TIP = np.array(p3, float)
            return p3, miss
    return None


def height_over_rim(tip: np.ndarray) -> tuple[str, float, float]:
    """Nearest well name, tip height over that well's rim (mm), xy off-centre."""
    dists = [(float(np.linalg.norm(tip[:2] - w[:2])), name) for name, w in WELLS.items()]
    dist, name = min(dists)
    return name, float(WELLS[name][2] - tip[2]), dist


def save_top(eyes: wr.Eyes, stem: str) -> str:
    FRAMES.mkdir(parents=True, exist_ok=True)
    path = FRAMES / f"{stem}.jpg"
    cv2.imwrite(str(path), eyes.top.frame())
    return str(path)


def joints_within(q: dict) -> bool:
    return (SERVO1_RANGE[0] <= q[1] <= SERVO1_RANGE[1]
            and 150 <= q[2] <= 340 and 80 <= q[3] <= 320 and 580 <= q[4] <= 800)


def lift_until_clear(servos: wr.Servos, eyes: wr.Eyes, q: dict) -> dict:
    """Lift on servo 2 (tip mostly rises with it here), bounded and checked.

    Detector reliability collapses when the pose changes a lot against one
    background (lighting ghost at the lattice near servo 2 = 380+ made the
    measured height read as 60 mm UNDER the rim from a clear pose), so this
    uses measured heights only to stop early: it raises in steps, and if a
    measurement says clear (or reads implausibly deep — a ghost: physical
    clearance can only have grown while lifting), it finishes the lift,
    takes FRESH background frames, and demands a measured clear reading
    against them. Bounded by LIFT_MAX pulses from wherever it started."""
    eyes.background()
    start2 = q[2]
    while q[2] - start2 < 130:
        q = wr.jgo(servos, {**q, 2: q[2] + DESCEND_STEP}, q)
        got = measure_tip(eyes)
        if got is not None:
            tip, _ = got
            _, h, off = height_over_rim(tip)
            if h >= CLEAR_MM:
                break
            if h < -12.0 and off > 12.0:  # ghost: deep + far off any well while lifting
                break
    if q[2] - start2 >= 130:
        raise RuntimeError(f"lift did not clear in 130 pulses from {start2}")
    q = wr.jgo(servos, {**q, 2: q[2] + LIFT_MARGIN}, q)
    eyes.background()  # fresh pair at the clear pose: verify before any sideways move
    got = measure_tip(eyes)
    if got is None:
        # only the arm itself changed against the fresh background; the tip
        # is at the same place, take one reference fix and check height again
        got = measure_tip(eyes)
    if got is None:
        raise RuntimeError(f"no measured tip at lift top {q}: not moving sideways")
    tip, _ = got
    _, h, _ = height_over_rim(tip)
    if h < 6.0:
        raise RuntimeError(f"measured tip only {h:.1f} mm over rim at lift top {q}: not moving sideways")
    return q


def descend_to_touch(servos: wr.Servos, eyes: wr.Eyes, q: dict) -> tuple[dict, dict | None]:
    """Lower servo 2 in measured steps to the touch band; record the touch."""
    spent = 0
    misses = 0
    while spent <= MAX_DESCEND:
        got = measure_tip(eyes)
        if got is None:
            misses += 1
            if misses >= 4:  # detector dead at this pose: stop here, stay high
                return q, None
            continue
        misses = 0
        tip, miss = got
        well, h, off = height_over_rim(tip)
        if h <= -2.5 and off > 3.0:
            return q, {"abort": f"{h:.1f} mm below rim, {off:.1f} mm off {well}"}
        if h <= TOUCH_MM:
            return q, {"tip": tip, "miss": miss, "well": well, "height": h, "off": off}
        step = DESCEND_STEP if h > 8.0 else 2 if h > 3.0 else 1  # creep the last millimetres
        if q[2] <= 150:  # keep servo 2 in its sane band
            return q, None
        q = wr.jgo(servos, {**q, 2: q[2] - step}, q)
        spent += step
    return q, None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--points", type=int, default=100)
    args = ap.parse_args()

    servos = wr.Servos()
    eyes = wr.Eyes(time.strftime("%Y%m%dT%H%M%S") + "_raster100")
    now = servos.read()
    q = {k: int(now[k]) for k in wr.ARM}
    base = dict(q)
    print(f"start readback {q}", flush=True)

    # Background first (the arm starts clear of the plate), then lift by
    # measurement; continuity picking in measure_tip rejects the ghost.
    eyes.background()
    q = lift_until_clear(servos, eyes, q)
    got = measure_tip(eyes)
    if got is None:
        raise RuntimeError("no tip measured after background: not rastering")
    print(f"clear at {q}, tip {np.round(got[0], 1).tolist()}", flush=True)

    rows = []
    for d3, d4 in ROW_SHIFTS:
        row = {**base, 3: base[3] + d3, 4: base[4] + d4}
        if joints_within(row):
            rows.append(row)
    print(f"{len(rows)} row poses", flush=True)

    records = []
    idx = 0
    for ri, row in enumerate(rows):
        if idx >= args.points:
            break
        # Move to the row pose while clear, re-lifting after the 3/4 shift.
        q = wr.jgo(servos, {**q, 3: row[3], 4: row[4]}, q)
        q = lift_until_clear(servos, eyes, q)
        lo, hi = SERVO1_RANGE
        sweep = range(lo, hi + 1, SERVO1_STEP)
        if ri % 2:
            sweep = reversed(list(sweep))
        for s1 in sweep:
            if idx >= args.points:
                break
            q = wr.jgo(servos, {**q, 1: int(s1)}, q)
            q = lift_until_clear(servos, eyes, q)
            q, hit = descend_to_touch(servos, eyes, q)
            rb = servos.read()
            rec = {"i": idx, "row": ri, "pose_cmd": dict(q),
                   "readback": {k: int(rb[k]) for k in (1, 2, 3, 4, 5, 10)},
                   "when": time.strftime("%Y-%m-%dT%H:%M:%S")}
            if hit is None:
                rec["status"] = "no_measure"
            elif "abort" in hit:
                rec["status"] = "abort"
                rec["reason"] = hit["abort"]
                print(f"[{idx}] ABORT {rec['reason']} at {q}", flush=True)
            else:
                stem = f"p{idx:03d}_r{ri}_s1_{q[1]}"
                rec.update({"status": "touch",
                            "tip_sheet_mm": np.round(hit["tip"], 2).tolist(),
                            "ray_miss_mm": round(hit["miss"], 2),
                            "height_over_rim_mm": round(hit["height"], 2),
                            "nearest_well": hit["well"],
                            "off_centre_mm": round(hit["off"], 2),
                            "inside_well": bool(hit["off"] <= 2.5),
                            "photo_top": save_top(eyes, stem)})
                print(f"[{idx}] touch {hit['well']} off {hit['off']:.1f} mm "
                      f"h {hit['height']:+.1f} pose {q}", flush=True)
            records.append(rec)
            OUT_JSON.write_text(json.dumps(records, indent=1))
            idx += 1
            q = lift_until_clear(servos, eyes, q)

    OUT_JSON.write_text(json.dumps(records, indent=1))
    if records:
        keys = sorted({k for r in records for k in r} - {"pose_cmd", "readback"})
        with OUT_CSV.open("w", newline="") as f:
            wtr = csv.writer(f)
            wtr.writerow(["i", "row", "cmd1", "cmd2", "cmd3", "cmd4"] + keys)
            for r in records:
                wtr.writerow([r["i"], r["row"]] + [r["pose_cmd"][k] for k in wr.ARM]
                             + [r.get(k, "") for k in keys])
    print(f"done: {len(records)} points -> {OUT_JSON}", flush=True)


if __name__ == "__main__":
    main()
