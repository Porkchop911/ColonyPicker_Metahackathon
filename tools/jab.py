#!/usr/bin/env python3
"""THE JAB: the needle tip moves --fwd mm forward and --down mm down in one fluid shoulder/elbow/wrist move.

User, 2026-10-27: aimed from height, the dot lies ahead of the tip where its camera ray meets the
paper; the jab goes forward AND down, 10 mm each. This tool practises it in the air: from the current
pose (at least --down + MARGIN_MM up) each practice is one fluid move, measured from frames taken during
the motion: the tip's sheet position from each frame's grid pose (tip depth --lam), unwrapped frame to
frame by whole grid squares (the numbering slips by squares between frames). Three one-joint trials give the
shoulder/elbow/wrist -> (forward, down) Jacobian, every practice refines it (Broyden), and the pose goes back to the
start after each. Pan is never commanded; servo 2 only from --start's fixed value. Writes DIR/jab.json.

    python tools/jab.py --out DIR --start S E W [--fwd 10] [--down 10] [--practice 4] [--lam 206]
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
import aim_touch as am  # noqa: E402
import arm_touch as at  # noqa: E402
import paper_grid as pg  # noqa: E402
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import Camera  # noqa: E402
from tip_descend import read, send  # noqa: E402

# elbow -, wrist + is the jab's way (forward and down); trials go that way, one joint each
SERVOS = (2, 3, 4)          # shoulder, elbow, wrist, moved together (user: the jab is a combination)
# the jab's way: shoulder - (the user's forward/down drops in teleop), elbow -, wrist +; one joint per trial
TRIALS = ((-10, 0, 0), (0, -10, 0), (0, 0, 10))
PRELOAD = (8, 8, -8)        # every return to the start comes from this side, so jab moves never reverse
MOVE_MS = 8000              # slow enough that the tip moves < half a grid square between frames
MARGIN_MM = 8.0             # practise only this much above the paper after the full drop
MAX_PULSES = 30
STRAFE_V, STRAFE_PROBE_S = 0.05, 0.3  # live: base strafe speed (m/s) and probe pulse (0.3 s moved 3.4 mm, plan.md)
STEP_MM = 8.0               # live: most travel per placing move
JUMP_MM = 25.0              # live: a UGREEN tip this far from its prediction is a wrong detection
GAIN = 0.5                  # live: fraction of the placing error corrected per move
PLACE_FLOOR_MM = 3.0        # live: placing never takes the tip lower than this
PLACE_MM = 1.0              # live: tip placed this close to --fwd behind / --down above the dot
JAB_MS = 2500               # live: the jab itself, one fluid move  # 40-pulse moves outran the frame-to-frame unwrapping


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fwd", type=float, default=10.0, help="tip travel forward along the needle, mm")
    ap.add_argument("--down", type=float, default=10.0, help="tip travel down, mm")
    ap.add_argument("--practice", type=int, default=4, help="practice jabs after the two trial moves")
    ap.add_argument("--lam", type=float, default=206.0, help="tip depth along its pixel ray, mm")
    ap.add_argument("--start", type=int, nargs="+", metavar="PULSE",
                    help="start pose pulses (default: read-back, which sinks a few pulses per run)")
    ap.add_argument("--no-shoulder", action="store_true",
                    help="elbow and wrist only, servo 2 never commanded (it would not rise on a 10.85 V battery)")
    ap.add_argument("--live", type=Path, metavar="JAB_JSON",
                    help="the real thing with this learnt jab: find a black dot (UGREEN), place the tip --fwd behind "
                         "and --down above it, jab once, check where it landed, lift")
    ap.add_argument("--dq", type=int, nargs="+", metavar="PULSE",
                    help="repeat this jab command --practice times, no trials, no learning (repeatability)")
    args = ap.parse_args()
    global SERVOS, TRIALS, PRELOAD
    if args.no_shoulder:
        SERVOS, TRIALS, PRELOAD = SERVOS[1:], tuple(t[1:] for t in TRIALS[1:]), PRELOAD[1:]
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log, n = [], [0]

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: (np.round(v, 2).tolist() if isinstance(v, np.ndarray) else v) for k, v in row.items()}, flush=True)

    if args.start:  # go there before the tip pixel is taken
        send(dict(zip(SERVOS, args.start)), ms=1500)
        time.sleep(2.2)
    first = tt.snap_arm(3)
    free = np.array(tt.arm_tip(first))  # tip pixel in free air
    ray = at.unit_ray(free)

    def tip_now() -> tuple[np.ndarray, np.ndarray] | None:
        """(tip in this frame's sheet frame, mm; forward direction on the sheet, unit), or None."""
        n[0] += 1
        img = tt.snap_arm(1)
        cv2.imwrite(str(out / f"{n[0]:04d}_arm.png"), img)
        try:
            rot, t, _ = tm.arm_view(img)
        except RuntimeError:
            return None
        tip = rot.T @ (args.lam * ray - t)
        if -tip[2] < -20:  # a pose putting the tip under the paper is a wrong fit
            return None
        f = (rot.T @ ray)[:2]
        return tip, f / np.linalg.norm(f)

    def first_fix():
        for _ in range(10):  # an unbounded retry hung on 457 frames with too little grid in view
            got = tip_now()
            if got is not None:
                return got
        raise RuntimeError("no grid pose in 10 frames: too little of the sheet in view?")

    rb = read()
    # servo 2 from a fixed value, never re-sent read-back (it reads ~5 low and sinks): pass --start
    start = dict(zip(SERVOS, args.start)) if args.start else {j: rb[j] for j in SERVOS}

    ug = Camera()
    rig = tt.ugreen_rig(ug.frame())
    uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))

    def on_sheet(px, h: float) -> np.ndarray:
        c, r_ = tt.ray(px, uk, ud, ur, ut)
        return (c + r_ * ((-h - c[2]) / r_[2]))[:2]

    def ug_tip(img, other, h: float) -> tuple[np.ndarray, np.ndarray]:
        """(tip on the sheet at height h, the needle's forward direction on the sheet) in a UGREEN frame,
        the needle found as what moved against `other` (ugreen_tip's darkness test fails at dusk)."""
        pu, axis = am.moved_needle(img, other, rig["sheet"], with_axis=True)
        tip = on_sheet(pu, h)
        f = tip - on_sheet(np.array(pu) - 40 * axis, h)
        return tip, f / np.linalg.norm(f)

    last: dict = {}  # the UGREEN tip before / after the last tracked move, its forward direction, height

    def tracked(target: dict, ms: int) -> np.ndarray:
        """Send `target`, follow the tip through the move; (forward, sideways, down) travelled, mm.
        Down from the arm camera's grid pose; forward = horizontal travel seen by the UGREEN, whose
        sheet frame does not slip (the arm grid's y jittered 2-3 mm, wrong-way unwraps added 5 mm)."""
        u0 = ug.frame()
        got = first_fix()
        prev, fwd = got
        h0, acc = -prev[2], np.zeros(2)
        hs = []
        if target is not None:
            send(target, ms=ms)
        else:  # a base strafe instead of a servo move: `ms` < 0 left, > 0 right, |ms| = pulse length
            tm.drive(0.0, STRAFE_V * np.sign(ms), abs(ms) / 1000)
            ms = 0
        end = time.monotonic() + ms / 1000 + 0.8
        while time.monotonic() < end:
            got = tip_now()
            if got is None:
                continue
            d = got[0][:2] - prev[:2]
            acc += d - pg.SQUARE_MM * np.round(d / pg.SQUARE_MM)  # whole-square numbering slips drop out
            prev = got[0]
            hs.append(-prev[2])
        side = np.array([-fwd[1], fwd[0]])
        h1 = float(np.median(hs[-3:])) if hs else -prev[2]  # one bad grid fit read 0.3 mm for ~20 (live 2026-10-27)
        u1 = ug.frame()
        cv2.imwrite(str(out / f"{n[0]:04d}_ug0.jpg"), u0)
        cv2.imwrite(str(out / f"{n[0]:04d}_ug1.jpg"), u1)
        try:
            (t1, f1), (t0, f0) = ug_tip(u1, u0, h1), ug_tip(u0, u1, h0)
            f = (f0 + f1) / np.linalg.norm(f0 + f1)
            along = float((t1 - t0) @ f)  # signed: a horizontal magnitude read elbow - (back, down) as forward
            last.update(t0=t0, t1=t1, f=f, h=h1)
        except RuntimeError as error:
            note(phase="ugreen-miss", error=str(error))
            along = float("nan")
            last.update(t1=None, h=h1)
        return np.array([along, acc @ side, h0 - h1, acc @ fwd])

    def back_to_start() -> None:
        """Back to the start pose from the PRELOAD side: the same backlash state before every move
        (elbow trials after a plain return moved ~0 mm for 6 pulses, live 2026-10-27)."""
        # the shoulder sticks when asked to rise under load (handover); after a jab (-30) it stayed down and
        # the next start was 1.4 mm over the paper (live 2026-10-27): elbow up first to unload it, then the rest
        if 2 in SERVOS:
            rb_ = read()
            send({3: rb_[3] + 40}, ms=1200)
            time.sleep(1.6)
            send({2: start[2] + PRELOAD[0], 3: rb_[3] + 40}, ms=1500)
            time.sleep(2.0)
        send({j: start[j] + d for j, d in zip(SERVOS, PRELOAD)}, ms=2000)
        time.sleep(2.6)
        send(start, ms=800)
        time.sleep(1.4)
        got = read()
        if 2 in SERVOS and abs(got[2] - start[2]) > 6:
            note(phase="shoulder-short", want=start[2], got=got[2])

    def live(jab: dict) -> None:
        """Find a black dot, place the tip --fwd behind and --down above it, jab once, check, lift."""
        jac = np.array(jab["jac"])
        jab_dq = np.array(jab["dq"], int)
        black = am.ug_black(ug.frame(), rig)[0]
        back_to_start()  # with the shoulder unloaded first: it had stayed 19 pulses low after the last jab
        pose = {1: rb[1], **start}  # pan from read-back once; the rest from --start
        note(phase="live", black=len(black), jab_dq=jab_dq)
        if not len(black):
            raise RuntimeError("UGREEN sees no black dot")
        side = lambda f: np.array([-f[1], f[0]])  # noqa: E731
        # sideways by base strafe (user): pan barely moved the tip in the UGREEN (-0.28 mm/pulse)
        tracked(None, int(STRAFE_PROBE_S * 1000))
        if last.get("t1") is None:
            raise RuntimeError("UGREEN did not see the strafe probe")
        strafe_mm = float((last["t1"] - last["t0"]) @ side(last["f"])) / STRAFE_PROBE_S  # mm per second of +strafe
        note(phase="strafe-probe", mm_per_s=strafe_mm)
        if abs(strafe_mm) < 2:
            raise RuntimeError(f"strafe probe moved the tip {strafe_mm * STRAFE_PROBE_S:.1f} mm sideways only")
        land = last["t1"] + args.fwd * last["f"]
        dot = black[int(np.argmin(np.linalg.norm(black - land, axis=1)))]  # chosen once: switching dots mid-way
        tip, fdir, misses = last["t1"], last["f"], 0
        for k in range(24):
            h = last["h"]
            land = tip + args.fwd * fdir  # where the jab will put the tip
            e = dot - land
            ef, es, dh = float(e @ fdir), float(e @ side(fdir)), h - args.down
            note(phase="place", k=k, err_fwd_mm=ef, err_side_mm=es, height_mm=h, dot_mm=dot, seen=misses == 0)
            if misses <= 1 and max(abs(ef), abs(es), abs(dh)) <= PLACE_MM:  # one unseen small move is fine
                break
            # at most STEP_MM of travel per move, scaled as a whole: clipping joints one by one broke the
            # forward/down balance and sank the tip to 3 mm (live 2026-10-27)
            # half the error per move: near the paper the arm moved ~2.5x the model and the forward error
            # flipped +-10 mm every move (live 2026-10-27); a damped loop converges with the model that far off
            scale = GAIN * min(1.0, STEP_MM / max(abs(ef), abs(dh), 1e-9))
            # elbow + wrist only: going back asks the shoulder to rise, it sticks under load and the tip sank
            # 14.7 mm for 2.5 planned (live 2026-10-27); the shoulder moves only in the jab (down, which it follows)
            arm = [i for i, j in enumerate(SERVOS) if j != 2]
            dq = np.zeros(len(SERVOS), int)
            dq[arm] = np.round(np.linalg.solve(jac[:, arm], [ef * scale, dh * scale]))
            sec = float(np.clip(GAIN * es * min(1.0, STEP_MM / max(abs(es), 1e-9)) / strafe_mm, -0.5, 0.5))
            if abs(es) > PLACE_MM and abs(sec) < 0.1:  # under 0.08 s it never strafed and sat 4 mm off
                sec = 0.1 * np.sign(sec)
            if abs(sec) >= 0.08:  # sideways first, by the base, then forward/down by the arm
                tracked(None, int(round(1000 * sec)))
                if last.get("t1") is not None:  # mecanum slip: 8-51 mm/s between runs; learn it as we go
                    seen_side = float((last["t1"] - last["t0"]) @ side(last["f"]))
                    if abs(seen_side) > 0.5:
                        strafe_mm = 0.5 * strafe_mm + 0.5 * seen_side / sec
                    tip = last["t1"]
                else:
                    tip = tip + side(fdir) * strafe_mm * sec
            pose = {**pose, **{j: pose[j] + int(d) for j, d in zip(SERVOS, dq)}}
            tracked(dict(pose), 2500)
            predicted = tip + fdir * float(jac[0] @ dq)
            if last.get("t1") is not None and np.linalg.norm(last["t1"] - predicted) > JUMP_MM:
                note(phase="ugreen-outlier", jump_mm=float(np.linalg.norm(last["t1"] - predicted)))
                last["t1"] = None  # one reading jumped 130 mm (live 2026-10-27)
            if last.get("t1") is not None:
                tip, fdir, misses = last["t1"], last["f"], 0
            else:  # a small move the UGREEN could not see: carry the tip by the prediction
                tip, misses = predicted, misses + 1
                if misses >= 3:
                    raise RuntimeError("UGREEN lost the tip 3 moves running")
            if last["h"] < PLACE_FLOOR_MM:  # the next move corrects a sag; this only keeps it off the paper
                raise RuntimeError(f"placing went down to {last['h']:.1f} mm")
        else:
            raise RuntimeError("could not place the tip over the dot in 24 moves")
        h_before = last["h"]
        m = tracked({**pose, **{j: pose[j] + int(d) for j, d in zip(SERVOS, jab_dq)}}, JAB_MS)  # THE JAB
        miss = (dot - last["t1"]) if last.get("t1") is not None else None  # horizontal, mm, at the jab's end
        note(phase="jab", moved=m, height_before_mm=h_before, height_after_mm=last["h"], miss_mm=miss)
        end = {j: pose[j] + int(d) for j, d in zip(SERVOS, jab_dq)}
        send({**end, 3: end[3] + 40}, ms=1500)
        time.sleep(2.2)
        note(phase="lift", readback=read())

    try:
        if args.live:
            return live(json.loads(args.live.read_text()))
        got = first_fix()
        h = -got[0][2]
        note(phase="start", height_mm=h, pulses=start)
        if h < args.down + MARGIN_MM:
            raise RuntimeError(f"tip {h:.1f} mm up: practising a {args.down:g} mm jab needs >= {args.down + MARGIN_MM:g} mm")
        back_to_start()
        if args.dq:
            for k in range(args.practice):
                m = tracked({j: start[j] + d for j, d in zip(SERVOS, args.dq)}, MOVE_MS)
                note(phase="repeat", k=k, dq=args.dq, moved=m, miss_mm=m[[0, 2]] - [args.fwd, args.down],
                     arm_fwd_mm=m[3])
                back_to_start()
            return
        moves = []
        for dq in TRIALS:
            m = tracked({j: start[j] + d for j, d in zip(SERVOS, dq)}, MOVE_MS)
            note(phase="trial", dq=dq, moved=m)
            if not np.isfinite(m[[0, 2]]).all():
                raise RuntimeError("UGREEN did not see the trial move")
            moves.append((np.array(dq, float), m[[0, 2]]))
            back_to_start()
        dqs, ms_ = np.array([d for d, _ in moves]), np.array([m for _, m in moves])
        jac = np.linalg.solve(dqs, ms_).T  # (forward, down) per (shoulder, elbow, wrist) pulse; trials are one joint each
        note(phase="jacobian", jac=jac, cond=float(np.linalg.cond(jac)))
        goal = np.array([args.fwd, args.down])
        for k in range(args.practice):
            dq = np.clip(np.round(np.linalg.lstsq(jac, goal, rcond=None)[0]), -MAX_PULSES, MAX_PULSES)  # least pulses
            m = tracked({j: start[j] + int(d) for j, d in zip(SERVOS, dq)}, MOVE_MS)
            note(phase="practice", k=k, dq=dq, moved=m, miss_mm=m[[0, 2]] - goal)
            if np.isfinite(m[[0, 2]]).all():  # a UGREEN miss teaches nothing
                jac += np.outer(m[[0, 2]] - jac @ dq, dq) / (dq @ dq)
            back_to_start()
        dq = np.round(np.linalg.lstsq(jac, goal, rcond=None)[0])
        (out / "jab.json").write_text(json.dumps({"fwd": args.fwd, "down": args.down, "jac": jac.tolist(),
                                                 "dq": dq.tolist()}, indent=1))
        note(phase="learnt", jac=jac, dq=dq)
    except BaseException as error:  # any failure, Ctrl-C included, ends back at the start pose
        note(phase="abort", error=repr(error))
        back_to_start()
        raise


if __name__ == "__main__":
    main()
