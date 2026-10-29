#!/usr/bin/env python3
"""Replay a recorded model_jab run on the desktop: saved camera frames in, commands recorded, no robot, no lamps.

model_jab.py runs unchanged; its robot and camera I/O is swapped for the run's saved frames, the arm is a fake
that holds whatever was commanded (no sag, no lag) and sleeps are skipped. SAM 3.1 runs for real. The replayed
log is compared row by row with the recorded one; the first divergence is where a code change (or SAM) changed
a decision.

    <hackathon venv python> tools/replay_run.py evidence/descend/20261029T125405 --out DIR [-- model_jab args]

Runs from before 2026-10-29 13:00 saved only some UGREEN frames: they replay to the first hover. Later runs save
every UGREEN frame (ug_NNN.jpg) and the first arm pose (`pose0`).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import types
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import model_jab as mj  # noqa: E402

COMPARE = ("matched", "target_mm", "tip_mm", "err_mm", "planned", "dot_minus_landing_mm", "lam_mm", "height_mm",
           "wrist_land", "landing_mm", "fix_mm")


class OutOfFrames(RuntimeError):
    pass


def frames(run: Path, log: list[dict]) -> tuple[list[Path], list[Path], list[Path]]:
    """(arm, UGREEN, side) frames in the order the run read them."""
    arm = [run / "sam_arm_tip.png"] + sorted(run.glob("[0-9][0-9][0-9]_arm.png"))
    ug = sorted(run.glob("ug_[0-9][0-9][0-9].jpg"))
    if not ug:  # older run: ug_pre was never saved (the cube depth path fails on it either way), hover only the last
        ug = [run / "sam_ugreen0_cube.png", run / "sam_ugreen0_cube.png", run / "sam_ugreen.png", run / "hover_ugreen.jpg"]
    side = [run / f"side_wrist_{r['wrist']}.jpg" for r in log if r["phase"] == "wrist"]
    side += [run / f"side_contact_{r['extra']:02d}.jpg" for r in log if r["phase"] == "contact-step"]
    return arm, [p for p in ug if p.exists()], [p for p in side if p.exists()]


def replay(run: Path, out: Path, pose0: dict[int, int], extra: list[str]) -> list[dict]:
    log = json.loads((run / "log.json").read_text())
    arm, ug, side = frames(run, log)
    known = run.parent / "ugreen_dots.json"  # model_jab reads the notebook's dots from --out; the latest set, not the run's own
    if known.exists():
        (out / "ugreen_dots.json").write_text(known.read_text())
    state = dict(pose0)
    sent: list[dict] = []

    def serve(seq: list[Path], what: str):
        it = iter(seq)

        def next_frame(*_a, **_k) -> np.ndarray:
            try:
                return cv2.imread(str(next(it)))
            except StopIteration:
                raise OutOfFrames(f"replay: out of recorded {what} frames") from None
        return next_frame

    class FakeCamera:
        def __init__(self, index: int = mj.Camera.__init__.__defaults__[0], settle_s: float = 0.0):
            self.frame = serve(ug if index != mj.SIDE else side, "UGREEN" if index != mj.SIDE else "side")

    def send(target: dict, ms: int = 0) -> None:
        state.update({int(k): int(v) for k, v in target.items()})
        sent.append(dict(target))

    mj.read = lambda: dict(state)
    mj.send = send
    mj.drive = lambda *a, **k: sent.append({"drive": a})
    mj.Camera = FakeCamera
    mj.tt.snap_arm = serve(arm, "arm")
    mj.at.auto_light = lambda: [0, 254]
    mj.time = types.SimpleNamespace(sleep=lambda s: None, time=time.time, monotonic=time.monotonic,
                                    strftime=time.strftime)
    sys.argv = ["model_jab.py", "--out", str(out), *(extra or ["--lam", "190", "--jab"])]
    try:
        mj.main()
    except OutOfFrames as error:
        print(error)
    except Exception as error:  # the recorded run may have aborted the same way
        print("replay raised:", repr(error))
    newest = max(out.iterdir(), key=lambda p: p.stat().st_mtime)
    return json.loads((newest / "log.json").read_text())


def compare(gold: list[dict], new: list[dict]) -> None:
    """Rows in order of phase; numbers compared field by field; stops at the first phase mismatch."""
    skip = {"light", "pose0", "dots-filled", "retie-vote"}  # a filled dot is SAM missing one, judged by the rows after it
    g = [r for r in gold if r["phase"] not in skip]
    n = [r for r in new if r["phase"] not in skip]
    worst: dict[str, float] = {}
    for i, (a, b) in enumerate(zip(g, n)):
        if a["phase"] != b["phase"]:
            print(f"DIVERGED at row {i}: recorded {a['phase']} vs replay {b['phase']}")
            print("  recorded:", {k: a[k] for k in a if k in COMPARE})
            print("  replay:  ", {k: b[k] for k in b if k in COMPARE})
            break
        for k in COMPARE:
            if k in a and k in b and a[k] is not None and b[k] is not None:
                x, y = a[k], b[k]
                if isinstance(x, dict):
                    x, y = [x[q] for q in sorted(x)], [y.get(q, np.nan) for q in sorted(x)]
                d = float(np.nanmax(np.abs(np.subtract(x, y, dtype=float)))) if np.size(x) == np.size(y) else np.inf
                worst[k] = max(worst.get(k, 0.0), d)
    else:
        print(f"no divergence over {min(len(g), len(n))} rows (recorded {len(g)}, replay {len(n)})")
    print("largest field difference:", {k: round(v, 3) for k, v in worst.items()})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path, required=True, help="an existing folder; each replay adds a timestamped run folder")
    ap.add_argument("extra", nargs="*", help="model_jab arguments after --")
    ap.add_argument("--compare", type=Path, help="only compare this replayed log.json with the recorded one")
    args = ap.parse_args()
    gold = json.loads((args.run / "log.json").read_text())
    if args.compare:
        return compare(gold, json.loads(args.compare.read_text()))
    rows = {r["phase"]: r for r in reversed(gold)}
    if "pose0" in rows:
        pose0 = {int(k): v for k, v in rows["pose0"]["pose"].items()}
    else:  # older run: start from the logged start pose, then shift the elbow by what the replay's band loop added
        start = {int(k): v for k, v in rows["start"]["pose"].items()}
        first = replay(args.run, args.out, start, args.extra)
        got = next((r for r in first if r["phase"] == "start"), None)
        if got is None:
            raise SystemExit("replay did not reach 'start': cannot recover the first arm pose")
        pose0 = {k: v - (int(got["pose"][str(k)]) - start[k]) for k, v in start.items()}
    print("pose0", pose0)
    compare(gold, replay(args.run, args.out, pose0, args.extra))


if __name__ == "__main__":
    main()
