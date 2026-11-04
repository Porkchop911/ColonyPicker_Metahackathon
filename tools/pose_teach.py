#!/usr/bin/env python3
"""Teach poses with the arm powered: type absolute (or relative) pulses for several servos at once, see commanded and
read-back after every move, save named poses. Poses read back from the limp arm under a hand landed 10-20 mm off when
replayed (sag + backlash, 2026-10-31); a pose saved here is where the arm holds itself.

Commands (servos 1 pan, 2 shoulder, 3 elbow, 4 wrist; roll 5 and gripper 10 are shown, never commanded):
    1 612 2 460        absolute: servo 1 to 612 and servo 2 to 460 (any number of pairs)
    +3 10   -2 5       relative: servo 3 up 10, servo 2 down 5 (may be mixed with absolute pairs)
    save H1            store this pose (commanded and read-back) as H1 in evidence/tip/powered_poses.json
    go H1              move to a saved pose
    photo H1           both cameras' frames to evidence/tip/teach_H1.top.jpg / .side.jpg
    show               read-back again
    q                  quit (the arm holds where it is)

Every move goes through well_reach.jgo: <= 6 pulses per joint per command at 25 ms per pulse, joints 1-4 only,
targets 0-1000.

    <venv python> tools/pose_teach.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import well_reach as wr  # noqa: E402

ARM = (1, 2, 3, 4)
POSES = Path("evidence/tip/powered_poses.json")


def parse(words: list[str], q: dict) -> dict:
    """Target pose from 'servo value' pairs: '2 460' absolute, '+2 5' / '-2 5' relative."""
    if len(words) % 2:
        raise ValueError("pairs of servo and value, e.g. '1 612 2 460' or '+3 10'")
    out = dict(q)
    for s, v in zip(words[::2], words[1::2]):
        k, val = int(s.lstrip("+-")), int(v)
        if k not in ARM:
            raise ValueError(f"servo {k}: only 1-4 are moved here")
        out[k] = q[k] + (val if s.startswith("+") else -val if s.startswith("-") else 0) if s[0] in "+-" else val
        if not 0 <= out[k] <= 1000:
            raise ValueError(f"servo {k} to {out[k]}: outside 0-1000")
    return out


def show(servos, q: dict) -> None:
    r = servos.read()
    print("  servo      " + "  ".join(f"{k:>5}" for k in (1, 2, 3, 4, 5, 10)))
    print("  commanded  " + "  ".join(f"{q.get(k, '-'):>5}" for k in (1, 2, 3, 4, 5, 10)))
    print("  read-back  " + "  ".join(f"{r[k]:>5}" for k in (1, 2, 3, 4, 5, 10)), flush=True)


def main(servos=None, lines=None) -> None:
    servos = servos or wr.Servos()
    r = servos.read()
    q = {k: min(max(r[k], 0), 1000) for k in ARM}  # hold where it is (a read past an end clamps, as home does)
    if hasattr(servos, "td"):
        servos.td._sent.update(q)
    servos.send(q, 1000)
    show(servos, q)
    poses = json.loads(POSES.read_text()) if POSES.exists() else {}
    eyes = None
    for line in lines if lines is not None else iter(lambda: input("> "), None):
        words = line.split()
        if not words:
            continue
        try:
            if words[0] == "q":
                break
            if words[0] == "show":
                pass
            elif words[0] == "save" and len(words) == 2:
                poses[words[1]] = {"cmd": q, "read": {k: v for k, v in servos.read().items()}}
                POSES.write_text(json.dumps(poses, indent=1))
                print(f"  saved {words[1]}: {q}")
            elif words[0] == "go" and len(words) == 2:
                q = wr.jgo(servos, {int(k): v for k, v in poses[words[1]]["cmd"].items()}, q)
            elif words[0] == "photo" and len(words) == 2:
                import cv2
                from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
                eyes = eyes or (Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER))
                for cam, name in zip(eyes, ("top", "side")):
                    cv2.imwrite(f"evidence/tip/teach_{words[1]}.{name}.jpg", cam.frame())
                print(f"  photos evidence/tip/teach_{words[1]}.top.jpg / .side.jpg")
            else:
                q = wr.jgo(servos, parse(words, q), q)
            show(servos, q)
        except (ValueError, KeyError, AssertionError) as e:
            print(f"  not done: {e}", flush=True)
    print("  the arm holds where it is")


if __name__ == "__main__":
    main()
