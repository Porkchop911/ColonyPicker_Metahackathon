"""Static proof that the needle tip is where it is measured (2026-11-01).

The wrist floor (tools/tip_descend.py WRIST_FLOOR = 400) blocks every pose in well_reach's fit
(wrist 259-279), so the fit cannot run under the restriction. This proof needs no fit pose: it
follows fit()'s own safe prefix (park -> background -> back to the live pose it started from),
with the tool's own Eyes (ac.measure.thin = True, expectation off) and triangulates the bare
re-seated needle at the pose the arm already sits in. No model is trusted: the two cameras'
rays either meet at one point or they do not.

Every send in this process (well_reach.Servos and arm_calibrate.send) is wrapped to refuse any
command below the user's wrist floor, so a bug here cannot send the wrist under 400.
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import arm_calibrate as ac  # noqa: E402
import numpy as np  # noqa: E402
import well_reach as wr  # noqa: E402

FLOOR = 400


class SafeServos(wr.Servos):
    def send(self, q, ms):
        q = {int(k): int(v) for k, v in q.items()}
        if q.get(4, 999) < FLOOR:
            raise RuntimeError(f"proof wrapper refused: wrist {q.get(4)} < floor {FLOOR}")
        super().send(q, ms)


def refuse_low(q, ms=1000):
    if int(q.get(4, 0)) < FLOOR:
        raise RuntimeError(f"proof wrapper refused: wrist {q.get(4)} < floor {FLOOR}")
    return _real_send(q, ms=ms)


_real_send = ac.send
ac.send = refuse_low


def main() -> None:
    servos = SafeServos()
    now = servos.read()
    print(f"[proof] readback at start: {now}", flush=True)
    if now[4] < FLOOR:
        raise RuntimeError(f"arm already below floor: {now}")
    start_q = {k: now[k] for k in wr.ARM}

    eyes = wr.Eyes(time.strftime("%Y%m%dT%H%M%S") + "_proof")
    ac.measure.expect = None
    ac.measure.expect_mm = float("inf")
    print(f"[proof] cameras up, tag={eyes.tag}", flush=True)

    q = start_q
    p3 = None
    target_q = start_q
    if len(sys.argv) > 1:  # e.g. "595,358,209,590": a documented measured pose to prove against
        target_q = {k: int(v) for k, v in zip(wr.ARM, sys.argv[1].split(","))}
        assert target_q[4] >= FLOOR, target_q
    try:
        q = wr.jgo(servos, wr.PARK, q)
        eyes.background()
        print(f"[proof] background taken at PARK {q}", flush=True)
        q = wr.jgo(servos, target_q, q)
        p3 = eyes.free_tip("proof")
        if p3 is None:
            print("[proof] free_tip returned None at the live pose: no triangulation", flush=True)
        else:
            px = getattr(ac.measure, "px", None)
            print(f"[proof] triangulated tip (sheet mm): {np.round(p3, 2).tolist()}", flush=True)
            print(f"[proof] height over paper: {-p3[2]:.1f} mm", flush=True)
            print(f"[proof] pixels: top={px[0] if px else None} side={px[1] if px else None}", flush=True)
        Path("evidence/tip/proof_result.json").write_text(json.dumps({
            "when": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "start_pose": {str(k): v for k, v in start_q.items()},
            "tip_sheet_mm": None if p3 is None else [float(x) for x in p3],
            "height_over_paper_mm": None if p3 is None else float(-p3[2]),
            "pixels": None if p3 is None else getattr(ac.measure, "px", None),
            "frame": "plate_rig",
            "eyes_tag": eyes.tag,
        }, indent=1))
    finally:
        try:
            q = wr.jgo(servos, start_q, q)
        except Exception as e:  # noqa: BLE001
            print(f"[proof] could not return to start pose: {e}", flush=True)
    print(f"[proof] done; readback: {servos.read()}", flush=True)


if __name__ == "__main__":
    main()
