"""Replay of marker_tip on saved frames (2026-10-31, arm over A1 ~26 mm up, both teal bands in view): the tip as picked
by eye in both views (rays meet 0.11 mm). Camera poses saved with the frames.   python tools/test_well_servo.py"""
import json
import sys
import types
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_calibrate as ac  # noqa: E402
import well_servo as ws  # noqa: E402

E = Path("evidence/tip")
EYE_TIP = np.array([26.8, -48.6, -42.4])


def tip(expect, frames=("band_ref_over", "band_ref_bg")):
    rig = lambda p: tuple(np.array(json.loads((E / p).read_text())[q]) for q in ("k", "d", "r", "t"))  # noqa: E731
    frame = lambda f: types.SimpleNamespace(frame=lambda: cv2.imread(str(E / f)))  # noqa: E731
    eyes = types.SimpleNamespace(n=0, tag="test", top=frame(f"{frames[0]}_top.jpg"), side=frame(f"{frames[0]}_side.jpg"),
                                 urig=rig("band_ref_top_pose.json"), srig=rig("band_ref_side_pose.json"),
                                 bg_side=cv2.imread(str(E / f"{frames[1]}_side.jpg")), snapshot=lambda *a: None)
    ac.measure.top_bg = cv2.imread(str(E / f"{frames[1]}_top.jpg"))
    write, cv2.imwrite = cv2.imwrite, lambda *a: True  # no evidence files from a test
    try:
        return ws.marker_tip(eyes, np.asarray(expect, float), 25.0, "test")
    finally:
        cv2.imwrite = write


def main():
    for off in ([0, 0, 0], [5, -5, 10], [-5, 3, -10]):  # expectations around the tip: the lower band, within 1 mm
        t = tip(EYE_TIP + off)
        assert t is not None and np.linalg.norm(t - EYE_TIP) < 1.0, (off, t)
    # expected where the upper band's reading would put the tip: never the upper band taken for the lower
    t = tip(EYE_TIP - [0, 0, ws.BAND_GAP_MM])
    assert t is None or np.linalg.norm(t - EYE_TIP) < 1.0, t
    # 70 mm up over the dark wood at lamp 110 (live D12): the band blurred to hue 24-30; the free tip there met 0.39 mm
    free = np.array([96.3, -33.0, -88.7])
    t = tip(free + [3, -3, 5], ("band_ref_high", "band_ref_high_bg"))
    assert t is not None and np.linalg.norm(t - free) < 1.5, t
    print("ok")


if __name__ == "__main__":
    main()
