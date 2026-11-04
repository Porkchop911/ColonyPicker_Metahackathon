"""Strafe the base until the red cube sits at GOAL_X in the arm camera's start view (before touch 3's strafe)."""
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, sys.argv[1])
import tip_triangulate as tt  # noqa: E402
from tip_motion import drive  # noqa: E402

GOAL_X, V = 490.0, 0.05


def cube_x() -> float:
    hsv = cv2.cvtColor(tt.snap_arm(3), cv2.COLOR_BGR2HSV)
    m = cv2.inRange(hsv, (0, 120, 80), (8, 255, 255)) | cv2.inRange(hsv, (172, 120, 80), (180, 255, 255))
    xs = np.nonzero(m)[1]
    if len(xs) < 500:
        raise RuntimeError("cube not seen")
    return float(np.median(xs))


x = cube_x()
print("cube x", x, flush=True)
sign = 1.0
for i in range(8):
    if abs(x - GOAL_X) <= 8:
        break
    sec = float(np.clip(abs(x - GOAL_X) / 45 * 0.3, 0.15, 0.4))
    drive(0.0, V * sign * np.sign(x - GOAL_X), sec)
    nx = cube_x()
    print(f"pulse {i}: {sign * np.sign(x - GOAL_X):+.0f} {sec:.2f}s  cube x {x:.0f} -> {nx:.0f}", flush=True)
    if abs(nx - GOAL_X) > abs(x - GOAL_X) + 3:
        sign = -sign  # went the wrong way: the sign of vy vs image x is learnt here
    x = nx
print("final cube x", x)
