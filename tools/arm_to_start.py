"""Step the arm to the start pose, elbow 190 (12 interpolated moves, stops on a lagging joint).

    python tools/arm_to_start.py OUT_DIR
"""
import sys, time
sys.path.insert(0, 'tools')
import cv2, tip_triangulate as tt
from tip_descend import read, send
out = sys.argv[1]
goal = {1: 497, 2: 420, 3: 190, 4: 269, 5: 502}  # tip ~45-70 mm up; model_jab's band loop lowers it, measured, to ~160
# (from 265 it took 10-11 views, ~19 s, 2026-10-29)
q0 = read(); print("from", q0)
q0 = {k: min(max(q0[k], 0), 1000) for k in goal}  # the bridge takes 0-1000; the elbow read -54
send(q0, ms=1000); time.sleep(1.5); print("holding", read(), flush=True)
N = 12
for i in range(1, N + 1):
    step = {k: int(round(q0[k] + (goal[k] - q0[k]) * i / N)) for k in goal}
    send(step, ms=700); time.sleep(1.0)
    now = read()
    lag = {k: now[k] - step[k] for k in goal}
    print(i, step, "lag", lag, flush=True)
    if max(abs(v) for k, v in lag.items() if k != 2) > 25 or abs(lag[2]) > 40:
        send({k: now[k] for k in goal}, ms=300)  # hold where it is
        raise SystemExit(f"stopped: joint lagging {lag} (blocked?)")
time.sleep(1.5); print("at", read())
cv2.imwrite(f"{out}/arm_at_start.jpg", tt.snap_arm(1))
