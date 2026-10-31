"""Fold the arm up and back (needle clear of the paper) before power-off; same stepping as arm_to_start.

    python tools/arm_park.py OUT_DIR
"""
import sys, time
sys.path.insert(0, 'tools')
import cv2, tip_triangulate as tt
from tip_descend import read, send
out = sys.argv[1]
goal = {1: 505, 2: 900, 3: 0, 4: 205, 5: 510}  # folded up and back (this morning: 2:952, 3:-64)
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
cv2.imwrite(f"{out}/arm_parked.jpg", tt.snap_arm(1))
