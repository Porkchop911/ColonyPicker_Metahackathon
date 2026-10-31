"""Teach by hand: the chosen servos go limp, the arm is guided by hand, every readback is logged.
The rough path only: the final needle pick-up stays camera-guided (plan 2026-10-30 B).

    python tools/teach.py OUT_DIR [--joints 1,2,3,4,5,10] [--seconds 120]

Hold the arm before it starts (the shoulder drops when limp). Ends after --seconds or when OUT_DIR/stop
exists; the servos then hold where the hand left them. Writes OUT_DIR/teach.jsonl (every read) and
OUT_DIR/keyframes.json (poses held still >= 1 s).
"""
import argparse, json, sys, time
from pathlib import Path
sys.path.insert(0, 'tools')
from rig_io import servo
from tip_descend import read

ap = argparse.ArgumentParser()
ap.add_argument('out'); ap.add_argument('--joints', default='1,2,3,4,5,10'); ap.add_argument('--seconds', type=float, default=120)
a = ap.parse_args()
out = Path(a.out); out.mkdir(exist_ok=True); joints = [int(j) for j in a.joints.split(',')]
log = open(out / 'teach.jsonl', 'w')
for n in (3, 2, 1):
    print(f'limp in {n}: hold the arm', flush=True); time.sleep(1)
servo({'torque': {str(j): False for j in joints}})
print('limp: guide it', flush=True)
t0 = time.monotonic(); rows = []
try:
    while time.monotonic() - t0 < a.seconds and not (out / 'stop').exists():
        q = read(); t = round(time.monotonic() - t0, 2)
        rows.append((t, q)); log.write(json.dumps({'t': t, 'pulses': q}) + '\n'); log.flush()
        time.sleep(0.2)
finally:
    q = read()
    # hold where the hand left it: a position command re-enables torque; the board's "torque on" (0x0B) left the arm
    # limp (2026-10-30), so it is not sent. Slow: a clamped joint (elbow read -63) still moves to 0
    for _ in range(2):
        servo({'set': {str(j): min(max(q[j], 0), 1000) for j in joints}, 'ms': 1200}); time.sleep(1.3)
    print('holding', q, flush=True)

# keyframes: the arm still (every joint within 3 pulses) for >= 1 s; one per still stretch
keys, start = [], 0
for i in range(1, len(rows) + 1):
    moving = i == len(rows) or max(abs(rows[i][1][j] - rows[start][1][j]) for j in joints) > 3
    if moving:
        if rows[i - 1][0] - rows[start][0] >= 1.0:
            keys.append({'t': rows[start][0], 'pulses': rows[start][1]})
        start = i
json.dump(keys, open(out / 'keyframes.json', 'w'), indent=1)
print(f'{len(rows)} reads, {len(keys)} keyframes -> {out}')
for k in keys:
    print(k['t'], {j: k['pulses'][j] for j in joints})
