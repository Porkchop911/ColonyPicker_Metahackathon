"""Measure the final drop: overhead tip at 4 and 8 mm over the agar vs where the same model descent lands (2.5 mm in).
From the held pose (evidence/tip/colony_hold.json). Writes evidence/tip/calib_drop_<label>.jpg crops (5x, 120 px around
U V) for reading the tip by eye. usage: <venv python> tools/calib_drop.py U V"""
import json
import sys
import time

sys.path.insert(0, 'tools')
import cv2  # noqa: E402
import a1lib as a  # noqa: E402
import well_reach as wr  # noqa: E402
from reach import ik  # noqa: E402
from rig_io import Camera, OVERHEAD  # noqa: E402

u, v = int(sys.argv[1]), int(sys.argv[2])
st = json.load(open('evidence/tip/colony_hold.json'))
servos = wr.Servos()
r = servos.read()
state = {'q': {k: r[k] for k in wr.ARM}}
a.assert_unfolded(state['q'])
top = Camera(OVERHEAD)
base = list(st['cmd'])
h0 = st['h_over_agar']


def go(h):  # same moves as colony_pick.go: shoulder from 8 above, pan unchanged
    cmd = (base[0], base[1] + (h - h0), base[2])
    q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
    assert all(0 <= x <= 1000 for x in q.values()) and q[4] >= 300 and q[2] < 600, q
    above = {**q, 2: q[2] + 8}
    wr.jgo(servos, above, state['q'])
    wr.jgo(servos, q, above)
    state['q'] = q


def shot(label):
    time.sleep(0.6)
    f = top.frame(average=1, settle=0.0)
    cv2.imwrite(f'evidence/tip/calib_drop_{label}.jpg', cv2.resize(f[v - 60:v + 60, u - 60:u + 60], (600, 600)))
    print('shot', label, state['q'], flush=True)


for h in (4.0, 8.0):
    go(h); shot(f'at{h:.0f}')
    go(-2.5); shot(f'from{h:.0f}_landed')
go(25.0)
print('done, needle up')
