"""Into a well in small steps from its learnt pose, frames at each height, back up. No camera steering.
usage: tools/step_in.py WELL [--dry] [--dx MM] [--dp PULSES] [--to H] [--tag T]. Heights in mm over the rim (negative = under it)."""
import json
import sys
import time

import cv2

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
from reach import ik  # noqa: E402

S = 'evidence/tip'
DRY = '--dry' in sys.argv
w = sys.argv[1]
opt = lambda n, d: type(d)(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
p = json.load(open(f'{S}/well_poses.json'))[w][-1]['hover']
z_off = json.load(open(f'{S}/well_z_offset.json'))['z_off']
x, pan = p[0] + opt('--dx', 0.0), p[2] + opt('--dp', 0.0)
end = opt('--to', -3.0)
assert end >= -3.0, end
tag = opt('--tag', w)
at = lambda h: (x, p[1] + h - 5.0 + z_off, pan)
HS = [h for h in (15, 10, 6, 4, 2, 0, -1, -2, -3) if h >= end]
qs = [{1: round(c[2]), **ik(c[0], c[1])} for c in (at(h) for h in HS)]
for q in qs:
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300 and q[2] < 600, q
print(w, 'x', round(x, 1), 'pan', round(pan, 1), flush=True)
if DRY:
    print(qs[-1]); sys.exit()
import well_reach as wr  # noqa: E402
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402
wr.DWELL_S = 0.25
s = wr.Servos()
r = s.read()
st = {k: r[k] for k in wr.ARM}
a.assert_unfolded(st)
cams = {'top': Camera(OVERHEAD), 'side': Camera(SIDE, paper_target=SIDE_PAPER)}


def go(q):
    global st
    above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != st[1] else {})}
    wr.jgo(s, above, st, dwell=0.2, fast=True)
    wr.jgo(s, q, above, fast=True)
    st = q


for h, q in zip(HS, qs):
    go(q)
    time.sleep(0.6)
    t, b = cams['top'].frame(), cams['side'].frame()
    cv2.imwrite(f'{S}/si_{tag}_{h}_top.jpg', t); cv2.imwrite(f'{S}/si_{tag}_{h}_side.jpg', b)
    print('h', h, q, 'read', s.read(), flush=True)
for h in (10, 15):
    go({1: round(pan), **ik(*at(h)[:2])})
print('up', flush=True)
