"""Hover each well H mm over the rim, no contact, and save an overhead crop with the rim centre + parallax marked, so the
tip's offset from the well can be read. usage: tools/hover_map.py A12 C10 ... [--h 2] [--last] [--dry]; poses: seed (first) entry,
--last = the newest. Writes evidence/tip/hm_<well>.jpg (3x, marker = where the tip should be)."""
import json
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
from reach import ik  # noqa: E402

S = 'evidence/tip'
DRY = '--dry' in sys.argv
opt = lambda n, d: type(d)(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
H = opt('--h', 2.0)
assert H >= 2.0
wells = [w for w in sys.argv[1:] if w[0] in 'ABCDEFGH' and w[1:].isdigit()]
poses = json.load(open(f'{S}/well_poses.json'))
z_off = json.load(open(f'{S}/well_z_offset.json'))['z_off']


def q_at(w, h, short=0.0):
    p = poses[w][-1 if '--last' in sys.argv else 0]['hover']
    q = {1: round(p[2] - short), **ik(p[0] - short, p[1] + h - 5.0 + z_off)}
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300 and q[2] < 600, q
    return q


plan = [(w, q_at(w, 15, 12.0), q_at(w, 15), q_at(w, 10), q_at(w, H)) for w in wells]  # all checked first; the path of well_tour --blind: without the 12 mm run-up the tip read 20-40 px off (11-04)
print(len(plan), 'wells', flush=True)
if DRY:
    sys.exit()
import well_reach as wr  # noqa: E402
from rig_io import Camera, OVERHEAD  # noqa: E402
wr.DWELL_S = 0.25
s = wr.Servos()
r = s.read()
st = {k: r[k] for k in wr.ARM}
a.assert_unfolded(st)
top = Camera(OVERHEAD)


def go(q):
    global st
    above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != st[1] else {})}
    wr.jgo(s, above, st, dwell=0.2, fast=True)
    wr.jgo(s, q, above, fast=True)
    st = q


for w, q0, q15, q10, qh in plan:
    go(q0)
    go(q15)
    go(q10)
    go(qh)
    time.sleep(0.8)
    f = top.frame()
    u = a.project('top', a.pr.well(w) + np.array([0, 0, -H]))
    x, y = int(u[0]), int(u[1])
    c = f[y - 80:y + 50, x - 65:x + 65].copy()
    c = cv2.resize(c, None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC)
    cv2.drawMarker(c, (195, 240), (0, 255, 0), cv2.MARKER_CROSS, 24, 1)
    cv2.putText(c, w, (6, 26), 0, .9, (0, 0, 255), 2)
    cv2.imwrite(f'{S}/hm_{w}.jpg', c)
    go(q15)
    print(w, 'done', flush=True)
