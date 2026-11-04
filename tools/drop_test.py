"""The well descent in free air: same reach and heights as a plate entry, pan turned so the tip is over the black mat.
usage: tools/drop_test.py X PAN [--dry]. Prints the triangulated tip at each height. Rim height = plate top."""
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
x, pan = float(sys.argv[1]), float(sys.argv[2])
z_off = json.load(open(f'{S}/well_z_offset.json'))['z_off']
rim_z = -a.pr.well('A12')[2] + a.Z_PAPER + z_off
HEIGHTS = (15, 10, 4, -3, 4, 10, 15)
for h in HEIGHTS:  # the whole path checked first
    q = {1: round(pan), **ik(x, rim_z + h)}
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300 and q[2] < 600, q
assert 0 < a.sheet_of(x, rim_z - 3 - z_off, pan)[1] < 60, 'tip not over the mat strip'
print('rim z', round(rim_z, 1), 'lowest tip over paper', round(-a.pr.well('A12')[2] - 3, 1), flush=True)
if DRY:
    sys.exit()
import well_reach as wr  # noqa: E402
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402
src = open('tools/colony_pick.py', encoding='utf8').read()
ns = {'np': np, 'cv2': cv2}
exec(src[src.index('GAP_ROWS ='):src.index('BAND_MM')], ns)
wr.DWELL_S = 0.25
s = wr.Servos()
r = s.read()
state = {k: r[k] for k in wr.ARM}
a.assert_unfolded(state)
cams = {'top': Camera(OVERHEAD), 'side': Camera(SIDE, paper_target=SIDE_PAPER)}
bg = {n: cv2.imread(f'{S}/st2_{n}.jpg') for n in cams}


def go(cmd_z):
    global state
    q = {1: round(pan), **ik(x, cmd_z)}
    above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != state[1] else {})}
    wr.jgo(s, above, state, dwell=0.2, fast=True)
    wr.jgo(s, q, above, fast=True)
    state = q


for i, h in enumerate(HEIGHTS):
    go(rim_z + h)
    time.sleep(0.8)
    t, b = cams['top'].frame(), cams['side'].frame()
    pred = a.sheet_of(x, rim_z + h - z_off, pan)
    gt, gs = a.project('top', pred), a.project('side', pred)
    tt_ = ns['follow_tip'](t, bg['top'], tuple(gt))
    ts = ns['follow_tip'](b, bg['side'], tuple(gs), along=1.0)
    cv2.imwrite(f'{S}/drop_{i}_top.jpg', t); cv2.imwrite(f'{S}/drop_{i}_side.jpg', b)
    if tt_ is None or ts is None:
        print(i, h, 'tip not found', tt_, ts, flush=True); continue
    X, miss = a.triangulate({'top': np.array(tt_), 'side': np.array(ts)})
    print(i, 'h', h, 'q', state, 'tip', np.round(X, 1), 'miss', round(float(miss), 2), 'model', np.round(pred, 1), flush=True)
go(rim_z + 25)
print('done', flush=True)
