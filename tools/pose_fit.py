"""Fit a smooth pose correction from hover readings and write it to every well. Input: JSON {well: [dpx_x, dpx_y]} = overhead px the
tip must move, read from hover_map crops. usage: tools/pose_fit.py READINGS.json BASEINDEX. Appends one entry per well to well_poses.json.
BASEINDEX: which entry the readings were taken from (0 = seed, -1 = newest)."""
import json
import sys
import time

import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402

S = 'evidence/tip'
read = json.load(open(sys.argv[1]))
base = int(sys.argv[2])
poses = json.load(open(f'{S}/well_poses.json'))
coords = lambda w: ('ABCDEFGH'.index(w[0]), int(w[1:]))
A, b = [], []
for w, e in read.items():
    c = poses[w][base]['hover']
    dx, dp = np.linalg.lstsq(a.image_jacobian((c[0], c[1], c[2])), np.array(e, float), rcond=None)[0]
    r_, c_ = coords(w)
    A.append([1, r_, c_]); b.append([dx, dp])
A, b = np.array(A, float), np.array(b)
sol = np.linalg.lstsq(A, b, rcond=None)[0]
print('fit (1, row, col) -> dx, dpan\n', np.round(sol, 3), '\nresidual dx dp', np.round(b - A @ sol, 2).tolist())
for w, rec in poses.items():
    r_, c_ = coords(w)
    d = np.array([1, r_, c_]) @ sol
    h = rec[base]['hover']; e = rec[base]['entry']
    rec.append({'hover': [h[0] + d[0], h[1], h[2] + d[1]], 'entry': [e[0] + d[0], e[1], e[2] + d[1]], 'gate_xy': None,
                'when': time.strftime('%Y-%m-%d %H:%M:%S'), 'by': 'hover map fit'})
json.dump(poses, open(f'{S}/well_poses.json', 'w'), indent=1)
