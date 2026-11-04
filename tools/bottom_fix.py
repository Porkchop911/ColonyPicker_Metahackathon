"""Shift well poses by bottom-photo readings. Input JSON {well: [ex, ey]} = display px the tip sits past the marker on entry_sheet.py
crops (1.168 display px per frame px). Appends one pose per well. usage: tools/bottom_fix.py READINGS.json [GAIN]"""
import json
import sys
import time

import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402

S = 'evidence/tip'
R = json.load(open(sys.argv[1]))
gain = float(sys.argv[2]) if len(sys.argv) > 2 else 0.8
P = json.load(open(f'{S}/well_poses.json'))
for w, e in R.items():
    h, en = P[w][-1]['hover'], P[w][-1]['entry']
    dx, dp = gain * np.linalg.lstsq(a.image_jacobian(tuple(h)), -np.array(e, float) / 1.168, rcond=None)[0]
    P[w].append({'hover': [h[0] + dx, h[1], h[2] + dp], 'entry': [en[0] + dx, en[1], en[2] + dp], 'gate_xy': None,
                 'when': time.strftime('%Y-%m-%d %H:%M:%S'), 'by': 'bottom photo'})
    print(w, round(float(dx), 2), round(float(dp), 2))
json.dump(P, open(f'{S}/well_poses.json', 'w'), indent=1)
