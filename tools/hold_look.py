"""Straight down to the saved 8 mm pose of a colony (colony_plan.json, last entry), frames under several lamp settings,
straight back up. No contact: refuses a target under 7 mm over the agar plan height. usage: tools/hold_look.py [--dry]"""
import json
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
import well_reach as wr  # noqa: E402
from reach import ik  # noqa: E402

DRY = '--dry' in sys.argv
S = 'evidence/tip'
plan = json.load(open(f'{S}/colony_plan.json'))[-1]
assert plan['h'] >= 7.0, plan
x, z, pan = plan['cmd']
LIGHTS = [('b110', (110, 153), (110, 153)), ('b60', (60, 153), (60, 153)), ('b170', (170, 153), (170, 153)),
          ('warm', (110, 400), (110, 400)), ('l13', None, (170, 153)), ('l12', (170, 153), None)]


class Fake:
    q = {1: round(pan), **ik(x, z + 25)}

    def read(self):
        return dict(self.q)

    def send(self, q, ms):
        assert all(abs(q[k] - self.q[k]) <= wr.STEP for k in wr.ARM), (self.q, q)
        self.q = dict(q)

    def dwell(self, s):
        pass


s = Fake() if DRY else wr.Servos()
r = s.read()
now = {k: r[k] for k in wr.ARM}
a.assert_unfolded(now)
hover = {1: round(pan), **ik(x, z + 25)}
assert max(abs(now[k] - hover[k]) for k in now) < 25, (now, hover)  # must start right above it
line = [{1: round(pan), **ik(x, z + h)} for h in range(23, -1, -2)]
line[-1] = {1: round(pan), **ik(x, z)}
for q in line:
    assert q[2] < 600 and q[4] >= 300, q
q = wr.jgo(s, hover, now, dwell=0.2)
for nq in line[:-1]:
    q = wr.jgo(s, nq, q, dwell=0.0)
q = wr.jgo(s, {**line[-1], 2: line[-1][2] + 8}, q, dwell=0.0)
q = wr.jgo(s, line[-1], q)
print('at', q, 'read', None if DRY else s.read(), flush=True)
try:
    if not DRY:
        import hue
        from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER
        for tag, l12, l13 in LIGHTS:
            for n, st in (('12', l12), ('13', l13)):
                hue.set_light(n, st is not None, **({'bri': st[0], 'ct': st[1]} if st else {}))
            time.sleep(1.5)
            for n, c in (('top', Camera(OVERHEAD)), ('side', Camera(SIDE, paper_target=SIDE_PAPER))):
                f = c.frame(); c.close()
                cv2.imwrite(f'{S}/hl_{tag}_{n}.jpg', f)
                print(tag, n, 'median', float(np.median(f)), flush=True)
finally:
    for nq in [{1: round(pan), **ik(x, z + h)} for h in range(2, 26, 2)]:
        q = wr.jgo(s, nq, q, dwell=0.0)
    print('up', q, flush=True)
    if not DRY:
        import hue
        bri, ct = json.load(open(f'{S}/lamps.json'))
        hue.set_light('12', True, bri=bri, ct=ct); hue.set_light('13', True, bri=bri, ct=ct)
