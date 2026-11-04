"""Needle-down straight line from the arm's current pose: x, z model mm changed in 3 mm pieces, pan held unless --pan.
usage: <venv python> tools/reach_out.py DX DZ [--pan P] [--tag T] [--dry]. Refuses under 40 mm over the paper (a1lib model).
Saves evidence/tip/ro_<tag>_{top,side}.jpg after the move. For finding the needle after the rig moved."""
import math
import sys
import time

import cv2

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
import well_reach as wr  # noqa: E402
from reach import ik  # noqa: E402

DRY = '--dry' in sys.argv
dx, dz = float(sys.argv[1]), float(sys.argv[2])
opt = lambda n, d: type(d)(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d
tag = opt('--tag', 'x')
MIN_H = 40.0
OUT_OF_VIEW = {1: 500, 2: 450, 3: 207, 4: 468}  # bg_refresh's


class Fake:  # --dry: prints the extremes, moves nothing
    q = {1: 583, 2: 162, 3: 429, 4: 530}

    def read(self):
        return dict(self.q)

    def send(self, q, ms):
        assert all(abs(q[k] - self.q[k]) <= wr.STEP for k in wr.ARM), (self.q, q)
        self.q = dict(q)

    def dwell(self, s):
        pass


def cost(v, now):
    try:
        q = ik(*v)
    except AssertionError:
        return 1e9
    return sum((q[k] - now[k]) ** 2 for k in (2, 3, 4))


s = Fake() if DRY else wr.Servos()
r = s.read()
now = {k: r[k] for k in wr.ARM}
a.assert_unfolded(now)
fit = lambda p: min(((u / 2, w / 2) for u in range(300, 760) for w in range(80, 400)), key=lambda v: cost(v, p))
x, z = fit(now)
if cost((x, z), now) >= 50:  # after a reboot the wrist sat 24 pulses off (sum 1165, not 1125): the raised vertical pose first
    near = {**OUT_OF_VIEW, 1: now[1]}
    assert max(abs(now[k] - near[k]) for k in now) <= 60, f'arm {now} is not on the needle-down model'
    print('to the raised vertical pose', near, flush=True)
    now = wr.jgo(s, near, now, dwell=0.5)
    x, z = fit(now)
    assert cost((x, z), now) < 50, now
pan = opt('--pan', float(now[1]))
n = max(1, math.ceil(max(abs(dx), abs(dz)) / 3.0))
print(f'at x {x} z {z} ({z - a.Z_PAPER:.0f} mm over the paper, model) {now}; to x {x + dx} z {z + dz} pan {pan}, {n} pieces', flush=True)
assert z + dz - a.Z_PAPER >= MIN_H, 'too low'
q, plan = now, []
for i in range(1, n + 1):
    t = i / n
    nq = {1: round(now[1] + (pan - now[1]) * t), **ik(x + dx * t, z + dz * t)}
    assert 0 <= nq[1] <= 1000 and nq[2] < 600 and nq[4] >= 300 and 0 <= nq[3] <= 1000, nq
    plan.append(nq)  # the whole path checked before the first piece moves
for nq in plan:
    q = wr.jgo(s, nq, q, dwell=0.0)
print('now', q, 'read', None if DRY else s.read(), flush=True)
if not DRY:
    from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER
    time.sleep(1.0)
    for n_, c in (('top', Camera(OVERHEAD)), ('side', Camera(SIDE, paper_target=SIDE_PAPER))):
        cv2.imwrite(f'evidence/tip/ro_{tag}_{n_}.jpg', c.frame()); c.close()
