"""Lift the needle straight up 30 mm from wherever it is (in a well or hovering), park it at the out-of-view pose
and retake the background frames evidence/tip/st2_{top,side}.jpg that tools/a1_place.py diffs against.
The plate creeps ~1 px over a few entries; its edge then shows in the diff and the tip finder follows it (B12, 2026-11-01)."""
import sys
import cv2
sys.path.insert(0, 'tools')
import well_reach as wr  # noqa: E402
from reach import ik  # noqa: E402
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402

OUT_OF_VIEW = {1: 500, 2: 450, 3: 207, 4: 468}  # pan 500 (user, 11-03 20:17)


def cost(v, now):
    try:
        q = ik(*v)
    except AssertionError:
        return 1e9
    return sum((q[k] - now[k]) ** 2 for k in (2, 3, 4))


s = wr.Servos()
r = s.read()
now = {k: r[k] for k in wr.ARM}
import a1lib  # noqa: E402
a1lib.assert_unfolded(now)
if 8 < max(abs(now[k] - OUT_OF_VIEW[k]) for k in now) <= 60:  # high, near park (an abort stopped the move there,
    wr.jgo(s, OUT_OF_VIEW, now, fast=True)                                # 2026-11-03): no model fit needed, straight on
elif max(abs(now[k] - OUT_OF_VIEW[k]) for k in now) > 60:  # parked out of view (<= 8): just retake
    best = min(((x / 2, z / 2) for x in range(400, 600) for z in range(80, 300)), key=lambda v: cost(v, now))
    assert cost(best, now) < 50, f'arm {now} is not on the needle-down model: lift by hand'
    q = now
    for k in range(1, 16):
        try:
            nq = {1: now[1], **ik(best[0], best[1] + 2 * k)}
        except AssertionError:  # out of reach higher up (after a tour ending over H10, 2026-11-03): park from here
            break
        wr.jgo(s, nq, q, dwell=0.0)  # 2 mm lift pieces back to back: one fluid lift (user, 11-03)
        q = nq
    wr.jgo(s, OUT_OF_VIEW, q, fast=True)
LAMP12 = '--lamp12' in sys.argv  # colony picking: overhead with the side lamp on (the silver needle glints against the black
if LAMP12:                        # mat), side frame with it off (the agar-line measurement was validated in that light)
    import time as _t
    import hue
    import json as _json  # both lamps overhead since 11-03: at 254 SAM loses the dish, the plate washes out
    _bri, _ct = _json.load(open('evidence/tip/lamps.json'))
    hue.set_light('12', True, bri=_bri, ct=_ct); hue.set_light('13', True, bri=_bri, ct=_ct)  # both (user); the side
    _t.sleep(2.0)                                                            # frame lit too: with both off it was black
    c = Camera(SIDE, paper_target=SIDE_PAPER); b = c.frame(); c.close()       # (median V 14) and the agar line failed
for attempt in range(3):  # the overhead autofocus once drifted: background sharpness 5 vs 190-330 (2026-11-02)
    t = Camera(OVERHEAD); a = t.frame(); t.close()
    sharp = cv2.Laplacian(cv2.cvtColor(a[400:1100, 1900:3000], cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    if sharp > 25:  # 4K reads ~53 sharp on the 2026-11-03 layout (1080p: 190-330), blurred ~5
        break
    print(f'overhead frame blurred (sharpness {sharp:.0f}), retaking')
else:
    raise SystemExit('overhead stays blurred: check the camera focus, background NOT retaken')
if not LAMP12:
    import numpy as _np
    for attempt in range(4):  # the side frame once came out black (median V 2 vs 55) and no well height was seen (11-03)
        c = Camera(SIDE, paper_target=SIDE_PAPER); b = c.frame(); c.close()
        if _np.percentile(cv2.cvtColor(b, cv2.COLOR_BGR2HSV)[..., 2], 90) > 100:  # the paper, not the median: the black sheet took that to 21 on a good frame (11-04); a black frame stays under
            break
        print('side frame too dark, retaking')
    else:
        raise SystemExit('side frame stays black: background NOT retaken')
cv2.imwrite('evidence/tip/st2_top.jpg', a)
cv2.imwrite('evidence/tip/st2_side.jpg', b)
print('background retaken at', s.read())

if '--no-plate' in sys.argv:  # colony picking needs no plate: skip the slow re-solve
    sys.exit()

# re-solve the plate from the same frames: entries nudge it ~0.3 mm (A11/B11, 2026-11-02) and the targets come from it
import json  # noqa: E402
import numpy as np  # noqa: E402
import plate_rig as pr  # noqa: E402
old = pr.well('A12')
try:
    top, side, plate = pr.solve(a, b)
    m = np.array(plate['sheet_plate'])  # solve() flips the columns to the print now
    new = m[:3, :3] @ np.array([11 * pr.PITCH_MM, 0, 0]) + m[:3, 3]  # A12
    jump = float(np.linalg.norm(new - old))
    if jump < 3.0 and max(abs(v) for v in side['chain_off_mm']) <= 1.5:
        pr.TOP_CACHE.write_text(json.dumps(top)); pr.SIDE_CACHE.write_text(json.dumps(side))
        pr.PLATE_CACHE.write_text(json.dumps({**plate, 'sheet_plate': m.tolist(),
                                              'source': plate['source']}))
        print(f'plate re-solved: A12 moved {jump:.2f} mm')
    else:
        print(f'plate solve kept OLD caches: A12 jump {jump:.1f} mm, side chain off {side["chain_off_mm"]}')
except RuntimeError as e:
    print('plate solve failed, old caches kept:', e)
