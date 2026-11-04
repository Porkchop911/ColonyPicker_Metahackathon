"""Needle-down IK for the 2026-11-01 build (needle clamped across link 4) + one staged move + frames + diff vs bg.
usage: reach.py X Z PAN TAG   (tip x forward of base_link, z up, mm)"""
import sys, math, cv2, numpy as np
sys.path.insert(0, 'tools')
PX, SZ, UP, FA = 46.6, 190.7, 85.13, 85.15   # hwr_core ArmKinematics
ALONG, NEEDLE = 78.0, 110.0                  # wrist axis -> clamp along link 4; clamp -> tip (photo estimate)
D = 0.24

def ik(x, z):
    r, h = x - PX - ALONG, z + NEEDLE - SZ
    d = math.hypot(r, h)
    c = (d * d - UP * UP - FA * FA) / (2 * UP * FA)
    assert -1 < c < 1, f'unreachable d={d:.0f}'
    el = math.acos(c)
    sh = math.atan2(r, h) - math.atan2(FA * math.sin(el), UP + FA * math.cos(el))
    wr = math.pi / 2 - sh - el
    return {2: round(500 - math.degrees(sh) / D), 3: round(500 - math.degrees(el) / D), 4: round(500 - math.degrees(wr) / D)}

if __name__ == '__main__':
    x, z, pan, tag = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    q = {1: pan, **ik(x, z)}
    print('pose', q, 'sum', q[2] + q[3] + q[4], flush=True)
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300
    if '--dry' in sys.argv: sys.exit()
    import well_reach as wr
    from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER
    s = wr.Servos(); r = s.read(); now = {k: r[k] for k in wr.ARM}
    wr.jgo(s, q, now); print('read', s.read(), flush=True)
    S = sys.argv[5]
    t = Camera(OVERHEAD); a = t.frame(); t.close(); c = Camera(SIDE, paper_target=SIDE_PAPER); b = c.frame(); c.close()
    cv2.imwrite(f'{S}/{tag}_top.jpg', a); cv2.imwrite(f'{S}/{tag}_side.jpg', b)
    for n, im in (('top', a), ('side', b)):
        bg = cv2.imread(f'{S}/st2_{n}.jpg')
        dd = (np.abs(im.astype(int) - bg.astype(int)).max(axis=2) > 40).astype(np.uint8)
        dd = cv2.morphologyEx(dd, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        ys, xs = np.nonzero(dd); print(n, 'changed px', len(xs), 'bbox', (xs.min(), ys.min(), xs.max(), ys.max()) if len(xs) else None)
    cv2.imwrite(f'{S}/{tag}_both.jpg', cv2.resize(cv2.vconcat([a, b]), (960, 1080)))
