"""EXPERIMENTAL (2026-10-29 night, branch tipcam-external): Persistent touch session for the upright grasp: SAM, cameras and the side pose loaded once; attempts back to back."""
import sys, json, time; sys.path.insert(0, 'tools')
import cv2, numpy as np, sam_detect as sam, tip_triangulate as tt, model_jab as mj, hwr_kinematics as hk
from pathlib import Path
from rig_io import Camera, SIDE, SIDE_PAPER
from tip_descend import read, send

S = Path(sys.argv[1]); ATTEMPTS = int(sys.argv[2]) if len(sys.argv) > 2 else 6
START = {2: 409, 3: 180, 4: 356}            # the tip ~20-40 mm over the paper, seen by both UGREENs (measured)
SIDE_RIG_FRAME = 'evidence/tip/side_rig_frame.jpg'  # the unmoved side camera without the needle in front
log_path = Path('evidence/descend') / ('vsession_' + time.strftime('%Y%m%dT%H%M%S') + '.json')
say = lambda *a: print(time.strftime('%H:%M:%S'), *a, flush=True)  # noqa: E731

p = sam.load(); det = lambda path, q, th: sam.detect(p, path, q, th)  # noqa: E731
top, side = Camera(), Camera(SIDE, paper_target=SIDE_PAPER)
u = top.frame(); rig = tt.ugreen_rig(u); uk, ud, ur, ut = (np.array(rig[q]) for q in ('k', 'd', 'r', 't'))
cv2.imwrite(str(S / 'vs.png'), u)
red = [d['centroid'] for d in det(str(S / 'vs.png'), 'small red dot', mj.SAM_THRESHOLD)
       if d['centroid'] and mj.RED_AREA[0] <= d['area'] <= mj.RED_AREA[1]]
if len(red) < 3:  # the needle can hang over one: the three red dots as found this evening (the overhead has not moved)
    red = [(903.7, 563.5), (1133.1, 479.5), (1302.4, 359.2)]
sheet = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in json.load(open('evidence/descend/ugreen_dots.json')) + list(red)])
srig = tt.side_rig(tt.side_marks(cv2.imread(SIDE_RIG_FRAME), det, S / 'vs2.png'), sheet)
sk = tuple(np.array(srig[q]) for q in ('k', 'd', 'r', 't'))
say(f"ready: side pose {srig['dots']} dots {srig['rms']:.1f} px, {abs(srig['centre_mm'][2]):.0f} mm up")
A = hk.ArmKinematics(); ug_last = np.array([1145.0, 468.0]); log = []


def measure():
    """Side tip (SAM), then the overhead teal blob whose ray best meets the side ray (old ink does not line up)."""
    global ug_last
    sf = side.frame(); cv2.imwrite(str(S / 'vs3.png'), sf)
    found = det(str(S / 'vs3.png'), 'thin metal needle', 0.15) or det(str(S / 'vs3.png'), 'black stick', 0.3)
    if not found:
        return None, 'no needle in the side view'
    b = max(found, key=lambda d: d['score']); vv, uu = np.nonzero(b['mask']); k = int(np.argmax(vv))
    spx = np.array([uu[k], vv[k]], float)
    # SAM's needle mask stops where a thick ink bulb begins: the bulb's lowest teal pixel within 60 px is the real end
    xs, ys = int(spx[0]), int(spx[1]); wx, wy = max(0, xs - 60), max(0, ys - 30)
    tm = cv2.inRange(cv2.cvtColor(sf[wy:ys + 60, wx:xs + 60], cv2.COLOR_BGR2HSV), (40, 25, 60), (110, 255, 255))
    nn, ll, ss, _ = cv2.connectedComponentsWithStats(tm)
    near = [j for j in range(1, nn) if ss[j, 4] >= 6 and ss[j, 1] + wy <= ys + 5]  # touching SAM's end, not a drip below
    if near:
        j = max(near, key=lambda j: ss[j, 4]); ty, tx = np.nonzero(ll == j)
        spx = np.array([tx[ty == ty.max()].mean() + wx, ty.max() + wy], float)
    side_ray = tt.ray(spx, *sk)
    f = top.frame(); x, y = int(ug_last[0]), int(ug_last[1]); x0, y0 = max(0, x - 150), max(0, y - 150)
    m = cv2.inRange(cv2.cvtColor(f[y0:y + 150, x0:x + 150], cv2.COLOR_BGR2HSV), (40, 25, 60), (110, 255, 255))
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    best = None
    for j in range(1, n):
        if st[j, 4] < 6:
            continue
        vs, us = np.nonzero(lab == j); px = np.array([us[vs == vs.max()].mean() + x0, vs.max() + y0])
        p3, miss = tt.meet(tt.ray(px, uk, ud, ur, ut), side_ray)
        if best is None or miss < best[1]:
            best = (p3, miss, px)
    if best is None or best[1] > 7.0:  # thick ink: overhead and side pick points ~5 mm apart on the needle (session 6/7)
        tag = time.strftime('%H%M%S')
        cv2.imwrite(str(S / f'lost_{tag}_top.jpg'), f); cv2.imwrite(str(S / f'lost_{tag}_side.jpg'), sf)
        return None, 'no overhead teal blob lines up with the side tip' if best else 'no teal near the last overhead tip'
    p3, miss, px = best; ug_last = px
    return (float(-p3[2]), float(miss), p3[:2].round(1).tolist(), float(spx[1])), f


def to_start():
    q0 = read()
    if q0[3] < START[3] + 10:  # off the paper first
        send({3: q0[3] + 12}, ms=400); time.sleep(0.6); q0 = read()
    for i in range(1, 5):
        send({k: int(round(q0[k] + (v - q0[k]) * i / 4)) for k, v in START.items()}, ms=400); time.sleep(0.6)


START_PX = np.array([1145.0, 468.0])  # the re-seated needle's tip at START (21:28)


def attempt(n: int) -> str:
    """One descent from START; returns why it ended."""
    global ug_last
    to_start()
    ug_last = START_PX.copy()  # the start pose puts the tip back here: a lost tip must not drag the window off
    r, info = measure()
    if r is None:
        return f'start: {info}'
    prev = r[0]; cmd = read(); gain = 3.5; side_prev = r[3]; stills = 0
    settled = prev < 11.0  # once the slack has gone, the start pose itself leaves the tip ~4 mm up (session 4)
    say(f'attempt {n}: tip {prev:.1f} mm up, rays {r[1]:.2f} mm, at {r[2]}')
    for i in range(40):
        want = 0.5 if prev < 6.0 else min(3.0, max(0.5, (prev - 0.5) / 2))
        j = hk.pulses_to_joints(cmd); x, y, z, pitch = A.fk(j)
        dz = want / gain / 1000.0
        s_ = A.ik(x, y, z - dz, pitch, j.roll)
        if s_ is None:
            return 'out of reach'
        tgt = {k: v for k, v in hk.joints_to_pulses(s_).items() if k in (2, 3, 4)}
        if all(tgt[k] == cmd[k] for k in tgt):
            tgt = {**{k: cmd[k] for k in (2, 3, 4)}, 3: cmd[3] - 1}
        send(tgt, ms=300); time.sleep(0.6); cmd = {**cmd, **tgt}
        r, info = measure()
        if r is None:
            return f'step {i}: {info}'
        h, miss, xy, side_y = r
        log.append({'attempt': n, 'step': i, 'cmd': tgt, 'height_mm': h, 'miss_mm': miss, 'xy': xy})
        if i == 0 and not settled:  # slack lets go on the first step down: 9-11 mm whatever its size
            say(f'  settle step: {prev:.1f} -> {h:.1f} mm')
            if h <= 0.5:
                return 'CONTACT'  # the settle step itself reached the paper
            prev = h
            continue
        if prev - h > 0.3:
            gain = 0.5 * gain + 0.5 * max(0.5, (prev - h) / (dz * 1000.0))
        say(f'  step {i}: want {want:.1f} -> {h:5.1f} mm (gain {gain:.1f}), rays {miss:.2f}, at {xy}')
        down = side_y - side_prev; side_prev = side_y
        if h < 6.0:  # near the paper the side view decides: the tip stops going down there (side-still, run 125405)
            stills = stills + 1 if down < 1.5 else 0
            say(f'    side tip moved {down:+.1f} px down ({stills} still)')
            if stills >= 2:
                return 'CONTACT (side camera: the tip stopped)'
        if h <= -2.0:
            return 'CONTACT (triangulated height)'
        if prev - h > 2.5 * want + 1.5:
            return f'dropped {prev - h:.1f} mm for {want:.1f} asked'
        prev = h
    return 'step limit'


before = top.frame()
try:
    for n in range(1, ATTEMPTS + 1):
        why = attempt(n)
        say(f'attempt {n} ended: {why}')
        if why.startswith('CONTACT'):
            break
finally:
    send({3: read()[3] + 15}, ms=400); time.sleep(0.6)  # off the paper
    to_start()
    after = top.frame()
    cv2.imwrite(str(log_path.with_suffix('.before.jpg')), before); cv2.imwrite(str(log_path.with_suffix('.after.jpg')), after)
    log_path.write_text(json.dumps(log, indent=1)); top.close(); side.close()
    say('session over, arm at the start pose; log', log_path)
