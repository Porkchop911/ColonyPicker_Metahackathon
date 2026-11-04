"""Tour through learnt wells: transit 15 mm over the rims, camera alignment 10 mm over the rim, a look at 4 mm,
then straight down; off at 4 mm -> back to 10 mm, correct there, look again (never sideways low).
usage: <venv python> tools/well_tour.py A12 B12 C12 ... [--dry] [--no-look] [--no-check] [--dwell A12=5,B12=2]

Poses (x, pan) come from evidence/tip/well_poses.json; heights come from the side camera (offset kept in
evidence/tip/well_z_offset.json). Every move ends from 8 shoulder pulses above and from a lower pan (backlash).
Results: evidence/tip/tour_results.jsonl."""
import json
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from reach import ik  # noqa: E402

S = 'evidence/tip'
DRY = '--dry' in sys.argv
LOOK = '--no-look' not in sys.argv
BLIND = '--blind' in sys.argv  # no camera at all: the last camera-confirmed pose, straight down (the plate does not move; user 11-03 19:33)
FIXED_H = '--fixed-height' in sys.argv  # trust well_z_offset.json (measured from triangulated tips)
PRELOAD = '--preload' in sys.argv or BLIND  # a blind entry is only repeatable with a fixed arrival direction
NO_CHECK = '--no-check' in sys.argv  # skip the 4 mm look (it misreads the tip over the white plate, 2026-11-03)
WELLS = [w for w in sys.argv[1:] if not w.startswith('--') and '=' not in w]
DWELL = ({w: float(t) for w, t in (x.split('=') for x in sys.argv[sys.argv.index('--dwell') + 1].split(','))}
         if '--dwell' in sys.argv else {})  # s at the bottom of a well (disinfectant contact time)
TRANSIT_H = 15.0   # mm over the rim (measured): pans only up here
ALIGN_H = 10.0     # mm over the rim: the camera alignment and every sideways correction
CHECK_H = 4.0      # mm over the rim: look only (height + xy); then straight down into the well
CHECK_TOL = 6.0    # px at CHECK_H; 9 let rim hits count as entered and learnt them (C12 rim 11-03 19:00): 6 again
CHECKS = 3
TRACE_TOP = 260    # overhead row: the robot's arm and wheels are above it
DEPTH = 3.0        # mm under the rim at the bottom of the entry

poses = json.load(open(f'{S}/well_poses.json'))
missing = [w for w in WELLS if w not in poses]
assert not missing, f'no learnt pose for {missing}: run tools/a1_place.py on them first'


def q_of(cmd):
    q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300, q
    return q


if DRY:
    state = {'q': {1: 500, 2: 450, 3: 207, 4: 468}}

    def go(cmd):
        q = q_of(cmd)
        print('  go', q)
        state['q'] = q

    def look(cmd):
        return a.sheet_of(*cmd), 0.3
else:
    import well_reach as wr
    from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER
    wr.DWELL_S = 0.25  # fluid: the 1.2 s settle after every move was ~10 s per well; the look below waits itself
    servos = wr.Servos()
    cams = {'top': Camera(OVERHEAD), 'side': Camera(SIDE, paper_target=SIDE_PAPER)}
    bgs = {n: cv2.imread(f'{S}/st2_{n}.jpg') for n in cams}
    r = servos.read()
    state = {'q': {k: r[k] for k in wr.ARM}}
    a.assert_unfolded(state['q'])

    def go(cmd):
        q = q_of(cmd)
        above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != state['q'][1] else {})}  # pan swing only on pan changes
        wr.jgo(servos, above, state['q'], dwell=0.2, fast=True)  # straight on into q
        wr.jgo(servos, q, above, fast=True)
        state['q'] = q

    def look(cmd, floor_z):
        time.sleep(0.8)  # settle before the frames
        got = []
        for _ in range(5):
            imgs = {n: c.frame() for n, c in cams.items()}
            X, miss, _ = a.measure(imgs, bgs, a.sheet_of(*cmd), floor_z=floor_z)
            if X is not None:
                got.append(X)
            if len(got) == 3:
                break
        return (np.median(got, axis=0), None) if got else (None, None)


def needle_x(frame, bg, tip):
    """The needle's column (px) in the side view, 60-200 px above the expected tip (clear of the plate), against the
    background; None if it is not there."""
    x0, y0, y1 = int(tip[0]) - 40, int(tip[1]) - 200, int(tip[1]) - 60
    d = np.abs(frame[y0:y1, x0:x0 + 80].astype(int) - bg[y0:y1, x0:x0 + 80].astype(int)).max(axis=2)
    col = (d > 20).sum(axis=0).astype(float)
    if col.max() < 0.5 * (y1 - y0):
        return None
    w = np.clip(col - 0.5 * col.max(), 0, None)
    return x0 + float((w * np.arange(80)).sum() / w.sum())


def descent_px(before, after, bg, tip, exp):
    """Vertical shift (px) of the needle in the side view between two frames: the brightness profile down the needle's
    own column (found against the background), so the still box behind it cannot match instead. Returns (shift, corr)."""
    x0, y0, y1 = int(tip[0]) - 30, int(tip[1]) - 220, int(tip[1]) - 10
    d = np.abs(before[y0:y1, x0:x0 + 60].astype(int) - bg[y0:y1, x0:x0 + 60].astype(int)).max(axis=2)
    cx = x0 + int(np.argmax(d.sum(axis=0)))  # the needle's column
    prof = lambda im, a_, b_: cv2.cvtColor(im[a_:b_, cx - 2:cx + 3], cv2.COLOR_BGR2GRAY).mean(axis=1)
    p0 = prof(before, y0, y1 - int(2 * exp))
    p0 = (p0 - p0.mean()) / (p0.std() + 1e-6)
    best = (-2.0, 0)
    for k in range(0, int(2 * exp) + 1):
        p1 = prof(after, y0 + k, y1 - int(2 * exp) + k)
        c = float(np.mean(p0 * (p1 - p1.mean()) / (p1.std() + 1e-6)))
        best = max(best, (c, k))
    return float(best[1]), best[0]


def side_height(frame, bg, w, h_exp):
    """Tip height (mm) over the rim of well w from the side camera alone: the needle's lowest end above the plate top,
    its ray met with the vertical through the rim centre (the side camera looks almost level, so a few mm of depth error
    barely changes the height). None if no needle end is seen. The model's heights were ~10 mm high, 2026-11-03."""
    r = a.pr.well(w)
    ru = a.project('side', r)
    pe = a.project('side', r + np.array([0.0, 0.0, -h_exp]))  # where the tip should be at the expected height
    ends = [e for e in (a.find_tip(frame, bg, ru, win=160, th=12, every=True) or [])
            if e[1] < ru[1] - 8 and abs(e[0] - pe[0]) < 70 and abs(e[1] - pe[1]) < 70]  # above the rim (not the
    # mirror, not the plate edge: -0.08 mm at C12/F12) and near the tip (not the park needle in the background: 64 mm)
    if not ends:
        return None
    c = a.CAMS['side']
    o, d = tt.ray(np.array(max(ends, key=lambda e: e[1]), float), np.array(c['k']), np.array(c['d']), c['r'], c['t'])
    v, w0 = np.array([0.0, 0.0, 1.0]), o - r
    b = d @ v
    s_ = (b * (v @ w0) - d @ w0) / (1 - b * b)
    return float(r[2] - (o + s_ * d)[2])


_cp = open('tools/colony_pick.py').read()  # the colony picker's needle trace + ridge tip (agar, and the white plate:
_ns = {'np': np, 'cv2': cv2}                 # 3 of 4 well frames within 3 px of the eye, find_tip 10-40 px, 2026-11-03)
exec(_cp[_cp.index('GAP_ROWS ='):_cp.index('BAND_MM')], _ns)  # same slice tools/test_colony_pick.py replays


def ridge_tip(frame, bg, target):
    """Overhead tip near target: the colony picker's ridge tip on the traced needle, else the trace end."""
    g = (target[0], target[1] - 60 * _ns['ALONG'])  # window top below the robot: its arm in the top rows hid the needle (F12-H12)
    tr = _ns['follow_tip'](frame, bg, g, path=True, up=max(int(g[1]) - TRACE_TOP, 40) / _ns['ALONG'])
    if not tr:
        return None
    m = _ns['mirror_tip'](frame, bg, tr)
    return tuple(m[0]) if m else tuple(tr[-1])


def measured_height(w):
    hs = [h for h in (side_height(cams['side'].frame(), bgs['side'], w, TRANSIT_H) for _ in range(3)) if h is not None]
    return float(np.median(hs)) if hs else None


Z_FILE = f'{S}/well_z_offset.json'
try:
    z_off = json.load(open(Z_FILE))['z_off']  # model mm to add so the commanded height is the measured one
except (OSError, ValueError, KeyError):
    z_off = 0.0
out = open(f'{S}/tour_results.jsonl', 'a') if not DRY else None
t0 = time.time()
for w in WELLS:
    p = poses[w][-1]
    hover, entry = list(p['hover']), list(p['entry'])  # hover: 5 mm over the rim (as taught)
    rim = a.pr.well(w)
    at = lambda c, h: (c[0], hover[1] + h - 5.0 + z_off, c[2])  # pose c at h mm (measured) over the rim
    if PRELOAD:  # 11-04: the bottom flipped between two spots 4.5 mm apart by arrival direction: every arrival now from 12 mm short
        go(at((hover[0] - 12.0, hover[1], hover[2] - 12.0), TRANSIT_H))
    go(at(hover, TRANSIT_H))  # transit: pans only up here
    status, err = 'off', None
    if BLIND and not DRY:
        status, cmd = 'entered', at(hover, ALIGN_H)
    elif LOOK and not DRY:
        # 1. height from the side camera, at a safe height, before anything sideways happens
        ok = False
        for k in range(5):  # measured at TRANSIT_H only: lower, behind row A, the tip is lost against the white plate
            h = TRANSIT_H if FIXED_H else measured_height(w)  # 11-04: the spiral behind A12 read 9-15 mm at a true 15
            print(f'  height {w} {k}: commanded {TRANSIT_H:.0f}, side camera {h} mm over the rim')
            if h is None or abs(TRANSIT_H - h) > 6.0:  # junk (one read -0.9 at 14 mm, C12 2026-11-03): look again
                continue
            if abs(TRANSIT_H - h) < 1.5:
                ok = True
                break
            z_off = float(np.clip(z_off + np.clip(TRANSIT_H - h, -3.0, 3.0), -15, 15))  # at most 3 mm per step
            go(at(hover, TRANSIT_H))
        if not ok:
            print(f'  {w}: height not confirmed by the side camera, well skipped')
        else:
            json.dump({'z_off': z_off, 'when': time.strftime('%Y-%m-%d %H:%M:%S')}, open(Z_FILE, 'w'))
            # 2. sideways alignment only at ALIGN_H; 3. look at CHECK_H (height and xy); off -> back up, correct, again
            cmd = a.image_align(go, cams['top'].frame, bgs['top'], w, at(hover, ALIGN_H), h=ALIGN_H, tol_px=8.0, say=print,
                                find=ridge_tip)
            target = a.image_target(bgs['top'], w, CHECK_H)
            if NO_CHECK and cmd is not None:  # baseline: trust the 10 mm alignment, straight down
                status = 'entered'
            for k in range(CHECKS if cmd is not None and target is not None and not NO_CHECK else 0):
                go(at(cmd, CHECK_H))
                sf = cams['side'].frame()
                cv2.imwrite(f'{S}/check_{w}_side.jpg', sf)
                hc = side_height(sf, bgs['side'], w, CHECK_H)  # near the plate top the end can blend in: None is allowed (the
                # height was confirmed at ALIGN_H just before, and this is a 6 mm straight drop)
                tf = [cams['top'].frame() for _ in range(3)]
                cv2.imwrite(f'{S}/check_{w}_{k}_top.jpg', tf[0])
                tips = [t for t in (ridge_tip(f, bgs['top'], target) for f in tf) if t is not None]
                good = [t for t in tips if sum(np.linalg.norm(np.array(t) - np.array(u)) <= 5 for u in tips) >= 2]
                if len(good) < 2:  # 2 of 3 must agree within 5 px (one junk frame in 3 is common)
                    print(f'  check {w} {k}: tip readings disagree {tips}: well skipped')
                    break
                tips = good
                if hc is not None and not 2.0 < hc < 7.0:
                    print(f'  check {w} {k}: height {hc} mm, tip {"seen" if tips else "not seen"}: well skipped')
                    break
                e = target - np.median(tips, axis=0)
                print(f'  check {w} {k}: {hc} mm over the rim (side), off {e.round(1)} px')
                if np.linalg.norm(e) <= CHECK_TOL:
                    status = 'entered'; break
                go(cmd)  # back up to ALIGN_H, then the sideways correction up there
                if np.linalg.norm(e) > 25:
                    print(f'  check {w}: {np.linalg.norm(e):.0f} px off at {CHECK_H:.0f} mm, well skipped'); break
                dx, dp = 0.8 * np.linalg.lstsq(a.image_jacobian(cmd), e, rcond=None)[0]
                if 0.2 < abs(dp) < 1.0:  # at least one pan pulse, else the rounded command does not move
                    dp = float(np.sign(dp))
                cmd = tuple(np.array(cmd) + np.array([dx, 0.0, dp]))
                go(cmd)
            if status == 'entered':
                d = np.array(cmd) - np.array(at(hover, ALIGN_H))
                err = round(float(np.hypot(d[0], d[2])), 2)  # how far the taught pose was, sideways
                if err < 15.0 and not NO_CHECK:  # only entries checked at CHECK_H teach the pose; the unchecked aligns snowballed (2026-11-03)
                    hover2 = [hover[0] + d[0], hover[1], hover[2] + d[2]]
                    poses[w].append({'hover': hover2, 'entry': [entry[0] + d[0], entry[1], entry[2] + d[2]], 'gate_xy': None,
                                     'when': time.strftime('%Y-%m-%d %H:%M:%S'), 'by': 'tour checked entry'})
                    json.dump(poses, open(f'{S}/well_poses.json', 'w'), indent=1)
    if DRY:
        status, cmd = 'entered', at(hover, ALIGN_H)
        go(cmd); go(at(cmd, CHECK_H))
    if status == 'entered':
        go(at(cmd, -DEPTH))  # straight down from the checked look (x and pan unchanged)
        if not DRY:
            top, side = cams['top'].frame(), cams['side'].frame()
            u, v = a.project('top', rim), a.project('side', rim)
            crop = lambda im, c: cv2.resize(im[int(c[1]) - 150:int(c[1]) + 100, int(c[0]) - 125:int(c[0]) + 125], (500, 500))
            photo = f'{S}/tour_{w}_{time.strftime("%m%d_%H%M%S")}.jpg'
            cv2.imwrite(photo, cv2.hconcat([crop(top, u), crop(side, v)]))
            time.sleep(DWELL.get(w, 0.0))
        go(at(cmd, ALIGN_H))  # straight up before any pan
    go(at(hover, TRANSIT_H))
    print(f'{time.time() - t0:6.1f}s {w}: {status} pose correction {err}', flush=True)
    if out:
        out.write(json.dumps({'when': time.strftime('%Y-%m-%d %H:%M:%S'), 'well': w, 'status': status, 'hover_err': err}) + '\n')
        out.flush()
last = poses[WELLS[-1]][-1]['hover']
try:
    go((last[0], last[1] + 25.0 + max(z_off, 0.0), last[2]))  # end clear of the plate
except AssertionError:  # out of reach over the far wells (H10): the loop already left the needle at TRANSIT_H
    pass
print(f'tour done in {time.time() - t0:.0f} s')
