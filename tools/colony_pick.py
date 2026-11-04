"""Pick a colony with the needle, by pixels: overhead for xy, the side view for the height over the agar.
usage: <venv python> tools/colony_pick.py [--colony I | --px U V] [--depth 0.7] [--dry] [--again] [--agar MM: earlier reading, same dish only] [--start 12-20]

The 3-D camera models are off out at the dish (the side model by ~100 px, it was solved on the plate), so nothing here
trusts them beyond small differences:
- xy: the tip pixel in the overhead goes onto the colony pixel (SAM, colony_detect.py, on the background frame) plus the
  model's parallax for the current height (a difference: errors cancel); steps via the overhead image Jacobian. Where the
  tip's mirror image in the wet agar is found (mirror_tip, lamp 12 on), the midpoint of tip and mirror replaces that.
- height: in the side view the colony right under the needle and the needle tip are both visible; their pixel gap /
  the side scale (px per mm, measured by a known 4 mm move) is the height over the agar.
Descends in steps re-aligning, then goes the last gap + --depth into the agar and lifts 25 mm.
Needle must start on the needle-down model (a1lib.assert_unfolded). Background frames: evidence/tip/st2_{top,side}.jpg."""
import json
import os
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
arg = lambda name, n=1, default=None: (sys.argv[sys.argv.index(name) + 1:sys.argv.index(name) + 1 + n]
                                       if name in sys.argv else default)
DEPTH = float(arg('--depth', 1, ['4.0'])[0])  # 4 mm: aligned picks stopped above the colony (user 19:36: 'further down')
FLOOR_MM = 2.0           # never command the tip under this over the paper (model)
AGAR_GUESS = 8.0          # mm over the paper, only for the first hover
START_UP = 35.0           # mm over the paper: clear of the dish wall (~15 mm)
log = open(f'{S}/colony_pick.log', 'a')


def say(*m):
    line = ' '.join(str(x) for x in m)
    print(line, flush=True)
    log.write(line + '\n')
    log.flush()


def colonies(bg_top):
    """SAM colonies on the overhead background: (px, diameter mm, score), big and confident ones only (the small
    hits at the dish edge were meniscus glare, 2026-11-02)."""
    try:  # SAM on call (tools/sam_server.py, or the demo page): seconds instead of a model load
        import psutil
        alive = psutil.pid_exists(int(open(f'{S}/sam_ready').read()))
    except (OSError, ValueError, ImportError):
        alive = False
    if alive:
        t0 = time.time()
        open(f'{S}/sam_request', 'w').write('1')
        while time.time() - t0 < 60:
            v = f'{S}/colony_variants.json'
            if os.path.exists(v) and os.path.getmtime(v) > t0:
                return [(np.array(c['px']), c['d_mm'], c['score']) for c in json.load(open(v))['normal']]
            time.sleep(0.2)
        say('SAM on call did not answer in 60 s: loading it here')
    import colony_detect as cd
    import sam_detect
    pred = sam_detect.load()
    _, cols = cd.detect_colonies(pred, bg_top)
    out = [(np.array(c['centroid'], float), float(c['d_mm']), float(c['score'])) for c in cols]
    g = cv2.cvtColor(bg_top, cv2.COLOR_BGR2GRAY).astype(float)
    yy, xx = np.mgrid[-16:17, -16:17]
    r = np.hypot(xx, yy)

    def contrast(p):  # white colony over the agar: colonies 44-82, bare agar 11, rim glare 24 (23:41)
        x, y = int(p[0]), int(p[1])
        w = g[y - 16:y + 17, x - 16:x + 17]
        return w[r <= 4].mean() - w[(r >= 10) & (r <= 14)].mean() if w.shape == r.shape else 0.0
    # scores fell to 0.57-0.64 on the same dish (23:36), so the score gate is low; size and contrast reject glare
    return [c for c in out if c[1] >= 1.8 and c[2] >= 0.5 and contrast(c[0]) > 30]  # new plate 11-04: colonies 65-91, faint smudges 24-27; same as variants 'normal' (halo'd colonies of the old plate read 13-18)


DISH_H_MM = 14.5  # dish outer height, rim to underside (55 mm dish; user to confirm)
MAT_MM = 1.0      # black mat under the dish (user to confirm)
AGAR_OFFSET = 0.0  # was -2.5 on 11-02; with 2 mm agar that commanded -0.5 mm over the paper. Re-measure by a watched touch


def agar_height(side_bg, span):
    """Agar surface over the paper (mm), from the side view of the dish's front wall, the dish as the ruler: rim
    highlight, agar line (green interior seen through the clear wall -> yellow agar body), bottom-edge highlight, on a
    few columns around cx. Agar height differs per plate (user), so this runs on every plate's background frame."""
    hsv = cv2.cvtColor(side_bg, cv2.COLOR_BGR2HSV).astype(float)
    fills = []
    lo, hi = span
    w = hi - lo
    for x in range(int(lo + 0.25 * w), int(hi - 0.25 * w) + 1, 8):  # middle half: the wall's ends read high
        col = hsv[:, x - 2:x + 3].mean(axis=1)
        hue, sat, v = col[:, 0], col[:, 1], col[:, 2]
        yel = (hue >= 40) & (hue <= 62) & (sat > 60) & (v > 70) & (v < 160)
        yel[:600] = False
        rows = np.nonzero(yel)[0]
        if len(rows) < 5:
            continue
        # the lowest run of yellow rows = the agar body behind the front wall (gaps <= 3 px)
        run_end = rows[-1]
        run_top = run_end
        for r_ in rows[::-1][1:]:
            if run_top - r_ > 3:
                break
            run_top = r_
        if run_end - run_top < 5:
            continue
        y_bot = run_end + int(np.argmax(v[run_end:run_end + 15]))          # bottom-edge highlight under the agar
        y_rim = y_bot - 65 + int(np.argmax(v[y_bot - 65:y_bot - 38]))      # rim highlight (front wall ~51 px here)
        fills.append((y_bot - run_top) / (y_bot - y_rim))
    if len(fills) < 3:
        return None
    return MAT_MM + float(np.median(fills)) * DISH_H_MM


def dish_cx_side(side_bg):
    """The dish's middle column in the side view: the yellow agar body seen through its front wall (hue 40-62,
    saturated, mid-bright) is nowhere else in the frame; not the side model (~100 px off out at the dish)."""
    hsv = cv2.cvtColor(side_bg, cv2.COLOR_BGR2HSV).astype(int)
    m = (hsv[..., 0] >= 40) & (hsv[..., 0] <= 62) & (hsv[..., 1] > 60) & (hsv[..., 2] > 80) & (hsv[..., 2] < 160)
    m[:600] = False
    dark = (hsv[..., 2] < 50).astype(np.uint8)  # the black mat under the dish
    below = np.zeros_like(dark)
    below[:-25] = cv2.dilate(dark, np.ones((25, 1), np.uint8), anchor=(0, 0))[25:]  # mat within 25 px below
    m &= below > 0  # the dish's agar band sits right on the mat; the yellowish desk does not
    on = np.convolve((m.sum(axis=0) >= 8).astype(float), np.ones(9), 'same') > 4  # bridge small gaps
    runs, start = [], None
    for x, v in enumerate(on):
        if v and start is None:
            start = x
        if not v and start is not None:
            runs.append((start, x - 1)); start = None
    if start is not None:
        runs.append((start, len(on) - 1))
    if not runs:
        return None
    lo, hi = max(runs, key=lambda r: r[1] - r[0])  # the dish: the widest run (the desk gives scattered bits)
    return (float(lo), float(hi)) if hi - lo >= 60 else None


def ground(px, z):
    c = a.CAMS['top']
    o, v = tt.ray(px, np.array(c['k']), np.array(c['d']), c['r'], c['t'])
    return o + v * ((z - o[2]) / v[2])


def side_colony_y(side, bg_side, x_needle, y_tip):
    """y (px) of the colony right under the needle in the side view: bright blob in a narrow column below the tip."""
    x0, x1, y0, y1 = int(x_needle) - 25, int(x_needle) + 25, int(y_tip) + 3, min(int(y_tip) + 220, side.shape[0])
    if x0 < 0 or x1 > side.shape[1] or y1 - y0 < 10:
        return None
    crop = side[y0:y1, x0:x1]
    g = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY).astype(float)
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    # agar in the side camera: greenish, saturated, darker (h 76 s 88 v 102); the plate behind is pale and bright (s 47 v 219)
    agar = cv2.dilate(((hsv[..., 1] > 60) & (hsv[..., 2] < 180)).astype(np.uint8), np.ones((9, 9), np.uint8))
    m = ((g > np.median(g) + 25) & (agar > 0)).astype(np.uint8)  # bright AND on agar: not the white plate behind
    k, lab, st, cen = cv2.connectedComponentsWithStats(m)
    cand = [cen[i] for i in range(1, k) if st[i, 4] >= 6]
    if not cand:
        return None
    c = min(cand, key=lambda p: abs(p[0] - 25) + 0.2 * p[1])  # nearest the needle column, then nearest the tip
    return float(c[1] + y0)


if DRY:
    say('dry: checks the reach to every SAM colony only')
if '--variants' in sys.argv:  # colony_cycle's fallback when the page's SAM does not answer: it read a stale file before
    import colony_detect as cd
    import sam_detect
    json.dump(cd.variant_picks(sam_detect.load(), cv2.imread(f'{S}/st2_top.jpg')), open(f'{S}/colony_variants.json', 'w'), indent=1)
bg = {n: cv2.imread(f'{S}/st2_{n}.jpg') for n in ('top', 'side')}
cols = colonies(bg['top']) if '--px' not in sys.argv else [(np.array([float(v) for v in arg('--px', 2)]), 0, 1)]
PICKED = f'{S}/colony_picked.json'  # overhead px of colonies already pierced (the dish creeps ~5 px per pick)
picked = json.load(open(PICKED)) if os.path.exists(PICKED) else []
for j, q in enumerate(picked):  # follow the creep: move each record onto its colony in this background
    near = min((c[0] for c in cols), key=lambda p: np.linalg.norm(p - np.array(q)), default=None)
    if near is not None and np.linalg.norm(near - np.array(q)) < 15:
        picked[j] = near.round(1).tolist()
if '--px' not in sys.argv:
    json.dump(picked, open(PICKED, 'w'))
done = lambda p: any(np.linalg.norm(p - np.array(q)) < 15 for q in picked)
for i, (p, d, sc) in enumerate(cols):
    g = ground(p, -AGAR_GUESS)
    cmd = a.model_of(np.array([g[0], g[1], -START_UP]))
    ok = True
    try:
        ik(cmd[0], cmd[1] - (START_UP - AGAR_GUESS))
    except AssertionError:
        ok = False
    say(f'colony {i}: px {p.round(1)} {d:.1f} mm score {sc:.2f} r {np.linalg.norm(g[:2] - a.C):.0f} mm reach {ok}'
        + (' PICKED' if done(p) else ''))
idx = int(arg('--colony', 1, ['0'])[0])
target_px = cols[idx][0]
AIM_OFFSET = np.array([-9.0, 11.0])  # px, 11-04 15:05: 8 rows landed 10 short, 17 rows 19 past on plate 4; 13:42: C12 landed just short of a 2 mm colony at 14 rows. 13:15: -12 landed slightly left, -6 about 7 px right. 11:21: two picks with the 12 mm run-up landed 11-15 rows short at 2. 10:55: the 12 mm drop landed 22 rows past at 24; 18 was the 8 mm drop. 10:17: pick at [5, 18] landed 11 px right, rows right; 30 rows landed ~2 mm past the centre: aligned to 3 px at 4 mm, the drop landed 5 left, 30 up (toward the robot), height right. Before, zeros: used from the 8 mm stage: pick 9 (11-03 20:38) aimed 8 px up and its hole sat 9 px up: lands where aimed
MEASURE = '--measure' in sys.argv
PLAN = f'{S}/colony_plan.json'
AIM4 = AIM_OFFSET  # the whole approach aims there on the new layout, no sideways move low; was [-2.4, -8.4]. px, the 4 mm stage: an empty-agar pierce from 4 mm landed 2.4 px right, 8.4 px toward the camera (hole by before/after frames, dish shift removed, 20:36)
colony_px, target_px = target_px.copy(), target_px + AIM_OFFSET  # the camera aims here; colony_px stays the record
assert DRY or '--again' in sys.argv or not done(target_px), f'colony {idx} was picked already (--again to repeat)'
if DRY:
    sys.exit()

import well_reach as wr  # noqa: E402
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402
wr.DWELL_S = 0.4
servos = wr.Servos()
r = servos.read()
state = {'q': {k: r[k] for k in wr.ARM}}
a.assert_unfolded(state['q'])
cams = {'top': Camera(OVERHEAD), 'side': Camera(SIDE, paper_target=SIDE_PAPER)}


def go(cmd):
    q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
    assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300 and q[2] < 600, q
    assert cmd[1] - Z_OFF - a.Z_PAPER >= min(FLOOR_MM, AGAR) - 0.01, f'{cmd} is under the floor'  # never into the dish bottom
    above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != state['q'][1] else {})}
    wr.jgo(servos, above, state['q'])
    wr.jgo(servos, q, above)
    state['q'] = q


def arrive(c):
    """Every pose near the agar from the same side, up over the rim: the tip flipped 25 rows (4 mm) with the direction of
    a small reach correction (11-04, the wells' backlash). Up 4 mm clears the 14.5 mm wall from 8 mm over 4 mm agar."""
    go(tuple(c + np.array([-12.0, 4.0, 0.0])))  # 12 short as the wells: 6 still flipped between rows 584 and 643 (11-04)
    go(tuple(c))


GAP_ROWS = 40
BLOB_TH, BLOB_GROW, BLOB_D = 25, 15, 45  # colony: this much over the median, grown by this (its halo); needle there reads over BLOB_D
SHADOW = False  # room light off (11-04): no arm shadow, and the needle reads darker than the agar
NEEDLE_TH = 0.2  # of th, on the shadow-free difference (replay on the 11-03 shadow frames)
ALONG = 0.5  # overhead rows per mm of needle, new camera vs the old one the rows below were tuned on: 8 mm of height is
             # 19 rows now (23 deg from vertical at 4K), 38 then (50 deg at 1080p); px/mm across the needle stayed ~6


DISH_ROI = (452, 788, 1883, 2219)  # rows, columns of the dish in the overhead (fourth plate, 11-04)


def bg_like(frame, bgs):
    """The background moved onto the frame: the whole view shifts ~1.8 rows when the arm reaches out (table flex, 11-04)
    and the dish edges then read as a needle in the difference; the trace ran off along the rim."""
    y0, y1, x0, x1 = DISH_ROI
    a_, b_ = (cv2.cvtColor(i[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32) for i in (bgs, frame))
    (dx, dy), _ = cv2.phaseCorrelate(a_, b_, cv2.createHanningWindow((x1 - x0, y1 - y0), cv2.CV_32F))
    if max(abs(dx), abs(dy)) > 6:  # the needle won the correlation
        return bgs
    return cv2.warpAffine(bgs, np.float32([[1, 0, dx], [0, 1, dy]]), bgs.shape[1::-1])


def follow_tip(frame, bgs, guess, th=35, up=260, down=200, half=120, path=False, along=None):
    """Needle tip by following the needle: brightness-matched difference to the background (exposure drifts with the
    daylight), the needle's column in the rows well above the guess, then row by row down while the needle continues
    (column allowed to drift 2 px a row for its slant). The 'lowest changed blob' picked dish-rim glints and box junk
    out at the dish, in both views (2026-11-02)."""
    along = ALONG if along is None else along  # the side camera passes 1: its rows did not change
    H, W = frame.shape[:2]
    x0, x1 = max(int(guess[0]) - half, 0), min(int(guess[0]) + half, W)
    y0, y1 = max(int(guess[1] - up * along), 0), min(int(guess[1] + down * along), H)
    f, b = frame[y0:y1, x0:x1].astype(float), bgs[y0:y1, x0:x1].astype(float)
    f *= np.median(b, axis=(0, 1)) / np.maximum(np.median(f, axis=(0, 1)), 1)
    if along < 1:  # overhead lamps: the arm's shadow sweeps the dish (11-03 night). Its edge is a step along a row, the
        raw = (f - b).mean(axis=2)  # needle a thin line: a wide median along the row keeps the step, loses the line
        pad = np.pad(raw, ((0, 0), (20, 20)), mode='edge')
        d = raw - np.median(np.lib.stride_tricks.sliding_window_view(pad, 41, axis=1), axis=-1)
        strength = d if SHADOW else np.abs(d)
        if not SHADOW:  # 11-04: a colony's edge never quite cancels and the trace ran from the tip into it; a needle over a colony is far stronger
            g_ = b.mean(axis=2)
            blob = cv2.dilate((g_ > np.median(g_) + BLOB_TH).astype(np.uint8), np.ones((BLOB_GROW, BLOB_GROW), np.uint8)) > 0
            blob[:max(int(guess[1] - 80 * along) - y0, 20)] = False  # up there the bright thing is the paper, and the needle is found on it
            strength = np.where(blob & (strength < BLOB_D), 0, strength)
        d = strength > th * NEEDLE_TH  # brighter only under the room light: the needle's thin shadow carries on past the tip
    else:
        d = np.abs(f - b).max(axis=2) > th
    top_rows = d[: max(int(guess[1] - 80 * along) - y0, 20)]
    col = np.convolve(top_rows.sum(axis=0), np.ones(5), 'same')
    if col.max() < 0.3 * top_rows.shape[0] * 5:
        return None
    cx, gap, last, trace = float(np.argmax(col)), 0, None, []
    for r_ in range(d.shape[0]):
        lo, hi = int(cx) - 4, int(cx) + 5
        seg = np.nonzero(d[r_, max(lo, 0):hi])[0]
        if len(seg):
            cx = max(lo, 0) + seg.mean(); last, gap = (cx, r_), 0
            trace.append((x0 + cx, y0 + r_))
        elif last is not None:
            gap += 1
            if gap > GAP_ROWS * along:  # bridges the dark band, which vanishes against the black mat (6 stopped at the band)
                break
    if path:
        return trace
    return None if last is None else (float(x0 + last[0]), float(y0 + last[1]))


def dark_tip(frame, bgs, guess, th=12, half=110, up=130, down=70):
    """Lowest point of the whole dark needle against the background: a third tip reading. The traces run down one edge of
    the wide needle and leave it at a colony; this one needs the needle darker than the agar (room light off, 11-04)."""
    x0, x1, y0, y1 = int(guess[0]) - half, int(guess[0]) + half, int(guess[1]) - up, int(guess[1]) + down
    f, b = frame[y0:y1, x0:x1].astype(float), bgs[y0:y1, x0:x1].astype(float)
    f *= np.median(b, axis=(0, 1)) / np.maximum(np.median(f, axis=(0, 1)), 1)
    raw = (f - b).mean(axis=2)
    g_ = b.mean(axis=2)
    blob = cv2.dilate((g_ > np.median(g_) + BLOB_TH).astype(np.uint8), np.ones((BLOB_GROW, BLOB_GROW), np.uint8)) > 0
    blob[:up - 60] = False  # the paper up there
    m = cv2.morphologyEx(((raw < -th) & ~(blob & (raw > -BLOB_D))).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))  # colony edges never cancel
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 21)))  # across the bright ring
    n, lab, st, _ = cv2.connectedComponentsWithStats(m)
    top = [i for i in range(1, n) if st[i, 1] <= 2]  # it comes in from above
    if not top:
        return None
    ys, xs = np.nonzero(lab == max(top, key=lambda k: st[k, 4]))
    return x0 + float(xs[ys >= ys.max() - 1].mean()), y0 + float(ys.max())


def mirror_tip(frame, bgs, trace, reach=480):
    """Overhead tip and its mirror image in the wet agar, along the traced needle line: (tip px, mirror-tip px or None)
    or None.
    The agar point straight under the tip is their midpoint and their gap is 2x the parallax of the height, so neither
    needs the camera model. Ridge (thin line brighter than 6 px either side) of the brighter-than-background difference:
    colonies cancel against the background and are too wide to be ridges. The needle reads > 15, the mirror 7-40."""
    if trace is None or len(trace) < 30 * ALONG:
        return None
    pts = np.array(trace[:-5], np.float32)
    vx, vy, lx, ly = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    xat = lambda y: lx + (y - ly) * vx / vy
    H, W = frame.shape[:2]
    y0 = int(trace[0][1])
    y1 = min(y0 + int(reach * ALONG), H - 3)
    xa, xb = int(min(xat(y0), xat(y1))) - 20, int(max(xat(y0), xat(y1))) + 21
    if xa < 0 or xb > W or y1 - y0 < 60 * ALONG:
        return None
    f, b = frame[y0:y1, xa:xb].astype(float), bgs[y0:y1, xa:xb].astype(float)
    f *= np.median(b, axis=(0, 1)) / np.maximum(np.median(f, axis=(0, 1)), 1)
    d = cv2.GaussianBlur((f - b).max(axis=2), (5, 5), 0)
    r = d - 0.5 * (np.roll(d, 6, axis=1) + np.roll(d, -6, axis=1))
    p = np.array([r[i, int(xat(y0 + i)) - xa - 5:int(xat(y0 + i)) - xa + 6].max() for i in range(y1 - y0)])
    runs = []  # strong runs (gaps < 6 rows bridged)
    for i in np.nonzero(p > 15)[0]:
        if runs and i - runs[-1][1] < 6 * ALONG:
            runs[-1][1] = i
        else:
            runs.append([i, i])
    groups = []  # runs >= 12 rows (colony-edge residue is <= 10), joined over gaps < 25 rows (the dark band, the dish rim
    for q in (q for q in runs if q[1] - q[0] >= 12 * ALONG):  # line); the needle is the longest group. The mirror starts >= 50 px
        if groups and q[0] - groups[-1][1] < 25 * ALONG:       # under the tip at 8 mm (2x parallax), so it stays apart
            groups[-1][1] = q[1]
        else:
            groups.append(list(q))
    if not groups:
        return None
    tip = max(groups, key=lambda q: q[1] - q[0])[1]
    # in dim light the last ~10 px of the needle read only as an edge (one side +, one side -), not a ridge: follow
    # the absolute difference down, at most 20 rows (colony 5 miss, 17:50 2026-11-02: tip read 10 px short)
    ad = cv2.GaussianBlur(np.abs(f - b).mean(axis=2), (5, 5), 0)
    xc = int(xat(y0 + tip)) - xa
    tail = int(20 * ALONG)
    noise = np.median(np.hstack([ad[tip:tip + tail, max(xc - 18, 0):xc - 11], ad[tip:tip + tail, xc + 12:xc + 19]]))
    for _ in range(tail):  # needle edge 5-9 over agar noise 1-3 (dim); daylight noise 5-6 stops it at once
        i = tip + 1
        if i >= len(p) or ad[i, int(xat(y0 + i)) - xa - 5:int(xat(y0 + i)) - xa + 6].max() < max(5, 2 * noise + 2):
            break
        tip = i
    tip_px = (float(xat(y0 + tip)), float(y0 + tip))
    span = int(24 * ALONG)
    for i in range(tip + int(8 * ALONG), len(p) - span):  # mirror tip: first 24 rows with >= 16 faint ridge hits, starting on one
        if p[i] > 7 and (p[i:i + span] > 7).sum() >= 2 * span // 3:  # (colony-edge residue: ~10 rows)
            return tip_px, (float(xat(y0 + i)), float(y0 + i))
    return tip_px, None


BAND_MM = 15.3  # the needle's lower dark band above the tip (CLAUDE.md key numbers)


def band_px(frame, trace, lo=30, hi=170):
    """Centre of the lower dark band on the traced needle: the longest run along the needle darker than the needle's
    own median by 35 grey levels, between lo and hi px above the traced end. None if not found."""
    if len(trace) < lo + 10:
        return None
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(float)
    vals = np.array([g[int(y), max(int(x) - 1, 0):int(x) + 2].mean() for x, y in trace])
    end = len(trace) - 1
    sel = range(max(end - hi, 0), end - lo)
    med = np.median(vals[max(end - hi, 0):end])
    dark = [i for i in sel if vals[i] < med - 35]
    if not dark:
        return None
    runs, run = [], [dark[0]]
    for i in dark[1:]:
        if i - run[-1] <= 2:
            run.append(i)
        else:
            runs.append(run); run = [i]
    runs.append(run)
    best = max(runs, key=len)
    if len(best) < 3:
        return None
    return np.array(trace[best[len(best) // 2]], float)


def side_tip(frame, bgs, guess):
    return follow_tip(frame, bgs, guess, along=1.0)


def tips():
    time.sleep(0.5)
    imgs = {n: c.frame() for n, c in cams.items()}
    if not DRY:
        r_ = servos.read()
        say(f'  servos cmd {state["q"]} read', {k: r_[k] for k in wr.ARM})
    bg_top = bg_like(imgs['top'], bg['top'])
    tr = follow_tip(imgs['top'], bg_top, state['top_guess'], path=True)
    dry = '--no-mirror' in sys.argv  # dry agar: no reflection, a false pair moved the aim 17 px (11-04); keep its ridge tip
    m = mirror_tip(imgs['top'], bg_top, tr) if h_model <= 10.0 or dry else None  # at 20 mm junk pairs passed the gate (colony 0)
    # two tip readings that fail in opposite cases: the ridge tip stops at the dark band when the needle under it is faint,
    # the trace end stops at the band (top-left colony) or overshoots into the mirror (first run). Take the one nearest
    # where the tip should be: after the last move (expect), else aligned over the colony
    ref = expect if expect is not None else target_px + par_at(h_model)
    te = tr[-1] if tr else None
    cands = [c for c in (te, m and m[0]) if c is not None]
    if m and len(cands) == 2 and np.linalg.norm(np.array(m[0]) - ref) > 40 and             np.linalg.norm(np.array(te) - ref) < np.linalg.norm(np.array(m[0]) - ref):
        m = None  # the ridge tip lost (at the band, ~150 px off): its mirror pair is junk too. Nearer than 40 px a mirror
                  # pair beats the trace end, which stopped 9 px short on the top-left colony and clipped its edge
    if dry and m:
        m = (m[0], None)
    if m and m[1] is None:
        cands, m = [c for c in (te, m[0]) if c is not None], None
    h_mir = (m[1][1] - m[0][1]) * h_model / (2 * np.linalg.norm(par_at(h_model))) if m else None
    # junk pair: a colony edge or the dish wall as the mirror (colony 0, 2026-11-02). Real pairs at 8 mm read 5.6-9.4 mm
    if m and not 0.65 * h_model < h_mir < 1.4 * h_model:
        say(f'mirror pair at {h_mir:.1f} mm ignored')
        m = h_mir = None
    dk = dark_tip(imgs['top'], bg_top, ref)
    if dk and not SHADOW and dry and np.linalg.norm(np.array(dk) - ref) < 40:  # (past 40 it stopped over the black sheet, far from the agar: the traces then) the traces locked on a colony 30-40 rows past the tip and the expectation followed them (11-04)
        cands = [dk]
    elif dk:
        cands.append(dk)
    t = m[0] if m else (min(cands, key=lambda c: np.linalg.norm(np.array(c) - ref)) if cands else None)
    s = side_tip(imgs['side'], bg['side'], state['side_guess'])
    return imgs, t, s, m, h_mir


_reads = []  # the colour gates are tuned to one exposure; a brighter afternoon frame (median V 57 vs 35) failed them all,
for _g in np.arange(0.5, 1.51, 0.05):  # so scan gains and take the median of the plausible readings (2026-11-02)
    _b = np.clip(bg['side'].astype(float) * _g, 0, 255).astype(np.uint8)
    _span = dish_cx_side(_b)
    _a = agar_height(_b, _span) if _span is not None else None
    if _a is not None and 2.0 < _a < 12.0:
        _reads.append(_a)
AGAR = float(np.median(_reads)) if len(_reads) >= 3 else None
if '--agar' in sys.argv:  # same dish, light the colour gates miss (orange evening light 2026-11-02): reuse its earlier reading
    AGAR = float(arg('--agar')[0])
if AGAR is None:
    say('agar height: not measured on the side background (dish front wall): stop'); sys.exit(1)
DEPTH = min(DEPTH, AGAR - 2.0)  # never deeper than ~2 mm over the paper (AGAR_OFFSET applies to both)
say(f'agar surface {AGAR:.1f} mm over the paper (side view, dish as the ruler); pierce {DEPTH:.1f} mm')
START_OVER_AGAR = 20.0  # first hover: ~10 mm over the dish wall; from ~30 mm the tip sat in front of the far rim in the
                       # overhead and its reading jumped 30-145 px (2026-11-02)
if '--start' in sys.argv:  # far-row colonies: at 20 mm their tip reads over the far rim; 12 mm is ~2 mm over the wall
    START_OVER_AGAR = float(arg('--start')[0])
    assert 12.0 <= START_OVER_AGAR <= 20.0, START_OVER_AGAR
g = ground(target_px, -AGAR)
h_model = START_OVER_AGAR                    # height over the agar surface as the arm model reaches it
cmd = np.array(a.model_of(np.array([g[0], g[1], -(AGAR + AGAR_OFFSET + h_model)])), float)
model0 = np.array(a.model_of(np.array([g[0], g[1], -(AGAR + AGAR_OFFSET)])), float)  # plain model at the agar surface: the taught offset is relative to this
# 11-03 evening: yesterday's picking (7 hits on 11-02) plus only the reboot corrections below (user: 'like yesterday')
try:  # the arm model's height error, measured by the side camera at the wells (+10-12 mm after the 11-03 reboot)
    Z_OFF = float(json.load(open(f'{S}/well_z_offset.json'))['z_off'])
except (OSError, ValueError, KeyError):
    Z_OFF = 0.0
cmd[1] += Z_OFF


def reach_pan_shift():
    """The wells' learnt entry minus the arm model, now vs 11-02 (when the colony picks hit with no shift): after the 11-03
    reboot the wells learnt +12 mm reach / +10 pan more, and the colony needle stood ~12 mm short at the far rim (13:50)."""
    import plate_rig as pr
    base = np.array([12.0, 4.5])  # x, pan learnt-minus-model on 11-02 (B12 C12 E12 A11)
    poses = json.load(open(f'{S}/well_poses.json'))
    d = [np.array(r[-1]['entry'])[[0, 2]] - np.array(a.model_of(np.array(pr.well(w))))[[0, 2]]
         for w, r in poses.items() if r and time.time() - time.mktime(time.strptime(r[-1]['when'], '%Y-%m-%d %H:%M:%S')) < 3600]  # latest only: the 11:xx entries were 4 mm less
    extra = np.array([6.5, 5.0])  # the dish needs more than the wells: hover photo at +14.3/+7.2 still 35 px short (13:57)
    return np.median(d, axis=0) - base + extra if len(d) >= 2 else np.zeros(2)  # B12 + A11: dipped at every start


XP = np.zeros(2)  # a1lib re-measured on the new layout, 2026-11-03 night; before: [20.1, 18.7], frozen 11-03 20:46: the shift of the hits at 20:38/20:42; reach_pan_shift() went to 0 when the well poses aged past 1 h
cmd[0] += XP[0]; cmd[2] += XP[1]
say(time.strftime('%H:%M:%S'), 'colony', idx, 'px', target_px.round(1), 'start cmd', cmd.round(1), f'(height {Z_OFF:+.1f}, reach {XP[0]:+.1f}, pan {XP[1]:+.1f})')
if h_model < 20.0:  # arrive at 20 mm (clear of the wall), then straight down: the joint path may dip
    go(tuple(cmd + np.array([0, 20.0 - h_model, 0])))
if '--near' in sys.argv:  # user, 11-03 20:20: go near the colony on the model alone, hold there; they guide (colony_step.py)
    if not DRY:
        go(tuple(cmd + np.array([0, 20.0 - h_model, 0])))
        go(tuple(cmd))
    json.dump({'cmd': [float(v) for v in cmd], 'q': state.get('q'), 'h_over_agar': h_model, 'agar': AGAR + AGAR_OFFSET,
               'model0': model0.tolist()}, open(f'{S}/colony_hold.json', 'w'))
    say(f'holding near: {h_model:.1f} mm over the agar (model), cmd {cmd.round(1)}')
    sys.exit(0)
if '--planned' in sys.argv:  # replay the pre-measured aligned 8 mm pose (no tracer), then the 8 mm drop of picks 9/10
    plan = json.load(open(PLAN))
    best = min(plan, key=lambda p: np.linalg.norm(np.array(p['px']) - colony_px))
    assert np.linalg.norm(np.array(best['px']) - colony_px) < 6, f'colony {colony_px} not in {PLAN}'
    c8 = np.array(best['cmd'])
    say(f"planned pick: measured pose {c8.round(1)} at {best['h']:.1f} mm (err {best['err_px']} px, {best['when']})")
    go(tuple(c8 + np.array([0, 12.0, 0])))
    go(tuple(c8))
    say(f"pick: planned {best['h']:.1f} mm over the agar; down {best['h'] + DEPTH:.1f} mm ({DEPTH} into the agar)")
    go(tuple(c8 + np.array([0, -(best['h'] + DEPTH), 0])))
    time.sleep(0.5)
    top, side = cams['top'].frame(), cams['side'].frame()
    u = target_px.astype(int)
    photo = f'{S}/pick_planned_{time.strftime("%m%d_%H%M%S")}.jpg'
    cv2.imwrite(photo, cv2.resize(top[u[1] - 180:u[1] + 70, u[0] - 125:u[0] + 125], (500, 500)))
    say('picked; photo', photo)
    json.dump(picked + [colony_px.round(1).tolist()], open(PICKED, 'w'))
    go(tuple(c8 + np.array([0, 25.0 - best['h'], 0])))
    say('done, needle up')
    sys.exit(0)
if '--taught' in sys.argv:  # user, 11-03 20:20: they guided one pick by eye; every pick replays that offset, no camera loop
    final = model0 + np.array(json.load(open(f'{S}/colony_taught.json'))['delta'])
    say(f'taught pick: final cmd {final.round(1)}')
    if DRY:
        sys.exit(0)
    go(tuple(final + np.array([0, 20.0, 0])))
    go(tuple(final))
    time.sleep(0.5)
    top = cams['top'].frame()
    u = target_px.astype(int)
    photo = f'{S}/pick_taught_{time.strftime("%m%d_%H%M%S")}.jpg'
    cv2.imwrite(photo, cv2.resize(top[u[1] - 180:u[1] + 70, u[0] - 125:u[0] + 125], (500, 500)))
    say('picked; photo', photo)
    json.dump(picked + [colony_px.round(1).tolist()], open(PICKED, 'w'))
    go(tuple(final + np.array([0, 25.0, 0])))
    say('done, needle up')
    sys.exit(0)
arrive(cmd)  # the same side from the first pose: the first correction moved 47 rows for a predicted 18 (11-04)
state['top_guess'] = a.project('top', np.array([g[0], g[1], -(AGAR + h_model)]))
state['side_guess'] = a.project('side', np.array([g[0], g[1], -(AGAR + h_model)]))


def par_at(h):
    gz = ground(target_px, -AGAR)
    return a.project('top', gz + np.array([0, 0, -h])) - a.project('top', gz)


MAX_OFF_PX = 90  # ~15 mm; the model start lands within ~20 px
par_corr = np.zeros(2)  # model-parallax offset measured by the last mirror reading
skip4 = False  # set when the 4 mm stage failed: pick from the aligned 8 mm pose
expect = None  # where the overhead tip should be after the last move (px): readings far from it are junk
ups = 0  # returns from 4 to 8 mm
try:
    for step in range(30):
        seen = []
        for retry in range(3):
            imgs, t, s, m, h_mir = tips()
            first = target_px + par_at(h_model) if expect is None else None  # the model start is good to ~25 px: a far first reading is junk
            if t is not None and np.linalg.norm(np.array(t) - (expect if first is None else first)) < (40 if first is None else 90):
                break
            if False and h_model >= 8.0 and t is not None and seen and np.linalg.norm(np.array(t) - seen[-1]) < 5:  # off 11-04: both times it fed junk in; not at 4 mm:
                # there junk repeats were accepted and the arm wandered 150-245 px at 4 mm over the agar (F11, 21:50)
                say(f'step {step}: tip {t} repeats (the step model was off, not the camera): accepted'); break
            if t is not None:
                seen.append(np.array(t))
            cv2.imwrite(f'{S}/cp_{step:02d}_rej{retry}_top.jpg', imgs['top'])  # rejected readings, for replay
            say(f'step {step}: tip reading {t} far from the expected {(target_px + par_at(h_model) if expect is None else expect).round(0)}, retake')
        else:
            t = None
        low_bad = h_model < 8.0 and (t is None or np.linalg.norm(target_px + par_at(h_model) - np.array(t) + par_corr) > 20)  # + the mirror-measured model offset (19:33)
        if low_bad:  # at 4 mm only small corrections: back to the aligned 8 mm pose and start the 4 mm stage again
            ups += 1
            if ups > 2:
                raise RuntimeError('4 mm alignment unstable twice')
            say(f'step {step}: 4 mm reading {t} implausible or > 20 px off: back to the aligned 8 mm pose, pick from there')
            cmd, h_model, expect = cmd8.copy(), 8.0, t8  # 11-03: the 4 mm reading failed run after run while 8 mm sat on
            go(tuple(cmd))                               # target (0 px); user: 'it would have worked going further down'
            skip4 = True
            target_px = colony_px + AIM_OFFSET
            continue
        if t is None:
            say(f'step {step}: no plausible overhead tip: stop'); break
        if np.linalg.norm(np.array(t) - (target_px + par_at(h_model))) > MAX_OFF_PX:  # 11-03 night: shadow-edge readings 'repeated'
            say(f'step {step}: tip {t} more than {MAX_OFF_PX} px from the aim: stop'); break  # and walked the arm off the dish
        state['top_guess'] = t
        if s is not None:
            state['side_guess'] = s
        cv2.imwrite(f'{S}/cp_{step:02d}_top.jpg', imgs['top'])  # debug
        cv2.imwrite(f'{S}/cp_{step:02d}_side.jpg', imgs['side'])  # true height at the hold (11-04: the dish slid at a commanded 4 mm)
        e_model = target_px + par_at(h_model) - np.array(t)  # via the camera model's parallax
        if m:  # the agar point under the tip: midpoint of tip and mirror tip, no camera model in it
            e = target_px - (np.array(m[0]) + np.array(m[1])) / 2
            par_corr = e - e_model  # 11-03: the model read ~25 px off the mirror (stale camera model): the two
            say(f'step {step}: {h_model:.1f} mm over the agar (mirror {h_mir:.1f}), xy off {e.round(1)} px')  # alternated
        else:
            e = e_model + par_corr
            say(f'step {step}: {h_model:.1f} mm over the agar, xy off {e.round(1)} px (no mirror{", corrected" if par_corr.any() else ""})')
        if np.linalg.norm(e) > (20.0 if h_model > 13.0 else 6.0 if h_model > 6.0 else 6.5):  # 8 mm: +-10 px noise (19:42)  # coarse at the start height (+-17 px noise there, 19:38); one pan pulse ~5 px out at the dish
            J = a.image_jacobian(tuple(cmd))
            cap = 3.0 if h_model >= 8.0 else 1.0
            dx, dp = np.clip(0.6 * np.linalg.lstsq(J, e, rcond=None)[0], -cap, cap)  # capped: one junk reading asked 145 px
            if 0 < abs(dp) < 1.0 and abs(e[0]) > 4.0:  # under one pulse the pan never moved (stuck 7 px, 19:32)
                dp = float(np.sign(dp))
            cmd = cmd + np.array([dx, 0.0, dp])
            arrive(cmd)
            expect = np.array(t) + J @ np.array([dx, dp])
            continue
        nxt = None if skip4 else next((v for v in (12.0,) if v < h_model - 0.5), None)  # 11-04: last sideways at 8 mm, the pan swing at 4 mm slid the loose dish 1.7 mm; then straight down
        if nxt is not None:
            if nxt < 8.0:
                cmd8, t8 = cmd.copy(), np.array(t)
                target_px = colony_px + AIM4  # the 4 mm drop lands off (AIM4); expect below is relative, unchanged
            expect = None  # 11-04: a straight descent also pulls the tip ~1-2 rows/mm toward the robot, so the first reading down there is judged against the aim (60 px), not this step
            cmd = cmd + np.array([0, -(h_model - nxt), 0]); h_model = nxt
            arrive(cmd)
            continue
        if MEASURE:  # user 11-03 20:55: pre-measure each colony (aligned 8 mm pose), pick the clean ones from the list later
            plan = json.load(open(PLAN)) if os.path.exists(PLAN) else []
            plan = [p for p in plan if np.linalg.norm(np.array(p['px']) - colony_px) > 6]
            plan.append({'px': colony_px.round(1).tolist(), 'cmd': [float(v) for v in cmd], 'h': h_model,
                         'err_px': [float(v) for v in e], 'steps': step, 'when': time.strftime('%H:%M:%S')})
            json.dump(plan, open(PLAN, 'w'), indent=1)
            say(f'measured: aligned {h_model:.1f} mm, err {e.round(1)} px in {step} steps; saved to {PLAN}')
            break
        if '--hold' in sys.argv:  # stop aligned at 3 mm, keep the needle there (stepping by hand: colony_step.py)
            json.dump({'cmd': [float(v) for v in cmd], 'q': state['q'], 'h_over_agar': h_model, 'agar': AGAR + AGAR_OFFSET, 'model0': model0.tolist()},
                      open(f'{S}/colony_hold.json', 'w'))
            say(f'holding {h_model:.1f} mm over the agar; state in {S}/colony_hold.json')
            sys.exit(0)
        say(f'pick: aligned {h_model:.1f} mm over the agar; down {h_model + DEPTH:.1f} mm ({DEPTH} into the agar)')
        go(tuple(cmd + np.array([0, -(h_model + DEPTH), 0])))
        time.sleep(0.5)
        top, side = cams['top'].frame(), cams['side'].frame()
        photo = f'{S}/pick_{idx}_{time.strftime("%m%d_%H%M%S")}.jpg'
        u = target_px.astype(int)
        sg = state['side_guess']
        cv2.imwrite(photo, cv2.hconcat([cv2.resize(top[u[1] - 180:u[1] + 70, u[0] - 125:u[0] + 125], (500, 500)),
                                        cv2.resize(side[int(sg[1]) - 150:int(sg[1]) + 100, int(sg[0]) - 125:int(sg[0]) + 125], (500, 500))]))
        say('picked; photo', photo)
        json.dump(picked + [colony_px.round(1).tolist()], open(PICKED, "w"))
        break
except (AssertionError, RuntimeError) as err:  # never leave the needle low: lift straight up on any failure
    say('error:', err, '- lifting')
go((cmd[0], cmd[1] + 25.0, cmd[2]))  # up, clear of the dish
say('done, needle up')
