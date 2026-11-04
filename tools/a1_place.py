"""Closed-loop needle into A1 (2026-11-01 build). usage: a1_place.py S [--dry] [--well A1]"""
import os, sys, time, json, numpy as np, cv2
S = sys.argv[1]; sys.path.insert(0, S)
import a1lib as a
from reach import ik
DRY = '--dry' in sys.argv
WELL = sys.argv[sys.argv.index('--well') + 1] if '--well' in sys.argv else 'A1'
HOVERS = [40.0, 20.0, 10.0, 5.0]  # mm over the rim
GATE = {40.0: 2.0, 20.0: 1.5, 10.0: 1.0, 5.0: 0.8}  # 0.5 limit-cycled (one pan pulse = 0.7-1.1 mm); the H12 plate push was the pan swing, not this gate
DEPTH = 3.0                       # final: tip this far under the rim
MISS_MAX = 5.0  # H12 corner: rays miss 2-3.5 mm (camera calibration there); junk misses 28+ or sits under the rim; the floor rule rejects plate-edge junk
GAIN = 0.5  # full steps oscillated +-6 mm across A1: the pan moves the tip ~1.1 mm/pulse there, model ~0.7 (live 21:32)

rim = a.pr.well(WELL)             # sheet, z down
RESULTS = f'{S}/well_results.jsonl'
last = {}

def record(status, h=None, shift=None, photo=None):
    """One line per run: the reliability tally reads this."""
    if DRY:
        return
    with open(RESULTS, 'a') as f:
        f.write(json.dumps({'when': time.strftime('%Y-%m-%d %H:%M:%S'), 'well': WELL, 'status': status, 'stopped_at': h,
                            'gate_xy': last.get('xy'), 'shift_px': shift, 'photo': photo}) + '\n')
log = open(f'{S}/a1_place.log', 'a')
def say(*m):
    line = ' '.join(str(x) for x in m); print(line, flush=True); log.write(line + '\n'); log.flush()

if DRY:
    BIAS = np.array([3.0, -2.0, -4.0])   # simulated: true tip = prediction + bias
    def look(cmd, floor_z=None):
        return a.sheet_of(*cmd) + BIAS + np.random.normal(0, 0.15, 3), 0.3
    def go(cmd):
        q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
        assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300, q
        say('  go', q)
else:
    import well_reach as wr
    from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER
    servos = wr.Servos()
    _r = servos.read(); a.assert_unfolded({k: _r[k] for k in wr.ARM})
    cams = {'top': Camera(OVERHEAD), 'side': Camera(SIDE, paper_target=SIDE_PAPER)}
    bgs = {n: cv2.imread(f'{S}/st2_{n}.jpg') for n in cams}
    state = {'q': None}
    def go(cmd):
        q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
        assert all(0 <= v <= 1000 for v in q.values()) and q[4] >= 300, q
        now = state['q'] or {k: servos.read()[k] for k in wr.ARM}
        # backlash: end every move from 8 shoulder pulses above, and from a lower pan only when the pan changes: a pan
        # swing on the straight-down steps put the tip into the well wall and pushed the plate (2026-11-01)
        above = {**q, 2: q[2] + 8, **({1: q[1] - 10} if q[1] != now[1] else {})}
        # (pan 561 -> 559 jumped the tip 9 mm, 566 -> 563 moved it 0, live 21:34)
        wr.jgo(servos, above, now); wr.jgo(servos, q, above)
        state['q'] = q; say('  go', q)
    def look(cmd, floor_z=None):
        got = []  # median of 3 good frames: single readings scatter +-1.5 mm and the 0.5 mm gate chased them (A12, 23:3x)
        for _ in range(5):
            imgs = {n: c.frame() for n, c in cams.items()}
            X, miss, tips = a.measure(imgs, bgs, a.sheet_of(*cmd), floor_z=floor_z)
            if X is not None:
                got.append((X, miss))
            if len(got) == 3:
                break
        if got:
            X, miss = np.median([g[0] for g in got], axis=0), float(np.median([g[1] for g in got]))
        stamp = time.strftime('%H%M%S')
        for n, im in imgs.items():
            cv2.imwrite(f'{S}/pl_{stamp}_{n}.jpg', im)
        say('  tips', tips, 'miss', None if miss is None else round(miss, 2))
        return X, miss

def settle(h, cmd):
    target = rim + np.array([0, 0, -h])
    for i in range(10):
        go(cmd)
        X, miss = look(cmd, floor_z=rim[2] - 1.0)
        if X is None or miss > MISS_MAX:
            say(f'h{h:.0f} {i}: tip not measured'); return None
        e = target - X
        say(f'h{h:.0f} {i}: tip {X.round(2)} err xy {np.linalg.norm(e[:2]):.2f} z {e[2]:+.2f}')
        last['xy'] = round(float(np.linalg.norm(e[:2])), 2)
        if np.linalg.norm(e[:2]) < GATE[h] and abs(e[2]) < 1.5:
            return cmd, X
        m, d = np.array(a.model_of(X)), np.array(a.model_of(target))
        step = GAIN * (d - m)
        if abs(e[2]) < 1.0:  # height good enough: leave the shoulder alone, its backlash added scatter at 5 mm (F12)
            step[1] = 0.0
        cmd = tuple(np.array(cmd) + step)
    say(f'h{h:.0f}: not within {GATE[h]} mm'); return None

if '--from-well' in sys.argv:  # straight up out of the well first, 2 mm pieces, before any sideways motion
    up = tuple(json.load(open(f'{S}/a1_last.json'))['cmd'])
    for k in range(1, 16):  # 30 mm: clear the rims before any sideways move
        go((up[0], up[1] + 2.0 * k, up[2]))
cmd = a.model_of(rim + np.array([0, 0, -HOVERS[0]]))
say(time.strftime('%H:%M:%S'), WELL, 'rim', rim.round(1), 'first cmd', np.round(cmd, 1), 'dry' if DRY else 'LIVE')
for h in HOVERS:
    r = settle(h, cmd)
    if r is None:
        say('ABORT high'); record('abort', h); sys.exit(1)
    cmd, X = r
    if h != HOVERS[-1]:
        nxt = HOVERS[HOVERS.index(h) + 1]
        cmd = (cmd[0], cmd[1] - (h - nxt), cmd[2])
hover_cmd = cmd
# into the well: straight down from the gated 5 mm hover, in 2 mm pieces
for dz in (2.0, 2.0, 2.0, 2.0):
    cmd = (cmd[0], cmd[1] - dz, cmd[2]); go(cmd)
X, miss = look(cmd)  # in the well the side camera loses the tip: the photo, not this reading, is the proof
say('in-well tip', None if X is None else X.round(2), 'vs rim', rim.round(2), 'below rim', None if X is None else round(float(X[2] - rim[2]), 2),
    'xy off', None if X is None else round(float(np.linalg.norm(X[:2] - rim[:2])), 2))
if not DRY:  # did the entry push the plate? the far plate row, away from the needle, against the background
    g = lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY).astype(np.float32)
    # the patch: same row, column 1 = the far end, well away from the needle (column 12 as the patch put the needle
    # itself in it for columns 10-11 and read 1.5-2.7 px of false 'plate shift', 2026-11-02)
    u = a.project('top', a.pr.well(WELL[0] + '1'))
    y0, x0 = int(u[1]) - 60, int(u[0]) - 120
    (dx, dy), _ = cv2.phaseCorrelate(g(bgs['top'])[y0:y0 + 120, x0:x0 + 240], g(cams['top'].frame())[y0:y0 + 120, x0:x0 + 240])
    moved = max(abs(dx), abs(dy)) > 2
    say(f'plate shift since the background: {dx:+.1f} {dy:+.1f} px', 'PLATE MOVED' if moved else 'ok')
    top, side = cams['top'].frame(), cams['side'].frame()  # proof crop: overhead | side around the well
    u, v = a.project('top', rim), a.project('side', rim)
    crop = lambda im, c: cv2.resize(im[int(c[1]) - 150:int(c[1]) + 100, int(c[0]) - 125:int(c[0]) + 125], (500, 500))
    photo = f'{S}/well_{WELL}_{time.strftime("%m%d_%H%M%S")}.jpg'
    cv2.imwrite(photo, cv2.hconcat([crop(top, u), crop(side, v)]))
    record('moved' if moved else 'entered', shift=[round(dx, 1), round(dy, 1)], photo=photo)
    if not moved:  # learnt pose for the fluid tour (tools/well_tour.py): model (x, z, pan) of the gated hover and the entry
        pf = f'{S}/well_poses.json'
        poses = json.load(open(pf)) if os.path.exists(pf) else {}
        poses.setdefault(WELL, []).append({'hover': [float(v) for v in hover_cmd], 'entry': [float(v) for v in cmd],
                                           'gate_xy': last.get('xy'), 'when': time.strftime('%Y-%m-%d %H:%M:%S')})
        json.dump(poses, open(pf, 'w'), indent=1)
say('DONE; arm left in the well. Lift with the lift command.')
if not DRY:
    json.dump({'cmd': list(cmd)}, open(f'{S}/a1_last.json', 'w'))
