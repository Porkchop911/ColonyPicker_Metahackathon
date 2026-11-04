"""Offline replay of colony_pick.mirror_tip on frames from the 2026-11-02 colony 1 run (lamp 12 on, 8 mm over the agar
except 01 at 20 mm). Expected values checked by eye on the frames. Run: python tools/test_colony_pick.py"""
import cv2
import numpy as np

src = open('tools/colony_pick.py').read()
ns = {'np': np, 'cv2': cv2}
exec(src[src.index('GAP_ROWS ='):src.index('BAND_MM')], ns)
ns['ALONG'] = 1.0  # these frames are from the old overhead
bg = cv2.imread('evidence/tip/mirror_bg_top.jpg')  # that run's background (st2_top.jpg is retaken per run)
# frame: (tip y, mirror y) by eye, +-8 px
CASES = {'01': (424, 585), '12': (466, 539), '21': (453, 531), '23': (426, 493)}
for k, (ty, my) in CASES.items():
    im = cv2.imread(f'evidence/tip/mirror_{k}_top.jpg')
    m = ns['mirror_tip'](im, bg, ns['follow_tip'](im, bg, (888, 480), path=True))
    assert m is not None, k
    assert abs(m[0][1] - ty) <= 8 and abs(m[1][1] - my) <= 8, (k, m)
# colony 0 run (other background): the ridge tip stops at the dark band, ~150 px above the trace end (right here);
# tips() then takes the trace end, the one nearer the expected tip
bg = cv2.imread('evidence/tip/mirror_bg3_top.jpg')
for k in ('02', '03', '08'):
    im = cv2.imread(f'evidence/tip/mirror_c0b_{k}_top.jpg')
    tr = ns['follow_tip'](im, bg, (950, 440), path=True)
    m = ns['mirror_tip'](im, bg, tr)
    assert m is None or m[0][1] < tr[-1][1] - 30, (k, m, tr[-1])
# top-left colony: the trace end stops at the dark band (403); the ridge tip must find the real tip (439 by eye)
bg = cv2.imread('evidence/tip/mirror_bg4_top.jpg')
im = cv2.imread('evidence/tip/mirror_tl_03_top.jpg')
m = ns['mirror_tip'](im, bg, ns['follow_tip'](im, bg, (826, 449), path=True))
assert m is not None and abs(m[0][1] - 439) <= 6, m
# top-left colony, final step: the mirror pair (tip 459, mirror 526) was right, the trace end (450) stopped short
im = cv2.imread('evidence/tip/mirror_tl2_04_top.jpg')
m = ns['mirror_tip'](im, bg, ns['follow_tip'](im, bg, (830, 449), path=True))
assert m is not None and m[1] is not None and abs(m[0][1] - 459) <= 6 and abs(m[1][1] - 526) <= 8, m
# colony 5, 17:50 dim evening light (missed, user): the last ~10 px of the needle read only as an edge; tip ~523 by eye
bg = cv2.imread('evidence/tip/mirror_bg5_top.jpg')  # a frame of the run before, needle far away (no background then)
im = cv2.imread('evidence/tip/mirror_c5_03_top.jpg')
m = ns['mirror_tip'](im, bg, ns['follow_tip'](im, bg, (859, 505), path=True))
assert m is not None and abs(m[0][1] - 523) <= 4, m
# new overhead (4K, 23 deg off vertical): a synthetic needle on the real background, 110 rows long, slant as the camera
# model gives at the dish (13.6 px per 95.9 rows), its mirror 38 rows under the tip (8 mm over the agar)
ns['ALONG'] = 0.5
# 11-04: the trace ran straight on from the tip along the edge of the colony below it, no gap at all
bg = cv2.imread('evidence/tip/gap_bg2_top.jpg')
for k, tip in (('c02', (2147, 590)), ('c04', (2142, 594))):
    im = cv2.imread(f'evidence/tip/gap_{k}_top.jpg')
    tr = ns['follow_tip'](im, ns['bg_like'](im, bg), (2142, 614), path=True)
    assert np.linalg.norm(np.array(tr[-1]) - tip) < 12, (k, tr[-1])
# 11-04: whole dark needle, frames where both traces left it
for k, g, tip, b_ in (('c04', (2142, 614), (2142, 594), 'gap_bg2'), ('e07', (2007, 617), (2019, 586), 'gap_bg3'), ('f02', (2015, 615), (2031, 615), 'gap_bg4')):
    im = cv2.imread(f'evidence/tip/gap_{k}_top.jpg')
    e = ns['dark_tip'](im, ns['bg_like'](im, cv2.imread(f'evidence/tip/{b_}_top.jpg')), g)
    assert e and np.linalg.norm(np.array(e) - tip) < 6, (k, e)
# shortcut: a faint smudge touching a colony (gap_16 frame, tip 504) can still catch the trace at row 524; pick colonies without one until the trace is fixed
# 11-04: colony right under the tip at 8 mm: the trace hopped the ~15 rows of agar onto the colony edge, 40 rows past the tip
bg = cv2.imread('evidence/tip/gap_bg_top.jpg')
for k, tip in (('04', (2069, 512)), ('05', (2065, 506)), ('02', (2068, 509))):
    im = cv2.imread(f'evidence/tip/gap_{k}_top.jpg')
    tr = ns['follow_tip'](im, ns['bg_like'](im, bg), (2055, 543), path=True)
    assert np.linalg.norm(np.array(tr[-1]) - tip) < 12, (k, tr[-1])
# new plate, 11-04 morning: the view shifted 1.8 rows against the background with the arm out and the trace ran off along
# the dish rim; true tips by eye, needle dark against the agar
bg = cv2.imread('evidence/tip/shift_bg_top.jpg')
for k, guess, tip in (('00', (2008, 526), (2044, 487)), ('01', (2021, 523), (2032, 501))):
    im = cv2.imread(f'evidence/tip/shift_{k}_top.jpg')
    tr = ns['follow_tip'](im, ns['bg_like'](im, bg), guess, path=True)
    cands = [tr[-1], ns['mirror_tip'](im, ns['bg_like'](im, bg), tr)[0]]  # tips() takes the one nearest the expectation
    assert min(np.linalg.norm(np.array(c) - tip) for c in cands) < 8, (k, cands)
bg = cv2.imread('evidence/tip/layout5_top.jpg')
im = bg.copy()
tip = np.array([2090.0, 700.0])
cv2.line(im, (int(tip[0] - 16), int(tip[1] - 110)), (int(tip[0]), int(tip[1])), (235, 235, 235), 3)
cv2.line(im, (int(tip[0] + 5), int(tip[1] + 38)), (int(tip[0] + 16), int(tip[1] + 110)), (120, 120, 120), 3)
tr = ns['follow_tip'](im, bg, (tip[0], tip[1]), path=True)
assert tr is not None, 'needle not traced'
m = ns['mirror_tip'](im, bg, tr)
assert m is not None and abs(m[0][1] - tip[1]) <= 6 and m[1] is not None and abs(m[1][1] - tip[1] - 38) <= 8, m
ns['SHADOW'] = True
# overhead lamps, 11-03 night: the arm's shadow edge across the dish took the first live run off the colony (tips by eye)
bg = cv2.imread('evidence/tip/shadow_bg_top.jpg')
for k, tip in {'00': (2108, 626), '03': (2126, 572), '05': (2148, 525)}.items():
    im = cv2.imread(f'evidence/tip/shadow_{k}_top.jpg')
    tr = ns['follow_tip'](im, bg, (tip[0] + 30, tip[1] + 20), path=True)
    assert tr and np.linalg.norm(np.array(tr[-1]) - tip) < 10, (k, tr and tr[-1])
# room light off (11-04): needle dark on the agar, two-sided difference; tip by eye and from three guesses
ns['SHADOW'] = False
bg = cv2.imread('evidence/tip/dark_bg_top.jpg')
im = cv2.imread('evidence/tip/dark_00_top.jpg')
for g in ((2090, 661), (2110, 640), (2130, 680)):
    tr = ns['follow_tip'](im, bg, g, path=True)
    assert tr and np.linalg.norm(np.array(tr[-1]) - (2116, 660)) < 6, (g, tr and tr[-1])
print('ok')
