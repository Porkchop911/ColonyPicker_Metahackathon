"""Standalone pictures of SAM 3.1 detections, not part of the demo: its own overhead photo, every dish in view ringed. No arm motion, no
demo files. Stop tools/sam_server.py and the demo page first (they hold the GPU and the camera).
usage: <venv python> tools/sam_photo.py TAG [--setting tight|loose|normal|strict] [--frame F.jpg] [--dense] [--tight N,N: stricter on these dishes]
Writes evidence/tip/sam_photo_<TAG>_<n>.jpg, one per dish, and a list in sam_photo_<TAG>.json."""
import json
import sys

import cv2
import numpy as np

sys.path.insert(0, 'tools')
import colony_detect as cd  # noqa: E402
import sam_detect  # noqa: E402
from rig_io import Camera, OVERHEAD  # noqa: E402

S = 'evidence/tip'
tag = sys.argv[1]
setting = sys.argv[sys.argv.index('--setting') + 1] if '--setting' in sys.argv else 'tight'
BANDS = ((0.7, (80, 220, 80)), (0.4, (0, 215, 255)), (0.0, (70, 70, 255)))  # BGR: green, yellow, red
LEGEND = [(0.7, BANDS[0][1], 'green: score 0.70+'), (0.4, BANDS[1][1], 'yellow: 0.40-0.69'), (0.0, BANDS[2][1], 'red: under 0.40')]
ZOOM = 3
MARGIN = 0.06  # of the dish width, around each crop
TIGHT = {int(v) for v in sys.argv[sys.argv.index('--tight') + 1].split(',')} if '--tight' in sys.argv else set()
TIGHT_CONTRAST = 15
DENSE = '--dense' in sys.argv
PROMPT, MIN_SCORE = 'round white spot', 0.5
AGAR = 0.88    # of the dish radius: the agar inside the wall


def merged(points, px=10):
    """One ring per colony: overlapping tiles report the same colony a few px apart."""
    out = []
    for p, sc in sorted(points, key=lambda t: -t[1]):  # the best score of a colony stays
        if all(np.linalg.norm(p - q) >= px for q, _ in out):
            out.append((p, sc))
    return out


def contrast(p):
    """Grey level of the middle over the ring around it, as colony_detect's gate; a wall glint reads far over a colony."""
    u, v = int(p[0]), int(p[1])
    w = gray[v - 16:v + 17, u - 16:u + 17]
    return float(w[DIST <= 4].mean() - w[(DIST >= 10) & (DIST <= 14)].mean())


def colour(sc):
    return next(c for lo, c in BANDS if sc >= lo)


if '--frame' in sys.argv:
    frame = cv2.imread(sys.argv[sys.argv.index('--frame') + 1])
else:
    cam = Camera(OVERHEAD)
    frame = cam.frame()
    cam.close()
cv2.imwrite(f'{S}/sam_photo_{tag}_frame.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(float)
DIST = np.hypot(*np.mgrid[-16:17, -16:17])
pred = sam_detect.load()
x, y = cd.SAM_ORIGIN
dishes = sam_detect.detect(pred, cd._pil(frame[y:y + cd.SAM_H, x:x + cd.SAM_W]), 'petri dish', 0.3)
if DENSE:  # hundreds of 1-2 mm colonies: small tiles, tiny-object prompt, lamps at 254 for the photo (--frame); 'round white spot' found none here
    cd.SMALL, cd.LARGE, cd.TILES, cd.TILE_DIV, cd.TILE_ZOOM = 'white speck', 'white speck', 8, 5, 3
    pts = merged([(np.array(c['centroid'], float), float(c['score'])) for c in cd.detect_colonies(pred, frame, threshold=0.15)[1]], 7)
elif setting == 'tight':  # these pictures only: prompt test 11-04, the overgrown plate fell from 32 rings to 9, the others kept theirs
    cd.SMALL, cd.LARGE = PROMPT, PROMPT
    pts = merged([(np.array(c['centroid'], float), float(c['score'])) for c in cd.detect_colonies(pred, frame, threshold=MIN_SCORE)[1]])
else:
    pts = merged([(np.array(c['px']), c['score']) for c in cd.variant_picks(pred, frame)[setting]])
report = []
for n, d in enumerate(dishes):
    x0, y0, x1, y1 = (int(round(v)) for v in d['box'])
    x0, y0, x1, y1 = x0 + x, y0 + y, x1 + x, y1 + y
    c, r = np.array([(x0 + x1) / 2, (y0 + y1) / 2]), (x1 - x0) / 2
    m = int(MARGIN * (x1 - x0))
    x0, y0, x1, y1 = max(x0 - m, 0), max(y0 - m, 0), x1 + m, y1 + m
    crop = cv2.resize(frame[y0:y1, x0:x1], None, fx=ZOOM, fy=ZOOM, interpolation=cv2.INTER_LANCZOS4)
    inside = [(p, sc) for p, sc in pts if np.linalg.norm(p - c) < AGAR * r]  # not the neighbouring dish, not the glints on the wall
    if n in TIGHT:  # overgrown plate: the haze scores like a colony, only a real bump stands out from the lawn around it
        kept = [(p, sc) for p, sc in inside if TIGHT_CONTRAST < contrast(p) < 45 and np.linalg.norm(p - c) < 0.75 * r]
        print('tight', n, sorted((round(contrast(p)), round(float(sc), 2)) for p, sc in inside), '->', len(kept), flush=True)
        inside = kept
    yy, xx = np.mgrid[:crop.shape[0], :crop.shape[1]]
    out_of_dish = np.hypot((xx + 0.5) / ZOOM + x0 - c[0], (yy + 0.5) / ZOOM + y0 - c[1]) > r * 1.01
    crop[out_of_dish] = (crop[out_of_dish] * 0.25).astype(np.uint8)
    for p, sc in inside:
        u, v = int((p[0] - x0) * ZOOM), int((p[1] - y0) * ZOOM)
        col = colour(sc)
        rad, font = (17, 0.34) if DENSE else (34, 0.55)
        cv2.circle(crop, (u, v), rad, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.circle(crop, (u, v), rad, col, 1 if DENSE else 2, cv2.LINE_AA)
        text = f'{sc:.2f}'
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font, 1)
        tx, ty = min(u + rad - 8, crop.shape[1] - tw - 4), max(v - rad + 4, th + 4)
        cv2.rectangle(crop, (tx - 2, ty - th - 3), (tx + tw + 2, ty + 3), (15, 15, 15), -1)
        cv2.putText(crop, text, (tx, ty), cv2.FONT_HERSHEY_SIMPLEX, font, col, 1, cv2.LINE_AA)
    for i, (lo, c_, name) in enumerate(LEGEND):
        cv2.putText(crop, name, (12, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(crop, name, (12, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, c_, 1, cv2.LINE_AA)
    out = f'{S}/sam_photo_{tag}_{n}.jpg'
    cv2.imwrite(out, crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
    report.append({'file': out, 'colonies': len(inside), 'score': round(float(d['score']), 2), 'box': [x0, y0, x1, y1],
                   'colony_scores': sorted((round(float(sc), 2) for _, sc in inside), reverse=True)})
    print(report[-1], flush=True)
json.dump({'setting': setting, 'dishes': report}, open(f'{S}/sam_photo_{tag}.json', 'w'), indent=1)
