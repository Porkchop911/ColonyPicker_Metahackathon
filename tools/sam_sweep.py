"""Which lamp setting lets SAM find the dish and its colonies? No arm motion: run it with the needle out of view.
usage: <venv python> tools/sam_sweep.py [--images F.jpg ...]   Frames and overlays: evidence/tip/sw_<tag>_{top,sam}.jpg
Restores lamps.json afterwards."""
import json
import sys
import time

import cv2

sys.path.insert(0, 'tools')
import colony_detect as cd  # noqa: E402
import sam_detect  # noqa: E402

S = 'evidence/tip'
SETTINGS = [(40, 153), (70, 153), (110, 153), (170, 153), (254, 153), (110, 400), (170, 400)]
pred = sam_detect.load()


def report(tag, frame):
    dishes, cols = cd.detect_colonies(pred, frame, threshold=0.2)
    big = [c for c in cols if c['d_mm'] >= 1.2]
    print(f'{tag}: median {int(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).mean())}, dishes '
          f'{[(round(d["score"], 2), [round(v) for v in d["box"]]) for d in dishes]}, colonies {len(cols)}, >=1.2 mm {len(big)}',
          flush=True)
    cv2.imwrite(f'{S}/sw_{tag}_sam.jpg', cd.draw(frame, dishes, cols))


if '--images' in sys.argv:
    for f in sys.argv[sys.argv.index('--images') + 1:]:
        report(f.replace('\\', '/').split('/')[-1].rsplit('.', 1)[0], cv2.imread(f))
    sys.exit()

import hue  # noqa: E402
from rig_io import Camera, OVERHEAD  # noqa: E402

cam = Camera(OVERHEAD)
keep = json.load(open(f'{S}/lamps.json'))
try:
    for bri, ct in SETTINGS:
        for lamp in ('12', '13'):
            hue.set_light(lamp, True, bri=bri, ct=ct)
        time.sleep(3)  # lamp fade, then the camera's exposure
        for _ in range(5):
            frame = cam.frame()
        tag = f'b{bri}_ct{ct}'
        cv2.imwrite(f'{S}/sw_{tag}_top.jpg', frame)
        report(tag, frame)
finally:
    for lamp in ('12', '13'):
        hue.set_light(lamp, True, bri=keep[0], ct=keep[1])
