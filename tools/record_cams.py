"""Record both cameras side by side from the demo page's shared frames (rig_io.SharedCamera) until a stop file appears.
usage: <venv python> tools/record_cams.py OUT.mp4   (stop: create evidence/tip/rec_stop)"""
import os
import sys
import time

import cv2

sys.path.insert(0, 'tools')
from rig_io import Camera, OVERHEAD, SIDE, SharedCamera  # noqa: E402

STOP = 'evidence/tip/rec_stop'
FPS = 10
out = sys.argv[1]
if os.path.exists(STOP):
    os.remove(STOP)
top, side = Camera(OVERHEAD), Camera(SIDE)
assert isinstance(top, SharedCamera) and isinstance(side, SharedCamera), 'start the demo page first (it holds the cameras)'
w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (1920, 540))
n, t0 = 0, time.time()
while not os.path.exists(STOP):
    a, b = top.frame(average=1, settle=0.0), side.frame(average=1, settle=0.0)
    img = cv2.hconcat([cv2.resize(a, (960, 540)), cv2.resize(b, (960, 540))])
    cv2.putText(img, time.strftime('%H:%M:%S'), (12, 528), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    w.write(img)
    n += 1
    time.sleep(max(0.0, t0 + n / FPS - time.time()))
w.release()
print(f'{n} frames, {n / FPS:.0f} s -> {out}')
