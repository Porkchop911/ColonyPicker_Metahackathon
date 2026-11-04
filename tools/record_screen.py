"""Film the demo page's window, so the page itself is in the video, until evidence/tip/rec_stop appears.
usage: <venv python> tools/record_screen.py OUT.mp4"""
import ctypes
import os
import sys
import time
from ctypes import wintypes

import cv2
import numpy as np
from PIL import ImageGrab

STOP = 'evidence/tip/rec_stop'
TITLE = 'Colony picker'
FPS, WIDTH = 10, 1920
out = sys.argv[1]
if os.path.exists(STOP):
    os.remove(STOP)
# only the page's window, maximized and on top: the rest of the desktop stays out of the video
user32 = ctypes.windll.user32
user32.SetProcessDPIAware()
hwnd = user32.FindWindowW(None, TITLE)
assert hwnd, f'no window called {TITLE}'
user32.ShowWindow(hwnd, 3)
user32.SetWindowPos(hwnd, -1, 0, 0, 0, 0, 3)
user32.SetForegroundWindow(hwnd)
time.sleep(1.0)
r = wintypes.RECT()
user32.GetWindowRect(hwnd, ctypes.byref(r))
box = (max(r.left, 0), max(r.top, 0), r.right, r.bottom)
first = ImageGrab.grab(bbox=box)
size = (WIDTH, round(first.height * WIDTH / first.width / 2) * 2)
w = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), FPS, size)
n, t0 = 0, time.time()
while not os.path.exists(STOP):
    w.write(cv2.resize(cv2.cvtColor(np.array(ImageGrab.grab(bbox=box)), cv2.COLOR_RGB2BGR), size, interpolation=cv2.INTER_AREA))
    n += 1
    if n % FPS == 0:  # the terminal came back over the page in the last run
        user32.SetWindowPos(hwnd, -1, 0, 0, 0, 0, 3)
    time.sleep(max(0.0, t0 + n / FPS - time.time()))  # a slow grab repeats no frame: the clip runs a little fast then
w.release()
user32.SetWindowPos(hwnd, -2, 0, 0, 0, 0, 3)  # no longer on top
print(f'{n} frames, {time.time() - t0:.0f} s -> {out}')
