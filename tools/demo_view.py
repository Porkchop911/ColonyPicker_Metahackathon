"""Audience window: shows evidence/tip/demo_status.jpg (written by tools/colony_cycle.py), refreshed twice a second.
q or Esc closes it."""
import os

import cv2

PATH = 'evidence/tip/demo_status.jpg'
cv2.namedWindow('colony picker', cv2.WINDOW_NORMAL)
seen = None
while True:
    try:
        m = os.path.getmtime(PATH)
        if m != seen:
            img = cv2.imread(PATH)
            if img is not None:
                cv2.imshow('colony picker', img)
                seen = m
    except OSError:
        pass
    if cv2.waitKey(500) & 0xFF in (ord('q'), 27) or cv2.getWindowProperty('colony picker', cv2.WND_PROP_VISIBLE) < 1:
        break
