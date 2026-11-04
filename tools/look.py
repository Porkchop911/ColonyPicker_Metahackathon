"""Overhead crop around a target pixel (red circle) to evidence/tip/look.jpg. usage: python tools/look.py U V"""
import sys
sys.path.insert(0, 'tools')
import cv2  # noqa: E402
from rig_io import Camera, OVERHEAD  # noqa: E402
u, v = int(sys.argv[1]), int(sys.argv[2])
f = Camera(OVERHEAD).frame(average=1, settle=0.0)
c = f[v - 100:v + 60, u - 100:u + 100].copy()
cv2.circle(c, (100, 100), 8, (0, 0, 255), 1)
cv2.imwrite('evidence/tip/look.jpg', cv2.resize(c, (800, 640)))
