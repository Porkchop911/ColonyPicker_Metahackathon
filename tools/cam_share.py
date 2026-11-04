"""Hold both cameras and share their frames, as the demo page does, so tools/record_cams.py can film while a pick runs.
Stops when evidence/tip/rec_stop appears. usage: <venv python> -u tools/cam_share.py"""
import os
import sys
import time

sys.path.insert(0, 'tools')
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402

STOP = 'evidence/tip/rec_stop'
if os.path.exists(STOP):
    os.remove(STOP)
cams = [Camera(OVERHEAD, publish=True), Camera(SIDE, paper_target=SIDE_PAPER, publish=True)]
print('cameras shared', flush=True)
while not os.path.exists(STOP):
    time.sleep(0.5)
for c in cams:
    c.close()
print('cameras closed')
