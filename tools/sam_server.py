"""SAM 3.1 loaded once and on call (user, 11-04). Same files as the demo page's worker: a tool writes evidence/tip/sam_request,
this answers with colony_variants.json from the background frame st2_top.jpg in seconds. colony_pick.py and colony_cycle.py
use it when sam_ready names a live process, else they load SAM themselves. The demo page leaves a running one alone.
usage: <venv python> -u tools/sam_server.py"""
import json
import os
import sys
import time

import cv2

sys.path.insert(0, 'tools')
import colony_detect as cd  # noqa: E402
import sam_detect  # noqa: E402

S = 'evidence/tip'
pred = sam_detect.load()
open(f'{S}/sam_ready', 'w').write(str(os.getpid()))
print('SAM 3.1 loaded, ready', flush=True)
while True:
    if os.path.exists(f'{S}/sam_request'):
        os.remove(f'{S}/sam_request')
        t0 = time.time()
        try:
            picks = cd.variant_picks(pred, cv2.imread(f'{S}/st2_top.jpg'))
            json.dump(picks, open(f'{S}/colony_variants.tmp', 'w'), indent=1)
            os.replace(f'{S}/colony_variants.tmp', f'{S}/colony_variants.json')
            print(f'{time.strftime("%H:%M:%S")} ' + ', '.join(f'{n} {len(c)}' for n, c in picks.items()) + f' in {time.time() - t0:.1f} s', flush=True)
        except Exception as e:  # a tool waits 60 s, then loads SAM itself
            print('SAM failed:', e, flush=True)
    time.sleep(0.2)
