"""Demo from pre-measured colonies (evidence/tip/colony_plan.json, colony_pick.py --measure): each colony -> its well,
then disinfect and rinse (hard-coded wells), with the demo page's status picture kept current (screen recording).
usage: <venv python> tools/demo_planned.py U,V,WELL [U,V,WELL ...]"""
import glob
import json
import os
import subprocess
import sys
import time

import cv2
import numpy as np

S = 'evidence/tip'
PY = sys.executable
STATUS = f'{S}/demo_status.jpg'
W = json.load(open(f'{S}/demo_wells.json'))
DIS, RINSE = W['disinfect'], W['rinse']
COLOURS = {'todo': (160, 160, 160), 'now': (0, 220, 255), 'done': (0, 200, 0), 'failed': (0, 0, 255)}
LOG = open(f'{S}/demo_planned.log', 'a')
jobs = [{'px': [float(a.split(',')[0]), float(a.split(',')[1])], 'well': a.split(',')[2], 'state': 'todo'} for a in sys.argv[1:]]
wells = {DIS: 'disinfect', RINSE: 'rinse', **{j['well']: 'todo' for j in jobs}}
bg = cv2.imread(f'{S}/st2_top.jpg')


def say(*m):
    line = time.strftime('%H:%M:%S ') + ' '.join(str(x) for x in m)
    print(line, flush=True)
    LOG.write(line + '\n'); LOG.flush()


def status(line, last=None):  # same picture as colony_cycle.status_picture
    cx, cy = int(np.mean([j['px'][0] for j in jobs])), int(np.mean([j['px'][1] for j in jobs]))
    ox, oy = max(cx - 200, 0), max(cy - 160, 0)
    dish = bg[oy:oy + 320, ox:ox + 400].copy()
    for k, j in enumerate(jobs):
        u, v, c = int(j['px'][0] - ox), int(j['px'][1] - oy), COLOURS[j['state']]
        cv2.circle(dish, (u, v), 13, c, 2)
        for col, th in (((0, 0, 0), 4), (c, 2)):
            cv2.putText(dish, f"{k + 1}:{j['well']}", (u + 12, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, th)
    dish = cv2.resize(dish, (600, 480))
    plate = np.full((480, 600, 3), 245, np.uint8)
    for r, row in enumerate('ABCDEFGH'):
        for col in range(1, 13):
            w = f'{row}{col}'
            u, v = 40 + (12 - col) * 46, 60 + r * 52
            st = wells.get(w)
            fill = {'disinfect': (255, 150, 60), 'rinse': (255, 230, 120)}.get(st) or COLOURS.get(st, (225, 225, 225))
            cv2.circle(plate, (u, v), 19, fill, -1)
            cv2.circle(plate, (u, v), 19, (120, 120, 120), 1)
            cv2.putText(plate, w, (u - 14, v + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (40, 40, 40), 1)
    cv2.putText(plate, f'{DIS} disinfect   {RINSE} rinse', (40, 470), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (90, 60, 20), 1)
    photo = cv2.resize(last, (480, 480)) if last is not None else np.zeros((480, 480, 3), np.uint8)
    img = cv2.hconcat([dish, plate, photo])
    bar = np.full((50, img.shape[1], 3), 30, np.uint8)
    cv2.putText(bar, line, (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
    cv2.imwrite(STATUS, cv2.vconcat([img, bar]))
    now = next((k for k, j in enumerate(jobs) if j['state'] == 'now'), None)
    nxt = next((j['well'] for k, j in enumerate(jobs) if j['state'] == 'todo' and (now is None or k > now)), None)
    json.dump({'line': line, 'when': time.strftime('%H:%M:%S'), 'disinfect': DIS, 'rinse': RINSE,
               'colonies': [{'n': k + 1, 'px': j['px'], 'well': j['well'], 'state': j['state']} for k, j in enumerate(jobs)],
               'current': now, 'target_well': None if now is None else jobs[now]['well'], 'next_well': nxt},
              open(f'{S}/demo_state.json', 'w'))
    say(line)


def run(args):
    p = subprocess.run([PY, '-u'] + args, capture_output=True, text=True)
    for ln in (p.stdout + p.stderr).splitlines():
        if 'Warn' not in ln:
            LOG.write('  ' + ln + '\n')
    LOG.flush()
    return p.returncode == 0, p.stdout


status(f'{len(jobs)} colonies (SAM 3.1, pre-measured): ' + ', '.join(f"{k + 1}->{j['well']}" for k, j in enumerate(jobs)))
last = None
for k, j in enumerate(jobs):
    j['state'] = 'now'; wells[j['well']] = 'now'
    status(f"colony {k + 1}: picking", last)
    ok, out = run(['tools/colony_pick.py', '--px', str(j['px'][0]), str(j['px'][1]), '--again', '--agar', open(f'{S}/agar_mm.txt').read().strip(), '--depth', '2.5', '--planned'])
    photos = sorted(glob.glob(f'{S}/pick_planned_*.jpg'), key=os.path.getmtime)
    last = cv2.imread(photos[-1]) if photos else None
    if not ok or 'picked; photo' not in out:
        j['state'] = 'failed'; wells[j['well']] = 'failed'
        status(f"colony {k + 1}: pick failed - stop", last)
        break
    status(f"colony {k + 1} picked: needle into {j['well']}", last)
    run(['tools/well_tour.py', j['well'], '--blind'])
    j['state'] = 'done'; wells[j['well']] = 'done'
    status(f"colony {k + 1} in {j['well']}: disinfect {DIS}, rinse {RINSE}", last)
    run(['tools/well_tour.py', DIS, RINSE, '--blind'])
status('demo done: ' + ', '.join(f"{k + 1}->{j['well']} {j['state']}" for k, j in enumerate(jobs)), last)
