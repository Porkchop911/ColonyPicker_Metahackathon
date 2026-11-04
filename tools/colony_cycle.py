"""Demo: every colony on the dish into its own target well, each followed by the disinfect and the rinse well.
usage: <venv python> tools/colony_cycle.py [--dry] [--agar MM] [--again] [--max N] [--dwell W=s,W=s] [--no-view] [--agent]
  --agent  Muse Spark chooses the colonies (tools/colony_agent.py) instead of the operator
  --dry    background + SAM + well assignment + status picture only, no picks or wells
  --agar   agar height (mm) when the side view cannot measure it (evening light); --again: also picked colonies

SAM runs once per dish; afterwards the dish creep (~5 px per pick) is measured on the lamp-12 backgrounds and moved
onto every colony. Colonies are picked near row first; a failed pick is skipped. The wells must not fail: a skipped
well is retried once, and if the disinfect or rinse well still fails the cycle stops (the needle would carry over unclean).
Status picture for the audience: evidence/tip/demo_status.jpg, shown by tools/demo_view.py (started here)."""
import glob
import json
import os
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import psutil

PY = sys.executable
S = 'evidence/tip'
DRY = '--dry' in sys.argv
arg = lambda name, default: sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default
WELLS = json.load(open(f'{S}/demo_wells.json'))  # wells that went in 3 times in a row (selection passes)
DIS, RINSE, TARGETS = WELLS['disinfect'], WELLS['rinse'], WELLS['targets']
DISH_C = (2051, 620)  # overhead px of the agar centre (fourth plate, 11-04)
USED = f'{S}/wells_used.json'  # target wells holding a colony already: one colony per Run demo fills them in turn (user, 11-03)
used = json.load(open(USED)) if os.path.exists(USED) else []
TARGETS = [w for w in TARGETS if w not in used]
DWELL = arg('--dwell', f'{DIS}=5,{RINSE}=2')
STOP_FILE = f'{S}/stop_demo'  # tools/demo_gui.py "Stop after this colony"
FAR_Y = 450  # overhead px: colonies farther than this start at 12 mm (at 20 mm their tip reads over the far rim); as on 11-02
STATUS = f'{S}/demo_status.jpg'


def run(args, log):
    with open(log, 'w') as f:
        subprocess.run([PY, '-u'] + args, stdout=f, stderr=subprocess.STDOUT, timeout=900)
    return open(log, errors='replace').read()


def say(*a):
    print(time.strftime('%H:%M:%S'), *a, flush=True)


def lamp12(on):
    subprocess.run([PY, '-c', f"import sys; sys.path.insert(0, 'tools'); import hue; hue.set_light('12', {on}); hue.set_light('13', {on})"])  # both lamps, as all day on 11-02 (user)
    time.sleep(2.0)  # let the lamp settle before a background (A12/B12 missed once right after a pick, 21:42)


def dish_shift(old, new, box):
    """Dish creep (px) between two lamp-12 overhead backgrounds: phase correlation on the dish; 0 if unsure."""
    x0, y0, x1, y1 = box
    g = lambda im: cv2.cvtColor(im[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY).astype(np.float32)
    (dx, dy), resp = cv2.phaseCorrelate(g(old), g(new))
    return np.array([dx, dy]) if resp > 0.3 and abs(dx) < 30 and abs(dy) < 30 else np.zeros(2)


COLOURS = {'todo': (160, 160, 160), 'now': (0, 220, 255), 'done': (0, 200, 0), 'failed': (0, 0, 255)}


def status_picture(bg, jobs, wells, line, last=None):
    """Dish with numbered colonies and their wells | 96-well plate map | the last photo; one status line under it."""
    cx = int(np.mean([j['px'][0] for j in jobs])) if jobs else 880
    cy = int(np.mean([j['px'][1] for j in jobs])) if jobs else 490
    ox, oy = max(cx - 200, 0), max(cy - 160, 0)
    dish = bg[oy:oy + 320, ox:ox + 400].copy()
    for k, j in enumerate(jobs):
        u, v = int(j['px'][0] - ox), int(j['px'][1] - oy)
        c = COLOURS[j['state']]
        cv2.circle(dish, (u, v), 13, c, 2)
        for col, th in (((0, 0, 0), 4), (c, 2)):
            cv2.putText(dish, f"{k + 1}:{j['well']}", (u + 12, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, th)
    dish = cv2.resize(dish, (600, 480))
    plate = np.full((480, 600, 3), 245, np.uint8)
    for r, row in enumerate('ABCDEFGH'):
        for col in range(1, 13):
            w = f'{row}{col}'
            u, v = 40 + (12 - col) * 46, 60 + r * 52  # column 12 on the left, as on the overhead
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
               'colonies': [{'n': k + 1, 'px': [float(v) for v in j['px']], 'well': j['well'], 'state': j['state']}
                            for k, j in enumerate(jobs)],
               'current': now, 'target_well': None if now is None else jobs[now]['well'], 'next_well': nxt},
              open(f'{S}/demo_state.json', 'w'))  # for the demo page (tools/demo_server.py)


def tour(wells_to_visit, tag):
    """well_tour.py over these wells; returns the wells it entered."""
    out = run(['tools/well_tour.py', *wells_to_visit, '--dwell', DWELL, *(['--blind'] if '--blind-wells' in sys.argv else [])], f'{S}/cycle_tour_{tag}.log')
    return {m.group(1) for m in re.finditer(r'^\s*[\d.]+s (\w+): entered', out, re.M)}


if not DRY:
    import well_reach
    well_reach.claim_arm()  # before anything moves; children inherit ARM_DRIVER
if '--no-view' not in sys.argv:
    subprocess.Popen([PY, 'tools/demo_view.py'])
lamp12(True)
run(['tools/bg_refresh.py', '--no-plate'], f'{S}/cycle_bg.log')
bg = cv2.imread(f'{S}/st2_top.jpg')
CHOICE = f'{S}/colony_choice.json'  # written by the demo page when the user picks a detection setting
if os.path.exists(CHOICE):
    os.remove(CHOICE)
def sam_variants():
    """SAM 3.1, three filter settings: from the demo page's loaded model when it runs (seconds), else in a process."""
    try:
        alive = psutil.pid_exists(int(open(f'{S}/sam_ready').read()))
    except (OSError, ValueError):
        alive = False
    if alive:
        t0 = time.time()
        open(f'{S}/sam_request', 'w').write('1')
        while time.time() - t0 < 60:
            if os.path.exists(f'{S}/colony_variants.json') and os.path.getmtime(f'{S}/colony_variants.json') > t0:
                return
            time.sleep(0.3)
        say('the page SAM did not answer in 60 s: running SAM here')
    run(['tools/colony_pick.py', '--dry', '--variants'], f'{S}/cycle_dry.log')


sam_variants()
variants = json.load(open(f'{S}/colony_variants.json'))
picked = json.load(open(f'{S}/colony_picked.json')) if os.path.exists(f'{S}/colony_picked.json') else []
fresh_px = lambda cs: sorted([np.array(c['px']) for c in cs if '--again' in sys.argv or
                              not any(np.linalg.norm(np.array(c['px']) - np.array(q)) < 15 for q in picked)],
                             key=lambda p: np.hypot(p[0] - DISH_C[0], p[1] - DISH_C[1]))  # nearest the dish centre first (user, 11-03): the rims broke the 4 mm reading; all shown, --max caps the picks
choices = {name: [p.tolist() for p in fresh_px(cs)] for name, cs in variants.items()}
say('detections: ' + ', '.join(f'{n} {len(c)}' for n, c in choices.items()))
json.dump({'line': 'Choose the detection: strict, normal or loose', 'when': time.strftime('%H:%M:%S'), 'choices': choices,
           'colonies': [], 'current': None, 'target_well': None, 'next_well': None, 'disinfect': DIS, 'rinse': RINSE},
          open(f'{S}/demo_state.json', 'w'))
if DRY:
    sys.exit()
# Run demo moves at once (user, 11-03): the dips run while the user looks at the three detections and chooses
# a dip in the disinfect well first: a clean needle, and the side camera re-measures the arm's height offset that the
# colony heights use (the agar was ploughed with a stale model height, 11-03)
got = {DIS, RINSE} if '--no-start-dip' in sys.argv else tour([DIS, RINSE], 'start')  # disinfect, then rinse, before the first colony; the demo skips it (user, 19:30: pick first)
for r in range(2):  # two retries (side readings lost at B12 in dimmer light, 14:01); a failed disinfect redoes the rinse too
    if DIS in got and RINSE in got:
        break
    run(['tools/bg_refresh.py', '--no-plate'], f'{S}/cycle_bg.log')  # a bad background fails every retry too (16:02)
    got = tour([DIS, RINSE] if DIS not in got else [RINSE], f'start_r{r}') | ({DIS} if DIS in got else set())
if DIS not in got or RINSE not in got:
    say(f'{DIS}/{RINSE} not entered at the start: stop'); sys.exit(1)
skip = json.load(open(f'{S}/colony_skip.json')) if os.path.exists(f'{S}/colony_skip.json') else []  # hand-kept: left out of the run, still shown on the page
cols = None
if '--agent' in sys.argv:  # Muse Spark chooses; if the API fails the operator chooses on the page, the run goes on
    import colony_agent
    say('Muse Spark is choosing the colonies')
    try:
        cols = [np.array(p) for p in colony_agent.decide(bg, variants, skip + ([] if '--again' in sys.argv else picked), DISH_C, min(int(arg('--max', len(TARGETS))), len(TARGETS)), say, sam=colony_agent.on_call)]
        pick = 'Muse Spark'
    except colony_agent.Refused as e:  # the critic's no is final: no operator fallback on a plate it refused
        say(f'Muse Spark refused this plate: {e}')
        status_picture(bg, [], {}, 'Muse Spark refused this plate: nothing picked')
        sys.exit()
    except Exception as e:
        say(f'Muse Spark did not decide ({e}): choose on the page')
if cols is None:
    say('waiting for the detection choice on the demo page')
    while not os.path.exists(CHOICE):  # the user looks at the three and clicks one; Stop ends the run here
        if os.path.exists(STOP_FILE):
            say('stopped before picking'); sys.exit()
        time.sleep(0.5)
    pick = json.load(open(CHOICE))['variant']
    cols = [np.array(p) for p in choices[pick] if not any(np.linalg.norm(np.array(p) - np.array(q)) < 15 for q in skip)][:int(arg('--max', len(TARGETS)))]
say(f'using the {pick} detection: {len(cols)} colonies')
jobs = [{'px': p, 'well': w, 'state': 'todo'} for p, w in zip(cols, TARGETS)]
wells = {DIS: 'disinfect', RINSE: 'rinse', **{j['well']: 'todo' for j in jobs}}
say(f'{len(jobs)} colonies: ' + ', '.join(f"{k + 1}@{j['px'].round(0)}->{j['well']}" for k, j in enumerate(jobs)))
status_picture(bg, jobs, wells, f'{len(jobs)} colonies ({pick}): needle into {DIS} and {RINSE} first')
if not jobs:
    say('no free target well left: all hold a colony (evidence/tip/wells_used.json; empty it for a new plate)' if not TARGETS
        else 'no colony to pick')
    sys.exit()
try:  # agar height: --agar, else the demo page's field (evidence/tip/agar_mm.txt), else measured by the side camera
    AGAR_MM = arg('--agar', '') if '--agar' in sys.argv else (open(f'{S}/agar_mm.txt').read().strip() or None)
except OSError:
    AGAR_MM = None
sam_bg = bg  # the frame SAM ran on
lamp12(True)
run(['tools/bg_refresh.py', '--no-plate'], f'{S}/cycle_bg.log')
bg = cv2.imread(f'{S}/st2_top.jpg')
box = (int(min(p[0] for p in cols)) - 80, int(min(p[1] for p in cols)) - 80,
       int(max(p[0] for p in cols)) + 80, int(max(p[1] for p in cols)) + 80)
d = dish_shift(sam_bg, bg, box)  # the dish moved after SAM (16:41: colonies doubled in the diff, the first pick missed)
for jj in jobs:
    jj['px'] = jj['px'] + d
say(f'dish moved {d.round(1)} px since SAM')
t_start = time.time()
for k, j in enumerate(jobs):
    if os.path.exists(STOP_FILE):
        say('stop requested: ending before colony', k + 1)
        break
    if k > 0:  # new lamp-12 background (the dish moved under the last pick): follow the creep
        lamp12(True)
        run(['tools/bg_refresh.py', '--no-plate'], f'{S}/cycle_bg.log')
        new = cv2.imread(f'{S}/st2_top.jpg')
        d = dish_shift(bg, new, box)
        for jj in jobs:
            jj['px'] = jj['px'] + d
        bg = new
        say(f'dish moved {d.round(1)} px')
    j['state'], wells[j['well']] = 'now', 'now'
    status_picture(bg, jobs, wells, f"colony {k + 1}/{len(jobs)}: picking")
    say(f"colony {k + 1} at {j['px'].round(1)} -> {j['well']}")
    for attempt in range(3):  # restart on failure (user, 11-04): a failed pick lifts the needle, nothing was pierced
        out = run(['tools/colony_pick.py', '--px', f"{j['px'][0]:.1f}", f"{j['px'][1]:.1f}", '--again', '--start', '12', '--no-mirror', '--depth', '2',  # as chain_picks.py, 11-04
                   *(['--agar', AGAR_MM] if AGAR_MM else [])], f'{S}/cycle_pick_{k}.log')
        photo = re.search(r'picked; photo (\S+)', out)
        if photo:
            break
        say(f'  pick attempt {attempt + 1} failed: {out.strip().splitlines()[-2:]}')
    if not photo:
        j['state'], wells[j['well']] = 'failed', None
        say(f'  pick failed ({out.strip().splitlines()[-2:]}), skipped')
        status_picture(bg, jobs, wells, f"colony {k + 1}: pick failed, skipped")
        continue
    say('  picked', photo.group(1))
    status_picture(bg, jobs, wells, f"colony {k + 1}: picked -> {j['well']}", cv2.imread(photo.group(1)))
    run(['tools/bg_refresh.py', '--no-plate'], f'{S}/cycle_bg.log')  # lamp stays on: switching re-locked the exposures
    plan = [j['well'], DIS, RINSE]
    got = tour(plan, k)
    missing = [w for w in plan if w not in got]
    if missing:  # one retry from the same background
        say(f'  wells not entered: {missing}, retry')
        got |= tour(missing, f'{k}r')
    j['state'] = wells[j['well']] = 'done' if j['well'] in got else 'failed'
    if j['well'] in got:
        used.append(j['well'])
        json.dump(used, open(USED, 'w'))
    say(f"  wells entered: {[w for w in plan if w in got]}")
    last = sorted(glob.glob(f"{S}/tour_{j['well']}_*.jpg"))
    status_picture(bg, jobs, wells, f"colony {k + 1}: in {j['well']}, disinfected {DIS}, rinsed {RINSE}" if j['state'] == 'done'
                   else f"colony {k + 1}: {j['well']} not reached", cv2.imread(last[-1]) if last else None)
    if DIS not in got or RINSE not in got:
        say(f"  {DIS}/{RINSE} not entered twice: stop (needle not clean)")
        status_picture(bg, jobs, wells, 'STOPPED: disinfect/rinse well not reached')
        break
done = sum(j['state'] == 'done' for j in jobs)
say(f'cycle done: {done}/{len(jobs)} colonies in their wells, {time.time() - t_start:.0f} s')
status_picture(bg, jobs, wells, f'done: {done}/{len(jobs)} colonies transferred')
json.dump([{'px': j['px'].round(1).tolist(), 'well': j['well'], 'state': j['state']} for j in jobs],
          open(f'{S}/cycle_result.json', 'w'), indent=1)
