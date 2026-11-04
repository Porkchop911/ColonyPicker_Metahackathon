"""Colony picks in a row: before each, a new background and SAM (on call), then colony_pick.py on the SAM colony nearest the
given position (the loose dish creeps between picks). Stops at the first pick that does not finish.
usage: <venv python> -u tools/chain_picks.py U,V [U,V ...] [--agar 4] [--depth 2] [--dry] [--wells] [--first-well N] [--again]
--wells: each colony then goes through the demo_wells.json loop, its own sample well -> disinfect -> rinse."""
import json
import os
import subprocess
import sys
import time

import numpy as np

S = 'evidence/tip'
PY = sys.executable
DRY = '--dry' in sys.argv
WELLS = '--wells' in sys.argv
AGAIN = ['--again'] if '--again' in sys.argv else []  # colonies picked before
arg = lambda n, d: sys.argv[sys.argv.index(n) + 1] if n in sys.argv else d
AGAR, DEPTH = arg('--agar', '4'), arg('--depth', '2')
FIRST_WELL = int(arg('--first-well', '0'))  # resume: skip this many sample wells
spots = [np.array([float(v) for v in a.split(',')]) for a in sys.argv[1:] if ',' in a]
NEAR_PX = 30  # the dish crept 6-8 px per pick


def sam_colonies():
    assert psutil.pid_exists(int(open(f'{S}/sam_ready').read())), 'start tools/sam_server.py first'
    t0 = time.time()
    open(f'{S}/sam_request', 'w').write('1')
    while time.time() - t0 < 60:
        v = f'{S}/colony_variants.json'
        if os.path.exists(v) and os.path.getmtime(v) > t0:
            return [np.array(c['px']) for c in json.load(open(v))['normal']]
        time.sleep(0.2)
    raise SystemExit('SAM on call did not answer')


import psutil  # noqa: E402

for k, spot in enumerate(spots):
    if not DRY:
        subprocess.run([PY, 'tools/bg_refresh.py', '--no-plate'], check=True)
    cols = sam_colonies()
    c = min(cols, key=lambda p: np.linalg.norm(p - spot))
    off = np.linalg.norm(c - spot)
    print(f'{time.strftime("%H:%M:%S")} pick {k + 1}/{len(spots)}: SAM colony {c.round(1)}, {off:.0f} px from {spot}', flush=True)
    if off > NEAR_PX:
        raise SystemExit('no SAM colony near that position: stop')
    r = subprocess.run([PY, '-u', 'tools/colony_pick.py', '--px', f'{c[0]:.1f}', f'{c[1]:.1f}', '--no-mirror', '--start', '12', '--agar', AGAR,
                        '--depth', DEPTH, *AGAIN, *(['--dry'] if DRY else [])], capture_output=True, text=True)
    open(f'{S}/chain_pick_{k + 1}.log', 'w').write(r.stdout + r.stderr)
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith(('picked', 'done', 'step')) or 'stop' in ln]
    print('\n'.join(lines[-3:]), flush=True)
    if not DRY and not any(ln.startswith('picked') for ln in lines):
        raise SystemExit(f'pick {k + 1} did not finish: stop, see {S}/chain_pick_{k + 1}.log')
    if WELLS:
        w = json.load(open(f'{S}/demo_wells.json'))
        plan = [w['targets'][k + FIRST_WELL], w['disinfect'], w['rinse']]
        r = subprocess.run([PY, '-u', 'tools/well_tour.py', *plan, '--blind', '--dwell', f"{w['disinfect']}=5,{w['rinse']}=2",
                            *(['--dry'] if DRY else [])], capture_output=True, text=True)
        open(f'{S}/chain_wells_{k + 1}.log', 'w').write(r.stdout + r.stderr)
        print(f'{time.strftime("%H:%M:%S")} wells {plan}:', r.stdout.strip().splitlines()[-1] if r.returncode == 0 else 'FAILED', flush=True)
        if r.returncode:  # shortcut: --blind logs 'entered' for every well, only the photos tell
            raise SystemExit(f'wells after pick {k + 1} failed: stop, see {S}/chain_wells_{k + 1}.log')
print('chain done', flush=True)
