"""Run tools/a1_place.py over a list of wells, one after another, for the reliability tally.
usage: <venv python> tools/well_batch.py A12 B12 ...   (needle anywhere on the needle-down model)

Stops at the first abort or moved plate. Retakes the background (tools/bg_refresh.py) first, every 4 wells, and after
any entry whose plate shift passed 1 px (the plate edge then shows in the diff). Results: evidence/tip/well_results.jsonl."""
import json
import subprocess
import sys

PY = sys.executable
RESULTS = 'evidence/tip/well_results.jsonl'


def lines():
    try:
        with open(RESULTS) as f:
            return f.readlines()
    except FileNotFoundError:
        return []


def run(args):
    print('>', ' '.join(args), flush=True)
    return subprocess.run([PY, '-u', *args]).returncode


wells = sys.argv[1:]
assert run(['tools/bg_refresh.py']) == 0
in_well, since_bg = False, 0
for w in wells:
    if since_bg >= 4:
        assert run(['tools/bg_refresh.py']) == 0
        in_well, since_bg = False, 0
    n = len(lines())
    run(['tools/a1_place.py', 'evidence/tip', '--well', w] + (['--from-well'] if in_well else []))
    if len(lines()) == n:
        sys.exit(f'stopped at {w}: a1_place wrote no result (crashed?)')
    r = json.loads(lines()[-1])
    print(f'== {w}: {r["status"]} gate {r["gate_xy"]} shift {r["shift_px"]} {r["photo"]}', flush=True)
    if r['well'] != w or r['status'] != 'entered':
        sys.exit(f'stopped at {w}: {r["status"]}')
    in_well, since_bg = True, since_bg + 1
    if max(abs(v) for v in r['shift_px']) > 1.0:
        assert run(['tools/bg_refresh.py']) == 0
        in_well, since_bg = False, 0
print('batch done')
