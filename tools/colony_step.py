"""Step the held needle straight down (or up) by a few mm from evidence/tip/colony_hold.json; keeps the file current.
usage: python tools/colony_step.py DZ [DREACH [DPAN]]   (mm, mm, pan pulses; DZ negative = down, DREACH + = away from the robot). Refuses to go under 2 mm over the paper (model)."""
import json
import sys

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
import well_reach as wr  # noqa: E402
from reach import ik  # noqa: E402

f = 'evidence/tip/colony_hold.json'
st = json.load(open(f))
if sys.argv[1] == 'save':  # 'drop' (user): this pose minus the plain model = the taught offset for every colony (--taught)
    d = [c - m for c, m in zip(st['cmd'], st['model0'])]
    json.dump({'delta': d, 'when': __import__('time').strftime('%Y-%m-%d %H:%M:%S')}, open('evidence/tip/colony_taught.json', 'w'))
    sys.exit(print('taught delta (reach mm, height mm, pan) =', [round(v, 1) for v in d]))
st['q'] = {int(k): v for k, v in st['q'].items()}
dz = float(sys.argv[1])
cmd = list(st['cmd'])
new_h = st['h_over_agar'] + dz
assert st['agar'] + new_h >= -0.5, f'would put the tip {st["agar"] + new_h:.1f} mm over the paper (model): refused'
cmd[1] += dz
cmd[0] += float(sys.argv[2]) if len(sys.argv) > 2 else 0.0  # reach, mm (user guiding by eye, 11-03)
cmd[2] += float(sys.argv[3]) if len(sys.argv) > 3 else 0.0  # pan, pulses
q = {1: round(cmd[2]), **ik(cmd[0], cmd[1])}
s = wr.Servos()
r = s.read()
now = {k: r[k] for k in wr.ARM}
a.assert_unfolded(now)
assert max(abs(now[k] - st['q'][k]) for k in (1, 3, 4)) <= 6, f'arm {now} is not where the hold file says {st["q"]}'
wr.jgo(s, {**q, 2: q[2] + 8}, now)
wr.jgo(s, q, {**q, 2: q[2] + 8})
json.dump({**st, 'cmd': cmd, 'q': q, 'h_over_agar': new_h}, open(f, 'w'))
print(f'cmd {[round(v, 1) for v in cmd]}; tip now {new_h:+.1f} mm over the measured agar ({st["agar"] + new_h:.1f} mm over the paper, model); q {q}')
