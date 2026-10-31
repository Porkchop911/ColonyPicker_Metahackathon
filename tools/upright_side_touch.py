"""EXPERIMENTAL (2026-10-29 night, branch tipcam-external): Side-camera-only touch: small fixed model steps down (pitch held); contact = the needle's end stops going down in the
side view for 2 steps. No overhead matching (thick ink made the two cameras pick points ~5 mm apart)."""
import sys, time; sys.path.insert(0, 'tools')
import cv2, numpy as np, sam_detect as sam, hwr_kinematics as hk
from pathlib import Path
from rig_io import Camera, SIDE, SIDE_PAPER
from tip_descend import read, send
S = Path(sys.argv[1]); STEP_MM = 0.5; START = {2: 409, 3: 195, 4: 356}  # elbow +15 over the last run: the ~10 mm settle drop reached the paper from 180; STEP_MM = 0.5  # model mm; the tip moved ~3.5x that today
say = lambda *a: print(time.strftime('%H:%M:%S'), *a, flush=True)  # noqa: E731
p = sam.load(); side = Camera(SIDE, paper_target=SIDE_PAPER); A = hk.ArmKinematics()

def side_end():
    f = side.frame(); cv2.imwrite(str(S / 'vside.png'), f)
    found = sam.detect(p, str(S / 'vside.png'), 'thin metal needle', 0.15) or sam.detect(p, str(S / 'vside.png'), 'black stick', 0.3)
    if not found: return None, f
    vv, uu = np.nonzero(max(found, key=lambda d: d['score'])['mask']); k = int(np.argmax(vv))
    return np.array([uu[k], vv[k]], float), f

q0 = read()
for i in range(1, 5):
    send({k: int(round(q0[k] + (v - q0[k]) * i / 4)) for k, v in START.items()}, ms=400); time.sleep(0.6)
cmd = read(); end, f = side_end(); start_end = end; say('ready, side end', end)
stills, why, rates = 0, 'step limit', []
track = []  # (cumulative joint pulses, side end px) of the real moves: progress along the in-air direction, slope fitted
pulses_total = 0.0
try:
    for i in range(45):
        j = hk.pulses_to_joints(cmd); x, y, z, pitch = A.fk(j)
        s_ = A.ik(x, y, z - STEP_MM / 1000, pitch, j.roll)
        if s_ is None: why = 'out of reach'; break
        tgt = {k: v for k, v in hk.joints_to_pulses(s_).items() if k in (2, 3, 4)}
        if abs(tgt[3] - cmd[3]) < 2:  # at least 2 elbow pulses: 1-pulse steps sat in the deadband (21:52, 21:53)
            tgt = {**tgt, 3: cmd[3] - 2}
        before_rb = read(); send(tgt, ms=300); time.sleep(0.6); cmd = {**cmd, **tgt}; after_rb = read()
        moved_joints = max(abs(after_rb[k] - before_rb[k]) for k in (2, 3, 4))
        new, f = side_end()
        if new is None or end is None: say(f'step {i}: needle not found'); end = new; continue
        down = new[1] - end[1]; jump = np.linalg.norm(new - end)
        if jump > 60:  # the settle drop or a misdetection: the new place becomes the reference (it stayed stale, 21:48)
            say(f'step {i}: end jumped {jump:.0f} px: new reference'); end = new; stills = 0; continue
        # contact = the joints moved but the tip did not: 1-pulse steps sat in the servo deadband and faked it (21:52)
        if moved_joints >= 2:  # a 1-pulse readback change is jitter, not motion (21:53)
            pulses_total += moved_joints
            track.append((pulses_total, new.copy()))
            if len(track) >= 10:
                # SAM's end flips between the ink bulb and the metal (+-6 px, mostly sideways): progress is measured along the
                # in-air direction and its slope fitted over the last 4 real moves, against the slope of the first 4
                d0 = track[3][1] - track[0][1]; d0 = d0 / max(np.linalg.norm(d0), 1e-6)
                pr = np.array([t[1] @ d0 for t in track]); pu = np.array([t[0] for t in track])
                theil = lambda x, y: float(np.median([(y[b] - y[a]) / (x[b] - x[a]) for a in range(len(x)) for b in range(a + 1, len(x)) if x[b] != x[a]]))  # noqa: E731
                # median of pairwise slopes over 6 moves: on try 6's flips a 4-move fit fell to 0.96 (false contact), this to 2.0
                air_slope = theil(pu[:4], pr[:4]); now_slope = theil(pu[-6:], pr[-6:])
                say(f'    slope now {now_slope:.2f} px/pulse, in the air {air_slope:.2f}')
                if now_slope < 0.4 * air_slope:
                    why = f'CONTACT: progress slowed to {now_slope:.2f} px/pulse (in the air {air_slope:.2f})'; end = new; break
        say(f'step {i}: cmd {tgt}, joints moved {moved_joints}, side end {new.round().tolist()}, moved {down:+.1f} px down ({stills} still)')
        end = new
        if stills >= 2: why = 'CONTACT: the tip stopped going down in the side view'; break
finally:
    cv2.imwrite(str(S / 'vside_contact.jpg'), f)
    send({3: read()[3] + 12}, ms=400); time.sleep(0.6)
    send(START, ms=900); time.sleep(1.2); side.close(); say('ended:', why, '- arm back at start')
