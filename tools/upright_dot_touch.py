"""Touch a dot with the needle's underside (level gripper, needle hanging down, its tip hidden from the arm camera).

Overhead UGREEN: aim the needle tip (SAM 3.1) at the dot (image Jacobian learned from two probe moves, re-aimed every step).
Side UGREEN: contact = the tip stops going down while the arm keeps stepping down. Model IK moves in base mm,
pitch and roll held. Plan 2026-10-30 A.

    <venv python> tools/upright_dot_touch.py OUT_DIR DOT_U DOT_V [FLOOR_Z_MM]  # dot in overhead pixels (1920x1080);
        # floor: FK tool height never commanded below it (hand-guided touch 2026-10-30: 125 mm, floor 110)
"""
import json, sys, time
from pathlib import Path
sys.path.insert(0, 'tools')
import cv2, numpy as np, hwr_kinematics as hk, sam_detect as sam
from rig_io import Camera, OVERHEAD, SIDE
from tip_descend import read, send

out = Path(sys.argv[1]); out.mkdir(exist_ok=True); dot = np.array([float(sys.argv[2]), float(sys.argv[3])])
floor_z = float(sys.argv[4]) / 1000 if len(sys.argv) > 4 else 0.0
A = hk.ArmKinematics()
STEP_DOWN_MM = 2.0
MAX_DOWN_MM = 150.0
AIM_PX = 6.0          # ~1 mm at the dots (grid: ~6 px/mm)
log = open(out / 'log.jsonl', 'a')
say = lambda *a: print(time.strftime('%H:%M:%S'), *a, flush=True)  # noqa: E731


def needle_tip(img, prev, name):
    """Lowest pixel of the SAM needle mask nearest `prev` (teal thresholds caught the keyboard box's line art)."""
    path = str(out / f'_sam_{name}.png'); cv2.imwrite(path, img)
    found = []
    for prompt, th in (('thin metal needle', 0.15), ('pen', 0.3), ('needle', 0.1), ('metal rod', 0.1)):  # the teal-inked needle reads as a pen (0.79)
        found = sam.detect(predictor, path, prompt, th)  # 0.15 alone missed a plainly visible needle (step 6, 09:50)
        if found:
            break
    tips = []
    for d in found:
        vs, us = np.nonzero(d['mask'])
        if us.size < 20:
            continue
        low = vs >= vs.max() - 3
        tips.append(np.array([float(us[low].mean()), float(vs.max())]))
    if not tips:
        return None
    return min(tips, key=lambda t: np.linalg.norm(t - prev)) if prev is not None else max(tips, key=lambda t: t[1])


predictor = sam.load()
ov, sd = Camera(OVERHEAD), Camera(SIDE)
q0 = read(); cmd = {k: q0[k] for k in (1, 2, 3, 4, 5)}
j0 = hk.pulses_to_joints(cmd); x, y, z, pitch = A.fk(j0); roll = j0.roll
pos = np.array([x, y, z])
n = [0]


def look(tag):
    n[0] += 1
    o, s = ov.frame(), sd.frame()
    cv2.imwrite(str(out / f'{n[0]:03d}_{tag}_o.jpg'), o); cv2.imwrite(str(out / f'{n[0]:03d}_{tag}_s.jpg'), s)
    to, ts = needle_tip(o, look.o, 'o'), needle_tip(s, look.s, 's')
    look.o = to if to is not None else look.o  # a miss keeps the last tip for tracking
    look.s = ts if ts is not None else look.s
    return to, ts


look.o = look.s = None


def move(new):
    global pos, cmd
    sol = A.ik(*new, pitch, roll)
    if sol is None:
        return None
    tgt = {k: v for k, v in hk.joints_to_pulses(sol).items() if k in (1, 2, 3, 4, 5)}
    send(tgt, ms=400); time.sleep(0.9); cmd, pos = tgt, new
    return tgt


try:
    o, s = look('start')
    if o is None or s is None:
        raise SystemExit(f'teal tip not found (overhead {o}, side {s})')
    say('tip overhead', o.round(), 'side', s.round(), 'dot', dot)
    # image Jacobian (overhead px per base mm) from a forward and a sideways probe
    J = np.zeros((2, 2))
    for col, d in enumerate((np.array([0.006, 0, 0]), np.array([0, 0.006, 0]))):
        if move(pos + d) is None:
            raise SystemExit('probe move out of reach')
        o2, _ = look('probe')
        if o2 is None:
            raise SystemExit('tip lost on a probe move')
        J[:, col] = (o2 - o) / 6.0; o = o2
    say('J px/mm', J.round(2).tolist())
    if abs(np.linalg.det(J)) < 1.0:
        raise SystemExit('probe moves did not move the tip in the overhead view')
    s_prev, rate, stalls, down, went_down = s, [], 0, 0.0, False
    for i in range(80):
        o, s = look('step')
        for _ in range(2):  # a miss is usually SAM, not the tip: look again before giving up
            if o is not None and s is not None:
                break
            o, s = look('retry')
        if o is None or s is None:
            say('tip lost: hold'); break
        err = dot - o
        log.write(json.dumps({'i': i, 'cmd': cmd, 'pos_mm': (pos * 1000).round(1).tolist(), 'over': o.tolist(),
                              'side': s.tolist(), 'err_px': err.tolist()}) + '\n'); log.flush()
        dv = s[1] - s_prev[1]  # side view: down is +v
        say(f'{i}: err {err.round(1)} px, side tip {s.round()} ({dv:+.1f} px), down {down:.0f} mm')
        if went_down:  # only a step that went down can stall
            rate.append(dv)
            air = np.median(rate[:5]) if len(rate) >= 3 else None
            if air and air > 1 and dv < 0.3 * air:
                stalls += 1
                if stalls >= 2:
                    say('CONTACT: the tip stopped going down in the side view'); break
            else:
                stalls = 0
        s_prev = s
        dxy = np.linalg.solve(J, err) / 1000 * 0.6  # mm -> m, damped
        dxy = np.clip(dxy, -0.006, 0.006)
        went_down = False
        if np.linalg.norm(err) > 4 * AIM_PX:  # far off: aim only, no descent (the hover parallax shrinks as it drops)
            if move(pos + np.array([dxy[0], dxy[1], 0.0])) is not None:
                continue
            say('aim move out of reach at this height: aim while stepping down')  # lower reaches further forward
        if down >= MAX_DOWN_MM:
            say('descent limit'); break
        if pos[2] - STEP_DOWN_MM / 1000 < floor_z:
            say(f'floor {floor_z * 1000:.0f} mm reached without contact'); break
        if move(pos + np.array([dxy[0], dxy[1], -STEP_DOWN_MM / 1000])) is None:
            say('out of reach even lower'); break
        down += STEP_DOWN_MM; went_down = True
finally:
    look('contact')
    send({**cmd, 2: cmd[2] + 30}, ms=600); time.sleep(1.0)  # lift: shoulder up
    look('lifted')
    ov.close(); sd.close(); say('lifted; frames in', out)
