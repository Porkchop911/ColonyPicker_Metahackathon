"""Needle pick-up from the tripod tube: look, remember, move blind, check (plan 2026-10-30 B).

The arm camera sees the needle from a standoff but not the fingers' closing point. So: SAM finds the needle
in arm-camera frames from a few poses near the taught "above" pose, each frame's needle line is a plane
through the camera centre, the planes meet in the needle's 3D axis (base frame, FK camera pose). That axis
is remembered; the last 30 mm to the grasp are planned on it and moved blind; a lift then checks the grasp
in the arm camera (a held needle moves with the camera, so it stays put in the image).

    <venv python> tools/needle_grasp.py look OUT_DIR          # views + 3D axis -> OUT_DIR/needle.json
    <venv python> tools/needle_grasp.py grasp OUT_DIR         # look, plan, approach, close, lift, check
    <venv python> tools/needle_grasp.py calibrate OUT_DIR     # look + grasp offset from the hand-taught grasp (no motion after the look)
    <venv python> tools/needle_grasp.py offline FRAMES.json   # axis from saved frames: [{"image", "pulses"}]
"""
from __future__ import annotations

import json, math, sys, time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hwr_kinematics as hk  # noqa: E402
import tip_triangulate as tt  # noqa: E402

A = hk.ArmKinematics()
PATH = Path('evidence/teach_20261030/path.json')  # the hand-taught rough path
# look from park turned right and tilted down: the whole needle against the boxes, nothing in front (2026-10-30);
# above the grasp it sat at the frame's edge and SAM lost it. Wrist stalls at 173 here.
LOOK = {1: 262, 2: 871, 3: 0, 4: 178, 5: 509}  # re-aimed after the tripod was set up again (2026-10-30 09:30)
# folded up, the camera sits near the pan axis: pan-only views had no baseline (axis 151 mm off, 2026-10-30).
# Views along the taught path down from the look pose instead: ~150 mm of camera travel
VIEW_STEPS = [0.0, 0.33, 0.66, 1.0]  # fractions of the way from LOOK to the taught "swing" pose
APPROACH_MM = 30.0
GRASP_OFFSET = Path('evidence/teach_20261030/grasp_offset.json')  # closing point minus FK tool point (calibration)


def needle_line(predictor, image_path: str):
    """(mask, unit plane normal in the camera frame) of the needle, or (None, None)."""
    import sam_detect as sam
    found = sam.detect(predictor, image_path, 'thin metal needle', 0.15)
    if not found:
        return None, None
    # the needle is long and thin: the most elongated confident mask, not the tube
    def elong(d):
        vs, us = np.nonzero(d['mask'])
        if us.size < 30:
            return 0.0
        ev = np.linalg.eigvalsh(np.cov(np.vstack([us, vs])))
        return d['score'] * math.sqrt(ev[1] / max(ev[0], 1e-6))
    best = max(found, key=elong)
    vs, us = np.nonzero(best['mask'])
    n = cv2.undistortPoints(np.column_stack([us, vs]).astype(np.float64).reshape(-1, 1, 2), tt.ARM_K, tt.ARM_D).reshape(-1, 2)
    vx, vy, x0, y0 = cv2.fitLine(n.astype(np.float32), cv2.DIST_HUBER, 0, 0.001, 0.001).ravel()
    p1 = np.array([x0 - vx, y0 - vy, 1.0]); p2 = np.array([x0 + vx, y0 + vy, 1.0])
    normal = np.cross(p1, p2)
    return best['mask'], normal / np.linalg.norm(normal)


def axis_from_planes(planes):
    """3D line (point, unit direction) closest to every plane (unit normal n, point o): n.x = n.o."""
    N = np.array([n for n, _ in planes]); d = np.array([n @ o for n, o in planes])
    direction = np.linalg.svd(N)[2][-1]  # the direction lying in every plane
    # least-squares point, held to the plane through the origin perpendicular to the direction
    M = np.vstack([N, direction]); point = np.linalg.lstsq(M, np.append(d, 0.0), rcond=None)[0]
    spread = max(math.degrees(math.acos(min(1.0, abs(float(a @ b))))) for a in N for b in N)  # widest pair of planes (conditioning)
    resid_mm = 1000 * float(np.abs(N @ point - d).max())
    return point, direction, spread, resid_mm


def plane_in_base(normal_cam, pulses):
    rot, origin = A.camera_pose(hk.pulses_to_joints(pulses))
    return rot @ normal_cam, np.asarray(origin, float)


def solve(predictor, views):
    """views: [(image path, readback pulses)] -> axis dict, per-view masks saved next to the images."""
    planes, used = [], []
    for image, pulses in views:
        mask, normal = needle_line(predictor, image)
        if mask is None:
            print('  no needle in', image); continue
        cv2.imwrite(str(Path(image).with_suffix('.mask.png')), mask.astype(np.uint8) * 255)
        planes.append(plane_in_base(normal, pulses)); used.append(image)
    if len(planes) < 2:
        raise SystemExit(f'needle seen in {len(planes)} views: need 2')
    point, direction, spread, resid = axis_from_planes(planes)
    print(f'  axis through {np.round(point * 1000, 1)} mm, direction {np.round(direction, 3)}, '
          f'planes {spread:.1f} deg apart, worst plane miss {resid:.2f} mm ({len(planes)} views)')
    return {'point': point.tolist(), 'direction': direction.tolist(), 'spread_deg': spread, 'resid_mm': resid, 'views': used}


def grasp_target(axis, taught):
    """Point on the axis nearest the taught grasp's FK tool point (the rough path says where along the needle)."""
    j = hk.pulses_to_joints(taught); x, y, z, pitch = A.fk(j)
    p, d = np.array(axis['point']), np.array(axis['direction'])
    tool = np.array([x, y, z]); target = p + d * ((tool - p) @ d)
    if GRASP_OFFSET.exists():  # IK goal for the FK tool point, so that the closing point lands on the needle
        target = target - np.array(json.load(open(GRASP_OFFSET))['offset_m'])
    return target, pitch, j.roll, tool


def main() -> None:
    mode, arg = sys.argv[1], sys.argv[2]
    import sam_detect as sam
    predictor = sam.load()
    if mode == 'offline':
        views = [(v['image'], {int(k): x for k, x in v['pulses'].items()}) for v in json.load(open(arg))]
        axis = solve(predictor, views)
        taught = {int(k): v for k, v in json.load(open(PATH))['grasp_open']['pulses'].items()}
        target, _, _, tool = grasp_target(axis, taught)
        print(f'  grasp target {np.round(target * 1000, 1)} mm; taught FK tool point {np.round(tool * 1000, 1)} mm; '
              f'distance {1000 * np.linalg.norm(target - tool):.1f} mm')
        return
    from tip_descend import read, send
    out = Path(arg); out.mkdir(exist_ok=True)
    P = json.load(open(PATH)); pose = lambda k: {int(j): v for j, v in P[k]['pulses'].items() if j != '10'}  # noqa: E731
    above, taught = pose('above_grasp') if 'above_grasp' in P else None, pose('grasp_open')
    if above is None:  # 30 mm over the taught grasp, 10 deg flatter (at the taught pitch it is out of the model's reach)
        j = hk.pulses_to_joints(taught); x, y, z, pitch = A.fk(j)
        above = {**taught, **{k: v for k, v in hk.joints_to_pulses(A.ik(x, y, z + 0.03, pitch + math.radians(10), j.roll)).items() if k in (2, 3, 4)}}

    def go(goal, n=6, ms=500, hold=False):
        q0 = {k: min(max(v, 0), 1000) for k, v in read().items()}  # the bridge takes 0-1000; the elbow read -66 folded up
        for i in range(1, n + 1):
            send({k: int(round(q0[k] + (goal[k] - q0[k]) * i / n)) for k in goal}, ms=ms); time.sleep(ms / 1000 + 0.3)
        time.sleep(0.5)
        cmd = dict(goal)
        for _ in range(3 if hold else 0):
            # the elbow held 11 pulses under its command at the grasp (sag): push the command by the readback error.
            # Not readback re-sent as a command (that sank servo 2): goal + error, bounded
            q = read(); err = {k: goal[k] - q[k] for k in goal}
            if max(abs(e) for e in err.values()) <= 2:
                break
            cmd = {k: cmd[k] + max(-15, min(15, err[k])) for k in goal}; send(cmd, ms=300); time.sleep(0.6)
        return read()

    send({10: 500}, ms=500); time.sleep(0.8)
    views = []
    swing = pose('swing')
    for i, f in enumerate(VIEW_STEPS):
        q = go({k: int(round(LOOK[k] + (swing[k] - LOOK[k]) * f)) for k in LOOK}, n=3)
        path = str(out / f'view{i}.png'); cv2.imwrite(path, tt.snap_arm(3)); views.append((path, q))
    axis = solve(predictor, views)
    json.dump(axis, open(out / 'needle.json', 'w'), indent=1)
    if mode == 'look':
        return
    if axis['spread_deg'] < 3 or axis['resid_mm'] > 3 or len(axis['views']) < 3:
        raise SystemExit('axis not trusted (too few views, planes too close or not meeting): no motion')
    if mode == 'calibrate':
        # the hand-taught grasp (torque off, so its readback is the true pose) is the correct grasp: the closing
        # point's offset from the FK tool point, against the needle axis seen now (the holder unmoved since teaching)
        q = {int(k): v for k, v in P['grasp_closed']['pulses'].items() if k != '10'}
        j = hk.pulses_to_joints(q); x, y, z, _ = A.fk(j); tool = np.array([x, y, z])
        p, d = np.array(axis['point']), np.array(axis['direction']); on_axis = p + d * ((tool - p) @ d)
        json.dump({'offset_m': (on_axis - tool).tolist(), 'pose': q}, open(GRASP_OFFSET, 'w'), indent=1)
        print(f'closing point - FK tool point = {np.round((on_axis - tool) * 1000, 1)} mm -> {GRASP_OFFSET}')
        return
    go(pose('swing'), n=8); go(above, n=4)  # down from the look pose along the taught path, not straight through the tube

    target, pitch, roll, tool = grasp_target(axis, taught)
    print(f'grasp target {np.round(target * 1000, 1)} mm, {1000 * np.linalg.norm(target - tool):.1f} mm from the taught grasp')
    if np.linalg.norm(target - tool) > 0.03:
        raise SystemExit('target over 30 mm from the taught grasp: wrong needle line?')
    # approach along the tool axis (pitch, toward the pan direction): the fingers come straight in from 30 mm off
    pan = math.atan2(target[1], target[0] - A.pan_x)
    axis_dir = np.array([math.cos(pitch) * math.cos(pan), math.cos(pitch) * math.sin(pan), math.sin(pitch)])
    poses = []
    for s in np.linspace(APPROACH_MM, 0, 7):
        p = target - axis_dir * s / 1000
        sol = A.ik(*p, pitch, roll)
        if sol is None:
            raise SystemExit(f'approach point {np.round(p * 1000)} mm out of reach at pitch {math.degrees(pitch):.0f}')
        poses.append({k: v for k, v in hk.joints_to_pulses(sol).items() if k in (1, 2, 3, 4, 5)})
    go(poses[0], n=4, hold=True)
    for q in poses[1:]:
        go(q, n=1, ms=400, hold=True)
    before = tt.snap_arm(3); cv2.imwrite(str(out / 'at_grasp.png'), before)
    send({10: 662}, ms=600); time.sleep(1.0)
    closed = tt.snap_arm(3); cv2.imwrite(str(out / 'closed.png'), closed)
    lift = A.ik(*(target + np.array([0, 0, 0.02])), pitch, roll)
    go({k: v for k, v in hk.joints_to_pulses(lift).items() if k in (1, 2, 3, 4, 5)}, n=3)
    lifted = tt.snap_arm(3); cv2.imwrite(str(out / 'lifted.png'), lifted)
    # held: the needle moved with the camera, so its pixels match before and after the lift
    _, n_closed = needle_line(predictor, str(out / 'closed.png'))
    _, n_lifted = needle_line(predictor, str(out / 'lifted.png'))
    if n_closed is None or n_lifted is None:
        print('CHECK: needle not found in a check frame'); return
    turn = math.degrees(math.acos(min(1.0, abs(float(n_closed @ n_lifted)))))
    print(f'CHECK: needle plane turned {turn:.1f} deg between closed and lifted ->', 'HELD' if turn < 1.5 else 'MISSED')


if __name__ == '__main__':
    main()
