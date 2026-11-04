"""Refine the plate pose from the well openings seen in the overhead background frame (needle out of view).

The plate solve put every well centre ~(10, 8) px off the visible opening (~1.5 mm, toward the near rim): the column-12
tours hit rims (2026-11-02). This finds each opening's ellipse centre in evidence/tip/st2_top.jpg, casts it onto the
rim plane, fits a 2-D rigid transform (rotation + shift in the sheet plane) cache -> openings, and applies it to the plate
cache. --poses also shifts the learnt tour poses (evidence/tip/well_poses.json) by each well's correction.
usage: python tools/plate_refine.py [--write] [--poses]"""
import json
import sys

import cv2
import numpy as np

sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
import plate_rig as pr  # noqa: E402
import tip_triangulate as tt  # noqa: E402


def opening(gray, guess, r=34, th=10):
    """Centre (px) and axes of the well opening nearest `guess`: the gray blob under the white top."""
    x0, y0 = int(guess[0]) - r, int(guess[1]) - r
    c = cv2.GaussianBlur(gray[y0:y0 + 2 * r, x0:x0 + 2 * r], (3, 3), 0)
    m = cv2.morphologyEx((c < np.percentile(c, 95) - th).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    k, lab, st, cen = cv2.connectedComponentsWithStats(m)
    cand = [j for j in range(1, k) if st[j, 4] > 80]
    if not cand:
        return None
    i = min(cand, key=lambda j: np.hypot(*(cen[j] - r)))
    cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0][0]
    if len(cnt) < 5:
        return None
    (cx, cy), axes, _ = cv2.fitEllipse(cnt)
    return np.array([cx + x0, cy + y0]), sorted(axes)


def on_plane(px, z):
    c = a.CAMS['top']
    o, v = tt.ray(px, np.array(c['k']), np.array(c['d']), c['r'], c['t'])
    return o + v * ((z - o[2]) / v[2])


gray = cv2.cvtColor(cv2.imread('evidence/tip/st2_top.jpg'), cv2.COLOR_BGR2GRAY)
names = [r + str(n) for r in 'ABCDEFGH' for n in range(1, 13)]
src, dst = [], []
for w in names:
    rim = a.pr.well(w)
    got = opening(gray, a.project('top', rim))
    if got is None or not (15 < got[1][0] < 30 and 30 < got[1][1] < 50):  # merged with the tape / the plate edge
        continue
    src.append(rim[:2]); dst.append(on_plane(got[0], rim[2])[:2])
src, dst = np.array(src), np.array(dst)
m, inl = cv2.estimateAffinePartial2D(src, dst, method=cv2.RANSAC, ransacReprojThreshold=0.8)  # inliers only
keep = inl.ravel() == 1
ps, pd = src[keep], dst[keep]
cs, cd = ps.mean(0), pd.mean(0)
u, _, vt = np.linalg.svd((ps - cs).T @ (pd - cd))  # Kabsch: the rigid part (the free scale came out 1.011, 2026-11-02)
R = (u @ vt).T
if np.linalg.det(R) < 0:
    vt[-1] *= -1; R = (u @ vt).T
t = cd - R @ cs
res = np.linalg.norm(ps @ R.T + t - pd, axis=1)
ang = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
print(f'{len(src)} openings, {int(keep.sum())} inliers; rigid: rotation {ang:+.2f} deg, shift {t.round(2)} mm; '
      f'mean move {np.linalg.norm(pd - ps, axis=1).mean():.2f} mm; residual median {np.median(res):.2f} max {res.max():.2f} mm')
assert abs(ang) < 2 and np.median(res) < 1.0, 'implausible fit: not written'
T = np.eye(4); T[:2, :2] = R; T[:2, 3] = t
if '--write' in sys.argv:
    old = {w: a.pr.well(w) for w in names}
    c = json.loads(pr.PLATE_CACHE.read_text())
    c['sheet_plate'] = (T @ np.array(c['sheet_plate'])).tolist()
    c['source'] += ' + refined on the overhead well openings (plate_refine.py)'
    pr.PLATE_CACHE.write_text(json.dumps(c))
    print('plate cache refined')
    if '--poses' in sys.argv:
        pf = 'evidence/tip/well_poses.json'
        poses = json.load(open(pf))
        for w, lst in poses.items():
            d = np.array(a.model_of(a.pr.well(w))) - np.array(a.model_of(old[w]))
            for p in lst:
                p['hover'] = list(np.array(p['hover']) + d); p['entry'] = list(np.array(p['entry']) + d)
            print(f'  {w}: pose shifted by x {d[0]:+.2f} z {d[1]:+.2f} pan {d[2]:+.2f}')
        json.dump(poses, open(pf, 'w'), indent=1)
