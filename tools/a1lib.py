"""A1 placing on the 2026-11-01 build: needle clamped across link 4, shoulder = servo 5 (rig_io.PHYSICAL).
Geometry fitted live: pan axis on the sheet from two pan poses; model radius/height offsets from one triangulated tip."""
import sys, json, math, cv2, numpy as np
sys.path.insert(0, 'tools')
import plate_rig as pr, tip_triangulate as tt
from reach import ik

C = np.array([-165.9, 62.1])      # pan axis, sheet mm (six triangulated tips, x and pan moved, 2026-11-04)
ANG600, DEG_PER_PAN = -25.0, -0.24  # tip bearing about C at pan 600; pan +1 pulse turns it clockwise
R_OFF, Z_PAPER = -25.0, 47.8      # measured radius = model radius + R_OFF; paper at model z = Z_PAPER
CAMS = {n: json.loads(p.read_text()) for n, p in (('top', pr.TOP_CACHE), ('side', pr.SIDE_CACHE))}

def sheet_of(x, z, pan):
    """Predicted tip, sheet mm (z down), for model tip (x, z) at pan."""
    r, a = x - 46.6 + R_OFF, math.radians(ANG600 + (pan - 600) * DEG_PER_PAN)
    return np.array([C[0] + r * math.cos(a), C[1] + r * math.sin(a), -(z - Z_PAPER)])

def model_of(p):
    """Inverse of sheet_of: (x, z, pan) for a sheet point."""
    v = p[:2] - C
    return (float(np.linalg.norm(v)) + 46.6 - R_OFF, Z_PAPER - p[2],
            600 + (math.degrees(math.atan2(v[1], v[0])) - ANG600) / DEG_PER_PAN)

def project(n, X):
    c = CAMS[n]
    return cv2.projectPoints(np.array([X], float), np.array(c['r']), np.array(c['t']), np.array(c['k']), np.array(c['d']))[0][0, 0]

def find_tip(img, bg, guess, win=160, th=35, every=False):
    """Lower end of the needle: the largest changed blob in a window around the predicted pixel."""
    x0, y0 = int(max(guess[0] - win, 0)), int(max(guess[1] - 2 * win, 0))
    x1, y1 = int(min(guess[0] + win, img.shape[1])), int(min(guess[1] + win, img.shape[0]))
    if x1 - x0 < 20 or y1 - y0 < 20:
        return None
    f, b = img[y0:y1, x0:x1].astype(float), bg[y0:y1, x0:x1].astype(float)
    if not every:  # overhead only: the side window mixes dark desk and white plate, matching made its heights jump
        f *= np.median(b, axis=(0, 1)) / np.maximum(np.median(f, axis=(0, 1)), 1)  # daylight drifts after the bg:
    d = np.abs(f - b).max(axis=2) > th  # unmatched, the whole window differed and junk blobs won (wells, 2026-11-03)
    d = cv2.morphologyEx(d.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    k, lab, st, _ = cv2.connectedComponentsWithStats(d)
    if k < 2:
        return None
    best, ends = None, []  # the blob whose lower end is nearest the guess (exposure drift makes other, bigger blobs)
    for i in range(1, k):
        if st[i, 4] < 15 or (every and st[i, 3] < 25):  # side: only needle-like (tall) blobs; small ones are glints
            continue
        ys, xs = np.nonzero(lab == i); j = int(np.argmax(ys))
        e = (float(xs[j] + x0), float(ys[j] + y0))
        ends.append(e)
        dist = math.hypot(e[0] - guess[0], e[1] - guess[1])
        if best is None or dist < best[0]:
            best = (dist, e)
    if every:
        return ends
    return None if best is None else best[1]

def triangulate(tips):
    return tt.meet(*(tt.ray(tips[n], np.array(CAMS[n]['k']), np.array(CAMS[n]['d']), CAMS[n]['r'], CAMS[n]['t']) for n in tips))

def measure(imgs, bgs, pred, miss_max=5.0, floor_z=None):
    """Overhead tip by the nearest blob; side tip = the LOWEST side blob end that meets the overhead ray: a second blob
    higher up the same ray was taken every other frame at 20 mm (live 21:38, 11 mm / 7.5 mm jumps)."""
    top = find_tip(imgs['top'], bgs['top'], project('top', pred), th=35)
    if top is None:
        return None, None, {}
    ok = []
    for e in find_tip(imgs['side'], bgs['side'], project('side', pred), th=12, every=True) or []:
        X, miss = triangulate({'top': top, 'side': e})
        if miss < miss_max and (floor_z is None or X[2] < floor_z):  # floor: sheet z down, nothing under the rim while hovering
            ok.append((e[1], X, miss, e))
    if not ok:
        return None, None, {'top': top}
    _, X, miss, e = max(ok, key=lambda c: c[0])
    return overhead_xy(top, e), miss, {'top': top, 'side': e}


def overhead_xy(top_px, side_px):
    """Tip from the overhead ray, at the height the side ray gives where it passes the overhead ray. The plate targets
    come from the overhead (plate_refine.py), and the side camera looks almost level: its height is good, its depth is
    not; the plain midpoint was pulled up to 1.3 mm by the side's depth (H wells hit rims, 2026-11-02)."""
    (c1, d1), (c2, d2) = (tt.ray(p, np.array(CAMS[n]['k']), np.array(CAMS[n]['d']), CAMS[n]['r'], CAMS[n]['t'])
                          for n, p in (('top', top_px), ('side', side_px)))
    w = c1 - c2
    q, r_, s_, u = d1 @ d2, d2 @ d2, d1 @ w, d2 @ w
    m2 = c2 + (d1 @ d1 * u - q * s_) / (d1 @ d1 * r_ - q * q) * d2  # side ray's point nearest the overhead ray
    return c1 + d1 * ((m2[2] - c1[2]) / d1[2])


def opening_px(gray, guess, r=34, th=10):
    """Centre (px) of the well opening nearest `guess` in an overhead frame with the needle out of view."""
    x0, y0 = int(guess[0]) - r, int(guess[1]) - r
    c = cv2.GaussianBlur(gray[y0:y0 + 2 * r, x0:x0 + 2 * r], (3, 3), 0)
    m = cv2.morphologyEx((c < np.percentile(c, 95) - th).astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    k, lab, st, cen = cv2.connectedComponentsWithStats(m)
    cand = [j for j in range(1, k) if st[j, 4] > 80]
    if not cand:
        return None
    i = min(cand, key=lambda j: math.hypot(*(cen[j] - r)))
    cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0][0]
    if len(cnt) < 5:
        return None
    (cx, cy), axes, _ = cv2.fitEllipse(cnt)
    if not (15 < min(axes) < 30 and 30 < max(axes) < 50):  # merged with the tape or the plate edge
        return None
    return np.array([cx + x0, cy + y0])


def image_target(bg_top, well, h):
    """Overhead pixel the tip must sit on, h mm over the rim: the opening's centre plus the model's parallax for h.
    The opening comes from rim_fit (edge ring, works in daylight); the offset used is the median over this well and its
    row/column neighbours, so a stray edge (the printed "12" next to A12) cannot pull it. None if the fit is weak."""
    gray = cv2.cvtColor(bg_top, cv2.COLOR_BGR2GRAY)
    row, col = well[0], int(well[1:])
    near = [well] + [r + str(c) for r, c in ((row, col - 1), (row, col + 1)) if 1 <= c <= 12] +            [chr(ord(row) + d) + str(col) for d in (-1, 1) if 'A' <= chr(ord(row) + d) <= 'H']
    offs = []
    for w in near:
        c, ratio = rim_fit(gray, w)
        if ratio > 1.5:
            offs.append(c - project('top', pr.well(w)))
    if len(offs) < 2:
        return None
    rim = pr.well(well)
    return project('top', rim) + np.median(offs, axis=0) + (project('top', rim + np.array([0, 0, -h])) - project('top', rim))


def image_jacobian(cmd, d=(1.0, 0.0, 1.0)):
    """d(overhead px)/d(model x, pan) at cmd, from the model (only the step direction needs to be right)."""
    base = project('top', sheet_of(*cmd))
    jx = project('top', sheet_of(cmd[0] + d[0], cmd[1], cmd[2])) - base
    jp = project('top', sheet_of(cmd[0], cmd[1], cmd[2] + d[2])) - base
    return np.column_stack([jx / d[0], jp / d[2]])


def image_align(go, grab_top, bg_top, well, cmd, h=3.0, tol_px=4.0, tries=8, say=print, find=None):
    """Final centring in the overhead image alone: tip pixel onto the opening's centre (+ parallax for h), stepping model
    x and pan. The 3-D look left the tip ~3 mm right of the opening on 4 of 5 column-12 wells (photos, 2026-11-02).
    go(cmd) moves; grab_top() returns an overhead frame. Returns the aligned cmd, or None if it could not align."""
    target = image_target(bg_top, well, h)
    if target is None:  # never enter blind: a blurred background once skipped the align on every well (loop 1)
        say(f'  image align {well}: no clean opening (tape? blurred background?), well skipped')
        return None
    cmd = np.array(cmd, float)
    last = None
    for i in range(tries):
        go(tuple(cmd))
        tips = [(find or (lambda f, b, t: find_tip(f, b, t, win=80)))(grab_top(), bg_top, target) for _ in range(3)]
        tips = [t for t in tips if t is not None]
        if not tips:
            say(f'  image align {well} {i}: tip not found'); return None
        e = target - np.median(tips, axis=0)
        say(f'  image align {well} {i}: off {e.round(1)} px')
        if np.linalg.norm(e) < tol_px:
            return tuple(cmd)
        if last is not None and np.linalg.norm(e - last) < 2.0:  # moved, reading did not: tracking something still
            say(f'  image align {well}: reading does not follow the arm, well skipped'); return None  # (F12, 2026-11-03)
        last = e
        dx, dp = 0.5 * np.linalg.lstsq(image_jacobian(tuple(cmd)), e, rcond=None)[0]  # 0.7 swung +-10 px at G12
        if 0.2 < abs(dp) < 1.0:  # under one pan pulse the rounded command never moves: x stuck at 5.3 px (A12, 2026-11-03)
            dp = float(np.sign(dp))
        dx, dp = float(np.clip(dx, -2.0, 2.0)), float(np.clip(dp, -3.0, 3.0))  # small steps (F12 wandered 30 pan pulses)
        cmd = cmd + np.array([dx, 0.0, dp])
    return None


def assert_unfolded(q):
    """Refuse to drive from a folded or unknown pose (rule 1). After a reboot the arm read {631, 935, -66, 579} and the
    tour built its first step from it; only the bridge's range check stopped it (2026-11-02 08:26)."""
    bad = q[2] > 600 or q[4] < 300 or not 0 <= q[3] <= 1000
    # on the needle-down model = the needle hangs near vertical (link 4 ~90 deg from vertical); an IK round trip misjudged
    # poses near full stretch (2026-11-02)
    link4 = (1500 - q[2] - q[3] - q[4]) * 0.24
    on_model = abs(link4 - 90.0) < 12.0
    if bad or not on_model:
        raise SystemExit(f'arm {q} is folded or not on the needle-down model: send nothing, ask the user to unfold it by hand')


def rim_fit(gray, well, r_mm=3.25, search=16):
    """Opening centre (px) by sliding the projected rim circle over the edge image: works in daylight, where the gray
    blob of opening_px vanishes (2026-11-02). Returns (centre px, score ratio best/median) ."""
    rim = pr.well(well)
    m = np.array(json.loads(pr.PLATE_CACHE.read_text())['sheet_plate'])
    a_ = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    ring = np.array([project('top', rim + m[:3, :3] @ np.array([r_mm * math.cos(t), r_mm * math.sin(t), 0.0])) for t in a_])
    c0 = project('top', rim)
    g = cv2.GaussianBlur(gray, (3, 3), 0).astype(np.float32)
    edge = cv2.magnitude(cv2.Sobel(g, cv2.CV_32F, 1, 0), cv2.Sobel(g, cv2.CV_32F, 0, 1))
    scores = {}
    for dy in range(-search, search + 1):
        for dx in range(-search, search + 1):
            pts = np.round(ring + [dx, dy]).astype(int)
            scores[(dx, dy)] = float(edge[pts[:, 1], pts[:, 0]].mean())
    best = max(scores, key=scores.get)
    return c0 + np.array(best, float), scores[best] / float(np.median(list(scores.values())))
