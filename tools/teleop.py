#!/usr/bin/env python3
"""Pilot the arm by keyboard over the live arm-camera view, recording every step.

Overlay: green ticks around the tip = where the tip lands (its pixel ray meets the paper there whatever its depth),
red rings = ink dots, yellow ring = the target (left-click a dot to choose it, re-click anytime),
top line = tip height (grid pose, tip depth --lam) and the target's (else the nearest dot's) offset
from the landing point in mm. Each move logs commanded and read-back pulses, the frame, and that
measurement, so a demonstrated approach gives real joint-to-motion data and touch poses to replay.

    panel on the right: per servo -/+ by the chosen step, step size, MARK, LIFT; or keys:
    a/d pan   t/g shoulder   w/s elbow   r/f wrist   q/e roll      (servos 1, 2, 3, 4, 5)
    1 2 5 0   step 1 / 2 / 5 / 10 pulses
    space     mark this frame (e.g. "touching now")      l  lift (elbow +40)      x  quit

    python tools/teleop.py --out DIR [--lam 202.5]
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_touch as at  # noqa: E402
import tip_motion as tm  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from tip_descend import POSE_HELP, commanded, go_start, read, send  # noqa: E402

KEYS = {"a": (1, -1), "d": (1, 1), "t": (2, 1), "g": (2, -1), "w": (3, 1), "s": (3, -1),
        "r": (4, 1), "f": (4, -1), "q": (5, -1), "e": (5, 1)}
STEPS = {"1": 1, "2": 2, "5": 5, "0": 10}
SERVOS = [(1, "pan"), (2, "shoulder"), (3, "elbow"), (4, "wrist"), (5, "roll"), (10, "gripper")]  # gripper for the needle pick-up (plan 2026-10-30 B1)
PANEL_STEPS = (1, 2, 5, 10, 20)
PATCH = 10       # template half-size, px, for following the clicked mark
SEARCH_PX = 60   # how far the mark may move in one step, px


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--lam", type=float, default=202.5, help="tip depth along its pixel ray, mm")
    ap.add_argument("--pose", type=int, nargs=6, help=POSE_HELP)
    args = ap.parse_args()
    out = args.out / time.strftime("%Y%m%dT%H%M%S")
    out.mkdir(parents=True)
    log, n = [], [0]

    def note(**row):
        log.append({"t": round(time.time(), 2), **row})
        (out / "log.json").write_text(json.dumps(log, indent=1, default=lambda o: np.asarray(o).tolist()))
        print({k: v for k, v in row.items() if k != "dots"}, flush=True)

    note(phase="go-start", pulses=go_start(args.pose))
    note(phase="light", lamps=at.auto_light())
    first = tt.snap_arm(3)
    free, axis = tt.arm_tip(first, with_axis=True)
    ray = at.unit_ray(free)

    m: dict = {}
    target = [None]  # chosen mark, px; followed by its image patch after each move
    patch = [None]   # gray patch around the target, for template matching
    clicks: list = []  # mouse clicks, handled in the main loop
    win = "teleop: click a dot = target, panel = servos"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, lambda ev, x, y, f, p: clicks.append((x, y)) if ev == cv2.EVENT_LBUTTONDOWN else None)

    def measure(img: np.ndarray) -> dict:
        """Height, landing point and dots (sheet mm, this frame's numbering); {} if the grid fails."""
        try:
            rot, t, _ = tm.arm_view(img)
        except RuntimeError:
            return {}
        foot = at.sheet_xy(free, rot, t)
        found = at.dots_near(img, free)
        pts = np.array([at.sheet_xy(f, rot, t) for f in found]).reshape(-1, 2)
        near = pts[np.argmin(np.linalg.norm(pts - foot, axis=1))] - foot if len(pts) else None
        aim = None
        if target[0] is not None:
            # follow the clicked mark by its own image patch: the dot detector misses some marks, and
            # "nearest detection within 40 px" jumped to a neighbouring dot (live 2026-10-27)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            x, y = (int(round(v)) for v in target[0])
            x1, y1 = max(0, x - SEARCH_PX), max(0, y - SEARCH_PX)
            area = gray[y1:y + SEARCH_PX + 1, x1:x + SEARCH_PX + 1]
            if area.shape[0] > patch[0].shape[0] and area.shape[1] > patch[0].shape[1]:
                score = cv2.matchTemplate(area, patch[0], cv2.TM_CCOEFF_NORMED)
                _, best, _, (bx, by) = cv2.minMaxLoc(score)
                if best >= 0.5:
                    half = patch[0].shape[0] // 2
                    target[0] = np.array([x1 + bx + half, y1 + by + half], float)
                    patch[0] = gray[int(target[0][1]) - half:int(target[0][1]) + half + 1,
                                    int(target[0][0]) - half:int(target[0][0]) + half + 1].copy()
            aim = at.sheet_xy(target[0], rot, t) - foot
        return {"height_mm": float(-(rot.T @ (args.lam * ray - t))[2]), "nearest_dot_mm": near,
                "target_mm": aim, "target_px": target[0], "dots_px": found}

    # right-hand panel (pixels in the 2x window): one row per servo with -/+ buttons, step sizes, actions
    x0, row_h = 1280, 64
    buttons = []  # (x1, y1, x2, y2, action)
    for i, (j, name) in enumerate(SERVOS):
        y = 30 + i * row_h
        buttons += [(x0 + 120, y, x0 + 210, y + 44, ("move", j, -1)), (x0 + 220, y, x0 + 310, y + 44, ("move", j, 1))]
    y = 30 + len(SERVOS) * row_h + 20
    for i, size in enumerate(PANEL_STEPS):
        buttons.append((x0 + 10 + i * 60, y, x0 + 62 + i * 60, y + 40, ("step", size)))
    y += 70
    buttons += [(x0 + 10, y, x0 + 150, y + 50, ("mark",)), (x0 + 170, y, x0 + 310, y + 50, ("lift",))]

    def panel(step: int, cmd: dict, now: dict) -> np.ndarray:
        img = np.full((800, 320, 3), 40, np.uint8)
        for i, (j, name) in enumerate(SERVOS):
            y = 30 + i * row_h
            cv2.putText(img, f"{j} {name}", (8, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img, f"{cmd[j]} / {now.get(j, '?')}", (8, y + 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        for x1, y1, x2, y2, action in buttons:
            on = action[0] == "step" and action[1] == step
            cv2.rectangle(img, (x1 - x0, y1), (x2 - x0, y2), (0, 160, 255) if on else (90, 90, 90), -1)
            if action[0] == "move":
                label = "-" if action[2] < 0 else "+"
            elif action[0] == "step":
                label = str(action[1])
            else:
                label = action[0].upper()
            cv2.putText(img, label, (x1 - x0 + 10, y2 - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img, "cmd / read", (8, 790), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
        return img

    step, cmd, last_img = 5, commanded(args.pose), first  # 1-2 pulse steps sit inside the servo deadband
    now = dict(cmd)
    m.update(measure(first))
    note(phase="start", tip_px=free, pulses=cmd)

    def do_move(j: int, delta: int, label: str) -> None:
        nonlocal m, now
        before = read()
        # change only this joint of the commanded pose: re-sending read-back sank servo 2 by ~5 pulses
        # per command (its read-back sits ~5 below where it holds under load; live 2026-10-27)
        cmd[j] += delta
        send(cmd, ms=250)
        time.sleep(0.35)
        now = read()
        n[0] += 1
        frame = tt.snap_arm(1)
        cv2.imwrite(str(out / f"{n[0]:03d}_arm.png"), frame)
        m = measure(frame)
        note(phase="move", key=label, servo=j, asked=delta, moved=now[j] - before[j],
             pulses=now, **{k: v for k, v in m.items() if k != "dots_px"})

    def do_mark() -> None:
        nonlocal m
        n[0] += 1
        cv2.imwrite(str(out / f"{n[0]:03d}_mark.png"), last_img)
        m = measure(last_img)
        note(phase="mark", pulses=read(), **{k: v for k, v in m.items() if k != "dots_px"})

    jobs: list = []  # moves and marks run here, off the UI thread: a blocking move froze the view ~2-3 s

    def worker() -> None:
        while True:
            if jobs:
                job = jobs.pop(0)
                try:
                    job()
                except Exception as error:  # keep piloting; the log shows what failed
                    note(phase="job-error", error=repr(error))
            else:
                time.sleep(0.02)

    threading.Thread(target=worker, daemon=True).start()

    while True:
        img = tt.snap_arm(1)
        last_img = img
        view = img.copy()
        cx, cy = (int(round(v)) for v in free)  # ticks around the tip, leaving the tip itself visible
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cv2.line(view, (cx + 5 * dx, cy + 5 * dy), (cx + 11 * dx, cy + 11 * dy), (0, 200, 0), 1)
        for d in m.get("dots_px", []):
            cv2.circle(view, tuple(int(v) for v in d), 8, (0, 0, 255), 1)
        if target[0] is not None:
            cv2.circle(view, tuple(int(v) for v in target[0]), 12, (0, 255, 255), 2)
        text = f"step {step}" + (f"  [{len(jobs)} queued]" if jobs else "")
        if m:
            text += f"  h {m['height_mm']:.1f} mm"
            if m.get("target_mm") is not None:
                text += f"  target {np.round(m['target_mm'], 1).tolist()} mm"
            elif m["nearest_dot_mm"] is not None:
                text += f"  dot {np.round(m['nearest_dot_mm'], 1).tolist()} mm"
        cv2.putText(view, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow(win, np.hstack([cv2.resize(view, None, fx=2, fy=2), panel(step, cmd, now)]))
        key = chr(cv2.waitKey(30) & 0xFF)
        while clicks:
            x, y = clicks.pop(0)
            if x < x0:  # in the camera view: choose the target dot
                p = np.array([x / 2.0, y / 2.0])
                dots = np.array(m.get("dots_px", [])).reshape(-1, 2)
                gaps = np.linalg.norm(dots - p, axis=1) if len(dots) else np.array([np.inf])
                target[0] = dots[np.argmin(gaps)] if gaps.min() <= 8 else p  # the click itself unless a dot is right there
                gray = cv2.cvtColor(last_img, cv2.COLOR_BGR2GRAY)
                tx, ty = (int(round(v)) for v in target[0])
                patch[0] = gray[ty - PATCH:ty + PATCH + 1, tx - PATCH:tx + PATCH + 1].copy()
                m = measure(last_img)
                note(phase="target", target_px=target[0], **{k: v for k, v in m.items() if k not in ("dots_px", "target_px")})
                continue
            for x1, y1, x2, y2, action in buttons:
                if x1 <= x <= x2 and y1 <= y <= y2:
                    if action[0] == "move":
                        j, d = action[1], action[2] * step
                        jobs.append(lambda j=j, d=d: do_move(j, d, f"panel {j}{'+' if d > 0 else '-'}"))
                    elif action[0] == "step":
                        step = action[1]
                    elif action[0] == "mark":
                        jobs.append(do_mark)
                    elif action[0] == "lift":
                        jobs.append(lambda: do_move(3, 40, "lift"))
        if key == "x":
            break
        if key in STEPS:
            step = STEPS[key]
        elif key in KEYS:
            j, sign = KEYS[key]
            jobs.append(lambda j=j, d=sign * step, k=key: do_move(j, d, k))
        elif key == "l":
            jobs.append(lambda: do_move(3, 40, "lift"))
        elif key == " ":
            jobs.append(do_mark)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
