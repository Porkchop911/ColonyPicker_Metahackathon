#!/usr/bin/env python3
"""Preflight before touches: fixes what it can (watchdog, stalled arm camera), reports the rest, one line per check.

    <hackathon venv python> tools/preflight.py [--sam] [--start]

--sam: also the SAM 3.1 checks (overhead dots, side-camera pose; ~15 s more). --start: move the arm to the start
pose at the end (tools/arm_to_start.py). Exit code 1 if a check failed. The dashboard holds both UGREENs: stop it first.
Checks: robot reachable; sensor_supervisor_node stopped (it restarts the arm camera every 16 s and comes back with every
stack restart); arm camera delivers (one driver restart if stalled); both UGREENs open at their exposure; overhead grid;
markers seen; teal tip seen; with --sam the notebook dots and the side camera's pose.
"""
from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import markers as mk  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import OVERHEAD, ROBOT, SIDE, SIDE_PAPER, Camera, reset_bridge, restart_arm_camera, servo  # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str = "") -> bool:
    RESULTS.append((name, ok, detail))
    print(f"{'OK  ' if ok else 'FAIL'} {name:<22} {detail}", flush=True)
    return ok


def ssh(cmd: str, timeout: float = 20) -> str:
    out = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", ROBOT, cmd],
                         capture_output=True, text=True, timeout=timeout)
    return out.stdout.strip()


def supervisor_pids() -> list[str]:
    # the quoted [e] keeps grep from matching its own command line (pgrep -f matched its shell, 2026-10-29)
    return ssh("ps -eo pid,args | grep 'sensor_supervisor_nod[e]' | awk '{print $1}'").split()


def arm_snaps(n: int = 8) -> int:
    ok = 0
    for _ in range(n):
        try:
            jpeg = base64.b64decode(servo({"snap": True})[0]["jpeg"])
            ok += cv2.imdecode(np.frombuffer(jpeg, np.uint8), 1) is not None
        except RuntimeError:
            pass
        time.sleep(0.3)
    return ok


def arm_frame() -> np.ndarray | None:
    try:
        return cv2.imdecode(np.frombuffer(base64.b64decode(servo({"snap": True})[0]["jpeg"]), np.uint8), 1)
    except RuntimeError:
        return None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sam", action="store_true", help="also the SAM 3.1 checks (dots, side pose)")
    ap.add_argument("--start", action="store_true", help="move the arm to the start pose at the end")
    ap.add_argument("--no-robot", action="store_true", help="only the PC side: cameras, markers, ink, dots")
    args = ap.parse_args()

    if not args.no_robot:
        if not check("robot reachable", ssh("echo up") == "up", ROBOT):
            sys.exit(1)
        pids = supervisor_pids()
        for pid in pids:  # SIGINT first (it ignores SIGTERM), SIGKILL if it stays
            ssh(f"kill -INT {pid}; sleep 2; ps -p {pid} >/dev/null && kill -KILL {pid}")
        check("camera watchdog", not supervisor_pids(), f"stopped pid {', '.join(pids)}" if pids else "not running")
        got = arm_snaps()
        if got < 6:
            restart_arm_camera()
            time.sleep(30)  # the driver respawns after ~8 s and streams some s later
            reset_bridge()
            got = arm_snaps()
            check("arm camera", got >= 6, f"{got}/8 snaps after a driver restart")
        else:
            check("arm camera", True, f"{got}/8 snaps")

    cams = {}
    for name, idx, kw in (("overhead", OVERHEAD, {}), ("side", SIDE, {"paper_target": SIDE_PAPER})):
        try:
            cams[name] = Camera(idx, **kw)
        except RuntimeError as error:
            check(f"{name} UGREEN", False, f"{error} (is the dashboard running?)")
    frames = {name: cam.frame() for name, cam in cams.items()}
    for cam in cams.values():
        cam.close()
    if "overhead" in frames:
        g = cv2.cvtColor(frames["overhead"], cv2.COLOR_BGR2GRAY)
        check("overhead UGREEN", 100 <= np.median(g) <= 180, f"median {np.median(g):.0f} (target 140)")
    if "side" in frames:
        g = cv2.cvtColor(frames["side"], cv2.COLOR_BGR2GRAY)
        check("side UGREEN", 160 <= np.percentile(g, 90) <= 210, f"paper {np.percentile(g, 90):.0f} (target {SIDE_PAPER:.0f})")
    rig = None
    if "overhead" in frames:
        try:
            rig = tt.ugreen_rig(frames["overhead"])
            check("overhead grid", rig["crossings"] >= 200, f"{rig['crossings']} crossings, {rig['rms']:.2f} px")
        except (RuntimeError, ValueError) as error:
            check("overhead grid", False, str(error))

    arm = None if args.no_robot else arm_frame()
    seen = {name: sorted(mk.detect(img)) for name, img in {**frames, "arm": arm}.items() if img is not None}
    want = {"overhead": set(mk.SHEET_IDS[:2]) | {mk.CHASSIS_ID}, "arm": set(mk.SHEET_IDS[:2])}
    for name, ids in seen.items():
        need = want.get(name, set())
        check(f"markers {name}", True,  # optional: without markers the dots and the cube tie the cameras
              f"ids {ids or 'none'}" + (f" (want {sorted(need)}; optional: the dots are the fallback)" if need - set(ids) else ""))

    if "overhead" in frames:
        hsv = cv2.cvtColor(frames["overhead"], cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, (45, 35, 50), (100, 255, 255))
        count, lab, st, _ = cv2.connectedComponentsWithStats(m)
        long_ = [j for j in range(1, count) if 30 <= st[j, 4] < 5000 and max(st[j, 2], st[j, 3]) >= 2 * min(st[j, 2], st[j, 3])]
        check("inked tip", bool(long_), "a long teal blob in the overhead view" if long_ else
              "none: re-ink the tip (dry tips cost SAM calls and leave no mark), or the arm is not over the sheet")

    if args.sam and rig is not None:
        import model_jab as mj
        import sam_detect as sam
        predictor = sam.load()
        tmp = Path("evidence/descend/_preflight.png")
        cv2.imwrite(str(tmp), frames["overhead"])
        img = frames["overhead"]
        gray, hsv = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        paper = float(np.median(gray[gray > np.percentile(gray, 40)]))
        red = [d["centroid"] for d in sam.detect(predictor, str(tmp), "small red dot", mj.SAM_THRESHOLD)
               if d["centroid"] and mj.RED_AREA[0] <= d["area"] <= mj.RED_AREA[1]]
        black = [d["centroid"] for d in sam.detect(predictor, str(tmp), "dot", 0.3)
                 if d["centroid"] and mj.SAM_AREA[0] <= d["area"] <= mj.SAM_AREA[1]
                 and gray[d["mask"]].min() <= mj.SAM_BLACK_FRAC * paper
                 and not (45 <= np.median(hsv[..., 0][d["mask"]]) <= 100 and np.median(hsv[..., 1][d["mask"]]) > 35)
                 and all(np.hypot(d["centroid"][0] - r[0], d["centroid"][1] - r[1]) > 6 for r in red)]
        known = np.array(json.loads(Path("evidence/descend/ugreen_dots.json").read_text()))
        near = [float(np.linalg.norm(known - q, axis=1).min()) for q in black]
        check("overhead dots", len(black) >= 1 and all(n <= 15 for n in near),
              f"{len(black)} black (others under ink are filled from ugreen_dots.json), {len(red)} red")
        if "side" in frames:
            uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
            sheet = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in known.tolist() + red])
            try:
                s = tt.side_rig(tt.side_marks(frames["side"], lambda p, q, th: sam.detect(predictor, p, q, th), tmp), sheet)
                check("side camera pose", 80 <= abs(s["centre_mm"][2]) <= 250,
                      f"{s['dots']} dots, {s['rms']:.1f} px, {abs(s['centre_mm'][2]):.0f} mm up")
            except RuntimeError as error:
                check("side camera pose", False, f"{error}: re-aim with tools/side_aim.py")
        tmp.unlink(missing_ok=True)

    if args.start and not args.no_robot:
        out = subprocess.run([sys.executable, "tools/arm_to_start.py", "evidence/descend"], capture_output=True, text=True)
        check("arm to start", out.returncode == 0, (out.stdout.strip().splitlines() or [""])[-1])
    failed = [n for n, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} OK" + (f"; fix: {', '.join(failed)}" if failed else ""))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
