#!/usr/bin/env python3
"""A1 well calibration: reliably place the fixed-geometry needle into well A1.

The plate moves between sessions, so NOTHING about A1 is cached. Every session:
  1. locate  - fresh stereo solve + live A1 measurement from the overhead camera
  2. measure - tip-in-camera for the fixed needle (once per needle seating, reused after)
  3. place   - closed-loop descend into A1 with well-centre correction

    <hackathon venv python> tools/a1_calibrate.py locate [--images TOP SIDE] [--click]
    <hackathon venv python> tools/a1_calibrate.py measure --roll 1000
    <hackathon venv python> tools/a1_calibrate.py place --roll 1000 [--well-depth 11.0]

Why this exists instead of reusing arm_calibrate.hover/dot directly: hover() aims at a
*printed grid crossing* (local_crossing homography, DOT_OFFSETS skid bias, memory per goal).
Over the plate there is no ruling (the plate covers it) and no printed crossing, so the goal
must be a *well centre on the plate-top plane*, refined from the well blob itself. The arm
model, approach-side terms, anchor-at-START, and measure-correct loop are reused unchanged.

Fixed-needle assumption: the needle geometry does not change between sessions (no re-grasp).
The tip in the camera frame (tip_by_roll[roll] in arm_model.json) is measured once with
`measure` and reused; only the base (anchor at START) is refit per session.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plate_rig as prig  # noqa: E402

A1_FILE = Path("evidence/tip/a1_location.json")
ARM_MODEL = Path("evidence/tip/arm_model.json")
WELL_PITCH_MM = 9.0
WELL_OPENING_RADIUS_MM = 3.0  # 9 mm pitch, ~6-7 mm opening: |xy - centre| must be within this at the bottom
DEFAULT_WELL_DEPTH_MM = 11.0  # plate top -> well bottom; refined by contact every place


def fresh_solve(top_img: np.ndarray, side_img: np.ndarray | None = None) -> tuple[dict, dict, np.ndarray]:
    """Overhead-only fresh solve: cached camera poses + live plate pose, no side re-solve.

    Only the plate moved between sessions; the cameras and the ruling did not, so the rig
    poses stay valid (checked: overhead followed for creep without changing the frame).
    The live plate pose comes from the overhead frame, which is also where the moved A1
    is measured. A full plate_rig.solve from scratch is deliberately NOT attempted: it
    re-ties the side through the plate and the standing graph-paper board behind the rig
    gets solved as the desk sheet (live 2026-11-01: side camera 78 mm up instead of ~140,
    mat overlap 0.41). Returns (top, side, sheet_plate); side is the cache for reference.
    """
    cached_side = json.loads(prig.SIDE_CACHE.read_text())
    lens = json.loads(prig.TOP_LENS.read_text())
    kt, dt = np.array(lens["k"]), np.array(lens["d"])
    top_plate = prig.plate_poses(top_img, kt, dt, "top")
    try:
        plate = next(p for p in top_plate
                     if (-cv2.Rodrigues(p[0])[0].T @ p[1].ravel())[0] > 0)
    except StopIteration:
        raise RuntimeError("locate: no overhead plate pose on the plate's +x side")
    rt, tt_, n_top, rms_top = prig.grid_pose(top_img, kt, dt, plate)
    print(f"overhead: {n_top} crossings on the lattice, {rms_top:.2f} px", flush=True)
    sheet_plate = (np.linalg.inv(prig.homog(rt, tt_)) @ prig.homog(*plate[:2])
                   @ prig.homog(*prig.TOP_PLATE_FIX))
    tilt = float(np.degrees(np.arccos(abs(sheet_plate[2, 2]))))
    print(f"plate: {tilt:.2f} deg off the sheet, top {-sheet_plate[2, 3]:.1f} mm above it", flush=True)
    if n_top < prig.CHECK_MIN or tilt > 2.0 or not 5.0 < -sheet_plate[2, 3] < 25.0:
        raise RuntimeError(f"overhead: {n_top} crossings, plate {tilt:.1f} deg off the sheet: no consistent pose")
    top = {"k": kt.tolist(), "d": dt.ravel().tolist(), "r": rt.ravel().tolist(),
           "t": tt_.ravel().tolist(), "crossings": n_top, "rms": rms_top,
           "source": "a1_calibrate: cached rig, live plate pose"}
    return top, cached_side, sheet_plate


def plate_top_plane(sheet_plate: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Point + unit normal of the well-top plane in the sheet frame (sheet z down into paper)."""
    point = sheet_plate @ np.array([0.0, 0.0, 0.0, 1.0])
    normal = sheet_plate[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return point[:3], normal / np.linalg.norm(normal)


def ray_to_plane(centre: np.ndarray, direction: np.ndarray, point: np.ndarray,
                 normal: np.ndarray) -> np.ndarray:
    denom = direction @ normal
    if abs(denom) < 1e-6:
        raise RuntimeError("locate: overhead ray parallel to the plate-top plane")
    return centre + direction * ((point - centre) @ normal / denom)


def plate_cell_to_sheet(top_img: np.ndarray, col: int, row: int) -> tuple[dict, np.ndarray, np.ndarray]:
    """Sheet (x, y) of a plate cell from the live overhead frame, no click needed.

    The wells are found and solved in the plate frame (x along the 12, y along the 8,
    9 mm pitch); the cell is chained onto the sheet exactly the way fresh_solve builds
    sheet_from_plate. This is how A1 is located: plate cell (col 0, row 0) after the
    labelling is verified against the printed letters (row letter along one edge,
    column numbers along the other). Returns (top, sheet_plate, sheet_xy).
    """
    cached_side = json.loads(prig.SIDE_CACHE.read_text())
    lens = json.loads(prig.TOP_LENS.read_text())
    kt, dt = np.array(lens["k"]), np.array(lens["d"])
    plate = [p for p in prig.plate_poses(top_img, kt, dt, "top")
             if (-cv2.Rodrigues(p[0])[0].T @ p[1].ravel())[0] > 0]
    if not plate:
        raise RuntimeError("locate: no overhead plate pose on the plate's +x side")
    r, t, _ = plate[0]
    rt, tt_, n_top, rms_top = prig.grid_pose(top_img, kt, dt, (r, t))
    sheet_plate = (np.linalg.inv(prig.homog(rt, tt_)) @ prig.homog(r, t)
                   @ prig.homog(*prig.TOP_PLATE_FIX))
    cell = sheet_plate @ np.array([col * WELL_PITCH_MM, row * WELL_PITCH_MM, 0.0, 1.0])
    top = {"k": kt.tolist(), "d": dt.ravel().tolist(), "r": rt.ravel().tolist(),
           "t": tt_.ravel().tolist(), "crossings": n_top, "rms": rms_top,
           "source": "a1_calibrate: cached rig, live plate pose, cell ray"}
    return top, sheet_plate, cell[:2]


def locate_a1(top_img: np.ndarray, top: dict, sheet_plate: np.ndarray,
              click_px: tuple | None = None) -> dict:
    """A1 sheet position from the live overhead frame.

    With --click: the operator clicks the A1 well centre (nearest well blob wins, so a sloppy
    click still lands on the opening). Without: the corner well nearest the camera-left is taken
    provisionally and MUST be confirmed on the annotated preview (--click is the reliable path;
    the auto guess exists so a headless run still records what it aimed at).
    The pixel ray meets the plate-TOP plane (not the paper): that is the A1 sheet (x, y); z of
    the plane gives the plate-top height above the paper.
    """
    import tip_triangulate as tt
    k, d, r, t = (np.array(top[q], float) for q in ("k", "d", "r", "t"))
    blobs = prig.wells(top_img, "top")
    if not len(blobs):
        raise RuntimeError("locate: no wells found in the overhead frame (plate in view?)")
    if click_px is None:
        guess = blobs[np.argmax(blobs[:, 1])]  # lowest in the image = nearest the camera
        print(f"locate: no --click, provisional A1 = well blob nearest the camera at {guess.round(1).tolist()} px "
              f"(confirm on the preview, re-run with --click)", flush=True)
    else:
        guess = blobs[np.argmin(np.linalg.norm(blobs - np.asarray(click_px), axis=1))]
        print(f"locate: click {list(click_px)} -> nearest well blob {guess.round(1).tolist()} px", flush=True)
    centre, direction = tt.ray(guess, k, d, r, t)
    plane_pt, plane_n = plate_top_plane(sheet_plate)
    hit = ray_to_plane(centre, direction, plane_pt, plane_n)
    plate_top_mm = float(-sheet_plate[2, 3])  # sheet z down: top above paper reads negative
    result = {"sheet_xy_mm": [round(float(hit[0]), 2), round(float(hit[1]), 2)],
              "plate_top_mm": round(plate_top_mm, 1),
              "a1_px": [round(float(guess[0]), 1), round(float(guess[1]), 1)],
              "source": "live overhead ray onto the plate-top plane"}
    preview = top_img.copy()
    cv2.drawMarker(preview, (int(guess[0]), int(guess[1])), (0, 0, 255), cv2.MARKER_CROSS, 40, 3)
    for b in blobs:
        cv2.circle(preview, (int(b[0]), int(b[1])), 6, (0, 255, 0), 1)
    out = Path("evidence/tip/a1_preview.jpg")
    cv2.imwrite(str(out), preview)
    print(f"locate: A1 sheet {result['sheet_xy_mm']} mm, plate top {plate_top_mm:.1f} mm above paper; preview {out}",
          flush=True)
    return result


def cmd_locate(args) -> None:
    if args.images:
        top_img, side_img = (cv2.imread(p) for p in args.images)
    else:
        from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
        cams = Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER)
        top_img, side_img = (c.frame() for c in cams)
        for c in cams:
            c.close()
    top, side, sheet_plate = fresh_solve(top_img, side_img)
    if args.click is not None:
        click = tuple(args.click)
        result = locate_a1(top_img, top, sheet_plate, click)
    else:
        # default: plate cell (0, 0) of the solved labelling, i.e. the current A1
        # candidate; the preview + printed letters decide whether it is really A1
        top, sheet_plate, cell_xy = plate_cell_to_sheet(top_img, args.col, args.row)
        plate_top_mm = float(-sheet_plate[2, 3])
        result = {"sheet_xy_mm": [round(float(cell_xy[0]), 2), round(float(cell_xy[1]), 2)],
                  "plate_top_mm": round(plate_top_mm, 1),
                  "cell": [args.col, args.row],
                  "source": "live plate pose chained to the sheet, cell ray"}
        preview = top_img.copy()
        k, d, r, t = (np.array(top[q], float) for q in ("k", "d", "r", "t"))
        for cc in range(12):
            for rr in range(8):
                pt = sheet_plate @ np.array([cc * WELL_PITCH_MM, rr * WELL_PITCH_MM, 0.0, 1.0])
                cam_pt = prig.homog(r, t) @ pt.reshape(4, 1)  # sheet -> overhead camera
                px = cv2.projectPoints(cam_pt[:3].ravel(), np.zeros(3), np.zeros(3), k, d)[0][0][0]
                cv2.circle(preview, (int(px[0]), int(px[1])), 6,
                           (0, 0, 255) if (cc, rr) == (args.col, args.row) else (0, 255, 0), 2)
        out = Path("evidence/tip/a1_preview.jpg")
        cv2.imwrite(str(out), preview)
        print(f"locate: cell {(args.col, args.row)} sheet {result['sheet_xy_mm']} mm, "
              f"plate top {plate_top_mm:.1f} mm above paper; preview {out} "
              f"(red = chosen well, check the printed letters)", flush=True)
    result.update({"plate_top_plane_pt": (sheet_plate @ np.array([0, 0, 0, 1.0]))[:3].round(2).tolist(),
                   "time": time.strftime("%Y%m%dT%H%M%S")})
    A1_FILE.write_text(json.dumps(result, indent=1))
    print(f"wrote {A1_FILE} (re-locate every session: the plate moves)", flush=True)


def cmd_measure(args) -> None:
    """Needle-geometry step (once per needle seating): tip in the camera frame at --roll."""
    import arm_calibrate as ac
    tip = ac.fit_tip(args.roll)
    print(f"measure: roll {args.roll} tip {np.round(tip, 1).tolist()} mm (camera frame); "
          f"reused until the needle is re-seated", flush=True)


def cmd_place(args) -> None:
    """Closed-loop place into the located A1. Requires locate + measure first.

    Differences from arm_calibrate.hover(touch=True): goal z is the WELL BOTTOM (plate top minus
    well depth, refined by contact), xy correction comes from the A1 well blob re-projected each
    try (no printed-crossing homography, no DOT_OFFSETS skid bias: the needle enters the well
    mouth vertically instead of pressing ink onto paper, so there is no skid to subtract), and
    the last 2 mm go blind from a pose measured within 1 mm (teal fails against white plastic).
    Contact = triangulated z at the well bottom + the side-row stall used by hover(); the run
    stops on first contact instead of pressing to -3 mm.
    """
    import arm_calibrate as ac
    import coarse_aim as ca
    from tip_descend import read, send
    if not A1_FILE.exists():
        raise SystemExit("no evidence/tip/a1_location.json: run `a1_calibrate.py locate --click U V` first")
    if not ARM_MODEL.exists():
        raise SystemExit("no evidence/tip/arm_model.json: run arm_calibrate fit-arm first")
    a1 = json.loads(A1_FILE.read_text())
    model = json.loads(ARM_MODEL.read_text())
    if str(args.roll) not in model.get("tip_by_roll", {}) and args.roll != ac.START[5]:
        raise SystemExit(f"no tip for roll {args.roll}: run `a1_calibrate.py measure --roll {args.roll}` first")
    goal_xy = np.asarray(a1["sheet_xy_mm"], float)
    well_bottom_z = -(a1["plate_top_mm"] - args.well_depth)  # sheet z down-negative above paper
    print(f"place: A1 {goal_xy.round(2).tolist()} mm, plate top {a1['plate_top_mm']} mm, "
          f"well bottom z {well_bottom_z:.1f} mm (depth {args.well_depth} mm)", flush=True)
    p = ac.session_with_tip(args.roll)
    top, side, det, urig, srig = ac.cameras()
    grid_frame = ac.auto_lamps(top, side, det, urig, srig)[2]
    _ = grid_frame  # lamps set; the desk grid is only needed for the START anchor, not for A1
    bg = ac.side_background(side)
    ac.measure.top_bg = top.frame()
    up = {k: ca.APPROACH for k in (1, 2, 3, 4)}
    q = dict(ac.START)
    try:
        ac.anchor({"calibrated": p}, top, side, det, urig, srig, bg)
        if args.roll != ac.START[5]:
            p_roll = ac.session_with_tip(args.roll)
            p_roll[9:] = p[9:]
            p = p_roll
            raised = {**ac.START, **{k: ac.START[k] + v for k, v in ac.RAISED.items()}}
            send(raised, ms=1500)
            time.sleep(1.6)
            send({**raised, 5: args.roll}, ms=2000)
            time.sleep(2.5)
            q = {**raised, 5: args.roll}
        corr = np.zeros(3)
        for h_mm in (40.0, 25.0, 15.0, 10.0):  # above the well mouth, same gates as hover()
            goal = np.array([goal_xy[0], goal_xy[1], -(a1["plate_top_mm"] + h_mm)])
            for _ in range(ac.HOVER_TRIES):
                q, resid = ac.aim_solve(p, goal + corr, q)
                if resid > 1.0:
                    raise RuntimeError(f"{h_mm:.0f} mm over A1: no solution ({resid:.1f} mm)")
                send({**q, **{k: q[k] - ca.APPROACH * ac.REPEAT_AWAY for k in (1, 2, 3, 4)}}, ms=1200)
                time.sleep(1.2)
                send(q, ms=800)
                time.sleep(0.8 + ac.REPEAT_DWELL)
                ac.measure.expect, ac.measure.expect_mm = goal, ac.EXPECT_MM
                got = next((g for g in (ac.measure(top, side, det, urig, srig, bg) for _ in range(5))
                            if g is not None), None)
                if got is None or got[1] > ac.MISS_MM:
                    raise RuntimeError(f"{h_mm:.0f} mm over A1: tip not measured")
                err = got[0] - goal
                print(f"{h_mm:.0f} mm over A1: error xy {np.linalg.norm(err[:2]):.2f} mm z {err[2]:+.2f} mm",
                      flush=True)
                if np.linalg.norm(err[:2]) <= ac.HOVER_XY_MM and abs(err[2]) <= ac.HOVER_Z_MM:
                    break
                corr -= err
        for h_above in (6.0, 3.0, 2.0, 1.0, 0.0):  # into the well; last 2 mm blind like hover()
            mouth_z = -(a1["plate_top_mm"] + h_above)
            goal = np.array([goal_xy[0], goal_xy[1], max(mouth_z, well_bottom_z)])
            q, resid = ac.aim_solve(p, goal + corr, q)
            send(q, ms=900)
            time.sleep(0.9 + ac.REPEAT_DWELL)
            ac.measure.expect, ac.measure.expect_mm = goal, ac.EXPECT_MM
            got = next((g for g in (ac.measure(top, side, det, urig, srig, bg) for _ in range(5))
                        if g is not None), None)
            if got is None:
                if h_above <= 2.0:
                    print(f"blind at {h_above:.0f} mm into A1 (tip within 1 mm by the last fix)", flush=True)
                    break
                raise RuntimeError(f"into A1 at {h_above:.0f} mm: tip not measured")
            err = got[0] - goal
            print(f"into A1 {h_above:.0f} mm: error xy {np.linalg.norm(err[:2]):.2f} mm, "
                  f"tip z {got[0][2]:.1f} mm", flush=True)
            if np.linalg.norm(err[:2]) > WELL_OPENING_RADIUS_MM:
                raise RuntimeError(f"{np.linalg.norm(err[:2]):.1f} mm off A1 centre: outside the well mouth, stopping")
            corr[:2] -= err[:2]
            if float(-got[0][2]) <= float(-well_bottom_z) + 0.5:
                print(f"contact: tip z {got[0][2]:.1f} mm at the well bottom (dot)", flush=True)
                break
        print(f"A1 place done: tip inside {a1['sheet_xy_mm']} within {WELL_OPENING_RADIUS_MM} mm, "
              f"verify on both frames before lifting", flush=True)
    finally:
        send({**q, 2: q[2] + 40, 3: q[3] + 40}, ms=1500)
        time.sleep(2.0)
        top.close()
        side.close()
        ac.measure.top_bg = ac.measure.expect = None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("cmd", choices=("locate", "measure", "place"))
    ap.add_argument("--images", nargs=2, metavar=("TOP", "SIDE"))
    ap.add_argument("--click", nargs=2, type=float, help="A1 well-centre pixel in the overhead frame")
    ap.add_argument("--col", type=int, default=0, help="plate column (0 = A1 candidate)")
    ap.add_argument("--row", type=int, default=0, help="plate row (0 = A1 candidate)")
    ap.add_argument("--roll", type=int, default=502)
    ap.add_argument("--well-depth", type=float, default=DEFAULT_WELL_DEPTH_MM)
    args = ap.parse_args()
    {"locate": cmd_locate, "measure": cmd_measure, "place": cmd_place}[args.cmd](args)


if __name__ == "__main__":
    main()
