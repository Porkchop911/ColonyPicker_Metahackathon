#!/usr/bin/env python3
"""Aim the side UGREEN: a live window with its pose from the notebook's dots (dots agreeing, rms, height, tilt).

    <hackathon venv python> tools/side_aim.py      q quits

The overhead UGREEN places the dots on the sheet once (black from evidence/descend/ugreen_dots.json, red by
SAM 3.1); each side frame is then solved like model_jab does at the tie (tip_triangulate.side_marks / side_rig).
The sheet grid cannot place it: its numbering slips at that slant. Good: >= 6 dots agree; the needle's landing
area mid-frame, the far edge of the notebook near the top.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import model_jab as mj  # noqa: E402
import sam_detect as sam  # noqa: E402
import tip_triangulate as tt  # noqa: E402
from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera  # noqa: E402

SCRATCH = Path("evidence/descend")  # SAM reads files: the crops go here and are overwritten


def main() -> None:
    predictor = sam.load()
    detect = lambda path, prompt, th: sam.detect(predictor, path, prompt, th)  # noqa: E731
    top_cam = Camera(OVERHEAD)
    top = top_cam.frame()
    top_cam.close()
    rig = tt.ugreen_rig(top)
    uk, ud, ur, ut = (np.array(rig[q]) for q in ("k", "d", "r", "t"))
    cv2.imwrite(str(SCRATCH / "side_aim_top.png"), top)
    red = [d["centroid"] for d in detect(str(SCRATCH / "side_aim_top.png"), "small red dot", mj.SAM_THRESHOLD)
           if d["centroid"] is not None and mj.RED_AREA[0] <= d["area"] <= mj.RED_AREA[1]]
    black = json.loads((SCRATCH / "ugreen_dots.json").read_text())
    sheet = np.array([tt.on_sheet(q, uk, ud, ur, ut)[:2] for q in black + red])
    print(f"overhead: {len(black)} black + {len(red)} red dots on the sheet", flush=True)
    cam = Camera(SIDE, paper_target=SIDE_PAPER)
    while True:
        img = cam.frame(average=1, settle=0.0)
        view = img.copy()
        marks = tt.side_marks(img, detect, SCRATCH / "side_aim_crop.png")
        for q in marks:
            cv2.circle(view, (int(q[0]), int(q[1])), 10, (0, 160, 255), 2)
        try:
            s = tt.side_rig(marks, sheet)
            sk, sd, sr, st = (np.array(s[q]) for q in ("k", "d", "r", "t"))
            centre, axis = tt.ray((960, 540), sk, sd, sr, st)
            ok = s["dots"] >= 6
            proj = cv2.projectPoints(np.column_stack([sheet, np.zeros(len(sheet))]), sr, st, sk, sd)[0][:, 0]
            for q in proj:
                cv2.drawMarker(view, (int(q[0]), int(q[1])), (0, 200, 0), cv2.MARKER_CROSS, 22, 2)
            text = (f"{'OK' if ok else 'not yet'}: {s['dots']} of {len(sheet)} dots, {s['rms']:.1f} px, "
                    f"height {abs(centre[2]):.0f} mm, tilt {np.degrees(np.arcsin(abs(axis[2]))):.0f} deg")
        except RuntimeError as error:
            text = f"not yet: {len(marks)} marks, {error}"
        print(text, flush=True)
        for colour, width in (((0, 0, 0), 5), ((255, 255, 255), 2)):
            cv2.putText(view, text, (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.1, colour, width)
        cv2.imshow("side camera aim (q quits)", cv2.resize(view, (1280, 720)))
        if cv2.waitKey(200) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
