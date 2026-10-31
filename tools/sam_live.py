#!/usr/bin/env python3
"""Live SAM 3.1 on the UGREEN stream: type a text prompt, watch the masks.

    .venv/Scripts/python tools/sam_live.py ["black dot"]

Console: type a new prompt + Enter to switch; "t 0.3" sets the score threshold.
Window: s = save frame + overlay to evidence/sam_live/, q = quit.
Every frame is a fresh detection (no tracking between frames), ~1-3 fps on the 3090.
"""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sam_detect  # noqa: E402
from rig_io import Camera  # noqa: E402

OUT = Path("evidence/sam_live")
COLOURS = [(255, 64, 64), (64, 255, 64), (64, 64, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]


def main() -> None:
    state = {"prompt": sys.argv[1] if len(sys.argv) > 1 else "black dot", "threshold": 0.5}

    def console() -> None:
        while True:
            line = input().strip()
            if line.startswith("t ") and line[2:].replace(".", "", 1).isdigit():
                state["threshold"] = float(line[2:])
            elif line:
                state["prompt"] = line
            print(f"prompt '{state['prompt']}'  threshold {state['threshold']}", flush=True)

    threading.Thread(target=console, daemon=True).start()
    print("loading SAM 3.1 ...", flush=True)
    predictor = sam_detect.load()
    camera = Camera()
    print(f"ready. prompt '{state['prompt']}' - type a new one + Enter, 't 0.3' for threshold", flush=True)
    while True:
        frame = camera.frame(average=1, settle=0.0)
        prompt, threshold = state["prompt"], state["threshold"]
        t0 = time.monotonic()
        found = sam_detect.detect(predictor, [Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))],
                                  prompt, threshold)
        dt = time.monotonic() - t0
        overlay = frame.copy()
        for i, f in enumerate(sorted(found, key=lambda f: -f["score"])):
            colour = COLOURS[i % len(COLOURS)]
            overlay[f["mask"]] = (0.45 * overlay[f["mask"]] + 0.55 * np.array(colour)).astype(np.uint8)
            x0, y0, x1, y1 = map(int, f["box"])
            cv2.rectangle(overlay, (x0, y0), (x1, y1), colour, 2)
            cv2.putText(overlay, f"{f['score']:.2f}", (x0, max(y0 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
        cv2.putText(overlay, f"'{prompt}'  {len(found)} found  thr {threshold}  {dt:.2f} s", (12, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 4)
        cv2.putText(overlay, f"'{prompt}'  {len(found)} found  thr {threshold}  {dt:.2f} s", (12, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
        cv2.imshow("SAM 3.1 live (UGREEN)", cv2.resize(overlay, (1280, 720)))
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            OUT.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%dT%H%M%S")
            cv2.imwrite(str(OUT / f"{stamp}_frame.jpg"), frame)
            cv2.imwrite(str(OUT / f"{stamp}_{prompt.replace(' ', '_')}.jpg"), overlay)
            print(f"saved {OUT / stamp}*", flush=True)


if __name__ == "__main__":
    main()
