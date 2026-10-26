#!/usr/bin/env python3
"""Live arm-camera window on the PC, pulled through the servo shell's read-only {"snap"}.

    python tools/arm_view.py            # window; s = save a frame to evidence/view/, q = quit

Read-only: works with the estop latched, beside any other tool (each tool has its own SSH session).
"""
from __future__ import annotations

import base64
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rig_io import servo  # noqa: E402

OUT = Path("evidence/view")


def main() -> None:
    last = time.monotonic()
    while True:
        try:
            reply = servo({"snap": True})[0]
        except RuntimeError as error:  # 'no camera frame yet' right after the shell starts
            reply = {"error": str(error)}
        if "jpeg" not in reply:
            print(f"no frame: {reply}")
            time.sleep(0.5)
            continue
        image = cv2.imdecode(np.frombuffer(base64.b64decode(reply["jpeg"]), np.uint8), cv2.IMREAD_COLOR)
        now = time.monotonic()
        cv2.putText(image, f"{1 / max(now - last, 1e-3):.1f} fps  age {reply.get('age', 0):.2f}s", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        last = now
        big = cv2.resize(image, None, fx=2, fy=2, interpolation=cv2.INTER_LINEAR)
        cv2.imshow("arm camera", big)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            OUT.mkdir(parents=True, exist_ok=True)
            path = OUT / f"{time.strftime('%Y%m%dT%H%M%S')}.jpg"
            cv2.imwrite(str(path), image)
            print(f"saved {path}")


if __name__ == "__main__":
    main()
