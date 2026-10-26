#!/usr/bin/env python3
"""Text-prompted instance segmentation with SAM 3.1 (Meta `sam3` package, facebook/sam3.1) on one image.

    .venv/Scripts/python tools/sam_detect.py IMAGE "black dot" [--threshold 0.5] [--out overlay.jpg]

Prints one line per instance (score, box, centroid, area) and writes an overlay. Runs in the
worktree venv (Python 3.12, CUDA torch, sam3 from GitHub, triton-windows); the first call downloads
the checkpoint (HF login + gated access). SAM 3.1 ships only as a video predictor, so an image is
run as a one-frame video.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np
from PIL import Image
from sam3.model_builder import build_sam3_predictor


def load():
    predictor = build_sam3_predictor(version="sam3.1", use_fa3=False, async_loading_frames=False)
    init = predictor.model.init_state
    # upstream start_session passes offload_state_to_cpu, which the 3.1 model's init_state lacks (sam3 f1647e8)
    predictor.model.init_state = lambda offload_state_to_cpu=False, **kw: init(**kw)
    return predictor


def detect(predictor, image: str, prompt: str, threshold: float = 0.5) -> list[dict]:
    session = predictor.handle_request({"type": "start_session", "resource_path": image})["session_id"]
    try:
        out = predictor.handle_request({"type": "add_prompt", "session_id": session, "frame_index": 0,
                                        "text": prompt, "output_prob_thresh": threshold})["outputs"]
    finally:
        predictor.handle_request({"type": "close_session", "session_id": session})
    found = []
    for mask, box, score in zip(out["out_binary_masks"], out["out_boxes_xywh"], out["out_probs"]):
        mask = np.asarray(mask, dtype=bool)
        h, w = mask.shape
        x, y, bw, bh = np.asarray(box) * [w, h, w, h]  # boxes are relative xywh
        vs, us = np.nonzero(mask)
        found.append({"score": float(score), "box": [round(float(b), 1) for b in (x, y, x + bw, y + bh)],
                      "centroid": (round(float(us.mean()), 1), round(float(vs.mean()), 1)) if us.size else None,
                      "area": int(mask.sum()), "mask": mask})
    return found


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image")
    ap.add_argument("prompt")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--out", default=None, help="overlay path (default: <image>_sam.jpg)")
    args = ap.parse_args()

    t0 = time.monotonic()
    predictor = load()
    t1 = time.monotonic()
    image = Image.open(args.image).convert("RGB")
    found = detect(predictor, args.image, args.prompt, args.threshold)
    t2 = time.monotonic()
    print(f"load {t1 - t0:.1f} s, infer {t2 - t1:.2f} s, {len(found)} x '{args.prompt}'")

    overlay = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    for i, f in enumerate(sorted(found, key=lambda f: -f["score"])):
        print(f"  {i:2d} score {f['score']:.2f} centroid {f['centroid']} area {f['area']} box {f['box']}")
        overlay[f["mask"]] = (0.5 * overlay[f["mask"]] + (0, 127, 0)).astype(np.uint8)
        x0, y0, x1, y1 = map(int, f["box"])
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (0, 0, 255), 2)
        cv2.putText(overlay, f"{i} {f['score']:.2f}", (x0, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    out = args.out or args.image.rsplit(".", 1)[0] + "_sam.jpg"
    cv2.imwrite(out, overlay)
    print(f"overlay {out}")


if __name__ == "__main__":
    main()
