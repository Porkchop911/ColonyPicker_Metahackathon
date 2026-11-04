#!/usr/bin/env python3
"""Find colonies on petri dishes with SAM 3.1: dish first, then zoomed tiles of each dish.

    .venv/Scripts/python tools/colony_detect.py IMAGE [--out overlay.jpg]
    .venv/Scripts/python tools/colony_detect.py --camera 1 --exposure -5 --lamp12 --min-mm 1.5 [--live]

Imaging (sweep 2026-10-30, camera 1, two plates): a black matte sheet under the plates is the big
lever (127 vs 18 colonies on the white notebook). On black, lamp 12 alone (side light, lamp 13 off)
beat room light for the pickable sizes (>= 1.5 mm: 32 vs 26) at exposure -5 (mean grey ~140).
DSHOW ignores a switch back to auto exposure on this camera, so give --exposure.
Sizes are equivalent diameters, scaled by assuming 90 mm dishes. Below ~1 mm (4 px) is the camera
limit; counts vary ~20 % between runs on one frame, the >= 1.5 mm set is the stable one.

At camera resolution a colony is a few pixels and SAM finds none ("colony" scores 0 everywhere).
So: "petri dish" on the frame, then each dish crop is cut into overlapping 2x2 tiles at 6x zoom and
prompted "white speck" (small colonies), and the whole crop at 3x is prompted "bubble" (the large
ringed ones; under side light it also takes small ones). The agar top is SAM's "liquid surface"
(the dish mask also covers the side wall). Prompts picked by a sweep on two sourdough plates on 2026-10-30 ("colony", "bacterial
colony", "yeast colony", "white dot", "mold" all found nothing; "agar" and "gel" found no surface).
Hits off the slightly eroded agar top (wall, rim, lid glare) are dropped; hits split across tile
seams are merged by centroid distance.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import sam_detect  # noqa: E402

SMALL, LARGE = "white speck", "bubble"
CROP_ZOOM, TILE_ZOOM = 3, 2   # tiles are 2x on top of the 3x crop
TILES, TILE_DIV = 3, 2         # grid per side, tile size as a fraction of the dish; denser plates need more, smaller tiles (sam_photo.py)
OVERLAP = 0.15                # tile overlap, fraction of the tile
RIM = 0.02                    # erode the agar top by this fraction of the dish width (meniscus glare)
MERGE_PX = 6                  # same colony if centroids are this close in frame pixels
WALL = 0.10                   # the same when only the whole dish is known: its wall and meniscus glints
SAM_ORIGIN, SAM_W, SAM_H = (1200, 100), 1920, 1080  # the part of the 4K overhead SAM sees: dish and plate, at the old scale
DISH_MM = 55.0                # dish diameter (user 11-04: all ours are 55 mm), sets the mm scale


def _pil(bgr: np.ndarray) -> list:
    return [Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))]


def _hits(predictor, img: np.ndarray, prompt: str, threshold: float, zoom: float, ox: float, oy: float,
          kind: str) -> list[dict]:
    """Detect on `img` (already zoomed by `zoom`, placed at frame offset ox, oy), in frame pixels."""
    out = []
    for f in sam_detect.detect(predictor, _pil(img), prompt, threshold):
        if f["centroid"] is None:
            continue
        x0, y0, x1, y1 = f["box"]
        out.append({"kind": kind, "score": f["score"],
                    "centroid": (ox + f["centroid"][0] / zoom, oy + f["centroid"][1] / zoom),
                    "box": (ox + x0 / zoom, oy + y0 / zoom, ox + x1 / zoom, oy + y1 / zoom),
                    "area": f["area"] / zoom ** 2})
    return out


def detect_colonies(predictor, frame: np.ndarray, threshold: float = 0.3) -> tuple[list[dict], list[dict]]:
    """Return (dishes, colonies) in frame pixels."""
    if frame.shape[1] > SAM_W:  # the whole 4K frame shrinks the dish past what SAM reads: 0 colonies (2026-11-03)
        x, y = SAM_ORIGIN
        dishes, colonies = detect_colonies(predictor, frame[y:y + SAM_H, x:x + SAM_W], threshold)
        for f in dishes + colonies:
            f["box"] = tuple(np.add(f["box"], (x, y, x, y)).tolist())
            f["centroid"] = (f["centroid"][0] + x, f["centroid"][1] + y)
        return dishes, colonies
    dishes = sam_detect.detect(predictor, _pil(frame), "petri dish", 0.3)  # 0.5 lost it at full lamp (2026-11-03)
    surface = np.zeros(frame.shape[:2], bool)
    for s in sam_detect.detect(predictor, _pil(frame), "liquid surface", 0.5):
        surface |= s["mask"]
    colonies = []
    for d in dishes:
        x0, y0, x1, y1 = (int(round(v)) for v in d["box"])
        # the dish less its wall: "liquid surface" found nothing under the overhead lamps (11-03), and on plate 4 only the
        # top half, which dropped every colony below it at all settings (11-04)
        k = max(3, int(WALL * (x1 - x0)) | 1)
        agar = cv2.erode(d["mask"].astype(np.uint8), np.ones((k, k), np.uint8)).astype(bool)
        crop = cv2.resize(frame[y0:y1, x0:x1], None, fx=CROP_ZOOM, fy=CROP_ZOOM, interpolation=cv2.INTER_CUBIC)
        found = _hits(predictor, crop, LARGE, threshold, CROP_ZOOM, x0, y0, "large")
        h, w = crop.shape[:2]
        th, tw = int(h / TILE_DIV * (1 + OVERLAP)), int(w / TILE_DIV * (1 + OVERLAP))
        for ty in np.linspace(0, h - th, TILES).astype(int):  # 3x3: with 2x2 a whole half of a dish gave no hits (plate 4, 11-04)
            for tx in np.linspace(0, w - tw, TILES).astype(int):
                tile = cv2.resize(crop[ty:ty + th, tx:tx + tw], None, fx=TILE_ZOOM, fy=TILE_ZOOM,
                                  interpolation=cv2.INTER_CUBIC)
                found += _hits(predictor, tile, SMALL, threshold, CROP_ZOOM * TILE_ZOOM,
                               x0 + tx / CROP_ZOOM, y0 + ty / CROP_ZOOM, "small")
        kept: list[dict] = []
        for f in sorted(found, key=lambda f: (f["kind"] != "large", -f["score"])):
            u, v = f["centroid"]
            if not agar[min(int(v), agar.shape[0] - 1), min(int(u), agar.shape[1] - 1)]:
                continue
            if f["kind"] == "small" and any(  # a speck inside a large colony is that colony's texture
                    g["kind"] == "large" and g["box"][0] <= u <= g["box"][2] and g["box"][1] <= v <= g["box"][3]
                    for g in kept):
                continue
            if any(np.hypot(u - g["centroid"][0], v - g["centroid"][1]) < MERGE_PX for g in kept):
                continue
            kept.append(f)
        px_mm = (x1 - x0) / DISH_MM
        for f in kept:
            f["d_mm"] = 2 * np.sqrt(f["area"] / np.pi) / px_mm
        colonies += kept
    return dishes, colonies


def draw(frame: np.ndarray, dishes: list[dict], colonies: list[dict]) -> np.ndarray:
    vis = frame.copy()
    for d in dishes:
        x0, y0, x1, y1 = map(int, d["box"])
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 200, 0), 2)
    for c in colonies:
        u, v = map(int, c["centroid"])
        if c["kind"] == "large":
            x0, y0, x1, y1 = map(int, c["box"])
            cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 0, 255), 2)
        else:
            cv2.circle(vis, (u, v), max(5, int(np.sqrt(c["area"] / np.pi)) + 2), (0, 255, 0), 1)
    n_large = sum(c["kind"] == "large" for c in colonies)
    text = f"{len(dishes)} dishes  {len(colonies) - n_large} small + {n_large} large colonies"
    for colour, width in (((0, 0, 0), 4), ((255, 255, 255), 2)):
        cv2.putText(vis, text, (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 1.1, colour, width)
    return vis


# score, diameter mm, contrast. One SAM run, three settings: the demo page shows all three and the user picks one.
# Loosened 11-03 (user: 9 colonies on the plate): scores fell to 0.57-0.64 on one dish, so the score gate is low;
# size and contrast reject glare (white colony over agar 44-82, bare agar 11, rim glare 24).
VARIANTS = {'strict': (0.6, 2.1, 45), 'normal': (0.45, 1.7, 30), 'loose': (0.05, 0.5, 5)}  # loose: small-colony plates (11-03)


def variant_picks(predictor, bg_top: np.ndarray) -> dict:
    """{setting: [{'px', 'd_mm', 'score'}]} for the three VARIANTS, from one SAM run on the overhead background."""
    _, cols = detect_colonies(predictor, bg_top, threshold=0.05)
    out = [(np.array(c['centroid'], float), float(c['d_mm']), float(c['score'])) for c in cols]
    g = cv2.cvtColor(bg_top, cv2.COLOR_BGR2GRAY).astype(float)
    yy, xx = np.mgrid[-16:17, -16:17]
    r = np.hypot(xx, yy)

    def contrast(p):
        x, y = int(p[0]), int(p[1])
        w = g[y - 16:y + 17, x - 16:x + 17]
        return w[r <= 4].mean() - w[(r >= 10) & (r <= 14)].mean() if w.shape == r.shape else 0.0
    return {name: [{'px': [float(v) for v in c[0]], 'd_mm': round(c[1], 1), 'score': round(c[2], 2)}
                   for c in out if c[1] >= d_min and c[2] >= sc_min and contrast(c[0]) > ct_min]
            for name, (sc_min, d_min, ct_min) in VARIANTS.items()}


def prompted(predictor, frame: np.ndarray, prompt: str) -> list[dict]:
    """SAM again with one text prompt and a low threshold: Muse Spark's re-run. No gates: Spark judges these marks itself."""
    global SMALL, LARGE
    keep = SMALL, LARGE
    SMALL, LARGE = prompt, prompt
    try:
        _, cols = detect_colonies(predictor, frame, threshold=0.05)
    finally:
        SMALL, LARGE = keep
    return [{'px': [float(v) for v in c['centroid']], 'd_mm': round(float(c['d_mm']), 1), 'score': round(float(c['score']), 2)} for c in cols]


def answer_rerun(predictor, s: str) -> None:
    """On call: a prompt in <s>/sam_rerun_request is answered in <s>/sam_rerun.json, on the frame in st2_top.jpg."""
    path = f'{s}/sam_rerun_request'
    if Path(path).exists():
        prompt = Path(path).read_text().strip()
        Path(path).unlink()
        marks = prompted(predictor, cv2.imread(f'{s}/st2_top.jpg'), prompt)
        Path(f'{s}/sam_rerun.tmp').write_text(json.dumps({'prompt': prompt, 'marks': marks}))
        Path(f'{s}/sam_rerun.tmp').replace(f'{s}/sam_rerun.json')


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", nargs="?")
    ap.add_argument("--camera", type=int, help="grab from this camera index instead of an image")
    ap.add_argument("--live", action="store_true", help="with --camera: keep detecting, show a window (q quits)")
    ap.add_argument("--exposure", type=float, help="with --camera: fixed exposure step (e.g. -5)")
    ap.add_argument("--lamp12", action="store_true", help="lamp 12 on at 254/ct 250, lamp 13 off, first")
    ap.add_argument("--min-mm", type=float, default=0.0, help="keep colonies at least this wide")
    ap.add_argument("--threshold", type=float, default=0.3)
    ap.add_argument("--out", default=None, help="overlay path (default: <image>_colonies.jpg)")
    args = ap.parse_args()
    if (args.image is None) == (args.camera is None):
        ap.error("give an IMAGE or --camera N")

    if args.lamp12:
        import hue
        hue.set_light("12", True, 254, 250)
        hue.set_light("13", False)
    predictor = sam_detect.load()
    camera = None
    if args.camera is not None:
        from rig_io import Camera
        camera = Camera(args.camera)
        if args.exposure is not None:
            camera.cap.set(cv2.CAP_PROP_EXPOSURE, args.exposure)
    while True:
        frame = camera.frame(average=1, settle=0.5) if camera else cv2.imread(args.image)
        t0 = time.monotonic()
        dishes, colonies = detect_colonies(predictor, frame, args.threshold)
        colonies = [c for c in colonies if c["d_mm"] >= args.min_mm]
        print(f"{time.monotonic() - t0:.1f} s: {len(dishes)} dishes, {len(colonies)} colonies", flush=True)
        vis = draw(frame, dishes, colonies)
        if not args.live:
            break
        cv2.imshow("colonies (SAM 3.1)", cv2.resize(vis, (1280, 720)))
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return
    for c in sorted(colonies, key=lambda c: c["centroid"]):
        print(f"  {c['kind']:5s} score {c['score']:.2f} at ({c['centroid'][0]:.1f}, {c['centroid'][1]:.1f}) "
              f"{c['d_mm']:.1f} mm")
    out = args.out or (args.image.rsplit(".", 1)[0] if args.image else f"camera{args.camera}") + "_colonies.jpg"
    cv2.imwrite(out, vis)
    print(f"overlay {out}")


if __name__ == "__main__":
    main()
