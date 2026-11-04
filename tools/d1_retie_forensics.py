#!/usr/bin/env python3
"""D1 (AGREED plan step 2): why did Touch 1's place `along` flip +-5-13 mm between 1-2 pulse moves?

Re-runs the retie decision offline on run 20261030T103024's saved arm frames: per placing iteration
the arm frame's own grid pose (tip_motion.arm_view), the SAM dots (model_jab's exact prompts), and
every candidate offset match_frames would consider — winner, runner-up, and which pairing each is —
in that frame's sheet mm. Plus the lattice slip (tip_motion.lattice_offset between consecutive views,
the sheet's own independent motion measure) and the target each candidate puts on the sheet.

Frame<->iteration mapping: model_jab.view() numbers every arm frame NNN_arm.png in call order; the
log's retie/place rows give the iteration sequence; this script prints frame index vs iteration so
the mapping is checked, not assumed. The grid pose is per-frame (live numbering), so slips computed
here are in sheet mm as live saw them.

    <hackathon venv python> tools/d1_retie_forensics.py [--round 2] [--frames 12-63]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_touch as at  # noqa: E402
import model_jab as mj  # noqa: E402
import tip_motion as tm  # noqa: E402
from sam_detect import load as sam_load  # noqa: E402

RUN = Path("evidence/descend/20261030T103024")


def candidates(src, dst, cube_src, cube_dst):
    """Every offset match_frames scores (score, off, rot k, lead pairing), best first."""
    out = []
    for k in range(4):
        c, s_ = np.cos(k * np.pi / 2), np.sin(k * np.pi / 2)
        r = np.array([[c, -s_], [s_, c]])
        anchor = cube_dst - r @ cube_src
        for pi, p in enumerate(src):
            for qi, q in enumerate(dst):
                off = q - r @ p
                if np.linalg.norm(off - anchor) > mj.CUBE_TOL_MM:
                    continue
                d = np.linalg.norm((src @ r.T + off)[:, None] - dst[None], axis=2).min(1)
                out.append(((int((d <= mj.MATCH_MM).sum()), -float(d[d <= mj.MATCH_MM].sum())), off, k, pi, qi))
    out.sort(key=lambda e: e[0], reverse=True)
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--round", type=int, default=2)
    ap.add_argument("--frames", default=None, help="frame range like 12-63 (default: all place frames)")
    args = ap.parse_args()
    log = json.loads((RUN / "log.json").read_text())
    tie = next(r for r in log if r["phase"] == "tie-inputs")
    ug = np.array(tie["ug_mm"], float)
    cube_u, cube_a = np.array(tie["cube_u"], float), np.array(tie["cube_a"], float)
    tgt_mm = np.array(next(r for r in log if r["phase"] == "frames")["target_mm"], float)
    places = [r for r in log if r["phase"] == "place"]
    rnd = places[len(places) // 2:] if args.round == 2 else places[: len(places) // 2]
    # UGREEN-side sheet coords of the tie dots: ug_tie = ug_mm ++ red (red px were in ug_tie; the four
    # black dots lead). match_frames(src=ug_tie sheet-mm of UGREEN frame, dst=arm sheet-mm).
    predictor = sam_load()

    def sam_find(img, name, prompt, area, dark, thresh=0.4):
        path = Path("evidence/regress_out") / f"d1_{name}.png"
        cv2.imwrite(str(path), img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        paper = float(np.median(gray[gray > np.percentile(gray, 40)]))
        from sam_detect import detect
        return np.array([d["centroid"] for d in detect(predictor, str(path), prompt, thresh)
                         if d["centroid"] is not None and area[0] <= d["area"] <= area[1]
                         and gray[d["mask"]].min() <= dark * paper], float).reshape(-1, 2)

    def arm_dots(img, name):
        pts = list(sam_find(img, name, "black dot", (20, 300), 0.4, 0.3))
        if not pts:
            pts = list(sam_find(img, name + "_dot", "dot", (10, 300), 0.85, 0.3))
        for q in sam_find(img, name + "_red", "small red dot", (10, 80), 1.0, 0.3):
            if all(np.linalg.norm(q - p_) > 6.0 for p_ in pts):
                pts.append(q)
        return np.array(pts, float).reshape(-1, 2)

    files = sorted(RUN.glob("[0-9][0-9][0-9]_arm.png"))
    lo, hi = (0, len(files)) if not args.frames else (int(args.frames.split("-")[0]), int(args.frames.split("-")[1]) + 1)
    print(f"{len(places)} place rows; round {args.round}: its {len(rnd)}; frames {lo}-{hi - 1} of {len(files)}")
    print("ug_tie sheet mm:", np.round(ug, 1).tolist(), "target_mm:", np.round(tgt_mm, 2).tolist())
    prev_blobs, prev_off = None, None
    for fi, f in enumerate(files[lo:hi], lo):
        img = cv2.imread(str(f))
        try:
            rot, t, blobs = tm.arm_view(img)
        except RuntimeError as e:
            print(f"frame {fi:03d}: grid pose FAILED ({e})")
            continue
        px = arm_dots(img, f"f{fi:03d}")
        dst = np.array([at.sheet_xy(q, rot, t) for q in px]).reshape(-1, 2)
        cands = candidates(ug, dst, cube_u, cube_a)
        it = fi - lo  # place iteration index within the round, if mapping holds (checked below)
        logged = rnd[it]["along_mm"] if it < len(rnd) else float("nan")
        if not cands:
            print(f"frame {fi:03d} it~{it}: {len(dst)} dots, NO candidate within cube tol (logged along {logged:.1f})")
            prev_blobs, prev_off = blobs, None
            continue
        (bw, _), off_w, kw, piw, qiw = cands[0]
        tgt_w = tgt_mm @ np.array([[np.cos(kw * np.pi / 2), np.sin(kw * np.pi / 2)],
                                   [-np.sin(kw * np.pi / 2), np.cos(kw * np.pi / 2)]]) + off_w
        slip = None
        if prev_blobs is not None:
            shift, hits = tm.lattice_offset(prev_blobs, blobs)
            slip = None if shift is None else (round(float(shift[0]), 1), round(float(shift[1]), 1), hits)
        runner = ""
        for (bs, _), off_r, kr, pir, qir in cands[1:]:
            if np.linalg.norm(off_r - off_w) > 1.0:
                runner = f" runner-up n={bs} off={np.round(off_r, 1)} k={kr} lead=({pir}->{qir}) d_off={np.linalg.norm(off_r - off_w):.1f}mm"
                break
        print(f"frame {fi:03d} it~{it}: dots={len(dst)} winner n={bw} off={np.round(off_w, 1)} k={kw} "
              f"lead=({piw}->{qiw}) target={np.round(tgt_w, 1)} logged_along={logged:.1f} "
              f"lattice(prev->this)={slip}{runner}")
        prev_blobs, prev_off = blobs, off_w
    print("\nUG inter-dot vectors (sheet mm):")
    for i, p in enumerate(ug):
        print(" ", i, np.round(p, 1), "-> others:", [f"{j}:{np.round(q - p, 1)}" for j, q in enumerate(ug) if j != i])


if __name__ == "__main__":
    main()
