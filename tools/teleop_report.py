#!/usr/bin/env python3
"""Per-servo deadband, backlash and image Jacobian column (mm/pulse) from tools/teleop.py logs.

A click "moved" when the image motion (d target_x, d target_y, d height) exceeds NOISE_MM. Read-back
`moved` is unreliable (servo 2 reads ~5 low), so it is only shown as a side column.

    python tools/teleop_report.py RUN/log.json [RUN2/log.json ...] [--per-run] [--json OUT] | --selftest
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

NOISE_MM = 0.3


def vec(r):
    t, h = r.get("target_mm"), r.get("height_mm")
    return None if t is None or h is None else np.array([t[0], t[1], h], float)


def clicks(rows):
    """Yield (servo, asked, moved_readback, image-motion vector or None) for each move row."""
    prev = None
    for r in rows:
        v = vec(r)
        if r.get("phase") == "move":
            yield r["servo"], r["asked"], r.get("moved"), (None if v is None or prev is None else v - prev)
        # a target row is a fresh baseline (new target), never differenced against the old one
        if r.get("phase") in ("move", "start", "target", "mark"):
            prev = v


def analyse(runs):
    """runs: list of row lists. Returns {servo: numbers}."""
    acc = {}
    for rows in runs:
        last = {}      # servo -> previous direction
        pending = {}   # servo -> backlash pulses accumulated since a reversal, None when not in one
        for j, asked, rb, d in clicks(rows):
            a = acc.setdefault(j, dict(rev=[], moved_ok=[], still_ok=[], jac=[], n=0, rb=[]))
            sgn = int(np.sign(asked))
            if sgn == 0:
                continue
            reversal = last.get(j) is not None and sgn != last[j]
            last[j] = sgn
            if reversal:
                pending[j] = 0
            if d is None:      # no measurement: cannot classify, drop any open backlash estimate
                pending[j] = None
                continue
            moved = float(np.linalg.norm(d)) > NOISE_MM
            a["n"] += 1
            if rb is not None:
                a["rb"].append(abs(rb) / abs(asked))
            if pending.get(j) is not None:
                if moved:
                    a["rev"].append(pending[j])
                    pending[j] = None
                else:
                    pending[j] += abs(asked)
            elif moved:
                a["moved_ok"].append(abs(asked))
                a["jac"].append(d / asked)
            else:
                a["still_ok"].append(abs(asked))
    out = {}
    for j, a in sorted(acc.items()):
        out[j] = dict(
            clicks=a["n"], reversals=len(a["rev"]),
            backlash=float(np.median(a["rev"])) if a["rev"] else None,
            deadband=min(a["moved_ok"]) if a["moved_ok"] else None,
            max_unmoved=max(a["still_ok"]) if a["still_ok"] else None,
            jac_n=len(a["jac"]),
            jac=np.median(a["jac"], axis=0).round(4).tolist() if a["jac"] else None,
            readback_ratio=float(np.median(a["rb"])) if a["rb"] else None)
    return out


def show(title, res):
    f = lambda v, p="{:g}": "n/m" if v is None else p.format(v)
    print(f"== {title}")
    print("servo clicks  deadband  max-unmoved  backlash(n)    jac mm/pulse (dx, dy, dh) [n]   readback/asked")
    notes = []
    for j, r in res.items():
        jac = "n/m" if r["jac"] is None else "(" + ", ".join(f"{x:+.3f}" for x in r["jac"]) + f") [{r['jac_n']}]"
        bl = f"{f(r['backlash'])} ({r['reversals']})"
        print(f"{j:>5} {r['clicks']:>6} {f(r['deadband']):>9} {f(r['max_unmoved']):>12}  {bl:<13}  {jac:<32} {f(r['readback_ratio'], '{:.2f}')}")
        for k, why in (("deadband", "no moving non-reversal click"), ("max_unmoved", "no non-moving non-reversal click"),
                       ("backlash", "no reversal followed by motion"), ("jac", "no moving non-reversal click")):
            if r[k] is None:
                notes.append(f"servo {j}: {k} not measured ({why})")
    for n in notes:
        print("  note:", n)


def selftest_rows():
    """Pulse-exact model: dead below 3, 5 pulses of play absorbed on each reversal."""
    rows = [dict(phase="target", target_mm=[0, 0], height_mm=10.0)]
    x, direction, play = 0.0, 1, 0  # the first push starts with the play taken up
    for asked in (5, 5, 1, 2, 3, -2, -2, -5, -5, -5, 4, 1, 5):
        sgn = 1 if asked > 0 else -1
        if sgn != direction:
            direction, play = sgn, 5
        eff = 0 if abs(asked) < 3 else abs(asked)
        absorbed = min(play, eff)
        play -= absorbed
        x += sgn * 1.2 * (eff - absorbed)
        rows.append(dict(phase="move", servo=1, asked=asked, moved=0, target_mm=[x, 0.0], height_mm=10.0))
    return rows


def run_selftest():
    r = analyse([selftest_rows()])[1]
    assert r["deadband"] == 3 and r["max_unmoved"] == 2, r
    # reversal 1: -2,-2,-5 unmoved (9 pulses); reversal 2: 4,1 unmoved (5 pulses) -> median 7
    assert r["reversals"] == 2 and r["backlash"] == 7, r
    assert np.allclose(r["jac"], [1.2, 0, 0], atol=1e-3), r
    print("selftest ok:", r)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", nargs="*")
    ap.add_argument("--per-run", action="store_true")
    ap.add_argument("--json", metavar="OUT")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return run_selftest()
    if not args.logs:
        ap.error("give at least one log.json")
    runs = {p: json.loads(Path(p).read_text()) for p in args.logs}
    if args.per_run:
        for p, rows in runs.items():
            show(Path(p).parent.name, analyse([rows]))
    res = analyse(list(runs.values()))
    show("all runs", res)
    if args.json:
        Path(args.json).write_text(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
