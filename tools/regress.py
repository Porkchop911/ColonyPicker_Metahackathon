#!/usr/bin/env python3
"""Regression check before code goes onto the robot: replays recorded runs (tools/replay_run.py) with the current code.

    <hackathon venv python> tools/regress.py --out DIR [--save]

FAIL when a replay raises a Python error that is not one of model_jab's own aborts (KeyError, NameError, ValueError,
TypeError, ...: a code bug; two reached live runs on 2026-10-29), or when it diverges from its recording earlier than in
the saved baseline (evidence/descend/regress_baseline.json; --save writes it). A replay is open-loop: past the first
changed decision its rows mean nothing, so the divergence row is the measure, not the end of the run.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

RUNS = {  # recorded runs with every frame saved: what each exercises
    "20261029T170928": "touch on red dot 1: tie, rounds, tracked descent, tri-height stop",
    "20261029T172244": "reach strafe, re-tie, UGREEN rounds, practice arc",
    "20261029T152609": "touch on a black dot chosen with --dot, long placing",
    "20261029T171326": "reach strafe, then an arm view with no dots (the vote_offset crash)",
}
BASELINE = Path("evidence/descend/regress_baseline.json")
OWN_ABORTS = ("RuntimeError", "OutOfFrames", "KeyboardInterrupt")


def replay(run: str, out: Path) -> dict:
    proc = subprocess.run([sys.executable, "tools/replay_run.py", f"evidence/descend/{run}", "--out", str(out)],
                          capture_output=True, text=True, timeout=900)
    text = proc.stdout + proc.stderr
    raised = re.findall(r"replay raised: (\w+)\(", text)
    div = re.search(r"DIVERGED at row (\d+)", text)
    errors = sorted({e for e in raised if e not in OWN_ABORTS} | ({"crash"} if proc.returncode else set()))
    return {"diverged_row": int(div.group(1)) if div else None, "raised": raised[-1] if raised else None,
            "bugs": errors, "tail": text.strip().splitlines()[-3:]}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, required=True, help="an existing folder for the replays' run folders")
    ap.add_argument("--save", action="store_true", help="store these results as the baseline")
    args = ap.parse_args()
    base = json.loads(BASELINE.read_text()) if BASELINE.exists() else {}
    results, failed = {}, []
    for run, what in RUNS.items():
        r = results[run] = replay(run, args.out)
        old = base.get(run, {}).get("diverged_row")
        earlier = old is not None and r["diverged_row"] is not None and r["diverged_row"] < old
        bad = bool(r["bugs"]) or earlier
        failed += [run] if bad else []
        note = f"bugs {r['bugs']}" if r["bugs"] else (f"diverges at row {r['diverged_row']}, baseline {old}" if earlier else "")
        print(f"{'FAIL' if bad else 'OK  '} {run}  diverged at row {r['diverged_row']}, ended by {r['raised']}  "
              f"({what}) {note}", flush=True)
        if r["bugs"]:
            print("      " + "\n      ".join(r["tail"]))
    if args.save:
        BASELINE.write_text(json.dumps({k: {"diverged_row": v["diverged_row"], "raised": v["raised"]} for k, v in results.items()}, indent=1))
        print(f"baseline saved: {BASELINE}")
    print(f"\n{len(RUNS) - len(failed)}/{len(RUNS)} OK")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
