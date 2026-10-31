#!/usr/bin/env python3
"""Roll flip: ONE defined move that rotates servo 5 by 90 deg (quarter turn) and, reversed, returns.

CORRECTION (live, after user caught it): an earlier version of this file claimed 500 -> 875 was
180 deg. It is not: 375 pulses x 0.24 deg/pulse = 90 deg. 180 deg is 750 pulses, and from centre
500 a position-mode move can reach at most +-500 pulses = +-120 deg. A TRUE 180 deg rotation
away from the normal orientation is therefore IMPOSSIBLE in position mode alone; the candidates
are (a) the servo offset register (only +-30 deg - useless), (b) motor mode (LX-16A cmd 29 -
not exposed by the RRC firmware's subcommand vocabulary; would need a raw-frame probe), or
(c) mechanically re-indexing the servo horn by 180 deg (one-time, keeps position mode).
This tool implements the 90 deg quarter-turn, which IS position-mode legal from centre,
verified live both directions (landings 874/875 out, 500/501 back).

    python tools/roll_flip.py probe                # read-only: report the roll state
    python tools/roll_flip.py flip [--ms N]        # normal (500) -> quarter turn (875)
    python tools/roll_flip.py unflip [--ms N]      # quarter turn -> normal (500)
    python tools/roll_flip.py flip --dry-run       # no motion: print the plan only
    python tools/roll_flip.py park-high            # stage the roll at 875 (setup / recovery)

Why this is not motor mode: servo 5 is a 240 deg position servo (0-1000 pulses, 0.24 deg/pulse,
centre 500). A 90 deg move is 375 pulses (500 -> 875), comfortably inside position mode.
Low-level Hiwonder SERVO_OR_MOTOR_MODE does exist (LX-16A protocol cmd 29, speed -1000..1000),
but the RRC STM32 firmware exposes no subcommand for it (vendor SDK has no mode function at all),
and in motor mode the position readback goes blind: the return would be timed, not measured.
Position mode keeps absolute readback the whole way: this move is commanded, watched, and the
landing is verified to +-TOL pulses. Motor mode remains the documented fallback if a future probe
ever shows the firmware forwarding unknown bus-servo subcommands.

The move: staged steps of <=STEP pulses (same stepping discipline as arm_to_start/arm_park:
interpolate, check lag every step, abort holding position if a joint lags). ONLY servo 5 moves;
the other joints are re-sent holding the pose the arm is actually in (read-back clamped to
0-1000, the arm_to_start 'holding' pattern) so nothing else drifts. Put the arm somewhere sane
first (tools/arm_to_start.py): this tool itself only checks the ROLL state, by design.

"Normal roll control resumes" = unflip lands the roll back at NORMAL (500) - bit-identical state
to before the flip (same pulse, position mode throughout, torque state untouched), so
tools/model_jab.py roll0 logic, tip_cam re-solve and ROLL_MAX margins work unchanged.

SAFETY, read before first live flip:
  - The gripper cable winds 90 deg. Inspect slack through the whole arc by hand first
    (teach mode, torque off): gripper free, cable must not snag or pull tight at either stop.
  - The needle sweeps a half-turn about the roll axis. Run with the tip well clear of the paper.
  - The lag abort holds position and raises: nothing is auto-reversed, by design.
  - FLIP_MS is deliberately slow. Speed up only after a clean live run.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rig_io import servo  # noqa: E402
from tip_descend import read, send  # noqa: E402

# --- motor-mode probe (EXPERIMENTAL, RESULT: DEAD END, 2026-10-31): LX-16A
# SERVO_OR_MOTOR_MODE is cmd 29/30 at the silicon level, but the RRC firmware does not
# forward undocumented subcommands. Probed live through this raw path: mode-read guesses
# [0x2C,5], [0x2A,5], [0x1E,5], [0x0F,5] (all reply=None) and mode-write guesses
# [29,5,1,0,100,0] x2 and [29,1,0,100,0,5] (roll sampled 8 s each: 500+-1, no motion).
# Conclusion: motor mode is unreachable through the RRC stack. The only true-180 paths
# left are mechanically re-indexing the servo horn 180 deg, or replacing the RRC board.
# The raw passthrough stays: it is the cheapest firmware-capability probe we have.
MODE_READ_SUB = 0x2C          # first guess probed (dead); kept for probe-raw's default
MODE_REPLY_WAIT_S = 2.0

ROLL = 5
HIGH, NORMAL = 875, 500                    # 500 -> 875 = 375 pulses = 90 deg (quarter turn)
STEP = 100                                 # max pulses per staged step (24 deg)
FLIP_MS = 1200                             # per-step move duration, gentle
TOL = 8                                    # landing tolerance in pulses (~2 deg)
LAG_MAX = 40                               # per-step lag that aborts (blocked / snagged cable)
SETTLE_S = 0.5


def roll_state(pulses: dict) -> str:
    """Name the roll's place in the flip cycle from a read-back dict."""
    roll = pulses[ROLL]
    for name, stop in (("normal", NORMAL), ("high", HIGH)):
        if abs(roll - stop) <= TOL:
            return name
    return f"between (roll={roll})"


def _staged(target_roll: int, ms: int) -> dict:
    """Move ONLY the roll in <=STEP stages, lag-checked; other joints hold the pose they are in.
    Returns the final read-back. Raises (holding position) if the roll lags or lands off-target."""
    here = read()
    # hold the arm where it physically is (arm_to_start pattern); read-back can sit outside
    # 0-1000 (the elbow read -66 parked), so clamp before it goes back out as a command
    base = {k: min(max(here[k], 0), 1000) for k in (1, 2, 3, 4, ROLL, 10)}
    start = here[ROLL]
    if abs(start - target_roll) <= TOL:
        return read()
    steps = max(1, -(-abs(target_roll - start) // STEP))
    print(f"roll {start} -> {target_roll} in {steps} steps of <={STEP}", flush=True)
    for i in range(1, steps + 1):
        stage = int(round(start + (target_roll - start) * i / steps))
        send({**base, ROLL: stage}, ms=ms)
        time.sleep(ms / 1000 + SETTLE_S)
        now = read()
        lag = now[ROLL] - stage
        print(f"  step {i}/{steps}: cmd {stage} read {now[ROLL]} lag {lag}", flush=True)
        if abs(lag) > LAG_MAX:
            send({**base, ROLL: now[ROLL]}, ms=300)  # hold exactly where it stopped
            raise SystemExit(f"ABORT: roll lagging {lag} pulses at step {i} "
                             f"(cable snag / blocked?) - holding {now[ROLL]}, nothing reversed")
    end = read()
    if abs(end[ROLL] - target_roll) > TOL:
        raise SystemExit(f"ABORT: landed at {end[ROLL]}, target {target_roll} "
                         f"(outside +-{TOL}) - state is '{roll_state(end)}', inspect before retrying")
    return end


def flip(ms: int = FLIP_MS) -> dict:
    """normal (500) -> quarter-turn position (875): exactly 90 deg. The defined move."""
    state = roll_state(read())
    if state != "normal":
        raise SystemExit(f"flip needs the roll at normal ({NORMAL}); it is '{state}'. "
                         f"Run `unflip` or `park-high` first, or re-centre the roll.")
    end = _staged(HIGH, ms)
    print(f"FLIPPED: roll at {end[ROLL]} (90 deg from normal). "
          f"Normal roll control is SUSPENDED here; run `unflip` to resume it.")
    return end


def unflip(ms: int = FLIP_MS) -> dict:
    """flipped (875) -> normal (500): the exact reverse. After this, normal roll control resumes:
    position mode was never left, so model_jab's roll0/tip_cam state is valid unchanged."""
    state = roll_state(read())
    if state != "high":
        raise SystemExit(f"unflip needs the roll at the flipped stop ({HIGH}); it is '{state}'.")
    end = _staged(NORMAL, ms)
    print(f"UNFLIPPED: roll back at {end[ROLL]} = normal. Normal roll control resumed.")
    return end


def park_high(ms: int = FLIP_MS) -> dict:
    """Stage the roll at the high stop from wherever it is (first-time setup / recovery)."""
    end = _staged(HIGH, ms)
    print(f"roll staged at quarter-turn stop: {end[ROLL]} (state '{roll_state(end)}')")
    return end


def probe_mode_read(candidates: list[int]) -> None:
    """Motor-mode probe: READ-ONLY mode queries to servo 5, one candidate subcommand
    at a time. No motion is commanded by any of them. Result lands in the driver log:
        ssh ubuntu@192.168.0.223 "grep -a servo_raw ~/hwr/state/mode_session.log | tail"
    reply=[...] with the subcommand echoed at [1] = firmware forwards it; reply=None
    for every candidate = firmware drops unknown subcommands (mode path dead here)."""
    before = read()
    print(f"before: {before}")
    for sub in candidates:
        print(f"sending raw mode-read candidate [0x{sub:02X}, 5] (expect_reply)...", flush=True)
        servo({"raw": [sub, ROLL], "reply": True})
        time.sleep(MODE_REPLY_WAIT_S)
    after = read()
    print(f"after:  {after}")
    print("roll unchanged" if after[ROLL] == before[ROLL] else "ROLL MOVED - inspect!")
    print("now check the driver log for 'servo_raw ... reply=[...]' (see docstring)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("action", choices=("probe", "flip", "unflip", "park-high", "probe-raw"))
    ap.add_argument("--ms", type=int, default=FLIP_MS, help="per-step move duration ms (speed)")
    ap.add_argument("--subs", help="probe-raw: comma-separated subcommand bytes, e.g. 0x2A,0x2C,0x1E")
    ap.add_argument("--payload", help="probe-raw: full raw payload bytes, e.g. 0x1D,5,1,0,100,0")
    ap.add_argument("--reply", action="store_true", help="probe-raw with --payload: ask driver to log a reply")
    ap.add_argument("--watch", type=float, default=8.0, help="probe-raw --payload: seconds to sample the roll")
    ap.add_argument("--dry-run", action="store_true", help="print the plan, move nothing")
    args = ap.parse_args()

    if args.action == "probe-raw":
        if args.payload:
            payload = [int(b, 0) for b in args.payload.split(",")]
            before = read()
            print(f"before: {before}")
            servo({"raw": payload, "reply": bool(args.reply)})
            print(f"sent raw {payload}; sampling the roll for {args.watch:.0f} s...")
            end_t = time.monotonic() + args.watch
            while time.monotonic() < end_t:
                print(f"  roll={read()[ROLL]}", flush=True)
                time.sleep(0.7)
            print(f"after: {read()}")
            return
        subs = [int(s, 0) for s in args.subs.split(",")] if args.subs else [MODE_READ_SUB]
        probe_mode_read(subs)
        return
    now = read()
    print(f"read-back {now}; roll state: {roll_state(now)}")
    if args.action == "probe":
        return
    if args.dry_run:
        need = {"flip": "normal", "unflip": "high"}.get(args.action)
        if need and roll_state(now) != need:
            print(f"DRY RUN {args.action}: REFUSED - needs roll at '{need}', "
                  f"it is '{roll_state(now)}' (the live run raises the same way)")
            return
        target = {"flip": HIGH, "unflip": NORMAL, "park-high": HIGH}[args.action]
        steps = max(1, -(-abs(target - now[ROLL]) // STEP))
        print(f"DRY RUN {args.action}: roll {now[ROLL]} -> {target} "
              f"({abs(target - now[ROLL])} pulses = {abs(target - now[ROLL]) * 0.24:.0f} deg) "
              f"in {steps} steps x {args.ms} ms; other joints held at commanded pose")
        return
    {"flip": flip, "unflip": unflip, "park-high": park_high}[args.action](ms=args.ms)


if __name__ == "__main__":
    main()
