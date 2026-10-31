# Handover 2026-10-31: arm calibration, camera rig, dotting grid crossings

Branch `claude/mlp-servo-tip-mapping-61ee3b` (worktree `.claude/worktrees/mlp-servo-tip-mapping-61ee3b`), last commit
before this file `60337e4`. Not pushed, no PR. Plan and results: `docs/plan_2026-10-30_arm_calibration.md`.
Run everything with `D:/robot/highwonder_rework/.worktrees/hackathon/.venv/Scripts/python.exe`.

## Robot state right now (fix first)

| What | State | To restore |
|---|---|---|
| Roll servo 5 | holding at ~1000 (+122 deg from base), arm raised, pan centred (~498) | command roll back to **490** (base, cable neutral) slowly, inside 0-1000 only |
| Robot driver `~/hwr/ros2_ws/src/hwr_driver/hwr_driver/driver_node.py` | **patched**: servo 5 allowed up to 1240 (servo itself clamps at 1000, so useless) | `cp driver_node.py.bak_roll driver_node.py`, then restart the driver |
| Robot bridge `~/colony/servo_bridge.py` | **patched** the same way | `cp servo_bridge.py.bak_roll servo_bridge.py` (takes effect on next bridge spawn) |
| Driver process | runs from a **manual nohup** start (pid was 51867), not from `hwr.service` | after reverting: kill it, start `driver_node --ros-args -r __node:=hwr_driver --params-file .../driver.yaml` in **zsh** with `~/hwr/deploy/env.zsh` sourced, or restart the stack the usual way |
| Hue lamps | 13 on (bri 160, ct 350), 12 off | `auto_lamps` sets them each run |

Roll rules (also in memory `roll-servo-cable`): base 490, cable allows +-180 deg from base, the servo commands and senses
only 0-1000 (240 deg). Past 1000 it moves but its reading aliases (a vertical needle read 225, same as a flat pose 180 deg
away) and commands clamp at 999. A command after a hand rotation wound the gripper a full turn (cable at its limit).
**No roll command after anyone hand-turns it until the user has unwound it.**

## What works

- **Cameras** (`tools/plate_rig.py`): overhead (tilted ~46 deg) + side camera, both posed in one sheet frame from the
  ruling and the 96-well plate; overhead creep is followed by `refresh()` (same frame). Tip rays meet to 0.3-0.8 mm.
- **Arm model** (`evidence/tip/arm_model.json`): servo offsets + approach-side terms + per-session tie; per-roll tips
  (`tip_by_roll`). Live A/B at 60 mm: calibrated 0.95 mm vs nominal 1.91 mm xy median.
- **`arm_calibrate.py`** commands: `collect`, `fit-arm`, `fit-tie`, `fit-tip --roll R`, `aim`, `hover [--roll R]`,
  `dot [--roll R]`. Each run: lamp autotune -> anchor at START (gated 15 mm) -> printed crossing (local grid fit, cached)
  -> measured descent with per-goal memory (10 mm in ~40 s) -> touch -> mark scored by difference image.
- **Placement in the air**: 0.1-0.5 mm at 10 mm over a printed crossing (flat needle, roll 502).
- **Tip check (user's idea)**: `fit-tip` prints model-vs-measured per pose and refits the tip; roll 1000: 1.11 mm median.

## What does not (yet)

- **Ink marks land 1.3 / 2.8 / 8.4 mm from the crossing** (flat needle). The needle meets the paper at ~72 deg from
  vertical and skids by a varying amount once pressed. Height-only contact stopped above the paper (no ink); side-row
  stall contact fired falsely once. `DOT_STOP_AT_CONTACT = False`, offsets per roll in `evidence/tip/dot_offsets*.json`.
- **Vertical needle**: the user wants 180 deg roll (vertical). Roll 1000 (+122 deg) is steep but rejected by the user;
  with it the tip sits ~97 mm off the gripper axis and the crossing needs pan +145, which the user forbids (the needle
  can jam into the left wall): **keep pan near centre**.
- Wet ink lasts minutes; the user hangs a drop right before each `dot` (say "go" first, hands at the needle).

## Next

1. Restore the robot (table above).
2. Get a commandable vertical needle: re-seat the gripper on the roll spline so cable-neutral sits near pulse ~250 and
   vertical near ~1000 (user's call; hand-set-and-hold works only until the next roll command).
3. `fit-tip --roll <vertical pulse>` (~4 min), check reach of open-paper crossings with pan within +-25 of 497 (at roll
   1000 that was crossings around y = 140-155 mm in the sheet frame), then `dot` with fresh ink.
4. Later: more tie poses near the far crossings, `model_jab --calib`, push + PR.

## Standing user rules (memory)

No folders outside the repo; be brief; no e-stop; never suggest stopping; SAM 3.1 for targets; no open-loop low poses;
watch long runs with a stall watchdog; roll rules above; never run `highwonder_rework/deploy/sync.ps1`.
