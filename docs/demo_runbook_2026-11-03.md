# Colony-picking demo: runbook (2026-11-03)

Every colony on the dish goes into its own target well; after each colony the needle goes into the disinfect well
(5 s) and the rinse well (2 s). Wells: `evidence/tip/demo_wells.json` (2026-11-04: disinfect **A12**, rinse **A10**,
targets C10 C11 C12).

## Start without a terminal

Double-click `start_demo.bat` (repo root): it starts `tools/demo_server.py` and opens http://127.0.0.1:8765.
Buttons: **Check arm** (read only), **Park**, **Dry run** (SAM finds the colonies, wells assigned, no picking),
**Run demo** (the whole cycle), **Stop after this colony** (ends after the current colony incl. disinfect + rinse),
**ABORT** (kills the running step at once, then lifts the needle straight up and parks; if the arm is off the motion
model it says so and nothing moves: lift by hand). Panels: status (dish + plate map), both cameras live (frames
written by the tools while a camera is open) with the **target colony** (yellow), **target well** (green) and **next
well** (blue) on the overhead, and the log. Close the server by closing its window / `pythonw` in Task Manager.

State on 2026-11-02 evening: 8 of 9 colonies transferred one by one (`evidence/tip/all_picks_1102.jpg`); the
automatic cycle (`tools/colony_cycle.py`) passed its dry run but has **not yet run live end to end**: the first
live test was spoiled by a second run driving the arm at the same time (now prevented by the arm lock).

## Before the demo (no robot motion)

| Check | Why |
|---|---|
| Daylight or a fixed lamp setup, same as the rehearsal | evening light broke the agar height (side view) and the tip's mirror in the agar; SAM scores fell 0.85 -> 0.6 |
| Fresh dish in the same spot on the black mat; plate where it was; both taped down | dish creeps ~5 px (1.3 mm) per pick; the plate is light |
| Wells filled: disinfectant in A12, water in A10, medium in C10 C11 C12 | targets in this order (`evidence/tip/demo_wells.json`) |
| Black cover on the wood behind the plate | the needle tip is followed against it at 10 and 4 mm over the wells |
| No other arm program running (`well_tour`, `colony_*`, `bg_refresh`, `pose_gui`, `well_servo`) | two drivers at once = violent moves (23:31); a second mover now refuses (`evidence/tip/arm.lock`) |
| Arm unfolded, near the out-of-view pose {1:600, 2:450, 3:207, 4:468} | safety rule 1 |

Read the arm (no motion):
```
python -c "import sys; sys.path.insert(0,'tools'); import well_reach as wr; print(wr.Servos().read())"
```

## Rehearsal (robot, ~15 min)

`P` = `D:\robot\highwonder_rework\.worktrees\hackathon\.venv\Scripts\python.exe`, run from the repo root.

1. `P tools/test_colony_pick.py` -> `ok`.
2. `P tools/colony_cycle.py --dry` -> the window shows the dish with numbered colonies and their wells. Check by eye:
   every colony circled, nothing on bare agar or the rim. Wrong ones: see "Colony detection" below.
3. `P tools/colony_cycle.py --max 1` -> one full colony -> well -> A12 -> B12. Watch it; check the pick photo.
4. If the agar line is not measured (log: `agar height: not measured`), add `--agar 5.4` only if it is the same dish
   type and fill as 2026-11-02 (5.3-5.5 mm measured); otherwise fix the light.

## Demo

```
P -u tools/colony_cycle.py > evidence/tip/demo_run.log 2>&1
```
`--again` repeats on colonies already picked, `--max N` limits the count, `--dwell A12=5,B12=2` sets the contact times.
~4 min per colony. Result: `evidence/tip/cycle_result.json`; per step logs `evidence/tip/cycle_*.log`.

Behaviour: a colony that cannot be aligned is skipped (needle lifts). A well that is not entered is retried once; if
A12 or B12 still fails, the cycle stops (the needle would carry over unclean).

## When something fails

| Log / window says | Usual cause | Do |
|---|---|---|
| `0 colonies` | no dish, or SAM finds nothing in bad light | light; check `evidence/tip/st2_top.jpg` |
| colony circled on bare agar / rim | SAM glare | `colonies()` in `tools/colony_pick.py`: size >= 3 mm, score >= 0.5, contrast > 35 (colonies 44-82, glare 11-24) |
| `pick failed ... no plausible overhead tip` at 20 mm | far-row colony (tip over the far rim), or the arm model off | far row starts at 12 mm automatically (`FAR_Y`); else skip it |
| `back to the 8 mm pose` / `4 mm alignment unstable` | junk tip readings at 4 mm | guard working; colony skipped. Frames: `cp_*_rej*_top.jpg` |
| needle misses the colony by ~3 mm sideways | blind descent too long | final alignment is at 4 mm since 2026-11-02; check the light / mirror |
| A12/B12 missed (user saw it) | background taken while the lamp was still fading (suspected, 21:42) | 2 s settle is in; rerun `P tools/well_tour.py A12 B12` |
| `arm is driven by pid N` | another mover is running | stop that one; never run two |

## Wells: how a well is entered (2026-11-03)

Height from the side camera at 15 mm over the rim (offset to the arm model kept in `evidence/tip/well_z_offset.json`;
it was +10-12 mm after a reboot), camera alignment at 10 mm, a look at 4 mm (colony picker's tip finder, 2 of 3
frames agree, <= 6 px), corrections only back at 10 mm, then straight down 7 mm. Uncertain -> the well is skipped.
