# New layout: state and how it was reached, 2026-11-04

Read `CLAUDE.md` first (safety rules). This is the newest state. It supersedes the numbers in
`docs/demo_runbook_2026-11-03.md` and `docs/demo_operator_cookbook_2026-11-03.md`; their procedures still apply.
Work is in worktree `.claude/worktrees/camera-layout-2026-11-03`, branch `camera-layout-2026-11-03`, uncommitted.

## Repeatable wells

**A10 A11 A12 C10 C11 C12 D12 E10 E12 F11 G11 G12**: 12 wells, 3 of 3 passes, each pass in a different order.

```
<venv python> -u tools/well_tour.py A12 C10 F11 --blind
```

- Machine-readable list: `evidence/tip/repeatable_wells.json`. Poses: `evidence/tip/well_poses.json`, frozen copy
  `evidence/tip/well_poses.frozen12_1104.json`.
- Evidence: `evidence/tip/f{1,2,3}_{a,b}.jpg`. Green cross = well centre. Tips land about 2 mm up-left of it, inside the rim.
- Verdicts are by eye from both views. **`entered` in the log means only "the script went down"** under `--blind`.
- Depth: 3 mm under the rim.
- `demo_wells.json` (user, 11-04): disinfect A12, rinse A10, samples C10 C11 C12, all from this list.

## The rig now

| Item | State |
|---|---|
| Overhead | Camera 1, **4K (user requirement)**, 23 deg off vertical, ~6 px/mm. `rig_io.FRAME_SIZE` |
| Side | Camera 0, 1080p, ~127 mm up |
| Lamps | Hue 12 and 13 both overhead, `lamps.json` [170, 153] |
| Room light | **Must stay off**: it cast the arm's shadow across the dish |
| Rubber band | Stays (user). Runs between columns D and E; D12, E10 and E12 enter fine beside it |
| Chassis camera | Aurora 930 via the bridge's `snap`: sees the dish, soft, upside down. Not used yet |
| Reach | 35 wells by the model, columns 7-12 only. Reach limit about x = 294 |

## Plate moved, 2026-11-04 ~12:07

During a recorded page run the plate and dish shifted together around the C10 entry; the paper stayed. Cause not found. Recalibrated A10 A12 C10 C11 C12 only,
3 of 3 shuffled passes (`evidence/tip/v{1,2,3}.jpg`, `well_poses.frozen5_1104.json`). **The other 7 wells of the list are not valid now.** `hover_map.py` now uses the entry path
(12 mm run-up); without it the tip read 20-40 px off. On the bottom photos a centred tip sits 8 display px below the cross (parallax of the 3 mm depth).

Fresh plate from 11-04 12:00: agar 5 mm (user), in `evidence/tip/agar_mm.txt` for the page. The page's Record button clears `wells_used.json`. The plate shift was probably a bump while setting up a camera (user).

Final recording (user: perfect): `evidence/tip/demo_rec_1104_151634.mp4`, cameras `demo_rec_1104_151634_cams.mp4`. Plate 4, agar 3.3 mm, `AIM_OFFSET` [-9, 11], 3 colonies -> C10 C11 C12 -> A12 -> A10, 580 s.
Page Run demo does 3 colonies and retries a failed pick 3 times. `colony_skip.json` leaves colonies out of a run but on the page. SAM uses 3x3 tiles (2x2 missed half a dish). Open: last drop scatters up to 19 rows, A12 sometimes lands right of centre, the no-contact drop check was not run.

## Well procedure (what finally worked)

1. Seed poses from the model. 2. Hover at 2 mm with `tools/hover_map.py`, no contact, and read tip vs cross on `hm_<well>.jpg`.
3. `tools/pose_fit.py` for a smooth fit, then a per-well correction. 4. Enter with `well_tour.py --blind`, read bottom photos with
`tools/entry_sheet.py`. 5. Correct with `tools/bottom_fix.py`, **gain 0.45** (real moves were about 2x the Jacobian). 6. Freeze, run
3 shuffled passes, change nothing between them.

## What failed, and why

| Tried | Result | Lesson |
|---|---|---|
| Camera loop over the plate (`well_tour` without `--blind`) | Tip readings jump over the white plate; G11 and D9 wandered | Do not steer on the tip over the plate |
| Side camera height at A12 | Read 9-15 mm at a true 15 (spiral behind it) | `--fixed-height`, `well_z_offset.json` |
| Trusting the log | A12 logged `entered`, photo showed A11 | Always read the photo |
| Blind entry with no fixed arrival | Bottom flipped between two spots 4.5 mm apart | Backlash: **every arrival from 12 mm short** (automatic with `--blind`) |
| Free pan rate in the arm fit | -0.54 deg/pulse, unphysical | Keep -0.24 |
| Patching the tracer past the dark band | Each fix broke another frame; removed | Remove the cause (the shadow), do not tune around it |

## Colony picking state

- 2026-11-04 10:19, new plate (55 mm dish, loose): SAM target, `colony_pick.py --no-mirror --agar 4 --depth 1`, aligned 3 px at 8 mm, needle in the colony centre in both photos (`evidence/tip/pick_0_1104_101941.jpg`). **User: hit, could go 1 mm deeper**, so `--depth 2` and `FLOOR_MM` 2.
- Last sideways correction is at 12 mm (16 over the paper, over the 14.5 mm wall), then straight down: the pan swing at 4 mm pushed the needle into the inner wall and slid the loose dish 1.7 mm. At 8 mm the tip's image sits on the colony edge and every tip reading fails there.
- Every pose arrives by `arrive()`: 12 mm short and 4 mm up first, because the tip flipped 25-59 rows with the direction of a small reach correction (6 mm short still flipped). Overhead tip = `dark_tip`, the lowest point of the whole dark needle, when within 40 px of the expectation, else the traces.
- The view shifts ~1.8 rows with the arm out (`bg_like` aligns the background). A straight descent pulls the tip ~1-2 rows/mm toward the robot; `AIM_OFFSET` [-6, 14] is for the drop from 12 mm with the 12 mm run-up: 2 rows landed 11-15 rows short.
- Height checked with the colonies as a ruler: true 8.8 mm at a commanded 8, 19.7 at 20. `--agar` is taken as given; at 4 mm agar `--depth 2` is the limit (tip 2 mm over the paper).
- SAM found the dish at 7 lamp settings in daylight (`tools/sam_sweep.py`); not re-tested in the dark. SAM colony order changes between runs: name the colony by position (`--px` from the SAM list).
- Idea from the user: nudge the loose dish on purpose for picks the pan cannot reach, then reimage with SAM. Not built.
- The dish moved 5.7 px left over the failed attempts on one colony: retake the background and the SAM list before every pick.
- The demo page's `DISH`, `WORK` and `DISH_C` are set for this layout but have never been run.

## How to pick a colony (user-confirmed hit, 2026-11-04)

Dish loose, overhead lamps `lamps.json`, needle checked by the user. From the repo root, robot Python:

0. Keep `tools/sam_server.py` running: SAM stays loaded and answers in seconds; `colony_pick.py` falls back to loading it.
1. `tools/bg_refresh.py --no-plate`: arm out of view, new background. Redo after anything moves the dish.
2. `tools/colony_pick.py --dry --no-mirror --agar 4`: SAM lists colonies and reach. Agar number exactly as the user gives it.
3. Choose by position, not index (SAM reorders): isolated, more than ~8 mm from the wall, tip at the 20 mm start not over the rim.
   Avoid a colony with a faint smudge touching it: the tip trace can still run onto the smudge (test frame `gap_16`).
4. `tools/colony_pick.py --px U V --no-mirror --agar 4 --depth 2` (add `--again` to repeat one). It starts 20 mm over the agar,
   aligns on the overhead, drops to 12 mm, aligns to 6 px, goes straight down, photographs, lifts 25 mm.
5. Read `evidence/tip/pick_*.jpg`. A pick counts when the user confirms it.

Several in a row, filmed (11:23, three picks, 279 s, `evidence/tip/chain_picks_1104f.mp4`, verdict pending): run `tools/sam_server.py` and `tools/cam_share.py`,
then `tools/record_cams.py OUT.mp4`, then `tools/chain_picks.py U,V U,V U,V`; `touch evidence/tip/rec_stop` ends the filming and the shared cameras. The chain starts
at 12 mm (`--start 12`): from 20 mm the tip sits over the far wall for upper colonies.

With the wells (11:36-11:49): `chain_picks.py U,V ... --wells` runs colony -> its sample well -> A12 5 s -> A10 2 s from `demo_wells.json`, `--first-well N` resumes, `--again` allows a colony picked before.
Three loops done, in three runs (`evidence/tip/chain_wells_1104{,b,c}.mp4`, photos `chain_wells_sheet.jpg`), user verdict pending. Two stops, needle up, nothing pierced: a touching pair of colonies, and a tip reading off by 47 px at 12 mm. Avoid touching colonies.

Offsets to adjust by the photo: `AIM_OFFSET` in `colony_pick.py` (px, x then rows). Stops with the needle up are normal; the log says why.

## If something moved

| Moved | Redo |
|---|---|
| Plate, or either camera | `plate_rig.py --images TOP SIDE` (4K top, side at exposure -5), check labels, then the well procedure |
| Robot base | `a1lib` C, ANG600, R_OFF from triangulated tips (`tools/reach_out.py`, `tools/drop_test.py`), then the well procedure |
| Lights | Retake `bg_refresh.py --lamp12 --no-plate`. Wells under `--blind` use no camera; colony picks do |

Tools added: `reach_out.py`, `drop_test.py`, `step_in.py`, `hover_map.py`, `pose_fit.py`, `bottom_fix.py`, `entry_sheet.py`, `hold_look.py`.
Backups: `well_poses.oldlayout_1103.json`, `well_poses.seed_1104.json`, `well_poses.hover2_1104.json`.
