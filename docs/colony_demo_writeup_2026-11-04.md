# Colony demo write-up, 2026-11-04

The colony demo works end to end from the demo page on the new layout: SAM 3.1 finds colonies, the needle picks one, then goes
to its sample well, A12 (disinfect) and A10 (rinse), and the page records itself. Rig details and history are in
`docs/new_layout_2026-11-04.md`. This file is the summary and the way to run it. Which video is which: `docs/recordings_guide_2026-11-04.md`.

## Result

| | |
|---|---|
| Final recording (user: perfect) | `evidence/tip/demo_rec_1104_151634.mp4`, cameras `demo_rec_1104_151634_cams.mp4`. Local only, not in git |
| That take | 3 colonies -> C10 C11 C12 -> A12 5 s -> A10 2 s, 580 s, plate 4, agar 3.3 mm |
| Wells | A10 A12 C10 C11 C12 recalibrated, 3 of 3 shuffled passes by photo |
| Dishes | All 55 mm, loose |
| Lamps | `lamps.json` [170, 153] |
| Aim | `AIM_OFFSET` [-9, 11] px: last sideways correction 12 mm over the agar, then straight down |

## Run it

1. `<venv python> -u tools/sam_server.py` keeps SAM loaded. Then `start_demo.bat` opens the page (http://127.0.0.1:8765).
2. New plate: enter the agar height the user gives, unchanged. Clear `evidence/tip/colony_picked.json` and `colony_skip.json`.
3. Dish moved? Set `DISH_ROI` in `colony_pick.py`, `DISH_C` in `colony_cycle.py`, `DISH` in `demo_server.py` from a fresh `st2_top.jpg`.
4. Check SAM's marks on the page. Avoid touching colonies, or list them in `colony_skip.json` (out of the run, still on the page).
5. Minimize the terminal, press **Record on / off**, **Run demo**, choose a setting, press Record again at the end.
6. Judge from the photos and the video. `picked` and `entered` in the log only mean the needle went down. A pick counts when the user confirms it.

Run demo takes 3 colonies, retries a failed pick up to 3 times, and Record clears `wells_used.json`.
`tools/chain_picks.py U,V ... --wells` does the same without the page.

## Muse Spark critic

**Run: Muse Spark chooses** on the page runs `colony_cycle.py --agent`. `tools/colony_agent.py` looks at the dish, refuses a lawn, strikes bad marks, can word its own SAM prompt (at most 3 re-runs) and returns colony numbers. The arm goes to SAM's position for each. A refusal ends the run; if the API fails, the operator chooses on the page.
It has run on saved plates only, not yet on the arm: [offline_demo_guide.md](offline_demo_guide.md). Set `DISH_C` for the plate first, or the rim limit rejects good colonies.

## What made it work

| Problem | Fix |
|---|---|
| The view shifts about 1.8 rows when the arm reaches out; dish edges read as needle | `bg_like` aligns the background to each frame |
| Tip read as a colony edge or dish rim | `dark_tip`: lowest point of the whole dark needle, traces as fallback past 40 px |
| Tip flipped between two spots, up to 59 rows | `arrive()`: every pose from 12 mm short and 4 mm up, the same run-up as the wells |
| Pan swing at 4 mm slid the loose dish 1.7 mm | Last sideways move 12 mm over the agar, then straight down |
| Tip over the far wall at 20 mm | Start at 12 mm |
| SAM missed half a dish | 3x3 search tiles instead of 2x2 |
| Plate shifted about 5 mm around 12:07 (probably a bump while setting up a camera, user) | Plate re-solve, entry-path hover, bottom-photo correction, 3 shuffled passes |
| A hidden skip list made "loose" look empty | `colony_skip.json` is separate from what the page shows |

## Known limits

- **Last drop scatter.** The same colony landed 10 rows short at aim 8 and 19 past at aim 17. Scatter of about ±7 px or more remains, so 2 mm
  colonies (about 12 px) give a mix of hits, edge hits and near misses. 3-4 mm colonies were far steadier. The no-contact repeatability
  check proposed on 11-04 was **not run**.
- **Agar height is tuned per plate by eye.** Plate 3 ended at 5.5 mm, plate 4 at 3.3 mm. Pierce depth is cut to agar - 2 mm, so 1.3 mm at 3.3.
- **A12 sometimes lands right of centre**, near the rim, seen on 2 late loops. A rim hit is a plausible way to shift the plate.
- **The poses hold only while plate, robot and cameras stay put.** The other 7 wells of the old 12-well list were not recalibrated: do not use them.
- **Page video.** It films the "Colony picker" window; an always-on-top terminal can cover it. The Record button gives no warning if the window is missing.
- **A failed first tip reading** still stops a pick now and then; the retry usually clears it. Cause not found.

## Restore

Git tags: `restore-2026-11-04-picks` (first picks), `restore-2026-11-04-final` (this state). They hold code, poses and calibration files.
New `.mp4` files and `evidence/tip/phone recordings/` are ignored: they stay on this PC only.
