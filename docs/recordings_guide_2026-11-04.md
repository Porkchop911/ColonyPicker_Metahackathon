# Recordings guide, 2026-11-04

**Use this one:** `evidence/tip/demo_rec_1104_151634.mp4` (page) with `demo_rec_1104_151634_cams.mp4` (cameras). The user called it perfect.
Copies are also in `evidence/tip/phone recordings/`.

## Where things are

| What | Where | In git? |
|---|---|---|
| Page and camera videos | `evidence/tip/demo_rec_1104_<start HHMMSS>[_cams].mp4` | No, this PC only |
| First chained picks and loops | `evidence/tip/chain_picks_1104[b-f].mp4`, `chain_wells_1104[b,c].mp4` | Yes, in commit `91c1d2c`, not pushed |
| Phone recordings (user's) | `evidence/tip/phone recordings/202611_a` (42 files, 6.6 GB), `202611_b` (6 files) | No. Not reviewed by any agent |
| Pick photos | `evidence/tip/pick_0_1104_<HHMMSS>.jpg`: overhead left, side right | Yes |
| Well photos | `evidence/tip/tour_<well>_1104_<HHMMSS>.jpg` | Yes |
| Photo sheets | `evidence/tip/ui_run*_sheet.jpg`, `ui_run*_wells.jpg`, `chain_wells_sheet.jpg` | Yes |

Plain `demo_rec` = the page window at 1920x1046. `_cams` = overhead and side cameras at 1920x540, clock burned in.
Do not read length from the frame count: real length is file start to file modified time. Do not share a page video before watching it:
some show the terminal and session text. Match a video to photos by time.

## Takes

Picks are the user's verdict where given, else "unjudged"; `picked` and `entered` in logs prove nothing.

| Start | Plate, agar | Result | Use? |
|---|---|---|---|
| 11:06-11:28 `chain_picks` a-f | Plate 1, 4 mm | Picks only. `f` = 3 of 3 finished, user: hits. a-e failed or partial | `f` only |
| 11:35-11:49 `chain_wells`, b, c | Plate 1, 4 mm | One full loop each: C10, C11, C12. Stops on touching colonies and a bad reading | Yes, as first loops |
| 12:05 `120515` | Plate 2, 4 mm (real 5) | Picks missed, plate shifted ~5 mm at C10. Terminal in view | Evidence of the shift only |
| 12:41 `124110` | Plate 2, 5 mm | 2 of 3, clean page, C10 skipped | Superseded |
| 12:55 `125559_cams` | Plate 2 | C10 loop, no page video (window gone) | No |
| 13:01 `130135` | Plate 2, 5 mm | 3 of 3, terminal over the page | Cameras only |
| 13:17, 13:21, 13:25 | Plate 3, 6, 5, 5 mm | Aborted, height tuning | No |
| 13:29 `132905` | Plate 3, 5.5 mm | 3 of 3, clean page. C12 near miss (user) | Backup |
| 13:40 `134054` | Plate 3, 5.5 mm | 3 of 3, terminal in view. Last pick clipped, a hit (user) | Cameras only |
| 13:54-14:11 four takes | Plate 3, 5.5 mm | Aborted or 2 of 3 on C11, C12 misses | No |
| 14:25-14:35 four takes | Plate 4, 4, 3.5 mm | Aborted, height tuning | No |
| 14:36 `143623` | Plate 4, 3.3 mm | 3 of 3 on old pierce marks. C12 miss, C11 clipped (user) | No |
| 14:48-15:15 eight takes | Plate 4, 3.3 mm | Aborted, row aim tuning | No |
| **15:16 `151634`** | **Plate 4, 3.3 mm** | **3 of 3 fresh colonies, clean page. User: perfect** | **Final** |

`demo_rec_1104_115835.mp4` and `125945_cams` are a short test and a stray, ignore them. Plate numbers are the order used on 11-04.
