# Operator cookbook: colony-picking demo (for an LLM agent with a shell), 2026-11-03

You operate and record the demo the way the human-supervised operator did on 2026-11-03. **A human is always
watching the robot and gives the agar height and the hit/miss verdicts. You never decide a safety question alone.**
Repo root = cwd; all paths relative to it. Sources: `CLAUDE.md`, `docs/demo_runbook_2026-11-03.md`,
`tools/demo_server.py`, `tools/colony_cycle.py`, `tools/colony_pick.py`, `tools/well_tour.py`, `tools/record_cams.py`,
`tools/rig_io.py`, `start_demo.bat`, `evidence/tip/demo_wells.json`, `evidence/tip/cycle_pick_0.log`.

`P` = `D:\robot\highwonder_rework\.worktrees\hackathon\.venv\Scripts\python.exe` (robot Python, has `sam3`).

## 0. Safety rules (verbatim from CLAUDE.md)

1. **Never fold the arm.** No `arm_park.py`, no PARK, no pose with shoulder > ~600 or wrist < ~300: it hits the
   needle. If the arm reads folded or not where `evidence/tip/well_servo_state.json` says: **send nothing, ask the user.**
2. **Never command servo 5 (roll) or 10 (gripper).** Only joints 1-4 (pan, shoulder, elbow, wrist), values 0-1000.
3. **One driver.** Before moving the arm, check no other process drives it (look for running `well_servo`, `pose_gui`,
   `arm_calibrate`, `colony_*`, `well_tour`, `bg_refresh` Python processes). `Servos.send` also refuses while another
   live process tree holds `evidence/tip/arm.lock`. Never hand the user a run command while your own run is moving.
4. **Dry-run first.** Any new or edited motion code runs `--dry` (simulated arm) before the robot.
5. **Moves go through `well_reach.jgo`** (small interpolated pieces). Never send one big raw move.
6. Do not run `highwonder_rework/deploy/sync.ps1`. No e-stop exists; do not add one.
7. If the user's answer about safety is unclear, ask again. Do not proceed.

Cookbook additions: the page's **Park** is not the forbidden PARK (it is `bg_refresh.py --no-plate`, a raised vertical
pose; `demo_server.py:130`) but still press it only on the human's say-so. Do not create folders outside the repo.
Do not edit code during a demo.

## 1. Preconditions: one driver, one server

```powershell
# who drives the arm / holds the cameras? (expect only the one demo_server)
Get-CimInstance Win32_Process | ? { $_.CommandLine -match 'colony_cycle|well_tour|bg_refresh|colony_pick|pose_gui|arm_calibrate|well_servo|demo_server|record_cams' } | select ProcessId,Name,CommandLine
```

| Finding | Action |
|---|---|
| any of `colony_cycle/colony_pick/well_tour/bg_refresh/pose_gui/arm_calibrate/well_servo` running, not yours | **stop, ask the human** (rule 3) |
| `demo_server` count != 1 process tree (`pythonw` launcher + its child = ONE server) | two servers on port 8765 fight over the cameras: `Stop-Process -Id <each demo_server pid> -Force` (all of them), re-check none left, then start one |
| no `demo_server` | start one (below) |

Start (no terminal needed by the human; `start_demo.bat` = pythonw `tools\demo_server.py` + opens
http://127.0.0.1:8765):
```
cmd /c start_demo.bat
```
Wait until `GET http://127.0.0.1:8765/state` `log` contains both **`-- SAM 3.1 loaded, ready`** and
**`-- cameras open and shared`** (`demo_server.py:55,374`). Cameras are opened once by the page and shared through shared
memory (`rig_io.SharedCamera`); robot tools read them in 0.5 s instead of 20-30 s. If you see `(camera preview: ...)`,
another process holds a camera: find and stop it (with the human), the page retries every 5 s.
`GET /state` also gives `running`, `title`, `exit`, `log` (last 200 lines), `demo` (colonies, `choices`), `wells`, `agar`.

Arm check (read only; the page's Check arm = `POST /action/check`, prints `arm {...}` and `parked` or
`unfolded, not parked: press Park`). If it reads folded or away from `evidence/tip/well_servo_state.json`: rule 1.

## 2. Plate setup (human does the physical part; you do the files)

| Item | Rule |
|---|---|
| Dish | on its marked spot on the black mat; plate where it was; both taped (dish creeps ~5 px = 1.3 mm per pick) |
| Light | same as rehearsal (runbook); lamp 12 only (`hue.set_light('12', ...)`, 13 off) |
| Wells filled | disinfectant B12, water C12, medium in the target wells (`evidence/tip/demo_wells.json`: disinfect `B12`, rinse `C12`, targets `F12 E12 F11 F10`, in that order) |
| Agar height | the number the human gives (agar surface from the plate bottom, mm), **written as is, never add mat/floor**: `POST /agar/6.0` (page field; 3.0-12.0 accepted) = writes `evidence/tip/agar_mm.txt` (`demo_server.py:347`). `POST /agar/auto` = empty = side camera measures. 2026-11-03 value: `6.0` |
| New plate | `evidence/tip/colony_picked.json` -> `[]` (picked colonies by overhead px; a colony within 15 px of an entry is skipped), `evidence/tip/wells_used.json` -> `[]` |
| Same plate, continuing | `wells_used.json` lists target wells that already hold a colony; **a free well whose colony missed must NOT be listed**; a well the colony reached is appended automatically (`colony_cycle.py:226`) |
| Black cover | on the wood behind the plate; side background must not be black/dark (retaken automatically if < 40) |

```
echo [] > evidence/tip/wells_used.json ; echo [] > evidence/tip/colony_picked.json    # new plate only
```

## 3. Recording (start BEFORE the run, stop after the end checklist)

The browser window with the page must be **visible and maximised** (the page fits one screen: dish | cameras + log |
controls + plate). Stamp = `<date>_<HHMM>` (e.g. `1103_1828`).

Screen (ffmpeg from winget; `where.exe ffmpeg` gave
`C:\Users\<user>\AppData\Local\Microsoft\WinGet\Packages\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-8.1.1-full_build\bin\ffmpeg.exe`):
```powershell
$ff = (where.exe ffmpeg | select -First 1)
$t = Get-Date -f 'MMdd_HHmm'
Start-Process -WindowStyle Hidden $ff -ArgumentList "-y -f gdigrab -framerate 15 -i desktop -c:v libx264 -preset ultrafast -crf 26 -pix_fmt yuv420p evidence/tip/demo_screen_$t.mkv" -RedirectStandardError evidence/tip/rec_screen.err -PassThru | % { $_.Id }
```
Cameras (needs the page running; asserts `SharedCamera`; mp4v, 10 fps, both views side by side, timestamp burned in):
```
Start-Process -WindowStyle Hidden $P -ArgumentList "tools/record_cams.py evidence/tip/demo_cams_$t.mp4" -RedirectStandardOutput evidence/tip/rec_cams.log -RedirectStandardError evidence/tip/rec_cams.err
```
Note the PIDs (previous ones were saved in `evidence/tip/rec_pids.txt`). Existing recordings: `demo_screen_1103_1717.mkv`,
`demo_screen_1103_1828.mkv`, `demo_cams_1103_1717.mp4`, `demo_cams_1103_1828.mp4`.

Stop cleanly:
1. Cameras: create the stop file `evidence/tip/rec_stop` (`New-Item evidence/tip/rec_stop`); the script finishes the mp4 and
   prints `N frames, S s -> OUT`. Check `rec_cams.log`. (The script deletes a stale `rec_stop` at its start.)
2. Screen: `Stop-Process -Id <ffmpeg pid>`. Matroska (.mkv) stays playable when ffmpeg is killed: that is how the
   11-03 recordings were stopped (an .mp4 would be broken, hence .mkv for the screen).

## 4. A run, step by step (HTTP API; base `http://127.0.0.1:8765`)

| Step | Call | Notes |
|---|---|---|
| 1 | `POST /action/check` | read-only arm check; wait `running:false`, log shows `arm {...}` |
| 2 | (optional) `POST /action/dry` | `colony_cycle.py --dry --no-view`: SAM only, no motion; `--dry` skips `claim_arm` |
| 3 | `POST /action/run` | = `colony_cycle.py --no-view --max 1`: **one colony per Run** (`demo_server.py:132`) |
| 4 | poll `GET /state` ~1/s | until the log has `HH:MM:SS detections: strict N, normal N, loose N` |
| 5 | `GET /img/choice-normal`, `/img/choice-strict`, `/img/choice-loose` | look at the pictures (dish with numbered markers) |
| 6 | `POST /choose/<strict\|normal\|loose>` | log: `-- detection chosen: normal` |
| 7 | poll until `... cycle done: N/N colonies in their wells, S s` and `--- demo: exit 0` | ~4 min per colony |

Choosing: pick the setting whose markers sit **on real colonies only** (no glare, rim, bare agar). `normal` usually;
`loose` for plates with small colonies (~1.2 mm); `strict` found 0 on 2026-11-03. Markers are sorted nearest the dish centre
(DISH_C = 878,440) first and `--max 1` takes the first, so the picture's first (nearest-centre) marker must be a real colony.
Ask the human if unsure. Other pictures: `/img/dish`, `/img/top`, `/img/side` (live).

What the cycle does (`colony_cycle.py`): lamp 12 on -> background (`bg_refresh.py --lamp12 --no-plate`) -> SAM via the
page (`sam_request` -> `colony_variants.json`) -> **dips disinfect B12 (5 s) then rinse C12 (2 s) at once, while you choose**
(`well_tour.py B12 C12 --dwell B12=5,C12=2`, up to 2 retries) -> waits for `colony_choice.json` -> new background, dish shift
correction (`dish moved [x y] px since SAM`) -> `colony_pick.py --px X Y --again --start 12|20 [--agar A]` (start 12 mm over
the agar if colony y < 520 px, else 20) -> `picked; photo evidence/tip/pick_0_<MMDD>_<HHMMSS>.jpg` -> background ->
`well_tour.py <target well> B12 C12` (one retry for missing wells) -> `wells entered: [...]`.
Per-step logs: `evidence/tip/cycle_pick_<k>.log`, `cycle_tour_<k>.log`, `cycle_tour_start*.log`, `cycle_bg.log`;
result `cycle_result.json`; picture `demo_status.jpg`; `colony_pick.log` accumulates all picks.

## 5. Watching: signals

Read `GET /state` `log` (page) and `evidence/tip/cycle_pick_0.log` (pick). Library noise (`Warn`, `INFO`) is filtered from
the page; camera warnings like "can't grab frame" appear only in raw logs/stderr - if the picture on the page freezes,
see section 6.

| Log line | Meaning | Good / bad |
|---|---|---|
| `agar surface 6.0 mm over the paper (...); pierce 2.0 mm` | agar height used; DEPTH 2 mm | ok (`agar height: not measured ...: stop` = side light bad, give `--agar`/field) |
| `height check k: model 12 mm, side camera X mm over the agar` | model vs side cam; corrected if > 1.5 mm apart | good if close |
| `height not confirmed by the side camera and no recent well offset: no descent` | exit 1, needle lifts | safe skip |
| `step k: H mm over the agar, xy off [dx dy] px (no mirror)` / `(mirror M)` | alignment stage; H runs 12 -> 8 -> 4 -> 2 | the main progress signal |
| `step k: descent drift [x y] px/mm` | measured slant of the model's "down" after a descent | normal <= ~1; ignored if > 4 |
| `step k: reading T implies a drift of ... px/mm: junk, ignored` | physically implausible reading | system handles it |
| `mirror pair at X mm ignored` | tip/mirror pair rejected | neutral |
| `step k: tip reading T far from the expected E, retake` | rejected frame (saved `cp_NN_rejR_top.jpg`) | watch for repeats |
| `step k: tip lost, going on the predicted E (n)` | max 2 blind steps at >= 8 mm | warning |
| `step k: reading T was seen with the arm elsewhere: static junk` / `did not follow a N px move: static junk` | tracer on a static spot | guard working |
| `step k: tip T repeats (the step model was off, not the camera): accepted` | only at >= 8 mm | warning |
| `step k: 4 mm reading T implausible or > 20 px off: back to the 8 mm pose (n)` | 4 mm stage restarts; > 2 -> `error: 4 mm alignment unstable twice` | bad sign |
| `step k: no plausible overhead tip: stop` | alignment gave up | attempt ends |
| `attempt N: back up to the start pose, align again` | up to 3 attempts | bad sign |
| `pick (aligned): H mm over the agar; down D mm (2.0 into the agar)` | pierce after clean alignment | **the only kind that hit** |
| `pick (best pose seen, N px off): ...` | forced pick after failed alignment | **expect a miss** |
| `  descent drift [...]: compensated +x mm reach, -y pan` | drift compensation in the pierce | info |
| `picked; photo evidence/tip/pick_0_*.jpg` | needle went down and a photo was taken | **not a success by itself** |
| `forced pick failed:` / `pick failed (...), skipped` | no pick; needle up | skip |
| `done, needle up` | colony_pick finished | ok |
| `wells entered: ['F11','B12','C12']` | target well + disinfect + rinse reached | want all three |
| `wells not entered: [...], retry` | one retry from the same background | ok once |
| `B12/C12 not entered at the start: stop` | cycle exits 1, needle not clean | tell the human, no run |
| `B12/C12 not entered twice: stop (needle not clean)` | cycle stops | tell the human |
| well_tour: `check W k: ... off [..] px`, `W: 33 px off at 4 mm, well skipped` (CHECK_TOL 9 px at 4 mm), `NN.Ns W: entered pose correction c` / `off` | well entry detail (`cycle_tour_*.log`) | `entered` = in |
| `arm is driven by pid N` / `refusing ...` | another driver / envelope | stop, rule 3 |
| `no free target well left: all hold a colony ...` | `wells_used.json` covers all targets | new plate or edit file |
| `the page SAM did not answer in 60 s: running SAM here` | page SAM dead; slower fallback | ok |

**Key finding (2026-11-03):** the only confirmed hit came from `pick (aligned)` after **every stage 12 -> 8 -> 4 -> 2 mm
had a real reading**, **xy off 2 px at 2 mm** (<= 4 px gate), mirror-confirmed height (`step 6: 2.0 mm over the agar
(mirror 2.8), xy off [-2. -0.1] px`, `cycle_pick_0.log`, photo `pick_0_1103_181713.jpg`). Every forced, blind or
best-pose pick missed. Pierce rule in code: `colony_pick.py:581` (xy off must be <= 4 px at <= 2.5 mm, else <= 6.5 px
correction continues; gain 0.3, cap 3 mm at >= 8 mm, 2 mm lower), pierce goes to `min(mirror, model+1.5)`.

**A pick counts ONLY when the human confirms it.** After every `picked; photo ...`, show the human (or describe) the
photo `evidence/tip/pick_0_*.jpg` (left: overhead, right: side crop) and ask: **hit or miss?** Record the verdict.
Do not treat `picked` or `wells entered` as success. Note `colony_picked.json` is written at every pierce, hit or not.

## 6. Interventions (operator decisions)

| Trigger | Action |
|---|---|
| Reading physically implausible before a pierce (drift > 4 px/mm, xy jumping +-20 px, tip off by tens of px, `junk` lines repeating) or anything looks dangerous (needle near rim, arm not as expected, camera frozen while moving) | `POST /action/abort` once (not twice: a second press killed the first park, `demo_server.py:104`). It kills the step and its children, lifts straight up, parks with `bg_refresh.py --no-plate`; if the arm is off the motion model it says so and moves nothing: **ask the human to lift by hand** |
| Human wants to end after the current colony | `POST /action/stop` (creates `evidence/tip/stop_demo`; removed at next run start) |
| Colony missed (human says miss) | remove its px entry from `evidence/tip/colony_picked.json` so the next run retries it (entries are `[x, y]`, 15 px match). Do not list its target well in `wells_used.json` |
| Colony hit but target well missed | the well is not appended to `wells_used.json`; the colony stays in `colony_picked.json`; tell the human |
| Target well misses repeatedly | reorder `targets` in `evidence/tip/demo_wells.json` (column 12 wells were the most reliable: F12, E12 before F11, F10); ask the human first |
| `B12/C12 not entered` | `POST /action/park` only if the arm is raised/unfolded; ask the human about the wells (filled? background?), rerun |
| Side camera background too dark/black | bad light or missing black cover; fix light, rerun (cycle retakes bg when < 40) |
| Camera fighting / "camera preview" errors / frozen live images | exactly one demo_server (section 1); no other program opens the cameras; restart the server via `start_demo.bat` when no run is moving |
| `no free target well left` | new plate -> empty both json files (section 2), or edit `wells_used.json` with the human |
| Lamp | lamp 12 stays on during the whole cycle; do not switch lamps (switching re-locked the exposures) |

## 7. Known failure causes found on 2026-11-03 (git log) and their fixes

| Cause | Fix (commit) |
|---|---|
| Arm model off after a reboot: reach ~18 mm, pan shift vs the wells | start pose = wells' shift (`715a063`); `descent drift` measured and expected (`06d515f`); heights from the side camera first (`46e0677`) |
| Slanted "down": tip moved ~10 px per mm | drift measured per descent, compensated at the hover, then straight down (`f14371d`); readings implying > 4 px/mm ignored, correction capped at 4 mm (`203a6a3`) |
| Tracer stopped on the colony / rim | only the target colony blocks readings (`797abcc`, `13bad9f`); colonies nearest the dish centre first (`0f04461`) |
| Alternating / jumping tip readings (+-20 px) | static-junk guard (`715a063`, `6b16512`), median-nearest of three readings (`6662d37`) |
| Pan backlash | approach must stay **10 pulses**: with 3 corrections vanished in the gear play (`3b4e027`; `4aed7a7` caused it) |
| Alignment overshoot | gain 0.3, not 0.6 (`b8b04a9`) |
| Stopped short / forced blind picks missed | pierce 2 mm into the agar (`b8c3463`); extra aligned stage at 2 mm (`b795b3a`); pierce only after clean alignment <= 4 px, 3 attempts (`1ca5143`) |
| Camera open latency 20-30 s, frozen feeds | page holds both cameras, shared memory (`84e7c08`) |
| Lamp switching re-locked exposures; background taken while the lamp faded | lamp 12 stays on, 2 s settle (`0d77d41`, `lamp12()` in `colony_cycle.py`) |
| 4 mm well check refused good entries | gate 9 px (`36e9753`); E12 before F11 (`d84d1d7`) |
| Dark side background, stale background | retaken (`eb9f4f3`); dish shift corrected before the first colony too (`8ac4aa2`) |
| Rinse well hit the rim | rinse C12 instead of A11 (`abfddf1`) |

## 8. End-of-run checklist

1. `GET /state`: `running:false`, `exit:0`, log has `cycle done: N/N ...`; the needle is up (`POST /action/check` -> `arm {...}`, not folded).
2. Ask the human: **hit or miss** for each pick; look at `evidence/tip/pick_0_*.jpg`.
3. Update files: miss -> remove its entry from `colony_picked.json`; target well not reached -> not in `wells_used.json`; hit and reached -> already in `wells_used.json`.
4. Note `agar_mm.txt` value, detection chosen, run time, verdict, and the photo path in your report.
5. Stop recordings (section 3): `rec_stop` for the camera mp4 (check `rec_cams.log`), `q`/stop ffmpeg; confirm file sizes of `demo_screen_*.mkv` and `demo_cams_*.mp4`; delete `evidence/tip/rec_stop` if left over.
6. Do not close the page server unless asked (close the `pythonw` in Task Manager to stop it). No commits unless the human asks.
