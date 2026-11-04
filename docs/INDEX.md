# Documentation index

**Start with [README.md](../README.md).** The maintained set is these five files:

| Doc | What it answers | Keep current? |
|---|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | How the system is built: two machines, the servo protocol, frames and units, the canonical pipeline, calibration caches, safety model | yes — update when a layer changes |
| [TOOLS.md](TOOLS.md) | Every script: what it does, exact CLI, what it reads/writes, current vs superseded, hardware-required vs offline | yes — update when a tool is added or retired |
| [RIG.md](RIG.md) | Hardware, addresses, camera/lamp rules, physical limits, environment setup | yes — update when the bench changes |
| [STATUS.md](STATUS.md) | Robot state to restore, what works, what fails, next steps, git state | **yes — this is the living handover** |
| [ACCURACY.md](ACCURACY.md) | Why the tip misses: the ranked error budget, measured vs asserted numbers, contradictions in the dated record, the measurements that would close the unknowns | yes — update as new measurements land |

Everything else is archived in `docs/history/` and is **read-only history**: dated working documents
from the event. They were left exactly as written. Their claims are sometimes stale or contradicted by
the code — each entry below says whether it is still load-bearing.

---

## 1. The argument chain (how the thinking developed)

The history is not a pile; it is one argument about why the needle misses the dot. Read it in this order.

1. `rca_touch_dot.md` (2026-10-27, revised 10-28) — root-cause analysis of failed dot touches. Central
   insight (watch the descent, judge contact from images, fix offline first) is right and
   code-corroborated; its statistics are not auditable.
2. `critique_2026-10-28.md` and `critique_rca_plan_2026-10-28.md` — reviews of (1) and of the plan that
   followed. The second verdict stands: the plan "is **not executable as written**: its core loop
   (0.5 mm / 0.2 mm corrected steps judged to ≤1 mm) exceeds the measured resolution of every actuator
   and sensor it proposes to use".
3. `plan_2026-10-28.md` — the revised action plan for "first contact on a chosen dot", shaped by (2).
4. `needle_on_dot.md` (2026-10-27) — the procedure/literature behind (3): reach the paper reliably with
   the arm camera, then land on the dot (goal: ≤1 mm off on 4/5 dots).
5. `plan_speedup_2026-10-29.md` — battery/compute budget: move vision work to the desktop, keep the arm
   clock short. Also the source of an **obsolete ArUco spec** (20 mm, ids 0-3/10-13/20) that no longer
   matches `tools/markers.py`.
6. `plan_speed_accuracy_2026-10-30_meta.md`, `plan_review_2026-10-30_meta.md`,
   `plan_speed_accuracy_2026-10-30_AGREED.md` — two competing speed+accuracy plans, a review, then the
   merged plan the user approved. Target: **≤1 mm mark error at ≤120 s per touch**, measured on the
   ink mark, not on in-run estimates. Baseline Touch 1 (`evidence/descend/20261030T103024`): **223 s,
   mark 3.6 mm off** while in-run estimates claimed 0.8 / 0.1 mm. (The `..._OPUS.md` plan it reviews is
   not in this repo.)
7. `accuracy_analysis_2026-10-30.md` — why tuning constants is not enough; endorses the measured-Jacobian
   style of `tools/tip_descend.py` and one-chosen-dot tracking of `tools/dot_servo.py`, and names the
   model-led weaknesses of `tools/model_jab.py`. This is the document that led to the calibration path.
8. `plan_2026-10-30_arm_calibration.md` — the plan that produced `tools/arm_calibrate.py`: fit servo
   offsets + approach terms against externally triangulated tips, "a learnt residual only if earned".
9. `handover.md` (2026-10-30 morning) and `handover_2026-10-31_arm_dot.md` — session handovers. The
   10-31 one is authoritative for the last known state; its content is carried forward into
   [STATUS.md](STATUS.md).
10. `plan.md` — the running plan/log written *during* the hackathon itself, plus the pitch. Historical.
11. `hiwonder_support_request.md` — draft letter to Hiwonder about arm/servo behaviour (roll range,
    180° impossible, motor mode). Still useful; bracketed fields were never filled in.

## 2. Archive table

| File | Date | Type | Standing |
|---|---|---|---|
| `history/plan.md` | 2026-10-26→27 | event log + pitch, 5 milestones, scope decisions | superseded as a plan, still **authoritative for scope and milestones 1-5** and the 10-26/27 numbers; its architecture table names `tools/needle_touch.py` / `tools/servo_shell.py` from the **other** repo (its own preamble says their roles are taken by `rig_io.py` + `servo_bridge.py`) |
| `history/rca_touch_dot.md` | 2026-10-27 (rev. 10-28) | RCA | direction still valid; numbers superseded |
| `history/needle_on_dot.md` | 2026-10-27 | procedure + lit review | superseded by the calibration path, still the best explanation of the touch problem |
| `history/critique_2026-10-28.md` | 2026-10-28 | review | valid, read with (1) |
| `history/critique_rca_plan_2026-10-28.md` | 2026-10-28 | review | valid; its "not executable" verdict was later confirmed by measurement |
| `history/plan_2026-10-28.md` | 2026-10-28 | plan for first contact on a chosen dot | superseded by the 10-29/10-30 plans — but it is the document that **relaxed the acceptance gate to 2 mm** (1 mm only "if step 2 shows a fine axis … ≤ 0.3 mm per pulse"). `history/handover.md` still opens with the stale line "**Current plan: `docs/plan_2026-10-28.md`**" |
| `history/plan_speedup_2026-10-29.md` | 2026-10-29 | plan (49 KB, the largest): W1-W8 → §9 measured adjustments → §10 rethink R1-R6 → §11 S1-S7 → §12 markers + measured base | **authoritative record of 10-29** (origin of the 4K-overhead / 1080p-side camera split, `replay_run.py`, `markers.py`); superseded as a forward plan by AGREED; **stale** marker spec at line ~671 |
| `history/plan_2026-10-30.md` | 2026-10-30 | same-day goals: A underside touch, B needle pick-up, C plate lid | **superseded the same day** (handover: "abandoned today (user: back to the angled needle)"); still useful for the startup chain (`preflight`, park) |
| `history/plan_speed_accuracy_2026-10-30_meta.md` | 2026-10-30 | plan A + baseline (223 s / 3.6 mm) + an appended review of it | merged → AGREED |
| `history/plan_review_2026-10-30_meta.md` | 2026-10-30 | review of plan B: "adopt the Opus plan as the base", 6 amendments | merged → AGREED |
| `history/plan_speed_accuracy_2026-10-30_AGREED.md` | 2026-10-30 | **the user-approved merged plan**: ≤120 s per touch; mark ≤1.5 mm, then gate "1 mm, 2 in a row, 4 of 5"; steps 1-8 | authoritative for 10-30, **de-facto displaced that evening by the arm-calibration pivot**. Its Results log records only "Step 1 … DONE. Regress 5/5"; **steps 3-8 have no logged results**. D1 was closed by `tools/d1_retie_forensics.py` |
| `history/accuracy_analysis_2026-10-30.md` | 2026-10-30 | analysis (22 KB): measured-Jacobian MIMO placing, backlash-as-state, final-arc controller, experiments 1-7 | design direction that fed the calibration plan; its own header says "Status: reasoning only, no code changed" — **no evidence any specific recommendation shipped** |
| `history/plan_2026-10-30_arm_calibration.md` | 2026-10-30 | plan **plus the richest Results section** (gate 0, 84-pose collection, `fit-arm`, `--calib`, 60 mm A/B, 10 mm hover, first `dot`); `:247` documents the `coarse_aim.CALIB` wiring | **one of the two current-truth documents** (with the 10-31 handover); it came from `claude/mlp-servo-tip-mapping-61ee3b` and was merged into `tipcam-external` on 2026-10-31 13:21 |
| `history/handover.md` | 2026-10-30 | running handover, newest-first (10-30 morning back to 10-26) | superseded by the 10-31 handover; partially stale (plan pointer, milestone table) but the best catalogue of what was abandoned. Milestones: **M1 achieved in part, M2-M5 not started** |
| `history/handover_2026-10-31_arm_dot.md` | 2026-10-31 | **latest handover** | authoritative → distilled into [STATUS.md](STATUS.md) |
| `history/hiwonder_support_request.md` | 2026-10-31 | draft vendor request, 13 questions (roll range, torque behaviour, 180 deg) | **draft, never sent**: serial/order number, firmware/stack version and contact fields are still blank. Its §3 rests on a `tools/roll_flip.py` raw-payload probe of 2026-10-31 whose results were **never saved** — `evidence/rollflip/` on disk holds only `arm_at_start.jpg`. Cites `robot/hwr_driver_node.py:143-147` for the torque quirk |

**Referenced but missing from the repo** — a maintainer will look for these and not find them:
`plan_speed_accuracy_2026-10-30_OPUS.md` (half of AGREED's provenance), `tools/mark_bias.py`
(AGREED step 2 / D2 "every historical mark as (along, across)"), `evidence/needle_on_dot/` (promised by
`needle_on_dot.md` Step 6), and the RCA's original logs ("in the session scratchpad, not in the repo").

## 3. Chronology of the build

| Date | Commits | What happened |
|---|---|---|
| 2026-10-26 | 1 | **1 commit**, `fd33f30` 23:18 "rig I/O, grid calibration, needle-tip triangulation, SAM 3.1, Hue control". 5 mm ruling calibration (`paper_grid.py`), two-camera tip triangulation. First numbers: tip **75.2 mm** above paper, rays **0.55 mm** apart, ruler ~7.5 cm, repeats within 0.25 mm, ray miss 0.4-0.6 mm — still quoted verbatim in `tools/tip_triangulate.py:12`. Grid: **1649** UGREEN crossings, 1.7 px rms, f ~1147 |
| 2026-10-27 | 5 | `195a6b7` 10:36 arm-camera touch-off/teleop · `8e51fc1` 11:18 handover · `d705a48` 19:38 **THE JAB** (branch `the-jab`, FF-merged 19:41) · `d5ce078` 20:20 merge · `b2897b1` 23:50 RCA + plan for 10-28 + `model_jab`. Arm-camera-only touching (`arm_touch.py`), `rca_touch_dot.md`, `needle_on_dot.md`, README. Fixes: arm-camera USB dropouts (hub), stale frames passed as fresh, lamps blinding the arm camera |
| 2026-10-28 | 1 | `af0fd1b` 13:19 "wrist-turn touches on dots 1, 2 and red dot 2; red-dot targets, practice arc" — the day the learnt jab was **removed from `model_jab.py`**. Two critiques landed, plan revised. Roll qualified: "gate decided: roll qualifies, **gate 1 mm**" (`handover.md` §10-28). 10-28 dot 1 ink **1.0 mm**, dot 2 to **0.63 mm** (mark 2.35 mm), red 2 **2.7 mm** |
| 2026-10-29 | 4 | `ab37796` 13:32 on new branch `cameras-side-contact-replay`: "4K overhead + side camera, side contact stop, replay harness; **135 s touch**" · `5e30da7` 21:34 · `7d17eb9` 21:39 "Preflight, regression replays, arm calibration tool" · branch `tipcam-external` created 22:09 · `881953c` 22:12 external tip from overhead+side. **4K UGREEN becomes the overhead camera** (`rig_io.OVERHEAD = 1`), 1080p UGREEN re-tasked as the side camera (`SIDE_PAPER = 185.0`). Upright-needle experiments started that night. **Battery died 22:05** |
| 2026-10-30 | 42 | `433822d` 00:34 "Upright grasp test … handover for 2026-10-30". The main day, most of it uncommitted: Touch 1 `20261030T103024` **223 s, mark 3.6 mm**; AGREED plan merged; gate 0 `arm_repeat_20261030T141229` verdict **GO**; calibration collection `arm_calibration_data_20261030T152658` (**84 poses, 78 with pixels**); `servo_torque` driver patch + `teach.py`; evening new bench layout (overhead tilted ~46°, black mat, plates) → `plate_rig.py`, `arm_aim_20261030T212403`, `arm_hover_20261030T213200` (10 mm: **0.13 mm** xy), `arm_hover_20261030T214136` contact **0.81 mm** from the crossing but **no mark**. Ink dots: **1.3 mm** (`075775a`), **2.8 mm** (`5bbe6a8`), **8.4 mm** after a blind press (`2e2485e`) — the skid varies 1.3-8 mm |
| 2026-10-31 | 7 | `tipcam-external` fast-forwards the merged `claude/mlp-servo-tip-mapping-61ee3b` → `1821207` 13:21 (handover + PR #1 recorded), then `b66b291` 15:36 (tip). Overnight evidence: `fit_tip_1000_20261031T003101.json`, `arm_hover_20261031T003405.json`, `…T003713.json`. `fit-tip --roll 1000` **1.11 mm** median over 7 poses (`41523df`), START anchor gated to **15 mm** (`60337e4`), `roll_flip.py` + the `servo_raw` probe proving LX-16A motor mode is a dead end, Hiwonder support request drafted (**never sent**) |

Timestamps come from the local reflog (`+0100`). Commit counts are per-day on this branch. PR #1
(`d4b27e4`, 54 commits into `main`, no CI) is recorded by the 10-31 handover but is not verifiable from
the local repository.

## 4. Evidence archive (`evidence/`)

Run artefacts, not documentation — but they are what the numbers above came from. **Tracked at HEAD:
294 files in 4 directories. Total on disk: ~7,216 files** — the difference is 2026-10-30/31 output that
was never committed. Treat the untracked half as the current state and expect it to be missing on a
fresh clone.

| Directory | Written by | Contents |
|---|---|---|
| `evidence/descend/` | `tip_descend.py`, `model_jab.py` (dashboard `--out`), `rig_io.py` caches | **tracked: 92 files, 67 run dirs** (`20261026T161633` … `20261029T172620`; 66 of them contain only `log.json`), plus `regress_baseline.json`, `ugreen_dots.json`, 15 `teal_before_*/teal_after_*.jpg` lamp-test pairs. **Untracked:** ~114 further run dirs (working tree holds 181 entries), including the most-quoted baseline `20261030T103024`; gitignored `stop.flag`, `exposure.json`, `lamps.json`, `preflight.txt`, `tip_cam.json`, `side_aim_*.png` (`_preflight.png` is untracked, **not** ignored) |
| `evidence/tip/` | `tip_triangulate.py`, `arm_calibrate.py` | **tracked: 159 files** — the calibration state: `arm_model.json`, `arm_calibration_data_*` (4: `151228`, `152658`, `211239`, `run8`), ~30 `arm_hover_*` `.json` + `.top/.side/.zoom/.dot/.touch.*` photos (20261030T212955 → 20261031T003713), `arm_aim_*` (2), `arm_repeat_*` (6, two with `.report`), `fit_tip_1000_20261031T003101.json`, `dot_offsets.json`, `hover_memory.json`, `printed_crossings.json`, `side_pose_cache.json`, `top_pose_cache.json`, `lamps.json`, `side_rig_frame.jpg`, `ugreen_rig.json`, `ugreen4k_lens.json`, `ugreen_lens.json`, `ugreen_lens_source.jpg`, 8 tip triples from 10-26, `roll/` (18 files, 6 timestamps 16:00:48-16:03:22), `strafe/` (6 files, 2 timestamps) |
| `evidence/tip_motion/` | `tip_motion.py` | strafe-triangulation runs: `{n}_ugreen.jpg`, `{n}_arm.png`, `{n}_roll.txt`, `result.json`, `returned.json` |
| `evidence/regress_out/` | `regress.py`, `replay_run.py`, `d1_retie_forensics.py` | replay comparisons and `d1_*.png` scratch images |
| `evidence/dot_touch_20261030*/`, `evidence/teach_20261030/`, `evidence/teach_touch_20261030/`, `evidence/needle_grasp_20261030/` | the 2026-10-30 touch/teach sessions and the abandoned needle-grasp attempt | `path.json`, `grasp_offset.json`, run photos |
| `evidence/roll_tri/` | tip triangulation over roll poses | **tracked ≈19**: `20261026T213728/` (18 files: `0..5_arm.png`, `_roll.txt`, `_ugreen.jpg`) + `20261028T085059_spike/log.jsonl` (the roll/elbow/wrist mm-per-pulse spike cited by `handover.md`) |
| `evidence/tip_motion/` | `tip_motion.py` | **tracked ≈22**: `20261026T210137` (4), `20261026T212810` (9 incl. `result.json`), `20261026T215002` (9 incl. `still_0.png`) |
| `evidence/teach_20261030/`, `evidence/teach_touch_20261030/`, `evidence/dot_touch_20261030*`, `evidence/needle_grasp_20261030/`, `evidence/rollflip/`, `evidence/regress_out/`, `evidence/arm_at_start.jpg`, `evidence/markers_a4.png` | the 10-30 teach/touch sessions, the abandoned needle-grasp attempt, roll probes, regression scratch | **all untracked.** `teach_20261030/path.json` holds the hand-taught grasp poses (`park {1:502,2:726,3:48,4:151,5:500,10:501}`, `grasp_open {1:294,2:399,3:294,4:223,5:152,10:506}`, `grasp_closed … 10:650`); `teach_touch_20261030/keyframes.json` has 5 keyframes; `markers_a4.png` is a gitignored A4 2480×3507 ArUco sheet using the **old** ids (notebook 0-3, holder 10-13, chassis 20) — another reason the stale marker spec survives on disk |

Main-tree evidence breakdown (2026-11-01): `descend/` 5053 · `regress_out/` 1723 (scratch replays) ·
`tip/` 159 · `roll_tri/` 73 · `dot_touch_20261030*` 127 · `teach_20261030` 31 · `tip_motion/` 24 ·
`needle_grasp_20261030/` 21 · `rollflip/` 1.

**There is also 1.6 GB of stale agent worktrees in `.claude/worktrees/`** (8+ directories: `agent-failure-analysis-7c7a99`,
`colony-detection-plates-1a5cfe`, `colony-picking-accuracy-efbc71`, `colony-selection-703597`,
`mlp-servo-tip-mapping-61ee3b`, `resume-latest-handover-9dd246`, `upper-dot-touch-handover-6a04db`, …),
each a full copy of the repo including its own `evidence/` (one holds 4745 files in `evidence/tip` alone).
They are the reason historical documents quote inflated file counts such as "1635 `*_arm.png` files", and the
reason a naive `find` over the repo reports ~14,500 files. They are not part of this branch's history —
`mlp-servo-tip-mapping-61ee3b` corresponds to the already-merged calibration branch — and can be deleted
once confirmed with the tool that created them.

`log.json` inside each run dir is the real record: phases, commanded vs measured mm, per-step
corrections. `tools/replay_run.py` and `tools/teleop_report.py` read them; `tools/regress.py` grades
them. Gitignored runtime artefacts (`stop.flag`, `exposure.json`, `lamps.json`, `preflight.txt`,
`tip_cam.json`, `side_aim_*.png`, `markers_a4.png`) are deliberately not committed.

## 5. Known doc/code conflicts (fixed in the new docs, still wrong in the history)

| Claim in old docs | Reality in code |
|---|---|
| README (2026-10-27): "DSHOW with MSMF fallback" | `rig_io.py` opens **MSMF first**, DSHOW fallback |
| `plan_speedup_2026-10-29.md:671`: ArUco 4×4 20 mm, ids 0-3 / 10-13 / 20 | `markers.py`: `SIDE_MM = 30.0`, `SHEET_IDS = (8,19,24,32)`, `HOLDER_IDS = (9,45,46,6)`, `CHASSIS_ID = 5` |
| `handover.md:138`: "4 recorded runs" for regression | `regress.py`: 5 baseline runs |
| `plan.md` architecture table: `tools/needle_touch.py`, `tools/servo_shell.py` | those live in the pre-existing robot repo; here they are `rig_io.py` + `robot/servo_bridge.py` |
| README tool list (12 tools) | 36 scripts in `tools/` + 3 in `robot/`; see [TOOLS.md](TOOLS.md) |
| `handover.md` opens "**Current plan: `docs/plan_2026-10-28.md`**" | its own §10-30 says the agreed plan is `_AGREED`; the 10-31 handover points at `plan_2026-10-30_arm_calibration.md`. `critique_2026-10-28.md` §4.9 already demanded "Mark which document is current" |
| ACCEPTANCE GATE drifts across the docs | 1 mm (`plan.md` step 1.5) → 2 mm unless a fine axis ≤0.3 mm/pulse (`plan_2026-10-28.md`) → "roll qualifies, gate 1 mm" (`handover.md` 10-28) → ≤1.5 mm then 1 mm (`_AGREED`) — while the newest measured marks are **1.3 / 2.8 / 8.4 mm**. The critiques' objection ("the gate … is smaller than the smallest unit of motion and measurement") is still live |
| Three different "first touch" numbers are quoted as if they were one | 10-28 dot 1 ink **1.0 mm**; 10-30 Touch 1 **3.6 mm**; 10-30/31 contact **0.81 mm with no ink mark**. Only the last is current-method |
| Day-of counts in `plan_speedup_2026-10-29.md` ("68 run logs", "82 logged runs", "1635 `*_arm.png`") and in `rca_touch_dot.md` ("107 runs, 14 contacts", "about 50 code changes went straight onto the arm") | unverifiable from the repo (294 tracked evidence files, 67 tracked run dirs); both critiques label them unverifiable and the RCA itself now buckets 70 of 107 as "learning runs, user stops and aborts without a logged cause" |
| `needle_on_dot.md` §2 promotes closed-loop strafe trim | `rca_touch_dot.md` §3 records the same pulse giving 0, −3.3, 0, 0, 0, **+18 mm**; the gateway refused it ("drive outside bounded no-reverse envelope", `model_jab.py:906`); the 10-31 handover no longer lists strafe as a correction axis |
| Roll guidance `arm_to_start.py` 502 / `arm_park.py` 510 | latest handover: cable-neutral base **490**, no roll command after a hand turn |
| `plan_speedup_2026-10-29.md` cites code lines `tip_triangulate.py:291-321`, `:301-319`, `:311` | `snap_arm()` is now at `tools/tip_triangulate.py:381`; **all line-number citations in `docs/history/` are stale** — grep for the symbol, not the line |
