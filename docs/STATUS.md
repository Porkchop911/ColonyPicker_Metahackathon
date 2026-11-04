# Status

**Where the project stands, and what to do first.** Recorded from `docs/history/handover_2026-10-31_arm_dot.md`
(the newest handover) plus the code at `tipcam-external` tip `b66b291` (2026-10-31 15:36 +0100).
Commit activity spans 2026-10-26 → 2026-10-31. If you are reading this after a new session, update this
file rather than writing another handover.

---

## 1. Robot state to restore before anything else

| Item | State at last handover | Restore |
|---|---|---|
| Roll servo 5 | holding ~**1000** (+122° from base), arm raised, pan centred (~498) | command roll back to **490** (cable-neutral base), slowly, inside 0-1000 only |
| Robot driver `~/hwr/ros2_ws/src/hwr_driver/hwr_driver/driver_node.py` | **patched** to allow servo 5 up to 1240 (the servo itself clamps at 1000, so the patch is useless) | `cp driver_node.py.bak_roll driver_node.py` then restart the driver |
| Robot bridge `~/colony/servo_bridge.py` | **patched the same way** | `cp servo_bridge.py.bak_roll servo_bridge.py` (takes effect on the next bridge spawn) |
| Driver process | started manually with **nohup** (pid was 51867), not from `hwr.service` | kill it, then either `sudo systemctl restart hwr` or start `driver_node --ros-args -r __node:=hwr_driver --params-file .../driver.yaml` in **zsh** with `~/hwr/deploy/env.zsh` sourced |
| Hue lamps | 13 on (bri 160, ct 350), 12 off | `auto_lamps` resets them at the start of every run |

### Roll rules (hard-won, do not relearn them)

- Cable-neutral base is pulse **490**; the cable allows ±180° from there.
- The servo commands and senses only 0-1000 pulses = 240°. **Past 1000 it still moves but its reading
  aliases** — a vertical needle read 225, which is indistinguishable from a flat pose 180° away — and
  commands clamp at 999.
- One command issued after a hand rotation wound the gripper a full turn to the cable limit.
  **After anyone hand-turns the roll, no roll command until the user has unwound it.**
- Consequence: a true 180° roll (vertical needle) is not commandable in position mode; LX-16A motor mode
  failed ("the RRC firmware does not forward undocumented subcommands"). Remaining options: re-seat the
  gripper on the roll spline so cable-neutral sits near pulse ~250 and vertical near ~1000, or replace
  the RRC board.

## 2. What works today

| Capability | Measured result | How |
|---|---|---|
| Both rig cameras posed in one sheet frame | tip rays meet to **0.3-0.8 mm**; overhead tilted ~46°; overhead creep followed by `refresh()` without changing the frame | `tools/plate_rig.py` (caches in `evidence/tip/{top,side}_pose_cache.json`, `ugreen4k_lens.json`) |
| Arm model calibration | **1.8 mm median / 4.0 mm p90** over 78 poses (5-fold held out); without approach-side terms nothing got under ~9 mm | `tools/arm_calibrate.py collect` → `fit-arm` → `evidence/tip/arm_model.json` |
| Calibrated vs nominal, live A/B at 60 mm hover | **0.95 mm** vs **1.91 mm** xy median | `tools/arm_calibrate.py aim` |
| Tip-in-camera fit per roll | **1.11 mm median** inferred vs measured over 7 poses around the crossing, at `--roll 1000` | `tools/arm_calibrate.py fit-tip --roll 1000` (commit `41523df`) |
| Anchor repeatability (gate 0, `arm_repeat_20261030T141229`, verdict **GO**) | return-to-START scatter: elbow **0.05 mm**, wrist **0.07 mm**, shoulder **0.63 mm**, stationary drift **0.20 mm**; but pan-to-side **2.61 mm** scatter and a **15.2 mm** gap on the side; a 0.4 s move needs ~1.6 s settle | `docs/history/plan_2026-10-30_arm_calibration.md` §Results; `tools/arm_calibrate.py repeat` / `repeat-report` |
| Held-out tip model | **8.7 mm** (pulses only) → **1.9 mm median / 3.7 mm p90** (pulses + approach side); `fit-arm` **1.8 mm / 4.0 mm** on commanded moves | same |
| Hover in air | **0.1-0.5 mm at 10 mm** above a printed crossing (flat needle, roll 502); first try at 10 mm measured **0.13 mm** xy; reaching 10 mm takes ~40 s with per-goal memory | `tools/arm_calibrate.py hover [--roll R]`, `evidence/tip/arm_hover_20261030T213200.*` |
| Contact measurement accuracy | `dot` contact **0.81 mm** from the crossing — **with no ink mark** | `evidence/tip/arm_hover_20261030T214136.*` |
| Mark scoring | difference image → measured mark offset per roll (`evidence/tip/dot_offsets*.json`), subtracted from the next aim | `new_mark` / `ink_dot` in `arm_calibrate.py` |
| Offline regression | 5 recorded runs replay against current code; "Step 1 … DONE. Regress 5/5" is the only logged result of the AGREED plan | `tools/regress.py`, `evidence/descend/regress_baseline.json` |
| Detection | SAM 3.1 model load **8.7 s**, **0.33-0.75 s** per prompt per 1080p image (RTX 3090) | `docs/history/plan_speedup_2026-10-29.md` §1.2 |
| Time budget of a full touch (old `model_jab --jab` path) | **223 s** total in Touch 1: startup ~47 s (the `pose0` snap alone 15.5 s), placing 117 s (29 re-ties × 4.2 s), arcs ~25 s, descent 25-27 s. A 276 s run broke down as sleeps 100.8 s / SAM 74.4 s / snaps 32.9 s | `docs/history/plan_speed_accuracy_2026-10-30_AGREED.md` baseline table; `plan_speedup_2026-10-29.md` §10 |
| Actuator resolution (why sub-millimetre open-loop moves fail) | pan **2.2-2.95 mm/pulse**, deadband 2, backlash ~5, reversal jump ~20 mm; roll **0.26-0.42 mm/pulse**; elbow lag 11-14 pulses; shoulder rise shortfall ~11 pulses (17-26 at reach) | `docs/history/rca_touch_dot.md` §3, `critique_2026-10-28.md` §3.1, `hiwonder_support_request.md` §1-2 |

## 3. What does not work yet

1. **The ink mark, not the hover, is the failure.** Marks landed **1.3 mm / 2.8 mm / 8.4 mm** from the
   aimed crossing with the flat needle. The needle meets the paper at ~72° from vertical and **skids by a
   varying amount** once pressed (`TOUCH_STEPS_MM` presses to −3 mm commanded). Height-only contact
   stopped above the paper and left no ink; the side-row stall rule fired falsely once. That is why
   `DOT_STOP_AT_CONTACT = False`.
2. **Vertical needle is not commandable** (roll limits above). The user wants a vertical needle;
   roll 1000 (+122°) is steep and was rejected as the answer. At roll 1000 the tip sits ~97 mm off the
   gripper axis and the target crossing needs pan +145 — **forbidden**: the needle can jam into the left
   wall. Keep pan near centre (within ±25 of 497); usable crossings at roll 1000 were around
   `y = 140-155 mm` in the sheet frame.
3. **Wet ink is perishable.** A drop lasts minutes; the user hangs a drop on the needle right before each
   `dot` run (say "go" first, hands at the needle). Any automated ink supply is unbuilt.
4. **No verified pulse↔URDF angle sign for joints 2-4** between `hwr_kinematics.py` and the driver's
   `joint_states`. See [ARCHITECTURE.md §3](ARCHITECTURE.md) — one hardware check resolves it.
5. **`model_jab.py` has no `--calib` flag** even though `arm_model.json` exists; the dashboard runs the
   model-led placer rather than the calibrated path.
6. **The acceptance gate is still undefined in practice.** It moved 1 mm → 2 mm → 1 mm → "≤1.5 mm, then
   1 mm twice in a row, 4 of 5" across the documents while measured marks are 1.3 / 2.8 / 8.4 mm off.
   Until a mark under 1 mm exists, no gate number means anything.
7. **Most of the 2026-10-30/31 evidence is untracked** (294 tracked evidence files vs ~7,216 on disk),
   including the baseline run `evidence/descend/20261030T103024` that several documents quote.
8. **Milestones M2-M5 were never started** (`docs/history/handover.md` milestone table; M1 achieved in
   part), and the drafted Hiwonder support request (`docs/history/hiwonder_support_request.md`, 13
   questions) is still unsent — its serial, firmware and contact fields are blank.

## 4. Next steps, in the order the last handover left them

1. Restore the robot per §1 (roll → 490 slowly, revert both `.bak_roll` files, put the driver back under
   the service).
2. Make a commandable vertical needle: re-seat the gripper on the roll spline so cable-neutral ≈ pulse 250
   and vertical ≈ pulse 1000 (user's call; hand-set-and-hold only survives until the next roll command).
3. `tools/arm_calibrate.py fit-tip --roll <vertical pulse>` (~4 min), check reach of open-paper crossings
   with pan within ±25 of 497, then `tools/arm_calibrate.py dot --roll <vertical pulse>` with fresh ink.
4. Later: more tie poses near the far crossings; `model_jab --calib`; merge PR #1 once the robot is restored.

## 5. Repo/git state

- Current branch **`tipcam-external`** @ `b66b291`, **56 commits ahead of `origin/main`**
  (236 files changed, +78,425 / −31). PR #1 (54 commits) was opened from
  `claude/mlp-servo-tip-mapping-61ee3b` at `d4b27e4`; there is **no CI**.
- `git status` shows ~2,948 untracked paths, nearly all under `evidence/` (run photos, `sam_*.png`,
  `log.json`): the repo tracks **294** evidence files, the working tree holds **~7,216**. Committing the
  2026-10-30/31 runs would make the headline numbers reproducible by anyone but the author. Also
  untracked: `.pi/` and an empty stray root file `NUL` (a Windows `>` redirect artifact — removable, and
  worth adding `.pi/` and `NUL` to `.gitignore`).
- Other local branches: `main`, `the-jab`, `cameras-side-contact-replay`, `plate-well-reach`,
  `well-reach-v2`, `claude/*` worktree branches. `.claude/worktrees/` holds full duplicates of the repo
  for agent runs (1.6 GB) — do not read those copies as if they were this branch. Note they are excluded
  only by `.git/info/exclude` line 7, **not** by `.gitignore`, so the exclusion does not travel with a
  clone; `git status` in a fresh clone would list them.

## 6. Standing operating rules (from the operator)

No folders outside the repo · be brief · **no e-stop, never suggest stopping** · SAM 3.1 for targets ·
no open-loop low poses · watch long runs with a stall watchdog · the roll rules in §1 · never run
`highwonder_rework/deploy/sync.ps1`.

## 7. Housekeeping worth doing

- Fix `README.md`-class drift already found in old docs: DSHOW/MSMF order, the stale ArUco spec in
  `docs/history/plan_speedup_2026-10-29.md`, `tools/regress.py` uses 5 baseline runs where
  `docs/history/handover.md` says 4, and `docs/history/plan.md` still names `tools/needle_touch.py` /
  `tools/servo_shell.py` from the other repo.
- `config/hue.local.json` contains a live Hue app key and clientkey. It is gitignored, but history was
  never verified: run `git log --all -- config/hue.local.json`; if it was ever committed, re-pair
  (`tools/hue.py pair`) and revoke the old username on the bridge.
- `requirements.txt` pins nothing except `numpy<2` and installs SAM from a mutable git URL. Pinning the
  sam3 commit would make runs reproducible.
- **Evidence that is not in git** (see `docs/INDEX.md` §4 for sizes): ~114 untracked run dirs in
  `evidence/descend/` including the most-quoted baseline `20261030T103024`; all of
  `evidence/roll_tri/` except ≈19 files (the 10-28 `spike` log behind the roll/wrist mm-per-pulse numbers
  quoted in history is untracked); everything in `evidence/regress_out/`, `evidence/teach_*`,
  `evidence/dot_touch_20261030*`, `evidence/needle_grasp_20261030/`. `evidence/rollflip/` contains only
  `arm_at_start.jpg` — the 10-31 raw-payload probe results behind `history/hiwonder_support_request.md` §3
  were never saved anywhere. Commit the evidence that a claim in `STATUS.md` or `INDEX.md` depends on.
- **1.6 GB of stale agent worktrees** in `.claude/worktrees/` (8+ full repo copies, each with its own
  evidence tree). They are not part of this branch and inflate every file count measured with a naive
  `find`. Delete after confirming with the tool that created them.
- Bytecode for two interpreters exists in `tools/__pycache__/` (3.12 and 3.14). Use the 3.12 venv.
