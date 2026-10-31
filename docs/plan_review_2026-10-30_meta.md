# Review of `plan_speed_accuracy_2026-10-30_OPUS.md` — meta, 2026-10-30

Reviewed against `plan_speed_accuracy_2026-10-30_meta.md`, the code in the main checkout, and the
baseline log `evidence/descend/20261030T103024/log.json`. Verdict: **adopt the Opus plan as the base**,
with the amendments below. Its diagnosis is deeper (D1–D4 before code), and every factual claim I
checked holds. Where we differ it is mostly sequencing, plus one correction to my plan and one to its.

## Verified facts (both plans' common ground, re-checked)

- Baseline decomposition agrees: 223 s; placing round 2 = 81 s / 20 iterations at ~4.1 s
  (retie: settle 1.6 + SAM 1.75 + snap 0.5 + grid 0.2); startup ≈ 47 s; descent 25–27 s; side pose
  failed twice ("only 0/4 dots agree"); mark 3.6 mm = ~2.0 arc extrapolation + ~1.5 mark-vs-tip +
  0.9 sideways.
- Opus additions I verified in the log/code: `pose0` wait `snap 15.48 s` (arm camera stall is real and
  the single biggest startup item); 9 of 30 reties matched 0 dots (carry), 4 matched 1 — so ~43% of
  placing moves were driven by stale or near-stale ties; round-2 flips (−12.6/+9.9 mm after 1–2 pulse
  commands, e.g. it 8→9: +1/+1 pulses, along 2.3 → −12.6) cannot be motion. `settle(..., 1500, 2.3)`
  at `tools/model_jab.py:986` confirmed (minimum 1.5 s even for a 1-pulse move).
- **Correction to my plan:** I attributed round 2 to a "no convergence owner" logic gap. The
  oscillation is primarily *measurement* (tie aliases/carries), and my `place-skip` only helps once
  readings are trustworthy. S1a/S1b address the cause; `place-skip` stays as the cheap early exit.
  My Phase 2 ordering (settle speedups first) is reordered accordingly — S1a before settle work.

## Agreements (adopt as-is)

| Item | Note |
|---|---|
| Targets ≤120 s; ≤1.5 mm then the 1 mm gate (2 in a row, 4 of 5) | I had ≤1 mm straight away; the staged gate is better |
| Regress fix first (per-day `ugreen_dots.json`), then 4/4 gate before any live change | Same fix I identified (`tools/replay_run.py:54`); note I measured the symptom as 1/4 OK (152609 passes), Opus says 0/4 — either way it blocks everything |
| D1–D4 diagnostics before choosing S1a's rule | The grid-slip vs tie-alias distinction decides the fix; my plan guessed |
| S2 settle-by-move-size | Supersedes my "condition-based settle": `settle()` already early-outs on readback (`model_jab.py:389`); sizing `ms` by jump is smaller and safer. Keep 1500 when servo 2 rises |
| S6/A4/A3/A5: 5th dot, tri-height contact, ink last 2–3 mm, fresh dot each touch | Identical to my Phase 1 |
| A2 mark-bias via new `tools/mark_bias.py` (D2) over all historical marks | Better than my "tune `--along` from 2 touches": gives mean/std and refits a constant. My Phase 1.4 measurement becomes D2's live continuation |
| S4 descent step sizing (6/3/1) | Small, safe, I missed it |

## Amendments (meta → Opus)

1. **S1b risk is understated.** "Overhead places along for rnd ≥ 1" depends on D3 *and* on the teal
   tip being findable from hover height every iteration — Touch 1 lost it repeatedly (`teal_tip`
   took keyboard-box art; the `near=`/`on_paper` guards are unproven live). Keep S1b **behind** S1a,
   not parallel: if S1a's reject-and-median kills the flips, S1b's remaining value is ~2.8 s/iter on
   ~6 iterations ≈ 17 s — take it only if the ≤120 s budget needs it after steps 3–6 are measured.
2. **Step 5/6 order.** Opus does A1 (arc to 2.0 mm + quadratic fit) before live touch 2. Cheaper:
   run touch 2 with `--along 3.5` *first* (no code change, already verified in-tree) to bank a second
   mark and D2's most relevant data point; do A1 after, informed by D1/D2. If `--along` overshoots,
   that measurement is exactly what A2 needs.
3. **`place-skip` interaction with S1a.** With median-of-3 measuring (S1a), a "skip" decision must use
   the median, not a single retie, or we skip on a carry again. One line in S1a's implementation.
4. **Add `20261030T103024` to `tools/regress.py` RUNS** (my plan). It is the only recording exercising
   the `side_teal` guards and `place-skip`; after the dots fix it should replay (SAM-dependent, so
   divergence row is the criterion, not a clean run — same standard as the other four).
5. **A1's stop rule:** "stop also on side tri-height ≤ 1.5 mm when available" — note tri-height needs
   the 5th dot (step 4) *and* a working side pose at arc height; make A1 degrade gracefully to the
   arm height alone, as the descent loop already does (`model_jab.py:1133`).
6. **Writer discipline:** Opus states "one writer on `model_jab.py`" — agreed, and it applies now:
   the file currently has uncommitted changes from both today's sessions. Before step 3, one person
   commits (or we agree the diff, currently: teal guards + `CONTACT_EXTRA` 6 + side guards +
   place-skip) so replays have a stable base. Nothing is committed until the user says so.

## Merged course of action

| Step | Do | Owner | Pass gate |
|---|---|---|---|
| 1 | Regress dots fix; add run 103024 to RUNS; re-baseline | either (one writer) | 5/5, no crashes |
| 2 | D1 (flip diagnosis), D2 (`mark_bias.py`), D3 (parallax number), D4 (pose0 stall) | offline, parallelisable | four tables/numbers written into the plan |
| 3 | S1a reject-impossible-reties (+ median feeds place-skip) + S2 settle sizing | one writer | replay 103024: iteration count drops, regress 5/5 |
| 4 | 5th dot on sheet (physical) | user | `side_aim.py` ≥ 5 dots agree |
| 5 | Live touch 2, centre dot, `--along 3.5`, thin ink, mark measured via D2's tool | user + driver | time, mark, contact rule logged in handover |
| 6 | A1 lower/curved practice arc (+ A2 bias if D2 std < 0.7 mm) | one writer | regress 5/5; prediction vs tip end < 0.7 mm live |
| 7 | S1b only if budget requires; S4, S5 (per D4) | one writer | regress 5/5 |
| 8 | Live touches 3–5 | user + driver | ≤120 s, ≤1.5 mm; then 1 mm gate (2 in a row, 4 of 5) |

Differences from Opus order: touch 2 (its step 6) moves before A1 (its step 5); S1b demoted to
conditional. Differences from meta order: diagnostics (D1–D4) precede everything, S1a precedes settle
work, staged accuracy gate.

## Open questions

- For Opus: does D1 distinguish grid slip vs tie alias from the *saved* frames alone, or does it need
  the arm frames re-SAM'd with today's prompts (the "dot" fallback changed detection on 2026-10-30)?
  If the latter, D1's table should record which prompt found the dots.
- For the user: who is the one writer for steps 1/3/6, and may that writer commit after step 1 so the
  replay baseline has a stable base?
