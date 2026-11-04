# A1 calibration method (fixed needle)

`tools/a1_calibrate.py` — three commands, run in order. The plate moves between
sessions, so **A1 is never cached**: `locate` re-solves it from live frames every time.

## 0. Scene the solver needs (read this first — live 2026-11-01)

The full `plate_rig.solve` only succeeds when the rig looks like it did on 2026-10-30:

- The standing graph-paper board **behind** the rig must be removed or covered. The low
  side camera otherwise solves the vertical board as the desk sheet (live failure: 848
  side crossings, camera 70 mm up instead of ~140, mat overlap 0.37).
- Overhead: ≥ 150 crossings on the lattice, plate tilt ≤ 2°. If `locate` raises
  "no consistent pose", the camera moved or the plate is lifted — fix the scene, rerun.
- `locate` deliberately does **not** write `top_pose_cache.json` / `side_pose_cache.json`.
  Writing a bad solve into the shared caches would break every other tool (lesson of
  the 2026-10-30 14:47 refit).

## 1. `locate` — where A1 is right now

```
<hackathon venv python> tools/a1_calibrate.py locate --click U V
```

- Grabs both UGREENs (exposure-locked via `rig_io.Camera`), runs the full `plate_rig.solve`
  from the live frames, and chains the overhead plate pose into the sheet frame.
- The A1 pixel (your click; nearest well blob wins) is ray-cast onto the **plate-top plane**
  (not the paper — the plate is ~17 mm thick). Output: `evidence/tip/a1_location.json`
  with `sheet_xy_mm`, `plate_top_mm`, the clicked pixel, and a timestamp.
- Check `evidence/tip/a1_preview.jpg`: red cross = chosen well, green circles = all wells.
  Wrong well chosen = wrong A1 for the whole session; re-click costs seconds.
- Headless fallback (no `--click`): the well blob nearest the camera is taken
  provisionally. Treat it as a guess, not a measurement — confirm on the preview.

## 2. `measure` — the fixed needle, once per seating

```
<hackathon venv python> tools/a1_calibrate.py measure --roll 1000
```

- Thin wrapper over `arm_calibrate.fit_tip`: raises, turns the roll, visits 8 measured
  poses, Gauss-Newton on the camera-frame tip with worst-pose drop > 3.0 mm.
- Writes `tip_by_roll[roll]` into `evidence/tip/arm_model.json`. The needle geometry is
  fixed, so this survives across sessions — rerun **only** after re-seating the needle.
- Only the base is refit per session (anchor at START inside `place`); the tip is reused.

## 3. `place` — closed loop into A1

```
<hackathon venv python> tools/a1_calibrate.py place --roll 1000 [--well-depth 11.0]
```

- Preconditions: `a1_location.json` from this session, a tip for `--roll` in the model.
- Descends like `hover()`: 40→25→15→10 mm over the well mouth with measure-correct
  (gates 0.5 mm xy / 1.0 mm z, 3 tries), then 6→3→2→1→0 mm into the well.
- A1-specific differences from `hover(touch=True)`:
  - goal z is the **well bottom** (plate top minus `--well-depth`, contact-refined),
    not a printed crossing with a skid-bias subtraction — the needle enters the mouth
    vertically, so there is no press-skid to subtract (`DOT_OFFSETS` does not apply).
  - the last 2 mm go **blind** from a pose measured within 1 mm (teal fails against
    white plastic), same as `hover()`.
  - abort if xy error exceeds the well-mouth radius (3.0 mm): retry from 10 mm,
    never force the needle against the plastic.
- Verify on both camera frames before lifting; contact = triangulated z at the well
  bottom plus the side-row stall check inherited from `hover()`.

## Failure map

| Symptom | Cause | Fix |
|---|---|---|
| `no consistent pose` (36 crossings) | stale/grabbed frames, lamps off | rerun (exposure-locked grabs), `auto_lamps` state |
| side camera ~70 mm up, overlap < 0.4 | vertical board solved as sheet | remove/cover the standing graph paper |
| tip not measured over the plate | teal against white plastic | fresh thin ink film; last 2 mm are blind by design |
| xy error > 3 mm inside the well | base drifted / wrong A1 | re-`locate` (plate moved?), re-anchor at START |
| contact never fires | `--well-depth` too deep | reduce depth, contact refines it every place |
