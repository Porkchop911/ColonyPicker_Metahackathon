# Goal: needle into the wells of the 96-well plate (2026-10-31)

Agreed with the user before any work started (requirements interview, this session).

## Why

Picked colonies have to be put down somewhere. The wells of the 96-well plate are those destinations, so the needle
must reach as many wells as possible, accurately, before colony picking itself matters.

## Requirements (the user's answers)

| # | Requirement |
|---|---|
| 1 | Target: the 96-well plate (8 x 12, 9 mm pitch), lying where it is now; it is not moved |
| 2 | Accuracy gate: 1 mm |
| 3 | Output: the list of wells the needle can reach |
| 4 | Needle orientation: Claude's choice (the roll servo may be used) |
| 5 | Claude drives the robot unattended; the mecanum base may move at most 40 mm in any direction, and each base move is reversed before the next |
| 6 | Work on a clearly named branch; no more cryptic worktrees |
| 7 | Merge colony detection: colonies are the ultimate goal |
| 8 | Wells first (as destinations); colony targets come later, once accuracy is there |
| 9 | "Reached" = the tip goes into the well, below the rim; touching the wall is fine |
| 10 | Deliver the list of wells the needle gets into, not just hovers over |
| 11 | The base move is optional, only if the arm alone cannot reach |
| 12 | The pan limit comes from what the cameras see, not a fixed rule |
| 13 | The plate stays put |
| 14 | Check: the cameras show the tip inside the well it was aimed at (wells ~5 mm wide, ~10 mm deep) |
| 15-16 | Any needle pose; well entries may run unattended |
| 17 | Deliverables: a `well <id>` command, `well_reach.json` (per well: reached or not), a handover table |
| 18 | Empty plate, bare needle (no ink); every reached well gets a photo as proof |
| 19 | Live-test every well the model says is reachable, outer ring first |
| 20 | Roll commands allowed |

## What done looks like

- `arm_calibrate.py well --wells B7 ...` puts the tip into each named well, measured, and photographs it.
- `evidence/tip/well_reach.json` lists all 96 wells: reached (with photo) or not (with the reason).
- The motion is calm and predictable throughout, as on 2026-10-30 (0.13 mm hover over a printed crossing).
