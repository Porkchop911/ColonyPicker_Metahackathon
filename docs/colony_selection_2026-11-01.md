# Colony selection on a live plate, 2026-11-01

Scene: a petri dish with ~7 colonies on the black mat in front of the grid sheet, UGREEN view
(`evidence/20261101_plate_ugreen.jpg`). No arm motion in this step.

## SAM 3.1 detection (`tools/sam_detect.py`, worktree venv from the handover)

Run on a 3x crop of the dish (UGREEN px x 1040..1410, y 700..980), threshold 0.2:

| Prompt | Whole 1080p frame | Dish crop |
|---|---|---|
| "bacterial colony", "colony", "bacteria", "small bright spot" | 0 | 0 |
| "white spot", "small white dot" | 0 | **7** |

Use **"white spot"** on a dish crop. The whole frame finds nothing: colonies are ~10 px there.
Overlay: `evidence/20261101_plate_sam_white_spot.jpg`.

| # | score | crop centroid | UGREEN px (x, y) | area (crop px) |
|---|---|---|---|---|
| 0 | 0.75 | (759, 465) | **(1293, 855)** | 1379 |
| 1 | 0.72 | (389, 337) | (1170, 812) | 716 |
| 2 | 0.71 | (277, 450) | (1132, 850) | 537 |
| 3 | 0.68 | (532, 418) | (1217, 839) | 627 |
| 4 | 0.64 | (433, 506) | (1184, 869) | 351 |
| 5 | 0.63 | (749, 358) | (1290, 819) | 601 |
| 6 | 0.62 | (499, 315) | (1206, 805) | 359 |

## Pick

Target: **#0, UGREEN (1293, 855)**: highest score, largest, isolated, clear of the glare streaks.
It is the same colony chosen by eye before running SAM.

## Not done: the needle

UGREEN pixels only; no sheet-mm. The dish is on the black mat, off the calibrated grid sheet, and
`coarse_aim` / `model_jab` are calibrated to the sheet plane. Touching needs the dish-surface plane
and its height first.
