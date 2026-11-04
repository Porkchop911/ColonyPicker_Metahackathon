# ColonyPicker

A robot arm with a steel needle picks microbial colonies from an agar dish, on its own. **Meta SAM 3.1** finds the colonies on a live
overhead camera image. The arm then moves the needle into a colony, into its own sample well, into a disinfectant well and into a
rinse well, while a browser page shows each step. Meta models are the only AI models it calls at runtime: SAM 3.1 marks the colonies, and Muse Spark can act as its critic.

Picking colonies is repetitive hand work in the lab. This project puts Meta's models in charge of seeing and judging the plate, so a robot can take that work over.

**Try it without the robot:** double-click `start_offline_demo.bat`, pick a plate and press **Start**. See the [offline demo guide](docs/offline_demo_guide.md).

> Agents and new contributors: read `CLAUDE.md` first. The arm is real, and its safety rules there are not optional.

## A hardware project

This is real robotics, so the full demo depends on physical hardware: this robot arm, its needle, two fixed cameras, the lamps and the exact position of the plate.
Its calibration belongs to that one setup and can't be reproduced remotely. Without the rig, you can run the [offline demo](docs/offline_demo_guide.md), SAM 3.1 detection on a dish photo, and the offline checks below.
The working system is shown in the demo video.

## What it does

1. **Detect.** The 4K overhead photo of the dish goes to SAM 3.1 with a text prompt. Three detection settings appear on the page, and the operator picks one.
2. **Pick.** Two fixed cameras watch the needle tip. The arm aligns the tip over the chosen colony, then goes straight down into it.
3. **Transfer.** The needle enters that colony's sample well (C10, C11, C12), then A12 to disinfect for 5 s, then A10 to rinse for 2 s.
4. **Repeat** for the next colony. A failed pick lifts the needle and retries up to 3 times.

| Meta model | Where it runs | What it does |
|---|---|---|
| SAM 3.1 (`facebookresearch/sam3`, `facebook/sam3.1` weights) | Locally on the PC GPU (RTX 3090), kept loaded by `tools/sam_server.py` | Finds the dish and each colony, with a confidence score |
| Muse Spark (`muse-spark-1.3`, Meta Model API) | Over the API, from `tools/colony_agent.py` | Critic of SAM in a tool-calling loop: refuses a lawn or unusable plate, strikes marks that are not one clean colony, chooses what the arm picks |

SAM finds where things are, to the pixel, but it also rings glints, smears and bumps in a lawn. Muse Spark looks at the whole plate and judges what those marks are. It never gets coordinates or motion: it returns mark numbers, and the arm goes to SAM's position for each. The code rejects a choice too close to the rim or a neighbour, one it hasn't looked at, or one it struck, and the model must choose again. Spark also tunes SAM: it words its own prompt for what the colonies on a plate look like and sends SAM back, at most 3 times, a cap the code enforces.

On 14 dish views from the rig, judged by eye, it picked on the 5 clean ones and refused the 9 others, where a plain "3 largest marks" rule would have gone into 7 of them. Some views show the same plate in different light.

On the page this is the **Run: Muse Spark chooses** button. A refusal ends the run with no pick; if the API fails, the operator chooses. The filmed run uses the operator's selection; the critic runs on saved plates in the offline demo.

Everything else is classic code, with no other AI model: OpenCV image processing, NumPy, arm kinematics and a Hue lamp API.
The tip tracking is image differencing, not a learned model.

## Results

| | |
|---|---|
| Full loop from the page | 3 colonies -> 3 sample wells -> disinfect -> rinse, about 10 minutes |
| Wells | 5 used wells entered 3 of 3 shuffled passes, checked by photo |
| SAM | Dish found with score 0.93-0.98. On dense plates, 113-181 colonies ringed per dish, not every colony |
| Critic | 20 saved plates: refused the 9 lawn, hazy or crowded rig views, and chose colonies on the other 11, tuning SAM's prompt on each |

**Known limits.** The last drop still scatters by about 1-2 mm, so colonies of 3-4 mm are hit reliably and 2 mm colonies give a mix of hits, edge hits and misses.
Well positions are calibrated to one physical setup and must be redone if the plate, robot or cameras move. The arm reaches columns 7-12 of the plate from this position. Details:
[docs/colony_demo_writeup_2026-11-04.md](docs/colony_demo_writeup_2026-11-04.md).

## Run it

### Offline demo, no robot

This runs the Muse Spark critic on 20 saved plates with their saved SAM 3.1 detections: 14 dish views from the rig and 6 phone photos of other plates, whose mm scale is estimated from the plate type. It needs no GPU, robot or camera.

On Windows, double-click `start_offline_demo.bat`. It opens the demo page without the robot. Pick a dish photo and press **Start**, and the critic runs in front of you, as a recorded run or live with a key. The [guide](docs/offline_demo_guide.md) has the details. Elsewhere, or for the terminal version:

```bash
pip install opencv-python numpy
python tools/offline_page.py                          # the page
python tools/offline_demo.py 1 --replay               # a recorded run, no key needed: 1 clean plate, 4 lawn, 1-20 all work
MODEL_API_KEY=... python tools/offline_demo.py 1      # the same loop live, key from dev.meta.ai
```

It prints each tool call and reason, and writes `evidence/offline_result_1.jpg`: grey rings for SAM's marks, red crosses for marks the critic struck, yellow numbers for its picks in order. Live runs can choose differently.

### SAM 3.1 only, no robot

This needs a GPU and access to the gated `facebook/sam3.1` weights on Hugging Face, and works on any photo of a dish.

```bash
uv venv .venv --python 3.12
uv pip install --python .venv/Scripts/python.exe torch torchvision --index-url https://download.pytorch.org/whl/cu128
uv pip install --python .venv/Scripts/python.exe -r requirements.txt
hf auth login
.venv/Scripts/python tools/colony_detect.py YOUR_DISH_PHOTO.jpg --out overlay.jpg
```

### Offline checks, no robot

There is no full physics or visual simulation of the rig. These run without hardware:

```bash
python tools/aim_sim.py --runs 5                       # simulated servos with backlash, for an earlier placing method
python tools/well_tour.py A12 C10 A10 --blind --dry    # prints the joint commands of a well tour, nothing moves
```

### Full demo, with the rig

This needs the hardware below, calibrated: a Hiwonder ROSOrin Pro with a needle on its arm, a 4K overhead and a 1080p side webcam, two Philips Hue lamps,
a 96-well plate and a 55 mm dish. Stored poses only fit the original setup, so another rig must be recalibrated first. [docs/RIG.md](docs/RIG.md) has the hardware and environment setup,
and [docs/new_layout_2026-11-04.md](docs/new_layout_2026-11-04.md) the calibration.

```bash
# set HUE_BRIDGE_IP and ROBOT_IP to your own addresses first
python tools/hue.py pair                                   # press the Hue bridge link button first
scp robot/servo_bridge.py ubuntu@ROBOT_IP:colony/servo_bridge.py
.venv/Scripts/python -u tools/sam_server.py                # keeps SAM 3.1 loaded
start_demo.bat                                             # opens http://127.0.0.1:8765
```

On the page: enter the agar height, press **Dry run**, then **Run demo**, and choose a detection setting. **Record on / off** films the page and cameras.
Edit `start_demo.bat` to point at your own Python. Read the safety rules in `CLAUDE.md` before moving the arm.

## Repo map

| Path | What |
|---|---|
| `tools/colony_detect.py`, `tools/sam_detect.py`, `tools/sam_server.py` | SAM 3.1 detection, and the process that keeps it loaded |
| `tools/colony_cycle.py`, `tools/colony_pick.py`, `tools/well_tour.py` | The full cycle, one pick, and the well moves |
| `tools/demo_server.py`, `tools/demo_page.html` | The demo page |
| `tools/colony_agent.py`, `tools/test_colony_agent.py` | The Muse Spark critic loop and its offline test |
| `tools/offline_page.py`, `tools/offline_demo.py`, `evidence/offline_plate_*` | The offline demo and its saved plates |
| `tools/plate_rig.py`, `tools/a1lib.py`, `tools/well_reach.py`, `tools/rig_io.py` | Camera and plate geometry, arm model, safe moves, robot and camera I/O |
| `robot/` | The bridge that runs on the robot, and the arm URDF |
| `docs/` | Write-ups, newest dated file first, and the [offline demo guide](docs/offline_demo_guide.md). Older files are history |
| `evidence/tip/` | Calibration and state files for this rig |

## Pre-existing vs. built during the event

**Hackathon problem statement:** Build for Your World.

**Pre-existing (not in this repo):** the robot's own control stack — ROS 2 servo/motor driver, camera
driver, HTTP gateway (`/drive`), environment script. Also an earlier needle-touch prototype and a
colony-picker step-test plan (2026-10-24); their lessons are cited in comments, their code is not here.
Vendored unchanged: the arm URDF (`robot/hwr.urdf`) and its FK/IK (`tools/hwr_kinematics.py`).

**Built during the event (everything else):** the Muse Spark critic loop (`tools/colony_agent.py`), the offline demo (`tools/offline_page.py`), camera calibration on the 5 mm ruling, two-camera and
robot-motion triangulation of the needle tip, closed-loop strafing, Hue lighting control, SAM 3.1
detection, the descent/dotting pipeline, the PC↔robot JSON-lines bridge (`robot/servo_bridge.py` +
`tools/rig_io.py`), the arm-model calibration (`tools/arm_calibrate.py`), the demo dashboard, the
offline regression harness, and `docs/` + `evidence/`.

**Open-source dependencies:** SAM 3.1 (Meta, `facebookresearch/sam3`, gated `facebook/sam3.1` weights),
PyTorch, OpenCV, NumPy, Philips Hue local API.
