# Offline demo guide

See Muse Spark judge SAM 3.1's work on real plates in about a minute. You need Python 3.9 or newer, and no robot, GPU or camera. Nothing moves.

## Start

**Windows:** double-click `start_offline_demo.bat`. It installs `opencv-python` and `numpy` if they're missing, then opens the page. Close its window to stop.

**Other systems:**

```bash
pip install opencv-python numpy
python tools/offline_page.py    # then open http://127.0.0.1:8766
```

If the page looks out of date, close every copy of the demo and start it again.

## Use it

1. Click a plate. You see SAM 3.1's marks as grey rings, and nothing runs yet.
2. Choose **Recorded run** or **Live Muse Spark**.
3. Press **Start** and follow the dish and the log.

| On the dish | Meaning |
|---|---|
| Grey ring | A mark from SAM 3.1 |
| Blue ring | Muse Spark is looking at this mark |
| Red cross | Muse Spark struck it: not one clean colony |
| Yellow ring and well | Chosen, in picking order, with its target well |
| **REFUSED** | Nothing on this plate may be picked |

The log shows every tool call and reason, including the SAM prompt Spark words for that plate.

## Good plates to try

| Plate | What it shows |
|---|---|
| 1 or 14 | Large colonies: Spark strikes merged and misshapen ones |
| 3 | A plate half in shadow |
| 4 | A lawn: refused outright, though SAM marked it |
| 9 to 12 | Crowded plates: refused |
| 17 | Phone photo with dense patches and glare |
| 18 | Phone photo where Spark's own SAM prompt finds 10 new marks |

Plates 1 to 14 come from the rig's overhead camera, and some show the same plate in different light. Plates 15 to 20 are phone photos of other plates, and their sizes are estimates.

## Recorded or live

- **Recorded run** needs no key. It replays a real Muse Spark run at reading speed, against saved SAM 3.1 results.
- **Live Muse Spark** calls the Meta Model API, so the choices can differ from run to run. Get a key at [dev.meta.ai](https://dev.meta.ai), then either set `MODEL_API_KEY` before starting, or put `{"key": "YOUR_KEY"}` in `config/meta.local.json`, which git ignores.

A live run sends crops of the dish photo to Meta's API. With no GPU here, SAM can only re-run prompts that were saved with the plate. If Spark words a new prompt, the tool tells it so and the run carries on. A crowded plate can occasionally run out of turns on a live run; press **Start** again.

## How it works

`tools/colony_agent.py` is the loop, and the same code sits behind **Run: Muse Spark chooses** on the robot's page. Spark can call these tools:

| Tool | What it does |
|---|---|
| `view_dish` | Look at the whole plate |
| `refuse_plate` | Stop: nothing here may be picked |
| `list_colonies` | Read SAM's marks: size, score, distance to the rim and neighbours |
| `view_colonies` | See zoomed, numbered crops |
| `reject_colonies` | Strike marks |
| `rerun_sam` | Send SAM back with its own prompt, at most 3 times |
| `choose_colonies` | Submit its picks and reasons |

The code holds the hard limits: Spark must see the dish first, can only choose colonies it has looked at, can't choose near the rim or next to a neighbour, and can't pass the re-run cap. It returns numbers only, so it never handles coordinates or motion.

`python tools/test_colony_agent.py` checks those rules with a scripted model, without a key.
