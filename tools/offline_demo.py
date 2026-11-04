"""The colony decision without the robot: saved dish photos and their saved SAM 3.1 detections, then the real Muse Spark tool-calling loop.
It needs no GPU, robot or camera. With MODEL_API_KEY (from dev.meta.ai) it calls Muse Spark live; without a key it replays a recorded run of the same loop.
usage: python tools/offline_demo.py [PLATE 1-20] [--replay] [--n 3]
Writes evidence/offline_result_<PLATE>.jpg: SAM's marks grey, the ones the critic struck with a red cross, its choices yellow, numbered in picking order."""
import json
import os
import sys
import time

import cv2

sys.path.insert(0, 'tools')
import colony_agent as ca  # noqa: E402

E = 'evidence'
sys.stdout.reconfigure(encoding='utf-8', errors='replace')  # a model's reason can hold any character
ca.S = E
plate = next((a for a in sys.argv[1:] if a.isdigit() and sys.argv[sys.argv.index(a) - 1] not in ('--n', '--pace')), '1')
n = int(sys.argv[sys.argv.index('--n') + 1]) if '--n' in sys.argv else 3
PACE = float(sys.argv[sys.argv.index('--pace') + 1]) if '--pace' in sys.argv else 0
TAPE = f'{E}/offline_plate_{plate}_replay.json'
frame = cv2.imread(f'{E}/offline_plate_{plate}.jpg')
variants = json.load(open(f'{E}/offline_plate_{plate}.json'))
scale = variants.pop('scale', None)  # phone photos: their own mm scale. Dish size is estimated from the plate type
if scale:  # shortcut: on a rectangular tray the rim check is a circle past the corners, marks by the edge were dropped when the plate was made
    ca.PX_MM, ca.AGAR_R = scale['px_mm'], scale['agar_r_mm']
saved_sam = json.load(open(f'{E}/offline_plate_{plate}_sam.json'))  # real SAM 3.1 output per re-run prompt, so no GPU is needed
live = '--replay' not in sys.argv and (os.environ.get('MODEL_API_KEY') or ca.KEY_FILE.exists())
print(f'Plate {plate}: SAM 3.1 found ' + ', '.join(f'{k} {len(v)}' for k, v in variants.items()) + ' (saved detections)')
print(f'Muse Spark: {"live, " + ca.MODEL if live else "REPLAY of a recorded run, no API call (set MODEL_API_KEY for a live one)"}')

if live:
    tape = []

    def call(body):
        tape.append(ca.post(body))
        return tape[-1]
else:
    replies = iter(json.load(open(TAPE)))

    def call(body):
        time.sleep(PACE)  # the demo page plays a recorded run at reading speed
        return next(replies)



def sam(prompt):
    """SAM 3.1 for a re-run: what was saved for this prompt, else real SAM if --gpu-sam (how the recorded runs were made)."""
    if prompt in saved_sam:
        return saved_sam[prompt]
    if SAM_MODEL is None:
        raise RuntimeError(f'no GPU in this demo, saved prompts are {sorted(saved_sam)}')
    h, w = frame.shape[:2]
    T = int(30 * ca.PX_MM)  # 30 mm tiles at about 1000 px, as the rig's zoom
    zoom = 1000 / T
    found = []
    for ty in range(0, max(h - T, 0) + int(T * .8), int(T * .8)):
        for tx in range(0, max(w - T, 0) + int(T * .8), int(T * .8)):
            tile = frame[ty:ty + T, tx:tx + T]
            if min(tile.shape[:2]) > 40:
                found += cd._hits(SAM_MODEL, cv2.resize(tile, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_CUBIC), prompt, 0.05, zoom, tx, ty, 'small')
    kept = []
    for f in sorted(found, key=lambda f: -f['score']):
        u, v = f['centroid']
        d = 2 * (f['area'] / 3.1416) ** .5 / ca.PX_MM
        if d <= 6 and all((u - q['px'][0]) ** 2 + (v - q['px'][1]) ** 2 >= (0.7 * ca.PX_MM) ** 2 for q in kept):
            kept.append({'px': [round(u, 1), round(v, 1)], 'd_mm': round(d, 1), 'score': round(float(f['score']), 2)})
    saved_sam[prompt] = kept
    json.dump(saved_sam, open(f'{E}/offline_plate_{plate}_sam.json', 'w'))
    return kept


SAM_MODEL = None
if '--gpu-sam' in sys.argv:
    import colony_detect as cd
    import sam_detect
    SAM_MODEL = sam_detect.load()
try:
    chosen = ca.decide(frame, variants, [], (frame.shape[1] / 2, frame.shape[0] / 2), n, call=call, sam=sam)
except ca.Refused as e:
    print(f'REFUSED: {e}')
    chosen = []
if '--record' in sys.argv:  # how the replay files were made
    json.dump(tape, open(TAPE, 'w'), indent=1)

Z = 1150 / max(frame.shape[:2])
R = max(8, round(7 * Z))  # a ring around a colony, whatever the photo's scale
out = cv2.resize(frame, None, fx=Z, fy=Z, interpolation=cv2.INTER_LANCZOS4)
for c in variants['loose']:
    cv2.circle(out, (int(c['px'][0] * Z), int(c['px'][1] * Z)), R, (170, 170, 170), 1, cv2.LINE_AA)
for c in json.load(open(f'{E}/colony_agent.json'))['rejected']:  # the critic struck these
    u, v = int(c['px'][0] * Z), int(c['px'][1] * Z)
    cv2.line(out, (u - R, v - R), (u + R, v + R), (60, 60, 255), 2, cv2.LINE_AA)
    cv2.line(out, (u - R, v + R), (u + R, v - R), (60, 60, 255), 2, cv2.LINE_AA)
for k, p in enumerate(chosen, 1):
    u, v = int(p[0] * Z), int(p[1] * Z)
    cv2.circle(out, (u, v), R * 2, (0, 255, 255), 3, cv2.LINE_AA)
    for col, th in (((0, 0, 0), 6), ((0, 255, 255), 2)):
        cv2.putText(out, str(k), (u + R * 2, v - R), cv2.FONT_HERSHEY_SIMPLEX, 1.2, col, th, cv2.LINE_AA)
path = f'{E}/offline_result_{plate}.jpg'
cv2.imwrite(path, out, [cv2.IMWRITE_JPEG_QUALITY, 92])
print(f'Picture: {path}\nOn the robot, chosen SAM positions go to the pick code; here nothing moves.')
