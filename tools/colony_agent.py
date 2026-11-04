"""Muse Spark is the critic of SAM 3.1, in a tool-calling loop. SAM marks colonies; Spark judges the whole plate and refuses a lawn outright, strikes marks that are not
one clean colony, can send SAM back with another prompt (at most 3 times), and chooses what the arm picks from what survives. A choice that breaks the rules goes back to it as a tool error. It never gets coordinates or motion: colony_cycle.py
picks the SAM positions of the numbers it chose. Key: MODEL_API_KEY, else {"key": "..."} in config/meta.local.json (kept out of git).
usage: python tools/colony_agent.py [N]   decides on the saved st2_top.jpg and colony_variants.json, no arm, writes evidence/tip/colony_agent.json"""
import base64
import json
import os
import time
import urllib.error
import urllib.request
from pathlib import Path

import cv2
import numpy as np

S = 'evidence/tip'
URL = 'https://api.meta.ai/v1/chat/completions'
MODEL = 'muse-spark-1.3'  # not the -contributor one: that lets Meta train on these prompts
KEY_FILE = Path(__file__).resolve().parent.parent / 'config' / 'meta.local.json'
if not KEY_FILE.exists() and '.claude' in KEY_FILE.parts:  # a .claude/worktrees/<name> checkout: the main checkout's key
    KEY_FILE = Path(*KEY_FILE.parts[:KEY_FILE.parts.index('.claude')]) / 'config' / 'meta.local.json'
PX_MM = 6.0  # overhead px per mm at the dish
AGAR_R = 27.5  # mm, the 55 mm dish
RIM_MM, GAP_MM = 4.0, 0.5  # the tip reading breaks near the rim; a touching neighbour stops a pick
MAX_TURNS, MAX_VIEW = 14, 12
MAX_RERUNS = 3  # hard cap, counted here and not left to the model
SAM_PROMPTS = ('white speck', 'round white spot', 'bubble')  # the ones that found colonies here; 'colony' finds nothing
PROMPT = """You are the critic of SAM 3.1 for a robot arm with a needle. SAM marks colonies on an agar dish, and it also marks things that are not colonies. It cannot tell a
growing lawn from colonies; you can. Your judgement decides what the arm may touch.
1. Call view_dish and judge the whole plate first. Separate colonies are distinct round domes with clean agar between them. If the surface is a lawn or confluent film, a
   hazy or mottled sheet of growth, or colonies so crowded that the agar between them is mostly hidden, or the plate shows no clear colonies, call refuse_plate at once with the
   reason. Do not look for the best of a bad plate: a bump in a lawn is not a colony and a pick from it is not a pure culture. Refuse when in doubt.
2. Otherwise call list_colonies and inspect candidates with view_colonies. Use reject_colonies for every mark that is not one clean colony: glints, rim reflections, scratches,
   smears, bubbles, two merged colonies, bumps in haze. Say why.
   If SAM missed clear colonies or its marks are poor, call rerun_sam with another prompt: new marks join the list with new numbers. You have at most {reruns} re-runs.
3. Call choose_colonies with exactly {n} surviving colonies in picking order, each with a one-sentence reason. Every one must have been viewed. The needle lands within about 1-2 mm,
   so size matters most: largest first, under 2 mm only if nothing larger survives. If fewer than {n} survive, call refuse_plate instead."""
TOOLS = [{'type': 'function', 'function': f} for f in (
    {'name': 'view_dish', 'description': 'Photo of the whole dish, without marks.', 'parameters': {'type': 'object', 'properties': {}}},
    {'name': 'list_colonies', 'description': 'SAM 3.1 marks: number, diameter mm, SAM score, mm to the rim, mm to the nearest neighbour, pickable. strict has the fewest, loose all. Rejected marks are left out.',
     'parameters': {'type': 'object', 'properties': {'setting': {'type': 'string', 'enum': ['strict', 'normal', 'loose']}}, 'required': ['setting']}},
    {'name': 'view_colonies', 'description': f'Zoomed photo of these marks with numbered rings, at most {MAX_VIEW}.',
     'parameters': {'type': 'object', 'properties': {'numbers': {'type': 'array', 'items': {'type': 'integer'}}}, 'required': ['numbers']}},
    {'name': 'reject_colonies', 'description': 'Strike marks that are not one clean colony. They can no longer be chosen.',
     'parameters': {'type': 'object', 'required': ['rejects'], 'properties': {'rejects': {'type': 'array', 'items': {
         'type': 'object', 'properties': {'number': {'type': 'integer'}, 'reason': {'type': 'string'}}, 'required': ['number', 'reason']}}}}},
    {'name': 'rerun_sam', 'description': f'Run SAM 3.1 again on this photo with another text prompt, at most {MAX_RERUNS} times. New marks are added; marks you struck stay struck.',
     'parameters': {'type': 'object', 'required': ['prompt'], 'properties': {'prompt': {'type': 'string', 'enum': list(SAM_PROMPTS)},
                                                                             'min_score': {'type': 'number', 'description': 'Keep marks at or above this SAM score, 0.05-0.9. Default 0.3.'}}}},
    {'name': 'choose_colonies', 'description': 'Submit the final choice, in picking order.',
     'parameters': {'type': 'object', 'required': ['picks'], 'properties': {'picks': {'type': 'array', 'items': {
         'type': 'object', 'properties': {'number': {'type': 'integer'}, 'reason': {'type': 'string'}}, 'required': ['number', 'reason']}}}}},
    {'name': 'refuse_plate', 'description': 'Refuse the whole plate: nothing on it may be picked.',
     'parameters': {'type': 'object', 'properties': {'reason': {'type': 'string'}}, 'required': ['reason']}})]


class Refused(Exception):
    """The critic refused the plate: no pick, and no operator fallback."""


def post(body):
    key = os.environ.get('MODEL_API_KEY') or json.loads(KEY_FILE.read_text())['key']
    req = urllib.request.Request(URL, json.dumps(body).encode(), {'Authorization': f'Bearer {key}', 'Content-Type': 'application/json'})
    try:
        with urllib.request.urlopen(req, timeout=120) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        raise RuntimeError(f'Meta API {e.code}: {e.read()[:300].decode(errors="replace")}') from None


def candidates(variants, taken, dish_c):
    """The loose list (it holds the other two), numbered, without colonies picked or skipped, with what the rules need."""
    tight = lambda c, name: any(c['px'] == q['px'] for q in variants[name])
    all_ = [dict(c, setting=next(s for s in ('strict', 'normal', 'loose') if tight(c, s))) for c in variants['loose']]
    for c in all_:
        p = np.array(c['px'])
        c['rim_mm'] = round(AGAR_R - np.linalg.norm(p - dish_c) / PX_MM, 1)
        others = [np.linalg.norm(p - np.array(q['px'])) / PX_MM - (c['d_mm'] + q['d_mm']) / 2 for q in all_ if q is not c]
        c['gap_mm'] = round(float(min(others, default=99)), 1)
        c['pickable'] = bool(c['rim_mm'] >= RIM_MM and c['gap_mm'] >= GAP_MM)
    free = [c for c in all_ if not any(np.linalg.norm(np.array(c['px']) - np.array(q)) < 15 for q in taken)]
    free.sort(key=lambda c: -c['score'])
    return {i + 1: c for i, c in enumerate(free)}


def view(frame, cs):
    """Numbered rings on a crop of the 4K frame, as a JPEG data URL."""
    xs, ys = [c['px'][0] for c in cs.values()], [c['px'][1] for c in cs.values()]
    x0, y0 = max(int(min(xs)) - 50, 0), max(int(min(ys)) - 50, 0)
    crop = frame[y0:int(max(ys)) + 50, x0:int(max(xs)) + 50]
    z = min(4.0, 1400 / max(crop.shape[:2]))
    img = cv2.resize(crop, None, fx=z, fy=z, interpolation=cv2.INTER_LANCZOS4)
    for n, c in cs.items():
        u, v = int((c['px'][0] - x0) * z), int((c['px'][1] - y0) * z)
        r = int((c['d_mm'] * PX_MM / 2 + 8) * z)  # around the colony, so the model sees the colony itself
        cv2.circle(img, (u, v), r, (0, 255, 255), 2, cv2.LINE_AA)
        for col, th in (((0, 0, 0), 5), ((0, 255, 255), 2)):
            cv2.putText(img, str(n), (u + r, v - r), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, th, cv2.LINE_AA)
    return 'data:image/jpeg;base64,' + base64.b64encode(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1]).decode()


def on_call(prompt):
    """SAM re-run through tools/sam_server.py or the demo page, on the frame in st2_top.jpg."""
    done = f'{S}/sam_rerun.json'
    if os.path.exists(done):
        os.remove(done)
    open(f'{S}/sam_rerun_request', 'w').write(prompt)
    t0 = time.time()
    while time.time() - t0 < 90:
        if os.path.exists(done):
            return json.load(open(done))['marks']
        time.sleep(0.3)
    raise TimeoutError('SAM did not answer in 90 s')


def renumbered(old, new):
    """Marks keep their number when the list grows: a struck or viewed number stays the same mark."""
    known = {tuple(c['px']): i for i, c in old.items()}
    next_n = max(old, default=0) + 1
    out = {}
    for c in new.values():
        i = known.get(tuple(c['px']))
        if i is None:
            i, next_n = next_n, next_n + 1
        out[i] = c
    return out


def dish_view(frame, dish_c):
    r = int((AGAR_R + 3) * PX_MM)
    u, v = int(dish_c[0]), int(dish_c[1])
    crop = frame[max(v - r, 0):v + r, max(u - r, 0):u + r]
    return 'data:image/jpeg;base64,' + base64.b64encode(cv2.imencode('.jpg', cv2.resize(crop, None, fx=1000 / max(crop.shape[:2]), fy=1000 / max(crop.shape[:2]), interpolation=cv2.INTER_LANCZOS4), [cv2.IMWRITE_JPEG_QUALITY, 90])[1]).decode()


def decide(frame, variants, taken, dish_c, n, say=print, call=post, sam=None):
    """The critic loop. Returns the chosen SAM positions in picking order; raises Refused, or RuntimeError if the model does not settle."""
    variants = {k: list(v) for k, v in variants.items()}
    cs = candidates(variants, taken, np.array(dish_c, float))
    pickable = lambda: sum(c['pickable'] for c in cs.values())
    msgs = [{'role': 'user', 'content': PROMPT.format(n=n, reruns=MAX_RERUNS if sam else 0)}]
    trace, rejected, viewed, saw_dish, reruns = [], {}, set(), False, 0

    def save(**out):
        json.dump({'model': MODEL, 'calls': trace, 'reruns': reruns, 'rejected': [dict(cs[i], number=i, reason=w) for i, w in rejected.items()], **out}, open(f'{S}/colony_agent.json', 'w'), indent=1)

    if not pickable() and sam is None:
        save(refused='SAM found nothing pickable')
        raise Refused('SAM found nothing pickable')
    order = ('strict', 'normal', 'loose')
    for turn in range(MAX_TURNS):
        if turn == MAX_TURNS - 3:  # a crowded plate can use up every turn on looking
            msgs.append({'role': 'user', 'content': 'You have 3 turns left. Choose or refuse now.'})
        reply = call({'model': MODEL, 'messages': msgs, 'tools': TOOLS})['choices'][0]['message']
        msgs.append({k: v for k, v in reply.items() if v is not None})
        if not reply.get('tool_calls'):
            msgs.append({'role': 'user', 'content': 'Call one of the tools.'})
            continue
        images = []
        for t in reply['tool_calls']:
            name = t['function']['name']
            trace.append(name)
            try:
                a = json.loads(t['function']['arguments'] or '{}')
                if name == 'view_dish':
                    saw_dish = True
                    images.append({'type': 'image_url', 'image_url': {'url': dish_view(frame, dish_c)}})
                    out = 'Photo of the dish follows.'
                    say('Muse Spark -> view_dish')
                elif name == 'list_colonies':
                    rows = [{k: c[k] for k in ('d_mm', 'score', 'rim_mm', 'gap_mm', 'pickable')} | {'number': i} for i, c in cs.items()
                            if i not in rejected and order.index(c['setting']) <= order.index(a['setting'])]
                    out = json.dumps(rows)
                    say(f'Muse Spark -> list_colonies({a["setting"]}): {len(rows)} marks')
                elif name == 'view_colonies':
                    some = {i: cs[i] for i in a['numbers'][:MAX_VIEW]}
                    viewed |= set(some)
                    images.append({'type': 'image_url', 'image_url': {'url': view(frame, some)}})
                    out = f'Photo of marks {list(some)} follows.'
                    say(f'Muse Spark -> view_colonies({list(some)})')
                    save(viewing=[c['px'] for c in some.values()])
                elif name == 'reject_colonies':
                    for p in a['rejects']:
                        if p['number'] in cs:
                            rejected[p['number']] = p['reason']
                            say(f"Muse Spark rejected {p['number']}: {p['reason']}")
                    save()
                    out = f'Rejected so far: {list(rejected)}'
                elif name == 'rerun_sam':
                    if sam is None or reruns >= MAX_RERUNS:
                        out = f'REJECTED: no re-runs left ({reruns} of {MAX_RERUNS} used). Choose or refuse now.'
                        say(f'Muse Spark -> rerun_sam: refused, {reruns} of {MAX_RERUNS} used')
                    elif a['prompt'] not in SAM_PROMPTS:
                        out = f'REJECTED: prompt must be one of {SAM_PROMPTS}.'
                    else:
                        reruns += 1  # counted before the call: a failed re-run still uses one up
                        floor = max(0.05, min(0.9, float(a.get('min_score', 0.3))))
                        try:
                            marks = [m for m in sam(a['prompt']) if m['score'] >= floor]
                        except Exception as e:
                            marks, out = None, f'SAM failed: {e}'
                            say(f'Muse Spark -> rerun_sam({a["prompt"]}): SAM failed')
                        if marks is not None:
                            have = [np.array(c['px']) for c in variants['loose']]
                            fresh = [m for m in marks if all(np.linalg.norm(np.array(m['px']) - q) >= 10 for q in have)]
                            variants['loose'] += fresh
                            before = set(cs)
                            cs = renumbered(cs, candidates(variants, taken, np.array(dish_c, float)))
                            added = sorted(set(cs) - before)
                            out = f'Re-run {reruns} of {MAX_RERUNS}: {len(marks)} marks, {len(added)} new, numbers {added}. Use list_colonies(loose) to read them.'
                            say(f'Muse Spark -> rerun_sam({a["prompt"]}, min_score {floor}): {len(added)} new marks, re-run {reruns} of {MAX_RERUNS}')
                elif name == 'refuse_plate':
                    if not saw_dish:
                        out = 'REJECTED: call view_dish first.'
                    else:
                        say(f"Muse Spark REFUSED the plate: {a['reason']}")
                        save(refused=a['reason'])
                        raise Refused(a['reason'])
                elif name == 'choose_colonies':
                    nums = [p['number'] for p in a['picks']]
                    need = min(n, pickable())
                    bad = [i for i in nums if i not in cs or not cs[i]['pickable'] or i in rejected]
                    unseen = [i for i in nums if i not in viewed]
                    if not saw_dish:
                        out = 'REJECTED: call view_dish first.'
                    elif not need or bad or unseen or len(set(nums)) != need or len(nums) != need:
                        out = f'REJECTED: need {need} different numbers, pickable, not rejected and viewed. Bad: {bad}; not viewed: {unseen}'
                        say(f'Muse Spark -> choose_colonies({nums}): rejected by the rules')
                    else:
                        for p in a['picks']:
                            say(f"Muse Spark chose {p['number']}: {p['reason']}")
                        save(picks=[p | cs[p['number']] for p in a['picks']])
                        return [cs[i]['px'] for i in nums]
                else:
                    out = f'unknown tool {name}'
            except (KeyError, TypeError, ValueError) as e:
                out = f'ERROR: {e!r}'
            msgs.append({'role': 'tool', 'tool_call_id': t['id'], 'content': out})
        if images:  # a tool result is text only: the photo comes as the next message
            msgs.append({'role': 'user', 'content': [{'type': 'text', 'text': 'The requested photo.'}, *images]})
    raise RuntimeError(f'no choice in {MAX_TURNS} turns')


if __name__ == '__main__':
    import sys
    DISH_C = (2051, 620)  # as colony_cycle.py: importing it would start a run
    print(decide(cv2.imread(f'{S}/st2_top.jpg'), json.load(open(f'{S}/colony_variants.json')), [], DISH_C, int(sys.argv[1]) if len(sys.argv) > 1 else 3))
