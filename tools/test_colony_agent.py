"""Offline check of the Muse Spark loop with a scripted model: no key, no network, no arm.
usage: python tools/test_colony_agent.py"""
import json
import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, 'tools')
import colony_agent as ca  # noqa: E402

C = (500, 500)
near = {'px': [510.0, 500.0], 'd_mm': 3.0, 'score': 0.9}          # 1: touches 2
near2 = {'px': [522.0, 500.0], 'd_mm': 3.0, 'score': 0.85}        # 2
good = {'px': [400.0, 500.0], 'd_mm': 3.5, 'score': 0.8}          # 3
rim = {'px': [655.0, 500.0], 'd_mm': 3.0, 'score': 0.75}          # 4: 1.7 mm from the rim
small = {'px': [500.0, 400.0], 'd_mm': 1.0, 'score': 0.3}         # 5: loose only
done = {'px': [500.0, 600.0], 'd_mm': 3.0, 'score': 0.99}         # already picked
variants = {'strict': [near, near2, good, rim, done], 'normal': [near, near2, good, rim, done], 'loose': [near, near2, good, rim, small, done]}
frame = np.zeros((1000, 1000, 3), np.uint8)


def reply(*calls):
    return {'choices': [{'message': {'role': 'assistant', 'content': None, 'tool_calls': [
        {'id': f'c{i}', 'type': 'function', 'function': {'name': n, 'arguments': json.dumps(a)}} for i, (n, a) in enumerate(calls)]}}]}


script = iter([
    {'choices': [{'message': {'role': 'assistant', 'content': 'thinking'}}]},  # no tool call: nudged
    reply(('choose_colonies', {'picks': [{'number': 3, 'reason': 'x'}, {'number': 5, 'reason': 'x'}]})),  # dish not seen: rejected
    reply(('view_dish', {})),
    reply(('list_colonies', {'setting': 'normal'})),
    reply(('view_colonies', {'numbers': [3, 5]})),
    reply(('reject_colonies', {'rejects': [{'number': 2, 'reason': 'glint'}, {'number': 77, 'reason': 'x'}]})),
    reply(('choose_colonies', {'picks': [{'number': 2, 'reason': 'x'}, {'number': 3, 'reason': 'x'}]})),  # struck: rejected
    reply(('view_colonies', {'numbers': [99]})),  # unknown number: an error for the model, no crash
    reply(('choose_colonies', {'picks': [{'number': 1, 'reason': 'x'}, {'number': 3, 'reason': 'x'}]})),  # 1 touches 2: rejected
    reply(('choose_colonies', {'picks': [{'number': 3, 'reason': 'x'}, {'number': 3, 'reason': 'x'}]})),  # twice the same: rejected
    reply(('choose_colonies', {'picks': [{'number': 5, 'reason': 'big enough'}, {'number': 3, 'reason': 'apart'}]})),
])
seen = []


def fake(body):
    seen.append(json.loads(json.dumps(body)))
    return next(script)


with tempfile.TemporaryDirectory() as d:
    ca.S = d
    got = ca.decide(frame, variants, [done['px']], C, 2, say=lambda *a: None, call=fake)
    saved = json.load(open(os.path.join(d, 'colony_agent.json')))
assert got == [small['px'], good['px']], got  # SAM's own positions, nothing from the model
assert [p['number'] for p in saved['picks']] == [5, 3]
last = seen[-1]['messages']
texts = [m['content'] for m in last if m['role'] == 'tool']
assert sum('REJECTED' in t for t in texts) == 4 and any('ERROR' in t for t in texts), texts
assert all('px' not in r for r in json.loads(next(t for t in texts if t.startswith('[')))), 'the model must not see coordinates'
assert [r['number'] for r in saved['rejected']] == [2]
assert any(isinstance(m['content'], list) and m['content'][1]['image_url']['url'].startswith('data:image/jpeg') for m in last if m['role'] == 'user')


refuses = iter([reply(('refuse_plate', {'reason': 'x'})), reply(('view_dish', {})), reply(('refuse_plate', {'reason': 'lawn'}))])
with tempfile.TemporaryDirectory() as d:
    ca.S = d
    try:
        ca.decide(frame, variants, [], C, 1, say=lambda *a: None, call=lambda b: next(refuses))
        raise AssertionError('should refuse')
    except ca.Refused as e:
        assert str(e) == 'lawn'
    assert json.load(open(os.path.join(d, 'colony_agent.json')))['refused'] == 'lawn'
    try:
        ca.decide(frame, {k: [] for k in variants}, [], C, 1, say=lambda *a: None, call=None)  # nothing from SAM and no re-run possible: no API call
        raise AssertionError('should refuse')
    except ca.Refused:
        pass


# re-runs: the cap holds in code, numbers stay, marks too close to a known one are dropped, a SAM failure uses one up
asks = []


def sam(prompt):
    asks.append(prompt)
    if len(asks) == 2:
        raise OSError('gpu')
    return [{'px': [420.0, 420.0], 'd_mm': 4.0, 'score': 0.9}, {'px': [402.0, 500.0], 'd_mm': 3.0, 'score': 0.9}, {'px': [300.0, 500.0], 'd_mm': 3.0, 'score': 0.1}]


loop = iter([reply(('view_dish', {})), reply(('rerun_sam', {'prompt': 'round white spot'})), reply(('rerun_sam', {'prompt': 'rm -rf /'})),
             reply(('rerun_sam', {'prompt': 'bubble'})), reply(('rerun_sam', {'prompt': 'white speck'})), reply(('rerun_sam', {'prompt': 'bubble'})),
             reply(('view_colonies', {'numbers': [7, 4]})), reply(('choose_colonies', {'picks': [{'number': 7, 'reason': 'new'}, {'number': 4, 'reason': 'old'}]}))])
seen2 = []


def fake2(body):
    seen2.append(json.loads(json.dumps(body)))
    return next(loop)


with tempfile.TemporaryDirectory() as d:
    ca.S = d
    got = ca.decide(frame, variants, [], C, 2, say=lambda *a: None, call=fake2, sam=sam)
    saved = json.load(open(os.path.join(d, 'colony_agent.json')))
assert len(asks) == 3, asks  # the fourth call never reached SAM
assert saved['reruns'] == 3
assert got == [[420.0, 420.0], good['px']], got
assert len(variants['loose']) == 6, 'the caller\'s list stays as it was'
tool = [m['content'] for m in seen2[-1]['messages'] if m['role'] == 'tool']
assert any('no re-runs left' in x for x in tool) and any('plain words' in x for x in tool) and any('SAM failed' in x for x in tool), tool


def stubborn(body):
    return reply(('list_colonies', {'setting': 'normal'}))


try:
    ca.decide(frame, variants, [], C, 1, say=lambda *a: None, call=stubborn)
    raise AssertionError('should give up')
except RuntimeError as e:
    assert 'turns' in str(e)
print('ok')
