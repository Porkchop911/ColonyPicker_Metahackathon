"""The demo page with no robot: load a saved dish photo, then run the Muse Spark critic on its saved SAM 3.1 detections. It uses no GPU, camera, arm or lamps, and
imports none of that code. Start with start_offline_demo.bat, or: python tools/offline_page.py. Pick a picture and press Start: the critic runs live with MODEL_API_KEY, else as a recorded run."""
import collections
import glob
import json
import os
import re
import subprocess
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, 'tools')
import colony_agent as ca  # noqa: E402

E = 'evidence'
PORT = 8766
WELLS = ['C10', 'C11', 'C12']
PACE = '1.5'  # s per model turn on a recorded run, to be read as it happens
PLATES = sorted(int(re.search(r'_(\d+)\.jpg', f)[1]) for f in glob.glob(f'{E}/offline_plate_*.jpg') if 'replay' not in f)
HAS_KEY = bool(os.environ.get('MODEL_API_KEY') or ca.KEY_FILE.exists())
log = collections.deque(maxlen=400)
job = {'p': None, 'title': 'idle', 'exit': None}
view = {'plate': PLATES[0], 'live': HAS_KEY, 'demo': None}
thumbs = {}


def page():
    html = open('tools/demo_page.html', encoding='utf-8').read()
    thumbs_html = ''.join(f'<button class="thumb" id="p{n}" onclick="act(\'load-{n}\')"><img src="/img/thumb-{n}" alt="plate {n}"><span>{n}</span></button>' for n in PLATES)
    control = (f'<section class="card"><h2>Pick a picture, then press Start</h2><div class="thumbs">{thumbs_html}</div>'
               '<div class="cap">1, 2, 3, 13, 14: clean colonies from the rig. 4: a lawn. 5-12: hazy or crowded. 15-20: phone photos, scale estimated.</div>'
               '<div class="btns two"><button class="primary" style="grid-column:1/-1" onclick="act(\'start\')">Start</button><button id="m-recorded" class="mode" onclick="act(\'mode-recorded\')">Recorded run</button>'
               f'<button id="m-live" class="mode" onclick="act(\'mode-live\')"{"" if HAS_KEY else " disabled"}>Live Muse Spark</button></div>'
               f'<div class="cap">{"Live calls the Meta API." if HAS_KEY else "Live needs MODEL_API_KEY from dev.meta.ai."} Grey: SAM 3.1 marks. Blue: being looked at. Red: struck. Yellow: chosen. No robot here: nothing moves.</div><input id="agar" type="hidden"></section>')
    html, k = re.subn(r'<section class="card"><h2>Control</h2>.*?</section>', lambda m: control, html, count=1, flags=re.S)
    assert k == 1
    for a, b in (('<h1>Colony picker</h1>', '<h1>Colony picker, offline</h1>'),
                 ('<title>Colony picker</title>', '<title>Colony picker offline</title>'),
                 ('<section class="card"><div class="row"><h2>Cameras (live)</h2>', '<section class="card" style="display:none"><div class="row"><h2>Cameras (live)</h2>'),
                 ('let showCams=true;', 'let showCams=false;'),
                 ("try{showCams=localStorage.getItem('cams')!=='0'}catch(e){}", ''),
                 ('Press Dry run or Run demo: SAM 3.1 detects the colonies on a fresh frame.', 'Pick a picture, then press Start.'),
                 ('</style>', '.thumbs{display:grid;grid-template-columns:repeat(7,1fr);gap:4px}.thumb{padding:2px;position:relative;border-radius:8px}.thumb img{border-radius:6px}.thumb span{position:absolute;left:5px;top:2px;font-size:12px;color:#ffd400;text-shadow:0 0 3px #000}'
                  '.thumb.on,.mode.on{border-color:var(--now);box-shadow:0 0 0 2px var(--now)}.btns.two{grid-template-columns:1fr 1fr;margin-top:10px}button:disabled{opacity:.4;cursor:not-allowed}</style>'),
                 ('</script></body>', "setInterval(async()=>{try{const s=await (await fetch('/state')).json();document.querySelectorAll('.thumb').forEach(b=>b.classList.toggle('on',b.id==='p'+s.plate));document.querySelectorAll('.mode').forEach(b=>b.classList.toggle('on',(b.id==='m-live')===s.live))}catch(e){}},400);</script></body>")):
        assert html.count(a) == 1, a
        html = html.replace(a, b)
    return html


def state():
    running = job['p'] is not None and job['p'].poll() is None
    return {'running': running, 'title': job['title'], 'exit': job['exit'], 'log': list(log)[-200:], 'demo': view['demo'], 'plate': view['plate'], 'live': view['live'], 'agar': '',
            'wells': {'disinfect': 'A12', 'rinse': 'A10', 'targets': WELLS}}


def loaded(n):
    """The photo with SAM's marks, before the critic."""
    img = cv2.imread(f'{E}/offline_plate_{n}.jpg')
    z = 1150 / max(img.shape[:2])
    img = cv2.resize(img, None, fx=z, fy=z, interpolation=cv2.INTER_LANCZOS4)
    for c in json.load(open(f'{E}/offline_plate_{n}.json'))['loose']:
        cv2.circle(img, (int(c['px'][0] * z), int(c['px'][1] * z)), max(8, round(7 * z)), (230, 230, 230), 1, cv2.LINE_AA)
    return img, z


def dish():
    """SAM's marks, then the critic's work as it happens: colony_agent.json is rewritten at every step."""
    img, z = loaded(view['plate'])
    r = max(8, round(7 * z))
    try:
        d = json.load(open(f'{E}/colony_agent.json'))
    except (OSError, ValueError):  # no run yet, or read mid-write
        return img
    for p in d.get('viewing', []):
        cv2.circle(img, (int(p[0] * z), int(p[1] * z)), round(r * 1.5), (255, 170, 60), 3, cv2.LINE_AA)
    for c in d['rejected']:
        u, v = int(c['px'][0] * z), int(c['px'][1] * z)
        cv2.line(img, (u - r, v - r), (u + r, v + r), (60, 60, 255), 2, cv2.LINE_AA)
        cv2.line(img, (u - r, v + r), (u + r, v - r), (60, 60, 255), 2, cv2.LINE_AA)
    for k, c in enumerate(d.get('picks', []), 1):
        u, v = int(c['px'][0] * z), int(c['px'][1] * z)
        cv2.circle(img, (u, v), r * 2, (0, 255, 255), 3, cv2.LINE_AA)
        for col, th in (((0, 0, 0), 6), ((0, 255, 255), 2)):
            cv2.putText(img, f'{k} {WELLS[k - 1]}', (u + r * 2, v - r), cv2.FONT_HERSHEY_SIMPLEX, 1.1, col, th, cv2.LINE_AA)
    if 'refused' in d:
        h, w = img.shape[:2]
        (tw, _), _ = cv2.getTextSize('REFUSED', cv2.FONT_HERSHEY_DUPLEX, 3, 5)
        for col, th in (((0, 0, 0), 14), ((60, 60, 255), 5)):
            cv2.putText(img, 'REFUSED', ((w - tw) // 2, h // 2 + 30), cv2.FONT_HERSHEY_DUPLEX, 3, col, th, cv2.LINE_AA)
    return img


def thumb(n):
    img = cv2.imread(f'{E}/offline_plate_{n}.jpg')
    s = max(img.shape[:2])
    y, x = (s - img.shape[0]) // 2, (s - img.shape[1]) // 2
    return cv2.resize(cv2.copyMakeBorder(img, y, s - img.shape[0] - y, x, s - img.shape[1] - x, cv2.BORDER_CONSTANT), (90, 90), interpolation=cv2.INTER_AREA)


def busy():
    return job['p'] is not None and job['p'].poll() is None


def clear():
    if os.path.exists(f'{E}/colony_agent.json'):
        os.remove(f'{E}/colony_agent.json')


def load(n):
    """A click on a picture: only shows that plate. The critic waits for Start."""
    if busy():
        return
    clear()
    marks = json.load(open(f'{E}/offline_plate_{n}.json'))['loose']
    view.update(plate=n, demo={'line': f'Plate {n}: SAM 3.1 made {len(marks)} marks. Press Start.', 'when': '', 'colonies': []})
    log.append(f'-- plate {n} loaded')


def finish(n):
    """What the run decided, for the page's dish line and plate map."""
    try:
        d = json.load(open(f'{E}/colony_agent.json'))
    except (OSError, ValueError):
        view['demo'] = {'line': 'The run failed: see the log.', 'when': '', 'colonies': []}
        return
    picks = d.get('picks', [])
    line = (f"Refused: {d['refused']}" if 'refused' in d else
            f"Muse Spark struck {len(d['rejected'])} marks and chose {len(picks)} colonies, {d['reruns']} SAM re-runs.")
    view['demo'] = {'line': line, 'when': '', 'colonies': [{'n': k + 1, 'px': p['px'], 'well': w, 'state': 'todo'} for k, (p, w) in enumerate(zip(picks, WELLS))]}


def run():
    if busy():
        return
    clear()  # Start again on the same plate begins from SAM's marks
    live, n = view['live'], view['plate']
    view['demo'] = {'line': f'Plate {n}: Muse Spark is judging it...', 'when': '', 'colonies': []}
    title = 'Muse Spark' if live else 'recorded run'
    log.append(f'=== plate {n}: {title} ===')
    p = subprocess.Popen([sys.executable, '-u', 'tools/offline_demo.py', str(n), *([] if live else ['--replay', '--pace', PACE])], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, encoding='utf-8', errors='replace', env={**os.environ, 'PYTHONIOENCODING': 'utf-8'}, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    job.update(p=p, title=title, exit=None)

    def pump():
        for line in p.stdout:
            if not line.startswith(('Picture:', 'On the robot')):
                log.append(line.rstrip())
        job['exit'] = p.wait()
        finish(n)
    threading.Thread(target=pump, daemon=True).start()


class H(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == '/':
            return self.send(page().encode(), 'text/html; charset=utf-8')
        if self.path == '/state':
            return self.send(json.dumps(state()).encode(), 'application/json')
        if self.path.startswith('/img/'):
            name = self.path[5:].split('?')[0]
            if name == 'dish':
                img = dish()
            elif name.startswith('thumb-') and name[6:].isdigit() and int(name[6:]) in PLATES:
                n = int(name[6:])
                thumbs.setdefault(n, thumb(n))
                img = thumbs[n]
            else:
                img = np.full((8, 8, 3), 18, np.uint8)
            return self.send(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 88])[1].tobytes(), 'image/jpeg')
        self.send(b'not found', 'text/plain', 404)

    def do_POST(self):
        name = self.path.rsplit('/', 1)[-1]
        if name in ('mode-recorded', 'mode-live'):
            view['live'] = name == 'mode-live' and HAS_KEY
        elif name.startswith('load-') and name[5:].isdigit() and int(name[5:]) in PLATES:
            load(int(name[5:]))
        elif name == 'start':
            run()
        else:
            return self.send(b'not found', 'text/plain', 404)
        self.send(b'ok', 'text/plain')


if __name__ == '__main__':
    load(PLATES[0])
    server = ThreadingHTTPServer(('127.0.0.1', PORT), H)  # local only, as the real page
    print(f'offline demo on http://127.0.0.1:{PORT}: close this window to stop', flush=True)
    if '--no-browser' not in sys.argv:
        webbrowser.open(f'http://127.0.0.1:{PORT}')
    server.serve_forever()
