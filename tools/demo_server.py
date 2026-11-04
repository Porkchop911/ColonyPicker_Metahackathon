"""Demo page: run the colony demo from a browser, no terminal. Start with start_demo.bat (repo root), which opens
http://127.0.0.1:8765. Standard library only (http.server); the robot tools run as child processes, one at a time.

Buttons: Check arm (read only), Park, Dry run (SAM colonies + wells), Run demo (colony_cycle.py), Stop after this
colony, ABORT (kill the running step, then needle straight up and park). Panels: the cycle's status picture
(evidence/tip/demo_status.jpg), both cameras live (the tools write evidence/tip/live_<index>.jpg while a camera is
open) with the target colony and the current / next target well marked on the overhead, and the step's log."""
import collections
import json
import os
import subprocess
import sys
import time
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
sys.path.insert(0, 'tools')
import a1lib as a  # noqa: E402
from rig_io import Camera, OVERHEAD, SIDE, SIDE_PAPER  # noqa: E402

PY = sys.executable
S = 'evidence/tip'
STOP = f'{S}/stop_demo'
PORT = 8765
WORK = (1250, 250, 3070, 1560)  # overhead px (full size): dish and plate, the part worth watching
log = collections.deque(maxlen=400)
job = {'p': None, 'title': 'idle', 'exit': None}
lock = threading.Lock()


cams = {'open': [], 'want': True}


def release_cams():
    """The robot tools need the cameras: a camera opens in one process only."""
    cams['want'] = False
    for c in cams['open']:
        c.close()
    cams['open'] = []


def preview():
    """Both cameras open for good, published to shared memory: every robot step reads its frames there (rig_io.Camera)
    instead of opening the USB devices itself (20-30 s per open, frozen feeds between steps, 11-03)."""
    import time
    while True:
        if not cams['open']:
            try:
                cams['open'] = [Camera(OVERHEAD, publish=True), Camera(SIDE, paper_target=SIDE_PAPER, publish=True)]
                log.append('-- cameras open and shared')
            except Exception as e:  # another program holds a camera: try again later
                for c in cams['open']:
                    c.close()
                cams['open'] = []
                log.append(f'(camera preview: {e})')
                time.sleep(5)
        time.sleep(0.5)


def run(args, title):
    with lock:
        if job['p'] is not None and job['p'].poll() is None:
            log.append(f'!! {title}: "{job["title"]}" is still running')
            return
        if os.path.exists(STOP):
            os.remove(STOP)
        log.append(f'=== {title} ===')
        p = subprocess.Popen([PY, '-u', *args], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             errors='replace', creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
        job.update(p=p, title=title, exit=None)

    def pump():
        for line in p.stdout:
            if 'Warn' not in line and 'INFO' not in line:  # library noise (SAM, OpenCV)
                log.append(line.rstrip())
        job['exit'] = p.wait()
        log.append(f'--- {title}: exit {job["exit"]}')
    threading.Thread(target=pump, daemon=True).start()


def kill_tree(p):
    """Stop the running step and every child it started; the arm halts at its current small move (<= 6 pulses)."""
    import psutil
    try:
        root = psutil.Process(p.pid)
        procs = root.children(recursive=True) + [root]
    except psutil.Error:
        return
    for q in procs:
        try:
            q.kill()
        except psutil.Error:
            pass
    psutil.wait_procs(procs, timeout=5)


def abort():
    p = job['p']
    if p is not None and p.poll() is None and job['title'].startswith('abort'):
        log.append('!! ABORT: already lifting and parking')  # a second press killed the first park (11-03)
        return
    log.append('!! ABORT')
    if p is not None and p.poll() is None:
        kill_tree(p)
        log.append('   stopped the running step')
    job['p'] = None
    run(['tools/bg_refresh.py', '--no-plate'], 'abort: needle straight up, park')  # refuses off the model: by hand


CHECK = ("import sys; sys.path.insert(0, 'tools'); import well_reach as wr, a1lib as a; r = wr.Servos().read(); "
         "q = {k: r[k] for k in wr.ARM}; print('arm', q); a.assert_unfolded(q); "
         "print('parked' if max(abs(q[k] - v) for k, v in {1: 500, 2: 450, 3: 207, 4: 468}.items()) < 10 "
         "else 'unfolded, not parked: press Park')")
def fresh():
    """A new run detects the colonies again (SAM 3.1): drop the last run's picture so nothing stale is shown."""
    if job['p'] is not None and job['p'].poll() is None:
        return
    for f in ('demo_state.json', 'demo_status.jpg'):
        if os.path.exists(f'{S}/{f}'):
            os.remove(f'{S}/{f}')


REC = {'p': None}


def record():
    """Record button: both cameras side by side from the page's shared frames (tools/record_cams.py), press again to stop."""
    if REC['p'] is not None and REC['p'].poll() is None:
        open(f'{S}/rec_stop', 'w').close()
        log.append('-- recording stopped')
        return
    if os.path.exists(f'{S}/wells_used.json'):  # every recording starts with all sample wells free (user, 11-04)
        os.remove(f'{S}/wells_used.json')
    out = f"{S}/demo_rec_{time.strftime('%m%d_%H%M%S')}.mp4"
    REC['p'] = subprocess.Popen([PY, 'tools/record_screen.py', out])  # the page itself in the video (user, 11-04)
    subprocess.Popen([PY, 'tools/record_cams.py', out.replace('.mp4', '_cams.mp4')])
    log.append(f'-- recording the screen to {out}, cameras beside it')


ACTIONS = {
    'check': lambda: run(['-c', CHECK], 'check arm'),
    'park': lambda: run(['tools/bg_refresh.py', '--no-plate'], 'park'),
    'dry': lambda: (fresh(), run(['tools/colony_cycle.py', '--dry', '--no-view'], 'dry run')),
    'run': lambda: (fresh(), run(['tools/colony_cycle.py', '--no-view', '--max', '3', '--no-start-dip', '--blind-wells'], 'demo')),  # three colonies, C10 C11 C12 (user, 11-04)
    'record': record,
    'stop': lambda: (open(STOP, 'w').close(), log.append('-- stop requested: the demo ends after this colony')),
    'abort': abort,
}


def state():
    try:
        demo = json.load(open(f'{S}/demo_state.json'))
    except (OSError, ValueError):
        demo = None
    p = job['p']
    running = p is not None and p.poll() is None
    try:
        wells = json.load(open(f'{S}/demo_wells.json'))
    except (OSError, ValueError):
        wells = None
    if demo is None and running and job['title'] in ('demo', 'dry run'):
        demo = {'line': 'detecting colonies (SAM 3.1) on a fresh overhead frame...', 'when': '', 'colonies': []}
    return {'running': running, 'title': job['title'], 'exit': job['exit'], 'log': list(log)[-200:], 'demo': demo,
            'wells': wells}


def jpeg(img):
    return cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()


def overhead():
    """Live overhead with the target colony and the current / next target well marked (live frames are half size)."""
    img = cv2.imread(f'{S}/live_{OVERHEAD}.jpg')
    if img is None:
        return None
    k = img.shape[1] / 3840.0
    d = state()['demo'] or {}
    cur = d.get('current')
    if cur is not None:
        u, v = (int(c * k) for c in d['colonies'][cur]['px'])
        cv2.circle(img, (u, v), 16, (0, 220, 255), 3)
        cv2.putText(img, f"colony {cur + 1}", (u + 18, v - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)
    for name, col in (('target_well', (60, 220, 60)), ('next_well', (255, 170, 60))):
        w = d.get(name)
        if w:
            u, v = (int(c * k) for c in a.project('top', a.pr.well(w)))
            cv2.circle(img, (u, v), 14, col, 3)
            cv2.putText(img, f"{'target' if name == 'target_well' else 'next'} {w}", (u + 16, v + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
    x0, y0, x1, y1 = (int(c * k) for c in WORK)
    return img[y0:y1, x0:x1]


COL = {'todo': (255, 255, 255), 'now': (0, 215, 255), 'done': (80, 220, 80), 'failed': (70, 70, 255)}
DISH = (1845, 420, 2255, 820)  # overhead px (full size) around the whole dish with a margin


def dish(variant=None):
    """The dish, big: the last 4K background with every colony SAM found, its number and well,
    coloured by state (white waiting, yellow now, green done, red failed)."""
    import time
    img = cv2.imread(f'{S}/st2_top.jpg')  # the 4K still, never the half-size live frame (user, 11-04)
    if img is None:
        return None
    k = img.shape[1] / 3840.0
    d = state()['demo'] or {}
    cols = d.get('colonies') or []
    if variant:  # one of the three detection settings, on the frame SAM ran on
        cols = [{'n': i + 1, 'px': p, 'well': '', 'state': 'todo'} for i, p in enumerate((d.get('choices') or {}).get(variant, []))]
    box = DISH  # always the whole dish: a box around the colonies zoomed far in with one colony (user, 11-03)
    if cols:
        xs, ys = [c['px'][0] for c in cols], [c['px'][1] for c in cols]
        box = (min(box[0], min(xs) - 60), min(box[1], min(ys) - 60), max(box[2], max(xs) + 60), max(box[3], max(ys) + 60))
    x0, y0, x1, y1 = (int(max(v, 0) * k) for v in box)
    crop = img[y0:y1, x0:x1]
    z = 1000.0 / max(crop.shape[1], 1)
    crop = cv2.resize(crop, None, fx=z, fy=z, interpolation=cv2.INTER_LANCZOS4)
    for c in cols:
        u, v = int((c['px'][0] * k - x0) * z), int((c['px'][1] * k - y0) * z)
        col, r = COL.get(c['state'], (255, 255, 255)), int(22 * k * z)
        cv2.circle(crop, (u, v), r, (0, 0, 0), 7)
        cv2.circle(crop, (u, v), r, col, 3)
        text = f"{c['n']}  {c['well']}".strip()
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.75, 2)
        tx, ty = u + r + 6, v - r
        cv2.rectangle(crop, (tx - 6, ty - th - 8), (tx + tw + 6, ty + 6), (25, 25, 25), -1)
        cv2.rectangle(crop, (tx - 6, ty - th - 8), (tx + tw + 6, ty + 6), col, 2)
        cv2.putText(crop, text, (tx, ty), cv2.FONT_HERSHEY_DUPLEX, 0.75, col, 2)
    return crop


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><title>Colony picker</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{--bg:#0d0f14;--panel:#151922;--line:#252b38;--text:#eef0f4;--dim:#8a93a8;--accent:#4f8cff;--ok:#46d27f;--now:#ffd400;--bad:#ff4d5e;--dis:#3d7bff;--rin:#3fd0ff}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font:14px/1.45 "Segoe UI",system-ui,sans-serif}
header{display:flex;align-items:center;gap:14px;padding:14px 24px;border-bottom:1px solid var(--line)}
h1{font-size:19px;margin:0;font-weight:650}.pill{padding:4px 11px;border-radius:999px;font-size:12px;background:#1e2330;color:var(--dim)}
.pill.run{background:rgba(79,140,255,.16);color:var(--accent)}
main{display:grid;grid-template-columns:1.55fr 1fr;gap:18px;padding:18px 24px;align-items:start}
.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:16px}
.card h2{font-size:12px;text-transform:uppercase;letter-spacing:1.1px;color:var(--dim);margin:0 0 12px;font-weight:650}
img{width:100%;border-radius:12px;display:block;background:#090b0f}
.line{margin-top:12px;font-size:17px;font-weight:600}
.btns{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
button{border:1px solid var(--line);background:#1d2230;color:var(--text);border-radius:11px;padding:13px 10px;font:600 14px "Segoe UI",sans-serif;cursor:pointer;transition:.15s}
button:hover{background:#252b3b}button.primary{background:var(--accent);border-color:var(--accent)}button.primary:hover{filter:brightness(1.1)}
button.abort{grid-column:1/-1;background:var(--bad);border-color:var(--bad);font-size:17px;padding:17px;letter-spacing:1px}
.plate{display:grid;grid-template-columns:24px repeat(8,1fr);gap:5px;margin-top:4px;max-width:420px}
.plate .h{font-size:11px;color:var(--dim);text-align:center;align-self:center}
.w{aspect-ratio:1;border-radius:50%;border:2px solid #2c3343;background:#1a1f2b;display:flex;align-items:center;justify-content:center;font-size:10px;color:#5d6578}
.w.t{border-color:#e8ebf2;color:#e8ebf2}.w.now{border-color:var(--now);background:rgba(255,212,0,.22);color:var(--now);box-shadow:0 0 0 3px rgba(255,212,0,.25)}
.w.done{border-color:var(--ok);background:rgba(70,210,127,.25);color:var(--ok)}.w.failed{border-color:var(--bad);color:var(--bad)}
.w.next{border-color:var(--rin);border-style:dashed;color:var(--rin)}.w.dis{background:var(--dis);border-color:var(--dis);color:#fff}.w.rin{background:var(--rin);border-color:var(--rin);color:#04202b}
.legend{display:flex;flex-wrap:wrap;gap:14px;font-size:12px;color:var(--dim);margin-top:12px}.dot{display:inline-block;width:11px;height:11px;border-radius:50%;margin-right:5px;vertical-align:-1px;border:2px solid}
.cams{display:grid;grid-template-columns:1.6fr 1fr;gap:12px}.cams.hidden{display:none}.cap{font-size:12px;color:var(--dim);margin:6px 2px}.agar{margin-top:14px;display:grid;gap:4px}.agar input{width:90px;margin-left:8px;background:#1d2230;color:var(--text);border:1px solid var(--line);border-radius:8px;padding:6px 8px;font:600 14px 'Segoe UI',sans-serif}
pre{margin:0;max-height:240px;overflow:auto;font:12px/1.4 Consolas,monospace;color:#c4cad6;white-space:pre-wrap}
.row{display:flex;justify-content:space-between;align-items:center}.toggle{font-size:12px;color:var(--accent);cursor:pointer;user-select:none}
.stack{display:grid;gap:18px}.choices{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:14px}.choice{background:#10131a;border:1px solid var(--line);border-radius:12px;padding:10px}.choice h3{margin:0 0 8px;font-size:15px}.choice.picked{border:2px solid var(--now);box-shadow:0 0 0 3px rgba(255,212,0,.2)}.choice.picked button{background:var(--now);border-color:var(--now);color:#1a1500}.choice button{width:100%;margin-top:8px}.full{grid-column:1/-1}@media(max-width:1000px){main{grid-template-columns:1fr}}
/* one screen when maximised (user, 11-03): dish | cameras + log | controls + plate */
html,body{height:100%}body{display:flex;flex-direction:column;overflow:hidden}header{padding:10px 20px}
main{flex:1;min-height:0;grid-template-columns:1.25fr 1fr .7fr;grid-template-rows:minmax(0,1fr);gap:14px;padding:12px 18px;align-items:stretch}
.card{padding:12px}.c1{grid-column:1;grid-row:1;display:flex;flex-direction:column;min-height:0}
#dish{flex:1;min-height:0;object-fit:contain}.choices img{max-height:calc(100vh - 340px);object-fit:contain}
.c2{grid-column:2;grid-row:1;display:flex;flex-direction:column;gap:14px;min-height:0}
.cams{grid-template-columns:1fr;gap:6px}.cams img{max-height:calc((100vh - 330px) / 2);object-fit:contain}.cap{margin:3px 2px}
.logc{flex:1;min-height:90px;display:flex;flex-direction:column}pre{flex:1;min-height:0;max-height:none}
.stack{grid-column:3;grid-row:1;align-content:start;gap:14px;overflow:auto;min-height:0}.plate{max-width:min(270px,calc((100vh - 480px) * .62 + 30px));gap:4px}
button{padding:11px 8px}button.abort{padding:14px}
@media(max-width:1000px),(max-height:620px){body{overflow:auto;height:auto}main{grid-template-columns:1fr;grid-template-rows:none}.c1,.c2,.stack{grid-column:auto;grid-row:auto}#dish{flex:none}}
</style></head><body>
<header><h1>Colony picker</h1><span id="pill" class="pill">idle</span><span id="when" class="pill"></span></header>
<main>
<section class="card c1"><h2>Colonies found by SAM 3.1</h2>
<div id="choices" class="choices" style="display:none"></div><img id="dish" alt="dish">
<div class="legend"><span><i class="dot" style="border-color:#fff"></i>waiting</span><span><i class="dot" style="border-color:var(--now)"></i>picking now</span><span><i class="dot" style="border-color:var(--ok)"></i>done</span><span><i class="dot" style="border-color:var(--bad)"></i>failed</span></div>
<div class="line" id="line">-</div></section>
<div class="stack">
<section class="card"><h2>Control</h2><div class="btns">
<button onclick="act('check')">Check arm</button><button onclick="act('park')">Park</button><button onclick="act('dry')">Dry run</button>
<button class="primary" onclick="act('run')">Run demo</button><button onclick="act('stop')" style="grid-column:span 2">Stop after this colony</button><button onclick="act('record')">Record on / off</button>
<button class="abort" onclick="act('abort')">ABORT</button></div>
<div class="agar" style="display:none"><label>Agar surface from the plate bottom (mm): <input id="agar" type="number" step="0.1" min="3" max="12" placeholder="auto" onchange="setAgar(this.value)"></label><span class="cap">as measured, nothing added; empty = side camera</span></div></section>
<section class="card"><h2>Plate</h2><div class="plate" id="plate"></div>
<div class="legend"><span><i class="dot" style="background:var(--dis);border-color:var(--dis)"></i>disinfect</span><span><i class="dot" style="background:var(--rin);border-color:var(--rin)"></i>rinse</span><span><i class="dot" style="border-color:var(--now)"></i>target now</span><span><i class="dot" style="border-color:var(--rin);border-style:dashed"></i>next</span><span><i class="dot" style="border-color:var(--ok)"></i>done</span></div></section>
</div>
<div class="c2"><section class="card"><div class="row"><h2>Cameras (live)</h2><span class="toggle" onclick="cams()">show / hide</span></div>
<div id="cams" class="cams"><div><img id="top" alt="overhead"><div class="cap">Overhead: yellow = target colony, green = target well, blue = next well</div></div><div><img id="side" alt="side"><div class="cap">Side</div></div></div></section>
<section class="card logc"><h2>Log</h2><pre id="log"></pre></section></div>
</main>
<script>
const $=id=>document.getElementById(id);let showCams=true;try{showCams=localStorage.getItem('cams')!=='0'}catch(e){}
function cams(){showCams=!showCams;try{localStorage.setItem('cams',showCams?'1':'0')}catch(e){};$('cams').classList.toggle('hidden',!showCams)}
$('cams').classList.toggle('hidden',!showCams);
async function setAgar(v){await fetch('/agar/'+(v||'auto'),{method:'POST'});poll()}
async function act(a){await fetch('/action/'+a,{method:'POST'});poll()}
function plate(s){const d=s.demo||{},W=s.wells||{},st={};(d.colonies||[]).forEach(c=>st[c.well]=c.state);(W.targets||[]).forEach(w=>{if(!(w in st))st[w]='t'});
let h='<div></div>';for(const r of 'ABCDEFGH')h+='<div class="h">'+r+'</div>';
for(let c=12;c>=1;c--){h+='<div class="h">'+c+'</div>';for(const r of 'ABCDEFGH'){const w=r+c;let k='';
if(w===W.disinfect)k='dis';else if(w===W.rinse)k='rin';else if(w===d.target_well)k='now';else if(st[w]==='done')k='done';else if(st[w]==='failed')k='failed';else if(w===d.next_well)k='next';else if(st[w])k='t';
h+='<div class="w '+k+'" title="'+w+'">'+(k?w:'')+'</div>'}}$('plate').innerHTML=h}
let agarShown=false;
async function poll(){try{const s=await (await fetch('/state')).json();if(!agarShown){$('agar').value=s.agar||'';agarShown=true}$('pill').textContent=s.running?('running: '+s.title):('idle'+(s.exit!==null?' (last exit '+s.exit+')':''));
$('pill').className='pill'+(s.running?' run':'');const d=s.demo;$('line').textContent=d?d.line:'Press Dry run or Run demo: SAM 3.1 detects the colonies on a fresh frame.';$('when').textContent=d&&d.when?('updated '+d.when):'';plate(s);choices(d);
const L=$('log');const atEnd=L.scrollTop+L.clientHeight>=L.scrollHeight-20;L.textContent=s.log.join('\n');if(atEnd)L.scrollTop=L.scrollHeight}catch(e){$('pill').textContent='server not reachable'}}
let shown='';function choices(d){const C=d&&d.choices&&!(d.colonies||[]).length?d.choices:null;const el=$('choices');
if(!C){el.style.display='none';$('dish').style.display='';shown='';return}
const key=JSON.stringify(C);if(key===shown)return;shown=key;el.style.display='grid';$('dish').style.display='none';
el.innerHTML=['strict','normal','loose'].map(n=>'<div class="choice"><h3>'+n+' &middot; '+C[n].length+' colonies</h3><img src="/img/choice-'+n+'?'+Date.now()+'"><button class="primary" data-v="'+n+'" onclick="choose(this.dataset.v)">Use this</button></div>').join('')}
async function choose(n){await fetch('/choose/'+n,{method:'POST'});document.querySelectorAll('.choice').forEach(c=>{const on=c.querySelector('button').dataset.v===n;
c.classList.toggle('picked',on);c.querySelector('button').textContent=on?'Chosen':'Use this'});poll()}
function imgs(){const t=Date.now();$('dish').src='/img/dish?'+t;if(showCams){$('top').src='/img/top?'+t;$('side').src='/img/side?'+t}}
setInterval(poll,1000);setInterval(imgs,500);poll();imgs();
</script></body></html>"""


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
            return self.send(PAGE.encode(), 'text/html; charset=utf-8')
        if self.path == '/state':
            try:
                agar = open(f'{S}/agar_mm.txt').read().strip()
            except OSError:
                agar = ''
            return self.send(json.dumps({**state(), 'agar': agar}).encode(), 'application/json')
        if self.path.startswith('/img/'):
            name = self.path[5:].split('?')[0]
            img = (overhead() if name == 'top' else cv2.imread(f'{S}/live_{SIDE}.jpg') if name == 'side'
                   else dish() if name == 'dish' else dish(name[7:]) if name.startswith('choice-') else None)
            if img is None:
                img = np.full((270, 480, 3), 18, np.uint8)
                cv2.putText(img, 'no image yet', (150, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (120, 120, 120), 2)
            return self.send(jpeg(img), 'image/jpeg')
        self.send(b'not found', 'text/plain', 404)

    def do_POST(self):
        name = self.path.rsplit('/', 1)[-1]
        if self.path.startswith('/choose/') and name in ('strict', 'normal', 'loose'):
            json.dump({'variant': name}, open(f'{S}/colony_choice.json', 'w'))
            running = job['p'] is not None and job['p'].poll() is None and job['title'] == 'demo'
            log.append(f'-- detection chosen: {name}' + ('' if running else ' (dry run: shown only; Run demo asks again)'))
            return self.send(b'ok', 'text/plain')
        if self.path.startswith('/agar/'):  # agar surface over the paper, mm (mat 1 + floor 1 + agar); empty = measure
            try:
                v = '' if name in ('', 'auto') else f'{float(name):.1f}'
                assert v == '' or 3.0 <= float(v) <= 12.0
            except (ValueError, AssertionError):
                return self.send(b'bad agar height', 'text/plain', 400)
            open(f'{S}/agar_mm.txt', 'w').write(v)
            log.append(f'-- agar height: {v + " mm over the paper" if v else "measured by the side camera"}')
            return self.send(b'ok', 'text/plain')
        if self.path.startswith('/action/') and name in ACTIONS:
            ACTIONS[name]()
            return self.send(b'ok', 'text/plain')
        self.send(b'not found', 'text/plain', 404)


def sam_on_call():
    import psutil
    try:
        return psutil.pid_exists(int(open(f'{S}/sam_ready').read()))
    except (OSError, ValueError):
        return False


def sam_worker():
    """SAM 3.1 loaded once when the page starts and kept (user, 11-03): colony_cycle writes sam_request, this answers
    with colony_variants.json from the background frame st2_top.jpg, in seconds instead of a model load per run."""
    import time
    import colony_detect as cd
    import sam_detect
    if sam_on_call():  # tools/sam_server.py already answers sam_request: a second model would race it
        log.append('-- SAM 3.1 on call already')
        return
    try:
        pred = sam_detect.load()
    except Exception as e:  # the cycle then runs SAM itself
        log.append(f'-- SAM 3.1 did not load: {e}')
        return
    open(f'{S}/sam_ready', 'w').write(str(os.getpid()))
    log.append('-- SAM 3.1 loaded, ready')
    while True:
        if os.path.exists(f'{S}/sam_request'):
            os.remove(f'{S}/sam_request')
            try:
                picks = cd.variant_picks(pred, cv2.imread(f'{S}/st2_top.jpg'))
                json.dump(picks, open(f'{S}/colony_variants.tmp', 'w'), indent=1)
                os.replace(f'{S}/colony_variants.tmp', f'{S}/colony_variants.json')
                log.append('-- SAM: ' + ', '.join(f'{n} {len(c)}' for n, c in picks.items()))
            except Exception as e:
                log.append(f'-- SAM failed: {e}')
        time.sleep(0.2)


if __name__ == '__main__':
    print(f'demo page on http://127.0.0.1:{PORT}', flush=True)
    for f in ('demo_state.json', 'colony_choice.json', *(() if sam_on_call() else ('sam_ready',)), 'sam_request'):  # a fresh page: no colonies or wells left over from a test run
        try:
            os.remove(f'{S}/{f}')
        except OSError:
            pass
    threading.Thread(target=preview, daemon=True).start()
    threading.Thread(target=sam_worker, daemon=True).start()
    ThreadingHTTPServer(('127.0.0.1', PORT), H).serve_forever()
