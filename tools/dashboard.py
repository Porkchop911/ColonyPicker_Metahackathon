#!/usr/bin/env python3
"""Demo dashboard: every camera live with SAM 3.1 detections, click a dot to target it, run the touch.

    <hackathon venv python> tools/dashboard.py [--port 8000] [--lan]      then open http://localhost:8000

Overhead (4K UGREEN): black dots, red dots, cube, teal tip. Side UGREEN and arm camera: the needle tip.
Clicking the overhead view picks the nearest detected dot as the target. While model_jab runs it needs the
cameras, so the dashboard closes them and shows the frames the run saves (ug_*.jpg, side_*.jpg, NNN_arm.png)
with its log; afterwards it reopens them. Stop writes a stop file: model_jab aborts at its next move and lifts the elbow.
Standard library HTTP server, no extra dependencies.
"""
from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import markers as mk  # noqa: E402
import model_jab as mj  # noqa: E402
import sam_detect as sam  # noqa: E402
from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera, reset_bridge, servo  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
RUNS = ROOT / "evidence" / "descend"
PREFLIGHT_LOG = RUNS / "preflight.txt"
STOP = RUNS / "stop.flag"
CAMS = ("overhead", "side", "arm")
SNAP_PX = 40  # a click this close to a detected dot picks that dot
YELLOW, MAGENTA, RED, GREEN, CYAN = (0, 220, 255), (255, 0, 255), (0, 0, 255), (0, 200, 0), (255, 200, 0)


class Rig:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.sam_lock = threading.Lock()
        self.cams: dict[str, Camera | None] = {"overhead": None, "side": None}
        self.raw: dict[str, np.ndarray | None] = {c: None for c in CAMS}
        self.jpeg: dict[str, bytes] = {}
        self.dets: dict[str, dict] = {c: {} for c in CAMS}
        self.sam_s: dict[str, float] = {}
        self.err: dict[str, str] = {}  # the last grab error per panel, drawn on it
        self.arm_misses = 0
        self.target: dict | None = None
        self.proc: subprocess.Popen | None = None
        self.action = ""
        self.run_dir: Path | None = None
        self.started = 0.0
        self.predictor = None

    # cameras ------------------------------------------------------------------------------------------
    def open_cams(self) -> None:
        for name, index in (("overhead", OVERHEAD), ("side", SIDE)):
            try:
                self.cams[name] = Camera(index, paper_target=SIDE_PAPER if index == SIDE else None)
            except RuntimeError as error:
                print(f"{name} camera: {error}", flush=True)

    def close_cams(self) -> None:
        for name, cam in self.cams.items():
            if cam is not None:
                cam.close()
            self.cams[name] = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    # frames -------------------------------------------------------------------------------------------
    def grab(self, name: str) -> np.ndarray | None:
        if self.running() and self.action == "run":
            return self.run_frame(name)
        if name == "arm":  # a plain snap: tt.snap_arm restarts the camera driver on a stall, not the dashboard's call
            try:
                jpeg = base64.b64decode(servo({"snap": True})[0]["jpeg"])
                self.arm_misses = 0
                return cv2.imdecode(np.frombuffer(jpeg, np.uint8), 1)
            except Exception as error:  # robot off, bridge down or a stale frame: the panel says so
                self.err["arm"] = repr(error)[:90]
                self.arm_misses += 1
                if self.arm_misses % 20 == 0:  # ~5 s: some bridge sessions never see the camera (tip_triangulate.snap_arm)
                    reset_bridge()
                return None
        cam = self.cams[name]
        return None if cam is None else cam.frame(average=1, settle=0.0)

    def run_frame(self, name: str) -> np.ndarray | None:
        if self.run_dir is None:
            news = [d for d in RUNS.glob("2026*") if d.is_dir() and d.stat().st_mtime >= self.started - 1]
            self.run_dir = max(news, key=lambda d: d.name) if news else None
            if self.run_dir is None:
                return None
        pattern = {"overhead": "ug_*.jpg", "side": "side_*.jpg", "arm": "[0-9][0-9][0-9]_arm.png"}[name]
        files = sorted(self.run_dir.glob(pattern), key=lambda p: p.stat().st_mtime)
        return cv2.imread(str(files[-1])) if files else None

    # detections ---------------------------------------------------------------------------------------
    def find(self, img: np.ndarray, prompt: str, thresh: float) -> list[dict]:
        with self.sam_lock:
            return sam.detect(self.predictor, [Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))], prompt, thresh)

    def detect(self, name: str, img: np.ndarray) -> dict:
        t0 = time.monotonic()
        if name == "overhead":
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            paper = float(np.median(gray[gray > np.percentile(gray, 40)]))
            keep = lambda found, area, dark: [d["centroid"] for d in found if d["centroid"] is not None  # noqa: E731
                                             and area[0] <= d["area"] <= area[1] and gray[d["mask"]].min() <= dark * paper]
            red = keep(self.find(img, "small red dot", mj.SAM_THRESHOLD), mj.RED_AREA, 1.0)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # "dot" minus teal ink, as model_jab (small black dot went blind at dusk)
            dots = [d for d in self.find(img, "dot", 0.3) if not (45 <= np.median(hsv[..., 0][d["mask"]]) <= 100
                                                                and np.median(hsv[..., 1][d["mask"]]) > 35)]
            black = [q for q in keep(dots, mj.SAM_AREA, mj.SAM_BLACK_FRAC)
                     if all(np.hypot(q[0] - r[0], q[1] - r[1]) > 6 for r in red)]
            cube = [d["box"] for d in self.find(img, "red cube", 0.4) if d["area"] >= 500][:1]
            out = {"black": black, "red": red, "cube": cube, "tip": teal_tip(img)}
        else:  # the needle tip: lowest end in the side view, topmost in the arm view (model_jab.sam_needle_tip)
            found = self.find(img, "thin metal needle", 0.15)
            tip = None
            if found:
                vs, us = np.nonzero(max(found, key=lambda d: d["score"])["mask"])
                k = int(np.argmin(vs) if name == "arm" else np.argmax(vs))
                tip = (float(us[k]), float(vs[k]))
            out = {"tip": tip}
        out["markers"] = mk.detect(img)  # placement check: green outline + id per ArUco marker
        self.sam_s[name] = time.monotonic() - t0
        return out

    def draw(self, name: str, img: np.ndarray) -> np.ndarray:
        im = img.copy()
        d = self.dets.get(name, {}) if not (self.running() and self.action == "run") else self.run_marks(name)
        for q in d.get("black", []):
            cv2.circle(im, (int(q[0]), int(q[1])), 14, YELLOW, 2)
        for q in d.get("red", []):
            cv2.circle(im, (int(q[0]), int(q[1])), 12, MAGENTA, 2)
        for b in d.get("cube", []):
            cv2.rectangle(im, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), CYAN, 2)
            cv2.putText(im, "cube", (int(b[0]), int(b[1]) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.8, CYAN, 2)
        for mid, c in d.get("markers", {}).items():
            cv2.polylines(im, [c.astype(np.int32)], True, GREEN, 3)
            cv2.putText(im, f"id {mid}", (int(c[0][0]), int(c[0][1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.9, GREEN, 2)
        if d.get("tip") is not None:
            u, v = (int(x) for x in d["tip"])
            cv2.drawMarker(im, (u, v), RED, cv2.MARKER_CROSS, 34, 3)
        if name == "overhead" and self.target:
            u, v = (int(x) for x in self.target["px"])
            cv2.circle(im, (u, v), 26, GREEN, 4)
            cv2.putText(im, f"target ({self.target['kind']})", (u + 30, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, GREEN, 3)
        label = f"{name}  " + ("run frames" if self.running() and self.action == "run" else f"SAM {self.sam_s.get(name, 0):.1f} s")
        cv2.putText(im, label, (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5)
        cv2.putText(im, label, (14, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
        return im

    def run_marks(self, name: str) -> dict:
        """During a run: the target and the last tip the run logged."""
        rows = self.log_rows()
        out: dict = {}
        if name == "overhead":
            fr = next((r for r in reversed(rows) if r["phase"] == "frames"), None)
            if fr:
                out["black"] = fr["ugreen_black_px"]
            tip = next((r.get("tip_px") for r in reversed(rows) if r.get("tip_px") is not None), None)
            out["tip"] = tip
        if name == "side":
            out["tip"] = next((r["side_tip_px"] for r in reversed(rows) if r.get("side_tip_px") is not None), None)
        return out

    def log_rows(self) -> list[dict]:
        try:
            return json.loads((self.run_dir / "log.json").read_text()) if self.run_dir else []
        except (OSError, json.JSONDecodeError):  # mid-write
            return []

    # actions ------------------------------------------------------------------------------------------
    def start(self, action: str) -> str:
        if self.running():
            return f"busy: {self.action}"
        py = sys.executable
        if action == "run":
            STOP.unlink(missing_ok=True)
            args = [py, "tools/model_jab.py", "--out", str(RUNS), "--lam", "190", "--jab", "--stop-file", str(STOP)]
            if self.target:
                args += ["--red" if self.target["kind"] == "red" else "--dot", *(f"{x:.1f}" for x in self.target["px"])]
            self.close_cams()  # model_jab opens both UGREENs itself
        elif action == "start":
            args = [py, "tools/arm_to_start.py", str(RUNS)]
        elif action == "park":
            args = [py, "tools/arm_park.py", str(RUNS)]
        elif action == "preflight":
            args = [py, "tools/preflight.py", "--sam"]
            self.close_cams()  # it opens both UGREENs itself
        else:
            return f"unknown action {action}"
        self.action, self.run_dir, self.started = action, None, time.time()
        self.proc = subprocess.Popen(args, cwd=ROOT, stdout=PREFLIGHT_LOG.open("w") if action == "preflight" else None,
                                     stderr=subprocess.STDOUT if action == "preflight" else None)
        threading.Thread(target=self.wait, daemon=True).start()
        return f"{action} started"

    def wait(self) -> None:
        self.proc.wait()
        print(f"{self.action} finished: exit {self.proc.returncode}", flush=True)
        if self.action in ("run", "preflight"):
            self.open_cams()

    def stop(self) -> str:
        if not self.running():
            return "nothing running"
        if self.action != "run":
            return f"{self.action} cannot be stopped midway (it ends by itself)"
        STOP.write_text("stop")  # model_jab checks it before every sleep (CTRL_BREAK needs a shared console)
        return "stop sent: the run aborts at its next move and lifts the elbow"

    def set_target(self, x: float, y: float) -> dict | None:
        px = (x * 1920, y * 1080)
        d = self.dets["overhead"]
        cands = [("black", q) for q in d.get("black", [])] + [("red", q) for q in d.get("red", [])]
        kind, q = min(cands, key=lambda c: np.hypot(c[1][0] - px[0], c[1][1] - px[1]), default=(None, None))
        if q is not None and np.hypot(q[0] - px[0], q[1] - px[1]) <= SNAP_PX:
            self.target = {"kind": kind, "px": [float(q[0]), float(q[1])]}
        else:  # no dot there: model_jab --dot targets the black dot nearest this pixel
            self.target = {"kind": "black", "px": [px[0], px[1]]}
        return self.target


def teal_tip(img: np.ndarray):
    """The inked tip's lowest pixel (model_jab.teal_tip without the old-ink mask and SAM fallback)."""
    m = cv2.inRange(cv2.cvtColor(img, cv2.COLOR_BGR2HSV), (45, 35, 50), (100, 255, 255))
    count, lab, st, _ = cv2.connectedComponentsWithStats(m)
    best = None
    for j in range(1, count):
        if not 30 <= st[j, 4] < 5000 or st[j, 1] + st[j, 3] >= 0.9 * img.shape[0]:
            continue
        (_, _), (w, h), _ = cv2.minAreaRect(np.column_stack(np.nonzero(lab == j)[::-1]).astype(np.float32))
        if max(w, h) >= 2 * max(min(w, h), 1.0) and (best is None or st[j, 1] + st[j, 3] > st[best, 1] + st[best, 3]):
            best = j
    if best is None:
        return None
    vs, us = np.nonzero(lab == best)
    return (float(us[vs == vs.max()].mean()), float(vs.max()))


def frame_loop(rig: Rig, name: str) -> None:
    while True:
        try:
            img = rig.grab(name)
        except Exception as error:  # a camera closed under it (a run starting) or a read timed out: keep the panel alive
            rig.err[name] = repr(error)[:90]
            img = None
        if img is None and rig.raw.get(name) is not None:  # a gap (a run starting, a slow snap): hold the last frame
            img = rig.draw(name, rig.raw[name])
            cv2.putText(img, "waiting for a new frame", (14, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 200, 255), 3)
            time.sleep(0.3)
        elif img is None:
            img = np.full((540, 960, 3), 40, np.uint8)
            cv2.putText(img, f"{name}: no frame yet", (30, 280), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (200, 200, 200), 2)
            cv2.putText(img, rig.err.get(name, ""), (30, 330), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 160, 255), 1)
            time.sleep(0.3)
        else:
            rig.raw[name] = img
            img = rig.draw(name, img)
        small = cv2.resize(img, (960, int(960 * img.shape[0] / img.shape[1])))
        rig.jpeg[name] = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
        time.sleep(0.08 if name != "arm" else 0.25)


def detect_loop(rig: Rig) -> None:
    while True:
        if rig.running() and rig.action == "run":
            time.sleep(0.5)
            continue
        for name in CAMS:
            img = rig.raw.get(name)
            if img is not None:
                try:
                    rig.dets[name] = rig.detect(name, img)
                except Exception as error:  # a bad frame must not end the loop
                    print(f"detect {name}: {error!r}", flush=True)
        time.sleep(0.05)


PAGE = """<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Colony Picker</title><style>
:root{--bg:#f5f5f2;--fg:#1d1d1b;--panel:#fff;--line:#d8d8d2;--accent:#0a7a3d;--warn:#b42318}
@media (prefers-color-scheme:dark){:root{--bg:#121211;--fg:#ecece8;--panel:#1d1d1b;--line:#33332f;--accent:#3ccf7a;--warn:#ff6b5b}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:15px/1.4 system-ui,sans-serif}
header{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:12px 16px;border-bottom:1px solid var(--line)}
h1{font-size:18px;margin:0 12px 0 0}button{font:inherit;padding:8px 14px;border-radius:8px;border:1px solid var(--line);background:var(--panel);color:var(--fg);cursor:pointer}
button.go{background:var(--accent);color:#fff;border-color:var(--accent)}button.stop{background:var(--warn);color:#fff;border-color:var(--warn)}
#status{margin-left:auto;font-variant-numeric:tabular-nums}
main{display:grid;grid-template-columns:2fr 1fr;gap:12px;padding:12px 16px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:10px;overflow:hidden}
.panel h2{font-size:13px;margin:0;padding:6px 10px;border-bottom:1px solid var(--line);font-weight:600}
img{display:block;width:100%;height:auto}#overhead{cursor:crosshair}
.side{display:grid;gap:12px;align-content:start}
#log{font:12px/1.5 ui-monospace,monospace;padding:8px 10px;max-height:260px;overflow:auto;white-space:pre-wrap}
@media (max-width:800px){main{grid-template-columns:1fr}}
</style></head><body>
<header><h1>Colony Picker</h1>
<button onclick="act('start')">Arm to start</button>
<button class="go" onclick="act('run')">Run touch</button>
<button class="stop" onclick="act('stop')">Stop</button>
<button onclick="act('park')">Park</button>
<button onclick="act('preflight')">Preflight</button>
<span id="target">target: nearest dot to the landing point (click the overhead view to choose)</span>
<span id="status">…</span></header>
<main><div class="panel"><h2>Overhead (4K): black dots yellow, red dots magenta, cube cyan, tip red cross, target green</h2>
<img id="overhead" src="/stream/overhead" alt="overhead camera"></div>
<div class="side"><div class="panel"><h2>Side camera: needle tip</h2><img src="/stream/side" alt="side camera"></div>
<div class="panel"><h2>Arm camera: needle tip</h2><img src="/stream/arm" alt="arm camera"></div>
<div class="panel"><h2>Run log</h2><div id="log"></div></div></div></main>
<script>
async function act(a){const r=await fetch('/action/'+a,{method:'POST'});document.getElementById('status').textContent=await r.text()}
document.getElementById('overhead').addEventListener('click',async e=>{const im=e.target;
 const r=await fetch('/target',{method:'POST',body:JSON.stringify({x:e.offsetX/im.clientWidth,y:e.offsetY/im.clientHeight})});
 const t=await r.json();document.getElementById('target').textContent='target: '+t.kind+' dot at ('+t.px.map(v=>v.toFixed(0)).join(', ')+')'});
async function poll(){try{const s=await (await fetch('/state')).json();
 document.getElementById('status').textContent=s.running?(s.action+' running '+s.elapsed+' s'):'idle';
 document.getElementById('log').textContent=s.rows.join('\\n')}catch(e){}setTimeout(poll,1000)}poll();
</script></body></html>"""


def row_text(r: dict) -> str:
    keep = ("matched", "it", "round", "height_mm", "tri_height_mm", "dot_minus_landing_mm", "fix_mm", "by", "miss_mm",
            "markers", "rms_mm", "dots", "when", "across_ug_mm", "moved_mm", "residual_mm", "closed_loop", "error")
    parts = []
    for k in keep:
        if k in r and r[k] is not None:
            v = r[k]
            v = [round(x, 1) for x in v] if isinstance(v, list) and v and isinstance(v[0], float) else (round(v, 1) if isinstance(v, float) else v)
            parts.append(f"{k}={v}")
    return f"{r['phase']:<18} " + " ".join(parts)


def handler(rig: Rig):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *_):  # quiet
            pass

        def send(self, body: bytes, kind: str = "text/plain", code: int = 200) -> None:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                return self.send(PAGE.encode(), "text/html; charset=utf-8")
            if self.path == "/state":
                if rig.action == "preflight" and PREFLIGHT_LOG.exists():  # its checklist, OK/FAIL per line
                    rows = [ln for ln in PREFLIGHT_LOG.read_text(errors="replace").splitlines()
                            if ln.startswith(("OK", "FAIL")) or "OK;" in ln or ln.endswith(" OK")][-16:]
                else:
                    rows = [row_text(r) for r in rig.log_rows()[-14:]] if rig.action == "run" else []
                return self.send(json.dumps({"running": rig.running(), "action": rig.action, "rows": rows,
                                             "elapsed": round(time.time() - rig.started) if rig.started else 0,
                                             "target": rig.target}).encode(), "application/json")
            if self.path.startswith("/stream/") and self.path[8:] in CAMS:
                name = self.path[8:]
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        jpg = rig.jpeg.get(name)
                        if jpg:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    return
            self.send(b"not found", code=404)

        def do_POST(self):
            body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
            if self.path == "/target":
                p = json.loads(body or b"{}")
                return self.send(json.dumps(rig.set_target(float(p["x"]), float(p["y"]))).encode(), "application/json")
            if self.path.startswith("/action/"):
                a = self.path[8:]
                return self.send((rig.stop() if a == "stop" else rig.start(a)).encode())
            self.send(b"not found", code=404)
    return H


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--lan", action="store_true", help="listen on all interfaces (show it on another device)")
    args = ap.parse_args()
    rig = Rig()
    print("loading SAM 3.1 and opening the cameras ...", flush=True)
    loader = threading.Thread(target=lambda: setattr(rig, "predictor", sam.load()), daemon=True)
    loader.start()
    rig.open_cams()
    for name in CAMS:
        threading.Thread(target=frame_loop, args=(rig, name), daemon=True).start()
    loader.join()
    threading.Thread(target=detect_loop, args=(rig,), daemon=True).start()
    host = "0.0.0.0" if args.lan else "127.0.0.1"
    print(f"dashboard on http://{'localhost' if not args.lan else host}:{args.port}", flush=True)
    ThreadingHTTPServer((host, args.port), handler(rig)).serve_forever()


if __name__ == "__main__":
    main()
