#!/usr/bin/env python3
"""Philips Hue lamps over the bridge's local API (v1, plain HTTP on the LAN; no cloud).

    python tools/hue.py pair                 # press the bridge's link button first, then run within 30 s
    python tools/hue.py list                 # lights: id, name, on, brightness, reachable
    python tools/hue.py set 3 on --bri 254 --ct 250
    python tools/hue.py set 3 off

Bridge: 192.168.0.70 (BSB002, found by its ec:b5:fa MAC in the ARP table, 2026-10-26). The app key
lives in config/hue.local.json, kept out of git (.gitignore).
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

BRIDGE = "192.168.0.70"
KEY_FILE = Path(__file__).resolve().parent.parent / "config" / "hue.local.json"


def _call(method: str, path: str, body: dict | None = None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(f"http://{BRIDGE}/api{path}", data=data, method=method,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def _key() -> str:
    if not KEY_FILE.exists():
        raise SystemExit("not paired: press the bridge link button, then run `python tools/hue.py pair`")
    return json.loads(KEY_FILE.read_text())["username"]


def pair() -> None:
    reply = _call("POST", "", {"devicetype": "hwr_colony#pc", "generateclientkey": True})[0]
    if "error" in reply:
        raise SystemExit(f"bridge: {reply['error']['description']} (press the link button, then retry within 30 s)")
    KEY_FILE.write_text(json.dumps(reply["success"]))
    print(f"paired; key saved to {KEY_FILE}")


def lights() -> dict:
    return _call("GET", f"/{_key()}/lights")


def set_light(light: str, on: bool, bri: int | None = None, ct: int | None = None, xy=None,
              transition_ds: int = 0) -> dict:
    """Switch one light; bri 1-254, ct 153 (cold) .. 500 (warm) mired, xy CIE pair. Reads back the
    state after the bridge applied it, so callers can confirm before taking a picture."""
    state = {"on": on, "transitiontime": transition_ds}
    if on and bri is not None:
        state["bri"] = int(bri)
    if on and ct is not None:
        state["ct"] = int(ct)
    if on and xy is not None:
        state["xy"] = list(xy)
    reply = _call("PUT", f"/{_key()}/lights/{light}/state", state)
    errors = [r["error"]["description"] for r in reply if "error" in r]
    if errors:
        raise RuntimeError(f"light {light}: {errors}")
    time.sleep(0.1 * transition_ds + 0.3)
    return _call("GET", f"/{_key()}/lights/{light}")["state"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("pair")
    sub.add_parser("list")
    s = sub.add_parser("set")
    s.add_argument("light")
    s.add_argument("onoff", choices=("on", "off"))
    s.add_argument("--bri", type=int)
    s.add_argument("--ct", type=int)
    args = ap.parse_args()
    if args.cmd == "pair":
        pair()
    elif args.cmd == "list":
        for lid, l in lights().items():
            st = l["state"]
            print(f"{lid:>3}  {l['name']:<28} {l['type']:<28} on={st['on']} bri={st.get('bri')} "
                  f"ct={st.get('ct')} reachable={st.get('reachable')}")
    else:
        print(set_light(args.light, args.onoff == "on", args.bri, args.ct))


if __name__ == "__main__":
    main()
