#!/usr/bin/env python3
"""PC-side I/O for the rig: the UGREEN overhead camera and the robot's servo bridge.

UGREEN: 1080p UVC webcam on the PC. Exposure is locked after auto exposure settles, so frames stay
comparable while the robot moves; a background thread keeps reading so `frame()` never returns a
buffered old image.

Servo bridge: one long-lived SSH session to `robot/servo_bridge.py` on the robot (JSON lines on
stdin/stdout). Keep the session: sshd drops rapid reconnects (MaxStartups).
"""
from __future__ import annotations

import json
import subprocess
import threading
import time

import cv2
import numpy as np

ROBOT = "ubuntu@192.168.0.223"
# the robot's platform environment (pre-existing control stack: ROS 2, servo driver) + our bridge
BRIDGE = "source ~/hwr/deploy/env.zsh; python3 ~/colony/servo_bridge.py"
EXPOSURE_TARGET = 140.0  # scene median the MSMF exposure lock aims for (ruling visible, lamps on)


class Camera:
    """UGREEN capture with a locked exposure and a draining reader thread."""

    def __init__(self, index: int = 0, settle_s: float = 3.0):
        self.cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():  # DirectShow can lose the device after a USB re-enumeration
            self.cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
        if not self.cap.isOpened():
            raise RuntimeError(f"camera {index} does not open (DSHOW or MSMF)")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        end = time.monotonic() + settle_s
        while time.monotonic() < end:
            self.cap.read()
        self._lock_exposure()
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self._frames: list[tuple[float, np.ndarray]] = []
        threading.Thread(target=self._reader, daemon=True).start()

    def _lock_exposure(self) -> None:
        if self.cap.getBackendName() != "MSMF":
            value = self.cap.get(cv2.CAP_PROP_EXPOSURE)  # DSHOW reports the auto value truthfully
        else:
            # MSMF always reports -6; pick the manual step by the brightness it actually gives
            scores = []
            for step in range(-3, -11, -1):
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
                self.cap.set(cv2.CAP_PROP_EXPOSURE, step)
                for _ in range(4):
                    ok, img = self.cap.read()
                scores.append((abs(np.median(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)) - EXPOSURE_TARGET), step))
            value = min(scores)[1]
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, value)

    def _reader(self) -> None:
        while True:
            ok, img = self.cap.read()
            if ok:
                self._frames = (self._frames + [(time.monotonic(), img)])[-8:]

    def frame(self, average: int = 3, settle: float = 0.3) -> np.ndarray:
        """Mean of `average` frames captured at least `settle` s after the call."""
        since = time.monotonic() + settle
        end = since + 5.0
        while time.monotonic() < end:
            fresh = [img for t, img in self._frames if t > since]
            if len(fresh) >= average:
                return (np.mean([f.astype(np.float32) for f in fresh[:average]], 0)).astype(np.uint8)
            time.sleep(0.02)
        raise RuntimeError("UGREEN stopped delivering frames")


def restart_arm_camera() -> None:
    """Kill the stalled Aurora driver (the platform launch respawns it after 8 s) and reset its USB
    function. The closed driver stalls every few minutes: topic up, no frames, no kernel error."""
    subprocess.run(["ssh", ROBOT, "pkill -TERM -f '[a]urora930_node'; sleep 0.5; sudo -n usbreset 3251:1930"],
                   timeout=30, capture_output=True)


_bridge: subprocess.Popen | None = None
_lock = threading.RLock()  # one SSH session, several threads (teleop: live view + mover)


def _line() -> dict:
    while True:  # ROS may print log lines on stdout: only JSON lines are replies
        text = _bridge.stdout.readline()
        if not text:
            raise RuntimeError("servo bridge session ended")
        if text.startswith("{"):
            return json.loads(text)


def reset_bridge() -> None:
    """Drop the bridge session; the next servo() call opens a fresh one. A session that was listening
    when the camera driver restarted keeps reporting the old frame as stale for good (live 2026-10-27:
    598 s), while a new session gets frames at once."""
    global _bridge
    if _bridge is not None:
        _bridge.kill()
        _bridge = None


def servo(*commands: dict) -> list[dict]:
    """Send bridge commands ({"set": {id: pulse}, "ms": n}, {"read": true}, {"snap": true}) in
    order over the one session; raises on any error reply. Thread-safe."""
    with _lock:
        return _servo(*commands)


def _servo(*commands: dict) -> list[dict]:
    global _bridge
    if _bridge is None or _bridge.poll() is not None:
        _bridge = subprocess.Popen(["ssh", ROBOT, f'zsh -c "{BRIDGE}"'], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        hello = _line()
        if not hello.get("ready"):
            raise RuntimeError(f"servo bridge not ready: {hello}")
    replies = []
    for command in commands:
        _bridge.stdin.write(json.dumps(command) + "\n")
        _bridge.stdin.flush()
        replies.append(_line())
    errors = [r["error"] for r in replies if "error" in r]
    if errors:
        raise RuntimeError(errors)
    return replies
