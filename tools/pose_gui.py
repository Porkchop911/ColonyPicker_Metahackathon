#!/usr/bin/env python3
"""A small window to drive the arm by absolute servo values: one field per servo (number and name), Go, the live
read-back, and Save / Photo for teaching poses with the arm powered (where it holds itself: limp hand-taught poses
replayed 10-20 mm off, 2026-10-31).

Go moves through well_reach.jgo: <= 6 pulses per joint per command at 25 ms per pulse, servos 1-4 only, 0-1000.
Roll (5) and gripper (10) are shown, never commanded. Saved poses: evidence/tip/powered_poses.json.

    <venv python> tools/pose_gui.py
"""
from __future__ import annotations

import json
import sys
import threading
import tkinter as tk
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import well_reach as wr  # noqa: E402

SERVOS = [(1, "pan"), (2, "shoulder"), (3, "elbow"), (4, "wrist")]
SHOWN = [(5, "roll"), (10, "gripper")]
POSES = Path("evidence/tip/powered_poses.json")
AWAY = 8         # pulses above the target every changed servo comes from
OFF_PULSES = 6   # read-back further than this is shown as not done (the shoulder reads ~5 under where it holds)


class App:
    def __init__(self, root: tk.Tk, servos):
        self.root, self.servos, self.busy, self.done = root, servos, False, None
        r = servos.read()
        self.q = {k: min(max(r[k], 0), 1000) for k, _ in SERVOS}  # hold where it is (a read past an end clamps)
        if hasattr(servos, "td"):
            servos.td._sent.update(self.q)
        servos.send(self.q, 1000)
        root.title("Arm pose")
        for c, text in enumerate(("Servo", "Name", "Target", "Commanded", "Read-back")):
            tk.Label(root, text=text, font=("Segoe UI", 10, "bold")).grid(row=0, column=c, padx=6, pady=4)
        self.fields, self.cmd, self.back = {}, {}, {}
        for i, (k, name) in enumerate(SERVOS + SHOWN, 1):
            tk.Label(root, text=str(k)).grid(row=i, column=0)
            tk.Label(root, text=name).grid(row=i, column=1, sticky="w")
            if k in self.q:
                e = tk.Entry(root, width=7, justify="right")
                e.insert(0, str(self.q[k]))
                e.grid(row=i, column=2, padx=6)
                e.bind("<Return>", lambda _: self.go())
                self.fields[k] = e
                self.cmd[k] = tk.Label(root, width=6, anchor="e")
                self.cmd[k].grid(row=i, column=3)
            self.back[k] = tk.Label(root, width=6, anchor="e")
            self.back[k].grid(row=i, column=4)
        n = len(SERVOS + SHOWN) + 1
        self.go_btn = tk.Button(root, text="Go", width=10, command=self.go)
        self.go_btn.grid(row=n, column=2, columnspan=2, pady=6)
        tk.Label(root, text="Name").grid(row=n + 1, column=0)
        self.name = tk.Entry(root, width=8)
        self.name.grid(row=n + 1, column=1)
        tk.Button(root, text="Save pose", command=self.save).grid(row=n + 1, column=2)
        tk.Button(root, text="Photo", command=self.photo).grid(row=n + 1, column=3)
        self.status = tk.Label(root, text="holding", anchor="w", fg="gray30")
        self.status.grid(row=n + 2, column=0, columnspan=5, sticky="we", padx=6, pady=4)
        self.cams = None
        self.refresh()
        self.poll()

    def refresh(self) -> None:
        r = self.servos.read()
        for k, lab in self.back.items():
            lab.config(text=str(r[k]))
        for k, lab in self.cmd.items():
            lab.config(text=str(self.q[k]))

    def go(self) -> None:
        if self.busy:
            return
        try:
            target = {k: int(e.get()) for k, e in self.fields.items()}
        except ValueError:
            self.status.config(text="every field needs a whole number", fg="red")
            return
        bad = {k: v for k, v in target.items() if not 0 <= v <= 1000}
        if bad:
            self.status.config(text=f"outside 0-1000: {bad}", fg="red")
            return
        self.busy, self.done = True, None
        self.go_btn.config(state="disabled")
        self.status.config(text=f"moving to {list(target.values())} ...", fg="blue")

        def work():
            try:
                # changed servos end coming from AWAY pulses above: a small change alone often left a loaded servo
                # where it was (dead band, 2026-10-31), and one approach side keeps the backlash the same each time
                via = {k: min(v + AWAY, 1000) if v != self.q[k] else v for k, v in target.items()}
                self.q = wr.jgo(self.servos, via, self.q)
                self.q = wr.jgo(self.servos, target, self.q)
                self.servos.dwell(0.6)
                r = self.servos.read()
                off = {k: r[k] for k in target if abs(r[k] - target[k]) > OFF_PULSES}
                self.done = "at target" if not off else f"not done: read-back {off} (commanded {target})"
            except Exception as e:  # shown, not raised: the window stays usable
                self.done = f"not done: {e}"
        threading.Thread(target=work, daemon=True).start()

    def poll(self) -> None:
        """GUI thread: finish a move once the worker is done; refresh the read-back every ~2 s while idle."""
        if self.busy and self.done is not None:
            self.busy = False
            self.go_btn.config(state="normal")
            self.status.config(text=self.done, fg="red" if self.done.startswith("not") else "green")
            self.refresh()
        elif not self.busy:
            self.ticks = getattr(self, "ticks", 0) + 1
            if self.ticks % 10 == 0:
                self.refresh()
        self.root.after(200, self.poll)

    def save(self) -> None:
        name = self.name.get().strip()
        if not name or self.busy:
            self.status.config(text="type a name (and wait for the move)", fg="red")
            return
        poses = json.loads(POSES.read_text()) if POSES.exists() else {}
        poses[name] = {"cmd": self.q, "read": self.servos.read()}
        POSES.write_text(json.dumps(poses, indent=1))
        self.status.config(text=f"saved {name}: {list(self.q.values())}", fg="green")

    def photo(self) -> None:
        name = self.name.get().strip() or "pose"
        import cv2
        from rig_io import OVERHEAD, SIDE, SIDE_PAPER, Camera
        self.cams = self.cams or (Camera(OVERHEAD), Camera(SIDE, paper_target=SIDE_PAPER))
        for cam, view in zip(self.cams, ("top", "side")):
            cv2.imwrite(f"evidence/tip/teach_{name}.{view}.jpg", cam.frame())
        self.status.config(text=f"photos evidence/tip/teach_{name}.top.jpg / .side.jpg", fg="green")


def main(servos=None) -> tk.Tk:
    root = tk.Tk()
    App(root, servos or wr.Servos())
    return root


if __name__ == "__main__":
    main().mainloop()
