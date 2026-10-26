#!/usr/bin/env python3
"""Robot-side bridge: arm servo commands, joint readback and arm-camera snapshots as JSON lines.

Runs on the robot inside the platform's ROS 2 environment (the pre-existing control stack provides
the servo driver on `servo_cmd` / `joint_states` and the camera on `camera/color/image_raw`).
One command per stdin line, one JSON reply per line on stdout:

    {"set": {"1": 500, "10": 680}, "ms": 300}  -> {"ok": true}
    {"read": true}                              -> {"pulses": {"1": 500, ...}, "age": s}
    {"snap": true}                              -> {"jpeg": base64, "age": s}

Snapshots older than MAX_FRAME_AGE are refused: the camera can stall for ~10 s while the last frame
still sits here, and a stale frame passed off as new corrupts every measurement made from it.
"""
from __future__ import annotations

import base64
import json
import math
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from hwr_msgs.msg import ServoCommand  # platform message: ids[], positions[] (pulses), duration_ms
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, JointState

SERVO_IDS = {1, 2, 3, 4, 5, 10}  # pan, shoulder, elbow, wrist, roll, gripper
PULSE_RAD = math.radians(0.24)   # bus servo: 1000 pulses over 240 deg, 500 = joint zero
MAX_FRAME_AGE = 1.0


def main() -> None:
    rclpy.init()
    node = Node("colony_servo_bridge")
    publisher = node.create_publisher(ServoCommand, "servo_cmd", 10)
    state: dict = {}

    def on_joints(msg: JointState) -> None:
        state["joints"] = (time.monotonic(), {
            (int(name[-1]) if name.startswith("joint") else 10): round(500 + pos / PULSE_RAD)
            for name, pos in zip(msg.name, msg.position)})

    def on_image(msg: Image) -> None:
        state["image"] = (time.monotonic(), msg)

    node.create_subscription(JointState, "joint_states", on_joints, 10)
    # best effort: a reliable subscriber that falls behind gets throttled on the busy Jetson
    node.create_subscription(Image, "camera/color/image_raw", on_image,
                             QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT))
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # a command published before the driver has matched our publisher is lost: wait for both
    end = time.monotonic() + 10.0
    while ("joints" not in state or publisher.get_subscription_count() == 0) and time.monotonic() < end:
        time.sleep(0.1)
    print(json.dumps({"ready": "joints" in state and publisher.get_subscription_count() > 0,
                      "camera": "image" in state}), flush=True)

    for line in sys.stdin:
        try:
            cmd = json.loads(line)
            if "set" in cmd:
                targets = {int(k): int(v) for k, v in cmd["set"].items()}
                if not targets or any(k not in SERVO_IDS or not 0 <= v <= 1000 for k, v in targets.items()):
                    raise ValueError(f"bad targets {targets}")
                publisher.publish(ServoCommand(duration_ms=int(cmd.get("ms", 500)),
                                               ids=list(targets), positions=list(targets.values())))
                reply = {"ok": True}
            elif "read" in cmd:
                if "joints" not in state:
                    raise RuntimeError("no joint_states yet")
                stamp, pulses = state["joints"]
                reply = {"pulses": pulses, "age": round(time.monotonic() - stamp, 2)}
            elif "snap" in cmd:
                if "image" not in state:
                    raise RuntimeError("no camera frame yet")
                stamp, msg = state["image"]
                age = time.monotonic() - stamp
                if age > MAX_FRAME_AGE:
                    raise RuntimeError(f"stale camera frame ({age:.1f} s old)")
                img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
                if msg.encoding == "rgb8":
                    img = img[:, :, ::-1]
                ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                reply = {"jpeg": base64.b64encode(jpeg.tobytes()).decode("ascii"), "age": round(age, 2)}
            else:
                raise ValueError("expected set, read or snap")
        except Exception as error:  # report, keep serving
            reply = {"error": f"{type(error).__name__}: {error}"}
        print(json.dumps(reply), flush=True)


if __name__ == "__main__":
    main()
