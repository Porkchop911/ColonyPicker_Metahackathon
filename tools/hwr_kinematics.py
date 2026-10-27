# Vendored verbatim from Porkchop911/dozorin-private @ 62eb5b4, hwr_core/hwr_core/arm/kinematics.py
# (the pre-existing robot stack; arm model in plan/16 there). Do not edit here: change it upstream.
"""Arm FK/IK and eye-in-hand camera geometry in the URDF convention (plan/16 findings).

Joints 2-4 are flipped: URDF angle = (500 - pulse) * 0.24 deg; pan yaw = (pulse - 500) * 0.24 deg.
At zero the arm points straight up; joints 2-4 rotate about +Y, so a chain angle phi is measured
from vertical (phi = 90 deg -> horizontal forward). The camera rides on link4 along its axis.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

RAD_PER_PULSE = math.radians(0.24)


@dataclass(frozen=True)
class JointSolution:
    pan: float       # yaw, left positive
    shoulder: float  # URDF joint2
    elbow: float     # URDF joint3
    wrist: float     # URDF joint4
    roll: float = 0.0


def pulses_to_joints(pulses: dict) -> JointSolution:
    return JointSolution(pan=(pulses[1] - 500) * RAD_PER_PULSE,
                         shoulder=(500 - pulses[2]) * RAD_PER_PULSE,
                         elbow=(500 - pulses[3]) * RAD_PER_PULSE,
                         wrist=(500 - pulses[4]) * RAD_PER_PULSE,
                         roll=(pulses.get(5, 500) - 500) * RAD_PER_PULSE)


def joints_to_pulses(joints: JointSolution) -> dict:
    pulses = {1: 500 + joints.pan / RAD_PER_PULSE,
              2: 500 - joints.shoulder / RAD_PER_PULSE,
              3: 500 - joints.elbow / RAD_PER_PULSE,
              4: 500 - joints.wrist / RAD_PER_PULSE,
              5: 500 + joints.roll / RAD_PER_PULSE}
    out = {sid: round(value) for sid, value in pulses.items()}
    if any(not 0 <= value <= 1000 for value in out.values()):
        raise ValueError("joint angle is outside servo pulse range")
    return out


# Sim model (rosorin_pro_isaac.urdf): joint1 turns about -Z, so it is -pan; joints 2-4 match.
# FK agrees with ArmKinematics.fk to 2 mm (the vendor's 0.5 deg joint1 tilt).
# shortcut: joint5 = +roll sign not yet checked on hardware; verify with one wrist-roll photo.
def joints_to_urdf(joints: JointSolution) -> dict:
    return {"joint1": -joints.pan, "joint2": joints.shoulder, "joint3": joints.elbow,
            "joint4": joints.wrist, "joint5": joints.roll}


def urdf_to_joints(q: dict) -> JointSolution:
    return JointSolution(pan=-q["joint1"], shoulder=q["joint2"], elbow=q["joint3"],
                         wrist=q["joint4"], roll=q["joint5"])


# Gripper servo 10 pulse <-> URDF r_joint (the other four finger joints mimic it). Two live
# readbacks (poses.yaml, 2026-10-16) matched to the model's finger gap: empty jaws stall at 598
# = gap 0 (r_joint -0.70), a 3 cm cube at 486 = gap 30 mm (r_joint -0.147).
# shortcut: linear 2-point fit, fully open (pulse 200) is extrapolated; re-fit if open width matters.
GRIPPER_RAD_PER_PULSE = (-0.70 + 0.147) / (598 - 486)
GRIPPER_URDF_RANGE = (-0.70, 0.85)  # fingers touch .. widest gap in the model


def gripper_pulse_to_urdf(pulse: float) -> float:
    lo, hi = GRIPPER_URDF_RANGE
    return min(hi, max(lo, -0.147 + (pulse - 486) * GRIPPER_RAD_PER_PULSE))


def urdf_to_gripper_pulse(r_joint: float) -> int:
    return round(486 + (r_joint + 0.147) / GRIPPER_RAD_PER_PULSE)


class ArmKinematics:
    def __init__(self, pan_x: float = 0.0466, shoulder_z: float = 0.1907,
                 upper: float = 0.08513, forearm: float = 0.085147,
                 tool: float = 0.05577 + 0.08,
                 camera_along: float = 0.069, camera_up: float = 0.042,
                 joint_limit: float = 2.09):
        self.pan_x, self.shoulder_z = pan_x, shoulder_z
        self.upper, self.forearm, self.tool = upper, forearm, tool
        self.camera_along, self.camera_up = camera_along, camera_up
        self.joint_limit = joint_limit

    def _chain(self, j: JointSolution):
        """(wrist r, wrist z, link4 angle from vertical) in the arm plane."""
        phi1, phi2 = j.shoulder, j.shoulder + j.elbow
        r = self.upper * math.sin(phi1) + self.forearm * math.sin(phi2)
        z = self.shoulder_z + self.upper * math.cos(phi1) + self.forearm * math.cos(phi2)
        return r, z, phi2 + j.wrist

    def _to_base(self, r: float, z: float, pan: float):
        return self.pan_x + r * math.cos(pan), r * math.sin(pan), z

    def fk(self, joints: JointSolution) -> tuple[float, float, float, float]:
        """Tool tip (x, y, z) in base_link and tool pitch (0 = horizontal, -pi/2 = down)."""
        r, z, phi = self._chain(joints)
        x, y, z = self._to_base(r + self.tool * math.sin(phi), z + self.tool * math.cos(phi),
                                joints.pan)
        return x, y, z, math.pi / 2 - phi

    def ik(self, x: float, y: float, z: float, pitch: float = -math.pi / 2,
           roll: float = 0.0) -> JointSolution | None:
        if not all(math.isfinite(v) for v in (x, y, z, pitch, roll)):
            return None
        if z < 0.0:
            return None  # below the floor
        # shortcut: box keep-out guess for the chassis front; measure it in 14.7.
        if x < 0.13 and z < 0.12:
            return None
        pan = math.atan2(y, x - self.pan_x)
        r = math.hypot(x - self.pan_x, y)
        phi = math.pi / 2 - pitch
        a = r - self.tool * math.sin(phi)
        b = z - self.tool * math.cos(phi) - self.shoulder_z
        c = (a * a + b * b - self.upper ** 2 - self.forearm ** 2) / (2 * self.upper * self.forearm)
        if not -1.0 <= c <= 1.0:
            return None
        elbow = math.acos(c)  # elbow-up branch (positive), the one the forward pose uses
        shoulder = math.atan2(a, b) - math.atan2(self.forearm * math.sin(elbow),
                                                 self.upper + self.forearm * math.cos(elbow))
        solution = JointSolution(pan, shoulder, elbow, phi - shoulder - elbow, roll)
        if any(abs(v) > self.joint_limit for v in (pan, shoulder, elbow, solution.wrist, roll)):
            return None
        return solution

    def ik_any_pitch(self, x: float, y: float, z: float, roll: float = 0.0,
                     steepest: float = -math.pi / 2, flattest: float = -math.radians(30)):
        """Steepest reachable tool pitch (straight down first), 5 deg steps."""
        pitch = steepest
        while pitch <= flattest + 1e-9:
            solution = self.ik(x, y, z, pitch, roll)
            if solution is not None:
                return solution, pitch
            pitch += math.radians(5)
        return None, None

    def camera_pose(self, joints: JointSolution) -> tuple[np.ndarray, np.ndarray]:
        """(R, t): optical frame (x right, y down, z forward) -> base_link."""
        r, z, phi = self._chain(joints)
        along = (math.sin(phi), math.cos(phi))
        up = (-math.cos(phi), math.sin(phi))
        cam_r = r + self.camera_along * along[0] + self.camera_up * up[0]
        cam_z = z + self.camera_along * along[1] + self.camera_up * up[1]
        cp, sp = math.cos(joints.pan), math.sin(joints.pan)
        z_axis = np.array([along[0] * cp, along[0] * sp, along[1]])
        y_axis = -np.array([up[0] * cp, up[0] * sp, up[1]])
        x_axis = np.cross(y_axis, z_axis)
        return np.column_stack([x_axis, y_axis, z_axis]), np.array(self._to_base(cam_r, cam_z,
                                                                                 joints.pan))

    def pixel_to_plane(self, u: float, v: float, k: tuple, joints: JointSolution,
                       plane_z: float = 0.0):
        """Base-frame (x, y) where the pixel ray hits the horizontal plane z = plane_z, or None."""
        fx, fy, cx, cy = k
        rotation, origin = self.camera_pose(joints)
        ray = rotation @ np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        if ray[2] >= -1e-6:
            return None
        t = (plane_z - origin[2]) / ray[2]
        if t <= 0:
            return None
        hit = origin + t * ray
        return float(hit[0]), float(hit[1])

    def pixel_to_point(self, u: float, v: float, k: tuple, joints: JointSolution,
                       depth: float) -> tuple[float, float, float]:
        """Base-frame (x, y, z) of a pixel at optical-axis depth `depth` (m), any arm pose."""
        fx, fy, cx, cy = k
        rotation, origin = self.camera_pose(joints)
        point = origin + rotation @ (depth * np.array([(u - cx) / fx, (v - cy) / fy, 1.0]))
        return float(point[0]), float(point[1]), float(point[2])


class CameraAim:
    """Conservative camera bearing/elevation to bus-servo pulse mapping."""

    def __init__(self, pan_center: int = 500, tilt_forward: int = 300,
                 max_bearing: float = math.radians(60.0),
                 min_elevation: float = math.radians(-61.0),  # look_down tilt (pulse 50) is -60
                 max_elevation: float = math.radians(48.0)):
        self.pan_center = pan_center
        self.tilt_forward = tilt_forward
        self.max_bearing = max_bearing
        self.min_elevation = min_elevation
        self.max_elevation = max_elevation

    def pulses(self, bearing: float, elevation: float) -> tuple[int, int]:
        if not all(math.isfinite(v) for v in (bearing, elevation)):
            raise ValueError("camera aim must be finite")
        if abs(bearing) > self.max_bearing:
            raise ValueError("bearing outside conservative camera envelope")
        if not self.min_elevation <= elevation <= self.max_elevation:
            raise ValueError("elevation outside conservative camera envelope")
        scale = 1.0 / math.radians(0.24)
        pan = round(self.pan_center + bearing * scale)
        tilt = round(self.tilt_forward + elevation * scale)
        if not (0 <= pan <= 1000 and 0 <= tilt <= 1000):
            raise ValueError("camera aim outside servo pulse range")
        return pan, tilt

    def sweep(self, views: int, elevation: float = 0.0) -> list[tuple[int, int]]:
        if views < 1:
            raise ValueError("views must be positive")
        bearings = ([0.0] if views == 1 else
                    np.linspace(-self.max_bearing, self.max_bearing, views))
        return [self.pulses(float(bearing), elevation) for bearing in bearings]
