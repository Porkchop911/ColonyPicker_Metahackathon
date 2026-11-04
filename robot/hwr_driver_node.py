# VENDORED from the robot's own stack (repo highwonder_rework, ros2_ws/src/hwr_driver/hwr_driver/driver_node.py,
# as deployed on the robot at ~/hwr). Local change 2026-10-30: the `servo_torque` topic (teach by hand, tools/teach.py).
# Deploy: scp to ~/hwr/ros2_ws/src/hwr_driver/hwr_driver/driver_node.py (symlink install), sudo systemctl restart hwr.
"""hwr_driver — the one process that owns the RRC board.

Pubs:  imu (sensor_msgs/Imu, param rate), battery (sensor_msgs/BatteryState ~1 Hz),
       joint_states (arm read-back, param rate), button (std_msgs/String)
Subs:  wheel_speeds (hwr_msgs/WheelSpeeds), servo_cmd (hwr_msgs/ServoCommand),
       beep (std_msgs/UInt16, value = freq Hz)

Safety (hwr_core primitives, unit-tested): watchdog auto-zeros motors 0.4 s
after the last nonzero wheel_speeds; invalid wheel/servo commands are rejected
(logged, nothing latched); SIGINT/shutdown always stops motors.
Jazzy-ready: plain rclpy, no distro-specific APIs.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from hwr_msgs.msg import ServoCommand, WheelSpeeds
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_msgs.msg import String, UInt16

from hwr_core.rrc import Board, codecs
from hwr_core.safety import Watchdog, validate_wheel_rps

ARM_JOINTS = {1: "joint1", 2: "joint2", 3: "joint3", 4: "joint4", 5: "joint5"}
# Read back with the arm; a held cube stalls the close short of its target (grasp check).
READBACK_JOINTS = {**ARM_JOINTS, 10: "gripper"}
PULSE_CENTER = 500
RAD_PER_PULSE = math.radians(240.0 / 1000.0)
MAX_WHEEL_RPS = 4.0
VALID_SERVO_IDS = frozenset((*ARM_JOINTS, 10))


class DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("hwr_driver")
        self.declare_parameter("device", "/dev/rrc")
        self.declare_parameter("imu_rate", 50.0)
        self.declare_parameter("joint_state_rate", 5.0)
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("gyro_z_scale", 1.0)
        self.declare_parameter("watchdog_timeout", 0.4)
        self.declare_parameter("max_wheel_rps", MAX_WHEEL_RPS)

        device = self.get_parameter("device").value
        self.board = Board.open(device)
        self.board.set_motor_type(codecs.MOTOR_TYPE_JGB37)
        self.get_logger().info(f"board open on {device}")

        # known idle state at start; nothing stays latched
        self.board.stop_all_motors()
        self.board.stop_servos(list(ARM_JOINTS) + [10])
        self.watchdog = Watchdog(
            self._watchdog_stop,
            timeout=float(self.get_parameter("watchdog_timeout").value))

        sensor_qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub_imu = self.create_publisher(Imu, "imu", sensor_qos)
        self.pub_batt = self.create_publisher(BatteryState, "battery", 10)
        self.pub_joints = self.create_publisher(JointState, "joint_states", 10)
        self.pub_button = self.create_publisher(String, "button", 10)

        self.create_subscription(WheelSpeeds, "wheel_speeds", self._on_wheel_speeds, 10)
        self.create_subscription(ServoCommand, "servo_cmd", self._on_servo_cmd, 10)
        # EXPERIMENTAL probe (2026-10-31): verbatim bus-servo payloads via positions[];
        # ids[0] = expect_reply flag. Forwards to Board.servo_raw; reply bytes are
        # logged (no service plumbing for a probe). See tools/roll_flip.py probe-raw.
        self.create_subscription(ServoCommand, "servo_raw", self._on_servo_raw, 10)
        # teach mode: ServoCommand ids with position 0 go limp (torque off), 1 hold again; joint_states keeps reading them
        self.create_subscription(ServoCommand, "servo_torque", self._on_servo_torque, 10)
        self.create_subscription(UInt16, "beep", self._on_beep, 10)

        self._latest_imu = None  # (ImuSample, stamp) cached from the RX thread
        # A partial servo response must not disconnect the articulated camera
        # subtree in robot_state_publisher.  Seed every joint and retain the
        # last successful readback for individual timeouts.
        self._joint_positions = {name: 0.0 for name in READBACK_JOINTS.values()}
        self._joint_message = JointState(
            name=list(READBACK_JOINTS.values()),
            position=[0.0] * len(READBACK_JOINTS))
        self._joint_shape_logged = False
        self.board.subscribe_imu(lambda s, t: setattr(self, "_latest_imu", (s, t)))
        self.board.subscribe_battery(self._on_battery)
        self.board.subscribe_key(self._on_key)

        imu_period = 1.0 / float(self.get_parameter("imu_rate").value)
        self.create_timer(imu_period, self._publish_imu)
        js_rate = float(self.get_parameter("joint_state_rate").value)
        if js_rate > 0:
            self.create_timer(1.0 / js_rate, self._publish_joint_states)

    # ------------------------------------------------------------- safety paths

    def _watchdog_stop(self) -> None:
        self.board.stop_all_motors()
        self.get_logger().warning("watchdog: wheel_speeds source went silent -> motors stopped")

    # ------------------------------------------------------------- subscriptions

    def _on_wheel_speeds(self, msg: WheelSpeeds) -> None:
        limit = float(self.get_parameter("max_wheel_rps").value)
        try:
            rps = validate_wheel_rps(msg.rps, limit)
        except ValueError as exc:
            self.board.stop_all_motors()
            self.get_logger().error(
                f"invalid wheel command {list(msg.rps)!r}: {exc}")
            return
        self.board.set_motor_rps(list(zip((1, 2, 3, 4), rps)))
        if any(r != 0.0 for r in rps):
            self.watchdog.feed()
        else:
            self.watchdog.disarm()

    def _on_servo_cmd(self, msg: ServoCommand) -> None:
        ids = [int(sid) for sid in msg.ids]
        pulses = [int(pulse) for pulse in msg.positions]
        if (len(ids) != len(pulses) or not ids
                or any(sid not in VALID_SERVO_IDS for sid in ids)
                or any(not 0 <= pulse <= 1000 for pulse in pulses)):
            self.get_logger().error(
                f"invalid servo command ids={ids!r} positions={pulses!r}")
            return
        positions = list(zip(ids, pulses))
        if positions:
            self.board.set_servo_positions(int(msg.duration_ms), positions)

    def _on_servo_raw(self, msg: ServoCommand) -> None:
        payload = [int(p) for p in msg.positions]
        expect = bool(msg.ids and int(msg.ids[0]))
        try:
            reply = self.board.servo_raw(payload, expect_reply=expect, timeout=1.0)
        except Exception as exc:  # probe path: report, never crash the driver
            self.get_logger().error(f"servo_raw {payload!r} failed: {exc}")
            return
        self.get_logger().info(
            f"servo_raw sent={payload!r} reply={list(reply) if reply else None!r}")

    def _on_servo_torque(self, msg: ServoCommand) -> None:
        for sid, on in zip(msg.ids, msg.positions):
            if int(sid) in VALID_SERVO_IDS:
                self.board.set_servo_torque(int(sid), bool(on))

    def _on_beep(self, msg: UInt16) -> None:
        self.board.beep(int(msg.data) or 1500, 100, 100, 1)

    # ------------------------------------------------------------- report relays

    def _on_battery(self, mv: int, stamp: float) -> None:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = mv / 1000.0
        msg.percentage = float("nan")
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.present = True
        self.pub_batt.publish(msg)

    def _on_key(self, key_id: int, event: str, stamp: float) -> None:
        msg = String()
        msg.data = f"key{key_id}:{event}"
        self.pub_button.publish(msg)

    # ------------------------------------------------------------- timers

    def _publish_imu(self) -> None:
        if self._latest_imu is None:
            return
        sample, _ = self._latest_imu
        sample = codecs.scale_imu_gyro(
            sample, z_scale=float(self.get_parameter("gyro_z_scale").value))
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter("imu_frame").value
        msg.linear_acceleration.x = sample.ax
        msg.linear_acceleration.y = sample.ay
        msg.linear_acceleration.z = sample.az
        msg.angular_velocity.x = sample.gx
        msg.angular_velocity.y = sample.gy
        msg.angular_velocity.z = sample.gz
        msg.orientation_covariance[0] = -1.0  # no orientation estimate
        self.pub_imu.publish(msg)

    def _publish_joint_states(self) -> None:
        for sid, name in READBACK_JOINTS.items():
            pulse = self.board.servo_position(sid, timeout=0.2)
            if pulse is not None:
                self._joint_positions[name] = (pulse - PULSE_CENTER) * RAD_PER_PULSE
        msg = self._joint_message
        for index, name in enumerate(msg.name):
            msg.position[index] = float(self._joint_positions[name])
        msg.header.stamp = self.get_clock().now().to_msg()
        if len(msg.name) != len(msg.position):
            self.get_logger().error("refusing malformed joint-state publication")
            return
        if not self._joint_shape_logged:
            self.get_logger().info(
                f"joint-state publisher shape names={len(msg.name)} "
                f"positions={len(msg.position)}")
            self._joint_shape_logged = True
        self.pub_joints.publish(msg)

    # ------------------------------------------------------------- lifecycle

    def shutdown(self) -> None:
        self.watchdog.close()
        self.board.stop_all_motors()
        self.board.close()


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
