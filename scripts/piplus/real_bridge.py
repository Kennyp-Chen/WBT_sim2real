#!/usr/bin/env python3
"""Bridge PiPlus 22/23-DoF ROS2 topics to the sim2real LowState/LowCmd ABI.

The policy process always sees the canonical joint order from ``RobotCfg``.
The ROS2 ``JointState`` and ``MotorControlCommand`` messages use the hardware
order from instinct_onboard, so the conversion is deliberately kept here at
the process boundary.

ROS2 and ``hightorque_msgs`` are optional imports.  This keeps the pure mapping
functions importable in CI and lets the bridge fail with a useful message when
it is accidentally started outside the onboard ROS environment.
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np
import zmq
from loguru import logger

from sim2real.config.robots import get_robot_cfg
from sim2real.config.robots.base import RobotCfg
from sim2real.rl_policy.robot_io.piplus import (
    imu_xyzw_to_wxyz,
    map_hardware_to_policy,
    validate_hardware_contract,
)
from sim2real.utils.common import LowCmdMessage, LowStateMessage

try:  # pragma: no cover - ROS is only available on the onboard image.
    import rclpy
    from hightorque_msgs.msg import MotorControlCommand
    from hightorque_msgs.srv import ReleaseControl, RequestControl
    from rclpy.node import Node
    from sensor_msgs.msg import Imu, JointState
except ImportError as exc:  # pragma: no cover
    rclpy = None
    MotorControlCommand = Imu = JointState = Node = None  # type: ignore[assignment]
    ReleaseControl = RequestControl = None  # type: ignore[assignment]
    _ROS_IMPORT_ERROR: Exception | None = exc
else:  # pragma: no cover
    _ROS_IMPORT_ERROR = None


SUPPORTED_ROBOTS = frozenset({"piplus_h0w", "piplus_lse_23dof"})


def _named_or_indexed_values(
    values: Sequence[float],
    names: Sequence[str],
    hardware_names: Sequence[str],
) -> np.ndarray:
    """Normalize JointState arrays while tolerating unnamed fixed-order messages."""
    array = np.asarray(values, dtype=np.float32).reshape(-1)
    if array.size != len(hardware_names):
        raise ValueError(
            f"JointState array has {array.size} values, expected {len(hardware_names)}"
        )
    names = [str(name) for name in names]
    if not names:
        return array
    index = {name: idx for idx, name in enumerate(names)}
    missing = [name for name in hardware_names if name not in index]
    if missing:
        raise ValueError(f"JointState is missing hardware joints: {missing}")
    return np.asarray([array[index[name]] for name in hardware_names], dtype=np.float32)


@dataclass
class _LatestState:
    joint_positions: np.ndarray | None = None
    joint_velocities: np.ndarray | None = None
    quaternion_wxyz: np.ndarray = field(
        default_factory=lambda: np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
    )
    gyroscope: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    imu_ready: bool = False
    tick: int = 0

    @property
    def ready(self) -> bool:
        return (
            self.joint_positions is not None
            and self.joint_velocities is not None
            and self.imu_ready
        )


if Node is not None:  # pragma: no cover - exercised on the onboard ROS image.

    class PiPlusRealBridge(Node):
        """ROS2 node exposing PiPlus state and commands through ZMQ."""

        def __init__(self, args: argparse.Namespace):
            super().__init__(args.node_name)
            self.args = args
            self.robot_cfg = get_robot_cfg(args.robot)
            if self.robot_cfg.name not in SUPPORTED_ROBOTS:
                supported = ", ".join(sorted(SUPPORTED_ROBOTS))
                raise ValueError(
                    "scripts/piplus/real_bridge.py only supports PiPlus robot "
                    f"configs: {supported}"
                )
            validate_hardware_contract(self.robot_cfg)
            if args.joint_pos_protect_ratio <= 0.0:
                raise ValueError("joint_pos_protect_ratio must be positive")
            self.state = _LatestState()
            self._control_uuid: Any | None = None
            self._control_request = None
            self._release_request = None
            self._sequence = 0
            self._last_command_ns = 0
            self._unsafe_joint_state = False
            self._unsafe_joint_logged = False

            context = zmq.Context.instance()
            self._state_pub = context.socket(zmq.PUB)
            self._state_pub.setsockopt(zmq.SNDHWM, 1)
            self._state_pub.setsockopt(zmq.LINGER, 0)
            self._state_pub.bind(f"tcp://{args.state_bind_addr}:{args.state_port}")
            self._cmd_sub = context.socket(zmq.SUB)
            self._cmd_sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._cmd_sub.setsockopt(zmq.CONFLATE, 1)
            self._cmd_sub.setsockopt(zmq.RCVTIMEO, 0)
            self._cmd_sub.setsockopt(zmq.LINGER, 0)
            self._cmd_sub.connect(f"tcp://{args.cmd_host}:{args.cmd_port}")

            cmd_topic = args.cmd_topic
            if args.dryrun:
                cmd_topic += f"_dryrun_{np.random.randint(0, 65535)}"
            self._cmd_pub = self.create_publisher(MotorControlCommand, cmd_topic, 10)
            self._request_client = self.create_client(RequestControl, args.request_service)
            self._release_client = self.create_client(ReleaseControl, args.release_service)
            self.create_subscription(JointState, args.state_topic, self._on_joint_state, 10)
            self.create_subscription(Imu, args.imu_topic, self._on_imu, 10)
            self._timer = self.create_timer(1.0 / args.publish_hz, self._on_timer)
            self.get_logger().info(
                f"PiPlus bridge ready: state tcp://{args.state_bind_addr}:{args.state_port}, "
                f"command tcp://{args.cmd_host}:{args.cmd_port}, ROS topic {cmd_topic}"
            )

        def _on_joint_state(self, msg: Any) -> None:
            try:
                names = list(getattr(msg, "name", []))
                hardware_names = self.robot_cfg.hardware_joint_names
                positions = _named_or_indexed_values(msg.position, names, hardware_names)
                velocities = _named_or_indexed_values(
                    msg.velocity if msg.velocity else np.zeros_like(positions),
                    names if msg.velocity else [],
                    hardware_names,
                )
                self.state.joint_positions = map_hardware_to_policy(positions, self.robot_cfg)
                self.state.joint_velocities = map_hardware_to_policy(velocities, self.robot_cfg)
                lower = np.asarray(
                    [self.robot_cfg.joint_pos_lower_limit[name] for name in self.robot_cfg.joint_names],
                    dtype=np.float32,
                )
                upper = np.asarray(
                    [self.robot_cfg.joint_pos_upper_limit[name] for name in self.robot_cfg.joint_names],
                    dtype=np.float32,
                )
                midpoint = (lower + upper) * 0.5
                protect_half_range = (
                    (upper - lower) * 0.5 * self.args.joint_pos_protect_ratio
                )
                self._unsafe_joint_state = bool(
                    np.any(self.state.joint_positions < midpoint - protect_half_range)
                    or np.any(self.state.joint_positions > midpoint + protect_half_range)
                )
                if self._unsafe_joint_state and not self._unsafe_joint_logged:
                    self._unsafe_joint_logged = True
                    self.get_logger().error(
                        "PiPlus joint state exceeded the configured protection "
                        f"range ({self.args.joint_pos_protect_ratio:.3f}x); "
                        "commands are inhibited."
                    )
                self.state.tick += 1
            except (TypeError, ValueError) as exc:
                self.get_logger().error(f"Invalid JointState: {exc}")

        def _on_imu(self, msg: Any) -> None:
            try:
                orientation = msg.orientation
                self.state.quaternion_wxyz = imu_xyzw_to_wxyz(
                    [orientation.x, orientation.y, orientation.z, orientation.w]
                )
                self.state.gyroscope = np.asarray(
                    [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                    dtype=np.float32,
                )
                self.state.imu_ready = True
            except (AttributeError, ValueError) as exc:
                self.get_logger().error(f"Invalid IMU message: {exc}")

        def _request_control_if_ready(self) -> None:
            if self.args.dryrun or not self.state.ready or self._control_uuid is not None:
                return
            if self._control_request is None:
                if not self._request_client.service_is_ready():
                    return
                request = RequestControl.Request()
                request.node_name = self.get_name()
                request.motor_ids = list(range(len(self.robot_cfg.hardware_joint_names)))
                request.control_mode = 0
                request.default_behavior = 1
                request.timeout_ms = self.args.control_timeout_ms
                request.default_kp = [0.0] * len(request.motor_ids)
                request.default_kd = [0.0] * len(request.motor_ids)
                self._request_client.wait_for_service(timeout_sec=0.0)
                self._control_request = self._request_client.call_async(request)
                return
            if self._control_request.done():
                result = self._control_request.result()
                if result is None or not result.success:
                    message = getattr(result, "message", "request failed")
                    raise RuntimeError(f"PiPlus motor control request failed: {message}")
                self._control_uuid = result.uuid
                self.get_logger().info(f"Motor control granted: UUID={self._control_uuid}")

        def _publish_state(self) -> None:
            if not self.state.ready:
                return
            message = LowStateMessage(
                quaternion=self.state.quaternion_wxyz,
                gyroscope=self.state.gyroscope,
                joint_positions=self.state.joint_positions,
                joint_velocities=self.state.joint_velocities,
                tick=self.state.tick,
            )
            try:
                self._state_pub.send(message.to_bytes(), flags=zmq.DONTWAIT)
            except zmq.Again:
                pass

        def _read_command(self) -> LowCmdMessage | None:
            latest = None
            while True:
                try:
                    data = self._cmd_sub.recv(flags=zmq.DONTWAIT)
                except zmq.Again:
                    return latest
                try:
                    latest = LowCmdMessage.from_bytes(data)
                except ValueError as exc:
                    self.get_logger().error(f"Invalid LowCmdMessage: {exc}")

        def _publish_command(self, message: LowCmdMessage) -> None:
            if message.q_target.size != len(self.robot_cfg.joint_names):
                raise ValueError(
                    f"LowCmd has {message.q_target.size} joints, expected "
                    f"{len(self.robot_cfg.joint_names)}"
                )
            lower = np.asarray(
                [self.robot_cfg.joint_pos_lower_limit[name] for name in self.robot_cfg.joint_names],
                dtype=np.float32,
            )
            upper = np.asarray(
                [self.robot_cfg.joint_pos_upper_limit[name] for name in self.robot_cfg.joint_names],
                dtype=np.float32,
            )
            q_target = np.clip(message.q_target, lower, upper)
            cmd = MotorControlCommand()
            if self._control_uuid is not None:
                cmd.uuid = self._control_uuid
            joint_map, signs = validate_hardware_contract(self.robot_cfg)
            # MotorControlCommand arrays are paired with motor_ids.  Keep the
            # source node's policy-loop order and only map each motor id.
            cmd.motor_ids = joint_map.tolist()
            cmd.positions = (q_target * signs).tolist()
            cmd.velocities = (message.dq_target * signs).tolist()
            cmd.kp = message.kp.tolist()
            cmd.kd = message.kd.tolist()
            cmd.torques = message.tau_ff.tolist()
            self._cmd_pub.publish(cmd)
            self._last_command_ns = time.monotonic_ns()

        def _on_timer(self) -> None:
            self._request_control_if_ready()
            self._publish_state()
            if self._unsafe_joint_state or (self._control_uuid is None and not self.args.dryrun):
                return
            message = self._read_command()
            if message is not None:
                self._publish_command(message)

        def destroy_node(self) -> bool:
            if self._control_uuid is not None and not self.args.dryrun:
                request = ReleaseControl.Request()
                request.node_name = self.get_name()
                request.uuid = self._control_uuid
                request.release_mode = 1
                future = self._release_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            self._state_pub.close(linger=0)
            self._cmd_sub.close(linger=0)
            return super().destroy_node()

else:

    class PiPlusRealBridge:  # pragma: no cover
        def __init__(self, args: argparse.Namespace):
            error = str(_ROS_IMPORT_ERROR) if _ROS_IMPORT_ERROR else "ROS2 unavailable"
            raise RuntimeError(
                "PiPlusRealBridge requires ROS2 plus hightorque_msgs; "
                f"import failed with: {error}"
            )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="piplus_h0w")
    parser.add_argument("--node-name", default="sim2real_piplus_bridge")
    parser.add_argument("--state-topic", default="joint_states")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--cmd-topic", default="control_command")
    parser.add_argument("--request-service", default="request_control")
    parser.add_argument("--release-service", default="release_control")
    parser.add_argument("--state-bind-addr", default="*")
    parser.add_argument("--state-port", type=int, default=5590)
    parser.add_argument("--cmd-host", default="127.0.0.1")
    parser.add_argument("--cmd-port", type=int, default=5591)
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--control-timeout-ms", type=int, default=5000)
    parser.add_argument("--joint-pos-protect-ratio", type=float, default=1.0)
    parser.add_argument("--dryrun", action="store_true")
    return parser.parse_args()


def main() -> None:  # pragma: no cover - requires onboard ROS2 runtime.
    args = _parse_args()
    if rclpy is None:
        raise RuntimeError(
            "ROS2 is not available. Run this bridge in the PiPlus ROS2 onboard environment."
        ) from _ROS_IMPORT_ERROR
    rclpy.init()
    node = PiPlusRealBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Stopping PiPlus bridge")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
