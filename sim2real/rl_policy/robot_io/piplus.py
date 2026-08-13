"""Pure PiPlus hardware-order conversions used by the ROS2 bridge."""

from __future__ import annotations

from typing import Sequence

import numpy as np

from sim2real.config.robots.base import RobotCfg


def validate_hardware_contract(robot_cfg: RobotCfg) -> tuple[np.ndarray, np.ndarray]:
    count = len(robot_cfg.joint_names)
    if len(robot_cfg.hardware_joint_names) != count:
        raise ValueError(
            f"{robot_cfg.name} hardware_joint_names has length "
            f"{len(robot_cfg.hardware_joint_names)}, expected {count}"
        )
    if len(robot_cfg.hardware_joint_map) != count:
        raise ValueError(
            f"{robot_cfg.name} hardware_joint_map has length "
            f"{len(robot_cfg.hardware_joint_map)}, expected {count}"
        )
    if len(robot_cfg.hardware_joint_signs) != count:
        raise ValueError(
            f"{robot_cfg.name} hardware_joint_signs has length "
            f"{len(robot_cfg.hardware_joint_signs)}, expected {count}"
        )
    joint_map = np.asarray(robot_cfg.hardware_joint_map, dtype=np.int64)
    signs = np.asarray(robot_cfg.hardware_joint_signs, dtype=np.float32)
    hardware_count = len(robot_cfg.hardware_joint_names)
    if sorted(joint_map.tolist()) != list(range(hardware_count)):
        raise ValueError(
            f"{robot_cfg.name} hardware_joint_map must be a permutation of "
            f"0..{hardware_count - 1}, got {joint_map.tolist()}"
        )
    mapped_names = tuple(robot_cfg.hardware_joint_names[index] for index in joint_map)
    if mapped_names != tuple(robot_cfg.joint_names):
        raise ValueError(
            f"{robot_cfg.name} hardware_joint_map does not map hardware names to "
            f"policy order: {mapped_names}"
        )
    if not np.all(np.isfinite(signs)) or np.any(np.abs(signs) < 1e-6):
        raise ValueError(f"{robot_cfg.name} hardware_joint_signs must be finite and nonzero")
    return joint_map, signs


def map_hardware_to_policy(values: Sequence[float], robot_cfg: RobotCfg) -> np.ndarray:
    joint_map, signs = validate_hardware_contract(robot_cfg)
    hardware_values = np.asarray(values, dtype=np.float32).reshape(-1)
    if hardware_values.size != len(robot_cfg.hardware_joint_names):
        raise ValueError(
            f"Hardware array has {hardware_values.size} values, expected "
            f"{len(robot_cfg.hardware_joint_names)}"
        )
    return hardware_values[joint_map] * signs


def map_policy_to_hardware(values: Sequence[float], robot_cfg: RobotCfg) -> np.ndarray:
    joint_map, signs = validate_hardware_contract(robot_cfg)
    policy_values = np.asarray(values, dtype=np.float32).reshape(-1)
    count = len(robot_cfg.joint_names)
    if policy_values.size != count:
        raise ValueError(f"Policy array has {policy_values.size} values, expected {count}")
    hardware_values = np.zeros(count, dtype=np.float32)
    hardware_values[joint_map] = policy_values * signs
    return hardware_values


def reorder_policy_to_hardware(values: Sequence[float], robot_cfg: RobotCfg) -> np.ndarray:
    joint_map, _ = validate_hardware_contract(robot_cfg)
    policy_values = np.asarray(values, dtype=np.float32).reshape(-1)
    count = len(robot_cfg.joint_names)
    if policy_values.size != count:
        raise ValueError(f"Policy array has {policy_values.size} values, expected {count}")
    hardware_values = np.zeros(count, dtype=np.float32)
    hardware_values[joint_map] = policy_values
    return hardware_values


def imu_xyzw_to_wxyz(orientation: Sequence[float]) -> np.ndarray:
    """Convert sensor_msgs/Imu's xyzw quaternion to policy wxyz."""
    quat = np.asarray(orientation, dtype=np.float32).reshape(-1)
    if quat.size != 4:
        raise ValueError(f"IMU quaternion has {quat.size} values, expected 4")
    return np.asarray([quat[3], quat[0], quat[1], quat[2]], dtype=np.float32)
