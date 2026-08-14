from __future__ import annotations

import numpy as np
import pytest

from sim2real.config.robots import PIPLUS_H0W_CFG, PIPLUS_LSE_23DOF_CFG
from sim2real.rl_policy.robot_io.piplus import (
    imu_xyzw_to_wxyz,
    map_hardware_to_policy,
    map_policy_to_hardware,
    reorder_policy_to_hardware,
)


def test_piplus_hardware_contract_matches_instinct_onboard_order() -> None:
    cfg = PIPLUS_H0W_CFG
    assert len(cfg.joint_names) == 22
    assert cfg.hardware_joint_map == (
        16, 17, 18, 19, 12, 13, 14, 15, 20, 21, 11,
        10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
    )
    assert cfg.hardware_joint_names[-2:] == ("head_yaw_joint", "head_pitch_joint")


def test_piplus_state_command_mapping_round_trip() -> None:
    cfg = PIPLUS_H0W_CFG
    hardware = np.arange(22, dtype=np.float32)
    policy = map_hardware_to_policy(hardware, cfg)
    np.testing.assert_array_equal(policy, hardware[list(cfg.hardware_joint_map)])
    np.testing.assert_array_equal(map_policy_to_hardware(policy, cfg), hardware)


def test_piplus_lse_23dof_hardware_contract_matches_instinct_onboard() -> None:
    cfg = PIPLUS_LSE_23DOF_CFG
    assert len(cfg.joint_names) == 23
    assert cfg.hardware_joint_map == (
        11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 22,
        16, 17, 18, 19, 12, 13, 14, 15, 20, 21,
    )
    assert cfg.hardware_joint_names[-3:] == (
        "head_yaw_joint",
        "head_pitch_joint",
        "waist_yaw_joint",
    )


def test_piplus_lse_23dof_state_command_mapping_round_trip() -> None:
    cfg = PIPLUS_LSE_23DOF_CFG
    hardware = np.arange(23, dtype=np.float32)
    policy = map_hardware_to_policy(hardware, cfg)
    np.testing.assert_array_equal(policy, hardware[list(cfg.hardware_joint_map)])
    np.testing.assert_array_equal(map_policy_to_hardware(policy, cfg), hardware)


def test_piplus_gains_reorder_without_position_sign() -> None:
    cfg = PIPLUS_H0W_CFG
    policy = np.arange(22, dtype=np.float32) + 0.5
    hardware = reorder_policy_to_hardware(policy, cfg)
    np.testing.assert_array_equal(hardware[list(cfg.hardware_joint_map)], policy)


def test_piplus_mapping_rejects_wrong_shape() -> None:
    with pytest.raises(ValueError, match="expected 22"):
        map_hardware_to_policy(np.zeros(21), PIPLUS_H0W_CFG)


def test_imu_quaternion_is_xyzw_to_wxyz() -> None:
    np.testing.assert_array_equal(
        imu_xyzw_to_wxyz([0.1, 0.2, 0.3, 0.9]),
        np.asarray([0.9, 0.1, 0.2, 0.3], dtype=np.float32),
    )
