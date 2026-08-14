"""HT robot configurations used by the BFM-Zero checkpoints."""

from __future__ import annotations

import os
from pathlib import Path

from sim2real.config.robots.base import PROJECT_ROOT, RobotCfg


def _asset_path(model_name: str, filename: str, env_name: str) -> str:
    override = os.environ.get(env_name)
    if override:
        return str(Path(override).expanduser().resolve())
    candidates = (
        PROJECT_ROOT / "assets" / model_name / "xml" / filename,
        PROJECT_ROOT.parent.parent / "Assets" / "ht_urdf" / "ht_urdf" / model_name / "xml" / filename,
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    return str(candidates[0])


PIPLUS_LSE_23DOF_JOINT_NAMES = (
    "r_hip_pitch_joint", "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint",
    "r_ankle_pitch_joint", "r_ankle_roll_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_thigh_joint", "l_calf_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
    "waist_yaw_joint", "r_shoulder_pitch_joint", "r_shoulder_roll_joint",
    "r_upper_arm_joint", "r_elbow_joint", "l_shoulder_pitch_joint", "l_shoulder_roll_joint",
    "l_upper_arm_joint", "l_elbow_joint", "head_yaw_joint", "head_pitch_joint",
)
PIPLUS_LSE_23DOF_HARDWARE_JOINT_NAMES = (
    "l_ankle_roll_joint",
    "l_ankle_pitch_joint",
    "l_calf_joint",
    "l_thigh_joint",
    "l_hip_roll_joint",
    "l_hip_pitch_joint",
    "r_ankle_roll_joint",
    "r_ankle_pitch_joint",
    "r_calf_joint",
    "r_thigh_joint",
    "r_hip_roll_joint",
    "r_hip_pitch_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "waist_yaw_joint",
)
PIPLUS_LSE_23DOF_HARDWARE_JOINT_MAP = (
    11,
    10,
    9,
    8,
    7,
    6,
    5,
    4,
    3,
    2,
    1,
    0,
    22,
    16,
    17,
    18,
    19,
    12,
    13,
    14,
    15,
    20,
    21,
)
PIPLUS_LSE_23DOF_HARDWARE_JOINT_SIGNS = (1.0,) * len(
    PIPLUS_LSE_23DOF_JOINT_NAMES
)
PIPLUS_LSE_23DOF_BODY_NAMES = (
    "base_link", "r_hip_pitch_link", "r_hip_roll_link", "r_thigh_link", "r_calf_link",
    "r_ankle_pitch_link", "r_ankle_roll_link", "l_hip_pitch_link", "l_hip_roll_link",
    "l_thigh_link", "l_calf_link", "l_ankle_pitch_link", "l_ankle_roll_link",
    "waist_yaw_link", "torso_link", "r_shoulder_pitch_link", "r_shoulder_roll_link",
    "r_upper_arm_link", "r_elbow_link", "r_wrist_link", "l_shoulder_pitch_link",
    "l_shoulder_roll_link", "l_upper_arm_link", "l_elbow_link", "l_wrist_link",
    "head_yaw_link", "head_pitch_link", "camera_link",
)
PIPLUS_LSE_23DOF_BFM_BODY_NAMES = (
    "base_link", "r_hip_pitch_link", "r_hip_roll_link", "r_thigh_link", "r_calf_link",
    "r_ankle_pitch_link", "r_ankle_roll_link", "l_hip_pitch_link", "l_hip_roll_link",
    "l_thigh_link", "l_calf_link", "l_ankle_pitch_link", "l_ankle_roll_link",
    "waist_yaw_link", "r_shoulder_pitch_link", "r_shoulder_roll_link", "r_upper_arm_link",
    "r_elbow_link", "l_shoulder_pitch_link", "l_shoulder_roll_link", "l_upper_arm_link",
    "l_elbow_link", "head_yaw_link", "head_pitch_link",
)

PIPLUS_LSE_23DOF_DEFAULT = {
    "r_hip_pitch_joint": -0.25, "r_hip_roll_joint": 0.0, "r_thigh_joint": 0.0,
    "r_calf_joint": 0.65, "r_ankle_pitch_joint": -0.4, "r_ankle_roll_joint": 0.0,
    "l_hip_pitch_joint": -0.25, "l_hip_roll_joint": 0.0, "l_thigh_joint": 0.0,
    "l_calf_joint": 0.65, "l_ankle_pitch_joint": -0.4, "l_ankle_roll_joint": 0.0,
    "waist_yaw_joint": 0.0, "r_shoulder_pitch_joint": 0.18, "r_shoulder_roll_joint": -0.15,
    "r_upper_arm_joint": 0.0, "r_elbow_joint": -0.5, "l_shoulder_pitch_joint": 0.18,
    "l_shoulder_roll_joint": 0.15, "l_upper_arm_joint": 0.0, "l_elbow_joint": -0.5,
    "head_yaw_joint": 0.0, "head_pitch_joint": 0.0,
}
PIPLUS_LSE_23DOF_LOWER = {
    "r_hip_pitch_joint": -3.12, "r_hip_roll_joint": -3.14, "r_thigh_joint": -2.87,
    "r_calf_joint": -2.14, "r_ankle_pitch_joint": -0.92, "r_ankle_roll_joint": -0.78,
    "l_hip_pitch_joint": -3.12, "l_hip_roll_joint": -0.17, "l_thigh_joint": -2.87,
    "l_calf_joint": -2.14, "l_ankle_pitch_joint": -0.92, "l_ankle_roll_joint": -0.78,
    "waist_yaw_joint": -2.7, "r_shoulder_pitch_joint": -4.18, "r_shoulder_roll_joint": -3.34,
    "r_upper_arm_joint": -2.16, "r_elbow_joint": -1.73, "l_shoulder_pitch_joint": -4.18,
    "l_shoulder_roll_joint": -0.13, "l_upper_arm_joint": -2.16, "l_elbow_joint": -1.73,
    "head_yaw_joint": -1.43, "head_pitch_joint": -0.94,
}
PIPLUS_LSE_23DOF_UPPER = {
    "r_hip_pitch_joint": 2.94, "r_hip_roll_joint": 0.17, "r_thigh_joint": 2.87,
    "r_calf_joint": 2.35, "r_ankle_pitch_joint": 0.97, "r_ankle_roll_joint": 0.78,
    "l_hip_pitch_joint": 2.94, "l_hip_roll_joint": 3.14, "l_thigh_joint": 2.87,
    "l_calf_joint": 2.35, "l_ankle_pitch_joint": 0.97, "l_ankle_roll_joint": 0.78,
    "waist_yaw_joint": 2.7, "r_shoulder_pitch_joint": 1.04, "r_shoulder_roll_joint": 0.13,
    "r_upper_arm_joint": 2.16, "r_elbow_joint": 1.73, "l_shoulder_pitch_joint": 1.04,
    "l_shoulder_roll_joint": 3.34, "l_upper_arm_joint": 2.16, "l_elbow_joint": 1.73,
    "head_yaw_joint": 1.43, "head_pitch_joint": 0.88,
}


HI_25DOF_JOINT_NAMES = (
    "waist_yaw_joint", "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_upper_arm_joint",
    "r_elbow_joint", "r_wrist_joint", "l_shoulder_pitch_joint", "l_shoulder_roll_joint",
    "l_upper_arm_joint", "l_elbow_joint", "l_wrist_joint", "head_yaw_joint", "head_pitch_joint",
    "r_hip_pitch_joint", "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint",
    "r_ankle_pitch_joint", "r_ankle_roll_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_thigh_joint", "l_calf_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
)
HI_25DOF_BODY_NAMES = (
    "base_link", "waist_yaw_link", "torso_link", "r_shoulder_pitch_link", "r_shoulder_roll_link",
    "r_upper_arm_link", "r_elbow_link", "r_wrist_link", "l_shoulder_pitch_link",
    "l_shoulder_roll_link", "l_upper_arm_link", "l_elbow_link", "l_wrist_link",
    "head_yaw_link", "head_pitch_link", "camera_link", "r_hip_pitch_link", "r_hip_roll_link",
    "r_thigh_link", "r_calf_link", "r_ankle_pitch_link", "r_ankle_roll_link",
    "l_hip_pitch_link", "l_hip_roll_link", "l_thigh_link", "l_calf_link",
    "l_ankle_pitch_link", "l_ankle_roll_link",
)
HI_25DOF_BFM_BODY_NAMES = (
    "base_link", "waist_yaw_link", "r_shoulder_pitch_link", "r_shoulder_roll_link",
    "r_upper_arm_link", "r_elbow_link", "r_wrist_link", "l_shoulder_pitch_link",
    "l_shoulder_roll_link", "l_upper_arm_link", "l_elbow_link", "l_wrist_link",
    "head_yaw_link", "head_pitch_link", "r_hip_pitch_link", "r_hip_roll_link",
    "r_thigh_link", "r_calf_link", "r_ankle_pitch_link", "r_ankle_roll_link",
    "l_hip_pitch_link", "l_hip_roll_link", "l_thigh_link", "l_calf_link",
    "l_ankle_pitch_link", "l_ankle_roll_link",
)
HI_25DOF_DEFAULT = {
    "waist_yaw_joint": 0.0, "r_shoulder_pitch_joint": 0.18, "r_shoulder_roll_joint": -0.15,
    "r_upper_arm_joint": 0.0, "r_elbow_joint": -0.5, "r_wrist_joint": 0.0,
    "l_shoulder_pitch_joint": 0.18, "l_shoulder_roll_joint": 0.15, "l_upper_arm_joint": 0.0,
    "l_elbow_joint": -0.5, "l_wrist_joint": 0.0, "head_yaw_joint": 0.0, "head_pitch_joint": 0.0,
    "r_hip_pitch_joint": -0.25, "r_hip_roll_joint": 0.0, "r_thigh_joint": 0.0,
    "r_calf_joint": 0.65, "r_ankle_pitch_joint": -0.4, "r_ankle_roll_joint": 0.0,
    "l_hip_pitch_joint": -0.25, "l_hip_roll_joint": 0.0, "l_thigh_joint": 0.0,
    "l_calf_joint": 0.65, "l_ankle_pitch_joint": -0.4, "l_ankle_roll_joint": 0.0,
}
HI_25DOF_LOWER = {
    "waist_yaw_joint": -2.36, "r_shoulder_pitch_joint": -3.12, "r_shoulder_roll_joint": -3.21,
    "r_upper_arm_joint": -3.05, "r_elbow_joint": -1.57, "r_wrist_joint": -3.14,
    "l_shoulder_pitch_joint": -3.12, "l_shoulder_roll_joint": 0.0, "l_upper_arm_joint": -3.05,
    "l_elbow_joint": -1.57, "l_wrist_joint": -3.14, "head_yaw_joint": -1.43, "head_pitch_joint": -0.93,
    "r_hip_pitch_joint": -2.13, "r_hip_roll_joint": -2.76, "r_thigh_joint": -0.79,
    "r_calf_joint": 0.0, "r_ankle_pitch_joint": -0.91, "r_ankle_roll_joint": -0.38,
    "l_hip_pitch_joint": -2.13, "l_hip_roll_joint": -0.3, "l_thigh_joint": -0.79,
    "l_calf_joint": 0.0, "l_ankle_pitch_joint": -0.91, "l_ankle_roll_joint": -0.38,
}
HI_25DOF_UPPER = {
    "waist_yaw_joint": 2.36, "r_shoulder_pitch_joint": 2.95, "r_shoulder_roll_joint": 0.0,
    "r_upper_arm_joint": 3.05, "r_elbow_joint": 0.0, "r_wrist_joint": 3.14,
    "l_shoulder_pitch_joint": 2.95, "l_shoulder_roll_joint": 3.12, "l_upper_arm_joint": 3.05,
    "l_elbow_joint": 0.0, "l_wrist_joint": 3.14, "head_yaw_joint": 1.43, "head_pitch_joint": 1.28,
    "r_hip_pitch_joint": 2.07, "r_hip_roll_joint": 0.3, "r_thigh_joint": 0.79,
    "r_calf_joint": 1.9, "r_ankle_pitch_joint": 0.79, "r_ankle_roll_joint": 0.38,
    "l_hip_pitch_joint": 2.07, "l_hip_roll_joint": 2.76, "l_thigh_joint": 0.79,
    "l_calf_joint": 1.9, "l_ankle_pitch_joint": 0.79, "l_ankle_roll_joint": 0.38,
}


def _gains(joints: tuple[str, ...], *, high: float, ankle: float, arm: float, wrist: float) -> tuple[dict[str, float], dict[str, float]]:
    kp: dict[str, float] = {}
    kd: dict[str, float] = {}
    for name in joints:
        if name.startswith("head_"):
            value, damping = 7.80094, 0.49662
        elif "wrist" in name:
            value, damping = wrist, 2.06943
        elif any(part in name for part in ("shoulder", "upper_arm", "elbow")):
            value, damping = arm, 3.32054 if arm > 40 else 2.06943
        elif "ankle" in name:
            value, damping = ankle, 3.32054
        else:
            value, damping = high, 8.35391 if high > 100 else (2.2343 if "waist" in name else 3.32054)
        kp[name] = value
        kd[name] = damping
    return kp, kd


_PIPLUS_KP, _PIPLUS_KD = _gains(PIPLUS_LSE_23DOF_JOINT_NAMES, high=52.15889, ankle=52.15889, arm=32.50653, wrist=32.50653)
_HI_KP, _HI_KD = _gains(HI_25DOF_JOINT_NAMES, high=131.22294, ankle=52.15889, arm=52.15889, wrist=32.50653)
_PIPLUS_ARMATURE = {
    **{name: 0.013212 for name in PIPLUS_LSE_23DOF_JOINT_NAMES[:12]},
    "waist_yaw_joint": 0.008890,
    **{name: 0.008234 for name in PIPLUS_LSE_23DOF_JOINT_NAMES[13:21]},
    "head_yaw_joint": 0.001976,
    "head_pitch_joint": 0.001976,
}
_HI_ARMATURE = {
    **{name: 0.03323916 for name in HI_25DOF_JOINT_NAMES[:1]},
    **{name: 0.013212 for name in HI_25DOF_JOINT_NAMES[1:5]},
    "r_wrist_joint": 0.008234,
    **{name: 0.013212 for name in HI_25DOF_JOINT_NAMES[6:10]},
    "l_wrist_joint": 0.008234,
    "head_yaw_joint": 0.001976,
    "head_pitch_joint": 0.001976,
    **{name: 0.03323916 for name in HI_25DOF_JOINT_NAMES[13:17]},
    **{name: 0.013212 for name in HI_25DOF_JOINT_NAMES[17:19]},
    **{name: 0.03323916 for name in HI_25DOF_JOINT_NAMES[19:23]},
    **{name: 0.013212 for name in HI_25DOF_JOINT_NAMES[23:25]},
}

PIPLUS_LSE_23DOF_CFG = RobotCfg(
    name="piplus_lse_23dof",
    joint_names=PIPLUS_LSE_23DOF_JOINT_NAMES,
    body_names=PIPLUS_LSE_23DOF_BODY_NAMES,
    joint_pos_lower_limit=PIPLUS_LSE_23DOF_LOWER,
    joint_pos_upper_limit=PIPLUS_LSE_23DOF_UPPER,
    joint_velocity_limit={
        **{name: 7.12 for name in PIPLUS_LSE_23DOF_JOINT_NAMES[:12]},
        "waist_yaw_joint": 10.47,
        **{name: 15.91 for name in PIPLUS_LSE_23DOF_JOINT_NAMES[13:21]},
        "head_yaw_joint": 30.37,
        "head_pitch_joint": 30.37,
    },
    joint_effort_limit={name: 3.0 if name.startswith("head_") else 20.0 for name in PIPLUS_LSE_23DOF_JOINT_NAMES},
    safe_joint_kp=_PIPLUS_KP,
    safe_joint_kd=_PIPLUS_KD,
    joint_armature=_PIPLUS_ARMATURE,
    joint_frictionloss={name: 0.02 for name in PIPLUS_LSE_23DOF_JOINT_NAMES},
    mjcf_path=_asset_path("PiPlus_S_12L8A0G2H1W_LSE_260611", "PiPlus_S_12L8A0G2H1W_LSE_260611.xml", "SIM2REAL_PIPLUS_LSE_MJCF"),
    default_qpos=(0.0, 0.0, 0.38, 1.0, 0.0, 0.0, 0.0, *tuple(PIPLUS_LSE_23DOF_DEFAULT[name] for name in PIPLUS_LSE_23DOF_JOINT_NAMES)),
    publish_hz=50.0,
    interface=None,
    hardware_joint_names=PIPLUS_LSE_23DOF_HARDWARE_JOINT_NAMES,
    hardware_joint_map=PIPLUS_LSE_23DOF_HARDWARE_JOINT_MAP,
    hardware_joint_signs=PIPLUS_LSE_23DOF_HARDWARE_JOINT_SIGNS,
    viewer_track_body_names=("base_link",),
    elastic_band_attach_body_names=("torso_link", "base_link"),
    imu_body_names=("waist_yaw_link", "base_link"),
)

HI_25DOF_CFG = RobotCfg(
    name="hi_25dof",
    joint_names=HI_25DOF_JOINT_NAMES,
    body_names=HI_25DOF_BODY_NAMES,
    joint_pos_lower_limit=HI_25DOF_LOWER,
    joint_pos_upper_limit=HI_25DOF_UPPER,
    joint_velocity_limit={name: 300.0 for name in HI_25DOF_JOINT_NAMES},
    joint_effort_limit={
        **{name: 21.0 for name in HI_25DOF_JOINT_NAMES[0:5]},
        "r_wrist_joint": 10.0,
        **{name: 21.0 for name in HI_25DOF_JOINT_NAMES[6:10]},
        "l_wrist_joint": 10.0,
        "head_yaw_joint": 3.5,
        "head_pitch_joint": 3.5,
        **{name: 36.0 for name in HI_25DOF_JOINT_NAMES[13:]},
    },
    safe_joint_kp=_HI_KP,
    safe_joint_kd=_HI_KD,
    joint_armature=_HI_ARMATURE,
    joint_frictionloss={name: 0.02 for name in HI_25DOF_JOINT_NAMES},
    mjcf_path=_asset_path("HiPro_S_12L10A0G2H1W_FootBall_260611", "HiPro_S_12L10A0G2H1W_FootBall_260611.xml", "SIM2REAL_HI_MJCF"),
    default_qpos=(0.0, 0.0, 0.4315, 1.0, 0.0, 0.0, 0.0, *tuple(HI_25DOF_DEFAULT[name] for name in HI_25DOF_JOINT_NAMES)),
    publish_hz=50.0,
    interface=None,
    hardware_joint_names=HI_25DOF_JOINT_NAMES,
    hardware_joint_map=tuple(range(len(HI_25DOF_JOINT_NAMES))),
    hardware_joint_signs=(1.0,) * len(HI_25DOF_JOINT_NAMES),
    viewer_track_body_names=("base_link",),
    elastic_band_attach_body_names=("torso_link", "base_link"),
    imu_body_names=("waist_yaw_link", "base_link"),
)


__all__ = [
    "PIPLUS_LSE_23DOF_CFG", "PIPLUS_LSE_23DOF_JOINT_NAMES", "PIPLUS_LSE_23DOF_BFM_BODY_NAMES",
    "PIPLUS_LSE_23DOF_HARDWARE_JOINT_NAMES", "PIPLUS_LSE_23DOF_HARDWARE_JOINT_MAP",
    "HI_25DOF_CFG", "HI_25DOF_JOINT_NAMES", "HI_25DOF_BFM_BODY_NAMES",
]
