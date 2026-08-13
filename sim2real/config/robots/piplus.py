from __future__ import annotations

import os
from pathlib import Path

from sim2real.config.robots.base import PROJECT_ROOT, RobotCfg

PIPLUS_MJCF_ENV = "SIM2REAL_PIPLUS_MJCF"
PIPLUS_MODEL_NAME = "PiPlus_S_12L8A0G2H0W"
PIPLUS_MJCF_FILENAME = f"{PIPLUS_MODEL_NAME}_with_armature.xml"


def _resolve_piplus_mjcf_path() -> str:
    override = os.environ.get(PIPLUS_MJCF_ENV)
    if override:
        return str(Path(override).expanduser().resolve())

    candidates = (
        PROJECT_ROOT / "assets" / PIPLUS_MODEL_NAME / "xml" / PIPLUS_MJCF_FILENAME,
        PROJECT_ROOT.parent.parent
        / "Assets"
        / "ht_urdf"
        / "ht_urdf"
        / PIPLUS_MODEL_NAME
        / "xml"
        / PIPLUS_MJCF_FILENAME,
    )
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate.resolve())
    return str(candidates[0])


PIPLUS_H0W_JOINT_NAMES = (
    "r_shoulder_pitch_joint",
    "r_shoulder_roll_joint",
    "r_upper_arm_joint",
    "r_elbow_joint",
    "l_shoulder_pitch_joint",
    "l_shoulder_roll_joint",
    "l_upper_arm_joint",
    "l_elbow_joint",
    "head_yaw_joint",
    "head_pitch_joint",
    "r_hip_pitch_joint",
    "r_hip_roll_joint",
    "r_thigh_joint",
    "r_calf_joint",
    "r_ankle_pitch_joint",
    "r_ankle_roll_joint",
    "l_hip_pitch_joint",
    "l_hip_roll_joint",
    "l_thigh_joint",
    "l_calf_joint",
    "l_ankle_pitch_joint",
    "l_ankle_roll_joint",
)

# ROS ``JointState.position`` order from instinct_onboard's PiPlus BFM node.
# ``hardware_joint_map[policy_idx]`` selects the corresponding ROS index.
PIPLUS_H0W_HARDWARE_JOINT_NAMES = (
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
)
PIPLUS_H0W_HARDWARE_JOINT_MAP = (
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
)
PIPLUS_H0W_HARDWARE_JOINT_SIGNS = (1.0,) * len(PIPLUS_H0W_JOINT_NAMES)

PIPLUS_H0W_BODY_NAMES = (
    "base_link",
    "torso_link",
    "r_shoulder_pitch_link",
    "r_shoulder_roll_link",
    "r_upper_arm_link",
    "r_elbow_link",
    "r_wrist_link",
    "l_shoulder_pitch_link",
    "l_shoulder_roll_link",
    "l_upper_arm_link",
    "l_elbow_link",
    "l_wrist_link",
    "head_yaw_link",
    "head_pitch_link",
    "camera_link",
    "r_hip_pitch_link",
    "r_hip_roll_link",
    "r_thigh_link",
    "r_calf_link",
    "r_ankle_pitch_link",
    "r_ankle_roll_link",
    "l_hip_pitch_link",
    "l_hip_roll_link",
    "l_thigh_link",
    "l_calf_link",
    "l_ankle_pitch_link",
    "l_ankle_roll_link",
)

PIPLUS_H0W_DEFAULT = {
    "l_hip_pitch_joint": -0.25,
    "r_hip_pitch_joint": -0.25,
    "l_hip_roll_joint": 0.0,
    "r_hip_roll_joint": 0.0,
    "head_yaw_joint": 0.0,
    "l_shoulder_pitch_joint": 0.18,
    "r_shoulder_pitch_joint": 0.18,
    "l_thigh_joint": 0.0,
    "r_thigh_joint": 0.0,
    "head_pitch_joint": 0.0,
    "l_shoulder_roll_joint": 0.15,
    "r_shoulder_roll_joint": -0.15,
    "l_calf_joint": 0.65,
    "r_calf_joint": 0.65,
    "l_upper_arm_joint": 0.0,
    "r_upper_arm_joint": 0.0,
    "l_ankle_pitch_joint": -0.4,
    "r_ankle_pitch_joint": -0.4,
    "l_elbow_joint": -0.5,
    "r_elbow_joint": -0.5,
    "l_ankle_roll_joint": 0.0,
    "r_ankle_roll_joint": 0.0,
}

PIPLUS_H0W_LOWER = {
    "r_shoulder_pitch_joint": -4.18,
    "r_shoulder_roll_joint": -3.34,
    "r_upper_arm_joint": -2.16,
    "r_elbow_joint": -2.0,
    "l_shoulder_pitch_joint": -4.18,
    "l_shoulder_roll_joint": -0.13,
    "l_upper_arm_joint": -2.16,
    "l_elbow_joint": -2.0,
    "head_yaw_joint": -1.43,
    "head_pitch_joint": -1.23,
    "r_hip_pitch_joint": -1.76,
    "r_hip_roll_joint": -2.46,
    "r_thigh_joint": -2.87,
    "r_calf_joint": -2.14,
    "r_ankle_pitch_joint": -0.97,
    "r_ankle_roll_joint": -0.78,
    "l_hip_pitch_joint": -1.76,
    "l_hip_roll_joint": -0.17,
    "l_thigh_joint": -2.87,
    "l_calf_joint": -2.14,
    "l_ankle_pitch_joint": -0.97,
    "l_ankle_roll_joint": -0.78,
}

PIPLUS_H0W_UPPER = {
    "r_shoulder_pitch_joint": 1.04,
    "r_shoulder_roll_joint": 0.13,
    "r_upper_arm_joint": 2.16,
    "r_elbow_joint": 2.0,
    "l_shoulder_pitch_joint": 1.04,
    "l_shoulder_roll_joint": 3.34,
    "l_upper_arm_joint": 2.16,
    "l_elbow_joint": 2.0,
    "head_yaw_joint": 1.43,
    "head_pitch_joint": 1.01,
    "r_hip_pitch_joint": 1.76,
    "r_hip_roll_joint": 0.17,
    "r_thigh_joint": 2.87,
    "r_calf_joint": 2.35,
    "r_ankle_pitch_joint": 0.92,
    "r_ankle_roll_joint": 0.78,
    "l_hip_pitch_joint": 1.76,
    "l_hip_roll_joint": 2.46,
    "l_thigh_joint": 2.87,
    "l_calf_joint": 2.35,
    "l_ankle_pitch_joint": 0.92,
    "l_ankle_roll_joint": 0.78,
}

PIPLUS_H0W_EFFORT = {
    **{name: 20.0 for name in PIPLUS_H0W_JOINT_NAMES},
    "head_yaw_joint": 3.0,
    "head_pitch_joint": 3.0,
}
PIPLUS_H0W_ARMATURE = {
    **{name: 0.008234 for name in PIPLUS_H0W_JOINT_NAMES[:8]},
    "head_yaw_joint": 0.001976,
    "head_pitch_joint": 0.001976,
    **{name: 0.013212 for name in PIPLUS_H0W_JOINT_NAMES[10:]},
}
PIPLUS_H0W_FRICTION = {name: 0.02 for name in PIPLUS_H0W_JOINT_NAMES}
PIPLUS_H0W_KP = {
    **{name: 32.50653 for name in PIPLUS_H0W_JOINT_NAMES[:8]},
    "head_yaw_joint": 7.80094,
    "head_pitch_joint": 7.80094,
    **{name: 52.15889 for name in PIPLUS_H0W_JOINT_NAMES[10:]},
}
PIPLUS_H0W_KD = {
    **{name: 2.06943 for name in PIPLUS_H0W_JOINT_NAMES[:8]},
    "head_yaw_joint": 0.49662,
    "head_pitch_joint": 0.49662,
    **{name: 3.32054 for name in PIPLUS_H0W_JOINT_NAMES[10:]},
}


PIPLUS_H0W_CFG = RobotCfg(
    name="piplus_h0w",
    joint_names=PIPLUS_H0W_JOINT_NAMES,
    body_names=PIPLUS_H0W_BODY_NAMES,
    joint_pos_lower_limit=PIPLUS_H0W_LOWER,
    joint_pos_upper_limit=PIPLUS_H0W_UPPER,
    joint_velocity_limit={name: 60.0 for name in PIPLUS_H0W_JOINT_NAMES},
    joint_effort_limit=PIPLUS_H0W_EFFORT,
    safe_joint_kp=PIPLUS_H0W_KP,
    safe_joint_kd=PIPLUS_H0W_KD,
    joint_armature=PIPLUS_H0W_ARMATURE,
    joint_frictionloss=PIPLUS_H0W_FRICTION,
    mjcf_path=_resolve_piplus_mjcf_path(),
    default_qpos=(
        0.0,
        0.0,
        0.38,
        1.0,
        0.0,
        0.0,
        0.0,
        *tuple(PIPLUS_H0W_DEFAULT[name] for name in PIPLUS_H0W_JOINT_NAMES),
    ),
    publish_hz=50.0,
    interface=None,
    hardware_joint_names=PIPLUS_H0W_HARDWARE_JOINT_NAMES,
    hardware_joint_map=PIPLUS_H0W_HARDWARE_JOINT_MAP,
    hardware_joint_signs=PIPLUS_H0W_HARDWARE_JOINT_SIGNS,
    viewer_track_body_names=("base_link",),
    elastic_band_attach_body_names=("torso_link", "base_link"),
)
