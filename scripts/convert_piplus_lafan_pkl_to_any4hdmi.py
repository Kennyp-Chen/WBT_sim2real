"""Convert a PiPlus HumanoidVerse LAFAN pickle into any4hdmi qpos motions."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import joblib
import numpy as np
from any4hdmi.core.format import save_motion, write_manifest
from any4hdmi.dataset.interpolation import resampled_length, slerp
from sim2real.config.robots import get_robot_cfg

PIPLUS_H0W_JOINT_NAMES = [
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
]
PIPLUS_LSE_23DOF_JOINT_NAMES = [
    "r_hip_pitch_joint", "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint",
    "r_ankle_pitch_joint", "r_ankle_roll_joint", "l_hip_pitch_joint", "l_hip_roll_joint",
    "l_thigh_joint", "l_calf_joint", "l_ankle_pitch_joint", "l_ankle_roll_joint",
    "waist_yaw_joint", "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_upper_arm_joint",
    "r_elbow_joint", "l_shoulder_pitch_joint", "l_shoulder_roll_joint", "l_upper_arm_joint",
    "l_elbow_joint", "head_yaw_joint", "head_pitch_joint",
]
HI_25DOF_JOINT_NAMES = [
    "waist_yaw_joint", "r_shoulder_pitch_joint", "r_shoulder_roll_joint", "r_upper_arm_joint",
    "r_elbow_joint", "r_wrist_joint", "l_shoulder_pitch_joint", "l_shoulder_roll_joint",
    "l_upper_arm_joint", "l_elbow_joint", "l_wrist_joint", "head_yaw_joint", "head_pitch_joint",
    "r_hip_pitch_joint", "r_hip_roll_joint", "r_thigh_joint", "r_calf_joint", "r_ankle_pitch_joint",
    "r_ankle_roll_joint", "l_hip_pitch_joint", "l_hip_roll_joint", "l_thigh_joint", "l_calf_joint",
    "l_ankle_pitch_joint", "l_ankle_roll_joint",
]

ROBOT_JOINT_NAMES = {
    "piplus_h0w": PIPLUS_H0W_JOINT_NAMES,
    "piplus_lse_23dof": PIPLUS_LSE_23DOF_JOINT_NAMES,
    "hi_25dof": HI_25DOF_JOINT_NAMES,
}


def _safe_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._") or "motion"


def _resample_qpos(
    qpos: np.ndarray, source_fps: float, target_fps: float
) -> np.ndarray:
    if np.isclose(source_fps, target_fps):
        return qpos
    target_length = resampled_length(
        qpos.shape[0], source_fps=source_fps, target_fps=target_fps
    )
    source_times = np.arange(qpos.shape[0], dtype=np.float64) / float(source_fps)
    target_times = np.arange(target_length, dtype=np.float64) / float(target_fps)
    output = np.empty((target_length, qpos.shape[1]), dtype=np.float32)
    for start, end in ((0, 3), (7, qpos.shape[1])):
        output[:, start:end] = np.stack(
            [
                np.interp(target_times, source_times, qpos[:, dim])
                for dim in range(start, end)
            ],
            axis=-1,
        )
    output[:, 3:7] = slerp(target_times, source_times, qpos[:, 3:7]).astype(np.float32)
    return output


def convert(
    source_path: Path, output_root: Path, mjcf_path: Path, target_fps: float, robot: str
) -> None:
    robot_cfg = get_robot_cfg(robot)
    joint_names = list(robot_cfg.joint_names)
    source = joblib.load(source_path)
    if not isinstance(source, dict) or not source:
        raise ValueError(f"Expected a non-empty motion dict, got {type(source)!r}")
    output_root.mkdir(parents=True, exist_ok=True)
    motion_root = output_root / "motions"
    motion_root.mkdir(parents=True, exist_ok=True)
    used_names: set[str] = set()
    total_frames = 0
    entries: list[dict[str, object]] = []

    for source_name, payload in source.items():
        if not isinstance(payload, dict):
            raise ValueError(f"Motion {source_name!r} is not a dict")
        source_joint_names = [str(name) for name in payload["joint_names"]]
        if set(source_joint_names) != set(joint_names):
            missing = [name for name in joint_names if name not in source_joint_names]
            extra = [name for name in source_joint_names if name not in joint_names]
            raise ValueError(
                f"Motion {source_name!r} joint mismatch: missing={missing}, extra={extra}"
            )
        joint_indices = [source_joint_names.index(name) for name in joint_names]
        root_pos = np.asarray(payload["root_trans_offset"], dtype=np.float32)
        root_rot_xyzw = np.asarray(payload["root_rot"], dtype=np.float32)
        dof = np.asarray(payload["dof"], dtype=np.float32)[:, joint_indices]
        if root_pos.ndim != 2 or root_pos.shape[1] != 3:
            raise ValueError(
                f"Motion {source_name!r} root_trans_offset shape={root_pos.shape}"
            )
        if root_rot_xyzw.shape != (root_pos.shape[0], 4):
            raise ValueError(
                f"Motion {source_name!r} root_rot shape={root_rot_xyzw.shape}"
            )
        if dof.shape != (root_pos.shape[0], len(joint_names)):
            raise ValueError(f"Motion {source_name!r} dof shape={dof.shape}")

        # HumanoidVerse stores root rotations as xyzw; any4hdmi qpos is wxyz.
        qpos = np.concatenate(
            [root_pos, root_rot_xyzw[:, [3, 0, 1, 2]], dof], axis=-1
        ).astype(np.float32)
        source_fps = float(payload.get("fps", 30.0))
        qpos = _resample_qpos(qpos, source_fps, target_fps)
        motion_name = _safe_name(str(source_name))
        candidate = motion_name
        suffix = 1
        while candidate in used_names:
            suffix += 1
            candidate = f"{motion_name}_{suffix}"
        used_names.add(candidate)
        save_motion(motion_root / f"{candidate}.npz", qpos)
        total_frames += int(qpos.shape[0])
        entries.append(
            {
                "source_name": str(source_name),
                "filename": f"{candidate}.npz",
                "frames": int(qpos.shape[0]),
                "source_fps": source_fps,
                "target_fps": target_fps,
            }
        )
    qpos_names = [
        "root_tx",
        "root_ty",
        "root_tz",
        "root_qw",
        "root_qx",
        "root_qy",
        "root_qz",
        *joint_names,
    ]
    write_manifest(
        output_root,
        dataset_name=f"{robot_cfg.name}_lafan",
        mjcf=mjcf_path,
        timestep=1.0 / target_fps,
        qpos_names=qpos_names,
        num_motions=len(entries),
        source={
            "format": "HumanoidVerse LAFAN pickle",
            "pickle": str(source_path.resolve()),
            "target_fps": target_fps,
            "root_representation": "xyz + qx qy qz qw converted to xyz + qw qx qy qz",
            "robot": robot_cfg.name,
            "joint_names": joint_names,
            "entries": entries,
        },
        total_hours=total_frames / target_fps / 3600.0,
    )
    (output_root / "conversion.json").write_text(
        json.dumps(
            {
                "source": str(source_path.resolve()),
                "mjcf": str(mjcf_path.resolve()),
                "motions": entries,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"Converted {len(entries)} motions / {total_frames} frames to {output_root}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--target-fps", type=float, default=50.0)
    parser.add_argument("--robot", type=str, default="piplus_h0w")
    args = parser.parse_args()
    convert(args.source, args.output, args.mjcf, args.target_fps, args.robot)


if __name__ == "__main__":
    main()
