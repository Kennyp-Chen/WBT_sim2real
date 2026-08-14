"""Retarget GEM SMPL-X output to a robot any4hdmi motion and joint reference."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import torch
from any4hdmi.core.format import save_motion, write_manifest
from scipy.spatial.transform import Rotation, Slerp

from sim2real.config.robots import get_robot_cfg


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GEM_PATH = Path(
    "/home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt"
)
DEFAULT_SMPLX_ROOT = Path(
    "/home/sunteng/Projects/WBC_Telep/GENMO/inputs/checkpoints/body_models"
)
PIPLUS_GMR_CONFIG = (
    REPO_ROOT / "sim2real/teleop/gmr_configs/smplx_to_piplus_h0w.json"
)
GMR_ROBOT_NAMES = {"g1": "unitree_g1", "piplus_h0w": "piplus_h0w"}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Retarget GEM body_params_global to G1 or PiPlus qpos."
    )
    parser.add_argument("--gem-params", type=Path, default=DEFAULT_GEM_PATH)
    parser.add_argument("--robot", choices=sorted(GMR_ROBOT_NAMES), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--motion-name", default=None)
    parser.add_argument("--smplx-model-root", type=Path, default=DEFAULT_SMPLX_ROOT)
    parser.add_argument("--source-fps", type=float, default=30.0)
    parser.add_argument("--target-fps", type=float, default=50.0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--ankle-height", type=float, default=0.055)
    return parser.parse_args()


def _as_finite_array(value: Any, name: str) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    array = np.asarray(value, dtype=np.float32)
    if not np.isfinite(array).all():
        raise ValueError(f"{name} contains NaN or infinity")
    return array


def _load_gem_params(
    path: Path, max_frames: int | None
) -> dict[str, np.ndarray]:
    payload = torch.load(str(path), map_location="cpu", weights_only=False)
    if not isinstance(payload, dict):
        raise ValueError(f"Expected GEM dictionary, got {type(payload)!r}")
    params = payload.get("body_params_global")
    if not isinstance(params, dict):
        raise ValueError("GEM file is missing body_params_global")
    required = ("body_pose", "global_orient", "transl", "betas")
    missing = [name for name in required if name not in params]
    if missing:
        raise ValueError(f"GEM body_params_global is missing {missing}")

    result = {name: _as_finite_array(params[name], name) for name in required}
    result["body_pose"] = result["body_pose"].reshape(-1, 63)
    result["global_orient"] = result["global_orient"].reshape(-1, 3)
    result["transl"] = result["transl"].reshape(-1, 3)
    result["betas"] = result["betas"].reshape(-1, 10)
    frames = result["body_pose"].shape[0]
    if any(value.shape[0] != frames for value in result.values()):
        raise ValueError(
            "GEM frame count mismatch: "
            + ", ".join(f"{key}={value.shape}" for key, value in result.items())
        )
    if max_frames is not None:
        if max_frames <= 0:
            raise ValueError("max-frames must be positive")
        result = {key: value[:max_frames] for key, value in result.items()}
    if not result["body_pose"].shape[0]:
        raise ValueError("GEM file contains no frames")
    return result


def _gem_to_gmr_frames(
    params: dict[str, np.ndarray], smplx_model_root: Path, fps: float
) -> tuple[list[dict[str, tuple[np.ndarray, np.ndarray]]], float]:
    try:
        import smplx
        from general_motion_retargeting.utils.smpl import (
            get_gvhmr_data_offline_fast,
        )
    except ImportError as exc:
        raise RuntimeError(
            "GMR dependencies are missing. Install the project's retarget extra."
        ) from exc

    frames = params["body_pose"].shape[0]
    model = smplx.create(
        str(smplx_model_root),
        model_type="smplx",
        gender="neutral",
        use_pca=False,
        num_betas=10,
        batch_size=frames,
    )
    body_params = {
        key: torch.from_numpy(value).float() for key, value in params.items()
    }
    zeros45 = torch.zeros(frames, 45, dtype=torch.float32)
    zeros3 = torch.zeros(frames, 3, dtype=torch.float32)
    with torch.no_grad():
        output = model(
            **body_params,
            left_hand_pose=zeros45,
            right_hand_pose=zeros45,
            jaw_pose=zeros3,
            leye_pose=zeros3,
            reye_pose=zeros3,
            return_full_pose=True,
        )
    smplx_data = {
        "pose_body": params["body_pose"],
        "mocap_frame_rate": torch.tensor(float(fps)),
    }
    human_frames, aligned_fps = get_gvhmr_data_offline_fast(
        smplx_data, model, output, tgt_fps=float(fps)
    )
    return human_frames, float(aligned_fps)


def _make_retargeter(robot: str, actual_human_height: float):
    from general_motion_retargeting import GeneralMotionRetargeting as GMR
    from general_motion_retargeting import IK_CONFIG_DICT, ROBOT_XML_DICT

    robot_cfg = get_robot_cfg(robot)
    if robot == "piplus_h0w":
        ROBOT_XML_DICT["piplus_h0w"] = Path(robot_cfg.mjcf_path)
        IK_CONFIG_DICT.setdefault("smplx", {})["piplus_h0w"] = PIPLUS_GMR_CONFIG
    retargeter = GMR(
        src_human="smplx",
        tgt_robot=GMR_ROBOT_NAMES[robot],
        actual_human_height=float(actual_human_height),
        verbose=False,
    )
    if retargeter.configuration.model.nq == robot_cfg.qpos_size:
        retargeter.configuration.update(np.asarray(robot_cfg.default_qpos, dtype=float))
    return retargeter


def _retarget_frames(
    human_frames: list[dict[str, tuple[np.ndarray, np.ndarray]]],
    *,
    robot: str,
    actual_human_height: float,
) -> tuple[np.ndarray, Any]:
    retargeter = _make_retargeter(robot, actual_human_height)
    qpos = np.empty(
        (len(human_frames), retargeter.configuration.model.nq), dtype=np.float32
    )
    for index, human_frame in enumerate(human_frames):
        qpos[index] = retargeter.retarget(human_frame, offset_to_ground=False)
        if (index + 1) % 50 == 0 or index + 1 == len(human_frames):
            print(f"[retarget] {index + 1}/{len(human_frames)} frames", flush=True)
    if not np.isfinite(qpos).all():
        raise ValueError("Retargeted qpos contains NaN or infinity")
    return qpos, retargeter


def _joint_qpos_indices(model: mujoco.MjModel, joint_names: list[str]) -> np.ndarray:
    indices = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"GMR robot model is missing joint {name}")
        indices.append(int(model.jnt_qposadr[joint_id]))
    return np.asarray(indices, dtype=int)


def _normalize_root_quat(qpos: np.ndarray) -> None:
    norms = np.linalg.norm(qpos[:, 3:7], axis=-1, keepdims=True)
    if np.any(norms < 1.0e-8):
        raise ValueError("Retargeted root contains a zero quaternion")
    qpos[:, 3:7] /= norms
    for index in range(1, qpos.shape[0]):
        if np.dot(qpos[index - 1, 3:7], qpos[index, 3:7]) < 0:
            qpos[index, 3:7] *= -1


def _align_piplus_root_height(
    qpos: np.ndarray, model: mujoco.MjModel, ankle_height: float
) -> float:
    ankle_ids = [
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        for name in ("l_ankle_roll_link", "r_ankle_roll_link")
    ]
    if any(body_id < 0 for body_id in ankle_ids):
        raise ValueError("PiPlus MJCF is missing ankle-roll bodies")
    data = mujoco.MjData(model)
    lowest = np.empty(qpos.shape[0], dtype=np.float32)
    for index, frame in enumerate(qpos):
        data.qpos[:] = frame
        mujoco.mj_forward(model, data)
        lowest[index] = np.min(data.xpos[ankle_ids, 2])
    z_offset = float(ankle_height - np.median(lowest))
    qpos[:, 2] += z_offset
    return z_offset


def _resample_qpos(qpos: np.ndarray, source_fps: float, target_fps: float) -> np.ndarray:
    if np.isclose(source_fps, target_fps):
        return qpos.copy()
    duration = (qpos.shape[0] - 1) / source_fps
    target_frames = int(round(duration * target_fps)) + 1
    source_time = np.arange(qpos.shape[0], dtype=np.float64) / source_fps
    target_time = np.arange(target_frames, dtype=np.float64) / target_fps
    target_time[-1] = min(target_time[-1], source_time[-1])
    output = np.empty((target_frames, qpos.shape[1]), dtype=np.float32)
    for column in (*range(3), *range(7, qpos.shape[1])):
        output[:, column] = np.interp(target_time, source_time, qpos[:, column])
    rotation = Rotation.from_quat(qpos[:, [4, 5, 6, 3]])
    quat_xyzw = Slerp(source_time, rotation)(target_time).as_quat()
    output[:, 3:7] = quat_xyzw[:, [3, 0, 1, 2]]
    return output


def _validate_limits(
    qpos: np.ndarray, model: mujoco.MjModel, joint_names: list[str]
) -> dict[str, float | int]:
    indices = _joint_qpos_indices(model, joint_names)
    violations = 0
    largest = 0.0
    for name, qpos_index in zip(joint_names, indices):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if not bool(model.jnt_limited[joint_id]):
            continue
        lower, upper = model.jnt_range[joint_id]
        values = qpos[:, qpos_index]
        error = np.maximum(lower - values, values - upper)
        count = int(np.count_nonzero(error > 1.0e-5))
        violations += count
        largest = max(largest, float(np.max(error)))
    return {"joint_limit_violations": violations, "largest_limit_error": largest}


def _continuity_stats(qpos: np.ndarray, fps: float) -> dict[str, float]:
    if qpos.shape[0] < 2:
        return {
            "max_joint_step_rad": 0.0,
            "p99_joint_speed_rad_s": 0.0,
            "max_root_translation_step_m": 0.0,
            "max_root_rotation_step_rad": 0.0,
        }
    joint_step = np.diff(qpos[:, 7:], axis=0)
    root_step = np.diff(qpos[:, :3], axis=0)
    root_rotation = Rotation.from_quat(qpos[:, [4, 5, 6, 3]])
    root_rotation_step = (root_rotation[:-1].inv() * root_rotation[1:]).magnitude()
    return {
        "max_joint_step_rad": float(np.max(np.abs(joint_step))),
        "p99_joint_speed_rad_s": float(
            np.quantile(np.abs(joint_step) * float(fps), 0.99)
        ),
        "max_root_translation_step_m": float(
            np.max(np.linalg.norm(root_step, axis=-1))
        ),
        "max_root_rotation_step_rad": float(np.max(root_rotation_step)),
    }


def retarget_gem_params_to_qpos(
    gem_path: Path,
    *,
    robot: str,
    smplx_model_root: Path = DEFAULT_SMPLX_ROOT,
    source_fps: float = 30.0,
    max_frames: int | None = None,
    ankle_height: float = 0.055,
) -> tuple[np.ndarray, float, float, mujoco.MjModel, float]:
    """Convert GEM global SMPL-X parameters to RobotCfg-ordered source qpos.

    The returned qpos stays at the source clock. Consumers that publish or
    infer at another rate should resample it with the runtime motion loader.
    """
    if source_fps <= 0:
        raise ValueError("source_fps must be positive")
    params = _load_gem_params(gem_path.expanduser().resolve(), max_frames)
    human_height = 1.66 + 0.1 * float(np.median(params["betas"][:, 0]))
    human_frames, aligned_fps = _gem_to_gmr_frames(
        params, smplx_model_root.expanduser().resolve(), source_fps
    )
    qpos_gmr, retargeter = _retarget_frames(
        human_frames,
        robot=robot,
        actual_human_height=human_height,
    )

    robot_cfg = get_robot_cfg(robot)
    gmr_model = retargeter.configuration.model
    joint_names = list(robot_cfg.joint_names)
    gmr_joint_indices = _joint_qpos_indices(gmr_model, joint_names)
    qpos_source = np.concatenate(
        [qpos_gmr[:, :7], qpos_gmr[:, gmr_joint_indices]], axis=-1
    ).astype(np.float32)

    configured_mjcf = Path(robot_cfg.mjcf_path)
    model = (
        mujoco.MjModel.from_xml_path(str(configured_mjcf))
        if configured_mjcf.is_file()
        else gmr_model
    )
    _normalize_root_quat(qpos_source)
    qpos_source[:, :2] -= qpos_source[:1, :2]
    z_offset = 0.0
    if robot == "piplus_h0w":
        z_offset = _align_piplus_root_height(
            qpos_source, model, float(ankle_height)
        )
    return qpos_source, float(aligned_fps), human_height, model, z_offset


def main() -> None:
    args = _parse_args()
    if args.source_fps <= 0 or args.target_fps <= 0:
        raise ValueError("source-fps and target-fps must be positive")
    gem_path = args.gem_params.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    smplx_root = args.smplx_model_root.expanduser().resolve()
    if not gem_path.is_file():
        raise FileNotFoundError(gem_path)
    if not smplx_root.is_dir():
        raise FileNotFoundError(smplx_root)

    qpos_source, source_fps, human_height, model, z_offset = retarget_gem_params_to_qpos(
        gem_path,
        robot=args.robot,
        smplx_model_root=smplx_root,
        source_fps=args.source_fps,
        max_frames=args.max_frames,
        ankle_height=args.ankle_height,
    )
    robot_cfg = get_robot_cfg(args.robot)
    joint_names = list(robot_cfg.joint_names)
    joint_indices = _joint_qpos_indices(model, joint_names)

    output_dir.mkdir(parents=True, exist_ok=True)
    motion_dir = output_dir / "motions"
    motion_dir.mkdir(parents=True, exist_ok=True)
    motion_name = args.motion_name or gem_path.parent.name
    qpos_target = _resample_qpos(qpos_source, source_fps, args.target_fps)
    motion_path = save_motion(motion_dir / f"{motion_name}.npz", qpos_target)
    reference_path = output_dir / f"{motion_name}_joint_reference.npz"
    np.savez_compressed(
        reference_path,
        joint_pos=qpos_source[:, joint_indices].astype(np.float32),
        joint_names=np.asarray(joint_names),
        fps=np.asarray(source_fps, dtype=np.float32),
        qpos=qpos_source.astype(np.float32),
    )

    qpos_names = [
        "root_tx", "root_ty", "root_tz", "root_qw", "root_qx", "root_qy", "root_qz",
        *joint_names,
    ]
    write_manifest(
        output_dir,
        dataset_name=f"gem_{motion_name}_{args.robot}",
        mjcf=robot_cfg.mjcf_path,
        timestep=1.0 / args.target_fps,
        qpos_names=qpos_names,
        num_motions=1,
        total_hours=qpos_target.shape[0] / args.target_fps / 3600.0,
        source={
            "format": "GEM body_params_global retargeted with GMR",
            "gem_params": str(gem_path),
            "source_fps": source_fps,
            "target_fps": args.target_fps,
            "human_height": human_height,
            "gmr_robot": GMR_ROBOT_NAMES[args.robot],
            "gmr_config": str(PIPLUS_GMR_CONFIG) if args.robot == "piplus_h0w" else "GMR smplx_to_g1.json",
            "constant_root_z_offset": z_offset,
        },
    )
    report = {
        "robot": args.robot,
        "frames_source": int(qpos_source.shape[0]),
        "frames_target": int(qpos_target.shape[0]),
        "source_fps": source_fps,
        "target_fps": args.target_fps,
        "qpos_shape": list(qpos_target.shape),
        "root_z_range": [float(qpos_target[:, 2].min()), float(qpos_target[:, 2].max())],
        "root_quat_norm_error": float(
            np.max(np.abs(np.linalg.norm(qpos_target[:, 3:7], axis=-1) - 1.0))
        ),
        "continuity": _continuity_stats(qpos_target, args.target_fps),
        **_validate_limits(qpos_target, model, joint_names),
    }
    report_path = output_dir / "retarget_report.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"Saved any4hdmi motion: {motion_path}")
    print(f"Saved source-rate joint reference: {reference_path}")
    print(f"Saved validation report: {report_path}")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
