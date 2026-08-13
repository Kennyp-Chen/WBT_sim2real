#!/usr/bin/env python3
"""Replay GEM ``smpl_params.pt`` as the repository's SONIC SMPL ZMQ stream.

GEM emits body parameters, while SONIC expects canonical SMPL joints and a G1
joint reference.  This adapter computes the former with the existing SONIC FK
implementation.  G1 joint references can be supplied in an NPZ/PT file; when
omitted, the configured G1 default pose is published and a warning is shown.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any

import numpy as np
import torch
import tyro
import zmq

from sim2real.config.robots import get_robot_cfg
from sim2real.teleop.smpl_stream import (
    DEFAULT_HUMAN_JOINTS_INFO_PATH,
    axis_angle_to_quat_wxyz,
    build_smpl_frame_from_xrobot_raw,
    json_safe_payload,
    official_smpl_frame_from_body_pose_aa,
)


def _tensor_array(value: Any, name: str) -> np.ndarray:
    if isinstance(value, torch.Tensor):
        value = value.detach().cpu().numpy()
    array = np.asarray(value, dtype=np.float32)
    if not np.isfinite(array).all():
        raise ValueError(f"GEM field {name} contains NaN or infinity")
    return array


def _load_gem_params(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    raw = torch.load(str(path), map_location="cpu", weights_only=False)
    if not isinstance(raw, dict):
        raise ValueError(f"Expected a GEM parameter dictionary, got {type(raw)}")
    global_params = raw.get("body_params_global")
    if not isinstance(global_params, dict):
        raise ValueError("GEM file does not contain body_params_global")
    try:
        body_pose = _tensor_array(global_params["body_pose"], "body_pose")
        global_orient = _tensor_array(global_params["global_orient"], "global_orient")
    except KeyError as exc:
        raise ValueError(f"GEM body_params_global is missing {exc.args[0]}") from exc
    body_pose = body_pose.reshape(-1, 21, 3)
    global_orient = global_orient.reshape(-1, 3)
    if body_pose.shape[0] != global_orient.shape[0]:
        raise ValueError(f"GEM frame count mismatch: {body_pose.shape} vs {global_orient.shape}")
    if not body_pose.shape[0]:
        raise ValueError("GEM file contains no frames")
    fps = float(raw.get("fps", raw.get("frame_rate", 30.0)))
    if fps <= 0:
        raise ValueError(f"GEM FPS must be positive, got {fps}")
    return body_pose, global_orient, fps


def _load_joint_reference(path: Path | None, frames: int, joints: int) -> np.ndarray | None:
    if path is None:
        return None
    if path.suffix.lower() == ".npz":
        raw = np.load(path, allow_pickle=True)
        if "joint_pos" not in raw.files:
            raise ValueError(f"{path} must contain a joint_pos array")
        values = raw["joint_pos"]
    else:
        raw = torch.load(str(path), map_location="cpu", weights_only=False)
        if isinstance(raw, dict):
            values = raw.get("joint_pos")
        else:
            values = raw
        if values is None:
            raise ValueError(f"{path} must contain joint_pos")
    values = _tensor_array(values, "joint_pos").reshape(-1, joints)
    if values.shape[0] != frames:
        raise ValueError(f"joint_pos frame count {values.shape[0]} does not match GEM {frames}")
    return values


def convert_gem_params(
    gem_path: Path,
    *,
    human_joints_info: Path,
    joint_reference: Path | None = None,
    robot: str = "g1",
) -> tuple[dict[str, np.ndarray], float]:
    body_pose, global_orient, fps = _load_gem_params(gem_path)
    robot_cfg = get_robot_cfg(robot)
    reference = _load_joint_reference(joint_reference, body_pose.shape[0], len(robot_cfg.joint_names))
    if reference is None:
        default_qpos = np.asarray(robot_cfg.default_qpos, dtype=np.float32)
        reference = np.broadcast_to(
            default_qpos[robot_cfg.joint_pos_slice],
            (body_pose.shape[0], len(robot_cfg.joint_names)),
        ).copy()
        print("[gem-smpl-pub] warning: no robot reference supplied; using G1 default pose")

    smpl_root_quat_y_up = axis_angle_to_quat_wxyz(global_orient)
    joint_pos_root = np.empty((body_pose.shape[0], 24, 3), dtype=np.float32)
    root_quat_w = np.empty((body_pose.shape[0], 4), dtype=np.float32)
    for index in range(body_pose.shape[0]):
        joint_pos_root[index], root_quat_w[index] = official_smpl_frame_from_body_pose_aa(
            body_pose[index], smpl_root_quat_y_up[index], human_joints_info_path=human_joints_info
        )
    if reference.shape[0] > 1:
        joint_vel = np.gradient(reference, 1.0 / fps, axis=0).astype(np.float32)
    else:
        joint_vel = np.zeros_like(reference, dtype=np.float32)
    return {
        "smpl_body_pose_aa": body_pose,
        "smpl_joint_pos_root": joint_pos_root,
        "smpl_root_quat_w": root_quat_w,
        "joint_pos": reference,
        "joint_vel": joint_vel,
    }, fps


@dataclass
class Args:
    gem_params: Path
    bind: str = "tcp://*:28702"
    robot: str = "g1"
    human_joints_info: Path = Path(DEFAULT_HUMAN_JOINTS_INFO_PATH)
    joint_reference: Path | None = None
    future_frames: int = 4
    max_frames: int | None = None
    loop: bool = False
    dry_run: bool = False


def main(args: Args) -> None:
    robot_cfg = get_robot_cfg(args.robot)
    motion, fps = convert_gem_params(
        args.gem_params.expanduser().resolve(),
        human_joints_info=args.human_joints_info.expanduser().resolve(),
        joint_reference=args.joint_reference.expanduser().resolve() if args.joint_reference else None,
        robot=args.robot,
    )
    frames = motion["smpl_body_pose_aa"].shape[0]
    count = min(frames, args.max_frames) if args.max_frames is not None else frames
    if args.future_frames <= 0 or count <= 0:
        raise ValueError("future_frames and available frames must be positive")
    print(f"[gem-smpl-pub] {count} frames at {fps:g} Hz; robot={args.robot}; joints={len(robot_cfg.joint_names)}")
    if args.dry_run:
        for key, value in motion.items():
            print(f"  {key}: {value[:count].shape} {value.dtype}")
        return

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.bind(args.bind)
    time.sleep(0.25)
    period = 1.0 / fps
    frame = 0
    deadline = time.monotonic()
    try:
        while True:
            indices = np.minimum(np.arange(frame, frame + args.future_frames), count - 1)
            now_ns = time.time_ns()
            payload = {key: value[indices] for key, value in motion.items()}
            payload.update({
                "frame_index": indices.astype(np.int64),
                "publish_t_ns": np.asarray([now_ns + int(round((args.future_frames - 1) * period * 1e9))], dtype=np.int64),
                "motion_first_frame": np.asarray([frame == 0], dtype=np.bool_),
            })
            socket.send_json({"topic": "pose", "version": 3, "joint_names": list(robot_cfg.joint_names), **json_safe_payload(payload)})
            frame += 1
            if frame >= count:
                if not args.loop:
                    break
                frame = 0
            deadline += period
            time.sleep(max(0.0, deadline - time.monotonic()))
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main(tyro.cli(Args))
