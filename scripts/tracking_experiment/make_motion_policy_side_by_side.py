"""Pair a source qpos replay with an existing policy sim2sim MP4."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import imageio_ffmpeg
import mujoco
import numpy as np
import tyro
from PIL import Image

from sim2real.config.robots import get_robot_cfg
from sim2real.sim_env.utils.mjcf import load_sim_model


@dataclass
class Args:
    motion_path: str
    policy_video: str
    output: str
    robot: str = "piplus_h0w"
    width: int = 1280
    height: int = 720
    motion_fps: float = 50.0
    output_fps: float | None = None
    duration_s: float | None = None
    loop: bool = False
    camera_distance: float = 3.0
    camera_azimuth: float = 120.0
    camera_elevation: float = -20.0


def _tracking_camera(
    model: mujoco.MjModel, body_id: int, args: Args
) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = int(body_id)
    camera.distance = float(args.camera_distance)
    camera.azimuth = float(args.camera_azimuth)
    camera.elevation = float(args.camera_elevation)
    return camera


def main(args: Args) -> None:
    if args.width <= 0 or args.height <= 0 or args.width % 2 or args.height % 2:
        raise ValueError("width and height must be positive and even")

    motion_path = Path(args.motion_path).expanduser().resolve()
    policy_video = Path(args.policy_video).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    source_qpos = np.asarray(
        np.load(motion_path, allow_pickle=True)["qpos"], dtype=np.float32
    )
    if source_qpos.ndim != 2:
        raise ValueError(f"Expected motion qpos [T, nq], got {source_qpos.shape}")

    metadata_reader = imageio_ffmpeg.read_frames(str(policy_video), pix_fmt="rgb24")
    metadata = next(metadata_reader)
    source_width, source_height = metadata["size"]
    source_fps = float(metadata["fps"])
    metadata_reader.close()
    output_fps = float(args.output_fps or source_fps)

    robot_cfg = get_robot_cfg(args.robot)
    model = load_sim_model(robot_cfg)
    if source_qpos.shape[1] != model.nq:
        raise ValueError(
            f"Motion nq={source_qpos.shape[1]} does not match model nq={model.nq}"
        )
    data = mujoco.MjData(model)
    renderer_width = args.width // 2
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), renderer_width)
    model.vis.global_.offheight = max(
        int(model.vis.global_.offheight), int(args.height)
    )
    renderer = mujoco.Renderer(model, width=renderer_width, height=args.height)
    reference_camera = _tracking_camera(
        model,
        next(
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
            for name in robot_cfg.viewer_track_body_names
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) >= 0
        ),
        args,
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio_ffmpeg.write_frames(
        str(output),
        (args.width, args.height),
        fps=output_fps,
        codec="libx264",
        pix_fmt_in="rgb24",
        pix_fmt_out="yuv420p",
        macro_block_size=1,
        output_params=["-movflags", "+faststart"],
    )
    writer.send(None)

    target_frames = None
    if args.duration_s is not None:
        if args.duration_s <= 0:
            raise ValueError("duration_s must be positive")
        target_frames = int(round(float(args.duration_s) * output_fps))
    frame_count = 0
    try:
        while target_frames is None or frame_count < target_frames:
            policy_reader = imageio_ffmpeg.read_frames(
                str(policy_video), pix_fmt="rgb24"
            )
            next(policy_reader)
            cycle_frames = 0
            try:
                for raw_frame in policy_reader:
                    if target_frames is not None and frame_count >= target_frames:
                        break
                    policy_frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(
                        int(source_height), int(source_width), 3
                    )
                    resized_policy = np.asarray(
                        Image.fromarray(policy_frame).resize(
                            (renderer_width, args.height), Image.Resampling.BILINEAR
                        ),
                        dtype=np.uint8,
                    )
                    source_frame = cycle_frames if args.loop else frame_count
                    source_index = int(
                        round(source_frame * float(args.motion_fps) / source_fps)
                    )
                    if args.loop:
                        source_index %= source_qpos.shape[0]
                    else:
                        source_index = min(source_index, source_qpos.shape[0] - 1)
                    data.qpos[:] = source_qpos[source_index]
                    data.qvel[:] = 0.0
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=reference_camera)
                    reference_frame = np.asarray(renderer.render(), dtype=np.uint8)
                    writer.send(
                        np.ascontiguousarray(
                            np.concatenate([reference_frame, resized_policy], axis=1)
                        )
                    )
                    frame_count += 1
                    cycle_frames += 1
            finally:
                policy_reader.close()
            if cycle_frames == 0 or not args.loop:
                break
    finally:
        writer.close()
        renderer.close()

    print(f"Saved {frame_count} comparison frames to {output}")


if __name__ == "__main__":
    main(tyro.cli(Args))
