#!/usr/bin/env python3
"""Publish atomically written GEM SMPL chunks to the SONIC SMPL ZMQ stream."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import numpy as np

from sim2real.config.robots import get_robot_cfg
from sim2real.teleop.gem_smpl_pub import (
    convert_gem_params,
    json_safe_payload,
    resample_sonic_motion,
)
from sim2real.teleop.smpl_stream import DEFAULT_HUMAN_JOINTS_INFO_PATH


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--bind", default="tcp://*:28702")
    parser.add_argument("--robot", default="g1")
    parser.add_argument("--human-joints-info", type=Path, default=Path(DEFAULT_HUMAN_JOINTS_INFO_PATH))
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--future-frames", type=int, default=10)
    parser.add_argument("--poll-s", type=float, default=0.05)
    parser.add_argument("--idle-timeout-s", type=float, default=5.0)
    parser.add_argument("--startup-sleep-s", type=float, default=0.5)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def _publish_chunk(socket, path: Path, args: argparse.Namespace, frame: int) -> int:
    robot_cfg = get_robot_cfg(args.robot)
    motion, source_fps = convert_gem_params(
        path,
        human_joints_info=args.human_joints_info.expanduser().resolve(),
        robot=args.robot,
    )
    if not np.isclose(source_fps, args.publish_hz):
        motion = resample_sonic_motion(motion, source_fps=source_fps, target_fps=args.publish_hz)
    count = motion["smpl_body_pose_aa"].shape[0]
    period = 1.0 / args.publish_hz
    deadline = time.monotonic()
    while frame < count:
        indices = np.minimum(np.arange(frame, frame + args.future_frames), count - 1)
        fields = {key: value[indices] for key, value in motion.items()}
        fields.update({
            "frame_index": indices.astype(np.int64),
            "publish_t_ns": np.asarray([time.time_ns() + int((args.future_frames - 1) * period * 1e9)], dtype=np.int64),
            "motion_first_frame": np.asarray([frame == 0], dtype=np.bool_),
        })
        socket.send_json({"topic": "pose", "version": 3, "joint_names": list(robot_cfg.joint_names), **json_safe_payload(fields)})
        frame += 1
        deadline += period
        time.sleep(max(0.0, deadline - time.monotonic()))
    return frame


def main() -> None:
    args = _parse_args()
    if args.publish_hz <= 0 or args.future_frames <= 0 or args.poll_s <= 0 or args.startup_sleep_s < 0:
        raise ValueError("publish-hz, future-frames, and poll-s must be positive; startup-sleep-s cannot be negative")
    import zmq

    context = zmq.Context.instance()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt(zmq.SNDHWM, 1)
    socket.bind(args.bind)
    time.sleep(args.startup_sleep_s)
    args.input_dir.expanduser().resolve().mkdir(parents=True, exist_ok=True)
    played: set[Path] = set()
    frame = 0
    idle_since = time.monotonic()
    try:
        while True:
            paths = sorted(args.input_dir.expanduser().resolve().glob("chunk_*/smpl_params.pt"))
            pending = [path for path in paths if path not in played]
            if pending:
                for path in pending:
                    frame = _publish_chunk(socket, path, args, frame=0)
                    played.add(path)
                idle_since = time.monotonic()
                if args.once:
                    return
            elif args.once and time.monotonic() - idle_since >= args.idle_timeout_s:
                return
            time.sleep(args.poll_s)
    finally:
        socket.close(linger=0)


if __name__ == "__main__":
    main()
