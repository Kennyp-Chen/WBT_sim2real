"""Publish GEM SMPL output through the canonical BFM-Zero robot-motion ZMQ path.

The repository already has one canonical any4hdmi/NPZ publisher in
``sim2real.teleop.npz_pub``. This adapter only prepares a GEM recording as a
robot qpos motion and then delegates publishing to that existing implementation.
It supports both the G1 BFM-Zero checkpoint and the PiPlus BFM-Zero checkpoint.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from pathlib import Path
import sys

import tyro
from any4hdmi.core.format import save_motion, write_manifest


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.retarget_gem_smpl import (
    DEFAULT_SMPLX_ROOT,
    _resample_qpos,
    retarget_gem_params_to_qpos,
)
from sim2real.config.robots import get_robot_cfg
from sim2real.teleop.npz_pub import PublisherArgs, run_publish as run_npz_publish


@dataclass
class Args:
    gem_params: Path
    robot: str = "g1"
    bind: str = "tcp://*:28701"
    publish_hz: float = 50.0
    source_fps: float = 30.0
    smplx_model_root: Path = DEFAULT_SMPLX_ROOT
    max_frames: int | None = None
    ankle_height: float = 0.055
    output_dir: Path | None = None
    loop: bool = False
    startup_sleep_s: float = 0.5
    dry_run: bool = False


def prepare_gem_motion(args: Args) -> Path:
    """Retarget GEM once and save a canonical any4hdmi motion directory."""
    if args.publish_hz <= 0 or args.source_fps <= 0:
        raise ValueError("publish_hz and source_fps must be positive")
    gem_path = args.gem_params.expanduser().resolve()
    if not gem_path.is_file():
        raise FileNotFoundError(gem_path)
    robot_cfg = get_robot_cfg(args.robot)
    qpos_source, source_fps, human_height, _model, z_offset = retarget_gem_params_to_qpos(
        gem_path,
        robot=args.robot,
        smplx_model_root=args.smplx_model_root,
        source_fps=args.source_fps,
        max_frames=args.max_frames,
        ankle_height=args.ankle_height,
    )

    output_dir = args.output_dir
    if output_dir is None:
        output_dir = (
            REPO_ROOT
            / "outputs/gem_retarget"
            / gem_path.parent.name
            / args.robot
            / "zmq_motion"
        )
    output_dir = output_dir.expanduser().resolve()
    motion_dir = output_dir / "motions"
    motion_dir.mkdir(parents=True, exist_ok=True)
    motion_name = gem_path.stem
    qpos_target = _resample_qpos(qpos_source, source_fps, args.publish_hz)
    motion_path = save_motion(motion_dir / f"{motion_name}.npz", qpos_target)
    qpos_names = [
        "root_tx", "root_ty", "root_tz", "root_qw", "root_qx", "root_qy", "root_qz",
        *robot_cfg.joint_names,
    ]
    write_manifest(
        output_dir,
        dataset_name=f"gem_{motion_name}_{args.robot}_zmq",
        mjcf=robot_cfg.mjcf_path,
        timestep=1.0 / args.publish_hz,
        qpos_names=qpos_names,
        num_motions=1,
        total_hours=qpos_target.shape[0] / args.publish_hz / 3600.0,
        source={
            "format": "GEM body_params_global retargeted with GMR for npz_pub",
            "gem_params": str(gem_path),
            "source_fps": source_fps,
            "publish_fps": args.publish_hz,
            "human_height": human_height,
            "robot": args.robot,
            "constant_root_z_offset": z_offset,
        },
    )
    return motion_path


def run_publish(
    args: Args,
    *,
    stop_event: threading.Event | None = None,
    play_event: threading.Event | None = None,
    ready_event: threading.Event | None = None,
) -> None:
    """Prepare GEM motion and publish it through the normal robot ZMQ schema."""
    motion_path = prepare_gem_motion(args)
    print(f"[gem-bfmzero-pub] prepared motion: {motion_path}", flush=True)
    if args.dry_run:
        if ready_event is not None:
            ready_event.set()
        return
    run_npz_publish(
        PublisherArgs(
            motion_path=str(motion_path),
            robot=args.robot,
            bind=args.bind,
            publish_hz=args.publish_hz,
            startup_sleep_s=args.startup_sleep_s,
            loop=args.loop,
            hold_last=not args.loop,
            initial_source="motion",
            keyboard=False,
            start_playing=play_event is None,
            root_body_name=(
                "base_link"
                if "base_link" in robot_cfg.body_names
                else "pelvis"
            ),
        ),
        stop_event=stop_event,
        play_event=play_event,
        ready_event=ready_event,
    )


def main(args: Args) -> None:
    run_publish(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
