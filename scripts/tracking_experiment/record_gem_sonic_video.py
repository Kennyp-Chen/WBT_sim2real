"""Record GMR reference and SONIC SMPL sim2sim from one GEM parameter file."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path

import imageio_ffmpeg
import mujoco
import numpy as np
import tyro

from sim2real.config.robots import get_robot_cfg
from sim2real.rl_policy.tracking import Tracking, TrackingArgs
from sim2real.sim_env.utils.bridge import SimulationBridge
from sim2real.sim_env.utils.mjcf import load_sim_model
from sim2real.teleop.gem_smpl_pub import Args as GemPublisherArgs
from sim2real.teleop.gem_smpl_pub import run_publish


@dataclass
class Args:
    gem_params: str
    joint_reference: str
    reference_motion: str
    output: str
    policy_config: str = "checkpoints/sonic/release/smpl/policy.yaml"
    duration_s: float = 10.38
    fps: float = 30.0
    width: int = 1280
    height: int = 720
    sim_dt: float = 0.005
    rl_rate: float = 50.0
    source_fps: float = 30.0
    smpl_bind: str = "tcp://*:28702"
    smpl_connect: str = "tcp://127.0.0.1:28702"
    handshake_s: float = 0.75
    motion_tolerance_s: float = 0.04
    camera_distance: float = 3.0
    camera_azimuth: float = 120.0
    camera_elevation: float = -20.0
    inference_backend: str = "onnx-cpu"


def _camera(body_id: int, args: Args) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = int(body_id)
    camera.distance = float(args.camera_distance)
    camera.azimuth = float(args.camera_azimuth)
    camera.elevation = float(args.camera_elevation)
    return camera


def _run_policy(policy: Tracking, stop_event: threading.Event) -> None:
    next_tick = time.perf_counter()
    while not stop_event.is_set():
        policy.step()
        policy.total_inference_cnt += 1
        next_tick += policy.rl_dt
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
        else:
            next_tick = time.perf_counter()


def main(args: Args) -> None:
    if args.duration_s <= 0 or args.fps <= 0 or args.sim_dt <= 0:
        raise ValueError("duration_s, fps, and sim_dt must be positive")
    if args.width % 2 or args.height % 2:
        raise ValueError("width and height must be even")
    robot_cfg = get_robot_cfg("g1")
    gem_path = Path(args.gem_params).expanduser().resolve()
    joint_reference = Path(args.joint_reference).expanduser().resolve()
    reference_path = Path(args.reference_motion).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    for path in (gem_path, joint_reference, reference_path):
        if not path.is_file():
            raise FileNotFoundError(path)

    reference_qpos = np.asarray(
        np.load(reference_path, allow_pickle=True)["qpos"], dtype=np.float32
    )
    model = load_sim_model(robot_cfg)
    if reference_qpos.shape[1] != model.nq:
        raise ValueError(
            f"Reference nq={reference_qpos.shape[1]} does not match G1 nq={model.nq}"
        )
    model.opt.timestep = float(args.sim_dt)
    data = mujoco.MjData(model)
    reference_data = mujoco.MjData(model)
    data.qpos[:] = reference_qpos[0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    bridge = SimulationBridge(model, data, robot_cfg)
    for unitree_idx, qpos_addr in zip(bridge.joint_indices_unitree, bridge.qpos_adrs):
        joint_name = robot_cfg.joint_names[unitree_idx]
        bridge.cmd_q[unitree_idx] = float(data.qpos[qpos_addr])
        bridge.cmd_kp[unitree_idx] = float(robot_cfg.safe_joint_kp.get(joint_name, 0.0))
        bridge.cmd_kd[unitree_idx] = float(robot_cfg.safe_joint_kd.get(joint_name, 0.0))
    bridge.has_received_command = True

    policy = Tracking(
        TrackingArgs(
            policy_config=str(Path(args.policy_config).expanduser().resolve()),
            robot="g1",
            rl_rate=float(args.rl_rate),
            inference_backend=args.inference_backend,
            robot_io="zmq",
            controller="passive",
            motion_backend="smpl_zmq",
            motion_zmq_connect=args.smpl_connect,
            motion_zmq_hwm=1,
            motion_tolerance_s=float(args.motion_tolerance_s),
        )
    )
    policy.reset()
    policy.state_dict["control_mode"] = "policy"
    policy.state_dict["paused"] = False
    policy.total_inference_cnt = 0
    policy_start_delay_s = float(policy.motion_buffer.delay_s)

    publisher_stop = threading.Event()
    publisher_play = threading.Event()
    publisher_ready = threading.Event()
    publisher_args = GemPublisherArgs(
        gem_params=gem_path,
        bind=args.smpl_bind,
        robot="g1",
        joint_reference=joint_reference,
        target_fps=float(args.rl_rate),
        future_frames=10,
    )
    publisher_thread = threading.Thread(
        target=run_publish,
        args=(publisher_args,),
        kwargs={
            "stop_event": publisher_stop,
            "play_event": publisher_play,
            "ready_event": publisher_ready,
        },
        daemon=True,
        name="gem-smpl-publisher",
    )
    publisher_thread.start()

    track_body_id = -1
    for body_name in robot_cfg.viewer_track_body_names:
        track_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if track_body_id >= 0:
            break
    if track_body_id < 0:
        raise ValueError("Could not resolve G1 camera tracking body")

    half_width = args.width // 2
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), half_width)
    model.vis.global_.offheight = max(int(model.vis.global_.offheight), args.height)
    policy_renderer = mujoco.Renderer(model, width=half_width, height=args.height)
    reference_renderer = mujoco.Renderer(model, width=half_width, height=args.height)
    policy_camera = _camera(track_body_id, args)
    reference_camera = _camera(track_body_id, args)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio_ffmpeg.write_frames(
        str(output_path),
        (args.width, args.height),
        fps=float(args.fps),
        codec="libx264",
        pix_fmt_in="rgb24",
        pix_fmt_out="yuv420p",
        macro_block_size=1,
        output_params=["-movflags", "+faststart"],
    )
    writer.send(None)

    policy_stop = threading.Event()
    policy_thread = threading.Thread(
        target=_run_policy,
        args=(policy, policy_stop),
        daemon=True,
        name="sonic-policy",
    )
    policy_started = False
    frame_count = 0
    record_start = 0.0
    try:
        if not publisher_ready.wait(timeout=30.0):
            raise TimeoutError("GEM SMPL publisher did not bind within 30 seconds")
        warmup_steps = max(1, int(round(args.handshake_s / args.sim_dt)))
        next_tick = time.perf_counter()
        for _ in range(warmup_steps):
            bridge.publish_low_state()
            next_tick += args.sim_dt
            time.sleep(max(0.0, next_tick - time.perf_counter()))

        print(
            f"[gem-sonic-record] source starts at 0.000s; policy starts at "
            f"{policy_start_delay_s:.3f}s"
        )
        record_start = time.perf_counter()
        publisher_play.set()
        next_sim = time.perf_counter()
        next_render = 0.0
        sim_steps = int(round(args.duration_s / args.sim_dt))
        for sim_index in range(sim_steps):
            wall_elapsed = time.perf_counter() - record_start
            if not policy_started and wall_elapsed >= policy_start_delay_s:
                policy_thread.start()
                policy_started = True
            bridge.publish_low_state()
            bridge.compute_torques()
            data.ctrl[:] = bridge.torques
            mujoco.mj_step(model, data)

            sim_elapsed = sim_index * args.sim_dt
            while next_render <= sim_elapsed + 1.0e-9:
                source_elapsed = max(0.0, time.perf_counter() - record_start)
                source_index = min(
                    int(round(source_elapsed * args.source_fps)),
                    reference_qpos.shape[0] - 1,
                )
                reference_data.qpos[:] = reference_qpos[source_index]
                reference_data.qvel[:] = 0.0
                mujoco.mj_forward(model, reference_data)
                reference_renderer.update_scene(reference_data, camera=reference_camera)
                policy_renderer.update_scene(data, camera=policy_camera)
                reference_frame = np.asarray(reference_renderer.render(), dtype=np.uint8)
                policy_frame = np.asarray(policy_renderer.render(), dtype=np.uint8)
                writer.send(
                    np.ascontiguousarray(
                        np.concatenate([reference_frame, policy_frame], axis=1)
                    )
                )
                frame_count += 1
                next_render = frame_count / args.fps

            next_sim += args.sim_dt
            sleep_s = next_sim - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_sim = time.perf_counter()
    finally:
        policy_stop.set()
        publisher_stop.set()
        if policy_started:
            policy_thread.join(timeout=2.0)
        publisher_thread.join(timeout=2.0)
        try:
            policy.motion_buffer.close()
        except AttributeError:
            pass
        try:
            policy.controller.close()
        finally:
            policy.robot_io.close()
        for socket_name in ("low_state_pub", "low_cmd_sub"):
            socket = getattr(bridge, socket_name, None)
            if socket is not None:
                socket.close(linger=0)
        writer.close()
        policy_renderer.close()
        reference_renderer.close()
    wall_duration = time.perf_counter() - record_start if record_start else 0.0
    print(f"Saved {frame_count} frames to {output_path}")
    print(
        f"[gem-sonic-record] nominal_duration={args.duration_s:.3f}s "
        f"wall_duration={wall_duration:.3f}s"
    )


if __name__ == "__main__":
    main(tyro.cli(Args))
