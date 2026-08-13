"""Record a real-time ZMQ motion-to-policy sim2sim comparison video.

The left half displays the source qpos at the actual frame emitted by the
local ZMQ publisher. The right half is a MuJoCo simulation driven through the
same low-state/low-command transport used by deployment:
npz_pub -> Tracking -> SimulationBridge.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import imageio_ffmpeg
import mujoco
import numpy as np
import tyro
import zmq

from sim2real.config.robots import get_robot_cfg
from sim2real.rl_policy.tracking import Tracking, TrackingArgs
from sim2real.sim_env.utils.bridge import SimulationBridge
from sim2real.sim_env.utils.mjcf import load_sim_model
from sim2real.teleop.npz_pub import PublisherArgs, run_publish
from sim2real.teleop.gem_bfmzero_pub import Args as GemBfmZeroPublisherArgs, prepare_gem_motion


class _MotionFrameTap:
    """Track the actual motion frame emitted by the local ZMQ publisher."""

    def __init__(self, connect: str) -> None:
        self._context = zmq.Context.instance()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.RCVHWM, 1)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.connect(connect)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._latest_frame = -1
        self._latest_seq = -1
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="motion-frame-tap"
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                payload = json.loads(self._socket.recv_string(flags=zmq.NOBLOCK))
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception:
                time.sleep(0.01)
                continue
            if payload.get("source") != "npz":
                continue
            frame = int(payload.get("frame", -1))
            if frame < 0:
                continue
            seq = int(payload.get("seq", -1))
            with self._lock:
                self._latest_frame = frame
                self._latest_seq = seq

    def latest(self) -> tuple[int, int]:
        with self._lock:
            return self._latest_frame, self._latest_seq

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._socket.close(linger=0)


def _camera(
    model: mujoco.MjModel,
    body_id: int,
    distance: float,
    azimuth: float,
    elevation: float,
):
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    camera.trackbodyid = int(body_id)
    camera.distance = float(distance)
    camera.azimuth = float(azimuth)
    camera.elevation = float(elevation)
    return camera


@dataclass
class Args:
    policy_config: str
    output: str | None = None
    motion_path: str | None = None
    robot: str = "piplus_h0w"
    duration_s: float = 10.0
    fps: float = 30.0
    width: int = 1280
    height: int = 720
    sim_dt: float = 0.005
    rl_rate: float = 50.0
    motion_bind: str = "tcp://*:28701"
    motion_connect: str = "tcp://127.0.0.1:28701"
    motion_publish_hz: float = 50.0
    loop_motion: bool = False
    handshake_s: float = 0.75
    motion_tolerance_s: float = 0.04
    policy_start_delay_s: float | None = None
    publisher_ready_timeout_s: float = 120.0
    camera_distance: float = 3.0
    camera_azimuth: float = 120.0
    camera_elevation: float = -20.0
    inference_backend: str = "onnx-cpu"
    seed: int = 0
    gem_params: str | None = None
    gem_output_dir: str | None = None


def _default_output_path(robot_name: str) -> Path:
    robot_dir = {
        "g1": "G1_29dof",
        "piplus_h0w": "Piplus_22dof",
        "piplus_lse_23dof": "Piplus_23dof",
        "hi_25dof": "Hi_25dof",
    }.get(robot_name, robot_name)
    return Path(
        "outputs/policy_videos/bfm_zero"
    ) / robot_dir / "zmq_full60s_dance2_subject2_side_by_side_sync.mp4"


def _run_policy(policy: Tracking, stop_event: threading.Event) -> None:
    next_tick = time.perf_counter()
    while not stop_event.is_set():
        policy.step()
        policy.total_inference_cnt += 1
        next_tick += policy.rl_dt
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.perf_counter()


def main(args: Args) -> None:
    if args.duration_s <= 0 or args.fps <= 0 or args.sim_dt <= 0:
        raise ValueError("duration_s, fps, and sim_dt must be positive")
    if args.handshake_s < 0 or args.motion_tolerance_s < 0:
        raise ValueError("handshake_s and motion_tolerance_s must be non-negative")
    if args.policy_start_delay_s is not None and args.policy_start_delay_s < 0:
        raise ValueError("policy_start_delay_s must be non-negative")
    if args.publisher_ready_timeout_s <= 0:
        raise ValueError("publisher_ready_timeout_s must be positive")
    if args.width % 2 or args.height % 2:
        raise ValueError("width and height must be even")

    np.random.seed(int(args.seed))
    robot_cfg = get_robot_cfg(args.robot)
    if args.gem_params is not None and args.motion_path is not None:
        raise ValueError("Pass either --motion-path or --gem-params, not both")
    if args.gem_params is None and args.motion_path is None:
        raise ValueError("One of --motion-path or --gem-params is required")
    gem_publisher_args: GemBfmZeroPublisherArgs | None = None
    if args.gem_params is not None:
        gem_publisher_args = GemBfmZeroPublisherArgs(
            gem_params=Path(args.gem_params).expanduser().resolve(),
            robot=args.robot,
            bind=args.motion_bind,
            publish_hz=float(args.motion_publish_hz),
            output_dir=(
                Path(args.gem_output_dir).expanduser().resolve()
                if args.gem_output_dir is not None
                else None
            ),
            loop=bool(args.loop_motion),
        )
        motion_path = prepare_gem_motion(gem_publisher_args)
    else:
        motion_path = Path(args.motion_path).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser().resolve()
        if args.output is not None
        else _default_output_path(robot_cfg.name).resolve()
    )
    if not motion_path.is_file():
        raise FileNotFoundError(motion_path)

    source_qpos = np.asarray(
        np.load(motion_path, allow_pickle=True)["qpos"], dtype=np.float32
    )
    if source_qpos.ndim != 2:
        raise ValueError(f"Expected qpos [T, nq], got {source_qpos.shape}")

    model = load_sim_model(robot_cfg)
    if source_qpos.shape[1] != model.nq:
        raise ValueError(
            f"Motion nq={source_qpos.shape[1]} does not match MuJoCo nq={model.nq}"
        )
    model.opt.timestep = float(args.sim_dt)
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(robot_cfg.default_qpos, dtype=np.float64)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    reference_data = mujoco.MjData(model)

    track_body_id = -1
    for body_name in robot_cfg.viewer_track_body_names:
        track_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if track_body_id >= 0:
            break
    if track_body_id < 0:
        raise ValueError(
            f"Could not resolve tracking body from {robot_cfg.viewer_track_body_names}"
        )

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
            robot=args.robot,
            rl_rate=float(args.rl_rate),
            inference_backend=args.inference_backend,
            robot_io="zmq",
            controller="passive",
            motion_backend="zmq",
            motion_zmq_connect=args.motion_connect,
            motion_zmq_hwm=1,
            motion_tolerance_s=float(args.motion_tolerance_s),
        )
    )
    policy.reset()
    policy.state_dict["control_mode"] = "policy"
    policy.state_dict["paused"] = False
    policy.total_inference_cnt = 0
    motion_buffer_delay_s = float(policy.motion_buffer.delay_s)
    policy_start_delay_s = (
        motion_buffer_delay_s
        if args.policy_start_delay_s is None
        else float(args.policy_start_delay_s)
    )

    publisher_stop = threading.Event()
    publisher_play = threading.Event()
    publisher_ready = threading.Event()
    root_body_name = "base_link" if "base_link" in robot_cfg.body_names else "pelvis"
    publisher_target = run_publish
    publisher_args = PublisherArgs(
        motion_path=str(motion_path),
        robot=args.robot,
        bind=args.motion_bind,
        publish_hz=float(args.motion_publish_hz),
        startup_sleep_s=0.25,
        initial_source="motion",
        keyboard=False,
        start_playing=False,
        loop=bool(args.loop_motion),
        hold_last=not bool(args.loop_motion),
        root_body_name=root_body_name,
    )
    publisher_thread = threading.Thread(
        target=publisher_target,
        args=(publisher_args,),
        kwargs={
            "stop_event": publisher_stop,
            "play_event": publisher_play,
            "ready_event": publisher_ready,
        },
        daemon=True,
        name="npz-motion-publisher",
    )
    publisher_thread.start()
    motion_frame_tap = _MotionFrameTap(args.motion_connect)
    motion_frame_tap.start()

    half_width = args.width // 2
    model.vis.global_.offwidth = max(int(model.vis.global_.offwidth), half_width)
    model.vis.global_.offheight = max(
        int(model.vis.global_.offheight), int(args.height)
    )
    policy_renderer = mujoco.Renderer(model, width=half_width, height=args.height)
    reference_renderer = mujoco.Renderer(model, width=half_width, height=args.height)
    policy_camera = _camera(
        model,
        track_body_id,
        args.camera_distance,
        args.camera_azimuth,
        args.camera_elevation,
    )
    reference_camera = _camera(
        model,
        track_body_id,
        args.camera_distance,
        args.camera_azimuth,
        args.camera_elevation,
    )
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

    sim_steps = int(round(float(args.duration_s) / float(args.sim_dt)))
    render_period = 1.0 / float(args.fps)
    next_render = 0.0
    frame_count = 0
    policy_stop = threading.Event()
    policy_thread = threading.Thread(
        target=_run_policy,
        args=(policy, policy_stop),
        daemon=True,
        name="zmq-policy",
    )
    policy_started = False
    record_start = 0.0
    wall_elapsed = 0.0
    try:
        if not publisher_ready.wait(timeout=float(args.publisher_ready_timeout_s)):
            raise TimeoutError(
                f"Motion publisher was not ready after {args.publisher_ready_timeout_s:.1f}s"
            )
        # Establish ZMQ while the robot is frozen in its default pose.
        warmup_steps = max(1, int(round(float(args.handshake_s) / float(args.sim_dt))))
        warmup_next = time.perf_counter()
        for _ in range(warmup_steps):
            bridge.publish_low_state()
            warmup_next += float(args.sim_dt)
            sleep_s = warmup_next - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                warmup_next = time.perf_counter()
        print(
            f"[zmq-record] source starts at 0.000s; policy starts at "
            f"{policy_start_delay_s:.3f}s; realtime buffer delay="
            f"{motion_buffer_delay_s:.3f}s"
        )
        # The publisher and motion buffer run on wall-clock time. Use the same
        # clock for the reference panel and delayed policy startup so a slow
        # renderer/encoder cannot make the two sides drift apart.
        record_start = time.perf_counter()
        publisher_play.set()
        next_sim = time.perf_counter()

        for sim_index in range(sim_steps):
            sim_elapsed = sim_index * float(args.sim_dt)
            wall_elapsed = time.perf_counter() - record_start
            if not policy_started and wall_elapsed >= policy_start_delay_s:
                policy_thread.start()
                policy_started = True
            bridge.publish_low_state()
            bridge.compute_torques()
            data.ctrl[:] = bridge.torques
            mujoco.mj_step(model, data)

            while next_render <= sim_elapsed + 1.0e-9:
                # npz_pub advances frames against its real-time publish clock,
                # while the simulation loop below advances a fixed-step clock.
                # Sample the source using wall elapsed time to keep the left
                # panel aligned with the actual ZMQ stream under load.
                source_elapsed = max(0.0, time.perf_counter() - record_start)
                published_frame, _ = motion_frame_tap.latest()
                if published_frame >= 0:
                    source_index = published_frame
                else:
                    source_index = int(
                        round(source_elapsed * float(args.motion_publish_hz))
                    )
                if args.loop_motion:
                    source_index %= source_qpos.shape[0]
                else:
                    source_index = min(source_index, source_qpos.shape[0] - 1)
                reference_data.qpos[:] = source_qpos[source_index]
                reference_data.qvel[:] = 0.0
                mujoco.mj_forward(model, reference_data)
                reference_renderer.update_scene(reference_data, camera=reference_camera)
                policy_renderer.update_scene(data, camera=policy_camera)
                reference_frame = np.asarray(
                    reference_renderer.render(), dtype=np.uint8
                )
                policy_frame = np.asarray(policy_renderer.render(), dtype=np.uint8)
                writer.send(
                    np.ascontiguousarray(
                        np.concatenate([reference_frame, policy_frame], axis=1)
                    )
                )
                frame_count += 1
                next_render = frame_count * render_period

            next_sim += float(args.sim_dt)
            sleep_s = next_sim - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_sim = time.perf_counter()
    finally:
        if record_start > 0.0:
            wall_elapsed = time.perf_counter() - record_start
        policy_stop.set()
        publisher_stop.set()
        if policy_started:
            policy_thread.join(timeout=2.0)
        publisher_thread.join(timeout=2.0)
        motion_frame_tap.close()
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

    print(f"Saved {frame_count} frames to {output_path}")
    if record_start > 0.0:
        print(
            f"[zmq-record] nominal_duration={args.duration_s:.3f}s "
            f"wall_duration={wall_elapsed:.3f}s "
            f"clock_drift={wall_elapsed - args.duration_s:+.3f}s"
        )


if __name__ == "__main__":
    main(tyro.cli(Args))
