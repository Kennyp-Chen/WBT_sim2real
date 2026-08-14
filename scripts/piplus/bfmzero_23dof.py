#!/usr/bin/env python3
"""Run the PiPlus-LSE 23-DoF BFM-Zero policy for sim2real deployment."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import onnx
import tyro
import yaml

from sim2real.config.robots import get_robot_cfg
from sim2real.rl_policy.robot_io.piplus import validate_hardware_contract
from sim2real.rl_policy.tracking import Tracking, TrackingArgs


DEFAULT_POLICY_CONFIG = "checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml"
PIPLUS_23DOF_ROBOT = "piplus_lse_23dof"
EXPECTED_ONNX_INPUTS = {
    "actor_state": (None, 52),
    "last_action": (None, 23),
    "history_actor": (None, 300),
    "encoder_state": (8, 52),
    "privileged_state": (8, 403),
    "encoder_window_weight": (8, 1),
}


@dataclass
class Args:
    policy_config: str = DEFAULT_POLICY_CONFIG
    inference_backend: Literal["onnx-gpu", "onnx-cpu", "tensorrt"] = "onnx-cpu"
    controller: Literal["keyboard", "pico", "passive"] = "keyboard"
    motion_zmq_connect: str = "tcp://127.0.0.1:28701"
    motion_zmq_hwm: int = 1
    motion_tolerance_s: float = 0.04
    pico_zmq_connect: str = "tcp://127.0.0.1:5592"
    rl_rate: float = 50.0
    record: bool = False
    record_output: str | None = None
    preflight_only: bool = False


def validate_deploy_contract(policy_config: str) -> tuple[Path, Path]:
    """Check the 23-DoF robot, policy YAML, model, and hardware mapping."""
    robot_cfg = get_robot_cfg(PIPLUS_23DOF_ROBOT)
    validate_hardware_contract(robot_cfg)

    policy_path = Path(policy_config).expanduser().resolve()
    if not policy_path.is_file():
        raise FileNotFoundError(f"Policy config not found: {policy_path}")
    config = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"Policy config must contain a mapping: {policy_path}")

    policy_joints = tuple(config.get("policy_joint_names", ()))
    if policy_joints != robot_cfg.joint_names:
        raise ValueError(
            "Policy joint order does not match piplus_lse_23dof RobotCfg: "
            f"{policy_joints}"
        )
    observation_groups = set(config.get("observation", {}))
    if observation_groups != set(EXPECTED_ONNX_INPUTS):
        raise ValueError(
            "Policy observation groups do not match the semantic ONNX inputs: "
            f"{sorted(observation_groups)}"
        )
    model_path = policy_path.parent / str(config.get("model_path", "policy.onnx"))
    if not model_path.is_file():
        raise FileNotFoundError(f"Policy ONNX not found: {model_path}")

    model = onnx.load(str(model_path), load_external_data=False)
    initializers = {item.name for item in model.graph.initializer}
    graph_inputs = {
        item.name: tuple(
            int(dim.dim_value) if dim.dim_value > 0 else None
            for dim in item.type.tensor_type.shape.dim
        )
        for item in model.graph.input
        if item.name not in initializers
    }
    if graph_inputs != EXPECTED_ONNX_INPUTS:
        raise ValueError(
            "PiPlus-LSE 23-DoF ONNX input contract mismatch: "
            f"{graph_inputs}"
        )
    outputs = {
        item.name: tuple(
            int(dim.dim_value) if dim.dim_value > 0 else None
            for dim in item.type.tensor_type.shape.dim
        )
        for item in model.graph.output
    }
    action_shape = outputs.get("action")
    if action_shape is None or action_shape[-1] != 23:
        raise ValueError(f"PiPlus-LSE ONNX must expose action[..., 23], got {outputs}")
    return policy_path, model_path


def build_tracking_args(args: Args, policy_path: Path) -> TrackingArgs:
    return TrackingArgs(
        policy_config=str(policy_path),
        robot=PIPLUS_23DOF_ROBOT,
        rl_rate=args.rl_rate,
        inference_backend=args.inference_backend,
        robot_io="zmq",
        controller=args.controller,
        pico_zmq_connect=args.pico_zmq_connect,
        record=args.record,
        record_output=args.record_output,
        motion_backend="zmq",
        motion_zmq_connect=args.motion_zmq_connect,
        motion_zmq_hwm=args.motion_zmq_hwm,
        motion_tolerance_s=args.motion_tolerance_s,
    )


def main(args: Args) -> None:
    policy_path, model_path = validate_deploy_contract(args.policy_config)
    print(
        "PiPlus-LSE 23-DoF deploy contract OK: "
        f"policy={policy_path} model={model_path}"
    )
    if args.preflight_only:
        return
    Tracking(args=build_tracking_args(args, policy_path)).run()


if __name__ == "__main__":
    main(tyro.cli(Args))
