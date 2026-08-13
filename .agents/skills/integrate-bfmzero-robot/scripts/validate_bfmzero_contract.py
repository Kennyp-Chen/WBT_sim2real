#!/usr/bin/env python3
"""Static checks for a BFM-Zero sim2real policy package."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml


def _targets(value: Any) -> list[str]:
    found: list[str] = []
    if isinstance(value, dict):
        target = value.get("_target_")
        if target is not None:
            found.append(str(target))
        for item in value.values():
            found.extend(_targets(item))
    elif isinstance(value, list):
        for item in value:
            found.extend(_targets(item))
    return found


def _require_mapping(config: dict[str, Any], key: str) -> dict[str, Any]:
    value = config.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be a mapping")
    return value


def _check_policy(path: Path) -> tuple[dict[str, Any], int]:
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError("policy YAML must contain a mapping")

    simulation_joints = list(config.get("joint_names_simulation", []))
    policy_joints = list(config.get("policy_joint_names", []))
    bodies = list(config.get("body_names_simulation", []))
    if not simulation_joints or not policy_joints:
        raise ValueError("joint_names_simulation and policy_joint_names are required")
    if simulation_joints != policy_joints:
        raise ValueError("joint_names_simulation and policy_joint_names order differ")
    if not bodies:
        raise ValueError("body_names_simulation must be non-empty")

    action_scale = config.get("action_scale")
    if not isinstance(action_scale, list) or len(action_scale) != len(policy_joints):
        raise ValueError(
            f"action_scale must have {len(policy_joints)} values, got {action_scale!r}"
        )
    for key in ("default_joint_pos", "joint_kp", "joint_kd", "joint_effort_limit"):
        values = _require_mapping(config, key)
        missing = [name for name in policy_joints if name not in values]
        if missing:
            raise ValueError(f"{key} is missing joints: {missing}")

    motion = _require_mapping(config, "motion")
    if str(motion.get("motion_backend", "npz")).lower() != "npz":
        print("warning: this validator is intended for motion_backend=npz")
    future_steps = motion.get("future_steps")
    if future_steps is not None and not isinstance(future_steps, list):
        raise ValueError("motion.future_steps must be a list")
    targets = _targets(config.get("observation", {}))
    if not targets:
        raise ValueError("observation contains no _target_ entries")

    return config, len(policy_joints)


def _check_model(policy_path: Path, config: dict[str, Any], action_dim: int) -> None:
    model_value = config.get("model_path")
    model_path = (
        policy_path.with_suffix(".onnx")
        if model_value is None
        else policy_path.parent / str(model_value)
    )
    model_path = model_path.expanduser()
    if not model_path.is_file():
        raise FileNotFoundError(f"ONNX model not found: {model_path}")
    try:
        import onnxruntime as ort
    except ImportError:
        print(f"warning: onnxruntime unavailable; skipped graph inspection for {model_path}")
        return
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    inputs = [(item.name, item.shape, item.type) for item in session.get_inputs()]
    outputs = [(item.name, item.shape, item.type) for item in session.get_outputs()]
    if not outputs:
        raise ValueError("ONNX graph has no outputs")
    output_shape = outputs[0][1]
    if output_shape and isinstance(output_shape[-1], int) and output_shape[-1] != action_dim:
        raise ValueError(
            f"ONNX action dimension {output_shape[-1]} != policy joint count {action_dim}"
        )
    print(json.dumps({"inputs": inputs, "outputs": outputs}, indent=2, default=str))


def _check_motion(path_value: str | None, action_dim: int) -> None:
    if not path_value:
        return
    path = Path(path_value).expanduser()
    if not path.exists():
        raise FileNotFoundError(f"motion path not found: {path}")
    root = path if path.is_dir() else path.parent
    manifests = list(root.rglob("manifest.json")) if path.is_dir() else []
    if manifests:
        print(f"motion manifest: {manifests[0]}")
    if path.suffix == ".npz":
        try:
            import numpy as np

            with np.load(path, allow_pickle=False) as data:
                qpos = data.get("qpos")
                if qpos is not None:
                    print(f"motion qpos shape: {qpos.shape}")
                    if qpos.ndim != 2 or qpos.shape[1] != 7 + action_dim:
                        raise ValueError(
                            f"motion qpos must have shape [T, {7 + action_dim}], got {qpos.shape}"
                        )
        except Exception as exc:
            raise ValueError(f"failed to inspect motion {path}: {exc}") from exc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy-config", type=Path, required=True)
    parser.add_argument("--robot", help="Optional robot name; checked through sim2real registry")
    parser.add_argument("--motion-path")
    args = parser.parse_args()

    config, action_dim = _check_policy(args.policy_config.expanduser())
    if args.robot:
        from sim2real.config.robots import get_robot_cfg

        robot_cfg = get_robot_cfg(args.robot)
        if len(robot_cfg.joint_names) != action_dim:
            raise ValueError(
                f"robot config has {len(robot_cfg.joint_names)} joints, policy has {action_dim}"
            )
        policy_joints = list(config["policy_joint_names"])
        if list(robot_cfg.joint_names) != policy_joints:
            raise ValueError(
                "robot config joint order differs from policy_joint_names; "
                "reorder one side explicitly"
            )
        print(f"robot: {robot_cfg.name}, qpos_size={robot_cfg.qpos_size}")
    _check_model(args.policy_config.expanduser(), config, action_dim)
    _check_motion(
        args.motion_path or config.get("motion", {}).get("motion_path"),
        action_dim,
    )
    print(f"OK: BFM-Zero policy contract has {action_dim} action joints")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
