---
title: Robot I/O
slug: /reference/robot-io
---

# Robot I/O Modes

`robot_io` controls how the policy reads robot state and sends low-level
commands. It is separate from:

- `inference_backend`, which controls ONNX / TensorRT policy inference.
- `motion_backend`, which controls the reference motion source.

For sim2sim with `sim_env/base_sim.py`, keep the policy on `--robot-io zmq`.
For sim2real deployment, choose one of the three modes below.

## Quick Choice

| Mode | Processes | Use when |
| --- | --- | --- |
| `--robot-io inline --robot g1` | policy only | Preferred G1 path when the policy runs on a machine with `unitree_interface`; avoids the extra ZMQ bridge hop. |
| `--robot-io zmq` + `scripts/g1/real_bridge.py` | policy + Python DDS bridge | Use the G1 split-process bridge based on `unitree_sdk2py`. |
| `--robot-io zmq` + `scripts/g1/real_bridge_cpp.py` | policy + `unitree_interface` bridge | Keep the ZMQ contract while using the `unitree_interface` robot binding. |

:::tip
If hardware tests show violent joint shaking, or high-dynamic / high-speed
motions look much worse than expected, suspect unstable ZMQ I/O latency first.
Try `--robot-io inline` before retuning the policy or gains.
:::

## Inline

Inline mode creates the G1 `RobotIO` backend inside `BasePolicy`. The policy
runtime reads normalized `RobotState` values and sends commands through the
backend without starting a bridge process.

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot-io inline \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

Use this path first for real deployment when latency jitter matters. Add
`--robot-interface <robot_network_interface>` only when the robot network
interface is not the default `eth0`.

## ZMQ With `scripts/g1/real_bridge.py`

This mode keeps the policy and robot bridge in separate processes. The bridge
uses `unitree_sdk2py`, publishes `low_state` over ZMQ, and applies `low_cmd`
from ZMQ to the robot.

Terminal 1:

```bash
uv run scripts/g1/real_bridge.py
```

Terminal 2:

```bash
uv run sim2real/rl_policy/tracking.py \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

Add `--interface <robot_network_interface>` to the bridge command only when the
robot network interface is not the default `eth0`.

## ZMQ With `scripts/g1/real_bridge_cpp.py`

This mode keeps the same ZMQ contract as `scripts/g1/real_bridge.py`, but the bridge uses
`unitree_interface` for robot I/O.

Terminal 1:

```bash
uv run scripts/g1/real_bridge_cpp.py
```

Terminal 2:

```bash
uv run sim2real/rl_policy/tracking.py \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

Add `--interface <robot_network_interface>` to the bridge command only when the
robot network interface is not the default `eth0`.

## Rule Of Thumb

Use `inline` for the normal real-robot deploy path. Use ZMQ bridge mode when
you need process isolation, want to debug the policy and robot bridge
separately, or are running sim2sim with `base_sim.py`.

## PiPlus BFM-Zero

PiPlus 22-DoF H0W and 23-DoF LSE use the same `LowStateMessage` /
`LowCmdMessage` ZMQ contract, with the
ROS2 hardware adapter in `scripts/piplus/real_bridge.py`. The adapter owns the
PiPlus-specific ROS joint order, `xyzw` to `wxyz` IMU conversion, motor-control
services, and command safety clipping. The policy process remains unchanged.

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_h0w --dryrun
uv run python sim2real/rl_policy/tracking.py \
  --robot piplus_h0w --robot-io zmq --controller keyboard \
  --policy-config checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml
```

For PiPlus-LSE 23-DoF, use:

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_lse_23dof --dryrun
uv run python scripts/piplus/bfmzero_23dof.py --preflight-only
uv run python scripts/piplus/bfmzero_23dof.py
```

For the complete three-terminal deployment and the PiPlus BFM-Zero input
contract, see [PiPlus BFM-Zero sim2real](./piplus_bfmzero_sim2real.md).
The 23-DoF hardware map and deployment sequence are documented in
[PiPlus-LSE 23-DoF BFM-Zero Sim2real](./tutorials/piplus-lse-23dof-sim2real.md).
