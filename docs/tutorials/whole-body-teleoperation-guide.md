---
title: Whole-Body Teleoperation Guide
sidebar_position: 1
---

# Whole-Body Teleoperation Guide

You do not need a VR device to try whole-body motion tracking in MuJoCo. However,
the repository's ready-to-use path for tracking a human operator in real time is
PICO / XR with leg trackers and XRoboToolkit. A keyboard only controls runtime
states; it does not capture a human pose.

## Available Input Paths

| Path | VR required | Purpose |
| --- | ---: | --- |
| Offline NPZ tracking | No | The shortest path for making G1 track a whole-body motion clip. |
| NPZ stream replay | No | Exercises the same realtime motion interface used by live teleoperation. |
| PICO / XR live teleoperation | Yes | The built-in path for tracking a human operator in real time. |
| External camera, mocap, or SMPL source | Depends | Requires an upstream publisher that implements the repository's ZMQ contract. |

The tracking runtime supports three motion backends:

- `npz`: read a motion file directly.
- `zmq`: receive a realtime motion already retargeted to G1.
- `smpl_zmq`: receive SMPL references and G1 wrist references for compatible
  SONIC policies.

## Available Policies

The repository documents 10 policy families and reports benchmark results for
13 concrete policy variants:

1. Mimic-Lite
2. BFM-Zero
3. ScaleBFM
4. SONIC release
5. SONIC low-latency
6. HoloMotion v1.4.0
7. TeleopIT
8. Humanoid-GPT
9. HEFT
10. TWIST2

Some families contain multiple variants, including ScaleBFM, SONIC, and HEFT.
For the first run, use:

```text
checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

It is the default policy in the tutorials and consumes the standard G1 motion
interface. The documented motion-lookahead requirements are:

| Policy | Lookahead |
| --- | ---: |
| Mimic-Lite | 0.08 s |
| BFM-Zero | 0.12 s |
| ScaleBFM | 0.10 s |
| SONIC release | 0.90 s |
| SONIC low-latency | 0.18 s |
| HoloMotion | 0.20 s |
| TeleopIT | 0.00 s |
| Humanoid-GPT | 0.02 s |
| HEFT | 0.12 s |
| TWIST2 | 0.00 s |

## First Run Without VR

### 1. Install the root environment

Run from the repository root:

```bash
uv sync --extra inference-cpu
```

### 2. Download runtime artifacts

Download the shared [sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
and place them at the repository root:

```text
sim2real/
├── checkpoints/
│   └── mimic-lite/
│       └── 32x8192-huge/
│           ├── policy.yaml
│           └── policy.onnx
└── third_party/
```

See [Download Artifacts](/reference/artifacts) for the complete artifact layout.
Verify the initial policy files:

```bash
test -f checkpoints/mimic-lite/32x8192-huge/policy.yaml
test -f checkpoints/mimic-lite/32x8192-huge/policy.onnx
```

### 3. Test ONNX inference

```bash
uv run scripts/test_policy_inference.py \
  --policy_config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --inference_backend onnx-cpu
```

### 4. Run integrated MuJoCo tracking

```bash
uv run sim2real/sim_env/integrated_sim2sim.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-path hf://elijahgalahad/any4hdmi-g1-lafan/motions/walk1_subject1.npz
```

The runner initializes the robot from the first motion frame, waits five
seconds, tracks the clip, and holds the final pose. Open the mjviser URL printed
in the terminal. If Hugging Face is unavailable, replace `--motion-path` with a
local any4hdmi-compatible `.npz` file.

## Test the Realtime Path Without VR

This option replays an NPZ clip through the same canonical G1 ZMQ motion stream
used by PICO teleoperation. Open three terminals.

Terminal 1, start MuJoCo:

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
```

Terminal 2, publish a motion clip:

```bash
uv run sim2real/teleop/npz_pub.py \
  --motion-path /path/to/motion.npz
```

Terminal 3, connect the policy to the stream:

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-backend zmq
```

Publisher controls:

| Key | Action |
| --- | --- |
| `]` | Reset to frame 0 and pause. |
| `Space` | Play or pause. |
| `x` | Return to the default standing pose. |

## PICO Live Whole-Body Teleoperation

This path requires PICO, leg trackers, whole-body tracking, and XRoboToolkit.

### 1. Install the teleoperation environment

```bash
uv sync --project venv/pico
```

Install XRoboToolkit PC Service as described in
[Teleop Project (x86_64 PC)](/getting-started/teleop-x86-64), then prepare its
Python binding sources:

```bash
mkdir -p external
git clone https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git \
  external/XRoboToolkit-PC-Service-Pybind
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git \
  external/XRoboToolkit-PC-Service
bash scripts/setup/setup_xrobot_pybind.sh
```

### 2. Enable body tracking

1. Put on the headset and leg trackers.
2. Complete whole-body calibration.
3. Start XRoboToolkit and connect the headset.
4. Enable whole-body streaming.

### 3. Start the retarget publisher

```bash
uv run --project venv/pico \
  sim2real/teleop/pico_retarget_pub.py
```

Open the publisher's mjviser URL first and verify that the retargeted G1 motion
matches the operator.

### 4. Start MuJoCo

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
```

### 5. Start the tracking policy

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-backend zmq \
  --controller pico
```

PICO controls:

| Input | Action |
| --- | --- |
| `A` | Enter the initialization pose. |
| `A` + `B` | Enter policy mode. |
| `X` | Resume motion flow. |

When the publisher and policy run on different machines, add this policy
argument:

```bash
--motion-zmq-connect tcp://<publisher-ip>:28701
```

## Recommended Bring-Up Order

1. Run integrated offline tracking to validate the environment, artifacts, and
   policy.
2. Run the three-process NPZ stream path to validate the realtime interface.
3. Add PICO only after both earlier paths work.

This order isolates model, simulation, streaming, and human-input failures.

