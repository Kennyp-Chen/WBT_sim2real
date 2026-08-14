# sim2real

A lightweight and modular sim2sim and sim2real deployment stack.

Chinese version: [README_zh.md](./README_zh.md)

Full documentation: [https://egalahad.github.io/sim2real/](https://egalahad.github.io/sim2real/)

If you're looking for the HDMI deployment stack, go to [hdmi tag](https://github.com/EGalahad/sim2real/tree/hdmi).

## Runtime Artifacts

Large runtime artifacts are not stored in git. Download the shared
[sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
folder and place `checkpoints/` and `third_party/` at the repo root.

See [Download Artifacts](./docs/artifacts.md) for the expected directory
layout and onboard dependency notes.

## Quick Start

```bash
uv sync --extra inference-cpu
```

For G1 onboard installation or repair, invoke the repository Codex skill
`$configure-g1-sim2real` from `.agents/skills/configure-g1-sim2real`.

Run offline motion tracking (sim2sim):

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
uv run sim2real/rl_policy/tracking.py --robot g1 \
  --policy_config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion_path hf://elijahgalahad/any4hdmi-g1-lafan/motions/walk1_subject1.npz
```

After both processes are up, press `]` in the policy terminal to start. Open the mjviser URL printed by `base_sim.py`, then use the Elastic Band controls in the viewer UI to disable or tune the virtual gantry.

## PiPlus 22-DoF BFM-Zero

PiPlus runtime artifacts are not stored in git. Provide:

- The complete robot asset directory, including `xml/` and the sibling
  `meshes/` directory referenced by the MJCF.
- `policy.yaml` and the merged `policy.onnx` under the same checkpoint
  directory.
- A 50 Hz any4hdmi motion NPZ containing `qpos[T, 29]`.

Set the paths once per shell. `SIM2REAL_PIPLUS_MJCF` must point to the XML
inside the complete asset tree, not to an XML copied without its meshes.

```bash
export SIM2REAL_PIPLUS_MJCF=/absolute/path/to/PiPlus_S_12L8A0G2H0W/xml/PiPlus_S_12L8A0G2H0W_with_armature.xml
export PIPLUS_POLICY=checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml
export PIPLUS_MOTION=/absolute/path/to/any4hdmi_full/motions/dance2_subject2.npz
```

### Convert and merge artifacts

Convert the HumanoidVerse LAFAN pickle to 50 Hz any4hdmi motions:

```bash
uv run python scripts/convert_piplus_lafan_pkl_to_any4hdmi.py \
  --source /absolute/path/to/piplus_h0w_lafan_combined.pkl \
  --output /absolute/path/to/any4hdmi_full \
  --mjcf "$SIM2REAL_PIPLUS_MJCF" \
  --target-fps 50
```

Merge the decoder/actor and backward encoder into the single semantic-input
ONNX consumed by sim2real:

```bash
uv run --with onnx python scripts/merge_bfm_zero_piplus_onnx.py \
  --actor /absolute/path/to/exported/FBcprAuxModel.onnx \
  --encoder /absolute/path/to/exported/FBcprAuxModel_z_encoder.onnx \
  --output checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.onnx
```

### Record offline sim2sim

Record a deterministic policy-only video with the general batch recorder:

```bash
uv run python scripts/tracking_experiment/record_policy_videos.py \
  --policy bfm_zero_piplus \
  --robot piplus_h0w \
  --motion "$PIPLUS_MOTION" \
  --duration-s 60 \
  --output-dir outputs/policy_videos
```

To put the source qpos replay beside an existing policy video without ZMQ:

```bash
uv run python scripts/tracking_experiment/make_motion_policy_side_by_side.py \
  --robot piplus_h0w \
  --motion-path "$PIPLUS_MOTION" \
  --policy-video outputs/policy_videos/bfm_zero_piplus/MOTION_NAME.mp4 \
  --output outputs/policy_videos/bfm_zero_piplus/MOTION_NAME_side_by_side.mp4 \
  --duration-s 60
```

### Run the real-time ZMQ path

Use three terminals. Start the simulator first:

```bash
uv run python sim2real/sim_env/base_sim.py \
  --robot piplus_h0w \
  --sim-dt 0.005
```

Start the policy in the second terminal, then press `]` to enter policy mode:

```bash
uv run python sim2real/rl_policy/tracking.py \
  --robot piplus_h0w \
  --policy-config "$PIPLUS_POLICY" \
  --robot-io zmq \
  --controller keyboard \
  --motion-backend zmq \
  --motion-zmq-connect tcp://127.0.0.1:28701 \
  --motion-tolerance-s 0.04 \
  --rl-rate 50
```

Start the motion publisher in the third terminal. Press `space` to play or
pause; `]` resets to frame zero and pauses.

```bash
uv run python sim2real/teleop/npz_pub.py \
  --robot piplus_h0w \
  --motion-path "$PIPLUS_MOTION" \
  --initial-source motion \
  --root-body-name base_link \
  --publish-hz 50 \
  --bind 'tcp://*:28701'
```

For a self-contained synchronized recording of the same ZMQ path, use the
command below. It starts its own publisher, policy, and MuJoCo bridge; do not
run it at the same time as the three interactive terminals above on port
28701.

```bash
uv run python scripts/tracking_experiment/record_zmq_policy_videos.py \
  --robot piplus_h0w \
  --policy-config "$PIPLUS_POLICY" \
  --motion-path "$PIPLUS_MOTION" \
  --output outputs/policy_videos/bfm_zero_piplus/zmq_side_by_side_sync.mp4 \
  --duration-s 60 \
  --fps 30
```

The recorder runs motion and policy at 50 Hz, MuJoCo physics at 200 Hz, and
encodes at 30 FPS. Its left panel follows the frame actually emitted by the
ZMQ publisher, while the right panel shows the policy-driven simulation. The
final log reports nominal duration, wall duration, and recorder clock drift.

For PiPlus hardware deployment, use the ROS2-to-ZMQ adapter and the complete
state/command contract in [the PiPlus BFM-Zero sim2real guide](docs/piplus_bfmzero_sim2real.md).
The separate [PiPlus-LSE 23-DoF guide](docs/tutorials/piplus-lse-23dof-sim2real.md)
contains its non-identity hardware map, preflight launcher, and real-robot commands.

## Migrating to sim2real

This repo includes a Codex skill for adapting policies trained in external codebases into `sim2real`:

```text
.agents/skills/adapt-policy-to-sim2real
```

Converted checkpoints are distributed through the shared
[sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
folder.

Currently supported adapted / distributed checkpoint families:

| Policy family | Config path(s) | Notes |
| --- | --- | --- |
| Mimic-Lite | `checkpoints/mimic-lite` | Native mimic-lite tracking checkpoints. |
| BFM-Zero | `checkpoints/bfm-zero/exp_lafan40-100style_update_z10/policy.yaml` | Latent-conditioned motion tracker. |
| BFM-Zero PiPlus 22-DoF | `checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml` | PiPlus decoder and backward encoder merged into one semantic-input ONNX. |
| BFM-Zero PiPlus-LSE 23-DoF | `checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml` | PiPlus-LSE policy with a dedicated 23-DoF joint/body and hardware-order contract. |
| ScaleBFM | `checkpoints/scalebfm` | ScaleBFM Humanoid Transformer M and XL ONNX exports from [WeishuaiZeng/ScaleBFM](https://huggingface.co/WeishuaiZeng/ScaleBFM). |
| SONIC release | `checkpoints/sonic/release` | Release G1 and SMPL encoder variants. |
| SONIC low-latency | `checkpoints/sonic/low_latency` | Low-latency G1 and SMPL variants. |
| HoloMotion v1.4.0 | `checkpoints/holomotion/v1_4_0/policy.yaml` | Uses the official unmodified ONNX from [HorizonRobotics/HoloMotion_models](https://huggingface.co/HorizonRobotics/HoloMotion_models/resolve/main/HoloMotion_motion_tracking_model_v1.4.0/exported/model_14000.onnx); place it at `checkpoints/holomotion/v1_4_0/policy.onnx`. |
| TeleopIT | `checkpoints/teleopit/policy.yaml` | TeleopIT policy wrapper. |
| Humanoid-GPT | `checkpoints/humanoid-gpt/policy.yaml` | Humanoid-GPT policy wrapper. |
| HEFT | `checkpoints/heft` | PMG and compliance variants. |
| TWIST2 | `checkpoints/twist2/policy.yaml` | TWIST2 policy wrapper. |

![Unified cross-codebase tracking evaluation](assets/mimic_lite_cross_codebase_tracking_eval.png)

The comparison uses fresh runs for all 13 policy variants on LAFAN-40,
PHUMA-30, and a direction-clean Root-90 set whose clips move 1.5--3.0 m
without changing their labelled forward, backward, or sideward direction.

For a fair comparison, we report the motion-lookahead latency required by each
policy, defined by its furthest required future-reference frame. All values use
the shared 50 Hz reference-motion contract.

| Policy | MimicLite | BFM-Zero | ScaleBFM | SONIC release | SONIC low-latency | HoloMotion | TeleopIT | Humanoid-GPT | HEFT | TWIST2 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Motion-lookahead latency | 0.08 s | 0.12 s | 0.10 s | 0.90 s | 0.18 s | 0.20 s | 0.00 s | 0.02 s | 0.12 s | 0.00 s |

## Real-robot Environments

Robot SDKs are kept out of the generic root environment. G1 inline deployment
uses `uv sync --extra inference-cpu --extra robot-g1`. See
[Robot I/O Modes](./docs/robot_io.md) for setup and deployment commands.

## Next Steps

- [Docs Home](./docs/README.md)
- [Getting Started](./docs/getting-started/README.md)
- [Offline Motion Tracking Tutorial](./docs/tutorials/offline-motion-tracking.md)
- [Pico Teleoperation Tutorial](./docs/tutorials/pico-teleoperation.md)

## Citation

If you find sim2real useful in your research, please cite:

```bibtex
@misc{sim2real2026,
  author       = {{RoboParty Lab Team}},
  title        = {sim2real: A Lightweight and Modular Sim2sim and Sim2real Deployment Stack},
  year         = {2026},
  howpublished = {\url{https://github.com/EGalahad/sim2real}},
  note         = {Documentation: \url{https://egalahad.github.io/sim2real/}}
}
```
