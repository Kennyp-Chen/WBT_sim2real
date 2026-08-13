---
title: GEM Multimodal Motion Front End
slug: /reference/gem-multimodal-motion
---

# GEM Multimodal Motion Front End

Last verified: 2026-08-12 against NVlabs/GENMO commit
`16bebf402d8893184249ee206d957b8248cd8310`.

## Executive Summary

[GEM](https://github.com/NVlabs/GENMO), formerly GENMO, is a generalist human
motion model for pose estimation and motion generation. Its useful role in this
repository is an upstream human-motion front end:

```text
video / 2D keypoints / text / audio / music
    -> GEM
    -> SMPL body parameters
    -> robot retargeting and runtime adapter
    -> SONIC or BFM-Zero
    -> MuJoCo or robot hardware
```

The integration is feasible, but GEM is not a robot controller. It does not
directly produce G1 or PiPlus motor commands, and raw RGB, text, or audio cannot
be passed directly to the existing policy ONNX files.

The recommended first target is recorded-video GEM output driving SONIC SMPL
sim2sim. SONIC already consumes a canonical SMPL reference stream. PiPlus
BFM-Zero requires an additional SMPL-to-PiPlus 22-DoF retargeting layer.

## Work and Project

- Paper: [GEM: A Generalist Model for Human Motion](https://arxiv.org/abs/2505.01425), ICCV 2025 Highlight.
- Project page: [NVIDIA Research GEM](https://research.nvidia.com/labs/dair/gem/).
- Code: [NVlabs/GENMO](https://github.com/NVlabs/GENMO). The project was renamed from GENMO to GEM in December 2025.
- Full hands and face: [GEM-X](https://github.com/NVlabs/GEM-X).
- License: NVIDIA OneWay Noncommercial. The code and derivatives are limited to research or evaluation use unless separately licensed.

## What GEM Provides

GEM unifies human motion estimation and generation without separate
task-specific heads. The released GEM-SMPL model advertises video, text, audio,
and music conditioning.

| Input | Released path | Current maturity | Output relevant to sim2real |
|---|---|---|---|
| Recorded video | `scripts/demo/demo_smpl_hpe.py` | Official offline demo | `smpl_params.pt` |
| Mixed video and text | `scripts/demo/demo_smpl.py` | Official offline demo | SMPL parameters and rendered videos |
| Webcam or sequential video | `scripts/demo/demo_webcam.py` | Official ONNX streaming demo | Per-frame SMPL body parameters |
| Text | `demo_smpl.py` text segments | Segment generation; default 300 frames at 30 FPS | Generated SMPL motion |
| Audio and music | Model, datasets, and full training configuration | Supported by the model, but no equivalent live microphone demo is released | Generated SMPL motion after adding an input front end |

The offline output contains:

```text
smpl_params.pt
  body_params_global
    body_pose       [T, 21, 3] axis-angle
    global_orient   [T, 3] axis-angle
    transl          [T, 3]
    betas           [T, 10] when present
  body_params_incam
  K_fullimg
  segment_info      for mixed inputs
```

For robot tracking, `body_params_global` is the useful source. Camera-space
translation and rendering outputs are not robot commands.

## Video Pipeline

The official real-time demo runs this pipeline frame by frame:

```text
camera or video
    -> YOLOX person detection
    -> ByteTrack tracking
    -> ViTPose-H 2D keypoints
    -> HMR2 image features (optional)
    -> GEM-SMPL denoiser ONNX
    -> streaming global rollout
```

Important runtime properties:

- The default context is 120 frames, about four seconds at 30 FPS.
- The initial result is unavailable until the context window is filled.
- `--no_imgfeat` skips HMR2 and is the fastest official mode, with an expected accuracy tradeoff.
- The asynchronous pipeline improves throughput but can add pipeline lag. `--no_async_pipeline` removes that overlap at lower throughput.
- The ONNX artifacts total about 8.7 GB. The official installation notes recommend an A100-class or newer GPU and report approximately 16 GB of inference memory.
- The local workstation has a 16 GB GeForce RTX 5060 Ti. It is close to the published memory requirement, so measured latency and peak memory, rather than a nominal real-time claim, must determine whether full or `--no_imgfeat` mode is usable.

The practical deployment architecture should therefore run GEM on a workstation
GPU and publish timestamped motion to the policy computer. Running the complete
vision stack on a JetPack 5 robot computer is not the first implementation
target.

## Relationship to SONIC

SONIC is the shortest integration path because this repository already has a
SMPL-specific policy and runtime. The current SMPL ZMQ contract is documented in
[SONIC SMPL Input](/reference/sonic-smpl-input).

SONIC consumes:

| Field | Shape | Source |
|---|---:|---|
| `smpl_body_pose_aa` | `[N, 21, 3]` | GEM `body_pose` |
| `smpl_joint_pos_root` | `[N, 24, 3]` | Canonical SONIC SMPL FK |
| `smpl_root_quat_w` | `[N, 4]` | GEM `global_orient`, converted to the SONIC frame |
| `joint_pos` | `[N, 29]` | G1 retargeting; SONIC currently uses six wrist joints |

GEM supplies the first-stage human pose, but it does not supply the last G1
field. A complete SONIC integration still needs G1 retargeted wrist roll, pitch,
and yaw references. During interface bring-up, the publisher may deliberately
use the robot default pose for these six values, but that is a degraded validation
mode and not complete whole-body retargeting.

## Relationship to PiPlus BFM-Zero

PiPlus BFM-Zero is a robot motion-tracking controller. Its exported graph
expects semantic robot and reference-motion observations, not multimodal raw
input. A PiPlus path must add:

```text
GEM SMPL
    -> SMPL-to-PiPlus retargeting / IK
    -> PiPlus 22-DoF qpos and body FK
    -> BFM-Zero future reference observations
    -> PiPlus BFM-Zero policy
```

Required source-of-truth information includes PiPlus joint order, limits,
default pose, robot asset, root and body frames, training-time body selection,
motion FPS, and the exact observation implementation. The current PiPlus
BFM-Zero deployment already contains the merged policy, deploy YAML,
observations, MJCF support, motion conversion, ZMQ playback, and synchronized
video recording. It does not yet contain a SMPL-to-PiPlus retargeter.

## Feasibility by Input Mode

### Recorded Video

Feasible now and the recommended first milestone:

```text
video.mp4
    -> GEM offline inference
    -> smpl_params.pt
    -> GEM-to-SONIC publisher
    -> SONIC SMPL sim2sim
```

This isolates coordinate, skeleton, timing, and wrist-reference errors before
adding camera latency.

### Live Video

Technically feasible using the official ONNX webcam pipeline. The production
adapter must publish each completed GEM frame with its source capture timestamp,
buffer dropped frames, reject low-confidence or implausible poses, and hold a
stable reference on loss of tracking.

### Language

Feasible as chunked command-to-motion generation, not as instantaneous
word-by-word control. The official demo represents a prompt as a fixed-duration
segment, defaulting to ten seconds. An interactive implementation needs a prompt
queue, generation worker, overlap/crossfade between segments, cancellation, and
a safe transition to standing.

### Music and Audio

The released model and training data paths contain audio and music conditioning,
but the repository does not provide a live microphone interface comparable to
the webcam demo. A real-time implementation needs audio capture, feature
extraction matching the training representation, chunked inference, beat-aware
stitching, and the same robot feasibility filters used for text generation.

## Engineering Risks

1. Coordinate conventions: GEM, SONIC, MuJoCo, and robot hardware do not share a single up-axis, quaternion order, or root-frame convention.
2. Robot retargeting: SMPL body parameters do not determine robot wrist angles or a dynamically feasible robot configuration.
3. Timing: the policy runs at 50 Hz while GEM commonly operates at 30 Hz and the webcam model has a 120-frame context.
4. Motion quality: video pose estimates may jitter, lose occluded limbs, or jump between people.
5. Physical feasibility: text- or music-generated contacts and speeds may be outside the policy training distribution.
6. Compute: full GEM vision inference is large enough that model loading, memory pressure, and actual frame latency must be benchmarked on the target GPU.
7. Licensing: GEM's noncommercial license must be reviewed before any public or commercial deployment.

## Safety and Validation Requirements

Before robot hardware, every source must pass:

- shape, joint-order, quaternion-order, and frame validation;
- monotonic source and publish timestamps;
- fixed-FPS replay comparison against the source motion;
- pose velocity and joint-limit filtering;
- tracking-loss fallback to a stationary reference;
- sim2sim fall detection and root-trajectory logging;
- an explicit operator stop path and conservative command limits.

The detailed implementation sequence and live progress ledger are maintained in
[GEM Integration Plan](/tutorials/gem-integration-plan).

