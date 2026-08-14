---
title: GEM Integration Plan
slug: /tutorials/gem-integration-plan
---

# GEM Integration Plan

This document is the durable execution plan and progress ledger for connecting
GEM video, language, audio, and music motion to the base policies in this
repository. Update the status table, evidence links, blockers, and resume point
whenever work advances.

For the technical background, see
[GEM Multimodal Motion Front End](/reference/gem-multimodal-motion).

## Project Goal

Deliver one timestamped motion-source interface that can accept:

1. recorded video;
2. a live camera;
3. interactive language prompts;
4. live or recorded music/audio;

and drive, in order of implementation risk:

1. SONIC SMPL on G1;
2. PiPlus 22-DoF BFM-Zero;
3. robot hardware after sim2sim validation.

## Progress Ledger

Last updated: 2026-08-13.

| ID | Status | Deliverable | Evidence / current result |
|---|---|---|---|
| PRE-001 | Done | SONIC release and low-latency G1/SMPL ONNX integrated | `checkpoints/sonic/{release,low_latency}` |
| PRE-002 | Done | SONIC SMPL ZMQ contract implemented | `sim2real/teleop/smpl_stream.py`, `SmplRealtimeMotionBuffer` |
| PRE-003 | Done | Live PICO/XRobot SMPL publisher with GMR wrist references | `sim2real/teleop/pico_retarget_pub.py --publish-smpl` |
| PRE-004 | Done | PiPlus 22-DoF BFM-Zero merged ONNX and sim2real observations | commit `7a3b2c6` |
| PRE-005 | Done | PiPlus NPZ/ZMQ playback and synchronized video recording | `record_policy_videos.py`, `record_zmq_policy_videos.py` |
| PRE-006 | In progress | HT PiPlus-LSE 23-DoF and Hi 25-DoF BFM-Zero contracts/deploy artifacts | BFM-Zero observations, policy YAML/ONNX, MJCF resolution, and motion tooling are present; both still need full sim2sim validation |
| GEM-000 | Done | GEM paper/project/runtime investigation | GEM report and local clone at `/home/sunteng/Projects/WBC_Telep/GENMO` |
| GEM-001 | Done | Recorded GEM `smpl_params.pt` to SONIC SMPL ZMQ publisher | `gem_smpl_pub.py` converts GEM Y-up SMPL, resamples to the 50 Hz SONIC clock, and publishes ten future frames |
| GEM-002 | Done | Recorded-video GEM inference environment and benchmark | The official 312-frame tennis demo produced finite parameters and four rendered views under GENMO `outputs/gem_runs/tennis/` |
| GEM-003 | Done | Full G1 wrist retargeting from GEM SMPL | GMR produced 312 finite, in-limit G1 references in the policy's 29-joint order |
| GEM-004 | In progress | Recorded-video SONIC sim2sim and comparison video | 312-frame ZMQ sim2sim and three-panel video passed visual/timing validation; quantitative tracking/fall report remains |
| GEM-005 | Not started | Live webcam GEM-to-SONIC stream | Depends on recorded-video validation |
| GEM-006 | In progress | Interactive text-to-motion stream | GENMO supports mixed video/text segments; the repository adapter and bounded chunk queue are being added |
| GEM-007 | In progress | Music/audio-to-motion stream | GEM audio conditioning is available through its raw waveform encoder; file-based adapter and timing checks are being added |
| GEM-008 | Done | SMPL-to-PiPlus 22-DoF retargeter | PiPlus GMR mapping produced `[519,29]` at 50 Hz with no limit violations; any4hdmi contract validation passed |
| GEM-009 | In progress | PiPlus/G1 BFM-Zero multimodal sim2sim | 312-frame synchronized ZMQ reference/policy videos passed visual timing/stability inspection; quantitative tracking report remains |
| GEM-010 | Not started | Hardware safety gate and limited real-robot trial | Depends on stable sim2sim metrics |
| GEM-011 | Deferred | GEM retargeting for HT PiPlus-LSE 23-DoF and Hi 25-DoF | Start only after G1 and PiPlus 22-DoF BFM-Zero sim2sim are complete; do not assume the 22/23/25-DoF contracts are interchangeable |

Status values are `Not started`, `In progress`, `Blocked`, and `Done`. Mark a
task `Done` only after its acceptance criteria and evidence are present.

## Current Repository State

### SONIC

- Release SMPL policy is available at `checkpoints/sonic/release/smpl/policy.yaml`.
- The ONNX contract is `smpl_input[840]`, `proprioception[930]`, with outputs `action[29]` and `token[64]`.
- `motion_backend: smpl_zmq` consumes port 28702.
- The SMPL stream already handles future-window buffering, root-yaw continuity, timestamp alignment, and a stationary default pose.
- The encoder uses canonical 24-joint root-local SMPL positions, relative root orientation, and six retargeted G1 wrist joint angles.

### PiPlus BFM-Zero

- The PiPlus 22-DoF actor and backward encoder are merged into one policy ONNX.
- PiPlus-specific observation construction and deploy YAML are implemented.
- The LAFAN training pickle can be converted to any4hdmi NPZ.
- Offline, ZMQ, and synchronized side-by-side video workflows are implemented.
- GEM SMPL can now be retargeted into PiPlus any4hdmi qpos with
  `scripts/retarget_gem_smpl.py` and the checked-in PiPlus GMR map.
- `sim2real/teleop/npz_pub.py` is the canonical robot-motion ZMQ publisher on
  port 28701. It replays any4hdmi/NPZ qpos and publishes the
  `joint_pos`/`body_pos_w`/`body_quat_w` schema consumed by
  `motion_backend=zmq`.
- `sim2real/teleop/gem_bfmzero_pub.py` adapts GEM: it runs GMR once to convert
  `smpl_params.pt` to RobotCfg-ordered qpos, resamples to the publish FPS,
  saves a reproducible NPZ/manifest, and delegates to `npz_pub`.

### HT BFM-Zero robots

- `piplus_lse_23dof` and `hi_25dof` are now registered with their own joint/body
  contracts and BFM-Zero observations. Their deploy artifacts live under
  `checkpoints/bfm-zero/{piplus-lse-23dof,hi-25dof}`.
- These two robots are intentionally not yet connected to GEM. First finish
  full standalone sim2sim validation for G1 and PiPlus 22-DoF, then add a
  robot-specific SMPL/GMR mapping and motion contract for each new robot.
- Do not reuse the PiPlus 22-DoF GEM mapping or silently pad action dimensions:
  the new policies are 23-DoF and 25-DoF contracts with different body orders.

### GEM

- The official repository is cloned at `/home/sunteng/Projects/WBC_Telep/GENMO`.
- The workstation has an RTX 5060 Ti with 16 GB VRAM and sufficient disk space.
- The GENMO source is cloned at `/home/sunteng/Projects/WBC_Telep/GENMO`.
- `gem_smpl.ckpt` and the six official webcam ONNX files are downloaded into
  `inputs/pretrained/` and `inputs/onnx/`; all seven SHA-256 values match the
  NVIDIA GEM-X Hugging Face metadata.
- Raw HMR2 and ViTPose checkpoints are downloaded into
  `inputs/checkpoints/{hmr2,vitpose}/` from the public `camenduru/GVHMR` mirror;
  their byte sizes and SHA-256 values match that mirror's metadata.
- `SMPLX_NEUTRAL.npz` is installed under
  `inputs/checkpoints/body_models/smplx/`.
- The small GVHMR body-model runtime resources used by GEM are installed under
  `gem/utils/body_model/`.
- The official tennis demo output is at
  `/home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt`:
  312 finite frames with global translation, root orientation, body pose, and shape.
- GMR is cloned at `/home/sunteng/Projects/WBC_Telep/GMR`; it is an optional
  offline retarget dependency and is not required by policy inference.

## Target Architecture

```text
                         +------------------+
recorded/live video ---->| GEM pose estimate|----+
                         +------------------+    |
                                                  v
text prompt ------------>| GEM generation   |  canonical SMPL stream
music/audio ------------>| + chunk manager  |    |          |
                         +------------------+    |          |
                                                  |          +-> G1 retarget -> SONIC
                                                  |
                                                  +-> PiPlus retarget -> BFM-Zero
```

All sources must converge on a shared motion record with explicit source FPS,
frame index, source timestamp, confidence/validity state, and segment boundary.
Policy-specific retargeting belongs after this common representation.

## Concrete Recorded-Tennis Dataflow

The following is the exact flow used by the current tennis experiment. It is
important that the SONIC and BFM-Zero branches are not treated as the same
runtime interface.

### 1. Human video to GEM parameters

```text
GENMO/inputs/demo/tennis.mp4
    -> GEM recorded-video inference
    -> outputs/gem_runs/tennis/smpl_params.pt
```

The GEM file is a per-frame SMPL-X parameter record, not a robot trajectory and
not a policy action. The current file contains 312 frames and these fields:

| Field | Shape | Meaning |
|---|---:|---|
| `body_params_global.body_pose` | `[T,63]` | 21 non-root SMPL body joints as local axis-angle rotations |
| `body_params_global.global_orient` | `[T,3]` | SMPL root orientation as axis-angle |
| `body_params_global.transl` | `[T,3]` | Global root translation |
| `body_params_global.betas` | `[T,10]` | SMPL-X body shape coefficients |
| `body_params_incam.*` | same | The corresponding camera-coordinate parameters |
| `K_fullimg` | `[T,3,3]` | Per-frame camera intrinsics |

For this experiment the source clock is treated as 30 Hz. The source video
metadata is approximately 29.83 Hz, so exact timestamp reconstruction is a
separate future improvement.

### 2. SONIC branch: SMPL-ZMQ to G1 policy

```text
smpl_params.pt
    -> gem_smpl_pub.py
       - canonical SONIC SMPL FK
       - Y-up/root-frame conversion
       - GMR G1 wrist reference
       - 30 Hz -> 50 Hz resampling
       - 10-frame future window
    -> ZMQ tcp://*:28702
    -> Tracking(motion_backend=smpl_zmq)
    -> SONIC SMPL ONNX
    -> 29-joint action -> PD torque -> G1 MuJoCo / hardware
```

`gem_smpl_pub.py` uses the GEM body rotations to build the canonical SONIC
fields `smpl_joint_pos_root [N,24,3]` and `smpl_root_quat_w [N,4]`. It does not
send camera intrinsics to the policy. The six robot wrist angles are supplied
separately by GMR because SONIC's encoder expects G1 wrist roll/pitch/yaw, not
SMPL wrist positions. The publisher resamples all rotations with SLERP and
positions/joint references linearly to the SONIC 50 Hz control clock before
publishing future frames.

The policy receives the current G1 proprioception plus the future SMPL window.
Its deployed output is a 29-joint action (and the checkpoint's token/state
output where applicable); the controller converts that action into joint
targets and MuJoCo or robot torques. The `tennis_source_g1_reference_sonic.mp4`
recording shows source video, the GMR reference, and this SONIC execution.

### 3. BFM-Zero branch: SMPL-to-G1 NPZ to policy

```text
smpl_params.pt
    -> scripts/retarget_gem_smpl.py --robot g1
       - full SMPL-X global parameters
       - GMR SMPL-X -> G1 IK
       - root XY normalization and MJCF validation
    -> outputs/gem_retarget/tennis/g1/motions/tennis.npz [312,36] @ 30 Hz
    -> any4hdmi loader resamples to 50 Hz [519,36]
       and computes reference body FK/velocities
    -> integrated_sim2sim.py + BFM-Zero ONNX
    -> 29-joint action -> PD torque -> G1 MuJoCo / hardware
```

The offline recording uses `motion_backend=npz`: the 30 Hz NPZ is loaded by the
policy runtime at its configured 50 Hz target FPS, which is why the internal
motion sequence has 519 frames. The video writer remains at 30 fps and
therefore contains 312 video frames.
The policy consumes robot proprioception/history and the future G1 reference
window; it does not consume the raw SMPL fields or `smpl_params.pt` directly.

The current three-panel artifact is
`outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4`.

The canonical GEM-to-BFM-Zero ZMQ path is now:

```text
smpl_params.pt
    -> gem_bfmzero_pub.py
       - GMR SMPL-X -> RobotCfg qpos
       - qpos resampling to 50 Hz
       - reproducible any4hdmi NPZ
    -> npz_pub schema on tcp://*:28701
    -> Tracking(motion_backend=zmq)
    -> BFM-Zero ONNX -> robot action/torque
```

Run the adapter directly:

```bash
HF_HUB_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1 \
uv run python sim2real/teleop/gem_bfmzero_pub.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot g1 --bind 'tcp://*:28701'
```

The synchronized recorder accepts either an existing NPZ or GEM parameters:

```bash
MUJOCO_GL=egl HF_HUB_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1 \
uv run python scripts/tracking_experiment/record_zmq_policy_videos.py \
  --policy-config checkpoints/bfm-zero/exp_lafan40-100style_update_z10/policy.yaml \
  --robot g1 \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --motion-bind 'tcp://*:28712' \
  --motion-connect 'tcp://127.0.0.1:28712' \
  --output outputs/gem_retarget/tennis/policy_videos/bfm_zero_g1/tennis_zmq_side_by_side.mp4
```

The current three-panel artifact is
`outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4`.
Its right panel comes from this BFM-Zero ZMQ run and uses the same 640x720
rendering aspect ratio as the SONIC reference panel.

## Phase 1: Recorded Video to SONIC

### GEM-001: Generic GEM SMPL Publisher

Objective: load official GEM `smpl_params.pt`, convert its global body parameters
to the existing SONIC SMPL contract, and replay them on port 28702 at the SONIC
50 Hz control clock.

Implementation requirements:

- load tensors without importing the GEM repository;
- require `body_params_global.body_pose` and `global_orient`;
- validate `[T,21,3]` and `[T,3]` axis-angle layouts;
- construct canonical SONIC `smpl_joint_pos_root` using the existing
  `human_joints_info.pkl` FK path;
- convert GEM Y-up root orientation to the SONIC root quaternion convention;
- publish monotonic frame indices, source timestamps, publish timestamps, and
  `motion_first_frame`;
- publish a configurable future window so the SONIC ten-step observation has
  enough current/future data;
- support optional G1 joint reference arrays in NPZ/PT format;
- warn clearly when falling back to the G1 default pose, because wrist tracking
  is degraded in that mode;
- support `--dry-run` for file/schema validation without ZMQ.

Acceptance criteria:

- unit tests cover valid GEM tensors, malformed shapes, default wrist fallback,
  external G1 joint reference loading, and payload shapes;
- `uv run python -m py_compile` passes;
- a synthetic GEM file passes `--dry-run` and emits the documented shapes;
- a subscriber receives monotonic, correctly shaped messages at the requested
  FPS.

### GEM-002: GEM Environment and Recorded Inference

Objective: produce the first real `smpl_params.pt` from a short recorded human
video.

Tasks:

1. Create the GEM Python 3.10 CUDA environment outside the sim2real root.
2. Obtain `SMPLX_NEUTRAL.npz` and place it under the GEM input path.
3. Download or provide the GEM-SMPL checkpoint, HMR2, and ViTPose assets.
4. Run video-only inference with `--no_render` first.
5. Inspect tensor shapes, NaNs, orientation continuity, duration, GPU memory, and elapsed time.
6. Run the rendered comparison only after parameter output is valid.

Acceptance criteria:

- command, source video, exact checkpoint, GEM commit, GPU, elapsed time, peak
  memory, and output path are recorded below;
- `smpl_params.pt` contains finite global body parameters for the full clip;
- GEM rendering follows the subject without identity switches.

The licensed `SMPLX_NEUTRAL.npz` is installed locally and was used for the
recorded tennis run.

### GEM-003: Complete G1 Wrist Retargeting

Objective: replace the default-pose wrist fallback with robot joint references
derived from the same GEM motion.

Tasks:

1. Convert GEM SMPL rotations/joints into the source representation expected by a G1 retargeter.
2. Reuse GMR if it supports SMPL input; otherwise add a minimal SMPL-to-G1 IK adapter.
3. Preserve the exact IsaacLab joint order in the SONIC policy YAML.
4. Extract and verify the six wrist roll/pitch/yaw references.
5. Compare the result against a known official SONIC SMPL plus G1 pair.

Acceptance criteria:

- no default-pose fallback warning;
- all 29 G1 references are finite and within limits;
- wrist directions agree visually with the GEM human motion;
- the SONIC input vector is exactly 840 values and the wrist slice changes when the arms rotate.

### GEM-004: Recorded-Video SONIC Sim2sim

Install the retarget dependency once while GitHub is reachable, then generate G1
wrist references. After this installation, retargeting and policy inference can
run offline:

```bash
uv sync --extra retarget
uv run python scripts/retarget_gem_smpl.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot g1 \
  --target-fps 30 \
  --output-dir outputs/gem_retarget/tennis/g1
```

Run three terminals:

```bash
uv run python sim2real/teleop/gem_smpl_pub.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --joint-reference outputs/gem_retarget/tennis/g1/tennis_joint_reference.npz
```

For a self-contained synchronized recording:

```bash
MUJOCO_GL=egl uv run python scripts/tracking_experiment/record_gem_sonic_video.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --joint-reference outputs/gem_retarget/tennis/g1/tennis_joint_reference.npz \
  --reference-motion outputs/gem_retarget/tennis/g1/motions/tennis.npz \
  --output outputs/gem_retarget/tennis/policy_videos/sonic_release_smpl/tennis.mp4
```

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
```

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy-config checkpoints/sonic/release/smpl/policy.yaml \
  --inference-backend onnx-cpu \
  --robot-io zmq \
  --controller passive
```

Acceptance criteria:

- the policy starts after the SMPL buffer has accumulated its future window;
- reference and policy timelines are aligned;
- no NaNs, shape errors, or root-heading discontinuities occur;
- a side-by-side source-video/GEM/robot recording is saved;
- fall status and root tracking error are recorded.

## Phase 2: Live Video

### GEM-005: Webcam Stream

Tasks:

1. Benchmark the official ONNX modules with and without HMR2 image features.
2. Extend or wrap `demo_webcam.py` to emit a structured per-frame result rather than only render it.
3. Preserve camera capture timestamp through the asynchronous pipeline.
4. Publish the same SMPL ZMQ schema used by GEM-001.
5. Add confidence gating, person-locking, dropout hold, and controlled recovery.
6. Measure camera-to-policy and camera-to-simulation latency separately.

Acceptance criteria:

- sustained throughput is at least the selected input FPS or the publisher reports intentional downsampling;
- timestamps never make the policy appear ahead of the source video;
- tracking loss produces a stable hold rather than a sudden pose jump;
- a 60-second live-camera comparison video and latency report are saved.

## Phase 3: Language and Music

### GEM-006: Interactive Text

Implement a prompt service with fixed-duration generation chunks, a bounded
queue, cancellation, segment overlap, and transition-to-stand. Start with manual
prompts and do not connect speech recognition until text control is stable.

Acceptance criteria:

- a new prompt cannot reorder already published motion;
- the policy never consumes an empty future window;
- segment boundaries do not exceed configured pose/angular velocity limits;
- prompt, seed, generation time, and generated artifact are logged.

### GEM-007: Music and Audio

First reproduce an official offline audio/music-conditioned sample. Then add
microphone/file capture and feature extraction matching GEM's training input.
Use beat-aware chunk overlap rather than joining independent clips at arbitrary
frames.

Acceptance criteria:

- offline generated motion is reproducible from a saved audio file;
- live audio buffering has bounded latency and no timestamp reversal;
- a 60-second music-to-motion SONIC sim2sim recording is saved.

## Phase 4: PiPlus BFM-Zero

### GEM-008: SMPL-to-PiPlus Retargeting

Trace the PiPlus training motion representation and implement one reusable
retargeter that outputs the exact 22-DoF order and root/body frames used by the
BFM-Zero backward encoder. Validate the retargeted qpos through the PiPlus MJCF
before policy inference.

```bash
uv run python scripts/retarget_gem_smpl.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot piplus_h0w \
  --output-dir outputs/gem_retarget/tennis/piplus_h0w
```

Acceptance criteria:

- qpos is `[T,29]` in the configured MuJoCo order;
- all named joints match the deploy YAML and MJCF;
- FK body positions/quaternions are finite and continuous;
- converted known motions agree numerically or visually with the existing PiPlus LAFAN dataset.

### GEM-009: PiPlus/G1 Multimodal Sim2sim

Feed the retargeted motion through the existing normal ZMQ BFM-Zero path. Reuse
the synchronized recording workflow and record source video, retargeted
reference, and policy simulation on one timeline.

## Phase 5: Hardware Gate

### GEM-010: Controlled Robot Trial

Hardware is blocked until recorded and live sim2sim pass. Add input validity,
velocity, height, joint-limit, fall, communication-timeout, and operator-stop
gates. Begin with upper-body motion and fixed feet before permitting locomotion.

## Experiment Log

Append one row per meaningful run. Do not overwrite failed runs; they are useful
evidence.

| Date | ID | Source | Command / config | Output | Result / metrics |
|---|---|---|---|---|---|
| 2026-08-12 | GEM-000 | NVlabs/GENMO `16bebf4` | repository inspection | GEM report | Video/text official demos confirmed; audio/music model support confirmed; live webcam ONNX confirmed |
| 2026-08-12 | PRE-004 | PiPlus BFM-Zero | commit `7a3b2c6` | PiPlus policy/video workflow | Integrated and pushed before GEM work began |
| 2026-08-12 | GEM-002 | NVIDIA GEM-X HF mirror + `camenduru/GVHMR` + GENMO `16bebf4` | resumable `wget -c` transfers | `/home/sunteng/Projects/WBC_Telep/GENMO/inputs/{pretrained,onnx,checkpoints}` | 9/9 files complete; all hashes verified; only licensed `SMPLX_NEUTRAL.npz` remains |
| 2026-08-13 | GEM-002 | PiPlus `jumps1_subject1_...clip0.mp4` | `demo_smpl_hpe.py --no_render --ckpt_path inputs/pretrained/gem_smpl.ckpt` | `outputs/gem_runs/jumps1_subject1_20260806_PiPlus_S_12L8A0G2H0W_LSE_ZedMini_260804_clip0/{smpl_params.pt,0_kp2d_overlay.mp4}` | 300 frames; 22.8 s; global/incam body tensors and camera intrinsics all finite; rendering deferred |
| 2026-08-13 | GEM-002 | GVHMR official `tennis.mp4` demo | GEM recorded-video inference and rendering | `/home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/` | 312 finite frames; `0_kp2d_overlay`, `1_incam`, `2_global`, and `3_incam_global_horiz` videos saved |
| 2026-08-13 | GEM-003/008 | tennis `smpl_params.pt` | `scripts/retarget_gem_smpl.py` with GMR | `outputs/gem_retarget/tennis/{g1,piplus_h0w}` | G1 `[312,36]` at 30 Hz; PiPlus `[519,29]` at 50 Hz; normalized root quaternions and zero joint-limit violations |
| 2026-08-13 | GEM-004 | tennis GEM + G1 GMR reference | `record_gem_sonic_video.py`, release SMPL policy | `outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_sonic.mp4` | 312 policy frames at 30 fps; source starts at 0.000 s, policy at 0.220 s; no shape/NaN/runtime failure; visual stability passed |
| 2026-08-13 | GEM-009 | tennis PiPlus GMR reference | `record_zmq_policy_videos.py`, PiPlus BFM-Zero | `outputs/gem_retarget/tennis/comparisons/tennis_source_piplus_reference_bfmzero.mp4` | 312 policy frames at 30 fps; source starts at 0.000 s, policy at 0.180 s; synchronized reference/policy remained upright |
| 2026-08-13 | GEM-009 | tennis GEM -> G1 BFM-Zero robot-motion ZMQ | `record_zmq_policy_videos.py --gem-params`, `gem_bfmzero_pub.py`, BFM-Zero G1 policy | `outputs/gem_retarget/tennis/policy_videos/bfm_zero_g1/tennis_zmq_side_by_side.mp4`, `outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4` | 312 frames at 30 fps; GMR qpos at 30 Hz, NPZ/ZMQ at 50 Hz; `motion_backend=zmq` received 29 joints/33 bodies; wall drift +0.371 s; visual stability passed |

## Blockers and Required Inputs

There is no external blocker for the recorded-video path. Before moving to live
camera or hardware, add quantitative root/body tracking and fall metrics for the
two current recordings, then implement camera confidence/dropout handling.

Potential later inputs:

- a representative short recorded video with one visible full-body subject;
- the desired language prompt set and action safety vocabulary;
- representative music files for offline reproduction;
- representative camera and music inputs for the next phases.

## Resume Point

When resuming this project, do the following in order:

1. Read the progress ledger above and select the first `In progress` item.
2. Inspect `git status` and preserve unrelated user files.
3. Finish GEM-004/GEM-009 by adding a quantitative tracking/fall report to the two synchronized recorders.
4. Re-run the tennis artifacts above and compare the report with the saved videos.
5. Continue GEM-006/GEM-007 using the bounded adapters below; add every real run to this log.
6. Do not start GEM-011 until G1 and PiPlus 22-DoF BFM-Zero sim2sim acceptance is complete.

## Deferred Final Phase: GEM for the New HT Robots

GEM integration for `piplus_lse_23dof` and `hi_25dof` is deliberately the final
robot-specific phase. Before starting it, complete all of the following:

1. standalone offline and ZMQ sim2sim for both new policies;
2. final-frame, fall, root tracking, and action-limit checks;
3. confirmation of the exact training MJCF body order and motion FPS for each;
4. a separate GMR/SMPL mapping and any4hdmi manifest for each robot.

The final adapters must target their own policy YAML contracts. A successful
G1 or PiPlus 22-DoF GEM run is evidence for the shared architecture, but it is
not evidence that either new robot's observation or joint order is compatible.
