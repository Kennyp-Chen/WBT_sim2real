---
name: integrate-bfmzero-robot
description: Add a new robot BFM-Zero policy to the sim2real repository. Use when porting a BFM-Zero checkpoint, robot MJCF/configuration, observation implementation, ONNX export, policy.yaml, or any4hdmi motion assets, especially when using the local PiPlus 22DoF implementation as a reference.
---

# Integrate A BFM-Zero Robot

Use this workflow when a BFM-Zero model is being added to `sim2real`. The policy is a contract between training, ONNX export, observations, robot configuration, and motion data. Do not copy PiPlus dimensions or body names until the new training config proves they match.

## 1. Inventory the source

Separate the source into three categories before editing the repository:

- **Runtime assets:** `policy.onnx`, auxiliary ONNX graphs if the policy needs them, `policy.yaml`, robot MJCF plus meshes/textures, and any4hdmi motion data.
- **Training/provenance assets:** source `config.yaml`/`config.json`, exporter code, source `.pkl`, and training metadata. Keep these for reproducibility, but they are not a replacement for deployable ONNX.
- **Large training weights:** files such as `model.safetensors` are not needed by sim2real inference once ONNX has been exported.

Read [the PiPlus case study](references/piplus-case-study.md) before changing a BFM-Zero observation or exporter.

## 2. Freeze the model contract

Write down, from the training config and exported graph:

1. Ordered policy joint names and action dimension.
2. Simulation body order and source-motion body order. These can differ because fixed bodies may be collapsed during training.
3. Dimensions and formulas for state, last action, actor history, future encoder state, privileged future state, latent `z`, and action output.
4. Quaternion convention (`wxyz` in this runtime; convert source `xyzw` explicitly when needed).
5. Motion FPS, `future_steps`, `seq_length`, `motion_t_offset`, and whether final-frame clamping is required.
6. Action observation scale/clipping and actuator action scale. They are separate quantities.
7. ONNX input/output names, shapes, dtypes, and any recurrent/carry inputs.

If any item is unknown, stop at contract discovery and inspect the training/export code. A graph that merely runs with random inputs is not evidence that the observation contract is correct.

## 3. Add robot configuration and assets

Create `sim2real/config/robots/<robot>.py` using `sim2real/config/robots/piplus.py` as the structural template. Provide:

- A portable MJCF resolver with an environment override, for example `SIM2REAL_<ROBOT>_MJCF`, followed by repository-local asset paths.
- Canonical actuated `joint_names` in the exact order used by training and ONNX.
- Full MuJoCo `body_names`, position limits, velocity/effort limits, safe `kp`/`kd`, armature, friction, default joint pose, root qpos convention, publish rate, and viewer/elastic-band body names.
- MJCF, mesh, texture, and material files with all relative references resolvable on the target machine. Check `nq == 7 + len(joint_names)` for a floating-base model and verify actuator names match the joint names.

Register the config in `sim2real/config/robots/__init__.py` and export it in `__all__`. Check it with:

```bash
uv run python - <<'PY'
from sim2real.config.robots import get_robot_cfg
cfg = get_robot_cfg("<robot>")
print(cfg.name, len(cfg.joint_names), cfg.qpos_size, cfg.resolve_mjcf_path())
PY
```

## 4. Implement observations

Add `sim2real/rl_policy/observations/bfm_zero_<robot>.py` when the new robot's joint/body order, dimensions, or virtual-body construction differ from PiPlus. Implement the same semantic terms that the training policy consumes, with explicit shape assertions. Reuse shared quaternion/math helpers where possible; do not silently reorder or pad data.

Every class must be registered under the namespace used by the YAML, for example `namespace="bfm_zero"`, and the module must be imported by `sim2real/rl_policy/observations/__init__.py`. Otherwise `_target_: bfm_zero.<term>` cannot resolve.

For a policy with a different action dimension, change all dependent pieces together: joint selection, default pose lookup, last-action scaling, history layout, future state layout, privileged body layout, exporter input contract, and policy YAML. If the semantic contract is identical, a shared parameterized implementation is acceptable, but prove the equality first.

## 5. Export and package ONNX

Use the source project's exporter to create a complete inference graph. If actor and backward encoder are separate, adapt `scripts/merge_bfm_zero_piplus_onnx.py` rather than hand-editing a graph. Update all hard-coded dimensions, input names, latent normalization/weighting, output name, and metadata for the new contract.

The deploy directory should contain at least:

```text
<checkpoint>/
  policy.yaml
  policy.onnx
  [exported auxiliary ONNX graphs, if the runtime uses them]
```

Keep source configs and training checkpoints alongside provenance, not as substitutes for these files. Make `model_path`, `mjcf_path`, and motion paths portable or override them at runtime; never leave a machine-specific absolute path in a shared policy package.

## 6. Convert motion data

Create `scripts/convert_<robot>_<source>_to_any4hdmi.py` from `scripts/convert_piplus_lafan_pkl_to_any4hdmi.py`. The converter must:

- validate the source joint set and reorder into canonical policy order;
- validate root position/rotation and convert `xyzw` to runtime `wxyz` when applicable;
- resample with linear joint/root-position interpolation and quaternion SLERP;
- save qpos with shape `[T, 7 + action_dim]`;
- write `manifest.json` containing qpos names, timestep/FPS, MJCF reference, source format, and conversion metadata.

The motion's joint names, body names, MJCF kinematics, root body, and quaternion order must agree with the observation implementation. For an `npz` runtime, pass the any4hdmi dataset root or a single motion path; `integrated_sim2sim.py` requires `--motion-path`.

## 7. Write policy.yaml

Start from the training/export config, then make runtime fields explicit:

- `joint_names_simulation`, `policy_joint_names`, `body_names_simulation`;
- `joint_kp`, `joint_kd`, `default_joint_pos`, `joint_effort_limit`, `action_scale`;
- all observation `_target_` names and their dimensions/sequence parameters;
- `motion.motion_backend: npz`, `future_steps`, `body_names`, `joint_names`, `root_body_name`, and an overridable `mjcf_path`.

The YAML joint order must match the robot config and the motion manifest. The policy's actuator `action_scale` must not be confused with the observation's last-action scale.

## 8. Validate in increasing scope

Run the bundled static check first:

```bash
uv run python .agents/skills/integrate-bfmzero-robot/scripts/validate_bfmzero_contract.py \
  --policy-config checkpoints/<robot>/policy.yaml \
  --robot <robot> \
  --motion-path path/to/any4hdmi_or_motion.npz
```

Then run:

```bash
uv run python -m py_compile sim2real/config/robots/<robot>.py sim2real/rl_policy/observations/bfm_zero_<robot>.py
uv run python scripts/test_policy_inference.py \
  --policy-config checkpoints/<robot>/policy.yaml \
  --single --inference-backend onnx-cpu
uv run python sim2real/sim_env/integrated_sim2sim.py \
  --robot <robot> \
  --policy-config checkpoints/<robot>/policy.yaml \
  --motion-path path/to/motion.npz \
  --headless --run-once --max-runtime-s 15
```

For a visual smoke test, add `--video-output /tmp/<robot>.mp4 --video-fps 30 --video-width 1280 --video-height 720`. Use one shell argument per option; a line break without a trailing `\` turns `policy.yaml` or a motion filename into a separate shell command.

Check the following before calling the port complete:

- `Observation.resolve()` finds every YAML target.
- ONNX input/output shapes agree with the documented contract and action dimension.
- MuJoCo loads the MJCF and all referenced assets; body/joint/actuator names resolve.
- Motion qpos dimension and manifest order match the robot config.
- The sim remains stable for a full clip, including the final-frame clamp path.
- The same portable asset layout works after copying to the deployment host.

## Common failure modes

- **Unknown robot:** config was not added to the robot registry.
- **Observation target not found:** new observation module was not imported.
- **Shape mismatch:** a PiPlus constant or exporter dimension was copied unchanged.
- **Missing body:** training body order and MuJoCo body order were conflated; inspect fixed-body collapse and virtual hand/head extensions.
- **Bad posture or explosive torque:** default pose, joint order, action scale, or `kp`/`kd` does not match training.
- **Motion indexing error:** `future_steps`, `seq_length`, `motion_t_offset`, target FPS, or final-frame clamping disagree.
- **Works only on the author machine:** YAML or manifest contains an absolute MJCF/motion path.
- **Network access during startup:** on robot hosts export `HF_HUB_OFFLINE=1` and `HF_HUB_DISABLE_TELEMETRY=1` before running inference.
