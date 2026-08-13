# PiPlus 22DoF Reference

This is the local BFM-Zero PiPlus implementation audited in `sim2real`. Treat it as a traceable example, not as a universal BFM-Zero schema.

## Runtime files

The deployable checkpoint is under:

```text
checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/
  policy.yaml
  policy.onnx
  exported/FBcprAuxModel.onnx
  exported/FBcprAuxModel_z_encoder.onnx
```

`config.yaml`, `config.json`, and `tracking_inference/*.pkl` are useful for provenance/debugging. The multi-gigabyte `checkpoint/model/model.safetensors` is a training artifact and is not required by the sim2real ONNX runtime.

## Robot config

`sim2real/config/robots/piplus.py` defines `PIPLUS_H0W_CFG` with 22 actuated joints, full MuJoCo body order, limits, effort, armature, friction, safe gains, default qpos, and an environment override `SIM2REAL_PIPLUS_MJCF`.

The 22 policy joints are:

```text
r_shoulder_pitch_joint r_shoulder_roll_joint r_upper_arm_joint r_elbow_joint
l_shoulder_pitch_joint l_shoulder_roll_joint l_upper_arm_joint l_elbow_joint
head_yaw_joint head_pitch_joint
r_hip_pitch_joint r_hip_roll_joint r_thigh_joint r_calf_joint r_ankle_pitch_joint r_ankle_roll_joint
l_hip_pitch_joint l_hip_roll_joint l_thigh_joint l_calf_joint l_ankle_pitch_joint l_ankle_roll_joint
```

The runtime `RobotCfg.body_names` includes `torso_link`, wrists, and `camera_link`. The BFM motion/observation source order is different and starts with `base_link`; do not replace one list with the other.

## Observation contract

`sim2real/rl_policy/observations/bfm_zero_piplus.py` registers terms under `bfm_zero`:

| Term | Dimension | Meaning |
| --- | ---: | --- |
| `bfm_zero_piplus_state` | 50 | 22 joint position deltas + 22 joint velocities + projected gravity (3) + scaled base angular velocity (3) |
| `bfm_zero_piplus_last_action` | 22 | previous action scaled/clipped by 32 |
| `bfm_zero_piplus_history_actor` | 288 | four frames of action, base angular velocity, joint position, joint velocity, projected gravity |
| `bfm_zero_piplus_encoder_state_future` | `[8, 50]` | future state window |
| `bfm_zero_piplus_privileged_state_future` | `[8, 388]` | future privileged body state |
| `bfm_zero_piplus_future_window_weight` | `[8, 1]` | final-frame clamp weights |

The source body list has 23 bodies. The implementation appends virtual `r_hand_link`, `l_hand_link`, and `head_link`, making 26 bodies. The privileged dimension is:

```text
1 root height + (26*3 - 3) local positions + 26*6 tangent/normal rotations
+ 26*3 local linear velocities + 26*3 local angular velocities = 388
```

The motion window uses `seq_length=8`, `target_fps=50`, `motion_t_offset=-1`, and `future_steps=[-2,-1,0,1,2,3,4,5,6,7]`.

## ONNX merge contract

`scripts/merge_bfm_zero_piplus_onnx.py` expects the source actor input to be 616 and the backward encoder input to be 438:

```text
actor:   50 + 22 + 288 + 256 = 616
encoder: 50 + 388 = 438
output: 22 actions
```

The merge graph exposes `actor_state`, `last_action`, `history_actor`, `encoder_state`, `privileged_state`, and `encoder_window_weight`. It averages eight encoder `z` outputs with the window weights, normalizes the latent, multiplies by 16, and feeds it to the actor. A new robot must update these dimensions and any source graph names instead of assuming this exact graph is reusable.

## Motion conversion

`scripts/convert_piplus_lafan_pkl_to_any4hdmi.py` validates the 22-joint set, reorders source joints, converts root quaternion `xyzw -> wxyz`, resamples with SLERP, saves `[T, 29]` qpos, and writes an any4hdmi manifest. For a new robot, copy the structure but replace the joint schema, root convention, source FPS assumptions, MJCF, and metadata.
