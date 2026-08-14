---
title: PiPlus-LSE 23-DoF BFM-Zero Sim2real
---

# PiPlus-LSE 23-DoF BFM-Zero Sim2real

This deployment uses the `PiPlus_S_12L8A0G2H1W_LSE_BFM` contract from the
`instinct_onboard` `bfm` branch. The policy, simulation, and motion stream use
the 23-joint BFM-Zero order. The ROS2 middleware uses its physical motor order;
`scripts/piplus/real_bridge.py` is the only component that converts between
them.

## Contract

- Robot: `piplus_lse_23dof`
- Policy: `checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml`
- Action dimension: 23
- Policy state/history dimensions: 52 / 300
- Backward encoder dimensions: `8x52`, `8x403`, and `8x1`
- Motion qpos: `[root_xyz, root_quat_wxyz, 23 joints]`, or 30 values per frame
- ROS2 state: `joint_states` and `/imu`
- ROS2 command: `control_command`
- Control services: `request_control` and `release_control`

The hardware motor order is left leg, right leg, left arm, right arm, head,
then waist. The policy-to-hardware index map is:

```text
[11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
 22, 16, 17, 18, 19, 12, 13, 14, 15, 20, 21]
```

## Preflight And Sim2sim

Run the static deployment preflight first:

```bash
uv run python scripts/piplus/bfmzero_23dof.py --preflight-only
```

Run a short integrated sim2sim smoke test before using the robot:

```bash
uv run python sim2real/sim_env/integrated_sim2sim.py \
  --robot piplus_lse_23dof \
  --policy-config checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml \
  --motion-path checkpoints/bfm-zero/piplus-lse-23dof/motions/lafan/motions/dance2_subject2_20260706_PiPlus_S_12L8A0G2H1W_LSE_260424.npz \
  --headless \
  --run-once \
  --max-runtime-s 15
```

Set `SIM2REAL_PIPLUS_LSE_MJCF` if the complete MJCF asset tree is not in the
default `Assets/ht_urdf` location.

## Real Robot

Start the official PiPlus ROS2 bringup and verify the two state topics and two
control services. Then use three terminals from the `sim2real/` directory.

Terminal 1, inspect the bridge without taking motor control:

```bash
uv run python scripts/piplus/real_bridge.py \
  --robot piplus_lse_23dof \
  --dryrun
```

After checking the 23-joint state and command topic, remove `--dryrun` to
request real position control:

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_lse_23dof
```

The bridge uses the source deployment's `1.0` joint protection ratio by
default. Use `--joint-pos-protect-ratio` only when the hardware team has
approved a different limit.

Terminal 2, start the BFM-Zero policy:

```bash
uv run python scripts/piplus/bfmzero_23dof.py
```

Terminal 3, publish a canonical 50 Hz motion stream:

```bash
uv run python sim2real/teleop/npz_pub.py \
  --robot piplus_lse_23dof \
  --motion-path checkpoints/bfm-zero/piplus-lse-23dof/motions/lafan/motions/dance2_subject2_20260706_PiPlus_S_12L8A0G2H1W_LSE_260424.npz \
  --initial-source motion \
  --root-body-name base_link \
  --publish-hz 50 \
  --bind 'tcp://*:28701'
```

In the policy terminal, press `i` for init, `o` for zero, and `]` for policy
mode. Enter policy mode only after init is stable. `Ctrl-C` in the bridge
requests `release_control` with damping mode.

Any live teleoperation source may replace `npz_pub.py` if it publishes the
same canonical joint/body motion JSON for `piplus_lse_23dof`. A G1 or PiPlus
22-DoF stream cannot be reused without retargeting because its joint and body
contracts differ.
