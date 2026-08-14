---
title: PiPlus-LSE 23-DoF BFM-Zero Sim2real
---

# PiPlus-LSE 23-DoF BFM-Zero Sim2real

本部署使用 `instinct_onboard` 的 `bfm` 分支中
`PiPlus_S_12L8A0G2H1W_LSE_BFM` 的接口契约。policy、仿真和 motion stream
使用 BFM-Zero 的 23 关节顺序；ROS2 middleware 使用真实电机顺序，二者只在
`scripts/piplus/real_bridge.py` 的进程边界进行转换。

## 接口契约

- Robot：`piplus_lse_23dof`
- Policy：`checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml`
- Action 维度：23
- Policy state/history 维度：52 / 300
- Backward encoder 维度：`8x52`、`8x403`、`8x1`
- Motion qpos：`[root_xyz, root_quat_wxyz, 23 joints]`，每帧共 30 个值
- ROS2 state：`joint_states` 和 `/imu`
- ROS2 command：`control_command`
- 控制服务：`request_control` 和 `release_control`

真实电机顺序依次是左腿、右腿、左臂、右臂、头和腰。policy 到 hardware 的
索引映射为：

```text
[11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
 22, 16, 17, 18, 19, 12, 13, 14, 15, 20, 21]
```

## Preflight 和 Sim2sim

首先运行静态部署检查：

```bash
uv run python scripts/piplus/bfmzero_23dof.py --preflight-only
```

上真机前运行一段 integrated sim2sim smoke test：

```bash
uv run python sim2real/sim_env/integrated_sim2sim.py \
  --robot piplus_lse_23dof \
  --policy-config checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml \
  --motion-path checkpoints/bfm-zero/piplus-lse-23dof/motions/lafan/motions/dance2_subject2_20260706_PiPlus_S_12L8A0G2H1W_LSE_260424.npz \
  --headless \
  --run-once \
  --max-runtime-s 15
```

如果完整 MJCF 资产不在默认 `Assets/ht_urdf` 目录，请设置
`SIM2REAL_PIPLUS_LSE_MJCF`。

## 真机部署

先启动官方 PiPlus ROS2 bringup，并确认两个状态 topic 和两个控制 service
存在。然后从 `sim2real/` 目录打开三个终端。

终端 1，先在不获取电机控制权的情况下检查 bridge：

```bash
uv run python scripts/piplus/real_bridge.py \
  --robot piplus_lse_23dof \
  --dryrun
```

确认 23 关节 state 和 command topic 正确后，去掉 `--dryrun` 请求真实 position
control：

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_lse_23dof
```

bridge 默认使用与上游部署一致的 `1.0` joint protection ratio。只有硬件团队确认
新的限位后，才应通过 `--joint-pos-protect-ratio` 修改。

终端 2，启动 BFM-Zero policy：

```bash
uv run python scripts/piplus/bfmzero_23dof.py
```

终端 3，发布 canonical 50 Hz motion stream：

```bash
uv run python sim2real/teleop/npz_pub.py \
  --robot piplus_lse_23dof \
  --motion-path checkpoints/bfm-zero/piplus-lse-23dof/motions/lafan/motions/dance2_subject2_20260706_PiPlus_S_12L8A0G2H1W_LSE_260424.npz \
  --initial-source motion \
  --root-body-name base_link \
  --publish-hz 50 \
  --bind 'tcp://*:28701'
```

在 policy 终端按 `i` 进入 init、按 `o` 回到 zero、按 `]` 进入 policy。必须在
init 稳定后才能进入 policy。bridge 终端按 `Ctrl-C` 会调用
`release_control` 并请求 damping mode。

任何实时遥操作源只要为 `piplus_lse_23dof` 发布相同 canonical joint/body
motion JSON，都可以替换 `npz_pub.py`。G1 或 PiPlus 22-DoF 的 stream 关节和
body 契约不同，未经 retarget 不能直接复用。
