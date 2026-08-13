---
title: PiPlus BFM-Zero sim2real
slug: /tutorials/piplus-bfmzero-sim2real
---

# PiPlus BFM-Zero 遥操作 sim2real

本文记录 `instinct_onboard` 的 `bfm` 分支与本仓库 `sim2real` 的接口契约。
本地上游副本位于工作区的 `instinct_onboard/`，当前检出分支为 `bfm`，提交为
`b9f72860e4573c54e8ac33a562a777a8f88e418c`。

## 已确认的上游契约

- PiPlus ROS2 输入：`joint_states` (`sensor_msgs/JointState`) 和 `/imu`
  (`sensor_msgs/Imu`)。
- PiPlus ROS2 输出：`control_command` (`hightorque_msgs/MotorControlCommand`)。
- 电机控制服务：`request_control`、`release_control`。
- 上游控制循环为 50 Hz，IMU 四元数在 ROS 中是 `xyzw`；sim2real 的
  `LowStateMessage` 使用 `wxyz`。
- bridge 对反馈关节执行与上游一致的 1.5 倍关节范围保护；超限时只保留状态
  发布并禁止发送命令，避免把异常反馈继续放大到电机侧。
- policy 的 22 个关节顺序是肩、头、腿的 canonical order；ROS 的真实数组顺序
  是左脚到右脚、左臂、右臂、头。映射来源是
  `instinct_onboard/instinct_onboard/robot_cfgs.py` 的
  `PiPlus_S_12L8A0G2H0W.joint_map`，符号目前全部为 `+1`。
- BFM-Zero policy 输入为 `state[50]`、`last_action[22]`、`history_actor[288]`，
  另有 8 帧 `encoder_state[50]`、`privileged_state[388]` 和窗口权重；输出为
  22 维 action。动作历史顺序固定为 `actions`、`base_ang_vel`、`dof_pos`、
  `dof_vel`、`projected_gravity`。

## 运行拓扑

```text
ROS2 JointState + Imu
        |
        v
scripts/piplus/real_bridge.py
        |  LowStateMessage :5590
        v
tracking.py --robot-io zmq --robot piplus_h0w
        |  LowCmdMessage :5591
        +---------------------> real_bridge -> MotorControlCommand

npz_pub.py 或实时 motion publisher --:28701--> tracking.py
```

`real_bridge.py` 只在边界处做硬件重排。策略、观测、motion buffer 和 ONNX
始终使用 canonical policy order，因此仿真和真实部署共用同一份 checkpoint。

## 上机步骤

先在 PiPlus ROS2 环境启动官方 bringup，并确认 `joint_states`、`/imu`、
`request_control`、`release_control` 存在。然后在 `sim2real/` 目录执行：

```bash
export SIM2REAL_PIPLUS_MJCF=/absolute/path/to/PiPlus_S_12L8A0G2H0W/xml/PiPlus_S_12L8A0G2H0W_with_armature.xml
export PIPLUS_POLICY=checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml
export PIPLUS_MOTION=/absolute/path/to/any4hdmi_full/motions/dance2_subject2.npz
```

先做不拿电机控制权的 ROS dry-run，确认状态和命令链路：

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_h0w --dryrun
```

再启动真实 bridge：

```bash
uv run python scripts/piplus/real_bridge.py --robot piplus_h0w
```

启动策略：

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

启动动作 publisher；它可以替换成任何输出 sim2real canonical motion JSON 的
实时遥操作源：

```bash
uv run python sim2real/teleop/npz_pub.py \
  --robot piplus_h0w \
  --motion-path "$PIPLUS_MOTION" \
  --initial-source motion \
  --root-body-name base_link \
  --publish-hz 50 \
  --bind 'tcp://*:28701'
```

策略终端按 `i` 进入 init、按 `o` 回到 zero、确认机器人稳定后按 `]` 进入
policy。紧急停止时在 bridge 终端按 `Ctrl-C`，bridge 会调用 `release_control`
并请求 damping mode。

## 验证顺序

1. 用 `scripts/piplus/real_bridge.py --dryrun` 检查 ROS topic，不接真实控制权。
2. 检查 state 的四元数为 `wxyz`，关节数组长度为 22，且 head 两个关节位于
   ROS 数组最后。
3. 用 `tests/test_piplus_bridge.py` 验证 22-DoF 映射和往返一致性。
4. 先用 `sim2real/sim_env/base_sim.py` 跑同一 policy/motion 的 ZMQ sim2sim，
   再上真实机器人。

当前 `sim2real/teleop/pico_retarget_pub.py` 的 GMR 入口仍固定为 G1；因此本文
的实时源使用 canonical motion publisher 接口，不能把 G1 的 PICO retargeter
直接当作 PiPlus retargeter。接入 PiPlus 人体重定向器时，只需让它发布相同的
`body_names`、`joint_names`、`body_pos_w`、`body_quat_w` JSON，不需要改 policy
或 ROS bridge。
