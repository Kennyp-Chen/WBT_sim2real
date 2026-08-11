---
title: 全身遥操作入门指南
sidebar_position: 1
---

# 全身遥操作入门指南

在 MuJoCo 中尝试全身动作跟踪不需要 VR 设备。但如果目标是让机器人实时模仿
操作者本人，仓库目前开箱即用的方案是 PICO / XR、腿部 trackers 和
XRoboToolkit。键盘只能控制运行状态，不能采集人体全身姿态。

## 可用输入方式

| 方式 | 是否需要 VR | 用途 |
| --- | ---: | --- |
| 离线 NPZ 跟踪 | 否 | 最简单的 G1 全身动作跟踪方式。 |
| NPZ 实时流回放 | 否 | 模拟实时遥操作链路，适合调试延迟、暂停和断流。 |
| PICO / XR 实时全身遥操作 | 是 | 当前完整实现的真人实时全身捕捉方案。 |
| 外部相机、动捕或 SMPL 接入 | 视设备而定 | 上游需要实现仓库定义的 ZMQ 数据协议。 |

tracking runtime 支持三种 motion backend：

- `npz`：直接读取动作文件。
- `zmq`：接收已经 retarget 到 G1 的实时全身动作。
- `smpl_zmq`：为兼容的 SONIC policy 接收 SMPL reference 和 G1 手腕参考。

## 可选检查点

仓库文档列出了 10 个 policy family，统一基准评测使用了 13 个具体 policy
variant：

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

ScaleBFM、SONIC 和 HEFT 等 family 包含多个 variant。第一次运行建议使用：

```text
checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

这是仓库教程的默认模型，使用标准 G1 motion 接口。文档记录的 motion
lookahead 要求如下：

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

## 第一次启动：无 VR

### 1. 安装根项目环境

在仓库根目录运行：

```bash
uv sync --extra inference-cpu
```

### 2. 下载运行资产

下载共享的 [sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)，
放到仓库根目录：

```text
sim2real/
├── checkpoints/
│   └── mimic-lite/
│       └── 32x8192-huge/
│           ├── policy.yaml
│           └── policy.onnx
└── third_party/
```

完整目录结构见 [下载运行资产](/reference/artifacts)。检查初始 policy 文件：

```bash
test -f checkpoints/mimic-lite/32x8192-huge/policy.yaml
test -f checkpoints/mimic-lite/32x8192-huge/policy.onnx
```

### 3. 测试 ONNX 推理

```bash
uv run scripts/test_policy_inference.py \
  --policy_config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --inference_backend onnx-cpu
```

### 4. 一条命令运行 MuJoCo 全身跟踪

```bash
uv run sim2real/sim_env/integrated_sim2sim.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-path hf://elijahgalahad/any4hdmi-g1-lafan/motions/walk1_subject1.npz
```

runner 会把机器人初始化到动作第一帧，等待 5 秒，跟踪完整动作并保持最后姿态。
打开终端打印的 mjviser URL 即可观看。如果无法访问 Hugging Face，把
`--motion-path` 换成本地 any4hdmi 兼容的 `.npz` 文件。

## 无 VR 测试实时链路

这种方式把 NPZ 动作通过 PICO 遥操作使用的标准 G1 ZMQ motion stream 播放。
打开三个终端。

终端 1，启动 MuJoCo：

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
```

终端 2，发布动作：

```bash
uv run sim2real/teleop/npz_pub.py \
  --motion-path /path/to/motion.npz
```

终端 3，把 policy 接到实时流：

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-backend zmq
```

publisher 按键：

| 按键 | 操作 |
| --- | --- |
| `]` | 回到动作第 0 帧并暂停。 |
| `Space` | 播放或暂停。 |
| `x` | 回到默认站立姿态。 |

## PICO 实时全身遥操作

这条路径需要 PICO、腿部 trackers、whole-body tracking 和 XRoboToolkit。

### 1. 安装遥操作环境

```bash
uv sync --project venv/pico
```

按照 [Teleop Project (x86_64 PC)](/getting-started/teleop-x86-64) 安装
XRoboToolkit PC Service，然后准备 Python binding 源码：

```bash
mkdir -p external
git clone https://github.com/YanjieZe/XRoboToolkit-PC-Service-Pybind.git \
  external/XRoboToolkit-PC-Service-Pybind
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git \
  external/XRoboToolkit-PC-Service
bash scripts/setup/setup_xrobot_pybind.sh
```

### 2. 打开全身追踪

1. 戴好头显和腿部 trackers。
2. 完成 whole-body calibration。
3. 启动 XRoboToolkit 并连接头显。
4. 打开 whole-body streaming。

### 3. 启动 retarget publisher

```bash
uv run --project venv/pico \
  sim2real/teleop/pico_retarget_pub.py
```

先打开 publisher 打印的 mjviser URL，确认 retarget 后的 G1 动作与操作者一致。

### 4. 启动 MuJoCo

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
```

### 5. 启动 tracking policy

```bash
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy-config checkpoints/mimic-lite/32x8192-huge/policy.yaml \
  --motion-backend zmq \
  --controller pico
```

PICO 控制：

| 输入 | 操作 |
| --- | --- |
| `A` | 进入初始化姿态。 |
| `A` + `B` | 进入 policy mode。 |
| `X` | 恢复 motion flow。 |

如果 publisher 和 policy 不在同一台电脑，在 policy 命令中增加：

```bash
--motion-zmq-connect tcp://<publisher-ip>:28701
```

## 推荐启动顺序

1. 运行 integrated 离线跟踪，验证环境、运行资产和 policy。
2. 运行 NPZ 三进程实时流，验证 realtime interface。
3. 前两项正常后再接入 PICO。

这个顺序可以分别定位模型、仿真、数据流和人体输入问题。

