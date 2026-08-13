---
title: GEM 集成计划
slug: /tutorials/gem-integration-plan
---

# GEM 集成计划

本文档是把 GEM 视频、语言、音频和音乐动作接入本仓库基座策略的长期实施计划和进度账本。每次推进后都要更新状态表、验收证据、阻塞项和恢复入口。

技术背景见 [GEM 多模态动作前端](/reference/gem-multimodal-motion)。

## 项目目标

实现一个带时间戳的统一动作源接口，依次支持：

1. 录制视频；
2. 实时摄像头；
3. 交互式语言 prompt；
4. 实时或录制音乐/音频；

并按照实现风险依次驱动：

1. G1 SONIC SMPL；
2. PiPlus 22DoF BFM-Zero；
3. sim2sim 验证完成后的机器人真机。

## 进度账本

最后更新：2026-08-12。

| ID | 状态 | 交付物 | 证据 / 当前结果 |
|---|---|---|---|
| PRE-001 | 已完成 | SONIC release 和 low-latency G1/SMPL ONNX 集成 | `checkpoints/sonic/{release,low_latency}` |
| PRE-002 | 已完成 | SONIC SMPL ZMQ contract | `sim2real/teleop/smpl_stream.py`、`SmplRealtimeMotionBuffer` |
| PRE-003 | 已完成 | PICO/XRobot SMPL publisher 和 GMR 腕部参考 | `pico_retarget_pub.py --publish-smpl` |
| PRE-004 | 已完成 | PiPlus 22DoF BFM-Zero merged ONNX 和 observations | commit `7a3b2c6` |
| PRE-005 | 已完成 | PiPlus NPZ/ZMQ 播放及同步视频录制 | 两个录制脚本 |
| GEM-000 | 已完成 | GEM 论文、项目及 runtime 调研 | GEM 报告；本地 clone 位于 `/home/sunteng/Projects/WBC_Telep/GENMO` |
| GEM-001 | 进行中 | GEM `smpl_params.pt` 到 SONIC SMPL ZMQ publisher | 已开始 adapter；还需真实 GEM artifact 和 sim2sim |
| GEM-002 | 进行中 | 录制视频 GEM inference 环境和 benchmark | GENMO 位于 `/home/sunteng/Projects/WBC_Telep/GENMO`；9 个 GEM/GVHMR checkpoint 文件均已下载并校验；只剩授权的 SMPL-X body model |
| GEM-003 | 未开始 | GEM SMPL 到完整 G1 腕部重定向 | 尚无 GEM-SMPL-to-G1 GMR adapter |
| GEM-004 | 未开始 | 录制视频 SONIC sim2sim 和对比视频 | 依赖 GEM-001 至 GEM-003 |
| GEM-005 | 未开始 | 实时 webcam GEM-to-SONIC | 依赖录制视频验证 |
| GEM-006 | 未开始 | 交互式 text-to-motion stream | 依赖稳定 SMPL stream 和 transition manager |
| GEM-007 | 未开始 | music/audio-to-motion stream | 依赖特征提取和 transition manager |
| GEM-008 | 未开始 | SMPL 到 PiPlus 22DoF retargeter | 需要 PiPlus mapping 和 IK 验证 |
| GEM-009 | 未开始 | PiPlus BFM-Zero 多模态 sim2sim | 依赖 GEM-008 |
| GEM-010 | 未开始 | 真机安全门和有限真机试验 | 依赖稳定 sim2sim 指标 |

状态只使用“未开始、进行中、阻塞、已完成”。只有验收条件和证据都存在时才能标记已完成。

## 当前项目状态

### SONIC

- Release SMPL policy 位于 `checkpoints/sonic/release/smpl/policy.yaml`。
- ONNX contract 为 `smpl_input[840]`、`proprioception[930]`，输出 `action[29]`、`token[64]`。
- `motion_backend: smpl_zmq` 使用 28702 端口。
- SMPL stream 已支持 future-window buffer、root yaw 连续、时间戳对齐和默认静止姿态。
- Encoder 使用规范化 24 关节 root-local SMPL 位置、相对 root orientation 和六个 G1 腕部关节角。

### PiPlus BFM-Zero

- PiPlus 22DoF actor 和 backward encoder 已合并为一个 policy ONNX。
- PiPlus 专用 observation 和 deploy YAML 已实现。
- LAFAN 训练 pickle 可转换为 any4hdmi NPZ。
- 已支持离线、ZMQ 和同步左右对比视频。
- 当前缺少 SMPL 到 PiPlus reference 的重定向。

### GEM

- 官方仓库已 clone 到 `/home/sunteng/Projects/WBC_Telep/GENMO`。
- 工作站有 RTX 5060 Ti 16 GB 显存，磁盘空间足够。
- GENMO 源码已克隆到 `/home/sunteng/Projects/WBC_Telep/GENMO`。
- `gem_smpl.ckpt` 和官方 webcam 六个 ONNX 文件已下载到
  `inputs/pretrained/` 和 `inputs/onnx/`；7 个 SHA-256 均与 NVIDIA GEM-X Hugging Face 元数据一致。
- HMR2 与 ViTPose 原始 checkpoint 已从公开的 `camenduru/GVHMR` 镜像下载到
  `inputs/checkpoints/{hmr2,vitpose}/`，字节数和 SHA-256 与该镜像元数据一致。
- `SMPLX_NEUTRAL.npz` 需要接受 SMPL-X 许可后获取；HMR2/ViTPose 原始 PyTorch
  checkpoint 只由 GVHMR Google Drive 提供，目前该主机无法访问该地址。
- 当前 workspace 中还没有 GEM 生成的真实 `smpl_params.pt`。

## 目标架构

```text
                         +------------------+
录制/实时视频 ---------->| GEM 姿态估计     |----+
                         +------------------+    |
                                                  v
文本 prompt ------------>| GEM 动作生成     |  统一 SMPL stream
音乐/音频 -------------->| + chunk manager  |    |          |
                         +------------------+    |          |
                                                  |          +-> G1 retarget -> SONIC
                                                  |
                                                  +-> PiPlus retarget -> BFM-Zero
```

所有来源最终都要进入一个公共动作记录：明确 source FPS、frame index、source timestamp、confidence/validity 和 segment boundary。策略相关的机器人重定向放在公共表示之后。

## 阶段一：录制视频到 SONIC

### GEM-001：通用 GEM SMPL Publisher

目标：读取官方 GEM `smpl_params.pt`，把 global body parameters 转换成现有 SONIC SMPL contract，并按 source FPS 在 28702 端口播放。

实现要求：

- 不依赖 import GEM 仓库即可读取 tensor；
- 强制检查 `body_params_global.body_pose` 和 `global_orient`；
- 验证 `[T,21,3]` 和 `[T,3]` axis-angle；
- 复用 `human_joints_info.pkl` FK 构造 SONIC 规范化 `smpl_joint_pos_root`；
- 将 GEM Y-up root orientation 转换为 SONIC root quaternion；
- 发布单调 frame index、source/publish timestamp 和 `motion_first_frame`；
- 支持可配置 future window；
- 支持从 NPZ/PT 加载 G1 joint reference；
- 没有 G1 reference 时明确警告使用 default pose，腕部跟踪为降级模式；
- 支持 `--dry-run` 检查文件和 schema，不启动 ZMQ。

验收条件：

- 单元测试覆盖正确 tensor、错误 shape、默认腕部 fallback、外部 G1 joint reference 和 payload shape；
- `uv run python -m py_compile` 通过；
- 合成 GEM 文件可通过 `--dry-run` 并打印文档形状；
- subscriber 收到单调、形状正确、符合指定 FPS 的消息。

### GEM-002：GEM 环境和录制视频推理

目标：用一段短录制视频生成首个真实 `smpl_params.pt`。

任务：

1. 在 sim2real root 之外创建 GEM Python 3.10 CUDA 环境。
2. 获取 `SMPLX_NEUTRAL.npz` 并放到 GEM input 路径。
3. 下载或提供 GEM-SMPL checkpoint、HMR2 和 ViTPose 文件。
4. 首先用 `--no_render` 运行 video-only inference。
5. 检查 shape、NaN、orientation 连续性、时长、显存和耗时。
6. 参数输出正确后再运行渲染对比。

验收条件：记录命令、源视频、准确 checkpoint、GEM commit、GPU、耗时、峰值显存和输出路径；`smpl_params.pt` 全程有限值；渲染结果持续跟踪同一人物。

当前缺少的外部文件：根据 SMPL-X 许可证从官方获取的 `SMPLX_NEUTRAL.npz`。

### GEM-003：完整 G1 腕部重定向

目标：用同一 GEM 动作生成 robot joint reference，替换 default-pose 腕部 fallback。

任务：确认 GMR 是否支持 SMPL 输入，否则实现最小 SMPL-to-G1 IK；严格使用 SONIC YAML 的 IsaacLab joint order；验证六个 wrist roll/pitch/yaw；和已知 SONIC 官方 SMPL+G1 pair 对比。

验收条件：无 fallback 警告；29 个 G1 reference 有限且在限位内；腕部方向与人体动作一致；SONIC input 为 840 维并且手臂旋转时 wrist slice 会变化。

### GEM-004：录制视频 SONIC Sim2sim

三个终端：

```bash
uv run python sim2real/teleop/gem_smpl_pub.py \
  --gem-params /absolute/path/to/smpl_params.pt
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

验收条件：SMPL buffer 有足够 future window 后启动 policy；reference/policy 时间线一致；无 NaN、shape error 或 root heading 跳变；保存 source/GEM/robot 对比视频；记录跌倒状态和 root tracking error。

## 阶段二：实时视频

### GEM-005：Webcam Stream

分别 benchmark 带/不带 HMR2 的官方 ONNX；让 `demo_webcam.py` 输出结构化逐帧结果；保留 camera capture timestamp；复用 GEM-001 的 ZMQ schema；增加 confidence gating、人物锁定、丢失保持和受控恢复；分别测量 camera-to-policy 与 camera-to-simulation 延迟。

验收条件：吞吐达到选定输入 FPS 或明确报告降采样；时间戳不会让策略看起来早于源视频；跟踪丢失时稳定 hold；保存 60 秒摄像头对比视频和延迟报告。

## 阶段三：语言和音乐

### GEM-006：交互式文本

实现固定长度生成 chunk、有限队列、取消、片段 overlap 和回到站立的 prompt 服务。先做手动文本，不在其稳定前加入语音识别。

验收条件：新 prompt 不会重排已发布动作；policy 不会遇到空 future window；片段边界不超过姿态/角速度限制；记录 prompt、seed、生成耗时和 artifact。

### GEM-007：音乐和音频

先复现官方离线 audio/music 条件样本，再增加麦克风/文件采集及与训练输入一致的特征提取。片段连接使用 beat-aware overlap，不能任意拼接独立 clip。

验收条件：保存音频可复现离线动作；实时 audio buffer 延迟有界且时间戳不倒退；保存 60 秒 music-to-motion SONIC sim2sim 视频。

## 阶段四：PiPlus BFM-Zero

### GEM-008：SMPL 到 PiPlus 重定向

追踪 PiPlus 训练动作表示，实现可复用 retargeter，输出 backward encoder 使用的精确 22DoF 顺序和 root/body frame。策略推理前先通过 PiPlus MJCF 验证 qpos。

验收条件：qpos 为配置 MuJoCo 顺序的 `[T,29]`；所有关节名与 YAML/MJCF 一致；FK body position/quaternion 有限且连续；已知动作与现有 PiPlus LAFAN 数据在数值或视觉上相符。

### GEM-009：PiPlus 多模态 Sim2sim

通过现有 normal ZMQ BFM-Zero 链路输入重定向动作，复用同步录制脚本，把源视频、重定向 reference 和 policy simulation 放到同一时间线。

## 阶段五：真机安全门

### GEM-010：受控真机试验

录制和实时 sim2sim 通过前不进入真机。增加输入 validity、速度、高度、关节限位、跌倒、通信超时和操作员停止门。先固定脚做上半身动作，再允许 locomotion。

## 实验记录

每次有意义的运行都追加一行，失败结果也不覆盖。

| 日期 | ID | 来源 | 命令 / 配置 | 输出 | 结果 / 指标 |
|---|---|---|---|---|---|
| 2026-08-12 | GEM-000 | NVlabs/GENMO `16bebf4` | 仓库检查 | GEM 报告 | 已确认 video/text 官方 demo、audio/music 模型支持和 webcam ONNX |
| 2026-08-12 | PRE-004 | PiPlus BFM-Zero | commit `7a3b2c6` | PiPlus policy/video workflow | GEM 工作开始前已集成并 push |
| 2026-08-12 | GEM-002 | NVIDIA GEM-X HF 镜像 + `camenduru/GVHMR` + GENMO `16bebf4` | 可续传 `wget -c` | `/home/sunteng/Projects/WBC_Telep/GENMO/inputs/{pretrained,onnx,checkpoints}` | 9/9 文件完成，哈希已校验；只剩授权的 `SMPLX_NEUTRAL.npz` |

## 阻塞和所需输入

真实 GEM inference 当前的外部阻塞：

- 本地不存在 `SMPLX_NEUTRAL.npz`，需要按照官方 SMPL-X 许可获取。
- 录制视频路径引用的 `epoch=10-step=25000.ckpt` 与 `vitpose-h-multi-coco.pth`
  已通过公开 GVHMR 镜像获取；webcam pipeline 所需的 ONNX 也已完整下载。

后续可能需要：一段单人全身清楚可见的短视频；计划支持的语言 prompt 和动作安全词表；用于离线复现的音乐文件；若训练团队已有 PiPlus SMPL/robot retarget 配置，也需要提供。

## 随时恢复入口

重新继续时按以下顺序执行：

1. 阅读上面的进度账本，选择第一个“进行中”项目。
2. 检查 `git status`，保留无关的用户文件。
3. GEM-001：检查 `sim2real/teleop/gem_smpl_pub.py` 和测试，再运行文档中的验证命令。
4. GEM-002：先检查 `/home/sunteng/Projects/WBC_Telep/GENMO/inputs/checkpoints/body_models/smplx/SMPLX_NEUTRAL.npz` 是否存在。
5. 每个真实运行先写入实验记录，再把任务标为“已完成”。
