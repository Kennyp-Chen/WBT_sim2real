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

最后更新：2026-08-14。

| ID | 状态 | 交付物 | 证据 / 当前结果 |
|---|---|---|---|
| PRE-001 | 已完成 | SONIC release 和 low-latency G1/SMPL ONNX 集成 | `checkpoints/sonic/{release,low_latency}` |
| PRE-002 | 已完成 | SONIC SMPL ZMQ contract | `sim2real/teleop/smpl_stream.py`、`SmplRealtimeMotionBuffer` |
| PRE-003 | 已完成 | PICO/XRobot SMPL publisher 和 GMR 腕部参考 | `pico_retarget_pub.py --publish-smpl` |
| PRE-004 | 已完成 | PiPlus 22DoF BFM-Zero merged ONNX 和 observations | commit `7a3b2c6` |
| PRE-005 | 已完成 | PiPlus NPZ/ZMQ 播放及同步视频录制 | 两个录制脚本 |
| PRE-006 | 进行中 | HT PiPlus-LSE 23DoF 和 Hi 25DoF BFM-Zero contract/deploy artifacts | BFM-Zero observations、policy YAML/ONNX、MJCF 解析和动作工具已加入；两个机器人仍需完整 sim2sim 验证 |
| GEM-000 | 已完成 | GEM 论文、项目及 runtime 调研 | GEM 报告；本地 clone 位于 `/home/sunteng/Projects/WBC_Telep/GENMO` |
| GEM-001 | 已完成 | GEM `smpl_params.pt` 到 SONIC SMPL ZMQ publisher | 已完成 Y-up SMPL 转换、50 Hz 重采样和十帧 future window 发布 |
| GEM-002 | 已完成 | 录制视频 GEM inference 环境和 benchmark | 官方 tennis demo 生成 312 帧有限参数和四个渲染视频，位于 GENMO `outputs/gem_runs/tennis/` |
| GEM-003 | 已完成 | GEM SMPL 到完整 G1 腕部重定向 | GMR 已生成 312 帧、29 关节顺序正确且无越限的 G1 reference |
| GEM-004 | 进行中 | 录制视频 SONIC sim2sim 和对比视频 | 312 帧 ZMQ sim2sim 和三栏视频已通过视觉/时序检查；还缺定量 tracking/fall 报告 |
| GEM-005 | 未开始 | 实时 webcam GEM-to-SONIC | 依赖录制视频验证 |
| GEM-006 | 进行中 | 交互式 text-to-motion stream | `gem_text_motion_stream.py` 已生成有序、有界 text chunk；SONIC chunk publisher 已加入；实时 prompt UI 待完成 |
| GEM-007 | 进行中 | music/audio-to-motion stream | `gem_audio_motion_stream.py` 支持 GEM raw 18 kHz audio 或预计算 35 维 music embedding；实时采集和 beat-aware transition 待完成 |
| GEM-008 | 已完成 | SMPL 到 PiPlus 22DoF retargeter | PiPlus GMR mapping 输出 50 Hz `[519,29]`，无关节越限，any4hdmi contract 校验通过 |
| GEM-009 | 进行中 | PiPlus/G1 BFM-Zero 多模态 sim2sim | 312 帧同步 ZMQ reference/policy 视频通过视觉时序和稳定性检查；还缺定量 tracking 报告 |
| GEM-010 | 未开始 | 真机安全门和有限真机试验 | 依赖稳定 sim2sim 指标 |
| GEM-011 | 延后 | HT PiPlus-LSE 23DoF 和 Hi 25DoF 的 GEM 重定向 | 必须在 G1 和 PiPlus 22DoF BFM-Zero sim2sim 完成后开始；不能假设 22/23/25DoF contract 可互换 |

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
- `scripts/retarget_gem_smpl.py` 和已纳入仓库的 PiPlus GMR mapping 已支持生成 any4hdmi reference。
- `sim2real/teleop/npz_pub.py` 是已有的 canonical 机器人 motion ZMQ publisher，
  默认使用 28701 端口；它回放 any4hdmi/NPZ qpos，并发布
  `joint_pos`/`body_pos_w`/`body_quat_w`，供 `motion_backend=zmq` 使用。
- `sim2real/teleop/gem_bfmzero_pub.py` 是 GEM 适配器：先运行一次 GMR，把
  `smpl_params.pt` 转成 RobotCfg 顺序的 qpos，再按目标发布频率重采样、保存可复现
  的 NPZ/manifest，最后委托给 `npz_pub`。

### HT BFM-Zero 机器人

- `piplus_lse_23dof` 和 `hi_25dof` 已注册各自的 joint/body contract 和
  BFM-Zero observations；部署文件位于
  `checkpoints/bfm-zero/{piplus-lse-23dof,hi-25dof}`。
- 这两个机器人暂时不接入 GEM。先完成 G1 和 PiPlus 22DoF 的完整独立
  sim2sim 验证，再分别加入机器人专用的 SMPL/GMR mapping 和 motion contract。
- 不能复用 PiPlus 22DoF 的 GEM mapping，也不能静默补齐 action 维度：新策略是
  23DoF 和 25DoF，body order 也不同。

### GEM

- 官方仓库已 clone 到 `/home/sunteng/Projects/WBC_Telep/GENMO`。
- 工作站有 RTX 5060 Ti 16 GB 显存，磁盘空间足够。
- GENMO 源码已克隆到 `/home/sunteng/Projects/WBC_Telep/GENMO`。
- `gem_smpl.ckpt` 和官方 webcam 六个 ONNX 文件已下载到
  `inputs/pretrained/` 和 `inputs/onnx/`；7 个 SHA-256 均与 NVIDIA GEM-X Hugging Face 元数据一致。
- HMR2 与 ViTPose 原始 checkpoint 已从公开的 `camenduru/GVHMR` 镜像下载到
  `inputs/checkpoints/{hmr2,vitpose}/`，字节数和 SHA-256 与该镜像元数据一致。
- `SMPLX_NEUTRAL.npz` 已安装到 `inputs/checkpoints/body_models/smplx/`。
- GEM 使用的 GVHMR body-model 小型运行时资源已安装到 `gem/utils/body_model/`。
- 官方 tennis demo 输出位于 `/home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt`，包含 312 帧有限的全局位移、根姿态、body pose 和 shape。
- GMR clone 位于 `/home/sunteng/Projects/WBC_Telep/GMR`，它只用于离线重定向，不是 policy 在线推理依赖。

### Text 和 audio stream adapter

- `scripts/gem_text_motion_stream.py` 包装官方 GENMO video/text 混合 demo。每个
  prompt 是固定 30 Hz chunk，请求按 FIFO 排列，队列满时拒绝新 prompt，不重排或
  丢弃已经接受的 prompt。GENMO 仍然使用 anchor video 提供相机和尺度上下文。
  本地 GENMO checkpoint 不包含 `t5-3b`；工作站执行真实 text generation 前需要先
  缓存这个 Hugging Face 模型。adapter 已经使用 tennis 预处理缓存走通 GENMO Stage 2。
- `scripts/gem_audio_motion_stream.py` 构造模型实际需要的输入：录制音频必须是
  mono 18 kHz waveform（每个 30 Hz motion frame 对应 600 samples），或者已有的
  `[T,35]` GEM `music_embed`。输出是普通 GEM `smpl_params.pt`，可复用现有 retargeter。
  安装 PyAV 后还可以直接解码 MP3/WAV，不依赖系统 `ffmpeg` 可执行文件。
- `sim2real/teleop/gem_chunk_pub.py` 监听已完成的 text/audio chunk，按顺序发布到
  已有的 SONIC SMPL 28702 端口，不读取 `.tmp.pt`，也不打乱 chunk 顺序。

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

## 当前 Tennis 录制链路的准确数据流

下面是当前 tennis 实验真正跑通的流程。SONIC 和 BFM-Zero 两条分支使用的
运行时接口不同，不能把它们简单看成同一个 publisher。

### 1. 人类视频到 GEM 参数

```text
GENMO/inputs/demo/tennis.mp4
    -> GEM 录制视频推理
    -> outputs/gem_runs/tennis/smpl_params.pt
```

GEM 文件是逐帧 SMPL-X 参数记录，不是机器人轨迹，也不是策略 action。当前
文件有 312 帧，字段如下：

| 字段 | 形状 | 含义 |
|---|---:|---|
| `body_params_global.body_pose` | `[T,63]` | 21 个非根 SMPL 身体关节的 local axis-angle 旋转 |
| `body_params_global.global_orient` | `[T,3]` | SMPL 根部 axis-angle 旋转 |
| `body_params_global.transl` | `[T,3]` | 根部全局平移 |
| `body_params_global.betas` | `[T,10]` | SMPL-X 身体形状系数 |
| `body_params_incam.*` | 同上 | 相机坐标系下的对应参数 |
| `K_fullimg` | `[T,3,3]` | 每帧相机内参 |

本实验将输入时钟按 30 Hz 处理。原始视频元数据约为 29.83 Hz，因此严格的
原始时间戳重建属于后续改进项。

### 2. SONIC 分支：SMPL-ZMQ 到 G1 策略

```text
smpl_params.pt
    -> gem_smpl_pub.py
       - SONIC canonical SMPL FK
       - Y-up/根坐标系转换
       - GMR 生成 G1 腕部 reference
       - 30 Hz -> 50 Hz 重采样
       - 10 帧 future window
    -> ZMQ tcp://*:28702
    -> Tracking(motion_backend=smpl_zmq)
    -> SONIC SMPL ONNX
    -> 29 关节 action -> PD torque -> G1 MuJoCo / 真机
```

`gem_smpl_pub.py` 使用 GEM 的身体旋转生成 SONIC canonical 字段
`smpl_joint_pos_root [N,24,3]` 和 `smpl_root_quat_w [N,4]`。相机内参不会发给
策略。由于 SONIC encoder 需要 G1 的 wrist roll/pitch/yaw，而不是 SMPL wrist
位置，所以六个机器人腕部角度由 GMR 单独提供。publisher 在发送前，使用
SLERP 插值旋转、线性插值位置和机器人关节 reference，将全部数据重采样到
SONIC 的 50 Hz 控制时钟，并发送 future window。

策略接收当前 G1 proprioception 和未来 SMPL 窗口。部署输出是 29 关节 action
（以及 checkpoint 在适用时的 token/state 输出）；controller 再把 action 转成
关节目标和 MuJoCo/机器人 torque。`tennis_source_g1_reference_sonic.mp4`
展示了原始视频、GMR reference 和 SONIC 执行结果。

### 3. BFM-Zero 分支：SMPL-to-G1 NPZ 到策略

```text
smpl_params.pt
    -> scripts/retarget_gem_smpl.py --robot g1
       - 完整 SMPL-X global parameters
       - GMR SMPL-X -> G1 IK
       - 根部 XY 归一化和 MJCF 校验
    -> outputs/gem_retarget/tennis/g1/motions/tennis.npz [312,36] @ 30 Hz
    -> any4hdmi loader 重采样到 50 Hz [519,36]
       并计算 reference body FK/速度
    -> integrated_sim2sim.py + BFM-Zero ONNX
    -> 29 关节 action -> PD torque -> G1 MuJoCo / 真机
```

离线录制使用 `motion_backend=npz`：策略运行时按照配置的 50 Hz target FPS
加载 30 Hz NPZ，因此内部运动序列是 519 帧；视频写入仍然是 30 fps，所以视频
有 312 帧。

BFM-Zero 接收的是机器人 proprioception/history 和未来 G1 reference window，
不会直接接收原始 SMPL 字段或 `smpl_params.pt`。

当前三栏视频是：
`outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4`。

现在 GEM 到 BFM-Zero 的 canonical 录制 ZMQ 链路是：

```text
smpl_params.pt
    -> gem_bfmzero_pub.py
       - GMR SMPL-X -> RobotCfg qpos
       - qpos 重采样到 50 Hz
       - 可复现的 any4hdmi NPZ
    -> tcp://*:28701 上的 npz_pub schema
    -> Tracking(motion_backend=zmq)
    -> BFM-Zero ONNX -> robot action/torque
```

直接运行适配器：

```bash
HF_HUB_OFFLINE=1 HF_HUB_DISABLE_TELEMETRY=1 \
uv run python sim2real/teleop/gem_bfmzero_pub.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot g1 --bind 'tcp://*:28701'
```

同步录制脚本现在既接受已有 NPZ，也接受 GEM 参数：

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

当前三栏视频是：
`outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4`。
最右侧现在来自 BFM-Zero ZMQ 运行，并且和 SONIC reference 窗口一样使用 640×720
渲染比例。

## 阶段一：录制视频到 SONIC

### GEM-001：通用 GEM SMPL Publisher

目标：读取官方 GEM `smpl_params.pt`，把 global body parameters 转换成现有 SONIC SMPL contract，并按 SONIC 的 50 Hz 控制时钟在 28702 端口播放。

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

授权的 `SMPLX_NEUTRAL.npz` 已安装，并用于 tennis 推理。

### GEM-003：完整 G1 腕部重定向

目标：用同一 GEM 动作生成 robot joint reference，替换 default-pose 腕部 fallback。

任务：确认 GMR 是否支持 SMPL 输入，否则实现最小 SMPL-to-G1 IK；严格使用 SONIC YAML 的 IsaacLab joint order；验证六个 wrist roll/pitch/yaw；和已知 SONIC 官方 SMPL+G1 pair 对比。

验收条件：无 fallback 警告；29 个 G1 reference 有限且在限位内；腕部方向与人体动作一致；SONIC input 为 840 维并且手臂旋转时 wrist slice 会变化。

### GEM-004：录制视频 SONIC Sim2sim

首次在可以访问 GitHub 时安装 GMR 依赖，然后生成 G1 腕部 reference。安装完成后，
重定向和策略推理均可离线运行：

```bash
uv sync --extra retarget
uv run python scripts/retarget_gem_smpl.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot g1 --target-fps 30 \
  --output-dir outputs/gem_retarget/tennis/g1
```

三个终端：

```bash
uv run python sim2real/teleop/gem_smpl_pub.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --joint-reference outputs/gem_retarget/tennis/g1/tennis_joint_reference.npz
```

一条命令录制同步结果：

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

验收条件：SMPL buffer 有足够 future window 后启动 policy；reference/policy 时间线一致；无 NaN、shape error 或 root heading 跳变；保存 source/GEM/robot 对比视频；记录跌倒状态和 root tracking error。

## 阶段二：实时视频

### GEM-005：Webcam Stream

分别 benchmark 带/不带 HMR2 的官方 ONNX；让 `demo_webcam.py` 输出结构化逐帧结果；保留 camera capture timestamp；复用 GEM-001 的 ZMQ schema；增加 confidence gating、人物锁定、丢失保持和受控恢复；分别测量 camera-to-policy 与 camera-to-simulation 延迟。

验收条件：吞吐达到选定输入 FPS 或明确报告降采样；时间戳不会让策略看起来早于源视频；跟踪丢失时稳定 hold；保存 60 秒摄像头对比视频和延迟报告。

## 阶段三：语言和音乐

### GEM-006：交互式文本

实现固定长度生成 chunk、有限队列、取消、片段 overlap 和回到站立的 prompt 服务。先做手动文本，不在其稳定前加入语音识别。

验收条件：新 prompt 不会重排已发布动作；policy 不会遇到空 future window；片段边界不超过姿态/角速度限制；记录 prompt、seed、生成耗时和 artifact。

当前录制 chunk 实现：

```bash
uv run python scripts/gem_text_motion_stream.py \
  --anchor-video /home/sunteng/Projects/WBC_Telep/GENMO/inputs/demo/tennis.mp4 \
  --ckpt-path /home/sunteng/Projects/WBC_Telep/GENMO/inputs/pretrained/gem_smpl.ckpt \
  --prompt "a person walks forward" \
  --prompt "a person dances" \
  --output-dir outputs/gem_stream/text

uv run python sim2real/teleop/gem_chunk_pub.py \
  --input-dir outputs/gem_stream/text \
  --bind 'tcp://*:28702'
```

第一条命令对每个接受的 prompt 调用一次 GENMO，写出
`chunk_000000/smpl_params.pt`、`chunk_000001/smpl_params.pt`。第二条命令把已
完成的 chunk 发送给 SONIC；retarget 和 policy 执行不变。这是录制/离线 chunk
stream，还不是逐按键实时生成器。

### GEM-007：音乐和音频

先复现官方离线 audio/music 条件样本，再增加麦克风/文件采集及与训练输入一致的特征提取。片段连接使用 beat-aware overlap，不能任意拼接独立 clip。

验收条件：保存音频可复现离线动作；实时 audio buffer 延迟有界且时间戳不倒退；保存 60 秒 music-to-motion SONIC sim2sim 视频。

当前文件音频入口：

```bash
uv run python scripts/gem_audio_motion_stream.py \
  --audio /absolute/path/to/input.wav \
  --ckpt-path /home/sunteng/Projects/WBC_Telep/GENMO/inputs/pretrained/gem_smpl.ckpt \
  --output outputs/gem_stream/audio/smpl_params.pt
```

要把录制音频按顺序发布成 chunk，可以使用目录输出，再启动 chunk publisher：

```bash
uv run python scripts/gem_audio_motion_stream.py \
  --audio /absolute/path/to/input.wav \
  --output-dir outputs/gem_stream/audio/chunks \
  --chunk-frames 300

uv run python sim2real/teleop/gem_chunk_pub.py \
  --input-dir outputs/gem_stream/audio/chunks \
  --bind 'tcp://*:28702'
```

音乐条件输入必须是训练时的逐帧 `[T,35]` `music_embed`，单独的 MP3 不能直接作为
GEM music condition：

```bash
uv run python scripts/gem_audio_motion_stream.py \
  --music-embed /absolute/path/to/music_embed.pt \
  --output outputs/gem_stream/music/smpl_params.pt
```

两个输出都可以继续交给 `scripts/retarget_gem_smpl.py`、`gem_smpl_pub.py` 或
`gem_bfmzero_pub.py`。真实麦克风前端和 beat-aware overlap 要等代表性音频实际
生成并完成时序测量后再标记完成。raw WAV 和 `[T,35]` music 路径已经完成 60 帧真实
模型 smoke test，生成文件也已被 `gem_chunk_pub.py` 按 50 Hz 接收发布。

## 阶段四：PiPlus BFM-Zero

### GEM-008：SMPL 到 PiPlus 重定向

追踪 PiPlus 训练动作表示，实现可复用 retargeter，输出 backward encoder 使用的精确 22DoF 顺序和 root/body frame。策略推理前先通过 PiPlus MJCF 验证 qpos。

```bash
uv run python scripts/retarget_gem_smpl.py \
  --gem-params /home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/smpl_params.pt \
  --robot piplus_h0w \
  --output-dir outputs/gem_retarget/tennis/piplus_h0w
```

验收条件：qpos 为配置 MuJoCo 顺序的 `[T,29]`；所有关节名与 YAML/MJCF 一致；FK body position/quaternion 有限且连续；已知动作与现有 PiPlus LAFAN 数据在数值或视觉上相符。

### GEM-009：PiPlus/G1 多模态 Sim2sim

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
| 2026-08-13 | GEM-002 | PiPlus `jumps1_subject1_...clip0.mp4` | `demo_smpl_hpe.py --no_render --ckpt_path inputs/pretrained/gem_smpl.ckpt` | `outputs/gem_runs/jumps1_subject1_20260806_PiPlus_S_12L8A0G2H0W_LSE_ZedMini_260804_clip0/{smpl_params.pt,0_kp2d_overlay.mp4}` | 300 帧；22.8 秒；global/incam body tensor 和相机内参全部有限；渲染推迟 |
| 2026-08-13 | GEM-002 | GVHMR 官方 `tennis.mp4` demo | GEM 录制视频推理和渲染 | `/home/sunteng/Projects/WBC_Telep/GENMO/outputs/gem_runs/tennis/` | 312 帧有限参数；保存 `0_kp2d_overlay`、`1_incam`、`2_global`、`3_incam_global_horiz` |
| 2026-08-13 | GEM-003/008 | tennis `smpl_params.pt` | `scripts/retarget_gem_smpl.py` + GMR | `outputs/gem_retarget/tennis/{g1,piplus_h0w}` | G1 `[312,36]`/30 Hz；PiPlus `[519,29]`/50 Hz；根四元数归一化，关节越限为 0 |
| 2026-08-13 | GEM-004 | tennis GEM + G1 GMR reference | `record_gem_sonic_video.py`、release SMPL policy | `outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_sonic.mp4` | 312 帧/30 fps；源 0.000 秒开始，policy 0.220 秒启动；无 shape/NaN/runtime 错误，视觉稳定 |
| 2026-08-13 | GEM-009 | tennis PiPlus GMR reference | `record_zmq_policy_videos.py`、PiPlus BFM-Zero | `outputs/gem_retarget/tennis/comparisons/tennis_source_piplus_reference_bfmzero.mp4` | 312 帧/30 fps；源 0.000 秒开始，policy 0.180 秒启动；reference/policy 同步且保持站立 |
| 2026-08-13 | GEM-009 | tennis GEM -> G1 BFM-Zero robot-motion ZMQ | `record_zmq_policy_videos.py --gem-params`、`gem_bfmzero_pub.py`、BFM-Zero G1 policy | `outputs/gem_retarget/tennis/policy_videos/bfm_zero_g1/tennis_zmq_side_by_side.mp4`、`outputs/gem_retarget/tennis/comparisons/tennis_source_g1_reference_bfmzero.mp4` | 312 帧/30 fps；GMR qpos 30 Hz，NPZ/ZMQ 50 Hz；`motion_backend=zmq` 收到 29 joints/33 bodies；wall drift +0.371 秒；视觉稳定 |
| 2026-08-14 | GEM-006 | 有序 text chunk | `scripts/gem_text_motion_stream.py --dry-run`；使用 tennis 预处理缓存的真实 prompt 尝试 | `outputs/gem_stream/text_real_attempt/.genmo/chunk_000000/` | Stage 1/2 成功；唯一阻塞是本机缺少 Hugging Face `t5-3b`；有界 FIFO 和失败日志已验证 |
| 2026-08-14 | GEM-007 | raw audio/music 输入 contract | `scripts/gem_audio_motion_stream.py --audio .../gem_test_audio.wav` 和 `--music-embed .../gem_test_music_embed.npy` | `outputs/gem_stream/audio/real_attempt.pt`、`outputs/gem_stream/music/real_attempt.pt` | 两种 60 帧真实 GEM 生成成功；audio chunk 已由 publisher 以 50 Hz 接收；实时采集、beat-aware transition、60 秒视频待完成 |
| 2026-08-14 | GEM-007/009 | Kai Engel《Blizzard (PON I)》15 秒音乐 demo | PyAV 解码 -> `gem_audio_motion_stream.py` -> `retarget_gem_smpl.py` -> `record_zmq_policy_videos.py` | `outputs/gem_retarget/music_demo/videos/kai_engel_blizzard_g1_bfmzero.mp4`、`outputs/gem_retarget/music_demo/videos/kai_engel_blizzard_piplus_bfmzero.mp4` | 两段均为 450 帧、30 fps、15 秒；G1 全程保持站立且重定向关节越限为 0；PiPlus reference 有效但策略在中后段跌倒，PiPlus 音乐跟踪暂不验收 |

## 阻塞和所需输入

录制视频链路已无外部阻塞。进入实时摄像头或真机前，先为两个现有录制补充
root/body tracking 与跌倒定量指标，再实现摄像头 confidence/dropout 处理。

后续需要：代表性的实时摄像头输入、计划支持的语言 prompt 和动作安全词表，以及用于离线复现的音乐文件。

## 随时恢复入口

重新继续时按以下顺序执行：

1. 阅读上面的进度账本，选择第一个“进行中”项目。
2. 检查 `git status`，保留无关的用户文件。
3. 为两个同步录制器补充定量 tracking/fall 报告，完成 GEM-004/GEM-009。
4. 重跑上面的 tennis artifact，并将数值报告和现有视频对照。
5. 缓存 `t5-3b`，跑代表性 text chunk，再对 text/audio/music 做 retarget 和录制，然后再加入实时采集。
6. 在 G1 和 PiPlus 22DoF BFM-Zero sim2sim 验收完成前，不要开始 GEM-011。

## 延后最终阶段：新 HT 机器人的 GEM

`piplus_lse_23dof` 和 `hi_25dof` 的 GEM 接入明确放在机器人相关计划的最后。
开始前必须完成：

1. 两个新策略各自的离线和 ZMQ sim2sim；
2. final-frame、跌倒、root tracking 和 action-limit 检查；
3. 确认各自训练时实际使用的 MJCF body order 和 motion FPS；
4. 为每个机器人单独建立 GMR/SMPL mapping 和 any4hdmi manifest。

G1 或 PiPlus 22DoF GEM 跑通只能证明公共架构可行，不能证明新机器人的
observation 或 joint order 可以直接复用。
