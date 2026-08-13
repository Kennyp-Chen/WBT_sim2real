---
title: GEM 多模态动作前端
slug: /reference/gem-multimodal-motion
---

# GEM 多模态动作前端

最后核对时间：2026-08-12；对应 NVlabs/GENMO 提交
`16bebf402d8893184249ee206d957b8248cd8310`。

## 结论

[GEM](https://github.com/NVlabs/GENMO) 原名 GENMO，是用于人体姿态估计和动作生成的通用模型。它在本项目中的合理定位是上游人体动作前端：

```text
视频 / 2D 关键点 / 文本 / 音频 / 音乐
    -> GEM
    -> SMPL 人体参数
    -> 机器人重定向和 runtime 适配
    -> SONIC 或 BFM-Zero
    -> MuJoCo 或机器人真机
```

这条工作流可行，但 GEM 不是机器人控制策略。它不直接生成 G1 或 PiPlus 电机指令，也不能把 RGB、文本或原始音频直接传给现有策略 ONNX。

最推荐的第一条链路是“录制视频的 GEM 输出驱动 SONIC SMPL sim2sim”。SONIC 已经支持规范化 SMPL reference stream；PiPlus BFM-Zero 则还需要 SMPL 到 PiPlus 22DoF 的重定向层。

## 文献和项目

- 论文：[GEM: A Generalist Model for Human Motion](https://arxiv.org/abs/2505.01425)，ICCV 2025 Highlight。
- 项目主页：[NVIDIA Research GEM](https://research.nvidia.com/labs/dair/gem/)。
- 代码：[NVlabs/GENMO](https://github.com/NVlabs/GENMO)。项目于 2025 年 12 月由 GENMO 更名为 GEM。
- 完整手部和面部：[GEM-X](https://github.com/NVlabs/GEM-X)。
- 许可证：NVIDIA OneWay Noncommercial。除非另行获得许可，代码及其衍生工作仅限研究或评估使用。

## GEM 当前提供的能力

GEM 用统一模型完成姿态估计和动作生成，不依赖每个任务单独的 head。发布的 GEM-SMPL 声明支持视频、文本、音频和音乐条件。

| 输入 | 官方入口 | 当前成熟度 | 对 sim2real 有用的输出 |
|---|---|---|---|
| 录制视频 | `scripts/demo/demo_smpl_hpe.py` | 官方离线 demo | `smpl_params.pt` |
| 视频和文本混合 | `scripts/demo/demo_smpl.py` | 官方离线 demo | SMPL 参数和渲染视频 |
| 摄像头或顺序视频 | `scripts/demo/demo_webcam.py` | 官方 ONNX 流式 demo | 逐帧 SMPL body parameters |
| 文本 | `demo_smpl.py` 文本片段 | 分段生成；默认 30 FPS、300 帧 | 生成的 SMPL 动作 |
| 音频和音乐 | 模型、数据集及完整训练配置 | 模型支持，但没有同等成熟的实时麦克风 demo | 增加输入前端后生成 SMPL 动作 |

离线输出格式为：

```text
smpl_params.pt
  body_params_global
    body_pose       [T, 21, 3] axis-angle
    global_orient   [T, 3] axis-angle
    transl          [T, 3]
    betas           [T, 10]（若存在）
  body_params_incam
  K_fullimg
  segment_info      （混合输入时存在）
```

机器人跟踪应使用 `body_params_global`。相机坐标平移和渲染结果不是机器人命令。

## 视频推理链路

官方实时 demo 逐帧运行以下流程：

```text
摄像头或视频
    -> YOLOX 人体检测
    -> ByteTrack 跟踪
    -> ViTPose-H 2D 关键点
    -> HMR2 图像特征（可选）
    -> GEM-SMPL denoiser ONNX
    -> streaming global rollout
```

关键运行特性：

- 默认 context 为 120 帧，在 30 FPS 下约四秒。
- context 未填满之前没有正式输出。
- `--no_imgfeat` 跳过 HMR2，是官方最快模式，但通常会损失精度。
- 异步 pipeline 提高吞吐，但可能增加管线延迟；`--no_async_pipeline` 可取消并行重叠，但吞吐降低。
- ONNX 文件总计约 8.7 GB。官方建议 A100 级或更新 GPU，并说明推理约需 16 GB 显存。
- 当前工作站为 16 GB GeForce RTX 5060 Ti，接近官方显存需求，因此必须实测 full 和 `--no_imgfeat` 模式的峰值显存及延迟，不能只根据“real-time”命名判断。

实际部署建议让 GEM 在带 GPU 的工作站运行，并向策略机发布带时间戳的动作。完整视觉栈不应首先部署到 JetPack 5 机器人计算机。

## 与 SONIC 的关系

SONIC 是最短集成路径，因为仓库已经有 SMPL 专用策略和 runtime。当前接口见 [SONIC SMPL Input](/reference/sonic-smpl-input)。

SONIC 需要：

| 字段 | 形状 | 来源 |
|---|---:|---|
| `smpl_body_pose_aa` | `[N, 21, 3]` | GEM `body_pose` |
| `smpl_joint_pos_root` | `[N, 24, 3]` | SONIC 规范化 SMPL FK |
| `smpl_root_quat_w` | `[N, 4]` | GEM `global_orient` 转换到 SONIC 坐标系 |
| `joint_pos` | `[N, 29]` | G1 重定向；当前 SONIC 使用其中六个腕部关节 |

GEM 能提供人体姿态，但不会提供最后一个 G1 字段。完整 SONIC 集成仍需要 G1 腕部 roll、pitch、yaw 的重定向参考。接口验证阶段可以明确地使用机器人默认姿态填充这六个值，但这只是降级验证模式，不代表完整全身重定向已经完成。

## 与 PiPlus BFM-Zero 的关系

PiPlus BFM-Zero 是机器人动作跟踪控制器。它的导出图接收机器人状态和参考动作 observation，而不是多模态原始输入。PiPlus 链路必须增加：

```text
GEM SMPL
    -> SMPL 到 PiPlus 的重定向 / IK
    -> PiPlus 22DoF qpos 和 body FK
    -> BFM-Zero future reference observations
    -> PiPlus BFM-Zero policy
```

需要明确 PiPlus 关节顺序、限位、默认姿态、机器人资产、root/body 坐标、训练时 body selection、动作 FPS 以及真实 observation 实现。当前 PiPlus BFM-Zero 部署已经完成 merged policy、deploy YAML、observations、MJCF 支持、动作转换、ZMQ 播放和同步视频录制，但尚无 SMPL 到 PiPlus 的重定向器。

## 各输入模式的可行性

### 录制视频

现在即可实施，也是推荐的第一里程碑：

```text
video.mp4
    -> GEM 离线推理
    -> smpl_params.pt
    -> GEM-to-SONIC publisher
    -> SONIC SMPL sim2sim
```

这一步能在引入摄像头延迟之前，分别定位坐标、骨架、时间线和腕部参考问题。

### 实时视频

可使用官方 ONNX webcam pipeline 实现。正式 adapter 必须使用原始采集时间戳发布每个 GEM 输出帧，处理丢帧，对低置信度或不合理姿态做拒绝，并在跟踪丢失时保持稳定参考。

### 语言

可实现分块的“命令到动作”生成，但不是逐词即时控制。官方 demo 把 prompt 表示为固定长度片段，默认十秒。交互式实现需要 prompt queue、生成 worker、片段 overlap/crossfade、取消机制以及安全回到站立的过渡。

### 音乐和音频

发布模型和训练数据路径包含音频、音乐条件，但仓库没有与 webcam demo 同等级的实时麦克风入口。需要新增音频采集、与训练表示一致的特征提取、分块推理、beat-aware 拼接，以及与文本动作相同的机器人可行性过滤。

## 主要工程风险

1. 坐标约定：GEM、SONIC、MuJoCo 和真机的 up-axis、四元数顺序及 root frame 并不统一。
2. 机器人重定向：SMPL body parameters 不能唯一确定机器人腕部角度，也不保证得到动力学可行动作。
3. 时间线：策略为 50 Hz，GEM 常用 30 Hz，webcam 模型还使用 120 帧 context。
4. 动作质量：视频估计会出现抖动、遮挡肢体丢失和多人切换。
5. 物理可行性：文本或音乐生成的接触、速度可能超出策略训练分布。
6. 计算量：完整 GEM 视觉推理很大，必须在目标 GPU 上实测加载、峰值显存和逐帧延迟。
7. 许可证：任何公开或商业部署前都必须审查 GEM 的非商业许可。

## 安全和验收要求

进入真机之前，每种输入源都必须通过：

- shape、关节顺序、四元数顺序和坐标系验证；
- 单调的源时间戳和发布时间戳；
- 固定 FPS replay 与源动作对比；
- 姿态速度和关节限位过滤；
- 跟踪丢失时保持静止 reference；
- sim2sim 跌倒检测和 root trajectory 记录；
- 明确的操作员停止入口和保守命令限制。

详细实施顺序和可持续更新的进度记录见 [GEM 集成计划](/tutorials/gem-integration-plan)。

