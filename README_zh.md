# sim2real

root project 负责 inference、tracking policy，以及 MuJoCo 的 sim / sim2real runtime。Pico / XR teleoperation 工具请使用 `venv/pico`。

English version: [README.md](./README.md)

Full documentation: [https://egalahad.github.io/sim2real/](https://egalahad.github.io/sim2real/)

如果你在找 HDMI 的部署栈，请看 [hdmi tag](https://github.com/EGalahad/sim2real/tree/hdmi)。

## Runtime Artifacts

大文件不放在 git 里。先从共享的
[sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
下载，把 `checkpoints/` 和 `third_party/` 放到 repo 根目录。

目录结构和 onboard 依赖说明见 [Download Artifacts](./docs/artifacts.md)。

## 快速开始

```bash
uv sync --extra inference-cpu
```

在 G1 上安装或修复环境时，可以调用 repo 内置的 Codex skill
`$configure-g1-sim2real`；它位于 `.agents/skills/configure-g1-sim2real`。

运行离线动作跟踪（sim2sim）：

```bash
uv run sim2real/sim_env/base_sim.py --robot g1
uv run sim2real/rl_policy/tracking.py \
  --robot g1 \
  --policy_config checkpoints/mimic-lite/32x8192-huge/policy.yaml
```

两个进程都启动后，在 policy 终端按 `]` 开始跟踪，然后打开 `base_sim.py` 打印出来的 mjviser URL。虚拟 gantry / elastic band 的开关和长度在 viewer UI 里调。

## PiPlus 22-DoF BFM-Zero

PiPlus 的运行产物不放在 git 中，需要准备：

- 完整机器人资产目录，包括 `xml/` 和 MJCF 引用的同级 `meshes/` 目录。
- 位于同一 checkpoint 目录下的 `policy.yaml` 和合并后的 `policy.onnx`。
- 50 Hz any4hdmi 动作 NPZ，其中包含 `qpos[T, 29]`。

每个 shell 先设置以下路径。`SIM2REAL_PIPLUS_MJCF` 必须指向完整资产目录中的
XML，不能使用脱离配套 meshes 单独复制出来的 XML。

```bash
export SIM2REAL_PIPLUS_MJCF=/absolute/path/to/PiPlus_S_12L8A0G2H0W/xml/PiPlus_S_12L8A0G2H0W_with_armature.xml
export PIPLUS_POLICY=checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml
export PIPLUS_MOTION=/absolute/path/to/any4hdmi_full/motions/dance2_subject2.npz
```

### 转换和合并运行产物

把 HumanoidVerse LAFAN pickle 转成 50 Hz any4hdmi motions：

```bash
uv run python scripts/convert_piplus_lafan_pkl_to_any4hdmi.py \
  --source /absolute/path/to/piplus_h0w_lafan_combined.pkl \
  --output /absolute/path/to/any4hdmi_full \
  --mjcf "$SIM2REAL_PIPLUS_MJCF" \
  --target-fps 50
```

把 decoder/actor 和 backward encoder 合并成 sim2real 使用的单个语义输入 ONNX：

```bash
uv run --with onnx python scripts/merge_bfm_zero_piplus_onnx.py \
  --actor /absolute/path/to/exported/FBcprAuxModel.onnx \
  --encoder /absolute/path/to/exported/FBcprAuxModel_z_encoder.onnx \
  --output checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.onnx
```

### 录制离线 sim2sim

使用通用批量录像脚本录制确定性的 policy 单侧视频：

```bash
uv run python scripts/tracking_experiment/record_policy_videos.py \
  --policy bfm_zero_piplus \
  --robot piplus_h0w \
  --motion "$PIPLUS_MOTION" \
  --duration-s 60 \
  --output-dir outputs/policy_videos
```

不经过 ZMQ，把动作 qpos 播放放在已有 policy 视频左侧：

```bash
uv run python scripts/tracking_experiment/make_motion_policy_side_by_side.py \
  --robot piplus_h0w \
  --motion-path "$PIPLUS_MOTION" \
  --policy-video outputs/policy_videos/bfm_zero_piplus/MOTION_NAME.mp4 \
  --output outputs/policy_videos/bfm_zero_piplus/MOTION_NAME_side_by_side.mp4 \
  --duration-s 60
```

### 运行实时 ZMQ 路径

使用三个终端。第一个终端启动仿真器：

```bash
uv run python sim2real/sim_env/base_sim.py \
  --robot piplus_h0w \
  --sim-dt 0.005
```

第二个终端启动 policy，然后按 `]` 进入 policy 模式：

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

第三个终端启动动作 publisher。按 `space` 播放或暂停；按 `]` 回到第 0 帧并暂停。

```bash
uv run python sim2real/teleop/npz_pub.py \
  --robot piplus_h0w \
  --motion-path "$PIPLUS_MOTION" \
  --initial-source motion \
  --root-body-name base_link \
  --publish-hz 50 \
  --bind 'tcp://*:28701'
```

如果要录制同一条 ZMQ 路径的同步动作/policy 左右对比视频，请使用下面的
自包含录像器。它会自己启动 publisher、policy 和 MuJoCo bridge；不要和上面的
三个交互式终端同时运行同一个 28701 端口。

```bash
uv run python scripts/tracking_experiment/record_zmq_policy_videos.py \
  --robot piplus_h0w \
  --policy-config "$PIPLUS_POLICY" \
  --motion-path "$PIPLUS_MOTION" \
  --output outputs/policy_videos/bfm_zero_piplus/zmq_side_by_side_sync.mp4 \
  --duration-s 60 \
  --fps 30
```

录像器使用 50 Hz motion、50 Hz policy、200 Hz MuJoCo physics，并以 30 FPS
编码。左侧跟随 ZMQ publisher 实际发出的 frame，右侧显示 policy 驱动的仿真。
结束日志会报告名义时长、实际墙上时长和录像时钟漂移。

真实 PiPlus 部署请使用 ROS2 到 ZMQ 的 bridge；状态、关节映射和安全启动步骤见
[PiPlus BFM-Zero sim2real 指南](docs/piplus_bfmzero_sim2real.md)。
PiPlus-LSE 23-DoF 的非 identity 硬件映射、preflight 启动器和真机命令见
[PiPlus-LSE 23-DoF 指南](docs/i18n/zh-Hans/docusaurus-plugin-content-docs/current/tutorials/piplus-lse-23dof-sim2real.md)。

## Migrating to sim2real

这个 repo 内置了一个 Codex skill，用来把外部训练 codebase 里的 policy 适配到 `sim2real`：

```text
.agents/skills/adapt-policy-to-sim2real
```

已经转好的 checkpoints 统一放在共享的
[sim2real artifacts](https://drive.google.com/drive/folders/1lrPyiiy7anyG3P4wHNIQQQlydboLPd9e)
目录里。

目前已经支持的 adapted / distributed checkpoint：

| Policy family | Config path(s) | 说明 |
| --- | --- | --- |
| Mimic-Lite | `checkpoints/mimic-lite` | Native mimic-lite tracking checkpoints。 |
| BFM-Zero | `checkpoints/bfm-zero/exp_lafan40-100style_update_z10/policy.yaml` | Latent-conditioned motion tracker。 |
| BFM-Zero PiPlus 22-DoF | `checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml` | 将 PiPlus decoder 和 backward encoder 合并为一个语义输入 ONNX。 |
| BFM-Zero PiPlus-LSE 23-DoF | `checkpoints/bfm-zero/piplus-lse-23dof/policy.yaml` | 使用独立的 23-DoF joint/body 和真实硬件顺序契约。 |
| ScaleBFM | `checkpoints/scalebfm` | [WeishuaiZeng/ScaleBFM](https://huggingface.co/WeishuaiZeng/ScaleBFM) 的 Humanoid Transformer M 和 XL ONNX exports。 |
| SONIC release | `checkpoints/sonic/release` | Release G1 和 SMPL encoder variants。 |
| SONIC low-latency | `checkpoints/sonic/low_latency` | Low-latency G1 和 SMPL variants。 |
| HoloMotion v1.4.0 | `checkpoints/holomotion/v1_4_0/policy.yaml` | 使用官方未修改 ONNX：[HorizonRobotics/HoloMotion_models](https://huggingface.co/HorizonRobotics/HoloMotion_models/resolve/main/HoloMotion_motion_tracking_model_v1.4.0/exported/model_14000.onnx)，下载后放到 `checkpoints/holomotion/v1_4_0/policy.onnx`。 |
| TeleopIT | `checkpoints/teleopit/policy.yaml` | TeleopIT policy wrapper。 |
| Humanoid-GPT | `checkpoints/humanoid-gpt/policy.yaml` | Humanoid-GPT policy wrapper。 |
| HEFT | `checkpoints/heft` | PMG 和 compliance 两个版本。 |
| TWIST2 | `checkpoints/twist2/policy.yaml` | TWIST2 policy wrapper。 |

![统一的跨代码库动作跟踪评测](assets/mimic_lite_cross_codebase_tracking_eval.png)

图中使用 13 个 policy variants 的全新结果，数据集为 LAFAN-40、PHUMA-30
和清洗后的 Root-90。Root-90 每段沿标注的前进、后退或侧移方向持续运动，
root XY 位移为 1.5--3.0 m。

为了公平比较，我们报告每个 policy 所需的 motion-lookahead latency，并将
其定义为最远 future reference frame 对应的时间。所有数值均采用统一的
50 Hz reference-motion contract。

| Policy | MimicLite | BFM-Zero | ScaleBFM | SONIC release | SONIC low-latency | HoloMotion | TeleopIT | Humanoid-GPT | HEFT | TWIST2 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Motion-lookahead latency | 0.08 s | 0.12 s | 0.10 s | 0.90 s | 0.18 s | 0.20 s | 0.00 s | 0.02 s | 0.12 s | 0.00 s |

## 真机环境

机器人 SDK 不安装进通用 root 环境。G1 inline 部署使用
`uv sync --extra inference-cpu --extra robot-g1`。安装与部署命令见
[Robot I/O 模式](./docs/robot_io.md)。

Repo skills 统一放在 `.agents/skills/`，无需手动复制到
`~/.codex/skills/`。可以在 Codex 中显式调用
`$adapt-policy-to-sim2real`。

## 下一步

- [文档首页](https://egalahad.github.io/sim2real/zh-Hans/)
- [快速上手](https://egalahad.github.io/sim2real/zh-Hans/getting-started/overview)
- [Root Project Setup](https://egalahad.github.io/sim2real/zh-Hans/getting-started/root-project)
- [离线动作跟踪教程](https://egalahad.github.io/sim2real/zh-Hans/tutorials/offline-motion-tracking)
- [Pico Teleoperation 教程](https://egalahad.github.io/sim2real/zh-Hans/tutorials/pico-teleoperation)

## Citation

如果 sim2real 对你的研究有所帮助，请引用：

```bibtex
@misc{sim2real2026,
  author       = {{RoboParty Lab Team}},
  title        = {sim2real: A Lightweight and Modular Sim2sim and Sim2real Deployment Stack},
  year         = {2026},
  howpublished = {\url{https://github.com/EGalahad/sim2real}},
  note         = {Documentation: \url{https://egalahad.github.io/sim2real/}}
}
```
