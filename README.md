# xMate SR4-C 机器人与相机操作

本仓库控制珞石 xMate SR4-C、固定 D455 相机和 Modbus 夹爪，已完成 RGB 定位、抓取、搬运入盒、释放和图像验收。当前是实机验证过的工具与任务执行代码集合，**不是任意场景的自动抓取系统**。

## 文档入口

实机任务入口只有 [embodied-agent skill](.agents/skills/embodied-agent/SKILL.md)：观察后生成 plan.json，由执行器推进与审核。短计划默认在终端调用 PlanBuilder，不保存 build.py。本文供开发维护、安装/尺寸查询；计划指南按需读取，不是初始化必经步骤。

| 文档 | 内容与使用时机 |
| --- | --- |
| 本文 | 当前进展、安装与物件参数、任务入口、脚本索引 |
| [embodied-agent skill](.agents/skills/embodied-agent/SKILL.md) | 执行、规划或审查实机任务时的决策规则 |
| [计划与几何参考](.agents/skills/embodied-agent/references/plan-format.md) | JSON 字段、几何函数签名、审核接回细节；按需读取 |
| [工具与数据契约](docs/工具与数据契约.md) | 设备工具边界、路线格式、数值保护、返回证据；按需查阅 |
| [Web 控制台](docs/Web控制台.md) | 服务启动、网页操作、常驻传感器与控制权、新帧采样 |
| [标定](docs/标定.md) | 安装变化后的配对采集、外参求解、桌面与精度复核 |
| [维护记录](docs/维护记录.md)、[实机记录](docs/records/) | 变更和实验追溯；历史状态及当时的待办不代替当前状态 |

## 当前状态与开发方向

以下为最后已验证结果，不是实时读数；新任务重新观测，历史路线不得重放。

- **速度修复已入项目。** 2026-10-12 17:23 验收：每次路线显式设置在线倍率为 1，当次任务速率为 10%。同一 J6 12°行程，Web Jog 7.20°/秒、agent 单段 7.18、12 路点 7.20，相对原 1.08 约快 6.7 倍。最后机械臂返回、下电断开，空夹位置 480，倍率保留 1。[根因对照](docs/records/2026-10-12/speed-diagnostic/README.md)、[修复验收](docs/records/2026-10-12/speed-fix/README.md)。
- **完整抓放已成功，5 分钟目标尚未验收。** [14:12 回合](docs/records/2026-10-12/pick-place-1412.md)已验证统一计划、批量预检和 1 mm 过渡；[16:15 回合](docs/records/2026-10-12/pick-place-1615.md)请求到验收约 443 秒，连续阶段未再被控制权释放竞态阻塞。速度修复后尚未重跑完整抓放，不能按单轴提速比例推算总耗时。
- **验证边界。** 当前 FK 检查最大单轴间隔 1°；显式携物反馈闭环及候选成本排序已实现，尚无其完整抓放实机验收记录。下一步用新场景验证端到端耗时与效果，保留最终图像验收。
- **可配置速率已实机验证有效。** 2026-10-13 用户确认：提高 SDK 速率后，实际运动速度有提升。具体测试比例及定量速度未提供，不推断整个 1%..100% 范围均已验收。

个人项目优先直接使用当前设备，不要求离线接收、冻结或预览流程。策略由 agent 决定，执行器顺序调用并审核，适配器封装设备保护；换设备时替换工具实现、能力说明和现场配置。场景算法由任务代码编写或复用，不逐个注册为设备工具，不引入独立模型服务或异常自动恢复。

当前只支持 [计划 Schema v2](config/task-plan.schema.json)；[xMate 能力清单](config/capabilities/)版本 11，含 8 个工具。用户记录的策略配置为 `gpt-6-astra / medium`，沿用现有 Sub2API 凭据、不改全局默认；允许上传必要现场图片，费用不设上限，不上传凭据或整仓库。这不证明当前聊天实际模型。

## 当前设备与物件参数

确认仍为同一物件和安装、用户未说明更换时，直接复用尺寸；位置、朝向、支撑、夹持、盒内占用和动作目标按本轮确定。

| 对象 / 参数 | 已知值与边界 |
| --- | --- |
| 红色三联长方体 | 用户实测长 × 宽 × 高 **90.23 × 29.86 × 29.61 mm**；仅正常平放原桌面时顶面离桌 29.61 mm |
| 白色灰底纸盒 | 盒沿离桌 **50 mm**；用户允许忽略底厚、承托面近似桌面；换盒、内衬或垫高后重核 |
| 纸盒开口 | RGB 两次估计约 **268 × 192 mm**，不是精确实测内径或本轮可放置区域 |
| 工具 TCP | 法兰至合理接触点 **248 mm**，平移 `[0,0,0.248]` m、旋转零；参考系/基座变换零，执行前核对实际 toolset |
| 夹爪开合轴 | **工具 X 轴**，不等于图像水平轴或基座 X 轴 |
| 夹指最低点 | 朝下抓取姿态下比 TCP 低 **20 mm**；姿态变化按工具方向换算，不恒为基座竖直向下 |
| 夹爪最大外宽 | **78 mm**；不是两指内侧净开度 |
| 未知工具外廓 | 指长、指厚、毫米内开度及线缆外廓无实测记录；历史 110 × 60/70 mm 携物外廓仅为规划假设 |
| 夹爪位置 / 力 | **0..1000 寄存器单位**，0 闭、1000 全开，未标定为毫米；力由 YAML 配置，默认 **20%**，不是牛顿数 |
| 夹爪动作 | move_gripper 单次变化最多 **500**；close_gripper 从初始化完成、全开空夹 1000 开始，目标 **0..999**，不自动回零 |
| 机械臂速率 | **1%..100%**，默认读取 YAML；10% 写作 **0.10**，不是 10 mm/s；按本轮授权与几何确定 |

物块/盒子/TCP 来源见[首次抓放](docs/records/2026-10-06/README.md#本次几何与流程)与[再次抓放](docs/records/2026-10-06/repeat-pick-place/README.md#本轮定位与执行)；夹指 20 mm、外宽 78 mm 为用户 2026-10-10 18:38 补充。历史接触/释放高度及寄存器位置不是本轮动作默认值。

## 共享控制参数

[config/control.yaml](config/control.yaml) 是 Web 与 agent 的默认参数源：`robot.sdk_rate` 为 0.01..1.0 比例（默认 0.10），`gripper.force_percent` 为 20..100 整数（默认 20）。比例不是绝对角速度，力不是牛顿数；开放范围不代表所有速率已实机验收。

Web 参数栏读取并保存该文件；仅持有控制权且空闲时可保存，保存不触发动作。agent 新规划省略 rate 时读取 YAML，显式 rate 仅用于本轮明确覆盖；已检查路线固定原速率，不受后续配置修改影响。夹爪在每次动作接收时读取力度。在线倍率仍固定为 1，不作为第二个速度旋钮。配置错误拒绝读取/动作，不静默回退。

查看生效值：`python3 -m scripts.hardware.control_settings`。系统 Python 和 SDK 虚拟环境均需 PyYAML：`.venv/bin/python -m pip install 'PyYAML>=6,<7'`。新代码需在设备空闲后重启已有 Web 服务才能加载。

## 实机任务入口

默认流程是 **skill → 状态与初始观察 → 终端生成 plan.json → run_plan 执行与审核 → 验收**。不要求先读本 README、完整 Schema 或落盘 build.py。

- `python3 -m scripts.hardware.sensor_client status` 检查占用与连接；只在设备空闲且所有权允许时显式连接缺失传感器。Web 状态不代替 SDK 实读状态。
- `python3 scripts/run_tool.py TOOL --adapter xmate` 从 stdin 接收 JSON，也可 `--args 参数文件`；不熟悉某个工具时用 `python3 scripts/validate_task_plan.py --adapter xmate --tool TOOL` 查询它，不读全量清单。
- 完整实机任务须提交 plan.json。短计划默认通过终端 heredoc 调用现有 PlanBuilder，自动补字段和校验；也允许直接写合规 JSON。复杂、需反复调试或复用的生成逻辑才保存脚本。生成与执行分开，避免重试生成时重复运动。
- 直接调用限初始观察、只读查询及明确调试，不代替完整任务的计划审核。错误、缺失证据或结果不明必须停止，不自动重发；重新观察后仅规划剩余工作。不对运行中的计划穿插手动动作。携物绑定本次实测 holding_position，最终用新图验收。

工具保护、Schema 和执行器不变；直接调用没有 plan 层自动审核。场景几何及路径仍需由现有算法/工具提供，本次精简没有新增通用抓取能力。最小生成与执行示例在 skill，复杂字段和几何格式查[计划参考](.agents/skills/embodied-agent/references/plan-format.md)。

正常执行少交流、不查 Git/源码或修改共享框架；目标是请求到验收300秒内，不因赶时间提速、跳保护或强杀。复用同帧信息，等待已有执行器而不反复扫描日志；完成后立即报告实际状态、证据和耗时，维护另做。

## 环境与连接

| 项目 | 当前配置 |
| --- | --- |
| 主机 | Ubuntu 22.04 x86_64；工作区 `/home/embotic/agent-as-policy` |
| 机械臂 | xMate SR4-C 六轴，`XMS4-R800-B4S1A2C`，IP `192.168.0.160` |
| SDK | xCoreSDK-Python 0.7.1 / Python 3.10 ABI，控制器 xCore 3.2.1；解释器 `.venv/bin/python`，扩展 `vendor/xCoreSDK-Python/Release/linux/` |
| 视觉 / 串口 | 系统 `python3`；numpy、scipy、OpenCV、PyYAML、pyrealsense2、pymodbus、pyserial |
| 相机 | D455，serial `141322252639`，RGB + 深度 Z16 均为 1280×720 / 15 Hz；共用 `config/camera_rgb.json`，当前关闭 RGB 自动曝光 |
| 夹爪通信 | `/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ0296VS-if00-port0`，9600/8N1，Modbus 地址 2 |

**SDK 与 OpenCV/RealSense 必须分进程。** SDK 断开后原生库仍留在进程中；`sdk_runtime.py` 提供双向导入保护及 `logs/robot-sdk.lock` 独占锁。任务 SDK 与网页机械臂连接不可并存，常驻相机/串口复用现有服务。SDK 连接/断开本身可能停止运动，只读查询也须设备空闲。

新机器用 `git clone --recurse-submodules`，已有克隆用 `git submodule update --init --recursive`。SDK 二进制、`.venv`、USB 驱动另行准备，不复制 Windows 环境；`python3 -m venv .venv` 不会安装官方扩展。视觉/串口依赖与 SDK 分开管理。

```bash
mkdir -p logs/runs logs/sdk captures
python3 -m pip install --user -r requirements-policy.txt
python3 -c 'import cv2, numpy, scipy, yaml, pyrealsense2, pymodbus, serial'
```

网页入口 `python3 scripts/web_control.py`，本机 `http://127.0.0.1:8080`；启动不连接硬件。已有 `agent-policy-web` 用户服务时只保留一个实例，管理方式见 [Web 控制台](docs/Web控制台.md)。了解项目或整理文档不需要连接设备。

相机入口默认同时开启 RGB 与深度。常驻 observe 的 `rgb.depth_path` 指向同组帧对齐到 RGB 的 16 位 `-depth.png`；`rgb.metadata.depth` 保存尺寸、原始深度帧时间戳、帧号和 `meters_per_unit`。像素深度（米）为 PNG 原始值乘此比例，0 表示无效，不能当作零距离；深度为相机光轴方向距离，配对帧时间戳仍应核对。独立拍照和标定也保存深度；不对空洞插值、不自动套用历史物块尺寸。标定采集帧率由 10 改为 15 Hz，旧采集会话的配置检查会要求新建会话。

## 标定与坐标

| 数据 | 本机工作路径 |
| --- | --- |
| 内参、配对样本 | `captures/calibration/session-01/samples.yaml` |
| 相机至基座外参 | `captures/calibration/session-01/solve-15-validate-5/result.yaml` |
| 桌面平面 | `captures/calibration/session-01/table-plane.yaml` |

[标定快照](docs/records/2026-10-06/calibration/)不自动启用；仅相机/基座安装、RGB 分辨率和桌面基准未变且复核后，才能恢复到上述路径。原始照片另行保留，快照不能重做图像检测。重新采集与变更判断见[标定指南](docs/标定.md)。

- `T_base_camera` 将相机坐标变为基座坐标，平移用米；TCP 为米/弧度，Jog 距离为毫米/度。先用外参映射，不能把屏幕“向右”当基座轴。
- RGB 为 `inverse_brown_conrady`，用 RealSense 官方 `rs2_deproject_pixel_to_point` 去畸变；不直接套普通 OpenCV Brown 模型。裁剪/缩放须同步内参，或保持原图像素坐标。
- 桌面 `n·p+d=0`，n 朝上，`n·p+d` 是离桌高度。RGB 射线须与已知高度平面求交；分别处理顶面、侧面、盒沿和盒底，不用整体色块中心代替顶面中心。
- 当前外参 15 组拟合、5 组独立验证，验证平移最大约 3.44 mm；桌面仍为暂定估计。TCP 对估计目标的小残差不等于实物定位精度。

## 抓放与执行边界

抓取前确认目的地及通行姿态可达，按当前图像检查整臂、夹指、物体和线缆净空；本项目没有完整碰撞模型，IK/FK 通过不证明无碰撞，MoveAbsJ 不保证笛卡尔直线。

接近时高位对齐并提前转腕，竖直下降；TCP 是接触点，物体底面和夹指最低点须按实际抓持几何推算。携物底面应高于沿途障碍并留量。放置需容纳夹指、物块及释放外廓，竖直下降、小量释放、竖直退出，狭窄盒内不自动全开。闭合接触不识别对象，释放反馈不证明放置成功。

运动要求 manual、idle、控制权无冲突；检查不切电源，执行按需上电，结束 stop → 等待 idle → 下电。不得自动切模式、清报警或回零。场景、工具、负载或状态变化后重新读取规划；已尝试路线不得通过改名、换 run_id 或删除尝试标记重放。软件停止不等于硬件急停，断串口不会停止夹爪，机械臂下电也不会松爪。细化限位、监测阈值及异常收尾见[工具契约](docs/工具与数据契约.md)。

## 脚本索引

从仓库根运行。子目录入口用 `python3 -m scripts.目录.模块`，SDK 换 `.venv/bin/python`；根目录入口继续用 `python3 scripts/入口.py`。脚本参数/示例见对应专题，不从历史任务推断。

| 入口 | 职责 |
| --- | --- |
| `scripts/run_tool.py` | 初始观察、只读查询及明确调试：JSON 参数调用工具 |
| `scripts/run_plan.py`、`validate_task_plan.py` | 完整任务执行/审核接回、静态校验和工具契约查询 |
| `scripts/web_control.py` | 常驻控制台 |
| `scripts/runtime/`、`scripts/adapters/` | 计划组装、Schema/条件计算、工具注册及子进程适配 |
| `scripts/hardware/check_connection.py`（SDK） | 只读机器人状态，`--output` 保存结构化结果 |
| `scripts/hardware/joint_route.py`、`grasp_orientation_ik.py`（SDK） | 连续 IK/FK、路线检查与受限执行、防重放 |
| `scripts/hardware/route_geometry.py` | SDK-free TCP 采样与关节路点整理 |
| `scripts/runtime/worker_process.py` | 中断转发、等待 SDK worker 收尾，不自动重试或强杀 |
| `scripts/hardware/check_gripper.py`、`grasp_close.py` | 夹爪读取/闭合，`--resident` 复用连接，动作需 `--execute` |
| `scripts/hardware/sensor_client.py`、`gripper_monitor.py` | 常驻设备客户端、新鲜反馈 |
| `scripts/hardware/sdk_runtime.py`、`web_hardware.py` | SDK 独占与原生库隔离、控制台底层操作 |
| `scripts/vision/capture.py`、`camera_settings.py` | RGB 采集和共用曝光配置 |
| `scripts/vision/block_top.py`、`placement_rgb.py`、`scene_batch.py` | 场景算法与同帧并行计算，接口见计划指南 |
| `scripts/calibration/` | calibration_capture 配对采集、calibration_solve 求解、table_plane 桌面估计 |

## 文件与维护

`config/` 保存 Schema/能力/现场配置，`web/` 为控制台；`captures/`、`logs/` 为本机输出，保留活动标定、任务照片/计划、运行记录及尝试标记。历史清理不使旧计划可重放；新任务生成新证据。关键照片、日志摘要与标定快照整理到 `docs/records/`，不批量提交逐帧日志、SDK 二进制、虚拟环境或凭据。

已完成任务的固定坐标临时脚本已删除；仅明确维护/历史追溯时可查提交 `87a3349`，不能用旧坐标或状态发起新动作。临时脚本用完删除。

项目测试已按用户要求删除，维护做语法、引用、差异检查，必要时按当前任务实机验证。重要代码改动简记于维护记录；纯文档整理不另建或追加记录。当前状态按需更新本文，实机结果保留证据。用户沟通及文档时间用 Asia/Hong_Kong，日志保留原时区。
