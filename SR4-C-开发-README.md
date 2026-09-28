# 珞石 xMate SR4-C 开发 README

更新日期：2026-09-28。后续开发与运行迁移到 Ubuntu，Windows 仅作为 SSH 远程访问平台。USB 夹爪和 USB 相机后续直接连接 Ubuntu。RobotAssist 仅用于可选的查看、配置和厂家辨识，无需上传代码至控制器或登录控制器 Linux。此前运动验证在 Windows 完成，迁移文件不等于 Ubuntu 运动链路已经通过验证。

## 1. 设备与环境

| 项目 | 已确认信息 |
| --- | --- |
| 机械臂 | 珞石 xMate SR4-C，SDK 机型 `XMS4-R800-B4S1A2C`，6 轴 |
| 控制器 | `192.168.0.160`，xCore `3.2.1.C234.20260506` |
| 开发主机 | `embotic@192.168.0.10`，主机名 `embotic`，Ubuntu 22.04.5 LTS，x86_64；项目 `/home/embotic/gpt-for-policy` |
| 网络 | Ubuntu 与机械臂连接同一台路由器、位于 `192.168.0.0/24`；Ubuntu 经 `enp2s0` 访问控制器，迁移时两次 ping 成功。Windows 历史测试地址为 `192.168.0.2` |
| Python | Ubuntu 系统 Python 3.10.12，项目虚拟环境待创建；历史 Windows `.venv` 为 Python 3.12.12，不复制到 Ubuntu |
| SDK | 官方 xCoreSDK-Python `v0.7.1`；源码提交 `4f7979b2643429b2a0c0e75b6ddd4cbb0ec061c9`，以 Git 子模块固定；历史 Windows 库已验证，Linux x86_64 库及脚本加载适配待完成 |
| 外设 | USB 夹爪与 USB 相机均接 Ubuntu，型号、驱动、设备权限和反馈接口待验证；首版相机固定在工作区外部，相对桌面/基座静止 |
| 工具 | 自制夹爪；用户已确认现有 `tool2` 参数对应当前夹爪 |
| 默认速率 | `scripts/jog_once.py` 中 `DEFAULT_JOG_RATE = 0.10`，即 10% |

**已执行的运动测试全部使用 1%。5% 未实际采用；10% 仅完成代码设置和静态检查，尚未实机验证。** 本文状态是测试快照，不代表当前实时状态。

官方 SDK 文档要求 xCore >= 3.2.1。历史 Windows 环境使用 Python 3.8–3.12 对应的扩展；Ubuntu 需另行核对 Linux Release 的架构、Python ABI 和动态库依赖。仅克隆源码不包含运行所需的二进制库。已验证的功能不代表所有功能均已授权。软件版本和历史日志日期不能用于判断出厂日期。

## 2. 项目结构

```text
.
├── SR4-C-开发-README.md
├── scripts/
│   ├── check_connection.py       # 状态、工具、软限位、历史报警查询
│   └── jog_once.py               # 单轴点动、J6 绝对目标、初始角度恢复
├── config/
│   └── initial_joint_pose.json  # 初始角度及来源，不依赖日志文件运行
├── docs/
│   └── Astra-as-Policy-SR4-C实现方案.md
├── logs/
│   ├── runs/                    # 检查与运动 JSONL 原始记录
│   └── sdk/                     # SDK 原生日志工作目录
│       ├── _rokae_log_/
│       └── logs/
├── vendor/
│   ├── xCoreSDK-Python/
│   └── xCoreSDK-Python-0.7.1-win.zip
└── .venv/
```

原有日志全部保留，包括预览、取消和验证失败记录。脚本按自身位置定位根目录，新 JSONL 写入 `logs/runs/`；加载 SDK 前切换到 `logs/sdk/`，收拢按当前目录生成的原生日志。当前脚本仍加载 `vendor/xCoreSDK-Python/Release/windows/` 并调用 `os.add_dll_directory`，尚不能直接在 Ubuntu 运行。目录树中的 Windows ZIP 和 `.venv` 为历史本机资源；迁移保留 SDK 文件及日志，不复制 Windows `.venv`。Git 保存代码、配置、文档和历史 JSONL，忽略虚拟环境、SDK 二进制/ZIP、原生日志及凭据文件。

## 3. 运行命令

从 Windows 进入后续开发主机：

```powershell
ssh embotic@192.168.0.10
```

登录后进入 `/home/embotic/gpt-for-policy`。该目录是后续开发工作副本；仓库为私有仓库 `Flowhist/gpt-for-policy`，SDK 子模块版本已固定。重新克隆时使用 `git clone --recurse-submodules`；二进制依赖需单独准备。SSH 使用公钥登录，密码和私钥不写入项目。

**下面仅保留历史 Windows 操作命令，不能直接在 Ubuntu 执行。** 下一步先准备匹配的 Linux SDK、创建 Ubuntu 虚拟环境并适配加载逻辑，再在现场确认机器人空闲后复验连接和运动。

历史 PowerShell 命令从 Windows 项目根目录执行。本机 `python3` 指向 WindowsApps 别名，曾无输出退出；系统另有与 SDK 不匹配的 Python 3.14，因此明确使用项目解释器。

```powershell
# 状态及配置查询，不使能、不切模式、不清报警
.\.venv\Scripts\python.exe .\scripts\check_connection.py

# 只预览 J6 正向 0.1 度
.\.venv\Scripts\python.exe .\scripts\jog_once.py --joint 6 --delta-deg 0.1

# 实际执行，终端还需输入大写 EXECUTE
.\.venv\Scripts\python.exe .\scripts\jog_once.py --joint 6 --delta-deg 0.1 --execute

# J6 到绝对 0 度的预览；执行需追加 --execute
.\.venv\Scripts\python.exe .\scripts\jog_once.py --joint 6 --target-deg 0

# 恢复指定关节到首次测试前的角度，默认仍只预览
.\.venv\Scripts\python.exe .\scripts\jog_once.py --joint 6 --restore-initial
```

`check_connection.py` 查询机型、版本、上下电、操作模式、运行状态、关节、末端/法兰位姿、基坐标系、工具组、工具列表、软限位及最近十条警告/错误日志。关节位置原始返回 12 项，本体使用前 6 项。

**所有脚本只在机器人空闲、无人运行其他任务时使用。** 官方文档注明 Robot 初始化会进行运动重置，断开前会停止运动，因此连接查询并非完全无副作用。官方示例可能含运动或参数修改，不要直接运行整份示例。

### 点动脚本边界

- 仅匹配的六轴机型、手动模式、空闲状态可执行；不会自动切到自动模式。
- 相对位移：J1–J5 为 0.01–0.1 度，J6 为 0.01–5 度；正负号表示方向。
- `--target-deg` 仅支持 J6，距当前位置最多 20 度。恢复模式行程上限为 J1–J5 各 1 度、J6 30 度；绝对目标差小于 0.01 度时跳过运动。
- 默认 SDK Jog 速率 10%，与 HMI 程序速度无关。修改速率只改 `DEFAULT_JOG_RATE`，不要修改角度阈值中的 `0.01`。
- 执行前复查状态与位置；本机已验证 SDK 手动上电成功，但硬件若要求实体使能，不得绕过。
- 运动中检查各轴是否超出起点到目标的区间加 0.05 度余量；到位阈值 0.005 度，随后调用 `stop()`。
- 停止后五次采样，间隔 0.2 秒；位置误差上限 0.01 度、波动上限 0.002 度。保留速度记录，不以单个速度读数判定静止。
- 相对运动超时 10 秒，普通绝对目标 40 秒，恢复模式 60 秒；失败不重试、不自动回位。
- 运动退出或 Ctrl+C 后依次尝试停止、下电、核对下电状态、断开。网络阻塞或进程强制结束时，软件清理和超时检查不能替代现场急停。

脚本只检查关节范围，不检测障碍物、夹指、线缆和自碰撞。当前软限位接口返回关闭，代码未修改该配置。一次运动成功不等于其他路径已验证。

### 初始角度

“初始”是第一次逐轴测试前的角度，不是工厂 Home、全零姿态或编码器零点。

| J1 | J2 | J3 | J4 | J5 | J6 |
| --- | --- | --- | --- | --- | --- |
| -26.484764° | -2.738022° | -109.252522° | 0.755241° | -70.788387° | -24.690454° |

精确值见 [配置](config/initial_joint_pose.json)，来源为 [首次逐轴测试日志](logs/runs/jog-20260928-170224-802659.jsonl)。恢复需逐轴指定，不代表任意起点到该姿态都可无碰撞到达。

## 4. 工具与配置

- `tool2`：TCP 相对法兰 `[0,0,248] mm`，旋转为零；负载 `1.6 kg`，重心 `[100,100,200] mm`，SDK 惯量三项为零。HMI 截图显示当前选用 `tool2`，与 SDK 读取一致；用户确认对应当前夹爪，本次未重新辨识或修改。
- SDK 工具组与 RL 工程所选工具在接口语义上不同，后续修改不能假定自动同步。保存的 `tool1` 同为 248 mm，但绕 Z 旋转 45 度、负载为零，不要误选。
- 基坐标系相对世界坐标系六项均为零，实物安装方向未独立测量。
- 软限位：`enabled=false`；返回范围 J1 ±165°、J2 ±115°、J3 [-165°,140°]、J4 ±115°、J5 ±165°、J6 ±115°。不能据此推断其他安全限制关闭。
- 历史日志含 `13017`（未选任务）和 `50003`（MoveAbsJ 目标点过近），时间标记 2021-04-02；不能据此判断当前活动报警，控制器时钟未验证。

原始证据：[连接检查](logs/runs/connection-check.jsonl)、[配置检查](logs/runs/configuration-check-20260928-160540.jsonl)。

## 5. 实机测试记录

2026-09-28，UTC+8；以下运动均使用手动模式、SDK 速率 1%。

| 测试 | 反馈结果 | 日志 |
| --- | --- | --- |
| J1 +0.1° | +0.100113°；单次速度判据失败，停止下电后另行只读复查位置稳定 | [17:02](logs/runs/jog-20260928-170224-802659.jsonl) |
| J2 +0.1° | +0.100010° | [记录](logs/runs/jog-20260928-170340-988744.jsonl) |
| J3 +0.1° | +0.100044° | [记录](logs/runs/jog-20260928-170343-445434.jsonl) |
| J4 +0.1° | +0.099989° | [记录](logs/runs/jog-20260928-170345-899778.jsonl) |
| J5 +0.1° | +0.100044° | [记录](logs/runs/jog-20260928-170348-355925.jsonl) |
| J6 +0.1° | +0.100017° | [记录](logs/runs/jog-20260928-170350-819538.jsonl) |
| J6 +5° | +5.000002° | [记录](logs/runs/jog-20260928-170649-952676.jsonl) |
| J6 到 0° | 停止后约 -0.000014°，运动约 28 秒 | [记录](logs/runs/jog-20260928-170830-530754.jsonl) |
| 六轴逐轴恢复 | 六次各自到位验证及下电通过；日志时间前缀 171155、171158、171200、171203、171205、171208 | [末轴记录](logs/runs/jog-20260928-171208-157675.jsonl) |

J1 复查时，下电且位置基本稳定，速度仍偶发 ±0.05–0.06°/s；因此改用连续位置稳定性判据，原始失败日志保留。逐轴恢复后没有额外统一复测最终下电状态下的全部关节，不应将各次到位反馈理解为下电后角度仍精确保持原值。

16:57 的预览和 17:00 输入小写 `execute` 的取消记录均未发送运动。最后一次恢复日志确认下电、断开。

尚未验证：10% 实机运动、SDK 拖动、夹爪开合、笛卡尔运动、多点轨迹、实时力控、视觉闭环、自动负载辨识、断网时实际停止行为。

## 6. 后续开发

后续链路：`Windows -> SSH -> Ubuntu Python -> xCore SDK -> 同路由器局域网 -> 控制器 -> 机械臂`；相机采集、夹爪控制、模型调用和闭环程序均在 Ubuntu 运行。首版相机采用外部固定视角，腕部相机作为后续增强。历史 Windows Wi-Fi 非实时点动已验证；Ubuntu 目前仅验证 SSH 和到控制器的网络连通。先完成 Linux SDK 加载适配和外设接入，再验证 TCP 小步运动并接入模型空跑。后续将连接、状态、运动逐步封装为适配模块，明确单位、坐标系、超时、结果验证和日志；断线后不得自动重发未确认完成的运动。

SDK 支持 `enableDrag` / `disableDrag`。本地接口中 `enable_drag_button=True` 表示无需按住末端按钮；后续实现应保留按键条件。当前未创建拖动脚本，也未开启拖动。SDK 0.7.1 未找到公开的自动负载辨识启动接口，`loadIdentify` 是状态枚举，`calibrateForceSensor()` 不是负载辨识。

[Astra-as-Policy 实现方案](docs/Astra-as-Policy-SR4-C实现方案.md) 保留在 `docs/`，方案不等于已实现功能。Ubuntu 迁移不要求引入 ROS 2；首版继续使用 Python SDK，不修改控制器原厂系统。未来如需 ROS 2/MoveIt，再核对其版本和集成要求。

## 7. 官方资料

- [Python SDK](https://github.com/RokaeRobot/xCoreSDK-Python) · [v0.7.1 Release](https://github.com/RokaeRobot/xCoreSDK-Python/releases/tag/v0.7.1)
- [SDK 与 HMI 对应](https://docs.rokae.com/docs/SDK/sdk_assist/)
- [系统连接](https://docs.rokae.com/docs/xCore/机器人系统构成及连接/) · [基础操作](https://docs.rokae.com/docs/xCore/控制系统基础操作/)
- [工具与负载辨识](https://docs.rokae.com/docs/xCore/编程/工具列表/)
- [C++ SDK](https://github.com/RokaeRobot/xCoreSDK-CPP) · [ROS 2 软件栈](https://github.com/RokaeRobot/rokae_ros2)
- [软件下载中心](https://www.rokae.com/cn/service/11/2.html?cid=11) · [HMI 包直链](https://static.rokae.com/Downloads/HMI%20%E8%BD%AF%E4%BB%B6-v3.1.3.zip)

HMI 包中 PC 软件为 `RobotAssist_5.0.12.0557.zip`，本次已用于查看工具配置。不要将控制器升级文件当作 PC 安装程序。RobotAssist 不支持多个 HMI 客户端同时连接，该限制不能直接推断为所有 SDK 连接的限制。
