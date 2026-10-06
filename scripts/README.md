# 脚本索引

从项目根目录运行。`SDK` 表示 `.venv/bin/python`，`系统` 表示 `python3`。先读 [项目 README](../README.md)；“只读 SDK”也会连接控制器，只能在设备空闲且 SDK 无其他持有者时运行。

## 基础能力

| 入口 | 解释器 | 行为与边界 |
| --- | --- | --- |
| `check_connection.py` | SDK | 读取模式、电源、关节、工具、限位、历史报警，不上电/运动 |
| `check_gripper.py` | 系统 | 只读初始化、夹持状态、位置，不自动初始化 |
| `capture.py` | 系统 | 独占 D455 RGB 拍一张照片；网页持有相机时改用网页采集 |
| `web_control.py` | 系统 | 控制台服务；启动不连接硬件，相机/视觉独立进程 |
| `sdk_runtime.py` | SDK/库 | SDK 加载、独占锁、与视觉库的进程隔离守卫 |
| `web_hardware.py` | 库 | 机器人/夹爪适配器；调用方法可能产生物理动作 |
| `camera_settings.py` | 系统/库 | 统一 RGB 设置 |

## 标定与离线视觉

| 入口 | 解释器 | 行为与边界 |
| --- | --- | --- |
| `calibration_capture.py` | 系统 | 配对采样；默认复用网页，`--direct` 独占设备；`--check-image` 离线检测 |
| `calibration_solve.py` | 系统 | 离线求解/验证，参数见 `--help` |
| `table_plane.py` | 系统 | 已知标定板离桌高度时估计平面，结果仍需独立复核 |
| `block_top.py` | 系统 | 离线顶面四边检测；目前专用于已知尺寸红色长方体，尺寸常量和 ROI 不是通用默认值 |
| `placement_rgb.py` | 系统 | 两张图的盒沿边线候选，需传入盒沿高度/ROI；候选不等于已确认盒口 |
| `placement_layout.py` | 系统 | 本次盒内空位离线分析：假定候选 0 已经人工核对、障碍高度 0..50 mm、固定外廓尺寸；不输出运动命令 |
| `plan_grasp_orientation.py` | 系统 | 顶面+朝向规划；`--vision-only` 不接 SDK，否则调用独立 SDK 子进程核对起点/IK；必须新建 `--output` 目录 |
| `grasp_orientation_ik.py` | SDK | 只读连续 IK/FK 检查；输入带起点与矩阵采样路线 |

离线顶面示例：`python3 scripts/block_top.py --image captures/example.jpg --output captures/new-top-check`。先修改或参数化当前对象的尺寸/高度及 ROI，不能直接对新对象沿用硬编码常量。

## 受限动作工具

以下都不是通用碰撞规划器。先检查脚本限定的初始状态、工具和空间范围，不要靠扩大边界绕过未通过的规划。

| 入口 | 解释器 | 限制 |
| --- | --- | --- |
| `jog_once.py` | SDK | 单关节点动，`--execute` 才运动且有终端确认；旧 `--restore-initial` 不是任意现场的安全回位 |
| `orientation_joint_step.py` | SDK | 新指定关节路线的单段执行；拒绝已归档朝向路线及其复制版本 |
| `cartesian_step.py` | SDK | 旧高位对齐小范围单轴平移；可核对 `--expected-state`，1%～10% 速率 |
| `gripper_step.py` | 系统 | 空载到位状态的小步开合；不适用于当前夹持着物体时的释放 |
| `grasp_close.py` | 系统 | 必须从全开 1000 开始，20% 力逐步闭合；不能在已夹持时重跑 |
| `grasp_vertical.py` | SDK | 上下文指定的固定 XY 柱，接触高度 20..171 mm；携物检查只用于抬升 |
| `holding_observe.py` | SDK | 携物 J1 小段观察、每阶段最多 5°，其他关节固定；当前引用本次抓取上下文和 110..130 mm 高度范围 |

新任务按所需语义建立新上下文和路线执行器，复用这些工具的状态读取、限位、走廊、停止及反馈模式。需要连续执行时使用一次计划串联阶段，不反复启动单毫米命令。库函数可供复用，但固定场景的命令行工具不能伪装成通用接口。

## 历史归档

`archive/` 保留以下已完成实验的实现，**命令行入口全部拒绝运行**，不连接硬件、不覆盖历史数据：

- `correct_grasp_opening_axis.py`：纠正工具开合轴的单次实验，原逻辑会写旧计划目录。
- `search_grasp_joint_routes.py`：该次朝向路线搜索，写固定输出目录。
- `placement_ik.py`：2026-10-06 特定盒口/外廓的 471 点运动学检查。
- `placement_execute.py`：该计划的连续搬运、释放与退出；反馈守卫仍有离线测试覆盖。
- `placement_release.py`：该次已知夹持位置 412→462 的释放实现。

这些源文件保留用于理解已成功运行的代码，不能删除“归档拒绝”后直接重用其坐标或夹爪状态。归档内的 `main` 实现是历史参考，不是公开动作 API。
