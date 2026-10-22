# RGB-D 默认采集变更验证

时间：2026-10-15 14:45–14:48 Asia/Hong_Kong。

- D455 141322252639 可枚举，USB 3.2；设备列出的1280×720深度帧率为5/15/30，RGB支持5/10/15/30。
- 空闲、无所有者、机器人断开时停止预览并重启 agent-policy-web，恢复相机和夹爪连接。
- RGB10 + 深度30 启动结果：`Sensor connection rejected: Frame didn't arrive within 2000`；observe 同样拒绝，无图像/深度输出。
- 修改为双15后显式启动结果：`Sensor connection rejected: Device disconnected. Failed to reconnect: xioctl(VIDIOC_S_FMT) failed, errno=5 Last Error: Input/output error2000`。
- 未执行硬件复位、强杀、机械臂运动或夹爪开合；停止重连。最后夹爪位置800，相机启动失败。
- 当前代码的双流实机验收和(593,603)深度读数尚未完成，不以离线PNG检查代替设备验收。

## 15:00 起：官方示例对照与固件升级

用户要求排查实现，并明确授权更新电脑驱动及相机固件。

- 官方 [align-depth2color.py](https://github.com/realsenseai/librealsense/blob/v2.58.3/wrappers/python/examples/align-depth2color.py) 使用同一 pipeline 开启 Z16/BGR8 双流，640×480 / 30 Hz，以 `rs.align(color)` 对齐。[Open3D 示例](https://github.com/isl-org/Open3D/blob/main/examples/python/reconstruction_system/sensors/realsense_pcd_visualizer.py) 使用同样流程。
- 绕过项目采集实现，以独立进程和项目控制权执行官方配置：系统 pyrealsense2 2.58.3.10794 在 `pipeline.start()` 报同样 `VIDIOC_S_FMT errno=5`。隔离安装到 `/tmp/agent-policy-rs253` 的2.53.1.4623也失败，未替换系统SDK。
- 原1280×720 / 10 Hz RGB单流成功出图。深度单流640×480 / 30 Hz也失败，故障不只发生于双流组合。
- 相机原固件5.13.0.50；[SDK2.58.3官方支持表](https://github.com/realsenseai/librealsense/releases/tag/v2.58.3) 的D455固件为5.17.3.10或更新。版本不符合当前支持表，但不是已证实的唯一根因。
- 备份：`captures/D455-141322252639-before-5.17.3.10.bin`，2097152字节，仅本机保留。
- 官方[更新数据库](https://librealsense.realsenseai.com/Releases/rs_versions_db.json)为D455推荐5.17.3.10；[下载文件](https://librealsense.realsenseai.com/Releases/RS4xx/FW/D4XX_FW_Image-5.17.3.10.bin) SHA256为 `4c347e3a18eac97b41a97947ef3b225bc1eaf77c68ffd87598736e3cebf3e4d2`。
- 首次固件更新进入DFU后因缺少权限失败，未开始写入。安装v2.58.3官方 `99-realsense-libusb.rules` 到 `/etc/udev/rules.d/`，重载并仅触发8086:0adb设备规则。确认DFU更新序列号124423062932与原相机一致，使用官方工具恢复模式完成升级；工具出现一次重枚举等待超时并继续更新，最终退出0，实读固件5.17.3.10。
- 固件升级后官方配置仍在等待帧时断连，项目双流和深度单流也失败。未据此宣称固件修复了采集。
- [官方问题13685](https://github.com/realsenseai/librealsense/issues/13685)建议以RSUSB后端排查此类内核冲突。主机为原生Linux、内核6.8.0-138；隔离构建SDK2.58.3 RSUSB后端作对照，未更改系统内核。

## RSUSB对照结果

- 源码tag `v2.58.3`，commit `dfd6aa91250f5c31521d72d627865417989bb4e7`；构建目录 `/tmp/agent-policy-librealsense-2.58.3/build-rsusb`，`FORCE_RSUSB_BACKEND=ON`、Release、Python3.10，未安装覆盖系统SDK。
- 初次导入发现ROS环境中的同名旧共享库被优先加载，报 `undefined symbol: rs2_get_firmware_min_version`。对照进程同时设置 `PYTHONPATH` 和 `LD_LIBRARY_PATH` 为构建的 `Release` 目录后正常导入，不影响原有进程环境。
- 640×480 / 30 Hz双流：`pipeline.start()`成功，实读固件5.17.3.10；等待帧报 `Frame didn't arrive within 5000`。
- 640×480 / 5 Hz深度单流：启动成功，仍报 `Frame didn't arrive within 5000`。不是只有高分辨率双流才失败。
- 深度传感器实读：Inter Cam Sync Mode=0、Output Trigger Enabled=0、自动曝光=1、曝光8500、Emitter Enabled=1；未设置为外部从机触发。
- 最后停止所有对照采集，未自动替换系统SDK，也未恢复项目双流。相机仍可枚举，故障根因尚未确认，不能将其确定归因为项目对齐代码、USB线或相机硬件损坏。
- 下一步需要独立主机上的官方RealSense Viewer深度单流交叉验证，以区分本机USB/驱动环境与相机深度模块；本次没有获得目标像素深度。

## 16:57 用户提供成功脚本后的本机复现

用户提供Windows采集脚本、教学记录及元数据。附件记录SDK2.59.1.11053、固件5.17.3.10；成功方式为640×480/15Hz先RGB后深度，双流仍超时，深度未对齐。

- 将SDK2.59.1.11053隔离安装到 `/tmp/agent-policy-rs259`，系统SDK未替换。将用户脚本原样复制至 `captures/rgbd-sequential-comparison/capture_camera.py`，持有现有控制权运行，未执行附件中的硬件重置命令。
- 本机原样脚本成功：RGB和深度分别读取15帧，每次超时10秒，深度有效比例0.83720703125；原始深度单位0.0010000000474974513米。图片与元数据保存在 `captures/rgbd-sequential-comparison/captures/`。
- 使用同一隔离SDK测试项目1280×720/15Hz同时双流：启动成功，取帧仍报 `Device disconnected. Failed to reconnect: xioctl(VIDIOC_S_FMT) failed, errno=5 Last Error: Input/output error10000`。没有保存双流图片，也没有目标像素深度。
- 当前证据确认本机可以顺序采集RGB和深度，不能继续断言本机深度单流不可用；未控制设备重插状态等全部变量，因此不能单凭本轮断定SDK升级是唯一修复因素。顺序结果不等同于同步对齐RGB-D，未将项目默认双流静默替换为顺序采集。
