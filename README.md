# Wildlife Grid Mission ROS 2

H 题野生动物网格巡检、YOLO 识别和串口报告任务的独立归档仓库。

## 来源

- 原仓库：`mapetsss/Detect_ws`
- 提取提交：`41cf9563be804d201b163ad5f807736a939e4d90`
- 归档原因：该提交完成 `px4_offboard_mission` 与 `wildlife_vision` 的强类型视觉报告集成，且尚未导入 D 题工作空间。

## 目录

- `ros2_ws/src/`：飞行任务、D435 驱动、YOLO 推理和野生动物视觉包。
- `support/`：相机安装、标定、采集、检测和位姿记录工具。
- `ARCHIVE_MANIFEST.md`：启动顺序、模型、接口和已知问题。
- `SOURCE_README.md`：原提交中的工作空间说明。

构建时进入 `ros2_ws/`。当前尚无单一总 launch，恢复运行前请先阅读 `ARCHIVE_MANIFEST.md`。

本仓库尚未在 ROS 2 Humble、PX4/MAVROS、D435 或实机环境重新构建验证。

