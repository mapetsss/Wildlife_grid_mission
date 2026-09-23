# H 题野生动物网格巡检归档清单

## 来源

- Git 提交：`41cf9563be804d201b163ad5f807736a939e4d90`
- 对应标签：`archive/h-wildlife-41cf956`
- 选择原因：该提交完成飞行任务与 `wildlife_vision` 强类型消息的集成，尚未混入 D 题嵌套工作空间。

## ROS 2 包

- `px4_offboard_mission`：9×7 网格航线、串口桥接、雷达定位验证和返航降落。
- `wildlife_vision`：逐格触发检测、动物计数、报告消息和串口报告备选节点。
- `mono_camera_capture`：RealSense D435 RGB/RGB-D 驱动。
- `ros2_yolos_cpp`：YOLOs-CPP/ONNX Runtime 推理适配器和五分类模型。

## 主要入口

```text
ros2 launch px4_offboard_mission radar_localization.launch.py
ros2 launch mono_camera_capture d435_rgb_only.launch.py
ros2 launch wildlife_vision wildlife_vision.launch.py
ros2 launch px4_offboard_mission wildlife_mission.launch.py
```

当前没有统一启动全部组件的总 launch；以上顺序应在 Linux/ROS 环境复核后固化。

## 主要数据流

```text
串口路线 -> /mission/route_upload -> wildlife_mission_node
到达网格 -> /vision/trigger -> wildlife_vision_node
识别报告 -> /vision/animal_report -> serial_route_bridge_node
飞行目标 -> /mavros/setpoint_position/local
```

## 模型

- 文件：`ros2_ws/src/ros2_yolos_cpp/models/drone_yolo26n_480x640.onnx`
- 类别：`monkey`、`kq`、`wolf`、`tiger`、`ele`
- 输入形状：`[1, 3, 480, 640]`
- SHA-256：`470F60CAE4CE5AAD59EB31FBF9F8DEBC384CE1081C65C5B8E7A3E0E879241CB9`

## 已知问题

- `serial_route_bridge_node` 已能回传动物报告；`wildlife_report_serial_node` 是备选实现，同一串口只能选择一个所有者。
- `vision.yaml` 中保留目标机绝对路径，但标准 launch 会用包路径覆盖。
- README、相机配置和测试日志中的分辨率描述存在漂移。
- 只有人工测试节点/工具，没有完整任务级自动回归测试。

## 验证状态

- 已确认包、模型和消息文件来自指定 Git 提交。
- 尚未执行 ROS 构建、推理、串口或实机飞行测试。

