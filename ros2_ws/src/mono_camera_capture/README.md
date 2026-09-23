# mono_camera_capture（RealSense D435 驱动）

包名和可执行文件名为兼容旧工程而保留；内部已经不再使用普通单目 USB 摄像头，
而是直接通过 `librealsense2` 驱动 Intel RealSense D435。

## 功能

- 启动 D435 彩色 BGR8 流；
- 可选启动 Z16 深度流；
- 将深度对齐到彩色光学坐标系；
- 把深度统一转换为 ROS `16UC1` 毫米格式；
- 从 D435 流 profile 发布真实 `CameraInfo`；
- 支持序列号选择、红外投射器和激光功率设置；
- USB 采集失败后自动停止并重连；
- 支持 RGB-D 与 RGB-only 两套配置。

## 依赖

```text
ROS 2 rclcpp/rcl_interfaces/sensor_msgs
Intel librealsense2 + development headers
```

本包不依赖 `usb_cam`、OpenCV `VideoCapture` 或 `/dev/video0`。

安装 SDK：

```bash
./tools/install_librealsense_d435.sh
```

请优先使用该脚本安装的 ROS 软件源版本。某些独立安装的 librealsense 2.58
构建会导出其自带的 Fast DDS 符号，并与 ROS 2 Humble 默认 RMW 冲突；本包在
构建时会固定实际选中的 SDK 运行库目录。

## 启动

RGB-D：

```bash
ros2 launch mono_camera_capture d435_rgbd.launch.py
```

RGB-only：

```bash
ros2 launch mono_camera_capture d435_rgb_only.launch.py
```

历史入口 `mono_camera.launch.py` 仍可使用，默认等价于 RGB-D 模式。

## 配置

- `config/d435_rgbd.yaml`
- `config/d435_rgb_only.yaml`

RGB-D 默认话题：

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
```
