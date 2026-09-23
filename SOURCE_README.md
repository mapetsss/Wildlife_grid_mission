# Detect_ws：Intel RealSense D435 视觉工作空间

本工作空间的视觉链路已经适配 Intel RealSense D435，不再通过
`/dev/video*`、OpenCV `VideoCapture` 或 `usb_cam` 读取普通 USB 摄像头。

提供两种运行模式：

| 模式 | 相机流 | 检测输出 |
|---|---|---|
| RGB-D | D435 彩色 + 对齐深度 | 检测框、像素偏移、三维偏移、真实深度和直线距离 |
| RGB-only | 仅 D435 彩色 | 检测框、类别、置信度、像素偏移 |

## 视觉包

- `mono_camera_capture`：保留历史包名以兼容已有脚本，内部已改为原生
  `librealsense2` D435 驱动。
- `ros2_yolos_cpp`：YOLO ONNX 推理；RGB-D 模式使用对齐深度和 D435
  `CameraInfo` 进行三维反投影。

## 默认话题

```text
/camera/color/image_raw
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw
/camera/aligned_depth_to_color/camera_info
```

深度图编码为 `16UC1`，单位毫米，并已对齐到彩色图像。RGB-only 模式只发布前两个话题。

## 编译

系统需要 Intel librealsense2 开发包和 ROS 2 Humble/Jazzy：

```bash
cd ~/Detect_ws
./tools/install_librealsense_d435.sh

source /opt/ros/humble/setup.bash
colcon build --packages-select mono_camera_capture ros2_yolos_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

安装脚本固定使用 ROS 软件源中的 librealsense 构建，并校验其不会与 ROS 2
Humble 的 Fast DDS 发生符号冲突。编译和运行前都应先 source 对应 ROS 环境；
本包也会把构建时选中的 SDK 路径写入运行时搜索路径，避免误加载 `/usr/local`
中的同名库。

## 只启动相机

RGB-D：

```bash
ros2 launch mono_camera_capture d435_rgbd.launch.py
```

RGB-only：

```bash
ros2 launch mono_camera_capture d435_rgb_only.launch.py
```

如果日志反复出现：

```text
RealSense frame acquisition failed: Frame didn't arrive within 1000
RealSense pipeline is stopped; reconnecting to the first D435
```

说明 D435 已被 librealsense 打开，但实际帧流没有在 1 秒内到达。先确认没有其他
RealSense 程序占用相机，然后重新插拔 USB 3 数据线，并用以下命令判断相机本身
是否稳定出图：

```bash
rs-enumerate-devices --compact
ros2 launch mono_camera_capture d435_rgb_only.launch.py
ros2 topic hz /camera/color/image_raw
```

如果 RGB-only 稳定而 RGB-D 不稳定，再检查 USB 带宽、供电和深度流；必要时先用
RGB-only 完成普通 AVI 录制。`frame_timeout_ms` 可以临时调大用于慢启动排查，但
不能修复 USB 链路或相机固件没有持续出帧的问题。

## 录制与回放视频

开始录制前，先在一个终端启动 RGB-D 或 RGB-only 相机；以下录制命令在另一个
已经执行 `source /opt/ros/humble/setup.bash` 和 `source install/setup.bash` 的
终端运行。

### 录制为普通 AVI 视频

普通视频只保存 D435 彩色画面，不包含深度、相机内参或 ROS 时间戳。先安装
ROS 2 的 `image_view` 工具：

```bash
sudo apt install ros-${ROS_DISTRO}-image-view
mkdir -p recordings
```

将 `/camera/color/image_raw` 以 30 FPS 录制为 MJPG/AVI：

```bash
ros2 run image_view video_recorder --ros-args \
  -r image:=/camera/color/image_raw \
  -p filename:="$(pwd)/recordings/d435_color.avi" \
  -p codec:=MJPG \
  -p fps:=30.0 \
  -p encoding:=bgr8 \
  -p qos_overrides./camera/color/image_raw.subscription.reliability:=best_effort
```

按 `Ctrl+C` 停止并关闭视频文件。`fps` 应与相机配置中的 `color_fps` 保持一致；
如果修改了输出文件名，再次录制前应避免使用已经存在的重要文件。

录制固定时长时，用 `timeout` 发送 `SIGINT`，例如录制 60 秒：

```bash
timeout --signal=INT --kill-after=10s 60s \
  ros2 run image_view video_recorder --ros-args \
  -r image:=/camera/color/image_raw \
  -p filename:="$(pwd)/recordings/d435_color_60s.avi" \
  -p codec:=MJPG \
  -p fps:=30.0 \
  -p encoding:=bgr8 \
  -p qos_overrides./camera/color/image_raw.subscription.reliability:=best_effort
```

这里的 `60s` 是录制总时长，可改成 `30s`、`5m` 等。正常到时后 `timeout`
可能返回退出码 `124`，不代表录制失败；确认 AVI 文件能够正常播放即可。

### 录制 RGB-D 数据（推荐）

需要保留深度、内参，或者希望以后用新 YOLO 模型重复推理时，应使用
`rosbag2`。RGB-D 录制命令：

```bash
mkdir -p recordings
ros2 bag record \
  -o "recordings/d435_rgbd_$(date +%Y%m%d_%H%M%S)" \
  --compression-mode file \
  --compression-format zstd \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/aligned_depth_to_color/camera_info
```

只录制 RGB：

```bash
ros2 bag record \
  -o "recordings/d435_rgb_$(date +%Y%m%d_%H%M%S)" \
  --compression-mode file \
  --compression-format zstd \
  /camera/color/image_raw \
  /camera/color/camera_info
```

手动停止 rosbag 同样按 `Ctrl+C`。固定录制 60 秒的 RGB-D 数据：

```bash
timeout --signal=INT --kill-after=10s 60s \
  ros2 bag record \
  -o "recordings/d435_rgbd_60s_$(date +%Y%m%d_%H%M%S)" \
  --compression-mode file \
  --compression-format zstd \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /camera/aligned_depth_to_color/image_raw \
  /camera/aligned_depth_to_color/camera_info
```

不要把 `ros2 bag record --max-bag-duration 60` 当作总时长限制；该参数只会每
60 秒切分一个新 bag，录制进程仍会继续运行。需要自动结束时应使用上面的
`timeout --signal=INT`。

如果还要保存当前 YOLO 结果，可在命令末尾加入：

```text
/yolos_detector/detections
/yolos_detector/offset_px
/yolos_detector/offset_m
/yolos_detector/distance_m
/yolos_detector/timing
```

停止后检查和回放，其中 `<bag目录>` 是 `recordings/` 下本次生成的目录：

```bash
ros2 bag info <bag目录>
ros2 bag play <bag目录>
```

用录制数据重新测试 YOLO 时，先以 `start_camera:=false` 启动并激活检测器，
再执行 `ros2 bag play`。播放期间可运行可视化测试工具，它会定期触发检测服务：

```bash
python3 tools/test_yolo_camera_visual.py
```

## 启动 YOLO

RGB-D：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py
```

RGB-only：

```bash
ros2 launch ros2_yolos_cpp detector_rgb_only.launch.py
```

两个入口默认使用已经随包安装的 YOLO26n 五类别模型，固定输入为 `640×480`，
与当前 D435 彩色流分辨率一致。

检测器是生命周期节点，启动后执行：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

推理由 `/yolos_detector/detect` 服务按需触发。可使用：

```bash
python3 tools/test_yolo_camera_visual.py
```

## 更换 YOLO 模型

检测器直接读取 ONNX 文件。通过 launch 参数临时更换模型不需要重新编译工作空间；
若替换包内默认模型，则需要重新构建以更新 `install/`。模型必须是当前 YOLOs-CPP
支持的 ONNX 检测模型，不能把训练得到的 `.pt` 文件直接传给 `model_path`；应先用
对应训练框架导出为 `.onnx`。

当前默认部署文件位于：

```text
src/ros2_yolos_cpp/models/drone_yolo26n_480x640.onnx
src/ros2_yolos_cpp/models/classes.txt
src/ros2_yolos_cpp/models/model_manifest.yaml
```

模型类别依次为 `monkey`、`kq`、`wolf`、`tiger`、`ele`。清单记录了导出形状、
训练指标和 SHA-256。`src/models/` 下的单类别 `robocup` 模型是旧版遗留文件，
当前 launch 不再使用。

`classes.txt` 每行一个类别，行号从 0 开始，顺序必须与模型输出类别 ID 完全
一致。例如三类别模型应写成：

```text
person
ball
goal
```

### 临时切换模型

把新模型和对应类别文件放在任意位置，通过绝对路径启动即可：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  model_path:="$(realpath /path/to/new_model.onnx)" \
  labels_path:="$(realpath /path/to/new_classes.txt)" \
  use_gpu:=false
```

仅使用 D435 RGB 时，将启动文件换成：

```bash
ros2 launch ros2_yolos_cpp detector_rgb_only.launch.py \
  model_path:="$(realpath /path/to/new_model.onnx)" \
  labels_path:="$(realpath /path/to/new_classes.txt)" \
  use_gpu:=false
```

使用仓库中的默认模型无需再传路径：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py
```

默认 `yolo_version:=v26`。临时模型来自其他系列时，可在 launch 命令中传入
`yolo_version:=auto`，也可明确指定 `v7`、`v8`、`v10`、`v11`、`v26` 或 `nas`。

模型切换后需要重新启动检测器，并重新执行生命周期切换：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

如果 `configure` 失败，应依次检查：ONNX 路径是否存在、类别文件是否存在、
类别顺序是否匹配，以及 `use_gpu:=true` 时 ONNX Runtime GPU/CUDA 是否已经安装。

查看完整相机使用说明：[docs/camera_usage_zh.md](docs/camera_usage_zh.md)。
