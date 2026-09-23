# Intel RealSense D435 使用说明

`mono_camera_capture` 保留了原包名，但驱动实现已经完全改为 `librealsense2`。
它不会访问 `/dev/video*`，也不依赖 `usb_cam`。

## 使用前检查

先确认系统能识别相机：

```bash
rs-enumerate-devices
```

如果看不到 D435，应先检查 USB 3 连接、librealsense2 安装和 udev 权限。

## 编译

```bash
cd ~/Detect_ws
./tools/install_librealsense_d435.sh

source /opt/ros/humble/setup.bash
colcon build --packages-select mono_camera_capture ros2_yolos_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## RGB-D 模式

```bash
ros2 launch mono_camera_capture d435_rgbd.launch.py
```

发布：

```text
/camera/color/image_raw                         bgr8
/camera/color/camera_info                       D435 出厂内参
/camera/aligned_depth_to_color/image_raw        16UC1，毫米
/camera/aligned_depth_to_color/camera_info      与对齐深度匹配的内参
```

深度已经通过 librealsense 对齐到彩色坐标系，因此检测框中心 `(u,v)` 可以直接查询同位置深度。

## RGB-only 模式

```bash
ros2 launch mono_camera_capture d435_rgb_only.launch.py
```

该模式不启动深度传感器和红外投射器，只发布彩色图像及其 `CameraInfo`，适合：

- 只需要二维检测；
- 希望降低 USB 带宽和计算负载；
- 不需要目标真实距离。

## 检查话题

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/color/camera_info --once
```

保存一张彩色图：

```bash
python3 tools/save_image_once.py --count 1 --timeout 5
```

## 与 YOLO 一起启动

RGB-D：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/classes.txt
```

RGB-only：

```bash
ros2 launch ros2_yolos_cpp detector_rgb_only.launch.py \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/classes.txt
```

然后激活检测器：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

如果相机已单独启动，检测器应使用 `start_camera:=false`，避免重复占用 D435：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  start_camera:=false \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/classes.txt
```

## 多台 D435

在配置文件的 `serial_no` 中写入目标相机序列号：

```yaml
serial_no: "123456789"
```

序列号留空时连接 librealsense 找到的第一台设备。
