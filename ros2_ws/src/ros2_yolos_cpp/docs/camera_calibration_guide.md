# RealSense D435 标定与内参说明

D435 在设备中保存了彩色、左右红外相机的出厂标定和外参。当前驱动直接从
librealsense stream profile 读取当前分辨率对应的内参，并发布：

```text
/camera/color/camera_info
/camera/aligned_depth_to_color/camera_info
```

因此正常使用时不需要运行 ROS `camera_calibration`，也不需要在 YOLO YAML 中手工填写
`fx/fy/cx/cy`。

## 验证内参

```bash
ros2 topic echo /camera/color/camera_info --once
```

应检查：

- `width/height` 与彩色图一致；
- `k[0]` 和 `k[4]` 为非零焦距；
- `k[2]` 和 `k[5]` 是合理的主点；
- `frame_id` 为 `camera_color_optical_frame`。

## 深度对齐要求

RGB-D 检测必须使用：

```yaml
enable_color: true
enable_depth: true
align_depth_to_color: true
```

深度图尺寸和 `CameraInfo` 必须与彩色图一致，否则检测器拒绝进行三维反投影。

## 何时需要重新标定彩色相机

只有在镜头结构被改动、设备遭受机械冲击、出厂标定明显异常，或科研任务要求独立验证
内参时，才建议使用本工作空间的棋盘格工具：

```bash
python3 tools/capture_calibration_images.py \
  --topic /camera/color/image_raw \
  --output-dir ./calibration/images

python3 tools/calibrate_camera_from_images.py \
  --images-dir ./calibration/images \
  --cols 9 --rows 6 --square 0.03 \
  --output ./calibration/d435_color.yaml
```

该离线结果用于核对 D435 出厂内参；当前检测器仍以实时 `CameraInfo` 为准。

改变彩色流分辨率后无需手工缩放参数，librealsense 会为新 stream profile 发布对应内参。
