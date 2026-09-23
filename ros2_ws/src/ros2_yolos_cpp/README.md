# ros2_yolos_cpp：RealSense D435 目标检测

基于 YOLOs-CPP、ONNX Runtime 和 ROS 2 lifecycle component 的目标检测节点。
视觉输入已适配 Intel RealSense D435，提供 RGB-D 和 RGB-only 两种模式。

## RGB-D 模式

```bash
ros2 launch ros2_yolos_cpp detector.launch.py
```

默认加载包内的 `models/drone_yolo26n_480x640.onnx`，对应 D435 的
`640×480` 彩色流和 `monkey/kq/wolf/tiger/ele` 五个类别。

默认同时启动本工作空间的原生 D435 节点，并订阅：

```text
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/camera/color/camera_info
```

深度必须已经对齐到彩色图。检测器在检测框中心周围取有效深度中位数，并利用 D435
实时发布的内参计算：

```text
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
```

## RGB-only 模式

```bash
ros2 launch ros2_yolos_cpp detector_rgb_only.launch.py
```

该入口使用 `d435_rgb_only.yaml`，不启动深度传感器，检测器也不订阅深度。它只发布
二维检测和像素偏移，不发布 `offset_m`/`distance_m` 有效数据。

## 生命周期

启动后执行：

```bash
ros2 lifecycle set /yolos_detector configure
ros2 lifecycle set /yolos_detector activate
```

图像回调只缓存最新数据；调用以下服务才进行一次推理：

```text
/yolos_detector/detect
```

服务请求中的旧 `image` 字段为 API 兼容保留。节点使用缓存的 D435 彩色帧，以保证能够
和同一时间附近的对齐深度配对。

## 输出

| 话题 | 格式 | RGB-D | RGB-only |
|---|---|---:|---:|
| `~/detections` | `Detection2DArray` | 是 | 是 |
| `~/offset_px` | `[x_px, y_px]` | 是 | 是 |
| `~/offset_m` | `[X, Y]` 米 | 有有效深度时 | 否 |
| `~/distance_m` | `[Z, 欧氏距离]` 米 | 有有效深度时 | 否 |
| `~/timing` | 三阶段耗时，毫秒 | 可选 | 可选 |

坐标方向遵循彩色相机光学坐标：`X` 向右、`Y` 向下、`Z` 向前。

如果检测框中心附近没有有效深度，节点仍发布检测框和像素偏移，但跳过米制结果。

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `model_path` | 包内 YOLO26n | ONNX 模型；可通过 launch 参数覆盖 |
| `labels_path` | 包内五类别文件 | 类别文件；顺序必须与模型一致 |
| `use_gpu` | `false` | 使用 ONNX Runtime GPU |
| `conf_threshold` | `0.4` | 置信度阈值 |
| `nms_threshold` | `0.45` | NMS 阈值 |
| `yolo_version` | `auto` | `auto/v7/v8/v10/v11/v26/nas` |
| `use_depth` | `true` | 是否启用 D435 深度反投影 |
| `depth_sample_radius_px` | `3` | 中心深度取样半径 |
| `min_depth_m` | `0.15` | 最小有效深度 |
| `max_depth_m` | `10.0` | 最大有效深度 |
| `max_depth_age_s` | `0.20` | 彩色/深度最大时间差 |

检测参数配置：

- `config/default_params.yaml`：RGB-D；
- `config/rgb_only_params.yaml`：RGB-only。

## 外部启动相机

如果 D435 已单独启动：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  start_camera:=false \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/classes.txt
```

RGB-only 外部相机可在同一入口增加 `use_depth:=false`。

## 更换模型

默认部署模型随 ROS 包安装，因此构建后无需填写绝对路径。需要临时使用其他模型时可覆盖：

```bash
ros2 launch ros2_yolos_cpp detector.launch.py \
  model_path:=/path/to/model.onnx \
  labels_path:=/path/to/classes.txt \
  yolo_version:=auto
```

当前模型的导出形状、类别、训练指标和 SHA-256 记录在
`models/model_manifest.yaml`。更换文件时应同步更新 `classes.txt` 和清单。
