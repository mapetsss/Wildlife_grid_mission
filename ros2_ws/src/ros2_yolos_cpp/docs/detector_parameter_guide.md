# D435 检测参数指南

## 模式选择

RGB-D 使用 `config/default_params.yaml`，其中：

```yaml
use_depth: true
depth_sample_radius_px: 3
min_depth_m: 0.15
max_depth_m: 10.0
max_depth_age_s: 0.20
```

RGB-only 使用 `config/rgb_only_params.yaml`，其中 `use_depth: false`。

## YOLO 参数

- `model_path`：默认使用包内 `models/drone_yolo26n_480x640.onnx`，可覆盖；
- `labels_path`：默认使用包内 `models/classes.txt`，类别顺序必须与模型一致；
- `conf_threshold`：漏检多时降低，误检多时提高；
- `nms_threshold`：控制重叠框抑制；
- `use_gpu`：仅在 ONNX Runtime GPU/CUDA 环境正确时开启；
- `yolo_version`：launch 默认 `v26`；更换其他系列模型时一并覆盖。

## D435 深度参数

`depth_sample_radius_px` 控制目标中心周围的取样窗口。默认 3 表示使用 `7×7` 窗口，
筛除 0、NaN 和范围外深度后取中位数。深度边缘抖动明显时可以增大到 5；小目标不宜过大。

`min_depth_m` 和 `max_depth_m` 用于过滤 D435 无效值及场景外数据，应按实际工作距离设置。

`max_depth_age_s` 限制彩色与深度时间戳差。原生 D435 节点成组采集并使用同一 ROS 时间戳，
正常情况下远小于默认 0.20 秒。

## 不再使用的参数

旧版本的以下参数已经删除：

```text
camera_fx camera_fy camera_cx camera_cy target_plane_distance_m
```

内参直接来自 `/camera/color/camera_info`，前向距离直接来自 D435 对齐深度，不再依赖
手工填写的固定目标平面。

## 输出解释

```text
offset_px = [u - width/2, v - height/2]
offset_m  = [X, Y]
distance_m = [Z, sqrt(X² + Y² + Z²)]
```

`X` 向图像右侧，`Y` 向图像下方，`Z` 沿相机光轴向前，单位米。

RGB-only 模式只输出 `detections`、`offset_px` 和可选 `timing`。
