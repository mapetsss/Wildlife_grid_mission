# RealSense D435 参数说明

配置文件：

```text
src/mono_camera_capture/config/d435_rgbd.yaml
src/mono_camera_capture/config/d435_rgb_only.yaml
```

## 流参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `serial_no` | 空 | 指定 D435 序列号；空值选择第一台设备 |
| `enable_color` | `true` | 启用彩色流 |
| `color_width/height` | `640/480` | 彩色分辨率 |
| `color_fps` | `30` | 彩色帧率 |
| `enable_depth` | `true` | 启用 Z16 深度流 |
| `depth_width/height` | `640/480` | 深度分辨率 |
| `depth_fps` | `30` | 深度帧率 |
| `align_depth_to_color` | `true` | 将深度重投影到彩色图坐标系 |

RGB-D 检测时必须同时启用彩色、深度和 `align_depth_to_color`。

所选分辨率和帧率必须是 D435 固件支持的组合，否则 librealsense 启动会失败。默认
`640×480@30` 是兼顾稳定性、带宽和检测速度的配置。

## 深度传感器参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `enable_emitter` | `true` | 开启红外投射器，提高室内纹理不足区域的深度质量 |
| `laser_power` | `-1.0` | 小于 0 保留设备默认值；非负值会限制到设备允许范围 |

RGB-only 配置会关闭深度流和投射器。

## 发布与恢复参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `color_frame_id` | `camera_color_optical_frame` | 彩色光学坐标系 |
| `depth_frame_id` | `camera_aligned_depth_to_color_frame` | 对齐深度坐标系 |
| `publish_camera_info` | `true` | 发布 D435 设备内参和畸变参数 |
| `log_fps` | `true` | 每秒打印 frameset 帧率 |
| `reconnect_on_failure` | `true` | USB 断开或采集失败后自动重连 |
| `reconnect_period_ms` | `1000` | 重连间隔 |
| `frame_timeout_ms` | `1000` | 等待一组帧的超时时间 |

## 运行时修改

查看参数：

```bash
ros2 param list /d435_camera_node
```

例如切换到 1280×720 彩色流：

```bash
ros2 param set /d435_camera_node color_width 1280
ros2 param set /d435_camera_node color_height 720
```

修改序列号、流开关、尺寸、帧率、对齐、投射器或激光功率时，节点会重启
librealsense pipeline。建议一次性在 YAML 中设置好组合，避免逐项修改过程中出现暂时不合法的配置。

## CameraInfo

节点直接读取 D435 当前流 profile 的出厂内参，填充 `K/D/R/P`。YOLO RGB-D 模式订阅
`/camera/color/camera_info`，不再需要手工填写 `fx/fy/cx/cy`。
