# px4_offboard_mission

本包包含 MAVROS/PX4 Offboard 节点和串口航线桥接节点：

- `offboard_mission_node`：基础演示任务，保留。
- `wildlife_mission_node`：地面站航线驱动的 9 x 7 方格遍历任务主节点。
- `small_grid_test_node`：固定小范围方格测试任务，用于验证雷达定位和 local 坐标飞行精度。
- `serial_route_bridge_node`：读取树莓派串口航点，归一化后输入 `wildlife_mission_node`。

## wildlife_mission_node

当前主线只负责飞行逻辑：接收地面站方格航线，转换成 MAVROS local 绝对航点，然后依次执行。视觉接口保留，但不参与路径控制。

### 坐标约定

任务开始时飞机当前位置固定解释为 `HOME/A9B1` 中心，任务开始时机头方向固定解释为地图向上方向：`A9B1 -> A8B1 -> A1B1`。

方格范围为 `A1..A9`、`B1..B7`。默认每格 `0.5 m`：

```text
A8B1 = HOME 前方 0.5 m
A9B2 = HOME 右侧 0.5 m
A1B7 = HOME 前方 4.0 m、右侧 3.0 m
```

每个目标点都从 `HOME/A9B1` 直接计算，不使用上一航点增量累加。

### 状态机

```text
IDLE
-> WAIT_FCU
-> OFFBOARD_PREPARE
-> ARM
-> TAKEOFF
-> FOLLOW_ROUTE
-> HOLD_AT_GRID
-> RETURN_HOME
-> LAND
-> LANDING
-> DONE
```

- `IDLE`：等待有效 `/mission/route_upload` 和 `/mission/start`；无航线时不切 OFFBOARD、不解锁。
- `WAIT_FCU`：等待 MAVROS 连接和新鲜 `/mavros/local_position/pose`，锁定 `HOME/A9B1` 与 yaw。
- `OFFBOARD_PREPARE`：预发送 HOME 上方 setpoint，切 `OFFBOARD`。
- `ARM`：默认自动调用 `/mavros/cmd/arming`。
- `FOLLOW_ROUTE`：依次执行地面站上传的绝对方格航点。
- `RETURN_HOME`：全部航点完成后返回 `HOME/A9B1` 上方。
- `FAILSAFE`：定位、起飞、航点、返航超时或 `/mission/stop` 时请求 `AUTO.LAND`。

### 接口

| Topic/Service | Type | 说明 |
| --- | --- | --- |
| `/mission/route_upload` | `std_msgs/msg/String` | 地面站上传完整航线，如 `A9B1,A8B1,A7B1,A7B2` |
| `/mission/no_fly_zones` | `std_msgs/msg/String` | 禁飞格，如 `A4B3,A5B3` |
| `/mission/start` | `std_srvs/srv/Trigger` | 启动任务 |
| `/mission/stop` | `std_srvs/srv/Trigger` | 停止任务并 FAILSAFE 降落 |
| `/mission/land` | `std_srvs/srv/Trigger` | 立即降落 |
| `/mission/status` | `std_msgs/msg/String` | JSON 状态，含当前点、目标点和 local 坐标 |
| `/mission/waypoint_reached` | `std_msgs/msg/String` | 到达方格时发布 JSON |
| `/mission/done` | `std_msgs/msg/Bool` | 任务完成或失败 |
| `/vision/animal_report` | `std_msgs/msg/String` | 视觉预留 |
| `/vision/target_offset` | `geometry_msgs/msg/Vector3Stamped` | 视觉预留 |

### 串口航线上传

`wildlife_mission.launch.py` 默认同时启动 `serial_route_bridge_node`。桥接节点优先读取
`/dev/serial0`；如果该端口打不开，会继续尝试当前 DAP 的稳定 by-id 路径和 `/dev/ttyACM0`。
串口参数为 `115200 8N1`、无流控。收到一整行合法航点后发布到
`/mission/route_upload`，再调用 `/mission/start`；后续定位检查、OFFBOARD、起飞和
航点执行仍由 `wildlife_mission_node` 完成。

串口一行数据必须以 `\n` 结尾，支持空格、逗号或分号分隔，也兼容可选 `ROUTE` 前缀：

```text
A3B1 A3B2 A2B2 A1B2
A3B1,A3B2,A2B2,A1B2
ROUTE A3B1 A3B2 A2B2 A1B2
```

桥接节点只接受 `A1..A9`、`B1..B7`。内核日志、登录提示、乱码和非法航点会被丢弃并
打印 ROS WARN，不会发布给飞控。

### 关键参数

配置文件：`config/wildlife_mission.yaml`

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `target_height_m` | `1.2` | 巡航高度，相对 HOME 高度 |
| `grid_cell_size_m` | `0.5` | 方格边长 |
| `auto_arm_enabled` | `true` | OFFBOARD 后自动解锁 |
| `failsafe_land_on_error` | `true` | FAILSAFE 时请求 `AUTO.LAND` |
| `waypoint_hold_time_s` | `1.0` | 到点停留时间 |
| `horizontal_tolerance_m` | `0.10` | 水平到达阈值 |
| `vertical_tolerance_m` | `0.08` | 高度到达阈值 |
| `pose_timeout_s` | `2.5` | local pose 超时阈值 |

### 构建和运行

```bash
cd /home/banana/detect_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_offboard_mission --symlink-install
source install/setup.bash
ros2 launch px4_offboard_mission wildlife_mission.launch.py
```

### 独立启动雷达定位链路

飞行节点不直接启动 MID360、FAST-LIO 或 `drone_bridge`，只依赖 MAVROS 已经输出的新鲜
`/mavros/local_position/pose`。需要先单独启动雷达定位与 PX4 外部里程计链路：

```bash
cd /home/banana/detect_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch px4_offboard_mission radar_localization.launch.py
```

该 launch 会依次启动 `/home/banana/fastlio_ws` 中的 Livox MID360、FAST-LIO、MAVROS
和 FAST-LIO 到 MAVROS odometry 桥接。所有进程都由 ROS launch 前台管理，不使用后台
`&`；按 `Ctrl-C` 停止 `radar_localization.launch.py` 后，会向这些子进程发送关闭信号，
避免残留进程影响下一次启动。

默认飞控串口为当前 CH340 转串口：

```bash
fcu_url:=/dev/ttyUSB0:921600
```

如果现场 CH340 枚举成其他设备，启动时显式指定：

```bash
ros2 launch px4_offboard_mission radar_localization.launch.py fcu_url:=/dev/ttyUSB1:921600
```

也可以使用更稳定的 by-id 路径：

```bash
ros2 launch px4_offboard_mission radar_localization.launch.py fcu_url:=/dev/serial/by-id/<CH340设备>:921600
```

可选参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `fastlio_ws` | `/home/banana/fastlio_ws` | 雷达定位工作区 |
| `fcu_url` | `/dev/ttyUSB0:921600` | MAVROS 飞控串口 |
| `start_delay_s` | `3.0` | 各进程启动间隔 |
| `use_mavros` | `true` | 是否启动 MAVROS |
| `use_drone_bridge` | `true` | 是否启动 FAST-LIO 到 MAVROS odometry 桥 |

### small_grid_test_node

`small_grid_test_node` 是独立测试节点，不接收地面站航点，不启动串口桥接，也不订阅视觉处理
topic。固定路线为：

```text
A9B1 -> A8B1 -> A7B1 -> A6B1 -> A6B2 -> A7B2 -> A8B2 -> A9B2 -> A9B1 -> AUTO.LAND
```

启动前先用 `radar_localization.launch.py` 拉起雷达定位链路，确认 `/Odometry`、
`/mavros/odometry/out` 和 `/mavros/local_position/pose` 新鲜，再启动小范围测试：

```bash
ros2 launch px4_offboard_mission small_grid_test.launch.py
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

默认 `auto_arm_enabled=false`，节点只请求 `OFFBOARD`，等待遥控器或外部手动解锁。需要自动
请求 `/mavros/cmd/arming` 时必须显式开启：

```bash
ros2 launch px4_offboard_mission small_grid_test.launch.py auto_arm_enabled:=true
```

`/mission/status` 会发布 JSON 状态，除飞行状态和目标点外，还包含：

```text
radar_odom_fresh
px4_local_pose_fresh
radar_odom_x/y/z
px4_local_x/y/z
target_x/y/z
target_error_xy/z
radar_to_px4_error_xy/z
radar_status
```

到点判定仍以 `/mavros/local_position/pose` 为准；`/Odometry` 只用于状态观测和与 PX4
local pose 的一致性检查。默认 `radar_px4_max_error_m=0.30`，`radar_status=DIVERGED`
持续超过 `radar_diverged_timeout_s=1.0` 时进入 FAILSAFE 并请求降落。

串口桥接默认开启。开发机没有 `/dev/serial0` 时可关闭：

```bash
ros2 launch px4_offboard_mission wildlife_mission.launch.py use_serial_bridge:=false
```

现场需要指定串口设备或禁止自动调用 `/mission/start` 时：

```bash
ros2 launch px4_offboard_mission wildlife_mission.launch.py \
  serial_port:=/dev/ttyS0 \
  serial_start_after_route:=false
```

地面站流程：

```bash
ros2 topic pub --once /mission/route_upload std_msgs/msg/String "{data: 'A9B1,A8B1,A7B1,A7B2'}"
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

起飞前确认：飞机摆在 `A9B1` 中心，机头朝向 `A8B1/A1B1`。任务会自动解锁，实飞前必须确认 PX4、MAVROS、定位链路和遥控器接管方式正常。
