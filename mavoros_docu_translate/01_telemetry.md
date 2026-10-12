# 01 遥测与状态

[返回目录](README.md)

本页话题均由 MAVROS 发布，你的节点订阅。类型使用 ROS 2 的 `包/msg/类型` 写法。

## 1. sys_status：系统状态

**官方说明译文：**系统状态插件实现 MAVLink 心跳/连接协议，提供连接状态、飞行模式、电池状态和飞控状态文本，是其他插件工作的基础。

来源：[sys_status](https://mavros.readthedocs.io/en/latest/plugins/std/sys_status/)。

| 话题 | 消息类型 | 用途 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/msg/State` | 心跳连接、解锁、模式等 |
| `/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | 着陆状态、VTOL 状态 |
| `/mavros/sys_status` | `mavros_msgs/msg/SysStatus` | 系统传感器健康、负载等 |
| `/mavros/estimator_status` | `mavros_msgs/msg/EstimatorStatus` | 状态估计器标志和检验信息 |
| `/mavros/battery` | `sensor_msgs/msg/BatteryState` | 电压、电流、剩余电量等 |
| `/mavros/statustext/recv` | `mavros_msgs/msg/StatusText` | 飞控文本，例如解锁被拒绝的原因 |

### State 字段

| 字段 | 中文含义 |
|---|---|
| `connected` | 是否在连接超时范围内收到目标飞控心跳 |
| `armed` | 是否已解锁 |
| `guided` | 心跳中 GUIDED 标志位；不能用它单独判断 PX4 是否处于 OFFBOARD |
| `manual_input` | 心跳中的手动输入标志 |
| `mode` | 模式字符串，例如 OFFBOARD、AUTO.LOITER、AUTO.LAND |
| `system_status` | MAVLink 系统状态枚举，不是 ROS 错误码 |

`ExtendedState.landed_state`：0 未定义，1 地面，2 空中，3 起飞中，4 降落中。
`BatteryState.percentage` 是 0～1 的比例，不是 0～100；未知量可能为 NaN。
状态消息的单次默认值不宜作为完整健康判据。

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
ros2 topic echo /mavros/statustext/recv --qos-reliability best_effort
```

## 2. imu：姿态和惯性数据

**官方说明译文：**发布融合姿态和 IMU 数据、原始惯性数据、磁场、温度及气压。

来源：[imu](https://mavros.readthedocs.io/en/latest/plugins/std/imu/)。

| 话题 | 消息类型 | 说明／单位 |
|---|---|---|
| `/mavros/imu/data` | `sensor_msgs/msg/Imu` | 融合姿态四元数、角速度 rad/s、线加速度 m/s² |
| `/mavros/imu/data_raw` | `sensor_msgs/msg/Imu` | 原始惯性测量，不保证包含有效姿态 |
| `/mavros/imu/mag` | `sensor_msgs/msg/MagneticField` | 磁场，T |
| `/mavros/imu/static_pressure` | `sensor_msgs/msg/FluidPressure` | 静压，Pa |
| `/mavros/imu/diff_pressure` | `sensor_msgs/msg/FluidPressure` | 差压，Pa |
| `/mavros/imu/temperature_imu` | `sensor_msgs/msg/Temperature` | IMU 温度，℃ |
| `/mavros/imu/temperature_baro` | `sensor_msgs/msg/Temperature` | 气压计温度，℃ |

默认机体帧为 `base_link`。遵循 Imu 消息对协方差的约定：对应协方差数组首元素为 -1 表示该估计不存在；全零表示协方差未知。
线加速度不能直接当作“世界系中已去掉重力的运动加速度”使用，需要核对数据来源并进行相应处理。

```bash
ros2 topic echo /mavros/imu/data --qos-reliability best_effort
ros2 topic hz /mavros/imu/data
```

## 3. local_position：本地位置与速度

**官方说明译文：**把飞控本地位置发布为位姿、速度、里程计，并可选择发布 TF。

来源：[local_position](https://mavros.readthedocs.io/en/latest/plugins/std/local_position/)、[2.16.0 实现](https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/local_position.cpp)。

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/mavros/local_position/pose` | `geometry_msgs/msg/PoseStamped` | 本地 ENU 位置和姿态 |
| `/mavros/local_position/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 带协方差的位姿 |
| `/mavros/local_position/velocity_local` | `geometry_msgs/msg/TwistStamped` | 本地世界系速度 |
| `/mavros/local_position/velocity_body` | `geometry_msgs/msg/TwistStamped` | 机体系速度 |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 位姿与机体系速度；分别看 header 和 child_frame_id |
| `/mavros/local_position/accel` | `geometry_msgs/msg/AccelWithCovarianceStamped` | 带协方差的本地加速度，依赖对应 MAVLink 消息 |

**实现核对：**上游自动生成文档把 `velocity_local` 描述为 NED，但 2.16.0 实现先将速度转换到 ENU 后发布，因此此处按 ENU 解释。该版本此分支的 `header.frame_id` 又被设置为机体帧名称；在向 TF/融合系统转接时需核对并正确标注，不能只看该字符串推断数据轴向。

常用参数：`frame_id` 默认 map，`tf.send` 默认 false，`tf.frame_id` 默认 map，`tf.child_frame_id` 默认 base_link。启动 YAML 可以覆盖这些默认值。只有开启 TF 发布，才会出现相应变换。

```bash
ros2 topic echo /mavros/local_position/pose --qos-reliability best_effort
ros2 topic info /mavros/local_position/odom --verbose
```

## 4. global_position：全球定位

**官方说明译文：**发布全球位置，通过经纬高到地心地固坐标的转换，还可输出对应的本地位置。

来源：[global_position](https://mavros.readthedocs.io/en/latest/plugins/std/global_position/)。

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/mavros/global_position/raw/fix` | `sensor_msgs/msg/NavSatFix` | 原始 GPS 定位 |
| `/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` | 飞控融合后的全球位置 |
| `/mavros/global_position/raw/satellites` | `std_msgs/msg/UInt32` | 可见卫星数 |
| `/mavros/global_position/rel_alt` | `std_msgs/msg/Float64` | 飞控报告的相对高度，m |
| `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | 罗盘航向，度；北为 0，顺时针增大 |
| `/mavros/global_position/local` | `nav_msgs/msg/Odometry` | 由全球位置计算的本地里程计 |

NavSatFix 经纬度以度表示，altitude 为 WGS-84 椭球高，MAVROS 使用 GeographicLib 进行相关高度转换。
相对高度、椭球高、海拔高度和本地 z 不是同一物理量；相对高度也不能直接当作离地高度。
罗盘航向与 ENU yaw 的零点和正方向不同。
话题存在但没有数据时，还要检查飞控是否发送对应消息、插件是否启用和估计器是否有有效解。
