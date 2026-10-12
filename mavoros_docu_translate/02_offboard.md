# 02 Offboard 控制接口

[返回目录](README.md)

## 1. PX4 + MAVROS 的控制条件

**使用说明：**本页是 ROS 2 → MAVROS → MAVLink → PX4 路径，不能混用 PX4 原生 DDS 的 OffboardControlMode 示例。

- 在切换 OFFBOARD 前，先连续发送目标值超过 1 秒。
- PX4 要求持续的目标值流至少满足 2 Hz 活性要求；实践中建议固定 20 Hz 或更高，并留出调度裕量。
- 切换模式后仍要持续发布。停止后会受 COM_OF_LOSS_T 和相应失联处置参数影响。
- 位置/速度控制需要有效的对应状态估计。模式请求被发送不代表已切换成功。
- 同一时刻选择一个主要控制接口，避免多个节点同时向不同 setpoint 话题发送互相冲突的目标。

来源：[PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html)。频率和模式条件亦核对了本地 PX4 v1.17.0-beta1 文档。

## 2. setpoint_position：位置目标

**官方说明译文：**向飞控控制器发送位置目标，使用 MAVLink Offboard 控制协议。

来源：[setpoint_position](https://mavros.readthedocs.io/en/latest/plugins/std/setpoint_position/)。

以下均由你的节点发布，MAVROS 订阅：

| 话题 | 类型 | 用途 |
|---|---|---|
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | 本地位置和偏航目标 |
| `/mavros/setpoint_position/global` | `geographic_msgs/msg/GeoPoseStamped` | 全球位置目标 |
| `/mavros/setpoint_position/global_to_local` | `geographic_msgs/msg/GeoPoseStamped` | 将全球目标转换为本地目标；需要当前全球/本地位置 |

默认 `mav_frame=LOCAL_NED` 指传给 MAVLink 的帧枚举；普通 local ROS 输入仍按 ENU 数值填写。
位置控制中四元数主要用于提取偏航，并非完整姿态控制接口。
全球目标的高度基准需结合插件实现、MAV_FRAME 和飞控支持确认，不能直接复制 NavSatFix.altitude 当作任意全球目标高度。

以下为 SITL 中本地原点上方 2 m 的持续位置目标，**不是相对当前位置上升 2 m**：

```bash
ros2 topic pub --rate 20 /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped \
  "{header: auto, pose: {position: {x: 0.0, y: 0.0, z: 2.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Humble 的 `header: auto` 自动填入当前时间戳。程序中应逐次更新时间戳。此命令单独运行不会自动解锁或切换 OFFBOARD；
如果无人机已处于接受位置指令的状态，则会影响其运动。完整模式/解锁接口见[服务说明](03_services_parameters_missions.md)。

## 3. setpoint_velocity：速度目标

**官方说明译文：**向飞控发送速度设定值。

来源：[setpoint_velocity](https://mavros.readthedocs.io/en/latest/plugins/std/setpoint_velocity/)。

| 话题 | 类型 | 使用的字段 |
|---|---|---|
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/msg/TwistStamped` | twist.linear 和 twist.angular.z |
| `/mavros/setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | linear 和 angular.z |

默认本地模式下，linear 是 ENU 速度，单位 m/s；angular.z 是偏航角速度，单位 rad/s。
angular.x/y 不作为本接口的横滚、俯仰控制。
机体速度需要配置插件的 `mav_frame` 并核对 FLU→FRD 转换；只改 header.frame_id 不会自动变成机体控制。

零速度目标示例（与上面的位置发布示例二选一）：

```bash
ros2 topic pub --rate 20 /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: auto, twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

零速度目标不等于下达“锁定当前绝对位置”的位置目标。

## 4. setpoint_raw：组合控制目标

**官方说明译文：**发送位置等目标并发布飞控返回的目标状态。用户通过 IGNORE 位选择哪些字段参与控制。

来源：[setpoint_raw](https://mavros.readthedocs.io/en/latest/plugins/std/setpoint_raw/)、[PositionTarget 定义](https://github.com/mavlink/mavros/blob/2.16.0/mavros_msgs/msg/PositionTarget.msg)。

| 话题 | 类型 | 你的节点的方向 |
|---|---|---|
| `/mavros/setpoint_raw/local` | `mavros_msgs/msg/PositionTarget` | 发布：本地位置/速度/加速度目标 |
| `/mavros/setpoint_raw/global` | `mavros_msgs/msg/GlobalPositionTarget` | 发布：全球目标 |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/msg/AttitudeTarget` | 发布：姿态/角速度及推力目标 |
| `/mavros/setpoint_raw/target_local` | `mavros_msgs/msg/PositionTarget` | 订阅：飞控报告的本地目标 |
| `/mavros/setpoint_raw/target_global` | `mavros_msgs/msg/GlobalPositionTarget` | 订阅：飞控报告的全球目标 |
| `/mavros/setpoint_raw/target_attitude` | `mavros_msgs/msg/AttitudeTarget` | 订阅：飞控报告的姿态目标 |

返回话题不是 MAVROS 对你的消息做本地回显，是否有数据取决于飞控是否发送对应 MAVLink 消息，也不能单独作为执行完成确认。

### PositionTarget 字段和 type_mask

| 字段 | 中文说明 |
|---|---|
| coordinate_frame | MAV_FRAME 枚举；1 LOCAL_NED、7 LOCAL_OFFSET_NED、8 BODY_NED、9 BODY_OFFSET_NED；飞控不一定支持所有选项 |
| type_mask | IGNORE 位为 1 表示忽略该字段，不是启用 |
| position | 位置，m |
| velocity | 速度，m/s |
| acceleration_or_force | 加速度 m/s²；设置 FORCE 位时表示力，支持情况取决于飞控 |
| yaw / yaw_rate | 偏航角 rad / 偏航角速度 rad/s |

| 掩码常量 | 数值 |
|---|---|
| IGNORE_PX / PY / PZ | 1 / 2 / 4 |
| IGNORE_VX / VY / VZ | 8 / 16 / 32 |
| IGNORE_AFX / AFY / AFZ | 64 / 128 / 256 |
| FORCE | 512 |
| IGNORE_YAW / IGNORE_YAW_RATE | 1024 / 2048 |

例：只使用位置和 yaw，忽略速度、加速度和 yaw_rate：
`8+16+32+64+128+256+2048 = 2552`。
代码中推荐组合具名常量，避免难以解释的数字。

**实现核对：**raw 并不表示“坐标不转换”。2.16.0 的 local 回调对本地输入执行 ENU→NED；
BODY 分支对向量执行 FLU→FRD。即使 coordinate_frame=1，ROS 侧 z 向上仍为正。
不要把 type_mask=0 当作“未填写字段自动忽略”，那会启用所有字段，包括默认零值。
PX4 支持的字段组合以其 Offboard 文档为准。

### 姿态和推力

AttitudeTarget.orientation 为姿态四元数，body_rate 为机体角速度，thrust 为归一化推力。
忽略角速度的三个掩码位为 1、2、4；IGNORE_THRUST=64，IGNORE_ATTITUDE=128。
常见“姿态+推力”掩码为 7；“角速度+推力”掩码为 128。

ROS 姿态表示 base_link 相对 ENU 的姿态，机体角速度按 FLU；插件转换到飞控约定。
推力并非牛顿值，也不能把 0.5 当作任何机型的悬停保证。
检查 `setpoint_raw.thrust_scaling` 对应插件参数；2.16.0 的 px4_config.yaml 配置为 1.0。

## 5. setpoint_attitude：分离的姿态／推力话题

来源：[setpoint_attitude](https://mavros.readthedocs.io/en/latest/plugins/std/setpoint_attitude/)、[实现](https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/setpoint_attitude.cpp)。

上游自动生成页面中的 th_sub/pose_sub/twist_sub 是变量名。核对实现后的实际话题为：

| 话题 | 类型 | 条件 |
|---|---|---|
| `/mavros/setpoint_attitude/attitude` | `geometry_msgs/msg/PoseStamped` | use_quaternion=true |
| `/mavros/setpoint_attitude/cmd_vel` | `geometry_msgs/msg/TwistStamped` | use_quaternion=false；使用角速度 |
| `/mavros/setpoint_attitude/thrust` | `mavros_msgs/msg/Thrust` | 与上面的姿态或角速度消息同步 |

默认 use_quaternion=false。此插件同步两个话题的时间戳，单发姿态而不发推力通常不会形成完整控制消息。
初次做姿态控制时，单条 AttitudeTarget 更容易组织同步数据，但仍要正确配置掩码和推力缩放。
