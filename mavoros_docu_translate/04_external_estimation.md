# 04 外部定位与传感器

[返回目录](README.md)

本页接口主要来自 mavros_extras。安装软件包后，还需要启用相应插件并配置飞控融合策略。
“消息发到飞控”与“飞控估计器已经采用该数据”是两件事。

## 1. vision_pose / vision_speed：视觉定位

**官方说明译文：**将视觉估计器产生的位置、姿态或速度估计发送给飞控的位置和姿态估计器。

来源：[vision_pose](https://mavros.readthedocs.io/en/latest/plugins/extras/vision_pose_estimate/)、[vision_speed](https://mavros.readthedocs.io/en/latest/plugins/extras/vision_speed_estimate/)。

以下由你的 VIO/SLAM 转接节点发布：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/mavros/vision_pose/pose` | `geometry_msgs/msg/PoseStamped` | 外部位置和姿态 |
| `/mavros/vision_pose/pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 带协方差的外部位姿 |
| `/mavros/vision_speed/speed_twist` | `geometry_msgs/msg/TwistStamped` | 外部速度 |
| `/mavros/vision_speed/speed_twist_cov` | `geometry_msgs/msg/TwistWithCovarianceStamped` | 带协方差的外部速度 |
| `/mavros/vision_speed/speed_vector` | `geometry_msgs/msg/Vector3Stamped` | 只提供速度向量 |

位姿转换为 VISION_POSITION_ESTIMATE，速度转换为 VISION_SPEED_ESTIMATE。
ROS 侧通常按 ENU 世界坐标和 FLU 机体姿态提供数据。
相机光学坐标（右、下、前）不等于机体 FLU，必须先完成相机到机体的外参变换。
不要以为写上 header.frame_id 就会自动完成所有转换。

使用说明：

- 时间戳对应观测时间，并与系统时钟策略一致；不要重复发送不变的旧时间戳。
- 正确设置位姿参考系、外参和协方差。零协方差不能随意当作“未知”。
- SLAM 重定位、原点重置和尺度变化需要处理，不能把坐标跳变当作真实飞行运动。
- 根据当前 PX4 版本配置外部视觉融合，例如 EKF2_EV_CTRL、时延和传感器位置参数，并通过估计器状态与日志检查是否融合。
- 不同 MAVLink 外部估计消息在 PX4 中的支持可能不同；不能仅凭插件可发送就认定已被使用。

## 2. odometry：完整里程计

**官方说明译文：**向飞控估计器发送里程计，也把飞控发来的里程计发布到 ROS。
遵循 REP 147：位姿在父坐标系表达，四元数描述子系到父系的旋转，速度在子坐标系表达。

来源：[odometry](https://mavros.readthedocs.io/en/latest/plugins/extras/odom/)。

| 话题 | 类型 | 数据方向 |
|---|---|---|
| `/mavros/odometry/out` | `nav_msgs/msg/Odometry` | 你的节点发布 → MAVROS → 飞控 |
| `/mavros/odometry/in` | `nav_msgs/msg/Odometry` | 飞控 → MAVROS 发布 → 你的节点订阅 |

这里的 in/out 按 MAVROS 的 MAVLink 数据流理解，容易与用户节点视角相反。

| 字段 | 含义 |
|---|---|
| header.frame_id | 位姿的父参考系 |
| child_frame_id | 机体/子参考系，twist 的表达坐标系 |
| pose.pose | 位置和姿态 |
| twist.twist | 子坐标系中的线速度、角速度 |
| pose.covariance / twist.covariance | 6×6 协方差矩阵 |

需要提供插件所需的 TF 变换。对应参数有 fcu.odom_parent_id_des、fcu.odom_child_id_des、fcu.map_id_des，默认值来自 UAS 的帧配置。
与 vision_pose 一样，接收了 ROS 消息不表示已成功通过 TF 转换、MAVLink 转发并被 EKF 融合；同时检查 MAVROS 日志和 PX4 估计器日志。

```bash
ros2 topic info /mavros/odometry/out --verbose
ros2 topic echo /mavros/odometry/in
```

不要把飞控输出的里程计原封不动反馈为独立外部观测，这不是新增传感器信息。

## 3. obstacle_distance：障碍物距离

**官方说明译文：**把障碍物距离数组发送给飞控，为避障飞行提供输入。

来源：[obstacle_distance](https://mavros.readthedocs.io/en/latest/plugins/extras/obstacle_distance/)。

- 话题：`/mavros/obstacle_distance/send`。
- 类型：`sensor_msgs/msg/LaserScan`。
- 方向：你的节点发布 → MAVROS → MAVLink OBSTACLE_DISTANCE。
- 重点字段：angle_min、angle_increment（rad），ranges、range_min、range_max（m）。
- 插件参数 mav_frame 默认 GLOBAL；发送前必须配置为与扫描方向语义一致且受飞控支持的帧。

传感器扫描平面、角度零点和安装朝向必须正确；frame_id 不是保证自动完成外参变换的开关。
只发布扫描不会自动生成路径规划器，也不意味着 PX4 已启用避障/碰撞预防。需要确认飞控对应功能、模式和参数。

## 4. distance_sensor：单点距离传感器

**官方说明译文：**支持将外部计算机连接的测距传感器数据发送到飞控，也支持把飞控测距数据发布到 ROS。

来源：[distance_sensor](https://mavros.readthedocs.io/en/latest/plugins/extras/distance_sensor/)。

消息类型为 `sensor_msgs/msg/Range`，range、min_range、max_range 单位为 m，field_of_view 为 rad。
**话题名称按每个传感器的映射配置决定，不存在一个适用于所有安装的固定话题。**
插件 config 参数为传感器配置 YAML，包含 ID、话题、方向等；base_frame_id 默认 base_link。
查阅实际 px4_config.yaml 并运行 ros2 topic list -t，确认使用的是发送映射还是接收映射。
