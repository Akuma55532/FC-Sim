# MAVROS 常用接口中文说明（ROS 2 Humble）

本文档选译 MAVROS 官方插件说明，并补充 PX4 仿真中的使用方法，不是全站逐字翻译。

- 适用环境：Ubuntu 22.04、ROS 2 Humble、MAVROS **2.16.0**，PX4 v1.17.0-beta1 SITL。
- 核对日期：2026-10-01。
- 来源：[官方文档](https://mavros.readthedocs.io/en/latest/)、[插件目录](https://mavros.readthedocs.io/en/latest/plugins/)、[2.16.0 文档源码](https://github.com/mavlink/mavros/tree/2.16.0/mavros/docs)、[消息和服务定义](https://github.com/mavlink/mavros/tree/2.16.0/mavros_msgs)。
- 官网 `latest` 会变化；本译文以本机 2.16.0 的文档、消息定义和插件实现交叉核对。官网链接中的 Rolling 类型页面不代表必须安装 ROS Rolling。
- 各插件的“官方说明译文”来自上游；“使用说明”“示例”“实现核对”是针对当前环境补充的内容。
- 示例默认命名空间为 `/mavros`。重映射、多机命名空间、插件启停都会改变实际接口；以运行时查询为准。
- 示例已做静态核对，未在飞行中执行。控制示例用于 PX4 SITL。

## 阅读顺序

1. [遥测与状态](01_telemetry.md)：连接、姿态、位置、速度、GPS、电池。
2. [Offboard 控制](02_offboard.md)：位置、速度、原始目标、姿态、坐标系。
3. [服务、参数与航点](03_services_parameters_missions.md)：模式、解锁、参数和任务。
4. [外部定位与传感器](04_external_estimation.md)：视觉、里程计、障碍物距离。
5. [查询、QoS 与排障](05_tools_and_qos.md)：ROS 2 查询方法、订阅代码、链路验证。

## 接口方向怎么读

以下方向始终相对于**你的算法节点**：

| 本文用语 | 你的节点执行什么 | 数据流 |
|---|---|---|
| 订阅遥测 | 创建 subscriber | PX4 → MAVROS → 你的节点 |
| 发布控制目标／观测 | 创建 publisher | 你的节点 → MAVROS → PX4 |
| 调用服务 | 创建 service client | 你的节点请求 MAVROS，MAVROS执行或向 PX4 发请求 |

官方文档的 Publishers/Subscribers 则以 **MAVROS 插件**为主体：Publishers 是供你订阅的话题，Subscribers 是供你发布的话题。
官方 `~/pose` 表示插件私有名称，例如 local_position 插件下通常展开为 `/mavros/local_position/pose`。

## 环境与启动

每个终端先加载实际安装工作空间，本文按安装脚本默认路径：

```bash
source /opt/ros/humble/setup.bash
source ~/mavros_humble_ws/install/setup.bash
```

PX4/Gazebo 在单独终端启动：

```bash
cd ~/ws_simulator/PX4-Autopilot
make px4_sitl gz_x500
```

MAVROS 在另一个终端启动：

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
```

以上适用于默认本机单机 SITL。以 PX4 控制台 `mavlink status` 的本地/远端 UDP 端口为准。
`connected: true` 表示心跳连接正常，不等于估计器正常、所有传感器都有数据或可以解锁。

## 坐标与单位约定

| 内容 | 常用 ROS 表达 |
|---|---|
| 本地世界坐标 ENU | x 向东、y 向北、z 向上 |
| 机体坐标 FLU / base_link | x 向前、y 向左、z 向上 |
| 距离、速度、加速度 | m、m/s、m/s² |
| 角度、角速度 | rad、rad/s；经纬度、罗盘航向等接口另有说明 |
| 四元数 | ROS 字段顺序 x、y、z、w；应归一化，不能全零 |

PX4/MAVLink 常使用 NED（北、东、下）和 FRD（前、右、下），MAVROS 在对应插件中进行转换。
不要给普通 ENU 接口再次手动转换为 NED；也不能推断每个 MAVLink 原始字段都有自动转换。
`header.frame_id` 本身不会使所有插件自动查询 TF，必须遵循各接口约定。
本地原点由飞控估计器定义，不能默认它永远等于 Home 点或当前起飞点。
