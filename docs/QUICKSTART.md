# 03 · 首次起飞与降落

[上一章：编译](BUILD.md) · [文档导航](README.md)

目标是在本机 PX4 SITL 中完成“连接检查 → 相对起飞 1 m → 保持 → 降落”。先完成[安装](INSTALLATION.md)和[编译](BUILD.md)。本章命令面向默认单机端点；运行验证范围见[验证记录](VALIDATION.md)。

## 终端分工

| 终端 | 运行内容 | 环境准备 |
|---|---|---|
| A | PX4 + Gazebo | 设置 `PX4_ROOT` |
| B | MAVROS | 执行[ROS 终端公共前缀](BUILD.md#5-后续每个-ros-终端的公共前缀) |
| C | flight_core | 同上 |
| D | 状态查询和 Action 命令 | 同上 |
| E（可选） | RViz2 面板 | 同上 |

同一 PX4 实例只启动一套 MAVROS 和核心。此次流程由核心负责持续控制，不同时运行底层 MAVROS 目标发布示例。核心使用默认墙上时钟，不能设置 `use_sim_time=true`。

## 1. 终端 A：启动仿真

先按[目录约定](INSTALLATION.md#目录和版本)设置实际 `PX4_ROOT`；下方默认位置仅在变量未设置时生效。

```bash
export PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
cd "$PX4_ROOT"
make px4_sitl gz_x500
```

等待仿真启动完成、模型加载。在 PX4 控制台可输入 `mavlink status` 查看链路。本教程 MAVROS 默认本地监听 14540，向 PX4 的 14580 发送数据；实际端口以该控制台输出为准。

## 2. 终端 B：启动 MAVROS

执行公共前缀后：

```bash
ros2 launch flight_core mavros_gcs.launch.py
```

此入口加载上游 PX4 配置，并配置 MAVROS 发送 GCS 心跳。需要修改端口时，替换参数值：

```bash
ros2 launch flight_core mavros_gcs.launch.py \
  fcu_url:="udp://:14540@127.0.0.1:14580"
```

两条命令是不同写法，选择一条执行。

在终端 D 检查遥测：

```bash
ros2 topic echo /mavros/state mavros_msgs/msg/State --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /mavros/extended_state mavros_msgs/msg/ExtendedState --once \
  --qos-reliability reliable --qos-durability transient_local
ros2 topic echo /mavros/local_position/pose geometry_msgs/msg/PoseStamped --once \
  --qos-reliability best_effort --qos-durability volatile
```

起飞前应看到 `connected: true`、`armed: false`、`landed_state: 1`，位姿的 `frame_id` 为 `map` 且数值有效。连接正常只说明链路成立；还需要新鲜遥测和有效位置。

## 3. 终端 C：启动核心

执行公共前缀后：

```bash
ros2 launch flight_core basic_actions.launch.py
```

在终端 D 查询：

```bash
ros2 action list -t
ros2 topic echo /flight_core/control_status flight_interfaces/msg/ControlStatus \
  --qos-reliability reliable --qos-durability volatile
```

应存在 `/flight_core/takeoff`、`/flight_core/land_action` 等 Action。观察连续状态更新，准备起飞时至少满足：

| 字段 | 起飞前期望 |
|---|---|
| `connected` | `true` |
| `telemetry_valid` | `true` |
| `pose_valid` | `true` |
| `armed` | `false` |
| `ground_state` | `1`，确认在地面 |

Ctrl+C 结束状态查询即可继续操作，不要关闭核心终端。查询使用 volatile 订阅以等待新状态；单次缓存消息不能证明核心仍在运行。

## 4. 终端 D：起飞并确认保持

确认上述条件满足后发送：

```bash
ros2 action send_goal /flight_core/takeoff flight_interfaces/action/Takeoff \
  '{height_m: 1.0}' --feedback
```

核心负责目标预发送、OFFBOARD 切换与解锁。本例无需单独发解锁命令。`height_m` 是相对起飞时位置的高度，允许范围 `(0, 10]` m；起点 Z 不为零时，目标 Z 不等于 1。

等待命令返回最终结果：Action 状态应为 `SUCCEEDED`，嵌套结果字段 `result.outcome_code` 应为 `0`。仅看到 goal accepted 还不能进入下一步。失败时保留 `message`，并查看核心状态中的 `detail`。

随后查询最新状态：

```bash
ros2 topic echo /flight_core/control_status flight_interfaces/msg/ControlStatus \
  --once --qos-reliability reliable --qos-durability volatile
```

应看到 `armed: true`、`ground_state: 2`、`control_active: true`、`holding: true`，模式为 `OFFBOARD`，当前位置接近起飞目标。核心在 Action 完成后继续发布保持目标，需要保持终端 B、C 运行。

## 5. 终端 D：降落

```bash
ros2 action send_goal /flight_core/land_action flight_interfaces/action/Land \
  '{}' --feedback
```

等待 `SUCCEEDED` 和 `result.outcome_code: 0`，再查询最新 ControlStatus，确认 `ground_state: 1`、`armed: false`。落地和解除解锁状态均应确认，不能仅凭模型看起来落地判断结束。模式字符串可能仍显示 `OFFBOARD`，也不能仅凭 `control_active` 判断是否落地；本次实测情况见[验证记录](VALIDATION.md)。

`/flight_core/land_action` 是 Action；`/flight_core/land` 则是 Trigger 服务。教程主线使用 Action，便于等待最终结果。

## 6. 可选：打开 RViz 面板

在终端 E 执行公共前缀后：

```bash
ros2 launch flight_rviz_plugins panel.launch.py
```

面板入口仅启动 RViz。连接核心 `/flight_core` 后核对状态卡片；可按同样顺序通过按钮起飞和降落。继续阅读[RViz 面板教程](RVIZ_PANEL.md)和[航点任务教程](WAYPOINTS.md)。

完成降落后，依次关闭 RViz、核心、MAVROS，最后关闭 PX4/Gazebo。关闭 RViz 或命令行客户端不会取消核心已接收的任务；Ctrl+C 不是降落命令。

## 常见失败与恢复

| 现象 | 检查与处理 |
|---|---|
| 找不到包、Action 或状态话题 | 检查公共前缀、包路径及各终端 `ROS_DOMAIN_ID`；确认 B、C 仍运行 |
| MAVROS 不连接 | 检查 PX4 `mavlink status` 与 `fcu_url`，确认 UDP 端口没有被另一实例占用 |
| 有连接但 `pose_valid: false` | 检查本地位姿是否持续发布、时间戳与 `map` 帧；等待估计器就绪 |
| 起飞拒绝或失败 | 阅读 Action 结果和 `detail`；核对地面状态、外部解锁、PX4 预检及模式切换日志，纠正原因后再尝试 |
| `another flight_core` | 同一端点已有核心，回到原终端使用或在飞行结束后关闭多余实例 |
| 面板可见但按钮禁用 | 确认核心名称及状态持续更新；只启动面板不会启动核心 |

如果需要结束仍在执行的起飞或航点任务，可在终端 D 请求：

```bash
ros2 service call /flight_core/stop std_srvs/srv/Trigger '{}'
```

起飞中的 stop 触发降落清理；航点中的 stop 尝试保持当前位置；已保持时返回已保持；其他状态可能拒绝。它不是通用暂停键。服务成功表示请求被接受，仍需观察最终状态。

若 Action 客户端已退出但系统仍保持通信，可用第 5 步的降落 Action 收尾。遥测丢失时核心可能拒绝请求，应先查看 PX4 仿真控制台和失效保护状态，恢复链路并重新确认状态，不能假定请求已经执行。

## 本章验收

记录实际版本、起飞和降落 Action 终态与结果码、起飞后保持状态以及落地后的解锁状态。未取得这些运行记录前，只能算完成安装或启动检查，不能算完成首次飞行。
