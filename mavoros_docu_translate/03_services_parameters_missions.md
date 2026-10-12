# 03 服务、参数与航点

[返回目录](README.md)

本页“调用”均指你的节点或 ros2 CLI 作为客户端。服务存在不代表 PX4 支持其中每一种 MAVLink 命令。

## 1. 模式、解锁和基本命令

**官方说明译文：**command 插件发送 MAVLink 命令；通用 COMMAND_LONG 接口等待命令确认。sys_status 插件提供模式切换和消息发送频率设置。

来源：[command](https://mavros.readthedocs.io/en/latest/plugins/std/command/)、[sys_status](https://mavros.readthedocs.io/en/latest/plugins/std/sys_status/)。

| 服务 | 类型 | 请求重点 | 响应重点 |
|---|---|---|---|
| `/mavros/set_mode` | `mavros_msgs/srv/SetMode` | base_mode、custom_mode | mode_sent |
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | value：true 解锁、false 上锁 | success、result |
| `/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | min_pitch、yaw、latitude、longitude、altitude | success、result |
| `/mavros/cmd/land` | `mavros_msgs/srv/CommandTOL` | yaw、latitude、longitude、altitude | success、result |
| `/mavros/cmd/set_home` | `mavros_msgs/srv/CommandHome` | current_gps 或指定位置 | success、result |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | command、param1～param7 等 | success、result |
| `/mavros/set_message_interval` | `mavros_msgs/srv/MessageInterval` | message_id、message_rate | success |

SetMode 中 custom_mode 非空时通常令 base_mode=0。
**mode_sent=true 只说明模式字符串解析和发送成功，不代表飞控已经切换。**
要继续检查 /mavros/state.mode。command 的 result 是 MAV_RESULT 枚举；success 也不能代替实际位置、着陆状态等完成判据。

### 模式与解锁示例

以下会改变飞控状态，只用于你已准备好的 SITL；不是需要依次执行的安装步骤。
切换 OFFBOARD 前，必须在另一个节点持续发送有效目标，见[Offboard 控制](02_offboard.md)。

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'OFFBOARD'}"

ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool \
  "{value: true}"
```

在单独终端观察结果：

```bash
ros2 topic echo /mavros/state
ros2 topic echo /mavros/statustext/recv --qos-reliability best_effort
```

请求 PX4 自动降落：

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode \
  "{base_mode: 0, custom_mode: 'AUTO.LAND'}"
```

使用 extended_state.landed_state 确认着陆；不要把飞行中上锁当作正常降落方式。
CommandTOL 是通用 MAVLink 起降命令包装，其高度基准和参数语义按命令与飞控实现确定，
不能把 altitude 无条件解释为“离地高度”。Offboard 位置起飞也不要求先调用 cmd/takeoff。

### 修改 MAVLink 消息频率

请求 LOCAL_POSITION_NED（消息 ID 32）以 20 Hz 发送：

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
  "{message_id: 32, message_rate: 20.0}"
```

message_rate 的单位是 Hz，不是微秒。0 请求恢复默认，负值请求停发。
飞控可能受带宽和调度约束，实际频率需用 ros2 topic hz 检查。一个 ROS 话题也可能依赖多种 MAVLink 消息。

## 2. param：飞控参数

**官方说明译文：**实现 MAVLink 参数协议。可以从设备读取参数、设置单个参数，并通过 ROS 2 参数服务访问本地参数缓存。

来源：[param](https://mavros.readthedocs.io/en/latest/plugins/std/param/)、[ParamSetV2](https://github.com/mavlink/mavros/blob/2.16.0/mavros_msgs/srv/ParamSetV2.srv)。

| 接口 | 类型 | 说明 |
|---|---|---|
| 服务 `/mavros/param/pull` | `mavros_msgs/srv/ParamPull` | 拉取参数列表，force_pull=true 强制重新读取 |
| 服务 `/mavros/param/set` | `mavros_msgs/srv/ParamSetV2` | 写单个参数，含 force_set、param_id、value |
| 服务 `/mavros/param/get_parameters` | `rcl_interfaces/srv/GetParameters` | 读取本地缓存 |
| 服务 `/mavros/param/set_parameters` | `rcl_interfaces/srv/SetParameters` | 通过标准 ROS 2 参数接口写飞控 |
| 服务 `/mavros/param/list_parameters` | `rcl_interfaces/srv/ListParameters` | 列出缓存中的参数 |
| 话题 `/mavros/param/event` | `mavros_msgs/msg/ParamEvent` | 参数变化通知，你的节点订阅 |

先拉取，再读取：

```bash
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}"
ros2 param list /mavros/param
ros2 param get /mavros/param MAV_SYS_ID
```

pull 返回 success 和 param_received；成功且数量大于零是一个双向链路验证方法。
参数名随飞控版本变化。MAV_SYS_ID 是 PX4 的系统 ID 示例。
/mavros/param 下的飞控参数，与 /mavros/setpoint_velocity 等插件节点的 ROS 配置参数是两类不同对象。

写参数优先使用标准 ROS 2 参数 API：

```text
ros2 param set /mavros/param <参数名> <新值>
```

上面是语法模板，不可原样执行。修改前先记录原值，依据 PX4 参数说明判断类型、范围以及是否需要重启。
ParamSetV2.value 是 rcl_interfaces/ParameterValue：整数 type=2 并填 integer_value，浮点 type=3 并填 double_value。
force_set=true 会绕过部分检查，常规应用不需要使用。
set_parameters_atomically 在这个插件中不支持设备端原子写入，会报告失败。

**ROS 1 差异：**不要直接照搬使用 ParamGet、ParamSet 或 /mavros/param/get、/push 的旧教程。
消息包中有某个 srv 文件，不意味着当前 ROS 2 插件就提供对应服务。

## 3. waypoint：任务航点

**官方说明译文：**实现 MAVLink 任务协议，支持读取、上传、清空任务和设置当前航点，并发布当前任务列表及到达事件。

来源：[waypoint](https://mavros.readthedocs.io/en/latest/plugins/std/waypoint/)、[Waypoint](https://github.com/mavlink/mavros/blob/2.16.0/mavros_msgs/msg/Waypoint.msg)。

插件名是 waypoint，默认 ROS 接口前缀是 **/mavros/mission**。

| 接口 | 类型 | 说明 |
|---|---|---|
| 话题 `/mavros/mission/waypoints` | `mavros_msgs/msg/WaypointList` | 订阅当前任务列表 |
| 话题 `/mavros/mission/reached` | `mavros_msgs/msg/WaypointReached` | 订阅航点到达事件 |
| 服务 `/mavros/mission/pull` | `mavros_msgs/srv/WaypointPull` | 从飞控读取任务 |
| 服务 `/mavros/mission/push` | `mavros_msgs/srv/WaypointPush` | 上传任务 |
| 服务 `/mavros/mission/clear` | `mavros_msgs/srv/WaypointClear` | 清空飞控任务 |
| 服务 `/mavros/mission/set_current` | `mavros_msgs/srv/WaypointSetCurrent` | 设置当前航点索引 wp_seq |

只读检查：

```bash
ros2 service call /mavros/mission/pull mavros_msgs/srv/WaypointPull "{}"
ros2 topic echo /mavros/mission/waypoints --once \
  --qos-reliability reliable --qos-durability transient_local
```

pull 返回 success 和 wp_received。没有已上传任务时，wp_received=0 可以是正常结果。
push 请求包含 start_index 和 waypoints 数组；start_index=0 用于完整上传。
非零起始索引的部分更新需要飞控支持，不能默认可用。
响应字段拼写为 **wp_transfered**，不是 wp_transferred。

### Waypoint 主要字段

| 字段 | 中文说明 |
|---|---|
| frame | 坐标/高度基准；常用 FRAME_GLOBAL_REL_ALT=3 |
| command | MAV_CMD，例如 NAV_WAYPOINT=16、NAV_TAKEOFF=22、NAV_LAND=21 |
| is_current | 当前任务项标志 |
| autocontinue | 是否自动继续下一项 |
| param1～param4 | 随 command 改变的命令参数，不能统一解释 |
| x_lat / y_long | 全球任务项中为纬度/经度，度 |
| z_alt | 高度，m；基准由 frame 决定 |

对于普通 NAV_WAYPOINT，param1 是停留时间（秒）、param2 是接受半径（米）、param3 是穿越半径（米）、param4 是偏航（度）；飞控可能有额外约束。
FRAME_GLOBAL_REL_ALT 的高度相对 Home，FRAME_GLOBAL 为海拔基准；都不等于通用的离地高度。
ROS Waypoint 的经纬度使用度，不要自行乘 1e7，协议层转换由 MAVROS 处理。

上传任务成功不等于开始执行任务；任务执行还需要飞控状态满足条件并进入合适的任务模式，例如 PX4 AUTO.MISSION。
航点任务适合交给飞控执行预先安排的路线；持续轨迹控制通常使用 Offboard。
