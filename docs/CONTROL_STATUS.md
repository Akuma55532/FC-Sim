# 07 · 状态与结果参考

[文档导航](README.md) · [编程接入](ROS_API.md) · [面板教程](RVIZ_PANEL.md)

`/flight_core/control_status` 是核心对当前遥测、动作和任务的汇总，不是某个客户端专属的完成通知。判断自己提交的请求是否成功，使用对应 Action 终态和结果；判断飞机当前状况，使用新鲜状态与有效性字段。

依据：[消息定义](../pkgs/flight_pkgs/flight_interfaces/msg/ControlStatus.msg)、[节点实现](../pkgs/flight_pkgs/flight_core/src/nodes/flight_actions_node.cpp)、[MAVROS 后端](../pkgs/flight_pkgs/flight_core/src/backends/mavros/mavros_backend.cpp)。以下描述当前实现，不是对未来后端的假设。

## 发布频率、QoS 与时效

核心使用 50 ms 墙上时钟定时器，名义发布频率为 20 Hz；阻塞或系统负载会影响实际间隔。发布 QoS 为 reliable、transient_local、深度 1。核心拒绝 `use_sim_time=true`。

```bash
ros2 topic echo /flight_core/control_status flight_interfaces/msg/ControlStatus \
  --qos-reliability reliable --qos-durability volatile
```

上例等待新发布消息。使用 transient_local 订阅可拿到缓存，但缓存不能证明节点仍在持续更新。`header.stamp` 是核心组装状态的时间，不是原始传感器采样时间；`header.frame_id` 当前为 `map`。

建议检查：接收间隔是否持续正常、时间戳是否过期、telemetry_valid/pose_valid 是否为真。面板采用约 1 s 的状态过期判据；后端另有下游遥测时效检查，二者不能互相替代。

## 连接、位姿与控制字段

| 字段 | 当前含义 | 使用注意 |
|---|---|---|
| `connected` | MAVROS 报告连接，且 State 遥测新鲜 | 不代表估计器就绪或可以解锁 |
| `telemetry_valid` | 已连接，State 和 ExtendedState 都新鲜 | 不包含有效位姿要求 |
| `pose_valid` | telemetry_valid，位姿新鲜且几何数值有效 | 假时不得将 current_pose 当作有效实时位置 |
| `armed` | 最近遥测报告的解锁状态 | 遥测无效时可能是旧值 |
| `control_active` | MAVROS 模式字符串等于 OFFBOARD | 不代表核心拥有控制权、不保证已解锁或在空中 |
| `holding` | 后端处于保持，且没有活动航点任务或待执行的失败降落清理 | 与新鲜遥测及其他状态一起判断 |
| `ground_state` | 0=未知，1=地面，2=空中 | 起飞/降落中也归入空中；不要当成 MAVROS landed_state 的原值 |
| `backend_mode` | 当前后端报告的飞控模式字符串 | 模式可能显示为 CMODE(...)；不应单独据此判断完成 |
| `current_pose` | 最近缓存的本地位姿 | 距离单位 m，ROS 四元数 x/y/z/w，所属帧在 header 中 |
| `target_pose` | 当前显示的动作或任务目标 | 可能是保留的旧目标，不保证此刻仍在向飞控发送 |

当前 MAVROS 适配的默认时效：State 与 ExtendedState 各 3 s、位姿 0.5 s、未来时间戳容差 0.1 s，且检查接收时效和时间戳。位姿须为 map 帧并通过数值校验。这些值是 C++ Config 默认值，当前并非公开 ROS 参数。

MAVROS ExtendedState 的 landed_state 为 1 时映射地面；2/3/4 映射空中；扩展状态过期或其他值映射未知。

实测落地后可能仍有 `control_active: true` 和 `backend_mode: OFFBOARD`。落地确认应结合降落 Action 成功、ground_state=1、armed=false，而不是等待 control_active 变为 false。

## 操作、阶段与说明

| 字段 | 用途 |
|---|---|
| `operation` | 描述当前显示的操作：arm、disarm、takeoff、land、hold、waypoints |
| `phase` | 描述显示阶段，见下表；不是 ROS Action 协议状态 |
| `operation_id` | 核心内部请求标识，不是 Action UUID，也不保证一个用户任务始终同一值 |
| `outcome_code` | 当前显示的结果码，执行中也可能为 0；不等于已经成功 |
| `detail` | 人可读说明，可被后端失败信息覆盖；不要解析字符串作为稳定控制协议 |

operation_id 在核心重启后可能重复；航点后端请求使用另一编号区间，推进航点或切换保持时可能更新。客户端应保留自己的 Action goal handle 来关联请求。

| phase | 解释 |
|---|---|
| `idle` | 尚未执行动作；初始 operation 可能为 arm，不代表已发解锁 |
| `executing` | 正在执行或向目标运动 |
| `settling` | 满足到点条件，正在累计停留时间 |
| `succeeded` | 显示的动作或任务逻辑已成功 |
| `failed` | 动作、任务或后端报告失败 |
| `canceling` | 取消后的清理正在执行，例如起飞取消转降落 |
| `canceled` | 任务/动作逻辑取消完成；对应 Action 终态还取决于取消途径 |
| `holding` | 非保留航点展示覆盖时，显示普通位置保持 |
| `failure_hold` | 某些航点失败后的临时保持，计划约 2 s 后请求降落 |

字段存在显示优先级：先组合基本动作，普通保持可覆盖 operation/phase，保留的航点记录再覆盖这些描述，最后可能被失败收尾和后端拒绝覆盖。因此可能同时出现 `operation: waypoints`、`phase: succeeded`、`holding: true`；这表示任务已完成且正在保持，并不矛盾。

对航点 Timeout 或 InvalidGoal，若遥测、解锁和控制条件允许，核心尝试临时保持并安排降落。若条件丢失，不能假定自动降落一定成功。不能把 `failure_hold` 当作用户可长期停留或自由恢复的正常保持。

## 航点进度

| 字段 | 含义 |
|---|---|
| `mission_active` | 航点任务逻辑处于执行或到点停留 |
| `waypoint_index` | 当前或保留航点索引，从 0 开始 |
| `completed_count` | 已完成点数，循环时跨圈累计 |
| `cycle_count` | 循环任务完成的圈数；非循环任务完成时不要求为 1 |
| `dwell_elapsed_sec` | 当前连续满足到点条件的停留时长 |

任务成功或停止后，进度可能继续保留。`mission_active=false` 本身不区分成功、失败、取消或尚未执行；需结合 Action 结果。后续基本动作可能清除航点展示，此时不能再用状态话题恢复之前任务的完整结果。

## 结果码

代码定义在 [ActionResult.msg](../pkgs/flight_pkgs/flight_interfaces/msg/ActionResult.msg)，Action 结果与状态话题共用这些数值。

| 数值 / 名称 | 含义 | 客户端处理方向 |
|---|---|---|
| 0 OK | 正常结果，或执行中的无错误值 | 仍需检查对应 Action 是否 SUCCEEDED |
| 1 BUSY | 有其他操作正在执行 | 等待或主动收尾当前操作，避免连续重发 |
| 2 UNSUPPORTED | 后端不支持操作 | 检查接口和后端能力 |
| 3 INVALID_GOAL | 目标或准入条件不合法 | 检查高度、坐标、距离、地面状态和任务选项 |
| 4 TELEMETRY_LOST | 必需遥测或位姿不可用 | 检查通信、时效和位姿有效性 |
| 5 BACKEND_REJECTED | 后端请求或控制流程失败 | 阅读 message/detail 及 MAVROS/PX4 日志 |
| 6 TIMEOUT | 核心动作或单点期限超过 | 观察失败收尾，不直接提交下一任务 |
| 7 CONTROL_LOST | 控制条件不满足或丢失 | 核对当前核心是否建立控制及飞控实际状态 |
| 8 PREEMPTED | 被停止、取消或降落抢占 | 结合请求途径判断是否是预期结束 |
| 9 INTERNAL_ERROR | 内部条件异常，例如任务定时间隔超限 | 保留日志并检查运行负载和控制状态 |

忙碌也可能在 goal 接受之前直接拒绝，因此不保证总能拿到 BUSY 结果。Action 完成后，状态话题可能显示后续动作或后端错误；自己的请求结果应从 get_result_async() 获取。

## 常用判断组合

- **准备起飞：**状态新鲜、telemetry_valid、pose_valid、ground_state=1、armed=false；这些是客户端预检查，核心仍执行完整校验。
- **准备提交航点：**状态新鲜、有效位姿、armed、ground_state=2、holding；仅 OFFBOARD 不足以证明可以提交。
- **航点完成：**对应 Action 为 SUCCEEDED 且结果码 0；再查看保持情况。
- **停止完成：**停止请求被接受，并结合原任务终态及新状态的 mission_active=false、holding=true 判断。
- **落地完成：**降落 Action 成功，并有新鲜地面和未解锁状态。

不要编写 `phase == 'succeeded'` 或 `outcome_code == 0` 就执行下一步的通用判断。这些描述可能属于历史记录，且不能替代自己请求的 Action 终态。
