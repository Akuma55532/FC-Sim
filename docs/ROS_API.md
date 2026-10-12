# 06 · 编程接入

[文档导航](README.md) · 前置：[首次飞行](QUICKSTART.md) · [状态参考](CONTROL_STATUS.md)

本章用 Python ROS 2 客户端安排“起飞 → 两个航点 → 降落”。你的程序决定任务顺序，`flight_core` 负责持续目标输出、MAVROS 请求和动作完成判断。此接口提供离散动作与航点任务，不是持续速度或姿态控制接口。

## 1. 接口地图

默认核心名称为 `/flight_core`；以下路径都相对此名称。节点重命名或命名空间变化时应相应调整客户端路径。

| 相对路径 | 类型 | 用途 / 目标 |
|---|---|---|
| `/arm` | `flight_interfaces/action/Arm` | 地面解锁，空目标 |
| `/disarm` | `flight_interfaces/action/Disarm` | 地面上锁，空目标 |
| `/takeoff` | `flight_interfaces/action/Takeoff` | 相对起飞高度 `height_m`，范围 `(0,10]` m |
| `/land_action` | `flight_interfaces/action/Land` | 降落，空目标，可抢占当前任务 |
| `/execute_waypoints` | `flight_interfaces/action/ExecuteWaypoints` | 执行绝对 map/ENU 航点 |
| `/stop` | `std_srvs/srv/Trigger` | 请求停止；具体行为取决于当前操作 |
| `/land` | `std_srvs/srv/Trigger` | 请求降落，仅返回请求是否接受 |
| `/control_status` | `flight_interfaces/msg/ControlStatus` | 全局状态与任务进度 |
| `/pose` | `geometry_msgs/msg/PoseStamped` | 核心转发的已通过校验的 MAVROS 位姿，SensorDataQoS |

先查询实际接口：

```bash
ros2 action list -t
ros2 interface show flight_interfaces/action/ExecuteWaypoints
ros2 interface show flight_interfaces/msg/ControlStatus
```

`/land` 不是 Action，等待降落完成优先使用 `/land_action`。正常任务逐个提交；核心忙碌时一般拒绝新动作，降落有抢占规则。

## 2. 运行完整示例

脚本：[flight_sequence.py](examples/flight_sequence.py)。先按快速开始启动 PX4、MAVROS 和核心，飞机处于地面且未解锁；新终端执行[公共环境前缀](BUILD.md#5-后续每个-ros-终端的公共前缀)。示例依赖 `rclpy`、`action_msgs`、`flight_interfaces`，这些在教程环境中已具备。

```bash
python3 "$FC_SIM_ROOT/docs/examples/flight_sequence.py" --help
python3 "$FC_SIM_ROOT/docs/examples/flight_sequence.py" \
  --core /flight_core --height 1.0
```

第二条命令会实际控制仿真飞机。流程是：

1. 最多等待 30 s，确认新鲜状态、有效位姿、在地面且未解锁。
2. 发送起飞请求，检查 Action 成功，再等待空中保持状态。
3. 读取保持位置与 yaw，生成两个绝对航点：沿 ENU 的 +X 移动 1 m，然后回到该位置。高度和 yaw 保持不变。
4. 等待两个航点均完成。
5. 请求降落并确认在地面、未解锁，最后退出。

此处 +X 是地图的东向，不是机头前方。示例不使用原点假设，也不执行避障。需要不同路线时修改 `goal.waypoints`；航点限制见[航点教程](WAYPOINTS.md)。

预期日志包括 `takeoff: status=4, code=0`、`waypoints: status=4, code=0`、`land: status=4, code=0`，最后为 `Landing confirmed: ground_state=1, armed=false`。这是字段示意；本次运行结果见[验证记录](VALIDATION.md)。退出码 0 表示整段流程和降落确认成功，非零表示某一步失败或被中断，即使最终已降落也保留失败退出码。

## 3. Action 调用的三个阶段

示例使用 `ActionClient.send_goal_async()`；等待时调用 `rclpy.spin_once()`，使状态、反馈和 Future 回调继续运行。不要用阻塞 `sleep()` 等待消息自己更新，也不要把此示例的同步等待循环直接放进单线程节点回调。

1. **服务端可用：**`wait_for_server()` 只证明接口存在。
2. **目标接受：**`future.result().accepted` 表示核心接收目标。忙碌时可能直接拒绝，此时没有该目标的执行结果；其他准入检查也可能在接受之后失败。
3. **最终结果：**`handle.get_result_async()` 返回 ROS Action 状态和自定义结果，两者都要检查。

Python 中嵌套字段的读取方式：

```python
response = result_future.result()
ros_status = response.status
outcome = response.result.result.outcome_code
message = response.result.result.message
completed = response.result.completed_count  # 仅 ExecuteWaypoints
```

正常完成要求 `ros_status == GoalStatus.STATUS_SUCCEEDED`（4）且 `outcome == 0`。其他常见终态是 CANCELED（5）与 ABORTED（6）。自定义结果码见[状态参考](CONTROL_STATUS.md#结果码)。

## 4. 反馈与航点目标

反馈回调参数是封装消息，`event.feedback` 才是 Action 反馈：

- 基本动作公共字段在 `event.feedback.feedback`：phase、当前位置/目标、位姿有效性、位置及 yaw 误差。
- 航点额外提供 `current_index`、`completed_count`、`cycle_count`、`dwell_elapsed_sec`。
- `current_index` 从 0 开始；位姿不可用时误差可为 `-1`，不能作为有效测量参与控制。
- feedback.phase 是描述，不是 Action 的最终状态；示例仅在阶段或航点变化时打印反馈。

ExecuteWaypoints 的目标字段：

| 字段 | 单位 / 规则 |
|---|---|
| `frame_id` | 当前只接受 `map` |
| `waypoints` | 1～500 个 Waypoint，XYZ 为绝对米制 ENU，`yaw_rad` 为弧度 |
| `position_tolerance_m` | 正数，三维位置误差阈值 |
| `check_yaw` | 是否检查 yaw 到点；关闭仍会发送 yaw 目标 |
| `yaw_tolerance_rad` | `(0, π]`，即使关闭检查也需为合法值 |
| `dwell_sec` | 非负，到点持续停留时间 |
| `waypoint_timeout_sec` | 正数，不小于 dwell；每个点单独计时 |
| `loop` | true 时循环，任务不自行正常完成 |

不要依赖客户端语言的默认初始化差异，示例显式设置任务选项。航点目标与面板 YAML 的结构不同，尤其 yaw 字段单位不同，不能原样互换。

## 5. 状态订阅与失败处理

核心发布可靠、transient-local、深度 1 的状态。示例使用可靠、volatile、深度 1 的订阅，等待新消息，并同时检查接收间隔和消息时间戳。消息更新不代表下游遥测一定有效，还需要 `telemetry_valid` 与 `pose_valid`。

示例将以下情况视为失败，停止后续航点步骤：目标拒绝、非成功终态、结果码非零、状态失联、遥测失效、客户端超时。位姿丢失等下游条件也由核心独立检查。

只要尝试发过起飞，`finally` 就会尝试请求降落，并等待落地确认。因为发送后可能丢失回复，脚本在发送前标记“已请求起飞”。这是供单客户端 SITL 学习的收尾策略；多个客户端共享核心时，不应照搬为无条件抢占别人任务的策略。

客户端超时不会取消服务端任务，停止程序也不会停止核心。示例保留 ROS 上下文以便一次 Ctrl+C 后尝试降落；再次中断、进程被强制结束、持续断链或核心退出都可能使收尾失败。此时会报告 `Landing NOT confirmed`，应查看核心/PX4 状态并在通信可用时按快速开始的降落流程收尾。示例不是独立失效保护系统。

示例的 1 s 状态新鲜度和 75/130 s 等待期限是客户端策略，不是可修改的核心参数。

## 6. 停止、取消与降落抢占

| 操作 | 当前实现 |
|---|---|
| 航点中调用 stop | 尝试保持当前位置；服务接受后仍需等任务结果和状态；任务通常 ABORTED / PREEMPTED(8) |
| 航点 Action cancel | 使用该 goal handle 的 `cancel_goal_async()`；接受后完成保持收尾，正常取消路径为 CANCELED / 8 |
| 起飞 Action cancel | 接受后执行降落清理，确认完成才结束取消 |
| 起飞中调用 stop | 请求同类降落清理；因不是 Action cancel 协议，终态可为 ABORTED / 8 |
| Arm / Disarm / Land Action cancel | 当前不支持，取消请求被拒绝 |
| 发送降落 Action | 可抢占任务；被抢占动作/任务返回 PREEMPTED，降落本身另有最终结果 |

取消请求的响应不是动作结束，需检查响应的 `goals_canceling` 并继续等待原 Action 结果；取消或停止也可能因条件不满足而拒绝。失败降落清理期间 stop 会被拒绝。核心已丢失控制时不能假定所有停止请求都能建立保持。

这些交互语义已对照实现；本章的自动脚本只演示普通顺序执行与降落收尾，不包含交互式取消界面。
