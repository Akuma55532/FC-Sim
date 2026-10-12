# flight_core 代码导览

[工作空间首页](../../README.md) · [文档导航](../../docs/README.md) · [开发与测试](../../docs/DEVELOPMENT.md)

找通用动作与任务逻辑看 `core/`，找 MAVROS/PX4 的适配看 `backends/mavros/`。
头文件位于 `include/flight_core/`，实现位于 `src/`，两边使用一致的职责目录。

## 目录与职责

```text
flight_core/
├── include/flight_core/
│   ├── core/                  # 基本动作、航点任务、FlightBackend 抽象接口
│   ├── backends/mavros/        # 通信、PX4 流程及后端专用类型
│   ├── nodes/                 # 节点侧进程互斥工具
│   └── common/                # 共享类型与数值工具
├── src/
│   ├── core/                  # 基本动作和航点执行逻辑
│   ├── backends/mavros/        # MavrosBackend、MavrosAdapter 实现
│   └── nodes/                 # 唯一 Action 节点的装配入口
├── config/                    # 节点参数、MAVROS GCS 覆盖配置
├── launch/                    # 保留现有启动文件名
└── test/                      # 逻辑单元测试和隔离通信测试
```

## 核心层：决定“执行什么、何时完成”

| 文件 | 职责 |
|---|---|
| [core/flight_backend.hpp](include/flight_core/core/flight_backend.hpp) | 核心需要的后端契约：能力、遥测、begin/poll/detach；不依赖 MAVROS 消息或 ROS |
| [core/basic_action_machine.cpp](src/core/basic_action_machine.cpp) | 解锁、上锁、起飞、降落的通用准入与完成条件、超时和取消 |
| [core/waypoint_mission.cpp](src/core/waypoint_mission.cpp) | 航点顺序、XYZ/yaw 到点判断、停留、循环和停止 |

`FlightBackend` 属于核心定义的接口，具体后端来实现它。核心不需要根据 MAVLink 是否存在选择控制逻辑。
例如修改到点判据应从 `core/waypoint_mission.cpp` 开始，不需要修改 MAVROS 话题收发。

## MAVROS/PX4 适配层：决定“如何让这个下游执行”

| 文件 | 职责 |
|---|---|
| [mavros_backend.cpp](src/backends/mavros/mavros_backend.cpp) | 实现 FlightBackend；负责 PX4 的预发送、OFFBOARD/AUTO.LAND/AUTO.LOITER、解锁确认、地面模式复位与持续目标输出 |
| [mavros_adapter.cpp](src/backends/mavros/mavros_adapter.cpp) | ROS/MAVROS 通信：订阅状态和位姿、发送设定值、异步请求模式/解锁、验证遥测时效 |

这里的后端是 **MAVROS + PX4 的具体实现**，不能认为只要使用 MAVROS 就天然兼容任意飞控。
修改 MAVROS 话题、消息转换或服务调用看 Adapter；修改 PX4 模式准备与确认流程看 Backend。

## ROS 入口：装配和协调

- [nodes/flight_actions_node.cpp](src/nodes/flight_actions_node.cpp)：对外 Action/服务、ControlStatus 发布、动作与任务协调，并实例化当前 MAVROS 后端。
- [nodes/control_lock.hpp](include/flight_core/nodes/control_lock.hpp)：同机同 ROS 域与 MAVROS 端点的进程互斥。

```text
Action / RViz
    ↓
nodes/flight_actions_node
    ├── core/BasicActionMachine、WaypointMission
    │       ↓ core/FlightBackend 接口
    └── backends/mavros/MavrosBackend
                ↓ MavrosAdapter
              MAVROS → PX4
```

ROS 节点仍负责部分任务协调与失效收尾，不是只有 main() 的薄封装；本次目录整理没有把这些逻辑再拆成新模块。

## 共享类型与后端边界

[common/types.hpp](include/flight_core/common/types.hpp) 仅包含 Pose 与位姿校验、距离、偏航工具。
[backends/mavros/types.hpp](include/flight_core/backends/mavros/types.hpp) 保存具体后端需要的 Config、Telemetry、Action 和 Request。
核心通过 FlightBackend 使用统一 BackendState，MAVROS 类型不再放在公共几何类型文件中。

新增其他下游时，实现 `core/flight_backend.hpp`，放入 `backends/<后端名>/`，并在节点入口装配。
目前没有运行时插件选择框架，单个 flight_core 包仍声明 MAVROS 依赖；独立后端包可在确有第二种实现时再拆。

## 最小版本与兼容性

只保留 flight_actions_node 和 basic_actions.launch.py；旧算法流节点、启动、消息、参数、示例和专用测试已移除。
MAVROS 统一使用 mavros_gcs.launch.py，面板入口保持不变。现有 Action/航点接口、状态及控制行为保持不变。
类型头文件已分层，外部直接使用旧 C++ 接口的代码需迁移并重新构建。测试方法见 [开发与测试](../../docs/DEVELOPMENT.md)。
