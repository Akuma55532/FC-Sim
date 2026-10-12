# 05 查询、QoS 与排障

[返回目录](README.md)

## 1. 查询当前系统真实接口

```bash
source /opt/ros/humble/setup.bash
source ~/mavros_humble_ws/install/setup.bash

ros2 node list
ros2 topic list -t
ros2 service list -t

ros2 topic info /mavros/local_position/pose --verbose
ros2 interface show mavros_msgs/msg/State
ros2 interface show mavros_msgs/msg/PositionTarget
ros2 interface show mavros_msgs/srv/SetMode
ros2 interface show mavros_msgs/srv/ParamSetV2
```

服务定义以 --- 分隔请求和响应。类型文件中的常量不是需要发送的额外字段。
topic info --verbose 可显示发布者、订阅者与 QoS；仅看到话题名称不能证明有活跃的数据流。

插件配置参数以实际节点名查询：

```bash
ros2 param list /mavros/setpoint_velocity
ros2 param get /mavros/setpoint_velocity mav_frame
```

如果节点路径不存在，先查看 ros2 node list；多机系统可能有不同命名空间。

## 2. QoS：为什么话题存在却收不到

**官方说明译文：**MAVROS 为传感器、状态、参数事件和服务使用不同的 QoS 配置。

来源：[MAVROS QoS](https://mavros.readthedocs.io/en/latest/plugins/qos/)、[ROS 2 Humble QoS](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)。

| 配置 | 可靠性 | 持久性 | 队列深度 | 常见用途 |
|---|---|---|---|---|
| SensorDataQoS | Best effort | Volatile | 5 | IMU、位置等高频遥测 |
| StateQoS | Reliable | Transient local | 10 | state、extended_state |
| LatchedStateQoS | Reliable | Transient local | 1 | 任务列表等 |
| ServicesQoS | Reliable | Volatile | 10 | 普通服务 |

Best effort 发布者不能满足要求 Reliable 的订阅者；Reliable 发布者可以满足 Best effort 订阅者。
想在订阅后立即收到之前保留的状态，需要发布端支持 Transient local，并让订阅端也使用匹配的持久性。
实际配置以 ros2 topic info --verbose 为准。

```bash
ros2 topic echo /mavros/imu/data --qos-reliability best_effort

ros2 topic echo /mavros/state --once \
  --qos-reliability reliable --qos-durability transient_local
```

### ROS 2 Humble Python 订阅示例

下面代码仅订阅本地位姿，可保存为 read_pose.py 后使用已加载 MAVROS 环境的 python3 运行：

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped


class PoseReader(Node):
    def __init__(self):
        super().__init__("mavros_pose_reader")
        self.subscription = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.on_pose,
            qos_profile_sensor_data,
        )

    def on_pose(self, msg):
        p = msg.pose.position
        self.get_logger().info(
            f"ENU position: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f} m"
        )


def main():
    rclpy.init()
    node = PoseReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
```

高频场景可降低日志频率；示例侧重 QoS 与订阅方法。

## 3. 不起飞的链路验证

```bash
# 检查心跳
ros2 topic echo /mavros/state --once

# 检查持续遥测，Ctrl+C 停止
ros2 topic hz /mavros/imu/data

# 强制从飞控读取参数，验证请求和应答双向传输
ros2 service call /mavros/param/pull mavros_msgs/srv/ParamPull "{force_pull: true}"
```

通过标准：connected=true；IMU 持续有数据；参数读取 success=true 且 param_received>0。
这证明基本通信，不是全套飞行健康检查。
hz 的结果是当前订阅者观察到的频率，会受 QoS、CPU 负载和丢包影响。

## 4. 常见问题

| 现象 | 优先检查 |
|---|---|
| State 消息类型 invalid | 当前终端是否 source 正确的 install/setup.bash；用 ros2 interface show 验证 |
| connected=false | PX4 是否运行、UDP 端口是否匹配、是否有另一个 MAVROS 占用端口 |
| connected=true 但无位置 | 定位和估计器状态、PX4 消息流、插件及 QoS |
| 有话题但算法订阅不到 | topic info --verbose，特别是 Best effort/可靠性匹配 |
| mode_sent=true 但仍非 OFFBOARD | state.mode、目标流频率与持续时间、估计器条件、statustext |
| param get 找不到飞控参数 | 先 param/pull；节点通常是 /mavros/param，而不是 /mavros |
| 发位置目标却向错误方向运动 | ENU/NED 是否重复转换，目标是否为绝对本地坐标 |
| 外部里程计没有融合 | 插件日志、TF、时间戳、协方差、PX4 外部观测融合配置 |
| ROS 1 教程命令不能用 | 改用 ROS 2 消息/服务和 ros2 CLI；接口名也可能变化 |

消息模块检查：

```bash
ros2 pkg prefix mavros_msgs
python3 -c "from mavros_msgs.msg import State; print(State)"
```

仅切换到工作空间目录不等于加载环境。所有通信节点还应使用一致的 ROS_DOMAIN_ID；
涉及仿真时间时，只有在确实发布 /clock 并统一时钟策略后再使用 use_sim_time，避免时间戳不推进。

## 5. 记录实验数据

```bash
ros2 bag record /mavros/state /mavros/extended_state \
  /mavros/local_position/pose /mavros/local_position/velocity_local \
  /mavros/imu/data /mavros/setpoint_position/local
```

实验后结合 PX4 ULog 对照控制目标、飞控状态和运动响应。确保 rosbag2 对实际话题的 QoS 兼容。
