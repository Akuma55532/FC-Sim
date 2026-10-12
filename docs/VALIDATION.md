# 教程验证记录

[文档导航](README.md) · [快速开始](QUICKSTART.md)

## 2026-10-12：构建与无界面 SITL 首次飞行

使用本机已有的依赖和 PX4 工作空间完成验证；未重新执行系统安装脚本，也未重置 PX4 已有参数。因此本记录证明当前环境可以完成教程主线，不代表全新系统安装已验收。

| 项目 | 实际环境 |
|---|---|
| 操作系统 | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| MAVROS | 2.16.0，核对现有安装的 `package.xml` |
| MAVLink | MAVROS 启动日志报告 `2026.9.9` |
| PX4 | `git describe` 为 `v1.17.0-beta1` |
| Gazebo | Harmonic，gz-sim 8.15.0 |
| 项目位置 | `/home/nv/FC-Sim` |
| PX4 位置 | `/home/nv/ws_simulator/PX4-Autopilot` |
| ROS 通信 | 域 42，`ROS_LOCALHOST_ONLY=1` |
| MAVROS 端点 | `udp://:14540@127.0.0.1:14580` |

### 构建与入口检查

按 [BUILD.md](BUILD.md) 的命令从仓库根目录构建，关闭测试目标，三个包均成功：

```text
Starting >>> flight_interfaces
Finished <<< flight_interfaces [14.0s]
Starting >>> flight_core
Finished <<< flight_core [19.7s]
Starting >>> flight_rviz_plugins
Finished <<< flight_rviz_plugins [29.2s]
Summary: 3 packages finished [1min 3s]
```

`ros2 pkg executables flight_core` 返回 `flight_core flight_actions_node`。MAVROS 和 RViz 启动文件均通过 `--show-args` 解析；MAVROS 默认地址与教程一致。

### 实际飞行

PX4 使用 `HEADLESS=1 make px4_sitl gz_x500` 启动，随后启动项目的 MAVROS 和核心入口。PX4 日志确认 Onboard 链路本地端口 14580、远端端口 14540，并报告 `Ready for takeoff!`。

起飞前，核心报告连接、遥测和位姿有效，`armed: false`、`ground_state: 1`。发送 `height_m: 1.0` 的起飞 Action 后，实际终态输出为：

```text
Result:
    result:
  outcome_code: 0
  message: telemetry confirmed

Goal finished with status: SUCCEEDED
```

随后收到的新状态摘录：

```yaml
operation: hold
phase: holding
connected: true
telemetry_valid: true
pose_valid: true
armed: true
control_active: true
holding: true
ground_state: 2
backend_mode: OFFBOARD
```

此时实际 Z 为约 1.025 m，目标 Z 为约 1.031 m，体现相对起飞高度与绝对目标 Z 的区别。

发送空目标的 `land_action` 后，同样得到 `SUCCEEDED`、`outcome_code: 0` 和 `telemetry confirmed`。落地后收到的新状态摘录：

```yaml
operation: land
phase: succeeded
armed: false
holding: false
ground_state: 1
```

该落地状态样本中 `backend_mode` 仍显示 `OFFBOARD`，`control_active` 为 `true`；不能单凭模式字符串或该字段判断仍在飞行。落地验收使用 Action 最终结果、地面状态及解锁状态。

此次命令行 Action 调用省略了 `--feedback` 以减少日志量；起飞、保持与降落均通过真实 PX4/Gazebo 仿真完成，没有使用假 MAVROS 服务。

### 验证边界

已验证仓库根目录构建、启动文件解析、MAVROS 连接及教程使用的遥测查询 QoS、起飞—保持—降落主线。

尚未验证全新系统安装、默认参数重置后的 PX4、Gazebo 图形窗口、实际 RViz 插件加载和按钮操作、航点任务、stop/取消及故障恢复。此次未运行逻辑单测或 Qt 离屏测试，后续开发测试章节会单独记录。

## 2026-10-12：面板测试、示例文件与航点飞行

沿用上述 PX4/MAVROS/Gazebo 环境，重新启动无界面 SITL，在 ROS 域 42 验证。测试客户端读取仓库的 [waypoints_square.yaml](examples/waypoints_square.yaml)，将 `yaw_deg` 转换为弧度后提交 Action；未通过面板按钮发送本次飞行任务。

### 实际航点结果

| 步骤 | 实际结果 |
|---|---|
| 相对起飞 1 m | SUCCEEDED，结果码 0，随后 holding=true |
| 非循环五点方形任务 | SUCCEEDED，结果码 0，completed_count=5；保持最后目标 |
| 再次提交相同航线并开启循环 | 观察到 cycle_count=1、completed_count=5 |
| 调用 stop 服务 | success=true，`mission stop accepted; observe result` |
| 被停止的任务 | ABORTED，结果码 8，`mission stopped; holding current pose` |
| 停止后的新状态 | holding=true、mission_active=false |
| 降落收尾 | SUCCEEDED，结果码 0；ground_state=1、armed=false |

因此教程明确区分“停止成功并保持”与“航线正常完成”；不能因 stop 后任务终态是 ABORTED 就单独认定停止失败。

### 文件和面板自动化

启用已有测试目标并执行面板包测试：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/mavros_humble_ws/install/setup.bash"
export FC_SIM_ROOT="$HOME/FC-Sim"
cd "$FC_SIM_ROOT"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths pkgs/flight_pkgs --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source "$FC_SIM_ROOT/install/setup.bash"
colcon test --packages-select flight_rviz_plugins --event-handlers console_direct+
colcon test-result --verbose
```

两个 CTest 目标均通过，包含 3 个文件测试和 4 个面板测试，合计 7 个 gtest 用例；colcon 汇总同时统计 CTest 记录，显示 `9 tests, 0 errors, 0 failures, 0 skipped`。

文件测试覆盖多车辆往返、非法输入及空草稿；面板测试在 Qt offscreen 和独立 ROS 域 185 中验证插件发现、编辑不发送请求、过期状态禁用控制、异步结果和单步成功后选行。另使用项目的 `decode/encode` C++ 实现读取本次示例并往返编码，确认 `/flight_core` 下保留五个航点。

另外实际启动 `panel.launch.py` 运行约 12 秒，日志确认 RViz/OpenGL 4.6 初始化且无插件加载错误，随后正常退出。这是启动检查，未检查画面内容。

尚未完成真实 RViz 窗口中的按钮飞行、文件选择对话框保存/加载、2D Goal Pose 点选和画面人工验收。文件解析验证、离屏面板测试与真实 SITL Action 飞行分别记录，不能替代完整图形操作验收。

## 2026-10-12：Python 编程接入示例

对 [flight_sequence.py](examples/flight_sequence.py) 使用同一真实 PX4/Gazebo SITL 环境、ROS 域 42 执行：

```bash
python3 "$FC_SIM_ROOT/docs/examples/flight_sequence.py" --height 1.0
```

实际正常流程返回退出码 0，关键日志摘录：

```text
takeoff: status=4, code=0, telemetry confirmed
waypoints: status=4, code=0, all waypoints reached; holding final target
Two waypoints completed; landing next
land: status=4, code=0, telemetry confirmed
Landing confirmed: ground_state=1, armed=false
```

航点以起飞后实际保持位置为基准，沿地图 +X 移动 1 m 再返回；客户端确认 completed_count=2。反馈回调也实际收到 executing、settling 和航点索引变化。

另一次飞行在收到首个航点执行反馈时向客户端发送 SIGINT（等价于一次 Ctrl+C），实际收到 `Sequence stopped: KeyboardInterrupt`，随后降落 Action 成功并确认地面/未解锁状态，程序退出码为 1。未继续完成原航线。

在独立无核心的 ROS 域 186 中，示例在起飞前等待约 30 s 后退出，退出码为 1，没有请求降落。`--height nan`、`--height 0`、非绝对 `--core` 均在初始化 ROS 前以参数错误退出。

文档链接、Bash 代码块语法和 Python 语法检查通过。持续断链、运行中核心崩溃、所有拒绝/超时分支和 Action cancel 清理未在本次示例验证中逐项注入，不能视为已覆盖；取消语义依据现有核心源码说明。
