# flight_rviz_plugins：独立 RViz2 飞行控制面板

[文档导航](../../docs/README.md) · 前置：[快速开始](../../docs/QUICKSTART.md) · [状态参考](../../docs/CONTROL_STATUS.md) · [排障指南](../../docs/TROUBLESHOOTING.md)

面向 ROS 2 Humble。仅依赖 flight_interfaces 及标准 ROS/RViz/Qt 接口，不依赖 flight_core 的 C++ 实现、MAVROS 或 PX4。
基本动作与任务执行由独立运行的 flight_core 负责；面板不发送持续控制目标，不在界面内判断任务完成。

## 启动

```bash
source ~/ws_simulator/flight_ws/install/setup.bash
ros2 launch flight_rviz_plugins panel.launch.py
```

此入口只启动 RViz2。飞控、MAVROS 和 `ros2 launch flight_core basic_actions.launch.py` 分别运行。
也可在已有 RViz2 中选择 Panels → Add New Panel → flight_rviz_plugins/FlightPanel。
面板需要 basic_actions.launch.py 提供的 Action 和 ControlStatus 接口。

默认核心名称 `/flight_core`，可改为 `/uav1/flight_core` 等绝对名称后点击连接。
RViz 配置保存该名称；航点内容通过 YAML 文件保存，不隐式保存到 RViz 配置。
第一版只操作一个核心；切换名称时分别保留内存中的航点草稿，文件结构预留多机。
有活动请求且状态新鲜时不允许切换；状态失联时可重新连接，但这不会取消原核心任务。

## 使用流程

1. 核对连接、遥测、位姿、模式和地面状态，确认核心连接对象正确。
2. 设置起飞相对高度并点击起飞，等待动作成功及保持状态。也可单独地面解锁、上锁。
3. 编辑航点或点击“添加当前位置”；航点坐标为绝对 map/ENU，偏航输入单位为度。
4. 设置 XYZ 阈值、偏航判据、停留时间和单点超时；点击执行全部或单步执行选中点。
5. 停止按钮通过核心 stop 服务保持当前位置；降落按钮请求 land_action，可抢占任务。

起飞、解锁、上锁、降落按钮会发送实际控制请求；打开面板、加载文件和编辑草稿不会发送控制请求。
操作区结果行显示请求接受/拒绝及 Action 最终结果；状态区持续显示实际状态与错误原因。
收到状态超过 1 s 未更新，或状态时间戳过期时，控制按钮禁用；服务端还会独立执行完整准入检查。
关闭 RViz2 或失去面板连接不会取消已经提交的任务；循环任务尤其需要显式停止或降落。

## 状态卡片

- 顶部显示遥测是否实时更新，四个徽标分别表示连接、状态遥测、位置姿态和解锁状态。
- 绿色表示正常或已上锁，橙色表示无效或已解锁，灰色表示未知；同时提供文字说明。
- 分列显示飞控模式、中文操作/执行阶段及着陆状态；操作和阶段的英文原值可通过鼠标悬停查看。
- X/Y/Z 以米显示，Yaw 以度显示；位姿无效或状态过期时显示 `—`，避免将旧数值误认为当前位置。
- 航点进度、位置保持、状态说明分别显示，失败及故障保持使用红色提示；操作请求结果单独显示在卡片下方。

字段含义见 [ControlStatus 说明](../../docs/CONTROL_STATUS.md)。本次调整仅涉及面板展示，控制准入及动作执行逻辑保持原有规则。

## 航点编辑与到点判断

- 添加、插入、删除、上移、下移，直接编辑 X/Y/Z/Yaw；容量默认 50，上限 500。
- 容量调小不会截断已有航点，只限制继续添加。
- “新增点默认 Z”用于空点及 RViz 点选；“添加当前位置”使用当前遥测 XYZ/yaw。
- 默认检查 XYZ 距离及 yaw 阈值。取消“检查偏航到达”仅跳过偏航到点判据，仍发送 yaw 目标。
- 单步只提交选中一个点，禁用该次任务的循环；真正成功后才选中下一行。执行期间修改了航点表则不自动跳行。
- 已发送的是航点快照；编辑表格或加载文件不会改变执行中的任务。
- 界面不插值或规划避障轨迹；核心仍会验证到当前点/相邻点/闭合段的距离与任务条件。

## RViz 点选与可视化

配套配置使用 Fixed Frame=map。工具栏 2D Goal Pose 发布到 `/flight_panel/goal`，单击并拖动设置 XY 与偏航，
面板用“新增点默认 Z”补高度。只接受 map 帧及有效四元数；不隐式转换其他坐标系。

MarkerArray 发布到 `<核心名称>/panel_markers`：蓝色箭头与编号为草稿，橙色箭头标识核心报告的当前目标。
改草稿不改变橙色实际目标。状态过期后不显示当前目标，草稿仍可编辑。
默认 RViz 配置订阅 `/flight_core/panel_markers`；更换核心名称后需同步修改 MarkerArray 显示的 Topic。
同一核心建议只开一个面板实例，避免同名 Marker 互相覆盖；这不影响核心的任务仲裁。

## 实际飞行轨迹

面板从 `<核心名称>/control_status` 的 `current_pose` 记录实际位置，在现有 `panel_markers` 中发布青绿色线条（命名空间 `actual_trajectory`）。
配套 RViz 配置默认显示；已有 RViz 配置只需启用订阅该话题的 MarkerArray，并将 Fixed Frame 设为 map。
轨迹与蓝色航点草稿、橙色当前目标含义不同：它表示面板收到的实际位置历史，不是规划路径。

- “显示实际轨迹”默认勾选，取消仅隐藏，后台继续记录；“清空轨迹”删除历史，从下一份有效位置重新记录。
- 只记录连接、遥测和位姿均有效且新鲜的 map 坐标，按 XYZ 距离至少 5 cm 增加线段，保留最近 2000 条线段，线宽 3.5 cm。
- 遥测过期、时间戳间隔超过 1 s、乱序、位姿无效或坐标帧错误时断开连接；之前的轨迹仍可查看，恢复后另起一段。
- 记录从面板连接开始，包括地面移动；降落不会自动清空，便于查看完整起降。重连核心、切换核心或重启面板会清空历史。
- 历史只在面板内存中，不写入航点 YAML；未打开面板期间不记录，不能代替飞控日志。采样依据 ControlStatus 汇总时间，而非原始传感器时间。
- 切换核心名称时，RViz 的 MarkerArray Topic 也需对应修改。定位原点重置无法仅靠轨迹判断，重置后请清空历史。

## 航点文件

文件为 YAML，包含版本、map 坐标系、按核心命名空间隔离的车辆列表、航点及任务选项：

```yaml
schema_version: 1
frame_id: map
vehicles:
  - namespace: /flight_core
    options:
      position_tolerance_m: 0.1
      yaw_tolerance_deg: 10.0
      dwell_sec: 0.5
      waypoint_timeout_sec: 60.0
      check_yaw: true
      loop: false
    waypoints:
      - {x: 0.0, y: 0.0, z: 1.0, yaw_deg: 0.0}
```

加载时先完整解析并校验，失败不替换当前草稿；文件必须含当前核心名称，不能通过文件静默切换控制对象。
保存使用 QSaveFile 原子替换，并保留文档中其他车辆条目。文件上限 4 MiB、100 个车辆条目、每机 500 点。
不支持旧 ROS 1 面板文本文件直接导入。

## 验证与开发

构建与测试命令统一见 [开发与测试](../../docs/DEVELOPMENT.md)。
面板测试使用 Qt offscreen 和本地假服务，覆盖插件加载、编辑、异步请求、结果显示、状态过期、单步与偏航映射；文件测试覆盖多车辆 YAML 往返与非法输入。
离屏测试与截图已检查，真实 RViz2 + PX4 SITL 的完整人工操作验收仍需记录，历史验证见 [重构计划](../../../REFACTOR_PLAN.md)。
