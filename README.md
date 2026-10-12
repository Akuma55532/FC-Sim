# FC-Sim

基于 ROS 2 Humble、MAVROS 和 PX4 SITL 的单机飞行控制项目，提供起飞、降落、航点任务与 RViz2 操作面板。

## 开始使用

1. [安装仿真环境](docs/INSTALLATION.md)
2. [编译项目](docs/BUILD.md)
3. [首次起飞与降落](docs/QUICKSTART.md)
4. [RViz 飞行面板](docs/RVIZ_PANEL.md)
5. [航点任务](docs/WAYPOINTS.md)
6. [编程接入](docs/ROS_API.md)与[状态参考](docs/CONTROL_STATUS.md)

教程基线为 Ubuntu 22.04、MAVROS 2.16.0、PX4 v1.17.0-beta1 与 Gazebo Harmonic。已完成的检查及限制见[验证记录](docs/VALIDATION.md)。

## 项目资料

- [文档导航](docs/README.md)与[教程编写计划](docs/TUTORIAL_PLAN.md)
- [飞行核心代码导览](pkgs/flight_pkgs/flight_core/README.md)
- [RViz 面板说明](pkgs/flight_pkgs/flight_rviz_plugins/README.md)
- [MAVROS 中文接口参考](mavoros_docu_translate/README.md)

源码位于 `pkgs/flight_pkgs/`：`flight_interfaces` 定义接口，`flight_core` 执行动作与任务，`flight_rviz_plugins` 提供图形操作入口。
