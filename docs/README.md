# FC-Sim 使用文档

## 入门教程

1. [安装仿真环境](INSTALLATION.md)：版本、目录、ROS 2、MAVROS 与 PX4/Gazebo。
2. [编译项目](BUILD.md)：依赖、colcon 构建与每个终端的环境加载。
3. [首次起飞与降落](QUICKSTART.md)：启动、遥测检查、1 m 起飞、保持和降落。
4. [RViz 飞行面板](RVIZ_PANEL.md)：连接、状态、起飞降落与目标显示。
5. [航点任务](WAYPOINTS.md)：示例航线、单步、循环停止和 YAML 保存加载。

6. [编程接入](ROS_API.md)：Python 顺序任务示例、Action 反馈结果和异常收尾。
7. [状态与结果参考](CONTROL_STATUS.md)：字段、时效、阶段、进度和结果码。

实际检查结果与未覆盖范围见[验证记录](VALIDATION.md)。

## 配套参考

- [MAVROS 中文接口参考](../mavoros_docu_translate/README.md)：遥测、Offboard、服务、坐标系和 QoS。
- [飞行核心代码导览](../pkgs/flight_pkgs/flight_core/README.md)：代码职责与后端边界。
- [RViz 面板使用说明](../pkgs/flight_pkgs/flight_rviz_plugins/README.md)：面板操作与航点文件格式。
- [教程编写计划](TUTORIAL_PLAN.md)：完整章节安排与后续验收标准。

后续将补充排障和开发测试章节。包内部分旧文档链接仍待对应章节落地后整理。
