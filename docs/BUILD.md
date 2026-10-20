# 02 · 编译项目

[上一章：安装](INSTALLATION.md) · [文档导航](README.md) · [下一章：首次飞行](QUICKSTART.md)

本章将仓库中的三个 ROS 包构建到仓库根目录下的 `install/`，不需要另建 `flight_ws/src` 或移动源码。

## 1. 加载基础环境并检查包

打开新 Bash 终端，先按[目录约定](INSTALLATION.md#目录和版本)设置自己的路径；以下命令仅在变量未设置时使用默认位置。然后加载 ROS 和 MAVROS：

```bash
export FC_SIM_ROOT="${FC_SIM_ROOT:-$HOME/FC-Sim}"
source /opt/ros/humble/setup.bash
export MAVROS_WS="${MAVROS_WS:-$HOME/mavros_humble_ws}"
source "$MAVROS_WS/install/setup.bash"
cd "$FC_SIM_ROOT"
colcon list --base-paths pkgs/flight_pkgs
```

应列出 `flight_interfaces`、`flight_core` 和 `flight_rviz_plugins` 三个包。若找不到 colcon，检查上一章的安装脚本是否成功完成。

## 2. 安装项目依赖

上一章已通过 MAVROS 安装脚本初始化 rosdepc。保持基础环境已加载，然后运行：

```bash
rosdepc update --rosdistro humble
rosdepc install --from-paths pkgs/flight_pkgs --ignore-src \
  --rosdistro humble -y
```

依赖来自各包的 `package.xml`，包括 RViz、Qt、yaml-cpp、ROS Action 与消息库。若报告 MAVROS 相关依赖不可用，先确认 `ros2 pkg prefix mavros` 和 `ros2 pkg prefix mavros_msgs` 指向已加载的源码安装。

## 3. 构建

```bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths pkgs/flight_pkgs \
  --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

这会生成 `build/`、`install/` 和 `log/`。成功摘要应包含三个包完成且无失败。这里关闭测试目标以构建使用教程所需的程序；测试覆盖不等于实际飞行验收。

构建失败时先看第一个失败包的错误和 `log/latest_build/`。内存不足可将并行数改为 1。后续修改源码后可以重复构建，无需先删除构建目录。

## 4. 验证安装产物

```bash
source "$FC_SIM_ROOT/install/setup.bash"
ros2 pkg prefix flight_interfaces
ros2 pkg prefix flight_core
ros2 pkg prefix flight_rviz_plugins
ros2 pkg executables flight_core
ros2 interface show flight_interfaces/action/Takeoff
ros2 launch flight_core mavros_gcs.launch.py --show-args
ros2 launch flight_rviz_plugins panel.launch.py --show-args
```

包路径应位于当前仓库的 `install/` 下；核心可执行文件应为 `flight_actions_node`；Takeoff 目标包含 `float64 height_m`。MAVROS 启动参数应列出 `fcu_url`，默认值为 `udp://:14540@127.0.0.1:14580`。

`--show-args` 解析启动文件但不启动飞控连接或 RViz，不能代替实际插件加载检查。

## 5. 后续每个 ROS 终端的公共前缀

快速开始中的 B、C、D、E 终端均先按[目录约定](INSTALLATION.md#目录和版本)设置实际路径，再执行以下内容：

```bash
export FC_SIM_ROOT="${FC_SIM_ROOT:-$HOME/FC-Sim}"
source /opt/ros/humble/setup.bash
export MAVROS_WS="${MAVROS_WS:-$HOME/mavros_humble_ws}"
source "$MAVROS_WS/install/setup.bash"
source "$FC_SIM_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

教程使用 ROS 域 42 和本机通信，所有 ROS 终端必须一致。若该域已用于其他实验，应统一选择一个空闲域；ROS 域只隔离 ROS 通信，并不隔离 PX4 的 UDP 端口。这里按本机单机配置，不适用于跨机器部署。

`Package not found` 通常来自漏加载环境或加载了旧工作空间。检查 `ros2 pkg prefix` 的路径，并在干净终端按上述顺序重新加载。

完成后进入[首次起飞与降落](QUICKSTART.md)。
