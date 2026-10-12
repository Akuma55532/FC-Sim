# 01 · 安装仿真环境

[文档导航](README.md) · 下一章：[编译项目](BUILD.md)

本章准备 PX4、Gazebo、ROS 2 和 MAVROS。教程面向同一台 Ubuntu 22.04 机器上的单机 SITL。环境检查及验证范围见[验证记录](VALIDATION.md)。

## 目录和版本

以下目录贯穿教程。仓库不在 `~/FC-Sim` 时，将每处 `FC_SIM_ROOT` 改为实际位置；已有 PX4 源码也可直接指定其路径。

```bash
export FC_SIM_ROOT="$HOME/FC-Sim"
export PX4_ROOT="$HOME/ws_simulator/PX4-Autopilot"
```

| 组件 | 教程基线 | 位置 |
|---|---|---|
| ROS 2 | Humble，Ubuntu 22.04 | `/opt/ros/humble` |
| MAVROS | 2.16.0 | `~/mavros_humble_ws` |
| MAVLink 消息包 | ROS 发布标签 `release/rolling/mavlink/2026.9.9-1` | MAVROS 工作空间内 |
| PX4 | `v1.17.0-beta1` | `$PX4_ROOT` |
| Gazebo | Harmonic（gz-sim 8） | 系统安装 |
| 本项目 | 当前源码 | `$FC_SIM_ROOT` |

MAVLink 的发布标签包含 `rolling`，不代表需要安装 ROS Rolling。仓库中的 `pkgs/mavros_humble_ws/` 空目录不是 MAVROS 安装位置。

## 1. 准备 ROS 2 与安装工具

若尚未安装 ROS 2，按 [ROS 2 Humble Ubuntu deb 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)配置 locale、软件源并安装 `ros-humble-desktop`。本教程需要 RViz2，Desktop 安装适合作为起点。

打开 Bash 终端验证：

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
ros2 --help
```

`ROS_DISTRO` 应为 `humble`。仓库安装脚本使用 `rosdepc`，若缺失可安装：

```bash
sudo apt-get update
sudo apt-get install -y git python3-pip
python3 -m pip install --user rosdepc
export PATH="$HOME/.local/bin:$PATH"
command -v rosdepc
```

后续新终端也需能找到 `rosdepc`。若使用代理或镜像，先保证 apt、GitHub 和依赖索引可访问；下载失败应解决网络问题后重试。

## 2. 安装 MAVROS

在未加载其他 ROS 工作空间的新终端中执行，使用普通用户，脚本会按需调用 sudo：

```bash
export FC_SIM_ROOT="$HOME/FC-Sim"
bash "$FC_SIM_ROOT/deploy/install-humble-mavros.sh"
```

[脚本](../deploy/install-humble-mavros.sh)会安装构建工具、拉取固定标签、安装 ROS 依赖及 GeographicLib 数据集，并以两个编译任务构建到 `~/mavros_humble_ws/install`。它不接受路径参数。

如果目录已有不同版本的源码，脚本会退出。先核对现有版本并备份，再决定是否另行安置冲突目录；不要直接覆盖已有工作空间。

安装后检查：

```bash
source /opt/ros/humble/setup.bash
source "$HOME/mavros_humble_ws/install/setup.bash"
ros2 pkg prefix mavros
ros2 pkg executables mavros
ros2 interface show mavros_msgs/msg/State
ls /usr/share/GeographicLib/geoids/egm96-5.pgm \
   /usr/local/share/GeographicLib/geoids/egm96-5.pgm 2>/dev/null
```

应能找到 MAVROS 包、`mavros_node` 可执行文件、State 消息及至少一个数据集路径。缺少数据集时，重跑源码中的数据集安装脚本并检查下载错误。

## 3. 安装 PX4 与 Gazebo

以下克隆命令用于尚无 PX4 源码的环境：

```bash
mkdir -p "$HOME/ws_simulator"
git clone --branch v1.17.0-beta1 --recursive \
  https://github.com/PX4/PX4-Autopilot.git \
  "$HOME/ws_simulator/PX4-Autopilot"
export PX4_ROOT="$HOME/ws_simulator/PX4-Autopilot"
cd "$PX4_ROOT"
bash ./Tools/setup/ubuntu.sh --no-nuttx
```

该版本的 Ubuntu 22.04 安装分支选择 `gz-harmonic`。按安装脚本提示完成可能需要的重新登录或重启，再打开新终端。已有源码先用 `git describe --tags --always` 核对版本，不需要为教程删除 `build/`。

编译并启动一次仿真：

```bash
export PX4_ROOT="$HOME/ws_simulator/PX4-Autopilot"
cd "$PX4_ROOT"
make px4_sitl gz_x500
```

首次执行可能还需要下载模型资源。成功时应看到 PX4 控制台及 Gazebo 中的 X500；在 PX4 的 `pxh>` 提示符输入：

```text
ver all
mavlink status
```

记录版本和 UDP 端口。此处只检查仿真启动，不进行解锁。检查完用 Ctrl+C 关闭本次仿真，后续快速开始会重新启动。

没有图形会话的机器可用 `HEADLESS=1 make px4_sitl gz_x500` 运行物理仿真；这不验证 Gazebo 或 RViz 的图形显示。

## 验收与常见问题

- ROS 命令可用，MAVROS 包和消息可查询，GeographicLib 数据集存在。
- PX4 固定版本可启动 `gz_x500`，Gazebo 模型资源加载完成。
- `rosdepc: command not found`：检查安装位置和当前终端 PATH。
- 找不到 `gz_x500` 或 Gazebo 依赖：检查 PX4 标签、子模块和安装脚本输出；不要混用 Gazebo Classic 教程。
- Gazebo 没有窗口：检查图形会话与显卡支持；无界面启动仅用于区分渲染问题和仿真启动问题。

完成后进入[编译项目](BUILD.md)。
