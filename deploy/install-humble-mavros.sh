#!/usr/bin/env bash
# 前置条件：Ubuntu 22.04 + ROS 2 Humble，已安装 rosdepc。
# Ubuntu 22.04 + ROS 2 Humble，使用普通用户在未加载其他工作空间的终端运行：
# bash install-humble-mavros.sh
# 固定版本：MAVROS 2.16.0、ROS 打包版 MAVLink 2026.9.9-1。
set -eo pipefail

if (( EUID == 0 )); then
  echo "请用普通用户运行，脚本会在需要时调用 sudo。" >&2
  exit 1
fi
if (( $# != 0 )); then
  echo "本脚本不接受参数，直接执行 bash install-humble-mavros.sh。" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
workspace="$HOME/mavros_humble_ws"

if ! command -v rosdepc >/dev/null 2>&1; then
  echo "请先安装 rosdepc，并确保 rosdepc 命令可用。" >&2
  exit 1
fi

# 1. 安装构建工具
sudo apt-get update --error-on=any
sudo apt-get install -y git build-essential cmake python3-pip \
  python3-colcon-common-extensions geographiclib-tools

# 2. 下载固定发布标签，不使用持续变化的开发分支
mkdir -p "$workspace/src"
cd "$workspace/src"

if [[ ! -e mavros ]]; then
  git clone --depth 1 --branch 2.16.0 \
    https://github.com/mavlink/mavros.git mavros
fi
if [[ ! -e mavlink ]]; then
  git clone --depth 1 --branch release/rolling/mavlink/2026.9.9-1 \
    https://github.com/ros2-gbp/mavlink-gbp-release.git mavlink
fi

# 重复执行时只复用相同版本，不覆盖已有源码。
if [[ "$(git -C mavros rev-parse HEAD)" != \
      "$(git -C mavros rev-parse 'refs/tags/2.16.0^{commit}')" ]] ||
   [[ "$(git -C mavlink rev-parse HEAD)" != \
      "$(git -C mavlink rev-parse 'refs/tags/release/rolling/mavlink/2026.9.9-1^{commit}')" ]]; then
  echo "已有源码与固定版本不匹配，请先备份并移走冲突的源码目录后重试。" >&2
  exit 1
fi

# 3. 安装源码包依赖
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdepc init
fi
rosdepc update --rosdistro humble
rosdepc install --from-paths "$workspace/src" --ignore-src --rosdistro humble -y

# 4. 安装 GeographicLib 数据集
sudo bash "$workspace/src/mavros/mavros/scripts/install_geographiclib_datasets.sh"
if [[ ! -s /usr/share/GeographicLib/geoids/egm96-5.pgm &&
      ! -s /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  echo "egm96-5 数据集安装失败，请检查下载网络后重试。" >&2
  exit 1
fi

# 5. 编译：固定使用两个编译任务，降低内存占用
cd "$workspace"
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --symlink-install --executor sequential --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# 6. 验证消息接口和 MAVLink 配置路径
source "$workspace/install/setup.bash"
grep -F "mavlink_DIR:PATH=$workspace/install/mavlink/" build/mavros/CMakeCache.txt
ros2 interface show mavros_msgs/msg/State
ros2 pkg executables mavros

printf '\n安装完成。每个新终端执行：\nsource %s/install/setup.bash\n' "$workspace"
