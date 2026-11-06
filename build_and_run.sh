#!/usr/bin/env bash
# 一键增量编译并启动导航；可从任意目录执行。
set -eo pipefail

NAV_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
NAV_BUILD_JOBS="${NAV_BUILD_JOBS:-2}"
build_only=false
launch_args=()

usage() {
  cat <<'HELP'
用法：
  ./build_and_run.sh [--build-only] [--jobs N] [ROS 启动参数 ...]

示例：
  ./build_and_run.sh
  ./build_and_run.sh use_nav_serial:=False
  ./build_and_run.sh map_source:=prior use_rviz:=False
  ./build_and_run.sh --build-only --jobs 4

默认增量编译导航及其依赖，成功后启动导航和 RViz。
默认启用底盘串口，地图模式取自 navigation_map.yaml。
使用本仓库的源码配置，可传 navigation_config:=... 或 config_dir:=... 覆盖。
环境变量：ROS_SETUP（ROS 环境脚本）、NAV_BUILD_JOBS（编译线程数，默认 2）。
HELP
}

while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --build-only) build_only=true; shift ;;
    --jobs)
      if (($# < 2)); then
        echo "错误：--jobs 后需要正整数。" >&2
        exit 2
      fi
      NAV_BUILD_JOBS="$2"
      shift 2
      ;;
    *:=*) launch_args+=("$1"); shift ;;
    *) echo "错误：未知参数 $1；使用 --help 查看用法。" >&2; exit 2 ;;
  esac
done

if [[ ! "$NAV_BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "错误：编译线程数必须是正整数。" >&2
  exit 2
fi
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "错误：找不到 ROS 环境脚本 $ROS_SETUP。可通过 ROS_SETUP 指定。" >&2
  exit 1
fi

cd -- "$NAV_WORKSPACE"
source "$ROS_SETUP"
command -v colcon >/dev/null || { echo "错误：未找到 colcon，请先安装 colcon。" >&2; exit 1; }
command -v ros2 >/dev/null || { echo "错误：未找到 ros2，请检查 ROS 环境。" >&2; exit 1; }

# 包间顺序构建，包内限制线程，避免多个大型 C++ 包同时占满内存。
export CMAKE_BUILD_PARALLEL_LEVEL="$NAV_BUILD_JOBS"
export MAKEFLAGS="-j$NAV_BUILD_JOBS"
echo "[1/3] 编译导航及依赖（每包 $NAV_BUILD_JOBS 线程）"
colcon build --executor sequential --packages-up-to mas2027_nav_bringup \
  --cmake-args -DBUILD_TESTING=ON

echo "[2/3] 加载本次编译的工作区环境"
source "$NAV_WORKSPACE/install/setup.bash"
if "$build_only"; then
  echo "编译完成。再次执行本脚本即可增量编译并启动。"
  exit 0
fi

# 仅补充未指定的默认参数，调用者的显式配置优先。
for default_arg in \
  "navigation_config:=$NAV_WORKSPACE/mas2027_nav_bringup/config/navigation_map.yaml" \
  "config_dir:=$NAV_WORKSPACE/mas2027_nav_executor/config" \
  "use_rviz:=True"; do
  supplied=false
  for arg in "${launch_args[@]}"; do
    if [[ "${arg%%:=*}" == "${default_arg%%:=*}" ]]; then
      supplied=true
      break
    fi
  done
  if ! "$supplied"; then launch_args+=("$default_arg"); fi
done

echo "[3/3] 启动导航（Ctrl+C 退出）"
exec ros2 launch mas2027_nav_bringup nav_executor_launch.py "${launch_args[@]}"
