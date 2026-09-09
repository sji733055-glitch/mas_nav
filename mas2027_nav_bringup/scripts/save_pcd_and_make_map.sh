#!/usr/bin/env bash
# Save Small Point-LIO's accumulated cloud (/map_save) into bringup/pcd,
# transform it into /cloud_registered (odom) using T dumped by /map_save,
# then slice it to a Nav2 occupancy PGM/YAML with pcd2pgm.
#
# /map_save writes scan.pcd (LIO-internal world) on a detached thread and
# scan_T_odom_from_internal.txt immediately. This script waits for the PCD,
# then applies that 4x4 so bringup/pcd/<NAME>.pcd matches live GICP source.
# pcd2pgm is started on /pcd2pgm_map so it does not collide with Nav2 /map.
# Cleanup uses the child PID only — never pkill (this container shares the host PID namespace).
set -euo pipefail

usage() {
  cat <<'EOF'
保存 LIO 点云并切成 Nav2 二维占用图。在 mas_nav 容器里跑，工作目录不限。

用法:
  bash save_pcd_and_make_map.sh [NAME] [选项]

  NAME 默认 lab3，写出:
    mas2027_nav_bringup/pcd/<NAME>.pcd
    mas2027_nav_bringup/map/<NAME>.{pgm,yaml}

选项:
  --from-pcd          不调 /map_save，直接切已有的 bringup/pcd/<NAME>.pcd（已是 odom 系则不再乘 T）
  --no-align          调 /map_save 之后只拷贝，不乘 T（留 LIO 内部世界）
  --z-min FLOAT       Z 带下沿（默认 0.05）
  --z-max FLOAT       Z 带上沿（默认 1.5）
  --resolution FLOAT  栅格分辨率米（默认 0.05）
  --scan PATH         /map_save 写出的 scan.pcd（默认 LIO 源码 pcd/scan.pcd）
  -h, --help

调 /map_save 前，LIO 必须已经在跑且 save_pcd:=true：
  ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py \
    use_nav2:=False use_rviz:=True use_odom_localizer:=False

默认会把 scan.pcd 乘上 /map_save 写出的 T_odom_from_internal，使先验 PCD 与 /cloud_registered 同系。
这一步不做 map_edit、也不改 odom_localizer / ROG prior_map 路径。场地原点还要手改时仍用 pcd_trans --tx/--yaw。
EOF
}

NAME="lab3"
FROM_PCD=0
ALIGN=1
Z_MIN="0.05"
Z_MAX="1.5"
RESOLUTION="0.05"
SCAN_OVERRIDE=""
MAP_TOPIC="pcd2pgm_map"
SAVE_TIMEOUT_SEC=180
MAP_TIMEOUT_SEC=90
PCD2PGM_PID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --from-pcd)
      FROM_PCD=1
      shift
      ;;
    --no-align)
      ALIGN=0
      shift
      ;;
    --z-min)
      Z_MIN="$2"
      shift 2
      ;;
    --z-max)
      Z_MAX="$2"
      shift 2
      ;;
    --resolution)
      RESOLUTION="$2"
      shift 2
      ;;
    --scan)
      SCAN_OVERRIDE="$2"
      shift 2
      ;;
    --*)
      echo "未知选项: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      NAME="$1"
      shift
      ;;
  esac
done

if [[ ! "$NAME" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "NAME 只能含字母、数字、. _ - ，得到: $NAME" >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

resolve_bringup() {
  if [[ -d "${script_dir}/../pcd" && -d "${script_dir}/../map" ]]; then
    cd "${script_dir}/.." && pwd
    return
  fi
  if [[ -d /home/ros2_ws/src/mas2027_nav_bringup ]]; then
    echo /home/ros2_ws/src/mas2027_nav_bringup
    return
  fi
  if [[ -d /home/mas/mas_nav_2027/mas2027_nav_bringup ]]; then
    echo /home/mas/mas_nav_2027/mas2027_nav_bringup
    return
  fi
  echo "找不到 mas2027_nav_bringup（pcd/ 与 map/）" >&2
  exit 2
}

BRINGUP="$(resolve_bringup)"
REPO="$(cd "${BRINGUP}/.." && pwd)"
PCD_DIR="${BRINGUP}/pcd"
MAP_DIR="${BRINGUP}/map"
DEST_PCD="${PCD_DIR}/${NAME}.pcd"
DEST_STEM="${MAP_DIR}/${NAME}"
SCAN_PCD="${SCAN_OVERRIDE:-${REPO}/mas2027_perception/Odometry/small_point_lio/pcd/scan.pcd}"
T_FILE="$(dirname "${SCAN_PCD}")/scan_T_odom_from_internal.txt"

resolve_pcd_tool() {
  if [[ -f "${REPO}/mas2027_utils/pcd_trans/pcd_tool.py" ]]; then
    echo "${REPO}/mas2027_utils/pcd_trans/pcd_tool.py"
    return
  fi
  if [[ -f /home/ros2_ws/src/mas2027_utils/pcd_trans/pcd_tool.py ]]; then
    echo /home/ros2_ws/src/mas2027_utils/pcd_trans/pcd_tool.py
    return
  fi
  echo "找不到 pcd_trans/pcd_tool.py" >&2
  exit 2
}

PCD_TOOL="$(resolve_pcd_tool)"

backup_if_exists() {
  local path="$1"
  if [[ -e "$path" ]]; then
    local bak="${path}.bak.$(date +%Y%m%d-%H%M%S)"
    cp -a "$path" "$bak"
    echo "已有文件备份: $path -> $bak"
  fi
}

source_ros() {
  if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix pcd2pgm >/dev/null 2>&1 && ros2 pkg prefix nav2_map_server >/dev/null 2>&1; then
    return
  fi
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  if [[ -f /home/ros2_ws/install/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /home/ros2_ws/install/setup.bash
  elif [[ -f "${REPO}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO}/install/setup.bash"
  fi
  set -u
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 不在 PATH 里。先 source /opt/ros/humble 和 overlay。" >&2
    exit 2
  fi
}

wait_file_stable() {
  local path="$1"
  local timeout_sec="$2"
  local before_mtime="$3"
  local before_size="$4"
  local start now size last same mtime
  start="$(date +%s)"
  last=""
  same=0
  while true; do
    now="$(date +%s)"
    if (( now - start > timeout_sec )); then
      echo "等 ${path} 超过 ${timeout_sec}s。/map_save 在后台写盘，点云很大会慢。" >&2
      echo "save_pcd 不是 true、或 scan.pcd 路径不对（当前 ${path}）。" >&2
      exit 1
    fi
    if [[ -f "$path" ]]; then
      size="$(stat -c %s "$path" 2>/dev/null || echo 0)"
      mtime="$(stat -c %Y "$path" 2>/dev/null || echo 0)"
      if [[ "$size" -gt 1024 ]] && { [[ "$mtime" -gt "$before_mtime" ]] || [[ "$size" -ne "$before_size" ]]; }; then
        if [[ "$size" == "$last" ]]; then
          same=$((same + 1))
          if (( same >= 2 )); then
            echo "scan.pcd 写完: ${path} (${size} bytes)"
            return
          fi
        else
          same=0
          last="$size"
        fi
      fi
    fi
    sleep 1
  done
}

cleanup() {
  if [[ -n "${PCD2PGM_PID}" ]] && kill -0 "${PCD2PGM_PID}" 2>/dev/null; then
    kill "${PCD2PGM_PID}" 2>/dev/null || true
    wait "${PCD2PGM_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

mkdir -p "${PCD_DIR}" "${MAP_DIR}"
source_ros

if (( FROM_PCD == 0 )); then
  echo "等 /map_save 服务（LIO 要开着且 save_pcd:=true）..."
  if ! ros2 service type /map_save >/dev/null 2>&1; then
    echo "没有 /map_save。先起 LIO，不要 Nav2：" >&2
    echo "  ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py \\" >&2
    echo "    use_nav2:=False use_rviz:=True use_odom_localizer:=False" >&2
    echo "并把 mas2027_nav_bringup/config/small_point_lio_params.yaml 里 save_pcd 设成 true。" >&2
    exit 1
  fi

  before_mtime="$(stat -c %Y "${SCAN_PCD}" 2>/dev/null || echo 0)"
  before_size="$(stat -c %s "${SCAN_PCD}" 2>/dev/null || echo 0)"
  echo "调用 /map_save（写盘是异步的，服务会马上返回）..."
  call_out="$(ros2 service call /map_save std_srvs/srv/Trigger 2>&1)" || {
    echo "/map_save 调用失败:" >&2
    echo "${call_out}" >&2
    exit 1
  }
  echo "${call_out}"
  if echo "${call_out}" | grep -Eqi 'success[[:space:]:]*=?[[:space:]]*(False|false)'; then
    echo "/map_save 拒绝了。通常是 save_pcd 仍是 false：改 yaml、重启 LIO 后再跑本脚本。" >&2
    exit 1
  fi
  wait_file_stable "${SCAN_PCD}" "${SAVE_TIMEOUT_SEC}" "${before_mtime}" "${before_size}"
  backup_if_exists "${DEST_PCD}"
  if (( ALIGN == 1 )); then
    if [[ ! -f "${T_FILE}" ]]; then
      echo "没有 ${T_FILE}。" >&2
      echo "需要重新编译 small_point_lio：/map_save 会写出 T_odom_from_internal。" >&2
      echo "或加 --no-align 只拷贝内部世界系（GICP 多半锁不上）。" >&2
      exit 1
    fi
    echo "用 ${T_FILE} 把 scan.pcd 转到 /cloud_registered（odom）..."
    python3 "${PCD_TOOL}" --in "${SCAN_PCD}" --out "${DEST_PCD}" --matrix "${T_FILE}"
    cp -a "${T_FILE}" "${PCD_DIR}/${NAME}_T_odom_from_internal.txt"
    echo "已写出 ${DEST_PCD}（odom 系）和 ${PCD_DIR}/${NAME}_T_odom_from_internal.txt"
  else
    cp -a "${SCAN_PCD}" "${DEST_PCD}"
    echo "--no-align：已拷内部世界系到 ${DEST_PCD}"
  fi
else
  if [[ ! -f "${DEST_PCD}" ]]; then
    echo "--from-pcd 但文件不存在: ${DEST_PCD}" >&2
    exit 1
  fi
  echo "跳过 /map_save，用现有 ${DEST_PCD}（不再乘 T）"
fi

pcd_bytes="$(stat -c %s "${DEST_PCD}")"
if [[ "${pcd_bytes}" -lt 10240 ]]; then
  echo "${DEST_PCD} 只有 ${pcd_bytes} bytes，像一份空云。确认绕场跑过且 save_pcd 有缓存到点。" >&2
  exit 1
fi

backup_if_exists "${DEST_STEM}.pgm"
backup_if_exists "${DEST_STEM}.yaml"

echo "启动 pcd2pgm（话题 /${MAP_TOPIC}，不抢 Nav2 的 /map）..."
ros2 run pcd2pgm pcd2pgm_node --ros-args \
  -p use_sim_time:=false \
  -p "file_directory:=${PCD_DIR}/" \
  -p "file_name:=${NAME}" \
  -p "map_topic_name:=${MAP_TOPIC}" \
  -p "map_resolution:=${RESOLUTION}" \
  -p "thre_z_min:=${Z_MIN}" \
  -p "thre_z_max:=${Z_MAX}" \
  -p flag_pass_through:=false \
  -p thre_radius:=0.5 \
  -p thres_point_count:=10 &
PCD2PGM_PID=$!

map_start="$(date +%s)"
while true; do
  if ! kill -0 "${PCD2PGM_PID}" 2>/dev/null; then
    wait "${PCD2PGM_PID}" || true
    echo "pcd2pgm 退出了。多半是读不了 ${DEST_PCD}，或切片之后没剩点（试 --z-min / --z-max）。" >&2
    PCD2PGM_PID=""
    exit 1
  fi
  if ros2 topic list 2>/dev/null | grep -qx "/${MAP_TOPIC}"; then
    break
  fi
  if (( $(date +%s) - map_start > MAP_TIMEOUT_SEC )); then
    echo "等 /${MAP_TOPIC} 超过 ${MAP_TIMEOUT_SEC}s。" >&2
    exit 1
  fi
  sleep 0.5
done

# First OccupancyGrid can lag the topic advertisement while the cloud is sliced.
sleep 1
echo "map_saver_cli <- /${MAP_TOPIC} -> ${DEST_STEM}.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -t "/${MAP_TOPIC}" -f "${DEST_STEM}"

echo
echo "完成:"
echo "  3D  ${DEST_PCD}  (${pcd_bytes} bytes)"
echo "  2D  ${DEST_STEM}.pgm"
echo "  2D  ${DEST_STEM}.yaml"
echo
echo "下一步（人工）:"
echo "  1. 修墙: ros2 launch map_edit map_edit.launch.py"
echo "  2. 场地原点要对齐时，才用 pcd_trans --tx/--yaw（扶正已经做完）"
echo "  3. 换图时三处同名:"
echo "     odom_localizer  map.prior_pcd_file  -> ${DEST_PCD}"
echo "     nav2_params     projection.prior_map -> ${DEST_STEM}.yaml"
echo "     launch           map:=               ${DEST_STEM}.yaml"
echo "  4. 建图用完把 save_pcd 改回 false（很占内存）"
