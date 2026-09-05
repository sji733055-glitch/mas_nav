#!/usr/bin/env bash
# Record Small Point-LIO topics with QoS that matches the publishers.
# /cloud_registered is SensorDataQoS (best_effort, keep_last 1). A default
# rosbag2 subscription is reliable and will record zero clouds.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
QOS="${ROOT}/config/qos_mapping.yaml"
OUT="${1:-${PWD}/lio_bag}"

if [[ ! -f "${QOS}" ]]; then
  echo "QoS override file missing: ${QOS}" >&2
  exit 2
fi

echo "Recording to ${OUT}"
echo "Start LIO first, in another terminal:"
echo "  ros2 launch mas2027_nav_bringup rm_navigation_small_point_lio_launch.py use_nav2:=False use_rviz:=True"
echo "Ctrl-C stops the bag."

exec ros2 bag record -s mcap -o "${OUT}" \
  --qos-profile-overrides-path "${QOS}" \
  /cloud_registered /Odometry /tf /tf_static
