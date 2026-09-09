#!/usr/bin/env bash
# 把本机环境镜像打成 tar.gz，拷到新机器 load 后 compose up 即可，不用再装 apt/pip。
set -euo pipefail

IMAGE="${IMAGE:-mas_nav_image:latest}"
OUT="${1:-${PWD}/mas_nav_image.tar.gz}"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "本地没有 ${IMAGE}。先：docker compose build" >&2
  exit 1
fi

echo "saving ${IMAGE} -> ${OUT}"
if command -v pigz >/dev/null 2>&1; then
  docker save "${IMAGE}" | pigz -1 > "${OUT}"
else
  docker save "${IMAGE}" | gzip -1 > "${OUT}"
fi
ls -lh "${OUT}"
echo "拷到新机器后：bash scripts/load_nav_image.sh ${OUT}"
