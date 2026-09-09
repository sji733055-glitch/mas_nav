#!/usr/bin/env bash
# 新机器导入环境镜像。之后 docker compose up -d，不要 --build。
set -euo pipefail

IN="${1:?usage: $0 /path/to/mas_nav_image.tar.gz}"
IMAGE="${IMAGE:-mas_nav_image:latest}"

if [[ ! -f "${IN}" ]]; then
  echo "file not found: ${IN}" >&2
  exit 1
fi

echo "loading ${IN}"
if [[ "${IN}" == *.gz ]]; then
  gzip -dc "${IN}" | docker load
else
  docker load -i "${IN}"
fi

docker image inspect "${IMAGE}" >/dev/null
echo "ok: ${IMAGE}"
echo "然后：xhost +si:localuser:root && docker compose up -d"
