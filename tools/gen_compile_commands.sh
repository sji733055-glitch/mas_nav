#!/usr/bin/env bash
# 为 clangd / IDE 生成完整的 compile_commands.json。
#
# 背景：本工作区的构建目录由 colcon 逐个包生成，默认不导出编译命令；而 clangd 只认
# 「离源文件最近的 compile_commands.json」。缺条目时 clangd 会拿别的文件的命令来猜，
# 于是满屏 'xxx.hpp file not found' / 'use of undeclared identifier'，跨文件跳转也失效。
#
# 本脚本做两件事：
#   1. 对 build/ 下每个已配置的包重新 configure 一次，打开 CMAKE_EXPORT_COMPILE_COMMANDS
#      （只重新生成构建文件，不重新编译源码）；
#   2. 把所有包的编译命令合并成一份 build/compile_commands.json。
#
# 新增/删除源文件、切分支、改 CMakeLists 之后重跑一次即可；只改注释/普通源码不必重跑。
#
# 用法： bash src/tools/gen_compile_commands.sh
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD="$WS/build"
OUT="$BUILD/compile_commands.json"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u  # ROS 的 setup.bash 会引用未定义变量，与 set -u 冲突
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

echo "[1/2] 重新 configure 各包（打开 CMAKE_EXPORT_COMPILE_COMMANDS）"
for cache in "$BUILD"/*/CMakeCache.txt; do
  [ -e "$cache" ] || continue
  bdir="$(dirname "$cache")"
  sdir="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$cache" | head -1)"
  if [ -z "$sdir" ] || [ ! -f "$sdir/CMakeLists.txt" ]; then
    echo "  跳过 $(basename "$bdir")：缓存里的源码目录无效（$sdir）"
    continue
  fi
  echo "  $(basename "$bdir")"
  cmake -S "$sdir" -B "$bdir" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON >/dev/null
done

echo "[2/2] 合并 → $OUT"
python3 - "$BUILD" "$OUT" <<'PY'
import json, os, sys

build, out = sys.argv[1], sys.argv[2]
merged, seen, sources = [], set(), []
for root, dirs, files in os.walk(build):
    if "compile_commands.json" not in files:
        continue
    path = os.path.join(root, "compile_commands.json")
    if os.path.abspath(path) == os.path.abspath(out):
        continue
    sources.append(path)
    with open(path) as f:
        for entry in json.load(f):
            key = os.path.realpath(entry.get("file", ""))
            if not key or key in seen:
                continue
            seen.add(key)
            merged.append(entry)

with open(out, "w") as f:
    json.dump(merged, f, indent=1)

print(f"  来源数据库 {len(sources)} 份，条目 {len(merged)} 条")
PY
