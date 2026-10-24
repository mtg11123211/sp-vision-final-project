#!/usr/bin/env bash
# install.sh —— 车机上的"安装 + 校验 + 编译 + 验收"脚本（由 make_pack.ps1 打进包里）
#
# 用法（车机，工程根下）：
#   cd ~/fp_aim && tar -xzf ~/fp_aim_patch_*.tar.gz && bash tools/onsite/pack/install.sh
#
# 它做四件事：
#   1. 用包里带的 manifest.sha256 校验每个文件（解压不完整会当场报出来）
#   2. 确认清单里的文件都在（少一个就编不过）
#   3. 编译 task_2 与 keytest
#   4. 跑验收 grep（计数按 2026-10-21 副本版本）
#
# 注意：**不碰任何 git、不删任何文件**，只覆盖清单里那几个。

set -uo pipefail

PROJ="${FP_AIM_HOME:-$HOME/fp_aim}"
cd "$PROJ" || { echo "找不到工程目录：$PROJ" >&2; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
MAN="$HERE/manifest.sha256"

echo "=============================================================="
echo " fp_aim 副本补丁安装  工程目录：$PROJ"
echo "=============================================================="

echo "== 1/4 校验文件完整性（sha256）=="
if [ -f "$MAN" ]; then
  if sha256sum -c "$MAN" 2>&1 | sed 's/^/   /'; then :; fi
  if sha256sum -c "$MAN" >/dev/null 2>&1; then
    echo "   ⇒ 全部校验通过"
  else
    echo "   !! 有文件校验失败 ⇒ 包没传全/传坏了，重新传一次" >&2
    exit 1
  fi
else
  echo "   （包里没有 manifest.sha256，跳过校验）"
fi

echo "== 2/4 确认清单里的文件都在 =="
miss=0
if [ -f "$MAN" ]; then
  while read -r _sha _path; do
    [ -n "${_path:-}" ] || continue
    case "$_path" in \#*) continue ;; esac
    if [ ! -f "$_path" ]; then echo "   缺 $_path"; miss=1; fi
  done < "$MAN"
else
  for f in src/task_2.cpp src/task_3.cpp src/task_common.hpp src/fire_key.hpp src/low_plate.hpp \
           src/env_util.hpp src/test_summary.hpp tools/keytest.cpp; do
    [ -f "$f" ] || { echo "   缺 $f"; miss=1; }
  done
fi
if [ "$miss" -eq 0 ]; then echo "   全部就位"; else echo "   !! 缺文件 ⇒ 解压没覆盖全" >&2; exit 1; fi

echo "== 3/4 编译（改了 CMakeLists 要重新 cmake）=="
if [ ! -f build/CMakeCache.txt ]; then
  ( cd build && cmake -B . .. >/dev/null ) || echo "   ⚠ cmake 失败，看下面的报错"
elif [ ! -x build/keytest ]; then
  echo "   build/keytest 不存在 ⇒ 重新 cmake"
  ( cd build && cmake -B . .. >/dev/null ) || echo "   ⚠ cmake 失败，看下面的报错"
fi
for f in src/task_2.cpp src/task_3.cpp tasks/auto_aim/armor.cpp tasks/auto_aim/target.cpp \
         io/camera.cpp io/hikrobot/hikrobot.cpp io/gimbal/gimbal.cpp; do
  [ -f "$f" ] && touch "$f"
done
( cd build && make -j2 task_2 keytest ) || { echo "!! 编译失败 ⇒ 把上面的报错贴出来" >&2; exit 1; }
if [ ! "build/task_2" -nt src/task_2.cpp ]; then
  echo "!! 二进制不比源文件新 ⇒ make 跳过了编译" >&2; exit 1
fi
echo "   OK  build/task_2 与 build/keytest 就绪"

echo "== 4/4 验收 grep（计数按 2026-10-21 副本版本）=="
ck() { n=$(grep -c "$1" src/task_2.cpp || true); echo "   $1 → $n   ($2)"; }
ck plate_facing_deg "期望 >=4"
ck target_static    "期望 >=2"
ck DSH_LEAD         "期望 >=7"
ck fire_key         "期望 >=20"
ck low_plate        "期望 >=9"
ck test_summary     "期望 >=4"
ck 选板变化          "期望 >=1（副本才有这行日志）"
grep -nE '^(yaw_offset|pitch_offset):' configs/standard.yaml || true
grep -n  'fire_interval_s'  configs/standard.yaml || true

echo "=============================================================="
echo " 下一步："
echo "   1) 按键通道：./build/keytest      （backend 应是 evdev/termios）"
echo "   2) 一条龙：   ~/run_measure.sh preflight"
echo "   3) 测量：     ~/run_measure.sh lat / spin / sign / slope"
echo "   4) 拷回：     ~/run_measure.sh pull"
echo "=============================================================="
