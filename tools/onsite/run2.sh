#!/usr/bin/env bash
# run2.sh —— 车机上「编译 + 启动 task_2」的一键脚本（替换旧版 ~/run2.sh）
#
# 为什么旧版不能直接用了：
#   旧版只 touch src/task_2.cpp。这一版改动横跨 5 个 .cpp，其中 armor.cpp（颜色映射）
#   是 task_2 / task_3 共享的，io/camera.cpp 和 io/hikrobot/hikrobot.cpp（设备时间戳）
#   是库文件。这些文件的 mtime 如果比 .o 旧，make 会跳过 —— 于是你以为在测新代码，
#   其实跑的是旧二进制。旧版还有第二个坑：make 失败时它不会停，直接启动了旧程序。
#
# 用法（在车上，任意目录）：
#   ~/run2.sh                       # 编译 + 启动
#   ~/run2.sh --no-build            # 已经编过，只启动
#   DSH_LEAD=0.06 ~/run2.sh         # 环境变量照常往下透传
#   DSH_RECORD_DIR=$HOME/runs/t2_$(date +%m%d_%H%M) ~/run2.sh
#
# 前提（和以前一样）：
#   * 工程在 $HOME/fp_aim
#   * 必须在工程根启动（assets/yolov5.xml 是相对路径）
#   * 改了 CMakeLists.txt 才需要重新 cmake，改 .cpp/.hpp/.yaml 都不需要

set -euo pipefail

BIN="task_2"
SRC="src/task_2.cpp"
PROJ="${FP_AIM_HOME:-$HOME/fp_aim}"
BUILD=1

for a in "$@"; do
  case "$a" in
    --no-build) BUILD=0 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "未知参数：$a（--no-build 表示只启动）" >&2; exit 2 ;;
  esac
done

[ -d "$PROJ" ] || { echo "找不到工程目录：$PROJ" >&2; exit 1; }
cd "$PROJ"
[ -f configs/standard.yaml ] || { echo "找不到 configs/standard.yaml（目录不对？）" >&2; exit 1; }

echo "环境：DSH_LEAD=${DSH_LEAD:-未设=0}  DSH_ARRIVE_DEG=${DSH_ARRIVE_DEG:-未设=关}  DSH_RECORD_DIR=${DSH_RECORD_DIR:-未设=不录}"
echo "       DSH_LAT_CSV=${DSH_LAT_CSV:-未设=不记}  DSH_DET_CSV=${DSH_DET_CSV:-未设=不记}"

if [ "$BUILD" -eq 1 ]; then
  echo "== 1/3 标记需要重编的源文件 =="
  # 这一版所有改动过的 .cpp。头文件（latency_probe.hpp / detect_log.hpp / overlay.hpp）
  # 由编译器的依赖文件自动追踪，只要对应的 .cpp 被 touch 就会跟着重编。
  for f in "$SRC" \
           tasks/auto_aim/armor.cpp \
           io/camera.cpp \
           io/hikrobot/hikrobot.cpp \
           io/gimbal/gimbal.cpp ; do
    if [ -f "$f" ]; then
      touch "$f"
      echo "  touch $f"
    else
      echo "  （没有 $f，跳过）"
    fi
  done

  echo "== 2/3 编译（只看这行和 Linking 那行）=="
  ( cd build && make -j2 "$BIN" )

  echo "== 3/3 确认真的编了 =="
  ls -l --time-style='+%H:%M:%S' "$SRC" "./build/$BIN"
  if [ "./build/$BIN" -nt "$SRC" ]; then
    echo "  OK：二进制比源文件新，改动已经编进去"
  else
    echo "  !! 二进制不比源文件新 —— make 跳过了编译，现在跑的是旧代码" >&2
    echo "     先找上面有没有 'Building CXX object .../$SRC.o' 和 'Linking CXX executable $BIN'" >&2
    exit 1
  fi
fi

echo "== 启动（Ctrl-C 退出）=="
exec "./build/$BIN" configs/standard.yaml
