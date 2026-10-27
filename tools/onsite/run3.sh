#!/usr/bin/env bash
# run3.sh —— 车机上「编译 + 启动 task_3」的一键脚本（替换旧版 ~/run3.sh）
#
# 和 run2.sh 的差别只有三处：可执行文件、源文件、touch 列表里的 task_3.cpp。
# 共享的 armor.cpp（颜色映射）和 io/ 下的相机代码一样要 touch，原因见 run2.sh 顶部注释。
#
# 用法（在车上，任意目录）：
#   ~/run3.sh                       # 编译 + 启动
#   ~/run3.sh --no-build            # 已经编过，只启动
#
# 注意：task_3 打前哨站，验收时需要哨兵机器人 + 血量 1500 + 30 秒计时，
#       现场条件不满足时优先用 run2.sh 验证共享代码。

set -euo pipefail

BIN="task_3"
SRC="src/task_3.cpp"
# 配置文件：task_3 必须用哨兵配置（rotation 180 / 自己的 pitch_offset 0.0）。
# 旧版这里硬编码 configs/standard.yaml，是照抄 run2.sh 时漏改的——
# 哨兵相机倒装，rotation 0 ⇒ 画面上下颠倒必空；pitch 0.5 是 task2 的标定值，套错车。
# 要临时换：FP_AIM_CFG=configs/xxx.yaml ~/run3.sh
CFG="${FP_AIM_CFG:-configs/sentry.yaml}"
PROJ="${FP_AIM_HOME:-$HOME/fp_aim}"
BUILD=1

for a in "$@"; do
  case "$a" in
    --no-build) BUILD=0 ;;
    -h|--help)  sed -n '2,15p' "$0"; exit 0 ;;
    *) echo "未知参数：$a（--no-build 表示只启动）" >&2; exit 2 ;;
  esac
done

[ -d "$PROJ" ] || { echo "找不到工程目录：$PROJ" >&2; exit 1; }
cd "$PROJ"
[ -f "$CFG" ] || { echo "找不到 $CFG（目录不对？FP_AIM_HOME=$PROJ）" >&2; exit 1; }

echo "环境：DSH_RECORD_DIR=${DSH_RECORD_DIR:-未设=不录}  DSH_DET_CSV=${DSH_DET_CSV:-未设=不记}"

if [ "$BUILD" -eq 1 ]; then
  echo "== 1/3 标记需要重编的源文件 =="
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

echo "== 启动（Ctrl-C 退出）：$BIN + $CFG =="
exec "./build/$BIN" "$CFG"
