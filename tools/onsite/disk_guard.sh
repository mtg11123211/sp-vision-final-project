#!/usr/bin/env bash
# disk_guard.sh —— 车载电脑（Linux）上用的磁盘看护工具。
#
# 目的：录像非常吃空间（58 fps 时约 0.4-0.9 GB/分钟），到场开跑之前先看一眼、
#      把没用的旧场次清掉，不要等到录到一半才发现没空间。
#
# 用法（在车上，~/fp_aim 下）：
#   bash tools/onsite/disk_guard.sh                       # 只看，不动任何文件
#   bash tools/onsite/disk_guard.sh --keep 3              # 演练：只留最新 3 场，打印会删什么
#   bash tools/onsite/disk_guard.sh --keep 3 --yes        # 真删
#   bash tools/onsite/disk_guard.sh --older-than 7 --yes  # 删 7 天前的
#   bash tools/onsite/disk_guard.sh --min-free 20         # 可用空间低于 20 GB 时退出码 1
#
# 安全约定：
#   * 默认只读，删除必须显式加 --yes；
#   * 只允许删除「--dir 指定的录像目录」的直接子目录，且该目录名必须含 recordings；
#   * 不会碰 logs/、configs/、build/、源码。
#
# 注意：录像写失败【不会】让程序崩溃 —— tools/recorder.cpp:89 写图失败只 warn 并丢帧，
#      frames.csv 的写入失败也只是静默不写。所以盘满的代价是「丢数据」，不是「跑崩」。

set -uo pipefail

DIR="${FP_REC_DIR:-$HOME/fp_aim/recordings}"
KEEP=""
OLDER_THAN=""
MIN_FREE_GB=""
YES=0
RATE_MB_MIN=500          # 估算用：0.5 GB/分钟（1408x1104 q95 实测 0.4-0.9）
DRY=""

while [ $# -gt 0 ]; do
  case "$1" in
    --dir)        DIR="${2:-}"; shift 2 ;;
    --keep)       KEEP="${2:-}"; shift 2 ;;
    --older-than) OLDER_THAN="${2:-}"; shift 2 ;;
    --min-free)   MIN_FREE_GB="${2:-}"; shift 2 ;;
    --rate-mb-min) RATE_MB_MIN="${2:-}"; shift 2 ;;
    --yes|-y)     YES=1; shift ;;
    -h|--help)    sed -n '2,28p' "$0"; exit 0 ;;
    *) echo "未知参数：$1（--help 看用法）" >&2; exit 2 ;;
  esac
done

say() { printf '%s\n' "$*"; }
hr()  { printf '%s\n' "------------------------------------------------------------"; }

human() {  # bytes -> 人类可读
  awk -v b="$1" 'BEGIN{
    split("B KB MB GB TB",u," "); i=1;
    while (b>=1024 && i<5) { b/=1024; i++ }
    printf "%.1f %s", b, u[i]
  }'
}

# ---------- 0. 前置检查 ----------
if [ ! -d "$DIR" ]; then
  say "录像目录不存在：$DIR"
  say "（还没录过？先设 DSH_RECORD_DIR 跑一场就有了）"
  exit 0
fi

# 规范化路径，并做一道安全断言：只允许动 …/recordings 下面的东西
REAL="$(readlink -f "$DIR" 2>/dev/null || echo "$DIR")"
case "$REAL" in
  */recordings|*/recordings/*) : ;;
  *) say "拒绝操作：$REAL 看起来不是录像目录（路径里必须含 recordings）。"
     say "如果确实要清别的地方，请手动处理，这个脚本不越界。"
     exit 2 ;;
esac

# ---------- 1. 磁盘总览 ----------
hr; say "== 磁盘总览 =="
df -h "$REAL" | sed -n '1,2p'
AVAIL_KB="$(df -Pk "$REAL" | awk 'NR==2{print $4}')"
AVAIL_GB=$(( AVAIL_KB / 1024 / 1024 ))
say "本分区可用：${AVAIL_GB} GB"

# ---------- 2. 录像占用 ----------
hr; say "== 录像目录 =="
TOTAL_BYTES="$(du -sb "$REAL" 2>/dev/null | awk '{print $1}')"
say "$REAL"
say "合计：$(human "${TOTAL_BYTES:-0}")"
RUNS="$(find "$REAL" -mindepth 1 -maxdepth 1 -type d | sort)"
RUN_COUNT="$(printf '%s\n' "$RUNS" | grep -c . || true)"
say "场次：${RUN_COUNT} 个"

if [ "$RUN_COUNT" -gt 0 ]; then
  say ""
  printf '%-34s %10s %6s  %s\n' "目录" "大小" "图片数" "时间"
  # 新的排前面：按 mtime 倒序
  while IFS= read -r d; do
    [ -n "$d" ] || continue
    b="$(du -sb "$d" 2>/dev/null | awk '{print $1}')"
    n="$(find "$d" -maxdepth 1 -name '*.jpg' 2>/dev/null | wc -l | tr -d ' ')"
    t="$(date -r "$d" '+%m-%d %H:%M' 2>/dev/null || echo '?')"
    printf '%-34s %10s %6s  %s\n' "$(basename "$d")" "$(human "${b:-0}")" "$n" "$t"
  done < <(find "$REAL" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' 2>/dev/null \
             | sort -rn | cut -d' ' -f2-)
fi

# ---------- 3. 还能录多久 ----------
hr; say "== 还能录多久 =="
FREE_MB=$(( AVAIL_KB / 1024 ))
CAN_MIN=$(( FREE_MB / RATE_MB_MIN ))
say "按 ${RATE_MB_MIN} MB/分钟 估（可用 ${FREE_MB} MB）：约 ${CAN_MIN} 分钟"
say "这只是估算。同一台相机同分辨率下差异主要来自画面噪声，实测为准。"
say ""
say "三个可以立刻降体积的杠杆："
say "  1) configs/standard.yaml:96  record_every: 1 -> 4    体积直接降到 1/4（回放时 14.5 fps）"
say "  2) configs/standard.yaml:97  record_jpeg_quality: 95 -> 85   体积约降到 60%"
say "  3) 只录关键场次：调试跑位、试转速时把 DSH_RECORD_DIR 留空"

# ---------- 4. 清理 ----------
NEED_ACTION=0
if [ -n "$KEEP" ] || [ -n "$OLDER_THAN" ]; then NEED_ACTION=1; fi

if [ "$NEED_ACTION" -eq 0 ]; then
  hr
  if [ "$MIN_FREE_GB" != "" ] && [ "$AVAIL_GB" -lt "$MIN_FREE_GB" ]; then
    say "!! 可用空间 ${AVAIL_GB} GB < 要求 ${MIN_FREE_GB} GB —— 先清盘再开录："
    say "   bash tools/onsite/disk_guard.sh --keep 3 --yes"
    exit 1
  fi
  say "只读模式，什么都没删。要清旧场次："
  say "   bash tools/onsite/disk_guard.sh --keep 3 --yes      # 只留最新 3 场"
  say "   bash tools/onsite/disk_guard.sh --older-than 7 --yes # 删 7 天前的"
  exit 0
fi

# 候选集合：按 mtime 从新到旧
mapfile -t ALL < <(find "$REAL" -mindepth 1 -maxdepth 1 -type d -printf '%T@ %p\n' \
                     | sort -rn | cut -d' ' -f2-)

DEL=()
if [ -n "$KEEP" ]; then
  case "$KEEP" in
    ''|*[!0-9]*) say "--keep 需要一个非负整数" >&2; exit 2 ;;
  esac
  i=0
  for d in "${ALL[@]}"; do
    i=$((i+1))
    if [ "$i" -gt "$KEEP" ]; then DEL+=("$d"); fi
  done
fi

if [ -n "$OLDER_THAN" ]; then
  case "$OLDER_THAN" in
    ''|*[!0-9]*) say "--older-than 需要一个非负整数（天）" >&2; exit 2 ;;
  esac
  NOW=$(date +%s)
  for d in "${ALL[@]}"; do
    # 去掉 -mtime 的语义歧义：按目录 mtime 精确算秒
    MT=$(stat -c %Y "$d" 2>/dev/null || echo "$NOW")
    AGE_D=$(( (NOW - MT) / 86400 ))
    if [ "$AGE_D" -ge "$OLDER_THAN" ]; then DEL+=("$d"); fi
  done
fi

# 去重（--keep 与 --older-than 同时给时可能重复）
mapfile -t DEL < <(printf '%s\n' "${DEL[@]:-}" | awk 'NF' | sort -u)

if [ "${#DEL[@]}" -eq 0 ] && [ -z "$(printf '%s' "${DEL[0]:-}")" ]; then
  say "没有需要删除的场次。"
  exit 0
fi

FREED=0
say ""
say "将要删除（共 ${#DEL[@]} 场）："
for d in "${DEL[@]}"; do
  [ -n "$d" ] || continue
  b="$(du -sb "$d" 2>/dev/null | awk '{print $1}')"
  FREED=$(( FREED + ${b:-0} ))
  printf '  %-34s %10s\n' "$(basename "$d")" "$(human "${b:-0}")"
done
say "预计释放：$(human "$FREED")"
say "删除后可用约：$(( AVAIL_GB + FREED / 1024 / 1024 / 1024 )) GB"

if [ "$YES" -ne 1 ]; then
  say ""
  say "（演练模式，未删除。确认无误后加 --yes 再跑一次）"
  exit 0
fi

say ""
DELETED=0
for d in "${DEL[@]}"; do
  [ -n "$d" ] || continue
  # 最后一道闸：必须是 $REAL 的直接子目录
  case "$d" in
    "$REAL"/*) : ;;
    *) say "跳过（不在录像目录下）：$d"; continue ;;
  esac
  if [ "$(dirname "$d")" != "$REAL" ]; then
    say "跳过（不是直接子目录）：$d"; continue
  fi
  rm -rf -- "$d" && DELETED=$((DELETED+1))
done
say "已删除 ${DELETED} 场。"
df -h "$REAL" | sed -n '2p'
