#!/usr/bin/env bash
# =============================================================================
# collect_run.sh —— 把一次现场调试的「全部证据」打成一个包，整包拷回笔记本
#
# 车上用法（在 ~/fp_aim 下）：
#   bash tools/onsite/collect_run.sh                    # 时间戳命名，连图一起打
#   bash tools/onsite/collect_run.sh low3rad            # 指定名字
#   bash tools/onsite/collect_run.sh low3rad --meta-only    # 只打小文件（几 MB）
#   bash tools/onsite/collect_run.sh big --split 2G         # 分卷，方便 U 盘
#
# 选项：
#   --meta-only     不拷 jpg，只拷 frames.csv / 日志 / 配置 / 代码快照
#   --split SIZE    打完切成 SIZE 一片（2G / 1500M 都可以）
#   -o, --out DIR   包放哪（默认 ~/packs）
#   --force         空间不够也硬打
#   -h, --help      看帮助
#
# 打完会打印一行 scp 命令，在笔记本上照抄即可（或者用 pull_run.ps1）。
# =============================================================================
set -euo pipefail

usage() {
  sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
}

RUN=""
META_ONLY=0
SPLIT=""
FORCE=0
OUT="${OUT:-$HOME/packs}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --meta-only) META_ONLY=1; shift ;;
    --split)     SPLIT="${2:-}"; [[ -n "$SPLIT" ]] || { echo "--split 后面要跟大小，比如 2G" >&2; exit 2; }; shift 2 ;;
    -o|--out)    OUT="${2:-}"; [[ -n "$OUT" ]] || { echo "-o 后面要跟目录" >&2; exit 2; }; shift 2 ;;
    --force)     FORCE=1; shift ;;
    -h|--help)   usage; exit 0 ;;
    -*)          echo "不认识的选项：$1" >&2; usage; exit 2 ;;
    *)           RUN="$1"; shift ;;
  esac
done

PROJ="${PROJ:-$HOME/fp_aim}"
[[ -d "$PROJ" ]] || { echo "找不到工程目录：$PROJ" >&2; exit 1; }
[[ -d "$PROJ/recordings" ]] || echo "注意：$PROJ/recordings 不存在，这次可能根本没开录包" >&2

RUN="${RUN:-run_$(date +%m%d_%H%M)}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"

say() { printf '%s\n' "== $* =="; }

# ---------------------------------------------------------------------------
# 0. 先算体量：宁可现在停下，也不要在拷到一半时磁盘满
# ---------------------------------------------------------------------------
say "0/6 体量检查"
need=$(du -sb "$PROJ/recordings" "$PROJ/logs" 2>/dev/null | awk '{s+=$1} END {printf "%d", s+0}')
need=$(( need + 64*1024*1024 ))          # 给配置/代码/manifest 留 64 MB
avail=$(df -B1 --output=avail "$OUT" 2>/dev/null | tail -n 1 | tr -d ' ')
avail=${avail:-0}

human() { awk -v b="$1" 'BEGIN{ split("B KB MB GB TB",u," "); i=1; while (b>=1024 && i<5){b/=1024; i++} printf "%.1f %s", b, u[i] }'; }

# 打包时会在 $OUT/.stage 里摊一份同样大小的目录树（同一分区），再打成 tar，
# 所以全量模式需要 2 倍空间：一份临时树、一份最终包。--meta-only 只有几百 MB，按 1 倍算。
if [[ "$META_ONLY" -eq 1 ]]; then
  want=$need
else
  want=$(( need * 2 ))
fi

printf '  要拷贝的数据：%s\n' "$(human "$need")"
printf '  需要空闲空间：%s（全量打包要同时容纳临时树和 tar）\n' "$(human "$want")"
printf '  目标目录剩  ：%s（%s）\n' "$(human "$avail")" "$OUT"
if [[ "$META_ONLY" -eq 1 ]]; then
  printf '  --meta-only：实际只拷 csv/日志/配置，远小于上面的数\n'
fi
if (( avail < want )); then
  echo "  空间不够：至少还需要 $(human $((want - avail)))" >&2
  if [[ "$FORCE" -eq 0 ]]; then
    echo "  → 先清盘：bash tools/onsite/disk_guard.sh --keep 3 --yes" >&2
    echo "  → 或者加 --meta-only 只带小文件（csv/日志/配置/源码 diff）" >&2
    echo "  → 真要硬打加 --force，但可能写到一半失败" >&2
    exit 1
  fi
  echo "  --force：继续" >&2
fi

df -h "$OUT" | sed 's/^/  /'

# ---------------------------------------------------------------------------
# 1. 摊开目录
# ---------------------------------------------------------------------------
# 临时目录必须落在目标分区（$OUT）里。mktemp 默认在 /tmp，某些车机镜像把 /tmp
# 挂成 tmpfs（内存盘），几个 GB 的录像往里一拷就把内存吃光、系统直接卡死。
mkdir -p "$OUT/.stage"
STAGE="$(mktemp -d "$OUT/.stage/run.XXXXXX")"
trap 'rm -rf "$STAGE"; rmdir "$OUT/.stage" 2>/dev/null || true' EXIT
mkdir -p "$STAGE"/{recordings,logs,configs,code,system}

say "1/6 收集 recordings"
if [[ "$META_ONLY" -eq 1 ]]; then
  count=0
  while IFS= read -r -d '' f; do
    rel="${f#"$PROJ"/}"
    mkdir -p "$STAGE/$(dirname "$rel")"
    cp -p "$f" "$STAGE/$rel"
    count=$(( count + 1 ))
  done < <(find "$PROJ/recordings" -type f \( -name '*.csv' -o -name '*.md' -o -name '*.txt' \) -print0)
  printf '  %d 个小文件（csv/md/txt）\n' "$count"
else
  cp -a "$PROJ/recordings/." "$STAGE/recordings/"
  printf '  %s，%s 个文件\n' "$(du -sh "$STAGE/recordings" | cut -f1)" "$(find "$STAGE/recordings" -type f | wc -l)"
fi

say "2/6 收集 logs / configs"
if [[ -d "$PROJ/logs" ]]; then
  cp -a "$PROJ/logs/." "$STAGE/logs/"
  printf '  logs：%d 个文件 %s\n' "$(find "$STAGE/logs" -type f | wc -l)" "$(du -sh "$STAGE/logs" | cut -f1)"
fi
if [[ -d "$PROJ/configs" ]]; then
  cp -a "$PROJ/configs/." "$STAGE/configs/"     # 连 .bak 一起带，回滚才有据
fi

say "3/6 代码快照（未提交的改动 + 未跟踪文件都在里面）"
# 只拷源码：排除 build / recordings / logs / .git
( cd "$PROJ" && tar -cf - \
    --exclude=./build --exclude=./recordings --exclude=./logs --exclude=./.git . ) \
  | ( cd "$STAGE/code" && tar -xf - )
if [[ -d "$PROJ/.git" ]]; then
  ( cd "$PROJ" && git rev-parse HEAD > "$STAGE/code/HEAD.txt" 2>/dev/null || true )
  ( cd "$PROJ" && git status --porcelain > "$STAGE/code/status.txt" 2>/dev/null || true )
  ( cd "$PROJ" && git diff > "$STAGE/code/diff.patch" 2>/dev/null || true )
  ( cd "$PROJ" && git diff --stat > "$STAGE/code/diffstat.txt" 2>/dev/null || true )
fi
printf '  code：%s\n' "$(du -sh "$STAGE/code" | cut -f1)"

say "4/6 系统信息"
{
  date -Is
  uname -a
  ( cat /etc/os-release 2>/dev/null || true )
  echo "--- nproc: $(nproc 2>/dev/null || echo '?')"
  echo "--- free"; free -h 2>/dev/null || true
  echo "--- df"; df -h 2>/dev/null || true
  echo "--- ip"; (ip -br a 2>/dev/null || hostname -I 2>/dev/null || true )
  echo "--- 环境变量"; env | grep -E '^DSH_' | sort || true
} > "$STAGE/system/sysinfo.txt" 2>&1
echo "  已写 system/sysinfo.txt（含 DSH_* 环境变量，能还原这次到底用什么参数跑的）"

say "5/6 清单与说明"
{
  echo "包名：$RUN"
  echo "生成时间：$(date -Is)"
  echo "主机：$(hostname)"
  echo "工程：$PROJ"
  echo
  echo "目录："
  echo "  recordings/  录包（000000.jpg + frames.csv，回放和离线分析都用它）"
  echo "  logs/        每次运行的终端日志"
  echo "  configs/     参数快照，含 .bak（现场值可能没提交，这里才是真相）"
  echo "  code/        源码快照 + HEAD/status/diff.patch（未提交改动）"
  echo "  system/      机器与运行环境"
  echo "  MANIFEST.sha256  每个文件的校验和"
  echo
  echo "笔记本上怎么用："
  echo "  1) py tools\\analyze_frames.py <解包后的 recordings>      看帧率/缺口/挡位/发弹"
  echo "  2) py tools\\plot_frames.py   <解包后的 recordings>      画四元数角速度曲线"
  echo "  3) py tools\\latency_report.py <lat.csv>                 算链路延迟与云台到位"
  echo "  4) py tools\\lead_advisor.py  <lat.csv>                  给出该设多少 DSH_LEAD"
  echo
  echo "回放：DSH_REPLAY_DIR=<解包后的 recordings/子目录> ./build/task_2 configs/standard.yaml"
} > "$STAGE/README.txt"

( cd "$STAGE" && find . -type f -print0 | sort -z | xargs -0 sha256sum > MANIFEST.sha256 )

say "6/6 打包"
if [[ "$META_ONLY" -eq 1 ]]; then
  PK="$OUT/$RUN.tar.gz"
  tar -C "$STAGE" -czf "$PK" .
else
  # jpg 本来就压过了，再 gzip 只烧 CPU 不省空间
  PK="$OUT/$RUN.tar"
  tar -C "$STAGE" -cf "$PK" .
fi
chmod 644 "$PK"
printf '  %s（%s）\n' "$PK" "$(du -h "$PK" | cut -f1)"

if [[ -n "$SPLIT" ]]; then
  say "切分卷（$SPLIT）"
  split -b "$SPLIT" -d -a 3 "$PK" "$PK.part" && rm -f "$PK"
  sha256sum "$OUT/$RUN.tar"* > "$OUT/$RUN.sha256"
  printf '  %d 卷，清单 %s\n' "$(ls -1 "$OUT/$RUN.tar.part"* | wc -l)" "$OUT/$RUN.sha256"
  FETCH_NAME="$RUN.tar.part*"
else
  ( cd "$OUT" && sha256sum "$(basename "$PK")" > "$RUN.sha256" )
  printf '  校验和：%s\n' "$(cut -d' ' -f1 "$OUT/$RUN.sha256")"
  FETCH_NAME="$(basename "$PK")"
fi

echo
echo "----------------------------------------------------------------"
echo "下一步（笔记本上执行）："
echo "  scp 'rm@192.168.1.50:$OUT/$FETCH_NAME' D:\\DSH\\onsite_runs\\"
echo "  scp 'rm@192.168.1.50:$OUT/$RUN.sha256' D:\\DSH\\onsite_runs\\"
echo "或者交给脚本（自动校验 + 解包）："
echo "  powershell -ExecutionPolicy Bypass -File tools\\onsite\\pull_run.ps1 -Name $RUN -Extract"
echo "----------------------------------------------------------------"
