#!/usr/bin/env bash
# run_measure.sh —— 车机上「预检 → 测量 → 汇总」一条龙（task_2 专用的现场标定脚本）
#
# 与 run2.sh 的区别：run2.sh 只负责"编译 + 启动"；这一版把**测量流程**也脚本化了，
# 每一场都自动带上场次名、汇总文件、日志文件，并把"要做什么、看哪几个数"打在屏幕上。
#
# 用法（在车上，任意目录）：
#   ~/run_measure.sh preflight            # ① 预检：文件/编译/按键通道（最先跑这个）
#   ~/run_measure.sh lat     [tag] [秒]   # ② B1+B2：记延迟 CSV（默认 30 s）
#   ~/run_measure.sh static  [tag]        # ③ A2a/C4：静止靶（零位 + 共有偏置）
#   ~/run_measure.sh spin    [tag] [lead] # ④ A2b/C1-C3：旋转靶（默认 lead=0.080）
#   ~/run_measure.sh sign    [tag]        # ⑤ lead 符号自检（0 与 0.57 各 10 s，自动比）
#   ~/run_measure.sh slope   [tag]        # ⑥ 斜率法定标（两档 lead，自动提示读什么数）
#   ~/run_measure.sh report  [tag]        # ⑦ 汇总 + 判读
#   ~/run_measure.sh pull                # ⑧ 打印"把 logs/ 拷回笔记本"的命令（网线直连 / U 盘）
#   ~/run_measure.sh link  [笔记本IP]     # ⑨ 网线链路诊断（**只在拷不回来时跑**）
#   ~/run_measure.sh all                 # ① → ② → ④ → ⑦ 一条龙
#
# 网络前提：**车机永远没外网**，只有到笔记本的网线直连。
#   笔记本有线 IP 已固定 192.168.1.43；车机默认 192.168.1.50。
#   车机上没有 python3 ⇒ 所有分析（延迟报告/判读/记录表/斜率法求解）都在笔记本上做。
#
# 前提：工程在 $HOME/fp_aim；必须在工程根启动（assets/yolov5.xml 是相对路径）。
# 环境变量照常透传：DSH_LEAD=0.09 ~/run_measure.sh spin p5

set -euo pipefail

PROJ="${FP_AIM_HOME:-$HOME/fp_aim}"
# 配置文件名：默认走哨兵配置。换车/换配置用 DSH_CONFIG 覆盖，不用改脚本。
CFG="${DSH_CONFIG:-configs/sentry.yaml}"
BIN="task_2"
TS="$(date +%m%d_%H%M)"

usage() { sed -n '2,22p' "$0"; }

# ---------------------------------------------------------------- 公共
log()  { echo "== $* =="; }
step() { echo "--- $* ---"; }
die()  { echo "!! $*" >&2; exit 1; }

need_proj() {
  [ -d "$PROJ" ] || die "找不到工程目录：$PROJ（可用 FP_AIM_HOME 指定）"
  cd "$PROJ"
  [ -f "$CFG" ] || die "找不到 $CFG（目录不对？或用 DSH_CONFIG 指定）"
}

# 每场的固定三件套：场次名 / 汇总文件 / 日志文件
setup_run() {   # $1=tag  →  导出 DSH_TEST_TAG / DSH_SUMMARY_FILE / LOG
  TAG="$1"
  export DSH_TEST_TAG="$TAG"
  export DSH_SUMMARY_FILE="$PROJ/logs/summary_${TAG}_${TS}.txt"
  LOG="$PROJ/logs/${TAG}_${TS}.log"
  mkdir -p "$PROJ/logs"
}

run_task2() {   # $1=提示（屏幕上显示），其余环境变量由调用方设好
  echo "--------------------------------------------------------------"
  echo "  $1"
  echo "  跑完后按 q 或 Ctrl+C 退出（退出时自动打出汇总块）"
  echo "  配置：$CFG"
  echo "  日志：$LOG"
  echo "  汇总：$DSH_SUMMARY_FILE"
  echo "--------------------------------------------------------------"
  set +e
  "./build/$BIN" "$CFG" 2>&1 | tee "$LOG"
  set -e
  echo
  echo "== 本场汇总（$DSH_SUMMARY_FILE）=="
  if [ -f "$DSH_SUMMARY_FILE" ]; then cat "$DSH_SUMMARY_FILE"; else
    echo "（没有汇总文件，检查 DSH_SUMMARY_FILE 是否可写）" >&2
  fi
}

cmd_yaw_mean() {  # 从一份日志里取 cmd yaw 的平均值（判 lead 符号用）
  grep -o 'cmd yaw=-\?[0-9.]*' "$1" 2>/dev/null | sed 's/.*=//' \
    | awk '{s+=$1; n++} END { if (n>0) printf "%.4f", s/n; else print "nan" }'
}

# ---------------------------------------------------------------- ① 预检
preflight() {
  need_proj
  log "1/6 检查是不是副本版（6 个新文件/脚本）"
  miss=0
  for f in src/fire_key.hpp src/low_plate.hpp src/env_util.hpp src/test_summary.hpp tools/keytest.cpp \
           tools/parse_session.py; do
    if [ -f "$f" ]; then echo "  OK  $f"; else echo "  缺  $f"; miss=1; fi
  done
  [ "$miss" -eq 0 ] || die "这不是副本版（或没传全）——先按 docs/debug_flow.md §1.2 传文件"

  log "2/6 验收 grep（计数是 2026-10-21 副本版本的实测值）"
  ck() {  # $1=pattern $2=期望说明
    n=$(grep -c "$1" src/task_2.cpp || true)
    echo "  $1 → $n    ($2)"
  }
  ck plate_facing_deg "期望 >=4"
  ck target_static    "期望 >=2"
  ck DSH_LEAD         "期望 >=7"
  ck fire_key         "期望 >=20"
  ck low_plate        "期望 >=9"
  ck test_summary     "期望 >=4"
  ck 选板变化          "期望 >=1（副本才有这行日志）"
  grep -nE '^(yaw_offset|pitch_offset):' "$CFG" || true
  grep -n  'fire_interval_s' "$CFG" || true

  log "3/6 编译（改了 CMakeLists 要重新 cmake，keytest 是新目标）"
  need_cmake=0
  if [ ! -f build/CMakeCache.txt ]; then
    need_cmake=1
  elif [ ! -x build/keytest ]; then
    echo "  build/keytest 不存在 ⇒ 重新 cmake"
    need_cmake=1
  fi
  if [ "$need_cmake" -eq 1 ]; then
    if command -v cmake >/dev/null 2>&1; then
      ( cd build && cmake -B . .. >/dev/null ) || echo "  ⚠ cmake 失败，继续往下（下面的 make 会报真实原因）"
    else
      echo "  ⚠ 找不到 cmake，跳过重新配置（若是第一次编译，请手动：cd build && cmake -B . ..）"
    fi
  fi
  for f in src/task_2.cpp tasks/auto_aim/armor.cpp tasks/auto_aim/target.cpp \
           io/camera.cpp io/hikrobot/hikrobot.cpp io/gimbal/gimbal.cpp; do
    [ -f "$f" ] && touch "$f"
  done
  ( cd build && make -j2 "$BIN" keytest )
  if [ ! "build/$BIN" -nt src/task_2.cpp ]; then
    die "二进制不比源文件新 —— make 跳过了编译"
  fi
  echo "  OK  build/$BIN 与 build/keytest 就绪"

  log "4/6 按键通道（5 秒自动退出）"
  if [ -x build/keytest ]; then
    timeout 5 ./build/keytest 2>&1 | head -6 || true
    echo "  ↑ backend 必须是 evdev/termios；cvkey(window) ⇒ 窗口没焦点，改用 DSH_FIRE_KEY=toggle"
  else
    echo "  没有 build/keytest"
  fi

  log "5/6 挡位检查 + 启动参数（跑 6 秒，只跑一次，两处分别 grep）"
  tmp="$(mktemp)"
  set +e
  timeout 6 "./build/$BIN" "$CFG" >"$tmp" 2>&1
  set -e
  echo "  -- 挡位（关键：不是 AUTO_AIM 就不发弹）--"
  if grep -m1 '电控挡位' "$tmp"; then :; else echo "  （日志里没有挡位行！）" >&2; fi
  echo "  -- 启动参数 --"
  grep -E '\[task2\] (offset|到位门控|低位靶规则|出弹延时|预测时域)' "$tmp" | head -6 || true
  rm -f "$tmp"

  log "6/6 预检结论"
  echo "  ① 编译 OK  ② grep 计数对得上  ③ 按键通道看 backend  ④ 挡位是 AUTO_AIM"
  echo "  下一步：~/run_measure.sh lat  （测 B1/B2 延迟）"
}

# ---------------------------------------------------------------- ② B1+B2 延迟
stage_lat() {
  need_proj; setup_run "${1:-lat}"
  local secs="${2:-30}"
  export DSH_LAT_CSV="$PROJ/logs/lat_${TAG}_${TS}.csv"
  log "B1/B2：记 $secs 秒延迟探针"
  echo "  CSV：$DSH_LAT_CSV"
  echo "  ★ 别录像（record_dir 必须空），否则帧率 58→45，时间基准全乱"
  echo "  ★ 期间可以只跟踪不打；让靶子转起来更有意义（云台滞后才有数据）"
  set +e
  timeout "$secs" "./build/$BIN" "$CFG" 2>&1 | tee "$LOG"
  set -e
  echo
  if command -v python3 >/dev/null 2>&1; then
    echo "== 延迟报告（车机 python3）=="
    python3 tools/latency_report.py "$DSH_LAT_CSV" --markdown || true
  else
    echo "车机没有 python3 ⇒ 把这个 CSV 拷回笔记本跑："
    echo "    py tools\\latency_report.py logs\\lat_${TAG}_${TS}.csv --markdown"
  fi
  echo "  填进记录表：B1 视觉延迟 / B2 云台反馈滞后（相关系数 ≥0.5 才认）"
}

# ---------------------------------------------------------------- ③ 静止靶
stage_static() {
  need_proj; setup_run "${1:-static}"
  # 三个开关默认关；外面显式传了就听外面的（M6~M9 要用同一条命令只换一个变量）
  export DSH_LOW_PLATE="${DSH_LOW_PLATE:-0}"
  export DSH_FIRE_KEY_ENABLE=0   # 硬置 0：继承来的 DSH_FIRE_KEY_ENABLE=1 也不许把枪口关掉（10-23 现场要求）
  export DSH_ARRIVE_DEG="${DSH_ARRIVE_DEG:-1.2}"
  export DSH_LEAD="${DSH_LEAD:-0.0}"     # 静止靶视在角速度=0，lead 填 0 即可
  run_task2 "③ 静止靶：打 3 发，量落点相对靶心的偏移（mm），按 q 退出"
  echo "  记：偏移 mm（左右/上下）→ 用 py tools\\zero_fix.py --d 3.15 --dx <mm> --dy <mm> 算零位"
  echo "  ★ 静止靶标不了 lead（视在角速度=0 ⇒ lead 恒为 0）"
}

# ---------------------------------------------------------------- ④ 旋转靶
stage_spin() {
  need_proj; setup_run "${1:-spin}"
  local lead="${2:-}"
  export DSH_LOW_PLATE="${DSH_LOW_PLATE:-0}"
  export DSH_FIRE_KEY_ENABLE=0   # 硬置 0：继承来的 DSH_FIRE_KEY_ENABLE=1 也不许把枪口关掉（10-23 现场要求）
  export DSH_ARRIVE_DEG="${DSH_ARRIVE_DEG:-1.2}"
  # 只动中高速档：DSH_LEAD 会一票覆盖两档，这里显式清掉，避免 shell 里残留的旧值把低速档也改了。
  unset DSH_LEAD
  export DSH_LEAD_MID="${lead:-${DSH_LEAD_MID:-0.024}}"
  run_task2 "④ 旋转靶（中高速 lead=${DSH_LEAD_MID}s，低速 0s）：打 10 发（或跑 30 s），记命中数，按 q 退出"
  echo "  这次要读：命中/发数、errY 均值(C1/A2)、选板切换次数(C3)、dz(C1 高度差)"
}

# ---------------------------------------------------------------- ⑤ 符号自检
stage_sign() {
  need_proj; setup_run "${1:-sign}"
  export DSH_LOW_PLATE="${DSH_LOW_PLATE:-0}"
  export DSH_FIRE_KEY_ENABLE=0   # 硬置 0：继承来的 DSH_FIRE_KEY_ENABLE=1 也不许把枪口关掉（10-23 现场要求）
  export DSH_ARRIVE_DEG="${DSH_ARRIVE_DEG:-1.2}"
  local a="$PROJ/logs/sign0_${TS}.log" b="$PROJ/logs/sign57_${TS}.log"
  log "⑤ lead 符号自检：两档各 10 s（不发弹，只看指令角）"
  echo "  判据：0.57 那场的 cmd yaw 平均值应**小**约 8.3°（0.57 s × 14.55°/s）"
  set +e
  DSH_LEAD=0    timeout 10 "./build/$BIN" "$CFG" 2>&1 | tee "$a" >/dev/null
  DSH_LEAD=0.57 timeout 10 "./build/$BIN" "$CFG" 2>&1 | tee "$b" >/dev/null
  set -e
  local ma mb
  ma=$(cmd_yaw_mean "$a"); mb=$(cmd_yaw_mean "$b")
  echo "  cmd yaw 均值：lead=0 → $ma ；lead=0.57 → $mb"
  if [ "$ma" = "nan" ] || [ "$mb" = "nan" ]; then
    echo "  （没抓到 cmd yaw，检查日志里有没有 '[dbg] arrive' 行）" >&2
  else
    local d
    d=$(awk -v a="$ma" -v b="$mb" 'BEGIN{d=b-a; if(d<0) d=-d; printf "%.2f", d}')
    echo "  |Δ| = ${d}°   （期望 ≈ 8.3° = 0.57 s × 14.55°/s）"
    if awk -v d="$d" 'BEGIN{exit !(d>6.5 && d<10.5)}'; then
      echo "  ⇒ 量级对得上（补偿量级没问题）。★ **符号以肉眼判据为准**：Δ 的正负随靶子顺/逆时针互换，"
      echo "    所以别拿正负判断对错 —— 看画面：0.57 那场准星应明显**领先**板子（偏向板子转过去的方向）。"
      echo "    若准星明显落在板子**后面** ⇒ 符号反了，别发弹，先查 facing_of 的 '+ M_PI'。"
    else
      echo "  ⇒ 与 8.3° 差太远 ⇒ 靶子可能没在转、ω 不对，或 cmd yaw 样本太少（看日志里 '[dbg] arrive' 行够不够）"
    fi
  fi
  echo "  ★ 肉眼看画面再判一次：0.57 那场准星应明显**领先**板子（偏向板子转过去的方向）"
}

# ---------------------------------------------------------------- ⑥ 斜率法
stage_slope() {
  need_proj; setup_run "${1:-slope}"
  export DSH_LOW_PLATE="${DSH_LOW_PLATE:-0}"
  export DSH_FIRE_KEY_ENABLE=0   # 硬置 0：继承来的 DSH_FIRE_KEY_ENABLE=1 也不许把枪口关掉（10-23 现场要求）
  export DSH_ARRIVE_DEG="${DSH_ARRIVE_DEG:-1.2}"
  local la="${2:-0.057}" lb="${3:-0.171}"
  local tmpl="$PROJ/logs/slope_${TAG}_${TS}.txt"
  log "⑥ 斜率法定标：两档 lead，各打 3~5 发，量落点沿「靶子转动方向」的偏移(mm)"
  echo "  理论灵敏度 k = r·ω = 0.2 m × 4 rad/s = 800 mm/s"
  echo "  ⇒ 两档差 $(awk -v a="$la" -v b="$lb" 'BEGIN{printf "%.3f", b-a}') s ⇒ 落点应移动约 $(awk -v a="$la" -v b="$lb" 'BEGIN{printf "%.0f", (b-a)*800}') mm"
  echo "  ★ 偏移的正负：沿靶子转动方向为**正**、反向为**负**（判错符号 = 整个结论反了）"
  echo "  ★ 读不出就把两档差加大：0.057 vs 0.243（位移 150 mm，肉眼可辨）"
  echo

  # 两档各留各的日志与汇总（否则第二档会把第一档的 tee 覆盖掉）
  export DSH_LEAD="$la"
  LOG="$PROJ/logs/${TAG}_a_${TS}.log"
  export DSH_SUMMARY_FILE="$PROJ/logs/summary_${TAG}a_${TS}.txt"
  run_task2 "⑥-1 第一档 lead=$la：打 3~5 发，量偏移(mm)，按 q 退出"
  export DSH_LEAD="$lb"
  LOG="$PROJ/logs/${TAG}_b_${TS}.log"
  export DSH_SUMMARY_FILE="$PROJ/logs/summary_${TAG}b_${TS}.txt"
  run_task2 "⑥-2 第二档 lead=$lb：打 3~5 发，量偏移(mm)，按 q 退出"

  cat > "$tmpl" <<EOF
# 斜率法记录表：把 off_a / off_b 填上（mm，沿转动方向为正），其余不用改
# 生成于 $(date '+%Y-%m-%d %H:%M:%S')  场次 $TAG
lead_a = $la
off_a =
lead_b = $lb
off_b =
omega = 4
r = 0.2
dist = 3.15
# b1 = 25      ← 视觉延迟 ms（填了就能顺带拆出 B3）
# b2 = 32      ← 云台反馈滞后 ms
# fps = 58
EOF
  echo
  echo "== 已生成记录模板 =="
  echo "  $tmpl"
  echo "  填完 off_a / off_b 后，在笔记本上算："
  echo "    py tools\\lead_solve.py --from $tmpl"
}

# ---------------------------------------------------------------- ⑦ 汇总
stage_report() {
  need_proj
  local tag="${1:-}"
  echo "== 最新的汇总文件 =="
  ls -t logs/summary_*.txt 2>/dev/null | head -5 || echo "（没有 summary_*.txt）"
  echo
  if command -v python3 >/dev/null 2>&1; then
    if [ -n "$tag" ]; then
      python3 tools/parse_session.py "logs/${tag}"*.log --tag "$tag" --csv logs/sessions.csv || true
    else
      python3 tools/parse_session.py logs/*.log --csv logs/sessions.csv || true
    fi
  else
    echo "车机没有 python3 ⇒ 拷回笔记本再判读（见 ~/run_measure.sh pull 的命令）："
    echo "    py tools\\parse_session.py logs\\*.log --csv logs\\sessions.csv --out logs\\records.md"
    echo "    py tools\\make_record.py --csv logs\\sessions.csv --out logs\\记录表.md"
  fi
}

# ---------------------------------------------------------------- 拷回笔记本
# 拓扑：车机 ⇄（网线）⇄ 笔记本。车机**永远没外网**，只有这条链路。
# 笔记本的有线 IP 已固定为 192.168.1.43。
LAPTOP_IP="192.168.1.43"

stage_pull() {
  echo "== 车机没外网 ⇒ 拷回只有两条路：网线直连（主）或 U 盘（备）=="
  echo
  echo "① 网线直连（笔记本 PowerShell；笔记本有线 IP 已固定 192.168.1.43）"
  echo "     scp -r rm@192.168.1.50:~/fp_aim/logs/*  D:\\MTG\\Documents\\OC\\fp_aim_fkey\\logs\\"
  echo "   车机的 IP 以 'ip -4 addr' 显示的为准（默认 192.168.1.50）"
  echo "   ★ 拷不回来时再跑：~/run_measure.sh link   （打印车机 IP/路由/sshd/常见病因）"
  echo
  echo "② U 盘（网线不通时用，车机上跑）："
  echo "     lsblk                                    # 找 U 盘的设备名（通常 /dev/sdb1）"
  echo "     mkdir -p /media/\$USER/fp_logs && mount /dev/sdb1 /media/\$USER/fp_logs"
  echo "     cp -r ~/fp_aim/logs/* /media/\$USER/fp_logs/ && sync"
  echo "     umount /media/\$USER/fp_logs"
  echo "   然后在 Windows 上把 U 盘里的 fp_logs 拷到 D:\\MTG\\Documents\\OC\\fp_aim_fkey\\logs\\"
  echo
  echo "③ 拷回之后，在**笔记本**上做判读与记录表："
  echo "     py tools\\parse_session.py logs\\*.log --csv logs\\sessions.csv --out logs\\records.md"
  echo "     py tools\\make_record.py --csv logs\\sessions.csv --out logs\\记录表.md"
  echo "     py tools\\latency_report.py logs\\lat_*.csv --markdown"
  echo
  echo "④ 斜率法那一档的模板（logs/slope_*.txt）也要一起拷回来，填完 off_a/off_b 再算："
  echo "     py tools\\lead_solve.py --from logs\\slope_xxx.txt"
}

# ---------------------------------------------------------------- 链路诊断（只在拷不回来时跑）
stage_link() {
  local lip="${1:-$LAPTOP_IP}"
  echo "== 网线直连诊断（车机没外网，只有到笔记本的这条链路）=="
  echo "== 笔记本有线 IP：$lip（已固定）"
  echo
  echo "-- 车机的网卡与 IP --"
  ip -4 addr show 2>/dev/null | sed 's/^/   /' || echo "   （ip 命令不可用）"
  echo
  echo "-- 路由 --"
  ip route 2>/dev/null | sed 's/^/   /' || true
  echo
  echo "-- sshd 在不在（笔记本要 scp 过来，车机上必须是 sshd + 开着）--"
  if command -v systemctl >/dev/null 2>&1; then
    systemctl is-active ssh 2>/dev/null | sed 's/^/   ssh: /' || true
    systemctl is-active sshd 2>/dev/null | sed 's/^/   sshd: /' || true
  fi
  ss -ltn 2>/dev/null | grep -E ':22\b' | sed 's/^/   /' || echo "   （22 端口没在听 ⇒ 笔记本 scp 连不上）"
  echo
  echo "-- 车机 ping 笔记本 $lip（**这个方向才准**：笔记本的防火墙挡的是入站 ICMP，"
  echo "   但出站回包是允许的 ⇒ 笔记本 ping 不到车机可能是正常的）--"
  ping -c 2 -W 1 "$lip" 2>&1 | tail -3 | sed 's/^/   /' || true
  echo
  echo "-- 链路层：车机看到的笔记本 MAC（REACHABLE/STALE 说明二层通）--"
  ( ip neigh show "$lip" 2>/dev/null || true ) | sed 's/^/   /'
  echo
  echo "== 常见病因（按概率排序）=="
  echo "  ① 车机上没装/没开 sshd ⇒ 上面「22 端口没在听」会报出来"
  echo "     sudo apt install openssh-server && sudo systemctl enable --now ssh"
  echo "  ② 车机的 IP 不是 192.168.1.50 ⇒ 以「ip -4 addr」显示的为准"
  echo "  ③ 两边不在同一网段 ⇒ 「ip route」里没有 192.168.1.0/24"
  echo "  ④ 网线/网卡没插好、或车机网口 DOWN ⇒ ip link show 里看"
  echo "  ⑤ 笔记本有线口被 Windows 判成「未识别的网络」而没拿到 192.168.1.43 ⇒"
  echo "     Get-NetIPAddress -InterfaceAlias '以太网' 看实际拿到的 IP"
  echo
  echo "== 车机没外网 ⇒ 这几件事在车机上做不了，必须拷回笔记本 =="
  echo "   · 延迟报告（latency_report.py）· 判读（parse_session.py）· 记录表（make_record.py）· 斜率法求解（lead_solve.py）"
}

# ---------------------------------------------------------------- 全部
stage_all() {
  preflight
  stage_lat  "lat_$(date +%m%d)"
  stage_spin "spin_$(date +%m%d)"
  stage_report
}

case "${1:-}" in
  preflight) preflight ;;
  lat)      shift; stage_lat "$@" ;;
  static)   shift; stage_static "$@" ;;
  spin)     shift; stage_spin "$@" ;;
  sign)     shift; stage_sign "$@" ;;
  slope)    shift; stage_slope "$@" ;;
  report)   shift; stage_report "$@" ;;
  pull)     stage_pull ;;
  link)     shift; stage_link "$@" ;;
  all)      stage_all ;;
  -h|--help|help|"") usage ;;
  *) echo "未知子命令：$1（preflight/lat/static/spin/sign/slope/report/all）" >&2; exit 2 ;;
esac
