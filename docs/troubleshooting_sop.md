# 移动靶打不准：故障定位与排除 SOP

> 适用范围：`task_2`（小陀螺）与 `task_3`（前哨站）的命中问题。
> 版本：**2026-10-20b**（按「流水线 + 决策树」重排）。对应工作树 `main=c37d941` + 10 个已改文件 + 15 个新文件（3 个头、7 个脚本、`docs/`、`tools/onsite/`），车机侧另有 `~/run2.sh` / `~/run3.sh`（见 §4.1）。
> 一句话原则：**先分离变量，再改代码。一次只改一个东西，改完要么有数据支持，要么立刻回滚。**
> 第二原则：**命中容差是毫秒级的（5 rad/s 下 ±4.5 ms），所以别靠手调档位。**

**这份文件怎么用**

| 你现在的情况 | 从哪读起 |
|---|---|
| 刚在笔记本上改完代码（第一次上车前的第一步） | §4.0 传代码 → §4.1 编译启动 |
| 第一次上车，或换了一天的新参数 | 第二部分：§4 体检 → §5 逐阶段命令 |
| 车已经卡住了，某一枪打不出去 | §2 决策树（第一部分）→ §9 完全不开火（第三部分） |
| 只知道「打不准」，说不清卡在哪 | §3 症状索引，先分三种情况 |
| 想先看全量原因和量级换算 | §18（原因总表 + 毫秒/块宽换算） |
| 要发弹了，机会只有几次 | §3.2 → `docs/live_fire_plan.md` |
| 想离线复算一次跑车 | §12（探针 + 检测日志 + 录包） |
| 录像已经拷回来了，想一路做到打中 | **§20 从录像到命中（完整闭环，照顺序做）** |

**四部分结构**：① 一页纸（流水线 · 决策树 · 症状索引）｜② 流水线 P0–P7 逐阶段｜③ 决策树 D-1…D-5 逐节点｜④ 保障与参考（打包 · 磁盘 · 速查 · 坑 · 台账 · 原因总表 · 核查记录 · 附录）
**配套**：`docs/live_fire_plan.md`（发弹预算）｜`docs/param_rollback.md`（offset 启停/回滚）｜`docs/plots/`（曲线与 HUD 预览）

---

# 第一部分 一页纸：流水线 · 决策树 · 症状索引

## 1. 流水线 P0–P7（总表）

现场只有 20 发、3 次机会，所以整条链路的铁律是：**零发能解决的，绝不用子弹。**

| 阶段 | 干什么 | 弹药 | 耗时 | 产出 | 走不下去去哪 |
|---|---|---|---|---|---|
| **P0 体检** | `~/run2.sh`（内含 touch 5 个 `.cpp` + make + 自查） | 0 | 5 min | 新二进制 | §4.1 |
| **P1 通路** | 挡位、offset 打印、颜色 A/B | 0 | 3 min | 挡位= Auto、颜色方向 | §7.2 |
| **P2 探针** | `DSH_LAT_CSV` 跑 30 s，量视觉侧延迟 | 0 | 2 min | `logs/lat_run1.csv` | §11 B4 |
| **P3 阶跃** | `DSH_STEP_DEG=3` 跑 30 s，量云台 D 与 τ | 0 | 2 min | `logs/lat_step.csv` | §11 B4 |
| **P4 定 lead** | `py tools/lead_advisor.py logs/lat_step.csv` | 0 | 1 min | 建议 lead ± 不确定度 | §11 B3 |
| **P5 静止靶** | 3 发，验证 pitch 通道 | 3 | 5 min | 偏 / 正 | §11 B1 |
| **P6 双转速** | 低、中各 3–4 发，验 θ 是否 ∝ ω | 6–8 | 10 min | 线性关系 | §11 B2 |
| **P7 微调** | 在 P4 值附近 ≤5 ms 步长 | 6–9 | 10 min | 命中率峰值档 | §11 B3 |

**P0 之前还有一步**：只要在笔记本上改过 `.cpp` / `.hpp` / `yaml`，先按 **§4.0** 把代码传到车上，否则车跑的还是上一次的二进制。

**P0–P4 一共 0 发、13 分钟**，但它决定 P5 之后往哪走。跳过 P2–P4 直接打，等于用子弹去测一个 0 发就能测的物理量。

**为什么必须按毫秒级而不是档位级**：3 m 处 135 mm 板张角 2.58°，5 rad/s 下整块板 9 ms 就过去了 —— 打中要求 lead 准到 ±4.5 ms。而手调档位的步长是 50 ms，比容差大一个数量级，扫过去必然全空。详见 §11 B3。

**每阶段结束必须写下三行**：现象（原始日志/CSV 片段）、数字、下一步。写不出第三行就等于这阶段白跑了。

### 1.1 证据源：谁回答什么

| 工具 | 输入 | 回答 | 用不用上车 |
|---|---|---|---|
| `src/overlay.hpp`（窗口状态条） | 无，每帧都画 | 程序还活着吗、这一帧看到几块板、画了几条线 | 车上瞄一眼 |
| `src/latency_probe.hpp`（`DSH_LAT_CSV`） | 跑车时的帧 | 把链路上每个时刻写进 CSV | 车上跑一次 |
| `src/detect_log.hpp`（`DSH_DET_CSV`） | 跑车时的帧 | 每帧每块板的四个角点、编号、颜色、置信度 | 车上跑一次 |
| `tools/latency_report.py` | `lat_run1.csv` | **每一段各占多少 ms**、有没有队列积压 | 笔记本 |
| `tools/lead_advisor.py` | `lat_step.csv` | **总共该补多少 ms、补到多准才打中** | 笔记本 |
| `tools/analyze_frames.py` | `frames.csv` / 录包 | 这一场的帧率、ω、挡位占比、发弹情况 | 笔记本 |
| `tools/plot_frames.py` | `frames.csv` / 录包 | 画成曲线：四元数反解的 ω、累计转角、弹速/弹数/挡位 | 笔记本 |

前三个是跑车时顺手打开就有的；中间两个脚本读探针数据，是闭环里的一环；后两个只看历史录包，用来复盘和标定。

`detect.csv` 是唯一能让你**离线重算 EKF** 的输入（见 §12），其余都只是旁证。

---

## 2. 决策树 D-1 → D-5（总图）

**还没上车** → 直接照 §1 的 P0–P7 走，不用看这棵树。
**已经卡住**（云台不动 / 不出弹 / 打不中）→ 从这棵树进，它负责把你送到对应的分册。
**「完全不开火」**（整场一枪没请求、也没出弹）→ 直接跳 **§9**，那里先分「没目标 / 门控没过 / 请求了不出弹」三种，再按分册走。
**D-0 前置检查（30 秒，先做）**：HUD 上的 `f` / `fps` 在跳吗？不跳 = 程序没跑起来或已经冻住，先去 §4.1 / §4.2，别在这棵树上耗时间。

```
D-1  云台在追目标吗？（HUD 的 tgt 是 YES，或日志里有 fire-gate 行）
 ├─ 否 ─→ 检测 / 颜色问题 ······························· §7.2
 └─ 是 ─→ D-2  rounds 计数在涨吗？（[dbg] fire-gate 或 Plotter）
           ├─ 否 ─→ 开火门控没过 ························ §8（分不清就 §9）
           └─ 是 ─→ D-3  有弹丸飞出去吗？（慢动作视频 / 听声 / 人眼）
                     ├─ 没有 ─→ A 分支：不出弹 ········· · §10
                     └─ 有 ───→ D-4  打中了吗？（弹着点 / 靶机掉血）
                                 ├─ 没中 ─→ B 分支：出弹但偏 · §11
                                 └─ 中了 ─→ D-5  计分了吗？（裁判屏 / 规则，本文不覆盖）
```

**判断依据一览**：

| 节点 | 怎么判断 | 不需要的东西 |
|---|---|---|
| D-0 | HUD 的 `f` / `fps` 每帧都在变 | 不需要出弹 |
| D-1 | HUD `det>0` 且 `tgt=YES`；或 `grep -c '\[dbg\] fire-gate' logs/*.log` 有数 | 不需要出弹 |
| D-2 | `[dbg] fire-gate` 的 `stable/interval/hold`，或 Plotter 的 `rounds` | 不需要出弹 |
| D-3 | 手机慢动作视频、听声、看弹道 | 需要出弹一次 |
| D-4 | 靶机掉血 / 纸板弹孔 | 需要出弹一次 |
| D-5 | 裁判屏 / 规则 | — |

**先走完 D-1 到 D-5，不要跳步。** 跳过 D-3 直接调参数，是把“机构没出弹”当成“提前量不够”来治。

---

## 3. 症状索引：先确认自己在哪一格

### 3.1 三种「打不准」先分开

三种情况混在一起谈，永远谈不清。**每场先确定自己在哪一种。**

| 现象 | 判别 | 去哪 |
|---|---|---|
| **A 根本没出弹** | 打「开火」后听不到/看不到弹丸；`[dbg]` 里 `rounds` 不涨 | §10 |
| **B 出弹了但没中** | 有弹丸、靶纸上有孔但不在板上 | 本章 + §11 |
| **C 中了但不算分** | 打中了，但挡位/发数/计时不符合规则 | §17 + `docs/live_fire_plan.md` |

只有 B 才是「打不准」。A 是链路问题，C 是规则问题——**先确认 A 和 C 不成立，再谈 B**。

### 3.2 只有几次发弹机会时的顺序

> 下面说的「第 N 条」= §18.2 原因总表里的编号。

1. 看 HUD：`det` 是不是 > 0（没检测就别谈命中）
2. 静止靶 1 发：偏不偏、往哪偏（分 yaw / pitch 通道，顺带验第 6 条）
3. 有 `DSH_LEAD` 就扫 3 档（`0 / 0.05 / 0.10`），看落点是否沿旋转方向移动（验第 1、2 条）
4. 都没有改善 → 停手，转去查第 4 条（弹速）和机械（第 15 条）

# 第二部分 流水线：每个阶段怎么做

## 4. P0–P1 体检（流水线第 0 步的细节）

### 4.0 传代码：笔记本 → 车机（改过 .cpp/.hpp/yaml 就先做这一步）

> **先约定一下“在哪台机器上敲”**：```powershell``` 代码块 = 笔记本（Windows，PowerShell）；```bash``` 代码块 = 车机（Linux）；`ssh rm@192.168.1.50 "..."` 在笔记本上敲，但**引号里的命令在车上用 bash 执行**。路径同理：`D:\DSH\...` 是笔记本，`~/...` 是车机。

本地 `D:\DSH\fp_aim` 是 Windows 工作树，车上 `~/fp_aim` 是 Linux 工作树，两边**只靠 scp/tar 同步**。`git push/pull` 在这台笔记本上因 schannel 凭据问题不通（`SEC_E_NO_CREDENTIALS`），所以别指望 pull，也别在车上 `git checkout` —— 那会把现场标定值一起回滚。

**① 先看车上有没有别人改过的东西**（30 秒，别跳）

```powershell
ssh rm@192.168.1.50 'cd ~/fp_aim && git status --short && git diff --stat'
```

车上有 ` M configs/standard.yaml`（现场标定的 offset）就先备份，否则下一步的 tar 会直接盖掉它：

```powershell
scp rm@192.168.1.50:~/fp_aim/configs/standard.yaml D:\DSH\car_yaml_backup.yaml
```

**② 打一个包传过去**（15 个文件：10 个改过的 + 3 个新头 + 2 个车上要跑的脚本）

```powershell
cd D:\DSH\fp_aim
tar -cf D:\DSH\_send_code.tar configs/standard.yaml io/camera.hpp io/camera.cpp io/gimbal/gimbal.cpp io/hikrobot/hikrobot.hpp io/hikrobot/hikrobot.cpp src/task_2.cpp src/task_3.cpp src/latency_probe.hpp src/detect_log.hpp src/overlay.hpp tasks/auto_aim/armor.cpp tasks/auto_aim/target.cpp tools/onsite/collect_run.sh tools/onsite/disk_guard.sh
scp -o StrictHostKeyChecking=accept-new D:\DSH\_send_code.tar rm@192.168.1.50:~/send_code.tar
ssh rm@192.168.1.50 "cd ~/fp_aim && tar -xf ~/send_code.tar && rm ~/send_code.tar && git status --short"
```

**③ 传启动脚本 + 校验**（`run2.sh` 放家目录，不进工程树）

```powershell
scp D:\DSH\fp_aim\tools\onsite\run2.sh rm@192.168.1.50:~/run2.sh
ssh rm@192.168.1.50 "chmod +x ~/run2.sh && bash -n ~/run2.sh && echo run2-ok"
```

**④ 确认新代码真的到了**（三条都过才算成功）

```powershell
ssh rm@192.168.1.50 "cd ~/fp_aim && grep -c 'DSH_ARRIVE_DEG' src/task_2.cpp && grep -c 'detect_log' src/task_2.cpp && ls -l src/overlay.hpp"
```

期望：第一条 ≥ 5（到位门控的参数块 + 用法），第二条 = 2（include + 调用），第三个文件存在且时间是刚才。要更严就比 sha256：

```powershell
Get-FileHash D:\DSH\fp_aim\src\task_2.cpp -Algorithm SHA256 | Select-Object -ExpandProperty Hash
```
```powershell
ssh rm@192.168.1.50 'sha256sum ~/fp_aim/src/task_2.cpp'
```

**四个坑**

- **yaml 被覆盖**：tar 里带 `configs/standard.yaml`，会盖掉车上那份。先做第 ① 步的备份；只想同步代码就把 yaml 从 tar 列表里删掉。
- **换行符**：本仓库的 `.sh` 实测是纯 LF（0 个 CRLF），可以放心传；但如果用记事本另存过，Linux 会报 `bash: /usr/bin/env: bad interpreter: No such file or directory`（实际是行尾的 `^M`），用 `sed -i 's/\r$//' ~/run2.sh` 修。
- **mtime 比 `.o` 旧**：`tar -x` 保留打包时的 mtime，正常情况下比车上的 `.o` 新；`run2.sh` 里还会再 `touch` 一遍兜底。
- **别传**：`build/`、`recordings/`、`logs/`、`.git`、`docs/`、`tools/*.py`（Python 脚本在笔记本上跑，车上用不到）。

### 4.1 编译 + 启动：用 `~/run2.sh`，不要手敲

**一次性安装**（笔记本 → 车上，两条 scp + 一次 chmod；§4.0 ③ 已经传过 `run2.sh` 的话，这里只需补 `run3.sh`）：

```powershell
scp D:\DSH\fp_aim\tools\onsite\run2.sh rm@192.168.1.50:~/run2.sh
scp D:\DSH\fp_aim\tools\onsite\run3.sh rm@192.168.1.50:~/run3.sh
```

```powershell
ssh rm@192.168.1.50 'chmod +x ~/run2.sh ~/run3.sh'
```

**日常用法**：

```bash
~/run2.sh                                   # 编译 + 启动 task_2
~/run2.sh --no-build                        # 已经编过，只启动
DSH_LEAD=0.06 ~/run2.sh                     # 环境变量照常透传（DSH_* 都这么传）
DSH_RECORD_DIR=$HOME/runs/t2_$(date +%m%d_%H%M) DSH_LAT_CSV=$HOME/runs/lat.csv ~/run2.sh
~/run3.sh                                   # task_3，同形
```

**脚本替你做的四件事**（这四条就是「不能手敲、也不能用旧版」的理由）：

| 它做的事 | 不做会怎样 |
|---|---|
| touch **5 个 `.cpp`**：`src/task_2.cpp`、`tasks/auto_aim/armor.cpp`、`io/camera.cpp`、`io/hikrobot/hikrobot.cpp`、`io/gimbal/gimbal.cpp` | 只 touch `task_2.cpp` 时，共享的 `armor.cpp`（颜色）和 `io/` 下的相机代码 mtime 比 `.o` 旧，make **静默跳过**，颜色修复和设备时间戳根本没编进去 |
| `set -euo pipefail` | make 失败不停，接着启动**上一次的旧二进制**，你以为在测新代码 |
| 编译后用 `-nt` 核对 `build/task_2` 比源文件新 | 同上，只是换了一种方式被骗 |
| 启动前打印 `DSH_LEAD` / `DSH_ARRIVE_DEG` / `DSH_RECORD_DIR` / `DSH_LAT_CSV` / `DSH_DET_CSV` | 回看日志时不知道当场是怎么跑的 |

**旧版 `~/run2.sh` 如果还在用**（只 touch 一个文件、make 失败不停），最小补丁是两行：

```bash
set -e                                   # 加在 set -o pipefail 后面
touch src/task_2.cpp tasks/auto_aim/armor.cpp io/camera.cpp \
      io/hikrobot/hikrobot.cpp io/gimbal/gimbal.cpp
```

**判据（缺一不可）**：

- 出现 `Building CXX object CMakeFiles/task_2.dir/src/task_2.cpp.o`
- 出现 `Linking CXX executable task_2`
- 末尾 `OK：二进制比源文件新`

只有 `[100%] Built target task_2` = **什么都没编**，跑的还是旧二进制。

第一次用新版会重编 5 个编译单元再链接，比平时慢几分钟，别以为卡住了。

**什么时候仍然要手敲**：改了 `CMakeLists.txt` 要重新 `cmake ..`（脚本不代劳）；链接报奇怪的符号错误时才 `make clean`。改 `.cpp` / `.hpp` / `.yaml` 都不需要。

### 4.2 跑起来先看这几行

| 日志关键字 | 含义 | 异常时 |
|---|---|---|
| `[task2] offset yaw=.. pitch=..` | 偏置读到了，括号里标来源（yaml / env） | 没这行 = 配置没读到 |
| `[task2] 到位门控 = 关` / `= 开（门限 yaw .. / pitch ..）` | 到位门控状态（§11 B5） | 设了 `DSH_ARRIVE_DEG` 却没这行 = 没编进去 |
| `[lat] 延迟测量已开启，写入 …` | 探针开始写 CSV | 设了 `DSH_LAT_CSV` 却没这行 = 探针没编进去 |
| `[detlog] 检测日志 -> …` | 检测日志开始写 | 设了 `DSH_DET_CSV` 却没这行 = 没编进去 |
| `[task2] 电控弹速 0 不可用，改用配置兜底值 25` | 弹速吃兜底值 | 有这行 = 距离/弹道按 25 m/s 算 |
| `[task2] 视野里没有 red 色的装甲板…` | 颜色筛选一个都没剩 | 见 §7.2 |
| `[dbg] 电控挡位 = …` | 每 48 帧一次 | 不是 `AUTO_AIM` 就别指望出弹 |

窗口左下角还有 HUD 状态条（`OVERLAY RUNNING f=… fps …` / `det … drawn … tgt …`），判读方法见 §7.1。

### 4.3 环境

- 必须 `cd ~/fp_aim` 再运行（`assets/yolov5.xml` 是相对路径）；用 `~/run2.sh` 就不用手动切目录
- 手工编译必须在 `cd ~/fp_aim/build` 里 `make`（`~/run2.sh` 已封装）
- Plotter：`tools::Plotter` 默认发到 `127.0.0.1:9870`，PlotJuggler 选 **UDP server**
- 通道：`task2_aim/{yaw,pitch,distance,fb_yaw,fb_pitch}`、`task2_ekf/{w,abs_w,residual_yaw,residual_distance,nis,nis_failure_rate,rounds,fire}`

### 4.4 磁盘留够（做一次，30 秒）

```bash
df -h ~
bash tools/onsite/disk_guard.sh --min-free 15     # 退出码 1 = 不够，先清
bash tools/onsite/disk_guard.sh --keep 3 --yes    # 只留最新 3 场
```

盘满不会让程序崩（写图失败只 `warn` 并丢帧），但会让你录一整场却没有图，而且 `make` 会在同分区上写不出 `.o`。完整说明见 **§14**。

---

## 5. P0–P7 命令展开（每阶段可直接粘贴）

### P0 体检（0 发）

```bash
cd ~/fp_aim && touch src/task_2.cpp src/task_3.cpp tasks/auto_aim/armor.cpp \
  io/gimbal/gimbal.cpp io/camera.cpp io/hikrobot/hikrobot.cpp
cd build && make -j2
```

判据见 §4.1：必须看到 `Building CXX object .../xxx.cpp.o` + `Linking CXX executable`。

### P1 通路（0 发）

```bash
cd ~/fp_aim && ./build/task_2 configs/standard.yaml
# 另开一个终端跑 PlotJuggler（UDP server，默认 9870）
```

看四行日志（§4.2）+ 做颜色 A/B（§7.2）：

```bash
DSH_COLOR_ORDER=blue ./build/task_2 configs/standard.yaml   # 对比“视野里没有 red…”这句的多寡
```

### P2 探针：视觉侧延迟（0 发）

```bash
cd ~/fp_aim && DSH_LAT_CSV=logs/lat_run1.csv ./build/task_2 configs/standard.yaml
```

跑 30 s 即可（≈1700 行）。让云台追着靶动，不要静止不动——静止时看不出跟随滞后。

### P3 阶跃：云台 D 与 τ（0 发）

```bash
DSH_LAT_CSV=logs/lat_step.csv DSH_STEP_DEG=3 ./build/task_2 configs/standard.yaml
```

**前提：视野里有靶子且靶子静止**，否则程序交权、一个包都不发。这一步会强制不开火。

### P4 定 lead（0 发）

```bash
py tools/lead_advisor.py logs/lat_step.csv
```

输出建议 `DSH_LEAD`、不确定度 ±10 ms、以及每个转速下的容差。把它抄在纸上再上车。

### P5 静止靶（3 发）

靶子不转、距离 3 m，`DSH_LEAD=0`，打 3 发，记落点（§11 B1）。

- 上下偏 > 1 块板 → pitch 通道问题，**后面的 lead 全不用做了**，先修 §15 第一行
- 基本正中 → 进 P6

### P6 双转速（6–8 发）

低（3–5 rad/s）3–4 发、中（5–9 rad/s）3–4 发，先设 P4 给的 lead，记录落点偏移（§11 B2）。

- θ ∝ ω → 纯延迟，进 P7
- 偏移不随 ω 变 → 常数偏置，回 P5
- 同一档散布很大 → 不是系统误差，看 `frames` 有没有反复归零（§8）

### P7 微调（6–9 发）

以 P4 值为中心，±10 ms 内每 5 ms 一档，每档 3 发（§11 B3）。命中率最高的那档定稿。

### 预算账

| 阶段 | 累计用弹 |
|---|---|
| P0–P4 | 0 |
| + P5 | 3 |
| + P6 | 9–11 |
| + P7 | 15–20 |

**如果只剩一次机会**：跳过 P6/P7，直接 P4 的值 + 3 发静止靶。静止靶能中就说明 pitch 没问题，旋转靶按理论 lead 打，剩下交给运气 —— 这比"再扫四档"强，因为四档的步长根本够不着容差。

每阶段结束必须产出：一份 CSV/日志 + 一句话结论 + 一个"下一步"。

**只有几次发弹机会时，按 `docs/live_fire_plan.md` 打**：那份文档把 P5–P7 的每一发拆成了「这一发要回答哪个问题」，
规定靶纸怎么贴、每发必须留哪四样东西、什么时候必须停手；弹着点量回来喂 `tools/shot_fit.py` 直接出 `T*` 和零点。
这里的 P6/P7 是「机会充足」版，那份是「机会稀缺」版，别混用。

---

## 6. 每次动手前的三个问题

1. 我要验证的是哪一条假设？
2. 结果长什么样算证实、长什么样算证伪？（先写判据，再看数据）
3. 如果证实了，我改哪一行？如果证伪了，下个假设是什么？

答不上来就别上电。

---

# 第三部分 决策树：每个节点怎么判、怎么修

## 7. D-1 云台没在追：先看状态条，再看检测

### 7.1 先看窗口状态条

从 10-18 起，调试窗口**每帧**都画一条状态条（`src/overlay.hpp`），不管有没有目标：

```
■ OVERLAY RUNNING   f=001234   57.8 fps
  det 2   drawn 2   tgt YES   hold NO   aimline 1
```

| 字段 | 含义 | 什么时候说明有问题 |
|---|---|---|
| `f=...` | 帧号，每帧 +1 | **数字不动 = 窗口冻住或主循环停了**，这时候看什么曲线都没意义 |
| `fps` | 指数平滑的帧率 | 掉到个位数 = 链路被拖慢（相机/推理/串口） |
| **闪烁方块** | 每 10 帧换一次红/绿 | **静态画面上唯一会动的东西**；它不动就是真卡了 |
| `det` | 这一帧检测到几块板 | 数字在跳但 `det 0` = 程序活着、就是没检测到（查曝光/颜色/ROI） |
| `drawn` | 画了几个四边形 | `drawn < det` = 有点的板子关键点不是 4 个，检测层的问题 |
| `tgt` / `hold` | 有没有滤波器目标 / 是不是在保持期 | `tgt NO` 时不会开火，也不会发 Plotter 数据 |
| `aimline` | 这一帧画没画瞄准线（0/1） | 恒为 0 说明 `chosen` 一直是空的 |

画面上还会看到：每块板四个点连成的**四边形**（颜色 = 它被判成的颜色，红/蓝/灰/紫），画面中心的**准星圆**，以及从准星连到选中那块板中心的**橙色瞄准线**。

三个一眼判据：

- **数字在跳、`det 0`** → 程序没问题，是检测没出结果，别去改自瞄
- **数字不动** → 进程卡死或窗口冻结，先 `top` 看 CPU，别怀疑算法
- **`det > 0` 但 `drawn < det`** → 关键点数量不对，属于检测层，不是滤波层

### 7.2 D-1 云台根本没在追

| 检查 | 动作 | 结论 |
|---|---|---|
| 检测有没有 | 终端看 `[dbg]` 里的目标数 / Plotter 的 `distance` | 0 = 检测问题，不是自瞄问题 |
| 颜色对不对 | 看有没有 `视野里没有 red 色的装甲板` | 见下 |
| 挡位 | `[dbg] 电控挡位` | 非 `AUTO_AIM` → 电控没接管，我们发的角度会被忽略 |

**颜色映射是当前最大的不确定项。** 模型颜色头是 4 类（`tasks/auto_aim/yolos/yolov5.cpp:138` `colRange(9, 13)`），代码默认把 `color_id 0` 当红。如果反了，红靶会被判成蓝、当成友方滤掉。

一键回退（不用重编）：

```bash
DSH_COLOR_ORDER=blue ./build/task_2 configs/standard.yaml   # 0=蓝、1=红（旧行为）
./build/task_2 configs/standard.yaml                        # 0=红、1=蓝（当前默认）
```

**A/B 判据**：对着红靶机，看 `视野里没有 red 色的装甲板` 这句是变少（方向对）还是变多（方向反）。

### 7.3 Plotter 曲线：没目标就一条都没有

想看波形（`w` 抖不抖、指令和反馈差多少）时用 PlotJuggler。它经常“看起来坏了”，其实都是下面四件事之一。

**① 必须在车载电脑本机开。** `tools/plotter.cpp:15` 把目标地址写死成构造参数，而 `src/task_2.cpp` 里是 `Plotter plotter;`，缺省就是 `127.0.0.1:9870`（`tools/plotter.hpp:15`）。笔记本上开 PlotJuggler **永远收不到**，除非改代码。

**② 先开监听，再跑程序**，顺序反了前几秒的数据就丢了：

1. 车机上打开 PlotJuggler
2. 左上 **Streaming** → 插件选 **UDP Server** → 地址留 `127.0.0.1`、端口留 `9870` → **Start**
3. 另开一个终端跑 `~/run2.sh`（或 `./build/task_2 configs/standard.yaml`）

**③ 只有 `has_target` 才发数据。** `src/task_2.cpp` 的 `if (!has_target)` 分支**直接交权**：`gimbal.send(false, false, 0, 0)`，一条曲线都不画；只有 `else` 分支里才 `plotter.plot(...)`。所以没建立目标时左侧列表永远是空的。先看 HUD 的 `tgt`（§7.1），`tgt NO` 就别在 PlotJuggler 上耗时间。

**④ 变量要手动拖。** 左侧 `Timeseries List` 出现名字 ≠ 画出曲线，得把它**拖到右边空白区**。分组名按任务不同：

| 任务 | 分组 | 常用变量 |
|---|---|---|
| task_2 | `task2_aim` | `yaw` `pitch` `distance` `fb_yaw` `fb_pitch` |
| task_2 | `task2_ekf` | `w` `abs_w` `residual_yaw` `residual_distance` `nis` `nis_failure_rate` `rounds` `fire` |
| task_3 | `task3_aim` / `task3_ekf` | 同形 |
| task_1 | `aim` | `yaw` `pitch` `distance` |

**30 秒判别表**

| 现象 | 原因 |
|---|---|
| 左侧列表全空 | 没目标（③）、或 PlotJuggler 不在车机上（①）、或没点 Start（②） |
| 有 `task2_aim` 没有 `task2_ekf` | 程序在两个 `plotter.plot()` 之间崩了，看终端最后一行 |
| 曲线断断续续 | 目标在丢/重建，配合 `tgt`/`hold` 看 |
| 端口占用报错 | 先关掉别的监听（9870 是独占的，`watch_w.py` 和 PlotJuggler 不能同时开） |

**不装 PlotJuggler 的替代**：`src/task_2.cpp` 里画 `AIM`/`HOLD` 字样的那几行每帧在画面上写字（`AIM/HOLD yaw pitch dist w rounds`），截图就能当证据；离线复盘用 `py tools/plot_frames.py recordings/xxx` 画录包的 ω 曲线（§12）。

## 8. D-2 rounds 不涨：门控逐项对

看 debug 行（每 48 帧一次，`src/task_2.cpp` 的 `[dbg] fire-gate` 那一行）：

```
[dbg] fire-gate stable=False conv=True frames=15/15 nis=0.03/0.30 interval=True rounds=True mode=True hold=False | mode_now=1 bullet_alive=0 rounds 0/0
```

逐项对：

| 字段 | 什么意思 | 常见卡点 |
|---|---|---|
| `stable` | 三条同时成立：几何没跑飞 + 帧数够 + NIS 失败率低 | 拒收太多（见 §16 已知坑 7） |
| `conv` | `is_converged()`，几何收敛 | 目标在抖、或刚重建 |
| `frames=现在/要求` | 建滤波后帧数 ≥ `min_frames_for_fire`(15) | 重建后要重新等 15 帧 |
| `nis=失败率/上限` | 100 帧里的 NIS 拒收比例 < 0.30 | 拒收多了 `stable` 永远假 |
| `interval` | 距上次开火 ≥ `fire_interval_s` | 任务二应为 0.2 s（5 Hz） |
| `rounds` | 本挡位发数未超限 | 当前 `max_rounds_per_stage: 0` = 不限 |
| `mode` | 挡位要求满足 | `require_auto_aim_mode` 两个都是 `false`，所以通常恒真 |
| `hold` | 目标丢了但还在 `target_hold_ms` 保持期内 | 300 ms，旋转靶上这期间开火必空 |
| **竖线之后** | `mode_now` 电控挡位、`bullet_alive` 弹速是否可用、`rounds a/b` 本挡已打/上限 | 排查挡位和计数 |

**注意 `fire` 判据里还有一项没打印：`has_detection`**（`src/task_2.cpp` 的 `has_detection`）。它是 10-17 修幽灵开火时加的——原来漏了它，目标消失后 `stable`/`frames` 全部冻结在最后一帧的值，于是对着空气每 0.2 s 打一发。现场那次 `rounds 333` 就是 333 × 0.2 ≈ 67 s，正好一整场。所以看到 `stable=True` 但 `rounds` 不涨，先确认视野里真有板子。

---

## 9. 完全不开火：先定位卡在哪一环

「完全不开火」有三种完全不同的病，**先分清是哪一种，否则会修错地方**。

| 现象 | 卡在哪 | 去哪 |
|---|---|---|
| 云台**根本不跟**，窗口里 `det` 一直是 0，日志里**一条 `[dbg] fire-gate` 都没有** | 连目标都没建立（`has_target=false`，代码直接 `gimbal.send(false,…)` 交权） | §7.1 + §7.2（检测 / 颜色 / ROI） |
| 云台跟着动，`[dbg] fire-gate` 有，但某一项是 `False` | 门控没过 | §8 的字段表 + §9.4 |
| `fire-gate` 全是 `True`，但 `rounds` 不涨、`bullet_count` 不动 | 请求发出了，弹没出来 | §10（协议 / 机构） |

**最快的分流证据就是云台动不动**：`has_target=false` 时代码会把控制权交还给电控（`src/task_2.cpp` 的 `if (!has_target)`），云台自然不跟。**云台完全不跟 = 没有目标**，不是开火电路的问题。

### 9.1 三条命令看完

```bash
# 整场有没有建立过目标？0 = 一次都没有，直接去查检测
grep -c '\[dbg\] fire-gate' logs/2026-10-19_xx-xx-xx.log

# 最后一屏门控状态（哪些项是 False）
grep '\[dbg\] fire-gate' logs/*.log | tail -3

# 到位门控状态（只有设了 DSH_ARRIVE_DEG 才有意义）
grep '\[dbg\] arrive' logs/*.log | tail -3
grep '门控已累计挡住' logs/*.log      # 看门狗触发过没有
```

### 9.2 一组「全放开」的对照跑（3 分钟，0 发）

把这一版新加的门控全部关掉，只留最小链路：

```bash
cd ~/fp_aim
DSH_ARRIVE_DEG=0 DSH_LEAD=0 DSH_STEP_DEG=0 ~/run2.sh --no-build
```

对着**静止靶**（3 m，板正对）跑 30 秒，看：

- 能打 → 问题在门控（回到 §9.4 逐项对）
- 还是不能打 → 问题在协议/机构（§10），或根本没目标（§7）

> 为什么强调静止靶：静止靶上提前量为 0、角速度≈0，`DSH_LEAD` 和角速度估计都不参与，只考验「检测 → 收敛 → 开火」这一条主干。

### 9.3 判据：这一帧到底会不会开火

代码里 `fire` 是六个条件的与（`src/task_2.cpp` 的 `would_fire`）：

```
would_fire = has_detection && stable && interval_ok && rounds_ok && mode_ok && !holding
fire       = would_fire && arrive_ok && !step_enabled
```

`[dbg] fire-gate` 那行直接打印了 `stable / interval / rounds / mode / hold`，`has_detection` 用窗口 HUD 的 `det` 判断（`det 0` 就是没有），`arrive` 在第二行，`step` 看你有没有设 `DSH_STEP_DEG`。**六个都满足才可能出弹。**

### 9.4 「永远打不出去」的七个具体成因

| # | 成因 | 一眼证据 | 修正 |
|---|---|---|---|
| 1 | **阶跃测试忘了关**（`DSH_STEP_DEG` 设了） | `fire` 恒假；`grep '\[dbg\] fire-gate'` 全 True，但 `[lat]` 开着、yaw 指令是方波 | `unset DSH_STEP_DEG` 再跑（代码里 `:426` 明确 `&& !step_enabled`） |
| 2 | **到位门控太紧** | `arrive=1`、`errY` 长期 > `tol`、`blocked` 持续增长 | 放宽门限（静止靶可以先 1–2°），或 `DSH_ARRIVE_DEG=0` 关掉 |
| 3 | **门控的 pitch 符号约定错** | `errP ≈ 2×\|pitch\|`（约 20°），一直超门限 | 看 `[dbg] arrive` 末尾 `fb pitch` 与 `cmd pitch`：若**同号**，把 `:378-380` 的 `+` 改成 `-` |
| 4 | **颜色方向反了**（红板被判成蓝、当友方滤掉） | `视野里没有 red 色的装甲板`，HUD `det 0` | `DSH_COLOR_ORDER=blue` 跑一轮 A/B（§7.2） |
| 5 | **NIS 失败率过高** | `nis=0.50/0.30`，`stable=False`，`conv` 也常 False | §16 已知坑 7（拒收回滚污染窗口，已修；若还高，查关联/门限） |
| 6 | **挡位不是 AUTO_AIM** 且 `require_auto_aim_mode: true` | `mode=False`、`mode_now=0` | 把 yaml 的 `require_auto_aim_mode` 设成 `false`（当前就是 false），或让电控进自瞄挡位 |
| 7 | **发弹间隔设得太大** | `interval` 长期 False，十几秒才一次 | 检查 `task2.fire_interval_s`（应为 0.2 = 5 Hz） |

> 第 1 条最容易被忘：阶跃测试的方波直接叠在 yaw 指令上，为了安全代码在里面强制不开火。测完忘了 `unset`，第二天会以为是新 bug。

### 9.5 如果一切为真还是不出弹

那就是纯执行侧，按 §10 里的 A1 → A5 顺序走（挡位 → 模式字 `DSH_FIRE_MODE` → 手按开火键 → 协议 → 保底）。同时把这三样留作证据：

- 窗口 HUD 截图（`det / tgt / rounds` 都在上面）
- 该场 `logs/*.log` 里最后三行 `[dbg] fire-gate`
- `[dbg] 电控挡位 = …` 最后几次的值

---

## 10. D-3 请求了但不出弹（分支 A）

按“最便宜的先做”排序：

### A1. 挡位（30 秒）

- 让电控确认当前是 **Aim—Auto**（自瞄挡），不是手动。
- 日志 `[dbg] 电控挡位` 必须是 `AUTO_AIM`。
- 若挡位不对，我们发的角度会被电控丢弃 → 云台不动、也不出弹。

### A2. fire 模式字（2 分钟，不用重编）

`io/gimbal/gimbal.cpp:102` 读 `DSH_FIRE_MODE`，缺省 **2**（`mode`：0=不控制、1=控制不开火、2=控制且开火）。

```bash
DSH_FIRE_MODE=3 ./build/task_2 configs/standard.yaml   # 试第三种语义
DSH_FIRE_MODE=2 ./build/task_2 configs/standard.yaml   # 默认
```

判据：换成 3 之后有没有弹出来。注意 `DSH_FIRE_MODE` 是在**第一次发串口包时**读一次、之后进程内固定（`gimbal.cpp:102` 的 function-local static），所以改值**不用重编，但要重启进程**。

### A3. 机构（2 分钟）

手按开火键 / 手动开摩擦轮：

- 手按能出弹 → 机构没问题，是**协议**（回到 A1/A2）
- 手按也不出弹 → 机构/弹仓/摩擦轮，别改代码

### A4. 协议（需要电控配合，30 分钟）

让电控直接打印收到的 `mode` 字节，和我们对发一次。这是唯一能一锤定音的办法。

### A5. 保底

以上都不成立时，`fire` 恒 `false` 也要保证**云台跟随**能拿分（任务一 25 分与出弹无关）。不要为了试出弹把跟随也弄坏。

---

## 11. D-4 出弹但打偏（分支 B）

这是主战场。四段，**按 B1 → B4 → B3 → B2 的顺序做，不要跳**（编号是按主题排的，不是执行顺序）：

### B1. 静止靶单发（分离“偏置”与“延迟”）

**做法**：靶机不转，距离 3 m，打 3 发，人眼/纸板记录落点。

**判据表**：

| 落点 | 结论 | 下一步 |
|---|---|---|
| 上下偏 > 1 块板（3 m 处 13 cm ≈ 2.5°） | **pitch 通道整体错**，不是延迟 | 查 `c37d941` 的取反方向、`pitch_offset`（现在是 0.0）、`muzzle_offset_x` |
| 左右偏 | yaw 通道整体错 | 查 `yaw_offset`（现在是 0.0）、手眼标定 `R_camera2gimbal` |
| 基本正中 | 偏置没问题 → **问题在动态** | 走 B2 |

> 两个 offset 的启用/回滚见 `docs/param_rollback.md`：`DSH_YAW_OFFSET=0 DSH_PITCH_OFFSET=0` 一条命令就能回到「未启用」，不用改文件、不用重编。

静止靶是唯一能把“常数偏置”和“时间延迟”分开的实验。**这一步不做，后面全是猜。**

> 3 m 处 1° ≈ 5.2 cm；一块小装甲板 13.5 cm 宽 ≈ 2.6°。用这个换算落点偏移。

### B2. 双转速对比（线性性检验）

**做法**：低转速（3–5 rad/s）打 5 发，中转速（5–9 rad/s）打 5 发，各记平均落点偏移。

**换算**：偏移角 θ ≈ arctan(侧向偏移 / 3 m)，单位度。

**判据**：

| 观察 | 结论 |
|---|---|
| θ ∝ w（比如 4 rad/s 偏 10°，8 rad/s 偏 20°） | **纯延迟**，L = θ/w |
| θ 不随 w 变 | 常数偏置（回 B1） |
| θ 随 w 增大但增长变慢/变乱 | 估计环节（w 估不准、换板关联错） |
| 同一挡位内散布很大 | 不是系统误差，是抖动/关联/重建 |

### B3. `DSH_LEAD` 定值（先算，再微调）

**先看清容差**：3 m 处 135 mm 的板，张角只有 **2.58°**。靶机以 ω 旋转时，整块板从准星下通过的时间是：

| ω | 整块板通过 | 打中要求 lead 误差 |
|---|---|---|
| 3 rad/s | 15 ms | **±7.5 ms** |
| 5 rad/s | 9.0 ms | **±4.5 ms** |
| 7 rad/s | 6.4 ms | **±3.2 ms** |

lead 差 20 ms，在 5 rad/s 下就是 5.7°，两块多板 —— 必空。所以**不能靠手扫档位猜**：0.00 / 0.05 / 0.10 / 0.15 的步长是 50 ms，比容差大一个数量级，扫过去会**全部打空**，现场看起来就像"怎么调都不中"。这不是手法问题，是分辨率问题。

**第一步：用探针算出中心值**（B4 的零发数据）：

```bash
py tools/lead_advisor.py logs/lat_step.csv     # → DSH_LEAD 建议值、不确定度、容差表
```

它算的是 `lead = 视觉延迟 + 云台 D + 云台 τ`，并直接打印每个 ω 下"还能偏多少毫秒"。

**第二步：在已定基线附近以 ≤5 ms 步长微调**（基线 **57 ms** = 视觉 25 + 云台 32，见 §11 B5+；lead_advisor 打印的 164 ms 不可用，原因见 §12）：

```bash
DSH_LEAD=0.040 ./build/task_2 configs/standard.yaml   # 每档 3~5 发
DSH_LEAD=0.050 ./build/task_2 configs/standard.yaml
DSH_LEAD=0.057 ./build/task_2 configs/standard.yaml
DSH_LEAD=0.070 ./build/task_2 configs/standard.yaml
DSH_LEAD=0.085 ./build/task_2 configs/standard.yaml
```

注意：

- `DSH_LEAD` 补的是「板的视在角速度 × lead_s」（`src/task_2.cpp` 里按 `(x·vy − y·vx)/h²` 折算，`DSH_LEAD` 单位是秒），只在 ω 稳定时有效；换板瞬间 ω 会跳，那一段残差补不到
- 探针给出的是 ±10 ms 精度的中心值（一帧 17 ms 是分辨率下限），最后那 10 ms 只能靠打几发收
- 顶到 0.15 还越打越偏，说明方向错了（可能是 pitch 通道或 ω 估计），回 B1
- 手边没探针数据、只剩一次机会时才退回粗扫：0.06 / 0.08 / 0.10（步长 20 ms），只能定位量级，别指望扫中
- 定稿后写进配置或启动脚本，别留在环境变量里

### B4. 探针测量（不用子弹）= 流水线 P2 + P3

这套不消耗弹药，**应该先做**。两个场景各跑一次，产出两个 CSV 交给两个脚本：

```bash
# 1) 延迟与链路（P2）
DSH_LAT_CSV=logs/lat_run1.csv ./build/task_2 configs/standard.yaml

# 2) 云台阶跃（P3，会屏蔽开火，安全）
DSH_LAT_CSV=logs/lat_step.csv DSH_STEP_DEG=3 ./build/task_2 configs/standard.yaml

# 3) 出报告
py tools/latency_report.py logs/lat_run1.csv      # 逐段耗时、曝光 tick 回归、队列积压
py tools/lead_advisor.py  logs/lat_step.csv       # 直接给 DSH_LEAD（P4）
```

`latency_report.py` 回答"链路里每一段各占多少"，`lead_advisor.py` 回答"总共该补多少、补到多准"。两个都要跑，别只跑一个。

`latency_report.py` 给出：`取帧→串口写完` 的 p50/p95、曝光 tick 回归（频率 + 固定偏差 b）、yaw 跟随滞后、T90、超调。

**把它和 B3 对账**：

```
应有 DSH_LEAD ≈ (取帧→串口) + (云台 D + τ) + b
```

- 两者吻合 → 延迟就是主因，定稿 `DSH_LEAD`
- 探针说延迟很小、但 P5/P6 仍然偏 → 还有第三个误差源（标定/取反/ω 估计），回 B1
- b 无法单独分离（相机时钟差与曝光→入队延迟混在一起），要绝对值得拿手机毫秒表对镜头拍一帧标定

### B5. 到位门控：什么时候卡、怎么定门限、怎么防止它让你一枪不发

**它是什么**：开火前比较「我们刚发出去的 yaw 指令」和「电控这一帧回传的 yaw」，差值小于门限才允许放弹。
代码在 `src/task_2.cpp` 的 `arrive_ok` / `would_fire`（判定）与看门狗那一段，**默认打开，门限 0.8° / 1.3°，
且要求连续 3 帧都在门限内**（只卡单帧的话，云台扫过门限的那一瞬间也会被判到位）。

```bash
# 默认：门控 0.8° / 1.3°、连续 3 帧；一块板只打一发；发弹间隔 0.2 s（5 Hz）
./build/task_2 configs/standard.yaml

# 显式覆盖（现场调参用）
DSH_ARRIVE_DEG=0.8 DSH_ARRIVE_PITCH_DEG=1.0 ./build/task_2 configs/standard.yaml

# 打不出来时的放宽版
DSH_ARRIVE_DEG=1.5 DSH_ARRIVE_PITCH_DEG=2.0 ./build/task_2 configs/standard.yaml

# 显式给 0 才是关掉门控（0 = 关，不是「门限 0 度」）
DSH_ARRIVE_DEG=0 ./build/task_2 configs/standard.yaml

# 自动解除机制默认已关（缺省 0 = 永不解脱）。只有调试「门限是不是太紧」时才显式打开：
DSH_ARRIVE_DEG=0.8 DSH_ARRIVE_WATCHDOG_SHOTS=12 ./build/task_2 configs/standard.yaml
```

**两个轴的符号不一样，写错就恒被挡**：

| 轴 | 发指令 | 回传 | 到位判据 |
|---|---|---|---|
| yaw | `gimbal.cpp:106` **同号** | `:270` 原样 | `cmd_yaw − fb_yaw ≈ 0` |
| pitch | `:111` **取反**（任务一现场实测正确） | `:272` 原样、**没翻回来** | `cmd_pitch + fb_pitch ≈ 0` |

pitch 取反本身已经由任务一闭环（yaw 360°、pitch ±20° 的跟随实测通过），剩下的只是**回传没有跟着翻回来**，
所以比较要用「相加」。这一点是读代码推出来的，第一次上车看 `[dbg] arrive` 行末尾的 `fb/cmd` 就能当场确认：
`fb pitch ≈ −cmd pitch` 说明约定正确，`fb pitch ≈ +cmd pitch` 说明要改成相减（一行）。

**3 米处 1° 有多大**：`3000 × tan(1°) ≈ 52 mm`，接近半块小装甲板（67.5 mm）。所以门限 0.5° ≈ 26 mm 起才有意义，
1.0° 已经等于让掉半个板宽。转速越高、滞后越大，门控挡掉的帧也越多——它买的是准头，花的是发数。

**判断「这个参数有没有问题」——三步，不需要打弹**：

1. **先只看，不设**。跑一次静止靶（或手持板），每 48 帧会有一行：

   ```
   [dbg] arrive=0 tol=0.00/0.00° errY=0.42° errP=0.31° Ymin/max=0.11/0.73 n=48 blocked=0 tripped=0 | fb yaw=1.23 pitch=0.14 cmd yaw=1.65 pitch=0.02
   ```

   `errY`/`errP` 就是「云台还差多少度」。静止靶上应当很小（< 0.5°）；如果静止靶上就有 2°，那是**零位偏**（先回 §11 B1 修 offset），不是门限问题。
   末尾的 `fb`/`cmd` 顺便把 pitch 的符号约定验证掉（见上表）。

2. **看它挡住的是不是真该打的那一发**。`blocked` 只在「其它条件全都满足、就等到位」时才加。跑起来后：

   | 现象 | 判定 | 动作 |
   |---|---|---|
   | `blocked` 一直是 0 | 门控从没起过作用 | 可能转速太低、滞后本来就小 —— 不用开 |
   | `blocked` 涨、`rounds` 也涨 | 正常，只在等云台 | 保持 |
   | `blocked` 涨、`rounds` 不涨 | 门限比实际滞后还小 | 放宽门限，或等看门狗 |
   | `errP` 总是 `errY` 的几倍 | pitch 轴跟不上，或符号约定错 | 先看 `fb/cmd`，再放宽 `DSH_ARRIVE_PITCH_DEG` |
   | `tripped=1` | 看门狗已替你解除 | 看上一行告警里的 `err` 值，按它定新门限 |

3. **按实测滞后定门限**，不要拍脑袋。用同一场的 `[dbg] arrive` 行取 `errY` 的中位数 M：

   ```
   门限 ≈ 1.5 × M      （留一点余量，又不至于完全不卡）
   ```

   如果 M 已经大于 `0.5 × 板半宽对应的角度`（3 m 处 1.3°），说明**光卡到位救不了**，
   真正要补的是 `DSH_LEAD`（B3）——门控只是不让「明显没到位」的那几发浪费掉。

**看门狗（自动解除）默认已经关掉**：只有显式给 `DSH_ARRIVE_WATCHDOG_SHOTS=N`（N>0）才会启用
「累计挡掉 N 发就 `tripped=true`、本进程内永久解除门控」的旧行为。

为什么默认关：任务二每个转速挡位只有 10 发额度，累积到 12 发才解除，等于**只在第一个挡位有效**，
后面全程失效；而且它是**静默失效**，只有一行 `warn`，盯着屏幕时根本看不出来。
现在的语义反过来：不显式给正数就永不解脱，门控一旦打开就整场有效。

`blocked` 计数仍然保留 —— 它是判断「门限松不松」的唯一依据（用发数而不是时长，
是因为目标时隐时现会把计时反复清零，计数不会漏）。

想临时回到旧行为（只在调试「门限是不是太紧」时用）：

```bash
DSH_ARRIVE_DEG=0.8 DSH_ARRIVE_WATCHDOG_SHOTS=12 ./build/task_2 configs/standard.yaml
```

**开 `DSH_LEAD` 之后必须重定门限（最容易踩的坑）**：`err` 里包含「云台跟随指令的稳态滞后」。
指令是逐帧推进的，一个一阶滞后的执行机构在匀速跟踪时天然会差一个固定量（≈ `ω` × 伺服时间常数），
这个量**不会因为等待而消失**。静止靶上 `err` 可能只有 0.2°，5 rad/s 追起来可能变成 2°。
所以：

- 静止靶定的门限（比如 0.5°）**不能直接拿到旋转靶上用**，否则每一帧都被挡、一发都发不出去
- 正确的做法：先在目标转速下跑一次（不带门限），看 `[dbg] arrive` 行的 `窗口[min,max]`，
  把门限定成 **1.5 × 中位数**，它才只拦「明显没跟上」的瞬间，而不拦正常的稳态滞后
- 门限一旦接近 1×中位数，就会频繁触发；设成 5×中位数又等于没开 —— 1.5 倍是经验值，先按这个试

**上车判据**：日志里出现下面这一行，才算真的编进去了：

```
[task2] 到位门控 = 开（yaw 0.80° / pitch 1.28°，连续 3 帧）｜看门狗 关｜板面正对 ±25°、摆出 35° 才重新上膛（一块板只打一发）｜发弹间隔 0.2 s
```

### B5+. 板面朝向门控与「一块板只打一发」

**为什么还要再加一层**：到位门控只管「云台**朝指令方向**走到位了没有」，它管不了「这个指令方向本身对不对」。
旋转靶一圈四块板，板面只有转到大致正对镜头时才真正暴露给你：别的角度打出去，弹道要么擦边、
要么从两块板中间的空当穿过去 —— 观感就是「明明瞄了却没打中」，而且会一直追着同一块板连打，
一串弹全落在它转走的路上。

**判定量**（`src/task_2.cpp` 的 `facing_deg`）：板的外法线方向与「目标中心指向相机」的方向之差，0° = 正对。

| 量 | 计算 | 含义 |
|---|---|---|
| 板位置 | `center − r(cos a, sin a)`（`target.cpp` 的 `armor_xyza_list`） | `a` 是这块板在世界系里的角度 |
| 外法线 | `a + π` | 板面朝外的方向 |
| 视线方向 | `atan2(−y_center, −x_center)` | 目标中心指向相机（相机在原点） |
| `facing` | `limit_rad(a + π − 视线)` | 0° = 正对；±90° = 侧对 |

**瞄哪一块**：`aim_id` 原来直接取 EKF 关联出来的 `last_id` —— 关联只按「角度最近」，
不管这块板正面朝哪，于是会出现「追着刚转过窗口、马上要出视野的那块打」。
现在每帧在四块板里再挑一次 **`|facing|` 最小的那块**，只有明显更好（多 5° 以上）才换，
避免在临界角来回跳。换板之后 `plate_armed` 自动重新上膛，所以「一块一发」不受影响。

**两道闸**：

1. `|facing| ≤ 25°` 才允许开火（`DSH_PLATE_FACING_DEG`，四块板 90° 一块 ⇒ 正对窗口 50°）
2. **一块板只打一发**：打完这一发，同一个 `aim_id` 的板必须摆出 **35°**（`DSH_PLATE_REARM_DEG`）
   再转回来，才重新允许开火；换到别的板不受限

**lead 的符号（本轮修的第二个坑，已修）**：近侧那块板的**视在角速度**和目标**转动角速度是反号的**。
板在 `center − r(cos a, sin a)`，`a≈0` 时板的速度是 `(0, −r·w)`：板往右走（视在 yaw 变小）
对应的却是 `w > 0`，所以 `x[7] * lead_s` 直接往 `aim_yaw` 上加就正好补反了。
现在的实现直接算 `v = ω ẑ × (plate − center)`、`d(yaw)/dt = (x·vy − y·vx)/h²`，量级和符号都对，所以 `DSH_LEAD` 改回**秒**：`0.057` = 57 ms（旧的 `0.0036` 是给错公式打的补丁，作废）。

> **⚠️ 别把「准星在板左边」当成这条的证据。** 画面中心那个黄色十字是**光轴**
> （`src/overlay.hpp:75-78` 画在 `img.cols/2, img.rows/2`），它偏在板左边说明**云台指向落后于指令**，
> 也就是 `errY`，和 lead 的符号是两件事。lead 的正负只按上面的几何推，不靠看准星。

**约定自检（跑起来 1 秒就能判断）**：`facing` 依赖「`x[6]` 是板的外法线还是内法线」这个约定。
代码里有两处可以交叉验证：

- 位置模型 `h_armor_xyz = center − r(cos a, sin a)` 把板放在**近侧**；
- 量测噪声 `r_distance ∝ log(|a − bearing| + 1)`，在 `a ≈ bearing`（板正对我们）时最小。

两处都指向「`a` 是**朝里**的方向，外法线是 `a + π`」。另外 `[dbg] fire-gate` 每 0.8 秒打印的
`assoc=xx°` 是 **EKF 关联出来的那块板**的 facing（能关联上就说明它被看见了），所以它**必须**接近 0°。
**长期 ±150° 以上就是约定反了** —— 把 `facing_of` 里的 `+ M_PI` 删掉（一处，改完重编）。

发弹间隔保持赛规的 **0.2 s（5 Hz）**：防连打不靠拉长间隔，靠「一块板一发」（摆出 35° 才重新上膛）。

**自检行**（每 48 帧那行 `[dbg] fire-gate` 里多了两位）：

| 字段 | 含义 | 用法 |
|---|---|---|
| `plate=true/false` | 朝向窗口 + 是否已上膛，两个条件同时满足才为真 | 长期 false ⇒ 窗口太窄，或 `pick()` 一直挑到侧板 |
| `(N°)` | 这一帧的 `facing` 角度 | 在 ±25° 附近规律摆动 = 正常（板子在转） |

**代价**：发弹变稀（四块板转一圈最多 4 发，4 rad/s 下约 1.6 发/秒）。这是刻意的 ——
任务二每挡只有 10 发，宁可少打几发，也不要一串弹全打在空当里。

---

## 12. 离线复现：把一次跑车变成可以反复算的数据

**录的时候两个开关一起打开**（同一次跑，帧号才对得上）：

```bash
DSH_RECORD_DIR=recordings/run_1017 \
DSH_DET_CSV=logs/detect_1017.csv \
./build/task_2 configs/standard.yaml
```

| 文件 | 里面有什么 |
|---|---|
| `recordings/run_1017/frames.csv` | 云台四元数、弹速、弹数、挡位（九列） |
| `recordings/run_1017/*.jpg` | 每帧原图 |
| `logs/detect_1017.csv` | 每帧每块板：编号、颜色、置信度、四个角点、归一化中心 |

**两种用法，别混：**

1. **重跑整链**（要真结果）：
   ```bash
   DSH_REPLAY_DIR=recordings/run_1017 ./build/task_2 configs/standard.yaml
   ```
   图重新进 YOLO → 关联 → EKF，`task2_ekf/w`、残差、NIS 全部重新生成。**改任何参数都会连带改变全部输出**，适合验收，不适合归因。

2. **离线只算一层**（要归因）：
   `detect_1017.csv` 里有检测结果，可以脱离相机和 YOLO，单独拿它喂关联和滤波，反复扫 `nis_threshold`、`min_confidence`、关联门限、`Q/R`，不占车、不发弹。
   对照 `frames.csv` 里四元数反解出来的 ω（`tools/plot_frames.py`），就能判 EKF 报的 `w` 准不准——这是那 25 分的直接依据。

**对齐方式**：`detect.csv` 的 `t_sec` 以进程启动为起点，`frames.csv` 以开始录制为起点，两者差不到一帧。**用 `frame` 列对齐**，不要用时间。

**没有 matplotlib 也能画曲线**。车上往往没网、装不了扩展，而 `plot_frames.py` 依赖 matplotlib + numpy。这时用 `plot_svg.py`（纯标准库，输出 SVG，浏览器双击就开）：

```bash
python3 tools/plot_svg.py recordings/run_1017 -o ~/omega.svg
python3 tools/plot_svg.py recordings/run_1017.tar -o ~/omega.svg --panel w,count
```

它画三样：四元数反解的 `|ω|`（rad/s）、`bullet_count` 台阶、`mode` 台阶——判断「目标转不转、程序有没有认出挡位、有没有真的发弹」够用了。

**各工具的依赖（只有第一个需要扩展）**：

| 工具 | 依赖 | 能不能在裸 `python3` 上跑 |
|---|---|---|
| `plot_frames.py` | matplotlib + numpy | ❌ 需要装 |
| `plot_svg.py` | 无 | ✅ |
| `analyze_frames.py` | 无 | ✅ |
| `latency_report.py` | 无 | ✅ |
| `lead_advisor.py` | 无 | ✅ |
| `shot_fit.py` / `zero_fix.py` | 无 | ✅ |
| `runlog.py` | 无 | ✅ |
| `watch_w.py`（监听 9870） | 无 | ✅ |

所以「车上没扩展」不挡任何事：解析、拟合、报告、记录都能在车上跑，只有出 PNG 曲线这一步挪到笔记本（笔记本上有 matplotlib 3.11）。

**注意**：目前还没有直接读 `detect.csv` 的脚本（这份数据是 10-18 才加上的）。要离线扫参数，下一步得写一个 `tools/ekf_replay.py`，把 `detect.csv` 喂进 `auto_aim::Target`，这是这套东西的下一个增量。

---

# 第四部分 保障与参考

## 13. 收工：整包带走（不然下次到场啥都没了）

**先说最要命的一条**：**录包默认是关的**。`io/camera.cpp:20-26`——`record_dir` 为空、又没设 `DSH_RECORD_DIR`，就永远不 `open()`，那一场连一帧图、一行四元数都不会留下。

体积（1408×1104，q95）：

| 来源 | 单帧 | 58 fps | 每分钟 |
|---|---|---|---|
| 实测（合成截图） | 116 KB | 6.7 MB/s | **0.4 GB** |
| 真实画面（估计，噪声多） | 200–250 KB | 12–15 MB/s | **0.7–0.9 GB** |

→ 按「一场一录」走，别整场开着；`record_every: 4` 能把体积压到 1/4（代价是逐帧对齐变粗）。

**跑完立刻打包（车上，半分钟）：**

```bash
cd ~/fp_aim
bash tools/onsite/collect_run.sh low3rad            # 连图一起打
bash tools/onsite/collect_run.sh meta --meta-only   # 只打 csv/日志/配置，几 MB
bash tools/onsite/collect_run.sh big --split 2G     # 分卷，方便 U 盘
```

包里固定有这些：

| 目录/文件 | 里面是什么 |
|---|---|
| `recordings/` | 图 + `frames.csv`（回放、画曲线都靠它） |
| `logs/` | 每场的终端输出 |
| `configs/` | 参数快照，**连 `.bak` 一起**（现场值可能没提交，这里才是真相） |
| `code/` | 源码全量 + `HEAD.txt`/`status.txt`/`diff.patch`（**未提交改动全在**） |
| `system/sysinfo.txt` | 机器信息 + `DSH_*` 环境变量，能还原当时怎么跑的 |
| `MANIFEST.sha256` | 每个文件的校验和 |

脚本会先算体量，空间不够直接拒跑（`--force` 才硬来）。

**拉回笔记本（一条命令，自动校验）：**

```powershell
powershell -ExecutionPolicy Bypass -File tools\onsite\pull_run.ps1 -Name low3rad -Extract
```

scp → sha256 逐文件比对 → 解包到 `D:\DSH\onsite_runs\low3rad`。校验不过红字退出，不让你拿坏数据下结论。分卷也会自动拼。

**兜底顺序**（可靠性从高到低）：

1. scp 整包
2. `--split 2G` 分卷 → U 盘
3. `--meta-only` 只带小文件，图以后再补——**至少把 `frames.csv` 和 `diff.patch` 带出来**，这两个决定下次还能不能复现
4. 实在来不及：拍窗口截图。HUD 上有帧号和 fps，能证明程序在跑

**这条要进流程**：P0–P7 每跑完一轮、尤其是**换参数之前**，先 `collect_run.sh` 一次。参数一改，旧图就再也解释不了旧结论。

---

## 14. 磁盘：盘满不会跑崩，但会让你整场白跑

**先消除一个误解**：录像写失败**不会**把程序搞崩。`tools/recorder.cpp:89-90` 里 `cv::imwrite` 失败只打一条 `warn` 然后把这一帧丢掉；`frames.csv` 是普通 `ofstream`，写不进去也是静默失败。所以盘满的直接后果是**数据变少**，不是程序挂掉。

但同一个分区被填满，连带代价是真实的：`make` 写不出 `.o`（报 `No space left on device`，你会以为代码有错）、`logs/*.log` 写不进（出问题没有证据）、`git` 操作失败、系统整体变卡。**所以清盘要在开录之前做，不要等出事。**

### 14.1 到场第一件事（30 秒，只读）

```bash
cd ~/fp_aim
df -h ~                                        # 这条就够判断个大概
bash tools/onsite/disk_guard.sh --min-free 15  # 低于 15 GB 直接退出码 1
```

`disk_guard.sh` 会打印：分区可用、录像目录合计、每场的大小/图片数/时间（新的在前）、按 0.5 GB/分钟 估还能录多少分钟。**默认只读，什么都不删。**

### 14.2 清盘（默认演练，确认后才删）

```bash
bash tools/onsite/disk_guard.sh --keep 3            # 演练：只留最新 3 场，打印会删什么
bash tools/onsite/disk_guard.sh --keep 3 --yes      # 真删
bash tools/onsite/disk_guard.sh --older-than 7 --yes # 删 7 天前的
```

安全边界写在脚本里：只动 `recordings` 下面的**直接子目录**（路径里必须含 `recordings`，否则拒绝执行），不碰 `logs/`、`configs/`、`build/`、源码。

### 14.3 三个立刻降体积的杠杆

| 做法 | 改哪里 | 效果 | 代价 |
|---|---|---|---|
| `record_every: 1 → 4` | `configs/standard.yaml:96` | 体积 **1/4** | 回放变 14.5 fps，逐帧对齐变粗（算延迟时更明显） |
| `record_jpeg_quality: 95 → 85` | `configs/standard.yaml:97` | 体积约 **60%** | 关键点标注更糊，肉眼判读变难 |
| 不该录的场次留空 `DSH_RECORD_DIR` | 启动命令 | 完全省 | 没数据可回看 |

**建议**：录像放独立目录，别和源码/构建混在一个分区路径里 —— 比如 `export DSH_RECORD_DIR=$HOME/runs/$(date +%m%d_%H%M)`，这样 `disk_guard.sh --dir $HOME/runs` 就能整块清，也不会在 `git status` 里看到一堆图。

### 14.4 预算参考（按 0.5 GB/分钟）

| 场次类型 | 时长 | 体积 |
|---|---|---|
| 试录 10 秒（定标用） | 10 s | 0.08 GB |
| P2 探针 | 60 s | 0.5 GB |
| P3 阶跃 | 90 s | 0.75 GB |
| 一轮完整 P0–P7 | 8 min | **4 GB** |
| 一个下午（6 轮） | 50 min | **25 GB** |

→ 小电脑如果有 60 GB 空闲，**一个下午就会用掉一半**；每天收工前 `--keep 3 --yes` 一次是必须的。

---

## 15. 判据速查表

| 现象 | 最可能的原因 | 一次验证动作 | 修哪里 | 风险 |
|---|---|---|---|---|
| 静止也偏上/下 | pitch 符号或 offset | B1 | `gimbal.cpp:111` 取反、`pitch_offset` | 改符号影响 task_3 |
| 静止准、一转就偏 | 链路+云台延迟 | B3/B4 | `DSH_LEAD` | 只在 ω 稳定时有效 |
| 偏移随 w 线性 | 纯延迟 | B2 | `DSH_LEAD` | 同上 |
| 只在换板瞬间偏 | 目标关联/换板 | 看 Plotter `w` 有没有尖峰 | 关联逻辑、NIS 门限 | 改门限会连带 `stable` |
| 同一挡位散布大 | 重建/置信度抖动 | 看 `frames` 是否反复归零 | 发散判据、`min_confidence` | 0.88→0.8 会引入误检 |
| 远处偏、近处准 | 炮口视差 / 弹道补偿 | 换 5 m 再打一组 | `muzzle_offset_x`、`solve_actual_trajectory` | 弹速已定 25，不再怀疑它 |
| rounds 不涨 | 门控 | 看 `[dbg] fire-gate` | §8 | — |
| 目标消失还在打 | 幽灵开火（已修） | 手挡镜头 1 秒，看 `rounds` 是否还涨 | `fire` 判据加 `has_detection` | 已含在未提交改动里 |
| 打中了没计分 | 命中判定/规则口径 | 看靶机血量或裁判屏 | 不是代码问题 | — |

---

## 16. 已知坑（踩过的）

### 16.1 已结案：不要再在这四件事上花时间

| 事项 | 结论 | 依据 |
|---|---|---|
| 弹速真实值 | **按 25 m/s 用，不再追** | 学长答复：「代码里写了，想知道的话可以自己拿程序去读」。代码里唯一的值就是 `configs/standard.yaml:47` 的 `25.0`。17 mm 弹丸 25 m/s 本身也合理（规则上限 30） |
| `bullet_count` 恒 0 | 不关键，不查 | 电控从不上报发弹数；判命中只能靠靶机掉血/弹着点，不靠这个计数 |
| `mode` 字节语义 | 现场自测为 **Aim—Auto** | 电控口径；`DSH_FIRE_MODE` 可试 3，但默认 2 已能出弹 |
| 颜色映射方向 | 有 10 秒 A/B 兜底，不必再论证 | `DSH_COLOR_ORDER=blue` 一条命令回退，对着红靶看日志哪句变多变少即可定性 |

**重要推论**：交接文档里「实测反推弹速 19–21 m/s」**没有证据支撑**（反推所依赖的靶面像素尺寸本身未标定），且与学长口径冲突 —— 不要再把它当参数依据。弹速结案后，剩下的未知量全部是**可测的时序量**（链路延迟、云台到位时间），这是好消息。

1. **颜色映射方向未证实**。模型颜色头 4 类，代码默认 0=红。`DSH_COLOR_ORDER=blue` 可一键回退。
2. **`bullet_count` 恒 0**：电控从不上报发弹数，所以没有命中/发弹反馈。所有“打没打中”的判断目前都不可靠。
3. **弹速兜底 25 —— 已定案**：电控上报 `bullet_speed` 恒 0，代码回退到 `default_bullet_speed: 25`；学长口径「写在代码里」，故 25 即口径值，兜底与口径**恰好一致**，不存在"用了错值"。量级参考：3 m 处 25 与 20 的飞行时间差约 30 ms，在 5 rad/s 旋转靶上约等于 0.15 rad（8.6°），这部分会被 B3 扫 lead 时一起吸收。
4. **`pitch_offset` 曾经是 10.24，已归零**。10-20 现场确认：2 / 6.5 / 10 / 10.24 这些值都是「验证这两个变量有没有生效」时随手写的**探针值**，从来不是标定结果。10 月在 3.4 m 处命令云台比目标高 11.5°，弹道从靶上方 1.4 m 掠过——就是它。现在 `configs/standard.yaml:19-24` 两个 offset 都是 `0.0`，基准回归物理正确（`calc_aim` 给的本来就是世界系绝对角）。真要补偏差，用 3 发静止靶量出来再写，别凭感觉填。
5. **`c37d941` 的 pitch 取反**只凭一次观察定性，没复核过。
6. **两份 synthetic 录包不能验解算**：图是同一张截图贴进去的，云台在转、画面不动，物理上不自洽。只能验通路。
7. **NIS 拒收会污染失败率窗口**：`update()` 里先记失败、`target.cpp:310` 再还原状态，不 pop。窗口 100 帧，阈值 0.3。
8. **`target_hold_ms: 300`** 期间用的是外推值，旋转靶上 300 ms × 5 rad/s = 1.5 rad ≈ 86°，远超板缝 59°。这期间开火基本必空（`fire` 判据里的 `!holding` 就是拦这个）。
9. **Windows 终端乱码**：`chcp 65001` + `$env:PYTHONIOENCODING='utf-8'`。

---

## 17. 参数台账

改任何一个之前先查这张表，改完更新这张表。

| 参数 | 当前值 | 来源 | 可信度 | 回滚值 |
|---|---|---|---|---|
| `default_bullet_speed` | 25.0 | 代码原值 + 学长口径「以代码为准」 | 中高（口径已定，非实测） | 25.0 |
| `muzzle_offset_x` | 0.171 | 未复核 | 低 | 0.171 |
| `yaw_offset` | **0.0** | 基准归零（历史 2/0.48/0.72 都是探针值或未标定值） | 中 | `DSH_YAW_OFFSET=x` 临时覆盖 |
| `pitch_offset` | **0.0** | 基准归零；10-20 实测 10.24 会让枪口高 11.5°，弹道从靶上方 1.4 m 掠过 | 高（已由现场现象定案） | `DSH_PITCH_OFFSET=x` 临时覆盖 |
| `target_hold_ms` | 300 | 原设计 | 中 | 300 |
| `min_confidence` | 0.88 | 现场调参 | 中 | 0.8 |
| `use_roi` / `roi` | true / 304,152,800,800 | 现场调参 | 中 | false |
| `nis_reject_threshold` | 16.27 | χ²(4) 99.7% | 高 | 16.27 |
| `max_nis_failure_rate` | 0.3 | 原设计 | 中 | 0.3 |
| `min_frames_for_fire` | 15 | 原设计 | 中 | 15 |
| `task2.fire_interval_s` | 0.2 | **规则 5 Hz** | 高 | 0.05 |
| `task2.max_rounds_per_stage` | 0 | 人为放开 | **与规则 10 发冲突** | 0 |
| `task3.fire_interval_s` | 0.0834 | 规则 12 Hz | 高 | 0.0834 |
| `require_auto_aim_mode` | false ×2 | 调试期 | 低 | false |
| `DSH_LEAD` | 未设 = 0.0 | 环境变量 | — | 0.0 |
| `DSH_FIRE_MODE` | 未设 = 2 | 环境变量 | 低 | 2 |
| `DSH_ARRIVE_DEG` | 未设 = **0.8**（默认开） | 到位门控门限（°），见 §11 B5；显式给 0 才关闭 | 现场定 | 0.8 |
| `DSH_ARRIVE_PITCH_DEG` | 未设 = **1.6 × yaw**（1.3°） | pitch 单独的门限（°，云台俯仰轴更慢时可放宽） | — | 1.3 |
| `DSH_ARRIVE_FRAMES` | 未设 = **3** | 连续多少帧都在门限内才放行（防「扫过一下」） | — | 3 |
| `DSH_PLATE_FACING_DEG` | 未设 = **25** | 板面与视线夹角小于它才开火，见 §11 B5+ | — | 25 |
| `DSH_PLATE_REARM_DEG` | 未设 = **35** | 打完一发后要摆出这个角度才允许再打同一块板 | — | 35 |
| `DSH_ARRIVE_WATCHDOG_SHOTS` | 未设 = 0（永不解除） | 给正数才启用「累计挡掉 N 发后自动解除门控」 | — | 0 |

**与规则冲突的两项**（`max_rounds_per_stage: 0`）出弹数不受限，考核时会被按“前 10 发”计，超出的白打。要不要改回来需要明确决定，不要默认。

---

## 18. 打不准的全部原因（量级 · 总表 · 链路图）

**这一节是索引**：把目前已知的、能让弹丸打不中的东西全部列出来，标上量级、证据、状态。逐条的操作步骤在前面第 7–12 节。

### 18.1 量级标尺（一切都要换算到「几块板」）

3 m 处 135 mm 小装甲板：张角 **2.58°**，半宽 1.29°。板间缝 59°。旋转靶上，**时间误差比角度误差危险得多**：

| 误差 | 3 rad/s | 5 rad/s | 7 rad/s |
|---|---|---|---|
| 1 ms 的时间误差 | 0.29° = 0.11 板 | 0.29° = 0.11 板 | 0.29° = 0.11 板 |
| 10 ms | 1.7° = 0.67 板 | 2.9° = 1.1 板 | 4.0° = 1.6 板 |
| 50 ms | 8.6° = 3.3 板 | **14.3° = 5.6 板** | 20° = 7.8 板 |
| 100 ms | 17.2° = 6.7 板 | **28.6° = 11 板** | 40° = 15.5 板 |
| 打中所需的容差 | ±7.5 ms | **±4.5 ms** | ±3.2 ms |

**结论**：任何几十毫秒级别的东西，都足以让旋转靶全空；几十毫米的静态偏置（1 cm @ 2.4 m = 0.24°）连边都碰不到。**排查顺序必须按时间量排，不按「看起来像不像」。**

### 18.2 原因总表

按「对旋转靶的影响量级」从大到小排。状态栏：✅ 已改（**均未编译、未上车**）｜⚠️ 已知未修｜❓ 未验证｜✔ 已排除。

#### 一级：能单独造成全空的（几十毫秒级）

| # | 原因 | 机制 / 代码位置 | 量级（5 rad/s） | 状态 | 怎么消掉 |
|---|---|---|---|---|---|
| **1** | **提前量根本没补** | `DSH_LEAD` 缺省 0，`src/task_2.cpp` 的 `lead_s`（`std::getenv("DSH_LEAD")`）不设就是 0 | 链路 100 ms → **28.6° ≈ 11 板** | ⚠️ 未修（等测） | P2+P3 量出来，P4 用 `lead_advisor.py` 定值 |
| **2** | **云台到位时间没人管** | 角度与开火塞在**同一个串口包**：`src/task_2.cpp` 的 `gimbal.send(mode_ok, fire, ...)` `gimbal.send(mode_ok, fire, aim_yaw, ...)`；电控收到才开始转 | 几十~一百多 ms，与第 1 条同类叠加 | ✅ 已加门控（`DSH_ARRIVE_DEG=0.8`；自动解除已默认关闭，见 §11 B5） | P3 阶跃量 D 与 τ 并进 lead；要卡到位见 §11 B5 |
| **3** | **曝光时刻被换成取流时刻** | `io/hikrobot/hikrobot.cpp:211` `timestamp = steady_clock::now()`（取到帧的时刻，不是曝光时刻） | 数十 ms → 1–3 板 | ⚠️ 已加设备时间戳探针（未上车） | P2 跑 `DSH_LAT_CSV`，看 `grab_ns` vs `dev_ts` 的固定偏差 b |
| **4** | **弹速 25 m/s 是兜底值不是实测值** | 电控上报 `bullet_speed` 恒 0 ⇒ 永远走 `default_bullet_speed: 25`（`configs/standard.yaml:47`） | 若真实 21：飞行时间 120→143 ms，**差 23 ms ≈ 1.5 板@3rad/s** | ❓ 口径已定（学长「写在代码里」），非实测 | 看 `shot_fit.py` 算出的 `T*` 是否离谱地大（>120 ms） |
| **5** | **幽灵开火：没目标也在打** | 原 `fire` 判据缺 `has_detection`，目标消失后滤波器冻结、`is_converged_` 停在 true，仍按 0.2 s 一发打固定角 | 不产生偏移，但**把发数打光、把成绩打没** | ✅ 已改（`src/task_2.cpp` 的 `has_detection`、`src/task_3.cpp:272`） | 手挡镜头 >300 ms，看 `rounds` 还涨不涨 |

#### 二级：能造成明显偏移的（几度 / 十几厘米）

| # | 原因 | 机制 / 代码位置 | 量级（3 m） | 状态 | 怎么消掉 |
|---|---|---|---|---|---|
| **6** | ~~pitch 取反方向没复核~~ **已闭环** | `c37d941`「电控 pitch 正方向相反，发指令前取反」（`io/gimbal/gimbal.cpp:111`）；**任务一现场实测通过**（yaw 360°、pitch ±20° 跟得住）⇒ 取反方向正确 | 已排除 | ✔ 任务一实测 | 只剩回传 `:272` 没翻回来：比较用「相加」（§11 B5） |
| **7** | **`pitch_offset` 曾是 10.24°（探针值）** — **已归零** | 2 / 6.5 / 10 / 10.24 全是「验证变量有没有生效」时随手写的，不是标定 | 10-20 现场：命令高 11.5°，弹道从 3.4 m 靶上方 **1.4 m** 掠过 | ✔ 已定案 | 现在是 `0.0`；要补偏差用 3 发静止靶量，别凭感觉填 |
| **8** | **颜色映射方向未证实** | 模型颜色头 4 类（`tasks/auto_aim/yolos/yolov5.cpp:138` `colRange(9, 13)`），代码默认 0=红；实际顺序无从验证（仓库里没有训练工程） | 对 task_2 影响小（敌人只有一种颜色）；**对 task_3 会锁友方前哨站** | ❓ 已按 0=红 改，可一键回退 | `DSH_COLOR_ORDER=blue` 跑 10 s 对比 |
| **9** | **muzzle_offset_x = 0.171 m 未复核** | `configs/standard.yaml:49`，弹道解算直接用 | 若错 20%：0.034 m 视差 ≈ 0.65° ≈ 0.25 板 | ❓ 未复核 | 卡尺量「出弹口到 pitch 轴水平距离」 |

#### 三级：拖后腿、影响连续命中（不单独致全空）

| # | 原因 | 机制 / 代码位置 | 影响 | 状态 |
|---|---|---|---|---|
| **10** | NIS 拒收污染失败率窗口 | `tools/extended_kalman_filter.cpp:77` 先记 `recent_nis_failures`，`tasks/auto_aim/target.cpp:311` 只还原 `x/P`，不 pop | 拒收多了 `stable` 可能长期为假 → **不开火**（不是打偏，别混） | ✅ 已改：拒收帧从窗口退掉并重算失败率（`tasks/auto_aim/target.cpp:317-325`，未编译）。窗口 100 帧、阈值 0.3、统计门限 9.488 ≠ 拒收门限 16.27 |
| **11** | 发散重建后 15 帧禁火 | `min_frames_for_fire: 15`，重建后 `frames_since_init` 清零 | 连续命中率下降、换板瞬间漏打 | ⚠️ 设计如此，可调小但要担风险 |
| **12** | 大/小装甲判错 | `armor_properties` 查表，判错则距离差 1.67 倍 | PnP 距离错 → 提前量与 pitch 都错 | ❓ 未验证 |
| **13** | `pick()` 只按「离画面中心最近」 | `src/task_2.cpp` 的 `pick()`（`score = dx² + dy²`） | 多块板时锁到斜视板/装饰板 → 关联抖、滤波器重建 | ⚠️ 未改（要改先做离线验证） |
| **14** | `target_hold_ms: 300` 外推 | 丢目标后按恒定 ω 外推 300 ms | 外推期间角度是猜的 | ✅ 已由第 5 条的 `has_detection` 关掉开火 |
| **15** | 摩擦轮/装夹/云台间隙 | 机械 | 三发散布 > 半板宽就该停手查机构 | ❓ 需实测散布 |

#### 已排除 / 已结案（**别再在这上面花时间**）

| 说法 | 结论 |
|---|---|
| ✔ ROI 让 `center_norm` 偏到 (0.88, 0.69) | **假**。`armor.cpp` 先把 ROI 偏移加回关键点再算 center，`get_center_norm` 用整图宽高；ROI `304+800/2=704` 恰等于 `cx=704.31` ⇒ 中心就是 (0.5, 0.5) |
| ✔ 「弹速实测 19–21，所以兜底 25 是错的」 | **无证据**。反推所依赖的靶面像素尺寸未标定；且 25 是代码里唯一的数 = 口径值 |
| ✔ `bullet_count` 恒 0 导致没反馈 | 真，但**不是打不准的原因**；它只是让我们无法自动判命中 |
| ✔ `mode` 语义不清（0/1/2/3） | 已实测：自瞄挡位上报 `Aim—Auto` |
| ✔ 弹道模型没算重力/阻力 | **假**。`src/task_common.hpp:89` `solve_actual_trajectory(..., mode 3, ...)` 已含阻力，迭代 2 次 |
| ✔ 颜色反了导致 task_2 锁错板 | 对 task_2 影响很小（同色）；真正的风险在 task_3 |

### 18.3 一张图：误差从哪来

```
目标真实位置
   ↓ ① 曝光时刻（我们以为的 t0 其实是取流时刻，差 10–50 ms）        ← 第 3 条
   ↓ ② 推理+滤波+解算（10–20 ms）                                  ← P2 量
   ↓ ③ 串口发出 → 电控收 → 云台转到位（几十~上百 ms）               ← 第 2 条
   ↓ ④ 弹丸飞行（3 m / 25 m/s = 120 ms）                            ← 第 4 条
弹丸命中点
       而代码只补了 ④ 的提前量，①②③ 全靠 DSH_LEAD 手动补（缺省 0）  ← 第 1 条
```

**这就是全部问题的骨架**：链路延迟（①②③）在旋转靶上是单片最大误差源，而它今天完全没补。

## 19. 逐条代码核查记录（结论：原因总表 15 条里只有 2 条是代码缺陷）

这一节是拿源码一行一行对出来的，不是推测。**§18.2 的 15 条里，真正属于「代码写错了」的只有 2 条**，
其余是配置值、缺失功能、或者根本还没人量过的未知量。分清楚这三类，才不会把时间花在改代码上。

| # | 条目 | 代码证据 | 判定 |
|---|---|---|---|
| 1 | `DSH_LEAD` 缺省 0 | `src/task_2.cpp` 的 `lead_s` 读环境变量、缺省 `0.0`；`if (lead_s != 0.0)` 里按板的视在角速度折算后才加进 `aim_yaw` | 代码事实成立，但**该补多少是未知量**，不是 bug |
| 2 | 没有云台到位检查 | `src/task_2.cpp` 的 `gimbal.send(mode_ok, fire, ...)`：角度与 `fire` 走同一个包；全文没有一处拿 `fb_yaw/fb_pitch` 做门控 | **缺失功能**（可加，但 pitch 符号未知，只能先卡 yaw） |
| 3 | 曝光时刻被换成取流时刻 | `io/hikrobot/hikrobot.cpp:211` `steady_clock::now()`；探针已记录 `dev_ts`，但 `src/task_2.cpp` 的 `target.predict(t)` 对齐姿态仍用 `t` | 代码事实成立，**尚未启用修正**（属 P2/P3 的活） |
| 4 | 弹速 25 是兜底值 | `configs/standard.yaml:47`；`src/task_2.cpp` 的 `bullet_speed` 兜底（电控 <5 时用） | 配置值，不是 bug（口径已定 25） |
| 5 | 没目标也请求开火 | 原 `fire` 判据缺 `has_detection` | **代码缺陷，已修**（`src/task_2.cpp` 的 `has_detection`） |
| 6 | pitch 取反没复核 | `io/gimbal/gimbal.cpp:111` `tx_data_.pitch = -pitch;` | 代码在，但**符号对不对静态看不出来**，只能静止靶试射 |
| 7 | `pitch_offset` 曾 10.24 | `configs/standard.yaml:19-24`；`src/task_2.cpp` 的 `pitch_offset_eff` | 探针值，非标定；10-20 已归零，现场弹道从上方掠过即其证据 |
| 8 | 颜色方向未证实 | `tasks/auto_aim/armor.cpp:25-45` `to_color()` + `DSH_COLOR_ORDER` | 已参数化，**方向仍未知**，回放里 A/B 一次即可 |
| 9 | `muzzle_offset_x = 0.171` 未复核 | `configs/standard.yaml:49`；`src/task_common.hpp:89` 只用于弹道水平距离 | 配置/实测值，不影响 yaw |
| 10 | NIS 拒收污染失败率窗口 | 原 `tools/extended_kalman_filter.cpp:77` 无条件 `push_back`，`tasks/auto_aim/target.cpp:310` 回滚状态却不退窗口 | **代码缺陷，已修**（`target.cpp:310-325` 现在会把这一帧的 1 退掉） |
| 11 | 重建后 15 帧禁火 | `configs/standard.yaml` `min_frames_for_fire: 15`；`src/task_2.cpp` 的 `min_frames_for_fire` | 设计选择，58 帧每秒 ≈ 0.26 秒 |
| 12 | 大/小装甲判错 | `tasks/auto_aim/armor.cpp:204`、`:243` `type = num_id == 1 ? big : small` | **启发式**：把 135 当 225 距离差 1.67×，编号读错就中招 |
| 13 | `pick()` 只挑离画面中心最近的 | `src/task_2.cpp` 的 `pick()`，`score = dx² + dy²` | 设计选择，没有粘滞和边缘剔除 |
| 14 | `target_hold_ms: 300` 外推 | `configs/standard.yaml`；`src/task_2.cpp` 的 `target_hold_ms` | 配置值，300 毫秒在 5 rad/s 下外推 86° |
| 15 | 摩擦轮/装夹散布 | —— | 不可测，只能靠靶纸散布判断 |

**核查时顺手发现的另外三件事**：

- `use_traditional: true` 和 `threshold: 100` 是**死键**：全仓 `*.cpp/*.hpp` grep 不到任何读取它们的代码。
  别以为现在跑的是传统视觉，跑的就是 `yolov5`。
- NIS 的**统计阈值 9.488**（`tools/extended_kalman_filter.hpp:50`）和**拒收阈值 16.27**
  （`configs/standard.yaml` 的 `nis_reject_threshold`）不是同一个数。意思是：被接收的帧里
  仍有约 5% 会记成「失败」。正常时失败率在 0.05 附近，远低于门限 0.3，不影响开火。
- `Gimbal::q(t)` 不是零阶保持，而是拿前后两包姿态做 **slerp 插值**（`io/gimbal/gimbal.cpp:162-174`），
  所以「姿态对齐」这一环比预想的好，第 3 条的误差只剩曝光→取流那一段。

**结论**：想靠改代码把命中率改上去，能改的只有第 2 条（到位门控）。
第 1、3、6、8 条必须靠测量或试射给数，第 4、7、9、11、14 条是参数，第 12、13 条是模型/策略，
第 15 条在机械上。**先把数拿到，再谈改代码。**

---

## 20. 从录像到命中：完整闭环

前面 19 节是「按症状查」，这一节是「照顺序做一遍」。一次闭环只要 40 分钟：
**留证据 → 拷回来 → 算四个数 → 每发落点变成一行 → 改一处 → 打 3 发验证 → 记账**。

| 步 | 在哪 | 产出 | 判据 |
|---|---|---|---|
| 20.1 录 | 车机 | 日志 + lat CSV | 帧率 ≥55，bullet_count 涨 |
| 20.2 拷 | 笔记本 | `D:\DSH\onsite_runs\<NAME>\` | sha256 一致 |
| 20.3 算 | 笔记本 | 帧率 / 视觉 p50 / 云台滞后 / 真发弹数 | 见 20.3 表 |
| 20.4 落点 | 笔记本 | `shots.csv` + `T*` | 几发 `T*` 离散 < 10 ms |
| 20.5 改 | 笔记本 → 车机 | 重编的二进制 | `Linking CXX executable` |
| 20.6 验 | 车机 | 3 发落点 | 落在板内 = 收工 |
| 20.7 记 | 笔记本 | `docs/runs/<NAME>.json` | `show` 能横向比 |

### 20.1 录之前：让这次数据可用（车上，1 分钟）

```bash
cd ~/fp_aim
unset DSH_RECORD_DIR                       # 录像会把帧率从 58 压到 44.7，调参数时一律关
DSH_LAT_CSV=logs/lat_$(date +%m%d_%H%M).csv ~/run2.sh 2>&1 | tee logs/t2_$(date +%m%d_%H%M).log
```

三件事必须同时成立，缺一个这次数据就白跑：

1. `run2.sh` 第二行回显 `DSH_LEAD=...`（`未设=0` 就说明参数根本没进去）；
2. 日志里有 `[dbg] 电控挡位 = AUTO_AIM(自瞄)`；出现 `IDLE(手动/未接管)` 说明电控没接管，那场不算数；
3. 打 3 发，`rounds` 数字要涨（`[dbg] fire-gate ... rounds 3/0`）。

### 20.2 拷回来（笔记本 PowerShell）

先看车上有什么：

```powershell
ssh rm@192.168.1.50 "ls -lt ~/fp_aim/recordings | head -5; du -sh ~/fp_aim/recordings/*"
```

打包单场 + 算 sha（在车上做，别整目录打）：

```powershell
ssh rm@192.168.1.50 "cd ~/fp_aim/recordings && tar -cf ~/packs/<NAME>.tar <NAME> && sha256sum ~/packs/<NAME>.tar"
```

拉回来并校验：

```powershell
scp rm@192.168.1.50:~/packs/<NAME>.tar D:\DSH\onsite_runs\
Get-FileHash D:\DSH\onsite_runs\<NAME>.tar -Algorithm SHA256
```

两边的 sha256 必须一致；不一致就是传坏了，重传，不要拿坏包算数。
一键脚本等价于上面三步：`powershell -ExecutionPolicy Bypass -File tools\onsite\pull_run.ps1 -Name <NAME> -Extract`。

**没有网络时用 U 盘**（在车上）：

```bash
lsblk                                      # 找到 U 盘，例如 /dev/sdb1
sudo mkdir -p /mnt/usb && sudo mount /dev/sdb1 /mnt/usb
sudo cp /tmp/<NAME>.tar /mnt/usb/ && sync && sudo umount /mnt/usb
```

**只想先看数就只拉小文件**（几 MB，几十秒）：`frames.csv` + `logs/*.csv` + `logs/*.log`。

### 20.3 先算四个数（笔记本，5 分钟，不需要装任何包）

```powershell
cd D:\DSH\fp_aim
py tools\analyze_frames.py D:\DSH\onsite_runs\<NAME>
py tools\latency_report.py D:\DSH\onsite_runs\<NAME>\logs\lat_xxx.csv
py tools\plot_svg.py D:\DSH\onsite_runs\<NAME>      # 没有 matplotlib 也能出曲线
```

| 数 | 从哪看 | 合格线 | 不合格去哪 |
|---|---|---|---|
| 帧率 | `analyze_frames` 的 dt 分布 / `[Recorder] 频率` | ≥55 fps | 录像没关，回 20.1 |
| 视觉延迟 | `latency_report` 的「取帧→解算 p50」 | ≤30 ms | 降曝光、缩 ROI（§7） |
| 云台滞后 | `latency_report` 的 yaw 互相关 | 20–40 ms | 查 `DSH_STEP_DEG` 实验是否有效（§5 P3） |
| 真发弹数 | `bullet_count` 增量 | = 你按的发数 | 不涨 → 门控没过（§9） |

**顺手看一下云台跟得上没有**（这一步最容易被跳过）：

```bash
grep -h '\[dbg\] arrive' logs/*.log | tail -20
# [dbg] arrive=false tol=0.50/2.00° errY=1.51° errP=0.07° Ymin/max=0.00/6.31 n=48 ...
```

`errY` 是「指令角 − 反馈角」，单位是**世界系角度**，和落点偏移是 1:1：3.15 m 处 1° = 55 mm ≈ 0.4 块板。
开火那一瞬间 `errY` 有多大，弹就偏多少度。中位数超过 2° 就先查云台（§7），
别用 lead 硬补 —— lead 补的是「板在这段时间里走了多远」（55 ms 只值 0.8°），两者不是一回事。

### 20.4 每一发落点变成一行 `shots.csv`

一发的三样东西：**开火时刻的 ω、当时的 lead、弹着点偏移**。

- ω：`analyze_frames` 打出的 |ω| 分布，或 `[dbg] fire-gate` 那一行的 `w`；
- 落点：视频里量像素，用板宽 135 mm 当标尺换算成 mm；
- 符号：**dx 正 = 落后（落在靶子运动的反方向）**，dy 正 = 偏上。

```csv
shot,d_m,omega,lead_ms,dx_mm,dy_mm
1,3.15,0.25,0,86,12
2,3.15,0.25,0,94,8
3,3.15,0.25,0,79,15
```

> **`omega` 填「视在角速度」`r·|w|/d`，不是 EKF 的 `w`。**
> 板绕中心转，相机看到的是它在横向上的投影速度除以距离：`r=0.2 m`、`w=4 rad/s`、`d=3.15 m`
> ⇒ 视在角速度 `0.25 rad/s`。直接填 4 会把 `T*` 算大约 16 倍。

```powershell
py tools\shot_fit.py shots.csv --markdown
```

输出里的 `T*` 就是「这一批弹真正需要的 lead」。判据：**几发解出的 `T*` 离散 < 10 ms** 才可以采信；
离散很大说明落点量不准、或期间 ω 变了，重打一批再来。

### 20.5 把 `T*` 落到代码（✅ 已落到代码，2026-10-20）

当前实现把「目标自转角速度」当成「相机看到的角速度」直接乘，**放大了 `d/r ≈ 16` 倍**：

```cpp
// src/task_2.cpp 的 aim_yaw += x[7] * lead_s（旧）—— lead=55 ms 时实际把准星推了 12.6°，约 5 块板
aim_yaw += static_cast<float>(x[7] * lead_s + yaw_offset_eff * 0.017453292519943295);
```

改成用「板的速度向量」，物理上才对（`plate` 与 `x` 在这一行都已在作用域内）。**这段现在就是源码：**

```cpp
// src/task_2.cpp 的 aim_yaw +=（新）—— 板的速度 v = ω ẑ × r，再按 d(yaw)/dt = (x·vy − y·vx)/h² 折算
if (lead_s != 0.0) {
  const double rx = plate[0] - x[0];
  const double ry = plate[1] - x[2];
  const double h  = std::hypot(plate[0], plate[1]);
  if (h > 0.5) {
    const double vx_plate = -x[7] * ry;
    const double vy_plate =  x[7] * rx;
    aim_yaw += static_cast<float>((plate[0] * vy_plate - plate[1] * vx_plate) / (h * h) * lead_s);
  }
}
aim_yaw += static_cast<float>(yaw_offset_eff * 0.017453292519943295);
```

修完的量级感（心里有个数，不然会以为改坏了）：3.15 m、4 rad/s、`DSH_LEAD=0.055` ⇒ 只补 **0.8°**（约 0.33 块板）。
补偿本来就该这么小；之前 12.6° 那个数一眼就是错的。

改完编译 + 传车：

```powershell
ssh rm@192.168.1.50 "cd ~/fp_aim && touch src/task_2.cpp && cd build && make -j2"
scp D:\DSH\fp_aim\src\task_2.cpp rm@192.168.1.50:~/fp_aim/src/task_2.cpp
```

（顺序反过来也行，但**先传再编**更省事：传完在车上 `touch` + `make`。）

### 20.6 上车验证：只改一个数，打 3 发

```bash
cd ~/fp_aim
unset DSH_RECORD_DIR            # 确保不录包（录像把帧率从 58 压到 45，时间基准全乱）
DSH_LEAD=0.057 ~/run2.sh --no-build 2>&1 | tee logs/t2_verify.log
```

判据：

- 3 发全部落在板内 → **收工**，把这一场的参数写进 yaml；
- 全部偏同一侧、量级约 1 块板 → 用 `zero_fix.py` 算 offset，不要再用 lead 硬补：
  `py tools\zero_fix.py --d 3.15 --dx 40 --dy 10 --yaw-offset 0 --pitch-offset 0`；
- 落点随转速变、静止靶却正中 → 还是 lead 的问题，回 20.4 重算 `T*`；
- 静止靶都不中 → 不是 lead 的事，回 §7/§9 查通路。

### 20.7 记账（1 分钟，省的是下一次的时间）

```powershell
py tools\runlog.py new t2_lead055 --task task2 --stage P4
py tools\runlog.py set t2_lead055 d_m=3.15 w=4.0 lead_ms=55 shots=3 hits=2 verdict="偏右半块"
py tools\runlog.py lat t2_lead055 D:\DSH\onsite_runs\<NAME>\logs\lat_xxx.csv
py tools\runlog.py show
```

`show` 出来是一张按场次横排的表，下一次调参前先看一眼，能省掉重复试。

### 20.8 卡住时去哪

| 卡在 | 去哪 |
|---|---|
| 拷不回来 / sha 不一致 | §13、§14（磁盘与打包） |
| `bullet_count` 不涨 | §9（完全不开火，三分类） |
| 有弹但落点乱 | §20.4 重算 `T*`；`T*` 离散大就查录像帧率 |
| 云台不跟 / 跟得慢 | §7（D-1）、§5 P3（阶跃） |
| 弹速不对 | §18.2 第 3 条；`default_bullet_speed` 按实测改 |
| 参数不知道填多少 | 附录 B（环境变量）、§17（台账） |

---

## 附录 A：命令速查

```powershell
# ① 传代码（笔记本，工程根；完整说明见 §4.0）
cd D:\DSH\fp_aim
ssh rm@192.168.1.50 'cd ~/fp_aim && git status --short'     # 先看车上有没有现场改动
scp rm@192.168.1.50:~/fp_aim/configs/standard.yaml D:\DSH\car_yaml_backup.yaml   # 有改动就先备份 yaml
tar -cf D:\DSH\_send_code.tar configs/standard.yaml io/camera.hpp io/camera.cpp io/gimbal/gimbal.cpp io/hikrobot/hikrobot.hpp io/hikrobot/hikrobot.cpp src/task_2.cpp src/task_3.cpp src/latency_probe.hpp src/detect_log.hpp src/overlay.hpp tasks/auto_aim/armor.cpp tasks/auto_aim/target.cpp tools/onsite/collect_run.sh tools/onsite/disk_guard.sh
scp D:\DSH\_send_code.tar rm@192.168.1.50:~/send_code.tar
ssh rm@192.168.1.50 "cd ~/fp_aim && tar -xf ~/send_code.tar && rm ~/send_code.tar"
ssh rm@192.168.1.50 "cd ~/fp_aim && grep -c DSH_ARRIVE_DEG src/task_2.cpp && grep -c detect_log src/task_2.cpp"
```

```bash
# 编译 + 启动（车机，推荐）：脚本会 touch 5 个 .cpp、make 失败即停、用 -nt 自查
~/run2.sh                                            # task_2
DSH_LEAD=0.06 ~/run2.sh                              # 带环境变量
DSH_RECORD_DIR=$HOME/runs/t2_$(date +%m%d_%H%M) DSH_LAT_CSV=$HOME/runs/lat.csv ~/run2.sh
~/run2.sh --no-build                                 # 已经编过，只启动
~/run3.sh                                            # task_3

# 安装/更新这两个脚本（笔记本上）
# scp D:\DSH\fp_aim\tools\onsite\run2.sh rm@192.168.1.50:~/run2.sh
# scp D:\DSH\fp_aim\tools\onsite\run3.sh rm@192.168.1.50:~/run3.sh
# ssh rm@192.168.1.50 'chmod +x ~/run2.sh ~/run3.sh'

# 手工等价（只在脚本不可用时用；6 个文件都要 touch，漏一个就可能编不到）
cd ~/fp_aim && touch src/task_2.cpp src/task_3.cpp tasks/auto_aim/armor.cpp \
  io/gimbal/gimbal.cpp io/camera.cpp io/hikrobot/hikrobot.cpp
cd build && make -j2

# 跑车（必须在工程根）
cd ~/fp_aim && ./build/task_2 configs/standard.yaml

# 回放录包
DSH_REPLAY_DIR=recordings/synthetic_rotate ./build/task_2 configs/standard.yaml

# 录包
DSH_RECORD_DIR=recordings/run_1017 ./build/task_2 configs/standard.yaml

# 录包 + 检测日志（按这个组合录，离线才能重算 EKF）
DSH_RECORD_DIR=recordings/run_1017 DSH_DET_CSV=logs/detect_1017.csv ./build/task_2 configs/standard.yaml

# 探针
DSH_LAT_CSV=logs/lat_run1.csv ./build/task_2 configs/standard.yaml

# 报告（笔记本上用 py，车机上换成 python3；都只需要标准库）
py tools/latency_report.py logs/lat_run1.csv
py tools/lead_advisor.py logs/lat_step.csv
py tools/analyze_frames.py recordings/run_1017 --markdown
py tools/plot_frames.py recordings/run_1017 --out-dir docs/plots
python3 tools/plot_svg.py recordings/run_1017 -o omega.svg   # 车上没 matplotlib 时用这个（纯标准库）

# 发弹机会有限时：弹着点 → T* / 零点（见 docs/live_fire_plan.md）
py tools/shot_fit.py shots.csv --lateral-zero-mm 15

# 记录（见附录 C）：一条命令存一场，最后 show 出对比表
py tools/runlog.py new run1_1020 --task task2 --stage P5
py tools/runlog.py lat run1_1020 logs/lat_1612.csv
py tools/runlog.py set run1_1020 D_ms=17.1 tau_ms=19.8 lead_ms=72
py tools/runlog.py show

# 收工打包（车上）—— 每轮跑完都要做，见 §13
bash tools/onsite/collect_run.sh low3rad

# 磁盘（见 §14）：先看，再清，默认演练、加 --yes 才删
bash tools/onsite/disk_guard.sh
bash tools/onsite/disk_guard.sh --min-free 15
bash tools/onsite/disk_guard.sh --keep 3 --yes

# 拉回笔记本：拷贝 + sha256 校验 + 解包
powershell -ExecutionPolicy Bypass -File tools\onsite\pull_run.ps1 -Name low3rad -Extract
```

## 附录 B：环境变量一览

| 变量 | 作用 | 缺省 |
|---|---|---|
| `DSH_LEAD` | 旋转靶提前量（秒），`aim_yaw += ω·lead` | 0.0 |
| `DSH_YAW_OFFSET` | 覆盖 yaml 的 `yaw_offset`（度）；设 0 = 未启用 | 用 yaml |
| `DSH_PITCH_OFFSET` | 覆盖 yaml 的 `pitch_offset`（度）；设 0 = 未启用 | 用 yaml |
| `DSH_FIRE_MODE` | 串口 `mode` 的开火取值 | 2 |
| `DSH_COLOR_ORDER` | `blue` = 恢复旧的颜色映射 | 0=红 |
| `DSH_LAT_CSV` | 打开延迟探针，写 CSV | 关闭 |
| `DSH_DET_CSV` | 打开检测日志，每帧每块板一行 | 关闭 |
| `DSH_STEP_DEG` | 云台阶跃测试幅度（度，≤15），会屏蔽开火 | 关闭 |
| `DSH_ARRIVE_DEG` | 到位门控门限（度）：指令 yaw 与回传 yaw 差小于它才放弹 | 0.8（默认开；给 0 才关） |
| `DSH_ARRIVE_FRAMES` | 连续多少帧都在门限内才放行 | 3 |
| `DSH_PLATE_FACING_DEG` | 板面与视线夹角小于它才开火（板面朝向门控） | 25 |
| `DSH_PLATE_REARM_DEG` | 打完一发后要摆出这个角度才允许再打同一块板 | 35 |
| `DSH_ARRIVE_WATCHDOG_SHOTS` | 给正数才启用「累计挡掉 N 发就自动解除门控」；缺省 `0` = 永不解除 | 0 |
| `DSH_ARRIVE_PITCH_DEG` | pitch 的到位门限（度）；不设则 1.6 × `DSH_ARRIVE_DEG` | 0.8 |
| `DSH_RECORD_DIR` | 录包输出目录 | 关闭 |
| `DSH_REPLAY_DIR` | 从录包回放 | 关闭 |

## 附录 C：一次实验的记录模板

完整版见 **`docs/onsite_record.md`**（一页纸 + P0–P7 每步记什么 + 填好的样例）。机器可读的那份用：

```bash
py tools/runlog.py new run1_1020 --task task2 --stage P5
py tools/runlog.py lat run1_1020 logs/lat_1612.csv      # 自动填延迟/fps
py tools/runlog.py set run1_1020 D_ms=17.1 tau_ms=19.8 shots=3 hits=2 dx_mm=12 dy_mm=-8 dist_m=3.4 verdict=PARTIAL
py tools/runlog.py show                                  # 所有场次对比
```

存 `docs/runs/<名字>.json`，一场一个文件；`export` 汇总成 `docs/runs/runs.csv`。

下面是最小格式，写不出“结论”和“下一步”两行，说明这一轮白打了：

```
日期/时间：
commit / 工作树改动：
改了哪些参数（相对上一轮）：
环境变量：
包名（collect_run.sh 的 RUN，包在哪）：
场地：距离 ___ m，转速 ___ rad/s（怎么测的：）
弹药：___ 发
现象（原始日志片段）：
数字（落点偏移/命中数/延迟读数）：
结论：
下一步：
```
