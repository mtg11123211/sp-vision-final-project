# 交接包：fp_aim_fkey（自瞄大作业 task2）修复任务上下文

> 用途：交给 **GLM5.3** 指导 **DS4.1** 继续完成 fp_aim_fkey 的修复与上车。
> 本包是**自包含**的：不依赖此前任何对话，读完就能动手。
> 材料来源：minimax 完整会话记录（574 条消息，10-16 ~ 10-22）、DS4.1 自检报告、四审复核结论、基线 `logs/` 实测数据复算。

---

## 0. 三十秒版

| 项 | 内容 |
|---|---|
| 目标 | 10-25 前完成大作业 **task2（旋转装甲板自瞄）**：3 m、单发、≤5 Hz、低挡 3–5 / 中挡 5–9 rad/s、每挡 10 发；**命中 20 分 + 角速度估计 20 分** |
| 代码在哪 | 副本 `D:\DSH\fp_aim_fkey` ≡ `D:\MTG\Documents\OC\fp_aim_fkey`（两份 src 逐字节一致）；基线 `D:\DSH\fp_aim` |
| 红线 | **严禁改基线** `D:\DSH\fp_aim`（用户铁律，minimax 全程遵守）；副本可自由改 |
| 当前状态 | 副本 = 基线 + 9 个新文件 + 3 个改动文件 + 10 份文档；**从未在车机编译过、从未发过一发** |
| 头号问题 | `tools/lead_solve.py:170` **符号反了**（四审实跑抓到）⇒ M4 标定必然失败 |
| 其它必修 | task_3.cpp:358 编译错；fire_key 暖启动 600 ms；每挡 10 发无程序兜底；中挡 9 rad/s 无验证场 |
| 硬约束 | **发弹机会有限**（三任务共享）；车机无外网无 python3、只能编译和跑；编译只在车机做 |
| 建议分工 | GLM5.3 规划+复核，DS4.1 执行+自验；**每处改动必须编译/实跑验证**（这是本次翻车的根因） |

---

## 1. 工程布局与红线

```
D:\DSH\fp_aim            基线（DSH 本人交付版，task_2 619 行，git 有版本）   ← 禁改
D:\MTG\Documents\OC\fp_aim_fkey   副本（★权威工作副本，见下）              ← 可改
D:\DSH\fp_aim_fkey       同一副本的旧快照（docs 停在 10-21 23:12）          ← 别改
D:\DSH\final_project_aim 官方初始代码（sp_vision_tutorial_27 final_project_aim）
```

**★权威副本说明（10-22 13:48 实测）**：两份副本的 `src/` 与 `tools/` **完全一致**，但 `docs/debug_flow.md` 与 `docs/记录表.md` 不同——`D:\MTG\Documents\OC\fp_aim_fkey` 那份更新（13:48 做过一次 Markdown 格式化，内容等价、纯排版）。
⇒ **所有改动只做在 `D:\MTG\Documents\OC\fp_aim_fkey`**，不要再往 `D:\DSH\fp_aim_fkey` 同步（会重新分叉）。
⇒ 副作用提醒：格式化把 `0.06~0.1 s` 写成了 `0.06~~0.1 s`（Markdown 会当删除线渲染），顺手改回。

**用户铁律（原始表述转述）**：
- 10-20 21:13「**严禁修改原本目录中的代码，只允许复制一份到你的工作区来改**」；
- 10-20 22:23「**如果是你单开的副本，允许你自由修改**」——仅指副本；
- minimax 的执行方式：每轮报「原工程仍零改动」+ 哈希自证（`task_2.cpp` 81A2DD18… / `standard.yaml` 8C9B1E2C… / `debug_flow_v2.md` 57486E3F…）。**这套纪律要继续沿用**。

**必须用 git 规范管理代码**（飞书原文：「请大家规范使用github管理自己的代码，以便我们后续查验」）。

---

## 2. 任务要求（飞书口径）

飞书文档 `tj-superpower.feishu.cn/docx/D6TgdyAshoEMmgxZNnCcibM5nnc` 匿名只能读到任务一，task2 细则按基线 `docs/fix_log.md` §9 的转述口径执行：

| 项 | 要求 |
|---|---|
| 提交期 | 10-12 ~ **10-25**；地点嘉定校区开物馆地下车库；每日开放时段有限（**14:00–17:00** 段是实时的） |
| 提交次数 | **每个任务仅一次正式提交机会**（三个任务可分别提交）⇒ 调试不充分就没有第二次 |
| task2 距离 | 3 m（基线实测距离 p50 3.13 m） |
| 发弹 | 单发、**≤5 Hz**（yaml `fire_interval_s: 0.2`）、**每挡 10 发** |
| 转速挡位 | 低挡 3–5 rad/s、中挡 5–9 rad/s |
| 评分 | **命中 20 分 + 角速度估计 20 分**（角速度容差 ±0.6 rad/s；基线实测 `w 4.00` 误差 0.00 ⇒ 这 20 分基本稳） |
| 现场 | 场地提供靶机与机器人；**调试时必须有负责人在场**、安全区内无人才能发射 |

---

## 3. minimax 已交付的成果（**勿重做**）

### 3.1 血缘链（DS4.1 用逐字节比对认定）

```
sp_vision_25（25 赛季官方框架，485★，思路来源）
  └─ sp_vision_tutorial_27 @ 88f877c（课程官方初始代码，10-19 官方自己修过 h_armor_xyz 高度计算）
       └─ 基线 D:\DSH\fp_aim（已含该修复，tasks/ io/ configs/ 与官方逐文件比对过）
            └─ 副本 fp_aim_fkey（基线 + 改造）
```

### 3.2 从官方框架搬来并重实现的三条思路（**默认都是关的**）

| 机制 | 开关 | 默认 | 作用 |
|---|---|---|---|
| 到达时刻选板 | `DSH_AIM_SELECT=arrival` | `current` | 用"弹到达时"的朝向选板/门控，而非"现在朝向" |
| 出弹延时 | `DSH_FIRE_DELAY` | 0 | 方案①下**只作记录**，不再累加 |
| 线性到位容限 | `DSH_ARRIVE_MM` | 0 | 官方 shooter 口径：3°/2° ≈ 恒定 105 mm |

官方出处：`aimer.cpp` 的 `predict(future + fly_time)` 与「detector-aimer 0.005 + 发弹延时 0.1」注释；`shooter.cpp` 的「新命令与上一帧差太多时这一枪不打」。

### 3.3 副本文件清单

**新增（9）**：`src/fire_key.hpp`(442 行) `src/low_plate.hpp`(207) `src/test_summary.hpp`(251) `src/env_util.hpp`(38) `tools/keytest.cpp`(43) `tools/lead_solve.py` `tools/parse_session.py` `tools/make_record.py` `tools/onsite/run_measure.sh`(17.8 KB)
**改动（3）**：`src/task_2.cpp`(1036 行) `src/task_3.cpp`(393) `src/task_common.hpp`（`calc_aim` 加 `t_extra` 默认参，向后兼容）
**其它**：`CMakeLists.txt` 加 `keytest` 目标；`tools/onsite/pack_manifest.txt`（打包唯一来源）+ `make_pack.ps1` + `pack/install.sh`

### 3.4 方案①（`DSH_LEAD` = 总提前量，唯一功能旋钮）

```
aim = plate_position(t_now + t_flight[calc_aim 自动] + t_link + t_fire)
```

三段时延合并成一个数：

| 段 | 谁在补 | 实测量 |
|---|---|---|
| 视觉链路（曝光→解算） | DSH_LEAD | **24.9 ms**（p50）/ 29.3（p95） |
| 云台跟随滞后 | DSH_LEAD | **31.8 ms**（互相关 r=0.71） |
| 同包 1 帧量化 | DSH_LEAD | 1/58 s ≈ 17 ms |
| 出弹延时（摩擦轮） | DSH_LEAD（`DSH_FIRE_DELAY` 只记录） | **未实测**（官方注释 0.06–0.1 s，副本记录值 0.020 s） |

**铁律：57 ms 只能出现在 `DSH_LEAD` 一处**，两处同时补 = 补过头且会被误判成"符号反了"。

### 3.5 环境变量全集（25 个，副本新增的加粗）

```
DSH_LEAD  DSH_ARRIVE_DEG  DSH_ARRIVE_PITCH_DEG  DSH_ARRIVE_FRAMES  DSH_ARRIVE_MM
DSH_ARRIVE_WATCHDOG_SHOTS  DSH_AIM_SELECT  DSH_PLATE_FACING_DEG  DSH_PLATE_REARM_DEG
DSH_LOW_PLATE  DSH_LOW_PLATE_Z_TOL  DSH_LOW_PLATE_TRUST_S  DSH_STATIC_RATE_DPS
DSH_STATIC_FRAMES  DSH_FIRE_KEY  DSH_FIRE_KEY_ENABLE  DSH_FIRE_KEY_DEV  DSH_FIRE_KEY_HAS_EVDEV
DSH_FIRE_DELAY  DSH_LAT_CSV  DSH_DET_CSV  DSH_STEP_DEG  DSH_TEST_TAG  DSH_SUMMARY_FILE
DSH_YAW_OFFSET  DSH_PITCH_OFFSET
```

`configs/standard.yaml` 关键项（副本与基线逐字节一致）：

```
yaw_offset: 0.0                       # A1 定案：车机上遗留的 10.24 探针值已归零
pitch_offset: 0.0
default_bullet_speed: 25.0            # 电控给的 23.014441 是常数、不可信
min_frames_for_fire: 10
fire_interval_s: 0.2                  # task2：5 Hz（赛规给的就是 5 Hz）
max_rounds_per_stage: 0               # 0 = 不限发（用户硬要求：不能加发弹数上限）
require_auto_aim_mode: true           # task2 必须 AUTO_AIM 挡位才发弹
fire_interval_s: 0.0834               # task3：12 Hz
record_dir: ""                        # 延迟测量期间绝不能开录像
```

### 3.6 M0–M10 测量计划（`docs/现场测量清单.md`，321 行）

| M | 测什么 | 状态 | 决定什么 |
|---|---|---|---|
| M0 | 预检（编译/按键/挡位） | `~/run_measure.sh preflight` | 后面全部的前提 |
| M1 | 静止靶 3 发 → 零位 | `run_measure.sh static` | yaml 的 yaw/pitch offset |
| M2 | 转靶 30 s，一场 6 个数 | `run_measure.sh lat M2_xx 30` | A2 errY / B1 / B2 / C1 dz / C2 w / C3 选板频率 |
| M3 | lead 符号自检（不发弹） | `run_measure.sh sign` | 符号对不对 |
| M4 | 斜率法定总提前量 T | `run_measure.sh slope M4_xx 0.057 0.171` | **DSH_LEAD = T** |
| M5 | 到位门限定档 | 用 M2 的 errY×1.2~1.5，**上限 1.2°** | DSH_ARRIVE_DEG |
| M6 | 基线对照（策略全关） | `run_measure.sh spin` | 命中率对照 |
| M7 | 低位靶 warmup | 只改 `DSH_LOW_PLATE=1` | 命中率取舍 |
| M8 | 到达时刻选板 | 只改 `DSH_AIM_SELECT=arrival` | 命中率取舍 |
| M9 | 人工开火门 | `DSH_FIRE_KEY_ENABLE=1` | 每挡 10 发额度的唯一兜底 |
| M10 | 归档 | `run_measure.sh pull` + 三个分析脚本 | 记录表 |

**顺序约束（不可违反）**：M1 必须最先（零位偏置与 lead 误差在落点上等价）→ M2 必须在转靶上 → M3 先于 M4 → M6 在 M1/M3/M4/M5 之后 → M7/M8 一次只改一个变量。

### 3.7 工具链分工（重要，别搞反）

```
车机（无外网、无 python3）                          笔记本（Windows，python 可用）
  compile / run / 打靶 / 记日志        scp ────────►  lat_*.csv + *.log
  run_measure.sh 8 个子命令                          latency_report.py  → B1/B2
  keytest 验按键通道                                parse_session.py   → 场次记录
  出弹延时实测                                      lead_solve.py      → T（★有 bug）
  录视频（外部手机侧录，落点判读）                  make_record.py     → 记录表.md
```

### 3.8 minimax 抓过的真 bug（别重犯）

1. `cmake` 失败会中断整个预检 → 降级为警告继续
2. `grep -m2` 把挡位那行截掉 → 挡位与启动参数分开 grep
3. logger 前缀导致 `^` 锚定的正则一条都匹配不上 → 全部改成不锚定行首
4. `DSH_RECORD_DIR=""` 关不掉录像（`env_dir` 把"未设"和"设空"都返回空）→ 用 `unset` 或改 yaml
5. PowerShell 5.1 按 ANSI 读 .ps1，UTF-8 无 BOM 会把中文解析坏 → 加 BOM
6. `Get-Content` 默认按 ANSI 读 UTF-8 清单 → 漏文件 → 显式 `-Encoding UTF8`
7. 清单里写"同上"（不是路径）导致打包漏文件
8. `run_measure.sh` 里的 `export` 会**覆盖**你在外面 `export` 的同名变量 ⇒ 改参数要在脚本外层传，或改脚本

---

## 4. DS4.1 已核实为真的缺陷（**别重复劳动**）

DS4.1 的自检报告经四审逐条复核，20+ 条硬声明中 17 条完全复现（数据类：n=2686、24.9/29.2 ms、|cmd−fb| p50 1.30°/p95 4.65°、≤0.8° 占 8.0%、连续 3 帧 1.5%、fire=1 恰 198 帧全单帧段、NIS 686 行、弹速兜底 25、颜色门 WARN，全部复算一致）。已确认为真、**必须修**的：

| # | 缺陷 | 位置 | 修法 |
|---|---|---|---|
| 1 | **编译错**：`low` 未声明 + 缺 `low_plate.hpp` | task_3.cpp:358 | 删 HUD 的 low 字段 + F 段换 ASCII（推荐）；或补 include + 声明 |
| 2 | HUD 中文乱码（cv::putText Hershey 无中文） | task_3.cpp:355-358 | 改 ASCII（task_2 已全 ASCII） |
| 3 | fire_key 暖启动：构造 `last_press_=now` + `ms<600` ⇒ 启动 600 ms 门是开的 | fire_key.hpp:108/168-173 | 构造时把 `last_press_` 置为 epoch/负值 |
| 4 | 汇总块 B1/B2/B3 永远「（未测）」 | test_summary.hpp:139-145 | 设计如此（手工项用 `add()`，无人调）；`--lat` 只做备注 ⇒ B1/B2 需手动跑 latency_report.py 抄 |
| 5 | AGENTS.md 漏 9 个 `DSH_` 变量 | AGENTS.md | 补表 |
| 6 | 差异对比.md 行号表过期 130 行（自述 906 行 / 实际 1036 行） | docs/与原工程差异对比.md:6-7 | 重新生成 |
| 7 | 文档自相矛盾：§1.4 写"三段"、§1.5 写"方案①不再累加" | 同上 :70 vs :86-87 | 统一到方案①（代码站方案①，:604 注释自洽） |
| 8 | `t_arrive` 用选板前的 `dist_guess` | task_2.cpp:599-604 | 选板后用 `xyza_list[aim_id]` 重算（≤0.4 m → 16 ms → 中挡 8.3° 相位） |
| 9 | `cmd_stable` 随 `DSH_ARRIVE_MM` 同步放大（2×tol_eff） | task_2.cpp:769-774 | 知道即可，M8.5 应急时注意 |
| 10 | 每挡 10 发限额无程序兜底（`max_rounds_per_stage: 0`） | standard.yaml:74 | 见 §10 第 4 项：做一个 submit.yaml 两全 |

**DS4.1 的两处误判（不要继承）**：
- 「修订清单收尾是假的」——**时间线错**：minimax 的修订清单 13:37 写、13:38 复验通过（task_3 残留确为 0），16:50 之后的改动重新引入了 `low` 未声明。副本无 git 历史，DS4.1 看不到这段，把"后续回归"当成了"验收造假"。
- 「step_1508.csv 采样约 13 Hz 做不了互相关」——**数据算错**：实测 p50 间隔 12.0 ms（83 Hz）、平均 12.2 ms；而 **31.8 ms 那个数从来不是在 step_1508 上算的**，是 `latency_report.py` 在 lat_1510.csv 上互相关得到的，脚本可复现。

---

## 5. 头号新发现：`tools/lead_solve.py` 符号反了

`tools/lead_solve.py:170`：`T = lead_a + off_a / k_use` —— **正确应为 `T = lead_a - off_a / k_use`**。

物理模型：off（正 = 沿转动方向超前）= k×(L − T) ⇒ T = L − off/k。

用它自己 docstring 里的用例实跑（lead 0.057→off −30 mm；lead 0.171→off +61 mm；k=800 mm/s）：

```
数学正解：T = 0.057 + 30/800 = 94.5 ms      # 另一档独立解 0.171 - 61/800 = 94.9 ms，两档互证
脚本实跑：T = 19.4 ms  ⇒  "新的 DSH_LEAD = 0.0194"    # 错 5 倍
```

后果：M4 按 0.057/0.171 两档打 6–10 发弹，标定出的 lead 把欠补从真实值变成 75 ms —— **标定必然失败，且浪费最稀缺的弹药**。minimax 自己在脚本 :51 写了"只做过语法检查、没实跑过"。

**修法（两处）**：

```python
T   = lead_a - off_a / k_use                     # :170 符号修正
T_b = lead_b - off_b / k_use                     # 新增：B 档独立解
if abs(T - T_b) / max(1e-9, abs(T)) > 0.15:       # 新增：两档一致性自检
    print("⚠ 两档解出的 T 差 >15% ⇒ 读数或转速不一致，别用这个结果")
```

注：`k_meas/k_theory` 的比值自检是对的（斜率方向没反），错的只有"反解 T"这一步 ⇒ 这正是"语法检查 ≠ 正确性验证"的教科书案例。

---

## 6. 只在对话里、代码里读不到的实现细节

### 6.1 低位靶机制为什么成立（EKF 三列的物理）

`tasks/auto_aim/target.cpp` 的 `h_armor_xyz`：1、3 号板是"高"的那两块，半径 = `x[8]+x[9]`、高度 = `x[4]+x[10]`；0、2 号板（低板）半径 = `x[8]`、高度 = `x[4]`。

```
H.block<4,3>(0,8).setZero()   # 首次换板前，第 8/9/10 列不参与更新
rlh_initialized_ = true       # 第一次换板时置真 ⇒ 之后几何列解冻
```

| 量 | 低板（0、2） | 高板（1、3） |
|---|---|---|
| `x[6]` 相位 / `x[7]` 角速度 / `x[4]` 中心高度 | 单板可测 | 单板可测 |
| `x[8]` 基准半径 | 需换板才可分（但对当前观测板自洽抵消） | 同 |
| `x[9]` 水平相对量 / `x[10]` 垂直相对量 | **恒为 0（定义上不用测）** | **必须两块板同时可见才可测** |

⇒ **高板在换板前，模型等于宣称"1、3 号板和 0、2 号板同高同半径"** ⇒ 瞄点落在"高板应该待的位置"，差 0.1–0.2 m = 3.15 m 处 1.8–3.6°，而板半宽只有 1.29° ⇒ 几乎必偏。
⇒ warmup 的 4 s 计时盯的是 **`z_spread` 稳定**（几何列解锁并稳定），不是"高板出现次数"。
⇒ **文档口径必须这么讲**（minimax 初版文档说"整个挡位冻结"是错的——`target.cpp:227-229` 首次换板即解冻）。

### 6.2 lead 与固定靶的关系（决定了标定必须在转靶上做）

`aim_yaw += 板在相机里的视在角速度 × lead_s`，视在角速度 = r·ω/d：

| 目标 | 视在角速度 | lead=0.080 s 的补偿量 |
|---|---|---|
| 固定靶 | **0** | **0.00°** ⇒ 一模一样 |
| 4 rad/s 转靶 | 14.55 °/s | 1.16° ≈ 0.47 靶半宽 |
| 9 rad/s 转靶 | 32.7 °/s | 2.62° ⇒ 几乎必偏 |

⇒ **固定靶全中靶心 = lead 乘 0 = 补偿量 0**，**不能证明 lead 标对了**；也**绝不能用固定靶标 lead**。

### 6.3 基线 `fix_log.md` §1 总表（A1–D5，10-20 那一轮的状态）

```
A1 静止靶上方 23° 掠过 → pitch_offset=10.24 是探针值 → yaml 归零（10-21 已定案，车机需传文件落地）
A2 检测到蓝板 → 颜色映射按实车改过（上游已含）
A3 bullet_speed 四场恒 23.014441 → 电控常数不可信 → 只记录不改
B1 rounds 333 打空枪 → fire 没校验有无观测 → would_fire 加 has_detection
B2 云台还在追就开火 → arrive_tol_deg 默认 0=门控关 → 默认 0.8°/1.28°、连续 3 帧
B3 门控挡 12 发后自动解除 → watchdog 默认 12 → 默认 0=永不解除
B4 板转到 70° 侧面还在打 → 无板面朝向判断 → |facing| ≤ 25°
B5 对同一块板连打 → 只有全局间隔 → 一块板一发、摆出 35° 才重上膛
B6 0.2 s 内同板连吃两发 → 4 rad/s 下 0.2 s 只转 46° → fire_interval_s 0.2→0.35
B7 第一发迟迟不出 → 15 帧@45fps=0.33 s → min_frames_for_fire 15→10
B8 电控 IDLE 时软件仍请求开火 → require_auto_aim_mode false → task2 改 true
B9 追着快转出视野的板打 → aim_id=last_id 只按角度最近 → 取 |facing| 最小（真迟滞要把 aim_id 提到循环外）
C1 准星被推 12.6°（5 块板）→ 把自转角速度当视在角速度 → 改成视在角速度
C2 板往右转却往左补 → 近侧板视在角速度与 w 反号 → 符号一并修正
C3 lead 填多少全靠猜 → 没量过链路延迟 → 视觉 24.9 + 云台 31.8 = 57 ms
D1 开录像 58→44.7 fps → 调 lead 的场次必须不录像
D2 DSH_RECORD_DIR="" 关不掉录像 → 用 unset 或改 yaml
D3 四场录像里一发没打也没日志 → 挡位 IDLE + 没 tee → 必须 2>&1 | tee logs/*.log
```

### 6.4 用户自己写的复核稿 `audit_review.md`（10-20，基线 docs/ 下）

- 删 3 条（"每帧 0.25°"应为 3.9°，差 15 倍；"A→B→A 洞"稳态不可达；"打到没有板的地方"不成立）；
- 降级 2 条（`:556` 死代码、硬伤 1 候选集含看不见的板）；
- 改 3 处推导（必须用视在角速度 14.3°/s 推；是 **9** 个条件不是 8，漏了 `!latency.step_enabled()`；额度漏洞要加 `bullet_count_alive == false`）；
- 补 3 条漏的（两个 NIS 阈值：9.488 只做统计、16.27 才真拒收并回滚 x/P；窗口 100 帧 ≈ 1.6 s；门限上限 1.2°，1.5° 超过板半宽 1.29°）。
- **优先级重排（用户不同意原报告把 lead 排第一）**：① 确认 AUTO_AIM + `DSH_ARRIVE_DEG=1.2`（能不能打出去）→ ② `DSH_LEAD=0.057` 先验符号再定值 → ③ HUD 打出 aim_id/facing → ④ `target_static` 阈值 0.15 + 连续 10 帧 → ⑤ aim_id 提到循环外 → ⑥ 斜率法标定 lead。

---

## 7. 已核实的实测数据（可直接引用）

```
lat_1510.csv（基线 logs/，n=2686，62.4 fps，10-20 15:14 采集）
  取帧→解算 p50 24.92 ms / p95 29.25 ms      # B1
  解算→串口 p50 0.06 ms
  云台 yaw 滞后 31.8 ms（互相关 r=0.71）        # B2
  yaw 阶跃 T10≈48 ms、T90≈48 ms、超调 +252%     # 震荡大 ⇒ errY 压不下去的根源
  pitch 滞后 460.8 ms（不可信，信号太平）
  距离 p50 3.13 m（2.98–3.41）
  |cmd−fb| p50 1.30° / p95 4.65°

到位门限的出弹率可行性（同一份 csv 复算）
  tol 0.8° → 单帧 8.0%   连续3帧 1.5%    # 缺省值 ≈ 必然哑火
  tol 1.0° → 单帧 19.8%  连续3帧 8.5%
  tol 1.2° → 单帧 40.0%  连续3帧 25.3%  # M5 的上限，出弹率约 1–1.5 Hz
  tol 1.5° → 单帧 62.9%  连续3帧 48.3%

其它判据（现场测量清单 §6）
  板宽 135 mm / 半宽 1.23° @3.15 m
  视在角速度 14.55°/s @4 rad/s（靶柱相位 229°/s，别混）
  一圈 4 块各 1 发 ⇒ 2.55 Hz；只打低位靶 ⇒ 1.27 Hz
  面朝窗口 ±25° = 50° ⇒ 4 rad/s 下 0.22 s；中挡 9 rad/s 时窗口只剩 ~97 ms（5–6 帧）
  NIS 拒收风暴 ⇒ 最长 1.6 s 不发弹（不是永久）
  官方国赛命中率参照：39.6% 最高、≥30%（2 m、7/14 rad/s）
```

---

## 8. 用户的硬约束与偏好（**不可违反**）

1. **严禁改基线** `D:\DSH\fp_aim`；只改副本。每轮报告里给出哈希自证。
2. **不能加发弹数上限**（`max_rounds_per_stage: 0` 是用户自己的硬要求）⇒ 正式提交的 10 发额度靠人工门兜底（可用 submit.yaml 变通，但练习时必须仍不限发）。
3. **所有文档的代码块**：一行一条命令、注释右对齐（约 41–66 列）、fence 标 `bash`/`text`/`powershell`、无 BOM。
4. **说中文**（用户原话：「你说的是人话吗？？？？说中文」）。
5. 文档风格：结论先行、每条给代码位置、不写"可能是/也许是"、写明一条命令回退。
6. 改动要「零行为变化」还是「改行为」必须说清；`DSH_*` 是 `static` 一次性读取 ⇒ **改参数必须重启进程**。
7. 保留 minimax 的「原工程零改动 + 哈希自证」纪律。

---

## 9. 现场环境约束

```
笔记本 192.168.1.43（固定）        车机 192.168.1.50
车机永远没外网，只有到笔记本的网线直连
"笔记本 ping 不到车机" 很正常（防火墙挡入站 ICMP）→ 从车机 ping 笔记本，或直接试 scp
车机无 python3 → 所有分析在笔记本做
车机只做：编译 / 跑 / 打靶 / 记数
编译只在车机做（本机无 OpenCV/Eigen 工具链）→ 每处 C++ 改动必须留给车机验证
mac/Windows 差异：这台笔记本只有 Windows PowerShell 5.1（没有 pwsh）
```

---

## 10. 修复执行顺序（今天，半天）

| # | 动作 | 工作量 | 验证方式 |
|---|---|---|---|
| 1 | `lead_solve.py:170` 符号修正 + 两档一致性自检 | 5 min | **本地实跑正反例**（见 §5） |
| 2 | task_3.cpp:358 删 low 字段 + F 段换 ASCII | 10 min | 括号配平 + 车机 `make task_3` |
| 3 | fire_key.hpp 构造把 `last_press_` 置 epoch | 1 行 | 括号配平 |
| 4 | 新增 `configs/submit.yaml`（仅 `max_rounds_per_stage: 10` 不同），M9 改用它 | 5 min | yaml diff 人工确认 |
| 5 | 现场测量清单插 **M8.5 中挡验证场**（干跑 30 s + 2–3 发实弹） | 15 min | 文档 |
| 6 | M2 判据加 NIS 行；M5 补"应急先 ARRIVE_FRAMES=2" | 5 min | 文档 |
| 7 | （可选）`parse_session --lat` 自动填 B1/B2；`t_arrive` 选板后重算距离 | 30 min | 实跑 |
| 8 | 重新打包（`make_pack.ps1`）+ 重出 patch（`pack_manifest.txt` 是唯一文件来源） | 10 min | 车机 install.sh 冒烟 |

**验证纪律（这次翻车的根因）**：Python 脚本改完**必须实跑**（minimax 的四个脚本全部只做过语法检查）；C++ 改完**必须在车机编译**；每轮改动给出「原工程哈希未变」的证明。

---

## 11. 发弹预算（机会有限，三任务共享）

```
M1 零位 3 发（不可压）      M4 两档各 3 发 = 6（不可压）
M6 基线 3 发（取下限）      M8 arrival 3 发（取下限）      M8.5 中挡 2–3 发（不可砍）
M7 低位靶 3 发 —— 弹紧第一个砍（warmup 自动失效，默认开也接近无害）
task2 最小集 = 17–18 发；含 M7 则 20–21 发
裁剪顺序：M7 → M8 → M6；最坏情况 M1→M4→M8.5→提交 = 13 发
每发取双份信息：打完抄 HUD（errY/t_arr/assoc）+ 队友手机侧录落点（车机 record_dir 仍保持 ""）
```

---

## 12. 红线（别踩）

```
✗ 别把"改完没编译/没实跑"当成交付（minimax 的两次重大失误都是这个）
✗ 别在 DSH_LEAD 和 DSH_FIRE_DELAY 里同时放同一段延迟（双补 ⇒ 看着像符号反了）
✗ 别用固定靶标 lead（视在角速度=0 ⇒ lead 恒为 0）
✗ 别在 M1 之前做 M4（零位误差会被 lead 吸收）
✗ 别在延迟测量期间开录像（写盘抖动污染 send_ns）
✗ 别把 DSH_LEAD 设成 0.0194（那是 lead_solve 符号 bug 的产物）
✗ 别信 bullet_speed 23.014441（电控常数）；别信 pitch 滞后 460.8 ms
✗ 别改基线目录；别加发弹数上限到 standard.yaml
✗ 别在同一次改动里同时改多个变量（一次只改一个）
✗ 别把 task_3 的 HUD 中文留给 cv::putText
```

---

## 13. 交给 GLM5.3 的三个决策点（建议先拍板再动手）

1. **中挡 9 rad/s 怎么保证？** 建议：M8.5 干跑 30 s + 2–3 发实弹；若整圈无 arrive，顺序为 `DSH_ARRIVE_FRAMES=2` → `DSH_ARRIVE_DEG=1.5` → 接受中挡 0/10 保低挡 + 角速度分。
2. **每挡 10 发的兜底形式？** 建议：练习用 standard.yaml（不限发），正式提交换 submit.yaml（限 10 发）——程序兜底不违背"练习不限发"。
3. **10-23 / 10-24 / 10-25 怎么排？** 建议：10-23 一个完整时段走完 M0–M8.5；10-24 机动（返场/中挡调参/task_3）；10-25 **只提交，不做任何新改动**。
