> **本文已被 docs/debug_flow.md 取代（2026-10-21 重写合并版）。**
> 本文保留作为历史记录：勘误内容已全部并入新版手册，副本的新机制也已并入。
# 现场调试流水线 v3（副本版：低位靶 + 按住 F 开火）

> **对象**：`D:\MTG\Documents\OC\fp_aim_fkey`（副本）。**原工程 `D:\DSH\fp_aim` 一个字节没动**，
> 本流程的所有回退都指回原工程。
> **依据**：副本 `src/task_2.cpp`（含复核稿第 3/4/5 条 + 低位靶规则 + 按住 F 开火）、
> `src/fire_key.hpp`、`src/low_plate.hpp`、`tools/keytest.cpp`、`configs/standard.yaml`。
> **与旧流程的关系**：`docs/debug_flow_v2.md`（原工程版）的骨架、P0–P7、决策树全部保留，
> 这里只写**副本特有**的部分和被改掉的判据。
> 全文代码块标记：`powershell` = 笔记本，`bash` = 车机。

---

## 0. 三句话读完

1. **先传代码（第 1 节）**，再**先验按键通道（第 2 节）**——副本多了一道"人按住 F 才开火"的门，
   按键通道不通 ⇒ 一场都不发，**这比原工程多了一个新的哑火原因**；
2. **先证明能打出去**（P0–P4），再谈打中（P5–P9）；**P5 是一场"对照组"**，
   后面所有对比都跟它比；
3. 卡住就看决策树（第 4 节），**每一步只允许动一个变量**。

---

## 1. S0 传代码（18 个文件 = 原工程的 15 + 副本新增的 3）

### 1.1 原工程那 15 个（不重复列，见 `debug_flow_v2.md` §1.1）

`src/task_2.cpp` `src/task_3.cpp` `src/overlay.hpp` `src/detect_log.hpp` `src/latency_probe.hpp`
`tasks/auto_aim/armor.cpp` `tasks/auto_aim/target.cpp`
`io/camera.cpp` `io/camera.hpp` `io/gimbal/gimbal.cpp`
`io/hikrobot/hikrobot.cpp` `io/hikrobot/hikrobot.hpp`
`configs/standard.yaml` `tools/extended_kalman_filter.cpp` `tools/extended_kalman_filter.hpp`

> 其中 **`src/task_2.cpp` / `src/task_3.cpp` 要传副本的版本**（不是原工程的）。
> 若车机不是 HEAD，`io/gimbal/gimbal.hpp` 也要一起传（`send()` 的第 5 参 target 在它里面）。

### 1.2 副本新增的 3 个

| 文件 | 为什么 |
|---|---|
| `src/fire_key.hpp` | 按住 F 的按键门（evdev / Windows / 终端 / 窗口 四级降级） |
| `src/low_plate.hpp` | 低位靶判定（按 EKF 预测的板高分组，高低分不开时自动失效） |
| `tools/keytest.cpp` | **不依赖工程的按键自测程序**，第 2 节用 |

### 1.3 传（笔记本 PowerShell）

```powershell
cd D:\MTG\Documents\OC\fp_aim_fkey
$car = "rm@192.168.1.50"
# 副本新增的 3 个
scp src\fire_key.hpp src\low_plate.hpp   ${car}:~/fp_aim/src/
scp tools\keytest.cpp                     ${car}:~/fp_aim/tools/
# 副本版的 task_2 / task_3（覆盖车机上原工程那两份）
scp src\task_2.cpp src\task_3.cpp         ${car}:~/fp_aim/src/
```

原工程那 15 个按 `debug_flow_v2.md` §1.2 传（**注意 `task_2.cpp`/`task_3.cpp` 用副本版覆盖**）。

### 1.4 验收（车机 bash，8 条 grep，一条都不能空）

```bash
cd ~/fp_aim
# ── 原工程那 5 条（判据已按勘误表修正；计数按 2026-10-21 的副本版本实测）──
grep -c 'plate_facing_deg' src/task_2.cpp        # 期望 >= 4
grep -c 'target_static'    src/task_2.cpp        # 期望 >= 2
grep -c 'DSH_LEAD'         src/task_2.cpp        # 期望 >= 7（lead 已被 t_arrive 引用，注释里也有）
grep -nE '^(yaw_offset|pitch_offset):' configs/standard.yaml   # 恰好 25、26 两行，都是 0.0
grep -n 'fire_interval_s'  configs/standard.yaml               # task2 段必须是 0.2
# ── 副本新增的 3 条 ──
grep -c 'fire_key'   src/task_2.cpp             # 期望 >= 20
grep -c 'low_plate'  src/task_2.cpp             # 期望 >= 9
grep -c '选板变化'   src/task_2.cpp             # 期望 >= 1（副本才有这行日志）
```

> `grep -c` 匹配 0 行时返回码是 1，**别用 `&&` 串**。

### 1.5 编译

```bash
cd ~/fp_aim
touch src/task_2.cpp src/task_3.cpp src/fire_key.hpp src/low_plate.hpp
cd build && make -j2 2>&1 | tail -20
```

**通过判据**：出现 `Building CXX object .../task_2.cpp.o` + `Linking CXX executable .../task_2`。
只出现 `Built target task_2` = 没重编。

---

## 2. S0.5 验按键通道（**副本新增，必须在跑车前做**）

副本多了"人按住 F 才开火"这道门。按键通道不通 ⇒ `key=False` ⇒ **一场都不发**，
而其它日志字段全是 True ⇒ 很容易被误判成"程序坏了"。

```bash
cd ~/fp_aim
g++ -O2 -std=c++17 -I. tools/keytest.cpp -o /tmp/keytest && /tmp/keytest
```

期望：

```
backend = evdev:/dev/input/event3 /dev/input/event4      ← 车机上最常见
[允许开火] F=按住        ← 按住 F
[禁止开火] F=松开        ← 松开 F
```

| backend | 含义 | 处置 |
|---|---|---|
| `evdev:...` | ✅ 真按住（能读按下/松开），**不需要窗口焦点** | 直接用 |
| `termios(raw)` | ⚠️ 靠键盘自动重复近似"按住" | 建议改 `DSH_FIRE_KEY=toggle` |
| `cvkey(window)` | ⚠️ 只能靠 OpenCV 窗口收键，**要求窗口有焦点** | 见下 |
| `always` / `never` | 环境变量把门焊死了 | `unset DSH_FIRE_KEY` |

`cvkey(window)` 的处置（车机常是远程/X11 转发，窗口没焦点时收不到键）：

```bash
ls -l /dev/input/event*                      # 看设备在不在、权限对不对
cat /proc/bus/input/devices | grep -A3 keyboard
sudo usermod -aG input $USER                 # 或直接指定设备
DSH_FIRE_KEY_DEV=/dev/input/event3 /tmp/keytest
```

还是不行就 `DSH_FIRE_KEY=toggle`（按一次切一次，不依赖按住）。

---

## 3. 流水线 P0–P10

| 阶段 | 目标 | 命令要点 | 通过判据 | 失败去 |
|---|---|---|---|---|
| **P0** | 能启动 | `./build/task_2 configs/standard.yaml 2>&1 \| tee logs/p0.log` | 出现**四行** `[task2] offset` / `[task2] 到位门控` / `[task2] 低位靶规则` / `[task2] 出弹延时补偿`（还有一行"预测时域"） | D-7 |
| **P0.5** | 按键通 | 第 2 节的 `keytest`（现在有 CMake 目标：`./build/keytest`） | backend 是 evdev/termios，且按住 F 状态会变 | D-8 |
| **P1** | 看得见 | 画面有检测框、HUD `tgt YES` | 框跟着靶子走 | D-0 |
| **P2** | 挡位对 | `grep -m2 '电控挡位' logs/p0.log` | `AUTO_AIM(自瞄)` | D-3 |
| **P3** | 云台到位 | `grep -m3 'arrive' logs/p0.log` | `errY` < 门限、`blocked` 不持续涨 | D-2 |
| **P4** | lead 符号 | `DSH_LEAD=0` 与 `0.57` 各 10 s | 0.57 那场**准星明显领先板子**；`AIM yaw` 小约 8.3° | D-4 |
| **P5** | **对照组** | `DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.057`，打 3 发 | 中 ≥1 发；`rounds` 涨 3 | D-1/D-2/D-6 |
| **P5b** | 人工门 | `DSH_FIRE_KEY_ENABLE=1`（其余同 P5），按住 F 打 3 发 | 徽章变绿 ARM、状态行 `GATE=ON`、日志有"按键边沿" | D-8 |
| **P6** | 低位靶规则 | `DSH_LOW_PLATE=1`（其余同 P5），打 3 发 | `dz` > 容差且 `plate=LOW/HIGH` 在切；**发数变少是正常的**；信任计时走满后会打一行"放开高板" | D-9 |
| **P6b** | 到达时刻选板 | `DSH_AIM_SELECT=arrival`（其余同 P5） | `plate=(现朝向→到达朝向)`：现朝向为负、到达朝向≈0 | D-4 |
| **P6c** | 出弹延时 | `DSH_FIRE_DELAY=0.05`（再试 0.10） | 落点随延时**线性**移动（不是"打了就中"） | D-4 |
| **P6d** | 线性容限 | `DSH_ARRIVE_MM=105`（= 官方口径） | `blocked` 不再一直涨、`errY` 能进容限 | D-2 |
| **P7** | 选板稳不稳 | `grep -c '选板变化' logs/p5.log` | 4 rad/s 下 ≈ **2.5 次/秒**（每 90° 相位换一次板） | D-10 |
| **P8** | 静止靶零点 | 静止靶打 3 发（建议 `DSH_PLATE_REARM_DEG=0.01`） | 落点距靶心 < 1 靶宽 | D-5 |
| **P9** | 旋转靶 | 4 rad/s 打 3 发 | 中 ≥1 发 | D-4/D-6 |
| **P10** | 记录 | `runlog.py` + `collect_run.sh` | 三个数进表 | — |

**铁律（副本版）**：

- **P0.5 没过不许打 P5**（按键不通 ⇒ 全部字段 True 但一发不发，浪费整个挡位额度）；
- **P3 没过不许打 P5**；**P4 没过不许打 P9**；
- **P5 没打过不许开低位靶**（否则分不清命中率变化是"低位靶"带来的还是别的）；
- 每换一个变量，**重跑一次 P0**，确认启动日志里的参数是新的；
- 每一场都带 `2>&1 | tee logs/xxx.log`（`tools/onsite/collect_run.sh` 靠日志收数据）。

---

## 4. 决策树 D-0 … D-10

每个节点只问一个问题、只跑一条命令。D-0～D-7 与 `debug_flow_v2.md` 相同（只补了副本新增的字段），
**D-8～D-10 是副本特有的**。

### D-1 跟得住但一次都不发（`fire-gate` 有项 false）——**副本版**

```bash
grep -m6 'fire-gate' logs/p0.log | tail -6
```

按**出现顺序**排查，前一项过了才看下一项：

| 字段 | false 的含义 | 处置 |
|---|---|---|
| `key=False` | **人没按住 F**（或按键通道不通） | **D-8**（副本新增） |
| `low=False` 且 `低位靶=否` | 瞄的是高板，低位靶规则不让打 | **D-9**（副本新增） |
| `stable=False` + `nis=0.8~1.0` | NIS 失败率被顶满 | 等 1.7 s（窗口 100 帧）再看；持续说明关联错板 |
| `frames=3/10` 长期不动 | 目标在丢帧 | D-0 的丢帧分支 |
| `interval=False` | 刚打过 | 等 0.2 s |
| `mode=False` | 电控不在 AUTO_AIM | D-3 |
| `hold=True` | 目标丢失但还在保持期 | 正常，等 `target_hold_ms` |
| `plate=False(30°)` | 板不在 ±25° 窗口 | 等它转进来（窗口 50°，4 rad/s 下 **0.22 s**） |
| `plate=False` 且角度常在 ±60~120° | **facing 约定反了** | D-4 |
| 全部 True 但仍不发 | 到位门控 | D-2 |

> 副本的 `fire-gate` 行多了三个字段：`aim_id=`（真正瞄的那块）、`low=(dz=…低N共M块)`（低位靶规则状态）、
> `key=`（按键门）。

### D-8 【副本新增】所有字段都是 True，就是不发

```bash
grep -m3 'fire-gate' logs/p0.log | tail -3   # 看 key= 那一项
```

**先看右上角徽章**（4 态，40 m 外也能看清）：

| 徽章 | 含义 | 处置 |
|---|---|---|
| 灰 `GATE OFF (auto fire)` | 人工门没开（**默认就是关的**） | 调试要人工确认 ⇒ `DSH_FIRE_KEY_ENABLE=1` |
| 黄 `BACKEND ERR …` | 取键后端读键异常（设备热拔/权限变化） | `ls -l /dev/input/event*`、`DSH_FIRE_KEY_DEV=…`；实在不行用 toggle |
| 红 `SAFE NO FIRE` | 门开着但现在不允许（没按 F / toggle 未解锁） | 按住 F；或按一次 F 解锁 |
| 绿 `ARM FIRE ALLOWED` | 现在允许开火 | 若仍不发，看状态行其余字段（facing / plate / 到位） |

| 现象 | 结论 | 处置 |
|---|---|---|
| `key=False`，人确实按着 F | 按键通道没通 | 回第 2 节 `keytest`；`DSH_FIRE_KEY=toggle` 试 |
| `key=False`，人没按 | 正常 | 按住 F |
| `key=True` 但仍不发 | 不是按键的问题 | 回 D-1 其它字段 / D-2 |
| 状态行 `GATE=OFF` 但你以为门是开的 | `DSH_FIRE_KEY_ENABLE=0`（默认） | 开它 |
| 徽章颜色和手指不一致 | 通道问题（cvkey 兜底在扛） | 跑 `keytest` 看 backend |

### D-9 【副本新增】低位靶规则把机会全挡了

```bash
grep -m3 'fire-gate' logs/p0.log | tail -3   # 看 low= 与 dz=
```

| 现象 | 结论 | 处置 |
|---|---|---|
| `dz=0.00x` 一直很小 | EKF 的高度差（x[10]）还没学出来 ⇒ 规则其实**没生效**（`active=false`） | 正常，多跑几圈让 EKF 收敛；或把 `DSH_LOW_PLATE_Z_TOL` 调小再试 |
| `dz=0.1x` 以上且 `低位靶=否` 长期为真 | 规则在拦（你瞄的一直是高板） | 正常，等低板转正；或 `DSH_LOW_PLATE=0` 关掉 |
| `低位靶=否` 且 `plate=False` 同时出现 | 低位 + 面朝两道门叠加 | 看 `选板变化`：aim_id 是不是老钉在高板上 |
| 想要"每圈 4 发"而不是 2 发 | 低位靶规则的本意就是砍一半 | `DSH_LOW_PLATE=0` |

### D-10 【副本新增】选板在临界点来回跳

```bash
grep -c '选板变化' logs/p5.log
```

| 现象 | 结论 | 处置 |
|---|---|---|
| 4 rad/s 下 ≈ 2.5 次/秒（每 90° 换一次板） | ✅ 正常 | — |
| 几十次/秒 | 临界带在闪（两块 facing 差 ±5°） | 副本已把 `aim_id` 跨帧保持（迟滞 5° 真生效）；仍闪就查 `assoc=` 跳变 |
| 一条都没有 | 选板一直是同一块 | 查 `aim_id` 与 `plate` 字段是否同号 |

### D-2 门控挡住（`arrive=False`，`blocked` 持续涨）

| 判据 | 结论 | 处置 |
|---|---|---|
| `errY` 稳定在 0.8~1.5 | 门限太紧 | `DSH_ARRIVE_DEG=1.2`（**上限 1.2°**，超过板半宽 1.23° 就是放空） |
| `errY` > 2° | 云台跟不上或零位偏 | 先降到 3 rad/s |
| `errP` 一直 > 1.28 | pitch 符号约定错 | 看 `fb pitch` 与 `cmd pitch` |
| `tripped=1` | 看门狗已解除（缺省关） | 忽略 |

---

## 5. 副本特有：六个问题现在什么答案

| 问题 | 副本的答案 | 和原工程的差别 |
|---|---|---|
| **Q1 能开火吗** | 能，但**多了 1 个静默杀手**：`key=False`（人没按 / 按键通道不通） | 原工程 5 个杀手 → 副本 **6 个**；所以多了第 2 节的按键自检 |
| **Q2 单个目标只开一次吗** | 成立，语义是「**每块板每圈 1 发**」。4 rad/s 下平均 **2.55 Hz**（最快 3.5 Hz）；**开低位靶后每圈只有 2 块低板 ⇒ 1.27 Hz** | 副本把静止靶判定从 `|x[7]|<0.5` 改成**量 facing 的逐帧变化**，w 爬升/反向过零不再误判；`aim_id` 跨帧保持后 5° 迟滞真生效 |
| **Q3 只在时机最完美时开火吗** | 面朝门控（±25°）+ 到位门控（0.8°/1.28°、连续 3 帧）+ 低位靶（只打基准高度那组）+ 人工门 | 多了一道"只打低板"；代价是**机会减半** |
| **Q4 延迟补偿正常吗** | 公式对、符号待验；**缺省仍是 0（关）**，必须 `DSH_LEAD=0.057` 起步再按斜率法定标 | 与原工程一样；副本的 HUD 第二行不会显示 lead，注意别忘 |
| **Q5 会抖吗** | 瞄准不抖；选板在临界点会闪 ⇒ 副本新增了 `[dbg] 选板变化` 单独记一行来量化它 | 原工程每 48 帧打一次日志，**看不见** 0.15 s 的窄带；副本解决了"看不见" |
| **Q6 会哑炮吗** | 三种：① 门控挡住（`blocked` 涨）② 人没按 F（`key=False`）③ 低位靶规则把机会挡了（`low=否`） | 比原工程多两种，但**都能在日志里一眼看到**，不会静默 |

---

## 6. 与原工程那份流程的差异速查

| 项 | 原工程 `debug_flow_v2.md` | 副本（本文件） |
|---|---|---|
| 传文件 | 15 个 | **18 个**（+`fire_key.hpp` `low_plate.hpp` `keytest.cpp`，且 task_2/task_3 用副本版） |
| 跑车前的准备 | 无 | **第 2 节 验按键通道**（新增，不做就等于一场都不发） |
| 启动日志判据 | 2 行（且原文写的那行不存在，见勘误表 H1） | **3 行**：`offset` / `到位门控` / `低位靶规则` |
| 流水线 | P0–P7 | **P0–P10**（新增 P0.5 按键、P5 对照组、P6 低位靶、P7 选板抖动） |
| 决策树 | D-0–D-7 | D-0–D-7 + **D-8 按键门 / D-9 低位靶 / D-10 选板抖动** |
| `fire-gate` 字段 | stable/conv/frames/nis/interval/rounds/mode/hold/plate/assoc/mode_now/bullet_alive | 上面那些 + **`aim_id` / `low`(dz,低N共M) / `key`** |
| HUD | 1 行 | **2 行**（第二行：`F=按住 (后端) 低位靶=是 dz=… 瞄准板=… facing=…`） |
| 弹频 | 5 Hz 上限，一圈 4 发 | 一圈 4 发 ⇒ **平均 2.55 Hz**；开低位靶 ⇒ **1.27 Hz** |
| 补丁 3/4/5 | 待改 | **副本已落**（`docs/复核稿第345条_已落记录.md`） |

---

## 7. 判据速查（副本）

| 量 | 值 | 备注 |
|---|---|---|
| 板宽 / 半宽 @3.15 m | 2.46° / **1.23°** | 135 mm 板 |
| 视在角速度 @4 rad/s | **14.55°/s** | `r·w/d`，不是 229°/s |
| 到位门限 | 缺省 0.8°，**上限 1.2°** | 超过半宽 = 放注定打不中的弹 |
| 连续到位帧 | 3 帧 ≈ 50 ms | 应急可 `DSH_ARRIVE_FRAMES=2` |
| 开火间隔 | 0.2 s | 5 Hz 上限 |
| 一圈发数 | 4 块板各 1 发 ⇒ **平均 2.55 Hz**（4 rad/s） | 最快 3.5 Hz |
| **只打低位靶** | 每圈 2 块 ⇒ **1.27 Hz** | `DSH_LOW_PLATE=1` |
| 面朝窗口 / 重上膛 | ±25° / 35° | 窗口时长 50° ⇒ 4 rad/s 下 0.22 s |
| 低位靶容差 | 缺省 0.10 m | 要 **小于真实高度差、大于 EKF 高度噪声** |
| 静止靶判定 | facing 逐帧变化 < 0.5°，连续 10 帧 | 副本已改成不依赖 `x[7]` |
| NIS 窗口 / 阈值 | 100 帧 ≈ 1.7 s；9.488 / 16.27 | 拒收风暴只导致最长 1.6 s 不发弹 |
| `min_frames_for_fire` | 10 帧 ≈ 172 ms | |
| lead | 缺省 **0（关）**；0.057 s ⇒ 0.83° ≈ 0.34 靶宽 | 斜率法定标，**别直接填 0.08** |

---

## 8. 记录与回退

### 记录

```powershell
cd D:\MTG\Documents\OC\fp_aim_fkey
py tools\runlog.py new r1021_p9 --task task_2 --stage P9
py tools\runlog.py set r1021_p9 shots=3 hits=1 lead=0.057 arrive_deg=1.2 low_plate=1 fire_key=hold
py tools\runlog.py show
```

### 回退（**两条路，别用 git**）

**路 1：只回退两条新策略（推荐，一行）**——两条策略都在环境变量后面，关掉就与原工程等价：

```bash
DSH_LOW_PLATE=0 DSH_FIRE_KEY=always ./build/task_2 configs/standard.yaml
```

> 注意：这条路**不**回退复核稿第 3/4/5 条（选板跨帧、静止靶判定、诊断日志），
> 那三条是纯改进，留着更好。

**路 2：完全回到原工程**（含第 3/4/5 条）——把车机上副本版的两个文件换回原工程的：

```powershell
# 笔记本
scp D:\DSH\fp_aim\src\task_2.cpp D:\DSH\fp_aim\src\task_3.cpp rm@192.168.1.50:~/fp_aim/src/
```

```bash
# 车机
cd ~/fp_aim && touch src/task_2.cpp src/task_3.cpp && cd build && make -j2 task_2
```

**路 3：只回退第 3/4/5 条**（保留两条新策略）——用副本里的备份还原：

```bash
cd ~/fp_aim && cp src/task_2.cpp.bak_aim src/task_2.cpp   # 车机侧备份（fix_log §4.3）
cd build && make -j2 task_2
```

> **车机上大概率没有 .git**（仓库在笔记本上），所以 `git checkout -- src/task_2.cpp` 那条在车机上用不了。

---

## 9. 一页纸：副本的上车顺序

```bash
# ── 笔记本：传 18 个文件（见 §1.3）────────────────────────
# ── 车机 ──────────────────────────────────────────────
cd ~/fp_aim
./build/keytest 2>/dev/null || (g++ -O2 -std=c++17 -I. tools/keytest.cpp -o /tmp/keytest && /tmp/keytest)   # S0.5 按键
touch src/task_2.cpp src/task_3.cpp src/fire_key.hpp src/low_plate.hpp src/env_util.hpp
cd build && make -j2 2>&1 | tail -20
cd ~/fp_aim
# P0 启动
./build/task_2 configs/standard.yaml 2>&1 | tee logs/p0_$(date +%m%d_%H%M).log
# P5 对照组（人工门默认就是关的，这里显式写出来免得现场搞不清）
DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.057 \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/p5_base_$(date +%m%d_%H%M).log
# P6 低位靶（同条件，只改这一个变量）
DSH_LOW_PLATE=1 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.057 \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/p6_low_$(date +%m%d_%H%M).log
# 每场跑完先看这四条
grep -m1 '电控挡位' logs/p5_*.log        # AUTO_AIM？
grep -m3 'arrive'    logs/p5_*.log        # errY / blocked / jump
grep -m3 'fire-gate' logs/p5_*.log        # key= / low= / plate= / rounds
grep -c  '选板变化'  logs/p5_*.log        # ≈ 2.5 次/秒
```

**五个变量决定"能不能打出去"**（副本版）：

```bash
export DSH_ARRIVE_DEG=1.2      # 门控放宽（缺省 0.8）
export DSH_ARRIVE_FRAMES=2     # 连续帧放宽（缺省 3）
export DSH_LEAD=0.057          # 开补偿（缺省 0 = 关）；★ 必须重启进程才生效
export DSH_FIRE_KEY_ENABLE=1   # 人工门（缺省 0 = 关）
export DSH_LOW_PLATE=1         # 只打低位靶三态：0/off、1/warmup（缺省）、2/always
```

> **三条铁律**（评审修订 3/5/9）：
> ① `DSH_LEAD` / `DSH_FIRE_DELAY` 是 `static` 一次性读取的 ⇒ **改完必须重启进程**；
> ② **57 ms 只允许存在于 `DSH_LEAD` 一处** ⇒ 用 `DSH_FIRE_DELAY` 全包出弹延时就要把 `DSH_LEAD` 置 0；
> ③ `DSH_LOW_PLATE` 是**三态字符串**（0/off、1/warmup、2/always），不是布尔 ⇒ 别写 `DSH_LOW_PLATE=1` 之外的东西，
>    启动日志会把**原字符串和解析结果**都打出来，一眼可查。
