# 现场调试流水线 v2

> 依据：`src/task_2.cpp` 618 行那版（facing 选板 + 四道开火闸 + lead 符号修正）+ `configs/standard.yaml` + `tools/extended_kalman_filter.{hpp,cpp}`。
> 旧 SOP（`troubleshooting_sop.md`）讲「为什么会这样」，这份只讲「按什么顺序做、卡住怎么办」。
> 全文代码块标记：`powershell` = 笔记本，`bash` = 车机（或笔记本 `ssh ... "..."` 里的部分）。

---

## 0. 三句话读完

1. **先传代码**（第 1 节）——上次漏了这步，车上跑的可能是旧版；
2. **先证明能打出去**（P0–P4），再谈打中（P5–P7）；
3. **卡住就看决策树**（第 4 节），每一步只允许动一个变量。

---

## 1. S0 传代码（上次漏的第一步）

### 1.1 要传的文件（15 个）

| 文件 | 为什么 |
|---|---|
| `src/task_2.cpp` | 四道开火闸、facing 选板、lead 修正 |
| `src/overlay.hpp` | HUD 状态条 |
| `src/detect_log.hpp` | 检测 CSV（**新增文件**，不传就编不过） |
| `src/latency_probe.hpp` | 延迟探针（**新增文件**） |
| `src/task_3.cpp` | 前哨站同样加了 `has_detection` |
| `tasks/auto_aim/armor.cpp` | 颜色映射 0 红 1 蓝 |
| `tasks/auto_aim/target.cpp` | NIS 拒收回滚 |
| `io/camera.cpp` `io/camera.hpp` | 设备时间戳 + 录像开关 |
| `io/gimbal/gimbal.cpp` | pitch 取反一致性 + `DSH_FIRE_MODE` |
| `io/hikrobot/hikrobot.cpp` `io/hikrobot/hikrobot.hpp` | 取帧时间戳 |
| `configs/standard.yaml` | 0.2 s 间隔、offset 0、`record_dir: ""` |
| `tools/extended_kalman_filter.cpp` `tools/extended_kalman_filter.hpp` | 若车上还是 `nis_threshold = 1.5`，一起传 |

> **车机不是 HEAD 时再加两个**：`io/gimbal/gimbal.hpp`（5 参 `send()`）和 `src/task_common.hpp`（`calc_aim` 签名）。缺 `gimbal.hpp` 会在 `gimbal.send(…, target)` 处直接编译失败。

### 1.2 传（笔记本）

```powershell
cd D:\DSH\fp_aim
$car = "rm@192.168.1.50"
scp src\task_2.cpp src\task_3.cpp src\overlay.hpp src\detect_log.hpp src\latency_probe.hpp ${car}:~/fp_aim/src/
scp tasks\auto_aim\armor.cpp tasks\auto_aim\target.cpp ${car}:~/fp_aim/tasks/auto_aim/
scp io\camera.cpp io\camera.hpp ${car}:~/fp_aim/io/
scp io\gimbal\gimbal.cpp ${car}:~/fp_aim/io/gimbal/
scp io\hikrobot\hikrobot.cpp io\hikrobot\hikrobot.hpp ${car}:~/fp_aim/io/hikrobot/
scp configs\standard.yaml ${car}:~/fp_aim/configs/
scp tools\extended_kalman_filter.cpp tools\extended_kalman_filter.hpp ${car}:~/fp_aim/tools/
```

> `scp` 一次传多个文件时，目标必须是已存在的目录。先确认车上有 `~/fp_aim/tasks/auto_aim/`（有，原工程就在那）。

### 1.3 验收（车机，5 条 grep，一条都不能空）

```bash
cd ~/fp_aim
grep -c 'plate_facing_deg' src/task_2.cpp        # 期望 4
grep -c 'target_static' src/task_2.cpp           # 期望 2
grep -c 'DSH_LEAD' src/task_2.cpp                # 期望 3（两处注释 + 一处 getenv）
grep -n 'fire_interval_s' configs/standard.yaml  # task2 段必须是 0.2
grep -nE '^(yaw_offset|pitch_offset):' configs/standard.yaml   # 期望 25/26 行，两个 0.0
```

`grep -c` 匹配 0 行时返回码是 1，**别用 `&&` 串**，用 `;` 或分行敲。

### 1.4 编译

```bash
cd ~/fp_aim
touch src/task_2.cpp src/task_3.cpp tasks/auto_aim/armor.cpp tasks/auto_aim/target.cpp \
      io/gimbal/gimbal.cpp io/camera.cpp io/hikrobot/hikrobot.cpp
cd build && make -j2 2>&1 | tail -20
```

**通过判据**：出现 `Building CXX object .../task_2.cpp.o` 等多行 + 最后 `Linking CXX executable ../task_2`。
只出现 `Built target task_2` 说明没重编——`touch` 没生效，重来。

---

## 2. 流水线 P0–P7

| 阶段 | 目标 | 命令要点 | 通过判据 | 失败去 |
|---|---|---|---|---|
| **P0** | 能启动 | `./build/task_2 configs/standard.yaml 2>&1 \| tee logs/p0.log` | 恰好两行 `[task2]` 启动日志：`offset yaw=… pitch=…`（`:78`）和 `到位门控 = …｜看门狗 …`（`:159`）。**没有「开火闸」这行，别去找** | D-7 |
| **P1** | 看得见 | 画面有检测框、`has_target`=true | 框跟着靶子走（不是固定位置） | D-0 |
| **P2** | 挡位对 | 看 `[dbg] 电控挡位 = ...` | `AUTO_AIM(自瞄)`，不是 `IDLE` | D-3 |
| **P3** | 云台到位 | `grep -m3 'arrive' logs/p0.log` | `errY` < 门限、`blocked` 不持续涨 | D-2 |
| **P4** | lead 符号 | `DSH_LEAD=0` 与 `0.57` 各跑 10 s，比 HUD `AIM yaw` | `w>0` 时后者小约 8.3° | D-4 |
| **P5** | 静止靶零点 | 静止靶打 3 发 | 落点距靶心 < 1 靶宽 | D-5 |
| **P6** | 旋转靶 | 4 rad/s 打 3 发 | 中 1 发以上即通过 | D-4/D-6 |
| **P7** | 记录 | `runlog.py` + `collect_run.sh` | 三个数进表 | — |

**铁律**：P3 没过不许打 P5，P4 没过不许打 P6。每换一个变量，重跑一次 P0（确认启动日志里的参数是新的）。

---

## 3. D-0 到 D-7 决策树

每个节点只问一个问题，只跑一条命令。

### D-0 云台完全不动（`has_target` = false）

```bash
grep -c '视野里没有 red\|视野里没有 blue' logs/p0.log   # 多 → 颜色/曝光
grep -m3 'detect' logs/p0.log | head -3
```

| 判据 | 结论 | 处置 |
|---|---|---|
| 大量「视野里没有 red…」 | 颜色反了 | `DSH_COLOR_ORDER=blue` 试一场（A/B 看哪句变少） |
| 有框但 `has_target=false` | 关联阈值太严 | yaml `min_confidence` 0.8 → 0.7 |
| 完全没框 | ROI/曝光/相机 | 关 ROI（yaml `use_roi: false`）；查相机是否 `0x80000006` |

### D-1 跟得住但一次都不发（`fire-gate` 有项 false）

```bash
grep -m6 'fire-gate' logs/p0.log | tail -6
```

按 **出现顺序** 排查，前一项过了才看下一项：

| 字段 | false 的含义 | 处置 |
|---|---|---|
| `stable=False` + `nis=0.8~1.0` | NIS 失败率被顶满 | 等 1.7 秒（窗口 100 帧）再观察；持续如此说明关联错板 |
| `frames=3/10` 长期不动 | 目标在丢帧，`frames_since_init` 反复重置 | D-0 的丢帧分支 |
| `interval=False` | 刚打过，正常 | 等 0.2 s |
| `mode=False` | **电控不在 AUTO_AIM** | D-3 |
| `hold=True` | 目标丢失但还在保持期 | 正常，等 `target_hold_ms` 到期 |
| `plate=False(30°)` | 板不在 ±25° 窗口 | 等它转进来（窗口宽 50°/229°/s ≈ **0.22 s**，最长等一个板周期 0.39 s） |
| `plate=False` 且角度一直在 ±60~±120° | **facing 约定反了** | 见 D-4 的约定检查 |
| 全部 True 但仍不发 | 门控 | D-2 |

### D-2 门控挡住（`arrive=False`，`blocked` 持续涨）

```bash
grep -m6 'arrive' logs/p0.log | tail -6
```

| 判据 | 结论 | 处置 |
|---|---|---|
| `errY` 稳定在 0.8~1.5，`blocked` 涨 | 门限太紧 | `DSH_ARRIVE_DEG=1.2` 重启 |
| `errY` > 2° | 云台跟不上或零位偏 | 先降转速到 3 rad/s 再看 |
| `errP` 一直 > 1.28 | **pitch 符号约定错**（相加 ≠ 0） | 看 `fb pitch` 与 `cmd pitch`：同号说明要改成减法 |
| `tripped=1` | 看门狗已解除（默认关，只有显式设了才可能） | 忽略 |

**门限上限 1.2°**：板半宽 1.23°（3.15 m），门限超过它等于允许「注定打不中」的弹放出去。

### D-3 挡位不是 AUTO_AIM

```bash
grep -m2 '电控挡位' logs/p0.log
```

| 值 | 含义 | 处置 |
|---|---|---|
| `AUTO_AIM` | 正常 | — |
| `IDLE` | 电控没接管 | 遥控器拨到自瞄挡；确认 ECP 上电顺序 |
| 其他 | 未知挡位 | 同上，别改代码 |

### D-4 开了火但不中，或方向明显不对

先分清是「差半块板」还是「差好几块板」：

| 现象 | 量级 | 病因 | 处置 |
|---|---|---|---|
| 偏 ~0.5 个靶宽，方向一致 | 0.8~1.2° | 延迟未补 | 开 `DSH_LEAD=0.057` |
| 偏 3~5 个靶宽，方向一致 | 5~12° | 零位 / 偏置 | 查 offset 是否为 0；静止靶重标 |
| 偏多少没规律 | — | 关联错板 / EKF 跳变 | 看 `assoc=` |
| 左右**反向** | — | facing 约定 | `assoc=` 长期 ±150° ⇒ 删 `src/task_2.cpp:383` 的 `+ M_PI` |

**lead 符号验证（不发弹，10 秒）**：

```bash
DSH_LEAD=0   ./build/task_2 configs/standard.yaml   # 记 HUD 的 AIM yaw 平均值
DSH_LEAD=0.57 ./build/task_2 configs/standard.yaml  # 再记一次
```

`w>0` 时第二场应**小约 0.057×14.5°/s×10 ≈ 8.3°**（0.57 是 10 倍放大，方便肉眼）。变大就是符号还反。

### D-5 静止靶也打不准

```bash
py tools\zero_fix.py --d 3.15 --dx <左右 mm> --dy <上下 mm>
```

用输出的两个角度改 yaml 的 `yaw_offset` / `pitch_offset`，**不要用环境变量**（会忘）。改完重跑 P0 看启动日志里的值。

### D-6 哑炮（`rounds` 涨了但没弹）

```bash
grep -m3 'fire-gate' logs/p0.log | tail -3     # 看 bullet_alive=
```

| `bullet_alive` | 含义 | 处置 |
|---|---|---|
| `true` | 电控回报发弹数，`rounds` 是真实发射数 | 真哑炮：查弹仓/摩擦轮/超容 |
| `false` | 软件自计，**打没打出去它不知道** | 改计数口径没用，用眼睛数弹壳 |

### D-7 跑不起来 / 掉帧

| 现象 | 处置 |
|---|---|
| 启动就段错误 | 少传了 `overlay.hpp`/`detect_log.hpp`/`latency_probe.hpp`，看 S0 |
| 帧率从 58 掉到 44 | 录像开着，或 `DSH_DET_CSV` 每帧写盘 |
| `MV_CC_EnumDevices 0x80000006` | 相机被别的进程占用：`pkill task_2`；USB 重插 |

---

## 4. 六个问题：能保证什么、我预料的问题、预案

### Q1 能开火 —— 能，但有 5 个「静默不出弹」

| 预料到的问题 | 判据 | 预案 |
|---|---|---|
| 挡位 IDLE | `mode=False` | D-3 |
| 门限 0.8 骑在实测 0.64~1.51 中间 | `arrive=False`、`blocked` 涨 | `DSH_ARRIVE_DEG=1.2` |
| facing 约定反了 | `plate=False` 且角度 ±60~120° | 删 `+ M_PI` |
| NIS 窗口被顶满（1.7 s） | `nis=1.00` | 等；持续则查关联 |
| 板还没进窗口 | `plate=False(90°)` | 等 0.1 s |

### Q2 单个目标只开火一次 —— 成立，语义是「每块板每圈一发」

| 预料到的问题 | 判据 | 预案 |
|---|---|---|
| `w` 反向/爬升被误判成静止靶 → 连打 | `[dbg]` 里 `w` 接近 0 时 `rounds` 突增 | 见第 6 节补丁 2 |
| EKF 重初始化导致 `aim_id` 跳变 | 日志 `diverged` 或 `assoc` 突跳 | 无法完全消除，可接受 |
| 4 块板一圈 4 发，不是整场 1 发 | 设计如此 | 若要字面「整场一发」，改 `plate_armed` 的重置条件 |
| 实际节奏 **2.55 Hz**（一圈 1.571 s ÷ 4 发），3.5 Hz 只是上限 | 按 3.5 Hz 估弹药会少算 1/3 时间 | — |

### Q3 只在时机最完美时开火 —— 三道门成立，残差在 1 帧

| 预料到的问题 | 判据 | 预案 |
|---|---|---|
| lead 默认**是关的** | `DSH_LEAD` 没 export | 显式 `DSH_LEAD=0.057` |
| 开火位与角度同包 → 1 帧残差 | 无法从日志看 | 用 lead 一起吃掉（0.057 已含 25+32 ms） |
| 门控量化到 3 帧（≈50 ms） | 只是推迟，不改方向 | 不改 |

### Q4 延迟补偿正常吗 —— 公式对，符号未验证，量级只有 0.83°

| 预料到的问题 | 判据 | 预案 |
|---|---|---|
| 符号反 | `w>0` 时 yaw 变大 | D-4 的 10 秒验证 |
| `h < 0.5` 时跳过补偿 | 距离 < 0.5 m 才触发 | 现场不会 |
| `r` 只有 0.2 m，补偿量小 | 0.83° ≈ 0.34 靶宽 | 别指望它救命 |
| 真实出弹延迟未测 | 斜率法标定 | P4 之后做一次 |

### Q5 会抖动吗 —— 瞄准不抖，选板在临界点会闪

| 预料到的问题 | 判据 | 预案 |
|---|---|---|
| `aim_id` 每帧被重置成 `last_id`，5° 迟滞不跨帧 | 准星在两块板间跳 | 第 6 节补丁 1 |
| PnP 在 ±25° 边界抖 | `facing` 在 24/26 反复 | 不改，窗口本来就该在边界抖 |
| EKF 重初始化让瞄准点跳 | `AIM yaw` 突跳几度 | 门控会挡住不发，只是慢一拍 |

### Q6 会哑炮吗 —— 两种哑炮，一种已堵死

| 类型 | 状态 | 判据 |
|---|---|---|
| 「请求了但电控不执行」 | 已堵死（`require_auto_aim_mode: true`） | `mode=True` 才会请求 |
| 「该打却一直不打」 | **风险变大**（看门狗默认关） | `blocked` 涨、`rounds` 不动 |
| 「软件以为打了」 | 仅在 `bullet_alive=false` 时 | 看日志 `bullet_alive=` |

---

## 5. 审核报告里要撤回的一条、要保留的两条

### 5.1 撤回：我此前判「每帧 0.25°」是错的 —— 判错了

- **3.9°/帧** = 靶柱转过的**相位**（决定 `facing` 变化多快、板多久出窗口）；
- **0.25°/帧** = 瞄准点的**视在位移**（`r·w/d ÷ 58 fps`，决定 `aim_yaw` 每帧动多少）。

到位门限比的是 `aim_yaw` 与反馈的**绝对角差**，所以「每帧 0.25°」这个数本身没错，错的是我拿相位率去否定它。

真正要纠正的只有推论：0.25°/帧 ⇒ 0.8° 的门限 ≈ **3 帧的视在位移**，并不构成「门限太松」。

### 5.2 保留：这两条不用追

| 报告的说法 | 为什么不成立 | 追它会怎样 |
|---|---|---|
| 「A→B→A 跳板要堵」 | 任一时刻最多一块板在 ±25° 内，两块同时进窗口几何上不可能 | 加「已打集合」→ 状态变多、发弹更少 |
| 「`aim_id ≠ chosen` 会打到空处」 | 炮管指的就是 `aim_id` 的瞄准点，那是真实的板 | 去改检测关联 → 引入新 bug |

**共同点**：都在解释一个「多打了一发」或「偏了一点」的现象，而现场的主要矛盾是「打不出去」和「偏几个靶宽」。

---

## 6. 三处值得改、但还没改的代码（补丁）

### 补丁 1：让 5° 迟滞真正跨帧（`src/task_2.cpp:373`）

`static` 变量只判越界不够——**换目标必须三清**，否则新目标带着上一台的选板记忆。

```cpp
static int aim_id_persist = -1;
static const void * last_target = nullptr;
if (last_target != &target) {          // 换目标：三清
  last_target = &target;
  aim_id_persist = -1;
  last_fired_plate = -1;
  facing_still_frames = 0;
}
int aim_id = (aim_id_persist >= 0 && aim_id_persist < static_cast<int>(xyza_list.size()))
               ? aim_id_persist : target.last_id;
...
aim_id_persist = aim_id;               // 帧末回写
```

### 补丁 2：静止靶判定改成量「面朝角变化」（`src/task_2.cpp:412`）

`|x[7]| < 0.15` 在起步爬升和 w 反向过零时都不可信，而那正是「一块板一发」失效、5 Hz 连打同一块的时刻。改成直接量 `facing` 的变化，不依赖 `w`：

```cpp
static int facing_still_frames = 0;
const double d_facing = std::abs(facing_deg - last_facing_deg);
facing_still_frames = (d_facing < 0.5) ? facing_still_frames + 1 : 0;
const bool target_static = facing_still_frames >= 10;   // ≈ 0.17 s
```

### 补丁 3：HUD 第二行显示真正瞄的板（改 `src/task_2.cpp`，不动 `overlay.hpp`）

`overlay.hpp` 里**没有** `draw_overlay` 这个函数，真实接口是 `overlay::Hud::draw(img, armors, chosen, frame, has_target, holding)`。别改那个类，直接在 task_2 里追加一行：

```cpp
tools::draw_text(img, cv::format("aim={} facing={:.0f}° armed={} static={}",
                   aim_id, facing_deg, plate_armed, target_static),
                 {8, 96}, {0, 255, 255}, 0.6);
```

### 补丁 4（配套）：单独记一行「选板变化」

临界窄带只有 0.15 s，而 `fire-gate` 每 48 帧（≈0.8 s）才打一次，不单独记根本看不见：

```cpp
if (aim_id != last_logged_aim_id) {
  tools::logger()->info("[dbg] 选板 {} → {} facing={:.1f}° armed={} static={}",
                        last_logged_aim_id, aim_id, facing_deg, plate_armed, target_static);
  last_logged_aim_id = aim_id;
}
```

四条都只改「可观测性」和「临界稳定性」，不动行为主干；**一次只改一个，改完跑一次 P0–P3**。

---

## 7. 判据速查

| 量 | 值 | 备注 |
|---|---|---|
| 板宽 / 半宽 @3.15 m | 2.46° / 1.23° | 135 mm 板 |
| 板宽 / 半宽 @3.0 m | 2.58° / 1.29° | 旧数据常用 3 m |
| 视在角速度 @4 rad/s | ≈14.5°/s | `r·w/d`，不是 229°/s |
| lead 0.057 s 的补偿量 | 0.83° ≈ 0.34 靶宽 | |
| 到位门限 | 0.8° 缺省，**上限 1.2°** | 超过半宽就是放空 |
| 连续帧 | 3 帧 ≈ 50 ms | |
| 开火节奏 | 上限 5 Hz（0.2 s） | 4 块板一圈 4 发 = 平均 **2.55 Hz**（一圈 1.571 s） |
| `min_frames_for_fire` | 10 帧 ≈ 172 ms | |
| NIS 窗口 | 100 帧 ≈ 1.7 s | 拒收后要等它淡出 |
| 面朝窗口 | ±25°，重上膛 35° | 死区 10° |

---

## 8. 记录与回退

### 记录

```powershell
py tools\runlog.py new r1020_p6 --task task_2
py tools\runlog.py set r1020_p6 shots=3 hits=1 lead=0.057 arrive_deg=1.2
py tools\runlog.py show
```

### 回退

**笔记本**（有 git，HEAD 就是当前版本）：

```powershell
cd D:\DSH\fp_aim
git diff --stat
git checkout -- src/task_2.cpp        # 只回退这一个文件
```

**车机**（也有 git，但它的 HEAD 可能比笔记本旧很多，`checkout` 会退回到一个更早的版本，等于把整轮修改一起丢掉）：

```bash
cd ~/fp_aim
ls -l src/task_2.cpp.bak*              # 先确认有 19:57 那版的备份
cp src/task_2.cpp.bak_aim src/task_2.cpp
cd build && make -j2 task_2
```

`git checkout -- configs/standard.yaml` 会连带回退 ROI、`min_confidence`、`fire_interval_s`，动手前先 `cp configs/standard.yaml configs/standard.yaml.bak.$(date +%H%M)`。

### 三个变量决定「能不能打出去」

```bash
export DSH_ARRIVE_DEG=1.2      # 门控放宽（缺省 0.8）
export DSH_ARRIVE_FRAMES=2     # 连续帧放宽（缺省 3）
export DSH_LEAD=0.057          # 开补偿（缺省 0 = 关）
```

三个都不设时：门控 0.8°、3 帧、**lead 关闭**——这就是「代码改对了但默认还是打不准」的原因。
