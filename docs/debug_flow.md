# 现场调试手册（合并重写版，2026-10-21）

> **对象**：`D:\MTG\Documents\OC\fp_aim_fkey`（副本）；原工程 `D:\DSH\fp_aim` 的 `debug_flow_v2.md` 是它的前身。  
> **本手册取代**：`debug_flow_v2.md`（原工程版）、`debug_flow_v3_副本.md`、`debug_flow_v2_勘误表.md`  
> —— 三份里的**勘误已全部修掉**，副本的新机制已全部并入，数字全部按代码与实测重新核过。
>
> > **代码位置**：`src/task_2.cpp` 1024 行 / `src/task_3.cpp` 375 行> >   
> > **标记**：`powershell` = 笔记本，`bash` = 车机。
>
> **⚠ 主文档是《现场测量清单.md》**：那份以**数据测定**为主干（M0–M10：每一步测什么、什么时候测、>   
> 测出来的数决定哪个参数、谁依赖谁、顺序为什么这样）。>   
> **本文件是它的配套**：代码/日志层面的排障细节（决策树 D-0…D-11、屏显三件套、判据速查）。

---

## 0. 三十秒版

1. **先传代码**（§1）——车上跑的可能是旧版；**再验按键**（§2）——副本多了一道人工开火门；
2. **先证明能打出去**（P0–P4），再谈打中（P5+）；**P5 是对照组**，后面每轮只改一个变量；
3. 卡住看决策树（§5），**每一步只动一个变量**；
4. 改 `DSH_LEAD` / `DSH_FIRE_DELAY` / `DSH_STATIC_*` 必须**重启进程**（它们是 `static` 一次性读取）。

### 0.1 默认行为速查（上车前先知道"不设环境变量会怎样"）

| 机制                          | 默认                  | 不设的后果                                          |
| --------------------------- | ------------------- | ---------------------------------------------- |
| 到位门控                        | 开，0.8°/1.28°，连续 3 帧 | 0.8° 比官方严 2.4 倍 ⇒ 可能永远不开火 ⇒ 一发不发               |
| 链路延迟补偿 `DSH_LEAD`           | **0（关）**            | 4 rad/s 下系统性滞后 0.83°（1/3 个靶宽）                  |
| 出弹延时 `DSH_FIRE_DELAY`       | **0（关）**            | 摩擦轮那 0.06~~0.1 s 没补 ⇒ 偏晚 0.9~~1.5°（>板半宽 1.23°） |
| 到达时刻选板 `DSH_AIM_SELECT`     | `current`           | 打的是"现在正对"而非"到弹时正对"的板，4 rad/s 下差 29°            |
| 低位靶 `DSH_LOW_PLATE`         | `1`（warmup）         | 高度差没学准时只打低位（无害）；学准后自动放开                        |
| 人工开火门 `DSH_FIRE_KEY_ENABLE` | **0（关）**            | 自动开火（正式提交的安全默认值）                               |
| 发弹间隔 / 到达容限                 | 0.2 s / 固定 0.8°     | 与原工程一致                                         |
| 软件发数上限                      | `0` = 不限            | **10 发额度全靠人守**（`rounds` 只作参考）                  |

---

## 1. S0 传代码

### 1.1 传哪些（**21 个源文件** + 5 个脚本）

| 文件                                                                    | 为什么                                                  |
| --------------------------------------------------------------------- | ---------------------------------------------------- |
| `src/task_2.cpp`                                                      | **副本版**：四道开火闸 + 到位门控 + 低位靶 + 人工门 + 到达时刻选板            |
| `src/task_3.cpp`                                                      | 副本版（F 键门 + 出弹延时；低位靶已摘除）                              |
| `src/task_common.hpp`                                                 | `calc_aim` 的 `t_extra`（出弹延时）                         |
| `src/fire_key.hpp`                                                    | **新增**：F 键门（四级取键后端）                                  |
| `src/low_plate.hpp`                                                   | **新增**：低位靶三态 + 信任状态机                                 |
| `src/env_util.hpp`                                                    | **新增**：环境变量小工具                                       |
| `src/test_summary.hpp`                                                | **新增**：测试汇总器（退出时统一输出 A/B/C 三组；**不传直接编不过**）           |
| `src/overlay.hpp` `src/detect_log.hpp` `src/latency_probe.hpp`        | HUD / 检测日志 / 延迟探针（不传编不过）                             |
| `tasks/auto_aim/armor.cpp`                                            | 颜色映射 0 红 1 蓝                                         |
| `tasks/auto_aim/target.cpp`                                           | NIS 拒收回滚 + 前哨站分层                                     |
| `io/camera.cpp` `io/camera.hpp`                                       | 设备时间戳 + 录像开关                                         |
| `io/gimbal/gimbal.cpp`                                                | pitch 取反一致性 + `DSH_FIRE_MODE`                        |
| `io/hikrobot/hikrobot.cpp` `io/hikrobot/hikrobot.hpp`                 | 取帧时间戳                                                |
| `configs/standard.yaml`                                               | 0.2 s 间隔、offset 0.0、`record_dir: ""`                 |
| `tools/extended_kalman_filter.cpp` `tools/extended_kalman_filter.hpp` | NIS 阈值 9.488（**官方是 1.5，是错的**）                        |
| `tools/keytest.cpp`                                                   | **新增**：按键通道自测（`./build/keytest`）                     |
| `tools/parse_session.py` `tools/lead_solve.py` `tools/make_record.py` | **新增**：日志判读 / 斜率法定标 / 记录表生成（**都在笔记本上跑**，车机没 python3） |
| `tools/onsite/run_measure.sh`                                         | **新增**：车机上一条龙（预检/测量/汇总/拷回命令）                         |

> `io/gimbal/gimbal.hpp`、`src/task_common.hpp`、`tasks/auto_aim/target.hpp` 在 HEAD 里已含新版，  
> **车机若不是 HEAD**，`gimbal.hpp` 必须一起传（`send()` 的第 5 参 target 在它里面）。

### 1.2 传：**打成一个包，一条命令过去**（推荐）

```powershell
# ① 笔记本：打包（清单的唯一来源 = tools\onsite\pack_manifest.txt）
cd D:\MTG\Documents\OC\fp_aim_fkey
powershell -ExecutionPolicy Bypass -File tools\onsite\make_pack.ps1
#    ⇒ 生成 D:\DSH\fp_aim_patch_MMDD_HHMM.tar.gz，并打印 sha256 与下面两条命令

# ② 传到车机（笔记本有线 IP 192.168.1.43；车机没外网，只有这条网线）
scp D:\DSH\fp_aim_patch_*.tar.gz rm@192.168.1.50:~/
#    网线不通就用 U 盘：把这个 tar.gz 拷到 U 盘，车机上再 cp 过去

# ③ 车机：解压 + 校验 + 编译 + 验收（一条命令）
cd ~/fp_aim && tar -xzf ~/fp_aim_patch_*.tar.gz && bash tools/onsite/pack/install.sh
```

**包里有什么**（27 个文件，路径相对工程根，解压即覆盖）：  
`src/` 的 10 个（task_2 / task_3 / task_common / fire_key / low_plate / env_util / test_summary / overlay / detect_log / latency_probe）、  
`tasks/auto_aim/` 2 个、`io/` 5 个、`tools/` 4 个 + `tools/onsite/` 2 个、`configs/standard.yaml`、`CMakeLists.txt`，  
外加 `tools/onsite/pack/{manifest.sha256, manifest.txt, install.sh}`。

**`install.sh` 在车机上做四件事**（冒烟测试通过）：

1. 用包里的 `manifest.sha256` 逐个校验 sha256（**包没传全会当场报出来**）；
2. 确认清单里 27 个文件都在（少一个就编不过）；
3. 编译 `task_2` 与 `keytest`（改了 `CMakeLists.txt` 会自动重新 cmake）；
4. 跑下面那 8 条验收 grep。

> `io/gimbal/gimbal.hpp`、`tasks/auto_aim/target.hpp` 在 HEAD 里已含新版，>   
> **车机若不是 HEAD**，`gimbal.hpp` 要另外补传（`send()` 的第 5 参 target 在它里面）——>   
> 它不在包里，因为它相对 HEAD 没改。

### 1.2b 逐文件传（不想打包装时的退路）

```powershell
cd D:\MTG\Documents\OC\fp_aim_fkey
$car = "rm@192.168.1.50"
scp src\task_2.cpp src\task_3.cpp src\task_common.hpp src\fire_key.hpp src\low_plate.hpp src\env_util.hpp src\test_summary.hpp `
    src\overlay.hpp src\detect_log.hpp src\latency_probe.hpp  ${car}:~/fp_aim/src/
scp tasks\auto_aim\armor.cpp tasks\auto_aim\target.cpp            ${car}:~/fp_aim/tasks/auto_aim/
scp io\camera.cpp io\camera.hpp                                   ${car}:~/fp_aim/io/
scp io\gimbal\gimbal.cpp                                          ${car}:~/fp_aim/io/gimbal/
scp io\hikrobot\hikrobot.cpp io\hikrobot\hikrobot.hpp             ${car}:~/fp_aim/io/hikrobot/
scp configs\standard.yaml                                         ${car}:~/fp_aim/configs/
scp tools\extended_kalman_filter.cpp tools\extended_kalman_filter.hpp ${car}:~/fp_aim/tools/
scp tools\keytest.cpp tools\parse_session.py tools\lead_solve.py tools\make_record.py  ${car}:~/fp_aim/tools/
scp tools\onsite\run_measure.sh                              ${car}:~/fp_aim/tools/onsite/
```

> `scp` 一次传多个文件时，目标必须是**已存在**的目录；车机上 `~/fp_aim/io/gimbal/`、`~/fp_aim/tools/` 都在>   
> （原工程就在那）。单文件的 `io/gimbal/gimbal.cpp` 单独传，路径别写成 `~/fp_aim/gimbal/`。

### 1.3 验收（车机 bash，8 条 grep + 2 条 yaml，一条都不能空）

> `install.sh` 第 4 步会自动跑这 8 条；手动复核时照抄。

```bash
cd ~/fp_aim
grep -c 'plate_facing_deg' src/task_2.cpp        # 期望 >= 4
grep -c 'target_static'    src/task_2.cpp        # 期望 >= 2
grep -c 'DSH_LEAD'         src/task_2.cpp        # 期望 >= 7
grep -c 'fire_key'         src/task_2.cpp        # 期望 >= 20
grep -c 'low_plate'        src/task_2.cpp        # 期望 >= 9
grep -c '选板变化'         src/task_2.cpp        # 期望 >= 1（副本才有）
grep -c 'test_summary'       src/task_2.cpp        # 期望 >= 4（副本才有：退出/按 s 统一输出汇总）
grep -nE '^(yaw_offset|pitch_offset):' configs/standard.yaml   # 恰好 25、26 两行，都是 0.0
grep -n 'fire_interval_s'  configs/standard.yaml               # task2 段必须是 0.2
```

> `grep -c` 匹配 0 行时返回码是 1，**别用 `&&` 串**，用 `;` 或分行敲。

### 1.4 编译

```bash
cd ~/fp_aim
touch src/task_2.cpp src/task_3.cpp src/task_common.hpp src/fire_key.hpp src/low_plate.hpp src/env_util.hpp
cd build && cmake -B . .. >/dev/null 2>&1     # 第一次要重新配置（新增了 keytest 目标）
make -j2 2>&1 | tail -20
```

**通过判据**：出现 `Building CXX object .../task_2.cpp.o` + `Linking CXX executable .../task_2`；  
只出现 `Built target task_2` = 没重编，`touch` 没生效。  
**新增目标**：`./build/keytest`（按键自测，下一节用）。

---

## 2. S0.5 验按键通道（副本必做）

副本多了一道"人按住 F 才开火"的门。按键通道不通 ⇒ `key=False` ⇒ **一场都不发**，  
而其它日志字段全是 True ⇒ 极易被误判成"程序坏了"。

```bash
cd ~/fp_aim && ./build/keytest
# 没有这个目标就：g++ -O2 -std=c++17 -I. tools/keytest.cpp -o /tmp/keytest && /tmp/keytest
```

期望：

```
backend = evdev:/dev/input/event3 /dev/input/event4      ← 车机上最常见
[允许开火] F=按住        ← 按住 F
[禁止开火] F=松开        ← 松开 F
```

| backend            | 含义                             | 处置                       |
| ------------------ | ------------------------------ | ------------------------ |
| `evdev:…`          | ✅ 真按住（能读按下/松开），**不需要窗口焦点**     | 直接用                      |
| `termios(raw)`     | ⚠️ 靠键盘自动重复近似"按住"               | 建议 `DSH_FIRE_KEY=toggle` |
| `cvkey(window)`    | ⚠️ 只能靠 OpenCV 窗口收键，**要求窗口有焦点** | 见下                       |
| `always` / `never` | 环境变量把门焊死了                      | `unset DSH_FIRE_KEY`     |

`cvkey(window)` 的处置（车机常是远程/X11 转发，窗口没焦点时收不到键）：

```bash
ls -l /dev/input/event*                      # 设备在不在、权限对不对
cat /proc/bus/input/devices | grep -A3 keyboard
sudo usermod -aG input $USER                 # 或者直接指定设备
DSH_FIRE_KEY_DEV=/dev/input/event3 ./build/keytest
```

还是不行就 `DSH_FIRE_KEY=toggle`（按一次切一次，不依赖按住）。

---

## 3. 启动日志怎么读（**四行 + 一行时域**）

```
[task2] offset yaw=0.00(yaml) pitch=0.00(yaml)  源: yaml 值 0.00/0.00
[task2] 到位门控 = 开（yaw 0.80° / pitch 1.28°，连续 3 帧）｜看门狗 关｜板面正对 ±25°、摆出 35° 才重新上膛（一块板只打一发）｜发弹间隔 0.20 s
[task2] 低位靶规则 = warmup(暖机期只打低位，可信后放开)（DSH_LOW_PLATE="1"，高度容差 0.10 m，信任 4 s）｜人工开火门 = 关（模式 hold，按键后端 evdev:…）
[task2] 出弹延时（记录值，不单独累加）= 0.020 s｜选板 = 当前(current)｜到位容限 = 0.80°（固定角度）｜静止靶判据 = 30 °/s 且连续 10 帧
[task2] 预测时域 = 总提前 0.080s（其中出弹延时约 0.020s，仅记录不单独累加）+ 飞行 ~0.126s（3.15 m @25 m/s 估）
```

| 行           | 必看什么                                     | 不对怎么办                                                          |
| ----------- | ---------------------------------------- | -------------------------------------------------------------- |
| `offset`    | yaw/pitch 都是 **0.00**、源是 `yaml`          | 非 0 ⇒ yaml 被改过；`env` ⇒ 有人在环境里塞了 `DSH_*_OFFSET`                 |
| `到位门控`      | 「开」+ 门限数值                                | 显示「关」⇒ `DSH_ARRIVE_DEG=0` 被设了                                  |
| `低位靶规则`     | **模式 + `DSH_LOW_PLATE` 的原字符串**（三态，别写成别的） | 原字符串与模式对不上 ⇒ 解析回落了                                             |
| `人工开火门`     | 开/关 + **按键后端**                           | 后端是 `cvkey` ⇒ 回去做 §2                                           |
| `出弹延时（记录值）` | 三个数值                                     | —                                                              |
| **`预测时域`**  | **总提前 = `DSH_LEAD`（唯一功能旋钮）**；出弹延时只作拆分记录  | **`DSH_FIRE_DELAY` 非 0 而 `DSH_LEAD=0` ⇒ 程序会打一条 ⚠ 警告：出弹延时没进补偿** |

---

## 4. 流水线 P0–P10

| 阶段       | 目标      | 命令要点                                                                            | 通过判据                                                        | 失败去         |
| -------- | ------- | ------------------------------------------------------------------------------- | ----------------------------------------------------------- | ----------- |
| **P0**   | 能启动     | `./build/task_2 configs/standard.yaml 2>&1 \| tee logs/p0.log`                  | §3 的五行日志都在 + 模型加载完                                          | D-7         |
| **P0.5** | 按键通     | `./build/keytest`                                                               | backend 是 evdev/termios，按住 F 状态会变                           | D-8         |
| **P1**   | 看得见     | 画面有检测框、HUD `tgt YES`                                                            | 框跟着靶子走（不是固定位置）                                              | D-0         |
| **P2**   | 挡位对     | `grep -m2 '电控挡位' logs/p0.log`                                                   | `AUTO_AIM(自瞄)`，不是 `IDLE`                                    | D-3         |
| **P3**   | 云台到位    | `grep -m3 'arrive' logs/p0.log`                                                 | `errY` < 门限、`blocked` 不持续涨                                  | D-2         |
| **P4**   | lead 符号 | `DSH_LEAD=0` 与 `DSH_LEAD=0.57` 各跑 10 s                                          | 0.57 那场**准星明显领先板子**；`AIM yaw` 小约 8.3°                       | D-4         |
| **P5**   | **对照组** | `DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.057`，打 3 发 | 中 ≥1 发；`rounds` 涨 3                                         | D-1/D-2/D-6 |
| **P5b**  | 人工门     | `DSH_FIRE_KEY_ENABLE=1`（其余同 P5），按住 F 打 3 发                                      | 徽章变绿、状态行 `GATE=ON`、日志有"按键边沿"                                | D-8         |
| **P6**   | 低位靶     | `DSH_LOW_PLATE=1`（其余同 P5），打 3 发                                                 | `dz` > 容差、`plate=LOW/HIGH` 在切；**发数变少是正常的**；信任计时走满会打一行"放开高板" | D-9         |
| **P6b**  | 到达时刻选板  | `DSH_AIM_SELECT=arrival`（其余同 P5）                                                | `plate=(现朝向→到达朝向)`：**现朝向为负、到达朝向 ≈ 0**                       | D-4         |
| **P6c**  | 出弹延时    | `DSH_FIRE_DELAY=0.05`（再试 0.10）                                                  | 落点随延时**线性移动**（不是"打了就中"）                                     | D-4         |
| **P6d**  | 线性容限    | `DSH_ARRIVE_MM=105`（= 官方口径）                                                     | `blocked` 不再一直涨、`errY` 能进容限                                 | D-2         |
| **P7**   | 选板稳不稳   | `grep -c '选板变化' logs/p5.log`                                                    | 4 rad/s 下 ≈ **2.5 次/秒**（每 90° 相位换一次板）                       | D-10        |
| **P8**   | 静止靶零点   | 静止靶打 3 发                                                                        | 落点距靶心 < 1 靶宽                                                | D-5         |
| **P9**   | 旋转靶     | 4 rad/s 打 3 发                                                                   | 中 ≥1 发                                                      | D-4/D-6     |
| **P10**  | 记录      | `runlog.py` + `collect_run.sh`                                                  | 三个数进表                                                       | —           |

**铁律**：

- **P0.5 没过不许打 P5**（按键不通 ⇒ 全部字段 True 但一发不发）；
- **P3 没过不许打 P5**；**P4 没过不许打 P9**；
- **P5 没打过不许开低位靶**（否则分不清命中率变化是谁带来的）；
- 每换一个变量，**重跑一次 P0**（确认启动日志里的参数是新的）；
- 每一场都 `2>&1 | tee logs/xxx.log`。

## 4.1 测试结束统一输出（汇总）—— 不用再从几百行日志里翻数

**问题**：要确认/标定的数散落在好几处——启动日志（参数）、`[dbg] arrive`（errY / blocked）、  
`[dbg] fire-gate`（每一项为什么没发）、`选板变化`（切板频率）、`logs/lat_*.csv`（视觉延迟与云台滞后）。  
几百上千行里翻这些，很容易漏项、或者把上一场的数当这一场的。


**机制**：程序在**退出时（按 `q` 或 Ctrl+C）**自动打出一整块汇总，按 **A/B/C 三组**排列；
运行中随时按 **`s`** 也能打一次当前快照。
`DSH_SUMMARY_FILE` 设了的话，汇总会**另存一份独立文件**（不依赖 `tee` 捕获，天然不会被淹没）。

```bash
# 车机：给这一场起个名，并把汇总单独存一份
export DSH_TEST_TAG=p5_base
export DSH_SUMMARY_FILE=logs/summary_$(date +%m%d_%H%M).txt
DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.080 \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/p5_$(date +%m%d_%H%M).log
# 退出后：logs/summary_*.txt 里就是这一场的全部关键数值
```

汇总长这样（节选）：

```
================ 测试汇总 p5_base ================
  [A 组] 能不能打出去
    A1 电控挡位        : AUTO_AIM(自瞄)  ✓
    A2 云台滞后 errY    : 均值 0.62°  最小 0.21°  最大 1.94°  (门限 1.20°（按距离换算）)
    A3 按键通道        : evdev:/dev/input/event3，人工门=关
  [B 组] 打不打得准
    B1 视觉延迟        : （未测）
    B2 云台反馈滞后    : （未测）
    B3 出弹延时        : （未测）   (记录值 0.020 s，仅记录，不再单独累加)
       总提前量        : 0.080 s   ← 方案①：唯一功能旋钮
       飞行时间        : （未测）
  [C 组] 选板 / 高板
    C1 高度差 x[10]    : 均值 0.24 m  最大 0.25 m  (末值 0.25 m, warmup...)
    C2 角速度 w        : 末值 3.981 rad/s  （未测）
    C3 选板切换        : 31 次 / 12.3 s = 2.52 次/秒  (4 rad/s 期望 ≈2.5)
  [统计]
    总帧数 712，有目标帧 700，无目标帧 12
    开火请求 9 次，电控回报发弹 9 发  （未测）
    被挡：到位门控 12，面朝/上膛/低位靶 5，人工门 0，电控挡位 0，滤波未收敛 0
```

**B1/B2/B3、命中数、C2 的真值**这几项是**量出来的、程序测不到**，用 `add()` 的同名字段补录即可
（键名见 `src/test_summary.hpp` 的 `get("…")`：`B1 视觉延迟`、`B2 云台反馈滞后`、`B3 出弹延时`、
`飞行时间`、`命中数`、`C2 角速度真值`）。补录后重新打一次汇总，缺的项就会显示成你的值而不是"（未测）"。


---

## 5. 决策树 D-0 … D-11

每个节点只问一个问题、只跑一条命令。

### D-0 云台完全不动（`tgt NO` / 没有检测框）

```bash
grep -c '视野里没有 red\|视野里没有 blue' logs/p0.log   # 多 → 颜色/曝光
grep -m3 'detect' logs/p0.log | head -3
```

| 判据 | 结论 | 处置 |
|---|---|---|
| 大量「视野里没有 red…」 | 颜色反了 | `DSH_COLOR_ORDER=blue` 试一场（A/B 看哪句变少） |
| 有框但 `tgt NO` | 关联太严 | yaml `min_confidence` 0.88 → 0.8 |
| 完全没框 | ROI/曝光/相机 | 关 ROI（`use_roi: false`）；查相机是否 `0x80000006`（被别的进程占） |

### D-1 跟得住但一次都不发（`fire-gate` 有一项 false）

```bash
grep -m6 'fire-gate' logs/p0.log | tail -6
```

按**出现顺序**排查，前一项过了才看下一项：

| 字段 | false 的含义 | 处置 |
|---|---|---|
| `key=False` | **人没按住 F / 按键门没开 / 通道不通** | **D-8**（副本新增） |
| `low=否`（`low=(dz=…)`） | 瞄的是高板，低位靶规则在拦 | **D-9**（副本新增） |
| `stable=False` + `nis=0.8~1.0` | NIS 失败率被顶满 | 等 1.7 s（窗口 100 帧）再看；持续 ⇒ 关联错板 |
| `frames=3/10` 长期不动 | 目标在丢帧 | D-0 的丢帧分支 |
| `interval=False` | 刚打过 | 等 0.2 s |
| `mode=False` | **电控不在 AUTO_AIM** | D-3 |
| `hold=True` | 目标丢失但还在保持期 | 正常，等 `target_hold_ms` |
| `plate=False(30°)` | 板不在 ±25° 窗口 | 等它转进来（窗口 50°，4 rad/s 下 **0.22 s**） |
| `plate=False` 且角度常在 ±60~120° | **facing 约定反了** | D-4 |
| 全部 True 但仍不发 | 到位门控 | D-2 |

### D-2 门控挡住（`arrive=False`，`blocked` 持续涨）

```bash
grep -m6 'arrive' logs/p0.log | tail -6
```

| 判据 | 结论 | 处置 |
|---|---|---|
| `errY` 稳定在 0.8~1.5 | 门限太紧 | `DSH_ARRIVE_DEG=1.2`（**上限 1.2°**：超过板半宽 1.23° 就是放注定打不中的弹） |
| `eff` 与 `tol` 不一致 | 用了线性容限 | 记下 `eff`，按 `DSH_ARRIVE_MM` 调 |
| `errY` > 2° | 云台跟不上或零位偏 | 先降到 3 rad/s 再看 |
| `errP` 一直 > 1.28 | **pitch 符号约定错** | 看 `fb pitch` 与 `cmd pitch`：同号 ⇒ 要改成减法 |
| `jump` 很大（> 2×eff） | 命令在猛跳（刚换板/刚大幅摆动） | 正常：这一枪会被 `cmd_stable` 挡住，等下一帧 |
| `tripped=1` | 看门狗已解除（缺省关） | 忽略 |

### D-3 挡位不是 AUTO_AIM

```bash
grep -m2 '电控挡位' logs/p0.log
```

| 值 | 含义 | 处置 |
|---|---|---|
| `AUTO_AIM` | 正常 | — |
| `IDLE` | 电控没接管 | 遥控器拨到自瞄挡；确认 ECP 上电顺序 |
| 其他 | 未知挡位 | 同上，**别改代码** |

### D-4 开了火但不中，或方向明显不对

**先分清是「差半块板」还是「差好几块板」**：

| 现象 | 量级 | 病因 | 处置 |
|---|---|---|---|
| 偏 ~0.5 个靶宽，方向一致 | 0.8~1.2° | 延迟未补 | 开 `DSH_LEAD=0.057`（并按 P6c 标定） |
| 偏 3~5 个靶宽，方向一致 | 5~12° | 零位 / 偏置 | 查 offset 是否为 0；静止靶重标（P8） |
| 偏多少没规律 | — | 关联错板 / EKF 跳变 | 看 `assoc=` |
| 左右**反向** | — | facing 约定 | `assoc=` 长期 ±150° ⇒ 删 `task_2.cpp:527` 的 `+ M_PI` |
| 现朝向为正、到达朝向也为正 | 符号错 | `DSH_AIM_SELECT=arrival` 的 w 符号或 facing 约定 | 对照 P6b 的判据 |

**lead 符号验证（不发弹，10 秒）**：

```bash
DSH_LEAD=0    ./build/task_2 configs/standard.yaml   # 记 HUD 的 AIM yaw
DSH_LEAD=0.57 ./build/task_2 configs/standard.yaml   # 再记一次（0.57 = 10 倍放大，肉眼可辨）
```

判据：0.57 那场**准星应明显领先板子**（偏向板子转过去的方向）；
`AIM yaw` 应小约 **8.3°**（= 0.57 s × 14.55°/s）。**变大就是符号反了，先别发弹。**

### D-5 静止靶也打不准

```bash
py tools\zero_fix.py --d 3.15 --dx <左右 mm> --dy <上下 mm>
```

用输出的两个角度改 yaml 的 `yaw_offset` / `pitch_offset`，**不要用环境变量**（会忘）。
改完重跑 P0 看启动日志里的 `offset` 值。
（副本的 `zero_fix.py` 默认值已是 0.0/0.0，与原工程同步。）

### D-6 哑炮（`rounds` 涨了但没弹）

```bash
grep -m3 'fire-gate' logs/p0.log | tail -3     # 看 bullet_alive=
```

| `bullet_alive` | 含义 | 处置 |
|---|---|---|
| `true` | 电控回报发弹数，`rounds` 是真实发射数 | 真哑炮：查弹仓 / 摩擦轮 / 超容 |
| `false` | 软件自计，**打没打出去它不知道** | 改计数口径没用，用眼睛数弹壳 |

### D-7 跑不起来 / 掉帧

| 现象 | 处置 |
|---|---|
| 启动就段错误 | 少传了 `overlay.hpp`/`detect_log.hpp`/`latency_probe.hpp`/`fire_key.hpp`/`low_plate.hpp`/`env_util.hpp`，回 §1 |
| `keytest` 编不过 | `g++ -O2 -std=c++17 -I. tools/keytest.cpp -o /tmp/keytest`（`-I.` 必须带） |
| 帧率 58 → 44 | 录像开着，或 `DSH_DET_CSV` 每帧写盘 |
| `MV_CC_EnumDevices 0x80000006` | 相机被别的进程占：`pkill task_2`；USB 重插 |

### D-8 【副本】所有字段都是 True，就是不发

**先看右上角徽章**（§6.1）：

| 徽章 | 含义 | 处置 |
|---|---|---|
| 灰 `GATE OFF (auto fire)` | 人工门没开（**默认就是关的**） | 要人工确认 ⇒ `DSH_FIRE_KEY_ENABLE=1` |
| 黄 `BACKEND ERR …` | 取键后端读键异常（设备热拔/权限变化） | `ls -l /dev/input/event*`、`DSH_FIRE_KEY_DEV=…`；不行用 toggle |
| 红 `SAFE NO FIRE` | 门开着但现在不允许（没按 F / toggle 未解锁） | 按住 F；或按一次 F 解锁 |
| 绿 `ARM FIRE ALLOWED` | 现在允许开火 | 仍不发 ⇒ 看状态行其余字段（facing / plate / 到位） |

| 现象 | 结论 | 处置 |
|---|---|---|
| `key=False`，人确实按着 F | 按键通道没通 | 回 §2；`DSH_FIRE_KEY=toggle` 试 |
| 状态行 `GATE=OFF` 但你以为门是开的 | `DSH_FIRE_KEY_ENABLE=0`（默认） | 开它 |
| 徽章颜色和手指不一致 | 通道问题（cvkey 兜底在扛） | `keytest` 看 backend |
| 徽章红的、日志里没有"按键边沿" | 键根本没进到程序 | 同上 |

### D-9 【副本】低位靶规则把机会全挡了

```bash
grep -m3 'fire-gate' logs/p0.log | tail -3     # 看 low= 与 dz=
```

| 现象 | 结论 | 处置 |
|---|---|---|
| `dz` 一直 ≈ 0.00x | EKF 的高度差（x[10]）还没学出来 ⇒ 规则其实**没生效** | 正常，多跑几圈让 EKF 收敛；或把 `Z_TOL` 调小 |
| `dz` > 0.1x 且 `plate=HIGH` | 规则在拦（你瞄的一直是高板） | 正常，等低板转正；或 `DSH_LOW_PLATE=0` |
| `trust=x/y s` 一直不满 | 观察或 active 不满足 | 看 `plate=LOW/HIGH` 与 `dz` 是否稳定 |
| 想"每圈 4 发"而不是 2 发 | warmup 还没收敛 / 一直用 always | `DSH_LOW_PLATE=0`；等 `trust` 走满会自动放开 |
| `low=是` 但仍被拦 | 拦的是别的东西 | 回 D-1 其它字段 |

### D-10 【副本】选板在临界点来回跳

```bash
grep -c '选板变化' logs/p5.log
```

| 现象 | 结论 | 处置 |
|---|---|---|
| 4 rad/s 下 ≈ 2.5 次/秒 | ✅ 正常（每 90° 相位换一次板） | — |
| 几十次/秒 | 临界带在闪（两块 facing 差 ±5°） | 副本已把 `aim_id` 跨帧保持（5° 迟滞真生效）；仍闪就查 `assoc=` 跳变 |
| 一条都没有 | 选板一直是同一块 | 看 `aim_id` 与 `plate` 字段是否同号 |

### D-11 【副本】徽章与手指不一致 / 怀疑没按

- **徽章是代码看到的状态**：手指按下但徽章不变 ⇒ 通道问题（跑 `keytest`）；
- 徽章绿了仍不发 ⇒ 看状态行其余字段；
- **toggle 模式程序重启后锁存归 SAFE** ⇒ 徽章大红底一眼可见（防"重启后忘解锁零发弹"）；
- 事后争议用日志里的 `[task2] 开火门按键边沿` / `开火门 禁火→允许开火` 裁决（比 60 fps 画面可靠）。

---

## 6. 屏幕上的三件套

### 6.1 徽章（右上角 240×56，远距离可见）

| 颜色 | 文字 | 含义 |
|---|---|---|
| 灰 | `GATE OFF (auto fire)` | 人工门没开 |
| 黄 | `BACKEND ERR <后端>` | 取键后端异常 |
| 绿 | `ARM FIRE ALLOWED` | 现在允许开火 |
| 红 | `SAFE NO FIRE` | 门开着但现在不允许 |

### 6.2 状态行（屏幕左侧，三行）

```
AIM yaw 0.29 pitch 0.03 dist 3.15 w 4.00 rounds 3/0     ← 第 1 行（原有）
GATE=ON (hold, F=DOWN)                                  ← 第 2 行：开火门，**每帧都画**（没目标也在）
plate=LOW dz=0.07m trust=2.1/4.0s tgt_id=2 facing=-18->+3deg t_arr=183ms   ← 第 3 行：目标相关
```

> **屏幕文字一律 ASCII**：`cv::putText` 的 Hershey 字体不支持中文，中文会变乱码。
> **日志里**可以随便用中文。

### 6.3 边沿日志（forensic）

```
[task2] 开火门按键边沿：按下（后端 evdev:/dev/input/event3，第 1023 帧，pressed=1）
[task2] 开火门 禁火→允许开火（evdev:/dev/input/event3，第 1023 帧）
[task2] 低位靶规则：dz=0.11 m 稳定 4.0 s，x[10] 视为已收敛，放开高板
```

---

## 7. 判据速查（全部按代码/实测核过）

| 量 | 值 | 备注 |
|---|---|---|
| 板宽 / **半宽** @3.15 m | 2.46° / **1.23°** | 135 mm 板；打中容差是半宽 |
| 视在角速度 @4 rad/s | **14.55 °/s** | `r·w/d`（**不是** 229°/s，那是靶柱相位） |
| 靶柱相位速度 @4 rad/s | 229 °/s | 决定 facing 变化与出窗口速度 |
| 飞行时间 @3.15 m | 126 ms | 3.15/25 |
| lead 0.057 s 的补偿量 | **0.83°** ≈ 0.34 靶宽 | 官方 0.57 s 放大 = 8.3° |
| 出弹延时（官方注释） | 0.06~0.1 s | 4 rad/s 下 = 0.9~1.5°，**> 板半宽** |
| 到位门限 | 缺省 0.8°，**上限 1.2°** | 官方 3°/2° ≈ 恒定 **105 mm**；0.8°@3.15 m 只有 44 mm |
| 连续到位帧 | 3 帧 ≈ 50 ms | 应急可 `DSH_ARRIVE_FRAMES=2` |
| 发弹间隔 | 0.2 s（5 Hz 上限） | |
| **实际弹频** | 一圈 4 块各 1 发 ⇒ **平均 2.55 Hz**（4 rad/s），最快 3.5 Hz | 3 rad/s ⇒ 1.9 Hz；9 rad/s ⇒ 被 0.2 s 压到 5 Hz |
| **只打低位靶** | 每圈 2 块 ⇒ **1.27 Hz** | warmup 暖机约 4 s 后恢复 2.55 Hz |
| 面朝窗口 / 重上膛 | ±25° / 35° | 窗口时长 50° ⇒ 4 rad/s 下 **0.22 s** |
| 静止靶判据 | 30 °/s 且连续 10 帧 | 60 fps 下 ≈0.5°/帧；45 fps 下 ≈0.39 rad/s |
| 低位靶信任 | 高度差稳定 4 s | 信任走满 ⇒ 放开高板 |
| NIS 窗口 / 阈值 | 100 帧 ≈ **1.7 s**；9.488（统计）/ 16.27（拒收） | 拒收风暴 ⇒ 最长 1.6 s 不发弹，**不是永久** |
| `min_frames_for_fire` | 10 帧 ≈ 172 ms | |
| 官方实测命中率 | 国赛 **39.6% 最高、≥30%**（2 m、7/14 rad/s） | 期望标尺：4 rad/s 应 50~60%，9 rad/s 30~50% |

### 7.1 三段预测时域（方案①：**一个旋钮**）

| 时段 | 谁补 | 参数 | 说明 |
|---|---|---|---|
| 出膛→**弹着**（飞行） | **calc_aim 内部自动** | — | 按弹道迭代出的 `t_flight`（3.15 m/25 m/s = 126 ms） |
| 曝光→**算出瞄点** + **云台跟随滞后** + **摩擦轮出弹** | **`DSH_LEAD`（总提前量）** | 0.05~0.10 s | **唯一的功能旋钮**。三段合并成一个数补——形式都是"把瞄点往前推" |
| 其中"摩擦轮出弹"占多少 | `DSH_FIRE_DELAY` | 现场量 | **只作记录/拆分标注，不再单独累加** |

> **为什么合并成一个旋钮**：`DSH_LEAD` 修的是"炮管指旧了"（纯角度平移），
> `DSH_FIRE_DELAY` 修的是"靶子真转了"（绕中心旋转）——两者**数值形式相同**
> （都是"视在角速度 × 时间"），二阶差 ≈ 0.1°，可忽略。
> 分开填而实际填的是同一段延迟 ⇒ **补过头 0.83°，还会让人误判成"符号反了"**。
> ⇒ **方案①：`DSH_LEAD` = 总提前量（唯一功能旋钮）；`DSH_FIRE_DELAY` 只用来记录拆分。**
> ⇒ 若 `DSH_FIRE_DELAY` 非 0 而 `DSH_LEAD=0`，启动日志会打一条 ⚠（出弹延时没进补偿）。
> ⇒ **`DSH_LEAD` / `DSH_FIRE_DELAY` 都是 `static` 一次性读取 ⇒ 改完必须重启进程。**

---

## 8. A/B 试验顺序（一次只动一个变量）

| 轮 | 变量 | 判据 |
|---|---|---|
| 基线 | 全关（见 §4 的 P5 命令） | hits/shots、errY、rounds |
| A1 | `DSH_AIM_SELECT=arrival` | 命中率↑；`plate=(现→到达)` 现朝向为负、到达 ≈0 |
| A2 | `DSH_FIRE_DELAY=0.05` → `0.10` | 落点**线性**移动（官方"拍慢动作比击杀时间"那套） |
| A3 | `DSH_ARRIVE_MM=105` | `blocked` 不再涨、`errY` 进容限 |
| A4 | `DSH_LOW_PLATE=1` | 命中率↑（代价：机会减半） |
| A5 | `DSH_FIRE_KEY_ENABLE=1` | 省额度、留 10 发 |
| A6 | `DSH_STATIC_RATE_DPS` 30→60 | 低速段的"静止"误判变少 |

> **A2 的判据是"落点随延时线性移动"**，不是"打了就中"——因为 0.1 s × 14.55°/s = 1.46°，比容差还大，方向对了才谈命中率。

---

## 9. 记录与回退

### 记录

```powershell
cd D:\MTG\Documents\OC\fp_aim_fkey
py tools\runlog.py new r1021_p5 --task task_2 --stage P5
py tools\runlog.py set r1021_p5 shots=3 hits=1 lead=0.057 arrive_deg=1.2 low_plate=0 fire_key=0
py tools\runlog.py show
```

### 回退（三条路，别用 git）

```bash
# 路 1：只关掉新策略（行为≈原工程，但保留第 3/4/5 条与诊断）
DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_AIM_SELECT=current \
DSH_FIRE_DELAY=0 DSH_ARRIVE_MM=0 ./build/task_2 configs/standard.yaml

# 路 2：完全回到原工程（笔记本 scp 覆盖，车机上没有 .git）
#   scp D:\DSH\fp_aim\src\task_2.cpp D:\DSH\fp_aim\src\task_3.cpp D:\DSH\fp_aim\src\task_common.hpp \
#       rm@192.168.1.50:~/fp_aim/src/
#   cd ~/fp_aim && touch src/task_2.cpp && cd build && make -j2 task_2

# 路 3：只回退第 3/4/5 条（车机侧备份）
cd ~/fp_aim && cp src/task_2.cpp.bak_aim src/task_2.cpp && cd build && make -j2 task_2
```

> **车机上大概率没有 .git**（仓库在笔记本上），`git checkout -- src/task_2.cpp` 那条用不了。
> `configs/standard.yaml` 回退前先 `cp configs/standard.yaml configs/standard.yaml.bak.$(date +%H%M)`。

---

## 10. 一页纸命令

```bash
# ── 笔记本：传 18 个文件（§1.2，注意 io/gimbal 那行的路径）────────────
# ── 车机 ──────────────────────────────────────────────
cd ~/fp_aim
./build/keytest                                   # S0.5 按键：backend 必须是 evdev/termios
cd build && cmake -B . .. >/dev/null && make -j2 2>&1 | tail -20
cd ~/fp_aim
# 建议：每场都带标签，退出后自动得到一份汇总
export DSH_TEST_TAG=p5_base
export DSH_SUMMARY_FILE=logs/summary_$(date +%m%d_%H%M).txt
./build/task_2 configs/standard.yaml 2>&1 | tee logs/p0_$(date +%m%d_%H%M).log    # P0：看五行启动日志
DSH_LOW_PLATE=0 DSH_FIRE_KEY_ENABLE=0 DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.080 \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/p5_$(date +%m%d_%H%M).log    # P5 对照组
# 退出（q / Ctrl+C）后：logs/summary_*.txt 就是这一场的 A/B/C 三组汇总
# 运行中按 s 可随时打一次当前快照

# 每场跑完先看这四条
grep -m1 '电控挡位' logs/p5_*.log        # AUTO_AIM？
grep -m3 'arrive'    logs/p5_*.log        # errY / blocked / jump / eff
grep -m3 'fire-gate' logs/p5_*.log        # key= / low= / plate= / t_arr / rounds
grep -c  '选板变化'  logs/p5_*.log        # ≈ 2.5 次/秒
```

**改这些必须重启进程**：`DSH_LEAD`、`DSH_FIRE_DELAY`、`DSH_STATIC_RATE_DPS`、`DSH_STATIC_FRAMES`
（都是 `static` 一次性读取）。
