# 现场记录表（P0–P7 一张表走完）

> 目的：**每一步只记「能改变下一步决定」的数**，不记过程。  
> 两个载体配合用：
> - **本文件**：人看的，一次现场一页，按 P0–P7 填 6 个格子。
> - **`tools/runlog.py`**：机器看的，一条命令存一场，最后 `show` 出对比表。
> 记录不全的唯一后果是「下次到场不知道上次改了什么、为什么改」，所以宁少勿漏这两个：**命令是什么**、**判定是什么**。

---

## 0. 一页纸（每次现场复制这一节）

**日期**：____  **场地**：____  **操作人**：____  **剩余发弹机会**：____ 发

| 阶段 | 命令（原样抄） | 关键读数 | 判定 | 结论 / 下一步 |
|---|---|---|---|---|
| P0 体检 | | 编译看到 `Building CXX` □ `Linking` □ | 过了 □ / 没过 □ | |
| P1 通路 | | fps ___ / det ___ / 挡位 ___ / 颜色告警 ___ 条 | 过了 □ / 没过 □ | |
| P2 探针 | | 视觉 p50 ___ ms / 串口 p50 ___ ms | 可用 □ / 数据太散 □ | |
| P3 阶跃 | | D ___ ms / τ ___ ms / T90 ___ ms | 拟合残差 ___ % | |
| P4 定 lead | | 建议 `DSH_LEAD=___` ± ___ ms | 容差表最低 ___ ms | |
| P5 静止靶 | | 发数 ___ / dx ___ mm / dy ___ mm / d ___ m | 上纸 □ / 未上纸 □ | |
| P6 双转速 | | 低 3–5：___ 中 5–9：___ | 线性 □ / 不线性 □ | |
| P7 微调 | | 改的哪几个数： | 好转 □ / 恶化 □ / 没差别 □ | |

---

## 1. 每个阶段到底记什么

### P0 体检（0 发，约 3 分钟）

```bash
cd ~/fp_aim && touch src/task_2.cpp src/task_3.cpp tasks/auto_aim/armor.cpp \
  io/gimbal/gimbal.cpp io/camera.cpp io/hikrobot/hikrobot.cpp
cd build && make -j2 2>&1 | tee /tmp/build_$(date +%H%M).log
```

| 要记 | 怎么记 | 为什么 |
|---|---|---|
| 编没编上 | 输出里有没有 `Building CXX object .../task_2.cpp.o` 和 `Linking CXX executable task_2` | 只有 `[100%] Built target` = 用的是旧二进制，后面所有数据都作废 |
| 二进制时间 | `ls -l --time-style=full-iso build/task_2` | 同上，一秒确认 |
| 编译报错原文 | 有错就整段复制进「结论」栏 | 现场改代码最贵的错误是凭记忆复述报错 |

> 一条命令把编译日志也存进包里：`make -j2 2>&1 | tee ~/fp_aim/logs/build_$(date +%m%d_%H%M).log`

### P1 通路（0 发，约 5 分钟）

```bash
cd ~/fp_aim && DSH_RECORD_DIR=recordings/run_$(date +%m%d_%H%M) \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/run_$(date +%m%d_%H%M).log
```

| 要记 | 从哪看 |
|---|---|
| fps、检测到的板数 | HUD 左下角 `f=`、`det=`、`fps` |
| 挡位 | `[dbg] 电控挡位 = AUTO_AIM / IDLE` |
| 颜色方向 | `视野里没有 red 色的装甲板` 这句在两种 `DSH_COLOR_ORDER` 下各出现几次 |
| 启动日志四行 | `offset`、`弹速`、`弹道`、`到位门控`（§4.2） |

### P2 探针（0 发，30 秒）

```bash
DSH_LAT_CSV=logs/lat_$(date +%H%M).csv ./build/task_2 configs/standard.yaml
py tools/latency_report.py logs/lat_XXXX.csv      # 笔记本上跑
```

记三个数：**视觉 p50**、**串口 p50**、**总链路 p50**。总链路 = 取帧→串口写完，这是 lead 的地板。

### P3 阶跃（0 发，60 秒）

```bash
DSH_LAT_CSV=logs/step_$(date +%H%M).csv DSH_STEP_DEG=3 ./build/task_2 configs/standard.yaml
py tools/latency_report.py logs/step_XXXX.csv
```

记三个数：**D（通信/控制延迟）**、**τ（云台时间常数）**、**拟合残差**（>10% 就说明这段数据不能用来算 lead）。

### P4 定 lead（0 发，10 秒）

```bash
py tools/lead_advisor.py logs/lat_step_XXXX.csv
```

记：**建议 `DSH_LEAD`**、**± 不确定度**、**该转速下的容差**。  
判定：建议值的不确定度必须小于容差的一半，否则下一轮只准以 ≤5 ms 步长微调。

### P5 静止靶（3 发）

```bash
DSH_YAW_OFFSET=0 DSH_PITCH_OFFSET=0 DSH_LEAD=0 ./build/task_2 configs/standard.yaml
```

记四样：**发数**、**dx（正=偏右）**、**dy（正=偏上）**、**距离**。  
打完立刻算：`py tools/zero_fix.py --d 3.4 --dx 10 --dy -10`，把它吐的两个角写进 yaml。

### P6 双转速（6–8 发）

低转速（3–5 rad/s）2–3 发、中转速（5–9 rad/s）2–3 发，用 P4 的 lead。  
记：**每个转速下偏的方向和板数**。判定看「偏移是否随转速成比例」——比例成立 = 还缺时间，不成立 = 缺别的。

### P7 微调（≤5 发）

一次只改一个数，改完至少打 2 发再看。记「改了哪个数、从多少到多少」。

---

## 2. 用 `tools/runlog.py` 存成可对比的表

```bash
# 开一场（自动建 docs/runs/<名字>.json）
py tools/runlog.py new run1_1018 --task task2 --stage P1

# 手动补几个关键数（k=v，可以一次写多个）
py tools/runlog.py set run1_1018 fps=57.9 det_pct=100 lead_ms=0.075 dist_m=3.4

# 探针 CSV 直接喂进去，自动填视觉/串口延迟和 fps
py tools/runlog.py lat run1_1018 logs/lat_1530.csv

# 打完靶再补结果
py tools/runlog.py set run1_1018 shots=3 hits=2 dx_mm=10 dy_mm=-10 verdict=PASS

# 所有场次横向对比
py tools/runlog.py show
```

**字段一览**（`set` 里都能写）：

| 字段 | 含义 | 单位 |
|---|---|---|
| `task` / `stage` | 任务号 / P0–P7 | — |
| `fps` / `det_pct` | 帧率 / 有检测的帧占比 | Hz / % |
| `color_order` | `red`（默认）或 `blue` | — |
| `arrive_deg` | 到位门控门限，0=关 | ° |
| `lead_ms` | 实际用的 `DSH_LEAD` | ms |
| `off_yaw` / `off_pitch` | yaml 里的 offset | ° |
| `lat_vision_ms` / `lat_send_ms` | 视觉侧 / 串口侧延迟 p50 | ms |
| `D_ms` / `tau_ms` | 阶跃拟合的纯延迟 / 时间常数 | ms |
| `lead_suggest_ms` | `lead_advisor.py` 的建议值 | ms |
| `shots` / `hits` | 发数 / 命中数 | 发 |
| `dx_mm` / `dy_mm` | 落点偏差（正=右 / 正=上） | mm |
| `dist_m` | 靶距 | m |
| `verdict` | `PASS` / `FAIL` / `PARTIAL` | — |
| `note` | 一句话结论 | — |

---

## 3. 填好的样子（照这个抄）

```
日期：2026-10-20   场地：车库   剩余发弹机会：6 发

P0  编过（Building task_2.cpp.o + Linking）→ 通过
P1  fps 57.9 / det 100% / AUTO_AIM / 颜色告警 0 条 → 通过
P2  视觉 p50 34.2 ms / 串口 p50 1.1 ms → 可用
P3  D 17.1 ms / τ 19.8 ms / 残差 3% → 可用
P4  建议 DSH_LEAD=0.072 ±10 ms，5 rad/s 容差 ±4.5 ms → 不确定度偏大，微调步长 ≤5 ms
P5  3 发 2 中，dx +12 mm，dy −8 mm，d 3.4 m → zero_fix 给 yaw 0.20 / pitch −0.13
P6  未做（剩 3 发）→ 留给微调
```

对应命令：

```bash
py tools/runlog.py new run1_1020 --task task2 --stage P5
py tools/runlog.py set run1_1020 fps=57.9 det_pct=100 off_yaw=0 off_pitch=0
py tools/runlog.py lat run1_1020 logs/lat_1612.csv
py tools/runlog.py set run1_1020 D_ms=17.1 tau_ms=19.8 lead_suggest_ms=72 lead_ms=72 \
  shots=3 hits=2 dx_mm=12 dy_mm=-8 dist_m=3.4 verdict=PARTIAL note="静态零点已量，待写回yaml"
```

---

## 4. 三条纪律

1. **换参数之前先打包**：`bash tools/onsite/collect_run.sh <name>`。录像会丢，日志和 csv 不能丢。
2. **一次只改一个数**，并且把「改之前的值」写进 `note`。否则三场之后没人知道是哪个改动起的作用。
3. **判定栏必须写“下一步”**，不能只写“不行”。现场最贵的是停下来想“那现在干嘛”。
