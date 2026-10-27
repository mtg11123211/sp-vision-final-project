# 现场执行卡 V4：30 分钟从零到打中旋转靶

> **这一版和 V3 的区别（全部是「少打、打准」方向）**
> 1. **到位门控默认打开**：`0.8° / 1.3°`，而且要**连续 3 帧**都在门限内才算到位
>    （原来单帧就算，云台扫过门限的那一瞬间也会被判到位，等于没卡）；
> 2. **新加「板面朝向」门控**：只在板面与视线夹角 **≤25°**（正对着我们）时才开火；
> 3. **瞄点自动选「最正对镜头」的那块板**：EKF 关联出来的那块如果已经转过窗口，
>    就换到中间那块 —— 不再「追着快转走的那块打」；
> 4. **一块板只打一发**：打完这一发，同一块板必须摆出 **35°** 再转回来，才允许打第二发
>    —— 这就是「禁止对着同一块板连打」。换板不受限；
> 5. **发弹间隔保持 0.2 s（5 Hz）**：防连打靠「一块板一发」（摆出 35° 才重新上膛），不靠拉长间隔；
> 6. **`lead` 的符号修了**：近侧板的视在角速度和目标转动角速度**反号**，原来写的是 `+=`，
>    等于「板往右转、准星往左补」，误差翻倍；现在按「板的视在角速度」折算，`DSH_LEAD` 用**秒**（0.057）；
> 7. **看门狗自动解除已从代码里关掉**（缺省 0 = 永不解除）；
> 8. `require_auto_aim_mode: true`、`min_frames_for_fire: 15→10`。
>
> **代码和 yaml 都改过，第 0 步不能跳。**
> 时间轴：0 步 8 分钟 → 1–3 步 15 分钟（9 发）→ 4 步分流 → 7 步固化 5 分钟。

---

## 0. 传文件 + 编 + 验收（0 发）

### 0.1 传（笔记本 PowerShell）

```powershell
scp D:\DSH\fp_aim\configs\standard.yaml rm@192.168.1.50:~/fp_aim/configs/standard.yaml
scp D:\DSH\fp_aim\src\task_2.cpp    rm@192.168.1.50:~/fp_aim/src/task_2.cpp
```

### 0.2 编（车机 bash）

```bash
cd ~/fp_aim && touch src/task_2.cpp && cd build && make -j2 task_2
```

**成功判据（缺一不可）**：`Building CXX object CMakeFiles/task_2.dir/src/task_2.cpp.o` **和**
`Linking CXX executable task_2`。只出现 `[100%] Built target` 等于没重编 —— 旧二进制不认识新默认值。

### 0.3 验收（车机 bash，四条都要过）

```bash
cd ~/fp_aim
grep -nE '^(yaw_offset|pitch_offset):' configs/standard.yaml   # 25,26 行，两个都是 0.0
grep -n 'min_frames_for_fire: 10\|require_auto_aim_mode' configs/standard.yaml
#   必须出现 64:min_frames_for_fire: 10  和  82:  require_auto_aim_mode: true
#   （第 95 行那个 false 是 task3 的，不用管）
grep -n '^record_dir:' configs/standard.yaml                   # 必须是 record_dir: ""
grep -n 'fire_interval_s: 0.2' configs/standard.yaml           # task2 段必须是 0.2（5 Hz）
grep -c 'plate_facing_deg\|plate_rearm_deg' src/task_2.cpp     # 必须 >= 6
unset DSH_LEAD DSH_ARRIVE_DEG DSH_ARRIVE_PITCH_DEG DSH_ARRIVE_FRAMES \
      DSH_ARRIVE_WATCHDOG_SHOTS DSH_PLATE_FACING_DEG DSH_PLATE_REARM_DEG \
      DSH_STEP_DEG DSH_YAW_OFFSET DSH_PITCH_OFFSET DSH_RECORD_DIR DSH_DET_CSV DSH_LAT_CSV
```

**别开录像**（`DSH_RECORD_DIR` 与 yaml 的 `record_dir` 都要空）：58 → 45 fps，时间基准全乱。

---

## 1. 门控 baseline（3 发，只验证一件事：会不会发弹）

```bash
cd ~/fp_aim && ./build/task_2 configs/standard.yaml 2>&1 | tee logs/v1_gate_$(date +%H%M).log
```

**不用带任何环境变量**：门控、板面朝向、一块一发的默认值都在代码里，第 0 步过了就是这套。

启动后必须看到这三行：

```
[task2] offset yaw=0.00(yaml) pitch=0.00(yaml)
[task2] 到位门控 = 开（yaw 0.80° / pitch 1.28°，连续 3 帧）｜看门狗 关｜板面正对 ±25°、摆出 35° 才重新上膛（一块板只打一发）｜发弹间隔 0.2 s
[dbg] 电控挡位 = AUTO_AIM(自瞄)
```

打 3 发，**5 秒内一发没出就 Ctrl+C**，不要等（看门狗已经不救了）。

**开火之前先看一眼这两行**（各 0.8 秒一条，`grep` 一下就行）：

```bash
grep -m1 'fire-gate' logs/v1_gate_*.log
grep -m1 'arrive' logs/v1_gate_*.log
```

| 看什么 | 正常 | 不正常 → 动作 |
|---|---|---|
| `assoc=xx°` | **0–40°**（关联到的板一定是我们看见的那块） | 长期 ±150° 以上 = `facing` 的朝向约定反了，见 §11 B5+「约定自检」 |
| `plate=(true/false)` | 板转到中间时 `true` | 恒 `false` → 窗口太窄，`DSH_PLATE_FACING_DEG=40` |
| `errY=` | 平均 < 1°，偶尔到 3° | 长期 > 2° → 云台跟不上，见 §11 的 arrive 段 |

| 现象 | 含义 | 下一步 |
|---|---|---|
| 3 发全中 | 原来就是「没到位就开火」 | → ⑦ 固化，收工 |
| 会发弹、中的变多 | 门控生效，还差系统偏差 | → ② 叠 lead |
| 会发弹，但一发比一发间隔长很多 | 正常：一块板只打一发，四块板转一圈才放 4 发 | 继续 |
| 一发不发，日志全是 `arrive=false` | 云台在 0.8° 内停不住 | → ①-b |
| 一发不发，`fire-gate` 行里 `plate=false` 且 `face` 恒在 ±30° 外 | 板面窗口太窄（或目标识别一直挑到侧板） | `DSH_PLATE_FACING_DEG=40` 再试 |
| 一发不发，连 `fire-gate` 行都没有 | 没检测到目标 / 挡位 IDLE | → 回 §7.1 查检测与挡位 |

### ①-b 放宽门限（3 发）

```bash
cd ~/fp_aim && DSH_ARRIVE_DEG=1.5 DSH_ARRIVE_PITCH_DEG=2.0 \
  ./build/task_2 configs/standard.yaml 2>&1 | tee logs/v1b_gate15_$(date +%H%M).log
```

还是不发弹 ⇒ 云台在这个转速下咬不住，**lead 怎么调都没用**，直接 → ⑤。

---

## 2. 叠 lead（3 发）

```bash
cd ~/fp_aim && DSH_LEAD=0.057 ./build/task_2 configs/standard.yaml 2>&1 | tee logs/v2_lead36_$(date +%H%M).log
```

`0.057` 秒 = 视觉 25 ms + 云台 32 ms；4 rad/s 下换算成约 **0.83°**（1/3 个靶宽）。
注意数值和 V3 的 `0.0036` 不一样，但**修正量一样**：现在由代码自己算「板在相机里的角速度」，
不再需要你把 `r/d` 折进常数里。

| 结果 | 下一步 |
|---|---|
| 中了 | → ⑦ |
| 还偏左 | → ③ |
| 偏右了 | lead 给多了，改 `0.040` 重打 3 发 |

---

## 3. 斜率测试（3 发，决定 lead 值不值得继续调）

```bash
cd ~/fp_aim && DSH_LEAD=0.171 ./build/task_2 configs/standard.yaml 2>&1 | tee logs/v3_lead110_$(date +%H%M).log
```

`0.171` 是第 2 步的 3 倍（4 rad/s 下 2.5°，约 1 个靶宽）。**只改这一个数。**

| 结果 | 结论 | 下一步 |
|---|---|---|
| 落点明显右移（≈0.7 靶宽） | 时间型误差，lead 有效 | 二分：`新值 = 0.057 + 0.114 × (剩余偏差 ÷ 右移量)` |
| 落点几乎不动 | **不是 lead 的事** | 停手 → ④ |
| 偏右太多 | 补过头 | 在 0.040 与 0.171 之间二分 |

---

## 4. lead 无效时查两处（0 发）

```bash
grep -o 'errY=[0-9.]*' ~/fp_aim/logs/v3_*.log | sort -t= -k2 -n | tail -8   # 云台跟不跟得住
grep -c 'arrive=false' ~/fp_aim/logs/v3_*.log                               # 门限紧不紧
```

- `errY` 常年 2° 以上 ⇒ 云台咬不住，改控制参数才有用，**今天不要再调 lead**；
- `errY` 很小但仍然偏 ⇒ 瞄准链本身有固定偏差（零位/机械/弹道）→ 静止靶 3 发定位。
- `arrive=false` 占比 > 80% ⇒ 门限过紧，回 ①-b。

---

## 5. 电控挡位异常时的回退（只在完全不发弹时用）

`require_auto_aim_mode: true` 之后，挡位抖一下就会完全停发。现场应急改回 `false`，**不用重编**：

```bash
sed -i '0,/require_auto_aim_mode: true/s//require_auto_aim_mode: false/' ~/fp_aim/configs/standard.yaml
```

---

## 6. 判定速查

| 症状 | 先看 | 结论 |
|---|---|---|
| 完全不发弹，无 `fire-gate` 行 | HUD 的 `tgt` | 没目标 / 检测失败 / 挡位 IDLE |
| 完全不发弹，有 `fire-gate` 且 `arrive=false` | `[dbg] arrive` 的 `errY` | 门限过紧，或云台跟不上 |
| 完全不发弹，`fire-gate` 里某项 False | 那一位的名字 | 逐项对：`stable/interval/rounds/mode/holding` |
| `rounds` 涨、车上 `bullet_count` 不涨 | — | 电控不认开火位（SOP §10 分支 A） |
| 发弹但落点偏 | 第 2/3 步的右移量 | 门控只压随机误差，系统偏差要 lead 或零位 |

---

## 7. 固化（收工前 5 分钟）

### 7.1 写进启动脚本

```bash
cd ~/fp_aim && grep -n 'exec' run2.sh        # 在 exec 行之前插入下面这一行
```

```bash
export DSH_LEAD=0.057
```

> 门控、板面朝向、一块一发都不必写：这几个默认值已经写进代码（0.8°/1.3°/连续 3 帧、
> ±25°/摆出 35° 才重新上膛、间隔 0.2 s），只有想临时改时才用环境变量覆盖：
> `DSH_ARRIVE_DEG` `DSH_ARRIVE_PITCH_DEG` `DSH_ARRIVE_FRAMES` `DSH_PLATE_FACING_DEG`
> `DSH_PLATE_REARM_DEG`。
>
> 看门狗不用写：代码缺省已经是 `0`（永不解除）。只有专门调试「门限是不是太紧」时才临时给
> `DSH_ARRIVE_WATCHDOG_SHOTS=12`。

### 7.2 记一笔（30 秒）

```bash
cd ~/fp_aim
py tools/runlog.py new v4_$(date +%m%d) --task task_2 --stage P6
py tools/runlog.py set v4_$(date +%m%d) arrive_deg=0.8 lead_ms=57 shots=3 hits=2 verdict="门控+板面窗口生效"
py tools/runlog.py show
```

### 7.3 打包带走

```bash
cd ~/fp_aim && bash tools/onsite/collect_run.sh --name v4_$(date +%m%d) --meta-only
```

---

## 附：这几个数是怎么来的（有人问就照着说）

| 参数 | 值 | 依据 |
|---|---|---|
| `DSH_ARRIVE_DEG` | 0.8° | 4 rad/s 下稳态跟随滞后 ≈0.45°，0.8° = 1.6× 它；只拦「明显没跟上」的瞬间 |
| `DSH_ARRIVE_PITCH_DEG` | 1.3° | pitch 轴慢一档，放宽 1.6 倍；`errP` 实测常在 0.07–0.2° |
| 连续到位帧数 | 3 | 58 fps ≈ 50 ms；单帧会被「扫过 0.5°」骗过去 |
| `plate_facing_deg` | 25° | 四块板 90° 一块，正对窗口 50°；再宽就会打在侧板上 |
| `plate_rearm_deg` | 35° | 比正对窗口大 10°，避免临界抖动来回放行 |
| `fire_interval_s` | 0.2 s（5 Hz） | 防连打靠 plate_armed，不靠间隔 |
| `DSH_LEAD` | 0.057 s | 视觉 25 ms + 云台 32 ms；代码按「板的视在角速度」折算，4 rad/s → 0.83° |
| `min_frames_for_fire` | 10 | 58 fps 下 172 ms（旧值 15 = 259 ms，太保守） |
| 看门狗 | 0（关） | 一档只有 10 发，12 发才解除 ⇒ 只在第一档有效且静默失效 |

## 禁止事项

| 不要 | 原因 |
|---|---|
| 把 `DSH_ARRIVE_WATCHDOG_SHOTS` 设成正数 | 等于重新打开自动解除，挡满 N 发后门控永久失效 |
| 把 `DSH_PLATE_FACING_DEG` 调到 45 以上 | 侧板也放行，等于回到「不管朝向就开火」 |
| 把 `DSH_PLATE_REARM_DEG` 设得比 `DSH_PLATE_FACING_DEG` 小 | 重新上膛发生在窗口**之内**，防抖就没了（35 > 25 是有意的） |
| 用 `DSH_PITCH_OFFSET` 当回退手段 | 旧版这键是死配置，**新版是真推角度**，填 10 就是真偏 10° |
| 边录像边调 lead | 58 → 45 fps，时间基准全乱 |
| 用 `yaw_offset` 补旋转偏差 | 常值补不了正比于 ω 的量 |
| 设了 `DSH_STEP_DEG` 忘关 | `fire = would_fire && !step_enabled()` ⇒ 永远不开火 |
| 一次改两个变量 | 出了偏差无法归因 |
| 门控卡住还继续等 | 看门狗已经不救你了，5 秒没弹就 Ctrl+C |
