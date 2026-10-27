# 官方 sp_vision_25 的思路 → 副本落地记录

> **来源**：`github.com/TongjiSuperPower/sp_vision_25`（同济大学SuperPower战队 25 赛季视觉框架，485 star，
> 2024-08 建仓，2025-08 最后推送）。本工程（tutorial_27 的 `final_project_aim` 分支）是它的**教学化后代**，
> `tasks/auto_aim/target.cpp` 的状态向量与 `h_armor_xyz` 的 `center − r·cos(a)` 符号**与官方逐字一致** ⇒
> 下面的对照是"同源代码"级别的，不是"参考别人"。
> **落地位置**：`D:\MTG\Documents\OC\fp_aim_fkey`（副本）。原工程零改动。
> **原则**：官方 25 赛季已经把决策器"由基于经验主义的分段决策逻辑改为基于轨迹优化的规划器"——
> 也就是说**你现在这套四道闸正是他们放弃的那条路**。下面取的是他们**换掉之前**留下的、
> 且能在本工程结构里复现的部分。

---

## 一、从官方代码里读到的五条（附出处）

### ① 选板要在"弹丸到达时刻"的画面上选 —— `aimer.cpp:61,84-116`

```cpp
auto aim_point0 = choose_aim_point(target);        // 第一次
for (int iter = 0; iter < 10; ++iter) {
  auto predict_time = future + microseconds(prev_fly_time);   // 预测到"到达时刻"
  iteration_target[iter].predict(predict_time);
  auto aim_point = choose_aim_point(iteration_target[iter]);  // 在预测后的画面上选板
  ...
  if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) { converged = true; break; }
}
```

官方：**先 `predict(future + fly_time)`，再 `choose_aim_point`**，最多迭代 10 次，飞行时间差 <1 ms 收敛。

**为什么关键**：本工程是"**现在**正对就选板"，然后 `calc_aim` 把这块板的位置往前推飞行时间。
4 rad/s、飞行 126 ms ⇒ 靶子转过 **29°**：
- 现在正对的那块，到弹着时已经转走 29°（投影宽 135·cos29° = 118 mm，还是能打中，但**不是最优靶面**）；
- 真正"到弹着时正对"的是隔壁那块 ⇒ **现在正对 ≠ 命中时正对**。

**落地**（`task_2.cpp:463-495`）：`DSH_AIM_SELECT=arrival` 时，
`facing` 以 `w` 变化（外法线跟着靶子转）⇒ 到达时朝向 = 现在的朝向 + `w·t_arrive`，
选 |到达朝向| 最小的那块；面朝门控也用到达朝向判。
**默认 `current`（不改原行为）**，上车 A/B。

### ② 官方对"选哪块板"用的是 coming/leaving 双角 + w 的符号 —— `aimer.cpp:192-208`

```cpp
// 小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
for (int i = 0; i < armor_num; i++) {
  if (std::abs(delta_angle_list[i]) > coming_angle) continue;
  if (ekf_x[7] > 0 && delta_angle_list[i] <  leaving_angle) return {true, armor_xyza_list[i]};
  if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
}
```
`coming_angle: 55°` / `leaving_angle: 20°`（`configs/standard3.yaml:42-43`）——**进入窗口 55°、离开窗口 20° 的双阈值迟滞**，
配合 `w` 的符号选"正在进入"的那块。官方在非陀螺分支还有一个 **lock_id_ 锁定模式**：
"防止在两个都呈 45 度的装甲板之间来回切换"。

**落地**：副本的 `aim_id_persist`（跨帧保持）+ 5° 迟滞 **就是官方 lock_id_ 的等价物** ✓（已落，`docs/复核稿第345条_已落记录.md`）。

### ③ 出弹延时必须算进预测 —— `aimer.cpp:54-57`

```cpp
auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
```
官方注释：**检测→瞄准 5 ms ＋ 发弹延时 0.1 s**；并且 `delay_time` 按转速分两档
（`high_speed_delay_time` / `low_speed_delay_time`，切换阈值 `decision_speed: 7 rad/s`）。

**为什么关键**：本工程的 `calc_aim` 只预测到"飞行时间"，**没算摩擦轮从接到指令到子弹出膛这 0.06~0.1 s**。
这 0.1 s 里靶子照转 ⇒ 4 rad/s 下**系统性偏晚 1.46°**（板半宽只有 1.29°）⇒ 半数以上必偏。
这正好对应现场"偏了半个靶宽、方向一致"的现象。

**落地**：
- `src/task_common.hpp`：`calc_aim(...)` 增加 `double t_extra = 0.0`，预测时间改成 `t_flight + t_extra`；
- `task_2.cpp` / `task_3.cpp`：`DSH_FIRE_DELAY`（秒，**默认 0 = 不改原行为**）。

### ④ 开火门的容限是"线性距离"，不是固定角度 —— `shooter.cpp:19-41` + `configs/standard3.yaml:49-52`

```cpp
auto tolerance = (dist > judge_distance_) ? second_tolerance_ : first_tolerance_;
if (std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&   // 命令突变 → 不该射
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&     // 云台追上了上一帧的命令
    aimer.debug_aim_point.valid) { last_command_ = command; return true; }
```
官方参数：`first_tolerance: 3°`（<2 m）、`second_tolerance: 2°`（>2 m）、`judge_distance: 2 m`。
换算成米：**3°×2 m ≈ 105 mm、2°×3 m ≈ 105 mm** ⇒ **官方用的是恒定的线性容差 ~105 mm**。

**为什么关键**：本工程固定 `arrive_tol_deg = 0.8°`，在 3.15 m 处只有 **44 mm**，比官方严 **2.4 倍**
⇒ 这就是"门限永远打不开、`blocked` 一直涨"最可能的原因；而且**门限直接加进开火瞬间的瞄准误差**
⇒ 放宽到 1.5° 就超过板半宽 1.29°，等于放注定打不中的弹出去。

**落地**：`DSH_ARRIVE_MM`（毫米，默认 0 = 保持原角度门限；>0 时按距离换算成角度），
并加了官方那个**命令突变保护**（`cmd_jump < 2×容限` ⇒ 这一枪不打，等下一帧命令稳住）。

### ⑤ pitch 的取反放在 aimer 层，不在 io/gimbal —— `aimer.cpp:121` vs `io/gimbal/gimbal.cpp:86`

```cpp
// aimer.cpp:121
double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负
// io/gimbal/gimbal.cpp:86 —— 官方这里【不取反】
tx_data_.pitch = VisionToGimbal.pitch;
```

**为什么关键（直接关系"框架能不能改"）**：本工程把取反放在 `io/gimbal/gimbal.cpp:111`（**框架层**），
如果评测要求框架保持官方原样，俯仰就整个反了。
**官方的做法（取反放在 src/ 的 aimer 层）就是"下沉到 src"的现成范式** ⇒
把取反从 `io/gimbal` 挪到 `src/task_*.cpp`，`io/gimbal/gimbal.cpp` 就能回到官方原样，
**框架改动清单从 11 个文件降到 10 个**，而且不再有"框架被还原 ⇒ 25 分的 task_1 俯仰反了"的风险。

---

## 二、顺带对上的几处（我们的 vs 官方的）

| 量 | 官方 sp_vision_25 | 本工程 | 判断 |
|---|---|---|---|
| 状态向量 / 板位置公式 | `x vx y vy z vz a w r l h`，`center − r(cos a, sin a)` | **完全一致** | ✓ 我们就是它的后代 |
| EKF 过程噪声 | 车体 `v1=100, v2=400`（前哨站 10 / 0.1） | `kV1Car=4, kV2Car=16` | 我们的**小 25 倍** ⇒ 模型更"信模型"、w 收敛更慢。**可试的一个旋钮**（PlotJuggler 看 w 曲线收敛） |
| `auto_fire` 开关 | yaml 里有 `auto_fire: true` | 无 | 官方**框架自带"手动开火"模式** ⇒ 本工程"按住 F"是同源的正当做法 ✓ |
| 目标编号 / 左右枪 | `io::ShootMode`（left/right）+ 左右枪不同 yaw_offset | 只有 target 字段 | 英雄双枪才需要，本作业用不上 |
| 官方实测命中率 | 国赛 **39.6% 最高、≥30% 正常**（2 m、7/14 rad/s） | — | **期望值标尺**：我们 4 rad/s 应在 50~60%，9 rad/s 掉到 30~50% |
| 官方击杀时间 | 300 HP、2 m、7 rad/s ≈ 8 s；14 rad/s ≈ 10 s | — | 大作业若有击杀时间口径，这就是基准 |

---

## 三、这次新增的开关（全部默认"不改原行为"）

| 环境变量 | 默认 | 作用 | 出处 |
|---|---|---|---|
| `DSH_AIM_SELECT` | `current` | `arrival` = 在"到达时刻"选板 | 官方 aimer.cpp:61 |
| `DSH_FIRE_DELAY` | `0` | 预测里加上的出弹延时（秒） | 官方 aimer.cpp:55 |
| `DSH_ARRIVE_MM` | `0` | 到位容限按线性毫米给（0 = 保持固定角度） | 官方 shooter.cpp:27 |

启动日志会打出来，改了参数重跑一次就能确认生效：

```
[task2] 出弹延时补偿 = 0.000 s｜选板 = 当前(current)｜到位容限 = 0.80°（固定角度）
```

`fire-gate` 行新增 `t_arr=xx ms`（到达时间）、`plate=(现朝向→到达朝向)`；
`arrive` 行新增 `eff`（实际生效的容限）与 `jump`（命令突变度）。
HUD 第二行也加了 `facing=现→到达` 和 `t_arr`。

---

## 四、A/B 试验顺序（一次只动一个变量）

| 轮 | 变量 | 命令 | 判据 |
|---|---|---|---|
| 基线 | 全关 | `DSH_LOW_PLATE=0 DSH_FIRE_KEY=always DSH_ARRIVE_DEG=1.2 DSH_LEAD=0.057` | 记 hits/shots |
| A1 | 到达时刻选板 | 上面 + `DSH_AIM_SELECT=arrival` | 命中率↑；日志里 `plate=(现朝向→到达朝向)` 应出现"现朝向为负、到达朝向≈0" |
| A2 | 出弹延时 | 上面 + `DSH_FIRE_DELAY=0.05`（再试 0.10） | 落点系统性偏晚的量应随它线性改善 |
| A3 | 线性容限 | 上面 + `DSH_ARRIVE_MM=105`（= 官方口径） | `blocked` 不再一直涨；`errY` 能进容限 |
| A4 | 低位靶 | 上面 + `DSH_LOW_PLATE=1` | 命中率↑（代价：机会减半） |
| A5 | 人工门 | `DSH_FIRE_KEY=toggle` | 省额度 |

> **A2 的判据是"落点随延时线性移动"**（官方说的"拍摄慢动作视频、比较击杀时间"那套），
> 而不是"打了就中"——因为 0.1 s × 14.55°/s = 1.46°，比容差还大，方向对了才谈命中率。

---

## 五、明确**没有**搬过来的（以及为什么）

| 官方的 | 为什么没搬 |
|---|---|
| **自瞄轨迹规划器**（TinyMPC / 显式搜索，`planner/planner.cpp` + `tasks/auto_aim/planner/`） | 需要 TinyMPC 第三方库 + 状态空间规划 + 云台加速度模型，量级远超本作业；它解决的是"轨迹突变时的提前减速" |
| **"提前减速"策略**（未来要换板时提前减速向下一块过渡） | 思路可以以后做；本工程云台是"绝对角 + 到位门控"，没有速度/加速度规划，减不了速 |
| **整车（反陀螺）预测模型** | 官方是给高速电容陀螺用的第二套模型；本作业的 3~9 rad/s 靶机用板级预测够用 |
| **左右枪不同 yaw_offset**（`ShootMode`） | 英雄双枪专用 |
| **官方 EKF 的 `v1=100 / v2=400`** | 直接抄会把 w 估得更"跳"；建议作为**旋钮**试（PlotJuggler 看 `w` 收敛），不当默认值 |

---

## 六、引用

- 官方仓库：`github.com/TongjiSuperPower/sp_vision_25`（MIT）
- 官方 RM2025 开源说明（轨迹视角理论、命中率 39.6%、延迟清单）：`bbs.robomaster.com/article/803315`
- 本副本的其它文档：`docs/debug_flow_v3_副本.md`（现场流程）、`docs/官方初始代码差异表.md`（官方初始代码对照）、
  `docs/复核稿第345条_已落记录.md`（第 3/4/5 条）、`docs/debug_flow_v2_勘误表.md`（原流程勘误）
