# 改动记录与回退手册（2026-10-20 第 1 版）

> 这份文件回答四个问题：**改之前出了什么问题、为什么这么改、改完可能出什么新问题、想退回去怎么退。**
> 所有的“根因”都指向具体代码位置，不写“可能是”“也许是”。
> 配套文件：现场执行卡 `docs/field_card.md`、故障排查手册 `docs/troubleshooting_sop.md`、
> 本文件对应的完整补丁 `docs/patches/changes_1020.patch`（377 行 / 27.7 KB，相对 `HEAD`）。

---

## 0. 一句话现状

**代码和配置都已改到“只打正对着的那块板、到位才开火”的状态，但一行都还没在车上编译过、没打过一发。**
第一次上车的验证点写在 §6。

---

## 1. 总表

| # | 现象（现场看到的） | 根因 | 改动 | 状态 | 一条命令回退 |
|---|---|---|---|---|---|
| A1 | 静止靶上方 23° 掠过，HUD `pitch 0.20` | **已定案（10-21）**：测试 offset 是否生效时遗留的 10.24 没改回，车机旧版源码（`bak_aim`）真的读它（候选④） | yaml 两个 offset 保持 0.0（本次起成为活参数） | 归零后静止靶实测“几乎正中靶心”（§10.5） | ⚠️ **不要**再用 `DSH_PITCH_OFFSET` 回退（现在真的会推角度） |
| A2 | 检测到的是蓝板，框的位置对 | 颜色映射按实车改过 | 上游 `1fc2ae6` 已含 | 已在工作区 | `DSH_COLOR_ORDER=blue` |
| A3 | `bullet_speed` 四场完全相同 23.014441 | 电控给的是常数，不是实测 | **未改**，只记录 | 待确认 | — |
| B1 | `rounds 333`，一整个挡位打空枪 | `fire` 没校验“这一帧有没有观测” | `would_fire` 加 `has_detection` | 未上车 | 无（这是修 bug） |
| B2 | 云台还在追就开火，落点差好几度 | `arrive_tol_deg` 默认 0 = 门控关闭 | 默认 0.8° / 1.28°，连续 3 帧 | 未上车 | `DSH_ARRIVE_DEG=0` |
| B3 | 门控挡住 12 发后自动解除，后面全放行 | `arrive_watchdog_shots` 默认 12 | 默认 0 = 永不解除 | 未上车 | `DSH_ARRIVE_WATCHDOG_SHOTS=12` |
| B4 | 板转到 70° 侧面还在打 | 没有“板面朝向”判断 | `\|facing\| ≤ 25°` 才放行 | 未上车 | `DSH_PLATE_FACING_DEG=180` |
| B5 | 对着同一块板连打，串弹落在空当 | 只有全局发弹间隔 | 一块板一发，摆出 35° 或换板才重上膛（每帧现算，静止靶豁免） | 未上车 | `DSH_PLATE_REARM_DEG=0.01` |
| B6 | 0.2 s 内同一块板连吃两发 | 真正原因是 B5 的“一块板一发”没落地，不是间隔太短 | **保持 0.2 s**（5 Hz 照抄赛规），防连打交给 `plate_armed` | 未上车 | 无需回退 |
| B7 | 第一发迟迟不出 | 15 帧 @45 fps = 0.33 s | `min_frames_for_fire` 15 → 10 | 未上车 | `sed` 改回 15 |
| B8 | 电控 IDLE 时软件还在请求开火 | `require_auto_aim_mode: false` | task2 改 `true` | 未上车 | `sed` 改回 `false` |
| B9 | 追着快转出视野的那块打 | `aim_id = last_id` 只按角度最近 | 改取 `\|facing\|` 最小的板（5° 迟滞） | 未上车 | 改 `if (false && ...)` 一行 |
| C1 | 准星被推 12.6°（5 块板） | 把自转角速度当视在角速度，放大 16 倍 | 改成 `v=ω×r` 折算 | 未上车 | `DSH_LEAD=0` |
| C2 | 板往右转却往左补 | 近侧板视在角速度与 `w` **反号** | 同上，符号一并修正 | 未上车 | `DSH_LEAD=0` |
| C3 | lead 该填多少全靠猜 | 没量过链路延迟 | 视觉 24.9 + 云台 31.8 = 57 ms | 已量（离线） | `DSH_LEAD=0` |
| D1 | 一开录像帧率 58 → 44.7（−23%） | JPEG q95，250–400 KB/帧 | 调 lead 的场次必须不录像 | 已确认 | `record_dir: ""` |
| D2 | `DSH_RECORD_DIR=""` 关不掉录像 | `env_dir` 把“未设”和“设空”都返回空 | 用 `unset`，或改 yaml | 已确认 | — |
| D3 | 四场录像里一发没打、也没日志 | 挡位 IDLE + 没 `tee` | 现场纪律：必须 `2>&1 \| tee logs/*.log` | 已确认 | — |
| D4 | PlotJuggler 连不上 | `inet_addr("127.0.0.1")`，只能在车机本机 | 车机上开 PlotJuggler | 已验证可用 | — |
| D5 | `pwsh` 不是命令 | 这台笔记本只有 Windows PowerShell 5.1 | 文档改 `powershell -ExecutionPolicy Bypass -File` | 已改 | — |
| D6 | 命令贴到车上报 `D:DSHfp_aim` 不存在 | PowerShell 命令敲进了车机 bash | 文档全部标注 ```powershell``` / ```bash``` | 已改 | — |
| D7 | `git fetch` 报 `SEC_E_NO_CREDENTIALS` | schannel 后端 | `git -c http.sslBackend=openssl fetch` | 已验证 | — |
| D8 | tar 打出来只有 1024 B | 在 `C:\Users\MTG` 下打包，相对路径不存在 | 先 `cd` 到工程目录再打包 | 已解决 | — |
| D9 | `&&` 链在 `grep -c` 之后断掉 | `grep -c` 数到 0 时返回码 1 | 多条验证命令用 `;` 分隔 | 已改 | — |
| D10 | 设了 `DSH_STEP_DEG` 一发都不发 | `fire = ... && !step_enabled()` | 阶跃测试用完必须 `unset` | 已写进文档 | `unset DSH_STEP_DEG` |

---

## 2. 逐条说明

### A1 `pitch_offset` 的探针值（**根因已定案 2026-10-21，见下方“定案”**）

- **现象**：HUD `AIM yaw 0.11 pitch 0.20 dist 3.40`，pitch 0.20 rad ≈ 11.5°，而目标在画面中心下方约 12°，
  等于往上偏 23°；3.4 m 处相当于 1.4 m，“从目标上方很远处掠过”。
- **原写法（❌ 已被核验推翻）**：`aim_pitch = calc_aim(≈+1.2°) + pitch_offset(10.24°)`。
- **核验更正（2026-10-20 外部核验，我复核属实）**：
  `git show HEAD:src/task_2.cpp` 与 `src/task_2.cpp.bak` 里 **`yaw_offset`/`pitch_offset` 出现 0 次**，
  旧代码只读 `muzzle_offset_x`。也就是说这两个键在本次改动之前是**死配置**，
  **归零本身什么也没修**，23° 掠过不能用 offset 解释；**真因未定位**。
  候选（按可能性排）：① pitch 符号/约定（+11.5° 与 −12° 的**大小几乎相等而符号相反**，这个吻合更像符号问题）；
  ② 云台/相机零位；③ 弹道模型；④ 车上跑的是另一版源码（车机有自己的 `task_2.cpp.bak_aim`）。
- **定案（2026-10-21，用户确认）**：真因就是**遗留的探针值**——当年为测试 offset 是否生效，
  把 `pitch_offset` 设成 10.24，测完一直没改回 0；车机当时跑的那版源码（`task_2.cpp.bak_aim`）
  **确实读这两个键**，所以 10.24 真的生效了：HUD pitch 0.20 rad ≈ 11.5° ≈ calc_aim(+1.2°) + 10.24°，
  向上 11.5° 加上目标在中心下方 12°，正是 23° 掠过。**候选④成立**；上面的“原写法”与“核验更正”
  其实都对，只是版本不同（车机旧版读 offset、笔记本 HEAD 不读）。
  **归零已被实车验证**：§10.5 静止靶 3.1 m（offset 归零后）`pitch −0.00`、落点“几乎正中靶心”——
  候选①（pitch 符号约定错）随之排除，符号若反，归零后不可能正中。
  §6 第 0 步保留，但性质从“查真因”降级为**新代码的零成本自检**。
- **改法**：`configs/standard.yaml` 的两个 offset 保持 `0.0`。注意**语义变了**：
  本次改动新增了读取逻辑，它们从死参数变成**活参数**，填任何非零值都会立刻推角度。
- **改完的风险**：⚠️ 以前 `DSH_PITCH_OFFSET=10.24` 是空操作，**现在真的会把俯仰推 10°**。
  **禁止再用它当回退或实验手段**；真要补固定角偏差，用静止靶实测 + `zero_fix.py` 算出来的数。
- **验证**：启动日志 `[task2] offset yaw=0.00(yaml) pitch=0.00(yaml)`。
- **若上车后静止靶仍偏**：按 **pitch 符号约定 → 零位 → 弹道模型** 的顺序查，**不要往 offset 里填数**。
  最快的符号判据：把靶子放在**画面中心**，HUD 的 `pitch` 应 ≈ 0.00；若明显非零，就是符号或零位问题。

### A2 颜色映射

- **现象**：蓝板被判成红板 / 反过来。
- **根因**：训练时类别顺序是 `0=红, 1=蓝`，实车模型是反过来。
- **改法**：上游 `1fc2ae6` 加了 `to_color(int)`（[armor.cpp](../tasks/auto_aim/armor.cpp)），本工作区已经有这段。
- **回退**：`DSH_COLOR_ORDER=blue` 一条环境变量切回旧映射。
- **验证**：视野里放一块红板，看终端那句“视野里没有 red…”的多寡。

### A3 弹速不可信（**没改，只记录**）

- **现象**：四场 `frames.csv` 里 `bullet_speed` 恒为 `23.014441`，连小数位都一样。
- **含义**：电控把配置值当“实测弹速”发下来，不是真的测速。
- **影响**：`t_flight = d / bullet_speed` 直接决定俯仰和飞行时间补偿；如果真实弹速是 20 而按 23 算，
  3 m 处会差 30 ms，折合 **2.9 块板**的提前量（左右方向），俯仰上则表现为偏低。
- **判据**：日志出现 `电控弹速 ... 不可用，改用配置兜底值` 就是没拿到真实值。
- **动作**：**先不动**。等命中稳定后，用 2 m / 4 m 两个距离各打 3 发，看偏移是否随距离翻倍来反推真实弹速。

---

### B1 没有观测也开火（`rounds 333`）

- **现象**：某一挡 `rounds` 一路涨到 333，车上 `bullet_count` 一动不动。
- **根因**：目标丢失超过 `target_hold_ms` 后 `holding` 变假，但 `has_target` 和滤波器状态原样保留
  （`is_converged_` 停在最后一次 update 的值），于是 `fire` 在视野里什么都没有的情况下持续为真。
  333 × 0.2 s ≈ 67 s，正好一整场。
- **改法**：`would_fire` 加 `has_detection`（[task_2.cpp](../src/task_2.cpp) 的 `would_fire` 那一行）。
- **风险**：几乎没有 —— 这一帧真的没检测到，本来就不该开火。
- **回退**：无（属于修 bug）。
- **验证**：把靶子拿走，`rounds` 必须停住不动。

### B2 到位门控默认是关的

- **现象**：云台还在追、`errY` 还有几度的时候就开火。
- **根因**：`arrive_tol_deg` 缺省 0 ⇒ 门控完全不生效。
  这是唯一一个**直接管命中率**的阈值，默认却是关的。
- **改法**：缺省 **0.8°**（yaw）/ **1.28°**（pitch = 0.8 × 1.6），并且**连续 3 帧**都在门限内才放行。
  - 0.8 的来源：4 rad/s 下云台的稳态跟随滞后约 0.45°，取 1.6 倍只拦“明显没跟上”的瞬间；
    卡到 0.5 以下会把正常滞后也挡掉，一发都打不出去。
  - pitch 单独放宽：pitch 轴还要拖着零点补偿去追，常比 yaw 慢。
  - 连续 3 帧（≈50 ms）：只判单帧的话，云台扫过门限那一瞬间就会被判到位。
- **改完的风险**：**可能一发都不发**。云台跟不上时门控会一直挡。判据：`[dbg] arrive=` 行的
  `blocked=` 一直涨而 `rounds` 不动，5 秒没弹就 Ctrl+C。
  现场处置：`DSH_ARRIVE_DEG=1.5 DSH_ARRIVE_PITCH_DEG=2.0`。
- **回退**：`DSH_ARRIVE_DEG=0`（等于恢复旧行为）。
- **验证**：`[dbg] arrive=true tol=0.80/1.28° errY=0.31° ... n=5` —— `n` 是连续帧数。

### B3 看门狗会自动解除门控

- **现象**：门控挡了 12 发之后自动解除，之后每发都放行，等于没有门控。
- **根因**：`arrive_watchdog_shots` 缺省 12。而任务二每挡只有 10 发额度 ——
  看门狗永远在**第一挡**就触发，后面两挡全是裸奔。
- **改法**：缺省 **0 = 永不自动解除**。要旧行为就显式给正数。
- **改完的风险**：门控卡死时没有后备保护。现场纪律改成“**5 秒没弹就 Ctrl+C**”，不再靠看门狗。
- **回退**：`DSH_ARRIVE_WATCHDOG_SHOTS=12`。

### B4 板面朝向门控

- **现象**：板已经转到侧面（70–80°）还在打，有效靶面只剩 23 mm。
- **根因**：没有任何“这块板现在是不是正对着我”的判断。
- **改法**：`facing = a + π − 视线方向`，`|facing| ≤ 25°` 才放行。
  四块板 90° 一块 ⇒ 正对窗口约 50°。
- **改完的风险**：
  1. 转速很快（7 rad/s）时窗口只剩 50/400 ≈ 125 ms，中挡（9 rad/s）约 97 ms（≈5–6 帧），可能整圈都等不到；
  2. **facing 的朝向约定若反了，会去瞄背对我们那块板** —— 这是最严重的可能，见 §6 的自检。
- **回退**：`DSH_PLATE_FACING_DEG=180`（等于关闭）。
- **验证**：`[dbg] fire-gate ... plate=true(12°) assoc=8°`；`plate=` 里的角度应该长期在 ±25 内。

### B5 一块板只打一发

- **现象**：对着同一块板连续打，后面的弹在板已经转走之后才到，全落在空当里。
- **根因**：只有全局 `fire_interval_s`，没有“这块板打过没有”的状态。
- **改法**：`last_fired_plate` + `plate_armed`：打完这一发，同一块板必须**摆出 35°**
  （`DSH_PLATE_REARM_DEG`）或者**换成别的板**，才重新允许开火。
  **必须是“每帧现算”而不是置位保持**（核验抓到两个洞，已改）：

  ```cpp
  const bool target_static = std::abs(x[7]) < 0.5;          // 静止靶豁免
  plate_armed = target_static || (aim_id != last_fired_plate) ||
                (std::abs(facing_deg) > plate_rearm_deg);
  ```

  - **洞①（严重）**：置位保持版在**静止靶**上永不重新上膛 —— 靶子不转，摆不出 35°，
    打完第一发就永久锁死。而零位标定偏偏要“静止靶打 3 发”，流程和代码直接打架。现在用 `|w| < 0.5` 豁免。
  - **洞②（轻微）**：`aim_id` 从 A 跳到 B 再跳回 A 时，A 会把“已上膛”带回来。改成每帧现算后自然消失。
- **改完的风险**：低转速时“同一块板再打一发”要等它转满一圈；高转速时会很快跳到下一块板。
  两种都属于预期行为（每块板转正时打一发）。静止靶上则完全不限“一块一发”，只受间隔和朝向约束。
- **回退**：`DSH_PLATE_REARM_DEG=0.01`（几乎任何角度都重新上膛，等于关闭）。

### B6 `fire_interval_s` 保持 0.2（**曾改成 0.35，已撤回**）

- **根因**：0.2 s 时 4 rad/s 下板子转过 46°，看上去“还没换板就又能打”。
  但真正的病根是 B5 没落地（没有“一块板一发”），**不是间隔太短**。
- **改法**：**保持 0.2 s**（照抄赛规的 5 Hz）。防连打交给 `plate_armed`：
  打完一发后要摆出 35° 才上膛，而 35° > 25° 的 facing 窗口 ⇒ 出窗口前不可能再打同一块。
  0.35 s 只有一条 yaml 注释支持（而那条注释是我自己写的，属于自证），没必要冒这个险。
- **改完的风险**：无（比 0.35 s 更贴近赛规）。若现场发现同一块板仍连吃两发，先查 `plate_armed` 的日志字段，别加间隔。
- **回退**：不需要。

### B7 `min_frames_for_fire` 15 → 10

- **根因**：15 帧在 58 fps 下是 0.26 s，但开录像掉到 45 fps 就变成 0.33 s，白丢 1–2 发。
- **改法**：10 帧（0.17–0.22 s），够滤波收敛，又不至于把第一发拖太久。
- **改完的风险**：收敛早期可能提前开火（残差还大）。缓解：`stable` 里还有 NIS 失败率这道条件。
- **回退**：`sed -i 's/min_frames_for_fire: 10/min_frames_for_fire: 15/' configs/standard.yaml`。

### B8 `require_auto_aim_mode` false → true

- **现象**：电控被拨回手动（IDLE）时，软件还在请求开火，`rounds` 一路涨但弹不出来。
- **改法**：task2 改成 `true`（task3 的没动）。
- **改完的风险**：**挡位只要抖一下，就完全不发弹**。如果现场电控挡位不稳，这条要改回 `false`。
- **回退**：`sed -i '0,/require_auto_aim_mode: true/s//require_auto_aim_mode: false/' configs/standard.yaml`
  （`0,` 是为了只改 task2 那一行，不动 task3 的 false）。

### B9 瞄点选板：从“角度最近”改成“最正对”

- **现象**：准星压在右边缘那块、马上要转出视野的板上，中间那块不打。
- **根因**：`aim_id = target.last_id`，而 `last_id` 是 EKF 按**角度最近**关联出来的，
  **完全不看板面朝哪**。四块板 90° 一块，侧面 80° 的板照样能被关联上。
- **改法**：每帧在 `xyza_list` 里取 `|facing|` 最小的那块；只有比当前好 **5° 以上**才换（防临界抖动）。
- **改完的风险**：换板瞬间 `aim_id` 变化 ⇒ `plate_armed` 重新置真 ⇒ 可能“刚打完这块立刻打下一块”。
  这是预期（每块板各自打一发）。
- **回退**：把 `if (best_id != aim_id && ...)` 改成 `if (false && ...)`（一处，要重编）。
- **验证**：准星应该落在**有检测框**的那块板上；如果跑到对面没框的位置，说明 facing 约定反了（§6）。

---

### C1/C2 lead 的量纲与符号

- **现象（旧）**：`DSH_LEAD=0.055` 时准星被推 12.6° ≈ 5 块板，全空。
- **根因（量纲）**：`x[7]` 是**目标自转角速度**（rad/s），直接乘进 `aim_yaw` 相当于把它当成了
  **相机看到的角速度**；两者差 `d/r ≈ 16` 倍。
- **根因（符号）**：板在 `center − r(cos a, sin a)`，`a≈0` 时板的速度是 `(0, −r·w)`：
  **板往画面右走（视在 yaw 变小）对应的却是 `w > 0`**。所以 `+= w·lead` 一直是在往反方向补。
- **改法**：按板的速度向量算视在角速度：

  ```cpp
  v = ω ẑ × (plate − center);
  aim_yaw += (x·vy − y·vx) / h² * lead_s;
  ```

  量纲、符号、板走到哪个方位都自动正确。
- **`DSH_LEAD` 的单位随之变回“秒”**：`0.057`（= 视觉 25 ms + 云台 32 ms）。
  **旧值 `0.0036` 作废**：它是给错公式打的补丁，套到新公式上只剩 0.05°，等于没补。
- **改完的风险**：
  1. **如果实车 `w` 的符号和推导相反**（例如 EKF 的角度约定与位置公式不一致），
     修正会**反着加**，误差从 0.83° 变成 −0.83°，看不出差别但方向错。判据见 §6 的 10 倍放大法。
  2. 修正量只有 0.83°（1/3 个靶宽），**不要指望它单独把命中率拉起来** ——
     开机前偏得厉害的主因是云台滞后（`errY`）和零位，不是 lead。
- **回退**：`DSH_LEAD=0`（等于不补，其它逻辑不受影响）。

---

### D 组：采集与工具链的坑（不影响命中率，但会浪费现场时间）

| 坑 | 一句话 | 正确做法 |
|---|---|---|
| D1 录像掉帧 | JPEG q95 每帧 250–400 KB，60 fps ≈ 15–24 MB/s，实测 58 → 44.7 fps | 调 lead 的场次**不录像**：`unset DSH_RECORD_DIR` + yaml `record_dir: ""` |
| D2 环境变量关不掉录像 | `tools/replay.cpp` 的 `env_dir` 把“未设”和“设成空串”都返回空，再回落到 yaml | 只能 `unset` 或改 yaml |
| D3 白录一场 | 四场 `bullet_count` 全是 11、`mode=0`（IDLE），且**没存日志**（`grep 已录制` 为空） | 必须 `2>&1 \| tee logs/xxx.log` |
| D4 PlotJuggler | `inet_addr(host)` 缺省 `127.0.0.1`，且 `has_target` 为假时根本不发包 | 在**车机本机**开 PlotJuggler，UDP Server 9870 |
| D5 `pwsh` | 这台笔记本没有 PowerShell 7 | `powershell -ExecutionPolicy Bypass -File ...` |
| D6 命令贴错机器 | PowerShell 命令贴进车机 bash，`D:\` 被当成主机名 | 文档按 ```powershell``` / ```bash``` 标注；`ssh "..."` 里的命令在车上执行 |
| D7 git fetch | schannel 后端报 `SEC_E_NO_CREDENTIALS` | `git -c http.sslBackend=openssl fetch origin --prune` |
| D8 tar 空包 | 在 `C:\Users\MTG` 下打包，相对路径不存在，bsdtar 静默产出 1024 B | 先 `cd` 到工程目录 |
| D9 `grep -c` 断链 | 数到 0 时返回码 1，`&&` 后面的命令不执行 | 用 `;` 分隔 |
| D10 `DSH_STEP_DEG` | `fire = ... && !step_enabled()`，设了就永远不发弹 | 阶跃测试后必须 `unset` |

---

## 3. 环境变量总表（缺省值以代码为准）

| 变量 | 缺省 | 作用 | 想关掉就设成 |
|---|---|---|---|
| `DSH_ARRIVE_DEG` | **0.8** | 到位门控 yaw 门限（度） | `0` |
| `DSH_ARRIVE_PITCH_DEG` | 0.8 × 1.6 = **1.28** | 同上 pitch 门限 | `0` |
| `DSH_ARRIVE_FRAMES` | **3** | 连续多少帧在门限内才放行 | `1` |
| `DSH_ARRIVE_WATCHDOG_SHOTS` | **0** | 挡满 N 发自动解除（0 = 永不） | `0`（本来就是） |
| `DSH_PLATE_FACING_DEG` | **25** | 板面正对窗口 | `180` |
| `DSH_PLATE_REARM_DEG` | **35** | 一块板重新上膛需要摆出的角度 | `0.01` |
| `DSH_LEAD` | **0** | 链路延迟补偿（**秒**） | `0` |
| `DSH_YAW_OFFSET` / `DSH_PITCH_OFFSET` | 不设 = 用 yaml 的 0.0 | 覆盖 yaml 的固定角偏置（度） | `0` |
| `DSH_RECORD_DIR` | 不设 = 用 yaml | 录像目录 | `unset` + yaml `""` |
| `DSH_STEP_DEG` | 不设 | 阶跃激励（调试云台） | `unset` |
| `DSH_COLOR_ORDER` | 不设 = 新映射（0 红 1 蓝） | 颜色映射 | `blue` = 旧映射 |
| `DSH_LAT_CSV` / `DSH_DET_CSV` | 不设 | 延迟探针 / 检测日志 CSV | `unset` |

---

## 4. 回退（三种粒度，从轻到重）

### 4.1 只回退一个行为（不重编、不改文件）

```bash
DSH_ARRIVE_DEG=0 ~/run2.sh --no-build                     # 回到“没有到位门控”
DSH_PLATE_FACING_DEG=180 ~/run2.sh --no-build             # 回到“不看板面朝向”
DSH_PLATE_REARM_DEG=0.01 ~/run2.sh --no-build             # 回到“可以对着同一块连打”
DSH_LEAD=0 ~/run2.sh --no-build                            # 回到“不补提前量”
DSH_ARRIVE_WATCHDOG_SHOTS=12 ~/run2.sh --no-build         # 回到“挡 12 发就自动解除”
```

### 4.2 回退 yaml 里的某一项（不用重编）

```bash
cd ~/fp_aim
sed -i 's/min_frames_for_fire: 10/min_frames_for_fire: 15/'   configs/standard.yaml
sed -i '0,/require_auto_aim_mode: true/s//require_auto_aim_mode: false/' configs/standard.yaml
```

### 4.3 整个文件回到出厂

- **笔记本**：`git apply -R docs/patches/changes_1020.patch`（把 `task_2.cpp` + `standard.yaml`
  退回到 `HEAD` 的样子），或者 `git checkout -- src/task_2.cpp configs/standard.yaml`。
  ⚠️ 这会连 B1–B9、C1–C2 一起退掉，等于回到“偏移 10°、没有门控”的原始状态。
- **车机**：车上有一份 `src/task_2.cpp.bak_aim`（改动前的备份），
  `cp ~/fp_aim/src/task_2.cpp.bak_aim ~/fp_aim/src/task_2.cpp` 再 `make -j2 task_2`。
- **只退一步（推荐）**：先把当前状态存成补丁再退，
  `git diff > docs/patches/changes_$(date +%m%d_%H%M).patch`，随时可以 `git apply` 回来。

---

## 5. 改动清单（哪些文件被动过）

| 文件 | 改了什么 |
|---|---|
| [src/task_2.cpp](../src/task_2.cpp) | 到位门控（默认开 + 连续 3 帧 + 看门狗默认 0）、板面朝向门控、一块板一发、选板 argmin\|facing\|、lead 改成视在角速度、`would_fire` 加 `has_detection`、`[dbg]` 行加 `plate=`/`assoc=`、启动日志单行 |
| [configs/standard.yaml](../configs/standard.yaml) | 两个 offset 归 0、`min_frames_for_fire` 10、`fire_interval_s` 0.2（5 Hz）、`require_auto_aim_mode: true`、`max_rounds_per_stage: 0`、`record_dir: ""`、`min_confidence` 0.88、ROI 800×800 并启用 |
| [docs/field_card.md](field_card.md) | 现场执行卡 V4（八步、判据表、开火前先看两行） |
| [docs/troubleshooting_sop.md](troubleshooting_sop.md) | §9 完全不开火、§11 开火门控、§20 从录像到命中、约定自检 |
| [docs/onsite_record.md](onsite_record.md) | 现场记录表（P0–P7 每阶段记什么） |
| [tools/runlog.py](../tools/runlog.py) | 一场一个 JSON + 对比表 + CSV 导出 |
| [tools/plot_svg.py](../tools/plot_svg.py) | 无 matplotlib 时画 `\|ω\|` / 弹量曲线 |

**没动的**：`src/task_1.cpp`（已验收）、`src/task_3.cpp`、`tasks/auto_aim/*` 的算法、`tools/*.sh` 的逻辑。

---

## 6. 还没验证的事（上车按顺序做）

| 顺序 | 验证什么 | 怎么做 | 判据 | 不对怎么办 |
|---|---|---|---|---|
| 0 | **pitch 符号/零位**（A1 已定案为遗留探针值；此步降级为新代码自检，仍先做） | 靶子放在画面**正中**，看 HUD `pitch` | ≈ 0.00 rad（±0.05） | 明显非零 ⇒ 查 `gimbal.cpp` 的取反与 `xyz2ypd` 约定，**不要填 offset** |
| 1 | **能不能编过** | `touch src/task_2.cpp && cd build && make -j2 task_2` | 看到 `Building CXX object .../task_2.cpp.o` + `Linking CXX executable task_2` | 把报错贴出来 |
| 2 | **facing 约定对不对** | 跑起来 `grep -m3 'fire-gate'` | `assoc=` 在 **0–40°** | 长期 ±150° ⇒ 删掉 `facing_of` 里的 `+ M_PI` |
| 3 | **准星落在有框的板上** | 肉眼看画面（**注意：画面画的是检测层，不是 aim_id**） | 小黄圈在**有检测框**那块板上 | 跑到没框那侧 ⇒ 约定反了 |
| 4 | **lead 符号对不对** | `DSH_LEAD=0` 与 `DSH_LEAD=0.57` 各跑 10 秒，读 HUD `AIM yaw` | `w>0` 时第二行**小约 8.3°** | 反而变大 ⇒ 符号仍反，别发弹 |
| 5 | **门控会不会卡死** | 打 3 发 | 5 秒内出弹 | `blocked=` 一直涨 ⇒ 临时 `DSH_ARRIVE_DEG=1.2`，事后必须回头修云台跟踪 |
| 6 | **会不会连打同一块板** | 打完一轮看 `[dbg] fire-gate` | 每发之间 `plate=` 先摆出 35° 或换板 | `DSH_PLATE_REARM_DEG=45` |
| 7 | **静止靶能不能连打**（新增） | 静止靶连打 3 发 | 3 发都能出弹 | 只出 1 发 ⇒ `plate_armed` 逻辑没生效，查 `\|w\|<0.5` 豁免 |
| 8 | **弹速是不是常数** | `grep -m3 '电控弹速' logs/*.log` | 有真实数值 | 出现“兜底值” ⇒ 命中会系统性偏低 |

---

## 7. 关键数值备查

| 量 | 值 | 来源 |
|---|---|---|
| 小装甲板宽 / 灯条长 | 135 mm / 56 mm | `tasks/auto_aim/solver.cpp` |
| 3.15 m 处板半宽张角 | 1.29°（67.5 mm） | `atan(0.0675/3.15)` |
| 板在相机里的**视在**角速度 | `r·\|w\|/d ≈ 14.5°/s` @4 rad/s、3.15 m | 推导 |
| ⇒ 1° 视在偏差 | **70 ms** | 同上 |
| 飞行时间 | 3.15 m / 25 m/s = **126 ms** | `calc_aim` |
| 飞行时间内板转过的相位 | 4 × 0.126 = 0.5 rad = **29°** | 同上 |
| 折算成相机角 | **1.8–2.0°**（≈0.75 个靶宽） | 同上 |
| 视觉延迟 p50 | **24.9 ms**（63 fps 场） | `latency_report.py logs/lat_1510.csv` |
| 云台反馈滞后（互相关） | **31.8 ms**（相关系数 0.71） | 同上 |
| ⇒ `DSH_LEAD` | **0.057 s** → 4 rad/s 下 **0.83°** | 24.9 + 31.8 |
| 到位门限 | 0.8° / 1.28°，连续 3 帧 | 4 rad/s 稳态滞后 ≈0.45° 的 1.6 倍 |
| 一块板重新上膛 | 35° | 比 facing 窗口 25° 大 10°，防抖 |
| 发弹间隔 | **0.2 s（5 Hz，照抄赛规）** | 防连打靠 `plate_armed`，不靠间隔 |
| 录像对帧率的影响 | 58 → 44.7 fps（−23%） | `[Recorder] 已录制 频率 44.7 帧每秒` |

---

## 8. 代码、文档、数据都存在哪

### 8.1 三份副本

| 位置 | 路径 | 当前版本 | 用途 |
|---|---|---|---|
| **笔记本（唯一改动源头）** | `D:\DSH\fp_aim` | 分支 `offline_replay`，HEAD `5b769f5` | 所有编辑都在这里做 |
| **车机** | `rm@192.168.1.50:~/fp_aim` | 由笔记本 `scp` / `tar` 覆盖 | 编译、跑车、打靶 |
| **GitHub** | `mtg11123211/sp-vision-final-project` | `origin/main` = `9ecee8f` | 备份与交付 |

> **三份的关系**：车机是笔记本的子集；GitHub 的 `origin/main` 与**本地 HEAD** 的代码文件逐字节相同
> （2026-10-20 用 blob 哈希逐个比对过），差别只有 `README.md` / `.gitignore`。
> 本地 **10 个已改文件 + 16 项未跟踪，从未提交、从未推送**。

### 8.2 目录结构（这个工程里重要的东西）

```
D:\DSH\fp_aim\
├─ configs\standard.yaml          ← 唯一配置文件（阈值、挡位、录像、ROI 全在这）
├─ src\
│  ├─ task_1.cpp                  ← 云台追踪（已验收，不要动）
│  ├─ task_2.cpp                  ← 旋转装甲板（本次全部改动集中在这）
│  ├─ task_3.cpp                  ← 前哨站
│  ├─ task_common.hpp             ← calc_aim：飞行时间补偿 + 弹道
│  ├─ overlay.hpp                 ← 画面叠加（准星、检测框、HUD）【新增】
│  ├─ detect_log.hpp              ← DSH_DET_CSV 检测日志【新增】
│  └─ latency_probe.hpp           ← DSH_LAT_CSV 延迟探针【新增】
├─ io\
│  ├─ camera.cpp / .hpp           ← 相机 + 时间戳 + 录像钩子（已改）
│  ├─ hikrobot\                   ← 海康 MVS 驱动封装（已改）
│  └─ gimbal\gimbal.cpp           ← 串口协议、pitch 取反（已改）
├─ tasks\auto_aim\
│  ├─ armor.cpp                   ← 颜色映射 0红/1蓝（已改）
│  ├─ target.cpp                  ← EKF + 关联 + last_id（已改）
│  └─ solver.cpp / yolo.cpp       ← 内参外参、推理（未动）
├─ tools\                         ← 离线分析（Python，多数纯标准库）
│  ├─ latency_report.py           ← 延迟报告（视觉 + 云台互相关）
│  ├─ lead_advisor.py             ← 提前量建议
│  ├─ shot_fit.py                 ← 弹着点 → T* 拟合
│  ├─ zero_fix.py                 ← 零位修正（mm → 度）
│  ├─ analyze_frames.py           ← 录像逐帧分析
│  ├─ plot_frames.py              ← 画图（唯一需要 matplotlib 的）
│  ├─ plot_svg.py                 ← 没有 matplotlib 时画 SVG
│  ├─ runlog.py                   ← 一场一个 JSON + 对比表
│  └─ onsite\
│     ├─ run2.sh / run3.sh        ← 带 touch + 编译自检的启动器
│     ├─ collect_run.sh           ← 打包（整目录 / --meta-only）
│     ├─ pull_run.ps1             ← 笔记本侧拉包（sha256 校验 + 解包）
│     └─ disk_guard.sh            ← 磁盘清理（默认只读预览）
├─ docs\
│  ├─ field_card.md               ← 现场执行卡 V4（30 分钟八步）
│  ├─ fix_log.md                  ← 本文件
│  ├─ troubleshooting_sop.md      ← 故障手册（1500 行）
│  ├─ onsite_record.md            ← 现场记录表 P0–P7
│  ├─ record_session.md           ← 有录像那次的四场记录表
│  ├─ live_fire_plan.md           ← 发弹机会预算（10 发以内）
│  ├─ param_rollback.md           ← 参数回退清单
│  ├─ patches\changes_1020.patch  ← 本次全部改动的补丁（377 行 / 27.7 KB）
│  └─ plots\                      ← 合成数据的曲线图
├─ logs\                          ← 留存的 21 个 .log + 3 个探针 CSV
└─ recordings\                    ← 只有 synthetic_rotate / synthetic_static
```

### 8.3 传到车机要传哪些

**必须传（15 个）**：

```
configs/standard.yaml
io/camera.hpp              io/camera.cpp
io/gimbal/gimbal.cpp
io/hikrobot/hikrobot.hpp   io/hikrobot/hikrobot.cpp
src/task_2.cpp             src/task_3.cpp
src/latency_probe.hpp      src/detect_log.hpp      src/overlay.hpp
tasks/auto_aim/armor.cpp   tasks/auto_aim/target.cpp
tools/onsite/collect_run.sh  tools/onsite/disk_guard.sh
```

**另外单独传**：`tools/onsite/run2.sh`、`run3.sh`（放 `~/`，`chmod +x`）。
**按需传**：`tools/*.py`（用哪个传哪个；`plot_svg.py` / `runlog.py` 车上还没有）。

**传完的验收（用 `;` 分隔，不要用 `&&`）**：

```bash
cd ~/fp_aim
grep -c "DSH_ARRIVE_DEG" src/task_2.cpp     # 期望 5
grep -c "detect_log" src/task_2.cpp         # 期望 2
grep -c "plate_facing" src/task_2.cpp       # 期望 ≥3（本次新增）
ls -l src/overlay.hpp                       # 期望存在，约 6 KB
```

### 8.4 备份与回退材料

| 文件 | 内容 | 怎么用 |
|---|---|---|
| `docs/patches/changes_1020.patch` | 本次全部改动的 diff（377 行 / 27.7 KB） | `git apply -R` 整体回退 |
| `configs/standard.yaml.bak` | 上一版配置 | `cp` 回去 |
| `configs/standard.yaml.bak.1017_1048` | 10-17 那版配置 | `cp` 回去 |
| `src/task_2.cpp.bak`（笔记本） | 本轮改动前的源码 | `cp` 回去 + 重编 |
| `src/task_2.cpp.bak_aim`（**车机**） | 最初上车那版 | 车机上 `cp` + `make` |

### 8.5 数据在哪

| 数据 | 车机 | 笔记本 |
|---|---|---|
| 运行日志 `.log` | `~/fp_aim/logs/` | `D:\DSH\fp_aim\logs\`（21 个，都是 10-17 的） |
| 延迟探针 CSV | `~/fp_aim/logs/lat_1510.csv`（281 KB）、`step_1508.csv`（453 KB） | 同左，已拷回 |
| 录像 | `~/fp_aim/recordings/`（四场，共约 1.5 GB） | 只有 `synthetic_rotate` / `synthetic_static` |
| 打包产物 | `~/packs/` | `D:\DSH\onsite_runs\`（`pull_run.ps1` 默认落点） |

> **笔记本和车机现在不通**（`ping 192.168.1.50` 100% 丢包、ssh 22 端口超时）。要拿数据只能：
> ① 修网（同一路由器 / 网线直连 / 确认车机 IP 有没有变）；② U 盘（`lsblk` → `mount /dev/sdb1 /mnt/usb` → `cp` → `sync`）；
> ③ 只拷 `frames.csv`（几 MB，一般能过）。

---

## 9. 大作业要求（满分 100）

### 9.1 分数分布

| 任务 | 内容 | 分值 | 其中要打弹的部分 |
|---|---|---|---|
| task_1 | 云台追踪（yaw 360°、pitch ±20°） | **25** | 0 |
| task_2 | 旋转装甲板命中 + 角速度估计 | **40** = 20 命中 + 20 角速度 | 20 |
| task_3 | 前哨站击打 + 角速度估计 | **35** = 30 血量 + 5 角速度 | 30 |

> **100 分里有 50 分**（task_1 的 25 + task_2 角速度的 20 + task_3 角速度的 5）**根本不需要打中任何东西**。
> 先把这 50 分拿稳，再谈命中。

### 9.2 task_1：云台追踪（25 分，**已现场验收通过**）

- yaw 能连续转 **360°**，pitch 覆盖 **±20°**。
- 判据是“跟不跟、超不超限位、丢不丢目标”。
- **本次改动一行没碰 `src/task_1.cpp`**，就是为了不破坏这 25 分。

### 9.3 task_2：旋转装甲板（40 分）

| 项 | 要求 |
|---|---|
| 距离 | 约 **3 m**（实测 3.0–3.4 m） |
| 射击模式 | **单发** |
| 发弹频率 | **5 Hz**（0.2 s 间隔）。yaml 注释里的口径是 `>= 0.2 s`，即 5 Hz 是**上限** |
| 转速 | 低挡 **3–5 rad/s**、中挡 **5–9 rad/s**（**赛规固定，不能自己降**） |
| 发弹数 | **每个转速挡位 10 发** |
| 命中 | **1 中 1 分**，满分 **20**（两挡各 10 发全中） |
| 角速度 | 估 `\|ω\|`，低挡 **±0.6 rad/s**、中挡 **±1.0 rad/s**，占 20 分 |

**推论（决定了整个打法）**：

- 每挡只有 10 发，**打空一发就是丢 1 分**，没有补射。
- 发弹间隔按赛规的 **0.2 s（5 Hz）**；防连打靠“一块板一发”，不靠拉长间隔（见 §2 B6）。
- 20 分角速度分与命中无关，EKF 的 `w` 估得准就能拿满。

### 9.4 task_3：前哨站（35 分）

| 项 | 要求 |
|---|---|
| 目标 | 前哨站（三块装甲板，顶部板另有 z 过滤） |
| 血量 | **1500 HP** |
| 时限 | **30 s** |
| 频率 | **12 Hz**（0.0834 s，对应 yaml 的 `task3.fire_interval_s`） |
| 发弹数 | **不限** |
| 伤害 | **20 HP / 发** ⇒ 打满 30 分要 **75 发命中** |
| 计分 | **1 分 / 50 HP**，上限 **30 分** |
| 角速度 | 前哨站转速 **2.5 rad/s**，容差 **±0.4 rad/s**，占 5 分 |
| 机会 | 正式考核前**最多 3 次发弹调试机会** |

### 9.5 硬约束（违反了直接丢分）

1. **不能加发弹数上限** —— `max_rounds_per_stage` 必须保持 `0`（用户明确要求）。
2. **task_1 不许碰** —— 已经现场验收通过，25 分在里面。
3. **转速由赛规给定**，不能靠“把靶子调慢”提命中率。
4. **3 次发弹机会**（task_3 考核前），每次都要有明确判据，不能“打打看”。
5. **每挡 10 发**，虚发（请求了但没弹出来）也算消耗额度。

### 9.6 我们的改动分别服务哪几分

| 改动 | 服务 | 分值 |
|---|---|---|
| offset 归零、颜色映射（A1 / A2） | task_2 命中 | 20 |
| 到位门控、一块一发、板面朝向（B2 / B4 / B5） | task_2 命中 | 20 |
| `has_detection` 补漏（B1） | 省弹（虚发也占 10 发额度） | 20 |
| `fire_interval_s`、`min_frames_for_fire`（B6 / B7） | task_2 命中节奏 | 20 |
| lead 量纲 / 符号修正（C1 / C2） | task_2 命中 | 20 |
| EKF 的 `w`（没动） | task_2 角速度 | 20 |
| task_3 的 12 Hz、z 过滤（没动） | task_3 血量 | 30 |

---

## 10. 参考数据

### 10.1 几何与换算（一切按这个算）

| 量 | 值 | 算法 |
|---|---|---|
| 小装甲板宽 | **135 mm** | `tasks/auto_aim/solver.cpp` |
| 灯条长 / 宽 | 56 mm / 14 mm | 同上 |
| 3.15 m 处板宽张角 | **2.45°** | `2·atan(0.0675/3.15)` |
| 3.0 m 处板宽张角 | **2.58°** | `2·atan(0.0675/3.0)` |
| 半宽（打中容差） | **1.29°** = 67.5 mm | 同上 |
| 板在相机里的**视在**角速度 | **14.5°/s** @4 rad/s、3.15 m | `r·ω/d`，r ≈ 0.2 m |
| ⇒ 1° 视在偏差 | **70 ms** | 1 ÷ 14.5°/s |
| ⇒ 1 个靶宽（2.45°） | **170 ms** | 同上 |
| 飞行时间 | **126 ms** | 3.15 m ÷ 25 m/s |
| 飞行中板转过的相位 | **29°** | 4 rad/s × 0.126 s |
| 折算成相机角 | **1.8–2.0°**（0.75 靶宽） | `2r·sin(φ/2)/d` |
| 整块板通过时间 | 15 / 9.0 / 6.4 ms | 3 / 5 / 7 rad/s |

### 10.2 实测延迟（`latency_report.py logs/lat_1510.csv`）

| 段 | 值 | 说明 |
|---|---|---|
| 取帧 → 解算完成 | **24.9 ms**（p50）、29.3 ms（p95） | 2686 帧 / 42.66 s / 63.0 fps |
| 解算 → 串口发送 | **0.06 ms** | 可忽略 |
| **云台反馈滞后** | **31.8 ms**（互相关系数 0.71） | 唯一可信的云台数 |
| ⇒ 合计 | **57 ms** | `DSH_LEAD=0.057` |
| pitch 通道 | 460.8 ms（**无意义**） | 信号太平，相关算不出来 |

**不可信的数**：`lead_advisor.py` 在 `step_1508.csv` 上报 D=0、τ=145.5、T90=335、建议 164 ms ——
那是 1 Hz 方波被当成一次阶跃拟合出来的，跨了边沿，**已明确否掉**。

### 10.3 帧率与录像代价

| 工况 | 帧率 | 备注 |
|---|---|---|
| 正常（不开录像） | **58–63 fps** | 启动没有 `[Recorder]` 行 |
| 开录像（JPEG q95） | **44.7 fps** | 250–400 KB/帧 ⇒ 15–24 MB/s |
| 一帧典型大小 | 115,944 B（合成）/ 200–250 KB（实景） | 一分钟 0.7–0.9 GB |

### 10.4 当前配置现值（`configs/standard.yaml`）

| 键 | 值 | 本次是否改过 |
|---|---|---|
| `enemy_color` | `"red"` | — |
| `min_confidence` | **0.88** | 改（0.8 → 0.88） |
| `roi` | `x 304 y 152 w 800 h 800`，`use_roi: true` | 改（原 600×600、关闭） |
| `yaw_offset` / `pitch_offset` | **0.0 / 0.0** | 改（2 / 6.5 → 0） |
| `target_hold_ms` | 300 | — |
| `nis_reject_threshold` | 16.27（卡方 4 自由度 99.7%） | — |
| `max_nis_failure_rate` | 0.3 | — |
| `min_frames_for_fire` | **10** | 改（15 → 10） |
| `muzzle_offset_x` | 0.171 m | — |
| task2 `fire_interval_s` | **0.2**（5 Hz） | 曾改 0.35，核验后撤回 |
| task2 `max_rounds_per_stage` | **0** | 不变（硬约束） |
| task2 `stage_gap_s` / `speed_band_split` | 5.0 / 5.0 | — |
| task2 `require_auto_aim_mode` | **true** | 改（false → true） |
| task3 `fire_interval_s` | 0.0834（12 Hz） | — |
| task3 `require_auto_aim_mode` | **false** | 不变 |
| `record_dir` | `""` | 改（关录像） |
| `record_jpeg_quality` | 95 | — |

### 10.5 现场读到过的真实数值（可当基准）

| 场景 | 读数 |
|---|---|
| 静止靶 3.1 m（offset 归零后） | `AIM yaw 0.46 pitch −0.00 dist 3.09 w −0.00 rounds 0/0`，落点“几乎正中靶心” |
| 旋转靶 3.15 m、4 rad/s、门控关 | `AIM yaw 0.29 pitch 0.03 dist 3.15 w 4.00 rounds 62/0`；`errY=0.64° errP=0.07° Ymin/max=0.00/4.16` |
| 同上另一场 | `errY=1.51° errP=1.34°`、`Ymin/max=0.65/6.31`、`n=48 blocked=0` |
| 电控挡位正常 | `[dbg] 电控挡位 = AUTO_AIM(自瞄)` |
| 电控挡位异常 | `[dbg] 电控挡位 = IDLE(手动/未接管)`、`mode_now=0`、`rounds 0/0`（那场不算数） |
| 弹速 | 四场 `frames.csv` 全是 `23.014441`（常数，不可信）；配置兜底 25 m/s |
| 录到的最快帧率 | `[Recorder] 已录制 44.7 帧每秒` |

### 10.6 数据文件清单

| 文件 | 大小 | 内容 |
|---|---|---|
| `logs/lat_1510.csv` | 281,027 B | 2686 帧延迟探针（`dev_ts,grab_ns,infer_ns,send_ns,fb_yaw,cmd_yaw,…`） |
| `logs/step_1508.csv` | 453,056 B | 4295 帧阶跃探针（`DSH_STEP_DEG=3`） |
| `logs/step_1500.csv` | 8,537 B | 一次失败的短阶跃（没目标，仅留档） |
| `logs/2026-10-17_*.log` | 21 个，共约 240 KB | 10-17 那次上车的终端日志 |
| 车机 `recordings/run_1020_1446` | 450 MB / 928 文件 / 19.51 s | **一发没打**（`bullet_count` 一直 11） |
| 车机 `recordings/run_1020_1456` | 395 MB / 814 / 17.58 s | 同上 |
| 车机 `recordings/run_1020_1457` | 296 MB / 610 / 13.38 s | 同上 |
| 车机 `recordings/run_1020_1458` | 394 MB / 813 / 17.65 s | 同上 |
| `recordings/synthetic_rotate` | — | 合成数据（验证工具链用） |
| `recordings/synthetic_static` | — | 同上 |

> 四场录像的 `frames.csv` 末行完全相同：`bullet_count=11, mode=0, bullet_speed=23.014441`，
> 而且 `grep '已录制' logs/*.log` 是空的 —— **这四场既没打出一发，也没留日志**，对定 lead 无用。

### 10.7 还没量过的量（写在这里免得以后再忘）

| 量 | 怎么量 | 影响 |
|---|---|---|
| 真实弹速 | 2 m 与 4 m 各打 3 发，看偏移是否随距离翻倍 | 俯仰 + 飞行时间补偿 |
| 零位残差 | 静止靶 3 发，`zero_fix.py` | 左右系统偏差 |
| 云台真正的位置阶跃响应 | 需要**单次**阶跃（不是 1 Hz 方波） | 决定门控能不能再收紧 |
| `w` 的符号约定 | `DSH_LEAD=0` 与 `0.57` 各 10 s，比 `AIM yaw` | lead 是加分还是添乱 |
| facing 的朝向约定 | `[dbg] fire-gate` 的 `assoc=` 是否在 0–40° | 会不会去瞄背对的那块板 |

---

## 11. 外部核验报告的采纳情况（2026-10-20）

> 对象：`C:\Users\MTG\Desktop\L4\fp_aim_1020改动核验报告.md`（235 行，自称源码逐行读 + 模型二审 + 复验）。
> 结论：**大方向正确（「有真 bug、不能直接上车」），逐条复核后 6 条接受、3 条记录、2 条不同意**。已改代码 2 处、yaml 1 处、文档 9 处。

### 11.1 接受并已改

| 报告条目 | 我的复核 | 改了什么 |
|---|---|---|
| A1 根因写错 | ✅ 成立，比报告说的更彻底 | §2 A1 改成「旧版两个 offset 是死配置，归零没修任何东西；23° 的真实原因待查」 |
| P4 静止靶只出 1 发 | ✅ 成立，是我引入的调用顺序 bug | `plate_armed` 改成每帧现算 + 目标转速 `abs(w) < 0.5` 时豁免（`src/task_2.cpp` 状态块） |
| A→B→A 不重新上膛 | ✅ 成立 | 同一处修复（换板即 `aim_id != last_fired_plate` → 上膛） |
| 0.35 s 间隔无必要 | ✅ 成立，且赛规写死 5 Hz | `configs/standard.yaml` 回 `0.2`，§2 B6 重写，文档 9 处同步 |
| `lead` 默认 0 = 症状没修 | ✅ 成立（我本来就是刻意的） | §6 加第 0 步自检：先 `DSH_LEAD=0.057` 跑 10 s 看 HUD `AIM yaw` 是否左移 |
| 中挡可能整圈等不到 | ✅ 成立 | §7 风险表与 §10.6 写明「中挡窗口 97 ms，可能零命中，要现场量」 |

### 11.2 记录但暂不改（代价大于收益）

| 报告条目 | 为什么不改 |
|---|---|
| 候选集没按「有检测框」过滤 | 要按「最近观测过」判定，得在 `Target` 里加时间戳字段，改动面比收益大；先靠 `assoc` 日志观察 |
| 画面不画 `aim_id` | 要改 `overlay` 入参（现在画的是 `chosen`），属于下一步；先用 `[dbg] fire-gate` 的 `assoc=` |
| 报告只覆盖 2 个文件 | 报告看的是补丁；§5 的表和 §8.3 已列全 15 个文件，按那张表传车 |

### 11.3 不同意

| 报告条目 | 理由 |
|---|---|
| 门控 0.8° 太紧 → 建议 1.5° | 报告自己也写「可用但不严格」。0.8 = 1.6× 稳态滞后 0.45°；1.5 已接近板半宽 1.29° 的一半。§6 第 3 步保留 1.2 只作应急 |
| 重新上膛角 35° 与窗口 25° 冗余 → 建议 20° | 20° < 25° 会让重新上膛发生在窗口**之内**，防抖直接失效。35 > 25 是故意的 |

### 11.4 报告没提、但更该担心的

1. **`DSH_PITCH_OFFSET` 从死配置变成了真推角度**：旧版 `pitch_offset` 只进环境变量日志、不进 `aim_pitch`，所以填 10 也看不出来；新版 `aim_pitch += pitch_offset_eff` 是真的偏 10°。现场若照旧版习惯填非零值，俯仰会当场偏掉。
2. **代码仍未编译**（本机没有 Linux 工具链），所有改动只做过结构检查（花括号/圆括号/占位符计数）→ 上车第一件事必须是 `make -j2 task_2`，判据是 `Building CXX object .../task_2.cpp.o` + `Linking CXX executable task_2`。
3. **这一轮两处修复都改的是「开火条件」**，而开火条件出错的表现是「一枪不发」，与「打不准」完全不同 —— 现场若出现零发弹，先查 §9 的挡位/门控，别怀疑视觉。
