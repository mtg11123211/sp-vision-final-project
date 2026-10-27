# 两个零点补偿参数的「启用 / 回滚」手册

> 适用：`configs/standard.yaml` 里的 `yaw_offset` 与 `pitch_offset`。
> 场景：你不想让它们生效了，或者想回到「这两个参数还没接进代码」的那版行为。
> 一句话：**三级回滚，最快的一级不用改文件、不用重编。**

---

## 0. 现状（2026-10-18）

| 参数 | 现在的值 | 之前的值 | 依据 |
|---|---|---|---|
| `yaw_offset` | **0.72** | 0.48 | 2.4 m 静止靶弹着点偏右 10 mm = 4.17 mrad = 0.239° |
| `pitch_offset` | **10.24** | 10 | 同场偏下 10 mm，同理往右…（往上）抬 0.239° |

- 谁在读：`src/task_2.cpp:71-72`（`tools::read_or`），**task_3 完全不读这两个量**（`src/task_3.cpp` 里没有 `yaw_offset`/`pitch_offset`）。
- 怎么起作用：`src/task_2.cpp` 的 `lead_s` / `aim_yaw +=`
  ```cpp
  aim_yaw   += x[7] * lead_s + yaw_offset_eff * 0.017453292519943295;
  aim_pitch +=                pitch_offset_eff * 0.017453292519943295;
  ```
- 方向约定（从代码推的，不是猜的）：`tools/math_tools.cpp:103` `yaw = atan2(y, x)`、本工程 x 前 / y 左 / z 上 ⇒ **正的 yaw 往左**；`pitch = atan2(z, 水平)` ⇒ **正的 pitch 往上**。
- 换算工具：`py tools/zero_fix.py --d 2.4 --dx 10 --dy -10`

### 「未启用」到底指什么

初始那份代码里，这两个 yaml 项**没有任何代码去读**，等于常数 0。

所以「未启用」有两个等价含义，别混：

| 你想要 | 该用的值 |
|---|---|
| 参数存在但**没有补偿作用** | `yaw_offset: 0`、`pitch_offset: 0` |
| 回到**初始代码**（读都不读） | 同上，效果完全一样；代码层面回滚只是形式 |

**关键结论：把两个值写成 0，就已经是「未启用」，不需要碰代码。** 剩下的问题只是「怎么在 10 秒内切过去」。

---

## 1. 三级回滚

### L0 · 环境变量（最快，不改文件；需要本版代码编过）

这次改动加了覆盖通道：`DSH_YAW_OFFSET` / `DSH_PITCH_OFFSET`（**单位：度**）一旦设置，就覆盖 yaml 的值。进程内只读一次，所以改的只是启动命令。

```bash
# 完全未启用（等价于这两个参数还没接进代码）
DSH_YAW_OFFSET=0 DSH_PITCH_OFFSET=0 ./build/task_2 configs/standard.yaml

# 只关掉横向、保留纵向
DSH_YAW_OFFSET=0 ./build/task_2 configs/standard.yaml

# 临时试别的值（不动文件，试错成本为零）
DSH_YAW_OFFSET=0.48 DSH_PITCH_OFFSET=10 ./build/task_2 configs/standard.yaml
```

**确认生效**：进程起来第一屏日志，`源:` 后面会标出谁在生效：

```
[task2] offset yaw=0.00(env DSH_YAW_OFFSET) pitch=0.00(env DSH_PITCH_OFFSET)  源: yaml 值 0.72/10.24
[task2] offset yaw=0.72(yaml) pitch=10.24(yaml)  源: yaml 值 0.72/10.24
```

注意事项：
- 只影响 `task_2`；`task_3` 本来就不读这两个量，回不回滚它都一样。
- 这一级要求**当前这版代码已经编译过**。如果车上还是旧二进制（没有 `DSH_YAW_OFFSET` 那段），环境变量会被忽略、静默失效——**所以一定要看那行日志**，别以为设了就生效了。
- 和 `DSH_LEAD` 一样是 `static`，改了要重启进程，热改无效。

### L1 · 改 yaml（不用编译，改文件）

```bash
cd ~/fp_aim
cp configs/standard.yaml configs/standard.yaml.bak.$(date +%m%d_%H%M)   # 先备份

# 方式 A：未启用
sed -i 's/^yaw_offset:.*/yaw_offset: 0/; s/^pitch_offset:.*/pitch_offset: 0/' configs/standard.yaml

# 方式 B：回到上一版调参值
sed -i 's/^yaw_offset:.*/yaw_offset: 0.48/; s/^pitch_offset:.*/pitch_offset: 10/' configs/standard.yaml

grep -n 'yaw_offset\|pitch_offset' configs/standard.yaml    # 核对
```

改完直接重启程序即可（yaml 在 `main` 里启动时读一次）。

### L2 · 代码层面回滚（只在真要退回初始版本时用）

**不要**直接 `git checkout -- configs/standard.yaml`：那个文件里还压着 ROI、`min_confidence`、`fire_interval_s` 等一堆现场调参，一起回滚会把这些也带走。

```bash
cd ~/fp_aim

# 只想撤掉「两个 offset 的读取与叠加」这一段：
git diff -U3 -- src/task_2.cpp | less      # 先看清楚这段在哪
# 手工把 299-300 行改回不叠加，或把 73-80 行的 env 覆盖段删掉

# 真要整份回滚代码（会丢掉全部未提交改动，先打包！见 SOP §5.6）：
bash tools/onsite/collect_run.sh rollback_backup    # 先把现场改动存成包
git stash push -m "回滚前现场版"                     # 可找回
git checkout -- .
```

**回滚前必须打包**——现在工作树上有 10 个文件未提交改动，`git checkout` 一下就没了。

---

## 2. 选哪一级：对照表

| 你想做的事 | 用哪级 | 要编译吗 | 要改文件吗 |
|---|---|---|---|
| 这一场临时关掉补偿做 A/B 对比 | **L0** | 否（前提是已编过） | 否 |
| 长期关掉 / 改回旧值，别人接手也一致 | L1 | 否 | 是 |
| 证明「不是 offset 导致的」 | L0，`DSH_YAW_OFFSET=0 DSH_PITCH_OFFSET=0` | 否 | 否 |
| 要交回干净代码 | L2 | 要 | 是 |

推荐做法：**A/B 一律用 L0**，把「文件里写什么」留到想清楚了再定。这样文件始终只有一份真相，不会出现「车上 21、笔记本 25」那种两份状态打架的事。

---

## 3. 回滚之前先看一眼的两件事

1. **pitch 通道的符号没被独立验证过。** `c37d941` 那笔「电控 pitch 正方向相反，发指令前取反」是拿一次现象定的性，而当时云台可能正处在没到位的状态。如果改完 `10.24` 之后下一发偏得**更**低（约 2 cm），说明方向反了，该写 `9.76`，不是继续加。
2. **0.72 是否真的对应「偏右 10 mm」**，取决于那 10 mm 是在 `yaw_offset=0.48 / pitch_offset=10` 生效时量的。如果量的那次用的是别的配置，用 `zero_fix.py --yaw-offset <当时的值> --pitch-offset <当时的值>` 重算。

---

## 4. 变更记录

| 日期 | 动作 | yaw_offset | pitch_offset | 备注 |
|---|---|---|---|---|
| 2026-10-17 之前 | 现场调参 | 0.48 | 10 | 交接文档记为「现场值」 |
| 2026-10-18 | 按 2.4 m 实测修正 | **0.72** | **10.24** | 偏差 10 mm / 2.4 m = 0.239° |
| 2026-10-18 | 增加 L0 环境变量覆盖通道 | — | — | `src/task_2.cpp:73-80` |
