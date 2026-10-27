# fp_aim_fkey — 项目指南

## 这是什么

机器人视觉「云台 + 相机 + 自瞄」离线工具线。**开发机是 Windows 笔记本（只负责编辑与分析），
编译与运行在 Linux 车机**（x86_64 或 aarch64），依赖 OpenVINO 2024.6 与海康 MvCameraControl SDK。

## 输出约定

- 一律用中文回答，技术名词保留英文原词。
- 每条结论必须给证据：`文件路径:行号` 或 `commit sha`。给不出证据的，标 `【推断】`。
- 禁止编造：不写"已测试通过""性能提升 30%"这类没有仓库证据的话。
- 代码只贴关键行（≤8 行），不要贴整段函数。
- 参数变化一律用表格：`参数 | 旧值 | 新值 | 位置 | 影响`。
- 项目背景：RM 上位机视觉，三个任务 task_1 基础自瞄 / task_2 小陀螺 / task_3 前哨站；
  共用 `tasks/auto_aim/target.cpp`（PnP + 状态估计）与 `src/task_common.hpp`（弹道解算）；
  参数分两层：`configs/*.yaml` 与 `DSH_*` 环境变量覆盖。

## 目录约定

| 目录 | 内容 |
| --- | --- |
| `src/` | 主程序 `task_1/2/3`；`fire_key.hpp` 按键门、`low_plate.hpp` 小装甲板、`latency_probe.hpp` 延迟探针、`overlay.hpp` 叠加层、`env_util.hpp` 环境变量读取、`detect_log.hpp` |
| `tools/` | 工具库（OBJECT lib）：EKF、轨迹、绘图、录包器 `recorder/replay`、logger、CRC；另有若干 `*.py` 分析脚本 |
| `io/` | 硬件层（STATIC lib）：`camera`、`gimbal`、`hikrobot`（海康 SDK）、`video`（回放假相机）、`serial`（**第三方串口库，勿改**） |
| `tasks/auto_aim/` | 自瞄算法库：`armor`（装甲板）、`solver`（解算）、`target`（目标）、`yolo`/`yolos`（检测） |
| `configs/` | 运行配置；**`standard.yaml` 是线上实际在用的那份** |
| `docs/` | 现场执行卡与排障 SOP（见下） |
| `assets/` | `yolov5.bin` / `yolov5.xml` 模型 |

## 源码树（路径 + 行号按 HEAD 实测，2026-10-26）

入口与任务主循环
- `src/task_1.cpp` 3 KB — 任务一：基础自瞄（`main` :21）
- `src/task_2.cpp` 70 KB — 任务二：小陀螺（最大文件，多层滤波与预测）
- `src/task_3.cpp` 24 KB — 任务三：前哨站（`main` :34）

共用模块（工程核心，讲解重点）
- `src/task_common.hpp` 5 KB — 弹道解算 + 三任务共用
- `src/low_plate.hpp` 8 KB — 低板判定 / 瞄准点选择
- `src/overlay.hpp` 6 KB — HUD 绘制（AIM/HOLD 行、右下角状态条）
- `src/fire_key.hpp` 14 KB — 发弹控制（频率、热量、模式）
- `src/env_util.hpp` 1 KB — `DSH_*` 环境变量读取
- `src/detect_log.hpp` 4 KB — 检测日志
- `src/latency_probe.hpp` 6 KB — 延迟打点
- `src/test_summary.hpp` 10 KB — 自测汇总

视觉与状态估计
- `tasks/auto_aim/armor.cpp` 10 KB — Armor 数据体（`to_color` :25）
- `tasks/auto_aim/solver.cpp` 11 KB — PnP 解算：`set_R_gimbal2world` :64、`solve` :71、
  `reproject_armor` :110、`optimize_yaw` :159、`SJTU_cost` :183、
  `armor_reprojection_error` :217、`world2pixel` :229/:263
- `tasks/auto_aim/target.cpp` 19 KB — 状态估计（EKF）：`predict` :128/:135、`update` :200、
  `update_ypda` :252、`update_checked` :306、`armor_z_max` :343、`armor_xyza_list` :357、
  `diverged` :369、`convergened` :397、`h_armor_xyz` :406、`h_jacobian` :431
- `tasks/auto_aim/yolos/yolov5.cpp` 9 KB — YOLOv5 OpenVINO 推理

弹道与工具
- `tools/trajectory.cpp` 5 KB — 弹道迭代解算
- `tools/extended_kalman_filter.{cpp,hpp}` — EKF 实现
- `tools/math_tools.{cpp,hpp}` — 数学工具（角度归一化等）
- `tools/logger.cpp` / `recorder.cpp` / `replay.cpp` / `plotter.cpp` — 日志/录制/回放

配置（参数都在这里，不在代码里）
- `configs/sentry.yaml` 4 KB — 哨兵车现场配置
- `configs/standard.yaml` 6 KB — 步兵车标准配置
- `configs/submit.yaml` 6 KB — 提交版配置

IO 层（课程提供，基本不改，讲时一句带过）
- `io/camera.cpp`、`io/gimbal/gimbal.cpp`、`io/hikrobot/`、`io/serial/`（第三方，勿改）、
  `io/video/`

离线分析（Python，不进板端）
- `tools/*.py`（analyze_frames / shot_fit / lead_solve / plot_frames 等 11 个）
- `tools/onsite/*.sh`（现场采集、打包、跑测脚本）

自查命令：
```bash
# 环境变量清单
grep -rn 'getenv("DSH_' src tasks tools
# 参数默认值：configs/*.yaml 与代码兜底值两边都要看
```

> 注：行号会随函数体改动漂移。改过 target.cpp / solver.cpp 后，以上数字须 `grep -n '<函数签名>' <文件>`
> 重新确认，不要按记忆引用。

## 构建（只能在 Linux 车机上）

```bash
cd ~/fp_aim && mkdir -p build && cd build && cmake .. && make -j2 task_2
# 只改了一个 .cpp：cd ~/fp_aim && touch src/task_2.cpp && cd build && make -j2 task_2
```

依赖：`OpenCV` `fmt` `Eigen3` `spdlog` `yaml-cpp` `nlohmann_json` `OpenVINO`
（`/opt/intel/openvino_2024.6.0/runtime/cmake/`）`MvCameraControl` `usb-1.0`。

**重编成功判据（缺一不可）**：同时出现
`Building CXX object CMakeFiles/task_2.dir/src/task_2.cpp.o` 和 `Linking CXX executable task_2`。
只看到 `[100%] Built target` = 没重编，旧二进制不认识新默认值。

可执行目标：`task_1` / `task_2` / `task_3` / `keytest`
（`keytest` 只依赖 `src/fire_key.hpp`，零工程依赖，用来上车前先确认 F 键通道通不通）。

## 关键约定

- **参数来源：yaml 优先，`DSH_*` 环境变量覆盖。**
  到位/选板：`DSH_ARRIVE_DEG` `DSH_ARRIVE_PITCH_DEG` `DSH_ARRIVE_FRAMES` `DSH_ARRIVE_MM`
  `DSH_ARRIVE_WATCHDOG_SHOTS` `DSH_PLATE_FACING_DEG` `DSH_PLATE_REARM_DEG` `DSH_STEP_DEG`
  `DSH_AIM_SELECT` `DSH_STATIC_RATE_DPS` `DSH_STATIC_FRAMES`
  提前量/低位靶：`DSH_LEAD` `DSH_FIRE_DELAY` `DSH_LOW_PLATE` `DSH_LOW_PLATE_Z_TOL` `DSH_LOW_PLATE_TRUST_S`
  人工门：`DSH_FIRE_KEY` `DSH_FIRE_KEY_DEV` `DSH_FIRE_KEY_ENABLE`
  标定/记录：`DSH_YAW_OFFSET` `DSH_PITCH_OFFSET` `DSH_RECORD_DIR` `DSH_REPLAY_DIR` `DSH_DET_CSV`
  `DSH_LAT_CSV` `DSH_TEST_TAG` `DSH_SUMMARY_FILE`
  低层（一般不动）：`DSH_FIRE_MODE`（电控发弹模式，缺省 2）、`DSH_COLOR_ORDER`（0 红 1 蓝）、`DSH_DEVICE`（推理设备）。
  改动行为前先查 `src/env_util.hpp` 与 `src/task_2.cpp` 里的 `getenv`。
- **所有 `DSH_*` 都在进程启动时读一次 ⇒ 改了必须重启进程**（`DSH_LEAD` / `DSH_FIRE_DELAY` /
  `DSH_STATIC_*` 是 `static`，最容易漏）。
- **现场流程：Windows 编辑 → `scp` 到车机 → `make` → `grep` 验收**，完整步骤见
  `docs/field_card.md` 第 0 步。改完必须跑那几条 `grep` 判据再上车。
- **别开录像**：`DSH_RECORD_DIR` 与 yaml 的 `record_dir` 都要空，否则 58 → 45 fps，时间基准全乱。
- **按键门**（`src/fire_key.hpp`）：「人按住 F 才允许开火」。取键优先级
  Linux evdev > Windows `GetAsyncKeyState` > OpenCV 窗口兜底（后两者只能拿到「按下那一帧」）。
  `DSH_FIRE_KEY=hold|toggle|always|never`。
- **task_2 的开火安全门**（改了必须同步 yaml 与文档）：到位门控 0.8°/1.3° 且连续 3 帧、
  板面正对 ≤25°、一块板只打一发（摆出 35° 才重新上膛）、发弹间隔 0.2 s。
- **代码风格**：`.clang-format`（Google 基线，100 列，`PointerAlignment: Middle`，
  函数/类/结构体后换行大括号）。注释与文档用中文。
- **第三方代码不要改**：`io/serial`、`io/hikrobot/include`、系统头。

## Token 消耗纪律（计费红线，2026-10-27 追加）

> 教训：本项目会话曾单请求携带 43 万 token 历史缓存、4 分钟 13 次调用 ≈ 560 万 token。
> AGENTS.md 每次请求都会随上下文发送——本节必须保持短小，禁止继续加长。

- **工具输出先裁剪再进对话**：读文件用 `grep -n`/`Select-String` 取关键行；代码 ≤8 行；禁止整文件倾倒。
- **同一验证只跑一次**：结果落盘 docs/ 后引用「结论+行号」；文件 blob sha 没变就禁止重跑同款检查。
- **长内容落盘**：完整 diff、逐行扫描、全文校验 → 写进文件，回复只给结论 + `file:///` 链接。
- **合并命令**：一条 bash 能做完的不拆多条；每条 bash 都是一次全量上下文计费。
- **图片/视频**：抽帧 ≤1 fps、只裁关注区（如 HUD 条），禁止整帧原样进对话。
- **体积自检**：单请求缓存命中 >10 万 token 时主动提示用户开新会话，并把状态摘要落盘交接。
- **先清单后动手**：多步任务先列 todo，避免边做边试的重复调用。

## 文档交付约定

- **交付任何文档（`docs/*.md` 等）时，回复里必须附完整可点击链接**，不能只给 Windows 路径：
  - 文件：`file:///D:/MTG/Documents/OC/fp_aim_fkey/docs/<文件名>.md`（中文/空格做百分号编码）
  - 目录：`file:///D:/MTG/Documents/OC/fp_aim_fkey/docs/`
  - 生成方法（PowerShell）：`([uri]("file:///" + $p.Replace('\','/'))).AbsoluteUri`
  - 示例：`file:///D:/MTG/Documents/OC/fp_aim_fkey/docs/NIS%E9%97%A8%E9%99%90%E8%AE%B2%E8%A7%A3%E4%B8%8E%E6%AD%BB%E9%94%81%E5%88%86%E6%9E%90_1026.md`
- Windows 路径可以附在链接之后作辅助说明，但链接必须在前。

## 文档索引

- `docs/field_card.md` — 现场执行卡 V4（30 分钟从零到打中旋转靶）：传/编/验收判据与每发操作
- `docs/troubleshooting_sop.md` — 排障 SOP（最长最全，改动前先搜这里）
- `docs/fix_log.md` — 修复记录；`docs/debug_flow.md`(+v2/v3) — 调试流程
- `docs/param_rollback.md` — 参数回滚；`docs/onsite_record.md` / `record_session.md` — 现场记录
- `docs/audit_review.md` — 代码审计结论

## 本机（Windows）能做什么

- **只能编辑与分析**。编译、运行、抓相机、按键门实机验证都必须在 Linux 车机上。
- 不要在 Windows 上试 `cmake` 构建：`OpenVINO_DIR` 指向 Linux 路径，`MvCameraControl` 是 `.so`。
- 需要实机验证时走 `docs/field_card.md` 的 `scp → make → grep` 流程。

## LSP 备注

- 已启用 clangd（`.cpp/.hpp`）、yaml-ls（`configs/*.yaml`）、pyright（`tools/*.py`），
  并打开了 formatter（clang-format，按本仓 `.clang-format`）。
- clangd 要出**完整**诊断需要 `compile_commands.json`：在车机 build 目录
  `cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..`，把生成的 `compile_commands.json` 拷回项目根目录。
- **本机（Windows）clangd 报的这两类是环境噪音，不是代码缺陷，别照着改**：
  1. `'opencv2/opencv.hpp' file not found` 之类 —— Windows 上没装 OpenCV/Eigen/OpenVINO；
  2. `'getenv' is deprecated` —— MSVC 的 `_CRT_SECURE_NO_WARNINGS` 提示，Linux 车机上不存在。
  真正的问题要在车机 `make` 的结果里看。
- 本机 pyright 对 `tools/*.py` 的 `tuple[Literal[...]]` 字面量收窄告警是已知噪音，
  不要为消警告去改脚本逻辑。
