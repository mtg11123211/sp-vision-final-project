# task_1 提交说明

- 基线：`c37d941`，分支 `submit/task_1`
- 范围：`src/task_1.cpp`，共用 `io/`、`tasks/auto_aim/`、`tools/`
- `configs/standard.yaml` 为仓库默认值
- tag：`task1-1017` 是 10-17 的快照，`task1-1018` 含颜色映射修正（第 10 节）

## 1. 功能

```
相机取帧 1408x1104 → YOLOv5(OpenVINO) 检测装甲板 → PnP + 坐标变换得世界坐标
→ yaw / pitch / 距离 → 串口 921600、30 字节协议发云台
→ 窗口左上角打印角度，Plotter 输出曲线，q 键退出
```

- 只对准装甲板中心，无提前量（提前量在 task_2）
- 丢目标后 300 ms 内保持上一次角度（`target_hold_ms`），超时停止发控制
- 退出前发一帧不控制，云台交还电控

## 2. 编译

依赖：OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann_json、OpenVINO 2024.6、海康 MVS SDK（`io/hikrobot/lib/amd64/libMvCameraControl.so` 随仓库带）。

```bash
mkdir -p build && cd build
cmake .. && make -j2 task_1
```

产物 `build/task_1`。`CMakeLists.txt:15` 的 `OpenVINO_DIR` 为写死路径，本机不同则改该行或 `cmake -DOpenVINO_DIR=<路径> ..`。

## 3. 运行

```bash
./build/task_1 configs/standard.yaml
```

`configs/standard.yaml:7` 模型路径为相对路径，须在工程根启动。

```bash
# 录包
DSH_RECORD_DIR=~/recordings/task1_$(date +%m%d_%H%M) ./build/task_1 configs/standard.yaml
# 离线回放（无硬件）
DSH_REPLAY_DIR=<录包目录> ./build/task_1 configs/standard.yaml
```

录包目录格式：`frames.csv` + `%06d.jpg`（1408x1104）。仓库不含录制数据。

## 4. 硬件

| 项 | 值 |
|---|---|
| 相机 | HikRobot USB，`vid_pid: 2bdf:0001`（yaml 第 25 行） |
| 云台串口 | `/dev/gimbal`，921600，需 udev 规则；无云台时打印 `Failed to open serial`，检测不受影响 |
| 分辨率 | 1408x1104，对应 `camera_matrix` 主点 704.312 / 551.847 |

## 5. 代码导览

| 位置 | 内容 |
|---|---|
| `src/task_1.cpp:32` | `tools::Plotter plotter;` |
| `src/task_1.cpp:62` | `q = gimbal.q(t);` 取云台姿态 |
| `src/task_1.cpp:81-82` | `last_yaw/last_pitch = armor.ypd_in_world[0]/[1]` |
| `src/task_1.cpp:87` | `gimbal.send(true, false, last_yaw, last_pitch);` 控制发出入口 |
| `src/task_1.cpp:90-95` | 左上角黄字 yaw / pitch / dist |
| `src/task_1.cpp:98-103` | Plotter 组 `aim` |
| `src/task_1.cpp:106` | `cv::imshow("task1", img)`；`:115` 退出前 `send(false,false,0,0)` |
| `io/camera.cpp` | 相机/回放分支，读 `DSH_RECORD_DIR` / `DSH_REPLAY_DIR` |
| `tasks/auto_aim/yolos/yolov5.cpp` | 前处理、推理、后处理；`:247` 帧计数，`:261` `detection` 窗口 |
| `tasks/auto_aim/solver.cpp:84/88/99` | 相机系 → 云台系 → 世界系 → `ypd` |
| `tools/math_tools.cpp:103-110` | `xyz2ypd`：`yaw=atan2(y,x)`，`pitch=atan2(z, sqrt(x*x+y*y))` |
| `io/gimbal/gimbal.hpp:40-54` | 下行 30 字节 `VisionToGimbal` 结构 |
| `io/gimbal/gimbal.cpp:100-108` | 写 mode 与 yaw/pitch 六个量 |
| `io/gimbal/gimbal.cpp:236-252` | 收包、包头 `SP` 定位、CRC16 校验 |

坐标系：世界系 z 朝上，pitch 为正表示目标在水平线上方。

## 6. 实测结果

- 录包帧率 37 fps 爬升至 49 fps，稳定约 48 fps，无严重掉帧
- 检测框稳定，云台跟随装甲板；窗口左上角白字 `[N]` 为帧计数

## 7. 参数（`configs/standard.yaml`）

| 行 | 键 | 值 | 作用 |
|---|---|---|---|
| 1 | `enemy_color` | `"red"` | 目标颜色。task_1 不读；task_2/task_3 用它与 `armor.color` 比较（第 10 节） |
| 7 | `yolov5_model_path` | `assets/yolov5.xml` | 相对路径 |
| 8 | `device` | `GPU` | 无可用 GPU 时 `export DSH_DEVICE=CPU` |
| 9 | `min_confidence` | `0.8` | 检测阈值 |
| 11 | `nms_threshold` | — | NMS |
| 25 | `vid_pid` | `2bdf:0001` | 相机 VID:PID |
| 33 | `camera_matrix` | fx 1809.234 / fy 1813.010 / cx 704.312 / cy 551.847 | 绑定 1408x1104 |
| 34 | `distort_coeffs` | `[-0.0772, 0.1504, 0.00157, 0.00104, 0]` | 畸变系数 |
| 43 | `com_port` | `"/dev/gimbal"` | 云台串口 |
| — | `baudrate` | 代码默认 921600 | 波特率，yaml 里没写，读不到就用默认（`io/gimbal/gimbal.cpp:39`） |
| 51 | `target_hold_ms` | `300` | 丢目标保持时间 |

## 8. 验收判据

1. 日志出现 `[Gimbal] Serial opened at 921600 baudrate.` 与 `[Gimbal] First q received.`
2. `detection` 窗口有实时画面与绿框
3. `task1` 窗口黄字随装甲板变化：yaw 连续、pitch 随高度变化、dist 随距离变化
4. 开启 `DSH_RECORD_DIR` 时终端每 5 秒打印一次录制帧率

## 9. 提交日志

| 提交 | 日期 | 说明 |
|---|---|---|
| `d15ccdd` | 10-13 | baseline：官方分支 + 已完成的 task_2/task_3/target.cpp |
| `1a69fcf` | 10-13 | chore：更新 .gitignore |
| `2f7f8d9` | 10-13 | fix(ekf)：卡方阈值 1.5 改为 9.488 / 19.68 |
| `e2ddb97` | 10-13 | fix(armor)：priority 补默认初值 |
| `d8a8445` | 10-13 | fix(gimbal)：send 增加目标编号；q(t) 防除零 |
| `104ef66` | 10-13 | feat(target)：过程噪声分组、几何冻结、发散判据、NIS 拒收 |
| `a4b6dfc` | 10-13 | config：任务二/三参数 |
| `acfd1f9` | 10-13 | feat(task2)：弹道解算、丢帧保持、5Hz 节流、10 发上限 |
| `31536dc` | 10-13 | feat(task3)：前哨站建模与顶部板过滤 |
| `32da667` | 10-13 | feat(replay)：录包器、回放数据集、假相机 |
| `c6d9a0b` | 10-13 | feat(io)：录包钩子与回放模式 |
| `30bfe6a` | 10-13 | build+config：CMake 与配置开关 |
| `f1b9aa5` | 10-14 | fix(yolo)：GPU 不可用退回 CPU |
| `c37d941` | 10-17 | fix(gimbal)：pitch 取反（本分支基线） |
| `2b867c0` | 10-18 | fix(armor)：颜色映射改为 0 红 1 蓝（第 10 节） |

## 10. 已知问题

颜色类别映射（已修正，`2b867c0`）。`tasks/auto_aim/armor.cpp` 原先把模型第 0 类映射成蓝、第 1 类映射成红，而实车模型是 0 红 1 蓝。受影响的是 task_2 / task_3 的颜色过滤：`enemy_color: "red"` 永远匹配不上，`src/task_2.cpp:173` 会打印「视野里没有 red 色的装甲板，退回不筛颜色」，等于不过滤，可能锁到友方装甲板。task_1 不读 `enemy_color`，它自身的行为不受这条影响。

现在统一走 `to_color()`：0 红、1 蓝、3 紫，其余灭灯；换模型时用 `DSH_COLOR_ORDER=blue` 退回旧映射。
