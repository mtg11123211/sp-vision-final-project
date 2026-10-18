# sp-vision-final-project

RM 上位机视觉大作业 fp_aim：task_1 基础自瞄、task_2 小陀螺、task_3 前哨站。

## 分支与标签

| 引用 | 说明 |
|---|---|
| `main` | 开发线，任务代码 + 录包回放工具，HEAD `c37d941` |
| `submit/task_1` | task_1 提交线，tag `task1-1017` |
| `offline_replay` | 录包回放工具线 |
| `v0.1` | `31536dc` 任务一二三完成 |
| `v0.2` | `f1b9aa5` YOLO CPU 回退 |
| `v0.3` | `c37d941` 云台 pitch 取反 |

## 提交日志

| 提交 | 日期 | 说明 |
|---|---|---|
| `d15ccdd` | 10-13 | baseline：官方 final_project_aim 分支 + task_2/task_3/target.cpp |
| `1a69fcf` | 10-13 | chore：更新 .gitignore |
| `2f7f8d9` | 10-13 | fix(ekf)：卡方阈值 1.5 改为 9.488（4 自由度）/ 19.68（11 自由度） |
| `e2ddb97` | 10-13 | fix(armor)：priority 补默认初值 |
| `d8a8445` | 10-13 | fix(gimbal)：send 增加目标编号；q(t) 防时间戳重合除零 |
| `104ef66` | 10-13 | feat(target)：过程噪声分组、前哨站几何冻结、发散判据补符号、NIS 拒收 |
| `a4b6dfc` | 10-13 | config：任务二/三公共参数与各自参数 |
| `acfd1f9` | 10-13 | feat(task2)：弹道解算、丢帧保持、5Hz 节流、10 发上限 |
| `31536dc` | 10-13 | feat(task3)：前哨站三块中部板建模、顶部板两级高度过滤 |
| `32da667` | 10-13 | feat(replay)：录包器、回放数据集、假相机 |
| `c6d9a0b` | 10-13 | feat(io)：相机与云台接入录包钩子和回放模式 |
| `30bfe6a` | 10-13 | build+config：录包回放接入 CMake，补配置开关 |
| `f1b9aa5` | 10-14 | fix(yolo)：GPU 不可用退回 CPU |
| `c37d941` | 10-17 | fix(gimbal)：电控 pitch 正方向相反，发指令前取反 |

## 目录

```
src/              三个任务主循环
tasks/auto_aim/   装甲板检测、目标建模与 EKF、弹道解算
io/               海康相机驱动、云台串口、视频回放
tools/            EKF、弹道、CRC、Plotter、日志、录包回放
configs/          standard.yaml、sentry.yaml
assets/           YOLOv5 OpenVINO IR 模型
```

## 构建

依赖：OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann_json、OpenVINO 2024.6、海康 MVS SDK（库文件在 `io/hikrobot/lib/<arch>/`）。

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2 task_1 task_2 task_3
```

## 运行

配置内模型路径为相对路径，必须在工程根启动：

```bash
./build/task_1 configs/standard.yaml
```

task_1 提交说明见 `submit/task_1` 分支 README。
