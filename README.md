# sp-vision-final-project

RM 上位机视觉大作业 fp_aim：task_1 基础自瞄、task_2 小陀螺、task_3 前哨站。

## 分支与标签

| 引用 | 说明 |
|---|---|
| `main` | 开发线，任务代码 + 录包回放工具 |
| `submit/task_1` | task_1 提交线，tag `task1-1017`、`task1-1018` |
| `offline_replay` | 录包回放工具线 |
| `v0.1` | 任务一二三完成 |
| `v0.2` | YOLO CPU 回退 |
| `v0.3` | 云台 pitch 取反 |

## 提交日志

| 日期 | 说明 |
|---|---|
| 10-13 | baseline：课程仓库 final_project_aim 分支 + 本人已完成的 task_2 / task_3 / target.cpp |
| 10-13 | chore：.gitignore 忽略自用文档 |
| 10-13 | fix(ekf)：卡方阈值按自由度取值 9.488 / 19.68 |
| 10-13 | fix(armor)：priority 补默认初值 |
| 10-13 | fix(gimbal)：send 带目标编号，q(t) 避免除零 |
| 10-13 | feat(target)：过程噪声分组、前哨站几何冻结、NIS 拒收 |
| 10-13 | config：补任务二、任务三参数 |
| 10-13 | feat(task2)：弹道解算、丢帧保持、发弹节流 |
| 10-13 | feat(task3)：前哨站三块中部板建模与顶部板过滤 |
| 10-13 | feat(replay)：录包器、回放数据集、假相机 |
| 10-13 | feat(io)：相机与云台接录包钩子与回放模式 |
| 10-13 | build：录包回放源文件接入 CMake |
| 10-14 | fix(yolo)：OpenVINO 设备不可用时回退 CPU |
| 10-16 | fix(gimbal)：发送前对 pitch 取反 |
| 10-18 | docs：README 改为大作业说明 |
| 10-18 | chore：.gitignore 只留构建与数据项 |
| 10-18 | docs：README 改为条目式并补提交表 |
| 10-18 | fix(armor)：颜色映射按实车模型改为 0 红 1 蓝 |

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
