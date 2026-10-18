# sp-vision-final-project

RM 上位机视觉大作业 `fp_aim`：task_1 基础自瞄、task_2 打小陀螺靶机、task_3 打前哨站。

## 分支与标签

| 引用 | 说明 |
|---|---|
| `main` | 开发线：任务代码 + 离线录包回放工具 |
| `submit/task_1` | task_1 提交线（tag `task1-1017`，基线 `c37d941`），提交说明见该分支的 `README.md` |
| `offline_replay` | 录包回放工具的历史线 |
| `v0.1` / `v0.2` / `v0.3` | 任务一二三完成、YOLO CPU 回退、云台 pitch 取反 |

## 目录

```
src/              三个任务的主循环（task_1 / task_2 / task_3）
tasks/auto_aim/   装甲板检测、目标建模与 EKF、弹道解算
io/               海康相机驱动、云台串口协议、视频回放
tools/            EKF、弹道、CRC、PlotJuggler 曲线输出、日志、录包回放
configs/          standard.yaml（三任务主配置）、sentry.yaml
assets/           YOLOv5 的 OpenVINO IR 模型
```

## 构建

依赖：OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann_json、OpenVINO 2024.6、海康 MVS SDK（库文件在 `io/hikrobot/lib/<arch>/`，已随仓库带）。

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2 task_1 task_2 task_3
```

## 运行

必须在工程根目录启动（配置里的模型路径是相对路径）：

```bash
./build/task_1 configs/standard.yaml
```

任务一的参数表、硬件核对与验收判据见 `submit/task_1` 分支的提交说明。
