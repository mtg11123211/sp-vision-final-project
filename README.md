# task_1 基础自瞄 —— 提交说明

- 提交基线：**`c37d941`**（本分支 `submit/task_1` / tag `task1-1017`）
- 代码范围：`src/task_1.cpp`，共用 `io/` `tasks/auto_aim/` `tools/`
- 本版 `configs/standard.yaml` 是**仓库原始值**，不含现场调试时的临时调参

---

## 1. 这个程序做什么

一帧一条链路：

```
相机取帧(1408×1104) → YOLOv5(OpenVINO) 检测装甲板 → PnP + 坐标变换求出装甲板的世界坐标
→ 转成 yaw / pitch / 距离 → 通过串口(921600, 30 字节协议)发给云台
→ 同时窗口左上角打印 yaw/pitch/dist，Plotter 推曲线，按 q 退出
```

细节：

- 只对准**装甲板中心**，不算提前量（那是 task_2 的事）
- 丢目标后 **300 ms 内保持上一次角度**（`target_hold_ms`），超过就不发控制
- 退出时主动发一帧「不控制」，把云台交还电控

---

## 2. 怎么编

依赖（缺一个都过不了 `cmake`）：OpenCV、Eigen3、fmt、spdlog、yaml-cpp、nlohmann_json、OpenVINO 2024.6、海康 MVS SDK（`io/hikrobot/lib/amd64/libMvCameraControl.so` 已随仓库带）。

```bash
cd final_project_aim
mkdir -p build && cd build
cmake .. && make -j2 task_1
```

产物：`build/task_1`

> ⚠️ `CMakeLists.txt:14` 把 `OpenVINO_DIR` 写死成 `/opt/intel/openvino_2024.6.0/runtime/cmake/`。
> 本机路径不一样就改这一行，或者 `cmake -DOpenVINO_DIR=<你的路径> ..`。

---

## 3. 怎么跑

```bash
cd final_project_aim                                    # 必须在工程根！
./build/task_1 configs/standard.yaml
```

> ⚠️ `configs/standard.yaml:7` 的 `yolov5_model_path: assets/yolov5.xml` 是**相对路径**。
> 在别的目录启动会报 `ov::Exception … Could not open the file: "assets/yolov5.xml"`。

带录包跑（每帧存 jpg + `frames.csv`）：

```bash
DSH_RECORD_DIR=~/recordings/任务一_$(date +%m%d_%H%M) ./build/task_1 configs/standard.yaml
```

---

## 4. 硬件

| 项 | 值 | 说明 |
|---|---|---|
| 相机 | HikRobot USB，`vid_pid: 2bdf:0001` | yaml 第 25 行 |
| 云台串口 | `/dev/gimbal`，921600 | 需要 udev 规则；没有云台时程序会打印 `Failed to open serial` 但检测照跑 |
| 分辨率 | 1408×1104 | 与 `camera_matrix` 的主点 `704.312 / 551.847` 对应 |

**没有硬件也能跑通前半段**（`io/camera.cpp` 支持回放）：

```bash
DSH_REPLAY_DIR=<录包目录> ./build/task_1 configs/standard.yaml
```

> 录包目录需要 `frames.csv` + `%06d.jpg`（1408×1104）。仓库里**不含**录制数据（`.gitignore` 排除了 `recordings/`），需要的话单独提供。

---

## 5. 代码导览（按这个顺序看，10 分钟）

| 位置 | 内容 |
|---|---|
| `src/task_1.cpp:32` | `tools::Plotter plotter;` |
| `src/task_1.cpp:62` | `q = gimbal.q(t);` 取云台姿态（判空后） |
| `src/task_1.cpp:81-82` | `last_yaw/last_pitch = armor.ypd_in_world[0]/[1]` |
| `src/task_1.cpp:87` | `gimbal.send(true, false, last_yaw, last_pitch);` ← **控制发出的唯一入口** |
| `src/task_1.cpp:90-95` | 左上角黄字 `yaw … pitch … dist … m` |
| `src/task_1.cpp:98-103` | Plotter 组 `aim`（yaw/pitch/distance） |
| `src/task_1.cpp:106` | `cv::imshow("task1", img)`，`:115` 退出前 `send(false,false,0,0)` |
| `io/camera.cpp` | 相机/回放二选一（读 `DSH_RECORD_DIR` / `DSH_REPLAY_DIR`） |
| `tasks/auto_aim/yolos/yolov5.cpp` | 前处理 + OpenVINO 推理 + 后处理；`:247` 左上角帧计数，`:261` `detection` 窗口 |
| `tasks/auto_aim/solver.cpp:84/88/99` | 相机系 → 云台系 → 世界系 → `ypd` |
| `tools/math_tools.cpp:103-110` | `xyz2ypd`：`yaw=atan2(y,x)`、`pitch=atan2(z,√(x²+y²))` |
| `io/gimbal/gimbal.hpp:40-54` | 下行 30 字节 `VisionToGimbal` 结构 |
| `io/gimbal/gimbal.cpp:100-108` | 写 `mode` + yaw/pitch 六个量 |
| `io/gimbal/gimbal.cpp:236-252` | 收包、找 `'S','P'`、CRC16 校验 |

坐标系约定：世界系 **z 朝上**，`pitch` 为正表示目标在水平线上方。

---

## 6. 实测结果

- 分辨率 1408×1104；终端 `[Recorder] 实际录制帧率` 从 **37 fps 爬到 49 fps**（稳定 ≈48 fps，无严重掉帧）
- 检测框稳定，云台能跟随装甲板；窗口中左上角白字 `[N]` 是**帧计数**，不是错误码
- 录包样本（可另行提供）：`任务一_1016_1531` / `1534` / `1552`

---

## 7. 已知问题 / 换车必看

1. **颜色类别反转（已定位未修）**
   `tasks/auto_aim/armor.cpp:175` 与 `:216` 把模型输出的颜色类别按 `[蓝, 红]` 解释，而模型实际输出是 `[红, 蓝]` ⇒ yaml 写 `enemy_color: "red"` 时程序只认蓝。
   task_1 本身不按颜色过滤，所以只影响调试显示；要做按色筛选必须改这两行。
2. **pitch 取反**
   本框架 `pitch` 为正 = 目标在上方，而本车电控的方向相反 ⇒ `io/gimbal/gimbal.cpp:106` 发的是 `tx_data_.pitch = -pitch;`。**换电控/换车必须先用「板子抬高、看云台是抬头还是低头」确认这一行。**
3. `yaw_offset` / `pitch_offset`（yaml:19/20）在本版是**保留字段，代码不读**（源码里没有任何读取点）。
4. `default_bullet_speed: 25.0` 是兜底值，task_1 不用它（没有提前量计算）。
5. `use_traditional`、`rotation_angle` 同样是死参数。

---

## 8. task_1 真正用到的参数（`configs/standard.yaml`）

| 行 | 键 | 值 | 作用 |
|---|---|---|---|
| 1 | `enemy_color` | `"red"` | 目标颜色（注意 §7.1 的反转） |
| 7 | `yolov5_model_path` | `assets/yolov5.xml` | 相对路径，必须在工程根启动 |
| 8 | `device` | `GPU` | 虚拟机无 GPU 时 `export DSH_DEVICE=CPU` |
| 9 | `min_confidence` | `0.8` | 检测阈值 |
| 11 | `nms_threshold` | — | NMS |
| 25 | `vid_pid` | `2bdf:0001` | 相机 VID:PID |
| 33 | `camera_matrix` | fx 1809.234 / fy 1813.010 / cx 704.312 / cy 551.847 | 与 1408×1104 绑定 |
| 34 | `distort_coeffs` | `[-0.0772, 0.1504, 0.00157, 0.00104, 0]` | 畸变，边缘误差大 |
| 43 | `com_port` | `"/dev/gimbal"` | 云台串口 |
| 45 | `baudrate` | `921600` | |
| 51 | `target_hold_ms` | `300` | 丢目标后保持时间 |

---

## 9. 怎么判断"跑对了"

1. 启动日志出现 `[Gimbal] Serial opened at 921600 baudrate.` 和 `[Gimbal] First q received.`（没有云台时只有前者失败提示）
2. `detection` 窗口能看到实时画面 + 绿框
3. `task1` 窗口左上角黄字随装甲板移动而变化（`yaw` 连续、`pitch` 随高度变化、`dist` 随距离变化）
4. 终端每 5 秒打印一次录制帧率（开了 `DSH_RECORD_DIR` 时）
