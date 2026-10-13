#ifndef TOOLS__RECORDER_HPP
#define TOOLS__RECORDER_HPP

// 录包：把「图像 + 曝光时刻 + 云台四元数 + 弹速」按帧写进一个目录。
//
// 钩子挂在两个地方，所以 task_1 / task_2 / task_3 一行都不用改：
//   Camera::read()  里调 on_image()
//   Gimbal::q(t)    里调 on_quaternion()
// 一帧里这两个调用用的是同一个曝光时刻 t，于是被配成同一行。
//
// 配置里不写 record_dir（或用环境变量 DSH_RECORD_DIR 覆盖）就永远不会打开，
// 对正常跑车没有任何影响。

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>

namespace tools
{

class Recorder
{
public:
  static Recorder & instance();

  // dir 为空表示不录。every 是隔几帧存一张（1 = 每帧都存）
  bool open(const std::string & dir, int every, int jpeg_quality);

  bool recording() const { return recording_; }

  void on_image(const cv::Mat & img, std::chrono::steady_clock::time_point t);
  void on_quaternion(
    const Eigen::Quaterniond & q, std::chrono::steady_clock::time_point t, double bullet_speed,
    uint16_t bullet_count, uint8_t mode);

private:
  Recorder() = default;

  std::mutex mutex_;
  bool recording_ = false;
  std::string dir_;
  int every_ = 1;
  int jpeg_quality_ = 95;

  long long seen_ = 0;     // 总共看到多少帧（含被跳过的）
  long long written_ = 0;  // 已经写出去多少帧，同时也是图片编号
  std::chrono::steady_clock::time_point t0_{};
  bool has_t0_ = false;

  cv::Mat pending_img_;
  std::chrono::steady_clock::time_point pending_t_{};
  bool has_pending_ = false;

  std::ofstream csv_;
};

}  // namespace tools

#endif  // TOOLS__RECORDER_HPP
