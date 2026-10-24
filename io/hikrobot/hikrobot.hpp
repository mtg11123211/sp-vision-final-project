#ifndef IO__HIKROBOT_HPP
#define IO__HIKROBOT_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

#include "MvCameraControl.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class HikRobot : public CameraBase
{
public:
  HikRobot(double exposure_ms, double gain, const std::string & vid_pid, int rotation_angle = 0);
  HikRobot(
    double exposure_ms, double gain, const std::string & vid_pid, const std::string & serial_number,
    int rotation_angle = 0);
  ~HikRobot() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout) override;
  bool is_alive() const override;

  // 最近一帧的曝光时间戳（nDevTimeStampHigh/Low 拼成的 64 位 tick），0 表示这台相机没给。
  // 只用来测量延迟，控制回路完全不读它。
  uint64_t dev_ts() const { return dev_ts_.load(); }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    uint64_t dev_ts = 0;
  };

  double exposure_us_;
  double gain_;
  int rotation_angle_;
  std::string serial_number_;

  std::thread daemon_thread_;
  std::atomic<bool> daemon_quit_;

  void * handle_;
  std::thread capture_thread_;
  std::atomic<bool> capturing_;
  std::atomic<bool> capture_quit_;
  std::atomic<uint64_t> dev_ts_{0};
  tools::ThreadSafeQueue<CameraData> queue_;

  int vid_, pid_;

  void capture_start();
  void capture_stop();

  void set_float_value(const std::string & name, double value);
  void set_enum_value(const std::string & name, unsigned int value);

  void set_vid_pid(const std::string & vid_pid);
  void reset_usb() const;

  void rotate(const cv::Mat & src, cv::Mat & dst) const;
};

}  // namespace io

#endif  // IO__HIKROBOT_HPP