#ifndef IO__VIDEO_CAMERA_HPP
#define IO__VIDEO_CAMERA_HPP

// 回放用的"假相机"：从录包目录读图片序列当相机。
//
// 关键点是**时间戳来自录包里的 frames.csv，不是按帧率算的**。
// 这样能一比一复现当时的帧间隔——包括丢帧和抖动，
// 回放出来的 EKF 行为和真车上跑的时候一致。

#include <chrono>
#include <opencv2/core.hpp>
#include <string>

#include "io/camera.hpp"

namespace io
{

class VideoCamera : public CameraBase
{
public:
  explicit VideoCamera(const std::string & dir, bool loop);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout) override;

  bool is_alive() const override;

private:
  std::string path_of(size_t index) const;

  std::string dir_;
  bool loop_ = true;
  size_t index_ = 0;
  long long cycle_ = 0;
  std::chrono::steady_clock::time_point origin_;
  double duration_ = 0.0;
  size_t count_ = 0;
  bool warned_missing_ = false;
};

}  // namespace io

#endif  // IO__VIDEO_CAMERA_HPP
