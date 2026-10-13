#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{
class CameraBase
{
public:
  virtual ~CameraBase() = default;
  virtual void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) = 0;
  virtual bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout) = 0;
  virtual bool is_alive() const = 0;
};

class Camera
{
public:
  Camera(const std::string & config_path);
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);
  bool try_read_for(
    cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
    std::chrono::milliseconds timeout);
  bool is_alive() const;

private:
  std::unique_ptr<CameraBase> camera_;
};

}  // namespace io

#endif  // IO__CAMERA_HPP