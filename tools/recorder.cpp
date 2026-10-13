#include "recorder.hpp"

#include <filesystem>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace tools
{

Recorder & Recorder::instance()
{
  static Recorder r;
  return r;
}

bool Recorder::open(const std::string & dir, int every, int jpeg_quality)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (recording_) return true;
  if (dir.empty()) return false;

  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    logger()->error("[Recorder] 建目录失败 {}：{}", dir, ec.message());
    return false;
  }

  csv_.open(dir + "/frames.csv", std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    logger()->error("[Recorder] 打不开 {}/frames.csv", dir);
    return false;
  }

  dir_ = dir;
  every_ = every < 1 ? 1 : every;
  jpeg_quality_ = jpeg_quality;
  seen_ = 0;
  written_ = 0;
  has_t0_ = false;
  has_pending_ = false;
  recording_ = true;

  csv_ << "# frame,t_sec,qw,qx,qy,qz,bullet_speed,bullet_count,mode\n";
  csv_ << std::fixed << std::setprecision(6);

  logger()->info(
    "[Recorder] 开始录包到 {}（每 {} 帧存一张，JPEG 质量 {}）", dir, every_, jpeg_quality_);
  return true;
}

void Recorder::on_image(const cv::Mat & img, std::chrono::steady_clock::time_point t)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recording_) return;

  const long long idx = seen_++;
  // 隔帧存的话，这一帧连克隆都不用做，省一次大块内存拷贝
  if (idx % every_ != 0) return;
  if (img.empty()) return;

  pending_img_ = img.clone();
  pending_t_ = t;
  has_pending_ = true;
}

void Recorder::on_quaternion(
  const Eigen::Quaterniond & q, std::chrono::steady_clock::time_point t, double bullet_speed,
  uint16_t bullet_count, uint8_t mode)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!recording_) return;
  if (!has_pending_) return;

  if (!has_t0_) {
    t0_ = pending_t_;
    has_t0_ = true;
  }

  std::ostringstream name;
  name << dir_ << "/" << std::setw(6) << std::setfill('0') << written_ << ".jpg";

  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
  if (!cv::imwrite(name.str(), pending_img_, params)) {
    logger()->warn("[Recorder] 写图失败，这一帧丢掉：{}", name.str());
    has_pending_ = false;
    return;
  }

  // 注意用的是 pending_t_（图的曝光时刻）而不是 t，两者本来是同一个值，
  // 万一对不上，以图为准更安全
  const double t_sec = delta_time(pending_t_, t0_);

  csv_ << written_ << "," << t_sec << "," << q.w() << "," << q.x() << "," << q.y() << ","
       << q.z() << "," << bullet_speed << "," << bullet_count << "," << static_cast<int>(mode)
       << "\n";

  written_++;
  has_pending_ = false;

  // 每 100 帧刷一次盘，万一被强杀也不会丢太多；顺便报一下实际录制帧率。
  // 这一条很重要：录包本身要花时间（JPEG 编码加写盘），主循环会被拖慢，
  // 所以录下来的帧率会低于平时跑车的帧率。看这个数就知道差多少，
  // 差太多就把 record_every 调成 2、或者把 record_jpeg_quality 降到 85。
  if (written_ % 100 == 0) {
    csv_.flush();
    const double elapsed = delta_time(pending_t_, t0_);
    const double fps = elapsed > 1e-6 ? static_cast<double>(written_) / elapsed : 0.0;
    logger()->info(
      "[Recorder] 已录 {} 帧，用时 {:.1f} 秒，实际录制帧率 {:.1f} 帧每秒", written_, elapsed, fps);
  }
}

}  // namespace tools
