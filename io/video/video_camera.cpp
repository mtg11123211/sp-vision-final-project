#include "video_camera.hpp"

#include <cstdlib>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <sstream>
#include <thread>

#include "tools/logger.hpp"
#include "tools/replay.hpp"

namespace io
{

VideoCamera::VideoCamera(const std::string & dir, bool loop) : dir_(dir), loop_(loop)
{
  auto & ds = tools::replay::dataset();
  if (!ds.load(dir)) {
    tools::logger()->error("[VideoCamera] 回放数据载入失败：{}", dir);
    exit(1);
  }

  count_ = ds.size();
  duration_ = ds.duration();
  origin_ = tools::replay::origin();

  tools::logger()->info(
    "[VideoCamera] 回放模式，共 {} 帧，loop = {}", count_, loop_ ? "true" : "false");
}

std::string VideoCamera::path_of(size_t index) const
{
  std::ostringstream oss;
  oss << dir_ << "/" << std::setw(6) << std::setfill('0') << index << ".jpg";
  return oss.str();
}

void VideoCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  const auto & ds = tools::replay::dataset();

  if (!ds.ok() || count_ == 0) {
    img = cv::Mat();
    timestamp = std::chrono::steady_clock::now();
    return;
  }

  if (index_ >= count_) {
    if (!loop_) {
      // 不放循环就慢下来，别把 CPU 空转满
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      img = cv::Mat();
      timestamp = std::chrono::steady_clock::now();
      return;
    }
    index_ = 0;
    cycle_++;
  }

  const double t_sec = static_cast<double>(cycle_) * duration_ + ds.at_index(index_).t;
  timestamp = origin_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                          std::chrono::duration<double>(t_sec));

  img = cv::imread(path_of(index_), cv::IMREAD_COLOR);

  if (img.empty() && !warned_missing_) {
    tools::logger()->warn("[VideoCamera] 读不到图，跳过后面的同名帧：{}", path_of(index_));
    warned_missing_ = true;
  }

  index_++;
}

bool VideoCamera::try_read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds /*timeout*/)
{
  read(img, timestamp);
  return !img.empty();
}

bool VideoCamera::is_alive() const { return count_ > 0; }

}  // namespace io
