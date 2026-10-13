#include "replay.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "tools/logger.hpp"

namespace tools
{
namespace replay
{

namespace
{
double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
}  // namespace

std::string env_dir(const char * name)
{
  const char * v = std::getenv(name);
  return v ? std::string(v) : std::string();
}

bool Dataset::load(const std::string & dir)
{
  // 已经载入过同一个目录就直接返回，避免相机和云台各读一遍
  if (ok_ && dir_ == dir) return true;

  dir_ = dir;
  frames_.clear();
  duration_ = 0.0;
  ok_ = false;

  const std::string path = dir + "/frames.csv";
  std::ifstream f(path);
  if (!f) {
    logger()->error("[replay] 打不开 {}", path);
    return false;
  }

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;

    std::stringstream ss(line);
    std::string tok;
    Frame fr;
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    double bullet_speed = 0.0, bullet_count = 0.0, mode = 0.0;

    try {
      // 第 1 列是图片编号，这里用不上（文件名自己带编号）
      if (!std::getline(ss, tok, ',')) continue;
      if (!std::getline(ss, tok, ',')) continue;
      fr.t = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      w = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      x = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      y = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      z = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      bullet_speed = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      bullet_count = std::stod(tok);
      if (!std::getline(ss, tok, ',')) continue;
      mode = std::stod(tok);
    } catch (const std::exception &) {
      logger()->warn("[replay] 跳过解析不了的一行：{}", line);
      continue;
    }

    fr.q = Eigen::Quaterniond(w, x, y, z).normalized();
    fr.bullet_speed = bullet_speed;
    fr.bullet_count = static_cast<uint16_t>(bullet_count);
    fr.mode = static_cast<uint8_t>(mode);
    frames_.push_back(fr);
  }

  if (frames_.size() < 2) {
    logger()->error("[replay] {} 里只有 {} 行有效数据，至少需要 2 行", path, frames_.size());
    return false;
  }

  // 总时长 = 末帧时刻 + 一个平均帧间隔。
  // 多留这一帧是为了循环播放时时间不会在接缝处停顿。
  duration_ = frames_.back().t + (frames_.back().t - frames_.front().t) /
                                   static_cast<double>(frames_.size() - 1);
  ok_ = true;

  logger()->info(
    "[replay] 载入 {} 帧，总时长 {:.2f} 秒：{}", frames_.size(), duration_, dir_);
  return true;
}

Frame Dataset::sample(double src_time) const
{
  if (frames_.empty()) return Frame{};
  if (src_time <= frames_.front().t) return frames_.front();
  if (src_time >= frames_.back().t) return frames_.back();

  const auto it = std::lower_bound(
    frames_.begin(), frames_.end(), src_time,
    [](const Frame & f, double v) { return f.t < v; });
  const size_t hi = static_cast<size_t>(it - frames_.begin());
  const size_t lo = hi - 1;

  const double span = frames_[hi].t - frames_[lo].t;
  const double k = span > 1e-9 ? clamp01((src_time - frames_[lo].t) / span) : 0.0;

  Frame out = frames_[lo];
  out.q = frames_[lo].q.slerp(k, frames_[hi].q).normalized();
  out.bullet_speed =
    frames_[lo].bullet_speed + k * (frames_[hi].bullet_speed - frames_[lo].bullet_speed);
  // 发弹次数是整数，不插值
  return out;
}

Dataset & dataset()
{
  static Dataset d;
  return d;
}

std::chrono::steady_clock::time_point origin()
{
  static const auto t0 = std::chrono::steady_clock::now();
  return t0;
}

}  // namespace replay
}  // namespace tools
