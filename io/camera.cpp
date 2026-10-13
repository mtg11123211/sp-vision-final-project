#include "camera.hpp"

#include <stdexcept>

#include "hikrobot/hikrobot.hpp"
#include "tools/recorder.hpp"
#include "tools/replay.hpp"
#include "tools/yaml.hpp"
#include "video/video_camera.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  // ---------- 录包（可选）----------
  // 配置里 record_dir 非空，或者环境变量 DSH_RECORD_DIR 非空，就开始录。
  // 两个都不写就永远不打开，对正常跑车没有任何影响。
  std::string record_dir = tools::replay::env_dir("DSH_RECORD_DIR");
  if (record_dir.empty()) record_dir = tools::read_or<std::string>(yaml, "record_dir", "");
  if (!record_dir.empty()) {
    tools::Recorder::instance().open(
      record_dir, tools::read_or<int>(yaml, "record_every", 1),
      tools::read_or<int>(yaml, "record_jpeg_quality", 95));
  }

  // ---------- 回放（可选）----------
  // 配置里 replay_dir 非空，或者环境变量 DSH_REPLAY_DIR 非空，就用录包代替真相机。
  // 云台那边看到同一个键也会切到回放模式，所以两边是一起生效的。
  std::string replay_dir = tools::replay::env_dir("DSH_REPLAY_DIR");
  if (replay_dir.empty()) replay_dir = tools::read_or<std::string>(yaml, "replay_dir", "");
  if (!replay_dir.empty()) {
    camera_ =
      std::make_unique<VideoCamera>(replay_dir, tools::read_or<bool>(yaml, "replay_loop", true));
    return;
  }

  // ---------- 真相机 ----------
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  auto rotation_angle = tools::read<int>(yaml, "rotation_angle");

  if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, rotation_angle);
  }

  else if (camera_name == "hikrobot_sn") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    auto serial_number = tools::read<std::string>(yaml, "serial_number");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid, serial_number, rotation_angle);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);

  // 录包的图像钩子。没开录包时这一句立刻返回，开销可以忽略。
  tools::Recorder::instance().on_image(img, timestamp);
}

bool Camera::try_read_for(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp,
  std::chrono::milliseconds timeout)
{
  const bool ok = camera_->try_read_for(img, timestamp, timeout);
  if (ok) tools::Recorder::instance().on_image(img, timestamp);
  return ok;
}

bool Camera::is_alive() const { return camera_->is_alive(); }

}  // namespace io
