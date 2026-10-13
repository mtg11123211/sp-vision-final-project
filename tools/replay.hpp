#ifndef TOOLS__REPLAY_HPP
#define TOOLS__REPLAY_HPP

// 离线拟真用的回放数据集。
//
// 录包目录里有两种文件：
//   000000.jpg 000001.jpg ...   每帧的图（已经是旋转之后的图，回放时不用再转）
//   frames.csv                  每帧的元数据，列顺序：
//                               frame,t_sec,qw,qx,qy,qz,bullet_speed,bullet_count,mode
//
// 相机和云台共用同一份 Dataset 和同一个时间原点：
//   相机负责产生时间戳（原点 + 第几圈乘总时长 + 这一帧在录包里的时刻）；
//   云台拿到这个时间戳之后，取模回到录包里，再查表得到当时的姿态与弹速。
// 两边用的是同一个 origin()，所以对得上。

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace tools
{
namespace replay
{

struct Frame
{
  double t = 0.0;                                    // 相对录制开始时刻的秒数
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // 该帧曝光时刻的云台姿态
  double bullet_speed = 0.0;
  uint16_t bullet_count = 0;
  uint8_t mode = 0;
};

class Dataset
{
public:
  bool load(const std::string & dir);
  bool ok() const { return ok_; }
  const std::string & dir() const { return dir_; }
  size_t size() const { return frames_.size(); }
  double duration() const { return duration_; }
  const Frame & at_index(size_t i) const { return frames_[i]; }

  // 按「录包内时刻」取一帧，帧与帧之间线性插值（姿态用球面插值）
  Frame sample(double src_time) const;

private:
  std::string dir_;
  std::vector<Frame> frames_;
  double duration_ = 0.0;
  bool ok_ = false;
};

// 单例：整个进程只载入一次
Dataset & dataset();

// 时间原点：第一次调用时固定下来，之后不变。相机和云台都用它
std::chrono::steady_clock::time_point origin();

// 读环境变量，没有就返回空串。
// 用途：DSH_RECORD_DIR / DSH_REPLAY_DIR 可以覆盖配置，省得每录一段就改一次 yaml
std::string env_dir(const char * name);

}  // namespace replay
}  // namespace tools

#endif  // TOOLS__REPLAY_HPP
