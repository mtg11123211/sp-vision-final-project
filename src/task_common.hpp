#ifndef TASK_COMMON_HPP
#define TASK_COMMON_HPP

// task_2 与 task_3 共用的几个纯函数。
// 写成头文件里的 inline 函数，避免两个可执行文件各抄一份。

#include <cmath>
#include <cstdint>
#include <string>

#include <Eigen/Dense>

#include "tasks/auto_aim/armor.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace task_common
{

// 把配置里的 "red" / "blue" 换成枚举。
// 返回 unknown 表示不按颜色过滤（配置写错时的兜底，不会把目标全滤掉）。
inline auto_aim::Color parse_color(const std::string & name)
{
  if (name == "red") return auto_aim::Color::red;
  if (name == "blue") return auto_aim::Color::blue;
  return auto_aim::Color::unknown;
}

// ArmorName 的枚举顺序恰好与电控协议的 target 编号一致，但这里仍然写成显式映射，
// 将来枚举顺序调整了也不会静默传错。
// 协议：0 英雄 / 1 工程 / 2 三号步兵 / 3 四号步兵 / 4 五号步兵 / 5 哨兵 / 6 前哨 / 7 基地 / 8 无目标
inline uint8_t to_target_id(auto_aim::ArmorName name)
{
  switch (name) {
    case auto_aim::ArmorName::one:
      return 0;
    case auto_aim::ArmorName::two:
      return 1;
    case auto_aim::ArmorName::three:
      return 2;
    case auto_aim::ArmorName::four:
      return 3;
    case auto_aim::ArmorName::five:
      return 4;
    case auto_aim::ArmorName::sentry:
      return 5;
    case auto_aim::ArmorName::outpost:
      return 6;
    case auto_aim::ArmorName::base:
      return 7;
    default:
      return 8;
  }
}

// 由「滤波器状态 x」与「要打的那块板 plate」算出瞄准角，两步：
//
//   第 1 步 提前量：把「旋转中心到装甲板」的向量按角速度转过弹丸飞行时间，
//                   再加上旋转中心自身的平移位移。x 的下标含义见 target.cpp 的状态向量注释。
//   第 2 步 弹道：把水平距离与高度差交给 tools::Trajectory，
//                   考虑重力与空气阻力，一次拿到真实的 pitch 和飞行时间；
//                   再用这个飞行时间回到第 1 步重算提前量，迭代两次足够收敛。
//
//   t_extra：飞行时间之外的**额外等待时间**（秒）。用来把「摩擦轮启动到子弹出膛」
//             这段延时算进预测——官方 sp_vision_25 在 aimer.cpp 的注释里写的是
//             「detector-aimer 耗时 0.005 + 发弹延时 0.1」，即出弹本身还有约 0.1 s。
//             不补这一项，4 rad/s 下会系统性偏晚 0.1*14.55 ≈ 1.46°，
//             比板半宽 1.29° 还大 ⇒ 半数以上必偏。缺省 0 = 不改变原有行为。
//
// 直线距离除初速得到的是「几何飞行时间」，既没有重力也没有阻力，
// 直接用它会打低（3 米、初速 25 米每秒时重力下沉 0.071 米，约 1.35 度）。
inline void calc_aim(
  const Eigen::VectorXd & x, const Eigen::Vector4d & plate, double v0, double muzzle_offset_x,
  float & yaw, float & pitch, double t_extra = 0.0)
{
  // 先用直线距离粗估一次飞行时间
  double t_flight = plate.head(3).norm() / v0;
  Eigen::Vector3d aim_xyz = plate.head(3);
  double pitch_rad = tools::xyz2ypd(plate.head(3))[1];

  for (int iter = 0; iter < 2; ++iter) {
    const double da = x[7] * (t_flight + t_extra);  // x[7] 是 EKF 拟合出来的角速度
    const double ca = std::cos(da), sa = std::sin(da);
    const double rx = plate[0] - x[0];
    const double ry = plate[1] - x[2];
    aim_xyz = {
      x[0] + ca * rx - sa * ry + x[1] * t_flight,  // x[0] 中心 x，x[1] 速度 vx
      x[2] + sa * rx + ca * ry + x[3] * t_flight,  // x[2] 中心 y，x[3] 速度 vy
      plate[2] + x[5] * t_flight};                 // plate[2] 该板高度，x[5] 速度 vz

    const double horizontal = std::hypot(aim_xyz.x(), aim_xyz.y());
    double fly_time = 0.0;
    double solved_pitch = pitch_rad;
    // mode = 3 是 17 毫米小弹丸（含空气阻力）的弹道模型
    if (!tools::solve_actual_trajectory(
          v0, horizontal, aim_xyz.z(), 3, muzzle_offset_x, 0.0, solved_pitch, fly_time)) {
      // 弹道无解（距离太小或初速太低）时退回纯几何 pitch，至少不会发出 NaN
      pitch_rad = tools::xyz2ypd(aim_xyz)[1];
      break;
    }
    // 迭代不收敛时可能给出离谱的解，超过 60 度就当成无效，同样退回几何 pitch
    if (!std::isfinite(solved_pitch) || std::abs(solved_pitch) > 60.0 * M_PI / 180.0) {
      pitch_rad = tools::xyz2ypd(aim_xyz)[1];
      break;
    }
    pitch_rad = solved_pitch;
    t_flight = fly_time;
  }

  yaw = static_cast<float>(std::atan2(aim_xyz.y(), aim_xyz.x()));
  pitch = static_cast<float>(pitch_rad);
}

}  // namespace task_common

#endif  // TASK_COMMON_HPP
