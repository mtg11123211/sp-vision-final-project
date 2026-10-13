#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.794;
constexpr int MAX_ITER = 100;

constexpr double k1 = (0.47 * 1.169 * M_PI * 0.02125 * 0.02125) / (2 * 0.041);
constexpr double k2 = (0.47 * 1.169 * M_PI * 0.0085 * 0.0085) / (2 * 0.003);

/* 
  假设空气阻力 f = k1 * v, 下方是k1的组成
  C_d = 0.47   无量纲系数，一般球体都用这个值
  p = 1.169    空气密度（kg/m^3）
  r = 0.02125  弹丸半径（m）
  m = 0.041    弹丸质量（kg）
*/

Trajectory::Trajectory(const double v0, const double d, const double h, int mode)
{
  if (mode == 1) {  // 不考虑空气阻力
    auto a = g * d * d / (2 * v0 * v0);
    auto b = -d;
    auto c = a + h;
    auto delta = b * b - 4 * a * c;

    if (delta < 0) {
      unsolvable = true;
      return;
    }

    unsolvable = false;
    auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
    auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
    auto pitch_1 = std::atan(tan_pitch_1);
    auto pitch_2 = std::atan(tan_pitch_2);
    auto t_1 = d / (v0 * std::cos(pitch_1));
    auto t_2 = d / (v0 * std::cos(pitch_2));

    pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
    fly_time = (t_1 < t_2) ? t_1 : t_2;
  }

  else if (mode == 2) {  // 考虑空气阻力(大弹丸)
    if (d < 1e-6) {
      unsolvable = true;
      return;
    }
    double theta = std::atan(h / d);
    double delta_z;
    double center_distance = d;  // 平面距离
    double flyTime;
    for (int i = 0; i < MAX_ITER; i++) {
      // 计算炮弹的飞行时间
      flyTime = (pow(M_E, k1 * center_distance) - 1) / (k1 * v0 * cos(theta));
      delta_z = h - v0 * sin(theta) * flyTime / cos(theta) +
                0.5 * g * flyTime * flyTime / cos(theta) / cos(theta);
      if (fabs(delta_z) < 1e-6) break;
      theta -= delta_z / (-(v0 * flyTime) / pow(cos(theta), 2) +
                          g * flyTime * flyTime / (v0 * v0) * sin(theta) / pow(cos(theta), 3));
    }
    unsolvable = false;
    fly_time = flyTime;
    pitch = theta;
  }

  else if (mode == 3) {  // 考虑空气阻力(小弹丸)
    if (d < 1e-6) {
      unsolvable = true;
      return;
    }
    double theta = std::atan(h / d);
    double delta_z;
    double center_distance = d;  // 平面距离
    double flyTime;
    for (int i = 0; i < MAX_ITER; i++) {
      // 计算炮弹的飞行时间
      flyTime = (pow(M_E, k2 * center_distance) - 1) / (k2 * v0 * cos(theta));
      delta_z = h - v0 * sin(theta) * flyTime / cos(theta) +
                0.5 * g * flyTime * flyTime / cos(theta) / cos(theta);
      if (fabs(delta_z) < 1e-6) break;
      theta -= delta_z / (-(v0 * flyTime) / pow(cos(theta), 2) +
                          g * flyTime * flyTime / (v0 * v0) * sin(theta) / pow(cos(theta), 3));
    }
    unsolvable = false;
    fly_time = flyTime;
    pitch = theta;
  }
}

// 设 L_x 为摩擦轮出弹点到 Pitch 轴的枪管方向距离 = 0.171 米
// 设 L_y 为垂直于枪管方向的偏移量 = 0 米
bool solve_actual_trajectory(
  double v0, double d_center, double h_center, int mode, double L_x, double L_y,
  double & final_pitch, double & final_fly_time)
{
  // 初始化第一次迭代：假设 pitch 为 0，或者直接用目标和云台中心的连线夹角作为初值
  double current_pitch = std::atan2(h_center, d_center);
  double last_pitch = current_pitch;

  const int MAX_OUTER_ITER = 5;         //
  const double PITCH_TOLERANCE = 1e-4;  // 角度收敛阈值 (弧度)

  for (int i = 0; i < MAX_OUTER_ITER; ++i) {
    // 1. 根据当前的 pitch 猜测值，计算实际的发射点坐标偏置
    double delta_x = L_x * std::cos(current_pitch) - L_y * std::sin(current_pitch);
    double delta_y = L_x * std::sin(current_pitch) + L_y * std::cos(current_pitch);

    // 2. 计算从实际发射点到目标的距离差
    double d_real = d_center - delta_x;
    double h_real = h_center - delta_y;

    // 3. 调用你现有的解算核心逻辑 (这里伪代码用构造函数示意)
    Trajectory traj(v0, d_real, h_real, mode);

    if (traj.unsolvable) {
      return false;  // 当前条件下无解
    }

    current_pitch = traj.pitch;
    final_fly_time = traj.fly_time;

    // 4. 判断是否收敛
    if (std::abs(current_pitch - last_pitch) < PITCH_TOLERANCE) {
      final_pitch = current_pitch;
      return true;  // 解算成功并收敛
    }

    last_pitch = current_pitch;
  }

  // 如果达到最大迭代次数仍未收敛，也可以接受最后一次的结果
  final_pitch = current_pitch;
  return true;
}

}  // namespace tools