#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;  // 抬头为正

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h, int mode = 1);
};

bool solve_actual_trajectory(
  double v0, double d_center, double h_center, int mode, double L_x, double L_y,
  double & final_pitch, double & final_fly_time);

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP