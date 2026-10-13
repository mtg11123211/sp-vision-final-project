#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  ArmorName name;
  ArmorType armor_type;
  ArmorPriority priority;
  bool jumped;
  int last_id;  // debug only

  Target() = default;
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius = 0.2,
    int armor_num = 4);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  // 带异常量测门限的更新。
  // 先照常走一遍 update()，如果这一帧观测的 NIS 超过 nis_reject_threshold，
  // 就把状态 x 与协方差 P 回滚到更新前（只保留装甲板关联结果 last_id），并返回 false。
  // 用途：挡住单帧误关联、以及前哨站顶部装甲板这类不属于当前模型的观测。
  // nis_reject_threshold 建议取 4 自由度卡方分布的 99.7% 分位 16.27
  // （95% 是 9.488、99% 是 13.28、99.9% 是 18.47）。
  bool update_checked(const Armor & armor, double nis_reject_threshold);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  // 当前三块（或四块）装甲板里最高的那块的世界系高度，单位米。
  // 前哨站的顶部装甲板明细高于三块中部装甲板，用它把顶部板挡在门外。
  double armor_z_max() const;

  bool diverged() const;

  void update_convergency();
  bool is_converged() const;
  bool convergened(); 

  bool isinit = false;

  bool checkinit();

private:
  int armor_num_;
  int switch_count_;
  int update_count_;

  bool is_switch_, is_converged_;
  bool rlh_initialized_ = false;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  bool is_layered_outpost() const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP