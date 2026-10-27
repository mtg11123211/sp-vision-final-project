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

  // ---- NIS 死锁看门狗（2026-10-27，详解见 docs/NIS门限讲解与死锁分析_1026.md）----
  // 连续被 NIS 门拒收的计数：接受即清零。超过 kRejectReinitFrames 帧（约 0.35 秒）
  // 说明状态已经锁死在错误解上，滤波器自己出不来，主循环应当丢掉目标、用当前帧重建。
  bool needs_reinit() const { return reject_streak_ >= kRejectReinitFrames; }
  int reject_streak() const { return reject_streak_; }
  static constexpr int kRejectReinitFrames = 20;

  // 构造合理性门禁（防「一帧毒」）：初值全部来自单帧 PnP，一旦那帧是斜板/半遮挡，
  // 整个滤波器会被永久带偏。valid() == false 表示这个 Target 不该入役，
  // 主循环应丢弃它、下一帧再试（详见文档 §2.5）。
  bool valid() const { return valid_; }

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
  int reject_streak_ = 0;   // 连续 NIS 拒收帧数（update_checked 维护，接受即清零）
  bool valid_ = true;       // 构造合理性门禁结果（前哨站 center_z 越界 ⇒ false，不入役）

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle
  bool is_layered_outpost() const;

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP