#ifndef SRC__LOW_PLATE_HPP
#define SRC__LOW_PLATE_HPP

// 「只打低位靶，不打高位靶」这条策略的公共实现（**三态**）。
//
// 背景（口径已按 2026-10-21 评审修正）：
//   车体四块板里 1、3 号比 0、2 号高，这个高度差就是 EKF 状态里的 x[10]。
//   target.cpp 的 update_ypda 只在**首次换板前**冻结几何列
//   （rlh_initialized_ 之前把 H 的后三列清零），换板之后是允许单板观测更新高度差的。
//   所以真实情况是**弱化版**：x[10] 从 0 起步、先验很紧（P0=0.001）、靠换板观测慢慢收敛，
//   **收敛之前高板的 z 偏低**（不是"永久冻结"）。
//   ⇒ "永久只打低位"是把一个**瞬态问题**当永久问题处理，收敛后白白丢掉一半开火机会。
//   ⇒ 因此做成三态：暖机期只打低位，高度差稳定后自动放开高板。
//
// 三态语义（DSH_LOW_PLATE）：
//   0 / off    完全不参与（连几何都不算）
//   1 / warmup 【默认】x[10] 不可信时只打低位；可信后自动放开高板
//   2 / always 永远只打低位（保守模式，正式提交想兜底再用）
//
// 「x[10] 是否可信」的判据（只用已有可观测量，不改框架）：
//   四块板的高度差 = z_max − z_min（对高板瞄时等于 z_aim − z_min）。
//   这个差值**稳定地**大于容差、并连续保持 DSH_LOW_PLATE_TRUST_S 秒（默认 4 s）、
//   且期间 EKF 没有重建 ⇒ 认为高度差已收敛 ⇒ warmup 模式放开高板。
//
// 两个防抖（评审附的 bug 清单 ★2 / ★3）：
//   ① `active`（高低分得开）加迟滞：进入用 spread > z_tol，退出用 spread < 0.5*z_tol，
//      避免差值在容差边界抖动导致 active 反复开关、高板被瞬时放行；
//   ② 信任计时只在「有观测 && active」的帧上累计，丢观测暂停（不清零），
    //      高低重新分不开（active 迟滞退出）才作废累计（不拿旧数据判收敛）。
//
// 平靶（四板同高）时 spread 永远 < tol ⇒ active=false ⇒ 规则自动失效，warmup 与 always 等价，安全。

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "src/env_util.hpp"

namespace policy
{

// ---------------- 低位靶三态 ----------------
enum class LowMode
{
  Off = 0,     // 不参与
  Warmup = 1,  // 暖机期只打低位，可信后放开
  Always = 2   // 永远只打低位
};

// 解析 DSH_LOW_PLATE。**不能用 env_bool**（"2" 会被当成非零真值、或者被当成非法回落，
// 静默降级）。返回值同时给出"原字符串"，启动日志要把它打出来，让人一眼看出解析成了什么。
inline LowMode parse_low_mode(const char * raw, std::string * raw_out)
{
  std::string s = (raw == nullptr) ? std::string() : std::string(raw);
  if (raw_out != nullptr) *raw_out = s;
  if (s.empty()) return LowMode::Warmup;  // 缺省 = warmup
  std::string t = s;
  std::transform(t.begin(), t.end(), t.begin(), [](unsigned char c) { return std::tolower(c); });
  if (t == "0" || t == "off" || t == "false" || t == "no" || t == "disable") return LowMode::Off;
  if (t == "1" || t == "warmup" || t == "true" || t == "yes" || t == "enable")
    return LowMode::Warmup;
  if (t == "2" || t == "always") return LowMode::Always;
  return LowMode::Warmup;  // 认不出来 ⇒ 回到 warmup（最温和的一态）
}

inline const char * low_mode_name(LowMode m)
{
  switch (m) {
    case LowMode::Off:
      return "off(不参与)";
    case LowMode::Warmup:
      return "warmup(暖机期只打低位，可信后放开)";
    case LowMode::Always:
      return "always(永远只打低位)";
  }
  return "unknown";
}

// ---------------- 一次评估的结果 ----------------
struct LowPlate
{
  LowMode mode = LowMode::Warmup;
  bool active = false;    // 高低确实分得开（带迟滞）
  bool trusted = false;   // warmup：高度差已稳定收敛
  double z_min = 0.0;    // 候选板里的最低高度
  double z_max = 0.0;    // 最高高度
  double z_spread = 0.0; // z_max − z_min，就是 x[10]（瞄高板时等于 z_aim − z_min）
  double z_aim = 0.0;    // 当前瞄的那块的高度
  double dz = 0.0;       // z_aim − z_min
  int low_count = 0;     // 低位组里有几块
  int total = 0;         // 候选板一共几块
  bool aim_is_low = true;// 当前瞄的这块是不是低位
};

// ---------------- 有状态的判定器（跨帧） ----------------
class LowPlateGate
{
public:
  LowPlateGate(LowMode mode, double z_tol, double trust_s)
  : mode_(mode), z_tol_(z_tol > 0.0 ? z_tol : 0.10), trust_s_(trust_s > 0.0 ? trust_s : 4.0)
  {
  }

  // plates：EKF 预测的候选板（每项 [x, y, z, a]）
  // aim_id：当前瞄的那块下标（越界按"不阻塞"处理）
  // has_detection：这一帧有没有观测
  // dt：这一帧的间隔（秒），用于信任计时
  // rebuilt：EKF/目标是否重建（换目标、发散重建）⇒ 高度差重新从 0 学起，信任作废
  bool allows_fire(
    const std::vector<Eigen::Vector4d> & plates, int aim_id, bool has_detection, double dt,
    bool rebuilt)
  {
    st_ = LowPlate{};
    st_.mode = mode_;

    if (mode_ == LowMode::Off) return true;
    if (plates.empty()) return true;

    st_.total = static_cast<int>(plates.size());
    st_.z_min = plates[0](2);
    st_.z_max = plates[0](2);
    for (const auto & p : plates) {
      st_.z_min = std::min(st_.z_min, p(2));
      st_.z_max = std::max(st_.z_max, p(2));
    }
    st_.z_spread = st_.z_max - st_.z_min;
    for (const auto & p : plates) {
      if (p(2) <= st_.z_min + z_tol_) st_.low_count++;
    }
    if (aim_id >= 0 && aim_id < st_.total) {
      st_.z_aim = plates[aim_id](2);
      st_.dz = st_.z_aim - st_.z_min;
      st_.aim_is_low = (st_.z_aim <= st_.z_min + z_tol_);
    }

    // ① active 迟滞：进入 spread > z_tol；退出 spread < 0.5 * z_tol
    if (!active_) {
      active_ = (st_.z_spread > z_tol_);
    } else if (st_.z_spread < 0.5 * z_tol_) {
      active_ = false;
    }
    st_.active = active_;

    // ② 信任计时：只在「有观测 && active」的帧累计；丢观测**暂停不清零**
    //    （旋转靶换板瞬间丢 1~3 帧检测是常态，一丢帧就清零会让 4 s 永远攒不满，
    //     warmup 静默退化成 always——2026-10-21 复验修正）；
    //    高低重新分不开（active 迟滞退出）或 EKF 重建才作废累计。
    if (rebuilt) {
      trust_acc_ = 0.0;
      trusted_ = false;
    } else if (has_detection && st_.active) {
      if (!trusted_) {
        trust_acc_ += (dt > 0.0 ? dt : 0.0);
        if (trust_acc_ >= trust_s_) {
          trusted_ = true;
          flipped_ = true;
        }
      }
    } else if (!st_.active) {
      trust_acc_ = 0.0;
      // 高低又分不开了 ⇒ 信任一并作废（否则 trusted_ 是"粘"的：
      // 一旦授予就永远有效，保护再也回不来，与上面的注释语义不符）
      trusted_ = false;
    }
    // has_detection == false && active：暂停（保住已累计的时间）
    st_.trusted = trusted_;

    // 放行判定
    if (mode_ == LowMode::Always) {
      return !st_.active || st_.aim_is_low;  // 永远只打低位
    }
    if (st_.trusted) return true;             // warmup：收敛后放开高板
    return !st_.active || st_.aim_is_low;     // warmup 未收敛：按老规矩只打低位
  }

  const LowPlate & state() const { return st_; }

  // 信任状态是否刚刚翻转（取走后清零），用来打一行日志
  bool take_flip()
  {
    const bool f = flipped_;
    flipped_ = false;
    return f;
  }

  double trust_progress() const { return trust_acc_; }
  double trust_need() const { return trust_s_; }

private:
  LowMode mode_ = LowMode::Warmup;
  double z_tol_ = 0.10;
  double trust_s_ = 4.0;
  bool active_ = false;     // 带迟滞的"高低分得开"
  bool trusted_ = false;    // 高度差已收敛
  bool flipped_ = false;
  double trust_acc_ = 0.0;
  LowPlate st_{};
};

}  // namespace policy

#endif  // SRC__LOW_PLATE_HPP
