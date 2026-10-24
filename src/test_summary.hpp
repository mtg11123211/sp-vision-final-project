#ifndef SRC__TEST_SUMMARY_HPP
#define SRC__TEST_SUMMARY_HPP

// 测试结束时的"统一汇总"机制：把散落在日志里的关键数值收成一整块，
// 在退出时（按 q / Ctrl+C）或按 s 键时一次性打出来，另存一份独立文件。
//
// 为什么需要它：
//   现场要看的东西分散在好几处——启动日志（参数）、[dbg] arrive（errY/blocked）、
//   [dbg] fire-gate（每一项为什么没发）、选板变化（切板频率）、
//   logs/lat_*.csv（视觉延迟与云台滞后，事后用 latency_report.py 算）。
//   几百上千行日志里翻这些数，很容易漏掉某一项 ⇒ 退出时汇总成一块，
//   按 A/B/C 三组排列，直接可以贴进记录表。
//
// 分组（与《现场调试手册》§7 的判据一一对应）：
//   A 组 能不能打出去：挡位、到位滞后 errY、按键后端与人工门
//   B 组 打不打得准  ：视觉延迟、云台反馈滞后、出弹延时、总提前量与补偿角
//   C 组 选板/高板   ：高度差 x[10]、角速度收敛、选板切换频率
//
// 手工补录项（B1/B2 来自 CSV、命中数、真值等）用 add() 塞进来。

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "tools/logger.hpp"

namespace summary
{

// 一帧的状态（由 task_2 每帧填一次）
struct Frame
{
  bool has_target = false;
  bool has_detection = false;
  double yaw_err_deg = 0.0;   // 当前帧的 errY（到位角差）
  bool arrive_ok = false;      // 到位门控是否放行
  bool plate_ok = false;       // 面朝/上膛/低位靶是否放行
  bool key_ok = false;         // 人工开火门是否放行
  bool mode_ok = false;        // 电控挡位是否放行
  bool stable = false;         // 滤波是否收敛
  bool would_fire = false;     // 除到位门控外，其它条件都满足
  bool fire = false;           // 真正发了开火请求
  double dz = 0.0;             // 瞄的板相对最低板的高度
  double z_spread = 0.0;       // 四块板的总高度差（≈ x[10]）
  double trust = 0.0;          // 高度差信任累计
  double w = 0.0;              // EKF 估的角速度
  double t_arrive_ms = 0.0;    // 到达时刻
  int aim_id = -1;
  int gear = 0;                // 电控挡位（0=IDLE,1=AUTO_AIM,...）
  int rounds = 0;              // 电控回报的累计发弹数
  bool plate_switch = false;   // 本帧选板是否变化
};

class TestSummary
{
public:
  // 配置区：启动时把"这次跑的是什么参数"全部登记进来
  void configure(const std::string & tag, const std::string & arrive_desc,
                 const std::string & low_desc, const std::string & key_desc,
                 const std::string & lead_desc, const std::string & fire_delay_desc,
                 const std::string & aim_desc, const std::string & static_desc,
                 const std::string & interval_desc, double lead_s, double fire_delay_s)
  {
    tag_ = tag;
    arrive_desc_ = arrive_desc;
    low_desc_ = low_desc;
    key_desc_ = key_desc;
    lead_desc_ = lead_desc;
    fire_delay_desc_ = fire_delay_desc;
    aim_desc_ = aim_desc;
    static_desc_ = static_desc;
    interval_desc_ = interval_desc;
    lead_s_ = lead_s;
    fire_delay_s_ = fire_delay_s;
  }

  void update(const Frame & f)
  {
    ++frames_;
    if (!f.has_target) {
      ++no_target_frames_;
      return;
    }
    target_frames_++;
    if (f.gear != 0) last_gear_ = f.gear;
    last_w_ = f.w;
    last_t_arrive_ms_ = f.t_arrive_ms;
    last_dz_ = f.dz;
    last_spread_ = f.z_spread;
    last_trust_ = f.trust;
    last_rounds_ = f.rounds;
    if (f.plate_switch) ++plate_switches_;
    if (f.has_detection) {
      err_n_++;
      err_sum_ += f.yaw_err_deg;
      err_max_ = std::max(err_max_, f.yaw_err_deg);
      err_min_ = (err_n_ == 1) ? f.yaw_err_deg : std::min(err_min_, f.yaw_err_deg);
      // 高度差统计：只在瞄到高板时才有意义（dz≈0 时是低板）
      if (f.dz > 0.02) {
        dz_n_++;
        dz_sum_ += f.dz;
        dz_max_ = std::max(dz_max_, f.dz);
      }
    }
    if (f.would_fire && !f.arrive_ok) ++blocked_arrive_;
    if (f.would_fire && f.arrive_ok && !f.plate_ok) ++blocked_plate_;
    if (f.would_fire && f.arrive_ok && f.plate_ok && !f.key_ok) ++blocked_key_;
    if (f.would_fire && f.arrive_ok && f.plate_ok && f.key_ok && !f.mode_ok) ++blocked_mode_;
    if (f.would_fire && f.arrive_ok && f.plate_ok && f.key_ok && f.mode_ok && !f.stable)
      ++blocked_stable_;
    if (f.fire) ++fire_req_;
  }

  // 手工补录（B1/B2 的 CSV 结果、命中数、真值……）
  void add(const std::string & key, const std::string & value)
  {
    manual_.emplace_back(key, value);
  }

  // 收一行启动日志（退出时原样附在汇总最后 ⇒ 这份记录自带"这次是怎么跑的"）
  void add_startup(const std::string & line) { startup_.push_back(line); }

  const std::string & tag() const { return tag_; }

  std::string text() const
  {
    std::ostringstream o;
    o << "\n================ 测试汇总 " << (tag_.empty() ? std::string("untagged") : tag_)
      << " ================\n";
    o << "  [A 组] 能不能打出去\n";
    o << "    A1 电控挡位        : " << gear_name(last_gear_)
      << (last_gear_ == 1 ? "  ✓" : "  ✗ 不是 AUTO_AIM 就是不发弹") << "\n";
    o << "    A2 云台滞后 errY    : 均值 " << fmt1(mean_err()) << "°  最小 " << fmt1(err_min_)
      << "°  最大 " << fmt1(err_max_) << "°  (门限 " << arrive_desc_ << ")\n";
    o << "    A3 按键通道        : " << key_desc_ << "\n";
    o << "  [B 组] 打不打得准\n";
    o << "    B1 视觉延迟        : " << get("B1 视觉延迟") << "\n";
    o << "    B2 云台反馈滞后    : " << get("B2 云台反馈滞后") << "\n";
    o << "    B3 出弹延时        : " << get("B3 出弹延时") << "   (记录值 " << fire_delay_desc_
      << "，仅记录，不再单独累加)\n";
    o << "       总提前量        : " << lead_desc_ << "   ← 方案①：唯一功能旋钮\n";
    o << "       飞行时间        : " << get("飞行时间") << "\n";
    o << "  [C 组] 选板 / 高板\n";
    o << "    C1 高度差 x[10]    : 均值 " << fmt1(dz_n_ ? dz_sum_ / dz_n_ : 0.0) << " m  最大 "
      << fmt1(dz_max_) << " m  (末值 " << fmt1(last_spread_) << " m, " << low_desc_ << ")\n";
    o << "    C2 角速度 w        : 末值 " << fmt2(last_w_) << " rad/s  " << get("C2 角速度真值")
      << "\n";
    o << "    C3 选板切换        : " << plate_switches_ << " 次 / " << fmt1(sec_since_start())
      << " s = " << fmt2(plate_switches_ / std::max(0.001, sec_since_start())) << " 次/秒"
      << "  (4 rad/s 期望 ≈2.5)\n";
    o << "  [统计]\n";
    o << "    总帧数 " << frames_ << "，有目标帧 " << target_frames_ << "，无目标帧 "
      << no_target_frames_ << "\n";
    o << "    开火请求 " << fire_req_ << " 次，电控回报发弹 " << last_rounds_ << " 发  "
      << get("命中数") << "\n";
    o << "    被挡：到位门控 " << blocked_arrive_ << "，面朝/上膛/低位靶 " << blocked_plate_
      << "，人工门 " << blocked_key_ << "，电控挡位 " << blocked_mode_ << "，滤波未收敛 "
      << blocked_stable_ << "\n";
    o << "  [其它]\n";
    o << "    选板模式 " << aim_desc_ << "；面朝窗口/重上膛 " << arrive_desc_ << "\n";
    o << "    静止靶判据 " << static_desc_ << "；发弹间隔 " << interval_desc_ << "\n";
    for (const auto & m : manual_) o << "    · " << m.first << ": " << m.second << "\n";
    if (!startup_.empty()) {
      o << "  [本次启动日志（原样）]\n";
      for (const auto & s : startup_) o << "    " << s << "\n";
    }
    o << "================ 测试汇总 结束 ================\n";
    return o.str();
  }

  // 打印到日志；DSH_SUMMARY_FILE 设了的话再写一份独立文件（防止淹没在日志里）
  void print() const
  {
    const std::string s = text();
    tools::logger()->info("{}", s);
    const char * path = std::getenv("DSH_SUMMARY_FILE");
    if (path != nullptr && path[0] != '\0') {
      std::ofstream fo(path, std::ios::app);
      if (fo) {
        fo << s;
        fo.close();
      }
    }
  }

private:
  static std::string gear_name(int g)
  {
    switch (g) {
      case 0: return "IDLE(手动/未接管)";
      case 1: return "AUTO_AIM(自瞄)";
      case 2: return "SMALL_BUFF(小陀螺)";
      case 3: return "BIG_BUFF(大陀螺)";
      default: return "未知(" + std::to_string(g) + ")";
    }
  }
  static std::string fmt1(double v)
  {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2) << v;
    return o.str();
  }
  static std::string fmt2(double v)
  {
    std::ostringstream o;
    o << std::fixed << std::setprecision(3) << v;
    return o.str();
  }
  double mean_err() const { return err_n_ ? err_sum_ / err_n_ : 0.0; }
  double sec_since_start() const
  {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
  }
  std::string get(const std::string & key) const
  {
    for (const auto & m : manual_) {
      if (m.first == key) return m.second;
    }
    return "（未测）";
  }

  std::string tag_;
  std::string arrive_desc_, low_desc_, key_desc_, lead_desc_, fire_delay_desc_, aim_desc_,
      static_desc_, interval_desc_;
  double lead_s_ = 0.0, fire_delay_s_ = 0.0;
  std::chrono::steady_clock::time_point t0_ = std::chrono::steady_clock::now();

  long frames_ = 0, target_frames_ = 0, no_target_frames_ = 0;
  int last_gear_ = 0;
  double last_w_ = 0, last_t_arrive_ms_ = 0, last_dz_ = 0, last_spread_ = 0, last_trust_ = 0;
  int last_rounds_ = 0;
  long plate_switches_ = 0, fire_req_ = 0;
  long err_n_ = 0;
  double err_sum_ = 0, err_min_ = 0, err_max_ = 0;
  long dz_n_ = 0;
  double dz_sum_ = 0, dz_max_ = 0;
  long blocked_arrive_ = 0, blocked_plate_ = 0, blocked_key_ = 0, blocked_mode_ = 0,
       blocked_stable_ = 0;
  std::vector<std::pair<std::string, std::string>> manual_;
  std::vector<std::string> startup_;  // 启动日志原样留存
};

}  // namespace summary

#endif  // SRC__TEST_SUMMARY_HPP
