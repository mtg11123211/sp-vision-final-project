#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "src/detect_log.hpp"
#include "src/env_util.hpp"
#include "src/fire_key.hpp"
#include "src/overlay.hpp"
#include "src/task_common.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // ---------------- 读配置 ----------------
  auto yaml = tools::load(config_path);
  auto cfg = yaml["task3"];  // 任务三专属参数；这一节缺省时下面的 read_or 会用兜底值

  // 场上有红蓝两个前哨站，只打敌人那个颜色。配置写错时不强行过滤（下面会退回）。
  const std::string enemy_color_name = tools::read_or<std::string>(yaml, "enemy_color", "red");
  const auto enemy_color = task_common::parse_color(enemy_color_name);

  const double default_bullet_speed = tools::read_or<double>(yaml, "default_bullet_speed", 25.0);
  const double muzzle_offset_x = tools::read_or<double>(yaml, "muzzle_offset_x", 0.171);
  const int target_hold_ms = tools::read_or<int>(yaml, "target_hold_ms", 300);
  const double nis_reject_threshold = tools::read_or<double>(yaml, "nis_reject_threshold", 16.27);
  const int min_frames_for_fire = tools::read_or<int>(yaml, "min_frames_for_fire", 15);
  const double max_nis_failure_rate = tools::read_or<double>(yaml, "max_nis_failure_rate", 0.3);

  // 任务三的硬指标（见大作业要求）：弹频 12 赫兹，30 秒计时，不限发数
  const double fire_interval_s = tools::read_or<double>(cfg, "fire_interval_s", 0.0834);
  const int max_rounds_per_stage = tools::read_or<int>(cfg, "max_rounds_per_stage", 0);
  const double stage_gap_s = tools::read_or<double>(cfg, "stage_gap_s", 3.0);
  const bool require_auto_aim_mode = tools::read_or<bool>(cfg, "require_auto_aim_mode", false);

  // 排除顶部装甲板用的两个高度余量，理由见下面的注释
  const double outpost_z_window = tools::read_or<double>(cfg, "outpost_z_window", 0.30);
  const double outpost_top_margin = tools::read_or<double>(cfg, "outpost_top_margin", 0.12);

  // ================= 新增策略（任务三默认全部关闭，行为与原版一致）=================
  // 1) 人工开火门：任务三要 12 赫兹连打 30 秒、75 发命中，人按不住，所以默认关。
  //    需要"手动确认"调试时用 DSH_FIRE_KEY_ENABLE=1 打开。
  const bool fire_key_enabled = policy::env_bool("DSH_FIRE_KEY_ENABLE", false);
  fire_key::Gate fire_key_gate;
  // 2) 出弹延时（摩擦轮从接到指令到子弹出膛）：**只作记录**，不再单独累加。
  //    官方 sp_vision_25 在 aimer.cpp 的注释里写的是「detector-aimer 0.005 + 发弹延时 0.1」。
  //    方案①（与 task_2 一致）：所有"非飞行"的提前量都放进 DSH_LEAD（总提前量），
  //    DSH_FIRE_DELAY 只用来在报告里记"其中出弹约多少" ⇒ 不会把同一段延迟补两遍。
  const double fire_delay_report_s = policy::env_double("DSH_FIRE_DELAY", 0.0);
  // 3) 总提前量 DSH_LEAD（链路 + 云台滞后 + 出弹延时）——**唯一的功能旋钮**。
  //    前哨站 2.5 rad/s、r=0.2765 m、3 m：视在角速度 ≈ 12.6 °/s ⇒ 缺 57 ms 就是 0.72°，
  //    约 0.6 个板半宽（129 mm 板半张角 1.23°）⇒ 不补就是系统性偏。
  //    注意：static 只在进程启动时读一次 ⇒ 改这个环境变量必须重启进程。
  static const double lead_s = []() { const char * v = std::getenv("DSH_LEAD"); return v ? std::atof(v) : 0.0; }();
  // 2) 低位靶规则在任务三**已摘除**（2026-10-21 评审修订 6）：
  //    前哨站没有低位靶的适应症——三块中部板全是合法目标，顶部那块"高位靶"
  //    已经有 outpost_z_window / outpost_top_margin 两道过滤；而且中部板的阶梯高度
  //    是图纸定值、EKF 全程冻结，不存在 x[10] 收敛问题。
  //    更关键：若沿用 task_2 的 DSH_LOW_PLATE_Z_TOL=0.10，会把 +0.2 m 台阶那块板拦掉
  //    ——12 Hz 拼血量的任务被拦 = 直接掉血量分。少一个门少一个雷。
  tools::logger()->info(
    "[task3] 人工开火门 = {}（模式 {}，按键后端 {}）｜出弹延时（记录值，不单独累加）= {:.3f} s",
    fire_key_enabled ? "开" : "关", fire_key_gate.mode_text(), fire_key_gate.backend_name(),
    fire_delay_report_s);
  tools::logger()->info(
    "[task3] 预测时域 = 总提前 {:.3f}s（其中出弹延时约 {:.3f}s，仅记录不单独累加）+ 飞行（弹道解算内部）",
    lead_s, fire_delay_report_s);
  if (fire_delay_report_s > 0.0 && lead_s <= 0.0) {
    tools::logger()->warn(
      "[task3] ⚠ DSH_FIRE_DELAY={:.3f}s 已设，但 DSH_LEAD=0 ⇒ 出弹延时**没有**进入补偿。"
      "方案①下请把它并进 DSH_LEAD（设 DSH_LEAD={:.3f}），否则这一段等于没补。",
      fire_delay_report_s, fire_delay_report_s);
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用
  // 检测日志：DSH_DET_CSV 没设就什么都不做；离线调 EKF 靠它
  detect_log::DetectLog detlog;
  // 窗口上的状态条：每帧都画，用来证明「还在画」而不是窗口冻住了
  overlay::Hud hud;

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  // 关掉 YOLO 自带的 detection 窗口：每帧一次整幅图克隆、缩放与窗口刷新都在控制链路里
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  // ---------------- 跨帧状态 ----------------
  auto_aim::Target target;  // 第一次看到前哨站时才真正建立
  bool has_target = false;
  int frames_since_init = 0;
  auto last_seen = std::chrono::steady_clock::now();

  // 发弹节流与计数
  uint16_t last_bullet_count = gimbal.state().bullet_count;
  bool bullet_count_alive = false;
  int rounds_this_stage = 0;
  auto last_fire_time = std::chrono::steady_clock::time_point{};
  auto last_round_time = std::chrono::steady_clock::now();

  const auto t_start = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    // Your code start

    // 刷新 F 键状态：每帧都刷，跟有没有目标无关
    fire_key_gate.poll();

    camera.read(img, t);
    if (img.empty()) {
      continue;  // 相机偶尔给空帧，跳过本轮，不发送任何指令
    }

    // 用照片曝光时刻对齐云台姿态
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    const int frame_id = frame_count++;
    auto armors = yolo.detect(img, frame_id);
    const auto gimbal_state = gimbal.state();
    const auto now = std::chrono::steady_clock::now();
    // 离线复现用：这一帧看到的所有板子（颜色、编号、置信度、四个角点）
    detlog.on_frame(frame_id, tools::delta_time(now, t_start), armors);

    // ---- 发弹计数 ----
    const uint16_t count_delta =
      static_cast<uint16_t>(gimbal_state.bullet_count - last_bullet_count);
    if (count_delta != 0) {
      rounds_this_stage += static_cast<int>(count_delta);
      last_bullet_count = gimbal_state.bullet_count;
      bullet_count_alive = true;
      last_round_time = now;
    }
    if (rounds_this_stage > 0 && tools::delta_time(now, last_round_time) > stage_gap_s) {
      rounds_this_stage = 0;
    }

    // ---- 挑目标：只打前哨站，并且要把顶部的第四块装甲板排掉 ----
    // 大作业要求里专门写了「注意是三块悬空的中部装甲板，而不是顶部装甲板」。
    // 但模型给的 outpost 类是按机器人编号来的，顶部板与三块中部板是同一类，
    // 所以只能从高度上区分，分两步挡：
    //   第 1 步（同一帧内）：三块中部装甲板彼此高度差约 0.2 米，
    //                        顶部装甲板在它们上方约 0.3 米，
    //                        所以只保留「最低那块往上 outpost_z_window」之内的检测。
    //   第 2 步（有滤波器时）：用 EKF 预测出来的三块板里最高的那块，
    //                        再往上留 outpost_top_margin，超过的丢掉。
    //   另外 update_checked 的 NIS 门限也会兜住漏网的顶部板。
    std::vector<auto_aim::Armor *> candidates;
    for (auto & armor : armors) {
      if (armor.name != auto_aim::ArmorName::outpost) continue;
      solver.solve(armor);  // 要拿到世界系高度，必须先解算
      candidates.push_back(&armor);
    }

    if (candidates.size() > 1) {
      double z_min = candidates.front()->xyz_in_world[2];
      for (auto * a : candidates) z_min = std::min(z_min, a->xyz_in_world[2]);
      candidates.erase(
        std::remove_if(
          candidates.begin(), candidates.end(), [&](auto_aim::Armor * a) {
            return a->xyz_in_world[2] > z_min + outpost_z_window;
          }),
        candidates.end());
    }

    // 如果配置了敌人颜色、并且视野里确实有这个颜色的前哨站，就只保留那一些。
    // 一块都没匹配上时不强行过滤，避免配置写反导致整场打不出子弹。
    if (enemy_color != auto_aim::Color::unknown && candidates.size() > 1) {
      bool any_matched = false;
      for (auto * a : candidates) any_matched = any_matched || (a->color == enemy_color);
      if (any_matched) {
        candidates.erase(
          std::remove_if(
            candidates.begin(), candidates.end(),
            [&](auto_aim::Armor * a) { return a->color != enemy_color; }),
          candidates.end());
      }
    }

    if (has_target && !candidates.empty()) {
      const double z_limit = target.armor_z_max() + outpost_top_margin;
      candidates.erase(
        std::remove_if(
          candidates.begin(), candidates.end(),
          [&](auto_aim::Armor * a) { return a->xyz_in_world[2] > z_limit; }),
        candidates.end());
    }

    // 在剩下的候选里取离画面中心最近的那块，也就是离准星最近的那块
    auto_aim::Armor * chosen = nullptr;
    {
      double best_score = 1e10;
      for (auto * a : candidates) {
        const double dx = a->center_norm.x - 0.5;
        const double dy = a->center_norm.y - 0.5;
        const double score = dx * dx + dy * dy;
        if (chosen == nullptr || score < best_score) {
          chosen = a;
          best_score = score;
        }
      }
    }

    const bool has_detection = (chosen != nullptr);
    const bool holding =
      !has_detection && has_target &&
      (now - last_seen) < std::chrono::milliseconds(target_hold_ms);

    if (has_detection) {
      auto & armor = *chosen;
      if (!has_target || target.name != armor.name) {
        // 第一次出现前哨站：建 3 块板的滤波器
        target = auto_aim::Target(armor, t, 0.0, 0);
        has_target = true;
        frames_since_init = 0;
      } else {
        target.predict(t);
        if (!target.update_checked(armor, nis_reject_threshold)) {
          tools::logger()->debug("[task3] 这一帧观测被 NIS 门限拒收");
        }
        frames_since_init++;
      }
      last_seen = now;
    } else if (holding) {
      // 前哨站一直在转，短时间漏检很常见，这段时间继续按预测值压着云台
      target.predict(t);
      frames_since_init++;
    }

    if (has_target && target.diverged()) {
      if (has_detection) {
        target = auto_aim::Target(*chosen, t, 0.0, 0);
        frames_since_init = 0;
      } else {
        has_target = false;
      }
    }

    if (!has_target) {
      gimbal.send(false, false, 0.0f, 0.0f);
    } else {
      const Eigen::VectorXd x = target.ekf_x();
      const auto xyza_list = target.armor_xyza_list();
      int aim_id = target.last_id;
      if (aim_id < 0 || aim_id >= static_cast<int>(xyza_list.size())) aim_id = 0;
      const Eigen::Vector4d plate = xyza_list[aim_id];

      double bullet_speed = gimbal_state.bullet_speed;
      if (bullet_speed < 5.0) {
        static bool warned = false;
        if (!warned) {
          tools::logger()->warn(
            "[task3] 电控弹速 {} 不可用，改用配置兜底值 {} 米每秒", gimbal_state.bullet_speed,
            default_bullet_speed);
          warned = true;
        }
        bullet_speed = default_bullet_speed;
      }

      // 提前量 + 弹道解算。
      // 前哨站的切向速度只有约 0.69 米每秒（2.5 弧度每秒 乘 0.2765 米），
      // 提前量约 0.08 米；而 3 米处重力下沉约 0.071 米，对应 pitch 误差 1.35 度，
      // 已经接近一块 129 毫米装甲板张角（约 2.46 度）的一半，所以弹道解算是必须的。
      float aim_yaw = 0.0f;
      float aim_pitch = 0.0f;
      // 2026-10-24 与 task_2 E1 同步：lead 段改走 t_extra 旋转推进（da = w*(t_flight+t_extra) 精确旋转）。
      // 视在角速度×lead 的小角近似在超前>40°失效（cos 为负），中速相位失配可达上百毫秒。
      task_common::calc_aim(
        x, plate, bullet_speed, muzzle_offset_x, aim_yaw, aim_pitch, lead_s);

      // 视在角速度线性补偿已删除：其功能由 calc_aim 的 t_extra 精确旋转接管
      // （与 task_2 E1 同步，2026-10-24）。两处并存会把同一段 lead 补两次（过头 0.83°+）。

      const double nis_failure_rate = target.ekf().data.at("recent_nis_failures");
      const bool stable = target.is_converged() && frames_since_init >= min_frames_for_fire &&
                          nis_failure_rate < max_nis_failure_rate;
      const bool interval_ok = tools::delta_time(now, last_fire_time) >= fire_interval_s;
      const bool rounds_ok =
        (max_rounds_per_stage <= 0) || (rounds_this_stage < max_rounds_per_stage);
      const bool mode_ok = !require_auto_aim_mode || gimbal.mode() == io::GimbalMode::AUTO_AIM;
      // 同 task_2：没有观测就不许开火。前哨站丢失超过 target_hold_ms 后 holding 变假，
      // 但滤波器与 has_target 都还在，不挡住这一条就会对着冻结的外推角度一直打。
      //
      // 新增：人按住 F 才允许开火（默认关）。低位靶规则在任务三已摘除（见上面的说明）。
      const bool key_ok = !fire_key_enabled || fire_key_gate.allowed();
      const bool fire =
        has_detection && stable && interval_ok && rounds_ok && mode_ok && !holding && key_ok;

      if (fire) {
        last_fire_time = now;
        last_round_time = now;
        if (!bullet_count_alive) rounds_this_stage++;
      }

      gimbal.send(mode_ok, fire, aim_yaw, aim_pitch, task_common::to_target_id(target.name));

      if (has_detection) {
        tools::draw_points(img, chosen->points, {0, 255, 0});
      }
      tools::draw_text(
        img,
        fmt::format(
          "{} yaw {:.2f} pitch {:.2f} dist {:.2f} w {:.2f} rounds {}", holding ? "HOLD" : "AIM",
          aim_yaw, aim_pitch, plate.head(3).norm(), x[7], rounds_this_stage),
        cv::Point(10, 70), cv::Scalar(0, 255, 255), 1.0, 2);
      // 新增策略的状态条（2026-10-21：任务三已摘除低位靶门，删掉 low.aim_is_low/low.dz
      // ——本文件没 include low_plate.hpp、也没有 low 的声明，原本是编译错误。
      // 屏幕文字一律 ASCII：cv::putText 的 Hershey 字体画不出中文，真机上是一串问号）
      const char * gate_txt = !fire_key_enabled
                                ? "OFF(auto)"
                                : (fire_key_gate.allowed() ? "ARM" : "SAFE");
      tools::draw_text(
        img,
        fmt::format(
          "F={} ({})  lowgate=OFF  aim_id={}", gate_txt, fire_key_gate.backend_name(), aim_id),
        cv::Point(10, 95), cv::Scalar(0, 255, 255), 0.6, 1);

      plotter.plot({
        {"task3_aim",
         {{"yaw", aim_yaw}, {"pitch", aim_pitch}, {"distance", plate.head(3).norm()}}},
        {"task3_ekf",
         {{"w", x[7]},
          {"abs_w", std::abs(x[7])},
          {"residual_yaw", target.ekf().data.at("residual_yaw")},
          {"residual_distance", target.ekf().data.at("residual_distance")},
          {"nis", target.ekf().data.at("nis")},
          {"nis_failure_rate", nis_failure_rate},
          {"rounds", rounds_this_stage},
          {"fire", fire ? 1 : 0}}}
      });
    }

    // 窗口状态条：放在所有分支之外，没目标时也要画，否则看不出程序还活着
    hud.draw(img, armors, chosen, frame_id, has_target, holding);

    cv::imshow("task3", img);
    const int key_code = cv::waitKey(1);
    if (key_code == 'q') {
      break;
    }
    // F 键的兜底通道：只有在既没有 evdev、又不是终端的时候才起作用
    if (fire_key_gate.uses_window_fallback()) fire_key_gate.feed(key_code);

    // Your code end
  }

  // 退出前主动交出控制权
  gimbal.send(false, false, 0.0f, 0.0f);
  return 0;
}
