#include <chrono>
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
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
  auto cfg = yaml["task2"];  // 任务二专属参数；这一节缺省时下面的 read_or 会用兜底值

  // 只打这个颜色的装甲板。
  // 原来 enemy_color 是死参数、全工程没有代码读它，等于红蓝不分，锁上队友也会打。
  const std::string enemy_color_name = tools::read_or<std::string>(yaml, "enemy_color", "red");
  const auto enemy_color = task_common::parse_color(enemy_color_name);
  if (enemy_color == auto_aim::Color::unknown) {
    tools::logger()->warn("[task2] enemy_color \"{}\" 无法识别，本轮不做颜色过滤", enemy_color_name);
  }

  const double default_bullet_speed = tools::read_or<double>(yaml, "default_bullet_speed", 25.0);
  const double muzzle_offset_x = tools::read_or<double>(yaml, "muzzle_offset_x", 0.171);
  const int target_hold_ms = tools::read_or<int>(yaml, "target_hold_ms", 300);
  const double nis_reject_threshold = tools::read_or<double>(yaml, "nis_reject_threshold", 16.27);
  const int min_frames_for_fire = tools::read_or<int>(yaml, "min_frames_for_fire", 15);
  const double max_nis_failure_rate = tools::read_or<double>(yaml, "max_nis_failure_rate", 0.3);

  // 任务二的硬指标（见大作业要求）：单发模式、弹频 5 赫兹、每个转速挡位只允许发射 10 发
  const double fire_interval_s = tools::read_or<double>(cfg, "fire_interval_s", 0.2);
  const int max_rounds_per_stage = tools::read_or<int>(cfg, "max_rounds_per_stage", 10);
  const double stage_gap_s = tools::read_or<double>(cfg, "stage_gap_s", 3.0);
  // 低转速与中转速的分界（要求里写的是 5 弧度每秒）
  const double speed_band_split = tools::read_or<double>(cfg, "speed_band_split", 5.0);
  // 是否要求云台处于自瞄挡位才接管控制。默认关，现场确认电控会发 mode=1 之后再打开。
  const bool require_auto_aim_mode = tools::read_or<bool>(cfg, "require_auto_aim_mode", false);

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  // YOLO 第二个参数是它自带的 detection 显示窗口开关。关掉它可以省掉每帧一次
  // 整幅图克隆、缩放与窗口刷新，控制链路延迟更低；需要看检测框时临时改成 true。
  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  // ---------------- 跨帧状态 ----------------
  auto_aim::Target target;  // 第一次看到目标时才真正建立
  bool has_target = false;
  int frames_since_init = 0;  // 滤波器建立后跑了多少帧，是开火判据的一部分
  auto last_seen = std::chrono::steady_clock::now();

  // 发弹节流与计数
  uint16_t last_bullet_count = gimbal.state().bullet_count;
  bool bullet_count_alive = false;  // 电控的累计发弹数是否真的在变化
  int rounds_this_stage = 0;
  auto last_fire_time = std::chrono::steady_clock::time_point{};
  auto last_round_time = std::chrono::steady_clock::now();
  // 拟合角速度落在哪个转速挡位（0 低速 / 1 中速），用来识别转速挡位切换
  int speed_band = -1;
  int band_at_last_round = -1;

  while (!exiter.exit()) {
    // Your code start

    camera.read(img, t);
    if (img.empty()) {
      continue;  // 相机偶尔给空帧，跳过本轮，不发送任何指令
    }

    // 用照片曝光时刻对齐云台姿态
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img, frame_count++);
    const auto gimbal_state = gimbal.state();
    const auto now = std::chrono::steady_clock::now();

    // ---- 发弹计数 ----
    // 电控上报的 bullet_count 是累计发弹数，最准；万一这个字段没接，
    // 就退回统计我们自己发出去的开火命令（见下面的 fire 分支）。
    const uint16_t count_delta =
      static_cast<uint16_t>(gimbal_state.bullet_count - last_bullet_count);
    if (count_delta != 0) {
      rounds_this_stage += static_cast<int>(count_delta);
      last_bullet_count = gimbal_state.bullet_count;
      bullet_count_alive = true;
      last_round_time = now;
    }

    // 转速挡位切换的判定。
    // 要求里是两个转速之间关闭摩擦轮，所以「很久没有新弹丸」是第一个信号；
    // 但同一次射击中途也可能停顿，所以再加上「拟合角速度换挡了」这个条件，
    // 免得同一次射击被误判成两次、把发数上限提前用光，害得下一个转速挡位一发都打不出去。
    // 兜底：停顿超过 3 倍 stage_gap_s 一律重新计数，防止角速度还没收敛时卡在旧计数上。
    if (rounds_this_stage > 0) {
      const double gap = tools::delta_time(now, last_round_time);
      const bool band_changed =
        speed_band >= 0 && band_at_last_round >= 0 && speed_band != band_at_last_round;
      if (gap > stage_gap_s && (band_changed || gap > 3.0 * stage_gap_s)) {
        rounds_this_stage = 0;
        band_at_last_round = -1;
      }
    }

    // ---- 挑目标 ----
    // 颜色必须是敌人；位置取离画面中心最近的那块，也就是离准星最近的那块。
    // 只解算选中的那一块，可以省掉其它装甲板的 optimize_yaw（每块 140 次重投影）。
    auto_aim::Armor * chosen = nullptr;
    {
      // 按「离画面中心最近」打分，filter_color 为假时不看颜色
      auto pick = [&](bool filter_color) -> auto_aim::Armor * {
        auto_aim::Armor * best = nullptr;
        double best_score = 1e10;
        for (auto & armor : armors) {
          if (filter_color && armor.color != enemy_color) continue;
          // center_norm 是归一化到 0 到 1 的像素坐标，云台正前方大约落在 (0.5, 0.5)
          const double dx = armor.center_norm.x - 0.5;
          const double dy = armor.center_norm.y - 0.5;
          const double score = dx * dx + dy * dy;
          if (best == nullptr || score < best_score) {
            best = &armor;
            best_score = score;
          }
        }
        return best;
      };

      if (enemy_color != auto_aim::Color::unknown) {
        chosen = pick(true);
        if (chosen == nullptr) {
          // 一块敌人颜色的板都没有，多半是配置颜色写反了。
          // 这里退回不筛颜色并告警一次，否则整场一颗子弹都打不出去。
          static bool warned = false;
          if (!warned) {
            tools::logger()->warn(
              "[task2] 视野里没有 {} 色的装甲板，退回不筛颜色（请核对配置里的 enemy_color）",
              enemy_color_name);
            warned = true;
          }
          chosen = pick(false);
        }
      } else {
        chosen = pick(false);
      }
      if (chosen != nullptr) solver.solve(*chosen);
    }

    const bool has_detection = (chosen != nullptr);
    // 短暂丢失：还在保持时间内就继续用滤波器外推，不松开云台
    const bool holding =
      !has_detection && has_target &&
      (now - last_seen) < std::chrono::milliseconds(target_hold_ms);

    if (has_detection) {
      auto & armor = *chosen;
      if (!has_target || target.name != armor.name) {
        // 第一次出现，或者换了一台车：重建滤波器
        target = auto_aim::Target(armor, t, 0.0, 0);
        has_target = true;
        frames_since_init = 0;
      } else {
        // 先预测到当前时刻，再用观测修正。
        // 观测的 NIS 超过门限就拒收，避免单帧误关联把角速度曲线拽出尖峰。
        target.predict(t);
        if (!target.update_checked(armor, nis_reject_threshold)) {
          tools::logger()->debug("[task2] 这一帧观测被 NIS 门限拒收");
        }
        frames_since_init++;
      }
      last_seen = now;
    } else if (holding) {
      // 只做预测、不做修正 —— 卡尔曼滤波的 predict 本来就是为「这帧没有观测」准备的
      target.predict(t);
      frames_since_init++;
    }

    // 滤波器发散：有观测就用这一帧重建，没观测只能放弃
    if (has_target && target.diverged()) {
      if (has_detection) {
        target = auto_aim::Target(*chosen, t, 0.0, 0);
        frames_since_init = 0;
      } else {
        has_target = false;
      }
    }

    if (!has_target) {
      // 真的没有目标：交出控制权
      gimbal.send(false, false, 0.0f, 0.0f);
    } else {
      const Eigen::VectorXd x = target.ekf_x();
      // 记录这次拟合出来的角速度落在哪个转速挡位，供下一帧判断挡位是否切换
      speed_band = (std::abs(x[7]) >= speed_band_split) ? 1 : 0;
      const auto xyza_list = target.armor_xyza_list();
      int aim_id = target.last_id;
      if (aim_id < 0 || aim_id >= static_cast<int>(xyza_list.size())) aim_id = 0;
      const Eigen::Vector4d plate = xyza_list[aim_id];

      // 弹速：电控不可用时用配置里的兜底值，并且只告警一次
      double bullet_speed = gimbal_state.bullet_speed;
      if (bullet_speed < 5.0) {
        static bool warned = false;
        if (!warned) {
          tools::logger()->warn(
            "[task2] 电控弹速 {} 不可用，改用配置兜底值 {} 米每秒", gimbal_state.bullet_speed,
            default_bullet_speed);
          warned = true;
        }
        bullet_speed = default_bullet_speed;
      }

      // 提前量 + 弹道解算（重力与空气阻力都在 tools::Trajectory 里）
      float aim_yaw = 0.0f;
      float aim_pitch = 0.0f;
      task_common::calc_aim(x, plate, bullet_speed, muzzle_offset_x, aim_yaw, aim_pitch);

      // ---- 开火判定 ----
      // 原来的 is_converged() 展开就是「几何没跑飞」，第二帧就为真，等于一直在请求开火。
      // 这里换成真正的判据：几何 + 跑够帧数 + NIS 失败率低，
      // 再叠上弹频节流与每个转速挡位的发弹上限。
      const double nis_failure_rate = target.ekf().data.at("recent_nis_failures");
      const bool stable = target.is_converged() && frames_since_init >= min_frames_for_fire &&
                          nis_failure_rate < max_nis_failure_rate;
      const bool interval_ok = tools::delta_time(now, last_fire_time) >= fire_interval_s;
      const bool rounds_ok =
        (max_rounds_per_stage <= 0) || (rounds_this_stage < max_rounds_per_stage);
      const bool mode_ok = !require_auto_aim_mode || gimbal.mode() == io::GimbalMode::AUTO_AIM;
      // 只有这一帧真的有观测才开火：靠外推硬打出去多半白费弹药
      const bool fire = stable && interval_ok && rounds_ok && mode_ok && !holding;

      if (fire) {
        last_fire_time = now;
        last_round_time = now;
        band_at_last_round = speed_band;
        // 电控没给发弹数时，用自己发出的开火命令计数
        if (!bullet_count_alive) rounds_this_stage++;
      }

      // 绝对角度控制；mode_ok 为假时交出控制权，避免和人手操作抢云台
      gimbal.send(mode_ok, fire, aim_yaw, aim_pitch, task_common::to_target_id(target.name));

      // ---- 画面 ----
      if (has_detection) {
        tools::draw_points(img, chosen->points, {0, 255, 0});
      }
      tools::draw_text(
        img,
        fmt::format(
          "{} yaw {:.2f} pitch {:.2f} dist {:.2f} w {:.2f} rounds {}/{}", holding ? "HOLD" : "AIM",
          aim_yaw, aim_pitch, plate.head(3).norm(), x[7], rounds_this_stage,
          max_rounds_per_stage),
        cv::Point(10, 70), cv::Scalar(0, 255, 255), 1.0, 2);

      // ---- 发给 PlotJuggler 的曲线 ----
      // task2_aim：发给云台的控制命令；task2_ekf：EKF 拟合出来的角速度与健康度。
      // 角速度那条是评分直接看的，残差与 NIS 用来判断滤波器是不是真的收敛了。
      plotter.plot({
        {"task2_aim",
         {{"yaw", aim_yaw}, {"pitch", aim_pitch}, {"distance", plate.head(3).norm()}}},
        {"task2_ekf",
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

    cv::imshow("task2", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }

    // Your code end
  }

  // 退出前主动交出控制权
  gimbal.send(false, false, 0.0f, 0.0f);
  return 0;
}
