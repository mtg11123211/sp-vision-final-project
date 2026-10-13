#include <chrono>
#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

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

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;
  int frame_count = 0;

  // 目标短暂丢失时，用最后一次看到的角度保持一段时间
  bool has_last_aim = false;
  float last_yaw = 0.0f;
  float last_pitch = 0.0f;
  std::chrono::steady_clock::time_point last_target_time;

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
    auto now = std::chrono::steady_clock::now();

    if (armors.empty()) {
      // 没看到目标：300毫秒内保持最后角度，超过才交出控制权
      if (has_last_aim && now - last_target_time < 300ms) {
        gimbal.send(true, false, last_yaw, last_pitch);
      } else {
        gimbal.send(false, false, 0.0f, 0.0f);
        has_last_aim = false;
      }
    } else {
      // 考核现场只有一块装甲板，取第一个
      auto & armor = armors.front();
      solver.solve(armor);

      last_yaw = static_cast<float>(armor.ypd_in_world[0]);
      last_pitch = static_cast<float>(armor.ypd_in_world[1]);
      last_target_time = now;
      has_last_aim = true;

      // 绝对角度控制：控制=true，开火=false
      gimbal.send(true, false, last_yaw, last_pitch);

      tools::draw_points(img, armor.points, {0, 255, 0});
      tools::draw_text(
        img,
        fmt::format(
          "yaw {:.2f} | pitch {:.2f} | dist {:.2f} m",
          armor.ypd_in_world[0], armor.ypd_in_world[1], armor.ypd_in_world[2]),
        cv::Point(10, 70), cv::Scalar(0, 255, 255), 1.4, 2);

      // 发给 PlotJuggler 的三条实时曲线
      plotter.plot({
        {"aim",
         {{"yaw", last_yaw},
          {"pitch", last_pitch},
          {"distance", armor.ypd_in_world[2]}}}
      });
    }

    cv::imshow("task1", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }

    // Your code end
  }

  // 退出前主动交出控制权
  gimbal.send(false, false, 0.0f, 0.0f);
  return 0;
}
