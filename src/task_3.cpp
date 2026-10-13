#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/target.hpp"
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

  // 跨帧保存的滤波器，第一次看到前哨站时再初始化
  auto_aim::Target target;
  bool has_target = false;

  while (!exiter.exit()) {
    // Your code start

    camera.read(img, t);
    if (img.empty()) {
      continue;  // 空帧直接跳过，不发送指令
    }

    // 用曝光时刻的云台姿态，把相机坐标系转到世界坐标系
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img, frame_count++);

    // 前哨站考核只打前哨站，滤掉其它装甲板
    auto_aim::Armor * outpost = nullptr;
    for (auto & armor : armors) {
      if (armor.name == auto_aim::outpost) {
        outpost = &armor;
        break;
      }
    }

    if (outpost == nullptr) {
      gimbal.send(false, false, 0.0f, 0.0f);
      has_target = false;
    } else {
      auto & armor = *outpost;
      solver.solve(armor);

      if (!has_target || target.name != armor.name) {
        // 第一次出现前哨站，建 3 块板的滤波器
        target = auto_aim::Target(armor, t, 0.0, 0);
        has_target = true;
      } else {
        target.predict(t);
        target.update(armor);
      }

      // 滤波器发散就丢掉重来
      if (target.diverged()) {
        target = auto_aim::Target(armor, t, 0.0, 0);
      }

      auto xyza_list = target.armor_xyza_list();
      int aim_id = target.last_id;
      if (aim_id < 0 || aim_id >= static_cast<int>(xyza_list.size())) {
        aim_id = 0;
      }
      Eigen::Vector4d plate = xyza_list[aim_id];

      // 弹丸飞行时间 = 距离 / 弹速
      double bullet_speed = gimbal.state().bullet_speed;
      if (bullet_speed < 5.0) {
        bullet_speed = 20.0;
      }
      double t_flight = plate.head(3).norm() / bullet_speed;

      // 按拟合出的角速度，把目标板转 t_flight 后的位置作为提前量
      Eigen::VectorXd x = target.ekf_x();
      double da = x[7] * t_flight;
      double ca = std::cos(da), sa = std::sin(da);
      double rx = plate[0] - x[0];
      double ry = plate[1] - x[2];
      Eigen::Vector3d aim_xyz{
        x[0] + ca * rx - sa * ry + x[1] * t_flight,
        x[2] + sa * rx + ca * ry + x[3] * t_flight,
        plate[2] + x[5] * t_flight};
      Eigen::Vector3d aim = tools::xyz2ypd(aim_xyz);

      // 控制云台，收敛后才开火
      gimbal.send(
        true, target.is_converged(), static_cast<float>(aim[0]), static_cast<float>(aim[1]));

      tools::draw_points(img, armor.points, {0, 255, 0});

      // 按考核要求输出控制命令与 EKF 拟合角速度
      plotter.plot({
        {"aim",
         {{"yaw", aim[0]},
          {"pitch", aim[1]},
          {"w", x[7]},
          {"distance", aim[2]}}}
      });
    }

    cv::imshow("task3", img);
    if (cv::waitKey(1) == 'q') {
      break;
    }

    // Your code end
  }

  // 退出前主动交出控制权
  gimbal.send(false, false, 0.0f, 0.0f);
  return 0;
}