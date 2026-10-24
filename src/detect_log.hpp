#ifndef SRC__DETECT_LOG_HPP
#define SRC__DETECT_LOG_HPP

// 把「这一帧看到哪几块装甲板」单独记一份 CSV。
//
// 为什么需要它：录包（frames.csv + jpg）里只有云台姿态和弹速，没有检测结果。
// 没有检测结果，离线就只能看云台转多快，没法重放「关联 → 滤波 → 瞄准」这条链，
// 也就没法在不占车、不发弹的情况下调 EKF 的参数。
//
// 打开方式（不设环境变量就完全不写文件，也不影响比赛行为）：
//     DSH_DET_CSV=detect.csv ./build/task_2 configs/standard.yaml
//
// 每块板一行，没检测到任何板时也写一行（count=0），这样帧号不会有空洞：
//     frame,t_sec,count,idx,name,color,conf,u0,v0,u1,v1,u2,v2,u3,v3,cnx,cny,cx,cy
//
// 注意：t_sec 以本对象构造时刻为起点，和录包 frames.csv 的 t_sec 可能差不到一帧，
// 对齐时优先用 frame 列（它和 yolo.detect 收到的帧号是同一个）。

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tools/logger.hpp"

namespace detect_log
{

class DetectLog
{
public:
  DetectLog()
  {
    const char * path = std::getenv("DSH_DET_CSV");
    if (path == nullptr || path[0] == '\0') return;

    file_ = std::fopen(path, "w");
    if (file_ == nullptr) {
      tools::logger()->warn("[detlog] 打不开 {}，检测日志不会记录", path);
      return;
    }
    std::fprintf(
      file_,
      "frame,t_sec,count,idx,name,color,conf,u0,v0,u1,v1,u2,v2,u3,v3,cnx,cny,cx,cy\n");
    std::fflush(file_);
    tools::logger()->info("[detlog] 检测日志 -> {}（每块板一行，没检测到也写一行）", path);
  }

  ~DetectLog()
  {
    if (file_ != nullptr) std::fclose(file_);
  }

  DetectLog(const DetectLog &) = delete;
  DetectLog & operator=(const DetectLog &) = delete;

  bool enabled() const { return file_ != nullptr; }

  // 用模板而不是写死 std::vector：框架里 yolo.detect 返回的是 std::list<Armor>，
  // 写死容器类型会在编译期炸掉，而这里根本不需要随机访问。
  template <typename ArmorContainer>
  void on_frame(long frame, double t_sec, const ArmorContainer & armors)
  {
    if (file_ == nullptr) return;

    if (armors.empty()) {
      std::fprintf(file_, "%ld,%.6f,0\n", frame, t_sec);
    } else {
      int idx = 0;
      for (const auto & a : armors) {
        const char * name = "?";
        const char * color = "?";
        const int ni = static_cast<int>(a.name);
        const int ci = static_cast<int>(a.color);
        if (ni >= 0 && ni < static_cast<int>(auto_aim::ARMOR_NAMES.size()))
          name = auto_aim::ARMOR_NAMES[ni].c_str();
        if (ci >= 0 && ci < static_cast<int>(auto_aim::COLORS.size()))
          color = auto_aim::COLORS[ci].c_str();

        // 四个关键点（左上、左下、右下、右上），少一个就补 0，宁可脏也别丢整行
        double u[4] = {0, 0, 0, 0}, v[4] = {0, 0, 0, 0};
        for (std::size_t i = 0; i < 4 && i < a.points.size(); ++i) {
          u[i] = a.points[i].x;
          v[i] = a.points[i].y;
        }

        std::fprintf(
          file_, "%ld,%.6f,%zu,%d,%s,%s,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,"
                 "%.2f,%.2f\n",
          frame, t_sec, armors.size(), idx, name, color, a.confidence, u[0], v[0], u[1], v[1], u[2],
          v[2], u[3], v[3], a.center_norm.x, a.center_norm.y, a.center.x, a.center.y);
        ++idx;
      }
    }

    if (++since_flush_ >= 32) {
      std::fflush(file_);
      since_flush_ = 0;
    }
  }

private:
  std::FILE * file_ = nullptr;
  int since_flush_ = 0;
};

}  // namespace detect_log

#endif  // SRC__DETECT_LOG_HPP
