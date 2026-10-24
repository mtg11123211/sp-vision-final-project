#ifndef SRC__OVERLAY_HPP
#define SRC__OVERLAY_HPP

// 画在调试窗口上的东西，以及「我到底有没有在画」的证据。
//
// 背景：原来的窗口只在 has_target 的时候画几个点，一旦没有目标画面就是一张
// 不动的照片，从外面完全看不出「是没目标，还是程序卡死了 / 窗口冻住了」。
// 这个 Hud 每帧都画，并且带三样会变的东西：
//   1. 帧号 f=...       —— 每帧加 1，窗口冻住立刻能看出来
//   2. fps              —— 掉到 0 说明主循环停了
//   3. 闪烁的小方块      —— 每 10 帧换一次颜色，静态画面上唯一会动的东西
//
// 另外它把所有检测到的装甲板画成四边形（原来只画四个点，看不出哪四点是一块板），
// 并把选中的那块用瞄准线连到画面中心，这就是「画线」本身；HUD 上 aimline=1
// 表示这一帧确实画了那条线。

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/armor.hpp"

namespace overlay
{

inline cv::Scalar color_of(auto_aim::Color c)
{
  switch (c) {
    case auto_aim::red:
      return {0, 0, 255};
    case auto_aim::blue:
      return {255, 0, 0};
    case auto_aim::extinguish:
      return {150, 150, 150};
    case auto_aim::purple:
      return {255, 0, 255};
    default:
      return {200, 200, 200};
  }
}

class Hud
{
public:
  // 模板参数是容器而不是写死类型：yolo.detect 返回 std::list<Armor>，
  // 而离线工具里可能用 std::vector，这里只遍历、不下标，两种都能吃。
  template <typename ArmorContainer>
  void draw(
    cv::Mat & img, const ArmorContainer & armors, const auto_aim::Armor * chosen, long frame,
    bool has_target, bool holding)
  {
    if (img.empty() || img.type() != CV_8UC3) return;
    tick();

    // ---- 所有检测到的装甲板：四点连成四边形 ----
    // 逐条边画而不是用 cv::polylines：后者把 vector<Point> 当成「多条曲线」，
    // 会按点的个数去取曲线数量，轻则画不出来重则越界。
    int drawn = 0;
    for (const auto & a : armors) {
      if (a.points.size() != 4) continue;
      std::vector<cv::Point> poly;
      poly.reserve(4);
      for (const auto & p : a.points) poly.emplace_back(cvRound(p.x), cvRound(p.y));
      const cv::Scalar c = color_of(a.color);
      for (int i = 0; i < 4; ++i) {
        cv::line(img, poly[i], poly[(i + 1) % 4], c, 2, cv::LINE_AA);
      }
      ++drawn;
    }

    // ---- 准星 + 瞄准线 ----
    const cv::Point center(img.cols / 2, img.rows / 2);
    cv::line(img, center - cv::Point(16, 0), center + cv::Point(16, 0), {0, 255, 255}, 1);
    cv::line(img, center - cv::Point(0, 16), center + cv::Point(0, 16), {0, 255, 255}, 1);
    cv::circle(img, center, 20, {0, 255, 255}, 1, cv::LINE_AA);

    int aim_line = 0;
    if (chosen != nullptr) {
      const cv::Point c(cvRound(chosen->center.x), cvRound(chosen->center.y));
      cv::line(img, center, c, {0, 165, 255}, 2, cv::LINE_AA);
      cv::circle(img, c, 8, {0, 255, 255}, 2, cv::LINE_AA);
      const std::string tag = cv::format(
        "%s %.2f",
        (static_cast<int>(chosen->name) >= 0 &&
         static_cast<int>(chosen->name) < static_cast<int>(auto_aim::ARMOR_NAMES.size()))
          ? auto_aim::ARMOR_NAMES[static_cast<int>(chosen->name)].c_str()
          : "?",
        chosen->confidence);
      put_text(img, tag, cv::Point(c.x + 10, c.y - 10), {0, 255, 255}, 0.55);
      ++aim_line;
    }

    // ---- 左下角状态条 ----
    // 放左下而不是左上：task_2 原来那句 "AIM yaw ... w ... rounds ..." 就画在
    // (10, 70)，左上角会和它叠在一起。
    const int pad = 8;
    const int box_w = 560;
    const int box_h = 70;
    const int top = img.rows - pad - box_h;
    cv::rectangle(img, cv::Rect(pad, top, box_w, box_h), {0, 0, 0}, cv::FILLED);

    // 闪烁方块：静态画面上唯一会变的东西，10 帧换一次颜色
    const bool blink_on = ((frame / 10) % 2) == 0;
    cv::rectangle(
      img, cv::Rect(pad + 10, top + 10, 22, 22),
      blink_on ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), cv::FILLED);

    put_text(
      img, cv::format("OVERLAY RUNNING   f=%06ld   %.1f fps", frame, fps_),
      cv::Point(pad + 42, top + 28), {0, 255, 0}, 0.58);
    put_text(
      img,
      cv::format(
        "det %d   drawn %d   tgt %s   hold %s   aimline %d", static_cast<int>(armors.size()), drawn,
        has_target ? "YES" : "NO", holding ? "YES" : "NO", aim_line),
      cv::Point(pad + 14, top + 56), {0, 255, 255}, 0.55);

    // ---- 右下角：累计画了多少帧 ----
    ++frames_drawn_;
    const std::string foot = cv::format("frames drawn %ld", frames_drawn_);
    const cv::Size ts = cv::getTextSize(foot, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, nullptr);
    const cv::Point org(img.cols - ts.width - 16, img.rows - 16);
    cv::rectangle(
      img, cv::Rect(org.x - 8, org.y - ts.height - 6, ts.width + 16, ts.height + 14), {0, 0, 0},
      cv::FILLED);
    put_text(img, foot, org, {200, 200, 200}, 0.55);
  }

private:
  void tick()
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_.time_since_epoch().count() != 0) {
      const double dt = std::chrono::duration<double>(now - last_).count();
      if (dt > 1e-6) {
        const double inst = 1.0 / dt;
        fps_ = (fps_ <= 0.0) ? inst : (0.9 * fps_ + 0.1 * inst);
      }
    }
    last_ = now;
  }

  static void put_text(
    cv::Mat & img, const std::string & s, const cv::Point & p, const cv::Scalar & color, double scale)
  {
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, {0, 0, 0}, 3, cv::LINE_AA);
    cv::putText(img, s, p, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA);
  }

  std::chrono::steady_clock::time_point last_{};
  double fps_ = 0.0;
  long frames_drawn_ = 0;
};

}  // namespace overlay

#endif  // SRC__OVERLAY_HPP
