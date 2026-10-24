#ifndef SRC__LATENCY_PROBE_HPP
#define SRC__LATENCY_PROBE_HPP

// 现场测量用探针：把「曝光 → 取流 → 推理/解算 → 发串口」这条链上能测的时刻全记下来，
// 顺便能把云台的「命令 vs 反馈」按帧存下来做阶跃响应。
//
// 不设环境变量时它什么都不做（每帧两次 if），对跑车没有任何影响。
//
//   DSH_LAT_CSV=logs/latency.csv ./build/task_2 configs/standard.yaml
//
// 输出一帧一行，时刻都是 steady_clock 的纳秒数：
//   dev_ts,grab_ns,infer_ns,send_ns,cmd_yaw,cmd_pitch,fb_yaw,fb_pitch,fire,dist
// 最后一列 dist 是解算出来的目标距离（米），拿来反解实际弹速：距离 + 打出去的 pitch +
// 落点偏差，三样凑齐就能算出初速，不需要测速仪。
//
// dev_ts 是相机自己的曝光时间戳（MV_FRAME_OUT_INFO_EX 的 nDevTimeStampHigh/Low 拼成的 64 位 tick），
// 单位随型号变，所以这里不猜它：离线用 grab_ns = a * dev_ts + b 拟合出 a 再换算成秒。
// 注意 b 里同时含「相机时钟与主机时钟的固定差」和「曝光完到我们拿到帧的固定传输延迟」，
// 这两项靠时间戳本身分不开（除非两台设备对过时），所以只能测出：
//   ① 曝光抖动的变化量、② 取流之后的全部耗时、③ 队列有没有积压（残差随时间漂移）。
// 想要绝对量，得拿手机毫秒表对着镜头拍一帧，记屏幕时间和 grab_ns 的差。
//
// 阶跃测试（可选）：
//   DSH_STEP_DEG=2 DSH_LAT_CSV=logs/step.csv ./build/task_2 configs/standard.yaml
// 让 yaw 指令以 1 Hz 方波叠加 ±2 度，并强制不开火。cmd 是方波、fb 是指数爬升，
// 两条曲线一叠就能读出云台的到位时间、超调和稳态误差。
// 注意它是在「对着目标」的时候叠加的，所以视野里得有一个靶子，否则程序会把控制权交出去，
// 什么都不发。测的时候让靶子静止、云台稳在它上面，再看 yaw 那一路的方波响应。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>

#include "tools/logger.hpp"

namespace probe
{

class Latency
{
public:
  Latency()
  {
    const char * csv = std::getenv("DSH_LAT_CSV");
    if (csv != nullptr && csv[0] != '\0') {
      file_.open(csv, std::ios::out | std::ios::trunc);
      if (!file_.is_open()) {
        tools::logger()->warn("[lat] 打不开 {}，本次不做延迟测量", csv);
      } else {
        file_ << "dev_ts,grab_ns,infer_ns,send_ns,cmd_yaw,cmd_pitch,fb_yaw,fb_pitch,fire,dist\n";
        enabled_ = true;
        tools::logger()->info("[lat] 延迟测量已开启，写入 {}", csv);
      }
    }

    const char * step = std::getenv("DSH_STEP_DEG");
    if (step != nullptr) {
      const double deg = std::atof(step);
      if (deg > 0.0 && deg <= 15.0) {
        step_deg_ = deg;
        tools::logger()->warn(
          "[lat] 阶跃测试已开启：yaw 指令按 1 Hz 方波叠加 ±{:.2f} 度（需视野里有靶子），"
          "期间强制不开火",
          step_deg_);
      } else if (deg > 0.0) {
        // 别把云台拧到机械限位
        tools::logger()->warn("[lat] DSH_STEP_DEG={} 超过 ±15 度，已忽略", deg);
      }
    }
  }

  bool enabled() const { return enabled_; }

  // 取到一帧：趁早记，t 就是相机给的时间戳（取流完成时刻），dev_ts 是曝光时间戳
  void on_grab(std::chrono::steady_clock::time_point t, uint64_t dev_ts)
  {
    if (!enabled_) return;
    grab_ = t;
    infer_ = t;  // 万一这一帧没走到推理，两者相等，统计时不会出现负值
    dev_ts_ = dev_ts;
    has_grab_ = true;
  }

  // 推理 + 解算完（这一帧从此不再有任何计算开销）
  void on_infer(std::chrono::steady_clock::time_point t)
  {
    if (!enabled_ || !has_grab_) return;
    infer_ = t;
  }

  // 串口写完：cmd 是我们发出去的角度，fb 是电控这一帧上报的姿态
  // dist 是这一帧解算出来的目标距离（米），用来事后反解弹速：距离 + 实际打的 pitch + 落点偏差
  // 三者一组，就能算出真实的初速，不用拆枪。
  void on_send(
    std::chrono::steady_clock::time_point t, float cmd_yaw, float cmd_pitch, float fb_yaw,
    float fb_pitch, bool fire, float dist = -1.0f)
  {
    if (!enabled_ || !has_grab_) return;
    has_grab_ = false;

    file_ << dev_ts_ << ',' << ns(grab_) << ',' << ns(infer_) << ',' << ns(t) << ',' << cmd_yaw
          << ',' << cmd_pitch << ',' << fb_yaw << ',' << fb_pitch << ',' << (fire ? 1 : 0) << ','
          << dist << '\n';

    if (++rows_ % 64 == 0) file_.flush();
  }

  // ---- 阶跃测试 ----
  bool step_enabled() const { return step_deg_ > 0.0; }

  // 1 Hz 方波：前半周期 +deg，后半周期 -deg。返回值已经换成弧度。
  float step_delta(std::chrono::steady_clock::time_point t) const
  {
    const double s = std::chrono::duration<double>(t - t0_).count();
    const double rad = step_deg_ * 0.017453292519943295;
    return static_cast<float>(std::fmod(s, 1.0) < 0.5 ? rad : -rad);
  }

private:
  static int64_t ns(std::chrono::steady_clock::time_point t)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count();
  }

  bool enabled_ = false;
  bool has_grab_ = false;
  uint64_t dev_ts_ = 0;
  std::chrono::steady_clock::time_point grab_{};
  std::chrono::steady_clock::time_point infer_{};
  std::chrono::steady_clock::time_point t0_ = std::chrono::steady_clock::now();
  double step_deg_ = 0.0;
  int rows_ = 0;
  std::ofstream file_;
};

}  // namespace probe

#endif  // SRC__LATENCY_PROBE_HPP
