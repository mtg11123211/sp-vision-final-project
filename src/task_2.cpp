#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include <opencv2/opencv.hpp>

#include "fmt/core.h"
#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "src/detect_log.hpp"
#include "src/env_util.hpp"
#include "src/fire_key.hpp"
#include "src/latency_probe.hpp"
#include "src/low_plate.hpp"
#include "src/overlay.hpp"
#include "src/task_common.hpp"
#include "src/test_summary.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/target.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/yaml.hpp"
#include <cstdlib>

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

// ---- 开火门徽章（右上角大色块，远距离可见）----
// 放在 task_2.cpp 内部而不是 fire_key.hpp：后者保持零工程依赖，keytest 才能单独编译。
// 注意：屏幕上的文字**必须用 ASCII** —— cv::putText 的 Hershey 字体不支持中文，
// 写中文会变成一串乱码（这坑我踩过一次）。日志里可以随便用中文。
namespace
{
// 右上角的开火门徽章。**屏幕文字一律 ASCII**（cv::putText 的 Hershey 字体不支持中文）。
// 第一行 = 开火门状态（门关 / 后端异常 / ARM / SAFE）；
// 第二行 = 现在**为什么打不出**（READY = 其它条件都满足、随时能打）。
void draw_fire_badge(
  cv::Mat & img, bool enabled, bool allowed, bool degraded, const std::string & backend,
  const std::string & state)
{
  const int w = 240, h = 74;
  const cv::Rect box(img.cols - w - 12, 12, w, h);
  cv::Scalar bg(90, 90, 90), fg(255, 255, 255);
  std::string text;
  if (!enabled) {
    bg = {90, 90, 90};
    text = "GATE OFF (auto fire)";
  } else if (degraded) {
    bg = {0, 200, 255};
    fg = {0, 0, 0};
    text = "BACKEND ERR " + backend;
  } else if (allowed) {
    bg = {0, 190, 0};
    text = "ARM  FIRE ALLOWED";
  } else {
    bg = {0, 0, 235};
    text = "SAFE NO FIRE";
  }
  cv::rectangle(img, box, bg, cv::FILLED);
  cv::putText(
    img, text, cv::Point(box.x + 8, box.y + 32), cv::FONT_HERSHEY_SIMPLEX, 0.62, fg, 2,
    cv::LINE_AA);
  // 副标题：READY 说明现在就差"按下 F"这一步
  cv::putText(
    img, state, cv::Point(box.x + 8, box.y + 62), cv::FONT_HERSHEY_SIMPLEX, 0.55, fg, 1,
    cv::LINE_AA);
}
}  // namespace

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

  // 光轴/机械零点补偿（单位：度）。落点偏右就调大 yaw_offset，偏下就调大 pitch_offset。
  // 现场回滚通道：DSH_YAW_OFFSET / DSH_PITCH_OFFSET（单位也是度）一旦设置就覆盖 yaml。
  // 把它们设成 0 就等价于「这两个参数还没接进代码」的未启用状态——不改文件、不重编、只重启进程。
  const double yaw_offset_deg = tools::read_or<double>(yaml, "yaw_offset", 0.0);
  const double pitch_offset_deg = tools::read_or<double>(yaml, "pitch_offset", 0.0);
  static const char * env_yaw_offset = std::getenv("DSH_YAW_OFFSET");
  static const char * env_pitch_offset = std::getenv("DSH_PITCH_OFFSET");
  const double yaw_offset_eff = env_yaw_offset ? std::atof(env_yaw_offset) : yaw_offset_deg;
  const double pitch_offset_eff = env_pitch_offset ? std::atof(env_pitch_offset) : pitch_offset_deg;
  tools::logger()->info(
    "[task2] offset yaw={:.2f}({}) pitch={:.2f}({})  源: yaml 值 {:.2f}/{:.2f}",
    yaw_offset_eff, env_yaw_offset ? "env DSH_YAW_OFFSET" : "yaml", pitch_offset_eff,
    env_pitch_offset ? "env DSH_PITCH_OFFSET" : "yaml", yaw_offset_deg, pitch_offset_deg);

  // ---------------- 云台到位门控 ----------------
  // 开火前确认炮管真的转到了要求的角度。指令和 fire 是塞在同一个串口包里发出去的
  // （见后面 gimbal.send），云台从收到角度转到位要几十到上百毫秒，这段时间里打出去的
  // 弹丸是沿「旧角度」飞的。3 米处落后 1° ≈ 52 毫米，已经接近半块小装甲板（67.5 毫米）。
  //
  //   DSH_ARRIVE_DEG 不设（或 <=0）→ 门控完全不生效，行为与之前完全一样（默认）。
  //   DSH_ARRIVE_DEG=0.5          → yaw、pitch 都在 0.5° 以内才允许开火。
  //   DSH_ARRIVE_PITCH_DEG=2.0    → pitch 单独放宽（云台俯仰轴慢、且有 10° 的零点补偿）
  //
  // 符号约定（两边不一样，别写错了）：
  //   yaw  ：io/gimbal/gimbal.cpp:106 发出去是同号，:270 回传也原样，所以比「相减」。
  //   pitch：gimbal.cpp:111 发出去取反（任务一现场实测：板子上移、云台下移，取反是对的），
  //          而 gimbal.cpp:272 回传是原样拷贝 rx_data_.pitch、没有再翻回来，
  //          所以到位时满足的是 aim_pitch + fb_pitch ≈ 0，比「相加」。
  //          这一条是读代码推出来的，第一次上车看 [dbg] arrive 那行的 fb/cmd 就能确认。
  //
  // 自动解除机制默认关闭（DSH_ARRIVE_WATCHDOG_SHOTS 缺省 0 = 永不解除）。
  //
  // 原来默认 12 发，理由是「宁可打偏、不能一枪不发」。现场跑下来这个理由站不住：
  // 任务二每个转速挡位只有 10 发额度，而看门狗在第 12 发被挡之后就永久解除门控 ——
  // 也就是说门控最多只在第一个挡位起作用，后面全程失效，等于白开。
  // 更糟的是它是「静默失效」：日志里只有一行 warn，人盯着屏幕时看不出门控已经没了。
  //
  // 现在改成：不显式给正数就永不解脱，门控一旦打开就一直有效。想临时回到旧行为，
  // 显式给 DSH_ARRIVE_WATCHDOG_SHOTS=12 即可（只在调试「门限是不是太紧」时用）。
  // 计数仍然保留：它记录「本来真会打出去、却被门控挡掉」的弹数，是判断门限松紧的
  // 唯一依据（用发数而不是时长计数，是因为目标时隐时现会把计时反复清零）。
  // 默认就开（0.8°）。显式给 DSH_ARRIVE_DEG=0 才是关闭。
  // 0.8 = 1.6 × 4 rad/s 下的稳态跟随滞后（≈0.45°），只拦「明显没跟上」的瞬间；
  // 卡到 0.5 以下会把正常的跟踪滞后也挡掉，一发都打不出去。
  const double arrive_tol_deg = []() {
    const char * v = std::getenv("DSH_ARRIVE_DEG");
    if (v == nullptr || v[0] == '\0') return 0.8;
    return std::atof(v);
  }();
  // pitch 单独的门限：不设就跟 yaw 一样放宽 1.6 倍（0.5° → 0.8°）。
  // pitch 轴拖着零点补偿去追，常常比 yaw 慢，卡同一个数会让门控频繁误挡。
  const double arrive_tol_pitch_deg = [&]() {
    const char * v = std::getenv("DSH_ARRIVE_PITCH_DEG");
    if (v == nullptr || v[0] == '\0') return arrive_tol_deg * 1.6;
    return std::atof(v);
  }();
  // 连续多少帧落在门限内才放行（3 帧 ≈ 50 ms @60 fps）。
  // 只卡单帧的话，云台扫过 0.5° 那一瞬间就会被判到位，等于没卡。
  const int arrive_frames_needed = []() {
    const char * v = std::getenv("DSH_ARRIVE_FRAMES");
    if (v == nullptr || v[0] == '\0') return 3;
    const int n = std::atoi(v);
    return n > 0 ? n : 1;
  }();

  // ---- 板面朝向门控 ----
  // 旋转靶一圈四块板，只有板面大致正对镜头那一下才打得中：别的角度打出去，
  // 弹道要么擦边、要么落在两块板中间的空当里，看起来就是「明明瞄了却没打中」。
  // 判定用的角度：板的外法线方向与「目标中心指向相机」的方向之差，0° = 正对。
  // 板位置 = 中心 − r(cos a, sin a)（target.cpp:404-405），所以外法线就是 a + π。
  const double plate_facing_deg = []() {
    const char * v = std::getenv("DSH_PLATE_FACING_DEG");
    if (v == nullptr || v[0] == '\0') return 25.0;  // 四块板 90° 一块，正对窗口约 50°
    return std::atof(v);
  }();
  // 「一块板只打一发」：打完这一发后，同一块板必须摆出这个角度之外（或换成别的板），
  // 才重新允许开火。比 facing 大 10° 左右，避免在临界上来回抖着放行。
  const double plate_rearm_deg = []() {
    const char * v = std::getenv("DSH_PLATE_REARM_DEG");
    if (v == nullptr || v[0] == '\0') return 35.0;
    return std::atof(v);
  }();
  // 缺省 0 = 永不自动解脱。给正数才启用「挡满 N 发就解除门控」的旧行为。
  const int arrive_watchdog_shots = []() {
    const char * v = std::getenv("DSH_ARRIVE_WATCHDOG_SHOTS");
    if (v == nullptr || v[0] == '\0') return 0;
    const int n = std::atoi(v);
    return n > 0 ? n : 0;
  }();
  // 一行把「这次到底允许怎么开火」全打出来。参数改错时，看这一行比看代码快。
  tools::logger()->info(
    "[task2] 到位门控 = {}（yaw {:.2f}° / pitch {:.2f}°，连续 {} 帧）｜看门狗 {}｜"
    "板面正对 ±{:.0f}°、摆出 {:.0f}° 才重新上膛（一块板只打一发）｜发弹间隔 {:.2f} s",
    arrive_tol_deg > 0.0 ? "开" : "关", arrive_tol_deg, arrive_tol_pitch_deg,
    arrive_frames_needed,
    arrive_watchdog_shots > 0 ? std::to_string(arrive_watchdog_shots) + " 发后解除" : "关",
    plate_facing_deg, plate_rearm_deg, fire_interval_s);

  // ================= 新增策略 1：只打低位靶，不打高位靶（三态）=================
  // 口径修正（2026-10-21 评审）：几何列只在**首次换板前**冻结，换板后是允许单板观测更新
  // 高度差的 ⇒ x[10] 是"从 0 起步、靠换板观测慢慢收敛"，**收敛前**高板 z 偏低。
  // 所以"永久只打低位"是把瞬态问题当永久问题 ⇒ 改成三态：
  //   0/off 完全不参与｜1/warmup【默认】不可信时只打低位、可信后自动放开｜2/always 永远只打低位
  std::string low_mode_raw = "";
  const policy::LowMode low_mode = policy::parse_low_mode(std::getenv("DSH_LOW_PLATE"), &low_mode_raw);
  const double low_plate_z_tol = policy::env_double("DSH_LOW_PLATE_Z_TOL", 0.10);
  // 高度差"稳定保持"多久就认为已收敛（warmup 放开高板的门槛）
  const double low_trust_s = policy::env_double("DSH_LOW_PLATE_TRUST_S", 4.0);
  policy::LowPlateGate low_gate(low_mode, low_plate_z_tol, low_trust_s);

  // ================= 锁板（守株待兔，默认关：DSH_LOCK_MODE=1 才开）=================
  // 看见一块（可选高度层的）板就锁住，**中途不换板**，一直等它自己转到
  // 「弹着时正对」（|facing + w*t_arrive| 小于 DSH_PLATE_FACING_DEG）再开火。
  // 为什么要锁：现在每帧重选，四块板在 45° 边界上「到达朝向」打平就来回跳，
  // 每跳一次 5.4°（r=0.2 m、3 m 处的弦角），云台刚加速又被拉回；锁住一块之后
  // 云台只在一条弧线上走，相位门每圈只开一次，节奏干净。
  // 解锁只有三种：打出去了 / 目标重建 / 超过 DSH_LOCK_TIMEOUT（默认 2.5 s，
  // 覆盖 3 rad/s 时 2.1 s 一圈；目标停转或转向反了就靠它兜底）。
  // DSH_PICK_BAND：0=不过滤 1=只锁最低那一层（默认） 2=只锁最高那一层
  // DSH_PICK_Z_TOL：同一层的容差(m)，默认 0.04；四块板同高时这条不产生任何影响。
  const bool lock_mode = policy::env_bool("DSH_LOCK_MODE", false);
  const double lock_timeout_s = policy::env_double("DSH_LOCK_TIMEOUT", 2.5);
  const int pick_band = static_cast<int>(policy::env_double("DSH_PICK_BAND", 1.0));
  const double pick_z_tol = policy::env_double("DSH_PICK_Z_TOL", 0.04);
  // 选板可见性上限（度）：facing 绝对值超过它就不选（180 = 完全不影响）
  const double vis_limit_deg = policy::env_double("DSH_PLATE_VIS_DEG", 180.0);
  tools::logger()->info(
    "[task2] 锁板 = {}（锁定后不换板，超时 {:.1f} s 解锁）｜高度带 = {}（容差 {:.2f} m）｜可见性 ≤{:.0f}°",
    lock_mode ? "开" : "关", lock_timeout_s,
    pick_band == 0 ? "不过滤" : (pick_band == 1 ? "只锁最低层" : "只锁最高层"), pick_z_tol, vis_limit_deg);
  // ================= 伏击模式（DSH_AMBUSH，默认关）=================
  // 炮口钉在「板转正时所在的那个弧点」上，用相位时钟决定发弹：
  //   弹着时刻 = now + B3(出弹) + t_flight，要求此刻板正好转过 0°。
  // 与追踪模式互斥：开伏击时不做选板竞争、不做 lead 补偿、facing 门由相位门替代。
  // 零行为变化：DSH_AMBUSH=0 时下面每一处都走原来的分支。
  const bool ambush_on = policy::env_bool("DSH_AMBUSH", false);
  // 伏击的 B3：DSH_FIRE_DELAY 显式设了就用它，否则 0.080 s（待 M11 标定）
  const double ambush_b3_s = policy::env_double("DSH_FIRE_DELAY", 0.080);
  const bool ambush_b3_given = policy::env_double("DSH_FIRE_DELAY", 0.0) > 0.0;
  tools::logger()->info(
    "[task2] 伏击模式 = {}（B3 = {:.3f} s{}）", ambush_on ? "开" : "关", ambush_b3_s,
    ambush_b3_given ? "" : "，用缺省 80 ms 待标定");

  // ================= 官方 sp_vision_25 的三条思路（默认全关，一项一项 A/B）=================
  // ① 出弹延时（摩擦轮从接到开火指令到子弹出膛，官方注释写 0.1 s）：
  //    **方案①：从 2026-10-21 起，它只作记录/拆分标注，不再单独累加。**
  //    全部"非飞行"的提前量都放进 DSH_LEAD（= 总提前量）⇒ **结构上杜绝"同一段延迟补两次"**
  //    （以前 DSH_LEAD=0.057 + DSH_FIRE_DELAY=0.057 会把 57 ms 补两遍 ⇒ 补过头 0.83°，
  //      还会让人误判成"符号反了"而去翻转符号，越修越糟）。
  //    要改补偿量 ⇒ 只改 DSH_LEAD 一个旋钮；这个数留着是为了把"其中出弹约多少"记进日志。
  const double fire_delay_report_s = policy::env_double("DSH_FIRE_DELAY", 0.0);
  // ② 选板用"弹丸到达时刻"的朝向：官方 aimer.cpp 是先 predict(future + fly_time) 再选板。
  //    这里用等价写法：facing 以 w 变化 ⇒ 到达时朝向 = 现在的朝向 + w * t_arrive。
  const bool aim_select_arrival =
    (std::string(std::getenv("DSH_AIM_SELECT") ? std::getenv("DSH_AIM_SELECT") : "arrival") ==
     "arrival");
  // ③ 到位容限按"线性距离"给：官方 shooter 是 3°(近)/2°(远) ≈ 恒定 105 mm；
  //    本工程原来固定 0.8°（3.15 m 处只有 44 mm），比官方严 2.4 倍 ⇒ 可能永远打不开。
  const double arrive_mm = policy::env_double("DSH_ARRIVE_MM", 0.0);

  // ================= 新增策略 2：人按住 F 才允许开火 =================
  // 程序一直跟踪、一直解算，但只有人认为可以打了才扣扳机。
  // DSH_FIRE_KEY=hold（默认）/toggle/always/never，取键后端见 src/fire_key.hpp。
  // **默认关**（2026-10-21 评审修订 3）：正式提交每任务只有一次机会，
  // 操作员忘按 F = 整场零发；调试期显式开，HUD 徽章会一直显示当前状态。
  const bool fire_key_enabled = policy::env_bool("DSH_FIRE_KEY_ENABLE", false);
  fire_key::Gate fire_key_gate;

  // 总提前量（秒）：DSH_LEAD = 链路 + 云台滞后 + 出弹延时，**方案①下这是唯一的功能旋钮**。
  // 声明提前到选板之前，因为到达时刻 t_arrive 也要用它（到达时刻 = 总提前 + 飞行）。
  // 注意：static 只在进程启动时读一次 ⇒ 改这个环境变量必须重启进程。
  // 总提前量（秒）= 取帧 → 弹丸离膛的三段延迟（链路 26 ms + 云台 34 ms + 出弹 ≈70 ms）。
  // 10-23 现场实测：缺省 0 时中速（6 rad/s）稳定偏后 0.8 板 = 108 mm ≈ 100 ms；
  // 低速（4.1 rad/s）同样 100 ms 只折合 74 mm，且板斜 29° 投影后仍能擦中 ⇒ 低速不用改。
  // 因此按转速分两档：低速保持 0（现场已验证全中），中高速补 0.024（H5 挡 10 发全中）；
  // 5.0~5.5 rad/s 线性过渡，正好贴着官方 speed_band_split=5.0 的带交界。
  // DSH_LEAD 依旧一票覆盖两档（旧命令 DSH_LEAD=0 会同时清零，行为与以前一致）。
  static double lead_low_s = []() { const char * v = std::getenv("DSH_LEAD_LOW"); return v ? std::atof(v) : 0.000; }();
  // 10-23 20:00 定档：中速 0.024 s 打在 H5 挡、10 发全中（此前 0.044 擦前边 5-8 mm、
  // 0.050 直接落到右边缘外 6 mm，斜率 6 mm/6 ms）。低速 0、中速 0.024 是本机的定稿值。
  static double lead_mid_s = []() { const char * v = std::getenv("DSH_LEAD_MID"); return v ? std::atof(v) : 0.024; }();
  static double lead_s = lead_mid_s;  // 日志/汇总里显示的那一档（中速）
  {
    const char * v = std::getenv("DSH_LEAD");
    if (v != nullptr) { lead_low_s = lead_mid_s = lead_s = std::atof(v); }
  }
  // 按目标转速（rad/s）取当前生效的提前量
  const auto lead_at = [](double w) {
    const double a = std::abs(w);
    const double lo = 5.0, hi = 5.5;
    double k = (a - lo) / (hi - lo);
    k = (k < 0.0) ? 0.0 : ((k > 1.0) ? 1.0 : k);
    return lead_low_s + (lead_mid_s - lead_low_s) * k;
  };

  // 选板迟滞（度）：新板必须比当前板好这么多才允许换。5° 在 6 rad/s 只值 15 ms，
  // 中速 ω·t_arrive≈89° 正贴 90° 格边界时，1° 的估计误差就会翻一格（≈141 mm）。
  static const double select_hyst_deg = []() {
    const char * v = std::getenv("DSH_SELECT_HYST");
    return v ? std::atof(v) : 15.0;
  }();

  // 静止靶判定参数：阈值按"每秒度数"给（与帧率无关），再叠加"连续帧数"。
  // DSH_STATIC_RATE_DPS：默认 30 °/s（等价于 60 fps 下 0.5°/帧）；DSH_STATIC_FRAMES：默认 10 帧。
  const double static_rate_dps = policy::env_double("DSH_STATIC_RATE_DPS", 30.0);
  const int static_frames_needed = []() {
    const int n = static_cast<int>(policy::env_double("DSH_STATIC_FRAMES", 10.0));
    return n > 0 ? n : 1;  // ★6：0/负数会变成"永判静止"⇒ 一块板连打，必须 clamp
  }();

  // ---- 测试汇总器：退出时（或按 s）把 A/B/C 三组的数一次性打出来 ----
  // 目的：这些数散落在启动日志、arrive/fire-gate 行、lat CSV 里，几百行日志翻着找很容易漏项。
  // 顺带把**启动日志原样收进来** ⇒ 退出时输出的那一块自带"这次是怎么跑的"，是自洽的记录。
  summary::TestSummary test_summary;
  {
    const char * tag_env = std::getenv("DSH_TEST_TAG");
    const std::string tag = (tag_env != nullptr && tag_env[0] != '\0') ? tag_env : "run";
    test_summary.configure(
      tag,
      arrive_mm > 0.0 ? fmt::format("{:.0f} mm（按距离换算）", arrive_mm)
                      : fmt::format("{:.2f}°（固定角度）", arrive_tol_deg),
      fmt::format("{}，容差 {:.2f} m，信任 {} s", policy::low_mode_name(low_mode), low_plate_z_tol,
                  low_trust_s),
      fmt::format("{}，人工门={}", fire_key_gate.backend_name(),
                  fire_key_enabled ? "开" : "关"),
      fmt::format("{:.3f} s", lead_s), fmt::format("{:.3f} s", fire_delay_report_s),
      aim_select_arrival ? "到达时刻(arrival)" : "当前(current)",
      fmt::format("{:.0f} °/s × {} 帧", static_rate_dps, static_frames_needed),
      fmt::format("{:.2f} s", fire_interval_s), lead_s, fire_delay_report_s);
  }
  // 启动日志：既打出来，也收进汇总（退出时原样附在汇总最后）
  auto startup_log = [&test_summary](const std::string & s) {
    tools::logger()->info("{}", s);
    test_summary.add_startup(s);
  };

  startup_log(fmt::format(
    "[task2] 低位靶规则 = {}（DSH_LOW_PLATE=\"{}\"，高度容差 {:.2f} m，信任 {} s）｜"
    "人工开火门 = {}（模式 {}，按键后端 {}）",
    policy::low_mode_name(low_mode), low_mode_raw, low_plate_z_tol, low_trust_s,
    fire_key_enabled ? "开" : "关", fire_key_gate.mode_text(), fire_key_gate.backend_name()));
  startup_log(fmt::format(
    "[task2] 出弹延时（记录值，不单独累加）= {:.3f} s｜选板 = {}｜到位容限 = {}｜"
    "静止靶判据 = {:.0f} °/s 且连续 {} 帧",
    fire_delay_report_s, aim_select_arrival ? "到达时刻(arrival)" : "当前(current)",
    arrive_mm > 0.0 ? fmt::format("{:.0f} mm（按距离换算）", arrive_mm)
                    : fmt::format("{:.2f}°（固定角度）", arrive_tol_deg),
    static_rate_dps, static_frames_needed));
  // 预测时域分解（方案①）：**总提前量 = DSH_LEAD，是唯一的功能旋钮**；
  // DSH_FIRE_DELAY 只用来记录"其中出弹延时约多少"，**不再单独累加**（防同一段补两次）。
  startup_log(fmt::format(
    "[task2] 预测时域 = 总提前 低速 {:.3f}s / 中高速 {:.3f}s（{:.1f}→{:.1f} rad/s 线性过渡，"
    "出弹延时约 {:.3f}s 仅记录不单独累加）+ 飞行 ~{:.3f}s（3.15 m @25 m/s 估）",
    lead_low_s, lead_mid_s, 5.0, 5.5, fire_delay_report_s, 3.15 / 25.0));
  if (fire_delay_report_s > 0.0 && lead_s <= 0.0) {
    const std::string warn = fmt::format(
      "[task2] ⚠ DSH_FIRE_DELAY={:.3f}s 已设，但 DSH_LEAD=0 ⇒ 出弹延时**没有**进入补偿。"
      "方案①下请把它并进 DSH_LEAD（设 DSH_LEAD={:.3f}），否则这一段等于没补。",
      fire_delay_report_s, fire_delay_report_s);
    tools::logger()->info("{}", warn);
    test_summary.add_startup(warn);
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用
  // 现场测量探针：DSH_LAT_CSV 没设就什么都不做
  probe::Latency latency;
  // 检测日志：DSH_DET_CSV 没设就什么都不做；离线调 EKF 靠它
  detect_log::DetectLog detlog;
  // 窗口上的状态条：每帧都画，用来证明「还在画」而不是窗口冻住了
  overlay::Hud hud;

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

  // ---- 到位门控的跨帧状态 ----
  bool arrive_tripped = false;     // 看门狗是否已经解除门控（本进程内不再恢复）
  int arrive_blocked_shots = 0;    // 累计被门控挡掉、本来真会打出去的弹数
  double arrive_err_min = 0.0;     // 统计窗口内 yaw 差的最小/最大值
  double arrive_err_max = 0.0;
  bool arrive_err_seen = false;
  int arrive_err_frames = 0;

  // ---- 开火节奏的跨帧状态 ----
  int arrive_ok_streak = 0;   // 连续落在到位门限内的帧数（防单帧蒙混过关）
  int last_fired_plate = -1;  // 上一发打的是哪一块板（armor id，0..3）
  bool plate_armed = true;    // 这块板是否已上膛（打完要摆出窗口再回来）
  bool plate_locked = false;  // 锁板模式：已锁定一块板（DSH_LOCK_MODE）
  std::chrono::steady_clock::time_point lock_start = std::chrono::steady_clock::now();
  bool ambush_was_ready = false;  // 伏击相位门的边沿（只用来入日志）
  bool ambush_aligned_logged = false;  // 转正帧日志：每圈只打一次
  // 选板变化时单独记一行：临界带（两块 facing 相差 ±5°）只有约 0.15 s，
  // 而 [dbg] fire-gate 每 48 帧才打一次 —— 不单独记就根本看不见选板在闪。
  int last_logged_aim_id = -1;
  double last_logged_facing = 0.0;
  // 上一帧真正选中的板（跨帧记忆）。  // 原来 aim_id 每帧都被重置成 target.last_id ⇒ `:399` 那个「5° 迟滞」的比较基准
  // 永远是 last_id、不跨帧 ⇒ 两块板 facing 相差 ±5° 的窄带里，选板会跟着 last_id 闪。
  // 复核稿第 5 条：把 aim_id 提到循环外，迟滞才有意义。
  int aim_id_persist = -1;
  // 静止靶判定用的跨帧状态（判据见下面 plate_armed 那段的说明）
  double last_facing_deg = 0.0;
  int facing_still_frames = 0;
  // 上一帧发出去的 yaw（命令突变保护用，官方 shooter.cpp 的第二个条件）
  float last_cmd_yaw = 0.0f;
  bool has_last_cmd = false;
  // 现在"为什么打不出"（徽章副标题用；READY = 其它条件都满足，只差按 F）
  std::string fire_state = "BOOT";
  // 当前瞄的那块板（EKF 列表里的 id），给 HUD 单独描框用
  int aimed_id = -1;
  // 上一帧开火门是否允许（用来打"允许↔禁火"的翻转日志）
  bool prev_fire_allowed = false;
  // 目标/滤波器重建的代数计数器：低位靶的"高度差已收敛"信任在重建后作废。
  // 用计数器统一判废（★4），不靠在三处散着清零，漏一处就会带着旧信任去判新靶。
  int target_generation = 0;
  int last_seen_generation = -1;
  // 帧间隔（秒）：静止靶判据按 °/s 给，需要真实 dt（★7：阈值不能是"度/帧"，否则换帧率就变紧）
  std::chrono::steady_clock::time_point last_frame_time = std::chrono::steady_clock::now();
  double frame_dt_s = 0.0;

  const auto t_start = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    // Your code start

    // 刷新 F 键状态：每帧都刷，跟有没有目标无关（没有目标时人按了 F 也要能被记住）
    fire_key_gate.poll();

    camera.read(img, t);
    if (img.empty()) {
      continue;  // 相机偶尔给空帧，跳过本轮，不发送任何指令
    }
    // 测量：取到帧的时刻 + 这一帧的曝光时间戳
    latency.on_grab(t, camera.dev_ts());

    // 用照片曝光时刻对齐云台姿态
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);

    if (frame_count % 48 == 0)
    {
      const char * gear = "?";
      switch (gimbal.mode())
      {
        case io::GimbalMode::IDLE: gear = "IDLE(手动/未接管)"; break;
        case io::GimbalMode::AUTO_AIM: gear = "AUTO_AIM(自瞄)"; break;
        case io::GimbalMode::SMALL_BUFF: gear = "SMALL_BUFF(小陀螺)"; break;
        case io::GimbalMode::BIG_BUFF: gear = "BIG_BUFF(大陀螺)"; break;
      }
      tools::logger()->info("[dbg] 电控挡位 = {}", gear);
    }
    const int frame_id = frame_count++;
    auto armors = yolo.detect(img, frame_id);
    const auto gimbal_state = gimbal.state();
    const auto now = std::chrono::steady_clock::now();
    frame_dt_s = tools::delta_time(now, last_frame_time);
    last_frame_time = now;
    // 离线复现用：这一帧看到的所有板子（颜色、编号、置信度、四个角点）
    detlog.on_frame(frame_id, tools::delta_time(now, t_start), armors);

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
    // 测量：推理 + 解算到此为止，后面只剩发指令
    latency.on_infer(std::chrono::steady_clock::now());

    const bool has_detection = (chosen != nullptr);
    // 短暂丢失：还在保持时间内就继续用滤波器外推，不松开云台
    const bool holding =
      !has_detection && has_target &&
      (now - last_seen) < std::chrono::milliseconds(target_hold_ms);

    if (has_detection) {
      auto & armor = *chosen;
      // 10-23 现场修：原来比的是 target.name != armor.name，但 name 编码的是**板号**
      // （armor.cpp:242，num_id 1..4 → ArmorName(0..3)）。四板车每转过 90°（4.4 rad/s
      // 下 0.36 s）检测到的板号就变一次，于是每 0.36 s 重建一次滤波器：角速度 w、高度 z、
      // 选板记忆、静止计数全部清零，lead 永远起不来 —— 现象就是「一直不停切换靶子」。
      // 几何其实只有两种：前哨站 3 块阶梯板、其余 4 块同高板，所以只按 outpost 比较。
      const bool same_geometry =
        (armor.name == auto_aim::ArmorName::outpost) ==
        (target.name == auto_aim::ArmorName::outpost);
      if (!has_target || !same_geometry) {
        // 第一次出现，或者换了一台车：重建滤波器
        // Target() = default 不会初始化 name，所以只有在已建过目标时才读旧名字
        if (has_target) {
          tools::logger()->info(
            "[dbg] 目标重建（旧 name={} 新 name={}）", static_cast<int>(target.name),
            static_cast<int>(armor.name));
        } else {
          tools::logger()->info("[dbg] 目标新建 name={}", static_cast<int>(armor.name));
        }
        target = auto_aim::Target(armor, t, 0.0, 0);
        has_target = true;
        frames_since_init = 0;
        ++target_generation;  // 换了目标 ⇒ 高度差重新从 0 学起，低位靶信任作废
        // 换了目标 ⇒ 选板和"打过哪块"都要清零：新目标的 0 号板不是"刚打过的那块"
        aim_id_persist = -1;
        plate_locked = false;
        last_fired_plate = -1;
        facing_still_frames = 0;
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
        ++target_generation;  // 滤波器重建 ⇒ 高度差重新从 0 学起，低位靶信任作废
        aim_id_persist = -1;      // 滤波器重建 ⇒ 选板与"打过哪块"都作废
        plate_locked = false;
        last_fired_plate = -1;
        facing_still_frames = 0;
      } else {
        has_target = false;
        aim_id_persist = -1;
        plate_locked = false;
        last_fired_plate = -1;
        facing_still_frames = 0;
      }
    }

    if (!has_target) {
      // 真的没有目标：交出控制权
      gimbal.send(false, false, 0.0f, 0.0f);
      fire_state = "NO TARGET";
      {  // 汇总器：无目标的帧也要计数（否则"看不见目标"这件事不会出现在汇总里）
        summary::Frame sf;
        sf.has_target = false;
        test_summary.update(sf);
      }
    } else {
      const Eigen::VectorXd x = target.ekf_x();
      // 记录这次拟合出来的角速度落在哪个转速挡位，供下一帧判断挡位是否切换
      speed_band = (std::abs(x[7]) >= speed_band_split) ? 1 : 0;
      const auto xyza_list = target.armor_xyza_list();
      // aim_id 跨帧保持：比较基准变成「上一帧真正选中的那块」，5° 迟滞这才成立。
      // 还没选过、或者上一帧那块已经越界（换目标/板数变了）⇒ 退回 EKF 的关联结果。
      int aim_id = aim_id_persist;
      if (aim_id < 0 || aim_id >= static_cast<int>(xyza_list.size())) {
        aim_id = (target.last_id >= 0 && target.last_id < static_cast<int>(xyza_list.size()))
                   ? target.last_id
                   : 0;
      }

      // ---- 选「最正对镜头」的那块板 ----
      // EKF 关联出来的 last_id 可能是刚转过去、马上要离开视野的那一块（关联只按角度最近，
      // 不管它正面朝哪）。这里再按几何挑一次：四块板里谁的法线最接近视线就瞄谁，
      // 也就是「只打转到中间的那块，不追着快转走的那块」。
      // 判定角度 = 板的外法线（a + π）与「目标中心指向相机」的方向之差，0° = 正对。
      const double center_bearing = std::atan2(-x[2], -x[0]);
      const auto facing_of = [&](const Eigen::Vector4d & p) {
        return tools::limit_rad(p(3) + M_PI - center_bearing) * 57.29577951308232;
      };
      // ---- 官方 sp_vision_25 的做法：选板要在「弹丸到达那一刻」的画面上选 ----
      // 官方 aimer.cpp：先 target.predict(future + fly_time)，再 choose_aim_point(预测后的 target)，
      // 并用 coming_angle / leaving_angle 两个角配合 w 的符号表达「选正在进入窗口的那块」。
      // 理由：4 rad/s、飞行 126 ms 时靶子转过 29°，「现在正对」的那块到弹着时已经转走 29°，
      // 而「再过 126 ms 正好正对」的是隔壁那块 ⇒ 现在正对 ≠ 命中时正对。
      // 等价写法：板的外法线跟着靶子以 w 转，所以 facing 也以 w 变化，
      // 「到达时的朝向」= 现在的朝向 + w * t_arrive；选 |到达朝向| 最小的那块。
      const double w_deg_per_s = x[7] * 57.29577951308232;
      const double v_guess = (gimbal_state.bullet_speed > 5.0) ? gimbal_state.bullet_speed
                                                               : default_bullet_speed;
      double dist_guess = xyza_list[aim_id].head(3).norm();
      // 到达时间 = 总提前(DSH_LEAD) + 飞行(距离/弹速)
      // 三段（链路 / 出弹 / 飞行）缺一不可：缺总提前 ⇒ 到达朝向低估 w×总提前；
      // 缺飞行 ⇒ 差一整块板（4 rad/s、126 ms ⇒ 29°）。
      // 方案①：出弹延时已经并进 DSH_LEAD（总提前量），这里不再单独加。
      // 到达时刻（选板/门控用）= **只算飞行时间**。
      // 10-23 现场：把 lead 也加进这里时（0.10 s ⇒ 相位再 +34°），四块板的 argmin 会翻到
      // 隔壁一格（一格 = 90° ≈ 283 mm），落点反而从偏后 0.8 板变成偏后 2~3 板。
      // 结论：预提前量只该补在**瞄点**（calc_aim 的 t_extra）上，不能同时进选板相位。
      double t_arrive = dist_guess / std::max(1.0, v_guess);
      double face_offset_deg = aim_select_arrival ? w_deg_per_s * t_arrive : 0.0;
      const auto facing_sel = [&](const Eigen::Vector4d & p) {
        return facing_of(p) + face_offset_deg;  // 「到达（开火）时的朝向」，选板与门控都用它
      };
      // ---- 高度带筛选（DSH_PICK_BAND）----
      // 只在指定高度层里挑；这一层一块都没挑到就退回不过滤（保证不会零候选）。
      std::vector<int> cand;
      cand.reserve(xyza_list.size());
      if (pick_band != 0 && !xyza_list.empty()) {
        double z_lo = xyza_list[0](2), z_hi = xyza_list[0](2);
        for (const auto & pl : xyza_list) {
          z_lo = std::min(z_lo, pl(2));
          z_hi = std::max(z_hi, pl(2));
        }
        const double z_ref = (pick_band == 1) ? z_lo : z_hi;
        for (int i = 0; i < static_cast<int>(xyza_list.size()); ++i) {
          if (std::abs(xyza_list[i](2) - z_ref) <= pick_z_tol) cand.push_back(i);
        }
      }
      if (cand.empty()) {
        for (int i = 0; i < static_cast<int>(xyza_list.size()); ++i) cand.push_back(i);
      }

      // ---- 锁板：锁住了就不再重选（守株待兔的核心）----
      const double lock_age_s = std::chrono::duration<double>(now - lock_start).count();
      const bool locked = lock_mode && plate_locked && aim_id_persist >= 0 &&
                          lock_age_s < lock_timeout_s &&
                          std::find(cand.begin(), cand.end(), aim_id) != cand.end();
      if (!locked) {
        int best_id = cand.front();
        double best_facing = std::abs(facing_sel(xyza_list[best_id]));
        for (int i : cand) {
          if (std::abs(facing_of(xyza_list[i])) > vis_limit_deg) continue;  // 背面/深侧面不选
          const double f = std::abs(facing_sel(xyza_list[i]));
          if (f < best_facing) {
            best_facing = f;
            best_id = i;
          }
        }
        // 锁板模式：直接取最优（不受迟滞限制，否则永远切不过去）；
        // 否则保留迟滞。10-23 现场：5° 在 6 rad/s 只值 15 ms，中速 ω·t_arrive≈89°
        // 正贴 90° 的格边界，1° 的估计误差就让 argmin 翻一格（≈141 mm）⇒ 缺省放大到 15°。
        if (best_id != aim_id &&
            (lock_mode || best_facing + select_hyst_deg < std::abs(facing_sel(xyza_list[aim_id])))) {
          aim_id = best_id;
        }
        aim_id_persist = aim_id;  // 跨帧记忆，供下一帧做迟滞/锁板判断
        aimed_id = aim_id;        // 给 HUD 描框用（真正瞄的是哪块）
        if (lock_mode) {
          plate_locked = true;
          lock_start = now;
        }
      } else {
        aimed_id = aim_id;  // 锁着：不改 aim_id
      }

      // 选板落定后，用**选中的那块板自己的**距离把 t_arrive / 到达朝向重算一遍：
      // 上面用的是选板前 EKF 关联板（last_id）的距离。四块板在半径 r 上绕同一圆心，
      // 相邻板距离差约 2r·sin45°（r≈0.2 m 时 0.28 m），中挡 5.5-9 rad/s 下
      // t_arrive 差 ≤16 ms 就是最多 8.3° 的相位——门控会拿另一块板的角度判「到没到位」。
      // 注意：选板**排序**本身仍用选板前的偏移（要自洽得迭代一次），这次只修后置量。
      // ---- 伏击模式：只在最低层里挑「最快转正」的那块板（不做 |facing| 竞争）----
      // 板的朝向 facing 随目标旋转以 ω 变化，转到 0° 就是正对相机。
      // 剩余时间 = 剩余角度 / |ω|，取 [0, 2π) 里的最小正数。
      double ambush_dt = 1e9;
      if (ambush_on) {
        const double kTwoPi = 6.283185307179586;
        const double w_abs = std::abs(x[7]);
        if (w_abs >= 1e-3 && !xyza_list.empty()) {
          double z_min = 1e9;
          for (const auto & pl : xyza_list) z_min = std::min(z_min, pl(2));
          int best = aim_id;
          double best_dt = 1e9;
          for (int i = 0; i < static_cast<int>(xyza_list.size()); ++i) {
            if (xyza_list[i](2) - z_min > pick_z_tol) continue;  // 只要最低层
            const double f = facing_of(xyza_list[i]) * 0.017453292519943295;
            double remain = (x[7] >= 0.0) ? std::fmod(-f, kTwoPi) : std::fmod(f, kTwoPi);
            if (remain < 0.0) remain += kTwoPi;
            const double dt_i = remain / w_abs;
            if (dt_i < best_dt) {
              best_dt = dt_i;
              best = i;
            }
          }
          if (best_dt < 1e8) {
            aim_id = best;
            aim_id_persist = best;
            aimed_id = best;
            ambush_dt = best_dt;
          }
        }
      }

      dist_guess = xyza_list[aim_id].head(3).norm();
      // 同上：这里也只算飞行时间。预提前量只进 calc_aim 的 t_extra。
      t_arrive = dist_guess / std::max(1.0, v_guess);
      face_offset_deg = aim_select_arrival ? w_deg_per_s * t_arrive : 0.0;

      const Eigen::Vector4d plate = xyza_list[aim_id];

      const double facing_deg = facing_of(plate);          // 现在的朝向
      const double facing_arrive_deg = facing_sel(plate);  // 到达（开火）时的朝向
      const bool facing_ok = std::abs(facing_arrive_deg) <= plate_facing_deg;
      // 伏击的相位门：还要多久转正 ≤ B3 + 飞行时间（+半帧，补采样离散）
      const double tf_amb = dist_guess / std::max(1.0, v_guess);
      const bool fire_phase_ok =
        ambush_on && (ambush_dt <= ambush_b3_s + tf_amb + 0.5 * frame_dt_s);
      if (ambush_on && fire_phase_ok != ambush_was_ready) {
        tools::logger()->info(
          "[dbg] ambush id={} dt_align={:.3f}s tf={:.0f}ms B3={:.0f}ms ready={}", aim_id, ambush_dt,
          tf_amb * 1000.0, ambush_b3_s * 1000.0, fire_phase_ok ? 1 : 0);
        ambush_was_ready = fire_phase_ok;
      }
      const bool facing_gate = ambush_on ? fire_phase_ok : facing_ok;
      // 单C补丁：板转正那一帧，「检测到的板中心 vs 画面中心」的像素偏差（归一化坐标）。
      // 干跑判据：连续 10 次转正的均值 < 0.02（= 2% 画面宽，约 25 px @1280）⇒ 零位交叉验证通过。
      // 只读诊断，不参与任何门控：伏击模式从有 target 起就下发瞄点，挡火交给 stable/frames。
      if (ambush_on && has_detection && ambush_dt < 0.5 * frame_dt_s + 1e-6) {
        if (!ambush_aligned_logged) {
          const cv::Point2f cn = chosen->center_norm;
          tools::logger()->info(
            "[dbg] ambush-aligned id={} err_px_x={:.4f} err_px_y={:.4f}", aim_id, cn.x - 0.5f,
            cn.y - 0.5f);
          ambush_aligned_logged = true;
        }
      } else if (ambush_dt > 0.3) {
        ambush_aligned_logged = false;
      }

      // 选板变化单独记一行（见上面 last_logged_aim_id 的说明）：
      // 这一行是「人眼看不见、系统日志每 48 帧才打一次」的那段临界带的唯一证据。
      bool plate_switched = false;
      if (aim_id != last_logged_aim_id) {
        plate_switched = true;
        tools::logger()->info(
          "[dbg] 选板变化 aim_id {} -> {}（facing {:.0f}° -> {:.0f}°，assoc {:.0f}°，候选 {} 块）",
          last_logged_aim_id, aim_id, last_logged_facing, facing_deg, facing_deg,
          static_cast<int>(xyza_list.size()));
        last_logged_aim_id = aim_id;
        last_logged_facing = facing_deg;
      }
      // 一块板只打一发：换板、或同一块板摆出 rearm 角，才重新上膛。
      // 没有这条，程序会在板子转走的整段时间里追着它连打，一串弹全落在空当里。
      //
      // 这里必须**每帧现算**，不能用「置位保持」的写法，保持版有两个洞：
      //   ① 静止靶不转，永远摆不出 rearm 角 ⇒ 打完一发就永久锁死；而零位标定正好要用静止靶连打 3 发。
      //   ② aim_id 从 A 跳到 B 再跳回 A 时，A 没摆出角度也会带着「已上膛」回来。
      //
      // 「靶子是不是静止」的判据（复核稿第 4 条）：
      //   原来用 |x[7]| < 0.5（EKF 估的角速度）。但 w 在**起步爬升**和**反向过零**时
      //   都会掉到很小 ⇒ 这条规则整个失效 ⇒ 5 Hz 对着同一块板连打（纯哑炮、白吃额度）。
      //   改成直接量「当前瞄的这块板，这一帧的朝向变了多少度」：
      //     旋转靶 3~9 rad/s ⇒ 每帧变 3.9~11.7°，计数器每帧清零 ⇒ 永远判成"在转"；
      //     静止靶            ⇒ 每帧只变 0.03°（量测噪声），计数器能攒起来。
      //   这样判据不依赖 w 的符号和收敛过程，w 反向过零也不受影响。
      // 注意 facing_deg 是**度**，不能拿 tools::limit_rad 去包（它是按弧度写的 2π 环绕），
      // 这里自己按 ±180° 环绕。
      double d_facing_deg = facing_deg - last_facing_deg;
      while (d_facing_deg > 180.0) d_facing_deg -= 360.0;
      while (d_facing_deg < -180.0) d_facing_deg += 360.0;
      d_facing_deg = std::abs(d_facing_deg);
      last_facing_deg = facing_deg;
      // 阈值按「°/s」给（与帧率无关）：默认 30 °/s ≈ 60 fps 下 0.5°/帧、45 fps 下 0.39 rad/s。
      // 再叠加「连续 N 帧」（DSH_STATIC_FRAMES，默认 10）。
      const double d_facing_rate = (frame_dt_s > 1e-3) ? d_facing_deg / frame_dt_s : 0.0;
      facing_still_frames = (d_facing_rate < static_rate_dps) ? facing_still_frames + 1 : 0;
      const bool target_static = facing_still_frames >= static_frames_needed;
      plate_armed = target_static || (aim_id != last_fired_plate) ||
                    (std::abs(facing_deg) > plate_rearm_deg);

      // ---- 只打低位靶，不打高位靶（三态，见 src/low_plate.hpp）----
      // 判定用 EKF 预测出来的四块板的高度差：z_max − z_min 就是 x[10]（瞄高板时等于 z_aim − z_min）。
      // warmup 模式下，这个差值稳定保持 DSH_LOW_PLATE_TRUST_S 秒 ⇒ 认为已收敛 ⇒ 放开高板；
      // 换目标 / 滤波器重建 ⇒ 高度差重新从 0 学起 ⇒ 信任作废（generation 计数器统一判废）。
      const bool rebuilt = (target_generation != last_seen_generation);
      const bool low_ok = low_gate.allows_fire(
        xyza_list, aim_id, has_detection, frame_dt_s, rebuilt);
      last_seen_generation = target_generation;  // 重建只在第一次被"看见"时判废
      const policy::LowPlate & low = low_gate.state();
      if (low_gate.take_flip()) {
        tools::logger()->info(
          "[task2] 低位靶规则：dz={:.2f} m 稳定 {:.1f} s，x[10] 视为已收敛，放开高板",
          low.z_spread, low_gate.trust_progress());
      }

      const bool plate_ok = facing_gate && plate_armed && low_ok;

      // arrival 模式自检（10-23 现场）：偏移量 = w × t_arrive。滤波器刚重建/刚接回目标时
      // x[7]≈0 ⇒ 偏移量≈0 ⇒ 选板退回「现在最正」的那块，正好是 0.266 s 后转走的那块。
      // 所以 arrival 模式下 w 太小就干脆不开火：宁可不打，也不打反。
      // 0.8 rad/s = 46 °/s，0.266 s 对应 12° 相位，低于这个量级偏移本来也没意义。
      const bool rotation_ok = !aim_select_arrival || std::abs(x[7]) >= 0.8;

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
      // 2026-10-24 修复：lead 段改走 t_extra 旋转推进（da = w*(t_flight+t_extra) 精确旋转）。
      // 视在角速度×lead 的小角近似在超前>40°失效（cos 为负），中速相位失配 ~120ms（慢动作实锤）。
      task_common::calc_aim(x, plate, bullet_speed, muzzle_offset_x, aim_yaw, aim_pitch,
                            (ambush_on ? 0.0 : lead_at(x[7])));
      if (ambush_on) {
        // 伏击点 = 中心 + r·u（u = 中心指向相机），高度用该板的 z。
        // 把状态副本的平移/旋转速度清零 ⇒ calc_aim 里 aim_xyz 就是这个静止点，
        // 弹道 pitch 仍由 tools::Trajectory 算（含重力与空气阻力）。
        const double cx = x[0], cy = x[2];
        const double cnorm = std::hypot(cx, cy);
        if (cnorm > 0.5) {
          const double r_low = std::hypot(plate[0] - cx, plate[1] - cy);
          Eigen::Vector4d amb(cx - r_low * cx / cnorm, cy - r_low * cy / cnorm, plate[2], plate[3]);
          Eigen::VectorXd xs = x;
          xs[1] = 0.0;
          xs[3] = 0.0;
          xs[5] = 0.0;
          xs[7] = 0.0;
          task_common::calc_aim(xs, amb, bullet_speed, muzzle_offset_x, aim_yaw, aim_pitch, 0.0);
        }
      }
      // 光轴补偿：把瞄准线掰回炮口实际指向（正 yaw = 往左，正 pitch = 往上）
      // 视在角速度线性补偿已删除：其功能由 calc_aim 的 t_extra 精确旋转接管（见上）。
      // 两处并存会把同一段 lead 补两次（过头 0.83°+）。
      aim_yaw += static_cast<float>(yaw_offset_eff * 0.017453292519943295);
      aim_pitch += static_cast<float>(pitch_offset_eff * 0.017453292519943295);

      // 阶跃测试：只在 DSH_STEP_DEG 打开时生效，把 1 Hz 方波直接叠在 yaw 指令上。
      // 命令是方波、反馈（下面的 fb_yaw）是爬升曲线，两者一叠就是云台的到位时间。
      if (latency.step_enabled()) aim_yaw += latency.step_delta(now);

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

      // ---- 人按住 F 才允许开火（人工确认开火）----
      // 这一条只挡「什么时候扣扳机」，不挡跟踪：云台照旧一直在跟，人不按就不发。
      const bool key_ok = !fire_key_enabled || fire_key_gate.allowed();

      // ---- 云台到位判定 ----
      // 差值按 ±180° 展开，只看 yaw（gimbal.cpp:106 里 yaw 同号，可比）。
      double dyaw = static_cast<double>(aim_yaw) - static_cast<double>(gimbal_state.yaw);
      while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
      while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
      const double yaw_err_deg = std::abs(dyaw) * 57.29577951308232;

      // 到位容限可以按「线性距离」给（官方 sp_vision_25 的 shooter 参数）：
      //   first_tolerance 3°（<2 m） / second_tolerance 2°（>2 m）
      // 换算成米：3°×2 m ≈ 105 mm、2°×3 m ≈ 105 mm ⇒ 官方用的是**恒定的线性容差**。
      // 本工程原来固定 0.8°，在 3.15 m 处只有 44 mm，比官方严 2.4 倍
      // ⇒ 这就是「门限永远打不开」最可能的原因。DSH_ARRIVE_MM > 0 时按距离换算。
      const double arrive_tol_deg_eff = (arrive_mm > 0.0 && dist_guess > 0.3)
                                         ? arrive_mm * 0.001 / dist_guess * 57.29577951308232
                                         : arrive_tol_deg;

      // 命令突变保护（官方 shooter.cpp 的第二个条件）：
      // 新命令和上一帧差太多时这一枪不打——此时炮管还在老位置，而下一帧的命令才是对的。
      const double cmd_jump_deg =
        has_last_cmd ? std::abs(static_cast<double>(aim_yaw) - last_cmd_yaw) * 57.29577951308232
                     : 0.0;
      const bool cmd_stable = cmd_jump_deg < 2.0 * arrive_tol_deg_eff;

      // pitch 用「相加」：发指令时 gimbal.cpp:111 取了反，回传 gimbal.cpp:272 没翻回来，
      // 所以到位时 aim_pitch + fb_pitch ≈ 0。写成减法就会得到一个恒等于 2×pitch 的假误差，
      // 门控会一直挡——这也是第一次上车要看 [task2] 那行 cmd/fb 的原因。
      const double pitch_err_deg =
        std::abs(static_cast<double>(aim_pitch) + static_cast<double>(gimbal_state.pitch)) *
        57.29577951308232;

      // 不管门控开没开都统计 yaw 差值：它就是「云台跟踪滞后」，
      // 也是判断门限设得对不对的唯一依据（静止靶应当接近 0，转靶才是真实滞后）。
      if (!arrive_err_seen) {
        arrive_err_min = arrive_err_max = yaw_err_deg;
        arrive_err_seen = true;
      } else {
        if (yaw_err_deg < arrive_err_min) arrive_err_min = yaw_err_deg;
        if (yaw_err_deg > arrive_err_max) arrive_err_max = yaw_err_deg;
      }
      arrive_err_frames++;

      const bool arrive_on = arrive_tol_deg > 0.0 && !arrive_tripped;
      // 要求连续 N 帧都在门限内：云台扫过 0.5° 的那一瞬间也会「看起来到位」，
      // 只卡单帧等于没卡，这一条就是防这个的。
      const bool arrive_in_tol =
        yaw_err_deg < arrive_tol_deg_eff && pitch_err_deg < arrive_tol_pitch_deg && cmd_stable;
      arrive_ok_streak = (arrive_in_tol && !arrive_tripped) ? arrive_ok_streak + 1 : 0;
      const bool arrive_ok =
        !arrive_on || (arrive_in_tol && arrive_ok_streak >= arrive_frames_needed);

      // 只有这一帧真的有观测才开火。原来这句只写在注释里，代码没兑现：
      // 目标丢失超过 target_hold_ms 之后 holding 变假，但 has_target 和滤波器状态都原样保留
      // （is_converged_ 停在最后一次 update 的值，frames_since_init 也停在上次的数），
      // 于是 fire 会在视野里什么都没有的情况下持续为真，每隔 fire_interval_s 打一发空枪。
      // 现场那次 rounds 333 就是这么来的：333 乘 0.2 秒约 67 秒，正好一整场。
      if (frame_count % 48 == 0)
      {
        tools::logger()->info(
          "[dbg] fire-gate aim_id={} stable={} conv={} frames={}/{} nis={:.2f}/{:.2f} interval={} rounds={} mode={} hold={} plate={}({:.0f}°→到达{:.0f}°) low={}(dz={:.2f}m 低{}共{}块) key={} t_arr={:.0f}ms assoc={:.0f}° | mode_now={} bullet_alive={} rounds {}/{} lock={}({:.1f}s)",
          aim_id, stable, target.is_converged(), frames_since_init, min_frames_for_fire,
          nis_failure_rate, max_nis_failure_rate, interval_ok, rounds_ok, mode_ok, holding,
          plate_ok, facing_deg, facing_arrive_deg, low.aim_is_low, low.dz, low.low_count, low.total,
          key_ok, t_arrive * 1000.0, facing_deg, static_cast<int>(gimbal.mode()),
          bullet_count_alive, rounds_this_stage, max_rounds_per_stage,
          (locked ? 1 : 0), lock_age_s);
        // 到位门控的自检行：err/窗口/blocked/tripped 就是「这个参数有没有问题」的证据。
        //   · 静止靶上 err 长期 > 门限  → 门限太紧或零位偏了
        //   · tripped=1                 → 看门狗已经替你解除了，本次运行不会再挡
        //   · blocked 一直是 0 而 tripped=0 → 门控从来没起作用（没碰到需要它的场景）
        //   · 末尾的 fb/cmd 是符号判据：到位时 fb yaw ≈ cmd yaw、fb pitch ≈ −cmd pitch。
        //     若 fb pitch ≈ +cmd pitch，说明我这条 pitch 相加的约定错了，改一行即可。
        tools::logger()->info(
          "[dbg] arrive={} tol={:.2f}/{:.2f}°(eff {:.2f}°) errY={:.2f}° errP={:.2f}° Ymin/max={:.2f}/{:.2f} n={} "
          "blocked={} tripped={} jump={:.2f}° | fb yaw={:.2f} pitch={:.2f} cmd yaw={:.2f} pitch={:.2f}",
          arrive_on, arrive_tol_deg, arrive_tol_pitch_deg, arrive_tol_deg_eff, yaw_err_deg,
          pitch_err_deg, arrive_err_min, arrive_err_max, arrive_err_frames, arrive_blocked_shots,
          arrive_tripped, cmd_jump_deg, gimbal_state.yaw, gimbal_state.pitch, aim_yaw, aim_pitch);
        arrive_err_seen = false;  // 下一行重新统计，每行覆盖约 0.8 秒
        arrive_err_frames = 0;
      }
      const bool would_fire =
        has_detection && stable && interval_ok && rounds_ok && mode_ok && !holding && plate_ok &&
        key_ok && rotation_ok;
      const bool fire = would_fire && arrive_ok && !latency.step_enabled();  // 阶跃测试期间一律不放弹

      // ---- 徽章副标题：现在为什么打不出（READY = 其它条件都满足，只差按 F）----
      if (fire) {
        fire_state = "FIRE";
      } else if (!has_detection) {
        fire_state = "NO TARGET";
      } else if (!mode_ok) {
        fire_state = "WAIT gear (need AUTO_AIM)";
      } else if (!key_ok) {
        fire_state = "WAIT F key";
      } else if (!stable) {
        fire_state = "WAIT ekf (frames/nis)";
      } else if (!rotation_ok) {
        fire_state = "WAIT w (arrival)";
      } else if (!plate_ok) {
        fire_state = "WAIT plate (facing/armed/low)";
      } else if (!arrive_ok) {
        fire_state = "WAIT gimbal arrive";
      } else if (!interval_ok) {
        fire_state = "WAIT interval";
      } else {
        fire_state = "READY";
      }

      // ---- 汇总器：把这一帧的判据状态喂进去（退出时统一输出 A/B/C 三组）----
      {
        summary::Frame sf;
        sf.has_target = true;
        sf.has_detection = has_detection;
        sf.yaw_err_deg = yaw_err_deg;
        sf.arrive_ok = arrive_ok;
        sf.plate_ok = plate_ok && rotation_ok;  // w 未收敛也计入「挡住」的一类
        sf.key_ok = key_ok;
        sf.mode_ok = mode_ok;
        sf.stable = stable;
        sf.would_fire = would_fire;
        sf.fire = fire;
        sf.dz = low.dz;
        sf.z_spread = low.z_spread;
        sf.trust = low_gate.trust_progress();
        sf.w = x[7];
        sf.t_arrive_ms = t_arrive * 1000.0;
        sf.aim_id = aim_id;
        sf.gear = static_cast<int>(gimbal.mode());
        sf.rounds = rounds_this_stage;
        sf.plate_switch = plate_switched;
        test_summary.update(sf);
      }

      // 看门狗：只有「本来真会打出去的那一发被挡了」才算损失，才计数
      if (arrive_on && !arrive_ok && would_fire) {
        arrive_blocked_shots++;
        if (arrive_watchdog_shots > 0 && arrive_blocked_shots >= arrive_watchdog_shots) {
          arrive_tripped = true;
          tools::logger()->warn(
            "[task2] 到位门控已累计挡掉 {} 发（当前 yaw 差 {:.2f}° > 门限 {:.2f}°），"
            "判定门限过紧或 yaw 零位有固定偏差，本次运行已自动解除门控（改 DSH_ARRIVE_DEG "
            "后重启生效）",
            arrive_blocked_shots, yaw_err_deg, arrive_tol_deg);
        }
      }

      if (fire) {
        last_fire_time = now;
        last_round_time = now;
        band_at_last_round = speed_band;
        // 记下打的是哪一块板，并把这块板「退膛」：要等它摆出去再转回来才允许再打
        last_fired_plate = aim_id;
        plate_locked = false;  // 打出去了 ⇒ 解锁，下一帧重新挑一块
        plate_armed = false;
        // 电控没给发弹数时，用自己发出的开火命令计数
        if (!bullet_count_alive) rounds_this_stage++;
      }

      // 绝对角度控制；mode_ok 为假时交出控制权，避免和人手操作抢云台
      gimbal.send(mode_ok, fire, aim_yaw, aim_pitch, task_common::to_target_id(target.name));
      // 记下这一帧发出去的角度（下一帧用来看命令有没有突变）
      last_cmd_yaw = aim_yaw;
      has_last_cmd = true;
      // 测量：串口写完的时刻 + 我们发的角 + 电控这一帧报回来的姿态（fb 是电控坐标系，
      // 注意 gimbal.send 里 pitch 取过反，比对时两边要统一符号）
      latency.on_send(
        std::chrono::steady_clock::now(), aim_yaw, aim_pitch, gimbal_state.yaw,
        gimbal_state.pitch, fire, static_cast<float>(plate.head(3).norm()));

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
      // 新增策略的状态条（目标相关部分）：一眼看出「现在为什么不发」——是瞄到高板了？
      // 屏幕文字一律 ASCII（cv::putText 不支持中文）
      tools::draw_text(
        img,
        fmt::format(
        "plate={} dz={:.2f}m trust={:.1f}/{:.1f}s  tgt_id={}{} facing={:.0f}->{:.0f}deg  "
        "t_arr={:.0f}ms",
        low.aim_is_low ? "LOW" : "HIGH", low.dz, low_gate.trust_progress(),
        low_gate.trust_need(), aim_id, (aim_id == target.last_id ? "" : "(pred)"), facing_deg,
        facing_arrive_deg, t_arrive * 1000.0),
        cv::Point(10, 120), cv::Scalar(0, 255, 255), 0.6, 1);

      // ---- 发给 PlotJuggler 的曲线 ----
      // task2_aim：发给云台的控制命令；task2_ekf：EKF 拟合出来的角速度与健康度。
      // 角速度那条是评分直接看的，残差与 NIS 用来判断滤波器是不是真的收敛了。
      plotter.plot({
        {"task2_aim",
         {{"yaw", aim_yaw},
          {"pitch", aim_pitch},
          {"distance", plate.head(3).norm()},
          // 电控反馈的姿态，和上面两条指令同框显示，就能直接读出云台滞后多少
          {"fb_yaw", gimbal_state.yaw},
          {"fb_pitch", gimbal_state.pitch}}},
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

    // 窗口状态条：放在所有分支之外，没目标时也要画，否则看不出程序还活着
    hud.draw(img, armors, chosen, frame_id, has_target, holding);

    // ---- 真正被瞄的那块板，单独用另一种颜色描出来（2026-10-21 三审第 11 条）----
    // HUD 的绿框画的是**检测层** chosen（离画面中心最近的那块），
    // 而真正被瞄的是 EKF 列表里的 aim_id —— 两者可能不是同一块（瞄的是"还没转进视野"的那块）。
    // aim_id == last_id 时，被瞄的就是 chosen 本身 ⇒ 直接把它高亮；
    // 不相等时说明瞄的是预测出来的板，画面上看不见 ⇒ 状态行会标 (pred)。
    if (has_detection && chosen != nullptr && aimed_id >= 0 && aimed_id == target.last_id) {
      tools::draw_points(img, chosen->points, {255, 255, 255}, 2);
    }

    // ---- 开火门状态行 + 徽章（放在分支外：没目标时最需要知道开火门状态）----
    {
      const std::string m = fire_key_gate.mode_text();
      const bool is_toggle = m.rfind("toggle", 0) == 0;
      std::string f_txt;
      if (!fire_key_enabled) {
        f_txt = "GATE=OFF (auto fire, no F needed)";
      } else {
        const std::string k = is_toggle ? (fire_key_gate.latched() ? "LOCKED" : "UNLOCKED")
                                        : (fire_key_gate.pressed() ? "DOWN" : "UP");
        f_txt = fmt::format("GATE=ON ({}, F={})", m, k);
      }
      tools::draw_text(img, f_txt, cv::Point(10, 95), cv::Scalar(0, 255, 255), 0.6, 1);
      draw_fire_badge(
        img, fire_key_enabled, fire_key_gate.allowed(), fire_key_gate.degraded(),
        fire_key_gate.backend_name(), fire_state);
    }

    // 边沿日志（forensic）：按键状态一变就打一行。事后对日志/录像时序、
    // 以及"到底按没按"的争议，靠这一行裁决（比 60 fps 的画面可靠）。
    if (fire_key_gate.take_edge()) {
      tools::logger()->info(
        "[task2] 开火门按键边沿：{}（后端 {}，第 {} 帧，pressed={}）",
        fire_key_gate.pressed() ? "按下" : "松开", fire_key_gate.backend_name(), frame_id,
        fire_key_gate.pressed());
    }
    {
      const bool f_now = fire_key_enabled && fire_key_gate.allowed();
      if (f_now != prev_fire_allowed) {
        tools::logger()->info(
          "[task2] 开火门 {}（{}，第 {} 帧）", f_now ? "禁火→允许开火" : "允许开火→禁火",
          fire_key_gate.backend_name(), frame_id);
        prev_fire_allowed = f_now;
      }
    }

    // 显示副本缩放：相机 1440×1080 时，imshow 的窗口会比笔记本屏幕还大，
    // 看起来像"HUD 比整个屏幕还大"。这里只缩「显示用的副本」，解算仍用原始 img。
    // 想再小/再大就改这两个数（1280×720 是笔记本 1366×768 屏能完整放下的上限）。
    cv::Mat shown;
    const double disp_w_max = 1280.0;
    const double disp_h_max = 720.0;
    const double disp_scale =
      std::min({1.0, disp_w_max / img.cols, disp_h_max / img.rows});
    if (disp_scale < 1.0) {
      cv::resize(
        img, shown, cv::Size(), disp_scale, disp_scale,
        disp_scale < 0.75 ? cv::INTER_AREA : cv::INTER_LINEAR);
    } else {
      shown = img;
    }
    cv::imshow("task2", shown);
    const int key_code = cv::waitKey(1);
    if (key_code == 'q') {
      break;
    }
    // 按 s 随时把当前汇总打出来（不用退出也能看这一场的中间状态）
    if (key_code == 's' || key_code == 'S') {
      tools::logger()->info("[task2] （手动触发汇总）");
      test_summary.print();
    }
    // F 键的兜底通道：只有在既没有 evdev、又不是终端的时候才起作用
    if (fire_key_gate.uses_window_fallback()) fire_key_gate.feed(key_code);

    // Your code end
  }

  // 测试汇总：退出时（按 q 或 Ctrl+C）统一输出 A/B/C 三组的关键数值。
  // 目的：这些数散落在启动日志、arrive/fire-gate 行、lat CSV 里，几百行日志翻着找很容易漏项。
  // DSH_SUMMARY_FILE 设了的话，汇总还会另存一份独立文件（不依赖 tee 捕获）。
  test_summary.print();

  // 退出前主动交出控制权
  gimbal.send(false, false, 0.0f, 0.0f);
  return 0;
}
