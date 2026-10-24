#ifndef SRC__FIRE_KEY_HPP
#define SRC__FIRE_KEY_HPP

// 「人按住 F 才允许开火」的按键门（人工确认开火）。
//
// 为什么要这个门：
//   任务二每个转速挡位只有 10 发额度，虚发也占额度；而「板面正对 + 云台到位」这种
//   完美窗口只有几十毫秒，自动判断偶尔会踩空、偶尔会打偏。按住 F 的语义是：
//   **程序一直在跟踪、一直在解算，但只有人认为可以打了才真正扣扳机**，
//   把"什么时候扣扳机"的决定权交回给人，程序只负责"这一枪瞄得准不准"。
//
// 取键方式按可靠性自动降级：
//   1) Linux evdev（/dev/input/event*）：直接读输入设备，能同时拿到「按下」和「松开」
//      两个事件，所以 held() 是**真按住**；而且不需要窗口有焦点——车机常是远程/X11
//      转发，没有焦点时 cv::waitKey 根本收不到键。
//   2) Windows GetAsyncKeyState：同样是真按住、不需要焦点。
//   3) 兜底：终端 raw 模式 / OpenCV 窗口按键。这两种只能拿到「按下的那一帧」，
//      靠键盘自动重复近似"按住"，首次重复前有约 500 ms 的空档（X11 默认）。
//
// 环境变量：
//   DSH_FIRE_KEY=hold    （默认）必须按住 F 才允许开火
//   DSH_FIRE_KEY=toggle  按一次 F 切换开火许可（适合鼠标/手柄/远程操作）
//   DSH_FIRE_KEY=always  永远允许开火（等于把这个门关掉）
//   DSH_FIRE_KEY=never   永远不允许开火（调试用）
//   DSH_FIRE_KEY_DEV=/dev/input/event3   指定用哪个 evdev 设备
//
// 本文件不依赖工程里的任何头文件，可以单独编译，方便在车上先单独验证按键门。

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#if defined(__linux__) && __has_include(<linux/input.h>)
#include <linux/input.h>
#include <sys/ioctl.h>
#define DSH_FIRE_KEY_HAS_EVDEV 1
#endif
#endif

namespace fire_key
{

enum class Mode
{
  Hold,    // 必须按住
  Toggle,  // 按一次切一次
  Always,  // 永远允许
  Never    // 永远不允许
};

inline Mode parse_mode(const char * env)
{
  if (env == nullptr || env[0] == '\0') return Mode::Hold;
  std::string v(env);
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) { return std::tolower(c); });
  if (v == "toggle" || v == "t" || v == "flip") return Mode::Toggle;
  if (v == "always" || v == "on" || v == "true" || v == "yes") return Mode::Always;
  if (v == "never" || v == "off" || v == "false" || v == "no") return Mode::Never;
  // 认不出来的取值一律按「必须按住 F」处理：宁可少打，不要乱打。
  // 注意「开关这个门」用 DSH_FIRE_KEY_ENABLE，不要拿 1/0 塞进 DSH_FIRE_KEY。
  return Mode::Hold;
}

// 模式的**纯 ASCII** 名字：这个值会进屏幕（HUD 状态行 / 徽章），
// 而 cv::putText 的 Hershey 字体不支持中文 ⇒ 这里必须英文，中文解释放日志里。
inline const char * mode_name(Mode m)
{
  switch (m) {
    case Mode::Hold:
      return "hold";
    case Mode::Toggle:
      return "toggle";
    case Mode::Always:
      return "always";
    case Mode::Never:
      return "never";
  }
  return "unknown";
}

class Gate
{
public:
  enum class Backend
  {
    Always,
    Never,
    Win,
    Evdev,
    Termios,
    CvKey,
    None
  };

  Gate()
  // last_press_ 留默认值（epoch）＝「还没按过」。别写成 now()：那会让启动后 600 ms 内
  // CvKey 通道的 (now - last_press_) 落到 [0,600) 里，pressed_ 恒为真、门一直开着。
  : mode_(parse_mode(std::getenv("DSH_FIRE_KEY"))), last_press_()
  {
    if (mode_ == Mode::Always) {
      backend_ = Backend::Always;
      return;
    }
    if (mode_ == Mode::Never) {
      backend_ = Backend::Never;
      return;
    }
    init_backend();
  }

  ~Gate() { cleanup(); }

  // 这一帧是否允许开火
  bool allowed()
  {
    switch (mode_) {
      case Mode::Always:
        return true;
      case Mode::Never:
        return false;
      case Mode::Hold:
        return pressed_;
      case Mode::Toggle:
        return latch_;
    }
    return false;
  }

  // OpenCV 窗口按键的兜底通道：主循环末尾把 cv::waitKey 的返回值喂进来。
  // 只有在没有更好的后端时才起作用。
  void feed(int key)
  {
    if (key != 'f' && key != 'F') return;
    const auto now = std::chrono::steady_clock::now();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_press_).count();
    if (ms < 200) return;  // 键盘自动重复，200 ms 内只算一次按下
    last_press_ = now;
    pressed_ = true;
    edge_pending_ = true;
  }

  // 每帧调用一次：刷新按键状态，并把「按下」事件转成 toggle 的翻转
  void poll()
  {
    bool edge = false;
    switch (backend_) {
      case Backend::Win:
        edge = query_win();
        break;
      case Backend::Evdev:
        edge = query_evdev();
        break;
      case Backend::Termios:
        edge = query_termios();
        break;
      case Backend::CvKey: {
        // 窗口按键只能靠自动重复维持"按住"：最后一次按下后 600 ms 内算按住
        const auto ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                                 last_press_)
            .count();
        pressed_ = (ms >= 0 && ms < 600);
        break;
      }
      default:
        break;
    }
    if (edge) edge_pending_ = true;
    if (edge) edge_taken_ = true;
    if (mode_ == Mode::Toggle && edge_pending_) {
      latch_ = !latch_;
      last_edge_ms_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                              last_press_)
          .count();
    }
    edge_pending_ = false;
  }

  // 按键后端的名字，启动时打一行日志，便于现场确认到底用的哪条通道
  std::string backend_name() const
  {
    switch (backend_) {      case Backend::Always:
        return "always";
      case Backend::Never:
        return "never";
      case Backend::Win:
        return "win:GetAsyncKeyState";
      case Backend::Evdev:
        return "evdev:" + evdev_paths_;
      case Backend::Termios:
        return "termios(raw)";
      case Backend::CvKey:
        return "cvkey(window)";
      default:
        return "none";
    }
  }

  bool latched() const { return latch_; }
  bool pressed() const { return pressed_; }
  long long last_edge_ms() const { return last_edge_ms_; }
  const char * mode_text() const { return mode_name(mode_); }

  // ---- 给 HUD 徽章用的三个状态（2026-10-21 评审修订 8）----
  // pressed()  : 原始键态（hold 模式下就等于 allowed()）
  // latch()    : toggle 模式的锁存态（锁存 ≠ 按键，措辞要分开）
  // degraded() : 后端读键异常（设备热拔、权限变化、read 报错）⇒ 状态不可信，要显式告诉人
  bool degraded() const { return degraded_; }
  // 边沿事件（按下/释放各一次），取走即清。用来打一行 forensic 日志：
  // 事后对日志/录像时序、以及"到底按没按"的争议，靠这一行裁决。
  bool take_edge()
  {
    const bool e = edge_taken_;
    edge_taken_ = false;
    return e;
  }

  // 只有在"只能靠 OpenCV 窗口按键"的后端下，才需要把 cv::waitKey 的返回值喂进来。
  // 别的后端（evdev / 终端）已经能收到同一次按键，再喂一遍会在 toggle 模式下翻两次。
  bool uses_window_fallback() const { return backend_ == Backend::CvKey; }

private:
  // ---------------------------------------------------------------- 平台实现
  void init_backend()
  {
#if defined(_WIN32)
    backend_ = Backend::Win;
#elif defined(DSH_FIRE_KEY_HAS_EVDEV)
    if (init_evdev()) return;
    if (init_termios()) return;
    backend_ = Backend::CvKey;
#else
    if (init_termios()) return;  // 非 Linux 的 POSIX：只有终端这一条路
    backend_ = Backend::CvKey;
#endif
  }

  void cleanup()
  {
#if defined(DSH_FIRE_KEY_HAS_EVDEV)
    for (auto & d : devs_) {
      if (d.fd >= 0) ::close(d.fd);
    }
    devs_.clear();
#endif
#if !defined(_WIN32)
    restore_tty();
#endif
  }

  // Windows：GetAsyncKeyState 能拿到「当前是否按住」，不需要窗口焦点
  bool query_win()
  {
#if defined(_WIN32)
    // 注意：WinUser.h **没有** VK_F 这个宏（只有 VK_F1~VK_F12），
    // 用字符码 'F'（0x46）—— A~Z 的虚拟键码就等于 ASCII 码。
    const bool down = (::GetAsyncKeyState('F') & 0x8000) != 0;
    const bool edge = down && !prev_down_;
    prev_down_ = down;
    pressed_ = down;
    return edge;
#else
    return false;
#endif
  }

  // Linux evdev：读 /dev/input/event*，按下/松开都有
  bool init_evdev()
  {
#if defined(DSH_FIRE_KEY_HAS_EVDEV)
    std::vector<std::string> paths;
    const char * dev = std::getenv("DSH_FIRE_KEY_DEV");
    if (dev != nullptr && dev[0] != '\0') {
      paths.emplace_back(dev);
    } else {
      for (int i = 0; i < 32; ++i) {
        char p[64];
        std::snprintf(p, sizeof(p), "/dev/input/event%d", i);
        if (::access(p, R_OK) == 0) paths.emplace_back(p);
      }
    }
    for (const auto & path : paths) {
      const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
      if (fd < 0) continue;
      // 这个设备能不能发 KEY_F：查 EV_KEY 能力位就够了。
      // 注意：EVIOCGKEYCODE 是**不带参数**的宏（_IOR('E',0x04,int[2])），写成 EVIOCGKEYCODE(k)
      // 等于"把整数当函数调用"，GCC 直接报 expression cannot be used as a function。
      // 而且它查的是"扫描码→键码"映射表，也不是设备能力，本来就不该用在这里。
      unsigned char keybits[(KEY_MAX + 7) / 8] = {0};
      if (::ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keybits)), keybits) < 0) {
        ::close(fd);
        continue;
      }
      if (((keybits[KEY_F / 8] >> (KEY_F % 8)) & 1u) == 0) {
        ::close(fd);
        continue;
      }
      const int f_code = KEY_F;  // evdev 报的就是键码，query_evdev 直接拿它比 ev.code
      devs_.push_back(Dev{fd, f_code, false, path});
      evdev_paths_ += path + " ";
    }
    if (devs_.empty()) return false;
    backend_ = Backend::Evdev;
    return true;
#else
    return false;
#endif
  }

  bool query_evdev()
  {
#if defined(DSH_FIRE_KEY_HAS_EVDEV)
    bool edge = false;
    bool any_down = false;
    for (auto & d : devs_) {
      struct input_event ev;
      while (true) {
        const ssize_t n = ::read(d.fd, &ev, sizeof(ev));
        if (n == static_cast<ssize_t>(sizeof(ev))) {
          if (ev.type != EV_KEY || ev.code != d.code) continue;
          if (ev.value != 0) {
            if (!d.down) edge = true;
            d.down = true;
            last_press_ = std::chrono::steady_clock::now();
          } else {
            d.down = false;
          }
          continue;
        }
        // 没数据了：EAGAIN 是正常的；其它错误说明设备出问题（热拔/权限变化）
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
          degraded_ = true;  // 徽章要变黄，提示"后端异常"，别让人以为是自己没按
        }
        break;
      }
      any_down = any_down || d.down;
    }
    pressed_ = any_down;
    return edge;
#else
    return false;
#endif
  }

  // 终端 raw 模式：只能看到"按下"，按住靠键盘自动重复近似
  bool init_termios()
  {
#if !defined(_WIN32)
    if (!::isatty(STDIN_FILENO)) return false;
    if (::tcgetattr(STDIN_FILENO, &tty_old_) != 0) return false;
    tty_new_ = tty_old_;
    tty_new_.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    tty_new_.c_cc[VMIN] = 0;
    tty_new_.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &tty_new_) != 0) return false;
    tty_saved_ = true;
    backend_ = Backend::Termios;
    return true;
#else
    return false;
#endif
  }

  void restore_tty()
  {
#if !defined(_WIN32)
    if (tty_saved_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &tty_old_);
      tty_saved_ = false;
    }
#endif
  }

  bool query_termios()
  {
#if !defined(_WIN32)
    bool edge = false;
    char c = 0;
    while (::read(STDIN_FILENO, &c, 1) == 1) {
      if (c != 'f' && c != 'F') continue;
      const auto now = std::chrono::steady_clock::now();
      const auto ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_press_).count();
      if (ms < 200) continue;  // 自动重复，200 ms 内只算一次
      last_press_ = now;
      edge = true;
    }
    // 终端只能看到"按下"，按住靠自动重复维持：最后一次按下后 250 ms 内算按住
    const auto ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
                                                             last_press_)
        .count();
    pressed_ = (ms >= 0 && ms < 250);
    return edge;
#else
    return false;
#endif
  }

private:
  Mode mode_ = Mode::Hold;
  Backend backend_ = Backend::None;
  std::string evdev_paths_;
  bool pressed_ = false;
  bool edge_pending_ = false;
  bool edge_taken_ = false;
  bool degraded_ = false;
  bool latch_ = false;
  bool prev_down_ = false;
  long long last_edge_ms_ = 0;
  std::chrono::steady_clock::time_point last_press_{};

#if defined(DSH_FIRE_KEY_HAS_EVDEV)
  struct Dev
  {
    int fd;
    int code;
    bool down;
    std::string path;
  };
  std::vector<Dev> devs_;
#endif
#if !defined(_WIN32)
  struct termios tty_old_{};
  struct termios tty_new_{};
  bool tty_saved_ = false;
#endif
};

}  // namespace fire_key

#endif  // SRC__FIRE_KEY_HPP
