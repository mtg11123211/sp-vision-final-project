// 单独验证「按住 F」这个按键门，不依赖工程里的任何东西（不用 OpenCV / Eigen / 相机）。
//
// 车上先跑这个，确认按键通道是通的，再去跑 task_2：
//   cd ~/fp_aim && g++ -O2 -std=c++17 -I. tools/keytest.cpp -o /tmp/keytest && /tmp/keytest
// （-I. 必须带：keytest.cpp 里是 #include "src/fire_key.hpp"，
//   而 fire_key.hpp 在 src/ 下，不在 tools/ 的同级目录里）
//
// 期望输出：
//   backend = evdev:/dev/input/event3 /dev/input/event4      ← 车机上最常见
//   [+] F 按下
//   [-] F 松开
//
// 如果 backend 是 cvkey(window)：说明没读到 evdev 也没读到终端，程序只能靠
// OpenCV 窗口的按键（要求窗口有焦点），按住的效果靠键盘自动重复近似，延迟约 0.5 s。
// 这种情况请改用 DSH_FIRE_KEY=toggle（按一次切一次），或者用 DSH_FIRE_KEY_DEV 指定设备。

#include <chrono>
#include <cstdio>
#include <thread>

#include "src/fire_key.hpp"

int main()
{
  fire_key::Gate gate;
  std::printf("backend = %s\n", gate.backend_name().c_str());
  std::printf("提示：按住 F 看下面这行怎么变；Ctrl+C 退出\n");

  bool last = gate.allowed();
  std::printf("[%s] F=%s\n", last ? "允许开火" : "禁止开火", last ? "按住" : "松开");

  while (true) {
    gate.poll();
    const bool held = gate.allowed();
    if (held != last) {
      last = held;
      std::printf("[%s] F=%s\n", last ? "允许开火" : "禁止开火", last ? "按住" : "松开");
      std::fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
