#include "monitor_signal.hpp"

#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <filesystem>
#include <system_error>
#include <vector>

#include "tools/logger.hpp"

namespace tools
{
namespace
{
constexpr std::size_t kBitsPerWord = sizeof(unsigned long) * 8;
constexpr std::size_t kKeyWords = KEY_MAX / kBitsPerWord + 1;

bool test_bit(const std::array<unsigned long, kKeyWords> & bits, unsigned int bit)
{
  return (bits[bit / kBitsPerWord] & (1UL << (bit % kBitsPerWord))) != 0;
}
}  // namespace

PowerButton::PowerButton()
{
  std::error_code ec;
  std::filesystem::directory_iterator entries("/dev/input", ec);
  if (ec) {
    tools::logger()->warn("PowerButton: Failed to open /dev/input: {}", ec.message());
    return;
  }

  for (const auto & entry : entries) {
    const auto name = entry.path().filename().string();
    if (name.rfind("event", 0) != 0) continue;

    int fd = open(entry.path().c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) continue;

    std::array<unsigned long, kKeyWords> key_bits{};
    if (
      ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits.data()) >= 0 &&
      test_bit(key_bits, KEY_POWER)) {
      fds_.push_back(fd);
      tools::logger()->info("PowerButton: Monitoring {}", entry.path().string());
    } else {
      ::close(fd);
    }
  }

  if (fds_.empty()) {
    tools::logger()->warn("PowerButton: No readable KEY_POWER input device");
    return;
  }

  thread_ = std::thread(&PowerButton::run, this);
}

PowerButton::~PowerButton()
{
  stop_.store(true, std::memory_order_release);
  if (thread_.joinable()) thread_.join();
  for (int fd : fds_) ::close(fd);
}

bool PowerButton::pressed() const { return pressed_.load(std::memory_order_acquire); }

void PowerButton::run()
{
  std::vector<pollfd> poll_fds;
  poll_fds.reserve(fds_.size());
  for (int fd : fds_) poll_fds.push_back({fd, POLLIN, 0});

  while (!stop_.load(std::memory_order_acquire)) {
    int ret = poll(poll_fds.data(), poll_fds.size(), 100);
    if (ret <= 0) continue;

    for (auto & item : poll_fds) {
      if ((item.revents & POLLIN) == 0) continue;

      input_event event{};
      while (read(item.fd, &event, sizeof(event)) == sizeof(event)) {
        if (event.type == EV_KEY && event.code == KEY_POWER && event.value == 1) {
          pressed_.store(true, std::memory_order_release);
          tools::logger()->warn("PowerButton: Power key pressed, request graceful exit");
          return;
        }
      }
    }
  }
}

}  // namespace tools
