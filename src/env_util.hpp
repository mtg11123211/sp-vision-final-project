#ifndef SRC__ENV_UTIL_HPP
#define SRC__ENV_UTIL_HPP

// 环境变量读取小工具（task_2 / task_3 / low_plate 共用）。
// 单独一个头文件的原因：low_plate.hpp 是"策略"、fire_key.hpp 必须保持零工程依赖
// （keytest 要能单独编译），所以这两个小工具放哪儿都不合适 ⇒ 独立成文件。

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <string>

namespace policy
{

// 读 double；没设或为空 ⇒ 返回缺省值
inline double env_double(const char * name, double def)
{
  const char * v = std::getenv(name);
  if (v == nullptr || v[0] == '\0') return def;
  return std::atof(v);
}

// 读 bool；没设或为空 ⇒ 返回缺省值；认不出来的取值 ⇒ 回到缺省值（不猜）
inline bool env_bool(const char * name, bool def)
{
  const char * v = std::getenv(name);
  if (v == nullptr || v[0] == '\0') return def;
  std::string s(v);
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  if (s == "1" || s == "on" || s == "true" || s == "yes" || s == "enable") return true;
  if (s == "0" || s == "off" || s == "false" || s == "no" || s == "disable") return false;
  return def;
}

}  // namespace policy

#endif  // SRC__ENV_UTIL_HPP
