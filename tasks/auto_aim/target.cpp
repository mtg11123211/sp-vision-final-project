#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace
{
// ============ EKF 调参常量（原 target.cpp 里 Task_2 / Task_3 TODO 指的就是这里） ============
//
// 过程噪声用的是「分段白噪声模型」，Q 的结构是
//     [ dt 的 4 次方 除 4 , dt 的 3 次方 除 2 ]
//     [ dt 的 3 次方 除 2 , dt 的平方          ]  再乘一个方差
// 所以这里的方差就是「加速度标准差」和「角加速度标准差」的平方。
//
// 取值锚点来自 Lecture5 自带的 rerun 仿真（同一个课程给的量级，最好交代）：
//     main_translation.py : SIGMA_VA = 1  米每二次方秒   -> 线加速度方差 1
//     main_rotation.py    : SIGMA_W  = 4  弧度每二次方秒 -> 角加速度方差 16
// 车体在赛场上会有启停和换向，比仿真里的匀速往返更剧烈，所以线加速度方差取 4（标准差 2）。
// 角加速度方差取课堂原值 16（标准差 4），这样拟合出来的角速度不会被量测噪声带得乱抖，
// 而任务二、任务三的评分恰恰只看这条角速度曲线。
//
// 前哨站转速恒定（稳定后约 2.5 弧度每秒）、位置固定，角加速度基本为 0，
// 所以过程噪声要比车体小两个数量级，否则滤波器会一直跟着单帧量测抖。
//
// 现场调参：改这四个数重编译即可。判断依据用 Plotter 上的
//   ekf/residual_yaw、ekf/residual_distance、ekf/nis 三条曲线，
// 残差应当零均值、nis 应当围绕量测维数 4 上下波动，长期偏大说明 Q 太小或 R 太大。
constexpr double kV1Car = 4.0;       // 车体线加速度方差，标准差 2 米每二次方秒
constexpr double kV2Car = 16.0;      // 车体角加速度方差，标准差 4 弧度每二次方秒
constexpr double kV1CarZ = 1.0;      // 车体竖直方向线加速度方差，标准差 1 米每二次方秒
constexpr double kV1Outpost = 0.01;  // 前哨站线加速度方差，标准差 0.1 米每二次方秒
constexpr double kV2Outpost = 0.01;  // 前哨站角加速度方差，标准差 0.1 弧度每二次方秒

// ============ 量测噪声 R ============
// 这一组是框架给的经验值，本次没有改动，取值理由记录在这里，便于答辩时交代：
//   r_cam_yaw / r_cam_pitch：像素级重投影误差折算到角度。配置里的标定重投影误差是 0.1343 像素，
//       按焦距约 1809 像素折算，角度标准差约 0.00007 弧度，这里给 0.0005 是留了余量。
//   r_distance：随距离平方增长，再乘一个与斜视程度有关的因子（板越斜，距离越不可信）。
//   r_armor_yaw：随距离平方增长并加一个 0.2 除 距离 的项，3 米处约 0.207，
//       标准差约 0.45 弧度，反映「斜视时 PnP 解不准板朝向」这一事实。
constexpr double kRCamYaw = 5e-4;
constexpr double kRCamPitch = 5e-4;
}  // namespace

Target::Target(
  const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num)
: name(armor.name),
  armor_type(armor.type),
  jumped(false),
  last_id(0),
  isinit(true),
  armor_num_(armor_num > 0 ? armor_num : (armor.name == ArmorName::outpost ? 3 : 4)),
  switch_count_(0),
  update_count_(0),
  is_switch_(false),
  is_converged_(false),
  t_(t)
{
  auto r = radius > 0.0 ? radius : (armor.name == ArmorName::outpost ? 0.2765 : 0.2);
  priority = armor.priority;
  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  // 旋转中心的坐标
  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  // x vx y vy z vz a w r l h
  // a: angle
  // w: angular velocity
  // tips for outpost:
  // l: dh1 for outpost / r2 - r1 for vehicles
  // h: dh2 for outpost / z2 - z1 for vehicles
  Eigen::VectorXd x0{{center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0}};  //初始化预测量

  // 车体：半径、长短轴半径差、高度差都是小量，先给一个小的初始不确定度，允许换板后微调
  Eigen::VectorXd P0_dig_car{
    {1, 64, 1, 64, 1, 64, 0.4, 100, 0.001, 0.001,
     0.001}};

  // 前哨站：半径与分层高度差都是图纸定值，几何量比车体可信得多，所以方差给得更小。
  // 图纸（前哨站示意图）上三块悬挂的中部装甲板互成 120 度、中心圆直径 Φ553 毫米，
  // 即半径 276.5 毫米；把前视图按红色灯条标注的 571 / 1110 毫米定标后量得
  // 左右两块可见中部板的中心高度差约 99 毫米，与下面的 0.1 米一致。
  Eigen::VectorXd P0_dig_outpost{
    {1, 64, 1, 64, 1, 64, 0.4, 100, 0.0001, 0.0001,
     0.0001}};

  Eigen::VectorXd P0_dig = P0_dig_car;
  if (is_layered_outpost()) {
    P0_dig = P0_dig_outpost;
    // 前哨站三块中部装甲板是台阶式布置的：0 号在基准高度，1 号加 dh1，2 号再加 dh2。
    // 图纸上量得相邻两块约差 0.1 米，所以 dh1 = dh2 = 0.1。
    // 注意：这两个量在 update_ypda 里被冻结（前哨站几何列不做量测更新），
    // 所以如果现场实测发现图纸值与实物不符，要改的是这两个初值，而不是等滤波器自己学。
    // 2026-10-25 现场卷尺实测：相邻两级台阶 0.10 米，与图纸一致 ⇒ 缺省 0.1 正确。
    // （早期曾按落点反推得到 0.23 米，是错的：那次落点偏差来自 id 锚点错位与俯仰零位，
    //  不是层高。层高一旦按 0.23 设，中/高板会分别偏高 0.13 / 0.26 米。）
    // 现场仍可用 DSH_OUTPOST_DH 覆盖，免得每换一个值都重编。
    static const double outpost_dh = []() {
      const char * v = std::getenv("DSH_OUTPOST_DH");
      return v ? std::atof(v) : 0.1;
    }();
    x0[9] = outpost_dh;   // dh1
    x0[10] = outpost_dh;  // dh2
  }

  Eigen::Matrix<double, 11, 11> P0 = P0_dig.asDiagonal();

  // 防止夹角求和出现异常值
  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = tools::limit_rad(c[6]);
    return c;
  };

  ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);  //初始化滤波器（预测量、预测量协方差）
}

void Target::predict(std::chrono::steady_clock::time_point t)
{
  auto dt = tools::delta_time(t, t_);
  predict(dt);
  t_ = t;
}

void Target::predict(double dt)
{
  // 状态转移矩阵
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };
  // clang-format on

  // Piecewise White Noise Model
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  //
  // 车体与前哨站要用两套过程噪声：前哨站固定不动、转速恒定，
  // 用同样的「目标随时可能大幅加速」假设会让角速度估计跟着单帧量测乱抖。
  double v1, v2, v1_z;
  if (is_layered_outpost()) {
    v1 = kV1Outpost;
    v2 = kV2Outpost;
    v1_z = kV1Outpost;
  } else {
    v1 = kV1Car;
    v2 = kV2Car;
    v1_z = kV1CarZ;
  }

  auto a = dt * dt * dt * dt / 4;
  auto b = dt * dt * dt / 2;
  auto c = dt * dt;
  // 预测过程噪声偏差的方差
  // clang-format off
  Eigen::MatrixXd Q{
    {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a * v1_z, b * v1_z,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b * v1_z, c * v1_z,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  // 防止夹角求和出现异常值
  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = tools::limit_rad(x_prior[6]);
    return x_prior;
  };

  ekf_.predict(F, Q, f);
}

void Target::update(const Armor & armor)
{
  // 装甲板匹配
  // id 必须有初值：一旦量测里出现 NaN，下面的比较全部为假，id 就会带着不确定值往下走
  int id = 0;
  auto min_angle_error = 1e10;
  const std::vector<Eigen::Vector4d> & xyza_list = armor_xyza_list();

  std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
  for (int i = 0; i < armor_num_; i++) {
    xyza_i_list.push_back({xyza_list[i], i});
  }

  std::sort(
    xyza_i_list.begin(), xyza_i_list.end(),
    [](const std::pair<Eigen::Vector4d, int> & a, const std::pair<Eigen::Vector4d, int> & b) {
      Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
      Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
      return ypd1[2] < ypd2[2];
    });

  // 取前3个distance最小的装甲板
  for (int i = 0; i < 3; i++) {
    const auto & xyza = xyza_i_list[i].first;
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
    auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                       std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

    if (std::abs(angle_error) < std::abs(min_angle_error)) {
      id = xyza_i_list[i].second;
      min_angle_error = angle_error;
    }
  }

  if (id != 0) jumped = true;

  if (id != last_id) {
    is_switch_ = true;
    rlh_initialized_ = true;
  } else {
    is_switch_ = false;
  }

  if (is_switch_) switch_count_++;

  last_id = id;
  update_count_++;
  update_convergency();

  update_ypda(armor, id);
}

void Target::update_ypda(const Armor & armor, int id)
{
  //观测jacobi
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  H.block<3, 1>(0, 6).setZero();

  // 首次换板前冻结几何参数；车体换板后允许单板观测更新半径及高度差。
  //
  // 前哨站例外：它的半径与分层高度差是图纸定值（半径 0.2765 米、相邻台阶 0.1 米），
  // 而且一旦换过板 rlh_initialized_ 就永远为真，单板量测噪声会把 dh1、dh2 慢慢带离图纸值。
  // 所以前哨站全程冻结几何列。如果现场实测发现图纸值与实物不符，
  // 应该改的是构造函数里的 x0[9]、x0[10] 初值，而不是放开这里让滤波器去学。
  if (!rlh_initialized_ || is_layered_outpost()) {
    H.block<4, 3>(0, 8).setZero();
  }
  // 随距自适应量测噪声
  const double center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  const double delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
  const double distance = armor.ypd_in_world[2];
  const double distance_sq = distance * distance;

  // 量测噪声取值理由见文件开头的注释块（框架给的经验值，本次未改动）
  const double r_cam_yaw = kRCamYaw;
  const double r_cam_pitch = kRCamPitch;
  const double r_distance = (1e-2 + 5e-3 * distance_sq) * (std::log(std::abs(delta_angle) + 1) + 1);
  const double r_armor_yaw = 5e-2 + 1e-2 * distance_sq + 2e-1 / distance;

  Eigen::MatrixXd R = Eigen::Vector4d(r_cam_yaw, r_cam_pitch, r_distance, r_armor_yaw).asDiagonal();

  // 定义非线性转换函数h: x -> z
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
    auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // 防止夹角求差出现异常值
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = tools::limit_rad(c[0]);
    c[1] = tools::limit_rad(c[1]);
    c[3] = tools::limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};  //获得观测量

  ekf_.update(z, H, R, h, z_subtract);
}

bool Target::update_checked(const Armor & armor, double nis_reject_threshold)
{
  // 先备份状态与协方差，update() 内部会直接改 ekf_.x 与 ekf_.P
  const Eigen::VectorXd x_before = ekf_.x;
  const Eigen::MatrixXd P_before = ekf_.P;

  update(armor);

  // NIS 是「新息」按新息协方差归一化后的平方和，4 维观测下服从 4 自由度卡方分布。
  // 正常帧的 NIS 应该在观测量维数 4 附近波动，超过 16.27（4 自由度卡方分布的 99.7% 分位）
  // 说明这一帧观测
  // 与当前模型严重不符，多半是关联错了板，或者混进了不属于本模型的装甲板
  // （例如前哨站顶部的第四块板）。
  if (ekf_.last_nis > nis_reject_threshold) {
    ekf_.x = x_before;
    ekf_.P = P_before;
    // 这一帧的失败标记已经进了 EKF 内部「最近 NIS 失败率」的滑动窗口，但观测被拒收、
    // 状态也已回滚，它不该继续算在失败率里。否则连续几帧拒收就会把失败率顶到 1.0，
    // 而这个窗口长 100 帧（58 帧每秒约 1.7 秒），stable 会一直为假 —— 现场表现是
    // 「有目标、云台也跟得住，但就是不请求开火」。这里把刚压进去的那个 1 退掉。
    auto & recent = ekf_.recent_nis_failures;
    if (!recent.empty()) {
      recent.pop_back();
      int failures = 0;
      for (int flag : recent) failures += flag;
      ekf_.data["recent_nis_failures"] =
        recent.empty() ? 0.0 : static_cast<double>(failures) / static_cast<double>(recent.size());
      ekf_.data["nis_fail"] = 0.0;
    }
    // 关联结果 last_id 保留：这一帧看到的确实是最接近那块板，只是位置不可信，
    // 下一帧继续按同一块板去预测即可。
    tools::logger()->debug("[Target] NIS {:.2f} over threshold, measurement rejected", ekf_.last_nis);
    return false;
  }
  return true;
}

double Target::armor_z_max() const
{
  const std::vector<Eigen::Vector4d> plates = armor_xyza_list();
  double z_max = -1e10;
  for (const auto & plate : plates) {
    if (plate[2] > z_max) z_max = plate[2];
  }
  return z_max;
}

Eigen::VectorXd Target::ekf_x() const { return ekf_.x; }

const tools::ExtendedKalmanFilter & Target::ekf() const { return ekf_; }

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const
{
  std::vector<Eigen::Vector4d> _armor_xyza_list;

  for (int i = 0; i < armor_num_; i++) {
    auto angle = tools::limit_rad(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
    Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
    _armor_xyza_list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return _armor_xyza_list;
}

bool Target::diverged() const
{
  auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;

  // 前哨站三块板共用同一半径，高度差是分层量，单独判两个高度差是否合理
  if (is_layered_outpost()) {
    // 注意要判正负：只用 abs 的话台阶可以翻过来（本来高的一层跑到下面），滤波器却认为没发散
    auto dh1_ok = ekf_.x[9] > 0.0 && ekf_.x[9] < 0.55;
    auto dh2_ok = ekf_.x[10] > 0.0 && ekf_.x[10] < 0.55;
    if (r_ok && dh1_ok && dh2_ok) return false;
    tools::logger()->debug("[Target] r={:.3f}, dh1={:.3f}, dh2={:.3f}", ekf_.x[8], ekf_.x[9], ekf_.x[10]);
    return true;
  }

  // 车体：l 是「长短轴半径差」，物理上非负。初值取 0（未知），所以要允许 0，
  // 只挡住明显为负的情况（原来只判 r + l 的范围，l 可以一直掉到 -0.15 才被发现）。
  auto l_ok = ekf_.x[9] > -0.02 && ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

  if (r_ok && l_ok) return false;

  tools::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
  return true;
}

void Target::update_convergency() { is_converged_ = !diverged(); }

bool Target::is_converged() const { return is_converged_; }

bool Target::convergened()
{
  update_convergency();
  return is_converged();
}

bool Target::is_layered_outpost() const { return name == ArmorName::outpost && armor_num_ == 3; }

// 计算出装甲板中心的坐标（考虑长短轴）
Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_outpost_h = is_layered_outpost();

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);

  double dh = 0.0;  // 高度差
  if (use_outpost_h) {
    // 前哨站三块板阶梯分层：0 号在基准高度，1 号加 dh1，2 号再加 dh2
    if (id == 1) dh = x[9];
    else if (id == 2) dh = x[9] + x[10];
  } else {
    // 车体：只有 1、3 号板在另一层，高度差为 z2 - z1
    dh = use_l_h ? x[10] : 0.0;
  }

  auto armor_z = x[4] + dh;  //task_3 tips

  return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd & x, int id) const
{
  auto angle = tools::limit_rad(x[6] + id * 2 * CV_PI / armor_num_);
  auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);
  auto use_outpost_h = is_layered_outpost();

  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);

  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

  auto dz_dh = use_l_h ? 1.0 : 0.0;
  auto dz_dl = 0.0;
  if (use_outpost_h) {
    // z 对 l(dh1) 的偏导：1、2 号板受影响
    dz_dl = (id >= 1) ? 1.0 : 0.0;
    // z 对 h(dh2) 的偏导：只有 2 号板受影响
    dz_dh = (id >= 2) ? 1.0 : 0.0;
  }

  // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0, dz_dl, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
  // clang-format on

  Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
  Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);
  // clang-format off
  Eigen::MatrixXd H_armor_ypda{
    {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
    {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
    {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
    {                0,                 0,                 0, 1}
  };
  // clang-format on

  return H_armor_ypda * H_armor_xyza;
}

bool Target::checkinit() { return isinit; }

}  // namespace auto_aim