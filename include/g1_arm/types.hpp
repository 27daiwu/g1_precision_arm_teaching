#pragma once

#include <array>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace g1_arm {
constexpr std::size_t kArmDof = 14;
constexpr std::size_t kWaistDof = 3;
using ArmQ = std::array<double, kArmDof>;
using WaistQ = std::array<double, kWaistDof>;
using Clock = std::chrono::steady_clock;

struct State {
  std::array<double, 35> q{};
  std::array<double, 35> dq{};
  std::array<double, 3> imu_gyro{};
  std::array<double, 3> imu_quaternion{};
  std::uint8_t mode_machine{};
  Clock::time_point received_at{};
};

struct StabilityConfig {
  double window_s{1.0};
  double wait_timeout_s{10.0};
  double filter_tau_s{0.20};
  double gyro_limit_rad_s{0.30};
  double waist_dq_limit_rad_s{0.15};
  double leg_dq_rms_limit_rad_s{0.30};
  double leg_dq_peak_limit_rad_s{0.80};
};

struct StabilityMetrics {
  double gyro_rad_s{0.0};
  double waist_dq_rad_s{0.0};
  double leg_dq_rms_rad_s{0.0};
  double leg_dq_peak_rad_s{0.0};
};

struct Point {
  std::string name;
  std::string recorded_at;
  ArmQ q{};
  double hold_s{0.0};
};

struct Trajectory {
  std::string name;
  std::string recorded_at;
  WaistQ waist_reference{};
  std::vector<Point> points;
};

enum class TeachMode { Drag, Lock };

class TeachState {
public:
  explicit TeachState(const ArmQ &initial);
  void drag(const ArmQ &measured);
  void lock(const ArmQ &measured, Clock::time_point now);
  ArmQ update(const ArmQ &measured);
  ArmQ reference() const { return reference_; }
  std::array<double, 14> gains(Clock::time_point now) const;
  TeachMode mode() const { return mode_; }
private:
  ArmQ reference_{};
  TeachMode mode_{TeachMode::Lock};
  Clock::time_point gain_ramp_start_{};
  std::array<double, 14> ramp_from_{};
};

struct ControlFrame {
  std::array<double, 35> q{};
  std::array<double, 35> kp{};
  std::array<double, 35> kd{};
  double weight{0.0};
};
} // namespace g1_arm
