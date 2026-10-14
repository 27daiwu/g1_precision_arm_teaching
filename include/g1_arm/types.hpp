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
  std::uint8_t mode_machine{};
  Clock::time_point received_at{};
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
