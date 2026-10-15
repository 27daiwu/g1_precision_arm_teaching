#pragma once

#include "g1_arm/types.hpp"

#include <json/value.h>

#include <filesystem>

namespace g1_arm {
constexpr std::size_t kCalligraphyDof = 17;
using CalligraphyQ = std::array<double, kCalligraphyDof>;

struct CalligraphySample {
  double t_monotonic{0.0};
  CalligraphyQ q{};
  CalligraphyQ dq{};
};

struct CalligraphyTrajectory {
  std::string character_name;
  std::string start_time;
  std::string paper_calibration_ref;
  std::string brush_tcp_ref;
  Json::Value brush_tcp_parameters{Json::objectValue};
  Json::Value hover_reference{Json::nullValue};
  CalligraphyQ start_joint_position{};
  CalligraphyQ end_joint_position{};
  std::vector<CalligraphySample> samples;
  double sample_rate_actual{0.0};
  std::size_t missed_samples{0};
  bool waist_pitch_extra_bias_known{false};
  double waist_pitch_extra_bias{0.0};
  std::string effective_waist_control_mode{"UNKNOWN"};
  Json::Value configuration_snapshot{Json::nullValue};
};

std::filesystem::path calligraphy_trajectory_path(
    const std::filesystem::path &directory, const std::string &character_name,
    const std::string &timestamp);
void validate_calligraphy_trajectory(const CalligraphyTrajectory &trajectory);
void save_calligraphy_trajectory(const CalligraphyTrajectory &trajectory,
                                 const std::filesystem::path &path);
CalligraphyTrajectory load_calligraphy_trajectory(const std::filesystem::path &path);
ArmQ calligraphy_arm_position(const CalligraphyQ &q17);
ArmQ interpolate_calligraphy_arm(const CalligraphyTrajectory &trajectory,
                                 double source_time_s);
bool calligraphy_waist_matches(const CalligraphyTrajectory &trajectory,
                               const WaistQ &actual, double tolerance_rad,
                               WaistQ *errors = nullptr);

class CalligraphyRecorder {
public:
  void start(const State &state, Clock::time_point now, std::string start_time);
  void stop(const State &state);
  void capture(const State &state, Clock::time_point now);
  bool recording() const { return recording_; }
  bool has_data() const { return !trajectory_.samples.empty(); }
  const CalligraphyTrajectory &trajectory() const { return trajectory_; }
  CalligraphyTrajectory &trajectory() { return trajectory_; }
private:
  CalligraphyTrajectory trajectory_;
  Clock::time_point started_at_{};
  Clock::time_point last_capture_{};
  bool recording_{false};
};
} // namespace g1_arm
