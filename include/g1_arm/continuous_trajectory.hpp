#pragma once

#include "g1_arm/types.hpp"

#include <filesystem>

namespace g1_arm {
struct ContinuousSample {
  double t_s{0.0};
  ArmQ q{};
  ArmQ dq{};
};

struct ContinuousTrajectory {
  std::string name;
  std::string recorded_at;
  ArmQ recording_start_q{};
  std::vector<ContinuousSample> samples;
};

std::filesystem::path continuous_trajectory_path(
    const std::filesystem::path &directory, const std::string &name);
void validate_continuous_trajectory(const ContinuousTrajectory &trajectory);
void save_continuous_trajectory(const ContinuousTrajectory &trajectory,
                                const std::filesystem::path &path);
ContinuousTrajectory load_continuous_trajectory(const std::filesystem::path &path);
ArmQ interpolate_continuous(const ContinuousTrajectory &trajectory, double t_s,
                            const ArmQ &acquire_q, double motion_scale);
ArmQ relative_continuous_target(const ArmQ &acquire_q, const ArmQ &recorded_q,
                                const ArmQ &recording_start_q, double motion_scale);

class ContinuousRecorder {
public:
  void start(TeachMode mode, const State &state, Clock::time_point now,
             std::string recorded_at);
  void stop() { recording_ = false; }
  void capture(const State &state, Clock::time_point now);
  bool recording() const { return recording_; }
  const ContinuousTrajectory &trajectory() const { return trajectory_; }
  ContinuousTrajectory &trajectory() { return trajectory_; }
private:
  ContinuousTrajectory trajectory_;
  Clock::time_point started_at_{};
  bool recording_{false};
};
} // namespace g1_arm
