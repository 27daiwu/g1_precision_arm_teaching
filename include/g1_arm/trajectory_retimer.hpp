#pragma once

#include "g1_arm/trajectory_processor.hpp"

namespace g1_arm {
struct RetimerConfig {
  double speed_scale{0.25};
  double max_joint_velocity_rad_s{0.7};
  double max_joint_acceleration_rad_s2{1.5};
  double junction_slowdown_s{0.025};
};

class TrajectoryRetimer {
public:
  TrajectoryRetimer(const ProcessedTrajectory &trajectory, RetimerConfig config = {});
  double nominal_speed(double s) const;
  double advance(double s, double speed, double dt, double speed_factor) const;
  double accelerate(double speed, double desired_speed, double dt) const;
  double max_progress() const { return 1.0; }
  double duration_s() const { return duration_s_; }
private:
  const ProcessedTrajectory &trajectory_;
  RetimerConfig config_;
  std::vector<double> node_speed_;
  double duration_s_{0.0};
  double progress_acceleration_{0.0};
};
} // namespace g1_arm
