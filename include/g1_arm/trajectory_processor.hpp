#pragma once

#include "g1_arm/continuous_trajectory.hpp"

namespace g1_arm {
struct ProcessedNode {
  double s{0.0};
  ArmQ q{};
  bool important{false};
};

struct ProcessedTrajectory {
  std::string name;
  std::string recorded_at;
  ArmQ recording_start_q{};
  std::vector<ProcessedNode> nodes;
  double source_duration_s{0.0};
  double max_geometric_deviation_rad{0.0};
};

struct ProcessorConfig {
  double max_gap_s{0.25};
  double noise_floor_rad{0.0005};
  double smoothing_weight{0.15};
  double resample_step_rad{0.01};
  double important_angle_rad{0.035};
  double pause_speed_rad_s{0.015};
  double max_geometry_error_rad{0.01};
};

ProcessedTrajectory process_trajectory(const ContinuousTrajectory &source,
                                       const ProcessorConfig &config = {});
ArmQ sample_processed(const ProcessedTrajectory &trajectory, double s);
void save_processed_trajectory(const ProcessedTrajectory &trajectory,
                               const std::filesystem::path &path);
} // namespace g1_arm
