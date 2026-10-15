#pragma once

#include "g1_arm/trajectory_retimer.hpp"

#include <fstream>
#include <sstream>

namespace g1_arm {
struct FeedbackConfig {
  double position_tolerance_rad{0.02};
  double stable_dq_rad_s{0.04};
  double stable_window_s{0.20};
  double node_timeout_s{5.0};
  double error_normal_rad{0.04};
  double error_pause_rad{0.15};
  double error_filter_tau_s{0.12};
  double speed_recovery_rad_s{1.0};
};

struct FeedbackStep {
  ArmQ q_ref{};
  ArmQ error{};
  double error_max{0.0};
  double progress_s{0.0};
  double speed_scale{0.0};
  std::size_t node_index{0};
  bool node_reached{false};
  double node_wait_s{0.0};
  bool done{false};
  bool aborted{false};
};

class FeedbackReplayController {
public:
  FeedbackReplayController(const ProcessedTrajectory &trajectory, const ArmQ &acquire_q,
                           RetimerConfig retimer_config = {},
                           FeedbackConfig feedback_config = {});
  FeedbackStep step(const State &state, double dt);
  const std::vector<bool> &node_reached() const { return node_reached_; }
  const std::vector<double> &node_errors() const { return node_errors_; }
  const std::vector<ArmQ> &node_joint_errors() const { return node_joint_errors_; }
  double elapsed_s() const { return elapsed_s_; }
  double max_tracking_error() const { return max_tracking_error_; }
  const std::array<double, kArmDof> &squared_error_sum() const { return squared_error_sum_; }
  std::array<double, kArmDof> rms_error() const;
  std::size_t sample_count() const { return sample_count_; }
  double progress() const { return progress_; }
  std::size_t current_node() const;
  const std::string &abort_reason() const { return abort_reason_; }
private:
  const ProcessedTrajectory &trajectory_;
  TrajectoryRetimer retimer_;
  FeedbackConfig config_;
  ArmQ acquire_q_{};
  double progress_{0.0};
  double speed_{0.0};
  double filtered_error_{0.0};
  double elapsed_s_{0.0};
  double node_wait_s_{0.0};
  double node_stable_s_{0.0};
  std::size_t next_node_{1};
  std::vector<std::size_t> important_nodes_;
  std::vector<bool> node_reached_;
  std::vector<double> node_errors_;
  std::vector<ArmQ> node_joint_errors_;
  std::array<double, kArmDof> squared_error_sum_{};
  std::size_t sample_count_{0};
  double max_tracking_error_{0.0};
  std::string abort_reason_;
};

std::string write_feedback_csv_header();
void write_feedback_csv_row(std::ostream &out, const std::string &timestamp,
                            const FeedbackStep &step, const State &state,
                            double dt);
} // namespace g1_arm
