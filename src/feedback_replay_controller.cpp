#include "g1_arm/feedback_replay_controller.hpp"

#include "g1_arm/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace g1_arm {
FeedbackReplayController::FeedbackReplayController(const ProcessedTrajectory &trajectory,
    const ArmQ &acquire_q, RetimerConfig retimer_config, FeedbackConfig feedback_config)
    : trajectory_(trajectory), retimer_(trajectory, retimer_config), config_(feedback_config),
      acquire_q_(acquire_q), node_reached_(trajectory.nodes.size(), false),
      node_errors_(trajectory.nodes.size(), 0.0),
      node_joint_errors_(trajectory.nodes.size()) {
  if (!(config_.position_tolerance_rad > 0 && config_.stable_dq_rad_s >= 0 &&
        config_.stable_window_s > 0 && config_.node_timeout_s > 0 &&
        config_.error_normal_rad > 0 && config_.error_pause_rad > config_.error_normal_rad &&
        config_.error_filter_tau_s > 0 && config_.speed_recovery_rad_s > 0))
    throw std::runtime_error("invalid feedback replay configuration");
  node_reached_[0] = true;
  important_nodes_.push_back(0);
  for (std::size_t i = 1; i + 1 < trajectory.nodes.size(); ++i)
    if (trajectory.nodes[i].important) important_nodes_.push_back(i);
  if (important_nodes_.back() != trajectory.nodes.size() - 1)
    important_nodes_.push_back(trajectory.nodes.size() - 1);
  next_node_ = 1;
}

FeedbackStep FeedbackReplayController::step(const State &state, double dt) {
  if (!(dt > 0 && dt <= .1)) throw std::runtime_error("invalid feedback control period");
  if (!abort_reason_.empty()) throw std::runtime_error(abort_reason_);
  FeedbackStep result;
  elapsed_s_ += dt;
  const auto processed_q = sample_processed(trajectory_, progress_);
  const bool initial_frame = progress_ == 0.0 && sample_count_ == 0;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    if (!std::isfinite(state.q[15 + i]) || !std::isfinite(state.dq[15 + i]))
      throw std::runtime_error("invalid arm LowState in feedback replay");
    result.q_ref[i] = initial_frame ? acquire_q_[i]
        : acquire_q_[i] + processed_q[i] - trajectory_.recording_start_q[i];
    result.error[i] = result.q_ref[i] - state.q[15 + i];
    result.error_max = std::max(result.error_max, std::abs(result.error[i]));
    squared_error_sum_[i] += result.error[i] * result.error[i];
  }
  ++sample_count_;
  max_tracking_error_ = std::max(max_tracking_error_, result.error_max);
  const double alpha = 1.0 - std::exp(-dt / config_.error_filter_tau_s);
  filtered_error_ += alpha * (result.error_max - filtered_error_);

  bool node_reached_this_step = false;
  const std::size_t target_node = next_node_ < important_nodes_.size()
      ? important_nodes_[next_node_] : trajectory_.nodes.size() - 1;
  if (next_node_ < important_nodes_.size() &&
      progress_ >= trajectory_.nodes[target_node].s - .015) {
    progress_ = std::max(progress_, trajectory_.nodes[target_node].s);
    node_wait_s_ += dt;
    const auto target = trajectory_.nodes[target_node].q;
    double max_error = 0.0, max_dq = 0.0;
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double q_target = acquire_q_[i] + target[i] - trajectory_.recording_start_q[i];
      node_joint_errors_[target_node][i] = q_target - state.q[15 + i];
      max_error = std::max(max_error, std::abs(node_joint_errors_[target_node][i]));
      max_dq = std::max(max_dq, std::abs(state.dq[15 + i]));
    }
    if (max_error <= config_.position_tolerance_rad && max_dq <= config_.stable_dq_rad_s)
      node_stable_s_ += dt;
    else
      node_stable_s_ = 0.0;
    if (node_stable_s_ >= config_.stable_window_s) {
      node_reached_[target_node] = true;
      node_errors_[target_node] = max_error;
      ++next_node_;
      node_wait_s_ = node_stable_s_ = 0.0;
      node_reached_this_step = true;
    } else if (node_wait_s_ >= config_.node_timeout_s) {
      node_errors_[target_node] = max_error;
      abort_reason_ = "NOT_REACHED node " + std::to_string(target_node) +
                      " error=" + std::to_string(max_error);
      result.aborted = true;
    }
  }

  if (next_node_ < important_nodes_.size() &&
      progress_ >= trajectory_.nodes[important_nodes_[next_node_]].s - .015) {
    const auto &node = trajectory_.nodes[important_nodes_[next_node_]];
    progress_ = node.s;
    for (std::size_t i = 0; i < kArmDof; ++i)
      result.q_ref[i] = acquire_q_[i] + node.q[i] - trajectory_.recording_start_q[i];
  }

  double factor = 1.0;
  if (filtered_error_ >= config_.error_pause_rad) factor = 0.0;
  else if (filtered_error_ > config_.error_normal_rad)
    factor = (config_.error_pause_rad - filtered_error_) /
             (config_.error_pause_rad - config_.error_normal_rad);
  if (node_reached_this_step) factor = 0.0;
  const double desired_speed = retimer_.nominal_speed(progress_) * factor;
  if (factor == 0.0) speed_ = 0.0;
  else speed_ = retimer_.accelerate(speed_, desired_speed, dt);
  if (next_node_ < important_nodes_.size() &&
      progress_ + speed_ * dt >= trajectory_.nodes[important_nodes_[next_node_]].s)
    progress_ = trajectory_.nodes[important_nodes_[next_node_]].s;
  else
    progress_ = retimer_.advance(progress_, speed_, dt, 1.0);

  result.progress_s = progress_;
  result.speed_scale = retimer_.nominal_speed(progress_) > 1e-9
      ? speed_ / retimer_.nominal_speed(progress_) : 0.0;
  result.node_index = next_node_ < important_nodes_.size()
      ? important_nodes_[next_node_] : trajectory_.nodes.size() - 1;
  result.node_reached = node_reached_this_step;
  result.node_wait_s = node_wait_s_;
  if (progress_ >= 1.0) {
    for (std::size_t i = 0; i < kArmDof; ++i) {
      const double final_target = acquire_q_[i] + trajectory_.nodes.back().q[i] -
                                  trajectory_.recording_start_q[i];
      const double error = final_target - state.q[15 + i];
      result.error[i] = error;
      result.error_max = std::max(result.error_max, std::abs(error));
    }
  }
  result.done = progress_ >= 1.0 && next_node_ >= important_nodes_.size();
  return result;
}

std::array<double, kArmDof> FeedbackReplayController::rms_error() const {
  std::array<double, kArmDof> result{};
  if (!sample_count_) return result;
  for (std::size_t i = 0; i < kArmDof; ++i)
    result[i] = std::sqrt(squared_error_sum_[i] / sample_count_);
  return result;
}

std::size_t FeedbackReplayController::current_node() const {
  return next_node_ < important_nodes_.size()
      ? important_nodes_[next_node_] : trajectory_.nodes.size() - 1;
}

std::string write_feedback_csv_header() {
  std::ostringstream out;
  out << "timestamp,dt_s,progress_s,speed_scale,node_index,node_reached,node_wait_s,e_max";
  for (int i = 0; i < 14; ++i) out << ",m" << i + 15 << "_q,m" << i + 15
                                   << "_dq,m" << i + 15 << "_q_ref,m" << i + 15 << "_error";
  return out.str();
}

void write_feedback_csv_row(std::ostream &out, const std::string &timestamp,
                            const FeedbackStep &step, const State &state, double dt) {
  out << std::setprecision(10) << timestamp << ',' << dt << ',' << step.progress_s << ','
      << step.speed_scale << ',' << step.node_index << ',' << step.node_reached << ','
      << step.node_wait_s << ',' << step.error_max;
  for (std::size_t i = 0; i < kArmDof; ++i)
    out << ',' << state.q[15 + i] << ',' << state.dq[15 + i] << ','
        << step.q_ref[i] << ',' << step.error[i];
  out << '\n';
}
} // namespace g1_arm
