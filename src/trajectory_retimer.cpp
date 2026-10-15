#include "g1_arm/trajectory_retimer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_arm {
TrajectoryRetimer::TrajectoryRetimer(const ProcessedTrajectory &trajectory,
                                     RetimerConfig config)
    : trajectory_(trajectory), config_(config) {
  if (trajectory_.nodes.size() < 2 || !std::isfinite(config_.speed_scale) ||
      config_.speed_scale <= 0 || config_.max_joint_velocity_rad_s <= 0 ||
      config_.max_joint_acceleration_rad_s2 <= 0 || config_.junction_slowdown_s <= 0)
    throw std::runtime_error("invalid trajectory retimer configuration");
  node_speed_.resize(trajectory_.nodes.size(), config_.speed_scale);
  const double nominal_progress_speed = config_.speed_scale /
      std::max(trajectory_.source_duration_s, .02);
  std::fill(node_speed_.begin(), node_speed_.end(), nominal_progress_speed);
  double max_path_derivative = 0.0;
  for (std::size_t i = 1; i < trajectory_.nodes.size(); ++i) {
    const auto &a = trajectory_.nodes[i - 1];
    const auto &b = trajectory_.nodes[i];
    const double ds = b.s - a.s;
    if (ds <= 0) continue;
    double max_dq_ds = 0.0;
    for (std::size_t j = 0; j < kArmDof; ++j)
      max_dq_ds = std::max(max_dq_ds, std::abs(b.q[j] - a.q[j]) / ds);
    max_path_derivative = std::max(max_path_derivative, max_dq_ds);
    const double joint_limited = max_dq_ds > 1e-9
        ? config_.max_joint_velocity_rad_s / max_dq_ds : config_.speed_scale;
    const double segment_speed = std::min(nominal_progress_speed, joint_limited);
    node_speed_[i - 1] = std::min(node_speed_[i - 1], segment_speed);
    node_speed_[i] = std::min(node_speed_[i], segment_speed);
    if (b.important && i + 1 < trajectory_.nodes.size()) node_speed_[i] = 0.0;
    duration_s_ += ds / std::max(segment_speed, 1e-6);
  }
  progress_acceleration_ = config_.max_joint_acceleration_rad_s2 /
      std::max(max_path_derivative, 1e-6);
  for (std::size_t i = 1; i + 1 < node_speed_.size(); ++i) {
    if (!trajectory_.nodes[i].important) continue;
    node_speed_[i] = 0.0;
    const double ds = config_.junction_slowdown_s;
    if (i > 0) node_speed_[i - 1] = std::min(node_speed_[i - 1],
        std::sqrt(2.0 * progress_acceleration_ * ds));
    if (i + 1 < node_speed_.size()) node_speed_[i + 1] = std::min(node_speed_[i + 1],
        std::sqrt(2.0 * progress_acceleration_ * ds));
  }
}

double TrajectoryRetimer::nominal_speed(double s) const {
  const auto &nodes = trajectory_.nodes;
  const auto upper = std::upper_bound(nodes.begin(), nodes.end(), s,
      [](double value, const ProcessedNode &node) { return value < node.s; });
  if (upper == nodes.begin()) return node_speed_.front();
  if (upper == nodes.end()) return node_speed_.back();
  const auto index = static_cast<std::size_t>(upper - nodes.begin());
  const auto &left = nodes[index - 1];
  const auto &right = nodes[index];
  const double u = (s - left.s) / std::max(right.s - left.s, 1e-9);
  return node_speed_[index - 1] + u * (node_speed_[index] - node_speed_[index - 1]);
}

double TrajectoryRetimer::advance(double s, double speed, double dt,
                                  double speed_factor) const {
  if (!std::isfinite(dt) || dt < 0 || !std::isfinite(speed_factor))
    throw std::runtime_error("invalid retimer step");
  const double next = s + std::max(0.0, speed) * std::clamp(speed_factor, 0.0, 1.0) * dt;
  return std::clamp(next, s, 1.0);
}

double TrajectoryRetimer::accelerate(double speed, double desired_speed, double dt) const {
  const double delta = progress_acceleration_ * std::max(0.0, dt);
  return speed + std::clamp(desired_speed - speed, -delta, delta);
}
} // namespace g1_arm
