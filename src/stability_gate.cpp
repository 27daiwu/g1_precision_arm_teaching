#include "g1_arm/stability_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_arm {
StabilityGate::StabilityGate(StabilityConfig config) : config_(config) {
  if (!std::isfinite(config_.window_s) || config_.window_s <= 0.0 ||
      !std::isfinite(config_.wait_timeout_s) || config_.wait_timeout_s <= 0.0 ||
      !std::isfinite(config_.filter_tau_s) || config_.filter_tau_s <= 0.0 ||
      !std::isfinite(config_.gyro_limit_rad_s) || config_.gyro_limit_rad_s <= 0.0 ||
      !std::isfinite(config_.waist_dq_limit_rad_s) || config_.waist_dq_limit_rad_s <= 0.0 ||
      !std::isfinite(config_.leg_dq_rms_limit_rad_s) || config_.leg_dq_rms_limit_rad_s <= 0.0 ||
      !std::isfinite(config_.leg_dq_peak_limit_rad_s) || config_.leg_dq_peak_limit_rad_s <= 0.0)
    throw std::runtime_error("invalid stability gate configuration");
}

StabilityMetrics StabilityGate::observe(const State &state) {
  if (state.received_at == Clock::time_point{})
    throw std::runtime_error("LowState has no receive timestamp");
  double gyro_sq = 0.0;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    if (!std::isfinite(state.imu_gyro[axis])) throw std::runtime_error("invalid IMU gyro");
    gyro_sq += state.imu_gyro[axis] * state.imu_gyro[axis];
  }
  const double gyro = std::sqrt(gyro_sq);
  double waist_peak = 0.0;
  for (int motor = 12; motor <= 14; ++motor) {
    if (!std::isfinite(state.dq[motor])) throw std::runtime_error("invalid waist dq");
    waist_peak = std::max(waist_peak, std::abs(state.dq[motor]));
  }
  double leg_sq = 0.0;
  double leg_peak = 0.0;
  for (int motor = 0; motor < 12; ++motor) {
    if (!std::isfinite(state.dq[motor])) throw std::runtime_error("invalid leg dq");
    const double speed = std::abs(state.dq[motor]);
    leg_sq += speed * speed;
    leg_peak = std::max(leg_peak, speed);
  }
  StabilityMetrics raw{gyro, waist_peak, std::sqrt(leg_sq / 12.0), leg_peak};
  if (!initialized_) {
    filtered_ = raw;
    initialized_ = true;
    last_sample_ = state.received_at;
  } else {
    double dt = std::chrono::duration<double>(state.received_at - last_sample_).count();
    if (dt <= 0.0) return filtered_;
    dt = std::min(dt, .25);
    const double alpha = 1.0 - std::exp(-dt / config_.filter_tau_s);
    filtered_.gyro_rad_s += alpha * (raw.gyro_rad_s - filtered_.gyro_rad_s);
    filtered_.waist_dq_rad_s += alpha * (raw.waist_dq_rad_s - filtered_.waist_dq_rad_s);
    filtered_.leg_dq_rms_rad_s += alpha * (raw.leg_dq_rms_rad_s - filtered_.leg_dq_rms_rad_s);
    filtered_.leg_dq_peak_rad_s += alpha * (raw.leg_dq_peak_rad_s - filtered_.leg_dq_peak_rad_s);
    last_sample_ = state.received_at;
  }

  const bool under_limits = filtered_.gyro_rad_s <= config_.gyro_limit_rad_s &&
      filtered_.waist_dq_rad_s <= config_.waist_dq_limit_rad_s &&
      filtered_.leg_dq_rms_rad_s <= config_.leg_dq_rms_limit_rad_s &&
      filtered_.leg_dq_peak_rad_s <= config_.leg_dq_peak_limit_rad_s;
  if (!under_limits) {
    if (++over_limit_samples_ >= 3) stable_since_.reset();
  } else {
    over_limit_samples_ = 0;
    if (!stable_since_) stable_since_ = state.received_at;
  }
  return filtered_;
}

double StabilityGate::stable_duration_s(Clock::time_point now) const {
  if (!stable_since_) return 0.0;
  return std::chrono::duration<double>(now - *stable_since_).count();
}
} // namespace g1_arm
