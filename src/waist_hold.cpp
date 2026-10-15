#include "g1_arm/waist_hold.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_arm {
void WaistHold::capture(const State &state) {
  for (std::size_t i = 0; i < reference_.size(); ++i) {
    reference_[i] = state.q[12 + i];
    if (!std::isfinite(reference_[i])) throw std::runtime_error("invalid waist LowState");
  }
  captured_ = true;
}

void WaistHold::set_target(const WaistQ &target) {
  for (double value : target)
    if (!std::isfinite(value)) throw std::runtime_error("invalid waist target");
  target_ = target;
}

ControlFrame WaistHold::apply(ControlFrame frame, double acquire_progress,
                              double release_progress, double extra_pitch_bias) const {
  if (!captured_) throw std::runtime_error("waist reference not captured");
  const double acquire = std::clamp(acquire_progress, 0.0, 1.0);
  const double release = std::clamp(release_progress, 0.0, 1.0);
  const double bias = (.010 + extra_pitch_bias) * (1.0 - release) * acquire;
  for (std::size_t i = 0; i < 3; ++i) {
    const auto motor = 12 + i;
    if (target_)
      frame.q[motor] = reference_[i] + ((*target_)[i] - reference_[i]) * acquire * (1.0 - release);
    else
      frame.q[motor] = reference_[i] + (i == 2 ? bias : 0.0);
    frame.kp[motor] = i == 2 ? 120.0 : 80.0;
    frame.kd[motor] = 1.5;
  }
  return frame;
}

void WaistHold::observe(const State &state, double acquire_progress,
                        double release_progress, double extra_pitch_bias) {
  if (!captured_) return;
  const double acquire = std::clamp(acquire_progress, 0.0, 1.0);
  const double release = std::clamp(release_progress, 0.0, 1.0);
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(state.q[12 + i])) continue;
    const double q_ref = target_
        ? reference_[i] + ((*target_)[i] - reference_[i]) * acquire * (1.0 - release)
        : reference_[i] + (i == 2 ? (.010 + extra_pitch_bias) * acquire * (1.0 - release) : 0.0);
    max_abs_error_[i] = std::max(max_abs_error_[i],
                                std::abs(state.q[12 + i] - q_ref));
  }
}
} // namespace g1_arm
