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

ControlFrame WaistHold::apply(ControlFrame frame, double acquire_progress,
                              double release_progress) const {
  if (!captured_) throw std::runtime_error("waist reference not captured");
  const double acquire = std::clamp(acquire_progress, 0.0, 1.0);
  const double release = std::clamp(release_progress, 0.0, 1.0);
  const double bias = .010 * (1.0 - release) * acquire;
  for (std::size_t i = 0; i < 3; ++i) {
    const auto motor = 12 + i;
    frame.q[motor] = reference_[i] + (i == 2 ? bias : 0.0);
    frame.kp[motor] = i == 2 ? 120.0 : 80.0;
    frame.kd[motor] = 1.5;
  }
  return frame;
}

void WaistHold::observe(const State &state) {
  if (!captured_) return;
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(state.q[12 + i])) continue;
    max_abs_error_[i] = std::max(max_abs_error_[i],
                                std::abs(state.q[12 + i] - reference_[i]));
  }
}
} // namespace g1_arm
