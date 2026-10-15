#pragma once

#include "g1_arm/types.hpp"

#include <optional>

namespace g1_arm {
constexpr double kMaxWaistPitchExtraBiasRad = 0.05;

class WaistHold {
public:
  void capture(const State &state);
  void set_target(const WaistQ &target);
  ControlFrame apply(ControlFrame frame, double acquire_progress,
                    double release_progress = 0.0,
                    double extra_pitch_bias = 0.0) const;
  const WaistQ &reference() const { return reference_; }
  void observe(const State &state, double acquire_progress = 1.0,
               double release_progress = 0.0, double extra_pitch_bias = 0.0);
  WaistQ max_abs_error() const { return max_abs_error_; }
private:
  WaistQ reference_{};
  WaistQ max_abs_error_{};
  std::optional<WaistQ> target_;
  bool captured_{false};
};
} // namespace g1_arm
