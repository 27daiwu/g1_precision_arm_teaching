#pragma once

#include "g1_arm/types.hpp"

namespace g1_arm {
class WaistHold {
public:
  void capture(const State &state);
  ControlFrame apply(ControlFrame frame, double acquire_progress,
                    double release_progress = 0.0) const;
  const WaistQ &reference() const { return reference_; }
  void observe(const State &state);
  WaistQ max_abs_error() const { return max_abs_error_; }
private:
  WaistQ reference_{};
  WaistQ max_abs_error_{};
  bool captured_{false};
};
} // namespace g1_arm
