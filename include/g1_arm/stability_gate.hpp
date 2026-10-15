#pragma once

#include "g1_arm/types.hpp"
#include <optional>

namespace g1_arm {
class StabilityGate {
public:
  explicit StabilityGate(StabilityConfig config = {});
  StabilityMetrics observe(const State &state);
  bool stable() const { return stable_since_.has_value() && over_limit_samples_ == 0; }
  double stable_duration_s(Clock::time_point now) const;
  const StabilityMetrics &metrics() const { return filtered_; }
  const StabilityConfig &config() const { return config_; }
private:
  StabilityConfig config_;
  StabilityMetrics filtered_{};
  bool initialized_{false};
  Clock::time_point last_sample_{};
  std::optional<Clock::time_point> stable_since_;
  unsigned over_limit_samples_{0};
};
} // namespace g1_arm
