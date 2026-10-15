#pragma once

#include "g1_arm/types.hpp"
#include <filesystem>
#include <fstream>
#include <optional>

namespace g1_arm {
class Logger {
public:
  explicit Logger(const std::filesystem::path &directory);
  void waist(const State &state, const WaistQ &max_abs_error,
             double extra_pitch_bias,
             double acquire_progress, double release_progress,
             const WaistQ &commanded_reference);
  void event(const std::string &name, const State &state,
             const StabilityMetrics &metrics,
             const std::optional<ArmQ> &arm_reference = std::nullopt,
             const std::optional<WaistQ> &waist_reference = std::nullopt,
             const std::optional<WaistQ> &max_abs_error = std::nullopt);
  const std::filesystem::path &path() const { return path_; }
private:
  std::filesystem::path path_;
  std::ofstream stream_;
  std::filesystem::path events_path_;
  std::ofstream events_stream_;
};
} // namespace g1_arm
