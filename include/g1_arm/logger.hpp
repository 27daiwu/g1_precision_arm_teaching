#pragma once

#include "g1_arm/types.hpp"
#include <filesystem>
#include <fstream>

namespace g1_arm {
class Logger {
public:
  explicit Logger(const std::filesystem::path &directory);
  void waist(const State &state, const WaistQ &reference,
             const WaistQ &max_abs_error);
  const std::filesystem::path &path() const { return path_; }
private:
  std::filesystem::path path_;
  std::ofstream stream_;
};
} // namespace g1_arm
