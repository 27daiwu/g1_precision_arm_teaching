#pragma once

#include "g1_arm/types.hpp"
#include <filesystem>
#include <json/value.h>

namespace g1_arm {
// Teaching metadata describes the command context, never an offset to saved q.
struct PoseContext {
  bool teaching{false};
  double waist_pitch_extra_bias{0.0};
  WaistQ waist_reference{};
};

void validate_pose_name(const std::string &name);
std::filesystem::path save_measured_pose(
    const std::string &name, const State &state,
    const std::filesystem::path &directory, bool replace = false,
    const PoseContext &context = {});
} // namespace g1_arm
