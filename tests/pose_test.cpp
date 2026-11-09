#include "g1_arm/pose.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <limits>
#include <stdexcept>
#include <unistd.h>

namespace {
void require(bool result, const char *message) {
  if (!result) throw std::runtime_error(message);
}
template <typename F> void rejected(F action) {
  bool failed = false;
  try { action(); } catch (const std::runtime_error &) { failed = true; }
  require(failed, "invalid capture unexpectedly accepted");
}
Json::Value read(const std::filesystem::path &path) {
  std::ifstream input(path);
  Json::Value value;
  Json::CharReaderBuilder builder;
  std::string errors;
  require(Json::parseFromStream(builder, input, &value, &errors), "invalid saved JSON");
  return value;
}
}

int main() {
  using namespace g1_arm;
  const auto directory = std::filesystem::temp_directory_path() /
      ("g1_pose_test_" + std::to_string(getpid()));
  try {
    State state;
    for (int motor = 12; motor <= 28; ++motor) {
      state.q[motor] = .001 * motor;
      state.dq[motor] = -.002 * motor;
    }
    state.received_at = Clock::now();
    const auto path = save_measured_pose("ready", state, directory);
    const auto pose = read(path);
    require(pose["schema_version"].asInt() == 1, "legacy schema version changed");
    require(pose["units"].asString() == "rad", "incorrect angle units");
    require(pose["capture_mode"].asString() == "read_only", "incorrect capture mode");
    require(pose["source_topic"].asString() == "rt/lowstate", "incorrect feedback topic");
    require(pose["motor_indices"].size() == 17 && pose["q"].size() == 17 &&
            pose["dq"].size() == 17 && pose["joint_names"].size() == 17, "incorrect DoF");
    for (int i = 0; i < 17; ++i) {
      require(pose["motor_indices"][i].asInt() == 12 + i, "motor mapping incorrect");
      require(pose["q"][i].asDouble() == state.q[12 + i], "q is not measured absolute position");
      require(pose["dq"][i].asDouble() == state.dq[12 + i], "dq is not measured velocity");
    }
    require(pose["joint_names"][2].asString() == "waist_pitch_joint" &&
            pose["joint_names"][3].asString() == "left_shoulder_pitch_joint" &&
            pose["joint_names"][10].asString() == "right_shoulder_pitch_joint" &&
            pose["joint_names"][16].asString() == "right_wrist_yaw_joint", "joint groups incorrect");
    require(pose["waist_control"]["pitch_extra_bias_rad"].isNull() &&
            pose["waist_control"]["reference_q"].isNull(), "invented external control metadata");

    rejected([&] { save_measured_pose("ready", state, directory); });
    require(read(path) == pose, "duplicate save changed existing pose");
    state.q[15] += .1;
    state.received_at = Clock::now();
    save_measured_pose("ready", state, directory, true);
    require(read(path)["q"][3].asDouble() == state.q[15], "explicit replacement failed");
    // A measured waist angle at the limit is valid; do not add the controller bias.
    state.q[14] = .52;
    PoseContext context{true, .05, {.01, .02, .03}};
    save_measured_pose("taught", state, directory, false, context);
    const auto taught = read(directory / "taught.json");
    require(taught["q"][2].asDouble() == .52, "waist bias applied to measured q");
    require(taught["waist_control"]["pitch_extra_bias_rad"].asDouble() == .05 &&
            taught["waist_control"]["reference_q"][2].asDouble() == .03, "teaching context missing");

    const auto valid = state;
    state.received_at = Clock::now() - std::chrono::seconds(1);
    rejected([&] { save_measured_pose("stale", state, directory); });
    state.received_at = {};
    rejected([&] { save_measured_pose("missing", state, directory); });
    state = valid;
    state.q[28] = std::numeric_limits<double>::quiet_NaN();
    rejected([&] { save_measured_pose("nan", state, directory); });
    state = valid;
    state.dq[12] = std::numeric_limits<double>::infinity();
    rejected([&] { save_measured_pose("inf", state, directory); });
    state = valid;
    state.q[14] = .53;
    rejected([&] { save_measured_pose("limit", state, directory); });
    rejected([&] { save_measured_pose("../outside", valid, directory); });
    rejected([&] { save_measured_pose("..", valid, directory); });
    rejected([&] { save_measured_pose("", valid, directory); });
    require(!std::filesystem::exists(directory / "limit.json") &&
            !std::filesystem::exists(directory / "stale.json"), "invalid capture left an output file");
    std::filesystem::remove_all(directory);
    std::cout << "17-DoF pose capture offline checks passed\n";
    return 0;
  } catch (const std::exception &error) {
    std::filesystem::remove_all(directory);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
