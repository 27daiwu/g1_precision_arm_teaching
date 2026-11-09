#include "g1_arm/pose.hpp"
#include "g1_arm/trajectory.hpp"

#include <cmath>
#include <fstream>
#include <json/json.h>
#include <stdexcept>

namespace g1_arm {
void validate_pose_name(const std::string &name) {
  if (name.empty() || name.size() > 128 || name == "." || name == ".." ||
      name.find_first_of("/\\") != std::string::npos)
    throw std::runtime_error("pose name must be 1..128 bytes without path separators");
  for (unsigned char ch : name)
    if (ch <= 32 || ch == 127)
      throw std::runtime_error("pose name must not contain whitespace or control characters");
}

std::filesystem::path save_measured_pose(
    const std::string &name, const State &state,
    const std::filesystem::path &directory, bool replace,
    const PoseContext &context) {
  validate_pose_name(name);
  const auto now = Clock::now();
  if (state.received_at == Clock::time_point{} || state.received_at > now ||
      now - state.received_at > std::chrono::milliseconds(250))
    throw std::runtime_error("cannot save pose: missing or stale LowState (>250ms)");

  static constexpr const char *names[] = {
      "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
      "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
      "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
      "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"};
  Json::Value pose;
  pose["schema_version"] = 1;
  pose["pose_name"] = name;
  pose["recorded_at"] = timestamp_local();
  pose["units"] = "rad";
  pose["velocity_units"] = "rad/s";
  pose["source_topic"] = "rt/lowstate";
  pose["capture_mode"] = context.teaching ? "teach" : "read_only";
  pose["feedback_age_s"] = std::chrono::duration<double>(now - state.received_at).count();
  for (int motor = 12; motor <= 28; ++motor) {
    validate_joint_position(motor, state.q[motor]);
    if (!std::isfinite(state.dq[motor])) throw std::runtime_error("non-finite pose velocity");
    pose["motor_indices"].append(motor);
    pose["joint_names"].append(names[motor - 12]);
    pose["q"].append(state.q[motor]);
    pose["dq"].append(state.dq[motor]);
  }
  pose["waist_control"]["mode"] = context.teaching ? "WaistHold_fixed" : "external_or_unknown";
  pose["waist_control"]["pitch_static_bias_rad"] = context.teaching ? Json::Value(.010) : Json::Value();
  pose["waist_control"]["pitch_extra_bias_rad"] = context.teaching
      ? Json::Value(context.waist_pitch_extra_bias) : Json::Value();
  pose["waist_control"]["reference_q"] = Json::nullValue;
  if (context.teaching) {
    pose["waist_control"]["reference_q"] = Json::arrayValue;
    for (double value : context.waist_reference)
      pose["waist_control"]["reference_q"].append(value);
  }
  pose["calibration_refs"]["paper"] = Json::nullValue;
  pose["calibration_refs"]["tcp"] = Json::nullValue;
  pose["tcp_pose"] = Json::nullValue;

  const auto path = directory / (name + ".json");
  if (!replace && std::filesystem::exists(path))
    throw std::runtime_error("pose already exists; use R " + name + " to replace it");
  if (!directory.empty()) std::filesystem::create_directories(directory);
  const auto temporary = path.string() + ".tmp";
  std::ofstream out(temporary, std::ios::trunc);
  if (!out) throw std::runtime_error("cannot write pose: " + temporary);
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "  ";
  out << Json::writeString(writer, pose) << '\n';
  out.close();
  if (!out) throw std::runtime_error("failed writing pose: " + temporary);
  std::filesystem::rename(temporary, path);
  return path;
}
} // namespace g1_arm
