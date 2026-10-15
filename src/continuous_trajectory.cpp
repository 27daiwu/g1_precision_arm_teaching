#include "g1_arm/continuous_trajectory.hpp"
#include "g1_arm/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <json/json.h>
#include <stdexcept>

namespace g1_arm {
namespace {
Json::Value json_array(const ArmQ &values) {
  Json::Value result(Json::arrayValue);
  for (double value : values) result.append(value);
  return result;
}
ArmQ parse_arm(const Json::Value &value, const char *field) {
  if (!value.isArray() || value.size() != kArmDof)
    throw std::runtime_error(std::string("invalid continuous ") + field + " length");
  ArmQ result{};
  for (Json::ArrayIndex i = 0; i < value.size(); ++i) {
    if (!value[i].isNumeric() || !std::isfinite(value[i].asDouble()))
      throw std::runtime_error(std::string("invalid continuous ") + field + " value");
    result[i] = value[i].asDouble();
  }
  return result;
}
void check_arm_limits(const ArmQ &q) {
  Trajectory single;
  single.name = "continuous";
  single.recorded_at = "continuous";
  single.waist_reference = {0.0, 0.0, 0.0};
  Point point;
  point.name = "A";
  point.recorded_at = "continuous";
  point.q = q;
  single.points.push_back(point);
  validate_trajectory(single);
}
}

std::filesystem::path continuous_trajectory_path(const std::filesystem::path &directory,
                                                 const std::string &name) {
  validate_name(name);
  return directory / (name + "_continuous.json");
}

void validate_continuous_trajectory(const ContinuousTrajectory &trajectory) {
  validate_name(trajectory.name);
  if (trajectory.samples.empty()) throw std::runtime_error("continuous trajectory has no samples");
  check_arm_limits(trajectory.recording_start_q);
  double previous = -1.0;
  for (const auto &sample : trajectory.samples) {
    if (!std::isfinite(sample.t_s) || sample.t_s < 0.0 || sample.t_s <= previous)
      throw std::runtime_error("continuous sample times must be finite and strictly increasing");
    previous = sample.t_s;
    check_arm_limits(sample.q);
    for (double velocity : sample.dq)
      if (!std::isfinite(velocity)) throw std::runtime_error("non-finite continuous joint velocity");
  }
}

void save_continuous_trajectory(const ContinuousTrajectory &trajectory,
                                const std::filesystem::path &path) {
  validate_continuous_trajectory(trajectory);
  Json::Value root;
  root["version"] = 1;
  root["kind"] = "continuous";
  root["name"] = trajectory.name;
  root["recorded_at"] = trajectory.recorded_at;
  root["units"] = "rad";
  root["time_units"] = "s_monotonic_relative";
  root["motor_ids"] = Json::arrayValue;
  for (int motor = 15; motor <= 28; ++motor) root["motor_ids"].append(motor);
  root["recording_start_q"] = json_array(trajectory.recording_start_q);
  root["samples"] = Json::arrayValue;
  for (const auto &sample : trajectory.samples) {
    Json::Value item;
    item["t_s"] = sample.t_s;
    item["q"] = json_array(sample.q);
    item["dq"] = json_array(sample.dq);
    root["samples"].append(item);
  }
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.string() + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create continuous trajectory: " + temporary);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  output << Json::writeString(builder, root) << '\n';
  output.close();
  if (!output) throw std::runtime_error("failed writing continuous trajectory: " + temporary);
  std::filesystem::rename(temporary, path);
}

ContinuousTrajectory load_continuous_trajectory(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open continuous trajectory: " + path.string());
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  if (!Json::parseFromStream(builder, input, &root, &error))
    throw std::runtime_error("invalid continuous trajectory JSON: " + error);
  if (root["version"].asInt() != 1 || root["kind"].asString() != "continuous" ||
      !root["motor_ids"].isArray() || root["motor_ids"].size() != kArmDof ||
      !root["samples"].isArray())
    throw std::runtime_error("unsupported continuous trajectory format");
  for (Json::ArrayIndex i = 0; i < kArmDof; ++i)
    if (root["motor_ids"][i].asInt() != static_cast<int>(15 + i))
      throw std::runtime_error("continuous trajectory motor mapping must be 15..28");
  ContinuousTrajectory trajectory;
  trajectory.name = root["name"].asString();
  trajectory.recorded_at = root.get("recorded_at", "").asString();
  trajectory.recording_start_q = parse_arm(root["recording_start_q"], "start q");
  for (const auto &item : root["samples"]) {
    if (!item["t_s"].isNumeric()) throw std::runtime_error("invalid continuous sample time");
    trajectory.samples.push_back({item["t_s"].asDouble(), parse_arm(item["q"], "q"),
                                  parse_arm(item["dq"], "dq")});
  }
  validate_continuous_trajectory(trajectory);
  return trajectory;
}

ArmQ relative_continuous_target(const ArmQ &acquire_q, const ArmQ &recorded_q,
                                const ArmQ &recording_start_q, double motion_scale) {
  if (!std::isfinite(motion_scale) || motion_scale < 0.0)
    throw std::runtime_error("motion scale must be finite and non-negative");
  ArmQ result{};
  for (std::size_t i = 0; i < result.size(); ++i)
    result[i] = acquire_q[i] + motion_scale * (recorded_q[i] - recording_start_q[i]);
  return result;
}

void ContinuousRecorder::start(TeachMode mode, const State &state, Clock::time_point now,
                               std::string recorded_at) {
  if (recording_) throw std::runtime_error("continuous recording is already active");
  if (mode != TeachMode::Drag)
    throw std::runtime_error("continuous recording can start only in DRAG mode");
  trajectory_ = {};
  trajectory_.name = "continuous";
  trajectory_.recorded_at = std::move(recorded_at);
  for (std::size_t i = 0; i < kArmDof; ++i) {
    trajectory_.recording_start_q[i] = state.q[15 + i];
  }
  started_at_ = now;
  recording_ = true;
  capture(state, now);
}

void ContinuousRecorder::capture(const State &state, Clock::time_point now) {
  if (!recording_) return;
  const double elapsed = std::chrono::duration<double>(now - started_at_).count();
  ContinuousSample sample;
  sample.t_s = elapsed;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    sample.q[i] = state.q[15 + i];
    sample.dq[i] = state.dq[15 + i];
  }
  if (trajectory_.samples.empty() || elapsed > trajectory_.samples.back().t_s)
    trajectory_.samples.push_back(sample);
}

ArmQ interpolate_continuous(const ContinuousTrajectory &trajectory, double t_s,
                            const ArmQ &acquire_q, double motion_scale) {
  if (!std::isfinite(t_s)) throw std::runtime_error("invalid replay time");
  if (trajectory.samples.empty()) throw std::runtime_error("continuous trajectory has no samples");
  const auto &samples = trajectory.samples;
  if (t_s <= samples.front().t_s) return acquire_q;
  if (t_s >= samples.back().t_s)
    return relative_continuous_target(acquire_q, samples.back().q,
                                     trajectory.recording_start_q, motion_scale);
  const auto upper = std::upper_bound(samples.begin(), samples.end(), t_s,
      [](double time, const ContinuousSample &sample) { return time < sample.t_s; });
  const auto &right = *upper;
  const auto &left = *(upper - 1);
  const double phase = (t_s - left.t_s) / (right.t_s - left.t_s);
  ArmQ recorded{};
  for (std::size_t i = 0; i < recorded.size(); ++i)
    recorded[i] = left.q[i] + phase * (right.q[i] - left.q[i]);
  return relative_continuous_target(acquire_q, recorded, trajectory.recording_start_q,
                                    motion_scale);
}
} // namespace g1_arm
