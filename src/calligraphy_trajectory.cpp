#include "g1_arm/calligraphy_trajectory.hpp"
#include "g1_arm/trajectory.hpp"
#include "g1_arm/waist_hold.hpp"

#include <cmath>
#include <algorithm>
#include <fstream>
#include <json/json.h>
#include <stdexcept>

namespace g1_arm {
namespace {
Json::Value array_json(const CalligraphyQ &values) {
  Json::Value result(Json::arrayValue);
  for (double value : values) result.append(value);
  return result;
}
CalligraphyQ parse_array(const Json::Value &value, const char *field) {
  if (!value.isArray() || value.size() != kCalligraphyDof)
    throw std::runtime_error(std::string("invalid calligraphy ") + field + " length");
  CalligraphyQ result{};
  for (Json::ArrayIndex i = 0; i < value.size(); ++i) {
    if (!value[i].isNumeric() || !std::isfinite(value[i].asDouble()))
      throw std::runtime_error(std::string("invalid calligraphy ") + field + " value");
    result[i] = value[i].asDouble();
  }
  return result;
}
void extract(const State &state, CalligraphyQ &q, CalligraphyQ &dq) {
  for (std::size_t i = 0; i < kCalligraphyDof; ++i) {
    q[i] = state.q[12 + i];
    dq[i] = state.dq[12 + i];
  }
}
}

std::filesystem::path calligraphy_trajectory_path(const std::filesystem::path &directory,
                                                   const std::string &character,
                                                   const std::string &timestamp) {
  if (character.empty() || character.find_first_of("/\\") != std::string::npos)
    throw std::runtime_error("character name must not be empty or contain path separators");
  return directory / (character + "_" + timestamp + ".json");
}

void validate_calligraphy_trajectory(const CalligraphyTrajectory &trajectory) {
  if (trajectory.character_name.empty()) throw std::runtime_error("character name is required");
  if (trajectory.samples.empty()) throw std::runtime_error("calligraphy trajectory has no samples");
  double previous = -1.0;
  Trajectory arm_limits;
  arm_limits.name = "calligraphy";
  arm_limits.recorded_at = "calligraphy";
  for (const auto &sample : trajectory.samples) {
    if (!std::isfinite(sample.t_monotonic) || sample.t_monotonic < 0 ||
        sample.t_monotonic <= previous)
      throw std::runtime_error("calligraphy timestamps must be strictly increasing");
    previous = sample.t_monotonic;
    for (double value : sample.q)
      if (!std::isfinite(value)) throw std::runtime_error("non-finite calligraphy position");
    for (double value : sample.dq)
      if (!std::isfinite(value)) throw std::runtime_error("non-finite calligraphy velocity");
    arm_limits.points.clear();
    arm_limits.waist_reference = {sample.q[0], sample.q[1], sample.q[2]};
    Point arm_point;
    arm_point.name = "A";
    arm_point.recorded_at = "calligraphy";
    arm_point.q = calligraphy_arm_position(sample.q);
    arm_limits.points.push_back(arm_point);
    validate_trajectory(arm_limits);
  }
  if (trajectory.samples.front().t_monotonic != 0.0)
    throw std::runtime_error("calligraphy time axis must start at zero");
  if (!std::isfinite(trajectory.sample_rate_actual) || trajectory.sample_rate_actual < 0)
    throw std::runtime_error("invalid measured sample rate");
  if (trajectory.waist_pitch_extra_bias_known &&
      (!std::isfinite(trajectory.waist_pitch_extra_bias) ||
       std::abs(trajectory.waist_pitch_extra_bias) > kMaxWaistPitchExtraBiasRad))
    throw std::runtime_error("invalid recorded waist pitch extra bias");
}

void save_calligraphy_trajectory(const CalligraphyTrajectory &trajectory,
                                 const std::filesystem::path &path) {
  validate_calligraphy_trajectory(trajectory);
  Json::Value root;
  root["Character"]["character_name"] = trajectory.character_name;
  root["Character"]["paper_calibration_ref"] = trajectory.paper_calibration_ref;
  root["Character"]["brush_tcp_ref"] = trajectory.brush_tcp_ref;
  root["Character"]["brush_tcp_parameters"] = trajectory.brush_tcp_parameters;
  root["Character"]["start_joint_position"] = array_json(trajectory.start_joint_position);
  root["Character"]["end_joint_position"] = array_json(trajectory.end_joint_position);
  root["Character"]["hover_reference"] = trajectory.hover_reference;
  root["Motion"]["motor_indices"] = Json::arrayValue;
  for (int motor = 12; motor <= 28; ++motor) root["Motion"]["motor_indices"].append(motor);
  root["Motion"]["time_monotonic"] = Json::arrayValue;
  root["Motion"]["q"] = Json::arrayValue;
  root["Motion"]["dq"] = Json::arrayValue;
  for (const auto &sample : trajectory.samples) {
    root["Motion"]["time_monotonic"].append(sample.t_monotonic);
    root["Motion"]["q"].append(array_json(sample.q));
    root["Motion"]["dq"].append(array_json(sample.dq));
  }
  root["Metadata"]["recording_mode"] = "whole_character";
  root["Metadata"]["waist_mode"] = "fixed";
  root["Metadata"]["sample_rate_target"] = 50;
  root["Metadata"]["sample_rate_actual"] = trajectory.sample_rate_actual;
  root["Metadata"]["duration"] = trajectory.samples.back().t_monotonic;
  root["Metadata"]["frame_count"] = static_cast<Json::UInt64>(trajectory.samples.size());
  root["Metadata"]["missed_samples"] = static_cast<Json::UInt64>(trajectory.missed_samples);
  root["Metadata"]["start_time"] = trajectory.start_time;
  root["Metadata"]["units"] = "rad";
  root["Metadata"]["waist_pitch_extra_bias"] = trajectory.waist_pitch_extra_bias_known
      ? Json::Value(trajectory.waist_pitch_extra_bias) : Json::Value(Json::nullValue);
  root["Metadata"]["effective_waist_control_mode"] =
      trajectory.effective_waist_control_mode.empty() ? "UNKNOWN" :
      trajectory.effective_waist_control_mode;
  root["Metadata"]["configuration_snapshot"] = trajectory.configuration_snapshot;
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.string() + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create calligraphy file: " + temporary);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  output << Json::writeString(builder, root) << '\n';
  output.close();
  if (!output) throw std::runtime_error("failed writing calligraphy file: " + temporary);
  std::filesystem::rename(temporary, path);
}

CalligraphyTrajectory load_calligraphy_trajectory(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open calligraphy file: " + path.string());
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  if (!Json::parseFromStream(builder, input, &root, &error))
    throw std::runtime_error("invalid calligraphy JSON: " + error);
  const auto &character = root["Character"];
  const auto &motion = root["Motion"];
  const auto &metadata = root["Metadata"];
  if (!motion["motor_indices"].isArray() || motion["motor_indices"].size() != kCalligraphyDof ||
      !motion["time_monotonic"].isArray() || !motion["q"].isArray() ||
      !motion["dq"].isArray())
    throw std::runtime_error("incomplete calligraphy file");
  for (Json::ArrayIndex i = 0; i < kCalligraphyDof; ++i)
    if (motion["motor_indices"][i].asInt() != static_cast<int>(12 + i))
      throw std::runtime_error("calligraphy motor order must be 12..28");
  const auto count = motion["time_monotonic"].size();
  if (motion["q"].size() != count || motion["dq"].size() != count)
    throw std::runtime_error("calligraphy motion arrays have mismatched frame counts");
  if (metadata["frame_count"].isNumeric() && metadata["frame_count"].asUInt64() != count)
    throw std::runtime_error("calligraphy metadata frame count mismatch");
  CalligraphyTrajectory trajectory;
  trajectory.character_name = character["character_name"].asString();
  trajectory.paper_calibration_ref = character.get("paper_calibration_ref", "").asString();
  trajectory.brush_tcp_ref = character.get("brush_tcp_ref", "").asString();
  trajectory.brush_tcp_parameters = character.get("brush_tcp_parameters", Json::Value(Json::objectValue));
  trajectory.hover_reference = character.get("hover_reference", Json::Value(Json::nullValue));
  trajectory.start_joint_position = parse_array(character["start_joint_position"], "start position");
  trajectory.end_joint_position = parse_array(character["end_joint_position"], "end position");
  trajectory.start_time = metadata["start_time"].asString();
  trajectory.sample_rate_actual = metadata["sample_rate_actual"].asDouble();
  trajectory.missed_samples = static_cast<std::size_t>(metadata.get("missed_samples", 0).asUInt64());
  if (metadata.isMember("waist_pitch_extra_bias") && metadata["waist_pitch_extra_bias"].isNumeric()) {
    trajectory.waist_pitch_extra_bias = metadata["waist_pitch_extra_bias"].asDouble();
    trajectory.waist_pitch_extra_bias_known = std::isfinite(trajectory.waist_pitch_extra_bias);
  }
  trajectory.effective_waist_control_mode = metadata.get("effective_waist_control_mode", "UNKNOWN").asString();
  if (metadata.isMember("configuration_snapshot"))
    trajectory.configuration_snapshot = metadata["configuration_snapshot"];
  for (Json::ArrayIndex i = 0; i < count; ++i) {
    if (!motion["time_monotonic"][i].isNumeric()) throw std::runtime_error("invalid frame time");
    trajectory.samples.push_back({motion["time_monotonic"][i].asDouble(),
                                  parse_array(motion["q"][i], "q"),
                                  parse_array(motion["dq"][i], "dq")});
  }
  validate_calligraphy_trajectory(trajectory);
  return trajectory;
}

ArmQ calligraphy_arm_position(const CalligraphyQ &q17) {
  ArmQ arm{};
  for (std::size_t i = 0; i < arm.size(); ++i) arm[i] = q17[i + kWaistDof];
  return arm;
}

ArmQ interpolate_calligraphy_arm(const CalligraphyTrajectory &trajectory,
                                 double source_time_s) {
  if (trajectory.samples.empty()) throw std::runtime_error("calligraphy trajectory has no samples");
  if (!std::isfinite(source_time_s)) throw std::runtime_error("invalid calligraphy replay time");
  const auto &samples = trajectory.samples;
  if (source_time_s <= samples.front().t_monotonic)
    return calligraphy_arm_position(samples.front().q);
  if (source_time_s >= samples.back().t_monotonic)
    return calligraphy_arm_position(samples.back().q);
  const auto upper = std::upper_bound(samples.begin(), samples.end(), source_time_s,
      [](double time, const CalligraphySample &sample) { return time < sample.t_monotonic; });
  const auto &right = *upper;
  const auto &left = *(upper - 1);
  const double phase = (source_time_s - left.t_monotonic) /
                       (right.t_monotonic - left.t_monotonic);
  ArmQ result{};
  const auto left_arm = calligraphy_arm_position(left.q);
  const auto right_arm = calligraphy_arm_position(right.q);
  for (std::size_t i = 0; i < result.size(); ++i)
    result[i] = left_arm[i] + phase * (right_arm[i] - left_arm[i]);
  return result;
}

bool calligraphy_waist_matches(const CalligraphyTrajectory &trajectory,
                               const WaistQ &actual, double tolerance_rad,
                               WaistQ *errors) {
  if (!std::isfinite(tolerance_rad) || tolerance_rad < 0)
    throw std::runtime_error("invalid waist match tolerance");
  bool matches = true;
  for (std::size_t i = 0; i < kWaistDof; ++i) {
    const double error = actual[i] - trajectory.start_joint_position[i];
    if (errors) (*errors)[i] = error;
    if (!std::isfinite(error) || std::abs(error) > tolerance_rad) matches = false;
  }
  return matches;
}

void CalligraphyRecorder::start(const State &state, Clock::time_point now,
                                std::string start_time) {
  if (recording_) throw std::runtime_error("calligraphy recording already active");
  trajectory_ = {};
  trajectory_.start_time = std::move(start_time);
  started_at_ = now;
  last_capture_ = Clock::time_point{};
  recording_ = true;
  capture(state, now);
}

void CalligraphyRecorder::stop(const State &state) {
  if (!recording_) return;
  capture(state, Clock::now());
  for (std::size_t i = 0; i < kCalligraphyDof; ++i)
    trajectory_.end_joint_position[i] = state.q[12 + i];
  recording_ = false;
  if (trajectory_.samples.size() > 1) {
    const double duration = trajectory_.samples.back().t_monotonic -
                            trajectory_.samples.front().t_monotonic;
    trajectory_.sample_rate_actual = duration > 0
        ? static_cast<double>(trajectory_.samples.size() - 1) / duration : 0.0;
  }
}

void CalligraphyRecorder::capture(const State &state, Clock::time_point now) {
  if (!recording_ || (last_capture_ != Clock::time_point{} && now <= last_capture_)) return;
  CalligraphySample sample;
  sample.t_monotonic = std::chrono::duration<double>(now - started_at_).count();
  extract(state, sample.q, sample.dq);
  if (!trajectory_.samples.empty()) {
    const double elapsed = std::chrono::duration<double>(now - last_capture_).count();
    trajectory_.missed_samples += static_cast<std::size_t>(std::max(0.0, std::round(elapsed / 0.020) - 1.0));
  }
  trajectory_.samples.push_back(sample);
  last_capture_ = now;
  if (trajectory_.samples.size() == 1) trajectory_.start_joint_position = sample.q;
}
} // namespace g1_arm
