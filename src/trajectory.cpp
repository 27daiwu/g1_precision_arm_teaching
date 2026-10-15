#include "g1_arm/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <json/json.h>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace g1_arm {
namespace {
constexpr std::array<double, 17> kLower{
    -2.618, -.52, -.52, -3.0892, -1.5882, -2.618, -1.0472,
    -1.972222054, -1.614429558, -1.614429558, -3.0892, -2.2515,
    -2.618, -1.0472, -1.972222054, -1.614429558, -1.614429558};
constexpr std::array<double, 17> kUpper{
    2.618, .52, .52, 2.6704, 2.2515, 2.618, 2.0944,
    1.972222054, 1.614429558, 1.614429558, 2.6704, 1.5882,
    2.618, 2.0944, 1.972222054, 1.614429558, 1.614429558};
void check_joint(std::size_t motor, double q) {
  if (!std::isfinite(q)) throw std::runtime_error("non-finite joint angle");
  const auto index = motor - 12;
  if (q < kLower[index] || q > kUpper[index])
    throw std::runtime_error("motor " + std::to_string(motor) +
                             " exceeds G1 URDF joint limit");
}
Json::Value array_json(const ArmQ &values) {
  Json::Value result(Json::arrayValue);
  for (double value : values) result.append(value);
  return result;
}
Json::Value array_json(const WaistQ &values) {
  Json::Value result(Json::arrayValue);
  for (double value : values) result.append(value);
  return result;
}
template <std::size_t N>
std::array<double, N> parse_array(const Json::Value &value, const char *name) {
  if (!value.isArray() || value.size() != N)
    throw std::runtime_error(std::string("invalid ") + name + " array length");
  std::array<double, N> result{};
  for (Json::ArrayIndex i = 0; i < N; ++i) {
    if (!value[i].isNumeric()) throw std::runtime_error("non-numeric trajectory value");
    result[i] = value[i].asDouble();
    if (!std::isfinite(result[i])) throw std::runtime_error("non-finite trajectory value");
  }
  return result;
}
} // namespace

void validate_name(const std::string &name) {
  static const std::regex valid("^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$");
  if (!std::regex_match(name, valid))
    throw std::runtime_error("name must contain 1..64 letters, digits, '_' or '-'");
}

void validate_trajectory(const Trajectory &trajectory) {
  validate_name(trajectory.name);
  if (trajectory.points.empty()) throw std::runtime_error("trajectory has no points");
  for (std::size_t i = 0; i < 3; ++i)
    check_joint(12 + i, trajectory.waist_reference[i] + (i == 2 ? .010 : 0.0));
  std::vector<std::string> names;
  for (const auto &point : trajectory.points) {
    validate_name(point.name);
    if (std::find(names.begin(), names.end(), point.name) != names.end())
      throw std::runtime_error("duplicate point name: " + point.name);
    names.push_back(point.name);
    if (!std::isfinite(point.hold_s) || point.hold_s < 0 || point.hold_s > 600)
      throw std::runtime_error("point hold time must be 0..600 seconds");
    for (std::size_t i = 0; i < point.q.size(); ++i) check_joint(15 + i, point.q[i]);
  }
}

void set_point(Trajectory &trajectory, const Point &point) {
  validate_name(point.name);
  for (auto &existing : trajectory.points) {
    if (existing.name == point.name) { existing = point; return; }
  }
  trajectory.points.push_back(point);
}

void delete_point(Trajectory &trajectory, const std::string &name) {
  auto it = std::find_if(trajectory.points.begin(), trajectory.points.end(),
                         [&](const Point &point) { return point.name == name; });
  if (it == trajectory.points.end()) throw std::runtime_error("point not found: " + name);
  trajectory.points.erase(it);
}

std::string timestamp_local() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S%z");
  return out.str();
}

std::string timestamp_filename_local() {
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;
  std::tm tm{};
  localtime_r(&time, &tm);
  std::ostringstream out;
  out << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << '-'
      << std::setw(3) << std::setfill('0') << ms.count();
  return out.str();
}

std::filesystem::path trajectory_path(const std::filesystem::path &directory,
                                      const std::string &name) {
  validate_name(name);
  return directory / (name + ".json");
}

Trajectory load_trajectory(const std::filesystem::path &path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open trajectory: " + path.string());
  Json::Value root;
  Json::CharReaderBuilder builder;
  std::string error;
  if (!Json::parseFromStream(builder, input, &root, &error))
    throw std::runtime_error("invalid trajectory JSON: " + error);
  if (root["version"].asInt() != 1 || !root["points"].isArray())
    throw std::runtime_error("unsupported trajectory JSON version");
  Trajectory trajectory;
  trajectory.name = root["name"].asString();
  trajectory.recorded_at = root.get("recorded_at", "").asString();
  trajectory.waist_reference = parse_array<3>(root["waist_q_reference"], "waist reference");
  for (const auto &item : root["points"]) {
    Point point;
    point.name = item["name"].asString();
    point.recorded_at = item["recorded_at"].asString();
    point.q = parse_array<14>(item["q"], "arm joint");
    point.hold_s = item.get("hold_s", 0.0).asDouble();
    trajectory.points.push_back(std::move(point));
  }
  validate_trajectory(trajectory);
  return trajectory;
}

void save_trajectory(const Trajectory &trajectory, const std::filesystem::path &path) {
  validate_trajectory(trajectory);
  Json::Value root;
  root["version"] = 1;
  root["name"] = trajectory.name;
  root["recorded_at"] = trajectory.recorded_at;
  root["units"] = "rad";
  root["motor_ids"] = Json::arrayValue;
  for (int motor = 15; motor <= 28; ++motor) root["motor_ids"].append(motor);
  root["waist_q_reference"] = array_json(trajectory.waist_reference);
  root["points"] = Json::arrayValue;
  for (const auto &point : trajectory.points) {
    Json::Value item;
    item["name"] = point.name;
    item["recorded_at"] = point.recorded_at;
    item["q"] = array_json(point.q);
    item["hold_s"] = point.hold_s;
    root["points"].append(item);
  }
  std::filesystem::create_directories(path.parent_path());
  const auto temporary = path.string() + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) throw std::runtime_error("cannot create trajectory: " + temporary);
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  output << Json::writeString(builder, root) << '\n';
  output.close();
  if (!output) throw std::runtime_error("failed writing trajectory: " + temporary);
  std::filesystem::rename(temporary, path);
}

double smoothstep(double x) {
  x = std::clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

double motion_duration(const ArmQ &from, const ArmQ &to, double speed_rad_s,
                       double minimum_s) {
  if (!std::isfinite(speed_rad_s) || speed_rad_s <= 0 ||
      !std::isfinite(minimum_s) || minimum_s < 0)
    throw std::runtime_error("invalid replay speed or duration");
  double delta = 0;
  for (std::size_t i = 0; i < from.size(); ++i)
    delta = std::max(delta, std::abs(to[i] - from[i]));
  return std::max(minimum_s, 1.5 * delta / speed_rad_s);
}

ArmQ interpolate(const ArmQ &from, const ArmQ &to, double elapsed_s,
                  double duration_s) {
  const double phase = duration_s <= 0 ? 1.0 : smoothstep(elapsed_s / duration_s);
  ArmQ result{};
  for (std::size_t i = 0; i < result.size(); ++i)
    result[i] = from[i] + (to[i] - from[i]) * phase;
  return result;
}

TeachState::TeachState(const ArmQ &initial) : reference_(initial) {
  ramp_from_.fill(40.0);
}

void TeachState::drag(const ArmQ &measured) {
  reference_ = measured;
  mode_ = TeachMode::Drag;
}

void TeachState::lock(const ArmQ &measured, Clock::time_point now) {
  ramp_from_ = gains(now);
  reference_ = measured;
  mode_ = TeachMode::Lock;
  gain_ramp_start_ = now;
}

ArmQ TeachState::update(const ArmQ &measured) {
  if (mode_ == TeachMode::Drag) {
    for (std::size_t i = 0; i < reference_.size(); ++i) {
      const double error = measured[i] - reference_[i];
      if (std::abs(error) > .015) reference_[i] += .25 * error;
    }
  }
  return reference_;
}

std::array<double, 14> TeachState::gains(Clock::time_point now) const {
  std::array<double, 14> result{};
  const double elapsed = std::chrono::duration<double>(now - gain_ramp_start_).count();
  const double phase = mode_ == TeachMode::Lock ? smoothstep(elapsed / .4) : 1.0;
  for (std::size_t i = 0; i < result.size(); ++i) {
    const bool shoulder_elbow = i == 0 || i == 1 || i == 2 || i == 3 ||
                                i == 7 || i == 8 || i == 9 || i == 10;
    const double drag_kp = shoulder_elbow ? 8.0 : 6.0;
    result[i] = mode_ == TeachMode::Drag ? drag_kp
                                        : ramp_from_[i] + (40.0 - ramp_from_[i]) * phase;
  }
  return result;
}
} // namespace g1_arm
