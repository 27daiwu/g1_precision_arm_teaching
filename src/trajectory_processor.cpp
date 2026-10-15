#include "g1_arm/trajectory_processor.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <json/json.h>
#include <limits>
#include <stdexcept>

namespace g1_arm {
namespace {
double path_distance(const ArmQ &a, const ArmQ &b) {
  double sum = 0.0;
  for (std::size_t i = 0; i < kArmDof; ++i) sum += (b[i] - a[i]) * (b[i] - a[i]);
  return std::sqrt(sum);
}
ArmQ lerp(const ArmQ &a, const ArmQ &b, double u) {
  ArmQ result{};
  for (std::size_t i = 0; i < kArmDof; ++i) result[i] = a[i] + (b[i] - a[i]) * u;
  return result;
}
double segment_error(const ArmQ &p, const ArmQ &a, const ArmQ &b) {
  double lo = 0.0, hi = 1.0;
  double numerator = 0.0, denominator = 0.0;
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const double d = b[i] - a[i];
    numerator += (p[i] - a[i]) * d;
    denominator += d * d;
  }
  if (denominator > 1e-14) lo = hi = std::clamp(numerator / denominator, 0.0, 1.0);
  const auto projected = lerp(a, b, lo);
  double max_error = 0.0;
  for (std::size_t i = 0; i < kArmDof; ++i)
    max_error = std::max(max_error, std::abs(p[i] - projected[i]));
  return max_error;
}
void validate_config(const ProcessorConfig &c) {
  if (!(c.max_gap_s > 0 && c.noise_floor_rad >= 0 && c.smoothing_weight >= 0 &&
        c.smoothing_weight <= .25 && c.resample_step_rad > 0 &&
        c.important_angle_rad > 0 && c.pause_speed_rad_s >= 0 &&
        c.max_geometry_error_rad > 0))
    throw std::runtime_error("invalid trajectory processor configuration");
}
}

ProcessedTrajectory process_trajectory(const ContinuousTrajectory &source,
                                       const ProcessorConfig &config) {
  validate_continuous_trajectory(source);
  validate_config(config);
  const auto &samples = source.samples;
  ProcessedTrajectory result;
  result.name = source.name;
  result.recorded_at = source.recorded_at;
  result.recording_start_q = source.recording_start_q;
  result.source_duration_s = samples.back().t_s - samples.front().t_s;
  std::vector<ArmQ> filtered(samples.size());
  for (std::size_t i = 0; i < samples.size(); ++i) filtered[i] = samples[i].q;

  // Densely sampled isolated jitter is damped; short reversals above the noise floor remain.
  for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
    const double dt0 = samples[i].t_s - samples[i - 1].t_s;
    const double dt1 = samples[i + 1].t_s - samples[i].t_s;
    if (dt0 > config.max_gap_s || dt1 > config.max_gap_s) continue;
    for (std::size_t j = 0; j < kArmDof; ++j) {
      const double neighbor_mid = .5 * (samples[i - 1].q[j] + samples[i + 1].q[j]);
      const double residual = samples[i].q[j] - neighbor_mid;
      const double local_motion = std::max(std::abs(samples[i].q[j] - samples[i - 1].q[j]),
                                           std::abs(samples[i + 1].q[j] - samples[i].q[j]));
      if (std::abs(residual) <= config.noise_floor_rad && local_motion <= .01)
        filtered[i][j] -= config.smoothing_weight * residual;
    }
  }

  std::vector<bool> important(samples.size(), false);
  important.front() = important.back() = true;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (samples[i].t_s - samples[i - 1].t_s > config.max_gap_s) {
      important[i - 1] = true;
      important[i] = true;
    }
  }
  std::vector<double> local_speed(samples.size(), 0.0);
  for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
    const double dt = samples[i + 1].t_s - samples[i - 1].t_s;
    local_speed[i] = dt > 0 ? path_distance(filtered[i - 1], filtered[i + 1]) / dt : 0.0;
    for (std::size_t j = 0; j < kArmDof; ++j) {
      const double before = filtered[i][j] - filtered[i - 1][j];
      const double after = filtered[i + 1][j] - filtered[i][j];
      if (before * after <= 0.0 &&
          std::min(std::abs(before), std::abs(after)) >= config.noise_floor_rad &&
          std::abs(filtered[i][j] - filtered[i - 1][j]) >= config.important_angle_rad)
        important[i] = true;
    }
  }
  for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
    const bool paused = local_speed[i] <= config.pause_speed_rad_s;
    const bool before_moving = local_speed[i - 1] > config.pause_speed_rad_s;
    const bool after_moving = local_speed[i + 1] > config.pause_speed_rad_s;
    if (paused && (before_moving || after_moving)) important[i] = true;
  }

  std::vector<std::size_t> anchors{0};
  for (std::size_t i = 1; i + 1 < samples.size(); ++i)
    if (important[i]) anchors.push_back(i);
  if (samples.size() > 1) anchors.push_back(samples.size() - 1);

  // Insert shared 14-joint samples by path arc length; never fit an overshooting spline.
  std::vector<ArmQ> resampled;
  std::vector<bool> resampled_important;
  resampled.push_back(filtered.front());
  resampled_important.push_back(true);
  for (std::size_t a = 0; a + 1 < anchors.size(); ++a) {
    const auto begin = anchors[a], end = anchors[a + 1];
    double arc = 0.0;
    for (std::size_t i = begin + 1; i <= end; ++i) {
      const double segment = path_distance(filtered[i - 1], filtered[i]);
      const auto count = std::max<std::size_t>(1, static_cast<std::size_t>(
          std::ceil(segment / config.resample_step_rad)));
      for (std::size_t k = 1; k <= count; ++k) {
        resampled.push_back(lerp(filtered[i - 1], filtered[i],
                                static_cast<double>(k) / count));
        resampled_important.push_back(k == count && important[i]);
      }
      arc += segment;
    }
    (void)arc;
  }
  if (resampled.size() == 1) {
    resampled.push_back(resampled.front());
    resampled_important.push_back(true);
  }
  double total = 0.0;
  for (std::size_t i = 1; i < resampled.size(); ++i) total += path_distance(resampled[i - 1], resampled[i]);
  double progress = 0.0;
  result.nodes.reserve(resampled.size());
  for (std::size_t i = 0; i < resampled.size(); ++i) {
    if (i) progress += path_distance(resampled[i - 1], resampled[i]);
    result.nodes.push_back({total > 0 ? progress / total : 1.0, resampled[i],
                            resampled_important[i] || i == 0 || i + 1 == resampled.size()});
  }
  const auto measure_deviation = [&](const ArmQ &q) {
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < result.nodes.size(); ++i)
      nearest = std::min(nearest, segment_error(q, result.nodes[i - 1].q, result.nodes[i].q));
    result.max_geometric_deviation_rad = std::max(result.max_geometric_deviation_rad, nearest);
  };
  for (std::size_t i = 0; i < filtered.size(); ++i) {
    measure_deviation(filtered[i]);
    measure_deviation(samples[i].q);
  }
  if (result.max_geometric_deviation_rad > config.max_geometry_error_rad)
    throw std::runtime_error("processed path exceeds configured geometric deviation");
  return result;
}

ArmQ sample_processed(const ProcessedTrajectory &trajectory, double s) {
  if (trajectory.nodes.empty()) throw std::runtime_error("processed trajectory has no nodes");
  s = std::clamp(s, 0.0, 1.0);
  const auto upper = std::upper_bound(trajectory.nodes.begin(), trajectory.nodes.end(), s,
      [](double progress, const ProcessedNode &node) { return progress < node.s; });
  if (upper == trajectory.nodes.begin()) return upper->q;
  if (upper == trajectory.nodes.end()) return trajectory.nodes.back().q;
  const auto &right = *upper;
  const auto &left = *(upper - 1);
  const double u = (s - left.s) / (right.s - left.s);
  return lerp(left.q, right.q, u);
}

void save_processed_trajectory(const ProcessedTrajectory &trajectory,
                               const std::filesystem::path &path) {
  Json::Value root;
  root["version"] = 1;
  root["kind"] = "processed_continuous";
  root["source_name"] = trajectory.name;
  root["recorded_at"] = trajectory.recorded_at;
  root["source_duration_s"] = trajectory.source_duration_s;
  root["max_geometric_deviation_rad"] = trajectory.max_geometric_deviation_rad;
  root["recording_start_q"] = Json::arrayValue;
  for (double q : trajectory.recording_start_q) root["recording_start_q"].append(q);
  root["nodes"] = Json::arrayValue;
  for (const auto &node : trajectory.nodes) {
    Json::Value item;
    item["s"] = node.s;
    item["important"] = node.important;
    item["q"] = Json::arrayValue;
    for (double q : node.q) item["q"].append(q);
    root["nodes"].append(item);
  }
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot write processed trajectory");
  Json::StreamWriterBuilder builder;
  builder["indentation"] = "  ";
  out << Json::writeString(builder, root) << '\n';
}
} // namespace g1_arm
