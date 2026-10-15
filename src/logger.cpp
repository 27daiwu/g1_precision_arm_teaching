#include "g1_arm/logger.hpp"

#include "g1_arm/trajectory.hpp"
#include <algorithm>
#include <stdexcept>

namespace g1_arm {
Logger::Logger(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  auto stamp = timestamp_filename_local();
  path_ = directory / ("g1_arm_" + stamp + ".csv");
  for (unsigned suffix = 2; std::filesystem::exists(path_); ++suffix) {
    stamp = timestamp_filename_local() + "_" + std::to_string(suffix);
    path_ = directory / ("g1_arm_" + stamp + ".csv");
  }
  stream_.open(path_);
  if (!stream_) throw std::runtime_error("cannot create control log " + path_.string());
  stream_ << "timestamp,motor,q,dq,q_ref,max_abs_error,waist_pitch_extra_bias_rad,"
             "acquire_progress,release_progress,tracking_error_rad\n";
  events_path_ = directory / ("g1_arm_events_" + stamp + ".jsonl");
  events_stream_.open(events_path_);
  if (!events_stream_) throw std::runtime_error("cannot create event log " + events_path_.string());
}
void Logger::event(const std::string &name, const State &state,
                   const StabilityMetrics &metrics,
                   const std::optional<ArmQ> &arm_reference,
                   const std::optional<WaistQ> &waist_reference,
                   const std::optional<WaistQ> &max_abs_error) {
  const auto stamp = timestamp_local();
  events_stream_ << std::setprecision(9) << "{\"event\":\"" << name
                 << "\",\"timestamp\":\"" << stamp << "\",\"imu_gyro_roll_rad_s\":"
                 << state.imu_gyro[0] << ",\"imu_gyro_pitch_rad_s\":" << state.imu_gyro[1]
                 << ",\"filtered_gyro_rad_s\":" << metrics.gyro_rad_s
                 << ",\"filtered_waist_dq_rad_s\":" << metrics.waist_dq_rad_s
                 << ",\"filtered_leg_dq_rms_rad_s\":" << metrics.leg_dq_rms_rad_s
                 << ",\"filtered_leg_dq_peak_rad_s\":" << metrics.leg_dq_peak_rad_s
                 << ",\"waist_q\":[" << state.q[12] << ',' << state.q[13] << ',' << state.q[14]
                 << "],\"waist_dq\":[" << state.dq[12] << ',' << state.dq[13] << ',' << state.dq[14] << ']';
  if (arm_reference) {
    events_stream_ << ",\"arm_q_ref\":[";
    for (std::size_t i = 0; i < arm_reference->size(); ++i)
      events_stream_ << (i ? "," : "") << (*arm_reference)[i];
    events_stream_ << ']';
  }
  if (waist_reference)
    events_stream_ << ",\"waist_q_ref\":[" << (*waist_reference)[0] << ','
                   << (*waist_reference)[1] << ',' << (*waist_reference)[2] << ']';
  if (max_abs_error)
    events_stream_ << ",\"waist_max_abs_error\":[" << (*max_abs_error)[0] << ','
                   << (*max_abs_error)[1] << ',' << (*max_abs_error)[2] << ']';
  events_stream_ << "}\n";
  events_stream_.flush();
}
void Logger::waist(const State &state, const WaistQ &max_abs_error,
                   double extra_pitch_bias,
                   double acquire_progress, double release_progress,
                   const WaistQ &commanded_reference) {
  const auto now = timestamp_local();
  for (std::size_t i = 0; i < 3; ++i)
    stream_ << now << ',' << 12 + i << ',' << state.q[12 + i] << ','
            << state.dq[12 + i] << ',' << commanded_reference[i]
            << ',' << max_abs_error[i] << ',' << (i == 2 ? extra_pitch_bias : 0.0)
            << ',' << acquire_progress << ',' << release_progress << ','
            << (commanded_reference[i] - state.q[12 + i])
            << '\n';
  stream_.flush();
}
} // namespace g1_arm
