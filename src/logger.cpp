#include "g1_arm/logger.hpp"

#include "g1_arm/trajectory.hpp"
#include <stdexcept>

namespace g1_arm {
Logger::Logger(const std::filesystem::path &directory) {
  std::filesystem::create_directories(directory);
  path_ = directory / ("g1_arm_" + timestamp_local() + ".csv");
  for (unsigned suffix = 2; std::filesystem::exists(path_); ++suffix)
    path_ = directory / ("g1_arm_" + timestamp_local() + "_" +
                         std::to_string(suffix) + ".csv");
  stream_.open(path_);
  if (!stream_) throw std::runtime_error("cannot create control log " + path_.string());
  stream_ << "timestamp,motor,q,dq,q_ref,max_abs_error\n";
}
void Logger::waist(const State &state, const WaistQ &reference,
                   const WaistQ &max_abs_error) {
  const auto now = timestamp_local();
  for (std::size_t i = 0; i < 3; ++i)
    stream_ << now << ',' << 12 + i << ',' << state.q[12 + i] << ','
            << state.dq[12 + i] << ',' << reference[i] + (i == 2 ? .010 : 0.0)
            << ',' << max_abs_error[i] << '\n';
  stream_.flush();
}
} // namespace g1_arm
