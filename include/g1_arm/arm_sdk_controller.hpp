#pragma once

#include "g1_arm/dds.hpp"
#include "g1_arm/waist_hold.hpp"
#include "g1_arm/logger.hpp"
#include <atomic>
#include <fstream>
#include <memory>

namespace g1_arm {
class ArmSdkController {
public:
  explicit ArmSdkController(std::unique_ptr<Dds> dds,
                            std::filesystem::path log_directory = "logs");
  ~ArmSdkController();
  void start(const std::string &interface);
  State latest() const;
  void acquire();
  void publish(const ArmQ &q, TeachMode mode, double weight = 1.0);
  void release() noexcept;
  void close() noexcept;
  bool acquired() const { return acquired_; }
  double weight() const { return weight_; }
  const WaistQ &waist_reference() const { return waist_.reference(); }
  WaistQ waist_max_abs_error() const { return waist_.max_abs_error(); }
  bool faulted() const { return !fault_.empty(); }
  const std::string &fault() const { return fault_; }
private:
  void send(const ArmQ &q, TeachMode mode, double weight,
            double bias_progress);
  void wait_period(Clock::time_point &deadline) const;
  std::unique_ptr<Dds> dds_;
  PublisherOwnership ownership_;
  WaistHold waist_;
  std::filesystem::path log_directory_;
  std::unique_ptr<Logger> waist_logger_;
  mutable std::mutex log_mutex_;
  std::ofstream log_;
  ArmQ captured_arm_{};
  std::uint8_t mode_machine_{};
  double weight_{0.0};
  bool started_{false};
  bool attempted_write_{false};
  bool acquired_{false};
  bool released_{false};
  std::string fault_;
  ArmQ last_command_{};
  TeachMode last_mode_{TeachMode::Lock};
};
} // namespace g1_arm
