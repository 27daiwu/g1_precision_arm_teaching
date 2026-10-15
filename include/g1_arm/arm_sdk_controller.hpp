#pragma once

#include "g1_arm/dds.hpp"
#include "g1_arm/waist_hold.hpp"
#include "g1_arm/logger.hpp"
#include "g1_arm/stability_gate.hpp"
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
  State wait_stable(const StabilityConfig &config = {});
  void set_wait_timeout(double timeout_s) { wait_timeout_s_ = timeout_s; }
  void set_waist_pitch_extra_bias(double bias_rad);
  void set_waist_reference_target(const WaistQ &target);
  void acquire();
  void publish(const ArmQ &q, TeachMode mode, double weight = 1.0);
  void publish(const ArmQ &q, const std::array<double, 14> &kp,
               double weight = 1.0);
  void publish(const ArmQ &q, const std::array<double, 14> &kp,
               const std::array<double, 14> &kd, double weight = 1.0);
  void release() noexcept;
  void close() noexcept;
  bool acquired() const { return acquired_; }
  bool stop_requested() const;
  double weight() const { return weight_; }
  const WaistQ &waist_reference() const { return waist_.reference(); }
  WaistQ waist_max_abs_error() const { return waist_.max_abs_error(); }
  bool faulted() const { return !fault_.empty(); }
  const std::string &fault() const { return fault_; }
private:
  void send(const ArmQ &q, TeachMode mode, double weight,
            double acquire_progress, double release_progress = 0.0);
  void send(const ArmQ &q, const std::array<double, 14> &kp, double weight,
            double acquire_progress, double release_progress = 0.0);
  void send(const ArmQ &q, const std::array<double, 14> &kp,
            const std::array<double, 14> &kd, double weight,
            double acquire_progress, double release_progress = 0.0);
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
  std::array<double, 14> last_kp_{};
  std::array<double, 14> last_kd_{};
  double wait_timeout_s_{10.0};
  double waist_pitch_extra_bias_{0.0};
  WaistQ waist_reference_target_{};
  bool has_waist_reference_target_{false};
  State stable_state_{};
  bool has_stable_state_{false};
  StabilityMetrics stability_metrics_{};
};
} // namespace g1_arm
