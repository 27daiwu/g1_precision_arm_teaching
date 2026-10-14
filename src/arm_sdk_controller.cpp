#include "g1_arm/arm_sdk_controller.hpp"

#include "g1_arm/logger.hpp"
#include "g1_arm/trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

namespace g1_arm {
namespace {
volatile std::sig_atomic_t g_stop_requested = 0;
void on_signal(int) { g_stop_requested = 1; }
}

ArmSdkController::ArmSdkController(std::unique_ptr<Dds> dds,
                                   std::filesystem::path log_directory)
    : dds_(std::move(dds)), log_directory_(std::move(log_directory)) {
  if (!dds_) throw std::runtime_error("DDS transport is required");
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
}

ArmSdkController::~ArmSdkController() { close(); }

void ArmSdkController::start(const std::string &interface) {
  if (started_) throw std::runtime_error("controller already started");
  dds_->initialize(interface);
  started_ = true;
  waist_logger_ = std::make_unique<Logger>(log_directory_);
  log_.open(log_directory_ / ("g1_commands_" + timestamp_local() + ".csv"));
  if (!log_) throw std::runtime_error("cannot create command log");
  log_ << "timestamp,mode,weight";
  for (int motor = 12; motor <= 28; ++motor)
    log_ << ",m" << motor << "_q,m" << motor << "_dq,m" << motor << "_q_ref";
  log_ << "\n";
  const auto until = Clock::now() + std::chrono::seconds(5);
  while (Clock::now() < until) {
    try {
      const auto state = dds_->latest();
      for (int motor = 0; motor < 29; ++motor)
        if (!std::isfinite(state.q[motor]) || !std::isfinite(state.dq[motor]))
          throw std::runtime_error("non-finite LowState motor " + std::to_string(motor));
      return;
    } catch (const std::runtime_error &) {
      if (g_stop_requested) throw std::runtime_error("stopped waiting for LowState");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  throw std::runtime_error("no valid LowState received within 5 seconds");
}

State ArmSdkController::latest() const {
  auto state = dds_->latest();
  if (state.mode_machine != mode_machine_ && acquired_)
    throw std::runtime_error("mode_machine changed during Arm SDK ownership");
  for (int motor = 12; motor <= 28; ++motor) {
    if (!std::isfinite(state.q[motor]) || !std::isfinite(state.dq[motor]))
      throw std::runtime_error("invalid LowState motor " + std::to_string(motor));
  }
  return state;
}

void ArmSdkController::wait_period(Clock::time_point &deadline) const {
  deadline += std::chrono::milliseconds(20);
  std::this_thread::sleep_until(deadline);
  if (Clock::now() - deadline > std::chrono::milliseconds(50))
    throw std::runtime_error("50Hz control loop interruption >50ms");
  if (g_stop_requested) throw std::runtime_error("operator stop requested");
  if (deadline < Clock::now()) deadline = Clock::now();
}

void ArmSdkController::acquire() {
  if (!started_ || acquired_) throw std::runtime_error("acquire not allowed");
  const auto state = latest();
  mode_machine_ = state.mode_machine;
  for (std::size_t i = 0; i < captured_arm_.size(); ++i)
    captured_arm_[i] = state.q[15 + i];
  waist_.capture(state);
  if (!waist_logger_)
    waist_logger_ = std::make_unique<Logger>(log_directory_);
  const auto arm = captured_arm_;
  // Golden semantics: first ownership frame keeps measured q and carries weight=1.
  attempted_write_ = true;
  weight_ = 1.0;
  send(arm, TeachMode::Lock, 1.0, 0.0);
  acquired_ = true;
  Clock::time_point deadline = Clock::now();
  const auto started = Clock::now();
  while (Clock::now() - started < std::chrono::seconds(2)) {
    wait_period(deadline);
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    const double progress = smoothstep(elapsed / 2.0);
    weight_ = progress;
    send(arm, TeachMode::Lock, weight_, progress);
  }
  weight_ = 1.0;
  send(arm, TeachMode::Lock, weight_, 1.0);
}

void ArmSdkController::send(const ArmQ &q, TeachMode mode, double weight,
                            double bias_progress) {
  std::array<double, 14> kp{};
  for (std::size_t i = 0; i < kp.size(); ++i) {
    const bool shoulder_elbow = i == 0 || i == 1 || i == 2 || i == 3 ||
                                i == 7 || i == 8 || i == 9 || i == 10;
    kp[i] = mode == TeachMode::Lock ? 40.0 : (shoulder_elbow ? 8.0 : 6.0);
  }
  send(q, kp, weight, bias_progress);
  last_mode_ = mode;
}

void ArmSdkController::send(const ArmQ &q, const std::array<double, 14> &kp,
                            double weight, double bias_progress) {
  const auto state = latest();
  waist_.observe(state);
  ControlFrame frame;
  for (int motor = 15; motor <= 28; ++motor) {
    const auto offset = static_cast<std::size_t>(motor - 15);
    frame.q[motor] = q[offset];
    frame.kp[motor] = kp[offset];
    frame.kd[motor] = 1.5;
  }
  frame.weight = weight;
  frame = waist_.apply(frame, bias_progress, 0.0);
  for (int motor = 12; motor <= 28; ++motor) {
    if (!std::isfinite(frame.q[motor]) || !std::isfinite(frame.kp[motor]) ||
        !std::isfinite(frame.kd[motor]))
      throw std::runtime_error("non-finite command");
  }
  std::string error;
  if (!dds_->publish(frame, error)) {
    fault_ = error.empty() ? "Arm SDK publish failed" : error;
    throw std::runtime_error(fault_);
  }
  last_command_ = q;
  last_kp_ = kp;
  if (log_.is_open()) {
    std::lock_guard<std::mutex> guard(log_mutex_);
    log_ << timestamp_local() << ",TEACH," << weight;
    for (int motor = 12; motor <= 28; ++motor)
      log_ << ',' << state.q[motor] << ',' << state.dq[motor] << ',' << frame.q[motor];
    log_ << '\n';
    waist_logger_->waist(state, waist_.reference(), waist_.max_abs_error());
  }
}

void ArmSdkController::publish(const ArmQ &q, TeachMode mode, double weight) {
  if (!acquired_ || released_) throw std::runtime_error("controller not acquired");
  if (g_stop_requested) throw std::runtime_error("operator stop requested");
  send(q, mode, weight, 1.0);
  weight_ = weight;
}

void ArmSdkController::publish(const ArmQ &q, const std::array<double, 14> &kp,
                               double weight) {
  if (!acquired_ || released_) throw std::runtime_error("controller not acquired");
  if (g_stop_requested) throw std::runtime_error("operator stop requested");
  for (double value : kp)
    if (!std::isfinite(value) || value < 6.0 || value > 40.0)
      throw std::runtime_error("invalid arm gain");
  send(q, kp, weight, 1.0);
  last_mode_ = TeachMode::Lock;
  weight_ = weight;
}

bool ArmSdkController::stop_requested() const { return g_stop_requested != 0; }

void ArmSdkController::release() noexcept {
  if (!attempted_write_ || released_) return;
  released_ = true;
  const ArmQ hold = last_command_;
  const double start_weight = weight_;
  const auto started = Clock::now();
  Clock::time_point deadline = Clock::now();
  try {
    while (Clock::now() - started < std::chrono::seconds(2)) {
      deadline += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(deadline);
      const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
      const double progress = smoothstep(elapsed / 2.0);
      const double remaining = 1.0 - progress;
      weight_ = start_weight * remaining;
      send(hold, last_kp_, weight_, progress);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    weight_ = 0.0;
    send(hold, last_kp_, 0.0, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } catch (const std::exception &error) {
    if (fault_.empty()) fault_ = error.what();
    try {
      send(hold, last_kp_, 0.0, 0.0);
    } catch (...) {}
  } catch (...) {
    try { send(hold, last_kp_, 0.0, 0.0); } catch (...) {}
  }
  acquired_ = false;
}

void ArmSdkController::close() noexcept {
  if (attempted_write_ && !released_) release();
  if (dds_) dds_->close();
  if (log_.is_open()) log_.close();
  started_ = false;
}
} // namespace g1_arm
