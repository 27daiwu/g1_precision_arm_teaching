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

void ArmSdkController::set_waist_pitch_extra_bias(double bias_rad) {
  if (!std::isfinite(bias_rad) || std::abs(bias_rad) > kMaxWaistPitchExtraBiasRad)
    throw std::runtime_error("waist pitch extra bias must be finite and within +/-0.050 rad");
  if (attempted_write_ || acquired_)
    throw std::runtime_error("waist pitch extra bias must be set before acquire");
  waist_pitch_extra_bias_ = bias_rad;
}

void ArmSdkController::set_waist_reference_target(const WaistQ &target) {
  if (attempted_write_ || acquired_)
    throw std::runtime_error("waist reference target must be set before acquire");
  for (double value : target)
    if (!std::isfinite(value)) throw std::runtime_error("invalid waist reference target");
  waist_reference_target_ = target;
  has_waist_reference_target_ = true;
}

void ArmSdkController::start(const std::string &interface) {
  if (started_) throw std::runtime_error("controller already started");
  dds_->initialize(interface);
  started_ = true;
  waist_logger_ = std::make_unique<Logger>(log_directory_);
  log_.open(log_directory_ / ("g1_commands_" + timestamp_filename_local() + ".csv"));
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

State ArmSdkController::wait_stable(const StabilityConfig &config) {
  if (!started_ || acquired_ || attempted_write_)
    throw std::runtime_error("WAIT_STABLE must run before Arm SDK acquire");
  StabilityGate gate(config);
  const double timeout_s = std::min(wait_timeout_s_, config.wait_timeout_s);
  const auto started = Clock::now();
  auto last_report = Clock::time_point{};
  bool waiting_reported = false;
  while (true) {
    if (g_stop_requested) throw std::runtime_error("operator stop requested during WAIT_STABLE");
    const State state = dds_->latest();
    for (int motor = 0; motor <= 28; ++motor) {
      if (!std::isfinite(state.q[motor]) || !std::isfinite(state.dq[motor]))
        throw std::runtime_error("invalid LowState motor " + std::to_string(motor) +
                                 " during WAIT_STABLE");
    }
    const auto metrics = gate.observe(state);
    stability_metrics_ = metrics;
    if (!waiting_reported) {
      std::cout << "WAIT_STABLE\nWaiting for robot to stabilize..." << std::flush;
      if (waist_logger_) waist_logger_->event("WAIT_STABLE", state, metrics);
      waiting_reported = true;
    }
    const auto now = Clock::now();
    if (now - last_report >= std::chrono::milliseconds(250)) {
      last_report = now;
      std::cout << "\rWaiting for robot to stabilize... "
                << "gyro=" << metrics.gyro_rad_s << " waist_dq=" << metrics.waist_dq_rad_s
                << " leg_rms=" << metrics.leg_dq_rms_rad_s
                << " leg_peak=" << metrics.leg_dq_peak_rad_s
                << " stable=" << gate.stable_duration_s(now) << "s" << std::flush;
    }
    if (gate.stable() && gate.stable_duration_s(now) >= config.window_s) {
      // Use the newest valid feedback frame, never the filtered window average.
      const State candidate = dds_->latest();
      for (int motor = 0; motor <= 28; ++motor) {
        if (!std::isfinite(candidate.q[motor]) || !std::isfinite(candidate.dq[motor]))
          throw std::runtime_error("invalid latest LowState at stable confirmation");
      }
      const auto confirmed = gate.observe(candidate);
      stability_metrics_ = confirmed;
      if (!gate.stable() || gate.stable_duration_s(candidate.received_at) < config.window_s)
        continue;
      stable_state_ = candidate;
      has_stable_state_ = true;
      std::cout << "\nSTABLE_CONFIRMED\n";
      if (waist_logger_) waist_logger_->event("STABLE_CONFIRMED", stable_state_, confirmed);
      return stable_state_;
    }
    if (std::chrono::duration<double>(now - started).count() >= timeout_s) {
      std::cout << "\nRobot did not stabilize within " << timeout_s
                << " seconds. Check stance and try again.\n";
      throw std::runtime_error("WAIT_STABLE timeout; no Arm SDK commands were published");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
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
  if (!has_stable_state_)
    throw std::runtime_error("WAIT_STABLE must complete before acquire");
  const auto state = stable_state_;
  mode_machine_ = state.mode_machine;
  for (std::size_t i = 0; i < captured_arm_.size(); ++i)
    captured_arm_[i] = state.q[15 + i];
  waist_.capture(state);
  if (has_waist_reference_target_) waist_.set_target(waist_reference_target_);
  if (!waist_logger_)
    waist_logger_ = std::make_unique<Logger>(log_directory_);
  waist_logger_->event("ACQUIRE_START", state, stability_metrics_, captured_arm_,
                       waist_.reference(), waist_.max_abs_error());
  const auto arm = captured_arm_;
  // Golden semantics: first ownership frame keeps measured q and carries weight=1.
  attempted_write_ = true;
  weight_ = 1.0;
  dds_->activate_publisher();
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
                            double acquire_progress, double release_progress) {
  std::array<double, 14> kp{};
  for (std::size_t i = 0; i < kp.size(); ++i) {
    const bool shoulder_elbow = i == 0 || i == 1 || i == 2 || i == 3 ||
                                i == 7 || i == 8 || i == 9 || i == 10;
    kp[i] = mode == TeachMode::Lock ? 40.0 : (shoulder_elbow ? 8.0 : 6.0);
  }
  send(q, kp, weight, acquire_progress, release_progress);
  last_mode_ = mode;
}

void ArmSdkController::send(const ArmQ &q, const std::array<double, 14> &kp,
                            double weight, double acquire_progress, double release_progress) {
  std::array<double, 14> kd{};
  kd.fill(1.5);
  send(q, kp, kd, weight, acquire_progress, release_progress);
}

void ArmSdkController::send(const ArmQ &q, const std::array<double, 14> &kp,
                            const std::array<double, 14> &kd, double weight,
                            double acquire_progress, double release_progress) {
  const auto state = latest();
  waist_.observe(state, acquire_progress, release_progress, waist_pitch_extra_bias_);
  ControlFrame frame;
  for (int motor = 15; motor <= 28; ++motor) {
    const auto offset = static_cast<std::size_t>(motor - 15);
    frame.q[motor] = q[offset];
    frame.kp[motor] = kp[offset];
    frame.kd[motor] = kd[offset];
  }
  frame.weight = weight;
  frame = waist_.apply(frame, acquire_progress, release_progress, waist_pitch_extra_bias_);
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
  last_kd_ = kd;
  if (log_.is_open()) {
    std::lock_guard<std::mutex> guard(log_mutex_);
    log_ << timestamp_local() << ",TEACH," << weight;
    for (int motor = 12; motor <= 28; ++motor)
      log_ << ',' << state.q[motor] << ',' << state.dq[motor] << ',' << frame.q[motor];
    log_ << '\n';
    WaistQ waist_command{};
    for (std::size_t i = 0; i < waist_command.size(); ++i)
      waist_command[i] = frame.q[12 + i];
    waist_logger_->waist(state, waist_.max_abs_error(),
                         waist_pitch_extra_bias_, acquire_progress, release_progress,
                         waist_command);
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

void ArmSdkController::publish(const ArmQ &q, const std::array<double, 14> &kp,
                               const std::array<double, 14> &kd, double weight) {
  if (!acquired_ || released_) throw std::runtime_error("controller not acquired");
  if (g_stop_requested) throw std::runtime_error("operator stop requested");
  for (double value : kp)
    if (!std::isfinite(value) || value < 6.0 || value > 40.0) throw std::runtime_error("invalid arm Kp");
  for (double value : kd)
    if (!std::isfinite(value) || value < 0.5 || value > 3.0) throw std::runtime_error("invalid arm Kd");
  send(q, kp, kd, weight, 1.0);
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
      send(hold, last_kp_, last_kd_, weight_, 1.0, progress);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    weight_ = 0.0;
    send(hold, last_kp_, last_kd_, 0.0, 1.0, 1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } catch (const std::exception &error) {
    if (fault_.empty()) fault_ = error.what();
    try {
      send(hold, last_kp_, last_kd_, 0.0, 1.0, 1.0);
    } catch (...) {}
  } catch (...) {
    try { send(hold, last_kp_, last_kd_, 0.0, 1.0, 1.0); } catch (...) {}
  }
  acquired_ = false;
  if (waist_logger_) {
    try {
      const auto state = dds_->latest();
      waist_logger_->event("RELEASE_COMPLETE", state, stability_metrics_, last_command_,
                           waist_.reference(), waist_.max_abs_error());
    } catch (...) {}
  }
}

void ArmSdkController::close() noexcept {
  if (attempted_write_ && !released_) release();
  if (dds_) dds_->close();
  if (log_.is_open()) log_.close();
  started_ = false;
}
} // namespace g1_arm
