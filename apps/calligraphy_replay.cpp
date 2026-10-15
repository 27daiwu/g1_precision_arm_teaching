#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/calligraphy_trajectory.hpp"
#include "g1_arm/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <thread>

namespace {
using namespace g1_arm;
struct Options {
  std::filesystem::path file;
  std::string interface{"eth0"};
  double speed_scale{0.25};
  double waist_pitch_extra_bias{0.05};
  double waist_tolerance{0.04};
  double max_joint_speed{0.20};
  double tracking_error_limit{0.15};
  double shoulder_kp{40.0}, elbow_kp{40.0}, wrist_kp{40.0}, motor_kd{1.5};
  std::array<double, 14> kp{};
  std::array<double, 14> kd{};
  bool dry_run{false}, dump_config{false};
  bool validate_only{false};
  bool allow_unknown_waist_metadata{false};
};

Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--file" && i + 1 < argc) options.file = argv[++i];
    else if (arg == "--interface" && i + 1 < argc) options.interface = argv[++i];
    else if (arg == "--speed-scale" && i + 1 < argc) options.speed_scale = std::stod(argv[++i]);
    else if (arg == "--waist-pitch-extra-bias" && i + 1 < argc)
      options.waist_pitch_extra_bias = std::stod(argv[++i]);
    else if (arg == "--waist-tolerance" && i + 1 < argc) options.waist_tolerance = std::stod(argv[++i]);
    else if (arg == "--max-joint-speed" && i + 1 < argc) options.max_joint_speed = std::stod(argv[++i]);
    else if (arg == "--tracking-error-limit" && i + 1 < argc)
      options.tracking_error_limit = std::stod(argv[++i]);
    else if (arg == "--shoulder-kp" && i + 1 < argc) options.shoulder_kp = std::stod(argv[++i]);
    else if (arg == "--elbow-kp" && i + 1 < argc) options.elbow_kp = std::stod(argv[++i]);
    else if (arg == "--wrist-kp" && i + 1 < argc) options.wrist_kp = std::stod(argv[++i]);
    else if (arg == "--motor-kd" && i + 1 < argc) options.motor_kd = std::stod(argv[++i]);
    else if (arg == "--motor-kd-joint" && i + 1 < argc) {
      const std::string spec = argv[++i]; const auto split = spec.find('=');
      if (split == std::string::npos) throw std::runtime_error("--motor-kd-joint expects MOTOR=VALUE");
      const int motor = std::stoi(spec.substr(0, split));
      if (motor < 15 || motor > 28) throw std::runtime_error("motor index must be 15..28");
      options.kd[static_cast<std::size_t>(motor - 15)] = std::stod(spec.substr(split + 1));
    }
    else if (arg == "--motor-kp" && i + 1 < argc) {
      const std::string spec = argv[++i];
      const auto split = spec.find('=');
      if (split == std::string::npos) throw std::runtime_error("--motor-kp expects MOTOR=VALUE");
      const int motor = std::stoi(spec.substr(0, split));
      if (motor < 15 || motor > 28) throw std::runtime_error("motor index must be 15..28");
      options.kp[static_cast<std::size_t>(motor - 15)] = std::stod(spec.substr(split + 1));
    }
    else if (arg == "--dry-run") options.dry_run = true;
    else if (arg == "--dump-effective-config") options.dump_config = true;
    else if (arg == "--validate-only") options.validate_only = true;
    else if (arg == "--allow-unknown-waist-metadata") options.allow_unknown_waist_metadata = true;
    else if (arg == "--help") {
      std::cout << "Usage: g1_calligraphy_replay --file PATH [--interface eth0] "
                   "[--speed-scale 0.25] [--waist-pitch-extra-bias 0.05] "
                   "[--waist-tolerance 0.04] [--max-joint-speed 0.20] "
                   "[--tracking-error-limit 0.15] "
                   "[--shoulder-kp 40 --elbow-kp 40 --wrist-kp 40] [--motor-kp MOTOR=VALUE] [--motor-kd 1.5] [--motor-kd-joint MOTOR=VALUE] "
                   "[--dry-run] [--dump-effective-config] "
                   "[--validate-only] [--allow-unknown-waist-metadata]\n";
      std::exit(0);
    } else throw std::runtime_error("invalid argument: " + arg);
  }
  if (options.file.empty()) throw std::runtime_error("--file is required");
  if (options.interface.empty()) throw std::runtime_error("--interface must not be empty");
  if (!std::isfinite(options.speed_scale) || options.speed_scale <= 0 || options.speed_scale > 1.0)
    throw std::runtime_error("speed scale must be within (0, 1]");
  if (!std::isfinite(options.waist_pitch_extra_bias) ||
      std::abs(options.waist_pitch_extra_bias) > kMaxWaistPitchExtraBiasRad)
    throw std::runtime_error("waist pitch extra bias must be finite and within +/-0.050 rad");
  if (!std::isfinite(options.waist_tolerance) || options.waist_tolerance <= 0 ||
      !std::isfinite(options.max_joint_speed) || options.max_joint_speed <= 0 ||
      options.max_joint_speed > 0.5 || !std::isfinite(options.tracking_error_limit) ||
      options.tracking_error_limit <= 0 || options.tracking_error_limit > 0.5)
    throw std::runtime_error("waist tolerance and max joint speed must be positive and bounded");
  for (auto *value : {&options.shoulder_kp, &options.elbow_kp, &options.wrist_kp})
    if (!std::isfinite(*value) || *value < 6.0 || *value > 40.0) throw std::runtime_error("group Kp must be in [6,40]");
  if (!std::isfinite(options.motor_kd) || options.motor_kd < .5 || options.motor_kd > 3.0)
    throw std::runtime_error("motor Kd must be in [0.5,3]");
  for (std::size_t i = 0; i < options.kp.size(); ++i) {
    const std::size_t within_arm = i % 7;
    if (options.kp[i] == 0.0)
      options.kp[i] = within_arm < 3 ? options.shoulder_kp : (within_arm == 3 ? options.elbow_kp : options.wrist_kp);
    if (!std::isfinite(options.kp[i]) || options.kp[i] < 6.0 || options.kp[i] > 40.0)
      throw std::runtime_error("motor Kp must be in [6,40]");
  }
  for (std::size_t i = 0; i < options.kd.size(); ++i) {
    if (options.kd[i] == 0.0) options.kd[i] = options.motor_kd;
    if (!std::isfinite(options.kd[i]) || options.kd[i] < .5 || options.kd[i] > 3.0)
      throw std::runtime_error("motor Kd must be in [0.5,3]");
  }
  std::cout << "EFFECTIVE speed_scale=" << options.speed_scale << " waist_pitch_extra_bias="
            << options.waist_pitch_extra_bias << " move_to_start_duration=auto profile=builtin Kp=[";
  for (std::size_t i = 0; i < options.kp.size(); ++i) std::cout << (i ? "," : "") << options.kp[i];
  std::cout << "] Kd=[";
  for (std::size_t i = 0; i < options.kd.size(); ++i) std::cout << (i ? "," : "") << options.kd[i];
  std::cout << "]\n";
  return options;
}

double log_frame(std::ofstream &log, const std::string &status, const ArmQ &q_ref,
                 const State &state, double dt);

bool wait_for_enter(ArmSdkController &controller, const ArmQ &hold,
                    const std::string &prompt, std::ofstream &log,
                    double tracking_error_limit) {
  std::cout << prompt << " Press Enter to continue, Ctrl+C to release.\n" << std::flush;
  log << timestamp_local() << ',' << prompt << ",0,0,0\n";
  auto deadline = Clock::now();
  auto previous = Clock::now();
  while (!controller.stop_requested()) {
    pollfd input{0, POLLIN, 0};
    if (poll(&input, 1, 0) > 0 && (input.revents & POLLIN)) {
      std::string line;
      if (!std::getline(std::cin, line)) return false;
      return true;
    }
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - previous).count();
    previous = now;
    if (dt > 0.1) throw std::runtime_error("control loop interruption >100ms while holding");
    const State state = controller.latest();
    if (log_frame(log, prompt, hold, state, dt) > tracking_error_limit)
      throw std::runtime_error("tracking error exceeded limit while holding");
    controller.publish(hold, TeachMode::Lock);
    deadline += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(deadline);
    if (deadline < Clock::now()) deadline = Clock::now();
  }
  return false;
}

double move_duration(const ArmQ &from, const ArmQ &to, double speed) {
  double delta = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) delta = std::max(delta, std::abs(to[i] - from[i]));
  return std::max(1.0, 1.875 * delta / speed);
}
double smoothstep5(double u) {
  u = std::clamp(u, 0.0, 1.0);
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

std::filesystem::path make_log_path() {
  const auto directory = std::filesystem::path("logs");
  std::filesystem::create_directories(directory);
  return directory / ("g1_calligraphy_replay_" + timestamp_filename_local() + ".csv");
}

double log_frame(std::ofstream &log, const std::string &status, const ArmQ &q_ref,
               const State &state, double dt) {
  const double frequency = dt > 0 ? 1.0 / dt : 0.0;
  double max_error = 0.0;
  for (std::size_t i = 0; i < kArmDof; ++i)
  {
    const double error = q_ref[i] - state.q[15 + i];
    max_error = std::max(max_error, std::abs(error));
    log << timestamp_local() << ',' << status << ',' << 15 + i << ',' << q_ref[i]
        << ',' << state.q[15 + i] << ',' << error << ',' << frequency << '\n';
  }
  log.flush();
  return max_error;
}
}

int main(int argc, char **argv) {
  using namespace g1_arm;
  try {
    const auto options = parse(argc, argv);
    const auto trajectory = load_calligraphy_trajectory(options.file);
    const double duration = trajectory.samples.back().t_monotonic;
    const double effective_rate = trajectory.samples.size() > 1 && duration > 0
        ? (trajectory.samples.size() - 1) / duration : 0.0;
    std::cout << "CHARACTER=" << trajectory.character_name
              << " frames=" << trajectory.samples.size() << " duration=" << duration
              << "s source_rate=" << effective_rate << "Hz\n"
              << "waist metadata=" << (trajectory.waist_pitch_extra_bias_known
                    ? std::to_string(trajectory.waist_pitch_extra_bias) : "UNKNOWN")
              << " effective_mode=" << trajectory.effective_waist_control_mode << '\n';
    if (options.validate_only || options.dry_run) {
      std::cout << "VALID: motor indices 12..28, " << trajectory.samples.size()
                << " synchronized frames, duration=" << duration
                << "s, waist metadata="
                << (trajectory.waist_pitch_extra_bias_known ? "known" : "UNKNOWN") << "\n";
      return 0;
    }
    if (!trajectory.waist_pitch_extra_bias_known && !options.allow_unknown_waist_metadata)
      throw std::runtime_error("recording waist bias is UNKNOWN; use --allow-unknown-waist-metadata only after confirming the recording setup");

    std::ofstream log(make_log_path());
    if (!log) throw std::runtime_error("cannot create calligraphy replay log");
    log << "timestamp,status,motor,q_ref,q_actual,error,control_rate_hz\n";
    ArmSdkController controller(make_unitree_dds());
    controller.set_waist_pitch_extra_bias(options.waist_pitch_extra_bias);
    controller.start(options.interface);
    const State stable = controller.wait_stable();
    WaistQ waist_actual{};
    for (std::size_t i = 0; i < kWaistDof; ++i) waist_actual[i] = stable.q[12 + i];
    WaistQ waist_error{};
    const bool waist_matches = calligraphy_waist_matches(trajectory, waist_actual,
        options.waist_tolerance, &waist_error);
    std::cout << "WAIST_CHECK q=[" << waist_actual[0] << ',' << waist_actual[1] << ','
              << waist_actual[2] << "] reference_error=[" << waist_error[0] << ','
              << waist_error[1] << ',' << waist_error[2] << "] tolerance="
              << options.waist_tolerance << "rad\n";
    log << timestamp_local() << ",WAIST_CHECK,12," << waist_actual[0] << ','
        << stable.q[12] << ',' << waist_error[0] << ",0\n";
    log << timestamp_local() << ",WAIST_CHECK,13," << waist_actual[1] << ','
        << stable.q[13] << ',' << waist_error[1] << ",0\n";
    log << timestamp_local() << ",WAIST_CHECK,14," << waist_actual[2] << ','
        << stable.q[14] << ',' << waist_error[2] << ",0\n";
    if (!waist_matches)
      throw std::runtime_error("recording and current waist references differ beyond tolerance; no Arm SDK commands sent");

    WaistQ recorded_waist{};
    for (std::size_t i = 0; i < kWaistDof; ++i)
      recorded_waist[i] = trajectory.start_joint_position[i];
    controller.set_waist_reference_target(recorded_waist);

    controller.acquire();
    ArmQ current{};
    for (std::size_t i = 0; i < kArmDof; ++i) current[i] = stable.q[15 + i];
    const ArmQ start = calligraphy_arm_position(trajectory.samples.front().q);
    const double duration_to_start = move_duration(current, start, options.max_joint_speed);
    std::cout << "MOVE_TO_START duration=" << duration_to_start << "s\n";
    auto deadline = Clock::now();
    const auto move_started = deadline;
    auto previous = Clock::now();
    while (!controller.stop_requested()) {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - previous).count();
      previous = now;
      if (elapsed > 0.1) throw std::runtime_error("control loop interruption >100ms during MOVE_TO_START");
      const double u = std::min(1.0, std::chrono::duration<double>(now - move_started).count() /
                                      duration_to_start);
      ArmQ ref{};
      const double phase = smoothstep5(u);
      for (std::size_t i = 0; i < kArmDof; ++i) ref[i] = current[i] + phase * (start[i] - current[i]);
      const State actual = controller.latest();
      if (log_frame(log, "MOVE_TO_START", ref, actual, elapsed) > options.tracking_error_limit)
        throw std::runtime_error("tracking error exceeded limit during MOVE_TO_START");
      controller.publish(ref, options.kp, options.kd);
      if (u >= 1.0) break;
      deadline += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(deadline);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    if (controller.stop_requested()) {
      controller.release();
      controller.close();
      return 2;
    }
    if (!wait_for_enter(controller, start, "READY_START", log,
                        options.tracking_error_limit)) {
      controller.release();
      controller.close();
      return 2;
    }

    std::cout << "REPLAY speed_scale=" << options.speed_scale << '\n';
    const auto replay_started = Clock::now();
    previous = replay_started;
    deadline = replay_started;
    double source_time = 0.0;
    ArmQ replay_reference = start;
    while (!controller.stop_requested()) {
      const auto now = Clock::now();
      const double elapsed = std::chrono::duration<double>(now - previous).count();
      previous = now;
      if (elapsed > 0.1) throw std::runtime_error("control loop interruption >100ms during REPLAY");
      const double requested_step = elapsed * options.speed_scale;
      double safe_step = requested_step;
      auto candidate = interpolate_calligraphy_arm(trajectory,
                                                    std::min(duration, source_time + safe_step));
      const double allowed_motion = options.max_joint_speed * elapsed;
      for (int iteration = 0; iteration < 12; ++iteration) {
        double max_delta = 0.0;
        for (std::size_t i = 0; i < kArmDof; ++i)
          max_delta = std::max(max_delta, std::abs(candidate[i] - replay_reference[i]));
        if (max_delta <= allowed_motion || safe_step <= 1e-9) break;
        safe_step *= allowed_motion / max_delta;
        candidate = interpolate_calligraphy_arm(trajectory,
                                                 std::min(duration, source_time + safe_step));
      }
      source_time = std::min(duration, source_time + safe_step);
      const ArmQ ref = interpolate_calligraphy_arm(trajectory, source_time);
      const State actual = controller.latest();
      if (log_frame(log, "REPLAY", ref, actual, elapsed) > options.tracking_error_limit)
        throw std::runtime_error("tracking error exceeded limit during REPLAY");
      controller.publish(ref, options.kp, options.kd);
      replay_reference = ref;
      if (source_time >= duration) break;
      deadline += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(deadline);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    if (controller.stop_requested()) {
      controller.release();
      controller.close();
      return 2;
    }
    const ArmQ end = calligraphy_arm_position(trajectory.samples.back().q);
    if (!controller.stop_requested() && wait_for_enter(controller, end, "HOLD_END", log,
                                                       options.tracking_error_limit)) {
      log << timestamp_local() << ",RELEASE,0,0,0,0,0\n";
      controller.release();
      controller.close();
      return controller.faulted() ? 2 : 0;
    }
    controller.release();
    controller.close();
    return 2;
  } catch (const std::exception &error) {
    std::cerr << "g1_calligraphy_replay: " << error.what() << '\n';
    return 2;
  }
}
