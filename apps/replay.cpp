#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/continuous_trajectory.hpp"
#include "g1_arm/feedback_replay_controller.hpp"
#include "g1_arm/trajectory.hpp"
#include "g1_arm/trajectory_processor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace {
using namespace g1_arm;
struct Options {
  std::string name;
  std::string interface;
  std::filesystem::path directory{"trajectories"};
  double wait_timeout_s{10.0};
  unsigned cycles{1};
  double hold{-1.0};
  double speed{.5};
  double speed_scale{.25};
  double motion_scale{1.0};
  double position_tolerance{.02};
  double node_timeout{5.0};
  bool forever{false};
  bool preview{false};
  bool list{false};
  bool continuous{false};
  bool high_precision{false};
};
Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--interface" && i + 1 < argc) options.interface = argv[++i];
    else if (arg == "--dir" && i + 1 < argc) options.directory = argv[++i];
    else if (arg == "--cycles" && i + 1 < argc) options.cycles = std::stoul(argv[++i]);
    else if (arg == "--hold" && i + 1 < argc) options.hold = std::stod(argv[++i]);
    else if (arg == "--speed" && i + 1 < argc) options.speed = std::stod(argv[++i]);
    else if (arg == "--speed-scale" && i + 1 < argc) options.speed_scale = std::stod(argv[++i]);
    else if (arg == "--motion-scale" && i + 1 < argc) options.motion_scale = std::stod(argv[++i]);
    else if (arg == "--position-tolerance" && i + 1 < argc) options.position_tolerance = std::stod(argv[++i]);
    else if (arg == "--node-timeout" && i + 1 < argc) options.node_timeout = std::stod(argv[++i]);
    else if (arg == "--wait-timeout" && i + 1 < argc) options.wait_timeout_s = std::stod(argv[++i]);
    else if (arg == "--forever") options.forever = true;
    else if (arg == "--preview") options.preview = true;
    else if (arg == "--list") options.list = true;
    else if (arg == "--continuous") options.continuous = true;
    else if (arg == "--high-precision") options.high_precision = true;
    else if (arg == "--help") {
      std::cout << "Usage: g1_replay NAME [--continuous] [--high-precision] [--interface eth0] [--cycles N|--forever] "
                   "[--hold SEC] [--speed RAD_PER_SEC] [--speed-scale 0.25] "
                   "[--motion-scale 1.0] [--position-tolerance 0.02] [--node-timeout 5] "
                   "[--wait-timeout SEC] [--preview]\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] != '-' && options.name.empty()) options.name = arg;
    else throw std::runtime_error("invalid argument: " + arg);
  }
  if (options.list && options.name.empty()) return options;
  if (options.name.empty()) throw std::runtime_error("trajectory name required");
  validate_name(options.name);
  if (!options.forever && options.cycles == 0) throw std::runtime_error("cycles must be positive");
  if (!std::isfinite(options.speed) || options.speed <= 0) throw std::runtime_error("speed must be positive");
  if (!std::isfinite(options.speed_scale) || options.speed_scale <= 0)
    throw std::runtime_error("speed scale must be positive");
  if (!std::isfinite(options.motion_scale) || options.motion_scale < 0)
    throw std::runtime_error("motion scale must be non-negative");
  if (!std::isfinite(options.position_tolerance) || options.position_tolerance <= 0 ||
      !std::isfinite(options.node_timeout) || options.node_timeout <= 0)
    throw std::runtime_error("position tolerance and node timeout must be positive");
  if (options.high_precision && !options.continuous)
    throw std::runtime_error("--high-precision requires --continuous");
  if (!std::isfinite(options.hold) || options.hold < -1 || options.hold > 600)
    throw std::runtime_error("hold must be 0..600 seconds");
  if (!options.preview && options.interface.empty()) throw std::runtime_error("--interface is required");
  if (!std::isfinite(options.wait_timeout_s) || options.wait_timeout_s <= 0.0)
    throw std::runtime_error("wait timeout must be positive");
  return options;
}
void move(ArmSdkController &controller, const ArmQ &from, const ArmQ &to,
          double duration) {
  const auto started = Clock::now();
  auto deadline = started;
  while (true) {
    if (controller.stop_requested()) break;
    const double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
    const double t = std::min(elapsed, duration);
    controller.publish(interpolate(from, to, t, duration), TeachMode::Lock);
    if (t >= duration) break;
    deadline += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(deadline);
  }
}
void wait_hold(ArmSdkController &controller, const ArmQ &q, double seconds) {
  const auto until = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                         std::chrono::duration<double>(seconds));
  auto deadline = Clock::now();
  while (Clock::now() < until) {
    if (controller.stop_requested()) break;
    controller.publish(q, TeachMode::Lock);
    deadline += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(deadline);
  }
}
void list(const std::filesystem::path &directory) {
  if (!std::filesystem::exists(directory)) return;
  for (const auto &entry : std::filesystem::directory_iterator(directory))
    if (entry.is_regular_file() && entry.path().extension() == ".json" &&
        entry.path().filename().string().find("_continuous.json") == std::string::npos) {
      try {
        const auto trajectory = load_trajectory(entry.path());
        std::cout << trajectory.name << " " << trajectory.points.size() << " points\n";
      } catch (const std::exception &error) {
        const std::string message = error.what();
        if (message.find("unsupported trajectory JSON version") != std::string::npos)
          std::cout << entry.path().filename().string()
                    << " (legacy session metadata; offline conversion required)\n";
        else
          std::cout << entry.path().filename().string() << " INVALID: " << message << '\n';
      }
    }
  for (const auto &entry : std::filesystem::directory_iterator(directory))
    if (entry.is_regular_file() && entry.path().filename().string().find("_continuous.json") != std::string::npos) {
      try {
        const auto trajectory = load_continuous_trajectory(entry.path());
        std::cout << trajectory.name << " continuous " << trajectory.samples.size() << " frames\n";
      } catch (const std::exception &error) {
        std::cout << entry.path().filename().string() << " INVALID: " << error.what() << '\n';
      }
    }
}
}

int main(int argc, char **argv) {
  using namespace g1_arm;
  try {
    const auto options = parse(argc, argv);
    if (options.list) { list(options.directory); return 0; }
    if (options.continuous) {
      const auto trajectory = load_continuous_trajectory(
          continuous_trajectory_path(options.directory, options.name));
      const double duration = trajectory.samples.back().t_s;
      std::cout << trajectory.name << " continuous " << trajectory.samples.size()
                << " frames, duration=" << duration << "s, speed_scale="
                << options.speed_scale << ", motion_scale=" << options.motion_scale << '\n';
      ProcessedTrajectory processed;
      if (options.high_precision) {
        processed = process_trajectory(trajectory);
        save_processed_trajectory(processed, options.directory /
            (options.name + "_processed.json"));
        std::cout << "high precision nodes=" << processed.nodes.size()
                  << " important=" << std::count_if(processed.nodes.begin(), processed.nodes.end(),
                       [](const ProcessedNode &node) { return node.important; })
                  << " geometric_deviation=" << processed.max_geometric_deviation_rad << "rad\n";
        if (options.preview) {
          return 0;
        }
      }
      if (options.preview) return 0;
      ArmSdkController controller(make_unitree_dds());
      controller.set_wait_timeout(options.wait_timeout_s);
      controller.start(options.interface);
      const State stable_state = controller.wait_stable();
      controller.acquire();
      ArmQ acquire_q{};
      for (std::size_t i = 0; i < acquire_q.size(); ++i) acquire_q[i] = stable_state.q[15 + i];
      if (options.high_precision) {
        RetimerConfig retimer_config;
        retimer_config.speed_scale = options.speed_scale;
        FeedbackConfig feedback_config;
        feedback_config.position_tolerance_rad = options.position_tolerance;
        feedback_config.node_timeout_s = options.node_timeout;
        std::filesystem::create_directories("logs");
        const auto feedback_log_path = std::filesystem::path("logs") /
            ("g1_high_precision_" + timestamp_filename_local() + ".csv");
        std::ofstream feedback_log(feedback_log_path);
        if (!feedback_log) throw std::runtime_error("cannot create high precision replay log");
        feedback_log << write_feedback_csv_header() << '\n';
        const auto task_started = Clock::now();
        std::size_t total_reached = 0, total_nodes = 0;
        std::array<double, kArmDof> aggregate_squared_errors{};
        std::size_t aggregate_samples = 0;
        double aggregate_max_error = 0.0;
        bool abort = false;
        for (unsigned cycle = 0; !controller.stop_requested() &&
             (options.forever || cycle < options.cycles); ++cycle) {
          for (int leg = 0; leg < 2 && !controller.stop_requested(); ++leg) {
            ProcessedTrajectory path = processed;
            ArmQ anchor = acquire_q;
            if (leg == 1) {
              const State current_state = controller.latest();
              for (std::size_t i = 0; i < kArmDof; ++i)
                anchor[i] = current_state.q[15 + i];
              const ArmQ zero = processed.nodes.front().q;
              for (std::size_t i = 0; i < path.nodes.size(); ++i) {
                const double u = static_cast<double>(i) / (path.nodes.size() - 1);
                for (std::size_t j = 0; j < kArmDof; ++j)
                  path.nodes[i].q[j] = (1.0 - u) * processed.nodes.back().q[j] + u * zero[j];
              }
              std::reverse(path.nodes.begin(), path.nodes.end());
              for (auto &node : path.nodes) node.s = 1.0 - node.s;
              std::sort(path.nodes.begin(), path.nodes.end(),
                  [](const ProcessedNode &a, const ProcessedNode &b) { return a.s < b.s; });
              path.recording_start_q = zero;
              for (std::size_t j = 0; j < kArmDof; ++j)
                path.nodes.front().q[j] = processed.nodes.back().q[j];
            }
            FeedbackReplayController replay(path, anchor, retimer_config, feedback_config);
            Clock::time_point deadline = Clock::now();
            auto previous = Clock::now();
            while (!controller.stop_requested()) {
              const auto now = Clock::now();
              const double dt = std::chrono::duration<double>(now - previous).count();
              previous = now;
              const State state = controller.latest();
              const auto output = replay.step(state, std::clamp(dt, .001, .1));
              write_feedback_csv_row(feedback_log, timestamp_local(), output, state,
                                     std::clamp(dt, .001, .1));
              controller.publish(output.q_ref, TeachMode::Lock);
              if (output.aborted) {
                feedback_log << "# NOT_REACHED " << replay.abort_reason() << '\n';
                const auto failed_node = replay.current_node();
                feedback_log << "# NOT_REACHED_JOINT_ERRORS node=" << failed_node;
                for (double error : replay.node_joint_errors()[failed_node])
                  feedback_log << ',' << error;
                feedback_log << '\n';
                std::cerr << "NOT_REACHED: " << replay.abort_reason() << '\n';
                abort = true;
                break;
              }
              if (output.done) break;
              deadline += std::chrono::milliseconds(20);
              std::this_thread::sleep_until(deadline);
              if (deadline < Clock::now()) deadline = Clock::now();
            }
            if (controller.stop_requested()) {
              abort = false;
              break;
            }
            const auto &node_flags = replay.node_reached();
            for (std::size_t i = 0; i < node_flags.size(); ++i) {
              if (!path.nodes[i].important) continue;
              ++total_nodes;
              if (node_flags[i]) ++total_reached;
              feedback_log << "# NODE cycle=" << cycle << " leg=" << leg
                  << " index=" << i << " reached=" << node_flags[i]
                  << " max_error_rad=" << replay.node_errors()[i] << " joint_errors_rad="
                  << replay.node_joint_errors()[i][0];
              if (path.nodes[i].important) {
                for (std::size_t joint = 1; joint < kArmDof; ++joint)
                  feedback_log << ',' << replay.node_joint_errors()[i][joint];
                feedback_log << '\n';
              }
            }
            const auto rms = replay.rms_error();
            const auto count = replay.sample_count();
            for (std::size_t i = 0; i < kArmDof; ++i)
              aggregate_squared_errors[i] += rms[i] * rms[i] * count;
            aggregate_samples += count;
            aggregate_max_error = std::max(aggregate_max_error, replay.max_tracking_error());
            if (abort) break;
          }
          if (abort) break;
        }
        feedback_log.flush();
        const double replay_elapsed = std::chrono::duration<double>(Clock::now() - task_started).count();
        std::cout << "high precision replay " << (abort ? "NOT_REACHED" : "complete")
                  << ", elapsed=" << replay_elapsed << "s, original="
                  << processed.source_duration_s << "s, extension="
                  << (processed.source_duration_s > 0 ? replay_elapsed / processed.source_duration_s : 0)
                  << "x, max_geometry_deviation=" << processed.max_geometric_deviation_rad
                  << "rad, node_reached=" << total_reached << '/' << total_nodes
                  << ", log=" << feedback_log_path.string() << '\n';
        std::cout << "maximum_tracking_error=" << aggregate_max_error << "rad, per_joint_rms_rad=";
        for (std::size_t i = 0; i < kArmDof; ++i) {
          const double rms = aggregate_samples
              ? std::sqrt(aggregate_squared_errors[i] / aggregate_samples) : 0.0;
          std::cout << (i ? "," : "") << rms;
        }
        std::cout << '\n';
        controller.release();
        controller.close();
        return abort || controller.faulted() ? 2 : 0;
      }
      auto deadline = Clock::now();
      unsigned cycle = 0;
      while (!controller.stop_requested() && (options.forever || cycle < options.cycles)) {
        const auto cycle_started = Clock::now();
        while (!controller.stop_requested()) {
          const double elapsed = std::chrono::duration<double>(Clock::now() - cycle_started).count();
          const double source_time = elapsed * options.speed_scale;
          if (source_time >= duration) break;
          controller.publish(interpolate_continuous(trajectory, source_time, acquire_q,
                                                    options.motion_scale), TeachMode::Lock);
          deadline += std::chrono::milliseconds(20);
          std::this_thread::sleep_until(deadline);
        }
        if (controller.stop_requested()) break;
        const ArmQ end_q = relative_continuous_target(acquire_q, trajectory.samples.back().q,
                                                       trajectory.recording_start_q,
                                                       options.motion_scale);
        controller.publish(end_q, TeachMode::Lock);
        const double return_duration = motion_duration(
            end_q, acquire_q, options.speed * options.speed_scale);
        const auto return_started = Clock::now();
        while (!controller.stop_requested()) {
          const double elapsed = std::chrono::duration<double>(Clock::now() - return_started).count();
          if (elapsed >= return_duration) break;
          controller.publish(interpolate(end_q, acquire_q, elapsed, return_duration), TeachMode::Lock);
          deadline += std::chrono::milliseconds(20);
          std::this_thread::sleep_until(deadline);
        }
        controller.publish(acquire_q, TeachMode::Lock);
        ++cycle;
      }
      controller.release();
      controller.close();
      return controller.faulted() ? 2 : 0;
    }
    const auto trajectory = load_trajectory(trajectory_path(options.directory, options.name));
    for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
      const auto &from = trajectory.points[i];
      const auto &to = trajectory.points[(i + 1) % trajectory.points.size()];
      std::cout << from.name << " -> " << to.name << " duration="
                << motion_duration(from.q, to.q, options.speed) << "s hold="
                << (options.hold >= 0 ? options.hold : from.hold_s) << "s\n";
    }
    if (options.preview) return 0;
    ArmSdkController controller(make_unitree_dds());
    controller.set_wait_timeout(options.wait_timeout_s);
    controller.start(options.interface);
    const auto stable_state = controller.wait_stable();
    controller.acquire();
    ArmQ current{};
    for (int i = 0; i < 14; ++i) current[i] = stable_state.q[15 + i];
    move(controller, current, trajectory.points.front().q,
         motion_duration(current, trajectory.points.front().q, options.speed));
    unsigned cycle = 0;
    while (!controller.stop_requested() && (options.forever || cycle < options.cycles)) {
      for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
        const auto &point = trajectory.points[i];
        const auto &next = trajectory.points[(i + 1) % trajectory.points.size()];
        const double hold = options.hold >= 0 ? options.hold : point.hold_s;
        wait_hold(controller, point.q, hold);
        move(controller, point.q, next.q,
             motion_duration(point.q, next.q, options.speed));
      }
      ++cycle;
    }
    controller.release();
    controller.close();
    return controller.faulted() ? 2 : 0;
  } catch (const std::exception &error) {
    std::cerr << "g1_replay: " << error.what() << '\n';
    return 2;
  }
}
