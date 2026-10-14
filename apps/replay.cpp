#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/trajectory.hpp"

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
  unsigned cycles{1};
  double hold{-1.0};
  double speed{.5};
  bool forever{false};
  bool preview{false};
  bool list{false};
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
    else if (arg == "--forever") options.forever = true;
    else if (arg == "--preview") options.preview = true;
    else if (arg == "--list") options.list = true;
    else if (arg == "--help") {
      std::cout << "Usage: g1_replay NAME [--interface eth0] [--cycles N|--forever] "
                   "[--hold SEC] [--speed RAD_PER_SEC] [--preview]\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] != '-' && options.name.empty()) options.name = arg;
    else throw std::runtime_error("invalid argument: " + arg);
  }
  if (options.list && options.name.empty()) return options;
  if (options.name.empty()) throw std::runtime_error("trajectory name required");
  validate_name(options.name);
  if (!options.forever && options.cycles == 0) throw std::runtime_error("cycles must be positive");
  if (!std::isfinite(options.speed) || options.speed <= 0) throw std::runtime_error("speed must be positive");
  if (!std::isfinite(options.hold) || options.hold < -1 || options.hold > 600)
    throw std::runtime_error("hold must be 0..600 seconds");
  if (!options.preview && options.interface.empty()) throw std::runtime_error("--interface is required");
  return options;
}
void move(ArmSdkController &controller, const ArmQ &from, const ArmQ &to,
          double duration) {
  const auto started = Clock::now();
  auto deadline = started;
  while (true) {
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
    controller.publish(q, TeachMode::Lock);
    deadline += std::chrono::milliseconds(20);
    std::this_thread::sleep_until(deadline);
  }
}
void list(const std::filesystem::path &directory) {
  if (!std::filesystem::exists(directory)) return;
  for (const auto &entry : std::filesystem::directory_iterator(directory))
    if (entry.is_regular_file() && entry.path().extension() == ".json") {
      try {
        const auto trajectory = load_trajectory(entry.path());
        std::cout << trajectory.name << " " << trajectory.points.size() << " points\n";
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
    controller.start(options.interface);
    controller.acquire();
    auto measured = controller.latest();
    ArmQ current{};
    for (int i = 0; i < 14; ++i) current[i] = measured.q[15 + i];
    move(controller, current, trajectory.points.front().q,
         motion_duration(current, trajectory.points.front().q, options.speed));
    unsigned cycle = 0;
    while (options.forever || cycle < options.cycles) {
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
