#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/recorder.hpp"

#include <iostream>
#include <poll.h>
#include <sstream>
#include <thread>

namespace {
using namespace g1_arm;
struct Options { std::string name, interface{""}; std::filesystem::path directory{"trajectories"}; };
Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--interface" && i + 1 < argc) options.interface = argv[++i];
    else if (arg == "--dir" && i + 1 < argc) options.directory = argv[++i];
    else if (arg == "--help") {
      std::cout << "Usage: g1_record NAME --interface eth0 [--dir trajectories]\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] != '-' && options.name.empty()) options.name = arg;
    else throw std::runtime_error("invalid argument: " + arg);
  }
  if (options.name.empty()) throw std::runtime_error("trajectory name required");
  g1_arm::validate_name(options.name);
  if (options.interface.empty()) throw std::runtime_error("--interface is required");
  return options;
}
void show(const Trajectory &trajectory) {
  std::cout << "Trajectory " << trajectory.name << " (motor15..28 rad)\n";
  for (const auto &point : trajectory.points) {
    std::cout << point.name << " " << point.recorded_at << " hold=" << point.hold_s;
    for (double q : point.q) std::cout << ' ' << q;
    std::cout << '\n';
  }
}
void usage() {
  std::cout << "F: toggle DRAG/LOCK, M: lock + mark, L: list, R NAME: replace, "
               "D NAME: delete, N NAME: rename, S: save/release, Q: discard/release\n";
}
}

int main(int argc, char **argv) {
  using namespace g1_arm;
  try {
    const auto options = parse(argc, argv);
    const auto path = trajectory_path(options.directory, options.name);
    if (std::filesystem::exists(path)) throw std::runtime_error("trajectory already exists");
    auto dds = make_unitree_dds();
    ArmSdkController controller(std::move(dds));
    controller.start(options.interface);
    const auto initial = controller.latest();
    Trajectory trajectory;
    trajectory.name = options.name;
    trajectory.recorded_at = timestamp_local();
    for (int i = 0; i < 3; ++i) trajectory.waist_reference[i] = initial.q[12 + i];
    Recorder recorder(std::move(trajectory));
    controller.acquire();
    ArmQ reference{};
    for (int i = 0; i < 14; ++i) reference[i] = initial.q[15 + i];
    TeachMode mode = TeachMode::Lock;
    TeachState teach_mode(reference);
    bool done = false;
    usage();
    auto deadline = Clock::now();
    while (!done && !controller.stop_requested()) {
      const auto state = controller.latest();
      ArmQ measured{};
      for (int i = 0; i < 14; ++i) measured[i] = state.q[15 + i];
      std::string command;
      pollfd input{0, POLLIN, 0};
      if (poll(&input, 1, 0) > 0) {
        if (!std::getline(std::cin, command)) done = true;
      }
      if (!command.empty()) {
        std::istringstream input_line(command);
        std::string op, name;
        input_line >> op >> name;
        if (op == "F" || op == "f") {
          if (mode == TeachMode::Drag) {
            mode = TeachMode::Lock;
            teach_mode.lock(measured, Clock::now());
          } else {
            mode = TeachMode::Drag;
            teach_mode.drag(measured);
          }
          std::cout << (mode == TeachMode::Drag ? "DRAG\n" : "LOCK\n");
        } else if (op == "M" || op == "m") {
          mode = TeachMode::Lock;
          teach_mode.lock(measured, Clock::now());
          reference = measured;
          recorder.mark(state);
          const auto &point = recorder.trajectory().points.back();
          std::cout << "Locked and recorded " << point.name << '\n';
        } else if (op == "L" || op == "l") show(recorder.trajectory());
        else if (op == "R" || op == "r") {
          if (name.empty()) throw std::runtime_error("R requires point name");
          mode = TeachMode::Lock;
          teach_mode.lock(measured, Clock::now());
          reference = measured;
          recorder.replace(name, state);
          std::cout << "Updated " << name << '\n';
        } else if (op == "D" || op == "d") recorder.erase(name);
        else if (op == "N" || op == "n") recorder.rename(name);
        else if (op == "S" || op == "s") {
          save_trajectory(recorder.trajectory(), path);
          done = true;
          break;
        } else if (op == "Q" || op == "q") { done = true; break; }
        else usage();
      }
      if (mode == TeachMode::Drag) reference = teach_mode.update(measured);
      controller.publish(reference, teach_mode.gains(Clock::now()));
      deadline += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(deadline);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    controller.release();
    done = true;
    controller.close();
    return controller.faulted() ? 2 : 0;
  } catch (const std::exception &error) {
    std::cerr << "g1_record: " << error.what() << '\n';
    return 2;
  }
}
