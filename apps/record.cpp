#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/calligraphy_trajectory.hpp"
#include "g1_arm/trajectory.hpp"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <thread>
#include <json/json.h>
#include <fstream>

namespace {
using namespace g1_arm;
struct Options {
  std::string character, interface{"eth0"};
  std::filesystem::path directory{"data/trajectories/calligraphy"};
  double wait_timeout_s{10.0};
  double waist_pitch_extra_bias{0.05};
  std::string pose_name;
  std::filesystem::path pose_dir{"data/poses"};
};
Options parse(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--interface" && i + 1 < argc) options.interface = argv[++i];
    else if (arg == "--dir" && i + 1 < argc) options.directory = argv[++i];
    else if (arg == "--wait-timeout" && i + 1 < argc) options.wait_timeout_s = std::stod(argv[++i]);
    else if (arg == "--waist-pitch-extra-bias" && i + 1 < argc)
      options.waist_pitch_extra_bias = std::stod(argv[++i]);
    else if (arg == "--pose-teach" && i + 1 < argc) options.pose_name = argv[++i];
    else if (arg == "--pose-dir" && i + 1 < argc) options.pose_dir = argv[++i];
    else if (arg == "--help") {
      std::cout << "Usage: g1_calligraphy_record CHARACTER | --pose-teach NAME "
                   "[--interface eth0] "
                   "[--dir data/trajectories/calligraphy] [--wait-timeout 10] "
                   "[--waist-pitch-extra-bias RAD (default 0.05)]\n";
      std::exit(0);
    } else if (!arg.empty() && arg[0] != '-' && options.character.empty())
      options.character = arg;
    else throw std::runtime_error("invalid argument: " + arg);
  }
  if (options.character.empty() && options.pose_name.empty()) throw std::runtime_error("character name required");
  if (!options.pose_name.empty() && options.pose_name.find_first_of("/\\") != std::string::npos)
    throw std::runtime_error("pose name must not contain path separators");
  if (!options.character.empty() && options.character.find_first_of("/\\") != std::string::npos)
    throw std::runtime_error("character name must not contain path separators");
  if (options.interface.empty()) throw std::runtime_error("--interface must not be empty");
  if (!std::isfinite(options.wait_timeout_s) || options.wait_timeout_s <= 0.0)
    throw std::runtime_error("--wait-timeout must be positive");
  if (!std::isfinite(options.waist_pitch_extra_bias) ||
      std::abs(options.waist_pitch_extra_bias) > kMaxWaistPitchExtraBiasRad)
    throw std::runtime_error("waist pitch extra bias must be finite and within +/-0.050 rad");
  return options;
}
void usage() {
  std::cout << "F: DRAG/LOCK, C: start/stop whole-character recording, "
               "L: status, S: save stopped recording, Q: release and quit\n";
}
}

int main(int argc, char **argv) {
  using namespace g1_arm;
  try {
    const auto options = parse(argc, argv);
    auto dds = make_unitree_dds();
    ArmSdkController controller(std::move(dds));
    controller.set_wait_timeout(options.wait_timeout_s);
    controller.set_waist_pitch_extra_bias(options.waist_pitch_extra_bias);
    std::cout << "motor14 extra pitch bias=" << options.waist_pitch_extra_bias << " rad\n";
    controller.start(options.interface);
    const auto initial = controller.wait_stable();
    controller.acquire();
    std::cout << "Arm SDK acquired on rt/arm_sdk; control loop active at 50Hz\n";
    ArmQ reference{};
    for (std::size_t i = 0; i < reference.size(); ++i) reference[i] = initial.q[15 + i];
    TeachState teach_mode(reference);
    TeachMode mode = TeachMode::Lock;
  CalligraphyRecorder recorder;
    bool saved = true;
    bool done = false;
    bool quit_armed = false;
  if (!options.pose_name.empty()) std::cout << "Pose teaching: F toggles DRAG/LOCK, P saves current measured 17-DoF pose, Q releases\n";
  else usage();
    auto deadline = Clock::now();
    while (!done && !controller.stop_requested()) {
      const State state = controller.latest();
      ArmQ measured{};
      for (std::size_t i = 0; i < measured.size(); ++i) measured[i] = state.q[15 + i];
      std::string command;
      pollfd input{0, POLLIN, 0};
      if (poll(&input, 1, 0) > 0 && (input.revents & POLLIN))
        if (!std::getline(std::cin, command)) done = true;
      if (!command.empty()) {
        std::istringstream line(command);
        std::string op;
        line >> op;
        if (op == "F" || op == "f") {
          if (recorder.recording()) std::cout << "F ignored while recording; DRAG remains active\n";
          else if (mode == TeachMode::Drag) {
            mode = TeachMode::Lock;
            teach_mode.lock(measured, Clock::now());
            std::cout << "LOCK\n";
          } else {
            mode = TeachMode::Drag;
            teach_mode.drag(measured);
            std::cout << "DRAG\n";
          }
        } else if (op == "C" || op == "c") {
          if (!options.pose_name.empty()) std::cout << "C is disabled during pose teaching; use P to save a pose\n";
          else if (!recorder.recording()) {
            if (mode != TeachMode::Drag) std::cout << "Enter DRAG with F before recording\n";
            else {
              mode = TeachMode::Drag;
              teach_mode.drag(measured);
              recorder.start(state, Clock::now(), timestamp_local());
              recorder.trajectory().character_name = options.character;
              recorder.trajectory().waist_pitch_extra_bias_known = true;
              recorder.trajectory().waist_pitch_extra_bias = options.waist_pitch_extra_bias;
              recorder.trajectory().effective_waist_control_mode = "WaistHold_fixed";
              recorder.trajectory().configuration_snapshot["config_version"] = "1.0";
              recorder.trajectory().configuration_snapshot["waist_pitch_extra_bias_rad"] = options.waist_pitch_extra_bias;
              recorder.trajectory().configuration_snapshot["waist_reference_source"] = "recording LowState";
              saved = false;
              quit_armed = false;
              std::cout << "RECORDING\n";
            }
          } else {
            recorder.stop(state);
            std::cout << "RECORD_STOPPED frames=" << recorder.trajectory().samples.size()
                      << " duration=" << recorder.trajectory().samples.back().t_monotonic
                      << "s rate=" << recorder.trajectory().sample_rate_actual << "Hz\n";
          }
        } else if ((op == "P" || op == "p") && !options.pose_name.empty()) {
          const auto current = controller.latest();
          Json::Value pose;
          pose["schema_version"] = 1;
          pose["pose_name"] = options.pose_name;
          pose["recorded_at"] = timestamp_local();
          pose["motor_indices"] = Json::arrayValue;
          pose["q"] = Json::arrayValue;
          for (int motor = 12; motor <= 28; ++motor) {
            pose["motor_indices"].append(motor);
            pose["q"].append(current.q[motor]);
          }
          pose["calibration_refs"]["paper"] = Json::nullValue;
          pose["calibration_refs"]["tcp"] = Json::nullValue;
          pose["tcp_pose"] = Json::nullValue;
          std::filesystem::create_directories(options.pose_dir);
          Json::StreamWriterBuilder writer; writer["indentation"] = "  ";
          const auto path = options.pose_dir / (options.pose_name + ".json");
          std::ofstream out(path);
          if (!out) throw std::runtime_error("cannot write pose: " + path.string());
          out << Json::writeString(writer, pose) << '\n';
          std::cout << "Saved measured pose " << path << "\n";
        } else if (op == "L" || op == "l") {
          const auto &data = recorder.trajectory();
          std::cout << (recorder.recording() ? "RECORDING" : (recorder.has_data() ? "STOPPED" : "IDLE"))
                    << " duration=" << (data.samples.empty() ? 0.0 : data.samples.back().t_monotonic)
                    << "s frames=" << data.samples.size() << " rate=" << data.sample_rate_actual
                    << "Hz missed=" << data.missed_samples << "\n";
        } else if (op == "S" || op == "s") {
          if (recorder.recording()) std::cout << "Stop recording with C before saving\n";
          else if (!recorder.has_data()) std::cout << "No stopped recording to save\n";
          else {
            auto filename_time = timestamp_filename_local();
            std::replace(filename_time.begin(), filename_time.end(), '-', '_');
            std::replace(filename_time.begin(), filename_time.end(), ':', '_');
            if (options.character.empty()) { std::cout << "S is only available for character recording\n"; continue; }
            const auto path = calligraphy_trajectory_path(options.directory, options.character,
                                                          filename_time);
            save_calligraphy_trajectory(recorder.trajectory(), path);
            saved = true;
            std::cout << "Saved " << path << "\n";
          }
        } else if (op == "Q" || op == "q") {
          if (recorder.recording()) {
            std::cout << "Stop recording with C before quitting\n";
          } else if (recorder.has_data() && !saved && !quit_armed) {
            std::cout << "Unsaved recording in memory. Press Q again to discard and release.\n";
            quit_armed = true;
          } else done = true;
        } else usage();
      }
      if (mode == TeachMode::Drag) reference = teach_mode.update(measured);
      recorder.capture(state, Clock::now());
      controller.publish(reference, teach_mode.mode(), 1.0);
      deadline += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(deadline);
      if (deadline < Clock::now()) deadline = Clock::now();
    }
    controller.release();
    controller.close();
    return controller.faulted() ? 2 : 0;
  } catch (const std::exception &error) {
    std::cerr << "g1_calligraphy_record: " << error.what() << '\n';
    return 2;
  }
}
