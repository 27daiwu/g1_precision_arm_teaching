#include "g1_arm/pose.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace {
void usage() {
  std::cout << "Usage: g1_calligraphy <pose|record|replay|validate|list|run> ...\n"
               "  pose record NAME [--interface eth0] [--pose-dir data/poses] (read only)\n"
               "  pose teach NAME [record options] (arm DRAG/LOCK, fixed waist)\n"
               "  pose list [--pose-dir DIR] | pose show NAME [--pose-dir DIR]\n"
               "  record CHARACTER [record options]\n"
               "  replay CHARACTER [replay options]\n"
               "  validate FILE | list | run CHARACTER (integration reserved)\n";
}
[[noreturn]] void exec_binary(const std::string &name, std::vector<std::string> args) {
  const auto self = std::filesystem::canonical("/proc/self/exe");
  const auto path = self.parent_path() / name;
  std::vector<char *> argv;
  argv.push_back(const_cast<char *>(path.c_str()));
  for (auto &arg : args) argv.push_back(arg.data());
  argv.push_back(nullptr);
  execv(path.c_str(), argv.data());
  throw std::runtime_error("cannot execute " + path.string());
}
}

int main(int argc, char **argv) {
  try {
    if (argc < 2 || std::string(argv[1]) == "--help") { usage(); return argc < 2 ? 2 : 0; }
    const std::string command = argv[1];
    if (command == "pose") {
      if (argc < 3) { usage(); return 2; }
      const std::string action = argv[2];
      if (action == "--help") { usage(); return 0; }
      if (action == "teach" || action == "record") {
        if (argc < 4) throw std::runtime_error("pose " + action + " requires NAME");
        if (std::string(argv[3]) == "--help") exec_binary("g1_calligraphy_record", {"--help"});
        std::vector<std::string> args{"--pose-teach", argv[3]};
        if (action == "record") args.emplace_back("--read-only");
        for (int i = 4; i < argc; ++i) args.emplace_back(argv[i]);
        exec_binary("g1_calligraphy_record", std::move(args));
      }
      std::filesystem::path dir = "data/poses";
      const bool show = action == "show";
      if (show && argc < 4) throw std::runtime_error("pose show requires NAME");
      for (int i = show ? 4 : 3; i < argc; ++i) {
        if (std::string(argv[i]) == "--pose-dir" && i + 1 < argc) dir = argv[++i];
        else throw std::runtime_error("invalid pose argument: " + std::string(argv[i]));
      }
      if (action == "list") {
        if (!std::filesystem::exists(dir)) return 0;
        for (const auto &entry : std::filesystem::directory_iterator(dir))
          if (entry.path().extension() == ".json") std::cout << entry.path().stem().string() << '\n';
        return 0;
      }
      if (show) {
        g1_arm::validate_pose_name(argv[3]);
        const auto path = dir / (std::string(argv[3]) + ".json");
        Json::Value value; Json::CharReaderBuilder builder; std::string errors;
        std::ifstream input(path);
        if (!input || !Json::parseFromStream(builder, input, &value, &errors))
          throw std::runtime_error("cannot read pose " + path.string());
        Json::StreamWriterBuilder writer; writer["indentation"] = "  ";
        std::cout << Json::writeString(writer, value) << '\n';
        return 0;
      }
      throw std::runtime_error("pose action must be record, teach, list, or show");
    }
    if (command == "record") {
      if (argc < 3) throw std::runtime_error("record requires CHARACTER");
      std::vector<std::string> args;
      for (int i = 2; i < argc; ++i) args.emplace_back(argv[i]);
      exec_binary("g1_calligraphy_record", std::move(args));
    }
    if (command == "replay" || command == "validate") {
      const bool validate = command == "validate";
      if (argc < 3) throw std::runtime_error(command + " requires CHARACTER or FILE");
      std::filesystem::path file;
      std::vector<std::string> args;
      if (validate) file = argv[2];
      else {
        const std::filesystem::path dir = "data/trajectories/calligraphy";
        const std::string character = argv[2];
        std::filesystem::file_time_type newest{};
        if (std::filesystem::exists(dir)) for (const auto &entry : std::filesystem::directory_iterator(dir)) {
          const auto name = entry.path().filename().string();
          if (entry.path().extension() == ".json" && name.rfind(character + "_", 0) == 0 &&
              std::filesystem::last_write_time(entry) >= newest) {
            newest = std::filesystem::last_write_time(entry); file = entry.path();
          }
        }
        if (file.empty()) throw std::runtime_error("no recording found for " + character);
      }
      args = {"--file", file.string()};
      if (validate) args.push_back("--validate-only");
      else for (int i = 3; i < argc; ++i) args.emplace_back(argv[i]);
      exec_binary("g1_calligraphy_replay", std::move(args));
    }
    if (command == "list") {
      const std::filesystem::path dir = "data/trajectories/calligraphy";
      if (std::filesystem::exists(dir)) for (const auto &entry : std::filesystem::directory_iterator(dir))
        if (entry.path().extension() == ".json") std::cout << entry.path().stem().string() << '\n';
      return 0;
    }
    if (command == "run") {
      std::cerr << "run is reserved: external PICK/TRANSFER/WRITE interface and grasp-success confirmation are not integrated; no motion started.\n";
      return 3;
    }
    usage();
    return 2;
  } catch (const std::exception &e) {
    std::cerr << "g1_calligraphy: " << e.what() << '\n';
    return 2;
  }
}
