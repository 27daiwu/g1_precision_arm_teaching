#pragma once

#include "g1_arm/trajectory.hpp"

namespace g1_arm {
class Recorder {
public:
  explicit Recorder(Trajectory trajectory) : trajectory_(std::move(trajectory)) {}
  void mark(const State &state);
  void replace(const std::string &name, const State &state);
  void erase(const std::string &name);
  void rename(std::string name);
  const Trajectory &trajectory() const { return trajectory_; }
  Trajectory &trajectory() { return trajectory_; }
private:
  Trajectory trajectory_;
};
} // namespace g1_arm
