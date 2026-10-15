#pragma once

#include "g1_arm/types.hpp"
#include <filesystem>

namespace g1_arm {
void validate_name(const std::string &name);
void validate_trajectory(const Trajectory &trajectory);
void set_point(Trajectory &trajectory, const Point &point);
void delete_point(Trajectory &trajectory, const std::string &name);
std::string timestamp_local();
std::string timestamp_filename_local();
std::filesystem::path trajectory_path(const std::filesystem::path &directory,
                                      const std::string &name);
Trajectory load_trajectory(const std::filesystem::path &path);
void save_trajectory(const Trajectory &trajectory,
                     const std::filesystem::path &path);
double smoothstep(double x);
double motion_duration(const ArmQ &from, const ArmQ &to, double speed_rad_s,
                       double minimum_s = 0.5);
ArmQ interpolate(const ArmQ &from, const ArmQ &to, double elapsed_s,
                  double duration_s);
} // namespace g1_arm
