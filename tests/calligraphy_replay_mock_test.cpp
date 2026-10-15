#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/calligraphy_trajectory.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace {
using namespace g1_arm;
class MockDds final : public Dds {
public:
  void initialize(const std::string &) override {
    state.q.fill(0.0);
    for (int motor = 12; motor <= 28; ++motor) state.q[motor] = .001 * motor;
    state.received_at = Clock::now();
  }
  void activate_publisher() override { active = true; }
  State latest() const override {
    auto current = state;
    current.received_at = Clock::now();
    return current;
  }
  bool publish(const ControlFrame &frame, std::string &) override {
    frames.push_back(frame);
    return true;
  }
  void close() noexcept override { closed = true; }
  State state;
  std::vector<ControlFrame> frames;
  bool active{false};
  bool closed{false};
};
}

int main() {
  using namespace g1_arm;
  const auto tmp = std::filesystem::temp_directory_path() / "g1_calligraphy_replay_mock";
  std::filesystem::remove_all(tmp);
  std::filesystem::create_directories(tmp);
  auto wire = std::make_unique<MockDds>();
  auto *mock = wire.get();
  ArmSdkController controller(std::move(wire), tmp / "logs");
  controller.set_waist_pitch_extra_bias(.05);
  WaistQ waist_target{};
  for (int i = 0; i < 3; ++i) waist_target[i] = .001 * (12 + i);
  waist_target[2] += .02;
  controller.set_waist_reference_target(waist_target);
  controller.start("mock");
  StabilityConfig stable;
  stable.window_s = .04;
  stable.wait_timeout_s = .5;
  stable.filter_tau_s = .05;
  const State initial = controller.wait_stable(stable);
  controller.acquire();
  assert(mock->active && !mock->frames.empty());

  CalligraphyTrajectory trajectory;
  trajectory.character_name = "mock";
  trajectory.waist_pitch_extra_bias_known = true;
  trajectory.waist_pitch_extra_bias = .05;
  for (int frame = 0; frame < 3; ++frame) {
    CalligraphySample sample;
    sample.t_monotonic = frame * .04;
    for (int joint = 0; joint < 14; ++joint) {
      sample.q[joint + 3] = .1 + frame * .01 + joint * .001;
      sample.dq[joint + 3] = .02;
    }
    sample.q[0] = initial.q[12];
    sample.q[1] = initial.q[13];
    sample.q[2] = waist_target[2];
    trajectory.samples.push_back(sample);
  }
  trajectory.start_joint_position = trajectory.samples.front().q;
  trajectory.end_joint_position = trajectory.samples.back().q;

  ArmQ current{};
  for (int i = 0; i < 14; ++i) current[i] = initial.q[15 + i];
  const ArmQ first = interpolate_calligraphy_arm(trajectory, 0.0);
  for (int i = 0; i < 14; ++i) assert(first[i] == trajectory.samples.front().q[i + 3]);
  const ArmQ middle = interpolate_calligraphy_arm(trajectory, .02);
  assert(std::abs(middle[0] - (trajectory.samples[0].q[3] +
                               trajectory.samples[1].q[3]) / 2.0) < 1e-12);
  const ArmQ last = interpolate_calligraphy_arm(trajectory, 99.0);
  assert(last[13] == trajectory.samples.back().q[16]);

  controller.publish(current, TeachMode::Lock);
  assert(mock->frames.back().q[12] == initial.q[12]);
  assert(mock->frames.back().q[13] == initial.q[13]);
  assert(std::abs(mock->frames.back().q[14] - waist_target[2]) < 1e-6);
  controller.publish(first, TeachMode::Lock);
  for (int motor = 15; motor <= 28; ++motor)
    assert(std::abs(mock->frames.back().q[motor] - trajectory.samples.front().q[motor - 12]) < 1e-9);
  controller.publish(middle, TeachMode::Lock);
  controller.publish(last, TeachMode::Lock);
  const auto end_frame = mock->frames.size();
  controller.publish(last, TeachMode::Lock);
  assert(mock->frames.size() == end_frame + 1);
  assert(std::abs(mock->frames.back().q[14] - waist_target[2]) < 1e-6);
  const auto release_start = mock->frames.size();
  controller.release();
  assert(mock->frames.back().weight == 0.0);
  assert(std::abs(mock->frames.back().q[14] - initial.q[14]) < 1e-6);
  assert(mock->frames[release_start].q[14] > mock->frames.back().q[14]);
  controller.close();
  assert(mock->closed);
  std::filesystem::remove_all(tmp);
  std::cout << "calligraphy mock replay checks passed\n";
}
