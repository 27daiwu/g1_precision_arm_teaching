#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/recorder.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <iostream>

namespace {
using namespace g1_arm;
class MockDds final : public Dds {
public:
  void initialize(const std::string &) override {
    for (std::size_t i = 0; i < state.q.size(); ++i)
      state.q[i] = static_cast<double>(i) * .001;
    state.received_at = Clock::now();
  }
  State latest() const override { return state; }
  bool publish(const ControlFrame &frame, std::string &) override {
    frames.push_back(frame);
    for (int i = 0; i < 12; ++i) {
      assert(frame.q[i] == 0.0);
      assert(frame.kp[i] == 0.0);
      assert(frame.kd[i] == 0.0);
    }
    return true;
  }
  void close() noexcept override { closed = true; }
  State state;
  std::vector<ControlFrame> frames;
  bool closed{false};
};
}

int main() {
  using namespace g1_arm;
  Trajectory trajectory;
  trajectory.name = "offline_test";
  trajectory.recorded_at = timestamp_local();
  trajectory.waist_reference = {.1, -.1, .05};
  for (int p = 0; p < 4; ++p) {
    Point point;
    point.name = std::string(1, 'A' + p);
    point.recorded_at = timestamp_local();
    for (std::size_t i = 0; i < point.q.size(); ++i) point.q[i] = .01 * (p + 1);
    trajectory.points.push_back(point);
  }
  const auto tmp = std::filesystem::temp_directory_path() / "g1_arm_core_test";
  std::filesystem::remove_all(tmp);
  std::filesystem::create_directories(tmp);
  save_trajectory(trajectory, tmp / "track.json");
  auto loaded = load_trajectory(tmp / "track.json");
  assert(loaded.points.size() == 4 && loaded.points[0].name == "A" && loaded.points[3].name == "D");
  assert(loaded.waist_reference == trajectory.waist_reference);
  assert(std::abs(interpolate(loaded.points[0].q, loaded.points[1].q, 0.0, 1.0)[0] -
                  loaded.points[0].q[0]) < 1e-12);
  assert(std::abs(interpolate(loaded.points[0].q, loaded.points[1].q, 1.0, 1.0)[0] -
                  loaded.points[1].q[0]) < 1e-12);

  ArmQ initial{};
  initial.fill(.1);
  TeachState mode(initial);
  auto displaced = initial;
  displaced[0] = .2;
  assert(std::abs(mode.update(displaced)[0] - .1) < 1e-12);
  mode.drag(initial);
  const auto followed = mode.update(displaced);
  assert(std::abs(followed[0] - .125) < 1e-12);
  displaced[0] = .13;
  assert(std::abs(mode.update(displaced)[0] - .125) < 1e-12);
  mode.lock(displaced, Clock::now());
  assert(mode.reference() == displaced);
  assert(mode.gains(Clock::now())[0] >= 8.0 && mode.gains(Clock::now())[0] < 40.0);
  assert(mode.gains(Clock::now() + std::chrono::milliseconds(450))[0] == 40.0);

  auto mock = std::make_unique<MockDds>();
  auto *wire = mock.get();
  ArmSdkController controller(std::move(mock), tmp / "logs");
  controller.start("mock");
  controller.acquire();
  assert(wire->frames.front().weight == 1.0);
  for (int motor = 12; motor <= 28; ++motor)
    assert(std::abs(wire->frames.front().q[motor] - wire->state.q[motor]) < 1e-9);
  assert(wire->frames.front().kp[12] == 80.0);
  assert(wire->frames.front().kp[13] == 80.0);
  assert(wire->frames.front().kp[14] == 120.0);
  assert(std::abs(controller.waist_reference()[2] - wire->state.q[14]) < 1e-12);
  assert(wire->frames.size() > 10);
  assert(wire->frames[1].weight > 0.0 && wire->frames[1].weight < 1.0);
  assert(wire->frames[1].q[14] > wire->state.q[14]);
  assert(wire->frames[1].q[14] < wire->state.q[14] + .010);

  ArmQ target{};
  for (std::size_t i = 0; i < target.size(); ++i) target[i] = wire->state.q[15 + i];
  controller.publish(target, TeachMode::Drag);
  assert(wire->frames.back().kp[15] == 8.0);
  assert(wire->frames.back().kp[18] == 8.0);
  assert(wire->frames.back().kp[19] == 6.0);
  assert(wire->frames.back().kp[22] == 8.0);
  assert(wire->frames.back().kp[26] == 6.0);
  assert(std::abs(wire->frames.back().q[14] - (wire->state.q[14] + .010)) < 1e-6);
  assert(std::abs(controller.waist_reference()[1] - wire->state.q[13]) < 1e-12);
  controller.publish(target, TeachMode::Lock);
  assert(wire->frames.back().kp[15] == 40.0 && wire->frames.back().kp[28] == 40.0);
  const auto release_start_index = wire->frames.size();
  controller.release();
  assert(wire->frames.back().weight == 0.0);
  assert(std::abs(wire->frames.back().q[14] - wire->state.q[14]) < 1e-6);
  assert(wire->frames[release_start_index].q[14] > wire->state.q[14]);
  assert(wire->frames[release_start_index].q[14] < wire->state.q[14] + .010);
  assert(std::abs(wire->frames.front().q[15] - wire->frames.back().q[15]) > 0.0);
  assert(wire->closed);
  std::filesystem::remove_all(tmp);
  std::cout << "G1 Arm SDK core offline checks passed\n";
}
