#include "g1_arm/arm_sdk_controller.hpp"
#include "g1_arm/calligraphy_trajectory.hpp"
#include "g1_arm/continuous_trajectory.hpp"
#include "g1_arm/recorder.hpp"
#include "g1_arm/stability_gate.hpp"
#include "g1_arm/feedback_replay_controller.hpp"
#include "g1_arm/trajectory_processor.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <json/json.h>
#include <regex>
#include <sstream>

namespace {
using namespace g1_arm;
class MockDds final : public Dds {
public:
  void initialize(const std::string &) override {
    for (std::size_t i = 0; i < state.q.size(); ++i)
      state.q[i] = static_cast<double>(i) * .001;
    state.received_at = Clock::now();
  }
  void activate_publisher() override { publisher_active = true; }
  State latest() const override {
    State result = state;
    result.received_at = Clock::now();
    return result;
  }
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
  bool publisher_active{false};
};
}

int main() {
  using namespace g1_arm;
  StabilityConfig gate_config;
  gate_config.window_s = 1.0;
  gate_config.filter_tau_s = .2;
  StabilityGate gate(gate_config);
  State sample;
  auto sample_time = Clock::now();
  for (int i = 0; i <= 60; ++i) {
    sample.received_at = sample_time + std::chrono::milliseconds(20 * i);
    sample.imu_gyro = {0.0, 0.0, 0.0};
    if (i == 20) sample.imu_gyro[0] = 10.0;
    gate.observe(sample);
  }
  assert(gate.stable());
  assert(gate.stable_duration_s(sample.received_at) >= 1.0);
  for (int i = 61; i <= 120; ++i) {
    sample.received_at = sample_time + std::chrono::milliseconds(20 * i);
    sample.imu_gyro[0] = 1.0;
    gate.observe(sample);
  }
  assert(!gate.stable());

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

  ContinuousTrajectory continuous;
  continuous.name = "continuous_test";
  continuous.recorded_at = timestamp_local();
  continuous.recording_start_q.fill(.1);
  for (int frame = 0; frame < 3; ++frame) {
    ContinuousSample sample_frame;
    sample_frame.t_s = frame * .04;
    sample_frame.q.fill(.1 + frame * .02);
    sample_frame.dq.fill(frame * .1);
    continuous.samples.push_back(sample_frame);
  }
  const auto continuous_path = continuous_trajectory_path(tmp, continuous.name);
  save_continuous_trajectory(continuous, continuous_path);
  const auto continuous_loaded = load_continuous_trajectory(continuous_path);
  assert(continuous_loaded.samples.size() == 3);
  assert(continuous_loaded.samples[1].t_s == .04);
  assert(continuous_loaded.samples[1].dq[0] == .1);
  ArmQ acquire_pose{};
  acquire_pose.fill(-.2);
  const auto first_relative = interpolate_continuous(continuous_loaded, 0.0,
                                                      acquire_pose, 1.0);
  assert(first_relative == acquire_pose);
  const auto middle_relative = interpolate_continuous(continuous_loaded, .02,
                                                       acquire_pose, 1.0);
  assert(std::abs(middle_relative[0] - (-.19)) < 1e-12);
  const auto scaled = interpolate_continuous(continuous_loaded, .02,
                                              acquire_pose, .5);
  assert(std::abs(scaled[0] - (-.195)) < 1e-12);
  auto invalid_times = continuous;
  invalid_times.samples[2].t_s = invalid_times.samples[1].t_s;
  bool rejected_times = false;
  try { validate_continuous_trajectory(invalid_times); }
  catch (const std::runtime_error &) { rejected_times = true; }
  assert(rejected_times);

  ContinuousTrajectory path_source;
  path_source.name = "processor_test";
  path_source.recorded_at = timestamp_local();
  path_source.recording_start_q.fill(.1);
  for (int frame = 0; frame <= 100; ++frame) {
    const double t = frame * .02;
    ContinuousSample item;
    item.t_s = t;
    double q = .1;
    if (t < 1.0) q += .1 * t;
    else if (t < 1.5) q += .1;
    else q += .1 - .1 * (t - 1.5);
    item.q.fill(q);
    item.dq.fill(t < 1.0 || t >= 1.5 ? .1 : 0.0);
    path_source.samples.push_back(item);
  }
  const auto processed = process_trajectory(path_source);
  assert(processed.nodes.size() > 2);
  assert(processed.nodes.front().s == 0.0 && processed.nodes.back().s == 1.0);
  assert(processed.max_geometric_deviation_rad <= .01);
  const auto processed_peak = sample_processed(processed, .5);
  assert(processed_peak[0] > path_source.recording_start_q[0]);
  RetimerConfig fast_retimer;
  fast_retimer.speed_scale = .25;
  TrajectoryRetimer retimer(processed, fast_retimer);
  assert(retimer.advance(.2, .1, .02, .5) > .2);
  assert(retimer.advance(.2, .1, .02, 0.0) == .2);
  const auto processed_path = tmp / "processor_test_processed.json";
  save_processed_trajectory(processed, processed_path);
  assert(std::filesystem::file_size(processed_path) > 0);

  ArmQ feedback_anchor{};
  feedback_anchor.fill(.2);
  FeedbackConfig feedback_config;
  feedback_config.error_normal_rad = .03;
  feedback_config.error_pause_rad = .08;
  feedback_config.stable_window_s = .04;
  feedback_config.node_timeout_s = .18;
  FeedbackReplayController feedback(processed, feedback_anchor, fast_retimer,
                                   feedback_config);
  State actual;
  for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = feedback_anchor[motor - 15];
  auto first_step = feedback.step(actual, .02);
  assert(first_step.q_ref == feedback_anchor);
  assert(first_step.node_index > 0);
  assert(first_step.progress_s > 0.0);
  const double normal_progress = first_step.progress_s;
  for (int i = 0; i < 8; ++i) {
    for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = feedback_anchor[motor - 15] - .2;
    first_step = feedback.step(actual, .02);
  }
  assert(first_step.speed_scale < 1.0);
  const auto slowed_progress = first_step.progress_s;
  for (int i = 0; i < 80; ++i) {
    for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = first_step.q_ref[motor - 15];
    first_step = feedback.step(actual, .02);
  }
  assert(first_step.progress_s >= slowed_progress);
  double previous_scale = first_step.speed_scale;
  bool smoothly_recovered = false;
  for (int i = 0; i < 100; ++i) {
    for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = first_step.q_ref[motor - 15];
    first_step = feedback.step(actual, .02);
    assert(first_step.speed_scale + 1e-9 >= previous_scale || first_step.speed_scale == 0.0);
    previous_scale = first_step.speed_scale;
    if (first_step.speed_scale > .5) smoothly_recovered = true;
  }
  assert(smoothly_recovered);

  auto node_path = processed;
  node_path.nodes = {{0.0, path_source.samples.front().q, true},
                     {.5, processed_peak, true},
                     {1.0, path_source.samples.back().q, true}};
  RetimerConfig node_retimer_config = fast_retimer;
  node_retimer_config.speed_scale = .5;
  FeedbackConfig node_config;
  node_config.position_tolerance_rad = .02;
  node_config.stable_dq_rad_s = .02;
  node_config.stable_window_s = .04;
  node_config.node_timeout_s = .5;
  FeedbackReplayController node_replay(node_path, feedback_anchor,
                                       node_retimer_config, node_config);
  actual = {};
  for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = feedback_anchor[motor - 15];
  bool node_held = false, node_advanced = false;
  double node_progress = -1.0;
  for (int i = 0; i < 300; ++i) {
    const auto output = node_replay.step(actual, .02);
    if (output.node_index == 1 && output.progress_s >= .5) {
      if (node_progress >= 0 && output.progress_s == node_progress) node_held = true;
      node_progress = output.progress_s;
      for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = output.q_ref[motor - 15];
      actual.dq.fill(0.0);
    } else if (node_held && output.progress_s > .5) {
      node_advanced = true;
      break;
    } else {
      for (int motor = 15; motor <= 28; ++motor) actual.q[motor] = output.q_ref[motor - 15];
      actual.dq.fill(0.0);
    }
  }
  assert(node_held && node_advanced);
  assert(node_replay.node_reached()[1]);
  assert(node_replay.node_errors()[1] <= .02);

  auto wait_path = processed;
  wait_path.nodes = {{0.0, path_source.samples.front().q, true},
                     {1.0, path_source.samples.back().q, true}};
  FeedbackConfig wait_config;
  wait_config.error_normal_rad = .03;
  wait_config.error_pause_rad = .08;
  wait_config.stable_dq_rad_s = .001;
  wait_config.stable_window_s = .04;
  wait_config.node_timeout_s = .1;
  FeedbackReplayController waiting(wait_path, feedback_anchor, fast_retimer, wait_config);
  actual.q[15] = -1.0;
  bool not_reached = false;
  for (int i = 0; i < 100 && !not_reached; ++i) {
    const auto output = waiting.step(actual, .02);
    not_reached = output.aborted;
  }
  assert(not_reached && waiting.abort_reason().find("NOT_REACHED") != std::string::npos);

  ContinuousRecorder capture;
  State capture_state;
  for (int motor = 15; motor <= 28; ++motor) {
    capture_state.q[motor] = .1;
    capture_state.dq[motor] = .02;
  }
  bool lock_rejected = false;
  try { capture.start(TeachMode::Lock, capture_state, Clock::now(), timestamp_local()); }
  catch (const std::runtime_error &) { lock_rejected = true; }
  assert(lock_rejected && !capture.recording());
  const auto capture_start = Clock::now();
  capture.start(TeachMode::Drag, capture_state, capture_start, timestamp_local());
  assert(capture.recording() && capture.trajectory().samples.size() == 1);
  capture_state.q[15] = .11;
  capture.capture(capture_state, capture_start + std::chrono::milliseconds(20));
  assert(capture.trajectory().samples.size() == 2);
  assert(capture.trajectory().samples[0].t_s == 0.0);
  assert(capture.trajectory().samples[1].t_s > capture.trajectory().samples[0].t_s);
  capture.stop();
  const auto stopped_frames = capture.trajectory().samples.size();
  capture.capture(capture_state, capture_start + std::chrono::milliseconds(40));
  assert(!capture.recording() && capture.trajectory().samples.size() == stopped_frames);

  CalligraphyRecorder calligraphy;
  State calligraphy_state;
  for (int motor = 12; motor <= 28; ++motor) {
    calligraphy_state.q[motor] = motor * .001;
    calligraphy_state.dq[motor] = motor * .0001;
  }
  const auto calligraphy_start = Clock::now();
  calligraphy.start(calligraphy_state, calligraphy_start, "2026-10-15T10:30:00+0800");
  for (int frame = 1; frame <= 4; ++frame) {
    calligraphy_state.q[12] += .001;
    calligraphy.capture(calligraphy_state,
                       calligraphy_start + std::chrono::milliseconds(20 * frame));
  }
  calligraphy.stop(calligraphy_state);
  assert(!calligraphy.recording() && calligraphy.trajectory().samples.size() == 5);
  assert(calligraphy.trajectory().samples.front().q[0] == .012);
  assert(calligraphy.trajectory().samples.front().q[16] == .028);
  assert(calligraphy.trajectory().samples.back().t_monotonic == .08);
  assert(calligraphy.trajectory().sample_rate_actual == 50.0);
  calligraphy.trajectory().character_name = "福";
  calligraphy.trajectory().waist_pitch_extra_bias_known = true;
  calligraphy.trajectory().waist_pitch_extra_bias = .05;
  calligraphy.trajectory().effective_waist_control_mode = "WaistHold_fixed";
  const auto calligraphy_path = calligraphy_trajectory_path(tmp, "福", "20261015_103000");
  save_calligraphy_trajectory(calligraphy.trajectory(), calligraphy_path);
  const auto calligraphy_loaded = load_calligraphy_trajectory(calligraphy_path);
  assert(calligraphy_loaded.character_name == "福");
  assert(calligraphy_loaded.samples.size() == 5);
  assert(calligraphy_loaded.samples[2].q == calligraphy.trajectory().samples[2].q);
  assert(calligraphy_loaded.samples[2].dq == calligraphy.trajectory().samples[2].dq);
  assert(calligraphy_loaded.samples[2].t_monotonic == .04);
  assert(calligraphy_loaded.waist_pitch_extra_bias_known);
  assert(calligraphy_loaded.waist_pitch_extra_bias == .05);
  assert(calligraphy_loaded.effective_waist_control_mode == "WaistHold_fixed");
  const auto mapped_start = calligraphy_arm_position(calligraphy_loaded.samples.front().q);
  assert(mapped_start[0] == calligraphy_loaded.samples.front().q[3]);
  assert(mapped_start[13] == calligraphy_loaded.samples.front().q[16]);
  const auto mapped_mid = interpolate_calligraphy_arm(calligraphy_loaded, .03);
  for (std::size_t i = 0; i < kArmDof; ++i) {
    const double expected = .5 * (calligraphy_loaded.samples[1].q[i + 3] +
                                  calligraphy_loaded.samples[2].q[i + 3]);
    assert(std::abs(mapped_mid[i] - expected) < 1e-12);
  }
  const auto mapped_end = interpolate_calligraphy_arm(calligraphy_loaded, 100.0);
  assert(mapped_end[13] == calligraphy_loaded.samples.back().q[16]);
  WaistQ waist_actual{calligraphy_loaded.samples.front().q[0],
                      calligraphy_loaded.samples.front().q[1],
                      calligraphy_loaded.samples.front().q[2]};
  WaistQ waist_errors{};
  assert(calligraphy_waist_matches(calligraphy_loaded, waist_actual, .04,
                                   &waist_errors));
  assert(std::abs(waist_errors[2]) < 1e-12);
  auto unknown_metadata = calligraphy_loaded;
  unknown_metadata.waist_pitch_extra_bias_known = false;
  unknown_metadata.waist_pitch_extra_bias = 0.0;
  assert(!calligraphy_waist_matches(unknown_metadata, waist_actual, .04));

  Json::Value saved_root;
  {
    std::ifstream input(calligraphy_path);
    Json::CharReaderBuilder builder;
    std::string error;
    const bool parsed_calligraphy = Json::parseFromStream(builder, input, &saved_root, &error);
    assert(parsed_calligraphy);
  }
  Json::Value legacy_root(Json::objectValue);
  legacy_root["Character"] = saved_root["Character"];
  legacy_root["Motion"] = saved_root["Motion"];
  Json::Value legacy_metadata(Json::objectValue);
  legacy_metadata["recording_mode"] = "whole_character";
  legacy_metadata["sample_rate_actual"] = 50.0;
  legacy_metadata["start_time"] = "legacy";
  legacy_root["Metadata"] = legacy_metadata;
  const auto legacy_calligraphy_path = tmp / "legacy_calligraphy.json";
  {
    Json::StreamWriterBuilder builder;
    std::ofstream output(legacy_calligraphy_path);
    output << Json::writeString(builder, legacy_root);
  }
  const auto legacy_calligraphy = load_calligraphy_trajectory(legacy_calligraphy_path);
  assert(!legacy_calligraphy.waist_pitch_extra_bias_known);
  assert(legacy_calligraphy.effective_waist_control_mode == "UNKNOWN");

  State waist_bias_state;
  waist_bias_state.q[12] = .1;
  waist_bias_state.q[13] = -.2;
  waist_bias_state.q[14] = -.03;
  WaistHold waist_bias_hold;
  waist_bias_hold.capture(waist_bias_state);
  ControlFrame waist_frame;
  const auto full_bias = waist_bias_hold.apply(waist_frame, 1.0, 0.0, .05);
  assert(std::abs(full_bias.q[12] - .1) < 1e-12);
  assert(std::abs(full_bias.q[13] + .2) < 1e-12);
  assert(std::abs(full_bias.q[14] - .03) < 1e-12);
  const auto acquire_half = waist_bias_hold.apply(waist_frame, .5, 0.0, .05);
  assert(std::abs(acquire_half.q[14]) < 1e-12);
  const auto release_half = waist_bias_hold.apply(waist_frame, 1.0, .5, .05);
  assert(std::abs(release_half.q[14]) < 1e-12);

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

  StabilityConfig stable_config;
  stable_config.window_s = .04;
  stable_config.wait_timeout_s = .5;
  stable_config.filter_tau_s = .05;
  {
  auto mock = std::make_unique<MockDds>();
  auto *wire = mock.get();
  ArmSdkController controller(std::move(mock), tmp / "logs");
  controller.set_waist_pitch_extra_bias(.005);
  controller.start("mock");
  controller.wait_stable(stable_config);
  assert(wire->frames.empty());
  assert(!wire->publisher_active);
  controller.acquire();
  assert(wire->publisher_active);
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
  assert(wire->frames[1].q[14] < wire->state.q[14] + .015);

  ArmQ target{};
  for (std::size_t i = 0; i < target.size(); ++i) target[i] = wire->state.q[15 + i];
  controller.publish(target, TeachMode::Drag);
  assert(wire->frames.back().kp[15] == 8.0);
  assert(wire->frames.back().kp[18] == 8.0);
  assert(wire->frames.back().kp[19] == 6.0);
  assert(wire->frames.back().kp[22] == 8.0);
  assert(wire->frames.back().kp[26] == 6.0);
  assert(std::abs(wire->frames.back().q[14] - (wire->state.q[14] + .015)) < 1e-6);
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
  bool readable_filename = false;
  bool event_records_complete = false;
  for (const auto &entry : std::filesystem::directory_iterator(tmp / "logs")) {
    const auto filename = entry.path().filename().string();
    if (std::regex_match(filename, std::regex("g1_arm_20[0-9]{2}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{3}\\.csv")))
      readable_filename = true;
    if (filename.find("g1_arm_events_") == 0) {
      std::ifstream events(entry.path());
      std::ostringstream content;
      content << events.rdbuf();
      const auto text = content.str();
      event_records_complete = text.find("WAIT_STABLE") != std::string::npos &&
          text.find("STABLE_CONFIRMED") != std::string::npos &&
          text.find("ACQUIRE_START") != std::string::npos &&
          text.find("RELEASE_COMPLETE") != std::string::npos &&
          text.find("arm_q_ref") != std::string::npos &&
          text.find("waist_max_abs_error") != std::string::npos;
    } else if (entry.path().extension() == ".csv" && filename.find("g1_arm_") == 0) {
      std::ifstream log(entry.path());
      std::ostringstream content;
      content << log.rdbuf();
      const auto text = content.str();
      event_records_complete = event_records_complete &&
          text.find("waist_pitch_extra_bias_rad") != std::string::npos &&
          text.find("acquire_progress") != std::string::npos &&
          text.find("tracking_error_rad") != std::string::npos;
    }
  }
  assert(readable_filename && event_records_complete);
  }

  auto moving_mock = std::make_unique<MockDds>();
  auto *moving_wire = moving_mock.get();
  moving_wire->state.dq[0] = 2.0;
  ArmSdkController timeout_controller(std::move(moving_mock), tmp / "timeout_logs");
  timeout_controller.start("mock");
  stable_config.wait_timeout_s = .08;
  bool timed_out = false;
  try { timeout_controller.wait_stable(stable_config); }
  catch (const std::runtime_error &) { timed_out = true; }
  assert(timed_out);
  assert(moving_wire->frames.empty());
  assert(!moving_wire->publisher_active);
  timeout_controller.close();
  std::filesystem::remove_all(tmp);
  std::cout << "G1 Arm SDK core offline checks passed\n";
}
