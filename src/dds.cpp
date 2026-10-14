#include "g1_arm/dds.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/dds_wrapper/common/crc.h>
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace g1_arm {
namespace {
std::mutex ownership_mutex;
bool publisher_owned = false;

class UnitreeDds final : public Dds {
public:
  void initialize(const std::string &interface) override {
    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<LowState>>(
        "rt/lowstate");
    subscriber_->InitChannel([this](const void *message) {
      LowState copy;
      std::memcpy(&copy, message, sizeof(copy));
      State state;
      for (std::size_t i = 0; i < state.q.size(); ++i) {
        state.q[i] = copy.motor_state()[i].q();
        state.dq[i] = copy.motor_state()[i].dq();
      }
      state.mode_machine = copy.mode_machine();
      state.received_at = Clock::now();
      std::lock_guard<std::mutex> guard(state_mutex_);
      latest_ = state;
      have_state_ = true;
    }, 1);
    publisher_ = std::make_unique<unitree::robot::ChannelPublisher<LowCmd>>(
        "rt/arm_sdk");
    publisher_->InitChannel();
  }

  State latest() const override {
    std::lock_guard<std::mutex> guard(state_mutex_);
    if (!have_state_) throw std::runtime_error("waiting for rt/lowstate");
    if (Clock::now() - latest_.received_at > std::chrono::milliseconds(250))
      throw std::runtime_error("rt/lowstate stale >250ms");
    return latest_;
  }

  bool publish(const ControlFrame &frame, std::string &error) override {
    if (!publisher_) { error = "rt/arm_sdk publisher not initialized"; return false; }
    LowCmd message;
    message.mode_pr(0);
    for (int motor = 12; motor <= 28; ++motor) {
      auto &command = message.motor_cmd()[motor];
      command.q(static_cast<float>(frame.q[motor]));
      command.dq(0.0f);
      command.kp(static_cast<float>(frame.kp[motor]));
      command.kd(static_cast<float>(frame.kd[motor]));
      command.tau(0.0f);
    }
    message.motor_cmd()[29].q(static_cast<float>(frame.weight));
    message.crc(crc32_core(reinterpret_cast<uint32_t *>(&message),
                           (sizeof(message) >> 2) - 1));
    const auto start = Clock::now();
    const bool result = publisher_->Write(message);
    if (!result) error = "rt/arm_sdk Write failed";
    else if (Clock::now() - start > std::chrono::milliseconds(50)) {
      error = "rt/arm_sdk Write exceeded 50ms";
      return false;
    }
    return result;
  }

  void close() noexcept override {
    publisher_.reset();
    subscriber_.reset();
  }
private:
  using LowCmd = unitree_hg::msg::dds_::LowCmd_;
  using LowState = unitree_hg::msg::dds_::LowState_;
  mutable std::mutex state_mutex_;
  State latest_{};
  bool have_state_{false};
  std::unique_ptr<unitree::robot::ChannelSubscriber<LowState>> subscriber_;
  std::unique_ptr<unitree::robot::ChannelPublisher<LowCmd>> publisher_;
};
} // namespace

std::unique_ptr<Dds> make_unitree_dds() { return std::make_unique<UnitreeDds>(); }

PublisherOwnership::PublisherOwnership() {
  std::lock_guard<std::mutex> guard(ownership_mutex);
  if (publisher_owned) throw std::runtime_error("another local arm publisher owns DDS");
  publisher_owned = true;
  owns_ = true;
}
PublisherOwnership::~PublisherOwnership() {
  if (!owns_) return;
  std::lock_guard<std::mutex> guard(ownership_mutex);
  publisher_owned = false;
}
} // namespace g1_arm
