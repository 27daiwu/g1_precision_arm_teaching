#pragma once

#include "g1_arm/types.hpp"
#include <memory>
#include <mutex>
#include <string>

namespace g1_arm {
class Dds {
public:
  virtual ~Dds() = default;
  virtual void initialize(const std::string &interface) = 0;
  virtual State latest() const = 0;
  virtual bool publish(const ControlFrame &frame, std::string &error) = 0;
  virtual void close() noexcept = 0;
};

std::unique_ptr<Dds> make_unitree_dds();

class PublisherOwnership {
public:
  PublisherOwnership();
  ~PublisherOwnership();
  PublisherOwnership(const PublisherOwnership &) = delete;
  PublisherOwnership &operator=(const PublisherOwnership &) = delete;
private:
  bool owns_{false};
};
} // namespace g1_arm
