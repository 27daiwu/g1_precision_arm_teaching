#include "g1_arm/recorder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g1_arm {
namespace {
ArmQ measured_arm(const State &state) {
  ArmQ q{};
  for (std::size_t i = 0; i < q.size(); ++i) {
    q[i] = state.q[15 + i];
    if (!std::isfinite(q[i])) throw std::runtime_error("invalid arm LowState");
  }
  return q;
}
}
void Recorder::mark(const State &state) {
  unsigned index = 0;
  for (;;) {
    const std::string name(1, static_cast<char>('A' + index));
    const bool exists = std::any_of(trajectory_.points.begin(), trajectory_.points.end(),
                                    [&](const Point &point) { return point.name == name; });
    if (!exists) {
      set_point(trajectory_, Point{name, timestamp_local(), measured_arm(state), 0.0});
      return;
    }
    ++index;
    if (index >= 26) throw std::runtime_error("recording supports at most 26 points");
  }
}
void Recorder::replace(const std::string &name, const State &state) {
  auto found = std::find_if(trajectory_.points.begin(), trajectory_.points.end(),
                            [&](const Point &point) { return point.name == name; });
  if (found == trajectory_.points.end()) throw std::runtime_error("point not found: " + name);
  found->q = measured_arm(state);
  found->recorded_at = timestamp_local();
}
void Recorder::erase(const std::string &name) { delete_point(trajectory_, name); }
void Recorder::rename(std::string name) {
  validate_name(name);
  trajectory_.name = std::move(name);
}
} // namespace g1_arm
