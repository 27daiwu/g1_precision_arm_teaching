#include "g1_arm/types.hpp"
#include <cassert>

int main() {
  using namespace g1_arm;
  assert(kArmDof == 14);
  constexpr int left[] = {15, 16, 17, 18, 19, 20, 21};
  constexpr int right[] = {22, 23, 24, 25, 26, 27, 28};
  for (int i = 0; i < 7; ++i) {
    assert(left[i] == 15 + i);
    assert(right[i] == 22 + i);
    assert(left[i] > 11 && left[i] < 29);
    assert(right[i] > 11 && right[i] < 29);
  }
  constexpr int waist[] = {12, 13, 14};
  for (int motor : waist) assert(motor >= 12 && motor <= 14);
  constexpr int ownership_weight = 29;
  assert(ownership_weight == 29 && ownership_weight >= 29);
  return 0;
}
