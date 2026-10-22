#include "vsim/models/vehicle/corners.hpp"

#include <stdexcept>

namespace vsim {

std::size_t corner_index(std::string_view name) {
  for (std::size_t i = 0; i < kCorners; ++i) {
    if (name == kCornerNames[i]) {
      return i;
    }
  }
  throw std::invalid_argument("unknown wheel '" + std::string(name) +
                              "' (expected fl, fr, rl or rr)");
}

}  // namespace vsim
