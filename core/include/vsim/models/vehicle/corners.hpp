#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

namespace vsim {

// Wheel corners, in this order everywhere (arrays, signal names).
inline constexpr std::size_t kCorners = 4;
inline constexpr std::array<std::string_view, kCorners> kCornerNames = {"fl", "fr", "rl", "rr"};

inline bool is_front(std::size_t corner) { return corner < 2; }
inline bool is_left(std::size_t corner) { return corner % 2 == 0; }

// Index of "fl", "fr", "rl" or "rr"; throws std::invalid_argument otherwise.
std::size_t corner_index(std::string_view name);

}  // namespace vsim
