#include "vsim/sim/rng.hpp"

#include <cmath>
#include <numbers>

namespace vsim {
namespace {

std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

}  // namespace

std::uint64_t fnv1a64(std::string_view text) {
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  for (const char c : text) {
    hash ^= static_cast<unsigned char>(c);
    hash *= 0x100000001b3ULL;
  }
  return hash;
}

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}

std::uint64_t derive_seed(std::uint64_t master_seed, std::string_view stream) {
  return splitmix64(splitmix64(master_seed) ^ fnv1a64(stream));
}

Rng::Rng(std::uint64_t seed) {
  std::uint64_t state = seed;
  for (auto& word : s_) {
    state = splitmix64(state);
    word = state;
  }
}

std::uint64_t Rng::next_u64() {
  const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
  const std::uint64_t t = s_[1] << 17;
  s_[2] ^= s_[0];
  s_[3] ^= s_[1];
  s_[1] ^= s_[2];
  s_[0] ^= s_[3];
  s_[2] ^= t;
  s_[3] = rotl(s_[3], 45);
  return result;
}

double Rng::uniform() {
  // Top 53 bits -> [0, 1) with full double precision.
  return static_cast<double>(next_u64() >> 11) * (1.0 / 9007199254740992.0);
}

double Rng::normal() {
  if (has_spare_) {
    has_spare_ = false;
    return spare_;
  }
  const double u1 = 1.0 - uniform();  // (0, 1], keeps log() finite
  const double u2 = uniform();
  const double radius = std::sqrt(-2.0 * std::log(u1));
  const double angle = 2.0 * std::numbers::pi * u2;
  spare_ = radius * std::sin(angle);
  has_spare_ = true;
  return radius * std::cos(angle);
}

}  // namespace vsim
