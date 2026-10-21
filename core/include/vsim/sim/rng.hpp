#pragma once

#include <cstdint>
#include <string_view>

namespace vsim {

std::uint64_t fnv1a64(std::string_view text);
std::uint64_t splitmix64(std::uint64_t x);

// Seed for an independent random stream, derived from the scenario's master
// seed and a stream name. Switching one disturbance source off never shifts
// another source's sequence, which keeps cause-isolation runs comparable.
std::uint64_t derive_seed(std::uint64_t master_seed, std::string_view stream);

// xoshiro256** generator. Uniform and normal draws are implemented here
// because std:: distributions produce different sequences per standard library.
class Rng {
 public:
  explicit Rng(std::uint64_t seed);

  std::uint64_t next_u64();
  double uniform();  // [0, 1)
  double normal();   // standard normal (Box-Muller)
  double normal(double mean, double stddev) { return mean + stddev * normal(); }

 private:
  std::uint64_t s_[4];
  bool has_spare_ = false;
  double spare_ = 0.0;
};

}  // namespace vsim
