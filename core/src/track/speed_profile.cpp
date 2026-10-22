#include "vsim/track/speed_profile.hpp"

#include <algorithm>
#include <cmath>

namespace vsim {

std::vector<double> quasi_steady_speed(const SampledCurve& line, const SpeedLimits& limits) {
  const std::vector<CurveSample>& samples = line.samples();
  const std::size_t n = samples.size();
  std::vector<double> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    const double kappa = std::abs(samples[i].curvature);
    v[i] = kappa > 1e-9 ? std::min(limits.top, std::sqrt(limits.lateral / kappa)) : limits.top;
  }
  const auto ds = [&](std::size_t i) {  // from sample i to the next
    return i + 1 < n ? samples[i + 1].s - samples[i].s : line.length() - samples[i].s;
  };
  if (line.closed()) {
    // Two laps of each pass settle the wrap-around.
    for (int lap = 0; lap < 2; ++lap) {
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t next = (i + 1) % n;
        v[next] = std::min(v[next], std::sqrt(v[i] * v[i] + 2.0 * limits.accel * ds(i)));
      }
    }
    for (int lap = 0; lap < 2; ++lap) {
      for (std::size_t k = n; k-- > 0;) {
        const std::size_t next = (k + 1) % n;
        v[k] = std::min(v[k], std::sqrt(v[next] * v[next] + 2.0 * limits.brake * ds(k)));
      }
    }
    return v;
  }
  v.front() = std::min(v.front(), limits.start);
  for (std::size_t i = 0; i + 1 < n; ++i) {
    v[i + 1] = std::min(v[i + 1], std::sqrt(v[i] * v[i] + 2.0 * limits.accel * ds(i)));
  }
  v.back() = std::min(v.back(), limits.end);
  for (std::size_t k = n - 1; k-- > 0;) {
    v[k] = std::min(v[k], std::sqrt(v[k + 1] * v[k + 1] + 2.0 * limits.brake * ds(k)));
  }
  return v;
}

}  // namespace vsim
