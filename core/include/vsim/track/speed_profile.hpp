#pragma once

#include <vector>

#include "vsim/track/track.hpp"

namespace vsim {

struct SpeedLimits {
  double lateral = 1.4 * 9.81;   // [m/s^2] cornering
  double accel = 1.0 * 9.81;     // [m/s^2] driving
  double brake = 1.4 * 9.81;     // [m/s^2] braking
  double top = 30.0;             // [m/s]
  double start = 0.0;            // [m/s] at the first sample of an open line
  double end = 0.0;              // [m/s] at the end of an open line
};

// Quasi-steady-state speed profile along a line (design doc §9.5): the
// cornering limit sqrt(a_lat / |curvature|), then a forward pass with the
// acceleration limit and a backward pass with the braking limit. Closed
// lines wrap around; open lines start at `start` and stop at `end`.
// One value per sample of the line.
std::vector<double> quasi_steady_speed(const SampledCurve& line, const SpeedLimits& limits);

}  // namespace vsim
