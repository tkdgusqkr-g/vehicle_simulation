#pragma once

#include <vector>

#include "vsim/sim/params.hpp"

namespace vsim {

// Ride height aero map (design doc §6.4): downforce, its front share and
// drag as functions of the front and rear ride heights, from a wind tunnel
// or CFD map. Ground effect wings and the undertray gain downforce as they
// come closer to the road until the flow under them stalls; pitch moves the
// balance (nose down: more on the front).
//
//   vehicle.aero.ride_height_map:
//     front_mm: [-30, -20, -10, 0, 10, 20]   # ride height change from static (axle centreline)
//     rear_mm:  [-30, -20, -10, 0, 10, 20]
//     cla_factor:        # x cla_m2; rows front_mm, columns rear_mm
//       - [...]
//     front_ratio_shift: # + front_ratio, same layout
//       - [...]
//     cda_factor:        # x cda_m2 (optional, 1 if missing)
//       - [...]
//
// Bilinear between the points, held at the edges. The full model (L3) moves
// the ride heights; the two-track model reads the map at static (0, 0).
struct AeroMap {
  struct Sample {
    double cla_factor = 1.0, front_ratio_shift = 0.0, cda_factor = 1.0;
  };

  bool enabled = false;
  std::vector<double> front, rear;  // [m]
  // Row-major: front index * rear.size() + rear index.
  std::vector<double> cla_factor, front_ratio_shift, cda_factor;

  static AeroMap from_params(const ParamTree& params);
  // At ride height changes (front, rear) [m] from static, down negative.
  Sample at(double front_change, double rear_change) const;
};

}  // namespace vsim
