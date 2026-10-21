#pragma once

namespace vsim {

// What every driver source produces (design doc §9.1). The VCU cannot tell
// whether a person, the MPC driver or a script is driving.
struct DriverCommand {
  double steer = 0.0;     // steering wheel angle [rad], positive = left
  double throttle = 0.0;  // accelerator pedal [0, 1]
  double brake = 0.0;     // brake pedal [0, 1]
};

}  // namespace vsim
