#pragma once

#include <string>

#include "vsim/sim/params.hpp"

namespace vsim {

// Scripted input over time, configured as e.g.
//   {type: const, value: 0.3}
//   {type: step, at_s: 2.0, value: 1.0, before: 0.0}
//   {type: ramp, start_s: 1.0, rate: 0.5, max: 1.0}
//   {type: sine, amplitude: 30, freq_hz: 0.2, start_s: 1.0, offset: 0}
// Values are in the unit of the channel the profile drives.
class Profile {
 public:
  Profile() = default;
  // `config` is the profile's subtree; an empty subtree means constant 0.
  static Profile from_params(const ParamTree& config, std::string_view what);

  double value(double t) const;

 private:
  enum class Type { Const, Step, Ramp, Sine };

  Type type_ = Type::Const;
  double value_ = 0.0;
  double before_ = 0.0;
  double at_ = 0.0;
  double rate_ = 0.0;
  double max_ = 0.0;
  double amplitude_ = 0.0;
  double freq_ = 0.0;
  double offset_ = 0.0;
};

}  // namespace vsim
