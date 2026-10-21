#pragma once

#include <string>
#include <vector>

#include "vsim/sim/block.hpp"

namespace vsim {

// Reference VCU, M0 subset (design doc §8.1): linear pedal map to motor
// torque, equal split across motors, total mechanical power limit.
// Inputs:  drv.cmd.throttle, pt.motor.<corner>.speed
// Outputs: pt.motor.<corner>.torque_cmd
class ReferenceVcu final : public Block {
 public:
  std::string_view name() const override { return "vcu"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

 private:
  struct Motor {
    SignalId speed_in = 0;
    SignalId torque_cmd_out = 0;
  };

  double period_ = 0.01;
  double torque_max_ = 0.0;   // per motor [N m]
  double power_limit_ = 0.0;  // total [W]
  SignalId throttle_in_ = 0;
  std::vector<Motor> motors_;
};

}  // namespace vsim
