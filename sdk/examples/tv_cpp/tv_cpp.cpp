// tv_cpp.cpp - torque vectoring as a native plugin in C++ (vsim_plugin.hpp).
//
// Runs after the VCU: takes its four torque commands and shifts torque
// between the left and right wheels so the yaw rate follows a neutral-steer
// reference. The same law is in tv_fmu (an FMU) and the example project's
// plugins/tv_py.py (Python), and the three give identical runs.
#include "vsim_plugin.hpp"

namespace {

double clamp(double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); }

class TorqueVectoring : public vsim::plugin::Plugin {
 public:
  TorqueVectoring(const vsim::plugin::Params& p, double /*period*/)
      : gain_(p.number("gain", 40.0)),
        wheelbase_(p.number("wheelbase", 1.53)),
        steer_ratio_(p.number("steer_ratio", 4.5)),
        understeer_(p.number("understeer", 0.0)),
        a_max_(p.number("a_max", 15.0)),
        max_shift_(p.number("max_shift", 8.0)),
        v_on_(p.number("v_on", 3.0)) {}

  // in: steer [rad, steering wheel], vx [m/s], yaw_rate [rad/s], tq_*_in [N*m]
  // out: tq_fl, tq_fr, tq_rl, tq_rr [N*m], shift [N*m]
  void step(double /*t*/, const double* in, double* out) override {
    const double steer = in[0], vx = in[1], yaw_rate = in[2];
    const double delta = steer / steer_ratio_;
    const double v = vx > 1.0 ? vx : 1.0;
    double r_ref = vx * delta / (wheelbase_ + understeer_ * vx * vx);
    const double r_lim = a_max_ / v;
    r_ref = clamp(r_ref, -r_lim, r_lim);
    const double fade = clamp((vx - v_on_) / 2.0, 0.0, 1.0);
    const double shift = clamp(gain_ * (r_ref - yaw_rate), -max_shift_, max_shift_) * fade;
    out[0] = in[3] - shift;
    out[1] = in[4] + shift;
    out[2] = in[5] - shift;
    out[3] = in[6] + shift;
    out[4] = shift;
  }

 private:
  double gain_, wheelbase_, steer_ratio_, understeer_, a_max_, max_shift_, v_on_;
};

}  // namespace

VSIM_PLUGIN(TorqueVectoring, "tv_cpp", "1.0", 0.01,
            {"steer", "rad", VSIM_IN},
            {"vx", "m/s", VSIM_IN},
            {"yaw_rate", "rad/s", VSIM_IN},
            {"tq_fl_in", "N*m", VSIM_IN},
            {"tq_fr_in", "N*m", VSIM_IN},
            {"tq_rl_in", "N*m", VSIM_IN},
            {"tq_rr_in", "N*m", VSIM_IN},
            {"tq_fl", "N*m", VSIM_OUT},
            {"tq_fr", "N*m", VSIM_OUT},
            {"tq_rl", "N*m", VSIM_OUT},
            {"tq_rr", "N*m", VSIM_OUT},
            {"shift", "N*m", VSIM_OUT})
