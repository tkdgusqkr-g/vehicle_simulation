#pragma once

#include <functional>

#include "vsim/models/powertrain/motor.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {

// Inverter + motor, Level B (design doc §7.3): a permanent magnet synchronous
// machine in the rotor (dq) frame under field oriented current control,
// stepped at the inverter's control rate inside each plant step with the
// rotor speed held (multi-rate: the currents settle in well under 1 ms, the
// mechanics barely move).
//
//   vehicle.powertrain.motor.level: B
//   vehicle.powertrain.motor.pmsm:
//     pole_pairs: 5
//     resistance_ohm: 0.1        # per phase
//     ld_mh: 0.12, lq_mh: 0.15
//     flux_linkage_wb: 0.03      # magnet flux linkage (peak, per phase)
//     current_max_a: 100         # peak phase current
//     control_rate_hz: 16000     # current loop and PWM
//     current_bandwidth_hz: 1000
//     dead_time_us: 1.0          # inverter dead time; compensation: share the controller cancels
//     dead_time_compensation: 0.8
//     resolver: {error_deg: 0.1, order: 2}   # angle error, mechanical degrees, order per revolution
//     inverter: {conduction_w_per_a: 2.0, switching_w_per_a: 1.5}   # x peak current (switching at nominal V)
//
// Torque references come from MTPA (most torque per ampere) with field
// weakening when the voltage the DC link can give runs out; the current loop
// is a PI per axis with decoupling, one control period of computation delay,
// delay-compensated modulation and anti-windup at the voltage limit
// (space vector modulation: |v| <= V_dc / sqrt(3)). The inverter's dead time
// shifts each phase voltage against its current; the resolver's angle error
// turns the controller's frame. Torque 1.5 p (psi iq + (Ld - Lq) id iq).
struct PmsmParams {
  double pole_pairs = 5.0;
  double resistance = 0.1;          // [ohm]
  double ld = 0.12e-3, lq = 0.15e-3;  // [H]
  double flux = 0.03;               // [Wb]
  double current_max = 100.0;       // [A] peak
  double control_rate = 16000.0;    // [Hz]
  double bandwidth = 1000.0;        // [Hz]
  double dead_time = 1e-6;          // [s]
  double dead_time_compensation = 0.8;
  double resolver_error = 0.0;      // [rad] mechanical
  double resolver_order = 2.0;
  double conduction_loss = 2.0;     // [W/A]
  double switching_loss = 1.5;      // [W/A] at the nominal voltage
  double nominal_voltage = 520.0;   // [V] (from the Level A section)

  // Reads vehicle.powertrain.motor.pmsm.*; `motor` supplies the nominal voltage.
  static PmsmParams from_params(const ParamTree& params, const MotorParams& motor);
  double torque(double id, double iq) const { return 1.5 * pole_pairs * (flux * iq + (ld - lq) * id * iq); }
};

class PmsmDrive {
 public:
  struct Input {
    double torque = 0.0;      // command [N m], already within the torque and power envelope
    double speed = 0.0;       // rotor [rad/s], held over the step
    double angle = 0.0;       // rotor [rad] at the start of the step
    double dc_voltage = 0.0;  // [V]
  };
  // Averages over the step (what the plant sees) and the state at its end.
  struct Output {
    double torque = 0.0;      // [N m]
    double dc_power = 0.0;    // [W] into the inverter
    double loss = 0.0;        // [W] copper and inverter
    double id = 0.0, iq = 0.0, vd = 0.0, vq = 0.0;  // controller frame, end of the step
    double id_ref = 0.0, iq_ref = 0.0;
    double modulation = 0.0;  // |v| / (V_dc / sqrt(3)), 1 at the limit
  };
  // One control period's values, for tests and traces.
  struct Sample {
    double time = 0.0, torque = 0.0, id = 0.0, iq = 0.0, ia = 0.0;
  };

  explicit PmsmDrive(const PmsmParams& params = {});

  Output step(const Input& in, double dt);
  // Current references for a torque at a speed and DC voltage (MTPA, field weakening, limits).
  void references(double torque, double speed, double dc_voltage, double& id, double& iq) const;
  void reset();
  // Called once per control period during step() if set.
  std::function<void(const Sample&)> trace;
  const PmsmParams& params() const { return p_; }

 private:
  PmsmParams p_;
  double id_ = 0.0, iq_ = 0.0;            // true currents (rotor frame)
  double int_d_ = 0.0, int_q_ = 0.0;      // PI integrators [V]
  double v_alpha_ = 0.0, v_beta_ = 0.0;   // voltage applied over the next period (stationary frame)
  double time_ = 0.0;
  double kp_d_ = 0.0, kp_q_ = 0.0, ki_ = 0.0;
};

}  // namespace vsim
