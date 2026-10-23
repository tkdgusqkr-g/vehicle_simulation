#pragma once

#include <cstddef>
#include <deque>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "vsim/sim/params.hpp"
#include "vsim/sim/rng.hpp"

namespace vsim {

// Error model of one sensor channel (design doc §10.5): what the transducer,
// its noise, the converter and the sensor's processing do to a physical
// value, sampled at the sensor's rate.
struct ChannelParams {
  // Per-unit errors, drawn once per sensor (realism.variation).
  double scale_error = 0.0;        // standard deviation of the relative scale error
  double bias = 0.0;               // standard deviation of the fixed bias [unit]
  // Running errors (realism.sensors).
  double noise_density = 0.0;     // white noise [unit / sqrt(Hz)]; Allan deviation N / sqrt(tau)
  double bias_instability = 0.0;   // Allan deviation floor is about 0.664 times this [unit]
  double bias_tau = 100.0;         // [s] correlation time of the drifting bias
  double random_walk = 0.0;        // [unit * sqrt(Hz)]; Allan deviation K sqrt(tau / 3)
  double bandwidth = 0.0;          // [Hz] first-order response of the transducer; 0: none
  double latency = 0.0;            // [s] output delay (processing, transport)
  double resolution = 0.0;         // [unit] quantization step (ADC, data format); 0: none
  double min = -std::numeric_limits<double>::infinity();  // output range (saturation)
  double max = std::numeric_limits<double>::infinity();
  // Transducer: temperature coefficients (per unit, realism.variation) against the sensor's
  // temperature, terminal-based nonlinearity (per unit, zero at 0 and full scale) and hysteresis.
  double bias_tempco = 0.0;        // standard deviation of the bias temperature coefficient [unit/K]
  double scale_tempco = 0.0;       // standard deviation of the scale temperature coefficient [1/K]
  double reference_temp = 25.0;    // [degC] where the coefficients give no error
  double nonlinearity = 0.0;       // standard deviation of the largest deviation, fraction of full scale
  double full_scale = 0.0;         // [unit]; 0: the larger end of the range (none if unbounded)
  double hysteresis = 0.0;         // [unit] width of the play band (realism.sensors)

  // Reads <prefix>.<key> for scale_error, bias, noise_density,
  // bias_instability, bias_tau_s, random_walk, bandwidth_hz, latency_s,
  // resolution, range: [min, max], bias_tempco, scale_tempco, reference_temp_c,
  // nonlinearity, full_scale, hysteresis; missing keys keep `defaults`.
  static ChannelParams from_params(const ParamTree& params, const std::string& prefix, ChannelParams defaults);
};

// A fault injected into a channel's output (design doc §10.5, sensor and
// wiring faults): from at_s until until_s (forever if negative).
struct ChannelFault {
  enum class Kind { Offset, Stuck, Value, Noise, Scale };
  Kind kind = Kind::Offset;
  double value = 0.0;   // offset, forced value (open: 0 V, short: supply), noise sigma, or scale factor
  double at = 0.0;      // [s]
  double until = -1.0;  // [s]; negative: until the end
};

class ChannelModel {
 public:
  // rate: samples per second. `real`: running errors on (realism.sensors);
  // `variation`: per-unit errors on (realism.variation). The two streams are
  // independent so switching one never shifts the other's random sequence.
  ChannelModel(const ChannelParams& params, double rate, Rng noise, Rng instance, bool real, bool variation);

  // One sample of the physical value at time t; returns the sensor output.
  double sample(double t, double truth);
  // The sensor's own temperature [degC], read at every sample (reference if not set).
  void set_temperature_source(const double* temperature) { temperature_ = temperature; }
  double output() const { return output_; }
  void add_fault(const ChannelFault& fault) { faults_.push_back(fault); }

 private:
  ChannelParams p_;
  double dt_;
  bool real_;
  Rng noise_;
  double scale_ = 0.0, bias_ = 0.0;  // per unit
  double bias_tempco_ = 0.0, scale_tempco_ = 0.0, nonlinear_ = 0.0;  // per unit
  double full_scale_ = 1.0, play_ = 0.0;
  bool play_started_ = false;
  const double* temperature_ = nullptr;
  double markov_ = 0.0, markov_decay_ = 0.0, markov_kick_ = 0.0;
  double walk_ = 0.0, walk_kick_ = 0.0;
  double white_ = 0.0;
  double filtered_ = 0.0, filter_gain_ = 1.0;
  bool started_ = false;
  std::deque<double> delay_;
  std::size_t delay_samples_ = 0;
  double output_ = 0.0;
  std::vector<ChannelFault> faults_;
  std::vector<double> stuck_;  // value held by each stuck fault, NaN until it starts
};

// Faults listed under <prefix>.faults: [{channel, type, at_s, until_s, value}]
// for the named channel. type: offset | stuck | open (value 0) | short
// (value = `supply`) | value | noise | scale.
std::vector<ChannelFault> faults_from_params(const ParamTree& params, const std::string& prefix,
                                             std::string_view channel, double supply);

}  // namespace vsim
