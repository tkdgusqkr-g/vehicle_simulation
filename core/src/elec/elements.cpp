#include "vsim/elec/elements.hpp"

#include <algorithm>
#include <cmath>

namespace vsim::elec {
namespace {

// Elements stop switching piecewise-linear modes after this many Newton
// iterations so a step cannot cycle between two modes.
constexpr int kFreezeAfter = 20;

double blend(double h, double tau) { return tau > 0.0 && h > 0.0 ? 1.0 - std::exp(-h / tau) : 1.0; }

}  // namespace

// --- Resistor, switch ----------------------------------------------------------------------
double Resistor::conductance() const { return open ? kOpenConductance : 1.0 / std::max(resistance(), 1e-9); }

void Resistor::stamp(Assembler& as, const Voltages& /*v*/, const Step& /*step*/) {
  as.conductance(a, b, conductance());
}

void Switch::stamp(Assembler& as, const Voltages& /*v*/, const Step& /*step*/) {
  as.conductance(a, b, closed ? 1.0 / r_on : kOpenConductance);
}

double Switch::current(const Voltages& v) const { return (closed ? 1.0 / r_on : kOpenConductance) * v.across(a, b); }

// --- Capacitor: trapezoidal or backward-Euler companion -------------------------------------
void Capacitor::stamp(Assembler& as, const Voltages& /*v*/, const Step& step) {
  if (step.dc()) {
    as.conductance(a, b, kOpenConductance);
    return;
  }
  const double k = step.euler ? step.h / c : step.h / (2.0 * c);
  g_ = 1.0 / (esr + k);
  hist_ = step.euler ? -vc * g_ : -(vc + k * i) * g_;
  as.conductance(a, b, g_);
  as.current(a, b, hist_);
}

void Capacitor::accept(const Voltages& v, const Step& step) {
  if (step.dc()) {
    return;
  }
  const double i_new = g_ * v.across(a, b) + hist_;
  vc += step.euler ? step.h / c * i_new : step.h / (2.0 * c) * (i_new + i);
  i = i_new;
}

void Capacitor::initialize(const Voltages& v) {
  if (!keep_initial) {
    vc = v.across(a, b);
  }
  i = 0.0;
}

// --- Inductor --------------------------------------------------------------------------------
void Inductor::stamp(Assembler& as, const Voltages& /*v*/, const Step& step) {
  if (step.dc()) {
    g_ = 1.0 / std::max(r, 1e-6);
    hist_ = 0.0;
  } else if (step.euler) {
    const double k = l / step.h;
    g_ = 1.0 / (r + k);
    hist_ = k * i * g_;
  } else {
    const double k = 2.0 * l / step.h;
    g_ = 1.0 / (r + k);
    hist_ = g_ * (u + (k - r) * i);
  }
  as.conductance(a, b, g_);
  as.current(a, b, hist_);
}

void Inductor::accept(const Voltages& v, const Step& /*step*/) {
  u = v.across(a, b);
  i = g_ * u + hist_;
}

void Inductor::initialize(const Voltages& v) {
  u = v.across(a, b);
  i = u / std::max(r, 1e-6);
}

// --- Sources ---------------------------------------------------------------------------------
void VoltageSource::stamp(Assembler& as, const Voltages& /*v*/, const Step& /*step*/) {
  if (open) {
    as.conductance(pos, neg, kOpenConductance);
    return;
  }
  as.conductance(pos, neg, 1.0 / r);
  as.current(neg, pos, e / r);
}

void CurrentSource::stamp(Assembler& as, const Voltages& /*v*/, const Step& /*step*/) { as.current(neg, pos, i); }

// --- Load ------------------------------------------------------------------------------------
double Load::motor_resistance() const {
  double running = 0.0;
  switch (law) {
    case Law::Power: running = value / v_nominal; break;
    case Law::Current: running = value; break;
    case Law::Resistance: running = v_nominal / value; break;
  }
  return v_nominal / (inrush * std::max(running, 1e-9));
}

double Load::current(const Voltages& v) const {
  if (!on) {
    return kOpenConductance * v.across(a, b);
  }
  const double u = v.across(a, b);
  if (motor()) {
    return u > emf ? (u - emf) / motor_resistance() : 0.0;
  }
  switch (law) {
    case Law::Resistance: return u / value;
    case Law::Current: return u >= v_knee ? value : value * u / v_knee;
    case Law::Power: return u >= v_knee ? value / u : value * u / (v_knee * v_knee);
  }
  return 0.0;
}

void Load::stamp(Assembler& as, const Voltages& v, const Step& /*step*/) {
  if (!on) {
    as.conductance(a, b, kOpenConductance);
    return;
  }
  const double u = v.across(a, b);
  if (motor()) {
    if (u > emf) {
      const double g = 1.0 / motor_resistance();
      as.conductance(a, b, g);
      as.current(a, b, -emf * g);
    } else {
      as.conductance(a, b, kOpenConductance);
    }
    return;
  }
  switch (law) {
    case Law::Resistance:
      as.conductance(a, b, 1.0 / value);
      break;
    case Law::Current:
      if (u >= v_knee) {
        as.current(a, b, value);
      } else {
        as.conductance(a, b, value / v_knee);
      }
      break;
    case Law::Power:
      if (u >= v_knee) {
        const double g = -value / (u * u);
        as.conductance(a, b, g);
        as.current(a, b, 2.0 * value / u);
      } else {
        as.conductance(a, b, value / (v_knee * v_knee));
      }
      break;
  }
}

void Load::accept(const Voltages& v, const Step& step) {
  if (!motor()) {
    return;
  }
  const double target = on ? std::max(0.0, v.across(a, b)) * (1.0 - 1.0 / inrush) : 0.0;
  emf += blend(step.h, inrush_tau) * (target - emf);
}

void Load::switch_on(double v_now) {
  (void)v_now;
  on = true;
  emf = 0.0;  // the motor starts from rest
}

// --- Diode -----------------------------------------------------------------------------------
double Diode::current(const Voltages& v) const {
  return on ? (v.across(a, b) - v_f) / r_on : 1e-9 * v.across(a, b);
}

void Diode::stamp(Assembler& as, const Voltages& v, const Step& /*step*/) {
  if (as.iteration() < kFreezeAfter) {
    const double u = v.across(a, b);
    const bool next = on ? u >= v_f - 1e-9 : u > v_f;
    if (next != on) {
      on = next;
      as.changed();
    }
  }
  if (on) {
    as.conductance(a, b, 1.0 / r_on);
    as.current(a, b, -v_f / r_on);
  } else {
    as.conductance(a, b, 1e-9);
  }
}

// --- Linear regulator ------------------------------------------------------------------------
// Modes are ordered by output current (off, regulating, limited) and move one
// step per iteration: jumping from limited straight to off (or back) would
// follow the extreme solution of each mode and cycle.
Regulator::Mode Regulator::choose(const Voltages& v) const {
  const double headroom = v.across(in, gnd) - dropout;
  const double e = std::min(v_set, headroom);
  if (e <= 0.0) {
    return Mode::Off;
  }
  const Mode linear = headroom < v_set ? Mode::Dropout : Mode::Regulating;
  const double i = (e - v.across(out, gnd)) / r_out;
  switch (mode) {
    case Mode::Off: return i > 0.0 ? linear : Mode::Off;
    case Mode::Limited: return i < i_max ? linear : Mode::Limited;
    default: return i > i_max ? Mode::Limited : i < 0.0 ? Mode::Off : linear;
  }
}

double Regulator::current(const Voltages& v) const {
  switch (mode) {
    case Mode::Regulating: return (v_set - v.across(out, gnd)) / r_out;
    case Mode::Dropout: return (v.across(in, out) - dropout) / r_out;
    case Mode::Limited: return i_max;
    case Mode::Off: return 0.0;
  }
  return 0.0;
}

void Regulator::stamp(Assembler& as, const Voltages& v, const Step& /*step*/) {
  if (as.iteration() < kFreezeAfter) {
    const Mode next = choose(v);
    if (next != mode) {
      mode = next;
      as.changed();
    }
  }
  const double g = 1.0 / r_out;
  switch (mode) {
    case Mode::Regulating:
      as.transconductance(in, out, gnd, out, g);
      as.current(in, out, v_set * g);
      break;
    case Mode::Dropout:
      as.conductance(in, out, g);
      as.current(in, out, -dropout * g);
      break;
    case Mode::Limited:
      as.current(in, out, i_max);
      break;
    case Mode::Off:
      as.conductance(in, out, kOpenConductance);
      break;
  }
  // Quiescent current, fading into a resistance below 1 V.
  if (v.across(in, gnd) >= 1.0) {
    as.current(in, gnd, i_q);
  } else {
    as.conductance(in, gnd, i_q);
  }
}

// --- DC-DC converter -------------------------------------------------------------------------
DcDc::Mode DcDc::choose(const Voltages& v) const {
  if (!on) {
    return Mode::Blocked;
  }
  const double i = (e - v.across(out_p, out_n)) / r_out;
  switch (mode) {  // one step at a time, as for the regulator
    case Mode::Blocked: return i > 0.0 ? Mode::Normal : Mode::Blocked;
    case Mode::Limited: return i < i_max ? Mode::Normal : Mode::Limited;
    case Mode::Normal: return i > i_max ? Mode::Limited : i < 0.0 ? Mode::Blocked : Mode::Normal;
  }
  return Mode::Normal;
}

double DcDc::output_current(const Voltages& v) const {
  switch (mode) {
    case Mode::Normal: return (e - v.across(out_p, out_n)) / r_out;
    case Mode::Limited: return i_max;
    case Mode::Blocked: return 0.0;
  }
  return 0.0;
}

double DcDc::input_current(const Voltages& v) const {
  if (!on) {
    return 0.0;
  }
  const double p = std::max(0.0, v.across(out_p, out_n) * output_current(v)) / efficiency + p_idle;
  const double u = v.across(in_p, in_n);
  return u >= v_knee ? p / u : p * u / (v_knee * v_knee);
}

void DcDc::stamp(Assembler& as, const Voltages& v, const Step& step) {
  if (step.dc() && on) {
    // Operating point: an ideal regulated output (the controller has settled).
    constexpr double r = 1e-4;
    const double i = (v_set - v.across(out_p, out_n)) / r;
    const Mode next = mode == Mode::Limited ? (i < i_max ? Mode::Normal : Mode::Limited)
                                            : (i > i_max ? Mode::Limited : Mode::Normal);
    if (next != mode && as.iteration() < kFreezeAfter) {
      mode = next;
      as.changed();
    }
    if (mode == Mode::Limited) {
      as.current(out_n, out_p, i_max);
    } else {
      as.conductance(out_p, out_n, 1.0 / r);
      as.current(out_n, out_p, v_set / r);
    }
  } else {
    if (as.iteration() < kFreezeAfter) {
      const Mode next = choose(v);
      if (next != mode) {
        mode = next;
        as.changed();
      }
    }
    switch (mode) {
      case Mode::Normal:
        as.conductance(out_p, out_n, 1.0 / r_out);
        as.current(out_n, out_p, e / r_out);
        break;
      case Mode::Limited:
        as.current(out_n, out_p, i_max);
        break;
      case Mode::Blocked:
        as.conductance(out_p, out_n, kOpenConductance);
        break;
    }
  }
  // Input: constant power at the present output power (fixed point within the step).
  if (!on) {
    as.conductance(in_p, in_n, kOpenConductance);
    return;
  }
  double i_out = 0.0;
  if (step.dc() && mode == Mode::Normal) {
    i_out = (v_set - v.across(out_p, out_n)) / 1e-4;
  } else {
    i_out = output_current(v);
  }
  const double p = std::max(0.0, v.across(out_p, out_n) * i_out) / efficiency + p_idle;
  const double u = v.across(in_p, in_n);
  if (u >= v_knee) {
    as.conductance(in_p, in_n, -p / (u * u));
    as.current(in_p, in_n, 2.0 * p / u);
  } else {
    as.conductance(in_p, in_n, p / (v_knee * v_knee));
  }
}

void DcDc::accept(const Voltages& v, const Step& step) {
  if (step.dc()) {
    return;
  }
  const double i_out = output_current(v);
  p_out = std::max(0.0, v.across(out_p, out_n) * i_out);
  if (!on) {
    e = 0.0;
    return;
  }
  const double target = v_set + r_out * std::clamp(i_out, 0.0, i_max);
  e += blend(step.h, response) * (target - e);
  e = std::clamp(e, 0.0, v_set + r_out * i_max);
}

void DcDc::initialize(const Voltages& v) {
  if (!on) {
    e = 0.0;
    p_out = 0.0;
    return;
  }
  const double u = v.across(out_p, out_n);
  const double i_out = mode == Mode::Limited ? i_max : std::clamp((v_set - u) / 1e-4, 0.0, i_max);
  e = std::min(u + r_out * i_out, v_set + r_out * i_max);
  p_out = u * i_out;
  mode = Mode::Normal;
}

void DcDc::turn(bool on_now, const Voltages& v) {
  if (on_now && !on) {
    e = std::max(0.0, v.across(out_p, out_n));  // soft start from the present output voltage
  }
  on = on_now;
}

// --- Potentiometer ---------------------------------------------------------------------------
void Potentiometer::stamp(Assembler& as, const Voltages& /*v*/, const Step& /*step*/) {
  const double t = std::clamp(x, 1e-4, 1.0 - 1e-4);
  as.conductance(a, w, 1.0 / (r * (1.0 - t)));
  as.conductance(w, b, 1.0 / (r * t));
}

double Potentiometer::supply_current(const Voltages& v) const {
  const double t = std::clamp(x, 1e-4, 1.0 - 1e-4);
  return v.across(a, w) / (r * (1.0 - t));
}

}  // namespace vsim::elec
