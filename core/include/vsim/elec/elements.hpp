#pragma once

#include <string>

#include "vsim/elec/circuit.hpp"

namespace vsim::elec {

inline constexpr double kOpenConductance = 1e-12;  // [S] an open contact or a broken wire

// Linear resistor. `open` and `extra` carry faults (a broken wire, a
// corroded pin); the network calls Circuit::invalidate() when they change.
class Resistor : public Element {
 public:
  Resistor(std::string id, NodeId from, NodeId to, double ohms) : Element(std::move(id)), a(from), b(to), r(ohms) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  double resistance() const { return r + extra; }
  double conductance() const;
  double current(const Voltages& v) const { return conductance() * v.across(a, b); }  // a -> b

  NodeId a, b;
  double r;
  double extra = 0.0;
  bool open = false;
};

// Contact or semiconductor switch: r_on closed, open otherwise.
class Switch : public Element {
 public:
  Switch(std::string id, NodeId from, NodeId to, double ohms_on)
      : Element(std::move(id)), a(from), b(to), r_on(ohms_on) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  double current(const Voltages& v) const;

  NodeId a, b;
  double r_on;
  bool closed = false;
};

// Capacitor with series resistance (ESR). State: capacitor voltage and current.
class Capacitor : public Element {
 public:
  Capacitor(std::string id, NodeId from, NodeId to, double farads, double esr_ohms)
      : Element(std::move(id)), a(from), b(to), c(farads), esr(esr_ohms) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  void accept(const Voltages& v, const Step& step) override;
  void initialize(const Voltages& v) override;

  NodeId a, b;
  double c, esr;
  double vc = 0.0;  // [V] across the capacitance
  double i = 0.0;   // [A] a -> b
  bool keep_initial = false;  // the operating point keeps vc (a given starting charge)

 private:
  double g_ = 0.0, hist_ = 0.0;  // companion of the step being solved
};

// Inductor with series resistance. State: current and voltage.
class Inductor : public Element {
 public:
  Inductor(std::string id, NodeId from, NodeId to, double henries, double ohms)
      : Element(std::move(id)), a(from), b(to), l(henries), r(ohms) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  void accept(const Voltages& v, const Step& step) override;
  void initialize(const Voltages& v) override;

  NodeId a, b;
  double l, r;
  double i = 0.0;  // [A] a -> b
  double u = 0.0;  // [V] across

 private:
  double g_ = 0.0, hist_ = 0.0;
};

// Voltage source e behind a series resistance (Norton form): a battery, a
// supply rail. Current is reported out of pos.
class VoltageSource : public Element {
 public:
  VoltageSource(std::string id, NodeId plus, NodeId minus, double volts, double ohms)
      : Element(std::move(id)), pos(plus), neg(minus), e(volts), r(ohms) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  double current(const Voltages& v) const { return (e - v.across(pos, neg)) / r; }

  NodeId pos, neg;
  double e, r;
  bool open = false;
};

// Current source: i flows out of pos into the circuit.
class CurrentSource : public Element {
 public:
  CurrentSource(std::string id, NodeId plus, NodeId minus, double amps)
      : Element(std::move(id)), pos(plus), neg(minus), i(amps) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;

  NodeId pos, neg;
  double i;
};

// A load drawing constant power (inverter DC side, switching converter
// input), constant current (linear regulators, ECUs) or a resistance, with a
// motor inrush model: at switch-on a DC motor draws `inrush` times its
// running current and its back-EMF builds with `inrush_tau`. Below `v_knee`
// the power and current laws fade into a resistance so the load stays
// defined at 0 V. Negative power feeds the circuit (regenerative braking).
class Load : public Element {
 public:
  enum class Law { Power, Current, Resistance };
  Load(std::string id, NodeId from, NodeId to, Law kind, double amount)
      : Element(std::move(id)), a(from), b(to), law(kind), value(amount) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  void accept(const Voltages& v, const Step& step) override;
  bool nonlinear() const override { return law != Law::Resistance; }
  double current(const Voltages& v) const;  // a -> b
  void switch_on(double v_now);              // starts the inrush

  NodeId a, b;
  Law law;
  double value;           // [W], [A] or [ohm]
  double v_knee = 1.0;    // [V]
  bool on = true;
  double inrush = 1.0;    // locked-rotor current / running current at v_nominal
  double inrush_tau = 0.0;
  double v_nominal = 12.0;
  double emf = 0.0;       // [V] back-EMF of the motor (inrush model)

 private:
  bool motor() const { return inrush > 1.0 && inrush_tau > 0.0; }
  double motor_resistance() const;
};

// Piecewise-linear diode: v_f + r_on when conducting, open otherwise.
class Diode : public Element {
 public:
  Diode(std::string id, NodeId anode, NodeId cathode, double forward, double ohms_on)
      : Element(std::move(id)), a(anode), b(cathode), v_f(forward), r_on(ohms_on) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  bool nonlinear() const override { return true; }
  double current(const Voltages& v) const;

  NodeId a, b;
  double v_f, r_on;
  bool on = false;
};

// Linear regulator (an ECU's 5 V sensor supply): pass element from `in` to
// `out`, output set point v_set behind r_out, dropout, current limit,
// quiescent current to `gnd`.
class Regulator : public Element {
 public:
  Regulator(std::string id, NodeId input, NodeId output, NodeId ground, double setpoint)
      : Element(std::move(id)), in(input), out(output), gnd(ground), v_set(setpoint) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  bool nonlinear() const override { return true; }
  double current(const Voltages& v) const;  // pass current in -> out

  NodeId in, out, gnd;
  double v_set;
  double dropout = 1.0;   // [V]
  double r_out = 0.05;    // [ohm]
  double i_max = 0.5;     // [A]
  double i_q = 0.002;     // [A]
  enum class Mode { Regulating, Dropout, Limited, Off } mode = Mode::Regulating;

 private:
  Mode choose(const Voltages& v) const;
};

// Switching DC-DC converter, average model: the output is an internal
// voltage e behind r_out (e settles so the output sits at v_set, with the
// time constant `response`, which makes load steps dip and recover), limited
// to [0, i_max]; the input draws the output power / efficiency plus idle power.
class DcDc : public Element {
 public:
  DcDc(std::string id, NodeId input_p, NodeId input_n, NodeId output_p, NodeId output_n, double setpoint)
      : Element(std::move(id)), in_p(input_p), in_n(input_n), out_p(output_p), out_n(output_n), v_set(setpoint) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  void accept(const Voltages& v, const Step& step) override;
  void initialize(const Voltages& v) override;
  bool nonlinear() const override { return true; }
  double output_current(const Voltages& v) const;
  double input_current(const Voltages& v) const;
  void turn(bool on_now, const Voltages& v);

  NodeId in_p, in_n, out_p, out_n;
  double v_set;
  double efficiency = 0.9;
  double r_out = 0.02;     // [ohm]
  double i_max = 20.0;     // [A]
  double response = 0.002; // [s]
  double p_idle = 2.0;     // [W]
  double v_knee = 20.0;    // [V] input
  bool on = true;
  double e = 0.0;          // [V] internal voltage
  double p_out = 0.0;      // [W] of the last accepted step
  enum class Mode { Normal, Limited, Blocked } mode = Mode::Normal;

 private:
  Mode choose(const Voltages& v) const;
};

// Potentiometer (pedal sensor track): r from a to wiper is r (1 - x), wiper
// to b is r x.
class Potentiometer : public Element {
 public:
  Potentiometer(std::string id, NodeId end_a, NodeId end_b, NodeId wiper, double ohms)
      : Element(std::move(id)), a(end_a), b(end_b), w(wiper), r(ohms) {}
  void stamp(Assembler& as, const Voltages& v, const Step& step) override;
  double supply_current(const Voltages& v) const;

  NodeId a, b, w;
  double r;
  double x = 0.5;
};

}  // namespace vsim::elec
