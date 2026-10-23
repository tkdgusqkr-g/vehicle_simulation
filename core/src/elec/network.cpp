#include "vsim/elec/network.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "vsim/elec/circuit.hpp"
#include "vsim/elec/elements.hpp"
#include "vsim/models/powertrain/battery.hpp"
#include "vsim/sim/context.hpp"
#include "vsim/sim/input_hub.hpp"
#include "vsim/sim/realism.hpp"

namespace vsim {

using elec::kGround;
using elec::NodeId;
using elec::Voltages;
using Json = nlohmann::json;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-12;

// Copper and aluminium at 20 degC.
constexpr double kRhoCopper = 1.724e-8, kAlphaCopper = 0.00393;
constexpr double kRhoAluminium = 2.82e-8, kAlphaAluminium = 0.0039;

// Current a single insulated conductor in free air carries for about 50 K
// rise (approximate figures for automotive and chassis wire; give rating_a
// for the real wire).
double default_ampacity(double area_mm2) {
  static const std::array<std::pair<double, double>, 16> table = {{
      {0.35, 6}, {0.5, 9}, {0.75, 12}, {1.0, 15}, {1.5, 19}, {2.5, 26}, {4, 35}, {6, 45},
      {10, 65}, {16, 87}, {25, 115}, {35, 140}, {50, 175}, {70, 215}, {95, 260}, {120, 300},
  }};
  if (area_mm2 <= table.front().first) {
    return table.front().second * area_mm2 / table.front().first;
  }
  for (std::size_t i = 1; i < table.size(); ++i) {
    if (area_mm2 <= table[i].first) {
      const double t = std::log(area_mm2 / table[i - 1].first) / std::log(table[i].first / table[i - 1].first);
      return table[i - 1].second * std::pow(table[i].second / table[i - 1].second, t);
    }
  }
  return table.back().second * std::pow(area_mm2 / table.back().first, 0.6);
}

bool is_identifier(const std::string& s) {
  if (s.empty() || !(std::isalpha(static_cast<unsigned char>(s[0])) || s[0] == '_')) {
    return false;
  }
  return std::all_of(s.begin(), s.end(), [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '_'; });
}

// A component's entry in electrical.components.
struct Spec {
  const ParamTree* p = nullptr;
  std::string prefix, id, type;

  std::string key(const std::string& k) const { return prefix + "." + k; }
  bool has(const std::string& k) const { return p->has(key(k)) || p->has_prefix(key(k)); }
  std::string where() const { return prefix + " (" + id + ")"; }
  [[noreturn]] void fail(const std::string& what) const { throw std::invalid_argument(where() + ": " + what); }
  double num(const std::string& k, double fallback) const { return p->get_double(key(k), fallback); }
  double num(const std::string& k) const {
    if (!p->has(key(k))) {
      fail("needs " + k + " (" + type + ")");
    }
    return p->get_double(key(k));
  }
  double positive(const std::string& k, double fallback) const {
    const double v = num(k, fallback);
    if (!(v > 0.0)) {
      fail(k + " must be positive");
    }
    return v;
  }
  double positive(const std::string& k) const {
    const double v = num(k);
    if (!(v > 0.0)) {
      fail(k + " must be positive");
    }
    return v;
  }
  std::string str(const std::string& k, const std::string& fallback) const { return p->get_string(key(k), fallback); }
  std::string str(const std::string& k) const {
    if (!p->has(key(k))) {
      fail("needs " + k + " (" + type + ")");
    }
    return p->get_string(key(k));
  }
  bool flag(const std::string& k, bool fallback) const { return p->get_bool(key(k), fallback); }
  std::vector<std::string> list(const std::string& k) const {
    return p->has(key(k)) ? std::vector<std::string>{p->get_string(key(k))} : p->get_string_list(key(k));
  }
};

// A condition on bus signals: `signal` alone means "above 0.5"; maps take
// above / below (with an optional hysteresis band, like a thermostat) and
// nest with all / any.
struct Condition {
  std::string signal;
  std::optional<SignalId> id;
  std::optional<double> above, below;
  double hysteresis = 0.0;
  mutable bool held = false;  // the leaf was true last time (hysteresis)
  std::vector<Condition> all, any;

  static Condition parse(const ParamTree& p, const std::string& prefix) {
    Condition c;
    if (p.has(prefix)) {
      c.signal = p.get_string(prefix);
      c.above = 0.5;
      return c;
    }
    for (const char* group : {"all", "any"}) {
      for (std::size_t i = 0; p.has_prefix(prefix + "." + group + "." + std::to_string(i)); ++i) {
        (group[1] == 'l' ? c.all : c.any).push_back(parse(p, prefix + "." + group + "." + std::to_string(i)));
      }
    }
    if (p.has(prefix + ".signal")) {
      c.signal = p.get_string(prefix + ".signal");
      if (p.has(prefix + ".above")) {
        c.above = p.get_double(prefix + ".above");
      }
      if (p.has(prefix + ".below")) {
        c.below = p.get_double(prefix + ".below");
      }
      if (!c.above && !c.below) {
        c.above = 0.5;
      }
      c.hysteresis = p.get_double(prefix + ".hysteresis", 0.0);
      if (c.hysteresis < 0.0) {
        throw std::invalid_argument(prefix + ".hysteresis must not be negative");
      }
    } else if (c.all.empty() && c.any.empty()) {
      throw std::invalid_argument(prefix + ": a condition needs signal (with above or below), all or any");
    }
    return c;
  }
  void resolve(const SignalBus& bus, const std::string& where, bool lenient) {
    if (!signal.empty()) {
      id = bus.find(signal);
      if (!id && !lenient) {
        throw std::invalid_argument(where + ": no signal '" + signal + "'");
      }
    }
    for (Condition& c : all) {
      c.resolve(bus, where, lenient);
    }
    for (Condition& c : any) {
      c.resolve(bus, where, lenient);
    }
  }
  bool eval(const SignalBus& bus) const {
    bool result = true;
    if (!signal.empty()) {
      const double v = id ? bus.get(*id) : 0.0;
      const double band = held ? hysteresis : 0.0;
      result = (!above || v > *above - band) && (!below || v < *below + band);
      held = result;
    }
    for (const Condition& c : all) {
      result = result && c.eval(bus);
    }
    if (!any.empty()) {
      bool one = false;
      for (const Condition& c : any) {
        one = one || c.eval(bus);
      }
      result = result && one;
    }
    return result;
  }
};

// A condition that has to hold for `delay_s`, optionally latching.
struct Trigger {
  Condition condition;
  double delay = 0.0;
  bool latch = false;
  double since = -1.0;
  bool state = false;

  static Trigger parse(const ParamTree& p, const std::string& prefix) {
    Trigger t;
    t.condition = Condition::parse(p, prefix);
    t.delay = p.get_double(prefix + ".delay_s", 0.0);
    t.latch = p.get_bool(prefix + ".latch", false);
    return t;
  }
  bool update(double t, const SignalBus& bus) {
    if (latch && state) {
      return true;
    }
    if (condition.eval(bus)) {
      if (since < 0.0) {
        since = t;
      }
      state = t - since >= delay - 1e-9;
    } else {
      since = -1.0;
      state = false;
    }
    return state;
  }
};

struct FaultEffect {
  bool open = false;
  double extra = 0.0;
  int stuck = 0;  // +1 closed, -1 open
  bool blow = false;
};

struct Fault {
  std::string type, target, where;
  double at = 0.0, until = kInf, r = 0.001;
  double rate = 0.0, duration = 0.0;  // intermittent: outages per second, each this long
  double outage_until = -1.0;
  elec::Resistor* short_element = nullptr;
};

struct Event {
  double t;
  std::string type, id, detail;
};

// Running statistics of a quantity sampled once per plant step.
struct Stat {
  double sum = 0.0, max = -kInf, min = kInf, sum_sq = 0.0;
  std::int64_t n = 0;
  void add(double v) {
    sum += v;
    sum_sq += v * v;
    max = std::max(max, v);
    min = std::min(min, v);
    ++n;
  }
  double mean() const { return n > 0 ? sum / static_cast<double>(n) : 0.0; }
  double rms() const { return n > 0 ? std::sqrt(sum_sq / static_cast<double>(n)) : 0.0; }
  double peak() const { return n > 0 ? max : 0.0; }
  double low() const { return n > 0 ? min : 0.0; }
};

class Device {
 public:
  virtual ~Device() = default;
  std::string id, type;
  // Signals produced by blocks initialised after the network: found at the first step.
  virtual void resolve(ElectricalNetwork::Impl& /*n*/, const SignalBus& /*bus*/) {}
  virtual void inputs(ElectricalNetwork::Impl& /*n*/, double /*t*/, const SignalBus& /*bus*/) {}
  // After each sub-step; true when a contact or a mode changed (the next
  // step starts with backward Euler from a fresh factorisation).
  virtual bool update(ElectricalNetwork::Impl& /*n*/, double /*t*/, double /*h*/, const Voltages& /*v*/) {
    return false;
  }
  virtual void outputs(ElectricalNetwork::Impl& /*n*/, double /*t*/, SignalBus& /*bus*/, const Voltages& /*v*/) {}
  // States from the starting point (initial: active) or from rest (off).
  virtual void start(ElectricalNetwork::Impl& /*n*/, const Voltages& /*v*/, bool /*active*/) {}
  // Fault effects for this tick; true when the circuit changed.
  virtual bool apply(const FaultEffect& /*f*/) { return false; }
  virtual bool takes(const std::string& /*fault*/) const { return false; }
  virtual void summary(Json& /*out*/) const {}
  // Controllers update after the other parts within a sub-step.
  virtual bool controller() const { return false; }
  // Terminal nodes by name (for short-circuit targets "<id>.<terminal>").
  std::map<std::string, NodeId> terminals;
};

}  // namespace

struct ElectricalNetwork::Impl {
  elec::Circuit circuit;
  std::string level = "E2";
  bool dynamic = true;
  bool active_start = true;
  bool standalone = false;
  double dt = 0.001, h = 0.001;
  int substeps = 1;
  std::vector<std::unique_ptr<Device>> devices;
  std::map<std::string, Device*> by_id;
  std::vector<Fault> faults;
  std::map<std::string, std::vector<Device*>> pins;  // connector pin -> wires through it
  std::vector<Event> events;
  std::optional<Rng> rng;
  Context* ctx = nullptr;
  SignalBus* bus = nullptr;
  bool started = false;
  bool discontinuity = true;
  bool recording = true;  // statistics and events (off while the start settles)
  double contact_r = 0.002;
  double ambient = 25.0;
  std::vector<SignalId> node_out;
  SignalId ts_voltage_out = 0, ts_active_out = 0;
  std::vector<std::string> inverter_motors;
  std::vector<std::string> plant_motors;
  bool carries_battery = false;

  SignalId out(const std::string& id, const std::string& what, const std::string& unit, const std::string& text) {
    return bus->declare("elec." + id + "." + what, unit, text);
  }
  NodeId node(const Spec& s, const std::string& key) {
    const std::string name = s.str(key);
    if (name.empty()) {
      s.fail(key + " is empty");
    }
    return circuit.node(name);
  }
  void event(double t, const std::string& type, const std::string& id, const std::string& detail) {
    if (recording && events.size() < 2000) {
      events.push_back({t, type, id, detail});
    }
  }

  void build(const ParamTree& params);
  void start(SignalBus& bus_in);
  void apply_faults(double t);
  void publish(double t, SignalBus& bus_out);
};

namespace {

using Impl = ElectricalNetwork::Impl;

std::string fmt(double v, int digits = 3) {
  char b[32];
  std::snprintf(b, sizeof b, "%.*g", digits, v);
  return b;
}

// --- Passive parts -----------------------------------------------------------------------------
class ResistorDevice : public Device {
 public:
  ResistorDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    r_ = &n.circuit.emplace<elec::Resistor>(s.id, a, b, s.positive("r_ohm"));
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
    p_out_ = n.out(s.id, "p", "W", "power dissipated");
  }
  bool takes(const std::string& f) const override { return f == "open" || f == "contact_resistance"; }
  bool apply(const FaultEffect& f) override {
    const bool changed = r_->open != f.open || r_->extra != f.extra;
    r_->open = f.open;
    r_->extra = f.extra;
    return changed;
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double i = r_->current(v);
    bus.set(i_out_, i);
    bus.set(p_out_, i * v.across(r_->a, r_->b));
  }

 protected:
  elec::Resistor* r_ = nullptr;
  SignalId i_out_ = 0, p_out_ = 0;
};

// Harness wire: resistance from length, cross-section and temperature,
// connector pins in series, lumped thermal model, ampacity check.
class WireDevice : public Device {
 public:
  WireDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    area_ = s.positive("area_mm2");
    if (s.has("route")) {
      for (std::size_t i = 0; s.p->has_prefix(s.key("route." + std::to_string(i))); ++i) {
        const std::vector<double> pt = s.p->get_double_list(s.key("route." + std::to_string(i)));
        if (pt.size() != 3) {
          s.fail("route points are [x, y, z] in m");
        }
        route_.push_back({pt[0], pt[1], pt[2]});
      }
      double length = 0.0;
      for (std::size_t i = 1; i < route_.size(); ++i) {
        length += std::hypot(route_[i][0] - route_[i - 1][0], route_[i][1] - route_[i - 1][1],
                             route_[i][2] - route_[i - 1][2]);
      }
      length_ = length * s.num("slack", 1.1);
    } else {
      length_ = s.positive("length_m");
    }
    if (!(length_ > 0.0)) {
      s.fail("the wire has no length (length_m or a route of two points or more)");
    }
    const std::string material = s.str("material", "copper");
    if (material == "copper") {
      rho_ = kRhoCopper;
      alpha_ = kAlphaCopper;
    } else if (material == "aluminium" || material == "aluminum") {
      rho_ = kRhoAluminium;
      alpha_ = kAlphaAluminium;
    } else {
      s.fail("material '" + material + "' (expected copper or aluminium)");
    }
    pin_names_ = s.list("pins");
    for (const std::string& pin : pin_names_) {
      n.pins[pin].push_back(this);
    }
    pin_r_ = static_cast<double>(pin_names_.size()) * s.num("contact_r_ohm", n.contact_r);
    rating_ = s.num("rating_a", default_ampacity(area_));
    rise_ = s.positive("rated_rise_k", 50.0);
    temp_ = s.num("temp_c", n.ambient);
    ambient_ = n.ambient;
    thermal_ = s.flag("thermal", true);
    // Copper (plus insulation) heat capacity and the thermal resistance that
    // makes the rated current give the rated rise.
    heat_capacity_ = 1.6 * area_ * 1e-6 * length_ * 8960.0 * 385.0;
    thermal_resistance_ = rise_ / (rating_ * rating_ * r_at(ambient_ + rise_));
    r_ = &n.circuit.emplace<elec::Resistor>(s.id, a, b, r_at(temp_));
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
    drop_out_ = n.out(s.id, "drop", "V", "voltage drop a -> b");
    temp_out_ = n.out(s.id, "temp", "degC", "conductor temperature");
    overload_out_ = n.out(s.id, "overload", "-", "1 while hotter than its rating (ambient + rated rise)");
  }
  double r_at(double temp) const { return rho_ * (1.0 + alpha_ * (temp - 20.0)) * length_ / (area_ * 1e-6) + pin_r_; }
  bool takes(const std::string& f) const override { return f == "open" || f == "contact_resistance"; }
  bool apply(const FaultEffect& f) override {
    const bool changed = r_->open != f.open || r_->extra != f.extra;
    r_->open = f.open;
    r_->extra = f.extra;
    return changed;
  }
  bool update(Impl& /*n*/, double /*t*/, double h, const Voltages& v) override {
    if (thermal_) {
      const double i = r_->current(v);
      const double heat = i * i * r_->r - (temp_ - ambient_) / thermal_resistance_;
      temp_ += h * heat / heat_capacity_;
      r_->r = r_at(temp_);
    }
    return false;
  }
  void outputs(Impl& n, double t, SignalBus& bus, const Voltages& v) override {
    const double i = r_->current(v);
    const double drop = v.across(r_->a, r_->b);
    const double level = std::max(std::abs(v(r_->a)), std::abs(v(r_->b)));
    bus.set(i_out_, i);
    bus.set(drop_out_, drop);
    bus.set(temp_out_, temp_);
    const bool over = temp_ > ambient_ + rise_;
    bus.set(overload_out_, over ? 1.0 : 0.0);
    if (over && !was_over_) {
      n.event(t, "wire_overheat", id, fmt(temp_) + " degC at " + fmt(std::abs(i)) + " A (rated " + fmt(rating_) + " A)");
    }
    was_over_ = over;
    if (!n.recording) {
      return;
    }
    current_.add(std::abs(i));
    drop_.add(std::abs(drop));
    if (level > 1.0) {
      drop_pct_ = std::max(drop_pct_, 100.0 * std::abs(drop) / level);
    }
    temp_max_ = std::max(temp_max_, temp_);
    loss_ += i * i * r_->resistance() * n.dt;
    over_time_ += over ? n.dt : 0.0;
  }
  void summary(Json& out) const override {
    Json& w = out["wires"][id];
    w = {{"length_m", length_}, {"area_mm2", area_}, {"r_mohm", r_->resistance() * 1e3}, {"rating_a", rating_},
         {"i_max_a", current_.peak()}, {"i_rms_a", current_.rms()}, {"utilization", current_.peak() / rating_},
         {"drop_max_v", drop_.peak()}, {"drop_max_pct", drop_pct_}, {"temp_max_c", temp_max_},
         {"loss_j", loss_}, {"overload_s", over_time_}};
    if (!pin_names_.empty()) {
      w["pins"] = pin_names_;
    }
  }

 private:
  elec::Resistor* r_ = nullptr;
  double area_ = 1.0, length_ = 1.0, rho_ = kRhoCopper, alpha_ = kAlphaCopper, pin_r_ = 0.0;
  double rating_ = 15.0, rise_ = 50.0, temp_ = 25.0, ambient_ = 25.0;
  double heat_capacity_ = 1.0, thermal_resistance_ = 1.0;
  bool thermal_ = true, was_over_ = false;
  std::vector<std::array<double, 3>> route_;
  std::vector<std::string> pin_names_;
  Stat current_, drop_;
  double drop_pct_ = 0.0, temp_max_ = -kInf, loss_ = 0.0, over_time_ = 0.0;
  SignalId i_out_ = 0, drop_out_ = 0, temp_out_ = 0, overload_out_ = 0;
};

// Fuse: cold resistance, melting integral with cooling; opens when it blows.
class FuseDevice : public Device {
 public:
  FuseDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    rating_ = s.positive("rating_a");
    i2t_ = s.positive("i2t_a2s", 1.6 * rating_ * rating_);
    // Cooling so 1.1 x rating just reaches the melting integral.
    tau_ = i2t_ / std::pow(1.1 * rating_, 2.0);
    r_ = &n.circuit.emplace<elec::Resistor>(s.id, a, b, s.positive("r_ohm", 0.075 / rating_));
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
    blown_out_ = n.out(s.id, "blown", "-", "1 once the fuse has blown");
    heat_out_ = n.out(s.id, "heat", "-", "melting integral / I2t (blows at 1)");
  }
  bool takes(const std::string& f) const override { return f == "blow" || f == "open" || f == "contact_resistance"; }
  bool apply(const FaultEffect& f) override {
    const bool open = blown_ || f.open || f.blow;
    const bool changed = r_->open != open || r_->extra != f.extra;
    r_->open = open;
    r_->extra = f.extra;
    return changed;
  }
  bool update(Impl& n, double t, double h, const Voltages& v) override {
    if (blown_) {
      return false;
    }
    const double i = r_->current(v);
    heat_ += h * (i * i - heat_ / tau_);
    heat_ = std::max(heat_, 0.0);
    heat_max_ = std::max(heat_max_, heat_ / i2t_);
    if (heat_ >= i2t_) {
      blown_ = true;
      blown_at_ = t;
      r_->open = true;
      n.event(t, "fuse_blown", id, fmt(std::abs(i)) + " A through a " + fmt(rating_) + " A fuse");
      return true;
    }
    return false;
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    bus.set(i_out_, r_->current(v));
    bus.set(blown_out_, blown_ ? 1.0 : 0.0);
    bus.set(heat_out_, heat_ / i2t_);
  }
  void summary(Json& out) const override {
    out["fuses"][id] = {{"rating_a", rating_}, {"heat_max", heat_max_},
                        {"blown_at_s", blown_ ? Json(blown_at_) : Json(nullptr)}};
  }

 private:
  elec::Resistor* r_ = nullptr;
  double rating_ = 1.0, i2t_ = 1.0, tau_ = 1.0, heat_ = 0.0, heat_max_ = 0.0, blown_at_ = 0.0;
  bool blown_ = false;
  SignalId i_out_ = 0, blown_out_ = 0, heat_out_ = 0;
};

class CapacitorDevice : public Device {
 public:
  CapacitorDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    c_ = &n.circuit.emplace<elec::Capacitor>(s.id, a, b, s.positive("c_f"), s.num("esr_ohm", 0.0));
    if (c_->esr < 0.0) {
      s.fail("esr_ohm must not be negative");
    }
    v0_ = s.num("v0", 0.0);
    terminals = {{"a", a}, {"b", b}};
    v_out_ = n.out(s.id, "v", "V", "capacitor voltage");
    i_out_ = n.out(s.id, "i", "A", "charging current a -> b");
  }
  void start(Impl& /*n*/, const Voltages& /*v*/, bool active) override {
    if (!active) {
      c_->vc = v0_;
      c_->i = 0.0;
    }
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& /*v*/) override {
    bus.set(v_out_, c_->vc);
    bus.set(i_out_, c_->i);
  }

 private:
  elec::Capacitor* c_ = nullptr;
  double v0_ = 0.0;
  SignalId v_out_ = 0, i_out_ = 0;
};

class InductorDevice : public Device {
 public:
  InductorDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    l_ = &n.circuit.emplace<elec::Inductor>(s.id, a, b, s.positive("l_h"), s.num("r_ohm", 0.0));
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
  }
  void start(Impl& /*n*/, const Voltages& /*v*/, bool active) override {
    if (!active) {
      l_->i = 0.0;
      l_->u = 0.0;
    }
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& /*v*/) override { bus.set(i_out_, l_->i); }

 private:
  elec::Inductor* l_ = nullptr;
  SignalId i_out_ = 0;
};

class DiodeDevice : public Device {
 public:
  DiodeDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    d_ = &n.circuit.emplace<elec::Diode>(s.id, a, b, s.num("v_f", 0.7), s.positive("r_on_ohm", 0.01));
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "forward current");
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override { bus.set(i_out_, d_->current(v)); }

 private:
  elec::Diode* d_ = nullptr;
  SignalId i_out_ = 0;
};

// --- Sources -----------------------------------------------------------------------------------
class SourceDevice : public Device {
 public:
  SourceDevice(Impl& n, const Spec& s) {
    const NodeId pos = n.node(s, "pos"), neg = n.node(s, "neg");
    if (s.has("signal")) {
      signal_ = s.str("signal");
      scale_ = s.num("scale", 1.0);
    }
    src_ = &n.circuit.emplace<elec::VoltageSource>(s.id, pos, neg, signal_.empty() ? s.num("v") : 0.0,
                                                   s.positive("r_ohm", 0.001));
    terminals = {{"pos", pos}, {"neg", neg}};
    i_out_ = n.out(s.id, "i", "A", "current out of pos");
  }
  void resolve(Impl& n, const SignalBus& bus) override {
    if (!signal_.empty()) {
      id_ = bus.find(signal_);
      if (!id_ && !n.standalone) {
        throw std::invalid_argument("electrical.components (" + id + "): no signal '" + signal_ + "'");
      }
    }
  }
  void inputs(Impl& /*n*/, double /*t*/, const SignalBus& bus) override {
    if (id_) {
      src_->e = scale_ * bus.get(*id_);
    }
  }
  bool takes(const std::string& f) const override { return f == "open"; }
  bool apply(const FaultEffect& f) override {
    const bool changed = src_->open != f.open;
    src_->open = f.open;
    return changed;
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override { bus.set(i_out_, src_->current(v)); }

 private:
  elec::VoltageSource* src_ = nullptr;
  std::string signal_;
  std::optional<SignalId> id_;
  double scale_ = 1.0;
  SignalId i_out_ = 0;
};

class CurrentSourceDevice : public Device {
 public:
  CurrentSourceDevice(Impl& n, const Spec& s) {
    const NodeId pos = n.node(s, "pos"), neg = n.node(s, "neg");
    src_ = &n.circuit.emplace<elec::CurrentSource>(s.id, pos, neg, s.num("i_a"));
    terminals = {{"pos", pos}, {"neg", neg}};
  }

 private:
  elec::CurrentSource* src_ = nullptr;
};

// The car's accumulator: the plant's open-circuit voltage (cells, RC
// polarisation) behind the pack resistance. Gives the plant its current.
class BatteryDevice : public Device {
 public:
  BatteryDevice(Impl& n, const Spec& s, const ParamTree& params) {
    const NodeId pos = n.node(s, "pos"), neg = n.node(s, "neg");
    const BatteryParams battery = BatteryParams::from_params(params);
    const double soc = params.get_double("initial.soc", 0.9);
    open_ = battery.series * battery.ocv_cell(soc);
    src_ = &n.circuit.emplace<elec::VoltageSource>(s.id, pos, neg, open_, battery.pack_resistance());
    terminals = {{"pos", pos}, {"neg", neg}};
    open_in_ = n.bus->declare("bat.open_voltage", "V", "pack voltage behind its resistance (OCV less RC)");
    current_out_ = n.bus->declare("elec.bat.current", "A", "accumulator current from the electrical network");
    voltage_out_ = n.out("bat", "voltage", "V", "accumulator terminal voltage");
    i_out_ = n.out(s.id, "i", "A", "current out of pos");
  }
  void inputs(Impl& /*n*/, double /*t*/, const SignalBus& bus) override {
    const double open = bus.get(open_in_);
    src_->e = open > 0.0 ? open : open_;  // before the plant's first output: from the initial SoC
    sum_ = 0.0;
    samples_ = 0;
  }
  bool update(Impl& /*n*/, double /*t*/, double /*h*/, const Voltages& v) override {
    sum_ += src_->current(v);
    ++samples_;
    return false;
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    // Mean over the sub-steps: the charge the plant integrates.
    bus.set(current_out_, samples_ > 0 ? sum_ / samples_ : src_->current(v));
    bus.set(voltage_out_, v.across(src_->pos, src_->neg));
    bus.set(i_out_, src_->current(v));
  }
  double terminal(const Voltages& v) const { return v.across(src_->pos, src_->neg); }

 private:
  elec::VoltageSource* src_ = nullptr;
  double open_ = 0.0, sum_ = 0.0;
  int samples_ = 0;
  SignalId open_in_ = 0, current_out_ = 0, voltage_out_ = 0, i_out_ = 0;
};

// Low-voltage battery: open-circuit voltage from SoC behind a resistance.
class LvBatteryDevice : public Device {
 public:
  LvBatteryDevice(Impl& n, const Spec& s) {
    const NodeId pos = n.node(s, "pos"), neg = n.node(s, "neg");
    capacity_ = s.positive("capacity_ah", 5.0);
    // The scenario's starting charge (initial.lv_soc) wins over the netlist's.
    soc_ = s.p->get_double("initial.lv_soc", s.num("soc", 0.9));
    if (!(soc_ >= 0.0 && soc_ <= 1.0)) {
      s.fail("soc must be in [0, 1]");
    }
    if (s.has("ocv_soc")) {
      soc_table_ = s.p->get_double_list(s.key("ocv_soc"));
      v_table_ = s.p->get_double_list(s.key("ocv_v"));
    } else {
      soc_table_ = {0.0, 1.0};
      v_table_ = {s.num("v_empty", 12.0), s.num("v_full", 13.6)};
    }
    if (soc_table_.size() < 2 || soc_table_.size() != v_table_.size() ||
        !std::is_sorted(soc_table_.begin(), soc_table_.end())) {
      s.fail("ocv_soc (ascending) and ocv_v need the same length, at least 2");
    }
    src_ = &n.circuit.emplace<elec::VoltageSource>(s.id, pos, neg, ocv(), s.positive("r_ohm", 0.02));
    terminals = {{"pos", pos}, {"neg", neg}};
    i_out_ = n.out(s.id, "i", "A", "current out of pos (discharging)");
    v_out_ = n.out(s.id, "v", "V", "terminal voltage");
    soc_out_ = n.out(s.id, "soc", "-", "state of charge");
  }
  double ocv() const {
    if (soc_ <= soc_table_.front()) {
      return v_table_.front();
    }
    for (std::size_t i = 1; i < soc_table_.size(); ++i) {
      if (soc_ <= soc_table_[i]) {
        const double t = (soc_ - soc_table_[i - 1]) / (soc_table_[i] - soc_table_[i - 1]);
        return v_table_[i - 1] + t * (v_table_[i] - v_table_[i - 1]);
      }
    }
    return v_table_.back();
  }
  bool takes(const std::string& f) const override { return f == "open"; }
  bool apply(const FaultEffect& f) override {
    const bool changed = src_->open != f.open;
    src_->open = f.open;
    return changed;
  }
  bool update(Impl& /*n*/, double /*t*/, double h, const Voltages& v) override {
    if (!src_->open) {
      soc_ = std::clamp(soc_ - src_->current(v) * h / (3600.0 * capacity_), 0.0, 1.0);
      src_->e = ocv();
    }
    return false;
  }
  void outputs(Impl& n, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double i = src_->open ? 0.0 : src_->current(v);
    bus.set(i_out_, i);
    bus.set(v_out_, v.across(src_->pos, src_->neg));
    bus.set(soc_out_, soc_);
    if (n.recording) {
      current_.add(i);
    }
  }
  void summary(Json& out) const override {
    out["lv_batteries"][id] = {{"soc_end", soc_}, {"i_mean_a", current_.mean()}, {"i_max_a", current_.peak()}};
  }

 private:
  elec::VoltageSource* src_ = nullptr;
  double capacity_ = 5.0, soc_ = 0.9;
  std::vector<double> soc_table_, v_table_;
  Stat current_;
  SignalId i_out_ = 0, v_out_ = 0, soc_out_ = 0;
};

// --- Power electronics, loads, ECUs --------------------------------------------------------------
class EcuDevice;

// Inverter DC side: constant power from the plant's motors while enabled
// (DC link inside its undervoltage / overvoltage window, logic supply on).
class InverterDevice : public Device {
 public:
  InverterDevice(Impl& n, const Spec& s) {
    const NodeId pos = n.node(s, "pos"), neg = n.node(s, "neg");
    motors_ = s.list("motors");
    if (motors_.empty() && s.has("motor")) {
      motors_ = {s.str("motor")};
    }
    if (motors_.empty()) {
      s.fail("needs motors (the driveline motors it feeds, e.g. [fl, fr])");
    }
    v_min_ = s.num("v_min", 60.0);
    v_on_ = s.num("v_on", v_min_ + 10.0);
    v_max_ = s.num("v_max", kInf);
    logic_ = s.str("logic", "");
    p_idle_ = s.num("p_idle_w", 0.0);
    if (s.has("enable")) {
      enable_ = Trigger::parse(*s.p, s.key("enable"));  // e.g. precharge done ("DC on")
    }
    load_ = &n.circuit.emplace<elec::Load>(s.id, pos, neg, elec::Load::Law::Power, 0.0);
    load_->v_knee = std::max(1.0, 0.5 * v_min_);
    terminals = {{"pos", pos}, {"neg", neg}};
    for (const std::string& m : motors_) {
      if (std::find(n.inverter_motors.begin(), n.inverter_motors.end(), m) != n.inverter_motors.end()) {
        s.fail("motor " + m + " is fed by two inverters");
      }
      n.inverter_motors.push_back(m);
      Motor motor;
      motor.power_in = n.bus->declare("pt.motor." + m + ".power_elec", "W", "DC power drawn");
      motor.voltage_out = n.bus->declare("elec.motor." + m + ".dc_voltage", "V", "inverter DC-link voltage");
      motor.enabled_out = n.bus->declare("elec.motor." + m + ".enabled", "-", "1 while the inverter can drive");
      motor_signals_.push_back(motor);
    }
    v_out_ = n.out(s.id, "v", "V", "DC-link voltage");
    i_out_ = n.out(s.id, "i", "A", "DC current drawn");
    on_out_ = n.out(s.id, "on", "-", "1 while enabled");
  }
  void link(Impl& n);
  void resolve(Impl& n, const SignalBus& bus) override {
    if (enable_) {
      enable_->condition.resolve(bus, "electrical.components (" + id + ").enable", n.standalone);
    }
  }
  void inputs(Impl& /*n*/, double t, const SignalBus& bus) override {
    allowed_ = !enable_ || enable_->update(t, bus);
    demand_ = p_idle_;
    for (const Motor& m : motor_signals_) {
      demand_ += bus.get(m.power_in);
    }
    load_->value = enabled_ ? demand_ : 0.0;
  }
  bool update(Impl& n, double t, double /*h*/, const Voltages& v) override;
  void start(Impl& n, const Voltages& v, bool active) override;
  void outputs(Impl& n, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double u = v.across(load_->a, load_->b);
    for (const Motor& m : motor_signals_) {
      bus.set(m.voltage_out, u);
      bus.set(m.enabled_out, enabled_ ? 1.0 : 0.0);
    }
    bus.set(v_out_, u);
    bus.set(i_out_, load_->current(v));
    bus.set(on_out_, enabled_ ? 1.0 : 0.0);
    if (n.recording) {
      power_.add(u * load_->current(v));
    }
  }
  void summary(Json& out) const override {
    out["loads"][id] = {{"kind", "inverter"}, {"p_mean_w", power_.mean()}, {"p_max_w", power_.peak()}};
  }
  double voltage(const Voltages& v) const { return v.across(load_->a, load_->b); }

 private:
  struct Motor {
    SignalId power_in = 0, voltage_out = 0, enabled_out = 0;
  };
  elec::Load* load_ = nullptr;
  std::vector<std::string> motors_;
  std::vector<Motor> motor_signals_;
  std::string logic_;
  EcuDevice* logic_ecu_ = nullptr;
  std::optional<Trigger> enable_;
  bool allowed_ = true;
  double v_min_ = 60.0, v_on_ = 70.0, v_max_ = kInf, p_idle_ = 0.0, demand_ = 0.0;
  bool enabled_ = false;
  Stat power_;
  SignalId v_out_ = 0, i_out_ = 0, on_out_ = 0;
};

// ECU or controller board: a supply load (current or power), reset below
// v_min, restart after boot_s once the supply is back above v_on. While off
// its CAN node is silent and the signals it drives (`forces`) read 0.
class EcuDevice : public Device {
 public:
  EcuDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    const bool power = s.has("p_w");
    load_ = &n.circuit.emplace<elec::Load>(s.id, a, b, power ? elec::Load::Law::Power : elec::Load::Law::Current,
                                           power ? s.positive("p_w") : s.positive("i_a", 0.2));
    load_->v_knee = s.num("v_knee", 2.0);
    v_min_ = s.num("v_min", 7.0);
    v_on_ = s.num("v_on", v_min_ + 0.5);
    boot_ = s.num("boot_s", 0.2);
    if (!(v_on_ >= v_min_) || boot_ < 0.0) {
      s.fail("v_on must be at least v_min and boot_s not negative");
    }
    forces_ = s.list("forces");
    terminals = {{"a", a}, {"b", b}};
    on_out_ = n.out(s.id, "on", "-", "1 while running (0 in reset or booting)");
    v_out_ = n.out(s.id, "v", "V", "supply voltage");
    i_out_ = n.out(s.id, "i", "A", "supply current");
    resets_out_ = n.out(s.id, "resets", "-", "brown-out resets so far");
    for (const std::string& node : s.list("can_node")) {
      can_out_.push_back(n.bus->declare("elec.can_node." + node + ".on", "-", "1 while the CAN node has power"));
    }
  }
  bool on() const { return on_; }
  void start(Impl& /*n*/, const Voltages& v, bool active) override {
    on_ = active && v.across(load_->a, load_->b) >= v_min_;
    boot_since_ = -1.0;
  }
  bool update(Impl& n, double t, double /*h*/, const Voltages& v) override {
    const double u = v.across(load_->a, load_->b);
    if (n.recording) {
      v_min_seen_ = std::min(v_min_seen_, u);
    }
    if (on_) {
      if (u < v_min_) {
        on_ = false;
        ++resets_;
        boot_since_ = -1.0;
        n.event(t, "ecu_reset", id, "supply " + fmt(u) + " V below " + fmt(v_min_) + " V");
      }
    } else if (u >= v_on_) {
      if (boot_since_ < 0.0) {
        boot_since_ = t;
      }
      if (t - boot_since_ >= boot_ - 1e-9) {
        on_ = true;
      }
    } else {
      boot_since_ = -1.0;
    }
    return false;
  }
  void resolve_forces(SignalBus& bus) {
    force_ids_.clear();
    for (const std::string& pattern : forces_) {
      const std::vector<SignalId> found = bus.match(pattern);
      if (found.empty()) {
        throw std::invalid_argument("electrical.components (" + id + "): forces '" + pattern + "' matches no signal");
      }
      force_ids_.insert(force_ids_.end(), found.begin(), found.end());
    }
  }
  void gate(SignalBus& bus) const {
    if (!on_) {
      for (const SignalId s : force_ids_) {
        bus.set(s, 0.0);
      }
    }
  }
  void outputs(Impl& n, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double u = v.across(load_->a, load_->b);
    const double i = load_->current(v);
    bus.set(on_out_, on_ ? 1.0 : 0.0);
    bus.set(v_out_, u);
    bus.set(i_out_, i);
    bus.set(resets_out_, static_cast<double>(resets_));
    for (const SignalId s : can_out_) {
      bus.set(s, on_ ? 1.0 : 0.0);
    }
    if (n.recording) {
      power_.add(u * i);
      off_time_ += on_ ? 0.0 : n.dt;
    }
  }
  void summary(Json& out) const override {
    out["loads"][id] = {{"kind", "ecu"}, {"p_mean_w", power_.mean()}, {"p_max_w", power_.peak()}};
    out["ecus"][id] = {{"resets", resets_}, {"v_min_v", v_min_seen_}, {"off_s", off_time_}, {"v_reset_v", v_min_}};
  }

 private:
  elec::Load* load_ = nullptr;
  double v_min_ = 7.0, v_on_ = 7.5, boot_ = 0.2, boot_since_ = -1.0, v_min_seen_ = kInf, off_time_ = 0.0;
  bool on_ = true;
  std::int64_t resets_ = 0;
  std::vector<std::string> forces_;
  std::vector<SignalId> force_ids_, can_out_;
  Stat power_;
  SignalId on_out_ = 0, v_out_ = 0, i_out_ = 0, resets_out_ = 0;
};

void InverterDevice::link(Impl& n) {
  if (logic_.empty()) {
    return;
  }
  const auto it = n.by_id.find(logic_);
  logic_ecu_ = it == n.by_id.end() ? nullptr : dynamic_cast<EcuDevice*>(it->second);
  if (logic_ecu_ == nullptr) {
    throw std::invalid_argument("electrical.components (" + id + "): logic '" + logic_ + "' is not an ecu");
  }
}

void InverterDevice::start(Impl& /*n*/, const Voltages& v, bool active) {
  const double u = voltage(v);
  enabled_ = active && allowed_ && u >= v_min_ && u <= v_max_ && (logic_ecu_ == nullptr || logic_ecu_->on());
}

bool InverterDevice::update(Impl& /*n*/, double /*t*/, double /*h*/, const Voltages& v) {
  const double u = voltage(v);
  const bool logic = allowed_ && (logic_ecu_ == nullptr || logic_ecu_->on());
  const bool next = logic && u <= v_max_ && (enabled_ ? u >= v_min_ : u >= v_on_);
  if (next != enabled_) {
    enabled_ = next;
    load_->value = enabled_ ? demand_ : 0.0;
    return true;
  }
  return false;
}

// General load: pump, fan, light, heater; on by a condition, motor inrush.
class LoadDevice : public Device {
 public:
  LoadDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    elec::Load::Law law = elec::Load::Law::Resistance;
    double value = 0.0;
    if (s.has("p_w")) {
      law = elec::Load::Law::Power;
      value = s.positive("p_w");
    } else if (s.has("i_a")) {
      law = elec::Load::Law::Current;
      value = s.positive("i_a");
    } else {
      value = s.positive("r_ohm");
    }
    load_ = &n.circuit.emplace<elec::Load>(s.id, a, b, law, value);
    load_->v_knee = s.num("v_knee", 2.0);
    load_->v_nominal = s.positive("v_nominal", 12.0);
    if (s.has("inrush")) {
      load_->inrush = s.num("inrush.factor", 5.0);
      load_->inrush_tau = s.num("inrush.tau_s", 0.2);
      if (!(load_->inrush >= 1.0 && load_->inrush_tau > 0.0)) {
        s.fail("inrush needs factor >= 1 and a positive tau_s");
      }
    }
    if (s.has("control")) {
      trigger_ = Trigger::parse(*s.p, s.key("control"));
    }
    load_->on = !trigger_;
    terminals = {{"a", a}, {"b", b}};
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
    p_out_ = n.out(s.id, "p", "W", "power drawn");
    on_out_ = n.out(s.id, "on", "-", "1 while switched on");
  }
  void resolve(Impl& n, const SignalBus& bus) override {
    if (trigger_) {
      trigger_->condition.resolve(bus, "electrical.components (" + id + ").control", n.standalone);
    }
  }
  void inputs(Impl& n, double t, const SignalBus& bus) override {
    if (!trigger_) {
      return;
    }
    const bool want = trigger_->update(t, bus) && !disconnected_;
    if (want != load_->on) {
      if (want) {
        load_->switch_on(0.0);
        if (!n.recording) {
          // Settling the starting point: it was already running (no inrush).
          load_->emf = load_->v_nominal * (1.0 - 1.0 / std::max(load_->inrush, 1.0));
        }
      } else {
        load_->on = false;
      }
      n.discontinuity = true;
      n.circuit.invalidate();
    }
  }
  void start(Impl& /*n*/, const Voltages& v, bool active) override {
    if (active && load_->on && load_->inrush > 1.0) {
      load_->emf = std::max(0.0, v.across(load_->a, load_->b)) * (1.0 - 1.0 / load_->inrush);  // already running
    }
  }
  bool takes(const std::string& f) const override { return f == "open"; }
  bool apply(const FaultEffect& f) override {
    if (f.open == disconnected_) {
      return false;
    }
    disconnected_ = f.open;
    load_->on = !disconnected_ && (!trigger_ || trigger_->state);
    return true;
  }
  void outputs(Impl& n, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double i = load_->current(v);
    const double p = i * v.across(load_->a, load_->b);
    bus.set(i_out_, i);
    bus.set(p_out_, p);
    bus.set(on_out_, load_->on ? 1.0 : 0.0);
    if (n.recording) {
      power_.add(p);
      on_time_ += load_->on ? n.dt : 0.0;
    }
  }
  void summary(Json& out) const override {
    out["loads"][id] = {{"kind", "load"}, {"p_mean_w", power_.mean()}, {"p_max_w", power_.peak()},
                        {"on_s", on_time_}};
  }

 private:
  elec::Load* load_ = nullptr;
  std::optional<Trigger> trigger_;
  bool disconnected_ = false;
  Stat power_;
  double on_time_ = 0.0;
  SignalId i_out_ = 0, p_out_ = 0, on_out_ = 0;
};

// DC-DC converter with input undervoltage lockout and an optional enable.
class DcDcDevice : public Device {
 public:
  DcDcDevice(Impl& n, const Spec& s) {
    const NodeId in_p = n.node(s, "in_pos"), in_n = n.node(s, "in_neg");
    const NodeId out_p = n.node(s, "out_pos"), out_n = n.node(s, "out_neg");
    d_ = &n.circuit.emplace<elec::DcDc>(s.id, in_p, in_n, out_p, out_n, s.positive("v_out"));
    d_->efficiency = s.num("efficiency", 0.9);
    d_->r_out = s.positive("r_out_ohm", 0.02);
    d_->i_max = s.positive("i_max_a", 20.0);
    d_->response = s.positive("response_s", 0.002);
    d_->p_idle = s.num("p_idle_w", 2.0);
    v_in_min_ = s.num("v_in_min", 0.0);
    v_in_on_ = s.num("v_in_on", v_in_min_ * 1.05 + 1.0);
    d_->v_knee = std::max(1.0, 0.5 * v_in_min_);
    if (!(d_->efficiency > 0.0 && d_->efficiency <= 1.0)) {
      s.fail("efficiency must be in (0, 1]");
    }
    if (s.has("enable")) {
      enable_ = Trigger::parse(*s.p, s.key("enable"));
    }
    terminals = {{"in_pos", in_p}, {"in_neg", in_n}, {"out_pos", out_p}, {"out_neg", out_n}};
    on_out_ = n.out(s.id, "on", "-", "1 while converting");
    i_out_ = n.out(s.id, "i_out", "A", "output current");
    i_in_out_ = n.out(s.id, "i_in", "A", "input current");
    p_out_ = n.out(s.id, "p_out", "W", "output power");
    limited_out_ = n.out(s.id, "limited", "-", "1 at the output current limit");
  }
  void resolve(Impl& n, const SignalBus& bus) override {
    if (enable_) {
      enable_->condition.resolve(bus, "electrical.components (" + id + ").enable", n.standalone);
    }
  }
  void inputs(Impl& /*n*/, double t, const SignalBus& bus) override {
    enabled_ = (!enable_ || enable_->update(t, bus)) && !failed_;
  }
  bool takes(const std::string& f) const override { return f == "open"; }
  bool apply(const FaultEffect& f) override {
    failed_ = f.open;  // a dead converter: off at the next update
    return false;
  }
  void start(Impl& /*n*/, const Voltages& v, bool active) override {
    d_->on = active && enabled_ && v.across(d_->in_p, d_->in_n) >= v_in_min_;
    if (!d_->on) {
      d_->e = 0.0;
    }
  }
  bool update(Impl& n, double t, double /*h*/, const Voltages& v) override {
    const double u = v.across(d_->in_p, d_->in_n);
    const bool next = enabled_ && (d_->on ? u >= v_in_min_ : u >= v_in_on_);
    if (next != d_->on) {
      d_->turn(next, v);
      if (!next) {
        n.event(t, "dcdc_off", id, "input " + fmt(u) + " V");
      }
      return true;
    }
    return false;
  }
  void outputs(Impl& n, double /*t*/, SignalBus& bus, const Voltages& v) override {
    const double i = d_->output_current(v);
    const double p = std::max(0.0, i * v.across(d_->out_p, d_->out_n));
    const bool limited = d_->on && d_->mode == elec::DcDc::Mode::Limited;
    bus.set(on_out_, d_->on ? 1.0 : 0.0);
    bus.set(i_out_, i);
    bus.set(i_in_out_, d_->input_current(v));
    bus.set(p_out_, p);
    bus.set(limited_out_, limited ? 1.0 : 0.0);
    if (n.recording) {
      current_.add(i);
      power_.add(p);
      limited_time_ += limited ? n.dt : 0.0;
      off_time_ += d_->on ? 0.0 : n.dt;
    }
  }
  void summary(Json& out) const override {
    out["dcdc"][id] = {{"p_out_mean_w", power_.mean()}, {"p_out_max_w", power_.peak()},
                       {"i_out_max_a", current_.peak()}, {"i_max_a", d_->i_max},
                       {"margin", 1.0 - current_.peak() / d_->i_max}, {"limited_s", limited_time_},
                       {"off_s", off_time_}};
  }

 private:
  elec::DcDc* d_ = nullptr;
  std::optional<Trigger> enable_;
  bool enabled_ = true, failed_ = false;
  double v_in_min_ = 0.0, v_in_on_ = 1.0, limited_time_ = 0.0, off_time_ = 0.0;
  Stat current_, power_;
  SignalId on_out_ = 0, i_out_ = 0, i_in_out_ = 0, p_out_ = 0, limited_out_ = 0;
};

class RegulatorDevice : public Device {
 public:
  RegulatorDevice(Impl& n, const Spec& s) {
    const NodeId in = n.node(s, "in"), out = n.node(s, "out"), gnd = n.node(s, "gnd");
    reg_ = &n.circuit.emplace<elec::Regulator>(s.id, in, out, gnd, s.positive("v_out", 5.0));
    reg_->dropout = s.num("dropout_v", 1.0);
    reg_->r_out = s.positive("r_out_ohm", 0.05);
    reg_->i_max = s.positive("i_max_a", 0.5);
    reg_->i_q = s.num("i_q_a", 0.002);
    terminals = {{"in", in}, {"out", out}, {"gnd", gnd}};
    v_out_ = n.out(s.id, "v", "V", "output voltage");
    i_out_ = n.out(s.id, "i", "A", "output current");
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    bus.set(v_out_, v.across(reg_->out, reg_->gnd));
    bus.set(i_out_, reg_->current(v));
  }

 private:
  elec::Regulator* reg_ = nullptr;
  SignalId v_out_ = 0, i_out_ = 0;
};

// --- Switching: switches, relays, contactors, precharge controller ---------------------------
// Switch: fixed, driven by a condition (`closed_when`), or opened by one
// (`open_when`, e.g. a BSPD, BMS or inertia switch in the shutdown circuit).
class SwitchDevice : public Device {
 public:
  SwitchDevice(Impl& n, const Spec& s) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    sw_ = &n.circuit.emplace<elec::Switch>(s.id, a, b, s.positive("r_on_ohm", 0.01));
    base_ = s.flag("closed", true);
    if (s.has("closed_when")) {
      trigger_ = Trigger::parse(*s.p, s.key("closed_when"));
      closes_ = true;
    } else if (s.has("open_when")) {
      trigger_ = Trigger::parse(*s.p, s.key("open_when"));
      closes_ = false;
    }
    sw_->closed = trigger_ ? !closes_ : base_;
    terminals = {{"a", a}, {"b", b}};
    closed_out_ = n.out(s.id, "closed", "-", "1 while closed");
    i_out_ = n.out(s.id, "i", "A", "current a -> b");
  }
  void resolve(Impl& n, const SignalBus& bus) override {
    if (trigger_) {
      trigger_->condition.resolve(bus, "electrical.components (" + id + ")", n.standalone);
    }
  }
  void inputs(Impl& n, double t, const SignalBus& bus) override {
    if (trigger_) {
      const bool hit = trigger_->update(t, bus);
      want_ = closes_ ? hit : !hit;
      if (!closes_ && hit && !reported_) {
        n.event(t, "switch_open", id, "opened by its condition");
        reported_ = true;
      }
      if (!hit) {
        reported_ = false;
      }
    } else {
      want_ = base_;
    }
    set(n);
  }
  bool takes(const std::string& f) const override { return f == "open" || f == "stuck_closed" || f == "stuck_open"; }
  bool apply(const FaultEffect& f) override {
    stuck_ = f.open ? -1 : f.stuck;
    return false;  // takes effect in inputs()
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    bus.set(closed_out_, sw_->closed ? 1.0 : 0.0);
    bus.set(i_out_, sw_->current(v));
  }

 private:
  void set(Impl& n) {
    const bool closed = stuck_ != 0 ? stuck_ > 0 : want_;
    if (closed != sw_->closed) {
      sw_->closed = closed;
      n.discontinuity = true;
      n.circuit.invalidate();
    }
  }
  elec::Switch* sw_ = nullptr;
  std::optional<Trigger> trigger_;
  bool closes_ = true, base_ = true, want_ = true, reported_ = false;
  int stuck_ = 0;
  SignalId closed_out_ = 0, i_out_ = 0;
};

// Relay or contactor: the coil is a resistance in the circuit; the contact
// pulls in above pull_in_v (after close_ms) and drops out below drop_out_v
// (after open_ms). A sagging coil supply makes it chatter (PH-16).
class RelayDevice : public Device {
 public:
  RelayDevice(Impl& n, const Spec& s, bool contactor) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b");
    const NodeId ca = n.node(s, "coil_a"), cb = n.node(s, "coil_b");
    coil_ = &n.circuit.emplace<elec::Resistor>(s.id + ".coil", ca, cb, s.positive("coil_r_ohm", contactor ? 24.0 : 90.0));
    contact_ = &n.circuit.emplace<elec::Switch>(s.id, a, b, s.positive("r_on_ohm", contactor ? 3e-4 : 0.01));
    pull_in_ = s.positive("pull_in_v", contactor ? 9.0 : 8.0);
    drop_out_ = s.num("drop_out_v", contactor ? 4.0 : 3.0);
    close_delay_ = s.num("close_ms", contactor ? 25.0 : 10.0) * 1e-3;
    open_delay_ = s.num("open_ms", contactor ? 10.0 : 5.0) * 1e-3;
    normally_closed_ = s.str("normally", "open") == "closed";
    rating_ = s.num("rating_a", contactor ? 200.0 : 30.0);
    chatter_count_ = static_cast<int>(s.num("chatter_count", 3));
    chatter_window_ = s.num("chatter_window_s", 1.0);
    if (!(drop_out_ < pull_in_) || drop_out_ < 0.0) {
      s.fail("drop_out_v must be below pull_in_v");
    }
    if (s.str("normally", "open") != "open" && !normally_closed_) {
      s.fail("normally must be open or closed");
    }
    contact_->closed = normally_closed_;
    terminals = {{"a", a}, {"b", b}, {"coil_a", ca}, {"coil_b", cb}};
    closed_out_ = n.out(s.id, "closed", "-", "1 while the contact is closed");
    coil_out_ = n.out(s.id, "coil_v", "V", "coil voltage");
    i_out_ = n.out(s.id, "i", "A", "contact current a -> b");
    closes_out_ = n.out(s.id, "closes", "-", "contact closings so far");
    chatter_out_ = n.out(s.id, "chatter", "-", "1 while it has closed chatter_count times within chatter_window_s");
  }
  bool closed() const { return contact_->closed; }
  std::int64_t closings() const { return closes_; }
  void start(Impl& /*n*/, const Voltages& v, bool active) override {
    energized_ = active && std::abs(v.across(coil_->a, coil_->b)) >= pull_in_;
    contact_->closed = stuck_ != 0 ? stuck_ > 0 : energized_ != normally_closed_;
    since_ = -kInf;
  }
  bool takes(const std::string& f) const override { return f == "stuck_closed" || f == "stuck_open" || f == "open"; }
  bool apply(const FaultEffect& f) override {
    stuck_ = f.open ? -1 : f.stuck;
    const bool closed = stuck_ != 0 ? stuck_ > 0 : energized_ != normally_closed_;
    if (stuck_ != 0 && closed != contact_->closed) {
      contact_->closed = closed;
      return true;
    }
    return false;
  }
  bool update(Impl& n, double t, double /*h*/, const Voltages& v) override {
    const double coil = std::abs(v.across(coil_->a, coil_->b));
    coil_v_ = coil;
    if (!energized_ && coil >= pull_in_) {
      energized_ = true;
      since_ = t;
    } else if (energized_ && coil <= drop_out_) {
      energized_ = false;
      since_ = t;
    }
    if (stuck_ != 0) {
      return false;
    }
    const bool target = energized_ != normally_closed_;
    if (target == contact_->closed) {
      return false;
    }
    const double delay = target ? close_delay_ : open_delay_;
    if (t - since_ < delay - 1e-9) {
      return false;
    }
    contact_->closed = target;
    if (target) {
      ++closes_;
      last_close_dv_ = v.across(contact_->a, contact_->b);
      recent_.push_back(t);
      while (!recent_.empty() && recent_.front() < t - chatter_window_) {
        recent_.pop_front();
      }
      if (static_cast<int>(recent_.size()) >= chatter_count_ && !chattering_) {
        chattering_ = true;
        ++chatter_episodes_;
        n.event(t, "contactor_chatter", id,
                fmt(static_cast<double>(recent_.size())) + " closings within " + fmt(chatter_window_) +
                    " s (coil " + fmt(coil) + " V)");
      }
    }
    return true;
  }
  void outputs(Impl& n, double t, SignalBus& bus, const Voltages& v) override {
    while (!recent_.empty() && recent_.front() < t - chatter_window_) {
      recent_.pop_front();
    }
    if (static_cast<int>(recent_.size()) < chatter_count_) {
      chattering_ = false;
    }
    const double i = contact_->current(v);
    bus.set(closed_out_, contact_->closed ? 1.0 : 0.0);
    bus.set(coil_out_, coil_v_);
    bus.set(i_out_, i);
    bus.set(closes_out_, static_cast<double>(closes_));
    bus.set(chatter_out_, chattering_ ? 1.0 : 0.0);
    if (n.recording) {
      current_.add(std::abs(i));
    }
  }
  void summary(Json& out) const override {
    out["relays"][id] = {{"closes", closes_}, {"chatter_episodes", chatter_episodes_}, {"i_max_a", current_.peak()},
                         {"rating_a", rating_}};
  }
  double last_close_dv() const { return last_close_dv_; }

 private:
  elec::Resistor* coil_ = nullptr;
  elec::Switch* contact_ = nullptr;
  double pull_in_ = 8.0, drop_out_ = 3.0, close_delay_ = 0.01, open_delay_ = 0.005, rating_ = 30.0;
  double since_ = -kInf, coil_v_ = 0.0, chatter_window_ = 1.0, last_close_dv_ = 0.0;
  bool energized_ = false, normally_closed_ = false, chattering_ = false;
  int stuck_ = 0, chatter_count_ = 3;
  std::int64_t closes_ = 0, chatter_episodes_ = 0;
  std::deque<double> recent_;
  Stat current_;
  SignalId closed_out_ = 0, coil_out_ = 0, i_out_ = 0, closes_out_ = 0, chatter_out_ = 0;
};

// Precharge controller (design doc §12.3 b): powered from the shutdown
// circuit, closes the precharge output, then the AIR output once the DC link
// reaches `threshold` of the pack voltage (or after time_s in time mode),
// then opens the precharge output. Times out into a latched fault. With
// `air_relay` it checks the ratio at the moment that contactor closes.
class PrechargeDevice : public Device {
 public:
  enum State { Off = 0, Precharging = 1, Closing = 2, Done = 3, Failed = 4 };

  PrechargeDevice(Impl& n, const Spec& s) {
    supply_ = n.node(s, "supply");
    ref_ = s.has("ref") ? n.node(s, "ref") : kGround;
    const std::vector<std::string> pack = s.list("pack"), link = s.list("link");
    if (pack.size() != 2 || link.size() != 2) {
      s.fail("pack and link are [positive net, negative net]");
    }
    pack_ = {n.circuit.node(pack[0]), n.circuit.node(pack[1])};
    link_ = {n.circuit.node(link[0]), n.circuit.node(link[1])};
    const std::vector<std::string> pre = s.list("pre_out"), air = s.list("air_out");
    if (pre.size() != 2 || air.size() != 2) {
      s.fail("pre_out and air_out are [net, net] (the controller's switched outputs)");
    }
    pre_ = &n.circuit.emplace<elec::Switch>(s.id + ".pre", n.circuit.node(pre[0]), n.circuit.node(pre[1]),
                                            s.positive("r_on_ohm", 0.05));
    air_ = &n.circuit.emplace<elec::Switch>(s.id + ".air", n.circuit.node(air[0]), n.circuit.node(air[1]),
                                            s.positive("r_on_ohm", 0.05));
    v_on_ = s.num("v_on", 8.0);
    mode_time_ = s.str("mode", "voltage") == "time";
    if (!mode_time_ && s.str("mode", "voltage") != "voltage") {
      s.fail("mode must be voltage or time");
    }
    threshold_ = s.num("threshold", 0.95);
    rule_ = s.num("rule_ratio", 0.9);
    time_ = s.num("time_s", 2.0);
    min_time_ = s.num("min_time_s", 0.0);
    timeout_ = s.num("timeout_s", 5.0);
    overlap_ = s.num("overlap_s", 0.05);
    air_relay_name_ = s.str("air_relay", "");
    terminals = {{"supply", supply_}};
    state_out_ = n.out(s.id, "state", "-", "0 off, 1 precharging, 2 closing, 3 done, 4 failed");
    ratio_out_ = n.out(s.id, "ratio", "-", "DC-link voltage / pack voltage");
    close_ratio_out_ = n.out(s.id, "close_ratio", "-", "ratio when the AIR closed");
    violation_out_ = n.out(s.id, "violation", "-", "1 once the AIR closed below rule_ratio");
  }
  void link(Impl& n) {
    if (air_relay_name_.empty()) {
      return;
    }
    const auto it = n.by_id.find(air_relay_name_);
    air_relay_ = it == n.by_id.end() ? nullptr : dynamic_cast<RelayDevice*>(it->second);
    if (air_relay_ == nullptr) {
      throw std::invalid_argument("electrical.components (" + id + "): air_relay '" + air_relay_name_ +
                                  "' is not a relay or contactor");
    }
  }
  bool controller() const override { return true; }
  double ratio(const Voltages& v) const {
    const double pack = v.across(pack_[0], pack_[1]);
    return pack > 1.0 ? v.across(link_[0], link_[1]) / pack : 0.0;
  }
  void start(Impl& /*n*/, const Voltages& v, bool active) override {
    const bool powered = v.across(supply_, ref_) >= v_on_;
    state_ = active && powered ? Done : Off;
    pre_->closed = false;
    air_->closed = state_ == Done;
    seen_closes_ = air_relay_ != nullptr ? air_relay_->closings() : 0;
  }
  bool update(Impl& n, double t, double /*h*/, const Voltages& v) override {
    const bool before_pre = pre_->closed, before_air = air_->closed;
    const double r = ratio(v);
    ratio_ = r;
    if (air_relay_ != nullptr && air_relay_->closings() != seen_closes_) {
      seen_closes_ = air_relay_->closings();
      close_ratio_ = r;
      if (r < rule_) {
        violation_ = true;
        n.event(t, "precharge_violation", id,
                "AIR closed at " + fmt(100.0 * r) + " % of the pack voltage (rule " + fmt(100.0 * rule_) + " %)");
      }
    }
    const bool powered = v.across(supply_, ref_) >= v_on_;
    if (!powered) {
      state_ = Off;
    } else {
      switch (state_) {
        case Off:
          state_ = Precharging;
          t0_ = t;
          break;
        case Precharging: {
          const bool ready = mode_time_ ? t - t0_ >= time_ - 1e-9 : r >= threshold_ && t - t0_ >= min_time_ - 1e-9;
          if (ready) {
            state_ = Closing;
            t1_ = t;
            done_time_ = t - t0_;
          } else if (t - t0_ > timeout_) {
            state_ = Failed;
            n.event(t, "precharge_fail", id,
                    "DC link at " + fmt(100.0 * r) + " % after " + fmt(timeout_) + " s");
          }
          break;
        }
        case Closing:
          if (t - t1_ >= overlap_ - 1e-9) {
            state_ = Done;
          }
          break;
        case Done:
        case Failed:
          break;
      }
    }
    pre_->closed = state_ == Precharging || state_ == Closing;
    air_->closed = state_ == Closing || state_ == Done;
    return pre_->closed != before_pre || air_->closed != before_air;
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& /*v*/) override {
    bus.set(state_out_, static_cast<double>(state_));
    bus.set(ratio_out_, ratio_);
    bus.set(close_ratio_out_, close_ratio_);
    bus.set(violation_out_, violation_ ? 1.0 : 0.0);
  }
  void summary(Json& out) const override {
    out["precharge"][id] = {{"state", state_}, {"precharge_s", done_time_ >= 0.0 ? Json(done_time_) : Json(nullptr)},
                            {"close_ratio", close_ratio_ >= 0.0 ? Json(close_ratio_) : Json(nullptr)},
                            {"violation", violation_}};
  }

 private:
  NodeId supply_ = kGround, ref_ = kGround;
  std::array<NodeId, 2> pack_{}, link_{};
  elec::Switch* pre_ = nullptr;
  elec::Switch* air_ = nullptr;
  std::string air_relay_name_;
  RelayDevice* air_relay_ = nullptr;
  std::int64_t seen_closes_ = 0;
  double v_on_ = 8.0, threshold_ = 0.95, rule_ = 0.9, time_ = 2.0, min_time_ = 0.0, timeout_ = 5.0, overlap_ = 0.05;
  bool mode_time_ = false, violation_ = false;
  State state_ = Off;
  double t0_ = 0.0, t1_ = 0.0, ratio_ = 0.0, close_ratio_ = -1.0, done_time_ = -1.0;
  SignalId state_out_ = 0, ratio_out_ = 0, close_ratio_out_ = 0, violation_out_ = 0;
};

// --- Sensor wiring -----------------------------------------------------------------------------
// Potentiometer track of an analog sensor: the wiper follows a pedal (from
// the sensor's own definition) or any signal.
class PotDevice : public Device {
 public:
  PotDevice(Impl& n, const Spec& s, const ParamTree& params) {
    const NodeId a = n.node(s, "a"), b = n.node(s, "b"), w = n.node(s, "wiper");
    pot_ = &n.circuit.emplace<elec::Potentiometer>(s.id, a, b, w, s.positive("r_ohm", 5000.0));
    if (s.has("sensor")) {
      // The pedal sensor's voltage map at its 5 V supply, as a wiper ratio.
      const std::string sensor = s.str("sensor"), channel = s.str("channel", "ch1");
      bool found = false;
      for (std::size_t i = 0; params.has_prefix("vehicle.sensors." + std::to_string(i)); ++i) {
        const std::string p = "vehicle.sensors." + std::to_string(i);
        if (params.get_string(p + ".name", "") != sensor) {
          continue;
        }
        if (params.get_string(p + ".type", "") != "pedal") {
          s.fail("sensor '" + sensor + "' is not a pedal sensor");
        }
        signal_ = "drv.cmd." + params.get_string(p + ".pedal", "throttle");
        const std::vector<double> map = params.get_double_list(p + "." + channel + "_v");
        const std::array<double, 2> volts = map.size() == 2 ? std::array<double, 2>{map[0], map[1]}
                                                            : (channel == "ch2" ? std::array<double, 2>{4.5, 0.5}
                                                                                : std::array<double, 2>{0.5, 4.5});
        ratio_ = {volts[0] / 5.0, volts[1] / 5.0};
        found = true;
      }
      if (!found) {
        s.fail("no sensor '" + sensor + "' in vehicle.sensors");
      }
    } else {
      signal_ = s.str("signal");
      const std::vector<double> r = s.p->get_double_list(s.key("ratio"));
      ratio_ = r.size() == 2 ? std::array<double, 2>{r[0], r[1]} : std::array<double, 2>{0.0, 1.0};
    }
    terminals = {{"a", a}, {"b", b}, {"wiper", w}};
  }
  void resolve(Impl& n, const SignalBus& bus) override {
    id_ = bus.find(signal_);
    if (!id_ && !n.standalone) {
      throw std::invalid_argument("electrical.components (" + id + "): no signal '" + signal_ + "'");
    }
  }
  void inputs(Impl& /*n*/, double /*t*/, const SignalBus& bus) override {
    const double travel = id_ ? std::clamp(bus.get(*id_), 0.0, 1.0) : 0.0;
    pot_->x = ratio_[0] + (ratio_[1] - ratio_[0]) * travel;  // wiper voltage / supply (r x from wiper to b)
  }

 private:
  elec::Potentiometer* pot_ = nullptr;
  std::string signal_;
  std::optional<SignalId> id_;
  std::array<double, 2> ratio_{0.0, 1.0};
};

// ADC input of an ECU: input resistance and pull resistor to its reference;
// ratiometric when vref is the sensor supply. Feeds a sensor channel
// (elec.sensor.<sensor>.<channel>, read by the sensor chain before its ADC).
class AdcDevice : public Device {
 public:
  AdcDevice(Impl& n, const Spec& s) {
    in_ = n.node(s, "a");
    ref_ = n.node(s, "ref");
    n.circuit.emplace<elec::Resistor>(s.id + ".rin", in_, ref_, s.positive("r_in_ohm", 1e6));
    if (s.has("pull_down_ohm")) {
      n.circuit.emplace<elec::Resistor>(s.id + ".pull", in_, ref_, s.positive("pull_down_ohm"));
    }
    if (s.has("pull_up")) {
      n.circuit.emplace<elec::Resistor>(s.id + ".pull", in_, n.circuit.node(s.str("pull_up.to")),
                                        s.positive("pull_up.r_ohm"));
    }
    if (s.has("vref")) {
      vref_ = n.circuit.node(s.str("vref"));
      v_nominal_ = s.positive("v_ref_nominal", 5.0);
    }
    terminals = {{"a", in_}, {"ref", ref_}};
    if (s.has("sensor")) {
      out_ = n.bus->declare("elec.sensor." + s.str("sensor") + "." + s.str("channel", "ch1"), "V",
                            "sensor voltage at the ECU input (through the wiring)");
    } else {
      out_ = n.out(s.id, "v", "V", "voltage at the ADC input");
    }
  }
  void outputs(Impl& /*n*/, double /*t*/, SignalBus& bus, const Voltages& v) override {
    double u = v.across(in_, ref_);
    if (vref_) {
      u *= v_nominal_ / std::max(v.across(*vref_, ref_), 0.1);
    }
    bus.set(out_, u);
  }

 private:
  NodeId in_ = kGround, ref_ = kGround;
  std::optional<NodeId> vref_;
  double v_nominal_ = 5.0;
  SignalId out_ = 0;
};

const std::vector<std::string> kReserved = {"bat", "motor", "sensor", "can_node", "v", "ts_voltage", "ts_active",
                                            "solver"};

}  // namespace

// --- Building the network ------------------------------------------------------------------------
void ElectricalNetwork::Impl::build(const ParamTree& params) {
  circuit.set_ground(params.has_prefix("electrical.ground") ? params.get_string_list("electrical.ground")
                                                            : std::vector<std::string>{"GND", "0"});
  contact_r = params.get_double("electrical.harness.contact_r_ohm", contact_r);
  ambient = params.get_double("environment.ambient_temp_c", 25.0);
  std::vector<InverterDevice*> inverters;
  std::vector<PrechargeDevice*> precharges;
  for (std::size_t i = 0;; ++i) {
    const std::string prefix = "electrical.components." + std::to_string(i);
    if (!params.has_prefix(prefix)) {
      break;
    }
    Spec s;
    s.p = &params;
    s.prefix = prefix;
    s.id = params.get_string(prefix + ".id", "");
    s.type = params.get_string(prefix + ".type", "");
    if (!is_identifier(s.id)) {
      throw std::invalid_argument(prefix + ".id '" + s.id + "': a component id is letters, digits and _");
    }
    if (std::find(kReserved.begin(), kReserved.end(), s.id) != kReserved.end()) {
      throw std::invalid_argument(prefix + ".id '" + s.id + "' is reserved for network signals");
    }
    if (by_id.count(s.id) != 0) {
      throw std::invalid_argument(prefix + ".id '" + s.id + "' is used twice");
    }
    std::unique_ptr<Device> d;
    const std::string& t = s.type;
    if (t == "resistor") {
      d = std::make_unique<ResistorDevice>(*this, s);
    } else if (t == "wire") {
      d = std::make_unique<WireDevice>(*this, s);
    } else if (t == "fuse") {
      d = std::make_unique<FuseDevice>(*this, s);
    } else if (t == "capacitor") {
      d = std::make_unique<CapacitorDevice>(*this, s);
    } else if (t == "inductor") {
      d = std::make_unique<InductorDevice>(*this, s);
    } else if (t == "diode") {
      d = std::make_unique<DiodeDevice>(*this, s);
    } else if (t == "voltage_source") {
      d = std::make_unique<SourceDevice>(*this, s);
    } else if (t == "current_source") {
      d = std::make_unique<CurrentSourceDevice>(*this, s);
    } else if (t == "battery_model") {
      if (carries_battery) {
        s.fail("only one battery_model (the accumulator)");
      }
      carries_battery = true;
      d = std::make_unique<BatteryDevice>(*this, s, params);
    } else if (t == "lv_battery") {
      d = std::make_unique<LvBatteryDevice>(*this, s);
    } else if (t == "inverter") {
      auto inv = std::make_unique<InverterDevice>(*this, s);
      inverters.push_back(inv.get());
      d = std::move(inv);
    } else if (t == "dcdc") {
      d = std::make_unique<DcDcDevice>(*this, s);
    } else if (t == "regulator") {
      d = std::make_unique<RegulatorDevice>(*this, s);
    } else if (t == "load") {
      d = std::make_unique<LoadDevice>(*this, s);
    } else if (t == "ecu") {
      d = std::make_unique<EcuDevice>(*this, s);
    } else if (t == "switch") {
      d = std::make_unique<SwitchDevice>(*this, s);
    } else if (t == "relay" || t == "contactor") {
      d = std::make_unique<RelayDevice>(*this, s, t == "contactor");
    } else if (t == "precharge") {
      auto pre = std::make_unique<PrechargeDevice>(*this, s);
      precharges.push_back(pre.get());
      d = std::move(pre);
    } else if (t == "potentiometer") {
      d = std::make_unique<PotDevice>(*this, s, params);
    } else if (t == "adc") {
      d = std::make_unique<AdcDevice>(*this, s);
    } else {
      throw std::invalid_argument(prefix + ".type '" + t +
                                  "' (expected resistor, wire, fuse, capacitor, inductor, diode, voltage_source, "
                                  "current_source, battery_model, lv_battery, inverter, dcdc, regulator, load, ecu, "
                                  "switch, relay, contactor, precharge, potentiometer or adc)");
    }
    d->id = s.id;
    d->type = t;
    by_id[s.id] = d.get();
    devices.push_back(std::move(d));
  }
  if (devices.empty()) {
    throw std::invalid_argument("electrical.components: the netlist is empty");
  }
  for (InverterDevice* inv : inverters) {
    inv->link(*this);
  }
  for (PrechargeDevice* pre : precharges) {
    pre->link(*this);
  }
  if (carries_battery && inverters.empty()) {
    throw std::invalid_argument("electrical: a battery_model needs inverters to feed the motors");
  }

  // Faults.
  for (std::size_t i = 0;; ++i) {
    const std::string p = "electrical.faults." + std::to_string(i);
    if (!params.has_prefix(p)) {
      break;
    }
    Fault f;
    f.where = p;
    f.type = params.get_string(p + ".type");
    f.target = params.get_string(p + ".target");
    f.at = params.get_double(p + ".at_s", 0.0);
    f.until = params.get_double(p + ".until_s", kInf);
    f.r = params.get_double(p + ".r_ohm", f.type == "short" ? 0.001 : 0.0);
    if (params.has_prefix(p + ".intermittent")) {
      f.rate = params.get_double(p + ".intermittent.rate_hz");
      f.duration = params.get_double(p + ".intermittent.duration_s");
      if (!(f.rate > 0.0 && f.duration > 0.0)) {
        throw std::invalid_argument(p + ".intermittent: rate_hz and duration_s must be positive");
      }
    }
    static const std::set<std::string> types = {"open", "short", "contact_resistance", "stuck_closed", "stuck_open",
                                                "blow"};
    if (types.count(f.type) == 0) {
      throw std::invalid_argument(p + ".type '" + f.type +
                                  "' (expected open, short, contact_resistance, stuck_closed, stuck_open or blow)");
    }
    if (f.type == "short") {
      // A node (net name) or a terminal "<id>.<terminal>", shorted to `to` (ground by default).
      NodeId from = kGround;
      const auto dot = f.target.find('.');
      const auto dev = dot == std::string::npos ? by_id.end() : by_id.find(f.target.substr(0, dot));
      if (dev != by_id.end()) {
        const auto term = dev->second->terminals.find(f.target.substr(dot + 1));
        if (term == dev->second->terminals.end()) {
          throw std::invalid_argument(p + ".target '" + f.target + "': no such terminal");
        }
        from = term->second;
      } else if (const auto node_id = circuit.find(f.target)) {
        from = *node_id;
      } else {
        throw std::invalid_argument(p + ".target '" + f.target + "': no net or component terminal of that name");
      }
      const std::string to_name = params.get_string(p + ".to", "GND");
      const auto to = circuit.find(to_name);
      if (!to) {
        throw std::invalid_argument(p + ".to '" + to_name + "': no such net");
      }
      if (!(f.r > 0.0)) {
        throw std::invalid_argument(p + ".r_ohm must be positive");
      }
      f.short_element = &circuit.emplace<elec::Resistor>("short." + std::to_string(i), from, *to, f.r);
      f.short_element->open = true;
    } else if (f.target.rfind("conn.", 0) == 0) {
      if (pins.count(f.target.substr(5)) == 0) {
        throw std::invalid_argument(p + ".target '" + f.target + "': no wire lists pin " + f.target.substr(5));
      }
      if (f.type != "open" && f.type != "contact_resistance") {
        throw std::invalid_argument(p + ": connector pins take open or contact_resistance");
      }
    } else {
      const auto it = by_id.find(f.target);
      if (it == by_id.end()) {
        throw std::invalid_argument(p + ".target '" + f.target + "': no such component");
      }
      if (!it->second->takes(f.type)) {
        throw std::invalid_argument(p + ": a " + it->second->type + " does not take " + f.type + " faults");
      }
    }
    if (f.type == "contact_resistance" && !(f.r > 0.0)) {
      throw std::invalid_argument(p + ".r_ohm must be positive");
    }
    faults.push_back(f);
  }

  // Node voltages as signals.
  for (const std::string& name : circuit.node_names()) {
    node_out.push_back(bus->declare("elec.v." + ElectricalNetwork::signal_name(name), "V", "node voltage to ground"));
  }
  ts_voltage_out = bus->declare("elec.ts_voltage", "V", "highest inverter DC-link voltage");
  ts_active_out = bus->declare("elec.ts_active", "-", "1 while the tractive system is above 60 V");
}

void ElectricalNetwork::Impl::apply_faults(double t) {
  std::map<Device*, FaultEffect> effects;
  for (auto& d : devices) {
    effects[d.get()] = FaultEffect{};
  }
  for (Fault& f : faults) {
    bool active = t >= f.at - kEps && t < f.until;
    if (active && f.rate > 0.0) {
      if (t >= f.outage_until && rng->uniform() < f.rate * dt) {
        f.outage_until = t + f.duration;
      }
      active = t < f.outage_until;
    }
    if (f.short_element != nullptr) {
      if (f.short_element->open == active) {
        f.short_element->open = !active;
        discontinuity = true;
        circuit.invalidate();
      }
      continue;
    }
    if (!active) {
      continue;
    }
    std::vector<Device*> targets;
    if (f.target.rfind("conn.", 0) == 0) {
      targets = pins[f.target.substr(5)];
    } else {
      targets = {by_id[f.target]};
    }
    for (Device* d : targets) {
      FaultEffect& e = effects[d];
      if (f.type == "open") {
        e.open = true;
      } else if (f.type == "contact_resistance") {
        e.extra += f.r;
      } else if (f.type == "stuck_closed") {
        e.stuck = 1;
      } else if (f.type == "stuck_open") {
        e.stuck = -1;
      } else if (f.type == "blow") {
        e.blow = true;
      }
    }
  }
  for (auto& [d, e] : effects) {
    if (d->apply(e)) {
      discontinuity = true;
      circuit.invalidate();
    }
  }
}

void ElectricalNetwork::Impl::start(SignalBus& bus_in) {
  for (auto& d : devices) {
    d->resolve(*this, bus_in);
  }
  apply_faults(0.0);
  for (auto& d : devices) {
    d->inputs(*this, 0.0, bus_in);
  }
  if (active_start) {
    // Contacts, controllers, ECUs and switched loads settle where the
    // operating point puts them (a few rounds: a relay closing can power
    // another one's coil, a pump can follow elec.ts_active).
    recording = false;
    for (int round = 0; round < 12; ++round) {
      circuit.operating_point(0.0);
      const std::vector<double> before = circuit.voltages();
      for (auto& d : devices) {
        d->start(*this, circuit.view(), true);
      }
      circuit.invalidate();
      circuit.operating_point(0.0);
      double change = 0.0;
      for (std::size_t i = 0; i < before.size(); ++i) {
        change = std::max(change, std::abs(before[i] - circuit.voltages()[i]) / (1.0 + std::abs(before[i])));
      }
      publish(0.0, bus_in);
      discontinuity = false;
      for (auto& d : devices) {
        d->inputs(*this, 0.0, bus_in);
      }
      if (change < 1e-4 && !discontinuity) {
        break;
      }
    }
    recording = true;
  } else {
    for (auto& d : devices) {
      d->start(*this, circuit.view(), false);
    }
  }
  discontinuity = true;
  started = true;
}

}  // namespace vsim

namespace vsim {

ElectricalNetwork::ElectricalNetwork() : impl_(std::make_unique<Impl>()) {}
ElectricalNetwork::~ElectricalNetwork() = default;

namespace {

std::string resolved_level(const ParamTree& params) {
  if (params.has("electrical.level")) {
    return params.get_string("electrical.level");
  }
  if (params.has("realism.electrical") && params.get_string("realism.electrical") == "ideal") {
    return "E0";
  }
  return "E2";
}

}  // namespace

bool ElectricalNetwork::configured(const ParamTree& params) {
  return params.get_bool("electrical.enabled", false) && params.has_prefix("electrical.components") &&
         resolved_level(params) != "E0";
}

bool ElectricalNetwork::powers_plant(const ParamTree& params) {
  if (!configured(params)) {
    return false;
  }
  for (std::size_t i = 0; params.has_prefix("electrical.components." + std::to_string(i)); ++i) {
    if (params.get_string("electrical.components." + std::to_string(i) + ".type", "") == "battery_model") {
      return true;
    }
  }
  return false;
}

std::string ElectricalNetwork::signal_name(const std::string& name) {
  std::string out;
  for (const char c : name) {
    if (std::isalnum(static_cast<unsigned char>(c)) || c == '_') {
      out += c;
    } else if (c == '+') {
      out += "_P";
    } else if (c == '-') {
      out += "_N";
    } else {
      out += '_';
    }
  }
  if (out.empty() || std::isdigit(static_cast<unsigned char>(out[0]))) {
    out = "_" + out;
  }
  return out;
}

void ElectricalNetwork::init(const ParamTree& params, Context& ctx) {
  Impl& n = *impl_;
  n.ctx = &ctx;
  n.bus = &ctx.bus();
  n.dt = ctx.dt();
  n.level = resolved_level(params);
  if (n.level != "E1" && n.level != "E2") {
    throw std::invalid_argument("electrical.level '" + n.level + "' (expected E0, E1 or E2)");
  }
  n.dynamic = n.level == "E2";
  std::string initial = params.get_string("electrical.initial", "active");
  if (initial == "false" || initial == "False") {
    initial = "off";  // `off` read by a YAML 1.1 parser
  }
  if (initial != "active" && initial != "off") {
    throw std::invalid_argument("electrical.initial '" + initial + "' (expected active or off)");
  }
  n.active_start = initial == "active";
  n.h = params.get_double("electrical.dt_s", n.dt);
  const double ratio = n.dt / n.h;
  n.substeps = static_cast<int>(std::lround(ratio));
  if (!(n.h > 0.0) || n.substeps < 1 || std::abs(ratio - n.substeps) > 1e-6) {
    throw std::invalid_argument("electrical.dt_s must divide time.dt");
  }
  n.circuit.max_iterations = static_cast<int>(params.get_int("electrical.max_iterations", 60));
  n.circuit.gmin = params.get_double("electrical.gmin_s", n.circuit.gmin);
  if (!(n.circuit.gmin > 0.0)) {
    throw std::invalid_argument("electrical.gmin_s must be positive");
  }
  n.rng.emplace(ctx.make_rng("electrical.faults"));
  n.build(params);

  // With the accumulator in the network every driveline motor needs an inverter.
  if (n.carries_battery && !n.standalone) {
    for (const SignalId id : n.bus->match("pt.motor.*.torque_cmd")) {
      const std::string name = n.bus->info(id).name;
      const std::string motor = name.substr(9, name.size() - 9 - 11);
      if (std::find(n.inverter_motors.begin(), n.inverter_motors.end(), motor) == n.inverter_motors.end()) {
        throw std::invalid_argument("electrical: motor " + motor + " has no inverter in the netlist");
      }
    }
    for (const std::string& m : n.inverter_motors) {
      if (!n.bus->find("pt.motor." + m + ".torque_cmd")) {
        throw std::invalid_argument("electrical: inverter motor '" + m + "' is not in the driveline");
      }
    }
  }
}

void ElectricalNetwork::step(double t, SignalBus& bus) {
  Impl& n = *impl_;
  if (!n.started) {
    n.start(bus);
  } else {
    n.apply_faults(t);
    for (auto& d : n.devices) {
      d->inputs(n, t, bus);
    }
  }
  for (int k = 0; k < n.substeps; ++k) {
    elec::Step step;
    step.t = t + static_cast<double>(k + 1) * n.h;
    step.h = n.dynamic ? n.h : 0.0;
    step.euler = n.discontinuity;
    n.discontinuity = false;
    n.circuit.solve(step);
    const Voltages v = n.circuit.view();
    bool changed = false;
    for (const bool controllers : {false, true}) {
      for (auto& d : n.devices) {
        if (d->controller() == controllers) {
          changed = d->update(n, step.t, n.h, v) || changed;
        }
      }
    }
    if (changed) {
      n.discontinuity = true;
      n.circuit.invalidate();
    }
  }
  n.publish(t, bus);
}

void ElectricalNetwork::Impl::publish(double t, SignalBus& bus_out) {
  const Voltages v = circuit.view();
  double ts = 0.0;
  for (auto& d : devices) {
    d->outputs(*this, t, bus_out, v);
    if (const auto* inv = dynamic_cast<const InverterDevice*>(d.get())) {
      ts = std::max(ts, inv->voltage(v));
    }
  }
  for (std::size_t i = 0; i < node_out.size(); ++i) {
    bus_out.set(node_out[i], v(static_cast<NodeId>(i)));
  }
  bus_out.set(ts_voltage_out, ts);
  bus_out.set(ts_active_out, ts > 60.0 ? 1.0 : 0.0);
}

void ElectricalNetwork::init_gate(SignalBus& bus) {
  for (auto& d : impl_->devices) {
    if (auto* ecu = dynamic_cast<EcuDevice*>(d.get())) {
      ecu->resolve_forces(bus);
    }
  }
}

void ElectricalNetwork::gate(double /*t*/, SignalBus& bus) {
  for (auto& d : impl_->devices) {
    if (const auto* ecu = dynamic_cast<const EcuDevice*>(d.get())) {
      ecu->gate(bus);
    }
  }
}

std::string ElectricalNetwork::summary_json() const {
  const Impl& n = *impl_;
  Json out;
  out["level"] = n.level;
  out["substeps"] = n.substeps;
  out["nodes"] = n.circuit.size();
  out["components"] = n.devices.size();
  const double steps = std::max<double>(1.0, static_cast<double>(n.circuit.steps));
  out["solver"] = {{"steps", n.circuit.steps},
                   {"iterations_mean", static_cast<double>(n.circuit.total_iterations) / steps},
                   {"factorizations", n.circuit.total_factorizations},
                   {"not_converged", n.circuit.failures}};
  out["events"] = Json::array();
  for (const Event& e : n.events) {
    out["events"].push_back({{"t", e.t}, {"type", e.type}, {"id", e.id}, {"detail", e.detail}});
  }
  for (const auto& d : n.devices) {
    d->summary(out);
  }
  // LV power budget: everything except the inverters.
  double mean = 0.0, peak = 0.0;
  if (out.contains("loads")) {
    for (auto& [id, load] : out["loads"].items()) {
      if (load["kind"] != "inverter") {
        mean += load["p_mean_w"].get<double>();
        peak += load["p_max_w"].get<double>();
      }
    }
  }
  out["lv_budget"] = {{"p_mean_w", mean}, {"p_peaks_sum_w", peak}};
  return out.dump();
}

namespace {

class ElecBlock final : public Block {
 public:
  ElecBlock(ElectricalNetwork& network, bool gate) : network_(network), gate_(gate) {}
  std::string_view name() const override { return gate_ ? "elec_gate" : "elec"; }
  void init(const ParamTree& params, Context& ctx) override {
    period_ = ctx.dt();
    if (gate_) {
      network_.init_gate(ctx.bus());
    } else {
      network_.init(params, ctx);
    }
  }
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override {
    if (gate_) {
      network_.gate(t, bus);
    } else {
      network_.step(t, bus);
    }
  }

 private:
  ElectricalNetwork& network_;
  bool gate_;
  double period_ = 0.001;
};

}  // namespace

std::unique_ptr<Block> ElectricalNetwork::make_block(bool gate) {
  return std::make_unique<ElecBlock>(*this, gate);
}

std::string ElectricalNetwork::operating_point_json(const ParamTree& params) {
  SignalBus bus;
  InputHub inputs;
  ParamTree p = params;
  if (!p.has("electrical.level") || p.get_string("electrical.level") == "E0") {
    p.set("electrical.level", "E2");
  }
  p.set("electrical.initial", "active");
  Context ctx(bus, inputs, RealismConfig::from_params(p), 0, p.get_double("time.dt", 0.001));
  ElectricalNetwork network;
  network.impl_->standalone = true;
  network.init(p, ctx);
  network.step(0.0, bus);
  const Impl& n = *network.impl_;
  Json out;
  out["nodes"] = Json::object();
  for (std::size_t i = 0; i < n.circuit.size(); ++i) {
    out["nodes"][n.circuit.node_names()[i]] = n.circuit.voltages()[i];
  }
  out["signals"] = Json::object();
  for (SignalId id = 0; id < bus.size(); ++id) {
    const std::string& name = bus.info(id).name;
    if (name.rfind("elec.", 0) == 0 && name.rfind("elec.v.", 0) != 0) {
      out["signals"][name] = bus.get(id);
    }
  }
  out["converged"] = n.circuit.failures == 0;
  out["summary"] = Json::parse(network.summary_json());
  return out.dump();
}

namespace {

// SPICE wants plain names; the mapping is written as comments.
std::string spice_name(const std::string& name) {
  std::string out;
  for (const char c : name) {
    if (std::isalnum(static_cast<unsigned char>(c))) {
      out += c;
    } else {
      out += c == '+' ? "_P" : c == '-' ? "_N" : "_";
    }
  }
  return out.empty() ? std::string("n") : out;
}

std::string spice_number(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.9g", v);
  return buf;
}

}  // namespace

std::string ElectricalNetwork::spice_netlist(const ParamTree& params, const std::string& title) {
  SignalBus bus;
  InputHub inputs;
  ParamTree p = params;
  if (!p.has("electrical.level") || p.get_string("electrical.level") == "E0") {
    p.set("electrical.level", "E2");
  }
  p.set("electrical.initial", "active");
  Context ctx(bus, inputs, RealismConfig::from_params(p), 0, p.get_double("time.dt", 0.001));
  ElectricalNetwork network;
  network.impl_->standalone = true;
  network.init(p, ctx);
  network.step(0.0, bus);
  const elec::Circuit& c = network.impl_->circuit;
  // Net names, unique after the clean-up.
  std::vector<std::string> nets;
  std::set<std::string> taken;
  for (const std::string& name : c.node_names()) {
    std::string net = "n_" + spice_name(name);
    for (int k = 2; !taken.insert(net).second; ++k) {
      net = "n_" + spice_name(name) + "_" + std::to_string(k);
    }
    nets.push_back(net);
  }
  const auto node = [&](elec::NodeId n) { return n < 0 ? std::string("0") : nets[static_cast<std::size_t>(n)]; };
  constexpr double kOpen = 1e12;
  std::ostringstream o;
  o << "* " << title << "\n";
  o << "* Exported by vsim (design doc §12.4): the operating point the simulator solves at rest.\n";
  o << "* Nets:";
  for (std::size_t i = 0; i < c.size(); ++i) {
    o << (i % 4 == 0 ? "\n*  " : "  ") << node(static_cast<elec::NodeId>(i)) << " = " << c.node_names()[i];
  }
  o << "\n";
  int k = 0;
  for (const auto& element : c.elements()) {
    const std::string id = spice_name(element->id()) + "_" + std::to_string(k++);
    if (const auto* r = dynamic_cast<const elec::Resistor*>(element.get())) {
      o << "R" << id << " " << node(r->a) << " " << node(r->b) << " " << spice_number(r->open ? kOpen : r->resistance())
        << "\n";
    } else if (const auto* sw = dynamic_cast<const elec::Switch*>(element.get())) {
      o << "R" << id << " " << node(sw->a) << " " << node(sw->b) << " " << spice_number(sw->closed ? sw->r_on : kOpen)
        << (sw->closed ? " ; contact closed" : " ; contact open") << "\n";
    } else if (const auto* cap = dynamic_cast<const elec::Capacitor*>(element.get())) {
      const std::string mid = cap->esr > 0.0 ? "x_" + id : node(cap->b);
      o << "C" << id << " " << node(cap->a) << " " << mid << " " << spice_number(cap->c) << "\n";
      if (cap->esr > 0.0) {
        o << "R" << id << "_esr " << mid << " " << node(cap->b) << " " << spice_number(cap->esr) << "\n";
      }
    } else if (const auto* l = dynamic_cast<const elec::Inductor*>(element.get())) {
      const std::string mid = "x_" + id;
      o << "L" << id << " " << node(l->a) << " " << mid << " " << spice_number(l->l) << "\n";
      o << "R" << id << "_r " << mid << " " << node(l->b) << " " << spice_number(std::max(l->r, 1e-6)) << "\n";
    } else if (const auto* v = dynamic_cast<const elec::VoltageSource*>(element.get())) {
      const std::string mid = "x_" + id;
      o << "V" << id << " " << mid << " " << node(v->neg) << " DC " << spice_number(v->e) << "\n";
      o << "R" << id << "_r " << node(v->pos) << " " << mid << " " << spice_number(v->open ? kOpen : v->r) << "\n";
    } else if (const auto* i = dynamic_cast<const elec::CurrentSource*>(element.get())) {
      o << "I" << id << " " << node(i->neg) << " " << node(i->pos) << " DC " << spice_number(i->i) << "\n";
    } else if (const auto* load = dynamic_cast<const elec::Load*>(element.get())) {
      const std::string a = node(load->a), b = node(load->b);
      if (!load->on) {
        o << "* " << element->id() << ": load off\n";
      } else if (load->law == elec::Load::Law::Resistance) {
        o << "R" << id << " " << a << " " << b << " " << spice_number(load->value) << "\n";
      } else {
        // Power or current, fading into a resistance below the knee voltage as in the simulator.
        const std::string amount = spice_number(load->value), knee = spice_number(load->v_knee);
        const std::string law = load->law == elec::Load::Law::Power
                                    ? amount + "/max(V(" + a + "," + b + ")," + knee + ")"
                                    : amount;
        o << "B" << id << " " << a << " " << b << " I=(" << law << ")*min(max(V(" << a << "," << b << "),0)/" << knee
          << ",1)\n";
      }
    } else if (const auto* d = dynamic_cast<const elec::Diode*>(element.get())) {
      // Exponential diode through v_f at 1 A, r_on in series.
      o << "D" << id << " " << node(d->a) << " " << node(d->b) << " D_" << id << "\n";
      o << ".model D_" << id << " D(IS=" << spice_number(std::exp(-d->v_f / 0.025852)) << " N=1 RS="
        << spice_number(d->r_on) << ")\n";
    } else if (const auto* reg = dynamic_cast<const elec::Regulator*>(element.get())) {
      const std::string in = node(reg->in), out = node(reg->out), gnd = node(reg->gnd);
      o << "B" << id << "_e x_" << id << " " << gnd << " V=max(min(" << spice_number(reg->v_set) << ",V(" << in << ","
        << gnd << ")-" << spice_number(reg->dropout) << "),0)\n";
      o << "V" << id << "_sense x_" << id << " y_" << id << " DC 0\n";
      o << "R" << id << "_out y_" << id << " " << out << " " << spice_number(reg->r_out) << "\n";
      o << "B" << id << "_in " << in << " " << gnd << " I=max(I(V" << id << "_sense),0)+" << spice_number(reg->i_q)
        << "\n";
    } else if (const auto* dc = dynamic_cast<const elec::DcDc*>(element.get())) {
      const std::string ip = node(dc->in_p), in_n = node(dc->in_n), op = node(dc->out_p), on = node(dc->out_n);
      if (!dc->on) {
        o << "* " << element->id() << ": converter off\n";
        continue;
      }
      // The internal voltage where the converter has settled (it holds the output at v_set).
      o << "V" << id << "_e x_" << id << " " << on << " DC " << spice_number(dc->e > 0.0 ? dc->e : dc->v_set) << "\n";
      o << "V" << id << "_sense x_" << id << " y_" << id << " DC 0\n";
      o << "R" << id << "_out y_" << id << " " << op << " " << spice_number(dc->r_out) << "\n";
      o << "B" << id << "_in " << ip << " " << in_n << " I=(V(" << op << "," << on << ")*max(I(V" << id
        << "_sense),0)/" << spice_number(dc->efficiency) << "+" << spice_number(dc->p_idle) << ")/max(V(" << ip
        << "," << in_n << ")," << spice_number(dc->v_knee) << ")\n";
    } else if (const auto* pot = dynamic_cast<const elec::Potentiometer*>(element.get())) {
      // Track a-b split at the wiper (r x from the wiper to b).
      o << "R" << id << "_a " << node(pot->a) << " " << node(pot->w) << " "
        << spice_number(std::max(pot->r * (1.0 - pot->x), 1e-3)) << "\n";
      o << "R" << id << "_b " << node(pot->w) << " " << node(pot->b) << " " << spice_number(std::max(pot->r * pot->x, 1e-3))
        << "\n";
    } else {
      o << "* " << element->id() << ": not exported\n";
    }
  }
  // Every net gets a path to ground so floating parts (an open contactor's far side) stay defined.
  for (std::size_t i = 0; i < c.size(); ++i) {
    o << "Rleak_" << i << " " << node(static_cast<elec::NodeId>(i)) << " 0 1e12\n";
  }
  o << ".options gmin=1e-12\n.op\n.end\n";
  return o.str();
}

}  // namespace vsim

