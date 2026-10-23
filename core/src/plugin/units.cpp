#include "vsim/plugin/units.hpp"

#include <array>
#include <numbers>
#include <stdexcept>
#include <string>

namespace vsim {
namespace {

struct Unit {
  std::string_view name;
  std::string_view kind;
  double scale;  // SI value = value * scale + offset
  double offset = 0.0;
};

constexpr double kPi = std::numbers::pi;

constexpr std::array kUnits = {
    Unit{"-", "ratio", 1.0},         Unit{"1", "ratio", 1.0},        Unit{"%", "ratio", 0.01},
    Unit{"m", "length", 1.0},        Unit{"mm", "length", 1e-3},     Unit{"cm", "length", 1e-2},
    Unit{"km", "length", 1e3},       Unit{"s", "time", 1.0},         Unit{"ms", "time", 1e-3},
    Unit{"us", "time", 1e-6},        Unit{"min", "time", 60.0},      Unit{"h", "time", 3600.0},
    Unit{"m/s", "speed", 1.0},       Unit{"km/h", "speed", 1.0 / 3.6}, Unit{"kph", "speed", 1.0 / 3.6},
    Unit{"mph", "speed", 0.44704},   Unit{"m/s^2", "acceleration", 1.0}, Unit{"m/s2", "acceleration", 1.0},
    Unit{"g", "acceleration", 9.80665},
    Unit{"rad", "angle", 1.0},       Unit{"deg", "angle", kPi / 180.0},
    Unit{"rad/s", "angular_speed", 1.0}, Unit{"deg/s", "angular_speed", kPi / 180.0},
    Unit{"rpm", "angular_speed", 2.0 * kPi / 60.0}, Unit{"rev/s", "angular_speed", 2.0 * kPi},
    Unit{"rad/s^2", "angular_acceleration", 1.0}, Unit{"deg/s^2", "angular_acceleration", kPi / 180.0},
    Unit{"N", "force", 1.0},         Unit{"kN", "force", 1e3},
    Unit{"N*m", "torque", 1.0},      Unit{"Nm", "torque", 1.0},      Unit{"N.m", "torque", 1.0},
    Unit{"Pa", "pressure", 1.0},     Unit{"kPa", "pressure", 1e3},   Unit{"MPa", "pressure", 1e6},
    Unit{"bar", "pressure", 1e5},    Unit{"psi", "pressure", 6894.757293168},
    Unit{"W", "power", 1.0},         Unit{"kW", "power", 1e3},
    Unit{"J", "energy", 1.0},        Unit{"kJ", "energy", 1e3},      Unit{"Wh", "energy", 3600.0},
    Unit{"kWh", "energy", 3.6e6},
    Unit{"A", "current", 1.0},       Unit{"mA", "current", 1e-3},
    Unit{"V", "voltage", 1.0},       Unit{"mV", "voltage", 1e-3},
    Unit{"Ohm", "resistance", 1.0},  Unit{"mOhm", "resistance", 1e-3},
    Unit{"K", "temperature", 1.0},   Unit{"degC", "temperature", 1.0, 273.15},
    Unit{"C", "temperature", 1.0, 273.15},
    Unit{"kg", "mass", 1.0},         Unit{"Hz", "frequency", 1.0},
    Unit{"1/m", "curvature", 1.0},
};

const Unit* find_unit(std::string_view name) {
  for (const Unit& u : kUnits) {
    if (u.name == name) {
      return &u;
    }
  }
  return nullptr;
}

}  // namespace

UnitConversion unit_conversion(std::string_view from, std::string_view to) {
  if (from == to || from.empty() || to.empty()) {
    return {};
  }
  const Unit* a = find_unit(from);
  const Unit* b = find_unit(to);
  if (a == nullptr || b == nullptr) {
    throw std::invalid_argument("cannot convert '" + std::string(from) + "' to '" + std::string(to) +
                                "': unknown unit '" + std::string(a == nullptr ? from : to) + "'");
  }
  if (a->kind != b->kind) {
    throw std::invalid_argument("cannot convert '" + std::string(from) + "' (" + std::string(a->kind) + ") to '" +
                                std::string(to) + "' (" + std::string(b->kind) + ")");
  }
  // SI = v * sa + oa;  v_to = (SI - ob) / sb.
  return {a->scale / b->scale, (a->offset - b->offset) / b->scale};
}

}  // namespace vsim
