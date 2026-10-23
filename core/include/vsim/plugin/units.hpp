#pragma once

#include <string_view>

namespace vsim {

// value_to = value_from * scale + offset.
struct UnitConversion {
  double scale = 1.0;
  double offset = 0.0;
  double apply(double value) const { return value * scale + offset; }
  bool identity() const { return scale == 1.0 && offset == 0.0; }
};

// Conversion between the unit of a plugin port and the unit of the signal it
// is connected to (design doc §11.4): "rpm" <-> "rad/s", "km/h" <-> "m/s",
// "deg" <-> "rad", "bar" <-> "Pa", "%" <-> "-", "degC" <-> "K", ... Identity
// when the units are equal or one side declares none (""). Throws
// std::invalid_argument for units of different kinds or an unknown unit
// that differs from the other side.
UnitConversion unit_conversion(std::string_view from, std::string_view to);

}  // namespace vsim
