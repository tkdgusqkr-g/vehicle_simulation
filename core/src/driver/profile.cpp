#include "vsim/driver/profile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace vsim {

Profile Profile::from_params(const ParamTree& config, std::string_view what) {
  Profile profile;
  if (config.empty()) {
    return profile;
  }
  const std::string type = config.get_string("type", "const");
  if (type == "const") {
    profile.type_ = Type::Const;
    profile.value_ = config.get_double("value", 0.0);
  } else if (type == "step") {
    profile.type_ = Type::Step;
    profile.at_ = config.get_double("at_s", 0.0);
    profile.value_ = config.get_double("value");
    profile.before_ = config.get_double("before", 0.0);
  } else if (type == "ramp") {
    profile.type_ = Type::Ramp;
    profile.at_ = config.get_double("start_s", 0.0);
    profile.rate_ = config.get_double("rate");
    profile.offset_ = config.get_double("offset", 0.0);
    profile.max_ = config.get_double(
        "max", profile.rate_ >= 0.0 ? std::numeric_limits<double>::infinity()
                                    : -std::numeric_limits<double>::infinity());
  } else if (type == "sine") {
    profile.type_ = Type::Sine;
    profile.at_ = config.get_double("start_s", 0.0);
    profile.amplitude_ = config.get_double("amplitude");
    profile.freq_ = config.get_double("freq_hz");
    profile.offset_ = config.get_double("offset", 0.0);
  } else if (type == "sweep") {
    profile.type_ = Type::Sweep;
    profile.at_ = config.get_double("start_s", 0.0);
    profile.amplitude_ = config.get_double("amplitude");
    profile.freq_ = config.get_double("f0_hz");
    profile.freq_end_ = config.get_double("f1_hz");
    profile.duration_ = config.get_double("duration_s");
    profile.offset_ = config.get_double("offset", 0.0);
    if (!(profile.duration_ > 0.0 && profile.freq_ >= 0.0 && profile.freq_end_ >= 0.0)) {
      throw std::invalid_argument(std::string(what) + ": sweep needs duration_s > 0 and frequencies >= 0");
    }
  } else if (type == "sine_dwell") {
    profile.type_ = Type::SineDwell;
    profile.at_ = config.get_double("start_s", 0.0);
    profile.amplitude_ = config.get_double("amplitude");
    profile.freq_ = config.get_double("freq_hz");
    profile.duration_ = config.get_double("dwell_s", 0.5);
    profile.offset_ = config.get_double("offset", 0.0);
    if (!(profile.freq_ > 0.0 && profile.duration_ >= 0.0)) {
      throw std::invalid_argument(std::string(what) + ": sine_dwell needs freq_hz > 0 and dwell_s >= 0");
    }
  } else {
    throw std::invalid_argument(std::string(what) + ": unknown profile type '" + type +
                                "' (expected const, step, ramp, sine, sweep or sine_dwell)");
  }
  return profile;
}

double Profile::value(double t) const {
  switch (type_) {
    case Type::Const:
      return value_;
    case Type::Step:
      return t >= at_ ? value_ : before_;
    case Type::Ramp: {
      const double v = offset_ + rate_ * std::max(0.0, t - at_);
      return rate_ >= 0.0 ? std::min(v, max_) : std::max(v, max_);
    }
    case Type::Sine:
      if (t < at_) {
        return offset_;
      }
      return offset_ + amplitude_ * std::sin(2.0 * std::numbers::pi * freq_ * (t - at_));
    case Type::Sweep: {
      const double tau = t - at_;
      if (tau < 0.0 || tau > duration_) {
        return offset_;
      }
      // Phase of a linear chirp: 2 pi (f0 tau + (f1 - f0) tau^2 / (2 T)).
      const double phase = 2.0 * std::numbers::pi * (freq_ * tau + (freq_end_ - freq_) * tau * tau / (2.0 * duration_));
      return offset_ + amplitude_ * std::sin(phase);
    }
    case Type::SineDwell: {
      const double tau = t - at_;
      const double period = 1.0 / freq_;
      const double peak = 0.75 * period;  // the second (negative) peak
      if (tau < 0.0) {
        return offset_;
      }
      if (tau < peak) {
        return offset_ + amplitude_ * std::sin(2.0 * std::numbers::pi * freq_ * tau);
      }
      if (tau < peak + duration_) {
        return offset_ - amplitude_;
      }
      if (tau < period + duration_) {
        return offset_ + amplitude_ * std::sin(2.0 * std::numbers::pi * freq_ * (tau - duration_));
      }
      return offset_;
    }
  }
  return 0.0;
}

}  // namespace vsim
