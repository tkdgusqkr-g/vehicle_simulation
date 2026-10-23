#include "vsim/driver/profile.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "vsim/util/paths.hpp"

namespace vsim {

namespace {

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> cells;
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) {
    const auto first = cell.find_first_not_of(" \t\r\"");
    const auto last = cell.find_last_not_of(" \t\r\"");
    cells.push_back(first == std::string::npos ? "" : cell.substr(first, last - first + 1));
  }
  return cells;
}

// Columns `time_column` and `column` of a CSV file with a header row.
void read_csv_columns(const std::filesystem::path& file, const std::string& time_column, const std::string& column,
                      std::vector<double>& times, std::vector<double>& values, std::string_view what) {
  std::ifstream in(file);
  if (!in) {
    throw std::invalid_argument(std::string(what) + ": cannot read " + path_to_utf8(file));
  }
  std::string line;
  std::getline(in, line);
  const std::vector<std::string> header = split_csv(line);
  const auto find = [&](const std::string& name) {
    const auto it = std::find(header.begin(), header.end(), name);
    if (it == header.end()) {
      throw std::invalid_argument(std::string(what) + ": no column '" + name + "' in " + path_to_utf8(file));
    }
    return static_cast<std::size_t>(it - header.begin());
  };
  const std::size_t ti = find(time_column), vi = find(column);
  std::size_t row = 1;
  while (std::getline(in, line)) {
    ++row;
    if (line.find_first_not_of(" \t\r") == std::string::npos) {
      continue;
    }
    const std::vector<std::string> cells = split_csv(line);
    if (cells.size() <= std::max(ti, vi)) {
      throw std::invalid_argument(std::string(what) + ": " + path_to_utf8(file) + " row " + std::to_string(row) +
                                  " is short");
    }
    try {
      times.push_back(std::stod(cells[ti]));
      values.push_back(std::stod(cells[vi]));
    } catch (const std::exception&) {
      throw std::invalid_argument(std::string(what) + ": " + path_to_utf8(file) + " row " + std::to_string(row) +
                                  " is not a number");
    }
  }
}

}  // namespace

Profile Profile::from_params(const ParamTree& config, std::string_view what, const std::filesystem::path& base_dir) {
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
  } else if (type == "pulse") {
    profile.type_ = Type::Pulse;
    profile.at_ = config.get_double("start_s", 0.0);
    profile.duration_ = config.get_double("period_s");
    profile.rate_ = config.get_double("width_s");
    profile.value_ = config.get_double("value", 1.0);
    profile.before_ = config.get_double("base", 0.0);
    profile.max_ = config.get_double("count", std::numeric_limits<double>::infinity());
    if (!(profile.duration_ > 0.0 && profile.rate_ >= 0.0 && profile.rate_ <= profile.duration_)) {
      throw std::invalid_argument(std::string(what) + ": pulse needs period_s > 0 and 0 <= width_s <= period_s");
    }
  } else if (type == "table" || type == "file") {
    profile.type_ = Type::Table;
    if (type == "table") {
      profile.times_ = config.get_double_list("t");
      profile.values_ = config.get_double_list("values");
    } else {
      std::filesystem::path file = path_from_utf8(config.get_string("path"));
      if (file.is_relative() && !base_dir.empty()) {
        file = base_dir / file;
      }
      read_csv_columns(file, config.get_string("time_column", "t"), config.get_string("column"), profile.times_,
                       profile.values_, what);
    }
    const double scale = config.get_double("scale", 1.0), offset = config.get_double("offset", 0.0);
    for (double& v : profile.values_) {
      v = scale * v + offset;
    }
    if (profile.times_.empty() || profile.times_.size() != profile.values_.size() ||
        !std::is_sorted(profile.times_.begin(), profile.times_.end())) {
      throw std::invalid_argument(std::string(what) + ": " + type + " needs as many ascending times as values");
    }
  } else {
    throw std::invalid_argument(std::string(what) + ": unknown profile type '" + type +
                                "' (expected const, step, ramp, sine, sweep, sine_dwell, pulse, table or file)");
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
    case Type::Pulse: {
      const double tau = t - at_;
      if (tau < 0.0 || std::floor(tau / duration_) >= max_) {
        return before_;
      }
      return std::fmod(tau, duration_) < rate_ ? value_ : before_;
    }
    case Type::Table: {
      if (t <= times_.front()) {
        return values_.front();
      }
      if (t >= times_.back()) {
        return values_.back();
      }
      const auto it = std::upper_bound(times_.begin(), times_.end(), t);
      const std::size_t i = static_cast<std::size_t>(it - times_.begin());
      const double span = times_[i] - times_[i - 1];
      const double f = span > 0.0 ? (t - times_[i - 1]) / span : 1.0;
      return values_[i - 1] + f * (values_[i] - values_[i - 1]);
    }
  }
  return 0.0;
}

}  // namespace vsim
