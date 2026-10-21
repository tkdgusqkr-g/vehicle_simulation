#pragma once

#include <string>
#include <vector>

#include "vsim/sim/signal_bus.hpp"

namespace vsim {

// Keeps logged signals in memory, column by column.
class Recorder {
 public:
  void configure(const SignalBus& bus, const std::vector<SignalId>& ids);
  void record(double t, const SignalBus& bus);
  void clear();

  const std::vector<std::string>& names() const { return names_; }
  const std::vector<double>& times() const { return times_; }
  const std::vector<std::vector<double>>& columns() const { return columns_; }

 private:
  std::vector<SignalId> ids_;
  std::vector<std::string> names_;
  std::vector<double> times_;
  std::vector<std::vector<double>> columns_;
};

}  // namespace vsim
