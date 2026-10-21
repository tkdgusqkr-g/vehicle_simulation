#include "vsim/sim/recorder.hpp"

namespace vsim {

void Recorder::configure(const SignalBus& bus, const std::vector<SignalId>& ids) {
  ids_ = ids;
  names_.clear();
  for (const SignalId id : ids_) {
    names_.push_back(bus.info(id).name);
  }
  times_.clear();
  columns_.assign(ids_.size(), {});
}

void Recorder::record(double t, const SignalBus& bus) {
  times_.push_back(t);
  for (std::size_t i = 0; i < ids_.size(); ++i) {
    columns_[i].push_back(bus.get(ids_[i]));
  }
}

void Recorder::clear() {
  times_.clear();
  for (auto& column : columns_) {
    column.clear();
  }
}

}  // namespace vsim
