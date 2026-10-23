#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace vsim {

// A CAN frame crossing the boundary of the simulation: to and from an
// external component (sdk/c/include/vsim_external.h) or a real CAN adapter.
struct ExternalFrame {
  std::size_t bus = 0;  // index into the component's / adapter's bus list
  std::uint32_t id = 0;
  bool extended = false, fd = false, brs = false;
  std::vector<std::uint8_t> data;
  double time = 0.0;  // [s] simulated time it was sent (or received)
};

// The CAN side of an external component (components.<name>: {type: external,
// can: [bus, ...]}): the virtual network hands it the frames on its buses and
// takes the frames it transmits, step by step.
class ExternalEndpoint {
 public:
  virtual ~ExternalEndpoint() = default;
  // Names of the buses it is on, in the order of ExternalFrame::bus.
  virtual const std::vector<std::string>& buses() const = 0;
  // A frame seen on one of its buses; delivered with its next step.
  virtual void receive(const ExternalFrame& frame) = 0;
  // Frames it sent in its steps since the last call.
  virtual std::vector<ExternalFrame> take_sent() = 0;
};

// The endpoint of the external component `component` of the running
// simulation, or nullptr (no such component, or it has no buses).
ExternalEndpoint* find_external_endpoint(const std::string& component);

}  // namespace vsim
