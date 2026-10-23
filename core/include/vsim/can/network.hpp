#pragma once

#include <filesystem>
#include <memory>
#include <string>

#include "vsim/sim/block.hpp"

namespace vsim {

// Virtual CAN (design doc §12.2): buses described by DBC files, nodes with
// their own clocks, and a map from DBC signals to bus signals. Defined under
// `can:` (the project's can/network.yaml):
//
//   can:
//     enabled: true
//     fidelity: timing         # ideal | timing | faults (default from realism.comms)
//     buses: [{name: can_main, bitrate: 1000000, data_bitrate: 2000000, dbc: [can/vehicle.dbc]}]
//     nodes:                   # nodes of the DBCs; unlisted ones are devices with a perfect clock
//       - {name: VCU, role: controller, clock_ppm: 30, jitter_us: 50, task_ms: 10}
//       - {name: INV_FL, role: device, clock_ppm: -45}
//     map: {VCU_Torque: {TorqueFL: pt.motor.fl.torque_cmd}, INV_FL_Status: {Speed: pt.motor.fl.speed}}
//     timeouts: {INV_FL_Status: 0.03, VCU_TorqueCmd: {after_s: 0.05, then: zero}}  # default 3 cycles, hold
//     load: [{bus: can_main, id: 0x080, period_ms: 1, length: 8}]   # extra traffic
//     faults: [{type: drop | delay | error_rate | bus_off | skip, ...}]
//     log: asc                 # asc | candump | both | none: <bus>.asc / <bus>.log in the run folder
//
// Devices (inverters, BMS, sensors) send what the plant and the sensors
// produce and receive what the plant consumes; controllers (the VCU, its
// plugins) the other way round. So a torque command mapped to
// pt.motor.fl.torque_cmd reaches the plant only through its frames, and a
// motor speed mapped to pt.motor.fl.speed reaches the VCU only through
// its frames, with the bus's latency, losses and timing.
//
// Two blocks run it: one before the controllers (devices send, controllers
// receive) and one after them (controllers send, the plant receives; the
// bus advances). Outputs per message: can.<msg>.<signal> (decoded),
// .latency, .age, .timeout, .lost, .missed / .doubled (per receiver task);
// per bus: can.<bus>.load; per node: can.<node>.tec, .bus_off.
class CanNetwork {
 public:
  CanNetwork();
  ~CanNetwork();
  CanNetwork(const CanNetwork&) = delete;
  CanNetwork& operator=(const CanNetwork&) = delete;

  static bool configured(const ParamTree& params);
  void init(const ParamTree& params, Context& ctx);

  void before_controllers(double t, SignalBus& bus);
  void after_controllers(double t, SignalBus& bus);
  void after_plant_inputs(SignalBus& bus);

  // Bus logs into the run folder (asc / candump, per network.log).
  void open_logs(const std::filesystem::path& dir);
  void close_logs();
  // Bus loads, message latencies, losses and timeouts as a JSON object.
  std::string summary_json() const;

  // The two blocks that run the network inside a tick.
  std::unique_ptr<Block> make_block(bool after_controllers);

  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace vsim
