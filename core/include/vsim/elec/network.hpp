#pragma once

#include <memory>
#include <string>

#include "vsim/sim/block.hpp"

namespace vsim {

// Electrical network (design doc §12.3): the HV and LV circuits, the
// shutdown circuit and the sensor wiring of the car, solved by modified nodal
// analysis every plant step. Defined under `electrical:` (the project's
// electrical/netlist.yaml):
//
//   electrical:
//     enabled: true
//     level: E2                # E0 off | E1 resistive (DC) | E2 transient
//     dt_s: 0.001              # sub-step, divides time.dt
//     initial: active          # active (precharged, relays where their coils put them) | off
//     components:
//       - {id: ACC, type: battery_model, pos: ACC+, neg: ACC-}        # the car's accumulator
//       - {id: AIR_P, type: contactor, a: ACC+, b: DC+, coil_a: SDC_OUT, coil_b: GND}
//       - {id: HV_CABLE, type: wire, a: DC+, b: INV_DC+, length_m: 1.2, area_mm2: 16}
//       - {id: C_LINK, type: capacitor, a: INV_DC+, b: DC-, c_f: 400e-6}
//       - {id: INV, type: inverter, pos: INV_DC+, neg: DC-, motors: [fl, fr, rl, rr]}
//       - {id: VCU, type: ecu, a: LV_VCU, b: GND, i_a: 0.4, v_min: 7.0, can_node: VCU}
//     faults: [{type: open | short | contact_resistance | stuck_closed | stuck_open | blow, target: ..., at_s: ...}]
//
// With a battery_model the network carries the traction current: the plant
// takes the battery current and each motor's DC voltage and enable from it
// (elec.bat.current, elec.motor.<m>.dc_voltage, elec.motor.<m>.enabled) and
// gives back its open-circuit voltage and the motors' DC power. Outputs:
// elec.v.<net> per node, elec.<id>.<quantity> per component.
class ElectricalNetwork {
 public:
  ElectricalNetwork();
  ~ElectricalNetwork();
  ElectricalNetwork(const ElectricalNetwork&) = delete;
  ElectricalNetwork& operator=(const ElectricalNetwork&) = delete;

  // electrical.enabled with components, at level E1 or E2. Without an
  // explicit level the network runs at E2 unless realism.electrical is ideal.
  static bool configured(const ParamTree& params);
  // The network carries the accumulator (a battery_model component).
  static bool powers_plant(const ParamTree& params);
  // Signal-name form of a net or component name (dots and signs replaced).
  static std::string signal_name(const std::string& name);

  void init(const ParamTree& params, Context& ctx);
  // Solves [t, t + dt): node voltages, currents, relays, fuses, ECU power.
  void step(double t, SignalBus& bus);
  // ECUs without power: the signals they drive (`forces`, resolved once
  // the controllers have declared them) are held at 0.
  void init_gate(SignalBus& bus);
  void gate(double t, SignalBus& bus);
  std::string summary_json() const;

  // The solve (after the driver, before the sensors) and the ECU gate
  // (after the controllers, before CAN sends and the plant).
  std::unique_ptr<Block> make_block(bool gate);

  // DC operating point of a netlist for editors: node voltages and component
  // currents as JSON, inputs at rest (no motor power, pedals released).
  static std::string operating_point_json(const ParamTree& params);

  struct Impl;

 private:
  std::unique_ptr<Impl> impl_;
};

}  // namespace vsim
