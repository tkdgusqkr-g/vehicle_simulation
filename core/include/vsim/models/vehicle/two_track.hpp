#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "vsim/models/driveline/driveline.hpp"
#include "vsim/models/powertrain/battery.hpp"
#include "vsim/models/powertrain/motor.hpp"
#include "vsim/models/tire/tire.hpp"
#include "vsim/models/vehicle/corners.hpp"
#include "vsim/sim/plant.hpp"

namespace vsim {

// L2 vehicle model (design doc §6.2): planar two-track chassis (x, y, yaw and
// their rates) on four tires with transient slip, quasi-static load transfer
// through roll stiffness and roll centres, wheel spin through the driveline
// graph, motors (Level A) and the accumulator equivalent circuit. Every power
// flow is integrated so the energy balance can be checked (V-03).
//
// Inputs:  drv.cmd.steer, drv.cmd.brake, pt.motor.<id>.torque_cmd
// Outputs: veh.*, pt.*, bat.*, energy.*
class TwoTrackModel final : public PlantModel {
 public:
  // Chassis states; the rest of the state vector is laid out at init().
  enum Chassis : std::size_t { kX, kY, kYaw, kVx, kVy, kYawRate, kAxFiltered, kAyFiltered, kChassisSize };

  // Integrated energies [J], in state order.
  enum Energy : std::size_t {
    kBatteryChemical,  // OCV * I: released by the cells
    kBatteryLoss,      // ohmic and RC resistor losses
    kAux,              // constant auxiliary load
    kMotorLoss,        // motor + inverter losses
    kMotorShaft,       // mechanical work at the motor shafts
    kGearLoss,
    kDiffLoss,
    kBrakeLoss,
    kSlipLoss,         // tire slip, including the transient carcass energy
    kRollingLoss,
    kDragLoss,
    kEnergyCount
  };

  struct Params {
    double mass = 280.0;
    double yaw_inertia = 110.0;
    double wheelbase = 1.53;
    double cg_to_front = 0.80;
    double cg_height = 0.28;
    double track_front = 1.22;
    double track_rear = 1.18;
    double roll_center_front = 0.03;   // [m] above ground
    double roll_center_rear = 0.05;
    double roll_stiffness_front = 0.5;  // front share of the total roll stiffness
    double load_transfer_tau = 0.05;    // [s] lag on ax, ay for load transfer
    double steering_ratio = 4.5;
    double max_steer_wheel = 1.92;      // [rad] at the steering wheel
    double ackermann = 0.0;             // 0 parallel, 1 full Ackermann
    double mechanical_trail = 0.02;     // [m] caster trail
    double wheel_radius = 0.203;
    double wheel_inertia = 0.3;         // [kg m^2] wheel, tire, hub
    double rolling_resistance = 0.015;
    double relaxation_x = 0.10;         // [m]
    double relaxation_y = 0.25;         // [m]
    double low_speed_damping = 0.5;     // damping ratio of the tire modes near standstill
    double low_speed_fade = 3.0;        // [m/s] damping fades out by this speed
    double cla = 3.2;
    double cda = 1.3;
    double aero_front_ratio = 0.45;
    double air_density = 1.2;
    double brake_pressure_max = 60.0;   // [bar]
    double brake_gain_front = 30.0;     // [N m / bar] axle total
    double brake_gain_rear = 18.0;
    double ambient_temp = 25.0;         // [degC]

    double cg_to_rear() const { return wheelbase - cg_to_front; }
    double track(std::size_t corner) const { return is_front(corner) ? track_front : track_rear; }
  };

  TwoTrackModel();
  ~TwoTrackModel() override;

  std::string_view name() const override { return "two_track"; }
  void init(const ParamTree& params, Context& ctx) override;
  std::size_t state_size() const override { return size_; }
  void initial_state(std::span<double> x) const override;
  void read_inputs(const SignalBus& bus) override;
  void derivatives(double t, std::span<const double> x, std::span<double> dx) const override;
  void write_outputs(double t, std::span<const double> x, SignalBus& bus) const override;

  static Params params_from(const ParamTree& params);
  const Params& params() const { return p_; }
  const MotorParams& motor_params() const { return motor_; }
  const BatteryParams& battery_params() const { return battery_; }
  const Driveline& driveline() const { return driveline_; }

  // State offsets, for tests and tools.
  std::size_t wheel_dof_offset() const { return wheel_offset_; }
  std::size_t energy_offset() const { return energy_offset_; }
  std::size_t battery_offset() const { return battery_offset_; }

 private:
  struct Eval;
  struct Signals;

  // Forces and flows for state x; also fills dx unless it is empty.
  void evaluate(std::span<const double> x, Eval& e, std::span<double> dx) const;
  double kinetic_energy(std::span<const double> x) const;

  Params p_;
  MotorParams motor_;
  BatteryParams battery_;
  DrivelineConfig driveline_config_;
  Driveline driveline_;
  std::array<std::unique_ptr<TireModel>, 2> tires_;  // front, rear

  // State layout.
  std::size_t wheel_offset_ = 0;    // driveline coordinates
  std::size_t slip_offset_ = 0;     // per corner: kappa', tan(alpha')
  std::size_t motor_offset_ = 0;    // per motor: torque, temperature
  std::size_t battery_offset_ = 0;  // soc, rc voltages (2), temperature
  std::size_t energy_offset_ = 0;
  std::size_t size_ = 0;

  // Initial conditions.
  double x0_ = 0.0, y0_ = 0.0, yaw0_ = 0.0, v0_ = 0.0;
  double soc0_ = 0.9, battery_temp0_ = 25.0, motor_temp0_ = 35.0;
  double kinetic0_ = 0.0;

  // Inputs, latched once per step.
  double steer_ = 0.0;
  double brake_ = 0.0;
  std::vector<double> torque_cmd_;

  std::unique_ptr<Signals> sig_;
};

}  // namespace vsim
