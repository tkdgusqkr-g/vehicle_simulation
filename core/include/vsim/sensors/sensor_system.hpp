#pragma once

#include <memory>
#include <string>
#include <vector>

#include "vsim/sim/block.hpp"

namespace vsim {

// The car's sensors (design doc §10.5): physical values at each sensor's
// mount, through its error chain, sampled at its own rate. Defined by
// vehicle.sensors (the vehicle builder places them):
//
//   - {name: imu, type: imu, position: [x, y, z], orientation_deg: [roll, pitch, yaw], rate_hz: 500}
//   - {name: ws_fl, type: wheel_speed, wheel: fl, teeth: 48}
//
// Types and outputs (sens.<name>.<channel>):
//   imu            ax, ay, az [m/s^2] specific force at the mount (gravity,
//                  lever arm, misalignment, motor-order vibration), gx, gy, gz [rad/s]
//   accelerometer  ax, ay, az        gyro  gx, gy, gz
//   wheel_speed    omega [rad/s] from toothed-wheel pulse timing, pulses
//   steering_angle angle [rad]       pedal  ch1, ch2 [V] (accelerator, two tracks)
//   brake_pressure pressure [bar]    gnss   x, y [m], speed [m/s], heading [rad], valid
//   current, voltage, temperature    value     suspension_travel  travel [m]
//
// realism.sensors: ideal -> the true value at the sample instants (no noise,
// delay, quantization, range limits, pulses); real -> the full chain. Faults
// (vehicle.sensors.<i>.faults) apply in both. realism.variation: per-unit
// scale, bias, mounting and tooth spacing errors.
class SensorSystem final : public Block {
 public:
  SensorSystem();
  ~SensorSystem() override;

  std::string_view name() const override { return "sensors"; }
  void init(const ParamTree& params, Context& ctx) override;
  double period() const override { return period_; }
  void step(double t, SignalBus& bus) override;

  class Sensor;

  // True if the scenario's vehicle lists any sensors.
  static bool configured(const ParamTree& params);

  // The listed sensors: name, type, and the wheel corner of wheel_speed and
  // suspension_travel sensors (from `wheel`, else their position), or -1.
  struct Entry {
    std::string name, type;
    int corner = -1;
  };
  static std::vector<Entry> entries(const ParamTree& params);

 private:
  double period_ = 0.001;
  std::vector<std::unique_ptr<Sensor>> sensors_;
};

}  // namespace vsim
