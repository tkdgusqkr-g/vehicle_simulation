#pragma once

namespace vsim {

struct ForceFeedbackSettings {
  double gain = 0.15;          // device torque per car steering torque
  double max_torque = 2.2;     // [N m] the device's peak torque (G29 ~2.2)
  double damping = 0.03;       // [N m s/rad] at the device
  double friction = 0.05;      // [N m] Coulomb-like, smoothed
  double end_stop = 8.0;       // [N m/rad] past the steering lock
  double timeout = 0.2;        // [s] without updates the road torque fades out
  double fade_time = 0.3;      // [s] fade in/out of the road torque
  bool invert = true;          // flips the device direction (set by calibration)
  double update_hz = 500.0;    // device update rate
};

// Turns the road torque at the car's steering wheel into a device command.
// Damping, friction and the end stop are computed locally from the wheel
// angle at the input rate, so they stay stable even when the simulation runs
// slower or pauses; the road torque fades in and out instead of jumping.
class ForceFeedbackShaper {
 public:
  explicit ForceFeedbackShaper(const ForceFeedbackSettings& settings) : s_(settings) {}

  // `angle` is the steering wheel angle [rad, positive left], `road_torque`
  // the car's steering wheel torque [N m, positive counter-clockwise] and
  // `fresh` whether it was updated within the timeout. Returns the device
  // command in [-1, 1] in device direction (inversion applied).
  double update(double dt, double angle, double road_torque, double lock, bool fresh);

  void reset();
  double torque() const { return torque_; }   // last device torque [N m], vsim direction
  double fade() const { return fade_; }
  bool clipping() const { return clipping_; }
  double angular_velocity() const { return velocity_; }

 private:
  ForceFeedbackSettings s_;
  bool has_angle_ = false;
  double last_angle_ = 0.0;
  double velocity_ = 0.0;
  double fade_ = 0.0;
  double torque_ = 0.0;
  bool clipping_ = false;
};

}  // namespace vsim
