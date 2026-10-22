#include "vsim/input/force_feedback.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace vsim {
namespace {

// Low-pass on the differentiated wheel angle (encoder steps are coarse).
constexpr double kVelocityCutoffHz = 40.0;
// Speed over which friction builds up to its full value [rad/s].
constexpr double kFrictionSmoothing = 0.3;

}  // namespace

void ForceFeedbackShaper::reset() {
  has_angle_ = false;
  velocity_ = 0.0;
  fade_ = 0.0;
  torque_ = 0.0;
  clipping_ = false;
}

double ForceFeedbackShaper::update(double dt, double angle, double road_torque, double lock, bool fresh) {
  if (has_angle_ && dt > 0.0) {
    const double raw = (angle - last_angle_) / dt;
    const double alpha = 1.0 - std::exp(-2.0 * std::numbers::pi * kVelocityCutoffHz * dt);
    velocity_ += alpha * (raw - velocity_);
  }
  last_angle_ = angle;
  has_angle_ = true;

  const double step = s_.fade_time > 0.0 ? dt / s_.fade_time : 1.0;
  fade_ = std::clamp(fade_ + (fresh ? step : -step), 0.0, 1.0);

  double torque = fade_ * s_.gain * road_torque;
  torque -= s_.damping * velocity_;
  torque -= s_.friction * std::tanh(velocity_ / kFrictionSmoothing);
  if (lock > 0.0) {
    if (angle > lock) {
      torque -= s_.end_stop * (angle - lock);
    } else if (angle < -lock) {
      torque -= s_.end_stop * (angle + lock);
    }
  }
  torque_ = torque;

  const double level = s_.max_torque > 0.0 ? torque / s_.max_torque : 0.0;
  clipping_ = std::abs(level) > 1.0;
  const double clamped = std::clamp(level, -1.0, 1.0);
  return s_.invert ? -clamped : clamped;
}

}  // namespace vsim
