#include "vsim/models/powertrain/pmsm.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

namespace vsim {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kSqrt3 = 1.7320508075688772;
constexpr int kRk4PerPeriod = 2;
constexpr double kDeadTimeCurrent = 1.0;  // [A] the dead time effect rounds off through zero current
constexpr double kVoltageMargin = 0.95;   // references keep this share of the voltage for control

// Phase voltage errors of the dead time (each phase pulled against its current), in alpha-beta.
std::array<double, 2> dead_time_voltage(double volts, double theta, double id, double iq) {
  const double c = std::cos(theta), s = std::sin(theta);
  const double i_alpha = c * id - s * iq, i_beta = s * id + c * iq;
  const std::array<double, 3> phase = {i_alpha, -0.5 * i_alpha + 0.5 * kSqrt3 * i_beta,
                                       -0.5 * i_alpha - 0.5 * kSqrt3 * i_beta};
  std::array<double, 3> v{};
  for (int k = 0; k < 3; ++k) {
    v[k] = -volts * std::tanh(phase[k] / kDeadTimeCurrent);
  }
  return {(2.0 / 3.0) * (v[0] - 0.5 * v[1] - 0.5 * v[2]), (v[1] - v[2]) / kSqrt3};
}

}  // namespace

PmsmParams PmsmParams::from_params(const ParamTree& params, const MotorParams& motor) {
  const std::string b = "vehicle.powertrain.motor.pmsm.";
  PmsmParams p;
  p.pole_pairs = params.get_double(b + "pole_pairs", p.pole_pairs);
  p.resistance = params.get_double(b + "resistance_ohm", p.resistance);
  p.ld = 1e-3 * params.get_double(b + "ld_mh", p.ld * 1e3);
  p.lq = 1e-3 * params.get_double(b + "lq_mh", p.lq * 1e3);
  p.flux = params.get_double(b + "flux_linkage_wb", p.flux);
  p.current_max = params.get_double(b + "current_max_a", p.current_max);
  p.control_rate = params.get_double(b + "control_rate_hz", p.control_rate);
  p.bandwidth = params.get_double(b + "current_bandwidth_hz", p.bandwidth);
  p.dead_time = 1e-6 * params.get_double(b + "dead_time_us", p.dead_time * 1e6);
  p.dead_time_compensation = params.get_double(b + "dead_time_compensation", p.dead_time_compensation);
  p.resolver_error = params.get_double(b + "resolver.error_deg", 0.0) * kPi / 180.0;
  p.resolver_order = params.get_double(b + "resolver.order", p.resolver_order);
  p.conduction_loss = params.get_double(b + "inverter.conduction_w_per_a", p.conduction_loss);
  p.switching_loss = params.get_double(b + "inverter.switching_w_per_a", p.switching_loss);
  p.nominal_voltage = motor.nominal_voltage;
  if (!(p.pole_pairs >= 1.0 && p.resistance > 0.0 && p.ld > 0.0 && p.lq > 0.0 && p.flux > 0.0 &&
        p.current_max > 0.0)) {
    throw std::invalid_argument(b + "pole_pairs >= 1; resistance, inductances, flux and current_max positive");
  }
  if (!(p.control_rate >= 1000.0 && p.bandwidth > 0.0 && p.bandwidth <= p.control_rate / 8.0)) {
    throw std::invalid_argument(b + "control_rate_hz >= 1000 and current_bandwidth_hz up to an eighth of it");
  }
  if (!(p.dead_time >= 0.0 && p.dead_time * p.control_rate < 0.1 && p.dead_time_compensation >= 0.0 &&
        p.dead_time_compensation <= 1.0 && p.resolver_error >= 0.0 && p.conduction_loss >= 0.0 &&
        p.switching_loss >= 0.0)) {
    throw std::invalid_argument(b + "dead time (under a tenth of the period), compensation in [0, 1], losses >= 0");
  }
  return p;
}

PmsmDrive::PmsmDrive(const PmsmParams& params) : p_(params) {
  const double wc = 2.0 * kPi * p_.bandwidth;
  kp_d_ = p_.ld * wc;
  kp_q_ = p_.lq * wc;
  ki_ = p_.resistance * wc;
}

void PmsmDrive::reset() {
  id_ = iq_ = int_d_ = int_q_ = v_alpha_ = v_beta_ = 0.0;
  time_ = 0.0;
}

void PmsmDrive::references(double torque, double speed, double dc_voltage, double& id, double& iq) const {
  const double t = std::abs(torque), sign = torque < 0.0 ? -1.0 : 1.0;
  const double dl = p_.ld - p_.lq, imax = p_.current_max;
  // MTPA: the current angle that gives the most torque for a current magnitude.
  const auto mtpa_id = [&](double is) {
    return std::abs(dl) < 1e-12 ? 0.0 : (-p_.flux + std::sqrt(p_.flux * p_.flux + 8.0 * dl * dl * is * is)) / (4.0 * dl);
  };
  const auto mtpa_torque = [&](double is) {
    const double d = mtpa_id(is);
    return p_.torque(d, std::sqrt(std::max(0.0, is * is - d * d)));
  };
  double is = imax;
  if (mtpa_torque(imax) > t) {
    double lo = 0.0, hi = imax;
    for (int k = 0; k < 50; ++k) {
      is = 0.5 * (lo + hi);
      (mtpa_torque(is) < t ? lo : hi) = is;
    }
    is = 0.5 * (lo + hi);
  }
  id = mtpa_id(is);
  iq = std::sqrt(std::max(0.0, is * is - id * id));

  // Field weakening: the flux linkage the DC link can drive at this speed.
  const double we = p_.pole_pairs * std::abs(speed);
  if (we > 1e-6) {
    const double flux_max = std::max(0.0, (kVoltageMargin * dc_voltage / kSqrt3 - p_.resistance * is) / we);
    const auto flux_of = [&](double d, double q) { return std::hypot(p_.lq * q, p_.flux + p_.ld * d); };
    if (flux_of(id, iq) > flux_max) {
      const double room = flux_max * flux_max - p_.lq * p_.lq * iq * iq;
      id = std::max(-imax, room > 0.0 ? (std::sqrt(room) - p_.flux) / p_.ld : -p_.flux / p_.ld);
      id = std::min(id, 0.0);
      // Keep the torque if the current allows; else ride the current limit.
      const double for_torque = t / std::max(1e-9, 1.5 * p_.pole_pairs * (p_.flux + dl * id));
      iq = std::min(for_torque, std::sqrt(std::max(0.0, imax * imax - id * id)));
      if (flux_of(id, iq) > flux_max) {
        // Where the current circle meets the voltage ellipse:
        // (Ld^2 - Lq^2) d^2 + 2 psi Ld d + psi^2 + Lq^2 Imax^2 - flux_max^2 = 0.
        const double a = p_.ld * p_.ld - p_.lq * p_.lq, b = 2.0 * p_.flux * p_.ld;
        const double c = p_.flux * p_.flux + p_.lq * p_.lq * imax * imax - flux_max * flux_max;
        double d = std::abs(a) < 1e-15 ? -c / b : 0.0;
        if (std::abs(a) >= 1e-15) {
          const double disc = b * b - 4.0 * a * c;
          d = disc >= 0.0 ? (-b + std::sqrt(disc)) / (2.0 * a) : -imax;
          if (d > 0.0 || d < -imax) {
            d = disc >= 0.0 ? (-b - std::sqrt(disc)) / (2.0 * a) : -imax;
          }
        }
        id = std::clamp(d, -imax, 0.0);
        iq = std::min(iq, std::sqrt(std::max(0.0, imax * imax - id * id)));
        // Past the characteristic speed even that is too much flux: give up the torque.
        if (flux_of(id, iq) > flux_max) {
          iq = std::sqrt(std::max(0.0, flux_max * flux_max - std::pow(p_.flux + p_.ld * id, 2.0))) / p_.lq;
        }
      }
    }
  }
  if (t == 0.0) {
    iq = 0.0;
    id = std::min(id, 0.0);
  }
  iq *= sign;
}

PmsmDrive::Output PmsmDrive::step(const Input& in, double dt) {
  Output out;
  if (!(in.dc_voltage > 1.0)) {
    // No DC link: the bridge is off and the currents die out.
    reset();
    return out;
  }
  const int periods = std::max(1, static_cast<int>(std::lround(dt * p_.control_rate)));
  const double h = dt / periods, sub = h / kRk4PerPeriod;
  const double we = p_.pole_pairs * in.speed;
  const double v_max = in.dc_voltage / kSqrt3;
  const double v_dead = p_.dead_time * p_.control_rate * in.dc_voltage;
  double id_ref = 0.0, iq_ref = 0.0;
  references(in.torque, in.speed, in.dc_voltage, id_ref, iq_ref);
  const double magnetic0 = 0.75 * (p_.ld * id_ * id_ + p_.lq * iq_ * iq_);  // stored in the windings

  // Current derivatives in the rotor frame with the stationary voltage (v_alpha_, v_beta_) applied.
  const auto derivative = [&](double tau, double d, double q, double& dd, double& dq) {
    const double theta = p_.pole_pairs * (in.angle + in.speed * tau);
    const std::array<double, 2> dead = dead_time_voltage(v_dead, theta, d, q);
    const double va = v_alpha_ + dead[0], vb = v_beta_ + dead[1];
    const double c = std::cos(theta), s = std::sin(theta);
    const double vd = c * va + s * vb, vq = -s * va + c * vb;
    dd = (vd - p_.resistance * d + we * p_.lq * q) / p_.ld;
    dq = (vq - p_.resistance * q - we * (p_.ld * d + p_.flux)) / p_.lq;
  };

  double torque_sum = 0.0, copper_sum = 0.0, current_sum = 0.0;
  double vd_ctrl = 0.0, vq_ctrl = 0.0, id_meas = 0.0, iq_meas = 0.0;
  for (int k = 0; k < periods; ++k) {
    const double tau = k * h;
    // Sample: the controller sees the currents through the resolver's angle.
    const double theta_m = in.angle + in.speed * tau;
    const double theta = p_.pole_pairs * theta_m;
    const double error = p_.pole_pairs * p_.resolver_error * std::sin(p_.resolver_order * theta_m);
    const double theta_meas = theta + error;
    const double ce = std::cos(error), se = std::sin(error);
    id_meas = ce * id_ + se * iq_;
    iq_meas = -se * id_ + ce * iq_;
    // PI with decoupling; the result is applied over the next period.
    const double ed = id_ref - id_meas, eq = iq_ref - iq_meas;
    vd_ctrl = kp_d_ * ed + int_d_ - we * p_.lq * iq_meas;
    vq_ctrl = kp_q_ * eq + int_q_ + we * (p_.ld * id_meas + p_.flux);
    // Modulate at the angle the next period is centred on.
    const double lead = theta_meas + 1.5 * we * h;
    const double cl = std::cos(lead), sl = std::sin(lead);
    double va = cl * vd_ctrl - sl * vq_ctrl, vb = sl * vd_ctrl + cl * vq_ctrl;
    if (p_.dead_time_compensation > 0.0 && v_dead > 0.0) {
      const std::array<double, 2> comp = dead_time_voltage(v_dead, lead, id_meas, iq_meas);
      va -= p_.dead_time_compensation * comp[0];
      vb -= p_.dead_time_compensation * comp[1];
    }
    const double magnitude = std::hypot(va, vb);
    const bool saturated = magnitude > v_max;
    if (saturated) {
      va *= v_max / magnitude;
      vb *= v_max / magnitude;
    } else {
      int_d_ += ki_ * ed * h;
      int_q_ += ki_ * eq * h;
    }
    out.modulation = std::min(magnitude, v_max) / v_max;

    // This period: the voltage computed last period, the currents follow (RK4).
    for (int j = 0; j < kRk4PerPeriod; ++j) {
      const double t0 = tau + j * sub;
      const double before = p_.torque(id_, iq_);
      const double copper_before = 1.5 * p_.resistance * (id_ * id_ + iq_ * iq_);
      double k1d, k1q, k2d, k2q, k3d, k3q, k4d, k4q;
      derivative(t0, id_, iq_, k1d, k1q);
      derivative(t0 + 0.5 * sub, id_ + 0.5 * sub * k1d, iq_ + 0.5 * sub * k1q, k2d, k2q);
      derivative(t0 + 0.5 * sub, id_ + 0.5 * sub * k2d, iq_ + 0.5 * sub * k2q, k3d, k3q);
      derivative(t0 + sub, id_ + sub * k3d, iq_ + sub * k3q, k4d, k4q);
      id_ += sub / 6.0 * (k1d + 2.0 * k2d + 2.0 * k3d + k4d);
      iq_ += sub / 6.0 * (k1q + 2.0 * k2q + 2.0 * k3q + k4q);
      torque_sum += 0.5 * (before + p_.torque(id_, iq_)) * sub;
      copper_sum += 0.5 * (copper_before + 1.5 * p_.resistance * (id_ * id_ + iq_ * iq_)) * sub;
      current_sum += std::hypot(id_, iq_) * sub;
    }
    v_alpha_ = va;
    v_beta_ = vb;
    if (trace) {
      const double t_end = tau + h;
      const double th = p_.pole_pairs * (in.angle + in.speed * t_end);
      trace({time_ + t_end, p_.torque(id_, iq_), id_, iq_, std::cos(th) * id_ - std::sin(th) * iq_});
    }
  }
  time_ += dt;

  const double magnetic1 = 0.75 * (p_.ld * id_ * id_ + p_.lq * iq_ * iq_);
  out.torque = torque_sum / dt;
  const double inverter = (p_.conduction_loss + p_.switching_loss * in.dc_voltage / p_.nominal_voltage) * current_sum / dt;
  out.loss = copper_sum / dt + inverter;
  // Into the inverter: shaft power at the held speed, losses, and what the windings store.
  out.dc_power = out.torque * in.speed + out.loss + (magnetic1 - magnetic0) / dt;
  out.id = id_meas;
  out.iq = iq_meas;
  out.vd = vd_ctrl;
  out.vq = vq_ctrl;
  out.id_ref = id_ref;
  out.iq_ref = iq_ref;
  return out;
}

}  // namespace vsim
