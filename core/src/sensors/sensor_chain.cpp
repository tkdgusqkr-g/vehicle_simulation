#include "vsim/sensors/sensor_chain.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace vsim {

ChannelParams ChannelParams::from_params(const ParamTree& params, const std::string& prefix, ChannelParams p) {
  const auto get = [&](const char* key, double& value) { value = params.get_double(prefix + "." + key, value); };
  get("scale_error", p.scale_error);
  get("bias", p.bias);
  get("noise_density", p.noise_density);
  get("bias_instability", p.bias_instability);
  get("bias_tau_s", p.bias_tau);
  get("random_walk", p.random_walk);
  get("bandwidth_hz", p.bandwidth);
  get("latency_s", p.latency);
  get("resolution", p.resolution);
  const std::vector<double> range = params.get_double_list(prefix + ".range");
  if (range.size() == 2) {
    p.min = range[0];
    p.max = range[1];
  } else if (!range.empty()) {
    throw std::invalid_argument(prefix + ".range needs [min, max]");
  }
  for (const double v : {p.scale_error, p.bias, p.noise_density, p.bias_instability, p.random_walk, p.bandwidth,
                         p.latency, p.resolution}) {
    if (!(v >= 0.0)) {
      throw std::invalid_argument(prefix + ": error parameters must not be negative");
    }
  }
  if (!(p.bias_tau > 0.0) || !(p.max > p.min)) {
    throw std::invalid_argument(prefix + ": bias_tau_s must be positive and range increasing");
  }
  return p;
}

ChannelModel::ChannelModel(const ChannelParams& params, double rate, Rng noise, Rng instance, bool real,
                           bool variation)
    : p_(params), dt_(1.0 / rate), real_(real), noise_(noise) {
  if (variation) {
    scale_ = p_.scale_error * instance.normal();
    bias_ = p_.bias * instance.normal();
  }
  if (real_) {
    // Drifting bias as a first-order Gauss-Markov process: its Allan
    // deviation peaks at about 0.62 sigma near tau = 1.9 tau_c, so sigma is
    // set to put that peak at the datasheet's bias instability floor.
    const double sigma = p_.bias_instability * 0.664 / 0.62;
    markov_decay_ = std::exp(-dt_ / p_.bias_tau);
    markov_kick_ = sigma * std::sqrt(1.0 - markov_decay_ * markov_decay_);
    markov_ = sigma * noise_.normal();
    walk_kick_ = p_.random_walk * std::sqrt(dt_);
    white_ = p_.noise_density * std::sqrt(rate);
    if (p_.bandwidth > 0.0) {
      filter_gain_ = 1.0 - std::exp(-2.0 * std::numbers::pi * p_.bandwidth * dt_);
    }
    delay_samples_ = static_cast<std::size_t>(std::lround(p_.latency * rate));
  }
}

double ChannelModel::sample(double t, double truth) {
  double y = truth;
  if (real_) {
    filtered_ = started_ ? filtered_ + filter_gain_ * (truth - filtered_) : truth;
    started_ = true;
    y = filtered_;
  }
  y = y * (1.0 + scale_) + bias_;
  if (real_) {
    markov_ = markov_decay_ * markov_ + markov_kick_ * noise_.normal();
    walk_ += walk_kick_ * noise_.normal();
    y += markov_ + walk_ + white_ * noise_.normal();
    if (p_.resolution > 0.0) {
      y = std::round(y / p_.resolution) * p_.resolution;
    }
    if (delay_samples_ > 0) {
      delay_.push_back(y);
      if (delay_.size() <= delay_samples_) {
        y = delay_.front();  // the first output repeats until the pipeline fills
      } else {
        y = delay_.front();
        delay_.pop_front();
      }
    }
    y = std::clamp(y, p_.min, p_.max);
  }

  if (stuck_.size() != faults_.size()) {
    stuck_.assign(faults_.size(), std::numeric_limits<double>::quiet_NaN());
  }
  for (std::size_t i = 0; i < faults_.size(); ++i) {
    const ChannelFault& f = faults_[i];
    if (t < f.at || (f.until >= 0.0 && t >= f.until)) {
      stuck_[i] = std::numeric_limits<double>::quiet_NaN();
      continue;
    }
    switch (f.kind) {
      case ChannelFault::Kind::Offset:
        y += f.value;
        break;
      case ChannelFault::Kind::Stuck:
        if (std::isnan(stuck_[i])) {
          stuck_[i] = output_;
        }
        y = stuck_[i];
        break;
      case ChannelFault::Kind::Value:
        y = f.value;
        break;
      case ChannelFault::Kind::Noise:
        y += f.value * noise_.normal();
        break;
      case ChannelFault::Kind::Scale:
        y *= f.value;
        break;
    }
  }
  output_ = y;
  return y;
}

std::vector<ChannelFault> faults_from_params(const ParamTree& params, const std::string& prefix,
                                             std::string_view channel, double supply) {
  std::vector<ChannelFault> out;
  for (std::size_t i = 0;; ++i) {
    const std::string base = prefix + ".faults." + std::to_string(i);
    if (!params.has_prefix(base)) {
      break;
    }
    const std::string target = params.get_string(base + ".channel", "");
    if (!target.empty() && target != channel) {
      continue;
    }
    const std::string type = params.get_string(base + ".type");
    ChannelFault f;
    f.at = params.get_double(base + ".at_s", 0.0);
    f.until = params.get_double(base + ".until_s", -1.0);
    f.value = params.get_double(base + ".value", 0.0);
    if (type == "offset") {
      f.kind = ChannelFault::Kind::Offset;
    } else if (type == "stuck") {
      f.kind = ChannelFault::Kind::Stuck;
    } else if (type == "open") {
      f.kind = ChannelFault::Kind::Value;
      f.value = 0.0;
    } else if (type == "short") {
      f.kind = ChannelFault::Kind::Value;
      f.value = supply;
    } else if (type == "value") {
      f.kind = ChannelFault::Kind::Value;
    } else if (type == "noise") {
      f.kind = ChannelFault::Kind::Noise;
    } else if (type == "scale") {
      f.kind = ChannelFault::Kind::Scale;
    } else {
      throw std::invalid_argument(base + ".type '" + type +
                                  "' (expected offset, stuck, open, short, value, noise or scale)");
    }
    out.push_back(f);
  }
  return out;
}

}  // namespace vsim
