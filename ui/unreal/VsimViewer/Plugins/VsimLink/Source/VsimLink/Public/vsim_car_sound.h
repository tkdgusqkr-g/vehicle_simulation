// Procedural sound of an electric formula student car for the Unreal viewer
// (design doc §11, M11e): motor whine (an order of the rotor speed, its second
// harmonic, inverter switching), gear mesh, tyre squeal past the grip peak,
// road roar, wind and kerb rumble, all from the published vehicle state. The
// same recipe as the browser view's CarSound (ui/view3d/index.html).
//
// Standard C++ only, no engine types: UVsimCarSound feeds it on the audio
// thread, tests/test_car_sound.cpp checks it without the engine.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "vsim_link.h"

namespace vsim_sound {

// What the sound follows, from one vehicle state.
struct Input {
  float rotor_hz = 0.0f;   // motor revolutions per second
  float load = 0.0f;       // 0..1 of the drive power
  float speed = 0.0f;      // [m/s]
  float slide = 0.0f;      // 0..1 tyre slip beyond the grip peak (worst tyre)
  float tire_load = 0.0f;  // [N] on that tyre
  float kerb = 0.0f;       // 1 while a wheel is on a kerb
  float active = 0.0f;     // 0 while paused or without a state
};

inline Input FromState(const vsim_link::VehicleState& s, bool on_kerb = false) {
  Input in;
  float wheel = 0.0f;
  for (const float w : s.wheel_omega) {
    wheel += std::fabs(w) / 4.0f;
  }
  const float ratio = s.info.gear_ratio > 0.0f ? s.info.gear_ratio : 1.0f;
  in.rotor_hz = wheel * ratio / (2.0f * std::numbers::pi_v<float>);
  in.load = std::min(1.0f, std::fabs(s.power_kw) / 40.0f);
  in.speed = static_cast<float>(s.speed);
  float slide = 0.0f, fz = 0.0f;
  for (int c = 0; c < 4; ++c) {
    const float w = std::max(0.0f, std::fabs(s.wheel_slip_angle[c]) - 0.05f) / 0.12f +
                    std::max(0.0f, std::fabs(s.wheel_slip_ratio[c]) - 0.06f) / 0.2f;
    if (w > slide) {
      slide = w;
      fz = s.wheel_load[c];
    }
  }
  in.slide = std::min(1.0f, slide) * std::min(1.0f, in.speed / 4.0f);
  in.tire_load = fz;
  in.kerb = on_kerb ? 1.0f : 0.0f;
  in.active = s.paused ? 0.0f : 1.0f;
  return in;
}

// Topology-preserving state variable filter (Zavalishin), stable for any
// cut-off below Nyquist and while the cut-off moves.
class Svf {
 public:
  void Set(float cutoff, float q, float sample_rate) {
    const float fc = std::clamp(cutoff, 10.0f, 0.45f * sample_rate);
    const float g = std::tan(std::numbers::pi_v<float> * fc / sample_rate);
    k_ = 1.0f / q;
    a1_ = 1.0f / (1.0f + g * (g + k_));
    a2_ = g * a1_;
    a3_ = g * a2_;
  }
  // Returns the low-pass output; band (peak gain 1) in `band`.
  float Process(float x, float& band) {
    const float v3 = x - ic2_;
    const float v1 = a1_ * ic1_ + a2_ * v3;
    const float v2 = ic2_ + a2_ * ic1_ + a3_ * v3;
    ic1_ = 2.0f * v1 - ic1_;
    ic2_ = 2.0f * v2 - ic2_;
    band = k_ * v1;
    return v2;
  }
  float Low(float x) {
    float band;
    return Process(x, band);
  }
  float Band(float x) {
    float band;
    Process(x, band);
    return band;
  }

 private:
  float k_ = 1.0f, a1_ = 1.0f, a2_ = 0.0f, a3_ = 0.0f, ic1_ = 0.0f, ic2_ = 0.0f;
};

class CarSynth {
 public:
  explicit CarSynth(float sample_rate = 48000.0f) { SetSampleRate(sample_rate); }

  void SetSampleRate(float sample_rate) { rate_ = sample_rate > 1000.0f ? sample_rate : 48000.0f; }
  float SampleRate() const { return rate_; }

  // Renders `count` mono samples in [-1, 1] following `in`; parameters glide
  // over about 40 ms so steps in the state (60 Hz) do not click.
  void Render(const Input& in, float* out, int count) {
    Targets target = Aim(in);
    for (int start = 0; start < count; start += kBlock) {
      const int end = std::min(count, start + kBlock);
      Glide(target, static_cast<float>(end - start));
      UpdateFilters();
      const Targets& p = now_;
      const float whine_dt = p.whine_hz / rate_, whine2_dt = 2.0f * p.whine_hz / rate_;
      const float gear_dt = p.gear_hz / rate_, pwm_dt = kPwmHz / rate_;
      for (int i = start; i < end; ++i) {
        const float saw = Saw(whine_phase_, whine_dt);
        float sample = p.whine * whine_filter_.Band(saw);
        sample += p.whine2 * std::sin(2.0f * std::numbers::pi_v<float> * whine2_phase_);
        sample += p.gear * (4.0f * std::fabs(gear_phase_ - 0.5f) - 1.0f);
        sample += p.pwm * std::sin(2.0f * std::numbers::pi_v<float> * pwm_phase_);
        Advance(whine_phase_, whine_dt);
        Advance(whine2_phase_, whine2_dt);
        Advance(gear_phase_, gear_dt);
        Advance(pwm_phase_, pwm_dt);
        sample += p.squeal * squeal_filter_.Band(Noise());
        sample += p.road * road_filter_.Low(Noise());
        sample += p.wind * wind_filter_.Band(Noise());
        sample += p.kerb * kerb_filter_.Low(Noise());
        // Soft limiter in place of the browser's compressor.
        out[i] = std::tanh(1.4f * sample) / std::tanh(1.4f);
        out[i] = std::clamp(out[i], -1.0f, 1.0f);
      }
    }
  }

 private:
  static constexpr int kBlock = 32;
  static constexpr float kPwmHz = 8000.0f;

  // Gains and pitches, the targets of the glide.
  struct Targets {
    float whine = 0, whine_hz = 100, whine2 = 0, gear = 0, gear_hz = 400, pwm = 0;
    float squeal = 0, squeal_hz = 900, road = 0, road_hz = 350, wind = 0, wind_hz = 500, kerb = 0;
  };

  static Targets Aim(const Input& in) {
    Targets t;
    const float on = in.active > 0.0f ? 1.0f : 0.0f;
    // Motor: electromagnetic whine (10th order) and its second harmonic, gear mesh (23 teeth), switching.
    t.whine_hz = std::clamp(in.rotor_hz * 10.0f, 20.0f, 5000.0f);
    t.whine = on * std::min(1.0f, in.rotor_hz / 20.0f) * (0.04f + 0.22f * in.load);
    t.whine2 = on * std::min(1.0f, in.rotor_hz / 30.0f) * 0.06f * in.load;
    t.gear_hz = std::clamp(in.rotor_hz * 23.0f, 30.0f, 9000.0f);
    t.gear = on * std::min(1.0f, in.rotor_hz / 40.0f) * (0.015f + 0.05f * in.load);
    t.pwm = on * 0.012f * in.load;
    // Tyres squeal past the grip peak, higher with the load.
    t.squeal = on * 0.8f * in.slide * in.slide;
    t.squeal_hz = 750.0f + 350.0f * std::min(1.0f, in.tire_load / 2500.0f) + 150.0f * in.slide;
    // Road roar and wind grow with speed; kerbs rumble.
    t.road = on * std::min(0.25f, 0.008f * in.speed);
    t.road_hz = 200.0f + 10.0f * in.speed;
    t.wind = on * std::min(0.3f, 0.00035f * in.speed * in.speed);
    t.wind_hz = 300.0f + 25.0f * in.speed;
    t.kerb = on * (in.kerb > 0.0f ? std::min(0.6f, 0.05f * in.speed) : 0.0f);
    return t;
  }

  void Glide(const Targets& to, float samples) {
    const float a = 1.0f - std::exp(-samples / (0.04f * rate_));
    const float kerb = 1.0f - std::exp(-samples / (0.02f * rate_));
    auto move = [](float& v, float target, float f) { v += f * (target - v); };
    move(now_.whine, to.whine, a);
    move(now_.whine_hz, to.whine_hz, a);
    move(now_.whine2, to.whine2, a);
    move(now_.gear, to.gear, a);
    move(now_.gear_hz, to.gear_hz, a);
    move(now_.pwm, to.pwm, a);
    move(now_.squeal, to.squeal, a);
    move(now_.squeal_hz, to.squeal_hz, a);
    move(now_.road, to.road, a);
    move(now_.road_hz, to.road_hz, a);
    move(now_.wind, to.wind, a);
    move(now_.wind_hz, to.wind_hz, a);
    move(now_.kerb, to.kerb, kerb);
  }

  void UpdateFilters() {
    whine_filter_.Set(1.5f * now_.whine_hz, 2.0f, rate_);
    squeal_filter_.Set(now_.squeal_hz, 6.0f, rate_);
    road_filter_.Set(now_.road_hz, 0.707f, rate_);
    wind_filter_.Set(now_.wind_hz, 0.5f, rate_);
    kerb_filter_.Set(120.0f, 1.0f, rate_);
  }

  // Band-limited sawtooth (polyBLEP), phase in [0, 1).
  static float Saw(float phase, float dt) {
    float blep = 0.0f;
    if (phase < dt) {
      const float t = phase / dt;
      blep = t + t - t * t - 1.0f;
    } else if (phase > 1.0f - dt) {
      const float t = (phase - 1.0f) / dt;
      blep = t * t + t + t + 1.0f;
    }
    return 2.0f * phase - 1.0f - blep;
  }

  static void Advance(float& phase, float dt) {
    phase += dt;
    phase -= std::floor(phase);
  }

  // White noise in [-1, 1) (xorshift32).
  float Noise() {
    noise_ ^= noise_ << 13;
    noise_ ^= noise_ >> 17;
    noise_ ^= noise_ << 5;
    return static_cast<float>(noise_) * (2.0f / 4294967296.0f) - 1.0f;
  }

  float rate_ = 48000.0f;
  Targets now_{};
  float whine_phase_ = 0, whine2_phase_ = 0, gear_phase_ = 0, pwm_phase_ = 0;
  Svf whine_filter_, squeal_filter_, road_filter_, wind_filter_, kerb_filter_;
  std::uint32_t noise_ = 0x9e3779b9u;
};

}  // namespace vsim_sound
