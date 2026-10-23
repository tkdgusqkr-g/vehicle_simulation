// The Unreal viewer's procedural car sound (ui/unreal/.../Public/vsim_car_sound.h),
// standard C++ only, so it is tested here with every compiler CI has.
#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

#include "vsim_car_sound.h"

namespace {

using vsim_sound::CarSynth;
using vsim_sound::Input;

constexpr float kRate = 48000.0f;

std::vector<float> render(CarSynth& synth, const Input& in, float seconds) {
  std::vector<float> out(static_cast<std::size_t>(seconds * kRate));
  // In buffers like an audio device asks for them.
  for (std::size_t i = 0; i < out.size(); i += 512) {
    const int n = static_cast<int>(std::min<std::size_t>(512, out.size() - i));
    synth.Render(in, out.data() + i, n);
  }
  return out;
}

double rms(const std::vector<float>& x, std::size_t from = 0, std::size_t to = 0) {
  to = to == 0 ? x.size() : to;
  double sum = 0.0;
  for (std::size_t i = from; i < to; ++i) {
    sum += double(x[i]) * x[i];
  }
  return std::sqrt(sum / double(to - from));
}

// Power at one frequency (Goertzel), normalised by the length.
double tone_power(const std::vector<float>& x, double hz) {
  const double w = 2.0 * std::numbers::pi * hz / kRate, c = 2.0 * std::cos(w);
  double s1 = 0.0, s2 = 0.0;
  for (const float v : x) {
    const double s0 = v + c * s1 - s2;
    s2 = s1;
    s1 = s0;
  }
  const double n = double(x.size());
  return (s1 * s1 + s2 * s2 - c * s1 * s2) / (n * n);
}

Input motor_only(float rotor_hz) {
  Input in;
  in.rotor_hz = rotor_hz;
  in.load = 1.0f;
  in.active = 1.0f;
  return in;
}

TEST(CarSound, SilentWhilePaused) {
  CarSynth synth(kRate);
  Input in = motor_only(150.0f);
  in.speed = 25.0f;
  in.slide = 1.0f;
  in.kerb = 1.0f;
  in.active = 0.0f;
  for (const float v : render(synth, in, 0.3f)) {
    ASSERT_EQ(v, 0.0f);
  }
}

TEST(CarSound, StaysInRangeAtTheExtremes) {
  CarSynth synth(kRate);
  Input in = motor_only(2000.0f);  // far past the whine's top pitch
  in.speed = 80.0f;
  in.slide = 1.0f;
  in.tire_load = 6000.0f;
  in.kerb = 1.0f;
  const std::vector<float> out = render(synth, in, 1.0f);
  for (const float v : out) {
    ASSERT_TRUE(std::isfinite(v));
    ASSERT_LE(std::abs(v), 1.0f);
  }
  EXPECT_GT(rms(out), 0.1);
}

TEST(CarSound, WhinePitchFollowsTheRotor) {
  // 10th order whine: 100 rev/s -> 1000 Hz, 130 rev/s -> 1300 Hz.
  for (const float rotor : {100.0f, 130.0f}) {
    CarSynth synth(kRate);
    std::vector<float> out = render(synth, motor_only(rotor), 0.5f);
    out.erase(out.begin(), out.begin() + out.size() / 2);  // after the glide
    const double own = tone_power(out, 10.0 * rotor);
    const double other = tone_power(out, 10.0 * (rotor == 100.0f ? 130.0f : 100.0f));
    EXPECT_GT(own, 100.0 * other) << rotor;
  }
}

TEST(CarSound, LouderWithSpeedAndWhenTheTyresSlide) {
  auto level = [](float speed, float slide) {
    CarSynth synth(kRate);
    Input in;
    in.active = 1.0f;
    in.speed = speed;
    in.slide = slide;
    in.tire_load = 2000.0f;
    const std::vector<float> out = render(synth, in, 0.5f);
    return rms(out, out.size() / 2);
  };
  EXPECT_GT(level(30.0f, 0.0f), 2.0 * level(5.0f, 0.0f));
  EXPECT_GT(level(15.0f, 1.0f), 2.0 * level(15.0f, 0.0f));
}

TEST(CarSound, GlidesInsteadOfClicking) {
  CarSynth synth(kRate);
  const std::vector<float> out = render(synth, motor_only(120.0f), 0.3f);
  // 40 ms glide: the first 2 ms are far quieter than the settled sound.
  EXPECT_LT(rms(out, 0, 96), 0.2 * rms(out, out.size() / 2));
}

TEST(CarSound, InputFromTheVehicleState) {
  vsim_link::VehicleState s{};
  s.info.gear_ratio = 12.0f;
  for (float& w : s.wheel_omega) {
    w = 50.0f;
  }
  s.power_kw = -20.0f;  // regenerating counts as load too
  s.speed = 10.0;
  s.wheel_slip_angle[2] = 0.02f;  // within the grip peak
  Input in = vsim_sound::FromState(s);
  EXPECT_NEAR(in.rotor_hz, 50.0f * 12.0f / (2.0f * std::numbers::pi_v<float>), 1e-3f);
  EXPECT_FLOAT_EQ(in.load, 0.5f);
  EXPECT_FLOAT_EQ(in.slide, 0.0f);
  EXPECT_FLOAT_EQ(in.active, 1.0f);
  EXPECT_FLOAT_EQ(in.kerb, 0.0f);

  s.wheel_slip_angle[2] = -0.11f;  // past the peak, the sign does not matter
  s.wheel_load[2] = 1800.0f;
  s.paused = 1;
  s.info.gear_ratio = 0.0f;  // unknown: 1
  in = vsim_sound::FromState(s, true);
  EXPECT_NEAR(in.slide, 0.5f, 1e-5f);
  EXPECT_FLOAT_EQ(in.tire_load, 1800.0f);
  EXPECT_NEAR(in.rotor_hz, 50.0f / (2.0f * std::numbers::pi_v<float>), 1e-4f);
  EXPECT_FLOAT_EQ(in.active, 0.0f);
  EXPECT_FLOAT_EQ(in.kerb, 1.0f);
}

}  // namespace
