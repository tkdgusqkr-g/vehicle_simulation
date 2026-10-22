#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "vsim/input/force_feedback.hpp"
#include "vsim/sim/params.hpp"

namespace vsim {

// How one raw axis (-32768..32767) maps to a pedal travel 0..1.
struct PedalAxis {
  int axis = -1;             // -1: not present
  double released = 32767.0;  // raw value at rest
  double pressed = -32768.0;  // raw value fully pressed
  double deadzone = 0.02;     // ignored travel at each end (fraction)
  double gamma = 1.0;         // >1 makes the start of the travel finer

  double value(double raw) const;
};

// Mapping and force feedback settings for one wheel model (design doc §9.4),
// stored as YAML under config/input/. `vsim-input calibrate` writes them.
struct WheelProfile {
  std::string name;
  // A device matches if its USB ids are listed or its name contains one of
  // the names (case-insensitive).
  std::vector<std::string> match_names;
  std::vector<std::pair<int, int>> usb_ids;  // (vendor, product)

  int steer_axis = 0;
  double steer_range_deg = 900.0;  // lock to lock, as set in the vendor tool
  double steer_center = 0.0;       // raw value with the wheel straight
  bool steer_invert = true;        // SDL axes are negative to the left; vsim is positive
  PedalAxis throttle;
  PedalAxis brake;
  PedalAxis clutch;
  int button_reset = -1;
  int button_pause = -1;

  bool ffb_enabled = true;
  ForceFeedbackSettings ffb;

  static WheelProfile from_params(const ParamTree& params);
  static WheelProfile from_yaml_file(const std::filesystem::path& file);
  // Profiles in a directory (*.yaml), sorted by file name.
  static std::vector<WheelProfile> load_directory(const std::filesystem::path& dir);
  // Built-in defaults, used when no profile file matches.
  static WheelProfile logitech_g29();

  std::string to_yaml() const;
  bool matches(std::string_view device_name, int vendor, int product) const;
  double steer_angle(double raw) const;  // [rad], positive left
};

// Picks the first matching profile, or nullptr.
const WheelProfile* find_profile(const std::vector<WheelProfile>& profiles, std::string_view device_name,
                                 int vendor, int product);

}  // namespace vsim
