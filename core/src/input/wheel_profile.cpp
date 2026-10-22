#include "vsim/input/wheel_profile.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

std::string lower(std::string_view text) {
  std::string out(text);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

// "046d:c24f" -> (0x046d, 0xc24f)
std::pair<int, int> parse_usb_id(const std::string& text) {
  unsigned vendor = 0, product = 0;
  char tail = 0;
  if (std::sscanf(text.c_str(), "%x:%x%c", &vendor, &product, &tail) != 2 || vendor > 0xffff ||
      product > 0xffff) {
    throw std::invalid_argument("wheel profile: usb id '" + text + "' must look like 046d:c24f");
  }
  return {static_cast<int>(vendor), static_cast<int>(product)};
}

PedalAxis read_pedal(const ParamTree& params, const std::string& key) {
  PedalAxis pedal;
  pedal.axis = static_cast<int>(params.get_int(key + ".axis", -1));
  pedal.released = params.get_double(key + ".released", pedal.released);
  pedal.pressed = params.get_double(key + ".pressed", pedal.pressed);
  pedal.deadzone = params.get_double(key + ".deadzone", pedal.deadzone);
  pedal.gamma = params.get_double(key + ".gamma", pedal.gamma);
  if (pedal.axis >= 0 && pedal.released == pedal.pressed) {
    throw std::invalid_argument("wheel profile: " + key + ".released and .pressed must differ");
  }
  if (!(pedal.deadzone >= 0.0 && pedal.deadzone < 0.5 && pedal.gamma > 0.0)) {
    throw std::invalid_argument("wheel profile: " + key + " needs deadzone in [0, 0.5) and gamma > 0");
  }
  return pedal;
}

void write_pedal(std::ostream& out, const char* name, const PedalAxis& p) {
  out << "  " << name << ": {axis: " << p.axis << ", released: " << p.released
      << ", pressed: " << p.pressed << ", deadzone: " << p.deadzone << ", gamma: " << p.gamma << "}\n";
}

}  // namespace

double PedalAxis::value(double raw) const {
  if (axis < 0) {
    return 0.0;
  }
  const double travel = (raw - released) / (pressed - released);
  const double active = std::clamp((travel - deadzone) / (1.0 - 2.0 * deadzone), 0.0, 1.0);
  return gamma == 1.0 ? active : std::pow(active, gamma);
}

WheelProfile WheelProfile::from_params(const ParamTree& params) {
  WheelProfile p;
  p.name = params.get_string("name");
  p.priority = static_cast<int>(params.get_int("priority", 0));
  p.match_names = params.get_string_list("match.names");
  for (const std::string& id : params.get_string_list("match.usb_ids")) {
    p.usb_ids.push_back(parse_usb_id(id));
  }
  p.steer_axis = static_cast<int>(params.get_int("steer.axis", p.steer_axis));
  p.steer_range_deg = params.get_double("steer.range_deg", p.steer_range_deg);
  p.steer_center = params.get_double("steer.center", p.steer_center);
  p.steer_invert = params.get_bool("steer.invert", p.steer_invert);
  p.throttle = read_pedal(params, "pedals.throttle");
  p.brake = read_pedal(params, "pedals.brake");
  p.clutch = read_pedal(params, "pedals.clutch");
  p.button_reset = static_cast<int>(params.get_int("buttons.reset", -1));
  p.button_pause = static_cast<int>(params.get_int("buttons.pause", -1));

  ForceFeedbackSettings& f = p.ffb;
  p.ffb_enabled = params.get_bool("ffb.enabled", p.ffb_enabled);
  f.invert = params.get_bool("ffb.invert", f.invert);
  f.gain = params.get_double("ffb.gain", f.gain);
  f.max_torque = params.get_double("ffb.max_torque_nm", f.max_torque);
  f.damping = params.get_double("ffb.damping_nm_s_per_rad", f.damping);
  f.friction = params.get_double("ffb.friction_nm", f.friction);
  f.end_stop = params.get_double("ffb.end_stop_nm_per_rad", f.end_stop);
  f.timeout = params.get_double("ffb.timeout_s", f.timeout);
  f.fade_time = params.get_double("ffb.fade_s", f.fade_time);
  f.update_hz = params.get_double("ffb.update_hz", f.update_hz);

  if (p.steer_axis < 0 || !(p.steer_range_deg > 0.0)) {
    throw std::invalid_argument("wheel profile '" + p.name + "': steer.axis and steer.range_deg are required");
  }
  if (!(f.max_torque > 0.0 && f.update_hz > 0.0 && f.timeout > 0.0 && f.fade_time >= 0.0)) {
    throw std::invalid_argument("wheel profile '" + p.name +
                                "': ffb.max_torque_nm, update_hz and timeout_s must be positive");
  }
  return p;
}

WheelProfile WheelProfile::from_yaml_file(const std::filesystem::path& file) {
  try {
    return from_params(ParamTree::from_yaml_file(file));
  } catch (const std::exception& e) {
    throw std::invalid_argument(path_to_utf8(file) + ": " + e.what());
  }
}

std::vector<WheelProfile> WheelProfile::load_directory(const std::filesystem::path& dir) {
  std::vector<std::filesystem::path> files;
  std::error_code error;
  for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  std::vector<WheelProfile> profiles;
  for (const auto& file : files) {
    profiles.push_back(from_yaml_file(file));
  }
  std::stable_sort(profiles.begin(), profiles.end(),
                   [](const WheelProfile& a, const WheelProfile& b) { return a.priority > b.priority; });
  return profiles;
}

WheelProfile WheelProfile::logitech_g29() {
  // DirectInput order on Windows with separate pedals: X wheel, Y throttle,
  // Rz brake, slider clutch; pedals read +32767 released. Run
  // `vsim-input calibrate` if your setup differs.
  WheelProfile p;
  p.name = "Logitech G29";
  p.match_names = {"G29"};
  p.usb_ids = {{0x046d, 0xc24f}, {0x046d, 0xc260}};
  p.steer_axis = 0;
  p.steer_range_deg = 900.0;
  p.steer_invert = true;
  p.throttle.axis = 1;
  p.brake.axis = 2;
  p.clutch.axis = 3;
  return p;
}

std::string WheelProfile::to_yaml() const {
  std::ostringstream out;
  out.precision(10);
  out << "# vsim wheel profile (docs/build.md: steering wheel). Written by vsim-input calibrate.\n";
  out << "name: \"" << name << "\"\n";
  out << "priority: " << priority << "                  # higher wins when several profiles match\n";
  out << "match:\n  names: [";
  for (std::size_t i = 0; i < match_names.size(); ++i) {
    out << (i ? ", " : "") << '"' << match_names[i] << '"';
  }
  out << "]\n  usb_ids: [";
  for (std::size_t i = 0; i < usb_ids.size(); ++i) {
    char id[16];
    std::snprintf(id, sizeof(id), "%04x:%04x", usb_ids[i].first, usb_ids[i].second);
    out << (i ? ", " : "") << '"' << id << '"';
  }
  out << "]\n";
  out << "steer:\n  axis: " << steer_axis << "\n  range_deg: " << steer_range_deg
      << "            # lock to lock, as set in the vendor software (G HUB)\n  center: " << steer_center
      << "\n  invert: " << (steer_invert ? "true" : "false") << "\n";
  out << "pedals:\n";
  write_pedal(out, "throttle", throttle);
  write_pedal(out, "brake", brake);
  write_pedal(out, "clutch", clutch);
  out << "buttons:\n  reset: " << button_reset << "\n  pause: " << button_pause << "\n";
  out << "ffb:\n  enabled: " << (ffb_enabled ? "true" : "false") << "\n  invert: "
      << (ffb.invert ? "true" : "false") << "\n  gain: " << ffb.gain
      << "              # wheel torque per car steering torque\n  max_torque_nm: " << ffb.max_torque
      << "\n  damping_nm_s_per_rad: " << ffb.damping << "\n  friction_nm: " << ffb.friction
      << "\n  end_stop_nm_per_rad: " << ffb.end_stop << "\n  timeout_s: " << ffb.timeout
      << "\n  fade_s: " << ffb.fade_time << "\n  update_hz: " << ffb.update_hz << "\n";
  return out.str();
}

bool WheelProfile::matches(std::string_view device_name, int vendor, int product) const {
  for (const auto& [v, p] : usb_ids) {
    if (v == vendor && p == product) {
      return true;
    }
  }
  const std::string device = lower(device_name);
  for (const std::string& part : match_names) {
    if (!part.empty() && device.find(lower(part)) != std::string::npos) {
      return true;
    }
  }
  return false;
}

double WheelProfile::steer_angle(double raw) const {
  const double half_range = 0.5 * steer_range_deg * std::numbers::pi / 180.0;
  const double angle = (raw - steer_center) / 32768.0 * half_range;
  return steer_invert ? -angle : angle;
}

const WheelProfile* find_profile(const std::vector<WheelProfile>& profiles, std::string_view device_name,
                                 int vendor, int product) {
  for (const WheelProfile& profile : profiles) {
    if (profile.matches(device_name, vendor, product)) {
      return &profile;
    }
  }
  return nullptr;
}

}  // namespace vsim
