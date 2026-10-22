#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "vsim/models/tire/tire.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

constexpr double kEpsilon = 1e-6;
// Tangent of the slip angle is used in the formulas; keep it finite.
constexpr double kMaxTanAlpha = 20.0;

double sign(double x) { return x > 0.0 ? 1.0 : (x < 0.0 ? -1.0 : 0.0); }

// D sin(C atan(B x - E (B x - atan(B x))))
double mf(double x, double b, double c, double d, double e) {
  const double bx = b * x;
  return d * std::sin(c * std::atan(bx - e * (bx - std::atan(bx))));
}

// cos(C atan(B x - E (B x - atan(B x)))), the weighting shape
double weight(double x, double b, double c, double e) {
  const double bx = b * x;
  return std::cos(c * std::atan(bx - e * (bx - std::atan(bx))));
}

std::string trim(std::string text) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

}  // namespace

std::map<std::string, double, std::less<>> read_tir_file(const std::filesystem::path& file) {
  std::ifstream in(file);
  if (!in) {
    throw std::invalid_argument("cannot open tire file '" + path_to_utf8(file) + "'");
  }
  std::map<std::string, double, std::less<>> values;
  std::string line;
  while (std::getline(in, line)) {
    if (const auto comment = line.find_first_of("$!"); comment != std::string::npos) {
      line.erase(comment);
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;  // [SECTION] headers and blank lines
    }
    std::string key = trim(line.substr(0, eq));
    std::string value = trim(line.substr(eq + 1));
    if (key.empty() || value.empty() || value.front() == '\'' || value.front() == '"') {
      continue;
    }
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
    std::istringstream number(value);
    double parsed = 0.0;
    if (number >> parsed) {
      values[key] = parsed;
    }
  }
  return values;
}

MagicFormulaTire MagicFormulaTire::from_file(const std::filesystem::path& file) {
  try {
    return MagicFormulaTire(read_tir_file(file));
  } catch (const std::invalid_argument& e) {
    throw std::invalid_argument(path_to_utf8(file) + ": " + e.what());
  }
}

MagicFormulaTire::MagicFormulaTire(std::map<std::string, double, std::less<>> coefficients)
    : k_(std::move(coefficients)) {
  fz0_ = c("FNOMIN") * c("LFZO");
  r0_ = c("UNLOADED_RADIUS");
  if (!(fz0_ > 0.0) || !(r0_ > 0.0)) {
    throw std::invalid_argument("tire file needs positive FNOMIN and UNLOADED_RADIUS");
  }
  if (c("PCX1") <= 0.0 || c("PDX1") <= 0.0 || c("PCY1") <= 0.0 || c("PDY1") == 0.0 || c("PKY1") == 0.0) {
    throw std::invalid_argument("tire file lacks the basic coefficients (PCX1, PDX1, PCY1, PDY1, PKY1)");
  }
  if (const double fittyp = coefficient("FITTYP"); fittyp != 0.0 && fittyp != 6.0 && fittyp != 21.0 &&
                                                   fittyp != 52.0 && fittyp != 61.0 && fittyp != 62.0) {
    notes_.push_back("FITTYP " + std::to_string(static_cast<int>(fittyp)) +
                     " is not MF 5.2/6.1; reading it as MF 6.1 coefficients");
  }
  // ISO-W files have a negative cornering stiffness (PKY1 * LKY < 0): a
  // positive ISO slip angle gives a negative lateral force.
  if (c("PKY1") * c("LKY") * c("PDY1") > 0.0) {
    alpha_sign_ = 1.0;
    notes_.push_back("cornering stiffness has the SAE sign; slip angle taken without ISO flip");
  }
  if (coefficient("TYRESIDE") != 0.0) {
    notes_.push_back("TYRESIDE ignored (no camber or conicity terms are used)");
  }
}

double MagicFormulaTire::coefficient(std::string_view name) const {
  const auto it = k_.find(name);
  return it == k_.end() ? 0.0 : it->second;
}

double MagicFormulaTire::c(std::string_view name) const {
  const auto it = k_.find(name);
  if (it != k_.end()) {
    return it->second;
  }
  // Scaling factors default to 1, the Ky curvature (MF 6.1 PKY4) to 2 as in 5.2.
  if (name.size() >= 2 && name.front() == 'L') {
    return 1.0;
  }
  return name == "PKY4" ? 2.0 : 0.0;
}

double MagicFormulaTire::longitudinal_stiffness(double fz) const {
  const double dfz = (fz - fz0_) / fz0_;
  return fz * (c("PKX1") + c("PKX2") * dfz) * std::exp(c("PKX3") * dfz) * c("LKX");
}

double MagicFormulaTire::cornering_stiffness(double fz) const {
  const double kya = c("PKY1") * fz0_ * std::sin(c("PKY4") * std::atan(fz / (c("PKY2") * fz0_))) * c("LKY");
  return std::abs(kya);
}

TireForces MagicFormulaTire::evaluate(const TireInput& in) const {
  TireForces out;
  if (in.fz <= 0.0) {
    return out;
  }
  const double fz = in.fz;
  const double dfz = (fz - fz0_) / fz0_;
  const double kappa = in.slip_ratio;
  const double alpha =
      std::clamp(std::tan(alpha_sign_ * in.slip_angle), -kMaxTanAlpha, kMaxTanAlpha);  // ISO, tan

  // --- Pure longitudinal ------------------------------------------------------
  const double shx = (c("PHX1") + c("PHX2") * dfz) * c("LHX");
  const double kx = kappa + shx;
  const double cx = c("PCX1") * c("LCX");
  const double mux = (c("PDX1") + c("PDX2") * dfz) * c("LMUX");
  const double dx = mux * fz;
  const double ex = std::min(1.0, (c("PEX1") + c("PEX2") * dfz + c("PEX3") * dfz * dfz) *
                                      (1.0 - c("PEX4") * sign(kx)) * c("LEX"));
  const double kxk = longitudinal_stiffness(fz);
  const double bx = kxk / (cx * dx + kEpsilon);
  const double svx = fz * (c("PVX1") + c("PVX2") * dfz) * c("LVX") * c("LMUX");
  const double fx0 = mf(kx, bx, cx, dx, ex) + svx;

  // --- Pure lateral -------------------------------------------------------------
  const double kya = c("PKY1") * fz0_ * std::sin(c("PKY4") * std::atan(fz / (c("PKY2") * fz0_))) * c("LKY");
  const double kya_safe = kya + (kya >= 0.0 ? kEpsilon : -kEpsilon);
  const double shy = (c("PHY1") + c("PHY2") * dfz) * c("LHY");
  const double svy = fz * (c("PVY1") + c("PVY2") * dfz) * c("LVY") * c("LMUY");
  const double ay = alpha + shy;
  const double cy = c("PCY1") * c("LCY");
  const double muy = (c("PDY1") + c("PDY2") * dfz) * c("LMUY");
  const double dy = muy * fz;
  const double ey = std::min(1.0, (c("PEY1") + c("PEY2") * dfz) * (1.0 - c("PEY3") * sign(ay)) * c("LEY"));
  const double by = kya / (cy * dy + kEpsilon);
  const double fy0 = mf(ay, by, cy, dy, ey) + svy;

  // --- Combined slip --------------------------------------------------------------
  const double shxa = c("RHX1");
  const double bxa = c("RBX1") * std::cos(std::atan(c("RBX2") * kappa)) * c("LXAL");
  const double cxa = c("RCX1");
  const double exa = std::min(1.0, c("REX1") + c("REX2") * dfz);
  double gxa = 1.0;
  if (bxa != 0.0 && cxa != 0.0) {
    gxa = weight(alpha + shxa, bxa, cxa, exa) / weight(shxa, bxa, cxa, exa);
  }
  out.fx = gxa * fx0;

  const double dvyk = muy * fz * (c("RVY1") + c("RVY2") * dfz) * std::cos(std::atan(c("RVY4") * alpha));
  const double svyk = dvyk * std::sin(c("RVY5") * std::atan(c("RVY6") * kappa)) * c("LVYKA");
  const double shyk = c("RHY1") + c("RHY2") * dfz;
  const double byk = c("RBY1") * std::cos(std::atan(c("RBY2") * (alpha - c("RBY3")))) * c("LYKA");
  const double cyk = c("RCY1");
  const double eyk = std::min(1.0, c("REY1") + c("REY2") * dfz);
  double gyk = 1.0;
  if (byk != 0.0 && cyk != 0.0) {
    gyk = weight(kappa + shyk, byk, cyk, eyk) / weight(shyk, byk, cyk, eyk);
  }
  out.fy = gyk * fy0 + svyk;

  // --- Aligning torque -------------------------------------------------------------
  const double cos_alpha = 1.0 / std::sqrt(1.0 + alpha * alpha);
  const double sht = c("QHZ1") + c("QHZ2") * dfz;
  const double alpha_t = alpha + sht;
  const double bt = (c("QBZ1") + c("QBZ2") * dfz + c("QBZ3") * dfz * dfz) * c("LKY") / c("LMUY");
  const double ct = c("QCZ1");
  const double dt = fz * (c("QDZ1") + c("QDZ2") * dfz) * c("LTR") * r0_ / fz0_;
  const double shf = shy + svy / kya_safe;
  const double alpha_r = alpha + shf;
  const double br = c("QBZ9") * c("LKY") / c("LMUY") + c("QBZ10") * by * cy;
  const double dr = fz * r0_ * (c("QDZ6") + c("QDZ7") * dfz) * c("LRES") * c("LMUY") * cos_alpha;

  // Equivalent slip angles include the longitudinal slip.
  const double ratio = kxk / kya_safe;
  const auto equivalent = [&](double a) {
    return std::atan(std::sqrt(a * a + ratio * ratio * kappa * kappa)) * (a >= 0.0 ? 1.0 : -1.0);
  };
  const double alpha_t_eq = equivalent(alpha_t);
  const double alpha_r_eq = equivalent(alpha_r);
  const double et = std::min(1.0, (c("QEZ1") + c("QEZ2") * dfz + c("QEZ3") * dfz * dfz) *
                                      (1.0 + c("QEZ4") * 2.0 / std::numbers::pi * std::atan(bt * ct * alpha_t)));
  const double trail = dt * weight(alpha_t_eq, bt, ct, et) * cos_alpha;
  const double residual = dr * std::cos(std::atan(br * alpha_r_eq));
  const double s = r0_ * (c("SSZ1") + c("SSZ2") * (out.fy / fz0_)) * c("LS");
  out.mz = -trail * (out.fy - svyk) + residual + s * out.fx;
  return out;
}

}  // namespace vsim
