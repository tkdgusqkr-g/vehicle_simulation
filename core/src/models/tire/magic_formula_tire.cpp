#include <algorithm>
#include <array>
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

// Coefficients the formulas use, cached by index at load.
#define VSIM_MF_COEFFICIENTS(X) \
  X(LCX) X(LCY) X(LEX) X(LEY) X(LHX) X(LHY) X(LKX) X(LKY) X(LMUX) X(LMUY) X(LRES) X(LS) X(LTR) \
  X(LVX) X(LVY) X(LVYKA) X(LXAL) X(LYKA) X(PCX1) X(PCY1) X(PDX1) X(PDX2) X(PDY1) X(PDY2) \
  X(PEX1) X(PEX2) X(PEX3) X(PEX4) X(PEY1) X(PEY2) X(PEY3) X(PHX1) X(PHX2) X(PHY1) X(PHY2) \
  X(PKX1) X(PKX2) X(PKX3) X(PKY1) X(PKY2) X(PKY4) X(PVX1) X(PVX2) X(PVY1) X(PVY2) X(QBZ1) \
  X(QBZ10) X(QBZ2) X(QBZ3) X(QBZ9) X(QCZ1) X(QDZ1) X(QDZ2) X(QDZ6) X(QDZ7) X(QEZ1) X(QEZ2) \
  X(QEZ3) X(QEZ4) X(QHZ1) X(QHZ2) X(RBX1) X(RBX2) X(RBY1) X(RBY2) X(RBY3) X(RCX1) X(RCY1) \
  X(REX1) X(REX2) X(REY1) X(REY2) X(RHX1) X(RHY1) X(RHY2) X(RVY1) X(RVY2) X(RVY4) X(RVY5) \
  X(RVY6) X(SSZ1) X(SSZ2)

enum Coefficient : std::size_t {
#define VSIM_MF_ENUM(name) name,
  VSIM_MF_COEFFICIENTS(VSIM_MF_ENUM)
#undef VSIM_MF_ENUM
  kCoefficientCount
};

constexpr std::array<const char*, kCoefficientCount> kCoefficientNames = {
#define VSIM_MF_NAME(name) #name,
    VSIM_MF_COEFFICIENTS(VSIM_MF_NAME)
#undef VSIM_MF_NAME
};

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
  v_.resize(kCoefficientCount);
  for (std::size_t i = 0; i < kCoefficientCount; ++i) {
    v_[i] = c(kCoefficientNames[i]);
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
  return fz * (v_[PKX1] + v_[PKX2] * dfz) * std::exp(v_[PKX3] * dfz) * v_[LKX];
}

double MagicFormulaTire::cornering_stiffness(double fz) const {
  const double kya = v_[PKY1] * fz0_ * std::sin(v_[PKY4] * std::atan(fz / (v_[PKY2] * fz0_))) * v_[LKY];
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
  const double shx = (v_[PHX1] + v_[PHX2] * dfz) * v_[LHX];
  const double kx = kappa + shx;
  const double cx = v_[PCX1] * v_[LCX];
  const double mux = (v_[PDX1] + v_[PDX2] * dfz) * v_[LMUX];
  const double dx = mux * fz;
  const double ex = std::min(1.0, (v_[PEX1] + v_[PEX2] * dfz + v_[PEX3] * dfz * dfz) *
                                      (1.0 - v_[PEX4] * sign(kx)) * v_[LEX]);
  const double kxk = longitudinal_stiffness(fz);
  const double bx = kxk / (cx * dx + kEpsilon);
  const double svx = fz * (v_[PVX1] + v_[PVX2] * dfz) * v_[LVX] * v_[LMUX];
  const double fx0 = mf(kx, bx, cx, dx, ex) + svx;

  // --- Pure lateral -------------------------------------------------------------
  const double kya = v_[PKY1] * fz0_ * std::sin(v_[PKY4] * std::atan(fz / (v_[PKY2] * fz0_))) * v_[LKY];
  const double kya_safe = kya + (kya >= 0.0 ? kEpsilon : -kEpsilon);
  const double shy = (v_[PHY1] + v_[PHY2] * dfz) * v_[LHY];
  const double svy = fz * (v_[PVY1] + v_[PVY2] * dfz) * v_[LVY] * v_[LMUY];
  const double ay = alpha + shy;
  const double cy = v_[PCY1] * v_[LCY];
  const double muy = (v_[PDY1] + v_[PDY2] * dfz) * v_[LMUY];
  const double dy = muy * fz;
  const double ey = std::min(1.0, (v_[PEY1] + v_[PEY2] * dfz) * (1.0 - v_[PEY3] * sign(ay)) * v_[LEY]);
  const double by = kya / (cy * dy + kEpsilon);
  const double fy0 = mf(ay, by, cy, dy, ey) + svy;

  // --- Combined slip --------------------------------------------------------------
  const double shxa = v_[RHX1];
  const double bxa = v_[RBX1] * std::cos(std::atan(v_[RBX2] * kappa)) * v_[LXAL];
  const double cxa = v_[RCX1];
  const double exa = std::min(1.0, v_[REX1] + v_[REX2] * dfz);
  double gxa = 1.0;
  if (bxa != 0.0 && cxa != 0.0) {
    gxa = weight(alpha + shxa, bxa, cxa, exa) / weight(shxa, bxa, cxa, exa);
  }
  out.fx = gxa * fx0;

  const double dvyk = muy * fz * (v_[RVY1] + v_[RVY2] * dfz) * std::cos(std::atan(v_[RVY4] * alpha));
  const double svyk = dvyk * std::sin(v_[RVY5] * std::atan(v_[RVY6] * kappa)) * v_[LVYKA];
  const double shyk = v_[RHY1] + v_[RHY2] * dfz;
  const double byk = v_[RBY1] * std::cos(std::atan(v_[RBY2] * (alpha - v_[RBY3]))) * v_[LYKA];
  const double cyk = v_[RCY1];
  const double eyk = std::min(1.0, v_[REY1] + v_[REY2] * dfz);
  double gyk = 1.0;
  if (byk != 0.0 && cyk != 0.0) {
    gyk = weight(kappa + shyk, byk, cyk, eyk) / weight(shyk, byk, cyk, eyk);
  }
  out.fy = gyk * fy0 + svyk;

  // --- Aligning torque -------------------------------------------------------------
  const double cos_alpha = 1.0 / std::sqrt(1.0 + alpha * alpha);
  const double sht = v_[QHZ1] + v_[QHZ2] * dfz;
  const double alpha_t = alpha + sht;
  const double bt = (v_[QBZ1] + v_[QBZ2] * dfz + v_[QBZ3] * dfz * dfz) * v_[LKY] / v_[LMUY];
  const double ct = v_[QCZ1];
  const double dt = fz * (v_[QDZ1] + v_[QDZ2] * dfz) * v_[LTR] * r0_ / fz0_;
  const double shf = shy + svy / kya_safe;
  const double alpha_r = alpha + shf;
  const double br = v_[QBZ9] * v_[LKY] / v_[LMUY] + v_[QBZ10] * by * cy;
  const double dr = fz * r0_ * (v_[QDZ6] + v_[QDZ7] * dfz) * v_[LRES] * v_[LMUY] * cos_alpha;

  // Equivalent slip angles include the longitudinal slip.
  const double ratio = kxk / kya_safe;
  const auto equivalent = [&](double a) {
    return std::atan(std::sqrt(a * a + ratio * ratio * kappa * kappa)) * (a >= 0.0 ? 1.0 : -1.0);
  };
  const double alpha_t_eq = equivalent(alpha_t);
  const double alpha_r_eq = equivalent(alpha_r);
  const double et = std::min(1.0, (v_[QEZ1] + v_[QEZ2] * dfz + v_[QEZ3] * dfz * dfz) *
                                      (1.0 + v_[QEZ4] * 2.0 / std::numbers::pi * std::atan(bt * ct * alpha_t)));
  const double trail = dt * weight(alpha_t_eq, bt, ct, et) * cos_alpha;
  const double residual = dr * std::cos(std::atan(br * alpha_r_eq));
  const double s = r0_ * (v_[SSZ1] + v_[SSZ2] * (out.fy / fz0_)) * v_[LS];
  out.mz = -trail * (out.fy - svyk) + residual + s * out.fx;
  return out;
}

}  // namespace vsim
