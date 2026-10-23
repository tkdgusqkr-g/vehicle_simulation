#include "vsim/track/track.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "vsim/sim/rng.hpp"
#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

using nlohmann::json;

double wrap_angle(double a) {
  return std::remainder(a, 2.0 * std::numbers::pi);
}

// sin and cos together, several times faster than the library's for the
// road's many waves: Cody-Waite reduction to [-pi/4, pi/4] (pi/2 split in
// three parts, good for phases up to ~1e6 rad) and Taylor polynomials,
// accurate to ~1e-15.
void fast_sincos(double x, double& s, double& c) {
  constexpr double kTwoOverPi = 0.63661977236758134308;
  constexpr double kPio2a = 1.5707963267341256e+00;  // pi/2 = a + b + c
  constexpr double kPio2b = 6.0771005065061922e-11;
  constexpr double kPio2c = 2.0222662487959506e-21;
  const double q = std::nearbyint(x * kTwoOverPi);
  const double r = ((x - q * kPio2a) - q * kPio2b) - q * kPio2c;
  const double r2 = r * r;
  const double sr = r * (1.0 + r2 * (-1.0 / 6 + r2 * (1.0 / 120 + r2 * (-1.0 / 5040 + r2 * (1.0 / 362880 + r2 * (-1.0 / 39916800 + r2 * (1.0 / 6227020800.0)))))));
  const double cr = 1.0 + r2 * (-0.5 + r2 * (1.0 / 24 + r2 * (-1.0 / 720 + r2 * (1.0 / 40320 + r2 * (-1.0 / 3628800 + r2 * (1.0 / 479001600.0 + r2 * (-1.0 / 87178291200.0)))))));
  switch (static_cast<long long>(q) & 3) {
    case 0: s = sr; c = cr; break;
    case 1: s = cr; c = -sr; break;
    case 2: s = -sr; c = -cr; break;
    default: s = -cr; c = sr; break;
  }
}

// Second derivatives of a cubic spline through (t[i], y[i]); natural ends when
// open, periodic when closed (then t has one extra entry closing the loop and
// y[n] == y[0]).
std::vector<double> spline_moments(const std::vector<double>& t, const std::vector<double>& y, bool closed) {
  const std::size_t n = closed ? t.size() - 1 : t.size();
  std::vector<double> m(t.size(), 0.0);
  if (n < 3 && !closed) {
    return m;  // straight line between two points
  }
  // Tridiagonal (cyclic when closed) system for the unknown moments.
  const std::size_t unknowns = closed ? n : n - 2;
  std::vector<double> a(unknowns), b(unknowns), c(unknowns), d(unknowns);
  for (std::size_t k = 0; k < unknowns; ++k) {
    const std::size_t i = closed ? k : k + 1;  // knot index
    const std::size_t prev = closed ? (i + n - 1) % n : i - 1;
    const double h0 = closed && i == 0 ? t[n] - t[n - 1] : t[i] - t[prev];
    const double h1 = t[i + 1] - t[i];
    const double y_next = y[i + 1];
    const double y_prev = closed && i == 0 ? y[n - 1] : y[prev];
    a[k] = h0;
    b[k] = 2.0 * (h0 + h1);
    c[k] = h1;
    d[k] = 6.0 * ((y_next - y[i]) / h1 - (y[i] - y_prev) / h0);
  }
  const auto solve_tridiagonal = [](std::vector<double> sub, std::vector<double> diag, std::vector<double> sup,
                                    std::vector<double> rhs) {
    const std::size_t size = diag.size();
    for (std::size_t i = 1; i < size; ++i) {
      const double w = sub[i] / diag[i - 1];
      diag[i] -= w * sup[i - 1];
      rhs[i] -= w * rhs[i - 1];
    }
    std::vector<double> x(size);
    x[size - 1] = rhs[size - 1] / diag[size - 1];
    for (std::size_t i = size - 1; i-- > 0;) {
      x[i] = (rhs[i] - sup[i] * x[i + 1]) / diag[i];
    }
    return x;
  };
  std::vector<double> solution;
  if (!closed) {
    a.front() = 0.0;
    c.back() = 0.0;
    solution = solve_tridiagonal(a, b, c, d);
    for (std::size_t k = 0; k < unknowns; ++k) {
      m[k + 1] = solution[k];
    }
    return m;
  }
  // Cyclic tridiagonal by Sherman-Morrison (corner terms a[0] and c[n-1]).
  const double alpha = a[0];
  const double beta = c[n - 1];
  const double gamma = -b[0];
  std::vector<double> diag = b;
  diag[0] -= gamma;
  diag[n - 1] -= alpha * beta / gamma;
  std::vector<double> sub = a, sup = c;
  sub[0] = 0.0;
  sup[n - 1] = 0.0;
  const std::vector<double> x = solve_tridiagonal(sub, diag, sup, d);
  std::vector<double> u(n, 0.0);
  u[0] = gamma;
  u[n - 1] = beta;
  const std::vector<double> z = solve_tridiagonal(sub, diag, sup, u);
  const double factor = (x[0] + alpha * x[n - 1] / gamma) / (1.0 + z[0] + alpha * z[n - 1] / gamma);
  for (std::size_t i = 0; i < n; ++i) {
    m[i] = x[i] - factor * z[i];
  }
  m[n] = m[0];
  return m;
}

// Value, first and second derivative of the spline segment [t0, t1].
struct SplineValue {
  double v, d1, d2;
};
SplineValue spline_eval(double t0, double t1, double y0, double y1, double m0, double m1, double t) {
  const double h = t1 - t0;
  const double a = t1 - t;
  const double b = t - t0;
  SplineValue out{};
  out.v = m0 * a * a * a / (6.0 * h) + m1 * b * b * b / (6.0 * h) + (y0 / h - m0 * h / 6.0) * a +
          (y1 / h - m1 * h / 6.0) * b;
  out.d1 = -m0 * a * a / (2.0 * h) + m1 * b * b / (2.0 * h) - (y0 / h - m0 * h / 6.0) + (y1 / h - m1 * h / 6.0);
  out.d2 = m0 * a / h + m1 * b / h;
  return out;
}

// Resamples a dense polyline (x, y) to steps of ~`step` and estimates heading
// and curvature with central differences.
std::vector<CurveSample> resample_polyline(const std::vector<double>& xs, const std::vector<double>& ys,
                                           bool closed, double step) {
  std::vector<double> cum(xs.size(), 0.0);
  for (std::size_t i = 1; i < xs.size(); ++i) {
    cum[i] = cum[i - 1] + std::hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]);
  }
  double length = cum.back();
  if (closed) {
    length += std::hypot(xs.front() - xs.back(), ys.front() - ys.back());
  }
  const std::size_t count = std::max<std::size_t>(2, static_cast<std::size_t>(std::llround(length / step)));
  const double ds = length / static_cast<double>(count);
  std::vector<CurveSample> out(closed ? count : count + 1);
  std::size_t j = 0;
  for (std::size_t k = 0; k < out.size(); ++k) {
    const double s = std::min(static_cast<double>(k) * ds, length);
    while (j + 1 < cum.size() && cum[j + 1] < s) {
      ++j;
    }
    double x0 = xs[j], y0 = ys[j], s0 = cum[j];
    double x1, y1, s1;
    if (j + 1 < cum.size()) {
      x1 = xs[j + 1];
      y1 = ys[j + 1];
      s1 = cum[j + 1];
    } else {  // closing segment
      x1 = xs.front();
      y1 = ys.front();
      s1 = length;
    }
    const double f = s1 > s0 ? std::clamp((s - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
    out[k].s = s;
    out[k].x = x0 + f * (x1 - x0);
    out[k].y = y0 + f * (y1 - y0);
  }
  const auto n = static_cast<long>(out.size());
  const auto neighbour = [&](std::size_t k, long offset) -> const CurveSample& {
    long index = static_cast<long>(k) + offset;
    index = closed ? (index + n) % n : std::clamp(index, 0L, n - 1);
    return out[static_cast<std::size_t>(index)];
  };
  for (std::size_t k = 0; k < out.size(); ++k) {
    const CurveSample& a = neighbour(k, -2);
    const CurveSample& b = neighbour(k, 2);
    out[k].heading = std::atan2(b.y - a.y, b.x - a.x);
  }
  for (std::size_t k = 0; k < out.size(); ++k) {
    const CurveSample& a = neighbour(k, -2);
    const CurveSample& b = neighbour(k, 2);
    const double distance = std::hypot(b.x - a.x, b.y - a.y);
    out[k].curvature = distance > 1e-9 ? wrap_angle(b.heading - a.heading) / distance : 0.0;
  }
  return out;
}

const json& require(const json& object, const char* key) {
  if (!object.contains(key)) {
    throw std::invalid_argument(std::string("track file: missing '") + key + "'");
  }
  return object.at(key);
}

}  // namespace

// --- TrackDefinition ------------------------------------------------------------

TrackDefinition TrackDefinition::from_json(std::string_view text) {
  json doc;
  try {
    doc = json::parse(text);
  } catch (const json::parse_error& e) {
    throw std::invalid_argument(std::string("track file is not valid JSON: ") + e.what());
  }
  TrackDefinition def;
  try {
    const int schema = doc.value("schema_version", 0);
    if (schema != kSchemaVersion) {
      throw std::invalid_argument("track file has schema_version " + std::to_string(schema) +
                                  "; this build reads version " + std::to_string(kSchemaVersion));
    }
    def.name = doc.value("name", "");
    def.closed = doc.value("closed", true);
    for (const json& p : require(doc, "points")) {
      Point point;
      point.x = require(p, "x").get<double>();
      point.y = require(p, "y").get<double>();
      point.z = p.value("z", 0.0);
      point.w_left = p.value("w_left", point.w_left);
      point.w_right = p.value("w_right", point.w_right);
      point.bank_deg = p.value("bank_deg", 0.0);
      def.points.push_back(point);
    }
    def.start_point = doc.value("start_point", 0);
    def.finish_point = doc.value("finish_point", -1);
    def.sector_points = doc.value("sector_points", std::vector<int>{});
    for (const json& l : doc.value("lines", json::array())) {
      Line line;
      line.name = require(l, "name").get<std::string>();
      for (const json& o : l.value("offsets", json::array())) {
        line.offsets.emplace_back(o.at(0).get<double>(), o.at(1).get<double>());
      }
      def.lines.push_back(std::move(line));
    }
    if (doc.contains("surface")) {
      const json& f = doc["surface"];
      def.surface.mu = f.value("mu", def.surface.mu);
      def.surface.roughness = f.value("roughness", def.surface.roughness);
      def.surface.grass_mu = f.value("grass_mu", def.surface.grass_mu);
      def.surface.grass_roughness = f.value("grass_roughness", def.surface.grass_roughness);
    }
    for (const json& k : doc.value("kerbs", json::array())) {
      Kerb kerb;
      kerb.from_point = require(k, "from_point").get<int>();
      kerb.to_point = require(k, "to_point").get<int>();
      kerb.side = k.value("side", kerb.side);
      kerb.width = k.value("width", kerb.width);
      kerb.height = k.value("height", kerb.height);
      def.kerbs.push_back(kerb);
    }
    for (const json& k : doc.value("patches", json::array())) {
      Patch patch;
      patch.from_point = require(k, "from_point").get<int>();
      patch.to_point = require(k, "to_point").get<int>();
      patch.n_min = k.value("n_min", patch.n_min);
      patch.n_max = k.value("n_max", patch.n_max);
      patch.mu = require(k, "mu").get<double>();
      def.patches.push_back(patch);
    }
  } catch (const json::exception& e) {
    throw std::invalid_argument(std::string("track file: ") + e.what());
  }
  def.validate();
  return def;
}

TrackDefinition TrackDefinition::from_file(const std::filesystem::path& file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) {
    throw std::invalid_argument("cannot open track file '" + path_to_utf8(file) + "'");
  }
  std::stringstream buffer;
  buffer << in.rdbuf();
  try {
    if (file.extension() == ".csv") {
      // TUMFTM racetrack-database: x_m, y_m, w_tr_right_m, w_tr_left_m (closed).
      TrackDefinition def;
      def.name = path_to_utf8(file.stem());
      std::string line;
      while (std::getline(buffer, line)) {
        if (line.empty() || line[0] == '#') {
          continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream fields(line);
        Point point;
        if (!(fields >> point.x >> point.y >> point.w_right >> point.w_left)) {
          throw std::invalid_argument("expected x_m, y_m, w_tr_right_m, w_tr_left_m in '" + line + "'");
        }
        def.points.push_back(point);
      }
      def.validate();
      return def;
    }
    return from_json(buffer.str());
  } catch (const std::invalid_argument& e) {
    throw std::invalid_argument(path_to_utf8(file) + ": " + e.what());
  }
}

std::string TrackDefinition::to_json() const {
  json doc;
  doc["schema_version"] = kSchemaVersion;
  doc["name"] = name;
  doc["closed"] = closed;
  json points_json = json::array();
  for (const Point& p : points) {
    points_json.push_back({{"x", p.x}, {"y", p.y}, {"z", p.z}, {"w_left", p.w_left}, {"w_right", p.w_right},
                           {"bank_deg", p.bank_deg}});
  }
  doc["points"] = points_json;
  doc["start_point"] = start_point;
  if (!closed) {
    doc["finish_point"] = finish_point;
  }
  doc["sector_points"] = sector_points;
  json lines_json = json::array();
  for (const Line& line : lines) {
    json offsets = json::array();
    for (const auto& [u, n] : line.offsets) {
      offsets.push_back({u, n});
    }
    lines_json.push_back({{"name", line.name}, {"offsets", offsets}});
  }
  doc["lines"] = lines_json;
  const Surface plain;
  if (surface.mu != plain.mu || surface.roughness != plain.roughness || surface.grass_mu != plain.grass_mu ||
      surface.grass_roughness != plain.grass_roughness) {
    doc["surface"] = {{"mu", surface.mu}, {"roughness", surface.roughness}, {"grass_mu", surface.grass_mu},
                      {"grass_roughness", surface.grass_roughness}};
  }
  if (!kerbs.empty()) {
    json kerbs_json = json::array();
    for (const Kerb& k : kerbs) {
      kerbs_json.push_back({{"from_point", k.from_point}, {"to_point", k.to_point}, {"side", k.side},
                            {"width", k.width}, {"height", k.height}});
    }
    doc["kerbs"] = kerbs_json;
  }
  if (!patches.empty()) {
    json patches_json = json::array();
    for (const Patch& k : patches) {
      patches_json.push_back({{"from_point", k.from_point}, {"to_point", k.to_point}, {"n_min", k.n_min},
                              {"n_max", k.n_max}, {"mu", k.mu}});
    }
    doc["patches"] = patches_json;
  }
  return doc.dump(1) + "\n";
}

void TrackDefinition::validate() const {
  const int count = static_cast<int>(points.size());
  if (count < (closed ? 3 : 2)) {
    throw std::invalid_argument(closed ? "a closed track needs at least 3 points"
                                       : "an open track needs at least 2 points");
  }
  for (int i = 0; i < count; ++i) {
    const Point& p = points[static_cast<std::size_t>(i)];
    if (!(p.w_left > 0.0 && p.w_right > 0.0)) {
      throw std::invalid_argument("point " + std::to_string(i) + ": widths must be positive");
    }
    const int next = i + 1 < count ? i + 1 : (closed ? 0 : -1);
    if (next >= 0) {
      const Point& q = points[static_cast<std::size_t>(next)];
      if (std::hypot(q.x - p.x, q.y - p.y) < 1e-3) {
        throw std::invalid_argument("points " + std::to_string(i) + " and " + std::to_string(next) +
                                    " coincide");
      }
    }
  }
  const auto check_index = [&](int index, const char* what) {
    if (index < 0 || index >= count) {
      throw std::invalid_argument(std::string(what) + " " + std::to_string(index) + " is not a point index");
    }
  };
  check_index(start_point, "start_point");
  if (!closed && finish_point >= 0) {
    check_index(finish_point, "finish_point");
  }
  for (const int sector : sector_points) {
    check_index(sector, "sector point");
  }
  const auto check_class = [](const std::string& c, const char* what) {
    if (!c.empty()) {
      try {
        iso8608_gd0(c);
      } catch (const std::invalid_argument&) {
        throw std::invalid_argument(std::string(what) + " '" + c + "' is not an ISO 8608 class A..H");
      }
    }
  };
  check_class(surface.roughness, "surface.roughness");
  check_class(surface.grass_roughness, "surface.grass_roughness");
  if (!(surface.mu > 0.0) || !(surface.grass_mu > 0.0)) {
    throw std::invalid_argument("surface friction must be positive");
  }
  for (const Kerb& k : kerbs) {
    check_index(k.from_point, "kerb from_point");
    check_index(k.to_point, "kerb to_point");
    if (k.side != "left" && k.side != "right") {
      throw std::invalid_argument("kerb side must be left or right, not '" + k.side + "'");
    }
    if (!(k.width > 0.0) || !(k.height >= 0.0) || k.height > 0.3) {
      throw std::invalid_argument("kerb width must be positive and height between 0 and 0.3 m");
    }
  }
  for (const Patch& k : patches) {
    check_index(k.from_point, "patch from_point");
    check_index(k.to_point, "patch to_point");
    if (!(k.mu > 0.0) || !(k.n_max > k.n_min)) {
      throw std::invalid_argument("patch needs mu > 0 and n_max > n_min");
    }
  }
  for (const Line& line : lines) {
    if (line.name.empty() || line.name == "center") {
      throw std::invalid_argument("line names must be non-empty and not 'center'");
    }
    for (const auto& [u, n] : line.offsets) {
      if (!(u >= 0.0 && u <= 1.0) || !std::isfinite(n)) {
        throw std::invalid_argument("line '" + line.name + "': offsets need u in [0, 1]");
      }
    }
  }
}

// --- SampledCurve -----------------------------------------------------------------

SampledCurve::SampledCurve(std::vector<CurveSample> samples, bool closed)
    : samples_(std::move(samples)), closed_(closed) {
  if (samples_.size() < 2) {
    throw std::invalid_argument("a curve needs at least two samples");
  }
  if (closed_) {
    const CurveSample& last = samples_.back();
    length_ = last.s + std::hypot(samples_.front().x - last.x, samples_.front().y - last.y);
  } else {
    length_ = samples_.back().s;
  }
}

double SampledCurve::wrap(double s) const {
  if (closed_) {
    s = std::fmod(s, length_);
    return s < 0.0 ? s + length_ : s;
  }
  return std::clamp(s, 0.0, length_);
}

std::size_t SampledCurve::index_at(double s) const {
  const auto upper = std::upper_bound(samples_.begin(), samples_.end(), s,
                                      [](double value, const CurveSample& sample) { return value < sample.s; });
  const std::size_t index = upper == samples_.begin() ? 0 : static_cast<std::size_t>(upper - samples_.begin()) - 1;
  return std::min(index, samples_.size() - 1);
}

CurveSample SampledCurve::at(double s) const {
  s = wrap(s);
  const std::size_t i = index_at(s);
  const bool last = i + 1 >= samples_.size();
  if (last && !closed_) {
    return samples_.back();
  }
  const CurveSample& a = samples_[i];
  const CurveSample& b = last ? samples_.front() : samples_[i + 1];
  const double span = (last ? length_ : b.s) - a.s;
  const double f = span > 0.0 ? (s - a.s) / span : 0.0;
  CurveSample out;
  out.s = s;
  out.x = a.x + f * (b.x - a.x);
  out.y = a.y + f * (b.y - a.y);
  out.heading = a.heading + f * wrap_angle(b.heading - a.heading);
  out.curvature = a.curvature + f * (b.curvature - a.curvature);
  return out;
}

CurveProjection SampledCurve::project(double x, double y, std::optional<double> hint, double window) const {
  const std::size_t n = samples_.size();
  const std::size_t segments = closed_ ? n : n - 1;
  std::size_t first = 0;
  std::size_t count = segments;
  if (hint) {
    const double step = length_ / static_cast<double>(segments);
    const auto half = static_cast<std::size_t>(std::ceil(window / step));
    if (2 * half + 1 < segments) {
      const std::size_t center = index_at(wrap(*hint));
      count = 2 * half + 1;
      if (closed_) {
        first = (center + segments - half) % segments;
      } else {
        first = center > half ? center - half : 0;
        count = std::min(count, segments - first);
      }
    }
  }
  double best = std::numeric_limits<double>::infinity();
  CurveProjection result;
  for (std::size_t k = 0; k < count; ++k) {
    const std::size_t i = (first + k) % segments;
    const CurveSample& a = samples_[i];
    const CurveSample& b = samples_[(i + 1) % n];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double len_sq = dx * dx + dy * dy;
    double f = len_sq > 0.0 ? ((x - a.x) * dx + (y - a.y) * dy) / len_sq : 0.0;
    f = std::clamp(f, 0.0, 1.0);
    const double px = a.x + f * dx;
    const double py = a.y + f * dy;
    const double d = (x - px) * (x - px) + (y - py) * (y - py);  // squared
    if (d < best) {
      best = d;
      const double span = (i + 1 < n ? b.s : length_) - a.s;
      result.s = a.s + f * span;
      result.heading = a.heading + f * wrap_angle(b.heading - a.heading);
      result.n = -(x - px) * std::sin(result.heading) + (y - py) * std::cos(result.heading);
      result.distance = d;
    }
  }
  result.distance = std::sqrt(result.distance);
  result.s = wrap(result.s);
  return result;
}

// --- Track ---------------------------------------------------------------------------

Track::Track(TrackDefinition definition) : def_(std::move(definition)) {
  def_.validate();
  const std::vector<TrackDefinition::Point>& pts = def_.points;
  const std::size_t n = pts.size();
  const std::size_t knots = def_.closed ? n + 1 : n;
  std::vector<double> t(knots), xs(knots), ys(knots);
  for (std::size_t i = 0; i < knots; ++i) {
    const TrackDefinition::Point& p = pts[i % n];
    xs[i] = p.x;
    ys[i] = p.y;
    t[i] = i == 0 ? 0.0 : t[i - 1] + std::hypot(p.x - xs[i - 1], p.y - ys[i - 1]);
  }
  std::vector<double> zs(knots);
  for (std::size_t i = 0; i < knots; ++i) {
    zs[i] = pts[i % n].z;
  }
  const std::vector<double> mx = spline_moments(t, xs, def_.closed);
  const std::vector<double> my = spline_moments(t, ys, def_.closed);
  const std::vector<double> mz = spline_moments(t, zs, def_.closed);

  // Dense samples along each segment, arc length from their chords.
  struct Dense {
    double s, x, y, heading, curvature, z;
    std::size_t segment;
  };
  std::vector<Dense> dense;
  point_s_.assign(n, 0.0);
  double s = 0.0;
  for (std::size_t i = 0; i + 1 < knots; ++i) {
    const double h = t[i + 1] - t[i];
    const auto steps = std::max<std::size_t>(16, static_cast<std::size_t>(std::ceil(h / 0.02)));
    for (std::size_t k = 0; k <= steps; ++k) {
      if (k == 0 && i > 0) {
        continue;  // shared with the previous segment's end
      }
      const double tt = t[i] + h * static_cast<double>(k) / static_cast<double>(steps);
      const SplineValue vx = spline_eval(t[i], t[i + 1], xs[i], xs[i + 1], mx[i], mx[i + 1], tt);
      const SplineValue vy = spline_eval(t[i], t[i + 1], ys[i], ys[i + 1], my[i], my[i + 1], tt);
      const double speed = std::hypot(vx.d1, vy.d1);
      if (!dense.empty()) {
        const Dense& prev = dense.back();
        s += std::hypot(vx.v - prev.x, vy.v - prev.y);
      }
      const double curvature = speed > 1e-12 ? (vx.d1 * vy.d2 - vy.d1 * vx.d2) / (speed * speed * speed) : 0.0;
      const double z = spline_eval(t[i], t[i + 1], zs[i], zs[i + 1], mz[i], mz[i + 1], tt).v;
      dense.push_back({s, vx.v, vy.v, std::atan2(vy.d1, vx.d1), curvature, z, i});
    }
    if (i + 1 < n) {
      point_s_[i + 1] = s;
    }
  }
  const double total = s;
  if (def_.closed) {
    dense.pop_back();  // the closing knot repeats the first point
  }
  const std::size_t count = std::max<std::size_t>(2, static_cast<std::size_t>(std::llround(total / kStep)));
  const double ds = total / static_cast<double>(count);

  std::vector<CurveSample> samples(def_.closed ? count : count + 1);
  widths_.resize(samples.size());
  z_.resize(samples.size());
  tan_bank_.resize(samples.size());
  std::size_t j = 0;
  for (std::size_t k = 0; k < samples.size(); ++k) {
    const double sk = std::min(static_cast<double>(k) * ds, total);
    while (j + 1 < dense.size() && dense[j + 1].s < sk) {
      ++j;
    }
    const Dense& a = dense[j];
    const Dense& b = j + 1 < dense.size() ? dense[j + 1] : dense[j];
    const double span = b.s - a.s;
    const double f = span > 0.0 ? std::clamp((sk - a.s) / span, 0.0, 1.0) : 0.0;
    CurveSample& out = samples[k];
    out.s = sk;
    out.x = a.x + f * (b.x - a.x);
    out.y = a.y + f * (b.y - a.y);
    out.heading = a.heading + f * wrap_angle(b.heading - a.heading);
    out.curvature = a.curvature + f * (b.curvature - a.curvature);
    // Widths vary linearly between control points.
    const std::size_t seg = a.segment;
    const double seg_start = point_s_[seg % n];
    const double seg_end = seg + 1 < n ? point_s_[seg + 1] : total;
    const double g = seg_end > seg_start ? std::clamp((sk - seg_start) / (seg_end - seg_start), 0.0, 1.0) : 0.0;
    const TrackDefinition::Point& p0 = pts[seg % n];
    const TrackDefinition::Point& p1 = pts[(seg + 1) % n];
    widths_[k] = {p0.w_left + g * (p1.w_left - p0.w_left), p0.w_right + g * (p1.w_right - p0.w_right)};
    // Elevation follows the spline; bank varies linearly like the widths.
    z_[k] = a.z + f * (b.z - a.z);
    const double bank = (p0.bank_deg + g * (p1.bank_deg - p0.bank_deg)) * std::numbers::pi / 180.0;
    tan_bank_[k] = std::tan(bank);
  }
  samples_ = std::move(samples);
  closed_ = def_.closed;
  length_ = total;

  for (const TrackDefinition::Point& p : pts) {
    relief_ = relief_ || p.z != 0.0 || p.bank_deg != 0.0;
  }
  relief_ = relief_ || !def_.kerbs.empty() || !def_.surface.roughness.empty();
  const std::uint64_t seed = fnv1a64("road:" + def_.name);
  road_waves_ = make_waves(def_.surface.roughness, seed);
  grass_waves_ = make_waves(def_.surface.grass_roughness, splitmix64(seed));
}

// --- Road surface ---------------------------------------------------------------------

double iso8608_gd0(const std::string& road_class) {
  if (road_class.size() != 1 || road_class[0] < 'A' || road_class[0] > 'H') {
    throw std::invalid_argument("ISO 8608 road class must be A..H, not '" + road_class + "'");
  }
  // Class A: 16e-6 m^3; every class is four times the one before.
  return 16e-6 * std::pow(4.0, road_class[0] - 'A');
}

// Displacement PSD Gd(n) = Gd(n0) (n / n0)^-2, n0 = 0.1 cycles/m, as cosines
// at log-spaced spatial frequencies with random phases. Each cosine also
// varies slowly across the road (a lateral wavenumber up to half its own),
// so the two wheel tracks see the same long waves and different short ones.
// On closed tracks the frequencies fit the lap length so the start line has
// no seam.
std::vector<Track::Wave> Track::make_waves(const std::string& iso_class, std::uint64_t seed) const {
  std::vector<Wave> waves;
  if (iso_class.empty()) {
    return waves;
  }
  constexpr int kCount = 64;
  constexpr double kLow = 0.01, kHigh = 5.0, kN0 = 0.1;  // cycles/m
  const double gd0 = iso8608_gd0(iso_class);
  const double ratio = std::pow(kHigh / kLow, 1.0 / (kCount - 1));
  const double band = std::sqrt(ratio) - 1.0 / std::sqrt(ratio);
  Rng rng(seed);
  for (int k = 0; k < kCount; ++k) {
    double n = kLow * std::pow(ratio, k);
    const double amplitude = std::sqrt(2.0 * gd0 * std::pow(n / kN0, -2.0) * n * band);
    if (closed_) {
      n = std::max(1.0, std::round(n * length_)) / length_;
    }
    const double lateral = rng.uniform() - 0.5;
    const double phase = 2.0 * std::numbers::pi * rng.uniform();
    waves.push_back({2.0 * std::numbers::pi * n, 2.0 * std::numbers::pi * n * lateral, amplitude, phase});
  }
  return waves;
}

void Track::add_waves(const std::vector<Wave>& waves, double s, double n, Ground& g) {
  for (const Wave& w : waves) {
    double sn = 0.0, c = 0.0;
    fast_sincos(w.ks * s + w.kn * n + w.phase, sn, c);
    g.z += w.amplitude * c;
    g.dzds -= w.amplitude * w.ks * sn;
    g.dzdn -= w.amplitude * w.kn * sn;
  }
}

double Track::elevation(double s) const { return centre_ground(s, 0.0).z; }

// Centerline elevation plus the bank's cross slope, linear between samples
// (so the slope is the exact derivative of the height).
Track::Ground Track::centre_ground(double s, double n) const {
  s = wrap(s);
  const std::size_t i = index_at(s);
  const bool last = i + 1 >= samples_.size();
  if (last && !closed_) {
    return {z_.back() + n * tan_bank_.back(), 0.0, tan_bank_.back()};
  }
  const std::size_t j = last ? 0 : i + 1;
  const double span = (last ? length_ : samples_[j].s) - samples_[i].s;
  const double f = span > 0.0 ? (s - samples_[i].s) / span : 0.0;
  const double tb = tan_bank_[i] + f * (tan_bank_[j] - tan_bank_[i]);
  const double dtb = span > 0.0 ? (tan_bank_[j] - tan_bank_[i]) / span : 0.0;
  const double zc = z_[i] + f * (z_[j] - z_[i]);
  const double dzc = span > 0.0 ? (z_[j] - z_[i]) / span : 0.0;
  return {zc + n * tb, dzc + n * dtb, tb};
}

// Kerb crest across its width with rumble ridges along it, ramped in at the
// ends. Adds to g and returns the kerb's width (scaled by the end ramps)
// where one runs along this side at s, else 0. e: distance beyond the edge.
double Track::kerb_at(double s, double e, bool left, Ground& g) const {
  for (const TrackDefinition::Kerb& k : def_.kerbs) {
    if ((k.side == "left") != left) {
      continue;
    }
    const double s0 = point_s(k.from_point);
    double s1 = point_s(k.to_point);
    double along = s - s0;
    if (closed_) {
      if (s1 <= s0) {
        s1 += length_;
      }
      if (along < 0.0) {
        along += length_;
      }
    }
    const double kerb_length = s1 - s0;
    if (along < 0.0 || along > kerb_length) {
      continue;
    }
    constexpr double kRamp = 1.5, kRidge = 0.5;
    const double pi = std::numbers::pi;
    const double ramp_in = std::min(1.0, along / kRamp), ramp_out = std::min(1.0, (kerb_length - along) / kRamp);
    const double ramp = std::min(ramp_in, ramp_out);
    if (e >= 0.0 && e <= k.width) {
      const double dramp =
          ramp_in < ramp_out ? (ramp_in < 1.0 ? 1.0 / kRamp : 0.0) : (ramp_out < 1.0 ? -1.0 / kRamp : 0.0);
      const double cross = std::sin(pi * e / k.width), dcross = pi / k.width * std::cos(pi * e / k.width);
      const double ridge = 0.8 + 0.2 * std::cos(2.0 * pi * along / kRidge);
      const double dridge = -0.2 * 2.0 * pi / kRidge * std::sin(2.0 * pi * along / kRidge);
      g.z += k.height * ramp * cross * ridge;
      g.dzds += k.height * cross * (dramp * ridge + ramp * dridge);
      g.dzdn += k.height * ramp * ridge * dcross * (left ? 1.0 : -1.0);
    }
    return k.width * ramp;
  }
  return 0.0;
}

double Track::kerb_width(double s, bool left) const {
  Ground scratch{0.0, 0.0, 0.0};
  return kerb_at(wrap(s), -1.0, left, scratch);
}

double Track::surface_height(double s, double n) const {
  Ground g = centre_ground(s, n);
  const Widths w = widths_at(s);
  if (n > w.left) {
    kerb_at(s, n - w.left, true, g);
  } else if (n < -w.right) {
    kerb_at(s, -n - w.right, false, g);
  }
  return g.z;
}

// Frenet coordinates with each sample interval taken as a circular arc (its
// heading change over its length), so s and n are smooth inside it and the
// height's gradient follows exactly: ds/dp = t / (1 - kappa n), dn/dp = n.
Track::Frenet Track::frenet(double x, double y, double* hint) const {
  const CurveProjection p = hint ? project(x, y, *hint, 2.0) : project(x, y);
  const std::size_t count = samples_.size();
  const std::size_t intervals = closed_ ? count : count - 1;
  std::size_t i = std::min(index_at(p.s), intervals - 1);
  Frenet f;
  for (int attempt = 0; attempt < 3; ++attempt) {
    const CurveSample& a = samples_[i];
    const std::size_t j = (i + 1) % count;
    const double span = (i + 1 < count ? samples_[j].s : length_) - a.s;
    const double turn = wrap_angle(samples_[j].heading - a.heading);
    const double kappa = span > 0.0 ? turn / span : 0.0;
    const double dx = x - a.x, dy = y - a.y;
    const double ca = std::cos(a.heading), sa = std::sin(a.heading);
    double along = 0.0;
    if (std::abs(kappa) < 1e-9) {
      along = ca * dx + sa * dy;
      f.n = -sa * dx + ca * dy;
    } else {
      const double r = 1.0 / kappa;  // signed: centre to the left when positive
      const double vx = dx + r * sa, vy = dy - r * ca;  // from the centre
      const double theta = std::atan2(vy, vx) - std::atan2(-r * ca, r * sa);  // from a's angle
      along = r * wrap_angle(theta);
      f.n = r - (r > 0.0 ? 1.0 : -1.0) * std::hypot(vx, vy);
    }
    f.kappa = kappa;
    f.heading = a.heading + kappa * along;
    f.s = a.s + along;
    // Past this interval: move to the neighbour (open ends extend straight on).
    if (along < 0.0 && (closed_ || i > 0) && attempt < 2) {
      i = (i + intervals - 1) % intervals;
    } else if (along > span && (closed_ || i + 1 < intervals) && attempt < 2) {
      i = (i + 1) % intervals;
    } else {
      break;
    }
  }
  f.s = wrap(f.s);
  if (hint) {
    *hint = f.s;
  }
  return f;
}

RoadPoint Track::road(double x, double y, double* hint) const {
  const Frenet p = frenet(x, y, hint);
  RoadPoint out;
  out.s = p.s;
  out.n = p.n;
  Ground g = centre_ground(p.s, p.n);
  const Widths w = widths_at(p.s);
  if (p.n <= w.left && p.n >= -w.right) {
    out.kind = SurfaceKind::Road;
    add_waves(road_waves_, p.s, p.n, g);
    out.mu = def_.surface.mu;
    for (const TrackDefinition::Patch& patch : def_.patches) {
      double along = p.s - point_s(patch.from_point);
      double length = point_s(patch.to_point) - point_s(patch.from_point);
      if (closed_) {
        along = along < 0.0 ? along + length_ : along;
        length = length <= 0.0 ? length + length_ : length;
      }
      if (along >= 0.0 && along <= length && p.n >= patch.n_min && p.n <= patch.n_max) {
        out.mu = patch.mu;
      }
    }
  } else {
    // Beyond the edge: a kerb if there is one, then grass. The road's
    // roughness hands over to the grass's within kFade, so the height has no
    // step anywhere.
    constexpr double kFade = 1.5;  // [m]
    const bool left = p.n > w.left;
    const double e = left ? p.n - w.left : -p.n - w.right;
    const double kerb = kerb_at(p.s, e, left, g);
    const double u = std::clamp((e - kerb) / kFade, 0.0, 1.0);
    const double fade = u * u * (3.0 - 2.0 * u);
    const double dfade_dn = (u > 0.0 && u < 1.0 ? 6.0 * u * (1.0 - u) / kFade : 0.0) * (left ? 1.0 : -1.0);
    Ground road{0.0, 0.0, 0.0}, grass{0.0, 0.0, 0.0};
    if (fade < 1.0) {
      add_waves(road_waves_, p.s, p.n, road);
    }
    if (fade > 0.0) {
      add_waves(grass_waves_, p.s, p.n, grass);
    }
    g.z += (1.0 - fade) * road.z + fade * grass.z;
    g.dzds += (1.0 - fade) * road.dzds + fade * grass.dzds;
    g.dzdn += (1.0 - fade) * road.dzdn + fade * grass.dzdn + dfade_dn * (grass.z - road.z);
    if (e <= kerb) {
      out.kind = SurfaceKind::Kerb;
      out.mu = 0.9 * def_.surface.mu;  // painted
    } else {
      out.kind = SurfaceKind::Grass;
      out.mu = def_.surface.grass_mu;
    }
  }
  out.z = g.z;
  const double c = std::cos(p.heading), sn = std::sin(p.heading);
  const double metric = 1.0 / std::max(0.1, 1.0 - p.kappa * p.n);
  out.dzdx = g.dzds * metric * c - g.dzdn * sn;
  out.dzdy = g.dzds * metric * sn + g.dzdn * c;
  return out;
}

Track::Widths Track::widths_at(double s) const {
  s = wrap(s);
  const std::size_t i = index_at(s);
  const bool last = i + 1 >= samples_.size();
  if (last && !closed_) {
    return widths_.back();
  }
  const std::size_t next = last ? 0 : i + 1;
  const double span = (last ? length_ : samples_[next].s) - samples_[i].s;
  const double f = span > 0.0 ? (s - samples_[i].s) / span : 0.0;
  return {widths_[i].left + f * (widths_[next].left - widths_[i].left),
          widths_[i].right + f * (widths_[next].right - widths_[i].right)};
}

double Track::point_s(int index) const {
  return point_s_.at(static_cast<std::size_t>(index));
}

double Track::finish_s() const {
  if (closed_) {
    return start_s();
  }
  return def_.finish_point >= 0 ? point_s(def_.finish_point) : length_;
}

std::vector<double> Track::sector_s() const {
  std::vector<double> out;
  for (const int index : def_.sector_points) {
    out.push_back(point_s(index));
  }
  return out;
}

double Track::line_offset(const TrackDefinition::Line& line, double s) const {
  const auto& offsets = line.offsets;
  if (offsets.empty()) {
    return 0.0;
  }
  if (offsets.size() == 1) {
    return offsets.front().second;
  }
  std::vector<std::pair<double, double>> sorted = offsets;
  std::sort(sorted.begin(), sorted.end());
  const double u = wrap(s) / length_;
  std::vector<double> knots_u, values;
  for (const auto& [ku, kn] : sorted) {
    if (!knots_u.empty() && ku - knots_u.back() < 1e-9) {
      continue;
    }
    knots_u.push_back(ku);
    values.push_back(kn);
  }
  if (closed_ && knots_u.size() >= 3) {
    // Periodic spline in u with the first knot repeated one lap later.
    knots_u.push_back(knots_u.front() + 1.0);
    values.push_back(values.front());
    const std::vector<double> m = spline_moments(knots_u, values, true);
    double uu = u;
    if (uu < knots_u.front()) {
      uu += 1.0;
    }
    std::size_t i = 0;
    while (i + 2 < knots_u.size() && knots_u[i + 1] < uu) {
      ++i;
    }
    return spline_eval(knots_u[i], knots_u[i + 1], values[i], values[i + 1], m[i], m[i + 1], uu).v;
  }
  if (knots_u.size() >= 3) {
    const std::vector<double> m = spline_moments(knots_u, values, false);
    if (u <= knots_u.front()) {
      return values.front();
    }
    if (u >= knots_u.back()) {
      return values.back();
    }
    std::size_t i = 0;
    while (i + 2 < knots_u.size() && knots_u[i + 1] < u) {
      ++i;
    }
    return spline_eval(knots_u[i], knots_u[i + 1], values[i], values[i + 1], m[i], m[i + 1], u).v;
  }
  // Two knots: linear (wrapping on closed tracks).
  const auto [u0, n0] = std::pair{knots_u[0], values[0]};
  const auto [u1, n1] = std::pair{knots_u[1], values[1]};
  if (!closed_) {
    return n0 + (n1 - n0) * std::clamp((u - u0) / (u1 - u0), 0.0, 1.0);
  }
  if (u >= u0 && u <= u1) {
    return n0 + (n1 - n0) * (u - u0) / (u1 - u0);
  }
  const double gap = 1.0 - (u1 - u0);
  const double along = u > u1 ? u - u1 : u + 1.0 - u1;
  return n1 + (n0 - n1) * along / gap;
}

SampledCurve Track::line(std::string_view name) const {
  if (name == "center") {
    return SampledCurve(samples_, closed_);
  }
  const auto it = std::find_if(def_.lines.begin(), def_.lines.end(),
                               [&](const TrackDefinition::Line& line) { return line.name == name; });
  if (it == def_.lines.end()) {
    std::string known = "center";
    for (const auto& line : def_.lines) {
      known += ", " + line.name;
    }
    throw std::invalid_argument("track '" + def_.name + "' has no line '" + std::string(name) + "' (has " +
                                known + ")");
  }
  std::vector<double> xs, ys;
  for (const CurveSample& c : samples_) {
    const double offset = line_offset(*it, c.s);
    xs.push_back(c.x - offset * std::sin(c.heading));
    ys.push_back(c.y + offset * std::cos(c.heading));
  }
  return SampledCurve(resample_polyline(xs, ys, closed_, kStep), closed_);
}

std::vector<std::string> Track::warnings() const {
  std::vector<std::string> out;
  double tightest = 0.0;
  double tightest_s = 0.0;
  bool folded = false;
  double folded_s = 0.0;
  for (std::size_t k = 0; k < samples_.size(); ++k) {
    const double kappa = samples_[k].curvature;
    if (std::abs(kappa) > std::abs(tightest)) {
      tightest = kappa;
      tightest_s = samples_[k].s;
    }
    const double inner = kappa > 0.0 ? widths_[k].left : widths_[k].right;
    if (!folded && std::abs(kappa) * inner > 0.95) {
      folded = true;
      folded_s = samples_[k].s;
    }
  }
  if (std::abs(tightest) > 1.0 / 4.5) {
    out.push_back("tightest centerline radius " + std::to_string(1.0 / std::abs(tightest)).substr(0, 4) +
                  " m at s = " + std::to_string(static_cast<int>(tightest_s)) +
                  " m is below 4.5 m (a formula car's turning circle)");
  }
  if (folded) {
    out.push_back("the inner boundary folds over itself near s = " + std::to_string(static_cast<int>(folded_s)) +
                  " m (width larger than the radius)");
  }
  return out;
}

std::optional<std::filesystem::path> resolve_track_file(const std::string& file, const std::string& template_name,
                                                        const std::filesystem::path& scenario_dir) {
  if (!file.empty()) {
    std::filesystem::path path = path_from_utf8(file);
    return path.is_relative() ? scenario_dir / path : path;
  }
  if (!template_name.empty()) {
    const std::filesystem::path dir = find_resource_dir("templates/tracks", scenario_dir);
    if (dir.empty()) {
      throw std::invalid_argument("track.template '" + template_name +
                                  "': templates/tracks not found (run from the source tree or use track.file)");
    }
    const std::filesystem::path path = dir / path_from_utf8(template_name + ".track.json");
    if (!std::filesystem::exists(path)) {
      throw std::invalid_argument("track.template '" + template_name + "' not found in " + path_to_utf8(dir));
    }
    return path;
  }
  return std::nullopt;
}

}  // namespace vsim
