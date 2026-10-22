#include "vsim/track/track.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <numbers>
#include <sstream>
#include <stdexcept>

#include "vsim/util/paths.hpp"

namespace vsim {
namespace {

using nlohmann::json;

double wrap_angle(double a) {
  return std::remainder(a, 2.0 * std::numbers::pi);
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
    const double d = std::hypot(x - px, y - py);
    if (d < best) {
      best = d;
      const double span = (i + 1 < n ? b.s : length_) - a.s;
      result.s = a.s + f * span;
      result.heading = a.heading + f * wrap_angle(b.heading - a.heading);
      result.n = -(x - px) * std::sin(result.heading) + (y - py) * std::cos(result.heading);
      result.distance = d;
    }
  }
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
  const std::vector<double> mx = spline_moments(t, xs, def_.closed);
  const std::vector<double> my = spline_moments(t, ys, def_.closed);

  // Dense samples along each segment, arc length from their chords.
  struct Dense {
    double s, x, y, heading, curvature;
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
      dense.push_back({s, vx.v, vy.v, std::atan2(vy.d1, vx.d1), curvature, i});
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
  }
  samples_ = std::move(samples);
  closed_ = def_.closed;
  length_ = total;
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
