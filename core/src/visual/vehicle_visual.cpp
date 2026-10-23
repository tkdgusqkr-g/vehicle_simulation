#include "vsim/visual/vehicle_visual.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace vsim {
namespace {

using geometry::ellipsoid;
using geometry::lathe;
using geometry::loft;
using geometry::superellipse_ring;
using geometry::tube;

constexpr double kPi = std::numbers::pi;

struct Dimensions {
  double wheelbase, a, b, cg_height, track_front, track_rear;
  double radius;           // loaded: hub height
  double unloaded_radius;  // tyre drawn with this
  double rim_radius, tyre_width;
  double xf() const { return a; }   // front axle in the car frame
  double xr() const { return -b; }  // rear axle
};

Dimensions read_dimensions(const ParamTree& p) {
  Dimensions d{};
  d.wheelbase = p.get_double("vehicle.wheelbase_m", 1.53);
  d.a = p.get_double("vehicle.cg_to_front_axle_m", 0.5 * d.wheelbase);
  d.b = d.wheelbase - d.a;
  d.cg_height = p.get_double("vehicle.cg_height_m", 0.28);
  d.track_front = p.get_double("vehicle.track_front_m", 1.22);
  d.track_rear = p.get_double("vehicle.track_rear_m", 1.20);
  d.radius = p.get_double("vehicle.wheel.radius_m", 0.203);
  d.unloaded_radius = p.get_double("vehicle.wheel.unloaded_radius_m", std::max(d.radius, 0.2286));
  d.rim_radius = 0.5 * 0.0254 * p.get_double("vehicle.wheel.rim_diameter_in", 10.0);
  d.tyre_width = p.get_double("vehicle.tire.width_m", 0.19);
  return d;
}

std::array<float, 3> hex_colour(const std::string& text, std::array<float, 3> fallback) {
  if (text.size() != 7 || text[0] != '#') {
    return fallback;
  }
  std::array<float, 3> c{};
  for (int i = 0; i < 3; ++i) {
    c[i] = static_cast<float>(std::stoi(text.substr(1 + 2 * i, 2), nullptr, 16)) / 255.0f;
  }
  return c;
}

// Default hardpoints (left side; x forward from the axle centre, y left from
// the centreline, z up from the ground), typical of a 10" FS car.
struct Hardpoints {
  Vec3 lower_front, lower_rear, lower_outer, upper_front, upper_rear, upper_outer;
  Vec3 tie_inner, tie_outer, push_outer, push_inner, wheel_centre;
};

Hardpoints default_hardpoints(bool front) {
  if (front) {
    return {{0.13, 0.20, 0.11},    {-0.17, 0.20, 0.11},  {0.0, 0.555, 0.115}, {0.10, 0.26, 0.285},
            {-0.14, 0.26, 0.285},  {-0.012, 0.535, 0.315}, {0.075, 0.21, 0.165}, {0.075, 0.565, 0.175},
            {0.0, 0.52, 0.125},    {0.0, 0.25, 0.46},    {0.0, 0.61, 0.203}};
  }
  return {{0.17, 0.22, 0.11},   {-0.13, 0.22, 0.11},  {0.0, 0.535, 0.115},  {0.15, 0.27, 0.29},
          {-0.12, 0.27, 0.29},  {0.012, 0.515, 0.315}, {-0.09, 0.22, 0.17},  {-0.08, 0.545, 0.18},
          {0.0, 0.50, 0.125},   {0.0, 0.26, 0.44},    {0.0, 0.59, 0.203}};
}

Hardpoints read_hardpoints(const ParamTree& p, bool front) {
  Hardpoints h = default_hardpoints(front);
  const std::string base = std::string("vehicle.suspension.") + (front ? "front" : "rear") + ".hardpoints.";
  const auto read = [&](const char* name, Vec3& v) {
    const std::vector<double> xyz = p.get_double_list(base + name);
    if (xyz.size() == 3) {
      v = {xyz[0], xyz[1], xyz[2]};
    } else if (!xyz.empty()) {
      throw std::invalid_argument(base + name + " needs [x, y, z]");
    }
  };
  read("lower_front", h.lower_front);
  read("lower_rear", h.lower_rear);
  read("lower_outer", h.lower_outer);
  read("upper_front", h.upper_front);
  read("upper_rear", h.upper_rear);
  read("upper_outer", h.upper_outer);
  read("tie_inner", h.tie_inner);
  read("tie_outer", h.tie_outer);
  read("push_outer", h.push_outer);
  read("push_inner", h.push_inner);
  read("wheel_centre", h.wheel_centre);
  return h;
}

// NACA 4-digit style section, inverted (downforce), leading edge at the
// origin pointing +x, chord running to -x, rotated so the trailing edge
// rises by `angle`. Closed loop counter-clockwise in (x, z).
std::vector<std::array<double, 2>> aerofoil(double chord, double thickness, double camber, double angle) {
  constexpr int kHalf = 18;
  const double m = -camber, p = 0.4, t = thickness;
  std::vector<std::array<double, 2>> upper, lower;
  for (int i = 0; i <= kHalf; ++i) {
    const double beta = kPi * i / kHalf;
    const double s = 0.5 * (1.0 - std::cos(beta));  // cosine spacing, 0 = LE
    const double yt = 5.0 * t * (0.2969 * std::sqrt(s) - 0.1260 * s - 0.3516 * s * s + 0.2843 * s * s * s -
                                 0.1036 * s * s * s * s);
    const double yc = s < p ? m / (p * p) * (2 * p * s - s * s) : m / ((1 - p) * (1 - p)) * (1 - 2 * p + 2 * p * s - s * s);
    upper.push_back({s, yc + yt});
    lower.push_back({s, yc - yt});
  }
  std::vector<std::array<double, 2>> loop;
  // Upper surface from the trailing edge to the leading edge, then the lower back.
  for (int i = kHalf; i >= 0; --i) {
    loop.push_back(upper[static_cast<std::size_t>(i)]);
  }
  for (int i = 1; i < kHalf; ++i) {
    loop.push_back(lower[static_cast<std::size_t>(i)]);
  }
  const double c = std::cos(-angle), sn = std::sin(-angle);
  for (auto& [s, z] : loop) {
    const double x = -s * chord, zz = z * chord;  // chord runs rearwards
    s = x * c - zz * sn;
    z = x * sn + zz * c;
  }
  double area = 0.0;
  for (std::size_t i = 0; i < loop.size(); ++i) {
    const auto& q0 = loop[i];
    const auto& q1 = loop[(i + 1) % loop.size()];
    area += q0[0] * q1[1] - q1[0] * q0[1];
  }
  if (area < 0.0) {
    std::reverse(loop.begin(), loop.end());
  }
  return loop;
}

void wing_element(MeshSection& m, double le_x, double le_z, double chord, double angle, double span, double camber = 0.08) {
  auto profile = aerofoil(chord, 0.12, camber, angle);
  for (auto& [x, z] : profile) {
    x += le_x;
    z += le_z;
  }
  geometry::extrude_y(m, profile, -0.5 * span, 0.5 * span);
}

// Open grid of rows of points; winding chosen so the surface faces `hint`.
void grid(MeshSection& m, const std::vector<std::vector<Vec3>>& rows, const Vec3& hint) {
  const std::size_t first_vertex = m.vertex_count(), first_index = m.indices.size();
  const std::size_t cols = rows.front().size();
  for (const auto& row : rows) {
    for (const Vec3& p : row) {
      m.add_vertex(p, hint, 0, 0);
    }
  }
  const Vec3 n = (rows[0][1] - rows[0][0]).cross(rows[1][0] - rows[0][0]);
  const bool flip = n.dot(hint) < 0.0;
  for (std::size_t r = 0; r + 1 < rows.size(); ++r) {
    for (std::size_t c = 0; c + 1 < cols; ++c) {
      const auto a = static_cast<std::uint32_t>(first_vertex + r * cols + c);
      const auto b = a + 1, d = static_cast<std::uint32_t>(a + cols), e = d + 1;
      if (flip) {
        m.add_triangle(a, d, b);
        m.add_triangle(b, d, e);
      } else {
        m.add_triangle(a, b, d);
        m.add_triangle(b, e, d);
      }
    }
  }
  geometry::smooth_normals(m, first_vertex, first_index);
}

// Coil spring as a helix tube between two points.
void coil(MeshSection& m, const Vec3& from, const Vec3& to, double coil_radius, double wire, int turns) {
  const Vec3 axis = (to - from).normalized();
  const Vec3 u = (std::abs(axis.z) < 0.9 ? axis.cross({0, 0, 1}) : axis.cross({1, 0, 0})).normalized();
  const Vec3 v = axis.cross(u);
  std::vector<Vec3> path;
  const int steps = turns * 12;
  for (int i = 0; i <= steps; ++i) {
    const double f = static_cast<double>(i) / steps;
    const double t = 2.0 * kPi * turns * f;
    path.push_back(from + (to - from) * f + (u * std::cos(t) + v * std::sin(t)) * coil_radius);
  }
  tube(m, path, wire, 6, false);
}

// Profile (radius, y) of the tyre for a left wheel (outer face +y).
std::vector<std::array<double, 2>> tyre_profile(const Dimensions& d) {
  const double r = d.rim_radius, ru = d.unloaded_radius, w = d.tyre_width;
  const double side = r + 0.4 * (ru - r);
  return {{r + 0.006, -0.46 * w}, {side, -0.53 * w}, {ru - 0.022, -0.51 * w}, {ru - 0.005, -0.45 * w},
          {ru, -0.37 * w},        {ru, 0.37 * w},    {ru - 0.005, 0.45 * w},  {ru - 0.022, 0.51 * w},
          {side, 0.53 * w},       {r + 0.006, 0.46 * w}};
}

VisualPart make_wheel(const Dimensions& d, int corner, const Vec3& hub) {
  VisualPart part;
  part.name = std::string("wheel_") + (corner == 0 ? "fl" : corner == 1 ? "fr" : corner == 2 ? "rl" : "rr");
  part.role = PartRole::Wheel;
  part.corner = corner;
  part.pivot = hub;
  const bool right = corner % 2 == 1;
  const double r = d.rim_radius, w = d.tyre_width;
  const Vec3 axle{0, 1, 0};

  MeshSection tyre = make_section("tyre", {0.07f, 0.07f, 0.075f}, 0.0f, 0.92f);
  lathe(tyre, tyre_profile(d), {}, axle, 56);

  MeshSection rim = make_section("rim", {0.70f, 0.72f, 0.75f}, 1.0f, 0.32f);
  // Flange lip, then the barrel seen through the spokes (faces inwards).
  lathe(rim, {{r + 0.014, 0.44 * w}, {r + 0.014, 0.49 * w}, {r - 0.004, 0.49 * w}, {r - 0.004, -0.40 * w}}, {}, axle,
        48);
  // Five spokes, the hub and its face.
  for (int k = 0; k < 5; ++k) {
    const double th = 2.0 * kPi * k / 5.0 + 0.3;
    const double mid = 0.5 * (0.035 + r - 0.006);
    geometry::box(rim, {mid * std::cos(th), 0.40 * w, -mid * std::sin(th)}, {0.5 * (r - 0.041), 0.007, 0.011}, 0.0,
                  th);
  }
  lathe(rim, {{0.040, 0.30 * w}, {0.040, 0.44 * w}, {0.0, 0.44 * w}}, {}, axle, 24);

  MeshSection nut = make_section("nut", hex_colour("#d9a441", {0.85f, 0.64f, 0.25f}), 1.0f, 0.3f);
  lathe(nut, {{0.022, 0.43 * w}, {0.022, 0.50 * w}, {0.0, 0.50 * w}}, {}, axle, 6);

  MeshSection disc = make_section("disc", {0.55f, 0.55f, 0.57f}, 0.9f, 0.45f);
  lathe(disc, {{0.060, -0.30 * w - 0.004}, {0.105, -0.30 * w - 0.004}, {0.105, -0.30 * w + 0.004},
               {0.060, -0.30 * w + 0.004}, {0.060, -0.30 * w - 0.004}},
        {}, axle, 40);

  for (MeshSection* m : {&tyre, &rim, &nut, &disc}) {
    if (right) {
      geometry::mirror_y(*m, 0, 0);
    }
    part.sections.push_back(std::move(*m));
  }
  return part;
}

VisualPart make_upright(const Dimensions& d, int corner, const Vec3& hub, const std::array<float, 3>& caliper_colour) {
  VisualPart part;
  part.name = std::string("upright_") + (corner == 0 ? "fl" : corner == 1 ? "fr" : corner == 2 ? "rl" : "rr");
  part.role = PartRole::Upright;
  part.corner = corner;
  part.pivot = hub;
  const bool right = corner % 2 == 1;
  const double w = d.tyre_width;

  MeshSection motor = make_section("motor", {0.18f, 0.19f, 0.21f}, 0.8f, 0.4f);
  // Hub motor and gearbox inside the rim.
  lathe(motor, {{0.0, -0.46 * w}, {0.085, -0.46 * w}, {0.085, -0.05 * w}, {0.05, 0.05 * w}, {0.0, 0.05 * w}}, {},
        {0, 1, 0}, 32);
  MeshSection upright = make_section("upright", {0.62f, 0.64f, 0.67f}, 0.9f, 0.35f);
  geometry::box(upright, {0.0, -0.52 * w, 0.01}, {0.03, 0.012, 0.10});
  geometry::box(upright, {0.06, -0.52 * w, -0.03}, {0.05, 0.01, 0.012});  // steering arm
  MeshSection caliper = make_section("caliper", caliper_colour, 0.3f, 0.35f);
  geometry::box(caliper, {-0.075, -0.30 * w, 0.055}, {0.028, 0.018, 0.045}, 0.0, -0.6);
  for (MeshSection* m : {&motor, &upright, &caliper}) {
    if (right) {
      geometry::mirror_y(*m, 0, 0);
    }
    part.sections.push_back(std::move(*m));
  }
  return part;
}

}  // namespace

Vec3 hub_position(const ParamTree& params, int corner) {
  const Dimensions d = read_dimensions(params);
  const bool front = corner < 2;
  const double x = front ? d.xf() : d.xr();
  const double half = 0.5 * (front ? d.track_front : d.track_rear);
  return {x, corner % 2 == 0 ? half : -half, d.radius};
}

VisualModel make_formula_car(const ParamTree& params) {
  const Dimensions d = read_dimensions(params);
  VisualModel model;
  model.name = "formula student car (procedural)";
  model.cg_height = d.cg_height;
  const auto paint_colour = hex_colour(params.get_string("vehicle.visual.color", "#d7263d"), {0.84f, 0.15f, 0.24f});
  const auto accent_colour = hex_colour(params.get_string("vehicle.visual.accent", "#f2f4f7"), {0.95f, 0.96f, 0.97f});
  const double xf = d.xf(), xr = d.xr();

  VisualPart body;
  body.name = "body";
  body.role = PartRole::Body;
  MeshSection paint = make_section("paint", paint_colour, 0.25f, 0.28f);
  MeshSection accent = make_section("paint_accent", accent_colour, 0.1f, 0.3f);
  MeshSection carbon = make_section("carbon", {0.08f, 0.085f, 0.09f}, 0.3f, 0.35f);
  MeshSection cockpit = make_section("cockpit", {0.03f, 0.03f, 0.035f}, 0.0f, 0.95f, kCastShadow | kDoubleSided);
  MeshSection steel = make_section("steel", {0.72f, 0.73f, 0.75f}, 0.95f, 0.3f);
  MeshSection suit = make_section("driver_suit", {0.12f, 0.23f, 0.45f}, 0.0f, 0.85f);
  MeshSection helmet = make_section("helmet", accent_colour, 0.2f, 0.18f);
  MeshSection visor = make_section("visor", {0.02f, 0.02f, 0.03f}, 0.6f, 0.08f);
  MeshSection glove = make_section("glove", {0.1f, 0.1f, 0.1f}, 0.0f, 0.8f);

  // Monocoque: superellipse stations, (x, half width, half height, centre z, exponent).
  struct Station {
    double x, a, b, zc, e;
  };
  const std::vector<Station> stations = {
      {xf + 0.74, 0.035, 0.03, 0.19, 2.0}, {xf + 0.66, 0.10, 0.08, 0.205, 2.4}, {xf + 0.48, 0.17, 0.13, 0.24, 2.8},
      {xf + 0.22, 0.225, 0.18, 0.285, 3.2}, {xf - 0.10, 0.265, 0.22, 0.325, 3.5}, {xf - 0.42, 0.295, 0.255, 0.355, 3.6},
      {xf - 0.80, 0.315, 0.27, 0.365, 3.6}, {xf - 1.10, 0.31, 0.27, 0.36, 3.6},  {xr + 0.28, 0.29, 0.24, 0.345, 3.4},
      {xr + 0.02, 0.24, 0.20, 0.32, 3.0},   {xr - 0.18, 0.15, 0.13, 0.30, 2.6},  {xr - 0.27, 0.06, 0.06, 0.30, 2.2}};
  {
    std::vector<std::vector<Vec3>> rings;
    for (auto it = stations.rbegin(); it != stations.rend(); ++it) {  // rear to front: advance +x
      rings.push_back(superellipse_ring(it->x, 0.0, it->zc, it->a, it->b, it->e, 48));
    }
    loft(paint, rings, true, true);
  }
  // Cockpit opening: a dark patch hugging the top between the hoops.
  {
    std::vector<std::vector<Vec3>> rows;
    const double x0 = xf - 0.38, x1 = xf - 1.08;
    for (int k = 0; k <= 12; ++k) {
      const double f = k / 12.0;
      const double x = x0 + (x1 - x0) * f;
      // Interpolate the body section at x.
      Station s = stations.front();
      for (std::size_t i = 0; i + 1 < stations.size(); ++i) {
        if (x <= stations[i].x && x >= stations[i + 1].x) {
          const double g = (stations[i].x - x) / (stations[i].x - stations[i + 1].x);
          s = {x, stations[i].a + g * (stations[i + 1].a - stations[i].a), stations[i].b + g * (stations[i + 1].b - stations[i].b),
               stations[i].zc + g * (stations[i + 1].zc - stations[i].zc), stations[i].e + g * (stations[i + 1].e - stations[i].e)};
          break;
        }
      }
      // Opening narrower at the front, rounded ends.
      const double width = 0.78 * std::sqrt(std::max(0.0, std::sin(kPi * std::min(1.0, f * 1.05 + 0.02)))) * (0.75 + 0.25 * f);
      std::vector<Vec3> row;
      for (int j = 0; j <= 10; ++j) {
        const double y = s.a * width * (-1.0 + 2.0 * j / 10.0);
        const double yn = std::min(1.0, std::abs(y) / s.a);
        const double z = s.zc + s.b * std::pow(std::max(0.0, 1.0 - std::pow(yn, s.e)), 1.0 / s.e) + 0.004;
        row.push_back({x, y, z});
      }
      rows.push_back(std::move(row));
    }
    grid(cockpit, rows, {0, 0, 1});
  }
  // Seat back and head rest.
  geometry::box(cockpit, {xf - 1.10, 0.0, 0.80}, {0.03, 0.13, 0.11}, 0.0, -0.2);

  // Side pods (left, then mirrored), inlet face dark.
  for (const double side : {1.0, -1.0}) {
    MeshSection pod = make_section("paint", paint_colour, 0.25f, 0.28f);
    const std::vector<Station> pod_stations = {{xf - 0.40, 0.10, 0.12, 0.25, 4.0}, {xf - 0.47, 0.12, 0.15, 0.25, 4.0},
                                               {xf - 0.85, 0.125, 0.155, 0.25, 4.0}, {xf - 1.12, 0.10, 0.12, 0.24, 3.5},
                                               {xf - 1.28, 0.05, 0.06, 0.22, 3.0}};
    std::vector<std::vector<Vec3>> rings;
    for (auto it = pod_stations.rbegin(); it != pod_stations.rend(); ++it) {
      rings.push_back(superellipse_ring(it->x, 0.43, it->zc, it->a, it->b, it->e, 32));
    }
    loft(pod, rings, true, false);
    std::vector<Vec3> inlet = superellipse_ring(pod_stations.front().x + 0.001, 0.43, 0.25, 0.09, 0.105, 4.0, 32);
    std::vector<Vec3> inner = superellipse_ring(pod_stations.front().x - 0.05, 0.43, 0.25, 0.085, 0.10, 4.0, 32);
    MeshSection hole = make_section("cockpit", {0.03f, 0.03f, 0.035f}, 0.0f, 0.95f, kCastShadow | kDoubleSided);
    loft(hole, {inner, inlet}, true, false);
    if (side < 0) {
      geometry::mirror_y(pod, 0, 0);
      geometry::mirror_y(hole, 0, 0);
    }
    for (MeshSection* m : {&pod, &hole}) {
      MeshSection& target = m == &pod ? paint : cockpit;
      const auto offset = static_cast<std::uint32_t>(target.vertex_count());
      target.positions.insert(target.positions.end(), m->positions.begin(), m->positions.end());
      target.normals.insert(target.normals.end(), m->normals.begin(), m->normals.end());
      target.uvs.insert(target.uvs.end(), m->uvs.begin(), m->uvs.end());
      for (const std::uint32_t i : m->indices) {
        target.indices.push_back(i + offset);
      }
    }
  }
  // Livery stripe along the top of the nose.
  {
    std::vector<std::vector<Vec3>> rows;
    for (int k = 0; k <= 10; ++k) {
      const double x = xf + 0.66 - (1.0 * k / 10.0);
      Station s = stations.front();
      for (std::size_t i = 0; i + 1 < stations.size(); ++i) {
        if (x <= stations[i].x && x >= stations[i + 1].x) {
          const double g = (stations[i].x - x) / (stations[i].x - stations[i + 1].x);
          s = {x, stations[i].a + g * (stations[i + 1].a - stations[i].a), stations[i].b + g * (stations[i + 1].b - stations[i].b),
               stations[i].zc + g * (stations[i + 1].zc - stations[i].zc), stations[i].e + g * (stations[i + 1].e - stations[i].e)};
          break;
        }
      }
      std::vector<Vec3> row;
      for (int j = 0; j <= 4; ++j) {
        const double y = 0.06 * (-1.0 + j / 2.0);
        const double yn = std::min(1.0, std::abs(y) / s.a);
        row.push_back({x, y, s.zc + s.b * std::pow(std::max(0.0, 1.0 - std::pow(yn, s.e)), 1.0 / s.e) + 0.002});
      }
      rows.push_back(std::move(row));
    }
    grid(accent, rows, {0, 0, 1});
  }

  // Front wing: three elements and end plates, inside the front tyres.
  const double front_span = d.track_front + d.tyre_width - 0.03;
  wing_element(carbon, xf + 0.84, 0.065, 0.30, 0.05, front_span);
  wing_element(carbon, xf + 0.56, 0.105, 0.17, 0.38, front_span);
  wing_element(carbon, xf + 0.43, 0.165, 0.12, 0.70, front_span);
  for (const double side : {1.0, -1.0}) {
    geometry::box(carbon, {xf + 0.62, side * 0.5 * front_span, 0.17}, {0.24, 0.004, 0.14});
  }
  // Rear wing, at most 1.2 m high and 250 mm behind the rear tyres.
  const double rear_span = std::min(1.0, d.track_rear - 0.1);
  wing_element(carbon, xr + 0.10, 0.88, 0.42, 0.10, rear_span);
  wing_element(carbon, xr - 0.24, 0.955, 0.24, 0.56, rear_span);
  wing_element(carbon, xr - 0.36, 1.035, 0.16, 0.90, rear_span);
  for (const double side : {1.0, -1.0}) {
    geometry::box(carbon, {xr - 0.17, side * 0.5 * rear_span, 0.94}, {0.28, 0.004, 0.24});
    geometry::box(carbon, {xr - 0.12, side * 0.13, 0.62}, {0.05, 0.006, 0.30});  // pylons
  }
  // Undertray and diffuser.
  geometry::box(carbon, {0.5 * (xf + 0.25 + xr + 0.15), 0.0, 0.035}, {0.5 * (xf + 0.25 - xr - 0.15), 0.50, 0.004});
  geometry::box(carbon, {xr - 0.08, 0.0, 0.09}, {0.25, 0.48, 0.004}, 0.0, 0.26);  // rises to the rear
  for (const double side : {1.0, -1.0}) {
    geometry::box(carbon, {0.5 * (xf + xr), side * 0.50, 0.07}, {0.5 * (xf - xr) + 0.1, 0.004, 0.04});
  }

  // Roll hoops and main hoop braces.
  {
    const double xm = xf - 1.12;
    tube(steel, {{xm, 0.28, 0.38}, {xm, 0.265, 0.78}, {xm, 0.23, 0.98}, {xm - 0.015, 0.14, 1.07}, {xm - 0.02, 0.0, 1.09},
                 {xm - 0.015, -0.14, 1.07}, {xm, -0.23, 0.98}, {xm, -0.265, 0.78}, {xm, -0.28, 0.38}},
         0.0127, 10);
    for (const double side : {1.0, -1.0}) {
      tube(steel, {{xm - 0.01, side * 0.21, 1.01}, {xm - 0.45, side * 0.24, 0.47}}, 0.0095, 8);
    }
    const double xh = xf - 0.40;
    tube(steel, {{xh, 0.27, 0.46}, {xh, 0.235, 0.62}, {xh, 0.12, 0.70}, {xh, 0.0, 0.715}, {xh, -0.12, 0.70},
                 {xh, -0.235, 0.62}, {xh, -0.27, 0.46}},
         0.0127, 10);
  }
  // Driver.
  ellipsoid(suit, {xf - 1.00, 0.0, 0.62}, {0.13, 0.20, 0.20}, 12, 20);
  ellipsoid(helmet, {xf - 0.97, 0.0, 0.93}, {0.145, 0.125, 0.14}, 14, 24);
  ellipsoid(visor, {xf - 0.895, 0.0, 0.925}, {0.085, 0.105, 0.05}, 10, 20);
  for (const double side : {1.0, -1.0}) {
    tube(suit, {{xf - 0.98, side * 0.17, 0.70}, {xf - 0.74, side * 0.23, 0.60}, {xf - 0.47, side * 0.12, 0.645}}, 0.045, 10);
    ellipsoid(glove, {xf - 0.455, side * 0.12, 0.645}, {0.04, 0.035, 0.045}, 8, 12);
  }

  // Dampers with springs on the rockers.
  for (const bool front : {true, false}) {
    const Hardpoints h = read_hardpoints(params, front);
    const double ax = front ? xf : xr;
    for (const double side : {1.0, -1.0}) {
      const Vec3 rocker{ax + h.push_inner.x, side * h.push_inner.y, h.push_inner.z};
      const Vec3 mount = front ? Vec3{ax + 0.03, side * 0.04, h.push_inner.z + 0.05}
                               : Vec3{ax + 0.26, side * 0.07, h.push_inner.z - 0.02};
      geometry::box(steel, rocker, {0.03, 0.025, 0.03});
      tube(carbon, {rocker, rocker + (mount - rocker) * 0.55}, 0.016, 12);
      tube(steel, {rocker + (mount - rocker) * 0.5, mount}, 0.007, 8);
      coil(accent, rocker + (mount - rocker) * 0.12, rocker + (mount - rocker) * 0.75, 0.026, 0.004, 7);
    }
  }

  for (MeshSection* m : {&paint, &accent, &carbon, &cockpit, &steel, &suit, &helmet, &visor, &glove}) {
    if (!m->indices.empty()) {
      body.sections.push_back(std::move(*m));
    }
  }
  model.parts.push_back(std::move(body));

  // Steering wheel facing the driver.
  {
    VisualPart wheel;
    wheel.name = "steering_wheel";
    wheel.role = PartRole::SteeringWheel;
    wheel.pivot = {xf - 0.43, 0.0, 0.645};
    const double tilt = 0.26;  // the face leans back towards the driver
    wheel.axis = Vec3{-std::cos(tilt), 0.0, std::sin(tilt)}.normalized();
    const Vec3 lateral{0, 1, 0};
    const Vec3 up = lateral.cross(wheel.axis);  // in the wheel plane
    MeshSection rim_section = make_section("carbon", {0.08f, 0.085f, 0.09f}, 0.3f, 0.35f);
    MeshSection grip = make_section("grip", {0.05f, 0.05f, 0.05f}, 0.0f, 0.9f);
    MeshSection display = make_section("display", {0.02f, 0.03f, 0.04f}, 0.2f, 0.15f);
    std::vector<Vec3> rim_path;
    for (int i = 0; i <= 40; ++i) {
      const double t = 2.0 * kPi * i / 40.0;
      const double c = std::cos(t), s = std::sin(t);
      const double u = 0.135 * (c >= 0 ? 1 : -1) * std::pow(std::abs(c), 0.5);
      const double v = 0.085 * (s >= 0 ? 1 : -1) * std::pow(std::abs(s), 0.5);
      rim_path.push_back(lateral * u + up * v);
    }
    tube(rim_section, rim_path, 0.012, 8, false);
    for (const double side : {1.0, -1.0}) {
      tube(grip, {lateral * (side * 0.135) + up * -0.05, lateral * (side * 0.135) + up * 0.05}, 0.02, 12);
    }
    // Plates in the wheel plane: their thin x axis pitched onto the wheel axis.
    geometry::box(rim_section, {}, {0.012, 0.10, 0.055}, 0.0, tilt);
    geometry::box(display, wheel.axis * 0.014, {0.003, 0.045, 0.025}, 0.0, tilt);
    wheel.sections.push_back(std::move(rim_section));
    wheel.sections.push_back(std::move(grip));
    wheel.sections.push_back(std::move(display));
    model.parts.push_back(std::move(wheel));
  }

  // Corners: wheel, upright, suspension links.
  const auto caliper = hex_colour(params.get_string("vehicle.visual.caliper", "#e0a020"), {0.88f, 0.63f, 0.13f});
  for (int corner = 0; corner < 4; ++corner) {
    const bool front = corner < 2;
    const double side = corner % 2 == 0 ? 1.0 : -1.0;
    const Vec3 hub = hub_position(params, corner);
    model.parts.push_back(make_wheel(d, corner, hub));
    model.parts.push_back(make_upright(d, corner, hub, caliper));
    const Hardpoints h = read_hardpoints(params, front);
    const double ax = front ? xf : xr;
    const auto inboard = [&](const Vec3& p) { return Vec3{ax + p.x, side * p.y, p.z}; };
    const auto outboard = [&](const Vec3& p) {
      const Vec3 rel = p - h.wheel_centre;
      return Vec3{rel.x, side * rel.y, rel.z};
    };
    struct LinkSpec {
      const char* name;
      Vec3 in, out;
      bool steers;
      double radius;
    };
    const LinkSpec links[] = {
        {"lower_front", h.lower_front, h.lower_outer, false, 0.009},
        {"lower_rear", h.lower_rear, h.lower_outer, false, 0.009},
        {"upper_front", h.upper_front, h.upper_outer, false, 0.009},
        {"upper_rear", h.upper_rear, h.upper_outer, false, 0.009},
        {front ? "tie_rod" : "toe_link", h.tie_inner, h.tie_outer, front, 0.008},
        {"pushrod", h.push_inner, h.push_outer, false, 0.008},
    };
    const char* corner_name = corner == 0 ? "fl" : corner == 1 ? "fr" : corner == 2 ? "rl" : "rr";
    for (const LinkSpec& l : links) {
      VisualPart link;
      link.name = std::string(l.name) + "_" + corner_name;
      link.role = PartRole::Link;
      link.corner = corner;
      link.pivot = inboard(l.in);
      link.outer = outboard(l.out);
      link.steers = l.steers;
      MeshSection rod = make_section("steel", {0.72f, 0.73f, 0.75f}, 0.95f, 0.3f);
      tube(rod, {{0, 0, 0}, {1, 0, 0}}, l.radius, 10);
      link.sections.push_back(std::move(rod));
      model.parts.push_back(std::move(link));
    }
  }
  return model;
}

}  // namespace vsim
