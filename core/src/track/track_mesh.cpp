#include "vsim/track/track_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <numbers>
#include <unordered_map>

#include "vsim/visual/gltf.hpp"

namespace vsim {
namespace {

// Heights keep coplanar layers from flickering.
constexpr float kGroundZ = -0.03f;
constexpr float kRoadZ = 0.0f;
constexpr float kPaintZ = 0.004f;

// FS cone dimensions (small cones), big orange ones at the start.
constexpr double kConeHeight = 0.325, kConeBase = 0.228, kConeTop = 0.045;
constexpr double kBigConeHeight = 0.505, kBigConeBase = 0.285;

void add_vertex(MeshSection& m, double x, double y, double z, double nx, double ny, double nz, double u, double v) {
  m.positions.insert(m.positions.end(), {static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
  m.normals.insert(m.normals.end(), {static_cast<float>(nx), static_cast<float>(ny), static_cast<float>(nz)});
  m.uvs.insert(m.uvs.end(), {static_cast<float>(u), static_cast<float>(v)});
}

// Where a course crosses or runs over itself (skidpad), the road of one
// part covers the boundary of another: no cones or edge lines there.
class OtherRoad {
 public:
  explicit OtherRoad(const Track& track) : track_(track) {
    for (const CurveSample& q : track.samples()) {
      cos_.push_back(std::cos(q.heading));
      sin_.push_back(std::sin(q.heading));
      widths_.push_back(track.widths_at(q.s));
    }
  }

  // Whether (x, y) is within `margin` of the road of a part of the track
  // more than kGap away (along the track) from s.
  bool covers(double x, double y, double s, double margin = 0.0) const {
    const auto& samples = track_.samples();
    for (std::size_t i = 0; i < samples.size(); ++i) {
      const CurveSample& q = samples[i];
      double ds = std::abs(q.s - s);
      if (track_.closed()) {
        ds = std::min(ds, track_.length() - ds);
      }
      if (ds < kGap) {
        continue;
      }
      const double dx = x - q.x, dy = y - q.y;
      const double along = cos_[i] * dx + sin_[i] * dy;
      if (std::abs(along) > 0.5 * Track::kStep + 1e-6) {
        continue;  // not abreast of this sample
      }
      const double n = -sin_[i] * dx + cos_[i] * dy;
      if (n <= widths_[i].left + margin && n >= -widths_[i].right - margin) {
        return true;
      }
    }
    return false;
  }

 private:
  static constexpr double kGap = 15.0;  // [m] closer parts are the same stretch of road
  const Track& track_;
  std::vector<double> cos_, sin_;
  std::vector<Track::Widths> widths_;
};

// Strip between two offsets from the centerline (n positive left) over
// [s0, s1]; a loop joins its last row to the first. The offsets are functions
// of the local widths so the strip follows them. Segments with a row that
// `keep(s, x, y)` (middle of the row) rejects are left out.
template <typename Right, typename Left, typename Keep>
void add_strip(MeshSection& m, const Track& track, double s0, double s1, bool loop, double step, float z,
               Right right_of, Left left_of, Keep keep) {
  const auto count = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil((s1 - s0) / step - 1e-9)));
  const std::size_t rows = loop ? count : count + 1;
  const auto first = static_cast<std::uint32_t>(m.vertex_count());
  const std::size_t first_index = m.indices.size();
  const bool relief = track.has_relief();
  std::vector<bool> kept(rows);
  for (std::size_t k = 0; k < rows; ++k) {
    const double s = s0 + (s1 - s0) * static_cast<double>(k) / static_cast<double>(count);
    const CurveSample c = track.at(s);
    const Track::Widths w = track.widths_at(s);
    const double nx = -std::sin(c.heading), ny = std::cos(c.heading);
    const double r = right_of(w), l = left_of(w);
    const double zr = z + (relief ? track.surface_height(s, r) : 0.0);
    const double zl = z + (relief ? track.surface_height(s, l) : 0.0);
    add_vertex(m, c.x + r * nx, c.y + r * ny, zr, 0, 0, 1, 0, s / 4.0);
    add_vertex(m, c.x + l * nx, c.y + l * ny, zl, 0, 0, 1, 1, s / 4.0);
    kept[k] = keep(s, c.x + 0.5 * (r + l) * nx, c.y + 0.5 * (r + l) * ny);
  }
  for (std::size_t k = 0; k < count; ++k) {
    if (!kept[k] || !kept[(k + 1) % rows]) {
      continue;
    }
    // a: right, a + 1: left; counter-clockwise from above.
    const auto a = first + static_cast<std::uint32_t>(2 * k);
    const auto b = first + static_cast<std::uint32_t>(2 * ((k + 1) % rows));
    m.indices.insert(m.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
  }
  if (relief) {
    geometry::smooth_normals(m, first, first_index);
  }
}

constexpr auto kKeepAll = [](double, double, double) { return true; };

// Strip along the whole track.
template <typename Right, typename Left, typename Keep>
void add_strip(MeshSection& m, const Track& track, double step, float z, Right right_of, Left left_of, Keep keep) {
  add_strip(m, track, 0.0, track.length(), track.closed(), step, z, right_of, left_of, keep);
}

// A band across the track at s, `width` long along it.
void add_cross_line(MeshSection& m, const Track& track, double s, double width) {
  add_strip(m, track, s - 0.5 * width, s + 0.5 * width, false, width, kPaintZ,
            [](const Track::Widths& w) { return -w.right; }, [](const Track::Widths& w) { return w.left; }, kKeepAll);
}

// Eight-sided cone frustum with flat-shaded faces and a top cap.
void add_cone(MeshSection& m, double x, double y, double z0, double height, double base, double top) {
  constexpr int kSides = 8;
  const double slope = (base - top) / 2.0 / height;  // radial change per unit height
  for (int i = 0; i < kSides; ++i) {
    const double a0 = 2.0 * std::numbers::pi * i / kSides;
    const double a1 = 2.0 * std::numbers::pi * (i + 1) / kSides;
    const double am = 0.5 * (a0 + a1);
    const double nz = slope / std::sqrt(1.0 + slope * slope);
    const double nr = 1.0 / std::sqrt(1.0 + slope * slope);
    const double nx = nr * std::cos(am), ny = nr * std::sin(am);
    const auto first = static_cast<std::uint32_t>(m.vertex_count());
    add_vertex(m, x + base / 2 * std::cos(a0), y + base / 2 * std::sin(a0), z0, nx, ny, nz, 0, 0);
    add_vertex(m, x + base / 2 * std::cos(a1), y + base / 2 * std::sin(a1), z0, nx, ny, nz, 1, 0);
    add_vertex(m, x + top / 2 * std::cos(a1), y + top / 2 * std::sin(a1), z0 + height, nx, ny, nz, 1, 1);
    add_vertex(m, x + top / 2 * std::cos(a0), y + top / 2 * std::sin(a0), z0 + height, nx, ny, nz, 0, 1);
    m.indices.insert(m.indices.end(), {first, first + 1, first + 2, first, first + 2, first + 3});
  }
  const auto centre = static_cast<std::uint32_t>(m.vertex_count());
  add_vertex(m, x, y, z0 + height, 0, 0, 1, 0.5, 0.5);
  for (int i = 0; i < kSides; ++i) {
    const double a = 2.0 * std::numbers::pi * i / kSides;
    add_vertex(m, x + top / 2 * std::cos(a), y + top / 2 * std::sin(a), z0 + height, 0, 0, 1, 0, 0);
  }
  for (std::uint32_t i = 0; i < kSides; ++i) {
    m.indices.insert(m.indices.end(), {centre, centre + 1 + i, centre + 1 + (i + 1) % kSides});
  }
}

MeshSection section(const char* name, float r, float g, float b, float roughness, std::uint32_t flags = 0) {
  return make_section(name, {r, g, b}, 0.0f, roughness, flags);
}

// Deterministic pseudo-random numbers, the same on every platform (the
// standard distributions are not).
class Random {
 public:
  explicit Random(std::uint64_t seed) : state_(seed != 0 ? seed : 0x9E3779B97F4A7C15ull) {}
  double uniform() {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 7;
    state_ ^= state_ << 17;
    return static_cast<double>(state_ >> 11) * (1.0 / 9007199254740992.0);
  }
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

 private:
  std::uint64_t state_;
};

std::uint64_t fnv1a(const std::string& text) {
  std::uint64_t h = 1469598103934665603ull;
  for (const unsigned char c : text) {
    h = (h ^ c) * 1099511628211ull;
  }
  return h;
}

// Distance from (x, y) to the nearest road edge (negative on the road).
double road_clearance(const Track& track, double x, double y) {
  const CurveProjection p = track.project(x, y);
  const Track::Widths w = track.widths_at(p.s);
  return p.distance - (p.n >= 0.0 ? w.left : w.right);
}

// Square pyramid roof turned by `yaw`.
void pyramid(MeshSection& m, const Vec3& base_centre, double half, double height, double yaw) {
  const double c = std::cos(yaw), sn = std::sin(yaw);
  const Vec3 apex = base_centre + Vec3{0, 0, height};
  Vec3 corners[4];
  const double local[4][2] = {{half, half}, {-half, half}, {-half, -half}, {half, -half}};  // counter-clockwise
  for (int i = 0; i < 4; ++i) {
    corners[i] = base_centre + Vec3{local[i][0] * c - local[i][1] * sn, local[i][0] * sn + local[i][1] * c, 0.0};
  }
  for (int i = 0; i < 4; ++i) {
    const Vec3& a = corners[i];
    const Vec3& b = corners[(i + 1) % 4];
    // (b - a) runs counter-clockwise seen from above: (a, b, apex) faces out.
    const Vec3 face = (b - a).cross(apex - a).normalized();
    const std::uint32_t i0 = m.add_vertex(a, face, 0, 0), i1 = m.add_vertex(b, face, 1, 0), i2 = m.add_vertex(apex, face, 0.5, 1);
    m.add_triangle(i0, i1, i2);
  }
}

void add_tree(MeshSection& trunk, MeshSection& leaves, double x, double y, double height, bool conifer, Random& rng) {
  const Vec3 base{x, y, 0.0};
  const Vec3 up{0, 0, 1};
  const double r = 0.018 * height + 0.05;
  geometry::lathe(trunk, {{0.0, 0.0}, {r, 0.0}, {0.75 * r, (conifer ? 0.35 : 0.55) * height}, {0.0, (conifer ? 0.35 : 0.55) * height}},
                  base, up, 7);
  if (conifer) {
    for (int i = 0; i < 3; ++i) {
      const double z0 = (0.22 + 0.2 * i) * height;
      const double radius = (0.27 - 0.06 * i) * height * rng.uniform(0.85, 1.1);
      geometry::lathe(leaves, {{0.0, z0}, {radius, z0}, {0.0, z0 + 0.42 * height}}, base, up, 9);
    }
  } else {
    const double spread = rng.uniform(0.26, 0.36) * height;
    geometry::ellipsoid(leaves, {x, y, 0.63 * height}, {spread, spread * rng.uniform(0.85, 1.0), 0.32 * height}, 7, 10);
  }
}

// Ground height away from the road: the road surface at the nearest
// centreline sample, its cross slope carried kShoulder beyond the edges and
// level from there. Zero on tracks without relief.
class Terrain {
 public:
  explicit Terrain(const Track& track) : track_(track), relief_(track.has_relief()) {
    if (!relief_) {
      return;
    }
    const auto& samples = track.samples();
    for (std::size_t i = 0; i < samples.size(); ++i) {
      cells_[key(cell(samples[i].x), cell(samples[i].y))].push_back(static_cast<std::uint32_t>(i));
    }
  }

  bool relief() const { return relief_; }

  double height(double x, double y) const {
    if (!relief_) {
      return 0.0;
    }
    const auto& samples = track_.samples();
    const std::int64_t cx = cell(x), cy = cell(y);
    double best = std::numeric_limits<double>::infinity();
    std::size_t nearest = 0;
    for (std::int64_t ring = 0; ring < 4096 && !(best < (static_cast<double>(ring) - 1.0) * kCell); ++ring) {
      for (std::int64_t i = cx - ring; i <= cx + ring; ++i) {
        for (std::int64_t j = cy - ring; j <= cy + ring; ++j) {
          if (std::max(std::abs(i - cx), std::abs(j - cy)) != ring) {
            continue;
          }
          const auto it = cells_.find(key(i, j));
          if (it == cells_.end()) {
            continue;
          }
          for (const std::uint32_t k : it->second) {
            const double d = std::hypot(samples[k].x - x, samples[k].y - y);
            if (d < best) {
              best = d;
              nearest = k;
            }
          }
        }
      }
    }
    const CurveSample& q = samples[nearest];
    const double dx = x - q.x, dy = y - q.y;
    const double along = std::cos(q.heading) * dx + std::sin(q.heading) * dy;
    const double n = -std::sin(q.heading) * dx + std::cos(q.heading) * dy;
    const double s = q.s + std::clamp(along, -Track::kStep, Track::kStep);
    const Track::Widths w = track_.widths_at(s);
    return track_.surface_height(s, std::clamp(n, -w.right - kShoulder, w.left + kShoulder));
  }

  // Lifts the vertices added since `first` onto the ground.
  void lift(MeshSection& m, std::size_t first = 0) const {
    if (!relief_) {
      return;
    }
    for (std::size_t i = first; i < m.vertex_count(); ++i) {
      m.positions[3 * i + 2] += static_cast<float>(height(m.positions[3 * i], m.positions[3 * i + 1]));
    }
  }

 private:
  static constexpr double kCell = 8.0, kShoulder = 4.0;
  static std::int64_t cell(double v) { return static_cast<std::int64_t>(std::floor(v / kCell)); }
  static std::int64_t key(std::int64_t i, std::int64_t j) { return (i << 32) ^ (j & 0xffffffff); }

  const Track& track_;
  bool relief_ = false;
  std::unordered_map<std::int64_t, std::vector<std::uint32_t>> cells_;
};

struct Keepout {
  double x, y, radius;
};

// Grandstand, pit tents and a start gantry near the start line, then trees
// wherever there is room.
void add_scenery(const Track& track, const Terrain& terrain, const std::array<double, 4>& area,
                 std::vector<MeshSection>& out) {
  Random rng(fnv1a(track.name()) ^ 0x5DEECE66Dull);
  MeshSection trunk = section("tree_trunk", 0.33f, 0.24f, 0.16f, 0.95f, kCastShadow);
  MeshSection leaves = section("tree_leaves", 0.20f, 0.40f, 0.16f, 0.9f, kCastShadow);
  MeshSection leaves_dark = section("tree_leaves_dark", 0.12f, 0.29f, 0.13f, 0.9f, kCastShadow);
  MeshSection stand = section("grandstand", 0.62f, 0.63f, 0.66f, 0.8f, kCastShadow);
  MeshSection seats = section("seats", 0.10f, 0.32f, 0.70f, 0.6f, kCastShadow);
  MeshSection tent_roof = section("tent_roof", 0.94f, 0.95f, 0.96f, 0.7f, kCastShadow | kDoubleSided);
  MeshSection tent_frame = section("tent_frame", 0.55f, 0.56f, 0.58f, 0.5f, kCastShadow);
  MeshSection gantry = section("gantry", 0.16f, 0.17f, 0.19f, 0.5f, kCastShadow);
  MeshSection banner = section("banner", 0.84f, 0.15f, 0.24f, 0.55f, kCastShadow);
  gantry.metallic = 0.6f;
  tent_frame.metallic = 0.8f;
  std::vector<Keepout> keepouts;

  const double s0 = track.start_s();
  const CurveSample c = track.at(s0);
  const Track::Widths w = track.widths_at(s0);
  const Vec3 t{std::cos(c.heading), std::sin(c.heading), 0.0};
  const Vec3 n{-t.y, t.x, 0.0};  // left
  const Vec3 origin{c.x, c.y, 0.0};

  // Start gantry across the track.
  {
    const double half = std::max(w.left, w.right) + 1.4;
    for (const double side : {1.0, -1.0}) {
      geometry::box(gantry, origin + n * (side * half), {0.18, 0.18, 2.3}, c.heading);
      geometry::box(gantry, origin + n * (side * half) + Vec3{0, 0, 2.3}, {0.18, 0.18, 2.3}, c.heading);
      keepouts.push_back({origin.x + side * half * n.x, origin.y + side * half * n.y, 3.0});
    }
    geometry::box(gantry, origin + Vec3{0, 0, 4.55}, {0.2, half, 0.2}, c.heading);
    geometry::box(banner, origin + Vec3{0, 0, 4.0} + t * 0.05, {0.08, half - 0.3, 0.42}, c.heading);
  }

  // Grandstand on the side with room, facing the track.
  for (const double side : {1.0, -1.0}) {
    const double offset = (side > 0 ? w.left : w.right) + 12.0;
    const Vec3 front = origin + n * (side * offset) + t * 8.0;
    bool clear = true;
    for (const double along : {-12.0, 0.0, 12.0}) {
      for (const double depth : {0.0, 8.0}) {
        const Vec3 p = front + t * along + n * (side * depth);
        clear = clear && road_clearance(track, p.x, p.y) > 6.0;
      }
    }
    if (!clear) {
      continue;
    }
    for (int k = 0; k < 6; ++k) {
      const double rise = 0.45 * (k + 1);
      const Vec3 centre = front + n * (side * (0.6 + 1.1 * k)) + Vec3{0, 0, 0.5 * rise};
      geometry::box(stand, centre, {12.0, 0.55, 0.5 * rise}, c.heading);
      geometry::box(seats, front + n * (side * (0.45 + 1.1 * k)) + Vec3{0, 0, rise + 0.06}, {11.6, 0.22, 0.06}, c.heading);
    }
    const Vec3 roof = front + n * (side * 3.3) + Vec3{0, 0, 6.2};
    geometry::box(stand, roof, {12.6, 4.2, 0.08}, c.heading);
    for (const double along : {-12.0, 12.0}) {
      for (const double depth : {-0.2, 6.9}) {
        geometry::box(tent_frame, front + t * along + n * (side * depth) + Vec3{0, 0, 3.1}, {0.1, 0.1, 3.1}, c.heading);
      }
    }
    keepouts.push_back({front.x + side * 3.5 * n.x, front.y + side * 3.5 * n.y, 16.0});

    // Pit tents beyond the grandstand.
    for (int i = 0; i < 4; ++i) {
      const Vec3 centre = origin + n * (side * (offset + 1.5)) - t * (12.0 + 4.2 * i);
      if (road_clearance(track, centre.x, centre.y) < 5.0) {
        continue;
      }
      for (const double a : {-1.4, 1.4}) {
        for (const double b : {-1.4, 1.4}) {
          geometry::box(tent_frame, centre + t * a + n * b + Vec3{0, 0, 1.1}, {0.03, 0.03, 1.1}, c.heading);
        }
      }
      pyramid(tent_roof, centre + Vec3{0, 0, 2.2}, 1.55, 0.9, c.heading);
      geometry::box(banner, centre + n * (side * 1.45) + Vec3{0, 0, 1.75}, {1.5, 0.02, 0.35}, c.heading);
      keepouts.push_back({centre.x, centre.y, 3.5});
    }
    break;
  }

  // Trees on a jittered grid, clear of the road and of everything above.
  const double spacing = 9.0;
  for (double gx = area[0] + 0.5 * spacing; gx < area[1]; gx += spacing) {
    for (double gy = area[2] + 0.5 * spacing; gy < area[3]; gy += spacing) {
      const double x = gx + rng.uniform(-0.4, 0.4) * spacing;
      const double y = gy + rng.uniform(-0.4, 0.4) * spacing;
      const double keep = rng.uniform();
      const bool conifer = rng.uniform() < 0.55;
      const double height = rng.uniform(6.0, 13.0);
      const bool dark = rng.uniform() < 0.5;
      if (keep > 0.45) {
        continue;
      }
      bool clear = road_clearance(track, x, y) > 7.0 + 0.3 * height;
      for (const Keepout& k : keepouts) {
        clear = clear && std::hypot(x - k.x, y - k.y) > k.radius;
      }
      if (clear) {
        add_tree(trunk, dark ? leaves_dark : leaves, x, y, height, conifer, rng);
      }
    }
  }

  for (MeshSection* m : {&trunk, &leaves, &leaves_dark, &stand, &seats, &tent_roof, &tent_frame, &gantry, &banner}) {
    if (!m->indices.empty()) {
      terrain.lift(*m);
      out.push_back(std::move(*m));
    }
  }
}

// Red and white kerb blocks (1 m each) over the kerb's profile.
void add_kerbs(const Track& track, MeshSection& red, MeshSection& white) {
  const TrackDefinition& def = track.definition();
  for (const TrackDefinition::Kerb& k : def.kerbs) {
    const bool left = k.side == "left";
    const double s0 = track.point_s(k.from_point);
    double s1 = track.point_s(k.to_point);
    if (track.closed() && s1 <= s0) {
      s1 += track.length();
    }
    constexpr double kStep = 0.25;
    constexpr int kAcross = 6;
    const auto rows = static_cast<std::size_t>(std::ceil((s1 - s0) / kStep)) + 1;
    for (std::size_t r = 0; r + 1 < rows; ++r) {
      const double sa = s0 + kStep * static_cast<double>(r), sb = std::min(s1, sa + kStep);
      MeshSection& m = static_cast<long>(std::floor((sa - s0) / 1.0)) % 2 == 0 ? red : white;
      const auto first = static_cast<std::uint32_t>(m.vertex_count());
      const std::size_t first_index = m.indices.size();
      for (const double s : {sa, sb}) {
        const CurveSample c = track.at(s);
        const Track::Widths w = track.widths_at(s);
        const double nx = -std::sin(c.heading), ny = std::cos(c.heading);
        for (int i = 0; i <= kAcross; ++i) {
          const double e = k.width * i / kAcross;
          const double n = left ? w.left + e : -w.right - e;
          add_vertex(m, c.x + n * nx, c.y + n * ny, track.surface_height(s, n) + 0.002, 0, 0, 1, e, s);
        }
      }
      for (std::uint32_t i = 0; i < kAcross; ++i) {
        const std::uint32_t a = first + i, b = first + i + 1, c = first + kAcross + 1 + i, d = c + 1;
        // Counter-clockwise from above: outward from the edge is +n on the left.
        if (left) {
          m.indices.insert(m.indices.end(), {a, c, b, b, c, d});
        } else {
          m.indices.insert(m.indices.end(), {a, b, c, b, d, c});
        }
      }
      geometry::smooth_normals(m, first, first_index);
    }
  }
}

}  // namespace

TrackMesh make_track_mesh(const Track& track, const TrackMeshOptions& o) {
  MeshSection asphalt = section("asphalt", 0.16f, 0.17f, 0.18f, 0.92f);
  MeshSection lines = section("edge_lines", 0.92f, 0.92f, 0.90f, 0.7f);
  MeshSection start = section("start_line", 0.97f, 0.97f, 0.97f, 0.7f);
  MeshSection sectors = section("sector_lines", 0.98f, 0.82f, 0.18f, 0.7f);
  MeshSection blue = section("cones_blue", 0.10f, 0.32f, 0.85f, 0.45f, kCastShadow);
  MeshSection yellow = section("cones_yellow", 0.98f, 0.82f, 0.10f, 0.45f, kCastShadow);
  MeshSection orange = section("cones_orange", 0.98f, 0.45f, 0.08f, 0.45f, kCastShadow);

  add_strip(asphalt, track, o.step, kRoadZ, [](const Track::Widths& w) { return -w.right; },
            [](const Track::Widths& w) { return w.left; }, kKeepAll);
  const OtherRoad other_road(track);
  const auto off_other_road = [&](double s, double x, double y) { return !other_road.covers(x, y, s); };
  const double lw = o.line_width;
  add_strip(lines, track, o.step, kPaintZ, [&](const Track::Widths& w) { return w.left - lw; },
            [](const Track::Widths& w) { return w.left; }, off_other_road);
  add_strip(lines, track, o.step, kPaintZ, [](const Track::Widths& w) { return -w.right; },
            [&](const Track::Widths& w) { return -w.right + lw; }, off_other_road);
  add_cross_line(start, track, track.start_s(), 0.3);
  if (!track.closed()) {
    add_cross_line(start, track, track.finish_s(), 0.3);
  }
  for (const double s : track.sector_s()) {
    add_cross_line(sectors, track, s, 0.2);
  }

  if (o.cone_spacing > 0.0) {
    const auto count = static_cast<std::size_t>(std::floor(track.length() / o.cone_spacing));
    const auto near_line = [&](double s, double line) {
      const double d = track.closed() ? std::abs(std::remainder(s - line, track.length())) : std::abs(s - line);
      return d < 2.5;
    };
    for (std::size_t k = 0; k <= count; ++k) {
      const double s = static_cast<double>(k) * o.cone_spacing;
      if (track.closed() && k == count && track.length() - s < 0.5 * o.cone_spacing) {
        continue;  // would sit on the first one
      }
      const CurveSample c = track.at(s);
      const Track::Widths w = track.widths_at(s);
      const double nx = -std::sin(c.heading), ny = std::cos(c.heading);
      const bool at_line = near_line(s, track.start_s()) || (!track.closed() && near_line(s, track.finish_s()));
      // Cones stand outside the kerbs.
      for (const double offset : {w.left + track.kerb_width(s, true) + o.cone_offset,
                                  -w.right - track.kerb_width(s, false) - o.cone_offset}) {
        const double x = c.x + offset * nx, y = c.y + offset * ny;
        if (other_road.covers(x, y, s, 0.3)) {
          continue;
        }
        const double z = track.has_relief() ? track.surface_height(s, offset) : 0.0;
        if (at_line) {
          add_cone(orange, x, y, z, kBigConeHeight, kBigConeBase, kConeTop);
        } else {
          add_cone(offset > 0.0 ? blue : yellow, x, y, z, kConeHeight, kConeBase, kConeTop);
        }
      }
    }
  }

  // Bounding box of the scenery so far, then a ground plane around it.
  TrackMesh mesh;
  double lo[3] = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), 0.0};
  double hi[3] = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), 0.0};
  for (const MeshSection* m : {&asphalt, &blue, &yellow, &orange}) {
    for (std::size_t i = 0; i < m->positions.size(); i += 3) {
      for (int a = 0; a < 3; ++a) {
        lo[a] = std::min(lo[a], static_cast<double>(m->positions[i + a]));
        hi[a] = std::max(hi[a], static_cast<double>(m->positions[i + a]));
      }
    }
  }
  MeshSection ground = section("ground", 0.27f, 0.42f, 0.20f, 1.0f);
  const double gx0 = lo[0] - o.ground_margin, gx1 = hi[0] + o.ground_margin;
  const double gy0 = lo[1] - o.ground_margin, gy1 = hi[1] + o.ground_margin;
  const Terrain terrain(track);
  if (!terrain.relief()) {
    add_vertex(ground, gx0, gy0, kGroundZ, 0, 0, 1, gx0 / 10, gy0 / 10);
    add_vertex(ground, gx1, gy0, kGroundZ, 0, 0, 1, gx1 / 10, gy0 / 10);
    add_vertex(ground, gx1, gy1, kGroundZ, 0, 0, 1, gx1 / 10, gy1 / 10);
    add_vertex(ground, gx0, gy1, kGroundZ, 0, 0, 1, gx0 / 10, gy1 / 10);
    ground.indices = {0, 1, 2, 0, 2, 3};
  } else {
    // A height grid following the road, a little below it.
    const double spacing = std::max(3.0, std::max(gx1 - gx0, gy1 - gy0) / 220.0);
    const auto nx = static_cast<std::uint32_t>(std::ceil((gx1 - gx0) / spacing));
    const auto ny = static_cast<std::uint32_t>(std::ceil((gy1 - gy0) / spacing));
    for (std::uint32_t j = 0; j <= ny; ++j) {
      for (std::uint32_t i = 0; i <= nx; ++i) {
        const double x = gx0 + (gx1 - gx0) * i / nx, y = gy0 + (gy1 - gy0) * j / ny;
        add_vertex(ground, x, y, terrain.height(x, y) + 2.0 * kGroundZ, 0, 0, 1, x / 10, y / 10);
      }
    }
    for (std::uint32_t j = 0; j < ny; ++j) {
      for (std::uint32_t i = 0; i < nx; ++i) {
        const std::uint32_t a = j * (nx + 1) + i, b = a + 1, c = a + nx + 1, d = c + 1;
        ground.indices.insert(ground.indices.end(), {a, b, d, a, d, c});
      }
    }
    geometry::smooth_normals(ground, 0, 0);
  }
  MeshSection kerb_red = section("kerb_red", 0.78f, 0.08f, 0.08f, 0.6f);
  MeshSection kerb_white = section("kerb_white", 0.93f, 0.93f, 0.92f, 0.6f);
  add_kerbs(track, kerb_red, kerb_white);

  for (MeshSection* m : {&ground, &asphalt, &lines, &start, &sectors, &kerb_red, &kerb_white, &blue, &yellow, &orange}) {
    if (!m->indices.empty()) {
      mesh.sections.push_back(std::move(*m));
    }
  }
  if (o.scenery) {
    add_scenery(track, terrain, {gx0, gx1, gy0, gy1}, mesh.sections);
  }
  double top = std::max(hi[2], static_cast<double>(kPaintZ));
  double bottom = kGroundZ;
  for (const MeshSection& m : mesh.sections) {
    for (std::size_t i = 2; i < m.positions.size(); i += 3) {
      top = std::max(top, static_cast<double>(m.positions[i]));
      bottom = std::min(bottom, static_cast<double>(m.positions[i]));
    }
  }
  mesh.min = {gx0, gy0, bottom};
  mesh.max = {gx1, gy1, top};
  return mesh;
}

std::string to_glb(const TrackMesh& mesh) {
  return write_glb({GltfNode{"track", &mesh.sections, {}, {}}}, "vsim track mesh");
}

}  // namespace vsim
