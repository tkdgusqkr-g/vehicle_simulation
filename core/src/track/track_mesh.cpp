#include "vsim/track/track_mesh.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>

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
  std::vector<bool> kept(rows);
  for (std::size_t k = 0; k < rows; ++k) {
    const double s = s0 + (s1 - s0) * static_cast<double>(k) / static_cast<double>(count);
    const CurveSample c = track.at(s);
    const Track::Widths w = track.widths_at(s);
    const double nx = -std::sin(c.heading), ny = std::cos(c.heading);
    const double r = right_of(w), l = left_of(w);
    add_vertex(m, c.x + r * nx, c.y + r * ny, z, 0, 0, 1, 0, s / 4.0);
    add_vertex(m, c.x + l * nx, c.y + l * ny, z, 0, 0, 1, 1, s / 4.0);
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
void add_cone(MeshSection& m, double x, double y, double height, double base, double top) {
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
    add_vertex(m, x + base / 2 * std::cos(a0), y + base / 2 * std::sin(a0), 0, nx, ny, nz, 0, 0);
    add_vertex(m, x + base / 2 * std::cos(a1), y + base / 2 * std::sin(a1), 0, nx, ny, nz, 1, 0);
    add_vertex(m, x + top / 2 * std::cos(a1), y + top / 2 * std::sin(a1), height, nx, ny, nz, 1, 1);
    add_vertex(m, x + top / 2 * std::cos(a0), y + top / 2 * std::sin(a0), height, nx, ny, nz, 0, 1);
    m.indices.insert(m.indices.end(), {first, first + 1, first + 2, first, first + 2, first + 3});
  }
  const auto centre = static_cast<std::uint32_t>(m.vertex_count());
  add_vertex(m, x, y, height, 0, 0, 1, 0.5, 0.5);
  for (int i = 0; i < kSides; ++i) {
    const double a = 2.0 * std::numbers::pi * i / kSides;
    add_vertex(m, x + top / 2 * std::cos(a), y + top / 2 * std::sin(a), height, 0, 0, 1, 0, 0);
  }
  for (std::uint32_t i = 0; i < kSides; ++i) {
    m.indices.insert(m.indices.end(), {centre, centre + 1 + i, centre + 1 + (i + 1) % kSides});
  }
}

MeshSection section(const char* name, float r, float g, float b) {
  MeshSection m;
  m.name = name;
  m.color = {r, g, b, 1.0f};
  return m;
}

}  // namespace

TrackMesh make_track_mesh(const Track& track, const TrackMeshOptions& o) {
  MeshSection asphalt = section("asphalt", 0.16f, 0.17f, 0.18f);
  MeshSection lines = section("edge_lines", 0.92f, 0.92f, 0.90f);
  MeshSection start = section("start_line", 0.97f, 0.97f, 0.97f);
  MeshSection sectors = section("sector_lines", 0.98f, 0.82f, 0.18f);
  MeshSection blue = section("cones_blue", 0.10f, 0.32f, 0.85f);
  MeshSection yellow = section("cones_yellow", 0.98f, 0.82f, 0.10f);
  MeshSection orange = section("cones_orange", 0.98f, 0.45f, 0.08f);

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
      for (const double offset : {w.left + o.cone_offset, -w.right - o.cone_offset}) {
        const double x = c.x + offset * nx, y = c.y + offset * ny;
        if (other_road.covers(x, y, s, 0.3)) {
          continue;
        }
        if (at_line) {
          add_cone(orange, x, y, kBigConeHeight, kBigConeBase, kConeTop);
        } else {
          add_cone(offset > 0.0 ? blue : yellow, x, y, kConeHeight, kConeBase, kConeTop);
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
  MeshSection ground = section("ground", 0.27f, 0.42f, 0.20f);
  const double gx0 = lo[0] - o.ground_margin, gx1 = hi[0] + o.ground_margin;
  const double gy0 = lo[1] - o.ground_margin, gy1 = hi[1] + o.ground_margin;
  add_vertex(ground, gx0, gy0, kGroundZ, 0, 0, 1, gx0 / 10, gy0 / 10);
  add_vertex(ground, gx1, gy0, kGroundZ, 0, 0, 1, gx1 / 10, gy0 / 10);
  add_vertex(ground, gx1, gy1, kGroundZ, 0, 0, 1, gx1 / 10, gy1 / 10);
  add_vertex(ground, gx0, gy1, kGroundZ, 0, 0, 1, gx0 / 10, gy1 / 10);
  ground.indices = {0, 1, 2, 0, 2, 3};

  for (MeshSection* m : {&ground, &asphalt, &lines, &start, &sectors, &blue, &yellow, &orange}) {
    if (!m->indices.empty()) {
      mesh.sections.push_back(std::move(*m));
    }
  }
  mesh.min = {gx0, gy0, kGroundZ};
  mesh.max = {gx1, gy1, std::max(hi[2], static_cast<double>(kPaintZ))};
  return mesh;
}

std::string to_glb(const TrackMesh& mesh) {
  using nlohmann::json;
  std::string bin;
  const auto append = [&](const void* data, std::size_t bytes) {
    const std::size_t offset = bin.size();
    bin.append(static_cast<const char*>(data), bytes);
    while (bin.size() % 4 != 0) {
      bin.push_back('\0');
    }
    return offset;
  };
  json doc;
  doc["asset"] = {{"version", "2.0"}, {"generator", "vsim track mesh"}};
  json buffer_views = json::array(), accessors = json::array(), materials = json::array(), primitives = json::array();
  const auto view = [&](std::size_t offset, std::size_t bytes, int target) {
    buffer_views.push_back({{"buffer", 0}, {"byteOffset", offset}, {"byteLength", bytes}, {"target", target}});
    return buffer_views.size() - 1;
  };
  constexpr int kArrayBuffer = 34962, kElementBuffer = 34963, kFloat = 5126, kUint = 5125;
  for (const MeshSection& m : mesh.sections) {
    const std::size_t n = m.vertex_count();
    // vsim (x east, y north, z up) -> glTF (x, y up, z towards the viewer): (x, z, -y).
    std::vector<float> pos(3 * n), nor(3 * n);
    float pmin[3] = {std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    float pmax[3] = {-pmin[0], -pmin[0], -pmin[0]};
    for (std::size_t i = 0; i < n; ++i) {
      const float p[3] = {m.positions[3 * i], m.positions[3 * i + 2], -m.positions[3 * i + 1]};
      const float q[3] = {m.normals[3 * i], m.normals[3 * i + 2], -m.normals[3 * i + 1]};
      for (int a = 0; a < 3; ++a) {
        pos[3 * i + a] = p[a];
        nor[3 * i + a] = q[a];
        pmin[a] = std::min(pmin[a], p[a]);
        pmax[a] = std::max(pmax[a], p[a]);
      }
    }
    const auto pos_view = view(append(pos.data(), pos.size() * 4), pos.size() * 4, kArrayBuffer);
    const auto nor_view = view(append(nor.data(), nor.size() * 4), nor.size() * 4, kArrayBuffer);
    const auto uv_view = view(append(m.uvs.data(), m.uvs.size() * 4), m.uvs.size() * 4, kArrayBuffer);
    const auto idx_view = view(append(m.indices.data(), m.indices.size() * 4), m.indices.size() * 4, kElementBuffer);
    accessors.push_back({{"bufferView", pos_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC3"},
                         {"min", {pmin[0], pmin[1], pmin[2]}}, {"max", {pmax[0], pmax[1], pmax[2]}}});
    accessors.push_back({{"bufferView", nor_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC3"}});
    accessors.push_back({{"bufferView", uv_view}, {"componentType", kFloat}, {"count", n}, {"type", "VEC2"}});
    accessors.push_back({{"bufferView", idx_view}, {"componentType", kUint}, {"count", m.indices.size()},
                         {"type", "SCALAR"}});
    const std::size_t a = accessors.size() - 4;
    // glTF colour factors are linear; the section colours are sRGB.
    const auto linear = [](float c) {
      return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f);
    };
    materials.push_back({{"name", m.name},
                         {"pbrMetallicRoughness",
                          {{"baseColorFactor", {linear(m.color[0]), linear(m.color[1]), linear(m.color[2]), m.color[3]}},
                           {"metallicFactor", 0.0},
                           {"roughnessFactor", 0.85}}}});
    primitives.push_back({{"attributes", {{"POSITION", a}, {"NORMAL", a + 1}, {"TEXCOORD_0", a + 2}}},
                          {"indices", a + 3},
                          {"material", materials.size() - 1}});
  }
  doc["buffers"] = {{{"byteLength", bin.size()}}};
  doc["bufferViews"] = buffer_views;
  doc["accessors"] = accessors;
  doc["materials"] = materials;
  doc["meshes"] = {{{"name", "track"}, {"primitives", primitives}}};
  doc["nodes"] = {{{"name", "track"}, {"mesh", 0}}};
  doc["scenes"] = {{{"nodes", {0}}}};
  doc["scene"] = 0;

  std::string text = doc.dump();
  while (text.size() % 4 != 0) {
    text.push_back(' ');
  }
  const auto u32 = [](std::string& out, std::uint32_t v) {
    char bytes[4];
    std::memcpy(bytes, &v, 4);  // glTF is little-endian, as are the targets
    out.append(bytes, 4);
  };
  std::string glb;
  u32(glb, 0x46546C67);  // "glTF"
  u32(glb, 2);
  u32(glb, static_cast<std::uint32_t>(12 + 8 + text.size() + 8 + bin.size()));
  u32(glb, static_cast<std::uint32_t>(text.size()));
  u32(glb, 0x4E4F534A);  // "JSON"
  glb += text;
  u32(glb, static_cast<std::uint32_t>(bin.size()));
  u32(glb, 0x004E4942);  // "BIN\0"
  glb += bin;
  return glb;
}

}  // namespace vsim
