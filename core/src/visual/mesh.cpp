#include "vsim/visual/mesh.hpp"

#include <algorithm>
#include <numbers>

namespace vsim {

std::uint32_t MeshSection::add_vertex(const Vec3& p, const Vec3& n, double u, double v) {
  const auto index = static_cast<std::uint32_t>(vertex_count());
  positions.insert(positions.end(), {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
  normals.insert(normals.end(), {static_cast<float>(n.x), static_cast<float>(n.y), static_cast<float>(n.z)});
  uvs.insert(uvs.end(), {static_cast<float>(u), static_cast<float>(v)});
  return index;
}

MeshSection make_section(std::string name, std::array<float, 3> srgb, float metallic, float roughness,
                         std::uint32_t flags) {
  MeshSection m;
  m.name = std::move(name);
  m.color = {srgb[0], srgb[1], srgb[2], 1.0f};
  m.metallic = metallic;
  m.roughness = roughness;
  m.flags = flags;
  return m;
}

namespace geometry {
namespace {

constexpr double kPi = std::numbers::pi;

// Any unit vector perpendicular to d.
Vec3 perpendicular(const Vec3& d) {
  const Vec3 helper = std::abs(d.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
  return d.cross(helper).normalized();
}

// Fan cap over a ring (triangles face along `facing`).
void cap(MeshSection& m, const std::vector<Vec3>& ring, const Vec3& facing) {
  Vec3 centre;
  for (const Vec3& p : ring) {
    centre = centre + p;
  }
  centre = centre * (1.0 / static_cast<double>(ring.size()));
  const Vec3 n = facing.normalized();
  const std::uint32_t c = m.add_vertex(centre, n, 0.5, 0.5);
  const auto first = static_cast<std::uint32_t>(m.vertex_count());
  for (const Vec3& p : ring) {
    m.add_vertex(p, n, 0, 0);
  }
  const auto count = static_cast<std::uint32_t>(ring.size());
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint32_t a = first + i, b = first + (i + 1) % count;
    // Wind so the triangle faces `facing`.
    const Vec3 pa = ring[i], pb = ring[(i + 1) % count];
    if ((pa - centre).cross(pb - centre).dot(n) >= 0.0) {
      m.add_triangle(c, a, b);
    } else {
      m.add_triangle(c, b, a);
    }
  }
}

}  // namespace

void smooth_normals(MeshSection& m, std::size_t first_vertex, std::size_t first_index) {
  const std::size_t n = m.vertex_count();
  std::vector<Vec3> acc(n - first_vertex);
  for (std::size_t t = first_index; t + 2 < m.indices.size(); t += 3) {
    const std::uint32_t a = m.indices[t], b = m.indices[t + 1], c = m.indices[t + 2];
    const Vec3 face = (m.position(b) - m.position(a)).cross(m.position(c) - m.position(a));  // area weighted
    for (const std::uint32_t v : {a, b, c}) {
      if (v >= first_vertex) {
        acc[v - first_vertex] = acc[v - first_vertex] + face;
      }
    }
  }
  for (std::size_t i = first_vertex; i < n; ++i) {
    const Vec3 normal = acc[i - first_vertex].normalized();
    m.normals[3 * i] = static_cast<float>(normal.x);
    m.normals[3 * i + 1] = static_cast<float>(normal.y);
    m.normals[3 * i + 2] = static_cast<float>(normal.z);
  }
}

void loft(MeshSection& m, const std::vector<std::vector<Vec3>>& rings, bool cap_start, bool cap_end) {
  if (rings.size() < 2) {
    return;
  }
  const std::size_t first_vertex = m.vertex_count();
  const std::size_t first_index = m.indices.size();
  const std::size_t count = rings.front().size();
  for (std::size_t r = 0; r < rings.size(); ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      m.add_vertex(rings[r][i], {0, 0, 1}, static_cast<double>(i) / static_cast<double>(count),
                   static_cast<double>(r) / static_cast<double>(rings.size() - 1));
    }
  }
  for (std::size_t r = 0; r + 1 < rings.size(); ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      const auto a = static_cast<std::uint32_t>(first_vertex + r * count + i);
      const auto b = static_cast<std::uint32_t>(first_vertex + r * count + (i + 1) % count);
      const auto c = static_cast<std::uint32_t>(a + count);
      const auto d = static_cast<std::uint32_t>(b + count);
      // The ring runs counter-clockwise about the direction it advances in,
      // so (b - a) x (c - a) points outwards.
      m.add_triangle(a, b, d);
      m.add_triangle(a, d, c);
    }
  }
  smooth_normals(m, first_vertex, first_index);
  const Vec3 advance = rings.back().front() - rings.front().front();
  const auto centre_of = [](const std::vector<Vec3>& ring) {
    Vec3 c;
    for (const Vec3& p : ring) {
      c = c + p;
    }
    return c * (1.0 / static_cast<double>(ring.size()));
  };
  const Vec3 along = centre_of(rings.back()) - centre_of(rings.front());
  const Vec3 dir = along.norm() > 1e-9 ? along : advance;
  if (cap_start) {
    cap(m, rings.front(), -dir);
  }
  if (cap_end) {
    cap(m, rings.back(), dir);
  }
}

std::vector<Vec3> superellipse_ring(double x, double y0, double z0, double a, double b, double exponent, int points) {
  std::vector<Vec3> ring;
  ring.reserve(static_cast<std::size_t>(points));
  for (int i = 0; i < points; ++i) {
    // From the bottom (-z) through +y: counter-clockwise about +x.
    const double t = 2.0 * kPi * i / points - kPi / 2.0;
    const double c = std::cos(t), s = std::sin(t);
    const double e = 2.0 / exponent;
    const double cy = (c >= 0 ? 1.0 : -1.0) * std::pow(std::abs(c), e);
    const double sz = (s >= 0 ? 1.0 : -1.0) * std::pow(std::abs(s), e);
    ring.push_back({x, y0 + a * cy, z0 + b * sz});
  }
  return ring;
}

void lathe(MeshSection& m, const std::vector<std::array<double, 2>>& profile, const Vec3& centre, const Vec3& axis,
           int segments) {
  const Vec3 w = axis.normalized();
  const Vec3 u = perpendicular(w);
  const Vec3 v = w.cross(u);
  std::vector<std::vector<Vec3>> rings;
  for (const auto& [radius, along] : profile) {
    std::vector<Vec3> ring;
    for (int i = 0; i < segments; ++i) {
      // Counter-clockwise about +axis.
      const double t = 2.0 * kPi * i / segments;
      ring.push_back(centre + w * along + (u * std::cos(t) + v * std::sin(t)) * radius);
    }
    rings.push_back(std::move(ring));
  }
  const std::size_t first_vertex = m.vertex_count();
  const std::size_t first_index = m.indices.size();
  const std::size_t count = static_cast<std::size_t>(segments);
  for (std::size_t r = 0; r < rings.size(); ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      m.add_vertex(rings[r][i], {0, 0, 1}, static_cast<double>(i) / static_cast<double>(count),
                   static_cast<double>(r) / static_cast<double>(rings.size() - 1));
    }
  }
  for (std::size_t r = 0; r + 1 < rings.size(); ++r) {
    for (std::size_t i = 0; i < count; ++i) {
      const auto a = static_cast<std::uint32_t>(first_vertex + r * count + i);
      const auto b = static_cast<std::uint32_t>(first_vertex + r * count + (i + 1) % count);
      const auto c = static_cast<std::uint32_t>(a + count);
      const auto d = static_cast<std::uint32_t>(b + count);
      m.add_triangle(a, b, d);
      m.add_triangle(a, d, c);
    }
  }
  smooth_normals(m, first_vertex, first_index);
}

void mirror_y(MeshSection& m, std::size_t first_vertex, std::size_t first_index) {
  for (std::size_t i = first_vertex; i < m.vertex_count(); ++i) {
    m.positions[3 * i + 1] = -m.positions[3 * i + 1];
    m.normals[3 * i + 1] = -m.normals[3 * i + 1];
  }
  for (std::size_t t = first_index; t + 2 < m.indices.size(); t += 3) {
    std::swap(m.indices[t + 1], m.indices[t + 2]);  // a mirror flips the winding
  }
}

void tube(MeshSection& m, const std::vector<Vec3>& path, double radius, int sides, bool caps) {
  if (path.size() < 2) {
    return;
  }
  std::vector<std::vector<Vec3>> rings;
  Vec3 normal = perpendicular((path[1] - path[0]).normalized());
  for (std::size_t k = 0; k < path.size(); ++k) {
    Vec3 tangent;
    if (k == 0) {
      tangent = path[1] - path[0];
    } else if (k + 1 == path.size()) {
      tangent = path[k] - path[k - 1];
    } else {
      tangent = (path[k + 1] - path[k]).normalized() + (path[k] - path[k - 1]).normalized();
    }
    tangent = tangent.normalized();
    // Parallel transport: remove the tangential part of the previous normal.
    normal = (normal - tangent * normal.dot(tangent)).normalized();
    const Vec3 binormal = tangent.cross(normal);
    std::vector<Vec3> ring;
    for (int i = 0; i < sides; ++i) {
      const double t = 2.0 * kPi * i / sides;
      ring.push_back(path[k] + (normal * std::cos(t) + binormal * std::sin(t)) * radius);
    }
    rings.push_back(std::move(ring));
  }
  loft(m, rings, caps, caps);
}

void box(MeshSection& m, const Vec3& c, const Vec3& h, double yaw, double pitch) {
  const double cy = std::cos(yaw), sy = std::sin(yaw), cp = std::cos(pitch), sp = std::sin(pitch);
  const auto rotate = [&](const Vec3& p) {
    // pitch about y (nose down positive), then yaw about z
    const Vec3 q{p.x * cp + p.z * sp, p.y, -p.x * sp + p.z * cp};
    return Vec3{q.x * cy - q.y * sy, q.x * sy + q.y * cy, q.z};
  };
  const Vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  const double half[3] = {h.x, h.y, h.z};
  for (int a = 0; a < 3; ++a) {
    for (const double side : {1.0, -1.0}) {
      const Vec3 n = axes[a] * side;
      const Vec3 u = axes[(a + 1) % 3], v = axes[(a + 2) % 3];
      const double hu = half[(a + 1) % 3], hv = half[(a + 2) % 3];
      const Vec3 base = n * half[a];
      Vec3 corners[4] = {base - u * hu - v * hv, base + u * hu - v * hv, base + u * hu + v * hv, base - u * hu + v * hv};
      const Vec3 rn = rotate(n);
      std::uint32_t idx[4];
      for (int k = 0; k < 4; ++k) {
        idx[k] = m.add_vertex(c + rotate(corners[k]), rn, k == 1 || k == 2, k >= 2);
      }
      // u x v = n for the + side: (0,1,2) is counter-clockwise seen from outside.
      if (side > 0) {
        m.add_triangle(idx[0], idx[1], idx[2]);
        m.add_triangle(idx[0], idx[2], idx[3]);
      } else {
        m.add_triangle(idx[0], idx[2], idx[1]);
        m.add_triangle(idx[0], idx[3], idx[2]);
      }
    }
  }
}

void ellipsoid(MeshSection& m, const Vec3& c, const Vec3& r, int rings, int segments) {
  const std::size_t first = m.vertex_count();
  for (int i = 0; i <= rings; ++i) {
    const double phi = kPi * i / rings - kPi / 2.0;  // bottom to top
    for (int j = 0; j <= segments; ++j) {
      const double theta = 2.0 * kPi * j / segments;
      const Vec3 unit{std::cos(phi) * std::cos(theta), std::cos(phi) * std::sin(theta), std::sin(phi)};
      const Vec3 p{c.x + r.x * unit.x, c.y + r.y * unit.y, c.z + r.z * unit.z};
      const Vec3 n = Vec3{unit.x / r.x, unit.y / r.y, unit.z / r.z}.normalized();
      m.add_vertex(p, n, static_cast<double>(j) / segments, static_cast<double>(i) / rings);
    }
  }
  const auto stride = static_cast<std::uint32_t>(segments + 1);
  for (int i = 0; i < rings; ++i) {
    for (int j = 0; j < segments; ++j) {
      const auto a = static_cast<std::uint32_t>(first + static_cast<std::size_t>(i) * stride + j);
      const auto b = a + 1, cc = a + stride, d = cc + 1;
      m.add_triangle(a, b, d);
      m.add_triangle(a, d, cc);
    }
  }
}

void extrude_y(MeshSection& m, const std::vector<std::array<double, 2>>& profile, double y0, double y1) {
  // Rings in planes y = y0 and y = y1, advancing towards +y; a profile that
  // is counter-clockwise in (x, z) seen from... -y looks at (x right, z up),
  // so a counter-clockwise (x, z) loop is clockwise seen from +y. Reverse it.
  std::vector<Vec3> a, b;
  for (auto it = profile.rbegin(); it != profile.rend(); ++it) {
    a.push_back({(*it)[0], y0, (*it)[1]});
    b.push_back({(*it)[0], y1, (*it)[1]});
  }
  if (y1 < y0) {
    std::reverse(a.begin(), a.end());
    std::reverse(b.begin(), b.end());
  }
  // Sharp edges (trailing edge) would smear with smooth normals; acceptable
  // for thin aerofoils.
  loft(m, {a, b}, true, true);
}

void transform(MeshSection& m, std::size_t first, const std::function<Vec3(const Vec3&)>& position,
               const std::function<Vec3(const Vec3&)>& normal) {
  for (std::size_t i = first; i < m.vertex_count(); ++i) {
    const Vec3 p = position(m.position(i));
    const Vec3 n = normal(Vec3{m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]}).normalized();
    m.positions[3 * i] = static_cast<float>(p.x);
    m.positions[3 * i + 1] = static_cast<float>(p.y);
    m.positions[3 * i + 2] = static_cast<float>(p.z);
    m.normals[3 * i] = static_cast<float>(n.x);
    m.normals[3 * i + 1] = static_cast<float>(n.y);
    m.normals[3 * i + 2] = static_cast<float>(n.z);
  }
}

}  // namespace geometry
}  // namespace vsim
