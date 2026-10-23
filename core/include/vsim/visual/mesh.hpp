#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "vsim/util/vec3.hpp"

namespace vsim {

// Section flags (renderer hints).
enum MeshFlags : std::uint32_t {
  kCastShadow = 1u,
  kDoubleSided = 2u,
};

// One part of a mesh with one material. Coordinates are in the vsim frame
// (x forward/east, y left/north, z up, metres); triangles wind
// counter-clockwise seen from the front.
struct MeshSection {
  std::string name;                                    // also names the material
  std::array<float, 4> color{1.0f, 1.0f, 1.0f, 1.0f};  // base colour, sRGB 0..1
  float metallic = 0.0f;
  float roughness = 0.85f;
  std::uint32_t flags = 0;
  std::vector<float> positions;        // x, y, z per vertex
  std::vector<float> normals;          // x, y, z per vertex
  std::vector<float> uvs;              // u, v per vertex
  std::vector<std::uint32_t> indices;  // three per triangle

  std::size_t vertex_count() const { return positions.size() / 3; }
  std::uint32_t add_vertex(const Vec3& p, const Vec3& n, double u = 0.0, double v = 0.0);
  void add_triangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) { indices.insert(indices.end(), {a, b, c}); }
  Vec3 position(std::size_t i) const { return {positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]}; }
};

MeshSection make_section(std::string name, std::array<float, 3> srgb, float metallic, float roughness,
                         std::uint32_t flags = kCastShadow);

// Geometry builders. Smooth surfaces share vertices and get averaged normals.
namespace geometry {

// Rings of points (all the same count) joined into a surface, ring after
// ring. Each ring runs counter-clockwise seen from the end the rings advance
// towards, so the surface faces outwards. Optional flat caps.
void loft(MeshSection& m, const std::vector<std::vector<Vec3>>& rings, bool cap_start, bool cap_end);

// Closed ring of a superellipse in the y-z plane at x (exponent 2: ellipse,
// larger: boxier), centre (y0, z0), half-width a, half-height b; counter-
// clockwise seen from +x.
std::vector<Vec3> superellipse_ring(double x, double y0, double z0, double a, double b, double exponent, int points);

// Surface of revolution of a profile (radius, axial position) around an
// axis through `centre` along `axis`. Drawn with the axis to the right and
// the radius up, the surface faces to the left of the profile's direction:
// running along +axis at some radius faces outwards, running inwards at the
// +axis end faces +axis.
void lathe(MeshSection& m, const std::vector<std::array<double, 2>>& profile, const Vec3& centre, const Vec3& axis,
           int segments);

// Tube of `radius` along a polyline (parallel transport frames), with caps.
void tube(MeshSection& m, const std::vector<Vec3>& path, double radius, int sides, bool caps = true);

// Flat-shaded box with half sizes, rotated by yaw about z and pitch about y.
void box(MeshSection& m, const Vec3& centre, const Vec3& half, double yaw = 0.0, double pitch = 0.0);

// Ellipsoid with semi-axes (smooth).
void ellipsoid(MeshSection& m, const Vec3& centre, const Vec3& radii, int rings, int segments);

// A closed 2D profile (x, z), counter-clockwise, extruded along y from y0 to
// y1 (wings), with flat end caps.
void extrude_y(MeshSection& m, const std::vector<std::array<double, 2>>& profile, double y0, double y1);

// Mirrors the vertices added since `first_vertex` in y (left <-> right) and
// keeps the triangles facing out.
void mirror_y(MeshSection& m, std::size_t first_vertex, std::size_t first_index);

// Applies p -> f(p) and n -> g(n) to the vertices added since `first`.
void transform(MeshSection& m, std::size_t first, const std::function<Vec3(const Vec3&)>& position,
               const std::function<Vec3(const Vec3&)>& normal);

// Recomputes smooth normals of the triangles added since index `first_index`
// (vertices shared between triangles get the area-weighted average).
void smooth_normals(MeshSection& m, std::size_t first_vertex, std::size_t first_index);

}  // namespace geometry
}  // namespace vsim
