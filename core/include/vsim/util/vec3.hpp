#pragma once

#include <cmath>

namespace vsim {

// Small 3D vector (metres, or any unit).
struct Vec3 {
  double x = 0.0, y = 0.0, z = 0.0;

  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double norm() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const {
    const double n = norm();
    return n > 0.0 ? *this * (1.0 / n) : Vec3{0.0, 0.0, 1.0};
  }
};

// Rotates p about the axis through `origin` along unit vector `axis` by
// `angle` (right hand).
inline Vec3 rotate_about(const Vec3& p, const Vec3& origin, const Vec3& axis, double angle) {
  const Vec3 v = p - origin;
  const double c = std::cos(angle), s = std::sin(angle);
  return origin + v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c));
}

}  // namespace vsim
