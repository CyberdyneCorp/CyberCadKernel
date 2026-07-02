// SPDX-License-Identifier: Apache-2.0
//
// vec.h — 3D vector / point / direction primitives for the native math kernel.
//
// Clean-room implementation from first principles. Conventions match OCCT's
// gp_* family (see /Users/leonardoaraujo/work/OCCT/src/FoundationClasses/TKMath/gp)
// so numeric parity holds against the OCCT oracle:
//   * Right-handed coordinate system.
//   * Column vectors; a transform applies as v' = M*v + t (see transform.h).
//   * fp64 throughout for exact modelling / determinism.
//
// OCCT-FREE: this file has no OCCT include and compiles with
//   clang++ -std=c++20
//
#ifndef CYBERCAD_NATIVE_MATH_VEC_H
#define CYBERCAD_NATIVE_MATH_VEC_H

#include <array>
#include <cmath>
#include <cstddef>

namespace cybercad::native::math {

/// Default linear tolerance used for normalization / degeneracy checks.
/// Mirrors gp::Resolution()-scale reasoning: a vector shorter than this is
/// treated as null. Callers needing a different tolerance pass it explicitly.
inline constexpr double kLinearTolerance = 1e-12;

// ─────────────────────────────────────────────────────────────────────────────
// Vec3 — a free 3D vector (also the storage for Point3 / Dir3 coordinates).
// ─────────────────────────────────────────────────────────────────────────────
struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Vec3() noexcept = default;
  constexpr Vec3(double px, double py, double pz) noexcept : x(px), y(py), z(pz) {}

  // Element access (0..2). No bounds check in release: hot path.
  constexpr double operator[](std::size_t i) const noexcept {
    return (i == 0) ? x : (i == 1) ? y : z;
  }

  constexpr Vec3 operator+(const Vec3& o) const noexcept { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3& o) const noexcept { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator-() const noexcept { return {-x, -y, -z}; }
  constexpr Vec3 operator*(double s) const noexcept { return {x * s, y * s, z * s}; }
  constexpr Vec3 operator/(double s) const noexcept { return {x / s, y / s, z / s}; }

  constexpr Vec3& operator+=(const Vec3& o) noexcept { x += o.x; y += o.y; z += o.z; return *this; }
  constexpr Vec3& operator-=(const Vec3& o) noexcept { x -= o.x; y -= o.y; z -= o.z; return *this; }
  constexpr Vec3& operator*=(double s) noexcept { x *= s; y *= s; z *= s; return *this; }

  constexpr bool operator==(const Vec3& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
};

constexpr Vec3 operator*(double s, const Vec3& v) noexcept { return v * s; }

/// Dot (scalar) product.
constexpr double dot(const Vec3& a, const Vec3& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Cross (vector) product. Right-handed: cross(X,Y)=Z.
constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
  return {a.y * b.z - a.z * b.y,
          a.z * b.x - a.x * b.z,
          a.x * b.y - a.y * b.x};
}

/// Squared magnitude (constexpr, no sqrt).
constexpr double normSquared(const Vec3& v) noexcept { return dot(v, v); }

/// Magnitude. Not constexpr in C++20 (std::sqrt), so runtime only.
inline double norm(const Vec3& v) noexcept { return std::sqrt(normSquared(v)); }

/// Distance between two coordinate triples.
inline double distance(const Vec3& a, const Vec3& b) noexcept { return norm(b - a); }

/// Linear interpolation a + t*(b-a).
constexpr Vec3 lerp(const Vec3& a, const Vec3& b, double t) noexcept {
  return a + (b - a) * t;
}

// ─────────────────────────────────────────────────────────────────────────────
// Point3 — a position. Distinct type from Vec3 to keep affine semantics honest
// (point ± vector, point − point → vector). Shares Vec3 storage.
// ─────────────────────────────────────────────────────────────────────────────
struct Point3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  constexpr Point3() noexcept = default;
  constexpr Point3(double px, double py, double pz) noexcept : x(px), y(py), z(pz) {}
  constexpr explicit Point3(const Vec3& v) noexcept : x(v.x), y(v.y), z(v.z) {}

  constexpr double operator[](std::size_t i) const noexcept {
    return (i == 0) ? x : (i == 1) ? y : z;
  }

  constexpr Vec3 asVec() const noexcept { return {x, y, z}; }

  constexpr Point3 operator+(const Vec3& v) const noexcept { return {x + v.x, y + v.y, z + v.z}; }
  constexpr Point3 operator-(const Vec3& v) const noexcept { return {x - v.x, y - v.y, z - v.z}; }
  constexpr Vec3   operator-(const Point3& p) const noexcept { return {x - p.x, y - p.y, z - p.z}; }

  constexpr bool operator==(const Point3& o) const noexcept {
    return x == o.x && y == o.y && z == o.z;
  }
};

inline double distance(const Point3& a, const Point3& b) noexcept { return norm(b - a); }

// ─────────────────────────────────────────────────────────────────────────────
// Dir3 — a UNIT vector. Construction normalizes; the invariant |Dir3| == 1 is
// maintained by the type (matches gp_Dir, which raises on a null vector).
// ─────────────────────────────────────────────────────────────────────────────
class Dir3 {
 public:
  /// Unnormalized default is +Z (a valid unit vector).
  constexpr Dir3() noexcept : v_{0.0, 0.0, 1.0} {}

  /// Construct and normalize. If the input is shorter than `tol`, the direction
  /// is left as the raw (near-null) vector and `valid()` returns false — callers
  /// that require a defined direction should check `valid()`. This mirrors the
  /// intent of gp_Dir's ConstructionError without throwing (kernel is noexcept).
  Dir3(double px, double py, double pz, double tol = kLinearTolerance) noexcept {
    normalizeFrom({px, py, pz}, tol);
  }
  explicit Dir3(const Vec3& d, double tol = kLinearTolerance) noexcept {
    normalizeFrom(d, tol);
  }

  constexpr double x() const noexcept { return v_.x; }
  constexpr double y() const noexcept { return v_.y; }
  constexpr double z() const noexcept { return v_.z; }
  constexpr const Vec3& vec() const noexcept { return v_; }
  constexpr double operator[](std::size_t i) const noexcept { return v_[i]; }

  constexpr bool valid() const noexcept { return valid_; }

  constexpr Dir3 reversed() const noexcept { return Dir3{-v_.x, -v_.y, -v_.z, valid_}; }

  /// Angle to another direction in [0, pi]. Uses atan2(|a×b|, a·b) for
  /// numerical robustness near 0 and pi (same formulation as gp_Dir::Angle).
  double angle(const Dir3& o) const noexcept {
    return std::atan2(norm(cross(v_, o.v_)), dot(v_, o.v_));
  }

  /// Build a Dir3 from an already-normalized vector without re-normalizing.
  static constexpr Dir3 fromUnit(const Vec3& unit) noexcept { return Dir3{unit.x, unit.y, unit.z, true}; }

 private:
  // Private trusted constructor: caller asserts the vector is unit-length.
  constexpr Dir3(double px, double py, double pz, bool trustedValid) noexcept
      : v_{px, py, pz}, valid_(trustedValid) {}

  void normalizeFrom(const Vec3& d, double tol) noexcept {
    const double n = norm(d);
    if (n > tol) {
      v_ = d / n;
      valid_ = true;
    } else {
      v_ = d;  // preserve raw (degenerate) value
      valid_ = false;
    }
  }

  Vec3 v_{0.0, 0.0, 1.0};
  bool valid_ = true;
};

constexpr double dot(const Dir3& a, const Dir3& b) noexcept { return dot(a.vec(), b.vec()); }
constexpr Vec3   cross(const Dir3& a, const Dir3& b) noexcept { return cross(a.vec(), b.vec()); }

/// True if the vector's magnitude is below tolerance (treated as null).
inline bool isNull(const Vec3& v, double tol = kLinearTolerance) noexcept {
  return normSquared(v) <= tol * tol;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_VEC_H
