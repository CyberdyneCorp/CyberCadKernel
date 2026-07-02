// SPDX-License-Identifier: Apache-2.0
//
// transform.h — Mat3 + affine Transform for the native math kernel.
//
// Clean-room. Convention matches OCCT gp_Mat / gp_Trsf
// (/Users/leonardoaraujo/work/OCCT/src/FoundationClasses/TKMath/gp):
//   * Mat3 stores m[row][col]; a matrix acts on a COLUMN vector: (M·v)_i = Σ_j m[i][j]·v_j.
//   * Transform is an AFFINE map v' = L·v + t, where L is the combined
//     scale·rotation linear part (exactly gp_Trsf's model: a 3×3 linear part
//     plus a translation, not a full 4×4 projective matrix). This is what CAD
//     needs — rigid motions, uniform/non-uniform scale, mirrors — and it inverts
//     and composes in closed form.
//   * Right-handed; determinant sign tracks handedness (mirror ⇒ det<0).
//
// OCCT-FREE. Compiles with clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_TRANSFORM_H
#define CYBERCAD_NATIVE_MATH_TRANSFORM_H

#include "vec.h"

#include <array>
#include <cmath>
#include <optional>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Mat3 — 3×3 matrix, row-major m[row][col]. Acts on column vectors.
// ─────────────────────────────────────────────────────────────────────────────
struct Mat3 {
  // m[row][col]
  std::array<std::array<double, 3>, 3> m{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

  constexpr Mat3() noexcept = default;
  constexpr Mat3(double m00, double m01, double m02,
                 double m10, double m11, double m12,
                 double m20, double m21, double m22) noexcept
      : m{{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}}} {}

  static constexpr Mat3 identity() noexcept { return Mat3{}; }

  static constexpr Mat3 diagonal(double sx, double sy, double sz) noexcept {
    return Mat3{sx, 0, 0, 0, sy, 0, 0, 0, sz};
  }

  constexpr double operator()(std::size_t r, std::size_t c) const noexcept { return m[r][c]; }
  constexpr double& operator()(std::size_t r, std::size_t c) noexcept { return m[r][c]; }

  /// Matrix · column-vector.
  constexpr Vec3 operator*(const Vec3& v) const noexcept {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }

  /// Matrix · matrix.
  constexpr Mat3 operator*(const Mat3& o) const noexcept {
    Mat3 r{0, 0, 0, 0, 0, 0, 0, 0, 0};
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j)
        r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
    return r;
  }

  constexpr double determinant() const noexcept {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  }

  constexpr Mat3 transposed() const noexcept {
    return Mat3{m[0][0], m[1][0], m[2][0],
                m[0][1], m[1][1], m[2][1],
                m[0][2], m[1][2], m[2][2]};
  }

  /// Inverse via adjugate/determinant (Cramer). Returns nullopt if singular.
  constexpr std::optional<Mat3> inverse(double tol = kLinearTolerance) const noexcept {
    const double det = determinant();
    if (det > -tol && det < tol) return std::nullopt;
    const double inv = 1.0 / det;
    Mat3 r{0, 0, 0, 0, 0, 0, 0, 0, 0};
    r.m[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
    r.m[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
    r.m[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
    r.m[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
    r.m[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
    r.m[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
    r.m[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
    r.m[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
    r.m[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
    return r;
  }

  /// Rotation about a UNIT axis by `angle` radians (Rodrigues' rotation formula).
  /// Right-handed (positive angle = CCW looking down the axis toward origin).
  static Mat3 rotation(const Dir3& axis, double angle) noexcept {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    const double t = 1.0 - c;
    const double x = axis.x(), y = axis.y(), z = axis.z();
    return Mat3{t * x * x + c,     t * x * y - s * z, t * x * z + s * y,
                t * x * y + s * z, t * y * y + c,     t * y * z - s * x,
                t * x * z - s * y, t * y * z + s * x, t * z * z + c};
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Transform — affine map v' = linear·v + translation. Models gp_Trsf.
// ─────────────────────────────────────────────────────────────────────────────
class Transform {
 public:
  constexpr Transform() noexcept = default;  // identity
  constexpr Transform(const Mat3& linear, const Vec3& translation) noexcept
      : linear_(linear), translation_(translation) {}

  constexpr const Mat3& linear() const noexcept { return linear_; }
  constexpr const Vec3& translation() const noexcept { return translation_; }

  static constexpr Transform identity() noexcept { return Transform{}; }

  static constexpr Transform translationOf(const Vec3& t) noexcept {
    return Transform{Mat3::identity(), t};
  }

  static Transform rotationOf(const Point3& center, const Dir3& axis, double angle) noexcept {
    const Mat3 r = Mat3::rotation(axis, angle);
    // Rotate about `center`: v' = R·(v − c) + c = R·v + (c − R·c).
    const Vec3 c = center.asVec();
    return Transform{r, c - r * c};
  }

  static constexpr Transform scaleOf(const Point3& center, double s) noexcept {
    const Mat3 m = Mat3::diagonal(s, s, s);
    const Vec3 c = center.asVec();
    return Transform{m, c - m * c};
  }

  static constexpr Transform scaleOf(const Point3& center, double sx, double sy, double sz) noexcept {
    const Mat3 m = Mat3::diagonal(sx, sy, sz);
    const Vec3 c = center.asVec();
    return Transform{m, c - m * c};
  }

  // Apply -------------------------------------------------------------------

  /// Map a point: full affine (linear + translation).
  constexpr Point3 applyToPoint(const Point3& p) const noexcept {
    const Vec3 r = linear_ * p.asVec() + translation_;
    return Point3{r.x, r.y, r.z};
  }

  /// Map a free vector: linear part only (translation does not move vectors).
  constexpr Vec3 applyToVector(const Vec3& v) const noexcept { return linear_ * v; }

  /// Map a direction: linear part then re-normalize (non-uniform scale can
  /// change length). For a pure rotation this is exact and length-preserving.
  Dir3 applyToDir(const Dir3& d) const noexcept { return Dir3{linear_ * d.vec()}; }

  // Compose / invert --------------------------------------------------------

  /// this ∘ other  — apply `other` first, then `this`.
  /// (this∘other)(v) = L·(Lo·v + to) + t = (L·Lo)·v + (L·to + t).
  constexpr Transform composedWith(const Transform& other) const noexcept {
    return Transform{linear_ * other.linear_, linear_ * other.translation_ + translation_};
  }

  /// Inverse affine map, or nullopt if the linear part is singular.
  /// v = L⁻¹·(v' − t)  ⇒  linear' = L⁻¹, translation' = −L⁻¹·t.
  constexpr std::optional<Transform> inverse(double tol = kLinearTolerance) const noexcept {
    const auto li = linear_.inverse(tol);
    if (!li) return std::nullopt;
    return Transform{*li, -(*li * translation_)};
  }

  constexpr double determinant() const noexcept { return linear_.determinant(); }
  constexpr bool isMirrored() const noexcept { return determinant() < 0.0; }

 private:
  Mat3 linear_{};       // combined scale·rotation
  Vec3 translation_{};
};

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_TRANSFORM_H
