// SPDX-License-Identifier: Apache-2.0
//
// inertia.h — GS5 mass-inertia: volume, centroid, the inertia tensor and its
// principal moments / axes of a solid, computed OCCT-FREE from the solid's M0
// triangulation (the native tessellator's watertight boundary mesh).
//
// METHOD (signed-tetra second moments + parallel-axis + symmetric eigen):
//   For a closed, outward-CCW-wound mesh the solid decomposes into signed
//   tetrahedra fanned from the origin, one per boundary triangle (a,b,c). Each
//   tetra (O,a,b,c) contributes its exact second-moment (covariance) integral
//       ∫_tet x xᵀ dV = (v/20)·(a aᵀ + b bᵀ + c cᵀ + s sᵀ),   s = a+b+c,
//   with signed volume v = a·(b×c)/6 = det[a b c]/6 (the /20 form is the exact
//   integral of the canonical tetra covariance (I+J)/120 mapped by the columns
//   [a b c] and scaled by det). Summing over the fan telescopes to the solid's
//   covariance about the ORIGIN; the parallel-axis shift −V·c cᵀ moves it to the
//   CENTROID c, and the inertia tensor about the centroid is
//       I = tr(Cov_c)·Id − Cov_c   (unit density; multiply by ρ for mass inertia).
//   A symmetric-3×3 cyclic-Jacobi eigen-decomposition yields the principal
//   moments (eigenvalues) and principal axes (eigenvectors).
//
//   Planar-faced solids (box/prism) are reproduced to machine precision (the mesh
//   is the exact boundary); curved solids match to a deflection-scaled tolerance
//   (the residual is M0 deflection, not the algorithm) — the same mesh-property
//   philosophy mesh.h already codifies for area/volume.
//
// SELF-VERIFY / HONEST DECLINE (std::nullopt): the caller MUST pass a WATERTIGHT
// mesh (tessellate::isWatertight) — an open / non-closed boundary has no defined
// enclosed inertia, so a non-watertight or ~zero-volume mesh DECLINES rather than
// emit a wrong tensor. OCCT is the ORACLE only (the sim gate GProp_PrincipalProps);
// no OCCT header is included here. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_INERTIA_H
#define CYBERCAD_NATIVE_ANALYSIS_INERTIA_H

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <utility>

#include "native/math/vec.h"
#include "native/tessellate/mesh.h"

namespace cybercad::native::analysis {

namespace math = cybercad::native::math;
namespace tessellate = cybercad::native::tessellate;

/// Principal inertia of a solid (unit density) computed about its centroid.
struct InertiaResult {
  double volume = 0.0;             ///< enclosed volume (> 0)
  math::Point3 centroid{};         ///< centre of mass
  std::array<double, 3> moments{}; ///< principal moments I1 ≤ I2 ≤ I3 (about centroid)
  std::array<math::Vec3, 3> axes{};///< principal axes (unit), axes[i] ↔ moments[i]
  std::array<std::array<double, 3>, 3> tensor{};  ///< inertia tensor about centroid
};

namespace detail {

/// Symmetric 3×3 cyclic-Jacobi eigen-decomposition. `a` is overwritten; on return
/// `eval[i]` are the eigenvalues and the columns of `evec` the unit eigenvectors
/// (evec[row][col]). Converges in a handful of sweeps for a 3×3 (rotations kill
/// each off-diagonal in turn); always terminates via the sweep cap.
inline void jacobiEigenSym3(double a[3][3], double eval[3], double evec[3][3]) {
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) evec[i][j] = (i == j) ? 1.0 : 0.0;

  for (int sweep = 0; sweep < 50; ++sweep) {
    const double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
    if (off < 1e-300) break;  // already diagonal
    for (int p = 0; p < 2; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        if (std::fabs(a[p][q]) <= 1e-300) continue;
        const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        const double t = (theta >= 0.0 ? 1.0 : -1.0) /
                         (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        const double c = 1.0 / std::sqrt(t * t + 1.0);
        const double s = t * c;
        const double app = a[p][p], aqq = a[q][q], apq = a[p][q];
        a[p][p] = c * c * app - 2.0 * s * c * apq + s * s * aqq;
        a[q][q] = s * s * app + 2.0 * s * c * apq + c * c * aqq;
        a[p][q] = a[q][p] = 0.0;
        const int r = 3 - p - q;  // the third index
        const double arp = a[r][p], arq = a[r][q];
        a[r][p] = a[p][r] = c * arp - s * arq;
        a[r][q] = a[q][r] = s * arp + c * arq;
        for (int i = 0; i < 3; ++i) {
          const double vip = evec[i][p], viq = evec[i][q];
          evec[i][p] = c * vip - s * viq;
          evec[i][q] = s * vip + c * viq;
        }
      }
    }
  }
  for (int i = 0; i < 3; ++i) eval[i] = a[i][i];
}

}  // namespace detail

/// Principal inertia of the solid bounded by `mesh`, or HONEST DECLINE
/// (std::nullopt) when the mesh is not watertight or encloses ~zero volume.
/// Precondition: `mesh` is the outward-oriented closed boundary of the solid.
inline std::optional<InertiaResult> principalInertia(const tessellate::Mesh& mesh) {
  if (!tessellate::isWatertight(mesh)) return std::nullopt;  // no defined inertia

  double vol6 = 0.0;                       // 6·V accumulator
  std::array<double, 3> cent{0, 0, 0};     // Σ v6·(a+b+c) (=> centroid after /(4·vol6))
  double cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};  // covariance about origin

  auto outer = [&](const math::Vec3& u, const math::Vec3& w, double f) {
    cov[0][0] += f * u.x * w.x; cov[1][1] += f * u.y * w.y; cov[2][2] += f * u.z * w.z;
    cov[0][1] += f * u.x * w.y; cov[0][2] += f * u.x * w.z; cov[1][2] += f * u.y * w.z;
  };

  for (const tessellate::Triangle& t : mesh.triangles) {
    const math::Vec3 a = mesh.vertices[t.a].asVec();
    const math::Vec3 b = mesh.vertices[t.b].asVec();
    const math::Vec3 c = mesh.vertices[t.c].asVec();
    const double v6 = math::dot(a, math::cross(b, c));  // = det[a b c] = 6·v
    vol6 += v6;
    const math::Vec3 s = a + b + c;
    cent[0] += v6 * s.x; cent[1] += v6 * s.y; cent[2] += v6 * s.z;
    // ∫x xᵀ over the tetra = (v/20)(aaᵀ+bbᵀ+ccᵀ+ssᵀ) and v = v6/6 ⇒ factor v6/120.
    const double f = v6 / 120.0;
    outer(a, a, f); outer(b, b, f); outer(c, c, f); outer(s, s, f);
  }

  if (std::fabs(vol6) <= 1e-15) return std::nullopt;  // degenerate: no robust inertia

  // Normalise the orientation sign so V > 0 and the covariance is positive-definite
  // (an inward-wound mesh flips every signed contribution consistently).
  const double sgn = (vol6 >= 0.0) ? 1.0 : -1.0;
  const double V = sgn * vol6 / 6.0;
  const math::Point3 c{cent[0] / (4.0 * vol6), cent[1] / (4.0 * vol6),
                       cent[2] / (4.0 * vol6)};  // sign-independent (num & den flip)

  // Mirror the upper triangle, apply the sign, then the parallel-axis shift to the
  // centroid: Cov_c = Cov0 − V·c cᵀ.
  double m[3][3];
  const double cc[3] = {c.x, c.y, c.z};
  m[0][0] = sgn * cov[0][0]; m[1][1] = sgn * cov[1][1]; m[2][2] = sgn * cov[2][2];
  m[0][1] = m[1][0] = sgn * cov[0][1];
  m[0][2] = m[2][0] = sgn * cov[0][2];
  m[1][2] = m[2][1] = sgn * cov[1][2];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) m[i][j] -= V * cc[i] * cc[j];

  // Inertia tensor about the centroid: I = tr(Cov_c)·Id − Cov_c.
  const double tr = m[0][0] + m[1][1] + m[2][2];
  double I[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) I[i][j] = (i == j ? tr : 0.0) - m[i][j];

  double eval[3], evec[3][3];
  double Icopy[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) Icopy[i][j] = I[i][j];
  detail::jacobiEigenSym3(Icopy, eval, evec);

  // Sort ascending by eigenvalue so moments[0] ≤ moments[1] ≤ moments[2], carrying
  // the matching axis (eigenvector column).
  int order[3] = {0, 1, 2};
  for (int i = 0; i < 3; ++i)
    for (int j = i + 1; j < 3; ++j)
      if (eval[order[j]] < eval[order[i]]) std::swap(order[i], order[j]);

  InertiaResult r;
  r.volume = V;
  r.centroid = c;
  for (int k = 0; k < 3; ++k) {
    const int e = order[k];
    r.moments[k] = eval[e];
    r.axes[k] = math::Vec3{evec[0][e], evec[1][e], evec[2][e]};
    for (int i = 0; i < 3; ++i) r.tensor[k][i] = I[k][i];
  }
  return r;
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_INERTIA_H
