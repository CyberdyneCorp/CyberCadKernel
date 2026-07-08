// SPDX-License-Identifier: Apache-2.0
//
// silhouette.h — closed-form QUADRIC-FACE silhouette tracing for orthographic
//                HLR (MOAT M-GS GS1, curved slice).
//
// The visible OUTLINE of a curved face is a true silhouette curve — the locus on
// the surface where the outward normal is perpendicular to the view direction
// (n·viewDir = 0) — NOT a projected topological edge. This header computes that
// locus in CLOSED FORM for the two cleanest quadrics and hands back world-space
// polylines that the caller appends to its straight-edge set and feeds through the
// EXISTING occlusion + visibility-split path (orthographic_hlr.h). The occlusion
// pass then classifies the silhouette VISIBLE/HIDDEN exactly like any other edge.
//
//   Cylinder (axis a, radius R): the two GENERATOR lines parallel to a at the
//     angles θ* where the radial normal ⟂ viewDir. Straight → 2-point polylines.
//     n(θ) = cosθ·X + sinθ·Y, n·d = 0 ⇒ θ* = atan2(−X·d, Y·d) and θ*+π.
//   Sphere  (centre C, radius R): the GREAT CIRCLE in the plane through C
//     perpendicular to viewDir, radius R. Discretized to a chord-deflection bound.
//
// HONEST DECLINE (never a guessed outline):
//   * viewDir ∥ axis  → the whole cylinder side is silhouette; no isolated
//     generator exists → decline (caller declines the whole projection).
//   * A face whose silhouette is not robustly traceable here (cone / partial
//     quadric / torus / freeform) is NOT handled by this header at all — the
//     caller keeps it a first-class decline.
//
// This header emits ONLY the silhouette geometry; it makes no visibility decision
// (that is the occlusion pass's job) and never fabricates an outline.
//
// OCCT-FREE: includes only src/native/math. Header-only, clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_DRAFTING_SILHOUETTE_H
#define CYBERCAD_NATIVE_DRAFTING_SILHOUETTE_H

#include "native/math/elementary.h"
#include "native/math/vec.h"

#include <cmath>
#include <vector>

namespace cybercad::native::drafting {

namespace math = cybercad::native::math;

/// A world-space silhouette outline. A straight generator is two points; a
/// circle/ellipse silhouette is a discretized loop (`closed` marks the loop so
/// the caller can re-close it if it wishes — the point list already repeats the
/// first point as the last).
struct SilhouettePolyline {
  std::vector<math::Point3> points;
  bool closed = false;
};

/// Outcome of tracing one curved face. `traced == false` means the silhouette is
/// NOT robustly reachable for this face+view — the caller MUST decline the whole
/// projection rather than emit a partial or guessed outline. `declineReason` is a
/// static string for diagnostics.
struct SilhouetteResult {
  std::vector<SilhouettePolyline> outlines;
  bool traced = false;
  const char* declineReason = nullptr;
};

namespace detail {

/// Any unit vector perpendicular to `d` (assumed non-null). Picks the world axis
/// least aligned with `d` to stay well-conditioned.
inline math::Vec3 anyPerp(const math::Vec3& d) noexcept {
  const math::Vec3 pick = (std::fabs(d.x) <= std::fabs(d.y) && std::fabs(d.x) <= std::fabs(d.z))
                              ? math::Vec3{1, 0, 0}
                              : (std::fabs(d.y) <= std::fabs(d.z) ? math::Vec3{0, 1, 0}
                                                                  : math::Vec3{0, 0, 1});
  return math::cross(d, pick);
}

/// Segment count to hold a circle of radius R under a chord-sagitta `deflection`,
/// mirroring tessellate::edgeSegments (Δ ≤ √(8·defl/‖C″‖), ‖C″‖ = R for a circle)
/// AND an angular cap so the loop is visually smooth and comparable to the OCCT
/// oracle's tangential-deflection sampling. Clamped to a sane [min, max].
inline int circleSegments(double radius, double deflection) noexcept {
  constexpr int kMin = 24, kMax = 512;
  constexpr double kTwoPi = 6.28318530717958647692;
  constexpr double kAngularStep = 0.2;  // rad — matches the oracle's angular deflection
  int nSag = kMin;
  if (radius > 0.0 && deflection > 0.0) {
    const double step = std::sqrt(8.0 * deflection / radius);
    if (step > 0.0) nSag = static_cast<int>(std::ceil(kTwoPi / step));
  }
  const int nAng = static_cast<int>(std::ceil(kTwoPi / kAngularStep));
  int n = nSag > nAng ? nSag : nAng;
  if (n < kMin) n = kMin;
  if (n > kMax) n = kMax;
  return n;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Cylinder side-face silhouette: the two generator lines where n·viewDir = 0.
// `frame` is the WORLD-placed Ax3 (origin O on axis, orthonormal X,Y, Z=axis).
// [vMin,vMax] is the face's trim extent measured ALONG Z from O. `viewDir` need
// not be unit. Declines (traced=false) when viewDir ∥ axis.
// ─────────────────────────────────────────────────────────────────────────────
inline SilhouetteResult cylinderSilhouette(const math::Ax3& frame, double radius, double vMin,
                                           double vMax, const math::Vec3& viewDir) {
  SilhouetteResult out;
  const math::Vec3 X = frame.x.vec(), Y = frame.y.vec(), Z = frame.z.vec();
  const double xd = math::dot(X, viewDir);
  const double yd = math::dot(Y, viewDir);
  // In-plane component of the view: if it vanishes the view looks straight down
  // the axis and the ENTIRE side is a silhouette — no isolated generator.
  const double planar = std::hypot(xd, yd);
  const double vn = math::norm(viewDir);
  if (vn <= 0.0 || planar <= 1e-9 * (vn > 0.0 ? vn : 1.0)) {
    out.declineReason = "cylinder: view parallel to axis (whole side is silhouette)";
    return out;
  }
  // n(θ)·d = cosθ·xd + sinθ·yd = 0 ⇒ θ* = atan2(−xd, yd); the antipode is θ*+π.
  const double theta0 = std::atan2(-xd, yd);
  constexpr double kPi = 3.14159265358979323846;
  for (double theta : {theta0, theta0 + kPi}) {
    const math::Vec3 radial = X * std::cos(theta) + Y * std::sin(theta);
    const math::Point3 base = frame.origin + radial * radius;
    SilhouettePolyline gen;
    gen.points.push_back(base + Z * vMin);
    gen.points.push_back(base + Z * vMax);
    out.outlines.push_back(std::move(gen));
  }
  out.traced = true;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Sphere silhouette: the great circle of radius R centred at C in the plane
// through C perpendicular to `viewDir`. `deflection` sizes the polyline (a
// deflection ≤ 0 falls back to a smooth default). Always traced (a sphere is
// never axis-degenerate); FULL-sphere trim is the caller's guard.
// ─────────────────────────────────────────────────────────────────────────────
inline SilhouetteResult sphereSilhouette(const math::Point3& center, double radius,
                                         const math::Vec3& viewDir, double deflection) {
  SilhouetteResult out;
  const double vn = math::norm(viewDir);
  if (vn <= 0.0 || radius <= 0.0) {
    out.declineReason = "sphere: degenerate view or radius";
    return out;
  }
  const math::Vec3 d = viewDir * (1.0 / vn);
  math::Vec3 e1 = detail::anyPerp(d);
  const double e1n = math::norm(e1);
  if (e1n <= 0.0) {
    out.declineReason = "sphere: could not build silhouette plane basis";
    return out;
  }
  e1 = e1 * (1.0 / e1n);
  const math::Vec3 e2 = math::cross(d, e1);  // unit (d,e1 orthonormal)

  const int n = detail::circleSegments(radius, deflection > 0.0 ? deflection : 0.1);
  constexpr double kTwoPi = 6.28318530717958647692;
  SilhouettePolyline loop;
  loop.closed = true;
  loop.points.reserve(static_cast<std::size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = (i == n) ? 0.0 : kTwoPi * static_cast<double>(i) / static_cast<double>(n);
    const math::Vec3 r = e1 * (radius * std::cos(t)) + e2 * (radius * std::sin(t));
    loop.points.push_back(center + r);
  }
  out.outlines.push_back(std::move(loop));
  out.traced = true;
  return out;
}

}  // namespace cybercad::native::drafting

#endif  // CYBERCAD_NATIVE_DRAFTING_SILHOUETTE_H
