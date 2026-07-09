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
//   Cone    (apex O, half-angle α): the two straight CONTOUR GENERATORS (rulings)
//     at the angles u* where the tilted surface normal ⟂ viewDir. The outward
//     normal on a cone is n(u) = cosα·(cos u·X + sin u·Y) − sinα·Z, so n·d = 0 ⇒
//     cosα·(cos u·Xd + sin u·Yd) = sinα·Zd, i.e. hypot(Xd,Yd)·cos(u−φ) = tanα·Zd
//     (φ = atan2(Yd,Xd)); the two roots are the two rulings, each a straight
//     segment over the frustum's axial trim [vMin,vMax] (or apex→rim for a full
//     cone). Straight → 2-point polylines, like the cylinder.
//   Torus   (major R, minor r): the TURNING-POINT contour under orthographic
//     projection — the locus where the outward normal ⟂ viewDir. The normal is
//     n(u,v) = cos v·(cos u·X + sin u·Y) + sin v·Z, so per MAJOR angle u the
//     silhouette minor angles solve cos v·P(u) + sin v·Zd = 0, P(u)=cos u·Xd +
//     sin u·Yd ⇒ v = atan2(−P(u), Zd) and v+π. Sweeping u over [0,2π) traces the
//     two closed turning contours (the outer + inner limbs). Discretized in u to a
//     chord bound; an on-surface, ⟂-view point set, NOT a rim circle.
//
// HONEST DECLINE (never a guessed outline):
//   * viewDir ∥ axis  → the whole cylinder/cone side is silhouette; no isolated
//     generator exists → decline (caller declines the whole projection).
//   * A cone view whose |tanα·Zd| exceeds hypot(Xd,Yd) sees the cone end-on (no
//     lateral silhouette ruling exists) → decline.
//   * A view exactly down a torus axis makes the turning contour the two rim
//     circles themselves (a boundary case) → decline rather than emit a
//     near-degenerate contour.
//   * A face whose silhouette is not robustly traceable here (partial quadric /
//     freeform B-spline / Bézier) is NOT handled by this header at all — the
//     caller keeps it a first-class decline. A native REVOLVE builds a torus as
//     rational-B-spline bands (Kind::BSpline), NOT a Kind::Torus face, so those
//     tori still decline; a Kind::Torus face (STEP-imported) is traced here.
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

// ─────────────────────────────────────────────────────────────────────────────
// Cone / cone-frustum lateral silhouette: the two straight CONTOUR GENERATORS
// (rulings) where the tilted cone normal n·viewDir = 0. `frame` is the WORLD Ax3
// (origin O on the axis at the reference height, Z=axis). `refRadius` is the cone
// radius at v=0 measured from O along Z; `semiAngle` α is the half-angle. The
// face's axial trim is [hMin,hMax] measured ALONG Z from O (axial HEIGHT, not the
// slant parameter); the ruling is the straight segment between the rim radii at
// those two heights (a full cone has one end at the apex where the radius is 0).
// Declines when the view is end-on (no lateral ruling exists) or parallel to the
// axis (whole side is silhouette).
//
// A cone point at (u,v): r(v) = refRadius + v·sinα, axial height h = v·cosα, so
//   P(u,h) = O + (refRadius + (h/cosα)·sinα)·(cos u·X + sin u·Y) + h·Z
//          = O + (refRadius + h·tanα)·(cos u·X + sin u·Y) + h·Z.
// The outward normal (independent of h along a ruling) is
//   n(u) = cosα·(cos u·X + sin u·Y) − sinα·Z.
// n·d = 0 ⇒ cosα·(cos u·Xd + sin u·Yd) = sinα·Zd.
// ─────────────────────────────────────────────────────────────────────────────
inline SilhouetteResult coneSilhouette(const math::Ax3& frame, double refRadius, double semiAngle,
                                       double hMin, double hMax, const math::Vec3& viewDir) {
  SilhouetteResult out;
  const math::Vec3 X = frame.x.vec(), Y = frame.y.vec(), Z = frame.z.vec();
  const double vn = math::norm(viewDir);
  if (vn <= 0.0 || !(hMax > hMin)) {
    out.declineReason = "cone: degenerate view or empty axial trim";
    return out;
  }
  const double xd = math::dot(X, viewDir);
  const double yd = math::dot(Y, viewDir);
  const double zd = math::dot(Z, viewDir);
  const double planar = std::hypot(xd, yd);
  if (planar <= 1e-9 * vn) {
    out.declineReason = "cone: view parallel to axis (whole side is silhouette)";
    return out;
  }
  const double ca = std::cos(semiAngle), sa = std::sin(semiAngle);
  // cosα·planar·cos(u−φ) = sinα·Zd, φ = atan2(yd, xd). Solve cos(u−φ) = rhs.
  const double denom = ca * planar;
  if (std::fabs(denom) <= 1e-15) {
    out.declineReason = "cone: degenerate half-angle for silhouette";
    return out;
  }
  const double rhs = (sa * zd) / denom;
  if (std::fabs(rhs) > 1.0 + 1e-12) {
    // |cos(u−φ)| would exceed 1: the cone is seen end-on, no lateral ruling ⟂ view.
    out.declineReason = "cone: view end-on (no lateral silhouette ruling)";
    return out;
  }
  const double clamped = rhs > 1.0 ? 1.0 : (rhs < -1.0 ? -1.0 : rhs);
  const double phi = std::atan2(yd, xd);
  const double delta = std::acos(clamped);
  // Two rulings at u = φ ± delta (antipodal when delta≈π/2, coincident when |rhs|→1).
  const double tana = ca != 0.0 ? sa / ca : 0.0;
  for (double u : {phi + delta, phi - delta}) {
    const math::Vec3 radial = X * std::cos(u) + Y * std::sin(u);
    auto conePoint = [&](double h) {
      const double r = refRadius + h * tana;  // rim radius at axial height h
      return frame.origin + radial * r + Z * h;
    };
    SilhouettePolyline gen;
    gen.points.push_back(conePoint(hMin));
    gen.points.push_back(conePoint(hMax));
    out.outlines.push_back(std::move(gen));
  }
  out.traced = true;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Torus turning-point silhouette: the two closed contours where the outward
// normal n·viewDir = 0. `frame` is the WORLD Ax3 (origin = torus centre, Z=axis).
// `majorRadius` R, `minorRadius` r. Per major angle u the silhouette minor angles
// solve cos v·P(u) + sin v·Zd = 0, P(u)=cos u·Xd + sin u·Yd ⇒ v* = atan2(−P(u),Zd)
// and v*+π. Sweeping u ∈ [0,2π) traces the two turning contours (outer + inner
// limbs) as discretized closed loops on the surface. Declines when the view is
// down the axis (Zd ≈ ±1 with P(u)→0 makes v* ill-conditioned: the turning
// contour degenerates to the rim circles) or degenerate radii.
// ─────────────────────────────────────────────────────────────────────────────
inline SilhouetteResult torusSilhouette(const math::Ax3& frame, double majorRadius,
                                        double minorRadius, const math::Vec3& viewDir,
                                        double deflection) {
  SilhouetteResult out;
  const double vn = math::norm(viewDir);
  if (vn <= 0.0 || minorRadius <= 0.0 || majorRadius <= 0.0) {
    out.declineReason = "torus: degenerate view or radii";
    return out;
  }
  const math::Vec3 X = frame.x.vec(), Y = frame.y.vec(), Z = frame.z.vec();
  const double xd = math::dot(X, viewDir) / vn;
  const double yd = math::dot(Y, viewDir) / vn;
  const double zd = math::dot(Z, viewDir) / vn;
  const double planar = std::hypot(xd, yd);
  // View (near) parallel to the axis: cos v·P(u)+sin v·Zd=0 has P(u)≈0 for all u,
  // so the turning contour collapses onto the two rim circles (a boundary case).
  // Decline rather than emit a near-degenerate contour.
  if (planar <= 1e-6) {
    out.declineReason = "torus: view parallel to axis (contour degenerates to rim circles)";
    return out;
  }
  // Sample the MAJOR angle finely enough that the widest contour (radius R+r) holds
  // the chord-sagitta deflection bound; one loop per branch (v* and v*+π).
  const int n = detail::circleSegments(majorRadius + minorRadius, deflection > 0.0 ? deflection : 0.1);
  constexpr double kTwoPi = 6.28318530717958647692;
  constexpr double kPi = 3.14159265358979323846;
  auto torusPoint = [&](double u, double v) {
    const double rr = majorRadius + minorRadius * std::cos(v);
    return frame.origin + (X * std::cos(u) + Y * std::sin(u)) * rr + Z * (minorRadius * std::sin(v));
  };
  for (double branch : {0.0, kPi}) {
    SilhouettePolyline loop;
    loop.closed = true;
    loop.points.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double u = (i == n) ? 0.0 : kTwoPi * static_cast<double>(i) / static_cast<double>(n);
      const double p = std::cos(u) * xd + std::sin(u) * yd;  // P(u)
      const double v = std::atan2(-p, zd) + branch;          // silhouette minor angle
      loop.points.push_back(torusPoint(u, v));
    }
    out.outlines.push_back(std::move(loop));
  }
  out.traced = true;
  return out;
}

}  // namespace cybercad::native::drafting

#endif  // CYBERCAD_NATIVE_DRAFTING_SILHOUETTE_H
