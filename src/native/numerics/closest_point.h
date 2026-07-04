// SPDX-License-Identifier: Apache-2.0
//
// closest_point.h — native closest-point / projection (the OCCT `Extrema`
// on-ramp), Phase-4 #2 numeric-foundations.
//
// This is the TYPED closest-point layer: it projects a 3D point onto any of the
// native math geometries (src/native/math) — Bézier / B-spline / NURBS curves &
// surfaces + the elementary analytic surfaces (plane / cylinder / cone / sphere /
// torus) — and reports the nearest parameter, the foot point, the distance, and
// whether the nearest lies on a parameter-range endpoint / boundary.
//
// ALGORITHM (mirrors OCCT Extrema_ExtPC / Extrema_GenExtPS — oracle only, not
// copied; see /Users/leonardoaraujo/work/OCCT/src/ModelingData/TKGeomBase/Extrema):
//
//   1. SAMPLE the parameter domain on a coarse grid and evaluate the squared
//      distance to the target at every node.
//   2. Find every LOCAL-MINIMUM node — a node whose squared distance is ≤ all of
//      its immediate grid neighbours (and the two end nodes are always kept as
//      candidates, so an endpoint minimum is never missed). OCCT calls this its
//      `isMin` grid test; each surviving node is a MULTI-START seed.
//   3. REFINE each seed with a local minimize of the squared distance over the
//      clamped parameter range (facade `minimize`, SciPP BFGS), then re-clamp.
//   4. DEDUP refined results that collapse to the same parameter and keep them as
//      the set of local minima, sorted nearest-first. The global nearest is the
//      first entry; `Result::extrema` optionally exposes all of them.
//
// Multi-start (one refine PER local-min seed, not just from the single best node)
// is what makes this robust on a wiggly curve / surface where the global minimum
// is not in the basin of the coarsest-distance seed — exactly the case the eval
// (docs/EVAL-numpp-scipp.md EXP1) exercises on a bicubic B-spline.
//
// This header sits ON TOP of the numerics.h facade (which owns the substrate) and
// the native math types. It NEVER includes NumPP / SciPP directly, so it inherits
// the same CYBERCAD_HAS_NUMSCI guard as the rest of the numerics module: only the
// facade TU (numerics.cpp) is compiled under the flag, and the projection helpers
// here are ordinary header code that calls the facade's `minimize`.
//
// clang++ -std=c++20. fp64, deterministic. OCCT-free.
//
#ifndef CYBERCAD_NATIVE_NUMERICS_CLOSEST_POINT_H
#define CYBERCAD_NATIVE_NUMERICS_CLOSEST_POINT_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include "native/math/bezier.h"
#include "native/math/bspline.h"
#include "native/math/elementary.h"
#include "native/math/torus.h"
#include "native/math/vec.h"
#include "native/numerics/numerics.h"

namespace cybercad::native::numerics {

using cybercad::native::math::Point3;

// ─────────────────────────────────────────────────────────────────────────────
// Results.
//
// A single extremum (local minimum of the point→geometry distance). The typed
// projection functions return the GLOBAL nearest in the top-level fields and,
// optionally, the full set of local minima in `extrema` (sorted nearest-first),
// mirroring the fact that OCCT Extrema returns multiple extrema.
// ─────────────────────────────────────────────────────────────────────────────

/// One local minimum of the distance from the target to a CURVE.
struct CurveExtremum {
  double t = 0.0;         ///< curve parameter of this extremum
  Point3 point{};         ///< C(t)
  double distance = 0.0;  ///< ‖C(t) − target‖
  bool onEndpoint = false;///< the nearest coincides with a range endpoint (t0 or t1)
};

/// One local minimum of the distance from the target to a SURFACE.
struct SurfaceExtremum {
  double u = 0.0;
  double v = 0.0;
  Point3 point{};         ///< S(u,v)
  double distance = 0.0;  ///< ‖S(u,v) − target‖
  bool onBoundary = false;///< the nearest lies on a (u,v) domain boundary
};

/// Full result of projecting onto a curve: the global nearest + all local minima.
struct CurveClosest {
  bool success = false;
  double t = 0.0;
  Point3 point{};
  double distance = 0.0;
  bool onEndpoint = false;
  std::vector<CurveExtremum> extrema;  ///< all local minima, sorted nearest-first
};

/// Full result of projecting onto a surface: the global nearest + all local minima.
struct SurfaceClosest {
  bool success = false;
  double u = 0.0;
  double v = 0.0;
  Point3 point{};
  double distance = 0.0;
  bool onBoundary = false;
  std::vector<SurfaceExtremum> extrema;  ///< all local minima, sorted nearest-first
};

// ─────────────────────────────────────────────────────────────────────────────
// Grid density defaults. A curve gets a 1-D scan; a surface a 2-D grid. These
// mirror OCCT's default sample counts (Extrema_ExtPS defaults ~10 per direction
// on an analytic surface, denser on a NURBS); the caller may raise them for a
// highly oscillatory geometry.
// ─────────────────────────────────────────────────────────────────────────────
inline constexpr int kDefaultCurveSamples = 32;
inline constexpr int kDefaultSurfaceSamplesU = 16;
inline constexpr int kDefaultSurfaceSamplesV = 16;

// ─────────────────────────────────────────────────────────────────────────────
// Core evaluator-based projection (the generic engine). Any typed geometry
// projects by handing its own value(t) / value(u,v) as a callable. This is the
// multi-start Extrema loop; the typed overloads below are thin adapters.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

inline double clampRange(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

inline double sq3(const Point3& p, const Point3& q) {
  const double dx = p.x - q.x, dy = p.y - q.y, dz = p.z - q.z;
  return dx * dx + dy * dy + dz * dz;
}

inline double dot3(const Point3& a, const Point3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Point3 sub3(const Point3& a, const Point3& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

// ── fp64 Newton polish of a single extremum (the accuracy-hardening step) ──────
//
// BFGS on the squared distance nails the DISTANCE but stops when its gradient is
// small; near the minimum the objective is very flat TANGENTIALLY (a large change
// in the parameter moves the foot point only slightly), so the returned parameter
// — and thus the foot point — can still sit ~1e-6..1e-7 off the true foot. That is
// exactly the residual the OCCT-Extrema parity harness sees on a sphere/cylinder.
//
// The true nearest parameter satisfies the FIRST-ORDER optimality (orthogonality)
// condition g = 0, where for a curve g(t) = (C(t) − T)·C'(t) and for a surface
// g(u,v) = ((S − T)·Sᵤ, (S − T)·Sᵥ). A few damped-Newton steps on g (Jacobian and
// derivatives from central finite differences of the evaluator — no geometry
// specifics, so it works for every native curve/surface) converge quadratically to
// fp64 and pin the foot point to ~1e-14. Steps stay inside the clamped range and
// are rejected if they do not reduce the squared distance, so polish can only
// improve (never worsen) the BFGS result. This is a local refinement of an
// already-correct basin, NOT a replacement for the multi-start global search.

/// Newton-polish a curve parameter `t` (in [t0,t1]) toward the orthogonality root.
template <class CurveFn>
double polishCurveParam(const CurveFn& c, const Point3& target, double t,
                        double t0, double t1) {
  const double span = t1 - t0;
  const double h = std::max(1e-7, 1e-7 * span);  // FD step, scaled to the domain
  double bestT = clampRange(t, t0, t1);
  double bestD2 = sq3(c(bestT), target);
  for (int it = 0; it < 8; ++it) {
    const Point3 cp = c(bestT);
    const Point3 cph = c(clampRange(bestT + h, t0, t1));
    const Point3 cmh = c(clampRange(bestT - h, t0, t1));
    const Point3 d1{(cph.x - cmh.x) / (2 * h), (cph.y - cmh.y) / (2 * h),
                    (cph.z - cmh.z) / (2 * h)};
    const Point3 d2{(cph.x - 2 * cp.x + cmh.x) / (h * h),
                    (cph.y - 2 * cp.y + cmh.y) / (h * h),
                    (cph.z - 2 * cp.z + cmh.z) / (h * h)};
    const Point3 r = sub3(cp, target);
    const double g = dot3(r, d1);                       // ½ d/dt ‖C−T‖²
    const double gp = dot3(d1, d1) + dot3(r, d2);       // its derivative
    if (std::fabs(gp) < 1e-30) break;
    double step = g / gp;
    // damped, range-clamped Newton; accept only if it reduces the squared distance.
    bool improved = false;
    for (int bt = 0; bt < 20; ++bt) {
      const double nt = clampRange(bestT - step, t0, t1);
      const double nd2 = sq3(c(nt), target);
      if (nd2 <= bestD2) { bestT = nt; bestD2 = nd2; improved = true; break; }
      step *= 0.5;
    }
    if (!improved || std::fabs(step) <= 1e-16 * (1.0 + std::fabs(bestT))) break;
  }
  return bestT;
}

/// Newton-polish a surface parameter pair toward the (gᵤ,gᵥ)=0 orthogonality root.
template <class SurfFn>
std::pair<double, double> polishSurfaceParam(const SurfFn& s, const Point3& target,
                                             double u, double v, double u0, double u1,
                                             double v0, double v1) {
  const double hu = std::max(1e-7, 1e-7 * (u1 - u0));
  const double hv = std::max(1e-7, 1e-7 * (v1 - v0));
  double bu = clampRange(u, u0, u1), bv = clampRange(v, v0, v1);
  double bestD2 = sq3(s(bu, bv), target);
  for (int it = 0; it < 8; ++it) {
    const Point3 p = s(bu, bv);
    const Point3 pu2 = s(clampRange(bu + hu, u0, u1), bv);
    const Point3 pu0 = s(clampRange(bu - hu, u0, u1), bv);
    const Point3 pv2 = s(bu, clampRange(bv + hv, v0, v1));
    const Point3 pv0 = s(bu, clampRange(bv - hv, v0, v1));
    const Point3 Su{(pu2.x - pu0.x) / (2 * hu), (pu2.y - pu0.y) / (2 * hu),
                    (pu2.z - pu0.z) / (2 * hu)};
    const Point3 Sv{(pv2.x - pv0.x) / (2 * hv), (pv2.y - pv0.y) / (2 * hv),
                    (pv2.z - pv0.z) / (2 * hv)};
    const Point3 Suu{(pu2.x - 2 * p.x + pu0.x) / (hu * hu),
                     (pu2.y - 2 * p.y + pu0.y) / (hu * hu),
                     (pu2.z - 2 * p.z + pu0.z) / (hu * hu)};
    const Point3 Svv{(pv2.x - 2 * p.x + pv0.x) / (hv * hv),
                     (pv2.y - 2 * p.y + pv0.y) / (hv * hv),
                     (pv2.z - 2 * p.z + pv0.z) / (hv * hv)};
    const Point3 puv = s(clampRange(bu + hu, u0, u1), clampRange(bv + hv, v0, v1));
    const Point3 Suv{(puv.x - pu2.x - pv2.x + p.x) / (hu * hv),
                     (puv.y - pu2.y - pv2.y + p.y) / (hu * hv),
                     (puv.z - pu2.z - pv2.z + p.z) / (hu * hv)};
    const Point3 r = sub3(p, target);
    const double gu = dot3(r, Su), gv = dot3(r, Sv);         // gradient of ½‖S−T‖²
    // 2×2 Hessian J of (gu,gv).
    const double J00 = dot3(Su, Su) + dot3(r, Suu);
    const double J01 = dot3(Su, Sv) + dot3(r, Suv);
    const double J11 = dot3(Sv, Sv) + dot3(r, Svv);
    const double det = J00 * J11 - J01 * J01;
    if (std::fabs(det) < 1e-30) break;
    double du = (J11 * gu - J01 * gv) / det;                 // J⁻¹ · g
    double dv = (J00 * gv - J01 * gu) / det;
    bool improved = false;
    for (int bt = 0; bt < 20; ++bt) {
      const double nu = clampRange(bu - du, u0, u1), nv = clampRange(bv - dv, v0, v1);
      const double nd2 = sq3(s(nu, nv), target);
      if (nd2 <= bestD2) { bu = nu; bv = nv; bestD2 = nd2; improved = true; break; }
      du *= 0.5; dv *= 0.5;
    }
    if (!improved ||
        (std::fabs(du) <= 1e-16 * (1.0 + std::fabs(bu)) &&
         std::fabs(dv) <= 1e-16 * (1.0 + std::fabs(bv))))
      break;
  }
  return {bu, bv};
}

/// Project onto a parametric CURVE given as an evaluator C:[t0,t1]→R³.
template <class CurveFn>
CurveClosest projectCurveEval(const CurveFn& c, double t0, double t1,
                              const Point3& target, int samples,
                              double endpointTol) {
  CurveClosest out;
  if (!(t1 > t0)) {  // degenerate range: single-point "curve"
    const Point3 p = c(t0);
    out.success = true; out.t = t0; out.point = p;
    out.distance = std::sqrt(sq3(p, target)); out.onEndpoint = true;
    out.extrema.push_back({t0, p, out.distance, true});
    return out;
  }
  const int n = std::max(2, samples);
  const double span = t1 - t0;

  // (1) sample the range.
  std::vector<double> d2(n + 1);
  for (int i = 0; i <= n; ++i)
    d2[i] = sq3(c(t0 + span * (double(i) / n)), target);

  // (2) collect local-minimum seeds (a node ≤ both neighbours; ends always kept).
  std::vector<double> seeds;
  for (int i = 0; i <= n; ++i) {
    const bool loOk = (i == 0) || d2[i] <= d2[i - 1];
    const bool hiOk = (i == n) || d2[i] <= d2[i + 1];
    if (loOk && hiOk) seeds.push_back(t0 + span * (double(i) / n));
  }
  if (seeds.empty()) seeds.push_back(t0);  // safety; cannot normally happen

  // (3) refine each seed with a local 1-D minimize of the clamped sq distance.
  auto obj = [&](const Vector& x) {
    return sq3(c(clampRange(x[0], t0, t1)), target);
  };
  for (double seed : seeds) {
    SolveResult m = minimize(obj, {seed}, 1e-12, 200);
    // pick whichever of {refined, seed} is nearer (refine may overshoot a clamp).
    double bestT = seed; double bestD2 = sq3(c(seed), target);
    if (m.success) {
      const double tc = clampRange(m.x[0], t0, t1);
      const double dc = sq3(c(tc), target);
      if (dc <= bestD2) { bestT = tc; bestD2 = dc; }
    }
    // fp64 Newton polish on the orthogonality condition (only improves).
    bestT = polishCurveParam(c, target, bestT, t0, t1);
    bestD2 = sq3(c(bestT), target);
    const Point3 p = c(bestT);
    const bool endp = (bestT - t0) <= endpointTol * span ||
                      (t1 - bestT) <= endpointTol * span;
    out.extrema.push_back({bestT, p, std::sqrt(bestD2), endp});
  }

  // (4) dedup collapsed extrema, sort nearest-first, publish the global nearest.
  std::sort(out.extrema.begin(), out.extrema.end(),
            [](const CurveExtremum& a, const CurveExtremum& b) { return a.distance < b.distance; });
  const double mergeTol = 1e-7 * span;
  std::vector<CurveExtremum> uniq;
  for (const CurveExtremum& e : out.extrema) {
    bool dup = false;
    for (const CurveExtremum& u : uniq)
      if (std::fabs(u.t - e.t) <= mergeTol) { dup = true; break; }
    if (!dup) uniq.push_back(e);
  }
  out.extrema = std::move(uniq);
  const CurveExtremum& g = out.extrema.front();
  out.success = true; out.t = g.t; out.point = g.point;
  out.distance = g.distance; out.onEndpoint = g.onEndpoint;
  return out;
}

/// Project onto a parametric SURFACE given as an evaluator S:[u0,u1]×[v0,v1]→R³.
template <class SurfFn>
SurfaceClosest projectSurfaceEval(const SurfFn& s, double u0, double u1,
                                  double v0, double v1, const Point3& target,
                                  int su, int sv, double boundaryTol) {
  SurfaceClosest out;
  const int nu = std::max(2, su), nv = std::max(2, sv);
  const double uSpan = (u1 > u0) ? u1 - u0 : 1.0;
  const double vSpan = (v1 > v0) ? v1 - v0 : 1.0;
  auto uAt = [&](int i) { return u0 + (u1 - u0) * (double(i) / nu); };
  auto vAt = [&](int j) { return v0 + (v1 - v0) * (double(j) / nv); };

  // (1) sample the grid.
  std::vector<double> d2(static_cast<std::size_t>(nu + 1) * (nv + 1));
  auto idx = [&](int i, int j) { return static_cast<std::size_t>(i) * (nv + 1) + j; };
  for (int i = 0; i <= nu; ++i)
    for (int j = 0; j <= nv; ++j)
      d2[idx(i, j)] = sq3(s(uAt(i), vAt(j)), target);

  // (2) collect local-minimum seeds (a node ≤ its 4-neighbours; edges compare
  //     only against existing neighbours, so a boundary cell can still seed).
  std::vector<std::pair<double, double>> seeds;
  for (int i = 0; i <= nu; ++i)
    for (int j = 0; j <= nv; ++j) {
      const double c = d2[idx(i, j)];
      const bool ok = (i == 0 || c <= d2[idx(i - 1, j)]) &&
                      (i == nu || c <= d2[idx(i + 1, j)]) &&
                      (j == 0 || c <= d2[idx(i, j - 1)]) &&
                      (j == nv || c <= d2[idx(i, j + 1)]);
      if (ok) seeds.push_back({uAt(i), vAt(j)});
    }
  if (seeds.empty()) seeds.push_back({u0, v0});

  // (3) refine each seed with a local 2-D minimize of the clamped sq distance.
  auto obj = [&](const Vector& x) {
    return sq3(s(clampRange(x[0], u0, u1), clampRange(x[1], v0, v1)), target);
  };
  for (auto [su0, sv0] : seeds) {
    SolveResult m = minimize(obj, {su0, sv0}, 1e-12, 400);
    double bu = su0, bv = sv0, bestD2 = sq3(s(su0, sv0), target);
    if (m.success) {
      const double uc = clampRange(m.x[0], u0, u1), vc = clampRange(m.x[1], v0, v1);
      const double dc = sq3(s(uc, vc), target);
      if (dc <= bestD2) { bu = uc; bv = vc; bestD2 = dc; }
    }
    // fp64 Newton polish on the (gᵤ,gᵥ)=0 orthogonality condition (only improves).
    std::tie(bu, bv) = polishSurfaceParam(s, target, bu, bv, u0, u1, v0, v1);
    bestD2 = sq3(s(bu, bv), target);
    const Point3 p = s(bu, bv);
    const bool onB = (bu - u0) <= boundaryTol * uSpan || (u1 - bu) <= boundaryTol * uSpan ||
                     (bv - v0) <= boundaryTol * vSpan || (v1 - bv) <= boundaryTol * vSpan;
    out.extrema.push_back({bu, bv, p, std::sqrt(bestD2), onB});
  }

  // (4) dedup collapsed extrema, sort nearest-first, publish the global nearest.
  std::sort(out.extrema.begin(), out.extrema.end(),
            [](const SurfaceExtremum& a, const SurfaceExtremum& b) { return a.distance < b.distance; });
  const double mu = 1e-7 * uSpan, mv = 1e-7 * vSpan;
  std::vector<SurfaceExtremum> uniq;
  for (const SurfaceExtremum& e : out.extrema) {
    bool dup = false;
    for (const SurfaceExtremum& u : uniq)
      if (std::fabs(u.u - e.u) <= mu && std::fabs(u.v - e.v) <= mv) { dup = true; break; }
    if (!dup) uniq.push_back(e);
  }
  out.extrema = std::move(uniq);
  const SurfaceExtremum& g = out.extrema.front();
  out.success = true; out.u = g.u; out.v = g.v; out.point = g.point;
  out.distance = g.distance; out.onBoundary = g.onBoundary;
  return out;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// PUBLIC: closest-point on a native CURVE.
//
// Overloaded per native curve kind so the caller passes typed geometry directly
// (no hand-written evaluator lambda). Every overload funnels into the same
// multi-start Extrema loop.
// ─────────────────────────────────────────────────────────────────────────────

/// Nearest point on a generic parametric curve (any callable t → Point3). This is
/// the escape hatch for a geometry with no dedicated overload.
template <class CurveFn>
CurveClosest project_point_to_curve(const Point3& target, const CurveFn& c,
                                    double t0, double t1,
                                    int samples = kDefaultCurveSamples,
                                    double endpointTol = 1e-7) {
  return detail::projectCurveEval(c, t0, t1, target, samples, endpointTol);
}

/// Nearest point on a Bézier curve (t ∈ [0,1]).
inline CurveClosest project_point_to_curve(
    const Point3& target, std::span<const cybercad::native::math::Point3> poles,
    int samples = kDefaultCurveSamples) {
  return detail::projectCurveEval(
      [&](double t) { return cybercad::native::math::bezierPoint(poles, t); },
      0.0, 1.0, target, samples, 1e-7);
}

/// Nearest point on a B-spline curve (t over the knot vector's clamped domain).
inline CurveClosest project_point_to_bspline_curve(
    const Point3& target, int degree,
    std::span<const cybercad::native::math::Point3> poles,
    std::span<const double> knots, int samples = kDefaultCurveSamples) {
  const double t0 = knots[static_cast<std::size_t>(degree)];
  const double t1 = knots[knots.size() - 1 - static_cast<std::size_t>(degree)];
  return detail::projectCurveEval(
      [&](double t) { return cybercad::native::math::curvePoint(degree, poles, knots, t); },
      t0, t1, target, samples, 1e-7);
}

/// Nearest point on a rational NURBS curve.
inline CurveClosest project_point_to_nurbs_curve(
    const Point3& target, int degree,
    std::span<const cybercad::native::math::Point3> poles,
    std::span<const double> weights, std::span<const double> knots,
    int samples = kDefaultCurveSamples) {
  const double t0 = knots[static_cast<std::size_t>(degree)];
  const double t1 = knots[knots.size() - 1 - static_cast<std::size_t>(degree)];
  return detail::projectCurveEval(
      [&](double t) {
        return cybercad::native::math::nurbsCurvePoint(degree, poles, weights, knots, t);
      },
      t0, t1, target, samples, 1e-7);
}

// ─────────────────────────────────────────────────────────────────────────────
// PUBLIC: closest-point on a native SURFACE.
//
// One overload per native surface kind. Elementary surfaces (plane / cylinder /
// cone / sphere / torus) carry their own value(u,v); the parametric surfaces
// (Bézier / B-spline / NURBS) take poles/knots. Every overload runs the same
// multi-start grid Extrema loop.
//
// Domain note: an elementary surface is periodic/unbounded in one or both
// parameters, so the caller supplies the (u,v) range to search over (e.g.
// [0,2π]×[vmin,vmax] for a cylinder). The parametric-surface overloads default to
// the natural clamped B-spline / [0,1]² Bézier domain.
// ─────────────────────────────────────────────────────────────────────────────

/// Nearest point on a generic parametric surface (callable (u,v) → Point3).
template <class SurfFn>
SurfaceClosest project_point_to_surface(const Point3& target, const SurfFn& s,
                                        double u0, double u1, double v0, double v1,
                                        int su = kDefaultSurfaceSamplesU,
                                        int sv = kDefaultSurfaceSamplesV,
                                        double boundaryTol = 1e-7) {
  return detail::projectSurfaceEval(s, u0, u1, v0, v1, target, su, sv, boundaryTol);
}

/// Nearest point on a Plane over the given (u,v) window.
inline SurfaceClosest project_point_to_surface(
    const Point3& target, const cybercad::native::math::Plane& pl,
    double u0, double u1, double v0, double v1,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval([&](double u, double v) { return pl.value(u, v); },
                                    u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a Cylinder over the given (u,v) window (u angular, v axial).
inline SurfaceClosest project_point_to_surface(
    const Point3& target, const cybercad::native::math::Cylinder& cy,
    double u0, double u1, double v0, double v1,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval([&](double u, double v) { return cy.value(u, v); },
                                    u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a Cone over the given (u,v) window.
inline SurfaceClosest project_point_to_surface(
    const Point3& target, const cybercad::native::math::Cone& co,
    double u0, double u1, double v0, double v1,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval([&](double u, double v) { return co.value(u, v); },
                                    u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a Sphere over the given (u,v) window (u longitude, v latitude).
inline SurfaceClosest project_point_to_surface(
    const Point3& target, const cybercad::native::math::Sphere& sp,
    double u0, double u1, double v0, double v1,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval([&](double u, double v) { return sp.value(u, v); },
                                    u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a Torus over the given (u,v) window (u major, v minor angle).
inline SurfaceClosest project_point_to_surface(
    const Point3& target, const cybercad::native::math::Torus& to,
    double u0, double u1, double v0, double v1,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval([&](double u, double v) { return to.value(u, v); },
                                    u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a Bézier surface (u,v ∈ [0,1]²).
inline SurfaceClosest project_point_to_bezier_surface(
    const Point3& target, std::span<const cybercad::native::math::Point3> poles,
    int nRows, int nCols, int su = kDefaultSurfaceSamplesU,
    int sv = kDefaultSurfaceSamplesV) {
  return detail::projectSurfaceEval(
      [&](double u, double v) {
        return cybercad::native::math::bezierSurfacePoint(poles, nRows, nCols, u, v);
      },
      0.0, 1.0, 0.0, 1.0, target, su, sv, 1e-7);
}

/// Nearest point on a B-spline surface (clamped domain from the knot vectors).
inline SurfaceClosest project_point_to_bspline_surface(
    const Point3& target, int degU, int degV,
    const cybercad::native::math::SurfaceGrid& grid,
    std::span<const double> knotsU, std::span<const double> knotsV,
    int su = kDefaultSurfaceSamplesU, int sv = kDefaultSurfaceSamplesV) {
  const double u0 = knotsU[static_cast<std::size_t>(degU)];
  const double u1 = knotsU[knotsU.size() - 1 - static_cast<std::size_t>(degU)];
  const double v0 = knotsV[static_cast<std::size_t>(degV)];
  const double v1 = knotsV[knotsV.size() - 1 - static_cast<std::size_t>(degV)];
  return detail::projectSurfaceEval(
      [&](double u, double v) {
        return cybercad::native::math::surfacePoint(degU, degV, grid, knotsU, knotsV, u, v);
      },
      u0, u1, v0, v1, target, su, sv, 1e-7);
}

/// Nearest point on a rational NURBS surface (clamped domain from the knots).
inline SurfaceClosest project_point_to_nurbs_surface(
    const Point3& target, int degU, int degV,
    const cybercad::native::math::SurfaceGrid& grid,
    std::span<const double> weights, std::span<const double> knotsU,
    std::span<const double> knotsV, int su = kDefaultSurfaceSamplesU,
    int sv = kDefaultSurfaceSamplesV) {
  const double u0 = knotsU[static_cast<std::size_t>(degU)];
  const double u1 = knotsU[knotsU.size() - 1 - static_cast<std::size_t>(degU)];
  const double v0 = knotsV[static_cast<std::size_t>(degV)];
  const double v1 = knotsV[knotsV.size() - 1 - static_cast<std::size_t>(degV)];
  return detail::projectSurfaceEval(
      [&](double u, double v) {
        return cybercad::native::math::nurbsSurfacePoint(degU, degV, grid, weights,
                                                         knotsU, knotsV, u, v);
      },
      u0, u1, v0, v1, target, su, sv, 1e-7);
}

}  // namespace cybercad::native::numerics

#endif  // CYBERCAD_NATIVE_NUMERICS_CLOSEST_POINT_H
