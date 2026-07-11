// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 5 — surface OFFSET
// (src/native/math/bspline_offset.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form — the offset locus is the *sampled* S + d·N, and for the cylinder/plane
// the offset has an exact analytic form:
//
//   1. OFFSET DISTANCE — every point of the fitted offset surface lies at distance
//      ≈ |d| from S along S's normal. For a dense grid on the fitted surface we project
//      each point onto S (numerics::closest_point_on_surface) and assert the distance is
//      |d| within the reported fit tolerance. Curved bicubic-bump patch.
//   2. ANALYTIC CYLINDER — offsetting a NURBS-represented cylinder of radius r by d
//      yields a surface whose points lie on the coaxial cylinder of radius r+d
//      (outward) / r−d (inward), to the fitting tolerance. A PLANE offsets to a
//      parallel plane exactly (the offset of a flat patch is flat, translated by d·n̂).
//   3. ERROR CONVERGENCE — the reported max offset error decreases (monotone, within a
//      small fp slack) as the fit sample grid refines. The error is the ACHIEVED
//      deviation from the true offset locus, never widened.
//   4. SELF-INTERSECTION GUARD — offsetting a tightly-curved patch by d greater than the
//      patch's minimum principal radius of curvature is DECLINED (status
//      SelfIntersection, ok=false, empty surface), NOT returned folded. A degenerate
//      (near-zero-normal) patch declines. A safe (small-|d|) offset of the same patch
//      succeeds — the guard is not merely rejecting everything.
//
// The routines are numsci-gated (offsetSurface fits through numerics::lin_solve and the
// distance oracle uses numerics::closest_point_on_surface), so the whole gate is under
// CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). With the guard OFF this compiles to
// a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_offset.h"
#include "native/math/bspline_ops.h"
#include "native/numerics/numerics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

using namespace cybercad::native::math;
namespace num = cybercad::native::numerics;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalSurf(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}
static double domLo(const std::vector<double>& k, int p) { return k[p]; }
static double domHi(const std::vector<double>& k, int p) { return k[k.size() - 1 - p]; }

// ── Test surfaces ────────────────────────────────────────────────────────────────

// A non-rational bicubic BUMP patch: a gentle sin/cos height field over a 6×6 net.
// Gently curved so any modest offset is fold-free (regular offset region).
static BsplineSurfaceData bicubicBump() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 6;
  s.nPolesV = 6;
  s.knotsU = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};  // 6+3+1=10
  s.knotsV = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      const double x = i * 0.4;
      const double y = j * 0.4;
      const double z = 0.35 * std::sin(0.9 * i) * std::cos(0.8 * j);
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A planar bicubic patch in the z=2 plane (control net all at z=2). The exact offset
// by d along +z is the z=2+d plane.
static BsplineSurfaceData planarPatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 5;
  s.nPolesV = 5;
  s.knotsU = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};  // 5+3+1=9
  s.knotsV = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j)
      s.poles.push_back({i * 0.6, j * 0.7, 2.0});
  return s;
}

// A RATIONAL NURBS quarter-cylinder of radius r about the z-axis, spanning 90° in the
// U (angular) direction and height h in V. Exact NURBS circle: 3 poles, weights
// {1, cos45, 1} with the middle pole pushed out to r/cos45 (the standard 90° arc).
static BsplineSurfaceData nurbsQuarterCylinder(double r, double h) {
  BsplineSurfaceData s;
  s.degreeU = 2;  // conic arc
  s.degreeV = 1;  // straight in height
  s.nPolesU = 3;
  s.nPolesV = 2;
  s.knotsU = {0, 0, 0, 1, 1, 1};  // 3+2+1=6, single 90° Bézier arc
  s.knotsV = {0, 0, 1, 1};        // 2+1+1=4, straight segment
  const double c = std::cos(M_PI / 4.0);  // = √2/2
  // Arc poles at z=0 and z=h. Middle pole out at (r, r) with weight cos45.
  // Angular poles: (r,0), (r,r), (0,r) — the standard rational 90° arc.
  const Point3 a0{r, 0, 0}, a1{r, r, 0}, a2{0, r, 0};
  const Point3 b0{r, 0, h}, b1{r, r, h}, b2{0, r, h};
  // Layout is pole(i,j)=poles[i*nPolesV+j], i over U (arc 0..2), j over V (height 0..1):
  // [ (arc0,h0),(arc0,h1), (arc1,h0),(arc1,h1), (arc2,h0),(arc2,h1) ].
  s.poles = {a0, b0, a1, b1, a2, b2};
  s.weights = {1, 1, c, c, 1, 1};
  return s;
}

// A tightly-curved patch: a near-hemispherical bump with a small radius of curvature,
// built as a rational-free polynomial approximation of a dome of radius R. The min
// principal radius of curvature is ≈ R, so an offset by d > R must fold (self-intersect).
static BsplineSurfaceData tightDome(double R) {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 5;
  s.nPolesV = 5;
  s.knotsU = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  // A paraboloid cap z = R − (x²+y²)/(2R) over a small footprint — curvature ≈ 1/R at
  // the apex (radius of curvature ≈ R). Footprint half-width = 0.5*R keeps it a cap.
  const double half = 0.5 * R;
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) {
      const double x = (-half) + (2 * half) * (i / 4.0);
      const double y = (-half) + (2 * half) * (j / 4.0);
      const double z = R - (x * x + y * y) / (2.0 * R);
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A degenerate patch: a control net that collapses to a line along U (all V-columns
// identical points) — the ∂S/∂v tangent is null, so the normal is undefined.
static BsplineSurfaceData degenerateNormalPatch() {
  BsplineSurfaceData s;
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 0, 1, 1, 1};
  // Every V column is the SAME point set (no extent in V) → ∂S/∂v ≡ 0 → null normal.
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      s.poles.push_back({i * 1.0, 0.0, 0.0});  // all rows share y=z=0, no V spread
  return s;
}

int main() {
  // ═══ 1. OFFSET DISTANCE (curved bicubic bump) ═══════════════════════════════════
  {
    const BsplineSurfaceData S = bicubicBump();
    for (double d : {0.15, -0.2}) {
      const OffsetResult res = offsetSurface(S, d, 1e-4);
      expectTrue(res.ok, "bump offset ok");
      if (!res.ok) continue;
      expectLE(res.maxError, 1e-4, "bump reported error within tol");

      // Every point of the fitted offset must lie |d| from S along S's normal.
      // Geometric check: project onto S and require the distance ≈ |d|.
      const BsplineSurfaceData& O = res.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      const double su0 = domLo(S.knotsU, S.degreeU), su1 = domHi(S.knotsU, S.degreeU);
      const double sv0 = domLo(S.knotsV, S.degreeV), sv1 = domHi(S.knotsV, S.degreeV);
      auto Seval = [&](double u, double v) { return evalSurf(S, u, v); };
      double worst = 0.0;
      const int N = 9;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
          const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
          const Point3 p = evalSurf(O, u, v);
          const num::SurfaceProjection pr =
              num::closest_point_on_surface(Seval, su0, su1, sv0, sv1, p, 24, 24);
          expectTrue(pr.success, "bump projection onto S succeeds");
          worst = std::max(worst, std::fabs(pr.distance - std::fabs(d)));
        }
      // The nearest-point distance to S equals |d| within the fit tolerance plus a
      // small projection-grid slack.
      expectLE(worst, 1e-3, "bump every offset point at distance |d| from S");
    }
  }

  // ═══ 2a. ANALYTIC CYLINDER ══════════════════════════════════════════════════════
  {
    const double r = 2.0, h = 3.0;
    const BsplineSurfaceData cyl = nurbsQuarterCylinder(r, h);
    for (double d : {0.5, -0.4}) {
      const OffsetResult res = offsetSurface(cyl, d, 1e-4);
      expectTrue(res.ok, "cylinder offset ok");
      if (!res.ok) continue;
      const BsplineSurfaceData& O = res.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      // The outward normal of a cylinder points radially; offset radius is r + d.
      const double want = r + d;
      double worst = 0.0;
      const int N = 11;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
          const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
          const Point3 p = evalSurf(O, u, v);
          const double radius = std::sqrt(p.x * p.x + p.y * p.y);  // dist to z-axis
          worst = std::max(worst, std::fabs(radius - want));
        }
      expectLE(worst, 2e-3, "cylinder offset lies on radius r+d");
    }
  }

  // ═══ 2b. ANALYTIC PLANE (exact parallel) ════════════════════════════════════════
  {
    const BsplineSurfaceData plane = planarPatch();
    const double d = 0.75;
    const OffsetResult res = offsetSurface(plane, d, 1e-6);
    expectTrue(res.ok, "plane offset ok");
    if (res.ok) {
      const BsplineSurfaceData& O = res.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      // Normal of the z=2 plane is +z, so the offset is the exact z=2+d plane.
      double worst = 0.0;
      const int N = 9;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
          const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
          const Point3 p = evalSurf(O, u, v);
          worst = std::max(worst, std::fabs(p.z - (2.0 + d)));
        }
      expectLE(worst, 1e-9, "plane offsets to exact parallel plane");
      expectLE(res.maxError, 1e-9, "plane offset error ~0 (flat is exactly fittable)");
    }
  }

  // ═══ 3. ERROR CONVERGENCE (monotone under grid refinement) ══════════════════════
  {
    const BsplineSurfaceData S = bicubicBump();
    const double d = 0.25;
    // Force distinct start/max grids so refinement actually happens; the module
    // reports the BEST (finest usable) achieved error. Compare coarse vs fine budgets.
    double prev = std::numeric_limits<double>::infinity();
    bool monotone = true;
    double lastErr = 0.0;
    for (int cap : {5, 9, 17, 33}) {
      const OffsetResult res = offsetSurface(S, d, /*tol=*/1e-12, /*startGrid=*/5,
                                             /*maxGrid=*/cap);
      // Even if tol is never met, a surface + honest error is reported.
      expectTrue(!res.surface.poles.empty(), "convergence: a fit is produced");
      // The finer budget must not do WORSE than the coarser (allow tiny fp slack).
      if (res.maxError > prev + 1e-9) monotone = false;
      prev = res.maxError;
      lastErr = res.maxError;
    }
    expectTrue(monotone, "offset error decreases monotonically as grid refines");
    // The finest budget drives the error well down from the coarse baseline.
    const OffsetResult coarse = offsetSurface(S, d, 1e-12, 5, 5);
    expectLE(lastErr, coarse.maxError + 1e-12, "finest budget ≤ coarsest error");
    expectLE(lastErr, 1e-3, "finest offset error is small");
  }

  // ═══ 4. SELF-INTERSECTION GUARD ═════════════════════════════════════════════════
  {
    const double R = 0.5;  // dome radius of curvature ≈ 0.5
    const BsplineSurfaceData dome = tightDome(R);

    // Offset INWARD (toward the center of curvature) by more than R → fold. The dome
    // opens downward (apex up, normal up), so the concave side is +normal for the cap;
    // try both signs and require at least one to be a declared self-intersection at a
    // large offset, and neither to be silently returned folded.
    const OffsetResult big1 = offsetSurface(dome, 1.5 * R, 1e-3);
    const OffsetResult big2 = offsetSurface(dome, -1.5 * R, 1e-3);
    const bool foldedDeclined =
        (big1.status == OffsetStatus::SelfIntersection && !big1.ok) ||
        (big2.status == OffsetStatus::SelfIntersection && !big2.ok);
    expectTrue(foldedDeclined, "large offset past curvature radius is DECLINED as fold");
    // A folded (self-intersecting) offset must NEVER be returned as a valid surface:
    // if a large-|d| offset was declared a self-intersection it must also be !ok with an
    // empty surface (honest decline, no folded geometry handed back).
    if (big1.status == OffsetStatus::SelfIntersection)
      expectTrue(!big1.ok && big1.surface.poles.empty(),
                 "over-radius offset (sign +) declined with no folded surface");
    if (big2.status == OffsetStatus::SelfIntersection)
      expectTrue(!big2.ok && big2.surface.poles.empty(),
                 "over-radius offset (sign -) declined with no folded surface");

    // Exactly ONE sign folds (the concave side): identify it as the SelfIntersection
    // decline, and require it to report the curvature radius it tripped on — ≈ R (the
    // dome's radius of curvature). This proves the guard is a genuine curvature test, not
    // a blanket rejection. The OTHER sign (convex side) offsets fold-free for any |d|.
    const OffsetResult* fold =
        (big1.status == OffsetStatus::SelfIntersection) ? &big1 : &big2;
    const OffsetResult* freeSide =
        (big1.status == OffsetStatus::SelfIntersection) ? &big2 : &big1;
    expectTrue(fold->status == OffsetStatus::SelfIntersection,
               "the concave-side over-radius offset trips the fold guard");
    expectNear(fold->minCurvatureRadius, R, 0.2 * R,
               "reported min curvature radius ≈ dome radius R");
    // The convex side does NOT fold (no principal radius on that side to exceed).
    expectTrue(freeSide->status != OffsetStatus::SelfIntersection,
               "the convex-side offset is not spuriously declared a fold");

    // A SAFE small offset of the same dome must SUCCEED (the guard is not rejecting all).
    // Offset on the CONVEX (+normal) side, where the offset stays regular for any |d|:
    // it must converge to a valid surface, and the reported curvature radius is a
    // non-negative honest value (0 when the offsetting side is fold-free).
    const double safeD = (fold == &big1) ? -0.1 * R : 0.1 * R;  // the fold-free sign
    const OffsetResult safe = offsetSurface(dome, safeD, 1e-3);
    expectTrue(safe.ok, "small safe offset of the tight dome succeeds");
    expectTrue(safe.minCurvatureRadius >= 0.0, "curvature radius reported (honest, ≥ 0)");

    // Degenerate (null-normal) patch declines with DegenerateNormal, never a crash.
    const BsplineSurfaceData deg = degenerateNormalPatch();
    const OffsetResult degRes = offsetSurface(deg, 0.2, 1e-3);
    expectTrue(!degRes.ok, "degenerate-normal patch declines");
    expectTrue(degRes.status == OffsetStatus::DegenerateNormal ||
                   degRes.status == OffsetStatus::DegenerateInput,
               "degenerate patch reports a degeneracy status");
  }

  std::printf("nurbs_offset: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("nurbs_offset: numsci disabled — trivially passing.\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
