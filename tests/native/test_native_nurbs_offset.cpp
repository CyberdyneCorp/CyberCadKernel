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

// A RATIONAL NURBS SPHERE BAND of radius R: revolve a meridian arc (equator up to 60°
// latitude) through 90° of azimuth about the z-axis — the exact tensor product of two
// rational Bézier arcs, bounded AWAY from the poles so the normal is defined everywhere (a
// full octant would collapse to a pole singularity with a null normal). U is the 90° azimuth
// arc (weights 1, cos45°, 1); V is the 60° meridian arc (weights 1, cos30°, 1). Every point
// lies exactly on the sphere of radius R (verified in the test).
static BsplineSurfaceData nurbsSphereBand(double R) {
  BsplineSurfaceData s;
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.knotsU = {0, 0, 0, 1, 1, 1};  // single 90° azimuth Bézier arc
  s.knotsV = {0, 0, 0, 1, 1, 1};  // single 60° meridian Bézier arc
  const double c45 = std::cos(M_PI / 4.0);
  const double c30 = std::cos(M_PI / 6.0);
  // Meridian arc in the xz-plane (azimuth 0): equator (R,0,0) up to 60° latitude
  // (R·cos60, 0, R·sin60). Rational quarter-arc middle control on the 30° bisector at
  // radius R/cos30, weight cos30 (standard rational-arc construction).
  const double mer[3][3] = {
      {R, 0.0, 0.0},
      {(R / c30) * std::cos(M_PI / 6.0), 0.0, (R / c30) * std::sin(M_PI / 6.0)},
      {R * std::cos(M_PI / 3.0), 0.0, R * std::sin(M_PI / 3.0)}};
  const double wv[3] = {1.0, c30, 1.0};
  const double wu[3] = {1.0, c45, 1.0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const double rho = mer[j][0];  // meridian radius in the xy-plane (my == 0)
      const double mz = mer[j][2];
      double px = 0.0, py = 0.0;  // revolve the meridian point through the 90° azimuth arc
      if (i == 0) { px = rho; py = 0.0; }
      else if (i == 1) { px = rho; py = rho; }
      else { px = 0.0; py = rho; }
      s.poles.push_back({px, py, mz});
      s.weights.push_back(wu[i] * wv[j]);
    }
  return s;
}

// A partial-fold bump: a bicubic patch that is nearly flat over most of the domain but has a
// sharply-curved dimple in ONE corner, so an offset by a modest |d| folds ONLY over that
// corner — the rest is fold-free. Used to exercise fold TRIMMING (keep the fold-free bulk).
static BsplineSurfaceData partialFoldBump() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 7;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};  // 7+3+1=11
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35;
      const double y = j * 0.35;
      // A tall, tight Gaussian dimple centred at the (i=1,j=1) corner region — high curvature
      // there, flat elsewhere. Amplitude/width tuned so a modest offset folds only locally.
      const double dx = i - 1.0, dy = j - 1.0;
      const double z = 0.9 * std::exp(-1.4 * (dx * dx + dy * dy));
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A CENTRAL-RIDGE bump: a tall narrow Gaussian ridge running as a BAND across the middle in
// u (peaked at the u-index-3 row, flat toward u=0 and u=1), spanning the whole v. An offset
// toward the ridge's centre of curvature folds ONLY the central band, leaving TWO large
// fold-free rectangles at low-u and high-u. Used to exercise MULTI-region fold trimming: the
// single-rectangle offsetSurfaceTrimmed keeps only one side; offsetSurfaceMultiTrimmed must
// recover BOTH.
static BsplineSurfaceData centralRidgeBump() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 7;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35, y = j * 0.35;
      const double du = i - 3.0;                       // ridge centred at the mid-u row
      const double z = 0.9 * std::exp(-1.4 * du * du);  // narrow ridge, invariant in v
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

  // ═══ 5. RATIONAL OFFSET — analytic exactness (sphere / cylinder / plane) ═════════
  {
    // 5a. Sphere: the RATIONAL offset of a NURBS sphere octant of radius R by d lies on the
    // concentric sphere of radius R±d, to a TIGHT bound (≤ 1e-6), and is itself rational.
    const double R = 2.0;
    const BsplineSurfaceData sph = nurbsSphereBand(R);
    // Sanity: the input band is a true sphere (every point at radius R).
    {
      double worst0 = 0.0;
      for (int i = 0; i <= 6; ++i)
        for (int j = 0; j <= 6; ++j) {
          const Point3 p = evalSurf(sph, i / 6.0, j / 6.0);
          worst0 = std::max(worst0, std::fabs(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - R));
        }
      expectLE(worst0, 1e-9, "rational sphere octant input lies on radius R");
    }
    for (double d : {0.5, -0.3}) {
      const OffsetResult res = offsetSurfaceRational(sph, d, 1e-6);
      expectTrue(res.ok, "rational sphere offset ok");
      if (!res.ok) continue;
      expectTrue(!res.surface.weights.empty(), "rational sphere offset is RATIONAL (weights)");
      const BsplineSurfaceData& O = res.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      const double want = R + d;
      double worst = 0.0;
      const int N = 11;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
          const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
          const Point3 p = evalSurf(O, u, v);
          worst = std::max(worst, std::fabs(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - want));
        }
      expectLE(worst, 1e-6, "rational sphere offset on radius R+d (tight)");
    }

    // 5b. Cylinder: the rational offset of a NURBS cylinder of radius r lies on radius r±d,
    // to ≤ 1e-6, and is rational.
    const double r = 2.0, h = 3.0;
    const BsplineSurfaceData cyl = nurbsQuarterCylinder(r, h);
    for (double d : {0.5, -0.4}) {
      const OffsetResult res = offsetSurfaceRational(cyl, d, 1e-6);
      expectTrue(res.ok, "rational cylinder offset ok");
      if (!res.ok) continue;
      expectTrue(!res.surface.weights.empty(), "rational cylinder offset is RATIONAL");
      const BsplineSurfaceData& O = res.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      const double want = r + d;
      double worst = 0.0;
      const int N = 11;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
          const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
          const Point3 p = evalSurf(O, u, v);
          worst = std::max(worst, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - want));
        }
      expectLE(worst, 1e-6, "rational cylinder offset on radius r+d (tight)");
    }

    // 5c. Plane: rational path on a non-rational plane falls back and offsets exactly.
    const BsplineSurfaceData plane = planarPatch();
    const OffsetResult pres = offsetSurfaceRational(plane, 0.75, 1e-6);
    expectTrue(pres.ok, "rational-path plane offset ok");
    if (pres.ok) {
      const BsplineSurfaceData& O = pres.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      double worst = 0.0;
      for (int i = 0; i < 9; ++i)
        for (int j = 0; j < 9; ++j) {
          const Point3 p =
              evalSurf(O, u0 + (u1 - u0) * (i / 8.0), v0 + (v1 - v0) * (j / 8.0));
          worst = std::max(worst, std::fabs(p.z - 2.75));
        }
      expectLE(worst, 1e-9, "rational-path plane offsets to exact parallel plane");
    }
  }

  // ═══ 6. RATIONAL ROUND-TRIP — offset by d then −d recovers S ═════════════════════
  {
    const double r = 2.0, h = 3.0;
    const BsplineSurfaceData cyl = nurbsQuarterCylinder(r, h);
    const double d = 0.5;
    const OffsetResult fwd = offsetSurfaceRational(cyl, d, 1e-6);
    expectTrue(fwd.ok, "round-trip forward offset ok");
    if (fwd.ok) {
      const OffsetResult back = offsetSurfaceRational(fwd.surface, -d, 1e-6);
      expectTrue(back.ok, "round-trip return offset ok");
      if (back.ok) {
        // Compare the twice-offset surface to the original cylinder by radius: it must be back
        // on radius r everywhere (parametrization-independent geometric recovery).
        const BsplineSurfaceData& O = back.surface;
        const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
        const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
        double worst = 0.0;
        const int N = 11;
        for (int i = 0; i < N; ++i)
          for (int j = 0; j < N; ++j) {
            const double u = u0 + (u1 - u0) * (i / (double)(N - 1));
            const double v = v0 + (v1 - v0) * (j / (double)(N - 1));
            const Point3 p = evalSurf(O, u, v);
            worst = std::max(worst, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - r));
          }
        expectLE(worst, 1e-5, "rational offset round-trips to original cylinder radius");
      }
    }
  }

  // ═══ 7. FOLD TRIMMING — keep the maximal fold-free region ════════════════════════
  {
    // A patch that folds only in one corner. Trimming must return a VALID offset over the
    // fold-free bulk (trimmed=true, strict sub-rectangle), constant-sign Jacobian over the
    // kept region, every kept point at distance |d| from S.
    const BsplineSurfaceData S = partialFoldBump();
    const double d = 0.8;  // toward the dimple's centre of curvature → fold over ONE corner

    // Baseline: the plain offset DECLINES this as a self-intersection (fold somewhere).
    const OffsetResult plain = offsetSurface(S, d, 1e-3);
    expectTrue(plain.status == OffsetStatus::SelfIntersection,
               "partial-fold patch: plain offset declines as self-intersection");

    // Trimmed: recovers a valid offset over the fold-free region.
    const OffsetResult tr = offsetSurfaceTrimmed(S, d, 1e-3);
    expectTrue(tr.ok, "fold-trim recovers a valid offset over the fold-free region");
    if (tr.ok) {
      expectTrue(tr.trimmed, "fold-trim reports the domain was trimmed");
      const double su0 = domLo(S.knotsU, S.degreeU), su1 = domHi(S.knotsU, S.degreeU);
      const double sv0 = domLo(S.knotsV, S.degreeV), sv1 = domHi(S.knotsV, S.degreeV);
      // Kept region is a STRICT sub-rectangle of the full domain (something was trimmed off).
      const bool strictSub = (tr.keptU0 > su0 + 1e-9) || (tr.keptU1 < su1 - 1e-9) ||
                             (tr.keptV0 > sv0 + 1e-9) || (tr.keptV1 < sv1 - 1e-9);
      expectTrue(strictSub, "kept region is a strict sub-rectangle (fold corner removed)");
      expectTrue(tr.keptU1 > tr.keptU0 && tr.keptV1 > tr.keptV0, "kept rectangle is non-empty");

      // FOLD-FREE over the kept region: sample the input's principal curvatures on a dense
      // grid across [keptU0,keptU1]×[keptV0,keptV1] and require (1 + d·κ) to keep CONSTANT
      // positive sign — the offset Jacobian never degenerates there.
      SurfaceGrid sg{std::span<const Point3>(S.poles), S.nPolesU, S.nPolesV};
      double worstFactor = std::numeric_limits<double>::infinity();
      const int N = 15;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = tr.keptU0 + (tr.keptU1 - tr.keptU0) * (i / (double)(N - 1));
          const double v = tr.keptV0 + (tr.keptV1 - tr.keptV0) * (j / (double)(N - 1));
          // Principal curvatures via the second fundamental form (mirrors the module).
          Vec3 dbuf[9];
          surfaceDerivs(S.degreeU, S.degreeV, sg, S.knotsU, S.knotsV, u, v, 2,
                        std::span<Vec3>(dbuf, 9));
          const Vec3 Su = dbuf[3], Sv = dbuf[1], Suu = dbuf[6], Suv = dbuf[4], Svv = dbuf[2];
          const Vec3 nRaw = cross(Su, Sv);
          const double nLen = norm(nRaw);
          if (nLen < 1e-12) continue;
          const Vec3 nn = nRaw / nLen;
          const double E = dot(Su, Su), F = dot(Su, Sv), G = dot(Sv, Sv);
          const double det1 = E * G - F * F;
          if (det1 < 1e-14) continue;
          const double L = dot(Suu, nn), M = dot(Suv, nn), Nn = dot(Svv, nn);
          const double K = (L * Nn - M * M) / det1;
          const double H = (E * Nn - 2.0 * F * M + G * L) / (2.0 * det1);
          double disc = H * H - K;
          if (disc < 0.0) disc = 0.0;
          const double root = std::sqrt(disc);
          for (double k : {H + root, H - root})
            worstFactor = std::min(worstFactor, 1.0 + d * k);
        }
      expectTrue(worstFactor > 0.0, "kept region is fold-free (Jacobian factor stays positive)");

      // Every kept offset point is at distance |d| from S (offset-locus property). Project onto
      // S restricted to the KEPT sub-domain so the nearest foot is the radial offset foot, not a
      // patch-boundary point (a bounded-patch artifact if we projected over the full domain).
      const BsplineSurfaceData& O = tr.surface;
      const double u0 = domLo(O.knotsU, O.degreeU), u1 = domHi(O.knotsU, O.degreeU);
      const double v0 = domLo(O.knotsV, O.degreeV), v1 = domHi(O.knotsV, O.degreeV);
      auto Seval = [&](double u, double v) { return evalSurf(S, u, v); };
      double worstDist = 0.0;
      const int M = 7;
      for (int i = 1; i < M - 1; ++i)  // interior cells only (avoid the kept-edge feet)
        for (int j = 1; j < M - 1; ++j) {
          const Point3 p = evalSurf(O, u0 + (u1 - u0) * (i / (double)(M - 1)),
                                    v0 + (v1 - v0) * (j / (double)(M - 1)));
          const num::SurfaceProjection pr = num::closest_point_on_surface(
              Seval, tr.keptU0, tr.keptU1, tr.keptV0, tr.keptV1, p, 32, 32);
          if (pr.success) worstDist = std::max(worstDist, std::fabs(pr.distance - std::fabs(d)));
        }
      expectLE(worstDist, 5e-3, "kept offset points lie at distance |d| from S");
    }

    // A fold-free trimmed offset (small |d|) reports trimmed=false + full kept domain.
    const OffsetResult noTrim = offsetSurfaceTrimmed(S, 0.05, 1e-3);
    expectTrue(noTrim.ok, "small offset via trimmed path succeeds");
    if (noTrim.ok) expectTrue(!noTrim.trimmed, "fold-free offset reports trimmed=false");

    // A tight dome folds over essentially the WHOLE domain on its CONCAVE (+normal) side (the
    // paraboloid cap curves toward +N everywhere): the trimmed path finds no fold-free region
    // of meaningful area and still honest-declines with SelfIntersection.
    const BsplineSurfaceData dome = tightDome(0.5);
    const OffsetResult allFold = offsetSurfaceTrimmed(dome, 1.5 * 0.5, 1e-3);
    expectTrue(!allFold.ok && allFold.status == OffsetStatus::SelfIntersection,
               "fully-folding offset still declines (no meaningful fold-free region)");
    expectTrue(allFold.surface.poles.empty(), "fully-folding decline returns no surface");
  }

  // ═══ 8. MULTI-REGION FOLD TRIMMING — recover BOTH sides of a fold band ═══════════
  {
    // A central ridge whose offset folds only the middle band, splitting the fold-free
    // parameter space into TWO large rectangles (low-u and high-u). The single-rectangle
    // offsetSurfaceTrimmed keeps ONLY one side; offsetSurfaceMultiTrimmed must recover BOTH.
    const BsplineSurfaceData S = centralRidgeBump();
    const double d = 0.6;  // toward the ridge's centre of curvature → fold the central band

    // Baseline: the plain offset declines (fold somewhere); the single trim keeps ONE side.
    const OffsetResult plain = offsetSurface(S, d, 1e-3);
    expectTrue(plain.status == OffsetStatus::SelfIntersection,
               "central-ridge: plain offset declines as self-intersection");
    const OffsetResult single = offsetSurfaceTrimmed(S, d, 1e-3);
    expectTrue(single.ok && single.trimmed, "central-ridge: single trim recovers one side");

    // Multi-trim: recovers BOTH fold-free rectangles.
    const std::vector<OffsetResult> multi = offsetSurfaceMultiTrimmed(S, d, 1e-3);
    expectTrue(multi.size() >= 2, "central-ridge: multi-trim recovers >= 2 fold-free regions");

    const double su0 = domLo(S.knotsU, S.degreeU), su1 = domHi(S.knotsU, S.degreeU);
    const double sv0 = domLo(S.knotsV, S.degreeV), sv1 = domHi(S.knotsV, S.degreeV);
    SurfaceGrid sg{std::span<const Point3>(S.poles), S.nPolesU, S.nPolesV};

    bool sawLowU = false, sawHighU = false;
    double coverAreaMulti = 0.0;
    for (const OffsetResult& r : multi) {
      expectTrue(r.ok && r.trimmed, "central-ridge: each recovered region is a valid trim");
      // Each kept rectangle is a strict sub-rectangle inside the domain.
      expectTrue(r.keptU1 > r.keptU0 && r.keptV1 > r.keptV0,
                 "central-ridge: recovered rectangle is non-empty");
      coverAreaMulti += (r.keptU1 - r.keptU0) * (r.keptV1 - r.keptV0);
      const double midU = 0.5 * (r.keptU0 + r.keptU1);
      if (midU < 0.5) sawLowU = true;
      if (midU > 0.5) sawHighU = true;

      // FOLD-FREE over the whole recovered rectangle: (1 + d·κ) stays strictly positive.
      double worstFactor = std::numeric_limits<double>::infinity();
      const int N = 13;
      for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
          const double u = r.keptU0 + (r.keptU1 - r.keptU0) * (i / (double)(N - 1));
          const double v = r.keptV0 + (r.keptV1 - r.keptV0) * (j / (double)(N - 1));
          Vec3 dbuf[9];
          surfaceDerivs(S.degreeU, S.degreeV, sg, S.knotsU, S.knotsV, u, v, 2,
                        std::span<Vec3>(dbuf, 9));
          const Vec3 Su = dbuf[3], Sv = dbuf[1], Suu = dbuf[6], Suv = dbuf[4], Svv = dbuf[2];
          const Vec3 nRaw = cross(Su, Sv);
          const double nLen = norm(nRaw);
          if (nLen < 1e-12) continue;
          const Vec3 nn = nRaw / nLen;
          const double E = dot(Su, Su), F = dot(Su, Sv), G = dot(Sv, Sv);
          const double det1 = E * G - F * F;
          if (det1 < 1e-14) continue;
          const double L = dot(Suu, nn), M = dot(Suv, nn), Nn = dot(Svv, nn);
          const double K = (L * Nn - M * M) / det1;
          const double H = (E * Nn - 2.0 * F * M + G * L) / (2.0 * det1);
          double disc = H * H - K;
          if (disc < 0.0) disc = 0.0;
          const double root = std::sqrt(disc);
          for (double k : {H + root, H - root})
            worstFactor = std::min(worstFactor, 1.0 + d * k);
        }
      expectTrue(worstFactor > 0.0,
                 "central-ridge: recovered region is fold-free (Jacobian factor positive)");
    }
    expectTrue(sawLowU && sawHighU,
               "central-ridge: multi-trim recovers BOTH the low-u and high-u fold-free sides");

    // The multi-region cover recovers strictly MORE material than the single largest rectangle
    // (the whole point: the single trim silently drops the mirror side).
    const double singleArea = (single.keptU1 - single.keptU0) * (single.keptV1 - single.keptV0);
    expectTrue(coverAreaMulti > singleArea + 1e-6,
               "central-ridge: multi-trim recovers more area than single-rectangle trim");

    // Pairwise NON-OVERLAP of the recovered rectangles (disjoint fold-free regions).
    for (std::size_t a = 0; a < multi.size(); ++a)
      for (std::size_t b = a + 1; b < multi.size(); ++b) {
        const OffsetResult& A = multi[a];
        const OffsetResult& B = multi[b];
        const bool disjoint = A.keptU1 <= B.keptU0 + 1e-9 || B.keptU1 <= A.keptU0 + 1e-9 ||
                              A.keptV1 <= B.keptV0 + 1e-9 || B.keptV1 <= A.keptV0 + 1e-9;
        expectTrue(disjoint, "central-ridge: recovered rectangles are pairwise disjoint");
      }

    // PASSTHROUGH: a small fold-free offset yields a SINGLE full-domain region (trimmed=false).
    const std::vector<OffsetResult> gentle = offsetSurfaceMultiTrimmed(S, 0.03, 1e-3);
    expectTrue(gentle.size() == 1 && gentle[0].ok && !gentle[0].trimmed,
               "central-ridge: fold-free offset returns one full-domain region");
    if (gentle.size() == 1) {
      const bool full = std::fabs(gentle[0].keptU0 - su0) < 1e-9 &&
                        std::fabs(gentle[0].keptU1 - su1) < 1e-9 &&
                        std::fabs(gentle[0].keptV0 - sv0) < 1e-9 &&
                        std::fabs(gentle[0].keptV1 - sv1) < 1e-9;
      expectTrue(full, "central-ridge: passthrough region is the full domain");
    }

    // FULLY-FOLDING dome: no meaningful fold-free region → EMPTY vector (honest-decline).
    const BsplineSurfaceData dome = tightDome(0.5);
    const std::vector<OffsetResult> allFold = offsetSurfaceMultiTrimmed(dome, 1.5 * 0.5, 1e-3);
    expectTrue(allFold.empty(),
               "central-ridge: fully-folding offset returns empty (never a folded region)");
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
