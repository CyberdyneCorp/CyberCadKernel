// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — boundary-filled Coons patch
// (src/native/math/bspline_coons.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form:
//
//   1. BOUNDARY CONTAINMENT (the core oracle) — the Coons surface evaluated along each
//      of its four edges reproduces the corresponding boundary curve POINTWISE on a
//      dense sample to ~1e-9: S(·,0)==c0, S(·,1)==c1, S(0,·)==d0, S(1,·)==d1.
//   2. CORNER INTERPOLATION — the four corners are interpolated exactly (~1e-12).
//   3. FLAT PATCH — a flat boundary (four coplanar edges of a rectangle) yields a flat
//      patch matching the plane exactly (every surface point on the plane, ~1e-12).
//   4. KNOWN-SURFACE ROUND-TRIP — extract the four boundary iso-curves of a KNOWN
//      tensor-product surface and Coons-fill them. For a RULED / bilinear surface
//      (which IS bilinearly-blended) the original is recovered POINTWISE (~1e-9); for a
//      general surface the boundary is reproduced exactly (the Coons interior is the
//      bilinear blend by definition — verified as a containment, not an interior match).
//   5. HONEST DECLINES — mismatched corners, a rational boundary, and a malformed
//      boundary decline (ok=false, with a reason), never a silently-wrong surface.
//
// The module sits with the rest of the numsci-gated Layer-6 surfacing family, so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_gordon). With the
// guard OFF this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_coons.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-52s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}

// Max distance between the surface edge iso-curve and the boundary curve, dense sample.
// dir=0: S(·, edgeParam) vs a u-curve; dir=1: S(edgeParam, ·) vs a v-curve.
static double edgeVsCurveMaxDev(const BsplineSurfaceData& s, int dir, double edgeParam,
                                const BsplineCurveData& c, int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double t = static_cast<double>(i) / nS;
    const Point3 sp = (dir == 0) ? evalSurface(s, t, edgeParam) : evalSurface(s, edgeParam, t);
    worst = std::max(worst, distance(sp, evalCurve(c, t)));
  }
  return worst;
}

// ── Boundary builders ────────────────────────────────────────────────────────────
// A clamped cubic curve on [0,1] with the two endpoints PINNED to given corners, so
// opposing boundaries can share corners exactly. Interior poles get a lateral bow.
static BsplineCurveData cubicEdge(const Point3& a, const Point3& b, const Vec3& bow) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};  // 5 poles
  const Point3 p1{a.x + 0.25 * (b.x - a.x) + bow.x,
                  a.y + 0.25 * (b.y - a.y) + bow.y,
                  a.z + 0.25 * (b.z - a.z) + bow.z};
  const Point3 p2{a.x + 0.50 * (b.x - a.x) + 1.3 * bow.x,
                  a.y + 0.50 * (b.y - a.y) + 1.3 * bow.y,
                  a.z + 0.50 * (b.z - a.z) + 1.3 * bow.z};
  const Point3 p3{a.x + 0.75 * (b.x - a.x) + bow.x,
                  a.y + 0.75 * (b.y - a.y) + bow.y,
                  a.z + 0.75 * (b.z - a.z) + bow.z};
  c.poles = {a, p1, p2, p3, b};
  return c;
}

// A straight degree-1 edge between two corners (2 poles).
static BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0, 0, 1, 1};
  c.poles = {a, b};
  return c;
}

// The four corners of a topological quad.
static const Point3 P00{0, 0, 0};
static const Point3 P10{4, 0, 0};
static const Point3 P01{0, 3, 0};
static const Point3 P11{4, 3, 0};

// A general (curved, non-planar) four-sided boundary with matching corners.
static CoonsBoundary curvedBoundary() {
  CoonsBoundary b;
  b.c0 = cubicEdge(P00, P10, {0, -0.6, 0.8});  // v=0 edge (in u)
  b.c1 = cubicEdge(P01, P11, {0, 0.7, -0.5});  // v=1 edge (in u)
  b.d0 = cubicEdge(P00, P01, {-0.5, 0, 0.6});  // u=0 edge (in v)
  b.d1 = cubicEdge(P10, P11, {0.6, 0, -0.4});  // u=1 edge (in v)
  return b;
}

// A KNOWN ruled (bilinear-in-one-direction) surface: linear interpolation between two
// cubic profile curves in v. This surface IS bilinearly-blended, so Coons of its four
// boundary iso-curves must recover it POINTWISE (the exactness oracle for Coons).
static BsplineSurfaceData knownRuledSurface() {
  // Two cubic U-profiles at v=0 and v=1; the surface ruled (degree 1) between them.
  BsplineCurveData prof0 = cubicEdge({0, 0, 0}, {5, 0, 0}, {0, -1.0, 1.2});
  BsplineCurveData prof1 = cubicEdge({0, 4, 0}, {5, 4, 0}, {0, 1.1, -0.9});
  const int N = static_cast<int>(prof0.poles.size());
  BsplineSurfaceData s;
  s.degreeU = 3;    s.degreeV = 1;
  s.nPolesU = N;    s.nPolesV = 2;
  s.knotsU = prof0.knots;   s.knotsV = {0, 0, 1, 1};
  s.poles.assign(static_cast<std::size_t>(N) * 2, Point3{});
  for (int i = 0; i < N; ++i) {
    s.poles[static_cast<std::size_t>(i) * 2 + 0] = prof0.poles[i];
    s.poles[static_cast<std::size_t>(i) * 2 + 1] = prof1.poles[i];
  }
  return s;
}

// Extract the four boundary iso-curves of a tensor surface as B-spline curves.
// c0 = S(u,0), c1 = S(u,1) (U-direction curves); d0 = S(0,v), d1 = S(1,v) (V-direction).
static CoonsBoundary extractBoundary(const BsplineSurfaceData& s) {
  CoonsBoundary b;
  // U-direction iso at fixed v: blend each U-pole row's V-poles at v.
  auto isoU = [&](double v) {
    BsplineCurveData c;
    c.degree = s.degreeU;
    c.knots = s.knotsU;
    c.poles.resize(s.nPolesU);
    for (int i = 0; i < s.nPolesU; ++i) {
      std::vector<Point3> vrow(s.nPolesV);
      for (int l = 0; l < s.nPolesV; ++l)
        vrow[l] = s.poles[static_cast<std::size_t>(i) * s.nPolesV + l];
      c.poles[i] = curvePoint(s.degreeV, vrow, s.knotsV, v);
    }
    return c;
  };
  // V-direction iso at fixed u: blend each V-pole column's U-poles at u.
  auto isoV = [&](double u) {
    BsplineCurveData c;
    c.degree = s.degreeV;
    c.knots = s.knotsV;
    c.poles.resize(s.nPolesV);
    for (int j = 0; j < s.nPolesV; ++j) {
      std::vector<Point3> urow(s.nPolesU);
      for (int i = 0; i < s.nPolesU; ++i)
        urow[i] = s.poles[static_cast<std::size_t>(i) * s.nPolesV + j];
      c.poles[j] = curvePoint(s.degreeU, urow, s.knotsU, u);
    }
    return c;
  };
  b.c0 = isoU(0.0);
  b.c1 = isoU(1.0);
  b.d0 = isoV(0.0);
  b.d1 = isoV(1.0);
  return b;
}

int main() {
  // ═══ 1. BOUNDARY CONTAINMENT (the core oracle) + 2. CORNER INTERPOLATION ═══════
  {
    const CoonsBoundary b = curvedBoundary();
    CoonsResult r = coonsPatch(b);
    expectTrue(r.ok, "coonsPatch ok on a curved four-sided boundary");
    expectTrue(r.surface.weights.empty(), "Coons surface is non-rational");
    expectLE(r.maxCornerError, 1e-12, "boundary corners coincide (exact)");

    // Each edge iso-curve reproduces its boundary pointwise.
    const double d0dev = edgeVsCurveMaxDev(r.surface, 0, 0.0, b.c0);  // S(·,0) == c0
    const double d1dev = edgeVsCurveMaxDev(r.surface, 0, 1.0, b.c1);  // S(·,1) == c1
    const double d2dev = edgeVsCurveMaxDev(r.surface, 1, 0.0, b.d0);  // S(0,·) == d0
    const double d3dev = edgeVsCurveMaxDev(r.surface, 1, 1.0, b.d1);  // S(1,·) == d1
    const double worst = std::max(std::max(d0dev, d1dev), std::max(d2dev, d3dev));
    expectLE(worst, 1e-9, "Coons surface contains all 4 boundary curves POINTWISE (containment)");
    std::printf("INFO boundary containment worst dev = %.3e\n", worst);

    // Corner interpolation: the surface's four corners equal the boundary corners.
    const Point3 s00 = evalSurface(r.surface, 0, 0), s10 = evalSurface(r.surface, 1, 0);
    const Point3 s01 = evalSurface(r.surface, 0, 1), s11 = evalSurface(r.surface, 1, 1);
    double cornerWorst = 0.0;
    cornerWorst = std::max(cornerWorst, distance(s00, evalCurve(b.c0, 0.0)));
    cornerWorst = std::max(cornerWorst, distance(s10, evalCurve(b.c0, 1.0)));
    cornerWorst = std::max(cornerWorst, distance(s01, evalCurve(b.c1, 0.0)));
    cornerWorst = std::max(cornerWorst, distance(s11, evalCurve(b.c1, 1.0)));
    expectLE(cornerWorst, 1e-12, "Coons interpolates the four corners exactly");
    std::printf("INFO corner interpolation worst dev = %.3e\n", cornerWorst);
  }

  // ═══ 3. FLAT PATCH ══════════════════════════════════════════════════════════════
  // Four coplanar edges of a rectangle in the z=0 plane → a flat patch on that plane.
  // Use CURVED (but coplanar) boundary edges so it is a genuine flat-plane test, not a
  // trivial straight-edge one — every interior surface point must still lie on z=0.
  {
    CoonsBoundary b;
    b.c0 = cubicEdge(P00, P10, {0, -0.8, 0});  // bow stays in-plane (z=0)
    b.c1 = cubicEdge(P01, P11, {0, 0.9, 0});
    b.d0 = cubicEdge(P00, P01, {-0.7, 0, 0});
    b.d1 = cubicEdge(P10, P11, {0.6, 0, 0});
    CoonsResult r = coonsPatch(b);
    expectTrue(r.ok, "coonsPatch ok on a flat (coplanar) boundary");

    // Every surface point lies on z = 0 (the plane the boundary spans).
    double maxZ = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (int iv = 0; iv <= 40; ++iv) {
        const double u = static_cast<double>(iu) / 40, v = static_cast<double>(iv) / 40;
        maxZ = std::max(maxZ, std::fabs(evalSurface(r.surface, u, v).z));
      }
    expectLE(maxZ, 1e-12, "flat boundary → flat patch (all surface points on z=0 plane)");
    std::printf("INFO flat-patch max |z| = %.3e\n", maxZ);
  }

  // A perfectly rectangular (straight-edge) boundary → the surface is the exact planar
  // bilinear patch: S(u,v) = P00(1-u)(1-v) + P10·u(1-v) + P01(1-u)v + P11·uv.
  {
    CoonsBoundary b;
    b.c0 = lineEdge(P00, P10);
    b.c1 = lineEdge(P01, P11);
    b.d0 = lineEdge(P00, P01);
    b.d1 = lineEdge(P10, P11);
    CoonsResult r = coonsPatch(b);
    expectTrue(r.ok, "coonsPatch ok on a rectangular straight boundary");
    double worst = 0.0;
    for (int iu = 0; iu <= 30; ++iu)
      for (int iv = 0; iv <= 30; ++iv) {
        const double u = static_cast<double>(iu) / 30, v = static_cast<double>(iv) / 30;
        const Point3 expect{
            P00.x * (1 - u) * (1 - v) + P10.x * u * (1 - v) + P01.x * (1 - u) * v + P11.x * u * v,
            P00.y * (1 - u) * (1 - v) + P10.y * u * (1 - v) + P01.y * (1 - u) * v + P11.y * u * v,
            0.0};
        worst = std::max(worst, distance(evalSurface(r.surface, u, v), expect));
      }
    expectLE(worst, 1e-12, "rectangular boundary → exact planar bilinear patch");
  }

  // ═══ 4. KNOWN-SURFACE ROUND-TRIP ═════════════════════════════════════════════════
  // Part (a): a RULED (bilinearly-blended) surface — Coons of its own four boundary
  // iso-curves recovers it POINTWISE (Coons is EXACT for bilinearly-blended surfaces).
  {
    const BsplineSurfaceData src = knownRuledSurface();
    const CoonsBoundary b = extractBoundary(src);
    CoonsResult r = coonsPatch(b);
    expectTrue(r.ok, "round-trip(a): Coons of a ruled surface's boundary ok");

    double worst = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (int iv = 0; iv <= 40; ++iv) {
        const double u = static_cast<double>(iu) / 40, v = static_cast<double>(iv) / 40;
        worst = std::max(worst, distance(evalSurface(r.surface, u, v), evalSurface(src, u, v)));
      }
    expectLE(worst, 1e-9, "round-trip(a): ruled surface recovered POINTWISE (Coons exact)");
    std::printf("INFO ruled-surface round-trip worst dev = %.3e\n", worst);
  }

  // Part (b): a GENERAL tensor surface — Coons reproduces the BOUNDARY exactly (the
  // interior is the bilinear blend by definition, not the original interior). Verified
  // as boundary containment: the four edge iso-curves match the extracted boundary.
  {
    BsplineSurfaceData src;
    src.degreeU = 3;    src.degreeV = 2;
    src.nPolesU = 6;    src.nPolesV = 5;
    src.knotsU = {0, 0, 0, 0, 0.4, 0.7, 1, 1, 1, 1};
    src.knotsV = {0, 0, 0, 0.5, 0.75, 1, 1, 1};
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 5; ++j) {
        const double x = i * 1.2, y = j * 1.5;
        const double z = std::sin(0.6 * i) + std::cos(0.5 * j) + 0.04 * i * j;
        src.poles.push_back({x, y, z});
      }
    const CoonsBoundary b = extractBoundary(src);
    CoonsResult r = coonsPatch(b);
    expectTrue(r.ok, "round-trip(b): Coons of a general surface's boundary ok");

    const double d0dev = edgeVsCurveMaxDev(r.surface, 0, 0.0, b.c0);
    const double d1dev = edgeVsCurveMaxDev(r.surface, 0, 1.0, b.c1);
    const double d2dev = edgeVsCurveMaxDev(r.surface, 1, 0.0, b.d0);
    const double d3dev = edgeVsCurveMaxDev(r.surface, 1, 1.0, b.d1);
    const double worst = std::max(std::max(d0dev, d1dev), std::max(d2dev, d3dev));
    expectLE(worst, 1e-9, "round-trip(b): general surface's four boundaries contained pointwise");
    std::printf("INFO general-surface boundary containment worst dev = %.3e\n", worst);
  }

  // ═══ 5. HONEST DECLINES ══════════════════════════════════════════════════════════
  {
    // Mismatched corner: displace one corner of d1 so it no longer meets c0/c1.
    CoonsBoundary bad = curvedBoundary();
    bad.d1.poles.front().z += 0.5;  // d1(0) no longer equals c0(1)
    CoonsResult r = coonsPatch(bad);
    expectTrue(!r.ok, "mismatched-corner boundary declines honestly");
    expectTrue(!r.reason.empty(), "mismatched-corner decline carries a reason");
    expectTrue(r.maxCornerError > 0.1, "mismatched-corner decline reports a large corner error");

    // verifyCoonsBoundary agrees.
    CoonsCornerCheck chk = verifyCoonsBoundary(bad);
    expectTrue(!chk.ok, "verifyCoonsBoundary declines the mismatched corner");

    // Rational boundary → non-rational scope declines.
    CoonsBoundary ratl = curvedBoundary();
    ratl.c0.weights.assign(ratl.c0.poles.size(), 1.0);  // non-empty ⇒ rational
    expectTrue(!coonsPatch(ratl).ok, "rational boundary declines (non-rational scope)");
    expectTrue(!verifyCoonsBoundary(ratl).ok, "verifyCoonsBoundary declines rational boundary");

    // Malformed boundary (degree 0 / bad knot vector) → declines.
    CoonsBoundary mal = curvedBoundary();
    mal.d0.degree = 0;
    expectTrue(!coonsPatch(mal).ok, "malformed boundary declines");

    // A consistent boundary still succeeds (the guard is not over-eager).
    expectTrue(coonsPatch(curvedBoundary()).ok, "consistent boundary still succeeds");
  }

  // ═══ 6. RATIONAL COONS PATCH (boolean sum in homogeneous R⁴) ═════════════════════
  // Rational analogue of the boundary-containment oracle, plus the strongest oracle: a
  // boundary of two coaxial rational circular ARCS + two straight radial edges fills to
  // the EXACT rational conical/annular strip (every arc iso-curve is a true circle).
  {
    // A rational quarter circle of radius R in the z-plane, centred at the origin, from
    // the +x axis to the +y axis (degree 2, one interior weight cos45°). Endpoints weight 1.
    auto ratQuarterArc = [](double R, double z) {
      BsplineCurveData c;
      c.degree = 2;
      c.knots = {0, 0, 0, 1, 1, 1};  // single Bézier segment, 3 poles
      const double w = 0.7071067811865476;  // 1/√2
      c.poles = {{R, 0, z}, {R, R, z}, {0, R, z}};
      c.weights = {1.0, w, 1.0};
      return c;
    };
    // A straight RATIONAL radial edge from inner radius Ri to outer radius Ro along a
    // direction. Degree-1, weights 1 (a straight line is trivially rational).
    auto ratRadial = [](const Point3& a, const Point3& b) {
      BsplineCurveData c;
      c.degree = 1;
      c.knots = {0, 0, 1, 1};
      c.poles = {a, b};
      c.weights = {1.0, 1.0};
      return c;
    };

    const double Ri = 2.0, Ro = 5.0, z = 0.0;
    // c0 (v=0): inner arc radius Ri; c1 (v=1): outer arc radius Ro (both run in u = angle).
    // d0 (u=0): radial edge at angle 0 from (Ri,0)→(Ro,0); d1 (u=1): radial at 90° (0,Ri)→(0,Ro).
    CoonsBoundary b;
    b.c0 = ratQuarterArc(Ri, z);
    b.c1 = ratQuarterArc(Ro, z);
    b.d0 = ratRadial({Ri, 0, z}, {Ro, 0, z});
    b.d1 = ratRadial({0, Ri, z}, {0, Ro, z});

    CoonsResult r = coonsPatchRational(b);
    expectTrue(r.ok, "coonsPatchRational ok on a rational arc/radial boundary");
    expectTrue(!r.surface.weights.empty(), "rational Coons surface IS rational (has weights)");
    expectLE(r.maxCornerError, 1e-12, "rational boundary corners coincide in R⁴ (exact)");

    // Rational surface evaluator.
    auto evalRatSurface = [](const BsplineSurfaceData& s, double u, double v) {
      SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
      return nurbsSurfacePoint(s.degreeU, s.degreeV, g,
                               std::span<const double>(s.weights), s.knotsU, s.knotsV, u, v);
    };
    auto evalRatCurve = [](const BsplineCurveData& c, double t) {
      return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, t);
    };

    // Boundary containment: each edge iso-curve reproduces its rational boundary pointwise.
    double worst = 0.0;
    for (int i = 0; i <= 200; ++i) {
      const double t = static_cast<double>(i) / 200;
      worst = std::max(worst, distance(evalRatSurface(r.surface, t, 0.0), evalRatCurve(b.c0, t)));
      worst = std::max(worst, distance(evalRatSurface(r.surface, t, 1.0), evalRatCurve(b.c1, t)));
      worst = std::max(worst, distance(evalRatSurface(r.surface, 0.0, t), evalRatCurve(b.d0, t)));
      worst = std::max(worst, distance(evalRatSurface(r.surface, 1.0, t), evalRatCurve(b.d1, t)));
    }
    expectLE(worst, 1e-9, "rational Coons contains all 4 rational boundary curves POINTWISE");
    std::printf("INFO rational boundary containment worst dev = %.3e\n", worst);

    // The STRONGEST oracle: the two arc iso-curves (v=0, v=1) are TRUE circles — every point
    // on them is at exactly the ring radius from the origin. (This proves an exact rational
    // surface, not a facet: only a correctly-weighted rational surface has circular isos.)
    double radErr = 0.0;
    for (int i = 0; i <= 200; ++i) {
      const double t = static_cast<double>(i) / 200;
      const Point3 pin = evalRatSurface(r.surface, t, 0.0);   // inner arc
      const Point3 pout = evalRatSurface(r.surface, t, 1.0);  // outer arc
      radErr = std::max(radErr, std::fabs(std::sqrt(pin.x * pin.x + pin.y * pin.y) - Ri));
      radErr = std::max(radErr, std::fabs(std::sqrt(pout.x * pout.x + pout.y * pout.y) - Ro));
    }
    expectLE(radErr, 1e-9, "rational Coons arc isos are TRUE circles (exact ring radii)");
    std::printf("INFO rational arc-iso radius error = %.3e\n", radErr);
  }

  // Rational round-trip: a KNOWN rational RULED surface (linear blend in v between two
  // rational arcs) — rational Coons of its own four boundary iso-curves recovers it
  // POINTWISE (rational Coons is EXACT for rational bilinearly-blended surfaces).
  {
    auto ratQuarterArc = [](double R) {
      BsplineCurveData c;
      c.degree = 2;
      c.knots = {0, 0, 0, 1, 1, 1};
      const double w = 0.7071067811865476;
      c.poles = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
      c.weights = {1.0, w, 1.0};
      return c;
    };
    // Rational ruled surface: v-degree-1 blend between an inner (v=0) and outer (v=1) arc.
    const BsplineCurveData a0 = ratQuarterArc(2.0);
    const BsplineCurveData a1 = ratQuarterArc(5.0);
    const int N = static_cast<int>(a0.poles.size());
    BsplineSurfaceData src;
    src.degreeU = 2;   src.degreeV = 1;
    src.nPolesU = N;   src.nPolesV = 2;
    src.knotsU = a0.knots;   src.knotsV = {0, 0, 1, 1};
    src.poles.assign(static_cast<std::size_t>(N) * 2, Point3{});
    src.weights.assign(static_cast<std::size_t>(N) * 2, 1.0);
    for (int i = 0; i < N; ++i) {
      src.poles[static_cast<std::size_t>(i) * 2 + 0] = a0.poles[i];
      src.weights[static_cast<std::size_t>(i) * 2 + 0] = a0.weights[i];
      src.poles[static_cast<std::size_t>(i) * 2 + 1] = a1.poles[i];
      src.weights[static_cast<std::size_t>(i) * 2 + 1] = a1.weights[i];
    }

    auto evalRatSurface = [](const BsplineSurfaceData& s, double u, double v) {
      SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
      return nurbsSurfacePoint(s.degreeU, s.degreeV, g,
                               std::span<const double>(s.weights), s.knotsU, s.knotsV, u, v);
    };

    // Extract the four rational boundary iso-curves.
    CoonsBoundary b;
    // c0 = S(u,0) = inner arc; c1 = S(u,1) = outer arc.
    b.c0 = a0; b.c1 = a1;
    // d0 = S(0,v): rational blend of the two arcs' first poles → straight radial (weights 1).
    // d1 = S(1,v): rational blend of the two arcs' last poles → straight radial (weights 1).
    auto ratRadial = [](const Point3& p, const Point3& q) {
      BsplineCurveData c;
      c.degree = 1; c.knots = {0, 0, 1, 1};
      c.poles = {p, q}; c.weights = {1.0, 1.0};
      return c;
    };
    b.d0 = ratRadial(a0.poles.front(), a1.poles.front());
    b.d1 = ratRadial(a0.poles.back(), a1.poles.back());

    CoonsResult r = coonsPatchRational(b);
    expectTrue(r.ok, "rational round-trip: Coons of a rational ruled surface's boundary ok");
    double worst = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (int iv = 0; iv <= 40; ++iv) {
        const double u = static_cast<double>(iu) / 40, v = static_cast<double>(iv) / 40;
        worst = std::max(worst,
                         distance(evalRatSurface(r.surface, u, v), evalRatSurface(src, u, v)));
      }
    expectLE(worst, 1e-9, "rational round-trip: rational ruled surface recovered POINTWISE");
    std::printf("INFO rational ruled round-trip worst dev = %.3e\n", worst);
  }

  // ═══ 7. RATIONAL COONS HONEST DECLINES ═══════════════════════════════════════════
  {
    auto ratQuarterArc = [](double R, double z) {
      BsplineCurveData c;
      c.degree = 2; c.knots = {0, 0, 0, 1, 1, 1};
      const double w = 0.7071067811865476;
      c.poles = {{R, 0, z}, {R, R, z}, {0, R, z}};
      c.weights = {1.0, w, 1.0};
      return c;
    };
    auto ratRadial = [](const Point3& a, const Point3& b) {
      BsplineCurveData c;
      c.degree = 1; c.knots = {0, 0, 1, 1};
      c.poles = {a, b}; c.weights = {1.0, 1.0};
      return c;
    };
    auto goodRatBoundary = [&]() {
      CoonsBoundary b;
      b.c0 = ratQuarterArc(2.0, 0.0);
      b.c1 = ratQuarterArc(5.0, 0.0);
      b.d0 = ratRadial({2, 0, 0}, {5, 0, 0});
      b.d1 = ratRadial({0, 2, 0}, {0, 5, 0});
      return b;
    };

    // A consistent rational boundary still succeeds (the guard is not over-eager).
    expectTrue(coonsPatchRational(goodRatBoundary()).ok,
               "consistent rational boundary still succeeds");

    // NON-rational boundary → rational scope declines (empty weights ⇒ use coonsPatch).
    CoonsBoundary nonrat = goodRatBoundary();
    nonrat.c0.weights.clear();
    expectTrue(!coonsPatchRational(nonrat).ok,
               "non-rational boundary declines coonsPatchRational (use coonsPatch)");
    expectTrue(!verifyRationalCoonsBoundary(nonrat).ok,
               "verifyRationalCoonsBoundary declines non-rational boundary");

    // Non-positive weight → declines (never divides by ≤ 0, never a faked net).
    CoonsBoundary badw = goodRatBoundary();
    badw.c0.weights[1] = -0.5;
    expectTrue(!coonsPatchRational(badw).ok, "non-positive weight declines honestly");

    // WEIGHT-mismatched corner: the two curves meeting at P00 carry different weights there
    // even though the POSITIONS coincide → homogeneous corner mismatch → declines.
    CoonsBoundary wmis = goodRatBoundary();
    wmis.d0.weights.front() = 2.0;  // d0(0) weight ≠ c0(0) weight, same position
    CoonsResult rw = coonsPatchRational(wmis);
    expectTrue(!rw.ok, "weight-mismatched corner declines (homogeneous corner check)");
    expectTrue(rw.maxCornerError > 0.1, "weight-mismatched corner reports a large R⁴ error");

    // POSITION-mismatched corner → declines.
    CoonsBoundary pmis = goodRatBoundary();
    pmis.d1.poles.front().z += 0.5;
    expectTrue(!coonsPatchRational(pmis).ok, "position-mismatched rational corner declines");

    // The non-rational coonsPatch is UNAFFECTED and still declines a rational boundary
    // (byte-unchanged behaviour of the original entry point).
    expectTrue(!coonsPatch(goodRatBoundary()).ok,
               "coonsPatch still declines a rational boundary (unchanged)");
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_coons: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_coons: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_coons (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
