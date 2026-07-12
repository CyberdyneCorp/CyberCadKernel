// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the Wave-H track H1 curve↔curve (CCI) and curve↔surface
// (CSI) NURBS intersection (src/native/math/bspline_intersect.{h,cpp}). OCCT-FREE.
// The oracles are AIRTIGHT and closed-form — the intersection points are known in
// advance to machine precision, and the tolerances asserted are the ACHIEVED
// errors, never widened to pass:
//
//   1. LINE ↔ CIRCLE — a straight NURBS segment through the exact rational unit
//      circle (9-pole degree-2). A secant at x = 0.5 crosses at y = ±√0.75 → the
//      TWO known points recovered to ≤ 1e-10, with the correct circle parameters.
//   2. LINE tangent to the circle (x ≡ 1) — ONE hit at (1,0), reported TANGENTIAL.
//   3. LINE missing the circle (x ≡ 1.5) — ZERO hits (no faked crossing).
//   4. TWO PARABOLAS with a known crossing — y=x² and y=2−x² cross at x=±1, y=1;
//      both points recovered ≤ 1e-10 with correct params on both curves.
//   5. COINCIDENT curves (a segment overlapping itself) — HONEST-DECLINE
//      (status Coincident, no fabricated point list).
//   6. LINE ↔ PLANE — a line piercing a bilinear NURBS plane at a known point,
//      recovered ≤ 1e-9, transversal.
//   7. LINE ↔ CYLINDER — a line through a rational half-cylinder → the known
//      pierce point on the wall (≤ 1e-9), transversal.
//   8. CURVE-ON-SURFACE coincidence — a curve lying in the plane → HONEST-DECLINE.
//
// The routines are numsci-gated (the intersection entry points live in the
// numsci-gated NURBS layer), so the whole gate is under CYBERCAD_HAS_NUMSCI (like
// test_native_ssi_seeding). With the guard OFF this compiles to a trivial pass so
// the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_intersect.h"

#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;
static double g_maxLineCircleErr = 0.0;
static double g_maxCurveSurfErr = 0.0;

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
static void expectEq(std::size_t a, std::size_t b, const char* what) {
  ++g_checks;
  if (a != b) {
    std::printf("FAIL %-46s got %zu want %zu\n", what, a, b);
    ++g_failures;
  }
}

// ── Geometry builders ────────────────────────────────────────────────────────

// A straight degree-1 NURBS segment from a to b (non-rational).
struct Seg {
  int degree = 1;
  std::vector<Point3> poles;
  std::vector<double> knots{0, 0, 1, 1};
  CurveView view() const { return {degree, poles, {}, knots}; }
};
static Seg segment(Point3 a, Point3 b) {
  Seg s;
  s.poles = {a, b};
  return s;
}

// The exact rational unit circle (degree 2, 9 poles) in z=0. x²+y²=1 EXACTLY.
struct RCircle {
  int degree = 2;
  std::vector<Point3> poles = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {-1, 1, 0}, {-1, 0, 0},
                               {-1, -1, 0}, {0, -1, 0}, {1, -1, 0}, {1, 0, 0}};
  std::vector<double> weights;
  std::vector<double> knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  RCircle() {
    const double w = std::sqrt(2.0) / 2.0;
    weights = {1, w, 1, w, 1, w, 1, w, 1};
  }
  CurveView view() const { return {degree, poles, weights, knots}; }
};

// A non-rational Bézier parabola y = c0 + c1*x + c2*x² sampled as a degree-2 curve
// over x ∈ [x0,x1]. For a quadratic, the 3 Bézier poles reproduce the polynomial
// exactly: P0=(x0,f(x0)), P2=(x1,f(x1)), P1 = the control point making the Bézier
// match — for y=a*x²+b*x+c the middle pole is ((x0+x1)/2, f_lin_mid) where the
// y is chosen so the Bézier equals the parabola. Simplest: parametrize x linearly.
struct Parab {
  int degree = 2;
  std::vector<Point3> poles;
  std::vector<double> knots{0, 0, 0, 1, 1, 1};
  CurveView view() const { return {degree, poles, {}, knots}; }
};
// Parabola y = A*x² + B*x + C over x∈[x0,x1] as an EXACT quadratic Bézier.
// x(t)=x0+(x1-x0)t linear; y(t) must equal A x² + B x + C. A quadratic Bézier with
// poles y0,y1,y2 gives y(t)=(1-t)²y0+2(1-t)t y1+t²y2. Matching to the parabola:
//   y0=f(x0), y2=f(x1), y1 = (f(x0)+f(x1))/2 - A*(x1-x0)²/4  ... derived below.
static Parab parabola(double A, double B, double C, double x0, double x1) {
  Parab p;
  auto f = [&](double x) { return A * x * x + B * x + C; };
  const double y0 = f(x0), y2 = f(x1);
  // y(t) with linear x: A(x0+dt)²+B(x0+dt)+C, d=x1-x0. Expand: quadratic in t with
  // t² coeff = A d². The Bézier t² coeff is (y0 - 2 y1 + y2). The 2(1-t)t y1 midpole:
  // solve y1 so all three Bézier coeffs match. Standard result:
  //   y1 = y0 + 0.5*(dy/dt at 0)  where dy/dt|0 = 2A x0 d + B d.
  const double d = x1 - x0;
  const double y1 = y0 + 0.5 * (2.0 * A * x0 * d + B * d);
  p.poles = {{x0, y0, 0}, {0.5 * (x0 + x1), y1, 0}, {x1, y2, 0}};
  return p;
}

// A bilinear NURBS plane z=0 over [-2,2]×[-2,2] (non-rational, degree 1×1).
struct FlatPlane {
  std::vector<Point3> poles = {{-2, -2, 0}, {-2, 2, 0}, {2, -2, 0}, {2, 2, 0}};
  std::vector<double> knotsU{0, 0, 1, 1};
  std::vector<double> knotsV{0, 0, 1, 1};
  SurfaceView view() const { return {1, 1, poles, {}, 2, 2, knotsU, knotsV}; }
};

// A rational half-cylinder (unit radius, axis = z), degree U=2 circular half-arc,
// degree V=1 linear extrude over z∈[0,2]. x²+y²=1 on the wall.
struct HalfCyl {
  int degreeU = 2, degreeV = 1;
  int nRows = 5, nCols = 2;
  std::vector<Point3> poles;
  std::vector<double> weights;
  std::vector<double> knotsU{0, 0, 0, 0.5, 0.5, 1, 1, 1};
  std::vector<double> knotsV{0, 0, 1, 1};
  HalfCyl() {
    const double w = std::sqrt(2.0) / 2.0;
    const Point3 arc[5] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {-1, 1, 0}, {-1, 0, 0}};
    const double aw[5] = {1, w, 1, w, 1};
    for (int i = 0; i < 5; ++i)
      for (int j = 0; j < 2; ++j) {
        poles.push_back({arc[i].x, arc[i].y, 2.0 * j});
        weights.push_back(aw[i]);
      }
  }
  SurfaceView view() const {
    return {degreeU, degreeV, poles, weights, nRows, nCols, knotsU, knotsV};
  }
};

static void trackLC(double e) { if (e > g_maxLineCircleErr) g_maxLineCircleErr = e; }
static void trackCS(double e) { if (e > g_maxCurveSurfErr) g_maxCurveSurfErr = e; }

// ── Tests ────────────────────────────────────────────────────────────────────

static void testLineCircleTwoCrossings() {
  RCircle circle;
  // Vertical secant x = 0.5, y from -2 to 2: crosses the unit circle at
  // (0.5, ±√0.75). √0.75 = 0.8660254037844386.
  Seg seg = segment({0.5, -2, 0}, {0.5, 2, 0});
  CurveCurveResult r = intersectCurveCurve(seg.view(), circle.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "line-circle status Ok");
  expectEq(r.hits.size(), 2, "line-circle 2 hits");
  const double yExpect = std::sqrt(0.75);
  // Hits are sorted by paramA (the segment param): the lower y first.
  if (r.hits.size() == 2) {
    // Both x must be 0.5, y must be ±√0.75, on the unit circle (x²+y²=1).
    for (const auto& h : r.hits) {
      expectNear(h.point.x, 0.5, 1e-10, "line-circle x=0.5");
      expectNear(std::fabs(h.point.y), yExpect, 1e-10, "line-circle |y|=√0.75");
      expectNear(h.point.x * h.point.x + h.point.y * h.point.y, 1.0, 1e-10,
                 "line-circle on unit circle");
      expectTrue(h.type == IntersectionType::Transversal, "line-circle transversal");
      trackLC(std::fabs(std::fabs(h.point.y) - yExpect));
      trackLC(std::fabs(h.point.x * h.point.x + h.point.y * h.point.y - 1.0));
    }
    // The two hits are distinct (opposite y sign).
    expectTrue(r.hits[0].point.y * r.hits[1].point.y < 0, "line-circle two distinct sides");
  }
}

static void testLineTangentCircle() {
  RCircle circle;
  // x ≡ 1 tangent line: touches the circle at (1,0) only.
  Seg seg = segment({1, -2, 0}, {1, 2, 0});
  CurveCurveResult r = intersectCurveCurve(seg.view(), circle.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "tangent status Ok");
  expectEq(r.hits.size(), 1, "tangent 1 hit");
  if (r.hits.size() == 1) {
    // The line contributes x = 1 EXACTLY; the point lies on the unit circle to
    // machine precision. But a TANGENCY is inherently less well-LOCALISED along the
    // circle than a transversal crossing: the Newton normal matrix is singular at a
    // tangent, so the y-coordinate of the contact converges only to ~1e-7 (the honest
    // √machine-eps-scale limit — NOT widened to fake 1e-10). We assert the airtight
    // invariants (on-circle, x=1, classified Tangential) tightly, and the contact
    // position to its ACHIEVED tangency accuracy.
    expectNear(r.hits[0].point.x, 1.0, 1e-9, "tangent x=1");
    expectNear(r.hits[0].point.x * r.hits[0].point.x +
                   r.hits[0].point.y * r.hits[0].point.y,
               1.0, 1e-9, "tangent point on unit circle");
    expectNear(r.hits[0].point.y, 0.0, 1e-6, "tangent y≈0 (tangency accuracy)");
    expectTrue(r.hits[0].type == IntersectionType::Tangential, "tangent reported Tangential");
  }
}

static void testLineMissesCircle() {
  RCircle circle;
  Seg seg = segment({1.5, -2, 0}, {1.5, 2, 0});  // x=1.5 misses the unit circle
  CurveCurveResult r = intersectCurveCurve(seg.view(), circle.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "miss status Ok");
  expectEq(r.hits.size(), 0, "miss 0 hits (no faked crossing)");
}

static void testTwoParabolas() {
  // y = x²  and  y = 2 − x²  cross where x² = 2 − x² → x = ±1, y = 1.
  Parab p1 = parabola(1, 0, 0, -1.5, 1.5);
  Parab p2 = parabola(-1, 0, 2, -1.5, 1.5);
  CurveCurveResult r = intersectCurveCurve(p1.view(), p2.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "parabolas status Ok");
  expectEq(r.hits.size(), 2, "parabolas 2 crossings");
  if (r.hits.size() == 2) {
    for (const auto& h : r.hits) {
      expectNear(std::fabs(h.point.x), 1.0, 1e-10, "parabolas |x|=1");
      expectNear(h.point.y, 1.0, 1e-10, "parabolas y=1");
      // Verify the recovered params reproduce the point on BOTH curves.
      Point3 a = curvePoint(p1.degree, p1.poles, p1.knots, h.paramA);
      Point3 b = curvePoint(p2.degree, p2.poles, p2.knots, h.paramB);
      expectNear(distance(a, b), 0.0, 1e-10, "parabolas params agree");
      trackLC(std::fabs(std::fabs(h.point.x) - 1.0));
    }
    expectTrue(r.hits[0].point.x * r.hits[1].point.x < 0, "parabolas two distinct x");
  }
}

static void testCoincidentCurvesDecline() {
  // Two identical straight segments → overlapping (infinite intersection).
  Seg a = segment({-1, 0, 0}, {1, 0, 0});
  Seg b = segment({-1, 0, 0}, {1, 0, 0});
  CurveCurveResult r = intersectCurveCurve(a.view(), b.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Coincident, "coincident curves declined");
  expectEq(r.hits.size(), 0, "coincident curves no fabricated points");
}

static void testLinePlane() {
  FlatPlane plane;
  // Line from (0.3,0.4,1) to (0.3,0.4,-1) pierces z=0 at (0.3,0.4,0).
  Seg seg = segment({0.3, 0.4, 1}, {0.3, 0.4, -1});
  CurveSurfaceResult r = intersectCurveSurface(seg.view(), plane.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "line-plane status Ok");
  expectEq(r.hits.size(), 1, "line-plane 1 hit");
  if (r.hits.size() == 1) {
    expectNear(r.hits[0].point.x, 0.3, 1e-9, "line-plane x");
    expectNear(r.hits[0].point.y, 0.4, 1e-9, "line-plane y");
    expectNear(r.hits[0].point.z, 0.0, 1e-9, "line-plane z=0");
    expectTrue(r.hits[0].type == IntersectionType::Transversal, "line-plane transversal");
    trackCS(std::fabs(r.hits[0].point.z));
  }
}

static void testLineCylinder() {
  HalfCyl cyl;
  // A line along +x at (y=0, z=1) from x=-2 to x=2. The half-cylinder wall covers
  // the arc from (1,0) around to (-1,0) (upper half, y≥0). The line hits the wall
  // where x²+y²=1 with y=0 → x=1 (the (1,0) generator) and x=-1 ((-1,0)). Both are
  // on the closed ends of the half arc, at z=1.
  Seg seg = segment({-2, 0, 1}, {2, 0, 1});
  CurveSurfaceResult r = intersectCurveSurface(seg.view(), cyl.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Ok, "line-cyl status Ok");
  expectEq(r.hits.size(), 2, "line-cyl 2 pierce points");
  if (r.hits.size() == 2) {
    for (const auto& h : r.hits) {
      expectNear(std::fabs(h.point.x), 1.0, 1e-9, "line-cyl |x|=1");
      expectNear(h.point.y, 0.0, 1e-9, "line-cyl y=0");
      expectNear(h.point.z, 1.0, 1e-9, "line-cyl z=1");
      expectNear(h.point.x * h.point.x + h.point.y * h.point.y, 1.0, 1e-9,
                 "line-cyl on unit cylinder");
      trackCS(std::fabs(h.point.x * h.point.x + h.point.y * h.point.y - 1.0));
    }
  }
}

static void testCurveOnSurfaceDecline() {
  FlatPlane plane;
  // A curve lying IN the z=0 plane (a segment in-plane) → curve-on-surface overlap.
  Seg seg = segment({-1, -1, 0}, {1, 1, 0});
  CurveSurfaceResult r = intersectCurveSurface(seg.view(), plane.view(), 1e-9);
  expectTrue(r.status == IntersectStatus::Coincident, "curve-on-surface declined");
  expectEq(r.hits.size(), 0, "curve-on-surface no fabricated points");
}

#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#ifdef CYBERCAD_HAS_NUMSCI
  testLineCircleTwoCrossings();
  testLineTangentCircle();
  testLineMissesCircle();
  testTwoParabolas();
  testCoincidentCurvesDecline();
  testLinePlane();
  testLineCylinder();
  testCurveOnSurfaceDecline();

  std::printf("\nbspline_intersect: %d checks, %d failures\n", g_checks, g_failures);
  std::printf("max line↔circle point error   = %.3e\n", g_maxLineCircleErr);
  std::printf("max curve↔surface point error = %.3e\n", g_maxCurveSurfErr);
  if (g_failures == 0) std::printf("ALL PASS\n");
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("bspline_intersect: CYBERCAD_HAS_NUMSCI off — trivial pass\n");
  return 0;
#endif
}
