// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native math library (OCCT-FREE, analytic assertions).
// Build: clang++ -std=c++20 tests/test_native_math.cpp src/native/math/bspline.cpp \
//        src/native/math/bezier.cpp -I src -o test_native_math
//
#include "native/math/native_math.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static void expectNear(double a, double b, double tol, const char* what) {
  if (std::fabs(a - b) > tol) {
    std::printf("FAIL %-40s got %.12g want %.12g (tol %g)\n", what, a, b, tol);
    ++g_failures;
  }
}
static void expectPt(const Point3& p, double x, double y, double z, const char* what,
                     double tol = 1e-10) {
  expectNear(p.x, x, tol, what);
  expectNear(p.y, y, tol, what);
  expectNear(p.z, z, tol, what);
}
static void expectVec(const Vec3& p, double x, double y, double z, const char* what,
                      double tol = 1e-10) {
  expectNear(p.x, x, tol, what);
  expectNear(p.y, y, tol, what);
  expectNear(p.z, z, tol, what);
}

int main() {
  constexpr double kPi = 3.14159265358979323846;

  // ── Vec / Dir ──────────────────────────────────────────────────────────────
  {
    Vec3 a{1, 2, 3}, b{4, 5, 6};
    expectNear(dot(a, b), 32.0, 1e-12, "dot");
    expectVec(cross(Vec3{1, 0, 0}, Vec3{0, 1, 0}), 0, 0, 1, "cross RH");
    Dir3 d{3, 0, 0};
    expectNear(norm(d.vec()), 1.0, 1e-15, "dir unit");
    expectNear(Dir3(1, 0, 0).angle(Dir3(0, 1, 0)), kPi / 2, 1e-12, "dir angle 90");
    Dir3 bad{0, 0, 0};
    if (bad.valid()) { std::printf("FAIL null dir marked valid\n"); ++g_failures; }
  }

  // ── Transform: identity, translate, rotate, compose, inverse ────────────────
  {
    Transform id;
    expectPt(id.applyToPoint({7, 8, 9}), 7, 8, 9, "identity point");

    // Rotate +90° about Z around origin: (1,0,0) -> (0,1,0).
    Transform rz = Transform::rotationOf(Point3{0, 0, 0}, Dir3{0, 0, 1}, kPi / 2);
    expectPt(rz.applyToPoint({1, 0, 0}), 0, 1, 0, "rotZ90 point");
    expectVec(rz.applyToVector({1, 0, 0}), 0, 1, 0, "rotZ90 vector");

    // Rotation is length preserving & inverse composes to identity.
    auto inv = rz.inverse();
    if (!inv) { std::printf("FAIL rot inverse\n"); ++g_failures; }
    Point3 rt = inv->applyToPoint(rz.applyToPoint({3, -2, 5}));
    expectPt(rt, 3, -2, 5, "rot inverse round-trip");

    // Rotation about a non-origin center.
    Transform rc = Transform::rotationOf(Point3{1, 1, 0}, Dir3{0, 0, 1}, kPi);
    expectPt(rc.applyToPoint({2, 1, 0}), 0, 1, 0, "rotPi about (1,1)");

    // Non-uniform scale + translate compose.
    Transform s = Transform::scaleOf(Point3{0, 0, 0}, 2, 3, 4);
    Transform t = Transform::translationOf({10, 0, 0});
    Transform ts = t.composedWith(s);  // scale first, then translate
    expectPt(ts.applyToPoint({1, 1, 1}), 12, 3, 4, "compose scale+trans");
    expectNear(s.determinant(), 24.0, 1e-12, "scale det");

    // Mirror detection.
    Transform mir = Transform::scaleOf(Point3{0, 0, 0}, -1, 1, 1);
    if (!mir.isMirrored()) { std::printf("FAIL mirror det\n"); ++g_failures; }
  }

  // ── Bézier: exact known values ─────────────────────────────────────────────
  {
    // Quadratic through (0,0,0),(1,2,0),(2,0,0). At t=0.5 => (1, 1, 0).
    std::array<Point3, 3> poles{{{0, 0, 0}, {1, 2, 0}, {2, 0, 0}}};
    expectPt(bezierPoint(poles, 0.0), 0, 0, 0, "bezier t0");
    expectPt(bezierPoint(poles, 1.0), 2, 0, 0, "bezier t1");
    expectPt(bezierPoint(poles, 0.5), 1, 1, 0, "bezier t0.5");

    // Derivative of the quadratic at t: C'(t) = 2[(1-t)(P1-P0) + t(P2-P1)].
    std::array<Vec3, 3> d{};
    bezierDerivs(poles, 0.5, 2, d);
    expectVec(d[0], 1, 1, 0, "bezierDerivs value");
    // At t=0.5: 2[(0.5)(1,2)+(0.5)(1,-2)] = 2[(1,0)] = (2,0,0).
    expectVec(d[1], 2, 0, 0, "bezierDerivs D1");
    // 2nd deriv constant: 2*(P0-2P1+P2) = 2*(0-2+2, 0-4+0,0) = (0,-8,0).
    expectVec(d[2], 0, -8, 0, "bezierDerivs D2");

    // Rational Bézier reproducing a circle arc quadrant (weights 1,1/√2,1):
    // P0=(1,0), P1=(1,1), P2=(0,1); at t=0.5 the point lies ON the unit circle.
    std::array<Point3, 3> cp{{{1, 0, 0}, {1, 1, 0}, {0, 1, 0}}};
    std::array<double, 3> w{{1.0, std::sqrt(0.5), 1.0}};
    Point3 mid = rationalBezierPoint(cp, w, 0.5);
    expectNear(std::sqrt(mid.x * mid.x + mid.y * mid.y), 1.0, 1e-12, "rational bezier on circle");
  }

  // ── Bézier surface: flat plane patch + normal ──────────────────────────────
  {
    // 2x2 bilinear patch on z=0 plane, unit square.
    std::array<Point3, 4> g{{{0, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 1, 0}}};
    Point3 c = bezierSurfacePoint(g, 2, 2, 0.5, 0.5);
    expectPt(c, 0.5, 0.5, 0, "bezier surf center");
    auto d1 = bezierSurfaceD1(g, 2, 2, 0.5, 0.5);
    // Normal should be ±Z.
    expectNear(std::fabs(d1.normal.z()), 1.0, 1e-12, "bezier surf normal");
  }

  // ── B-spline curve: a clamped cubic that reduces to a straight line ─────────
  {
    // Degree-1 (polyline) B-spline: exact linear interpolation checkpoint.
    // Poles on a line; clamped knots {0,0,1,2,2}? Use degree 2 Bezier-equivalent.
    // Clamped quadratic with knots {0,0,0,1,1,1} == a single Bézier segment.
    std::array<Point3, 3> poles{{{0, 0, 0}, {1, 2, 0}, {2, 0, 0}}};
    std::array<double, 6> knots{{0, 0, 0, 1, 1, 1}};
    expectPt(curvePoint(2, poles, knots, 0.5), 1, 1, 0, "bspline==bezier midpoint");
    // de Boor path must agree.
    expectPt(curvePointDeBoor(2, poles, knots, 0.5), 1, 1, 0, "deBoor==bezier midpoint");
    // Endpoints interpolate first/last pole (clamped).
    expectPt(curvePoint(2, poles, knots, 0.0), 0, 0, 0, "clamped start");
    expectPt(curvePoint(2, poles, knots, 1.0), 2, 0, 0, "clamped end");

    // Partition-of-unity: basis funcs sum to 1.
    int span = findSpan(2, 2, 0.5, knots);
    std::array<double, 3> N{};
    basisFuns(span, 0.5, 2, knots, N);
    expectNear(N[0] + N[1] + N[2], 1.0, 1e-14, "basis partition of unity");

    // Curve derivative equals the Bézier derivative for this equivalent curve.
    std::array<Vec3, 3> cd{};
    curveDerivs(2, poles, knots, 0.5, 2, cd);
    expectVec(cd[1], 2, 0, 0, "bspline D1");
    expectVec(cd[2], 0, -8, 0, "bspline D2");
  }

  // ── NURBS curve: exact circle (rational quadratic, 9-pole full circle) ──────
  {
    // Quarter-circle rational quadratic on unit circle. Knots {0,0,0,1,1,1},
    // poles (1,0),(1,1),(0,1), weights 1,√½,1. Every sampled point has r=1.
    std::vector<Point3> poles{{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
    std::vector<double> w{1.0, std::sqrt(0.5), 1.0};
    std::vector<double> knots{0, 0, 0, 1, 1, 1};
    for (double u : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      Point3 p = nurbsCurvePoint(2, poles, w, knots, u);
      expectNear(std::hypot(p.x, p.y), 1.0, 1e-12, "nurbs circle radius");
    }
    // Tangent at start (u=0, point (1,0)) is +Y direction (circle CCW).
    std::array<Vec3, 2> d{};
    nurbsCurveDerivs(2, poles, w, knots, 0.0, 1, d);
    expectPt(Point3{d[0].x, d[0].y, d[0].z}, 1, 0, 0, "nurbs circle start pt");
    // Tangent should be along +Y (x-component ~0).
    expectNear(d[1].x, 0.0, 1e-9, "nurbs circle tangent x");
    if (d[1].y <= 0) { std::printf("FAIL nurbs tangent y sign\n"); ++g_failures; }
  }

  // ── B-spline / NURBS surface + normal ───────────────────────────────────────
  {
    // Bilinear-equivalent quadratic×quadratic Bézier patch (knots clamped) on
    // a paraboloid-ish net; just verify point matches the Bézier surface path
    // and the normal is a unit vector.
    std::vector<Point3> g{
        {0, 0, 0}, {1, 0, 1}, {2, 0, 0},
        {0, 1, 1}, {1, 1, 2}, {2, 1, 1},
        {0, 2, 0}, {1, 2, 1}, {2, 2, 0}};
    SurfaceGrid grid{g, 3, 3};
    std::vector<double> ku{0, 0, 0, 1, 1, 1};
    std::vector<double> kv{0, 0, 0, 1, 1, 1};
    Point3 sp = surfacePoint(2, 2, grid, ku, kv, 0.5, 0.5);
    std::array<Point3, 9> bg{};
    for (int i = 0; i < 9; ++i) bg[i] = g[i];
    Point3 bp = bezierSurfacePoint(bg, 3, 3, 0.5, 0.5);
    expectPt(sp, bp.x, bp.y, bp.z, "bspline surf == bezier surf");

    Dir3 n = surfaceNormal(2, 2, grid, {}, ku, kv, 0.5, 0.5);
    expectNear(norm(n.vec()), 1.0, 1e-12, "surface normal unit");

    // NURBS surface with all weights 1 must equal the non-rational surface.
    std::vector<double> wgt(9, 1.0);
    Point3 rp = nurbsSurfacePoint(2, 2, grid, wgt, ku, kv, 0.5, 0.5);
    expectPt(rp, sp.x, sp.y, sp.z, "nurbs w=1 == bspline surf");
    std::array<Vec3, 4> rd{};
    nurbsSurfaceDerivs(2, 2, grid, wgt, ku, kv, 0.5, 0.5, 1, rd);
    std::array<Vec3, 4> nd{};
    surfaceDerivs(2, 2, grid, ku, kv, 0.5, 0.5, 1, nd);
    expectVec(rd[1 * 2 + 0], nd[1 * 2 + 0].x, nd[1 * 2 + 0].y, nd[1 * 2 + 0].z,
              "nurbs w=1 dU == bspline dU");
  }

  // ── Elementary surfaces (parity with ElSLib parametrization) ────────────────
  {
    Ax3 f;  // identity frame at origin
    Plane pl{f};
    expectPt(pl.value(3, 4), 3, 4, 0, "plane value");
    expectNear(pl.normal(0, 0).z(), 1.0, 1e-15, "plane normal");

    Cylinder cy{f, 2.0};
    expectPt(cy.value(0.0, 5.0), 2, 0, 5, "cyl value u0");
    expectPt(cy.value(kPi / 2, 1.0), 0, 2, 1, "cyl value u90", 1e-12);
    expectNear(cy.normal(0.0, 0.0).x(), 1.0, 1e-12, "cyl normal outward");

    Sphere sp{f, 3.0};
    // u=0, v=0 -> (R,0,0); north pole v=pi/2 -> (0,0,R).
    expectPt(sp.value(0, 0), 3, 0, 0, "sphere equator");
    expectPt(sp.value(0, kPi / 2), 0, 0, 3, "sphere pole", 1e-12);
    // Normal at equator points +X (outward).
    expectNear(sp.normal(0, 0).x(), 1.0, 1e-12, "sphere normal outward");
    // Every surface point is exactly R from center.
    expectNear(distance(sp.value(1.1, 0.4), f.origin), 3.0, 1e-12, "sphere radius");

    Cone co{f, 1.0, kPi / 4};  // 45° half-angle, base radius 1 at v=0
    // At v=0 it's the base circle of radius 1.
    expectPt(co.value(0, 0), 1, 0, 0, "cone base");
    // At v=√2 the radius grows by v·sin45 = 1, apexward height v·cos45 = 1.
    Point3 cpt = co.value(0, std::sqrt(2.0));
    expectPt(cpt, 2, 0, 1, "cone up", 1e-12);
    expectNear(norm(co.normal(0.3, 0.5).vec()), 1.0, 1e-12, "cone normal unit");
  }

  if (g_failures == 0) {
    std::printf("native math: ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("native math: %d FAILURES\n", g_failures);
  return 1;
}
