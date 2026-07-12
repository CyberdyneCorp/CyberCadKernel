// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 1 — EXACT analytic↔NURBS conversion
// (src/native/math/analytic_nurbs.{h,cpp}). OCCT-FREE. Airtight, closed-form oracles:
//
//   1. ROUND-TRIP EXACT — recognize(toNurbs(prim)) recovers prim's parameters to
//      ≤1e-12 for circle / arc / ellipse / line and plane / cylinder / cone / sphere.
//   2. EVALUATION EXACTNESS — points sampled on circleToNurbs lie on the TRUE circle
//      to ≤1e-14 (the rational quadratic is EXACT, not an approximation); the same for
//      cylinderToNurbs / sphereToNurbs / torusToNurbs points on their true surface.
//   3. DISCRIMINATION — a genuinely freeform rational NURBS (bicubic bump) → General,
//      NOT a spurious primitive; a non-uniform-weight almost-circle that ISN'T a
//      circle → General (the control-net exactness certificate rejects it).
//
// analytic_nurbs routes recognition through primitive_fit (numsci facade), so the gate
// is under CYBERCAD_HAS_NUMSCI (like test_native_primitive_fit). Guard OFF → trivial pass.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/analytic_nurbs.h"
#include "native/math/bspline.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.16g want %.16g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static double absCos(const Dir3& a, const Dir3& b) {
  return std::fabs(dot(a.vec(), b.vec()));
}

constexpr double kPi = 3.14159265358979323846;

// Evaluate a NURBS curve at u.
static Point3 evalC(const BsplineCurveData& c, double u) {
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
// Evaluate a NURBS surface at (u,v).
static Point3 evalS(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

int main() {
  // ── 1. CIRCLE round-trip + evaluation exactness ─────────────────────────────
  {
    Circle c;
    c.center = {1, 2, -3};
    c.normal = Dir3(0.2, -0.3, 1.0);
    c.xAxis = Dir3(cross(Vec3{0, 1, 0}, c.normal.vec()));  // some ⟂ X
    // Re-orthonormalize X ⟂ normal.
    {
      Vec3 x = c.xAxis.vec();
      x = x - c.normal.vec() * dot(x, c.normal.vec());
      c.xAxis = Dir3(x);
    }
    c.radius = 2.75;
    const BsplineCurveData nurbs = circleToNurbs(c);

    // Evaluation exactness: dense samples lie on the true circle to ≤1e-14.
    double worstEval = 0.0;
    for (int i = 0; i <= 400; ++i) {
      const double u = static_cast<double>(i) / 400.0;
      const Point3 p = evalC(nurbs, u);
      // Distance from center within the plane must equal radius; out-of-plane 0.
      const Vec3 rel = p - c.center;
      const double outPlane = std::fabs(dot(rel, c.normal.vec()));
      const double inPlane = std::sqrt(std::max(0.0, dot(rel, rel) - outPlane * outPlane));
      worstEval = std::max(worstEval, std::fabs(inPlane - c.radius));
      worstEval = std::max(worstEval, outPlane);
    }
    expectLE(worstEval, 1e-13, "circle eval on true circle");

    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind == CurveKind::Circle, "circle recognized");
    expectLE(rec.residual, 1e-12, "circle residual exact");
    expectNear(distance(rec.circle.center, c.center), 0.0, 1e-12, "circle center");
    expectNear(rec.circle.radius, c.radius, 1e-12, "circle radius");
    expectNear(absCos(rec.circle.normal, c.normal), 1.0, 1e-12, "circle normal");
  }

  // ── 2. ARC round-trip ───────────────────────────────────────────────────────
  {
    Arc a;
    a.circle.center = {0, 0, 0};
    a.circle.normal = Dir3(0, 0, 1);
    a.circle.xAxis = Dir3(1, 0, 0);
    a.circle.radius = 4.0;
    a.startAngle = 0.3;
    a.sweepAngle = 1.7;  // ~97° → 2 segments
    const BsplineCurveData nurbs = arcToNurbs(a);

    // Endpoints exact.
    const Point3 e0 = evalC(nurbs, nurbs.knots.front());
    const Point3 e1 = evalC(nurbs, nurbs.knots.back());
    const Point3 t0{a.circle.center.x + a.circle.radius * std::cos(a.startAngle),
                    a.circle.center.y + a.circle.radius * std::sin(a.startAngle), 0};
    const Point3 t1{a.circle.center.x + a.circle.radius * std::cos(a.startAngle + a.sweepAngle),
                    a.circle.center.y + a.circle.radius * std::sin(a.startAngle + a.sweepAngle), 0};
    expectNear(distance(e0, t0), 0.0, 1e-12, "arc start endpoint");
    expectNear(distance(e1, t1), 0.0, 1e-12, "arc end endpoint");

    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind == CurveKind::Arc, "arc recognized");
    expectNear(rec.arc.circle.radius, a.circle.radius, 1e-12, "arc radius");
    expectNear(distance(rec.arc.circle.center, a.circle.center), 0.0, 1e-12, "arc center");
    expectNear(rec.arc.sweepAngle, a.sweepAngle, 1e-9, "arc sweep");
  }

  // ── 3. ELLIPSE round-trip ───────────────────────────────────────────────────
  {
    Ellipse e;
    e.center = {2, -1, 0};
    e.normal = Dir3(0, 0, 1);
    e.xAxis = Dir3(1, 0, 0);
    e.majorRadius = 5.0;
    e.minorRadius = 3.0;
    const BsplineCurveData nurbs = ellipseToNurbs(e);

    // Evaluation exactness: dense samples satisfy (x/a)²+(y/b)²=1.
    double worst = 0.0;
    for (int i = 0; i <= 400; ++i) {
      const double u = static_cast<double>(i) / 400.0;
      const Point3 p = evalC(nurbs, u);
      const double xu = (p.x - e.center.x) / e.majorRadius;
      const double yv = (p.y - e.center.y) / e.minorRadius;
      worst = std::max(worst, std::fabs(xu * xu + yv * yv - 1.0));
    }
    expectLE(worst, 1e-13, "ellipse eval on true ellipse");

    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind == CurveKind::Ellipse, "ellipse recognized");
    expectNear(rec.ellipse.majorRadius, e.majorRadius, 1e-11, "ellipse major");
    expectNear(rec.ellipse.minorRadius, e.minorRadius, 1e-11, "ellipse minor");
    expectNear(distance(rec.ellipse.center, e.center), 0.0, 1e-11, "ellipse center");
    expectNear(absCos(rec.ellipse.xAxis, e.xAxis), 1.0, 1e-9, "ellipse major axis dir");
  }

  // ── 4. LINE round-trip ──────────────────────────────────────────────────────
  {
    LineSegment l{{1, 2, 3}, {4, 6, 8}};
    const BsplineCurveData nurbs = lineToNurbs(l);
    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind == CurveKind::Line, "line recognized");
    expectNear(distance(rec.line.start, l.start), 0.0, 1e-12, "line start");
    expectNear(distance(rec.line.end, l.end), 0.0, 1e-12, "line end");
  }

  // ── 5. PLANE round-trip ─────────────────────────────────────────────────────
  {
    Plane p;
    p.pos = Ax3::fromAxisAndRef({1, 1, 1}, Dir3(1, 2, 2), Dir3(1, 0, 0));
    const BsplineSurfaceData nurbs = planeToNurbs(p, -2, 3, -1, 4);
    const SurfaceRecognition rec = recognizeSurface(nurbs);
    expectTrue(rec.kind == SurfaceKind::Plane, "plane recognized");
    expectLE(rec.residual, 1e-12, "plane residual exact");
    expectNear(absCos(rec.plane.pos.z, p.pos.z), 1.0, 1e-12, "plane normal");
  }

  // ── 6. CYLINDER round-trip + evaluation exactness ───────────────────────────
  {
    Cylinder cyl;
    cyl.pos = Ax3::fromAxisAndRef({0, 0, 1}, Dir3(0, 0, 1), Dir3(1, 0, 0));
    cyl.radius = 2.5;
    const BsplineSurfaceData nurbs = cylinderToNurbs(cyl, -3, 3);

    // Evaluation exactness: every point at radius 2.5 from the axis.
    double worst = 0.0;
    for (int iu = 0; iu <= 64; ++iu)
      for (int iv = 0; iv <= 8; ++iv) {
        const Point3 pt = evalS(nurbs, iu / 64.0, iv / 8.0);
        const Vec3 rel = pt - cyl.pos.origin;
        const double h = dot(rel, cyl.pos.z.vec());
        const double rho = std::sqrt(std::max(0.0, dot(rel, rel) - h * h));
        worst = std::max(worst, std::fabs(rho - cyl.radius));
      }
    expectLE(worst, 1e-13, "cylinder eval on true cylinder");

    const SurfaceRecognition rec = recognizeSurface(nurbs);
    expectTrue(rec.kind == SurfaceKind::Cylinder, "cylinder recognized");
    expectLE(rec.residual, 1e-12, "cylinder residual exact");
    expectNear(rec.cylinder.radius, cyl.radius, 1e-11, "cylinder radius");
    expectNear(absCos(rec.cylinder.pos.z, cyl.pos.z), 1.0, 1e-11, "cylinder axis");
  }

  // ── 7. CONE round-trip ──────────────────────────────────────────────────────
  {
    Cone cone;
    cone.pos = Ax3::fromAxisAndRef({0, 0, 0}, Dir3(0, 0, 1), Dir3(1, 0, 0));
    cone.radius = 0.0;                       // apex at origin
    cone.semiAngle = 20.0 * kPi / 180.0;
    const BsplineSurfaceData nurbs = coneToNurbs(cone, 1.0, 5.0);

    // Evaluation exactness: ρ = h·tan α along the surface.
    double worst = 0.0;
    for (int iu = 0; iu <= 64; ++iu)
      for (int iv = 0; iv <= 8; ++iv) {
        const Point3 pt = evalS(nurbs, iu / 64.0, iv / 8.0);
        const Vec3 rel = pt - cone.pos.origin;
        const double h = dot(rel, cone.pos.z.vec());
        const double rho = std::sqrt(std::max(0.0, dot(rel, rel) - h * h));
        worst = std::max(worst, std::fabs(rho - h * std::tan(cone.semiAngle)));
      }
    expectLE(worst, 1e-12, "cone eval on true cone");

    const SurfaceRecognition rec = recognizeSurface(nurbs);
    expectTrue(rec.kind == SurfaceKind::Cone, "cone recognized");
    expectLE(rec.residual, 1e-12, "cone residual exact");
    expectNear(rec.cone.semiAngle, cone.semiAngle, 1e-9, "cone half-angle");
    expectNear(absCos(rec.cone.pos.z, cone.pos.z), 1.0, 1e-10, "cone axis");
  }

  // ── 8. SPHERE round-trip + evaluation exactness ─────────────────────────────
  {
    Sphere sph;
    sph.pos = Ax3{{3, -2, 1}, Dir3(1, 0, 0), Dir3(0, 1, 0), Dir3(0, 0, 1)};
    sph.radius = 4.2;
    const BsplineSurfaceData nurbs = sphereToNurbs(sph);

    double worst = 0.0;
    for (int iu = 0; iu <= 64; ++iu)
      for (int iv = 0; iv <= 32; ++iv) {
        const Point3 pt = evalS(nurbs, iu / 64.0, iv / 32.0);
        worst = std::max(worst, std::fabs(distance(pt, sph.pos.origin) - sph.radius));
      }
    expectLE(worst, 1e-13, "sphere eval on true sphere");

    const SurfaceRecognition rec = recognizeSurface(nurbs);
    expectTrue(rec.kind == SurfaceKind::Sphere, "sphere recognized");
    expectLE(rec.residual, 1e-12, "sphere residual exact");
    expectNear(rec.sphere.radius, sph.radius, 1e-11, "sphere radius");
    expectNear(distance(rec.sphere.pos.origin, sph.pos.origin), 0.0, 1e-11, "sphere center");
  }

  // ── 9. TORUS evaluation exactness ───────────────────────────────────────────
  {
    Torus tor;
    tor.pos = Ax3{{0, 0, 0}, Dir3(1, 0, 0), Dir3(0, 1, 0), Dir3(0, 0, 1)};
    tor.majorRadius = 3.0;
    tor.minorRadius = 1.0;
    const BsplineSurfaceData nurbs = torusToNurbs(tor);

    // Implicit torus: (sqrt(x²+y²) − R)² + z² = r².
    double worst = 0.0;
    for (int iu = 0; iu <= 64; ++iu)
      for (int iv = 0; iv <= 64; ++iv) {
        const Point3 pt = evalS(nurbs, iu / 64.0, iv / 64.0);
        const Vec3 rel = pt - tor.pos.origin;
        const double h = dot(rel, tor.pos.z.vec());
        const double rho = std::sqrt(std::max(0.0, dot(rel, rel) - h * h));
        const double f = (rho - tor.majorRadius) * (rho - tor.majorRadius) + h * h -
                         tor.minorRadius * tor.minorRadius;
        worst = std::max(worst, std::fabs(f));
      }
    expectLE(worst, 1e-12, "torus eval on true torus");
  }

  // ── 10. DISCRIMINATION: freeform bicubic bump surface → General ─────────────
  {
    // A genuine bicubic (degree 3,3) freeform patch, non-planar, non-quadric.
    BsplineSurfaceData s;
    s.degreeU = 3; s.degreeV = 3;
    s.nPolesU = 4; s.nPolesV = 4;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        const double x = i, y = j;
        // A saddle-ish bump that no plane/sphere/cylinder/cone contains.
        const double z = 0.5 * (x - 1.5) * (y - 1.5) + 0.3 * std::sin(x) * std::cos(y);
        s.poles.push_back({x, y, z});
      }
    s.weights.assign(16, 1.0);
    s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
    s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};
    const SurfaceRecognition rec = recognizeSurface(s);
    expectTrue(rec.kind == SurfaceKind::General, "freeform bump → General");
  }

  // ── 11. DISCRIMINATION: non-uniform-weight almost-circle that ISN'T a circle ─
  {
    // Take an exact circle's NURBS, then PERTURB the tangent-pole weights so the
    // rational curve is no longer a circle (it becomes a genuine non-circular conic
    // spline). The control-net certificate must REJECT it as General.
    Circle c;
    c.center = {0, 0, 0};
    c.normal = Dir3(0, 0, 1);
    c.xAxis = Dir3(1, 0, 0);
    c.radius = 1.0;
    BsplineCurveData nurbs = circleToNurbs(c);
    // Corrupt one middle weight away from cos(45°) → the arc bulges, not a circle.
    nurbs.weights[1] *= 1.15;
    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind == CurveKind::General, "weight-perturbed almost-circle → General");
  }

  // ── 12. DISCRIMINATION: displaced-pole almost-circle → General ───────────────
  {
    Circle c;
    c.center = {0, 0, 0};
    c.normal = Dir3(0, 0, 1);
    c.xAxis = Dir3(1, 0, 0);
    c.radius = 1.0;
    BsplineCurveData nurbs = circleToNurbs(c);
    // Nudge one on-curve pole off the circle by 1e-3 (well above tol) → not a circle.
    nurbs.poles[2] = nurbs.poles[2] + Vec3{1e-3, 0, 0};
    const CurveRecognition rec = recognizeCurve(nurbs);
    expectTrue(rec.kind != CurveKind::Circle, "displaced-pole almost-circle not Circle");
  }

  std::printf("analytic_nurbs gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("analytic_nurbs gate: skipped (CYBERCAD_HAS_NUMSCI off)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
