// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native analytic SSI (Stage S1). OCCT-FREE — Gate 1 (host,
// analytic) of the two-gate model: for every supported pair we assert the returned
// native conic branches satisfy the S1 CORRECTNESS INVARIANT — every sampled point
// on each returned curve lies on BOTH input surfaces within tolerance — plus the
// expected curve KIND / count (cross-checked against the closed-form geometry the
// OCCT oracle IntAna_QuadQuadGeo also produces). NotAnalytic pairs are asserted to
// be reported honestly (no faked curve).
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_ssi.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_ssi
//
#include "native/ssi/native_ssi.h"

#include "harness.h"

#include <cmath>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── on-surface distance oracles (closed-form, per surface kind) ─────────────────

double distToPlane(const nmath::Plane& p, const Point3& x) {
  return std::fabs(nmath::dot(x - p.pos.origin, p.pos.z.vec()));
}
double distToSphere(const nmath::Sphere& s, const Point3& x) {
  return std::fabs(nmath::distance(x, s.pos.origin) - s.radius);
}
double distToCylinder(const nmath::Cylinder& c, const Point3& x) {
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());
  const Vec3 radial = w - c.pos.z.vec() * axial;
  return std::fabs(nmath::norm(radial) - c.radius);
}
double distToCone(const nmath::Cone& c, const Point3& x) {
  // Distance-like residual: |radial| − expected radius at that axial height.
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());  // = v·cosα
  const Vec3 radial = w - c.pos.z.vec() * axial;
  const double v = axial / std::cos(c.semiAngle);
  const double expectedR = c.radius + v * std::sin(c.semiAngle);
  return std::fabs(nmath::norm(radial) - std::fabs(expectedR));
}
double distToTorus(const nmath::Torus& t, const Point3& x) {
  const Vec3 w = x - t.pos.origin;
  const double z = nmath::dot(w, t.pos.z.vec());
  const Vec3 planar = w - t.pos.z.vec() * z;
  const double rho = nmath::norm(planar);           // distance from axis
  // Distance to the tube centreline circle (radius R in the plane), then − r.
  const double dRho = rho - t.majorRadius;
  return std::fabs(std::sqrt(dRho * dRho + z * z) - t.minorRadius);
}

// Sample a curve across its natural range and check `residual(P(t)) ≤ tol` for all t.
template <class Fn>
bool curveLiesOn(const ssi::IntersectionCurve& c, Fn residual, double tol, bool& ok) {
  if (c.kind == ssi::CurveKind::Point) {
    const double d = residual(c.point);
    if (d > tol) { std::printf("  point residual %.3e > %.3e\n", d, tol); return false; }
    return true;
  }
  auto [t0, t1] = c.naturalRange();
  const int N = 64;
  double worst = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / N);
    worst = std::max(worst, residual(c.value(t)));
  }
  if (worst > tol) { std::printf("  curve worst residual %.3e > %.3e (kind %d)\n",
                                 worst, tol, int(c.kind)); }
  if (worst > tol) ok = false;
  return worst <= tol;
}

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

}  // namespace

// ── plane ∩ plane → Line ────────────────────────────────────────────────────────
CC_TEST(plane_plane_line) {
  nmath::Plane p1{frameZ()};                                   // z = 0
  nmath::Plane p2{Ax3{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}};  // x = 0
  auto r = ssi::intersect_surfaces(ssi::Surface::of(p1), ssi::Surface::of(p2));
  CC_CHECK(r.status == ssi::IntersectionStatus::Ok && r.curves.size() == 1);
  CC_CHECK(r.curves[0].kind == ssi::CurveKind::Line);
  bool ok = true;
  curveLiesOn(r.curves[0], [&](const Point3& x) {
    return std::max(distToPlane(p1, x), distToPlane(p2, x)); }, 1e-9, ok);
  CC_CHECK(ok);

  // parallel-disjoint and coincident
  nmath::Plane p3{frameZ({0, 0, 5})};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(p1), ssi::Surface::of(p3)).status ==
           ssi::IntersectionStatus::NoIntersection);
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(p1), ssi::Surface::of(p1)).status ==
           ssi::IntersectionStatus::Coincident);
}

// ── plane ∩ sphere → Circle / Point / Empty ─────────────────────────────────────
CC_TEST(plane_sphere_circle) {
  nmath::Plane pl{frameZ()};                       // z = 0
  nmath::Sphere sp{frameZ({0, 0, 3}), 5.0};        // centre (0,0,3), R=5
  auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(sp));
  CC_CHECK(r.ok_() && r.curves.size() == 1 && r.curves[0].kind == ssi::CurveKind::Circle);
  CC_CHECK(std::fabs(r.curves[0].radius - 4.0) < 1e-9);  // √(25−9)=4
  bool ok = true;
  curveLiesOn(r.curves[0], [&](const Point3& x) {
    return std::max(distToPlane(pl, x), distToSphere(sp, x)); }, 1e-9, ok);
  CC_CHECK(ok);

  // tangent → point; far → empty
  nmath::Sphere tan{frameZ({0, 0, 5}), 5.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(tan)).curves[0].kind ==
           ssi::CurveKind::Point);
  nmath::Sphere far{frameZ({0, 0, 10}), 5.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(far)).status ==
           ssi::IntersectionStatus::NoIntersection);
}

// ── plane ∩ cylinder → circle / ellipse / parallel lines ────────────────────────
CC_TEST(plane_cylinder) {
  nmath::Cylinder cy{frameZ(), 2.0};  // axis Z, R=2

  // (2) ⟂ axis → circle radius 2
  {
    nmath::Plane pl{frameZ({0, 0, 4})};
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(cy));
    CC_CHECK(r.ok_() && r.curves[0].kind == ssi::CurveKind::Circle);
    CC_CHECK(std::fabs(r.curves[0].radius - 2.0) < 1e-9);
    bool ok = true;
    curveLiesOn(r.curves[0], [&](const Point3& x) {
      return std::max(distToPlane(pl, x), distToCylinder(cy, x)); }, 1e-9, ok);
    CC_CHECK(ok);
  }
  // (3) oblique 45° → ellipse, semi-minor 2, semi-major 2/sin45 = 2√2
  {
    const double c = std::cos(kPi / 4), s = std::sin(kPi / 4);
    nmath::Plane pl{Ax3{{0, 0, 0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}};  // normal tilted 45°
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(cy));
    CC_CHECK(r.ok_() && r.curves[0].kind == ssi::CurveKind::Ellipse);
    CC_CHECK(std::fabs(r.curves[0].b - 2.0) < 1e-9);
    CC_CHECK(std::fabs(r.curves[0].a - 2.0 * std::sqrt(2.0)) < 1e-9);
    bool ok = true;
    curveLiesOn(r.curves[0], [&](const Point3& x) {
      return std::max(distToPlane(pl, x), distToCylinder(cy, x)); }, 1e-9, ok);
    CC_CHECK(ok);
  }
  // (1) ∥ axis, plane x=0.5 through the cylinder → 2 rulings at y=±√(4−0.25)
  {
    nmath::Plane pl{Ax3{{0.5, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}};  // x=0.5
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(cy));
    CC_CHECK(r.ok_() && r.curves.size() == 2);
    CC_CHECK(r.curves[0].kind == ssi::CurveKind::Line);
    bool ok = true;
    for (auto& cu : r.curves)
      curveLiesOn(cu, [&](const Point3& x) {
        return std::max(distToPlane(pl, x), distToCylinder(cy, x)); }, 1e-9, ok);
    CC_CHECK(ok);
  }
}

// ── plane ∩ cone → circle / ellipse / parabola / hyperbola ──────────────────────
CC_TEST(plane_cone) {
  nmath::Cone co{frameZ(), 1.0, kPi / 4};  // 45° half-angle, base radius 1 at v=0

  auto onBoth = [&](const nmath::Plane& pl, const Point3& x) {
    return std::max(distToPlane(pl, x), distToCone(co, x));
  };

  // ⟂ axis at z=3 → circle radius = coneR there. v=3/cos45; R=1+v sin45.
  {
    nmath::Plane pl{frameZ({0, 0, 3})};
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(co));
    CC_CHECK(r.ok_() && r.curves[0].kind == ssi::CurveKind::Circle);
    bool ok = true;
    curveLiesOn(r.curves[0], [&](const Point3& x) { return onBoth(pl, x); }, 1e-7, ok);
    CC_CHECK(ok);
  }
  // parabola: plane parallel to a generator. For a 45° cone, a plane tilted 45°
  // (normal at 45° to axis) is parallel to one generator → parabola.
  {
    const double c = std::cos(kPi / 4), s = std::sin(kPi / 4);
    nmath::Plane pl{Ax3{{0, 0, 2}, {1, 0, 0}, {0, c, s}, {0, -s, c}}};
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(co));
    CC_CHECK(r.ok_());
    CC_CHECK(r.curves[0].kind == ssi::CurveKind::Parabola);
    bool ok = true;
    curveLiesOn(r.curves[0], [&](const Point3& x) { return onBoth(pl, x); }, 1e-6, ok);
    CC_CHECK(ok);
  }
  // ellipse: plane tilted 20° (< 45° from horizontal ⇒ cuts all generators).
  {
    const double ang = kPi / 9;  // 20° tilt of the plane from horizontal
    const double c = std::cos(ang), s = std::sin(ang);
    nmath::Plane pl{Ax3{{0, 0, 3}, {1, 0, 0}, {0, c, s}, {0, -s, c}}};
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(co));
    CC_CHECK(r.ok_() && r.curves[0].kind == ssi::CurveKind::Ellipse);
    bool ok = true;
    curveLiesOn(r.curves[0], [&](const Point3& x) { return onBoth(pl, x); }, 1e-6, ok);
    CC_CHECK(ok);
  }
  // hyperbola: near-vertical plane (steeper than the generator) cutting both nappes'
  // region — a plane containing a direction steeper than 45°. Use a vertical plane
  // offset from the axis: x = 0.5.
  {
    nmath::Plane pl{Ax3{{0.5, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}};  // x=0.5, vertical
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(co));
    // A plane cutting a double-napped cone below its half-angle meets BOTH nappes, so
    // the intersection is a hyperbola with TWO branches — two curves (branch = ±1),
    // both on the cone. (Regression: an earlier version emitted only the +X branch,
    // dropping the second nappe's arc that OCCT's GeomAPI_IntSS also reports.)
    CC_CHECK(r.ok_() && r.curves.size() == 2);
    CC_CHECK(r.curves[0].kind == ssi::CurveKind::Hyperbola &&
             r.curves[1].kind == ssi::CurveKind::Hyperbola);
    CC_CHECK(r.curves[0].branch == 1.0 && r.curves[1].branch == -1.0);
    double worst = 0.0;
    // Hyperbola window: cosh/sinh grow fast, keep t small so points stay near the cone.
    for (auto& h : r.curves)
      for (int i = -20; i <= 20; ++i)
        worst = std::max(worst, onBoth(pl, h.value(0.05 * i)));
    CC_CHECK(worst <= 1e-6);
    // The two branches are genuinely distinct (opposite sides of the transverse axis).
    CC_CHECK(nmath::distance(r.curves[0].value(0.0), r.curves[1].value(0.0)) > 1e-6);
  }
}

// ── plane ∩ torus → circles (⟂ axis and axis-containing) ────────────────────────
CC_TEST(plane_torus) {
  nmath::Torus to{frameZ(), 4.0, 1.0};  // R=4, r=1, axis Z

  // ⟂ axis at z=0 → two concentric circles radii 5 and 3.
  {
    nmath::Plane pl{frameZ()};
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(to));
    CC_CHECK(r.ok_() && r.curves.size() == 2);
    bool ok = true;
    for (auto& cu : r.curves)
      curveLiesOn(cu, [&](const Point3& x) {
        return std::max(distToPlane(pl, x), distToTorus(to, x)); }, 1e-9, ok);
    CC_CHECK(ok);
  }
  // plane containing the axis (y=0 plane, normal +Y) → two tube circles radius 1.
  {
    nmath::Plane pl{Ax3{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 1, 0}}};  // normal Y
    auto r = ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(to));
    CC_CHECK(r.ok_() && r.curves.size() == 2);
    CC_CHECK(std::fabs(r.curves[0].radius - 1.0) < 1e-9);
    bool ok = true;
    for (auto& cu : r.curves)
      curveLiesOn(cu, [&](const Point3& x) {
        return std::max(distToPlane(pl, x), distToTorus(to, x)); }, 1e-9, ok);
    CC_CHECK(ok);
  }
  // oblique plane → deferred (NotAnalytic).
  {
    const double c = std::cos(kPi / 5), s = std::sin(kPi / 5);
    nmath::Plane pl{Ax3{{0, 0, 0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}};
    CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(pl), ssi::Surface::of(to)).status ==
             ssi::IntersectionStatus::NotAnalytic);
  }
}

// ── sphere ∩ sphere → Circle / Point / Empty / Coincident ───────────────────────
CC_TEST(sphere_sphere) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 5.0};
  nmath::Sphere s2{frameZ({6, 0, 0}), 5.0};  // d=6 → circle radius 4
  auto r = ssi::intersect_surfaces(ssi::Surface::of(s1), ssi::Surface::of(s2));
  CC_CHECK(r.ok_() && r.curves[0].kind == ssi::CurveKind::Circle);
  CC_CHECK(std::fabs(r.curves[0].radius - 4.0) < 1e-9);
  bool ok = true;
  curveLiesOn(r.curves[0], [&](const Point3& x) {
    return std::max(distToSphere(s1, x), distToSphere(s2, x)); }, 1e-9, ok);
  CC_CHECK(ok);

  nmath::Sphere far{frameZ({20, 0, 0}), 5.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(s1), ssi::Surface::of(far)).status ==
           ssi::IntersectionStatus::NoIntersection);
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(s1), ssi::Surface::of(s1)).status ==
           ssi::IntersectionStatus::Coincident);
  // order independence
  auto rab = ssi::intersect_surfaces(ssi::Surface::of(s1), ssi::Surface::of(s2));
  auto rba = ssi::intersect_surfaces(ssi::Surface::of(s2), ssi::Surface::of(s1));
  CC_CHECK(rab.curves.size() == rba.curves.size());
}

// ── coaxial sphere ∩ cylinder → circles ─────────────────────────────────────────
CC_TEST(sphere_cylinder_coaxial) {
  nmath::Sphere sp{frameZ(), 5.0};
  nmath::Cylinder cy{frameZ(), 3.0};  // coaxial (axis Z through centre), R=3
  auto r = ssi::intersect_surfaces(ssi::Surface::of(sp), ssi::Surface::of(cy));
  CC_CHECK(r.ok_() && r.curves.size() == 2);  // z=±4
  bool ok = true;
  for (auto& cu : r.curves)
    curveLiesOn(cu, [&](const Point3& x) {
      return std::max(distToSphere(sp, x), distToCylinder(cy, x)); }, 1e-9, ok);
  CC_CHECK(ok);

  // non-coaxial → NotAnalytic
  nmath::Cylinder off{frameZ({2, 0, 0}), 3.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(sp), ssi::Surface::of(off)).status ==
           ssi::IntersectionStatus::NotAnalytic);
}

// ── coaxial sphere ∩ cone → circles ─────────────────────────────────────────────
CC_TEST(sphere_cone_coaxial) {
  nmath::Sphere sp{frameZ(), 5.0};
  nmath::Cone co{frameZ(), 0.0, kPi / 4};  // apex at origin, 45°, coaxial
  auto r = ssi::intersect_surfaces(ssi::Surface::of(sp), ssi::Surface::of(co));
  // Apex-at-centre cone is double-napped and the sphere spans it → TWO circles, one
  // per nappe at z = ±√(Rs²/2). (Regression: an earlier version skipped the
  // negative-signed-radius root and returned only the +z nappe circle.)
  CC_CHECK(r.ok_() && r.curves.size() == 2);
  CC_CHECK(r.curves[0].frame.origin.z * r.curves[1].frame.origin.z < 0.0);  // opposite nappes
  bool ok = true;
  for (auto& cu : r.curves)
    curveLiesOn(cu, [&](const Point3& x) {
      return std::max(distToSphere(sp, x), distToCone(co, x)); }, 1e-7, ok);
  CC_CHECK(ok);
}

// ── cylinder ∩ cylinder (coaxial / parallel / skew) ─────────────────────────────
CC_TEST(cylinder_cylinder) {
  nmath::Cylinder c1{frameZ(), 3.0};
  // coaxial same radius → Coincident
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(c1), ssi::Surface::of(c1)).status ==
           ssi::IntersectionStatus::Coincident);
  // coaxial different radius → Empty
  nmath::Cylinder c2{frameZ(), 5.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(c1), ssi::Surface::of(c2)).status ==
           ssi::IntersectionStatus::NoIntersection);
  // parallel, offset 4, radii 3 & 3 → 2 rulings
  nmath::Cylinder c3{frameZ({4, 0, 0}), 3.0};
  auto r = ssi::intersect_surfaces(ssi::Surface::of(c1), ssi::Surface::of(c3));
  CC_CHECK(r.ok_() && r.curves.size() == 2 && r.curves[0].kind == ssi::CurveKind::Line);
  bool ok = true;
  for (auto& cu : r.curves)
    curveLiesOn(cu, [&](const Point3& x) {
      return std::max(distToCylinder(c1, x), distToCylinder(c3, x)); }, 1e-9, ok);
  CC_CHECK(ok);
  // skew → NotAnalytic
  nmath::Cylinder skew{Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, 3.0};  // axis X
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(c1), ssi::Surface::of(skew)).status ==
           ssi::IntersectionStatus::NotAnalytic);
}

// ── coaxial cylinder ∩ cone → circle(s) ─────────────────────────────────────────
CC_TEST(cylinder_cone_coaxial) {
  nmath::Cylinder cy{frameZ(), 2.0};
  nmath::Cone co{frameZ(), 0.0, kPi / 4};  // apex at origin, coaxial
  auto r = ssi::intersect_surfaces(ssi::Surface::of(cy), ssi::Surface::of(co));
  // Apex-at-origin cone is double-napped and the coaxial cylinder meets both nappes →
  // TWO circles of radius R_cyl at z = ±R_cyl (45°). (Regression: an earlier version
  // skipped the −R_cyl target and returned only the +z nappe circle.)
  CC_CHECK(r.ok_() && r.curves.size() == 2 && r.curves[0].kind == ssi::CurveKind::Circle);
  CC_CHECK(r.curves[1].kind == ssi::CurveKind::Circle);
  CC_CHECK(std::fabs(r.curves[0].radius - 2.0) < 1e-9 &&
           std::fabs(r.curves[1].radius - 2.0) < 1e-9);
  CC_CHECK(r.curves[0].frame.origin.z * r.curves[1].frame.origin.z < 0.0);  // opposite nappes
  bool ok = true;
  for (auto& cu : r.curves)
    curveLiesOn(cu, [&](const Point3& x) {
      return std::max(distToCylinder(cy, x), distToCone(co, x)); }, 1e-7, ok);
  CC_CHECK(ok);
}

// ── deferred families report NotAnalytic (never faked) ──────────────────────────
CC_TEST(deferred_not_analytic) {
  nmath::Cone c1{frameZ(), 1.0, kPi / 6};
  nmath::Cone c2{frameZ({3, 0, 0}), 1.0, kPi / 6};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(c1), ssi::Surface::of(c2)).status ==
           ssi::IntersectionStatus::NotAnalytic);  // general cone∩cone
  nmath::Torus t1{frameZ(), 4.0, 1.0};
  nmath::Sphere sp{frameZ({0, 0, 0}), 5.0};
  CC_CHECK(ssi::intersect_surfaces(ssi::Surface::of(sp), ssi::Surface::of(t1)).status ==
           ssi::IntersectionStatus::NotAnalytic);  // sphere∩torus
}

int main() { return cctest::run_all(); }
