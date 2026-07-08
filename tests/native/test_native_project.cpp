// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M-DM DM4 — the native `project_point_on_face`
// (src/native/directmodel/project.h), OCCT-FREE. Drop a 3-D point onto a face's
// analytic surface and read the CLOSED-FORM foot-of-perpendicular + distance. On
// plane / cylinder / sphere fixtures (built directly as B-rep faces) we assert, with
// no OCCT, that the foot and distance equal the closed form:
//   * PLANE     — foot = P − ((P−o)·n̂)n̂ (drop the normal component);
//   * CYLINDER  — radial push to the radius (foot on the ρ = R circle at the same z);
//   * SPHERE    — radial push to the radius (foot on the R-sphere along O→P).
// Plus the HONEST-DECLINE envelope (nullopt, a measured reason):
//   * a CONE face (out of the analytic slice);
//   * an AMBIGUOUS pose — a point on the cylinder axis / at the sphere centre;
//   * an out-of-range face id (foreign body).
// Pure analytic geometry — no NUMSCI substrate (always compiled).
//
#include "native/directmodel/project.h"

#include "harness.h"

#include <cmath>
#include <optional>
#include <vector>

namespace dm    = cybercad::native::directmodel;
namespace topo  = cybercad::native::topology;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::Shape;
using topo::ShapeBuilder;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool nearPt(const dm::ProjectionResult& r, double x, double y, double z, double tol = 1e-9) {
  return near(r.foot.x, x, tol) && near(r.foot.y, y, tol) && near(r.foot.z, z, tol);
}

// ── fixture builders (mirror test_native_section.cpp) ────────────────────────────
Shape lineEdge(const Point3& a, const Point3& b) {
  EdgeCurve c; c.kind = EdgeCurve::Kind::Line; c.frame = Ax3{a, Dir3{b - a}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nmath::norm(b - a), ShapeBuilder::makeVertex(a),
                                ShapeBuilder::makeVertex(b));
}
Shape circleEdge(const Point3& center, double r, const Dir3& normal, const Dir3& xref) {
  EdgeCurve c; c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(center, normal, xref); c.radius = r;
  return ShapeBuilder::makeEdge(c, 0.0, 2.0 * kPi, Shape{}, Shape{});
}
Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& normal) {
  FaceSurface s; s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, normal, Dir3{p1 - p0});
  return ShapeBuilder::makeFace(
      s, ShapeBuilder::makeWire({lineEdge(p0, p1), lineEdge(p1, p2), lineEdge(p2, p3),
                                 lineEdge(p3, p0)}));
}
Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0, 0, 0}, b{lx, 0, 0}, c{lx, ly, 0}, d{0, ly, 0};
  const Point3 e{0, 0, lz}, f{lx, 0, lz}, g{lx, ly, lz}, h{0, ly, lz};
  std::vector<Shape> faces = {planarQuad(a, d, c, b, Dir3{0, 0, -1}),
                              planarQuad(e, f, g, h, Dir3{0, 0, 1}),
                              planarQuad(a, b, f, e, Dir3{0, -1, 0}),
                              planarQuad(d, h, g, c, Dir3{0, 1, 0}),
                              planarQuad(a, e, h, d, Dir3{-1, 0, 0}),
                              planarQuad(b, c, g, f, Dir3{1, 0, 0})};  // face 6 = +X (x=lx)
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell(std::move(faces))});
}
Shape makeCylinder(double R, double H) {  // lateral cylinder = face 3
  const Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge({0, 0, H}, R, zp, xp)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, zp, xp); ls.radius = R;
  Shape lateral = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R, zp, xp), circleEdge({0, 0, H}, R, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lateral})});
}
Shape makeSphere(double R) {  // single sphere face = face 1
  FaceSurface s; s.kind = FaceSurface::Kind::Sphere;
  s.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}); s.radius = R;
  return ShapeBuilder::makeSolid(
      {ShapeBuilder::makeShell({ShapeBuilder::makeFace(s, ShapeBuilder::makeWire({}))})});
}
Shape makeCone(double R0, double H, double semiAngle) {  // cone lateral = face 3
  const Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  const double Rtop = R0 + H * std::tan(semiAngle);
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R0, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge({0, 0, H}, Rtop, zp, xp)}));
  FaceSurface cs; cs.kind = FaceSurface::Kind::Cone;
  cs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, zp, xp); cs.radius = R0; cs.semiAngle = semiAngle;
  Shape lateral = ShapeBuilder::makeFace(
      cs, ShapeBuilder::makeWire({circleEdge({0, 0, 0}, R0, zp, xp), circleEdge({0, 0, H}, Rtop, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lateral})});
}

}  // namespace

// ── PLANE: drop the normal component. Box +X face (x=10); P=(15,3,4) → (10,3,4), d=5. ──
CC_TEST(plane_foot_closed_form) {
  const Shape box = makeBox(10, 10, 10);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(box, /*faceId=*/6, Point3{15, 3, 4}, &why);
  CC_CHECK(r.has_value());
  CC_CHECK(why == dm::ProjectDecline::Ok);
  CC_CHECK(nearPt(*r, 10, 3, 4));
  CC_CHECK(near(r->distance, 5.0));
}

// ── PLANE (point already on the face): foot == P, distance 0. ────────────────────
CC_TEST(plane_point_on_face_zero_distance) {
  const Shape box = makeBox(10, 10, 10);
  const auto r = dm::projectPointOnFace(box, 6, Point3{10, 2, 7});
  CC_CHECK(r.has_value());
  CC_CHECK(nearPt(*r, 10, 2, 7));
  CC_CHECK(near(r->distance, 0.0));
}

// ── CYLINDER: radial push to R. R=5, axis +Z; P=(8,0,12) → (5,0,12), d=3. ────────
CC_TEST(cylinder_foot_closed_form) {
  const Shape cyl = makeCylinder(5, 20);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(cyl, /*faceId=*/3, Point3{8, 0, 12}, &why);
  CC_CHECK(r.has_value());
  CC_CHECK(why == dm::ProjectDecline::Ok);
  CC_CHECK(nearPt(*r, 5, 0, 12));
  CC_CHECK(near(r->distance, 3.0));
}

// ── CYLINDER interior point: 3-4-5 radial. P=(3,4,6), ρ=5 → foot on R=5 at same z,
// distance |5−5| = 0 (P is ON the cylinder). Off-axis direction preserved. ─────────
CC_TEST(cylinder_foot_off_axis_direction) {
  const Shape cyl = makeCylinder(5, 20);
  const auto r = dm::projectPointOnFace(cyl, 3, Point3{3, 4, 6});  // ρ = 5 = R
  CC_CHECK(r.has_value());
  CC_CHECK(nearPt(*r, 3, 4, 6, 1e-9));
  CC_CHECK(near(r->distance, 0.0));
}

// ── SPHERE: radial push to R. R=5 at O; P=(0,0,8) → (0,0,5), d=3; P=(6,8,0)→(3,4,0),d=5. ──
CC_TEST(sphere_foot_closed_form) {
  const Shape sph = makeSphere(5);
  const auto r1 = dm::projectPointOnFace(sph, 1, Point3{0, 0, 8});
  CC_CHECK(r1.has_value());
  CC_CHECK(nearPt(*r1, 0, 0, 5));
  CC_CHECK(near(r1->distance, 3.0));
  const auto r2 = dm::projectPointOnFace(sph, 1, Point3{6, 8, 0});  // ρ = 10
  CC_CHECK(r2.has_value());
  CC_CHECK(nearPt(*r2, 3, 4, 0));
  CC_CHECK(near(r2->distance, 5.0));
}

// ── HONEST DECLINE: a CONE face is out of the analytic slice. ────────────────────
CC_TEST(cone_declines_non_analytic) {
  const Shape cone = makeCone(4, 10, 0.3);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(cone, /*faceId=*/3, Point3{9, 0, 5}, &why);
  CC_CHECK(!r.has_value());
  CC_CHECK(why == dm::ProjectDecline::NonAnalyticFace);
}

// ── HONEST DECLINE: a point ON the cylinder axis — the foot is a whole circle. ──
CC_TEST(cylinder_axis_point_ambiguous) {
  const Shape cyl = makeCylinder(5, 20);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(cyl, 3, Point3{0, 0, 10}, &why);
  CC_CHECK(!r.has_value());
  CC_CHECK(why == dm::ProjectDecline::Ambiguous);
}

// ── HONEST DECLINE: a point AT the sphere centre — the foot is the whole sphere. ──
CC_TEST(sphere_centre_point_ambiguous) {
  const Shape sph = makeSphere(5);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(sph, 1, Point3{0, 0, 0}, &why);
  CC_CHECK(!r.has_value());
  CC_CHECK(why == dm::ProjectDecline::Ambiguous);
}

// ── HONEST DECLINE: an out-of-range face id (foreign / no such face). ────────────
CC_TEST(foreign_face_id_declines) {
  const Shape box = makeBox(10, 10, 10);
  dm::ProjectDecline why = dm::ProjectDecline::Ok;
  const auto r = dm::projectPointOnFace(box, /*faceId=*/99, Point3{1, 1, 1}, &why);
  CC_CHECK(!r.has_value());
  CC_CHECK(why == dm::ProjectDecline::ForeignBody);
}

CC_RUN_ALL()
