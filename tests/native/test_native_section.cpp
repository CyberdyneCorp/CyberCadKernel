// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) — OCCT-FREE closed-form proof of the native planar SECTION-CURVES
// service (MOAT M-GS GS2, src/native/section). For box / cylinder / sphere / cone
// primitives we assert the section loops:
//   * LIE on the cut plane and on the solid's faces (self-verify passes);
//   * CLOSE into the expected number of loops;
//   * enclose an area that matches the CLOSED-FORM value (box rectangle, cylinder
//     cross-section circle πR², cylinder axial rectangle 2R·H, sphere great/small
//     circle πr²) and have the closed-form perimeter length.
// Plus the HONEST DECLINES: an oblique cut of a cylindrical face (upstream ssi
// oblique-ellipse defect) and a cut plane coincident with a planar face.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_section.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_section && ./test_native_section
//
#include "native/section/native_section.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace sec = cybercad::native::section;
namespace topo = cybercad::native::topology;
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

bool near(double a, double b, double tol = 1e-7) { return std::fabs(a - b) <= tol; }

// ── fixture builders ─────────────────────────────────────────────────────────

Shape lineEdge(const Point3& a, const Point3& b) {
  const Vec3 d = b - a;
  const double len = nmath::norm(d);
  EdgeCurve c;
  c.kind = EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{d}, Dir3{}, Dir3{}};
  Shape v0 = ShapeBuilder::makeVertex(a);
  Shape v1 = ShapeBuilder::makeVertex(b);
  return ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);
}

Shape circleEdge(const Point3& center, double r, const Dir3& normal, const Dir3& xref) {
  EdgeCurve c;
  c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(center, normal, xref);
  c.radius = r;
  return ShapeBuilder::makeEdge(c, 0.0, 2.0 * kPi, Shape{}, Shape{});
}

// A planar quad face p0→p1→p2→p3 with outward `normal`.
Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& normal) {
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, normal, Dir3{p1 - p0});
  std::vector<Shape> edges = {lineEdge(p0, p1), lineEdge(p1, p2), lineEdge(p2, p3),
                              lineEdge(p3, p0)};
  return ShapeBuilder::makeFace(s, ShapeBuilder::makeWire(std::move(edges)));
}

// Axis-aligned box [0,lx]×[0,ly]×[0,lz], 6 planar faces with outward normals.
Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0, 0, 0}, b{lx, 0, 0}, c{lx, ly, 0}, d{0, ly, 0};
  const Point3 e{0, 0, lz}, f{lx, 0, lz}, g{lx, ly, lz}, h{0, ly, lz};
  std::vector<Shape> faces;
  faces.push_back(planarQuad(a, d, c, b, Dir3{0, 0, -1}));  // z=0  (−Z)
  faces.push_back(planarQuad(e, f, g, h, Dir3{0, 0, 1}));   // z=lz (+Z)
  faces.push_back(planarQuad(a, b, f, e, Dir3{0, -1, 0}));  // y=0  (−Y)
  faces.push_back(planarQuad(d, h, g, c, Dir3{0, 1, 0}));   // y=ly (+Y)
  faces.push_back(planarQuad(a, e, h, d, Dir3{-1, 0, 0}));  // x=0  (−X)
  faces.push_back(planarQuad(b, c, g, f, Dir3{1, 0, 0}));   // x=lx (+X)
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell(std::move(faces))});
}

// Full cylinder radius R, height H along +Z, base at z=0. Two planar cap disks +
// one cylindrical lateral face bounded by the two rim circles.
Shape makeCylinder(double R, double H) {
  const Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  // Bottom cap (−Z outward): wire = bottom rim circle.
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(
      bs, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, 0}, R, zp, xp)}));
  // Top cap (+Z outward): wire = top rim circle.
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(
      ts, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, H}, R, zp, xp)}));
  // Lateral cylinder face: wire = bottom rim + top rim circles.
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, zp, xp);
  ls.radius = R;
  Shape lateral = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, 0}, R, zp, xp),
                                  circleEdge(Point3{0, 0, H}, R, zp, xp)}));
  return ShapeBuilder::makeSolid(
      {ShapeBuilder::makeShell({bottom, top, lateral})});
}

// A full sphere radius R centred at origin, as a single Sphere face (pole-only
// wire; the seam is a parametric artifact and does not trim a section circle).
Shape makeSphere(double R) {
  FaceSurface s; s.kind = FaceSurface::Kind::Sphere;
  s.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0});
  s.radius = R;
  Shape wire = ShapeBuilder::makeWire({});  // no trimming boundary
  return ShapeBuilder::makeSolid(
      {ShapeBuilder::makeShell({ShapeBuilder::makeFace(s, wire)})});
}

// A frustum: Cone lateral face with base radius R0 at z=0 growing to R0+H·tanα at
// z=H (semiAngle α), closed by two planar cap disks.
Shape makeCone(double R0, double H, double semiAngle) {
  const Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  const double Rtop = R0 + H * std::tan(semiAngle);
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(
      bs, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, 0}, R0, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(
      ts, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, H}, Rtop, zp, xp)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cone;
  ls.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, zp, xp);
  ls.radius = R0; ls.semiAngle = semiAngle;
  Shape lateral = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({circleEdge(Point3{0, 0, 0}, R0, zp, xp),
                                  circleEdge(Point3{0, 0, H}, Rtop, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lateral})});
}

Dir3 orthoX(const Dir3& n) {  // any in-plane X for a plane with normal n
  const double ax = std::fabs(n.x()), ay = std::fabs(n.y()), az = std::fabs(n.z());
  Vec3 pick = (ax <= ay && ax <= az) ? Vec3{1, 0, 0} : (ay <= az) ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
  return Dir3{nmath::cross(n.vec(), pick)};
}

// A cut plane through `o` with unit normal `n` (X/Y complete a frame).
nmath::Plane cutPlane(const Point3& o, const Dir3& n) {
  return nmath::Plane{Ax3::fromAxisAndRef(o, n, orthoX(n))};
}

}  // namespace

// ── GATE (a) tests ─────────────────────────────────────────────────────────────

CC_TEST(box_axial_section_is_rectangle_48) {
  const Shape box = makeBox(10, 8, 6);
  const auto r = sec::sectionByPlane(box, cutPlane(Point3{5, 0, 0}, Dir3{1, 0, 0}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  CC_CHECK(near(r.totalArea(), 8.0 * 6.0));          // 48
  CC_CHECK(near(r.totalLength(), 2.0 * (8.0 + 6.0)));  // rectangle perimeter 28
}

CC_TEST(box_section_z_is_rectangle_80) {
  const Shape box = makeBox(10, 8, 6);
  const auto r = sec::sectionByPlane(box, cutPlane(Point3{0, 0, 3}, Dir3{0, 0, 1}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  CC_CHECK(near(r.totalArea(), 10.0 * 8.0));  // 80
}

CC_TEST(cylinder_cross_section_is_circle_piR2) {
  const double R = 3.0, H = 10.0;
  const Shape cyl = makeCylinder(R, H);
  const auto r = sec::sectionByPlane(cyl, cutPlane(Point3{0, 0, 5}, Dir3{0, 0, 1}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  if (r.loops.empty()) return;
  CC_CHECK(r.loops[0].shape == sec::LoopShape::Circle);
  CC_CHECK(near(r.loops[0].radius, R));
  CC_CHECK(near(r.totalArea(), kPi * R * R));
  CC_CHECK(near(r.totalLength(), 2.0 * kPi * R));
}

CC_TEST(cylinder_cross_section_outside_height_is_empty) {
  const Shape cyl = makeCylinder(3.0, 10.0);
  const auto r = sec::sectionByPlane(cyl, cutPlane(Point3{0, 0, 12}, Dir3{0, 0, 1}));
  CC_CHECK(r.status == sec::SectionStatus::Empty);
}

CC_TEST(cylinder_axial_section_is_rectangle_2RH) {
  const double R = 3.0, H = 10.0;
  const Shape cyl = makeCylinder(R, H);
  const auto r = sec::sectionByPlane(cyl, cutPlane(Point3{0, 0, 0}, Dir3{0, 1, 0}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  CC_CHECK(near(r.totalArea(), 2.0 * R * H));  // 60
}

CC_TEST(cylinder_oblique_cut_is_declined) {
  const Shape cyl = makeCylinder(3.0, 10.0);
  // Normal tilted between axis (+Z) and radial (+Y): oblique to the cylinder.
  const auto r = sec::sectionByPlane(cyl, cutPlane(Point3{0, 0, 5}, Dir3{0, 1, 1}));
  CC_CHECK(r.status == sec::SectionStatus::Declined);
}

CC_TEST(sphere_great_circle_area_piR2) {
  const double R = 5.0;
  const Shape sph = makeSphere(R);
  const auto r = sec::sectionByPlane(sph, cutPlane(Point3{0, 0, 0}, Dir3{0, 0, 1}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  if (r.loops.empty()) return;
  CC_CHECK(r.loops[0].shape == sec::LoopShape::Circle);
  CC_CHECK(near(r.loops[0].radius, R));
  CC_CHECK(near(r.totalArea(), kPi * R * R));
}

CC_TEST(sphere_offset_circle_radius4) {
  const double R = 5.0;
  const Shape sph = makeSphere(R);
  const auto r = sec::sectionByPlane(sph, cutPlane(Point3{0, 0, 3}, Dir3{0, 0, 1}));
  CC_CHECK(r.ok());
  CC_CHECK(near(r.loops[0].radius, 4.0));  // √(25−9)
  CC_CHECK(near(r.totalArea(), kPi * 16.0));
}

CC_TEST(sphere_plane_beyond_radius_is_empty) {
  const Shape sph = makeSphere(5.0);
  const auto r = sec::sectionByPlane(sph, cutPlane(Point3{0, 0, 6}, Dir3{0, 0, 1}));
  CC_CHECK(r.status == sec::SectionStatus::Empty);
}

CC_TEST(coincident_plane_face_is_declined) {
  const Shape box = makeBox(10, 8, 6);
  // Plane exactly on the z=0 face → coincident with a planar face.
  const auto r = sec::sectionByPlane(box, cutPlane(Point3{0, 0, 0}, Dir3{0, 0, 1}));
  CC_CHECK(r.status == sec::SectionStatus::Declined);
}

CC_TEST(cone_cross_section_is_circle) {
  // Base radius 2 at z=0, semiAngle atan(0.5): r(z)=2+0.5z ⇒ at z=2, r=3.
  const Shape cone = makeCone(2.0, 4.0, std::atan(0.5));
  const auto r = sec::sectionByPlane(cone, cutPlane(Point3{0, 0, 2}, Dir3{0, 0, 1}));
  CC_CHECK(r.ok());
  CC_CHECK_EQ(r.loopCount(), 1);
  if (r.loops.empty()) return;
  CC_CHECK(r.loops[0].shape == sec::LoopShape::Circle);
  CC_CHECK(near(r.loops[0].radius, 3.0, 1e-6));
  CC_CHECK(near(r.totalArea(), kPi * 9.0, 1e-5));
}

CC_RUN_ALL()
