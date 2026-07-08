// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) — OCCT-FREE closed-form proof of the native REFERENCE / TOPOLOGY
// reads (MOAT M-REF, src/native/reference). Every datum is asserted against a
// hand-computed closed-form value to machine precision:
//   * refPlaneFromFace  — a box face's outward datum plane (normal + on-plane origin)
//   * faceAxis / refAxisFromFace — a cylinder face's axis; a planar face DECLINES
//   * refAxisFromEdge   — a straight edge's axis; a circular edge DECLINES
//   * tangentChain      — growth across a C1 (collinear / line-tangent-arc) joint,
//                         and NO growth across a 90° corner
//   * outerRimChain     — the outer-wire edge ids of the seed's planar cap
//   * offsetFaceBoundary— a rectangle offset inward (exact inner rectangle); a
//                         growing convex offset DECLINES (OCCT would arc-round)
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_reference.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o t && ./t
//
#include "native/reference/reference.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ref = cybercad::native::reference;
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

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

// ── fixture builders ─────────────────────────────────────────────────────────
Shape lineEdge(const Shape& v0, const Shape& v1) {
  const Point3 a = *topo::pointOf(v0), b = *topo::pointOf(v1);
  const Vec3 d = b - a;
  EdgeCurve c;
  c.kind = EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{d}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nmath::norm(d), v0, v1);
}

// A WELDED planar quad face p0→p1→p2→p3 (shared vertices/edges), outward `normal`.
Shape weldedQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& normal) {
  Shape v0 = ShapeBuilder::makeVertex(p0), v1 = ShapeBuilder::makeVertex(p1);
  Shape v2 = ShapeBuilder::makeVertex(p2), v3 = ShapeBuilder::makeVertex(p3);
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, normal, Dir3{p1 - p0});
  Shape wire = ShapeBuilder::makeWire(
      {lineEdge(v0, v1), lineEdge(v1, v2), lineEdge(v2, v3), lineEdge(v3, v0)});
  return ShapeBuilder::makeFace(s, wire);
}

Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& normal) {
  Shape v0 = ShapeBuilder::makeVertex(p0), v1 = ShapeBuilder::makeVertex(p1);
  Shape v2 = ShapeBuilder::makeVertex(p2), v3 = ShapeBuilder::makeVertex(p3);
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, normal, Dir3{p1 - p0});
  Shape wire = ShapeBuilder::makeWire(
      {lineEdge(v0, v1), lineEdge(v1, v2), lineEdge(v2, v3), lineEdge(v3, v0)});
  return ShapeBuilder::makeFace(s, wire);
}

Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0, 0, 0}, b{lx, 0, 0}, c{lx, ly, 0}, d{0, ly, 0};
  const Point3 e{0, 0, lz}, f{lx, 0, lz}, g{lx, ly, lz}, h{0, ly, lz};
  std::vector<Shape> faces;
  faces.push_back(planarQuad(a, d, c, b, Dir3{0, 0, -1}));  // face1 z=0  (−Z)
  faces.push_back(planarQuad(e, f, g, h, Dir3{0, 0, 1}));   // face2 z=lz (+Z)
  faces.push_back(planarQuad(a, b, f, e, Dir3{0, -1, 0}));  // face3 y=0  (−Y)
  faces.push_back(planarQuad(d, h, g, c, Dir3{0, 1, 0}));   // face4 y=ly (+Y)
  faces.push_back(planarQuad(a, e, h, d, Dir3{-1, 0, 0}));  // face5 x=0  (−X)
  faces.push_back(planarQuad(b, c, g, f, Dir3{1, 0, 0}));   // face6 x=lx (+X)
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell(std::move(faces))});
}

Shape circleEdge(const Shape& v0, const Shape& v1, const Point3& center, double r,
                 const Dir3& normal, const Dir3& xref, double t0, double t1) {
  EdgeCurve c;
  c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(center, normal, xref);
  c.radius = r;
  return ShapeBuilder::makeEdge(c, t0, t1, v0, v1);
}

Shape makeCylinder(double R, double H) {
  const Dir3 zp{0, 0, 1}, xp{1, 0, 0};
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, xp);
  Shape bottom = ShapeBuilder::makeFace(
      bs, ShapeBuilder::makeWire({circleEdge(Shape{}, Shape{}, Point3{0, 0, 0}, R, zp, xp, 0, 2 * kPi)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(
      ts, ShapeBuilder::makeWire({circleEdge(Shape{}, Shape{}, Point3{0, 0, H}, R, zp, xp, 0, 2 * kPi)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, zp, xp);
  ls.radius = R;
  Shape lateral = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({circleEdge(Shape{}, Shape{}, Point3{0, 0, 0}, R, zp, xp, 0, 2 * kPi),
                                  circleEdge(Shape{}, Shape{}, Point3{0, 0, H}, R, zp, xp, 0, 2 * kPi)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lateral})});
}

Shape faceOf(const Shape& body, int id) {
  return topo::mapShapes(body, topo::ShapeType::Face).shape(id);
}
Shape edgeOf(const Shape& body, int id) {
  return topo::mapShapes(body, topo::ShapeType::Edge).shape(id);
}

}  // namespace

// ── refPlaneFromFace ───────────────────────────────────────────────────────────
CC_TEST(ref_plane_from_box_faces) {
  const Shape box = makeBox(2, 3, 4);
  // face1: z=0, outward −Z; origin = outer-wire centroid = (1, 1.5, 0).
  auto p1 = ref::refPlaneFromFace(faceOf(box, 1));
  CC_CHECK(p1.has_value());
  CC_CHECK(near((*p1)[0], 1.0) && near((*p1)[1], 1.5) && near((*p1)[2], 0.0));
  CC_CHECK(near((*p1)[3], 0.0) && near((*p1)[4], 0.0) && near((*p1)[5], -1.0));
  // face2: z=4, outward +Z; origin z=4.
  auto p2 = ref::refPlaneFromFace(faceOf(box, 2));
  CC_CHECK(p2.has_value());
  CC_CHECK(near((*p2)[2], 4.0));
  CC_CHECK(near((*p2)[3], 0.0) && near((*p2)[4], 0.0) && near((*p2)[5], 1.0));
  // face6: x=2, outward +X.
  auto p6 = ref::refPlaneFromFace(faceOf(box, 6));
  CC_CHECK(p6.has_value());
  CC_CHECK(near((*p6)[0], 2.0));
  CC_CHECK(near((*p6)[3], 1.0) && near((*p6)[4], 0.0) && near((*p6)[5], 0.0));
}

CC_TEST(ref_plane_declines_non_planar) {
  const Shape cyl = makeCylinder(1.5, 5.0);
  // face3 = lateral cylinder → no datum plane.
  CC_CHECK(!ref::refPlaneFromFace(faceOf(cyl, 3)).has_value());
}

// ── faceAxis / refAxisFromFace ───────────────────────────────────────────────
CC_TEST(face_axis_cylinder) {
  const Shape cyl = makeCylinder(1.5, 5.0);
  auto ax = ref::faceAxis(faceOf(cyl, 3));  // lateral face
  CC_CHECK(ax.has_value());
  CC_CHECK(near((*ax)[0], 0.0) && near((*ax)[1], 0.0) && near((*ax)[2], 0.0));
  CC_CHECK(near((*ax)[3], 0.0) && near((*ax)[4], 0.0) && near((*ax)[5], 1.0));
  // refAxisFromFace is identical.
  auto ax2 = ref::refAxisFromFace(faceOf(cyl, 3));
  CC_CHECK(ax2.has_value() && *ax == *ax2);
  // A planar cap face has no axis.
  CC_CHECK(!ref::faceAxis(faceOf(cyl, 1)).has_value());
  CC_CHECK(!ref::refAxisFromFace(faceOf(cyl, 1)).has_value());
}

CC_TEST(face_axis_declines_planar_box) {
  const Shape box = makeBox(2, 3, 4);
  CC_CHECK(!ref::faceAxis(faceOf(box, 1)).has_value());
  CC_CHECK(!ref::refAxisFromFace(faceOf(box, 1)).has_value());
}

// ── refAxisFromEdge ──────────────────────────────────────────────────────────
CC_TEST(ref_axis_from_line_edge) {
  // A single straight edge (0,0,0)→(3,0,0): axis origin (0,0,0), dir +X.
  Shape v0 = ShapeBuilder::makeVertex({0, 0, 0}), v1 = ShapeBuilder::makeVertex({3, 0, 0});
  Shape e = lineEdge(v0, v1);
  auto ax = ref::refAxisFromEdge(e);
  CC_CHECK(ax.has_value());
  CC_CHECK(near((*ax)[0], 0.0) && near((*ax)[1], 0.0) && near((*ax)[2], 0.0));
  CC_CHECK(near((*ax)[3], 1.0) && near((*ax)[4], 0.0) && near((*ax)[5], 0.0));
}

CC_TEST(ref_axis_declines_circular_edge) {
  Shape e = circleEdge(Shape{}, Shape{}, Point3{0, 0, 0}, 2.0, Dir3{0, 0, 1}, Dir3{1, 0, 0},
                       0, 2 * kPi);
  CC_CHECK(!ref::refAxisFromEdge(e).has_value());  // OCCT gp_Lin only → decline
}

// ── tangentChain ─────────────────────────────────────────────────────────────
CC_TEST(tangent_chain_grows_collinear) {
  // Two collinear line edges sharing a vertex → C1 → chain grows to both.
  Shape v0 = ShapeBuilder::makeVertex({0, 0, 0});
  Shape v1 = ShapeBuilder::makeVertex({1, 0, 0});
  Shape v2 = ShapeBuilder::makeVertex({2, 0, 0});
  Shape e0 = lineEdge(v0, v1), e1 = lineEdge(v1, v2);
  Shape wire = ShapeBuilder::makeWire({e0, e1});
  auto chain = ref::tangentChain(wire, {1});
  CC_CHECK(chain.has_value());
  CC_CHECK(chain->size() == 2);  // {1,2}
}

CC_TEST(tangent_chain_stops_at_corner) {
  // Two perpendicular edges → not C1 → chain = seed only.
  Shape v0 = ShapeBuilder::makeVertex({0, 0, 0});
  Shape v1 = ShapeBuilder::makeVertex({1, 0, 0});
  Shape v2 = ShapeBuilder::makeVertex({1, 1, 0});
  Shape wire = ShapeBuilder::makeWire({lineEdge(v0, v1), lineEdge(v1, v2)});
  auto chain = ref::tangentChain(wire, {1});
  CC_CHECK(chain.has_value());
  CC_CHECK(chain->size() == 1);
}

CC_TEST(tangent_chain_line_tangent_to_arc) {
  // Line (−1,0,0)→(0,0,0) [+X], meeting a circle (center (0,1,0), R=1) whose
  // tangent at (0,0,0) is +X → C1 → chain grows across the line↔arc joint.
  Shape a = ShapeBuilder::makeVertex({-1, 0, 0});
  Shape o = ShapeBuilder::makeVertex({0, 0, 0});
  Shape top = ShapeBuilder::makeVertex({1, 1, 0});  // quarter-arc end
  Shape line = lineEdge(a, o);
  Shape arc = circleEdge(o, top, Point3{0, 1, 0}, 1.0, Dir3{0, 0, 1}, Dir3{1, 0, 0},
                         -kPi / 2, 0.0);
  Shape wire = ShapeBuilder::makeWire({line, arc});
  auto chain = ref::tangentChain(wire, {1});
  CC_CHECK(chain.has_value());
  CC_CHECK(chain->size() == 2);
}

// ── outerRimChain ────────────────────────────────────────────────────────────
CC_TEST(outer_rim_chain_planar_cap) {
  // A single welded planar quad: seeding one edge returns the whole 4-edge rim.
  Shape face = weldedQuad({0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}, Dir3{0, 0, 1});
  auto rim = ref::outerRimChain(face, {1});
  std::sort(rim.begin(), rim.end());
  CC_CHECK(rim.size() == 4);
  CC_CHECK(rim == (std::vector<int>{1, 2, 3, 4}));
}

// ── offsetFaceBoundary ───────────────────────────────────────────────────────
CC_TEST(offset_boundary_inward_rectangle) {
  // Rectangle [0,10]×[0,6] in z=0, offset inward by 1 → [1,9]×[1,5].
  Shape face = weldedQuad({0, 0, 0}, {10, 0, 0}, {10, 6, 0}, {0, 6, 0}, Dir3{0, 0, 1});
  auto pts = ref::offsetFaceBoundary(face, 1.0);
  CC_CHECK(pts.has_value());
  CC_CHECK(pts->size() == 12);  // 4 corners × xyz
  // Collect the 4 corners; each must be one of the expected inner corners.
  const double ex[4][2] = {{1, 1}, {9, 1}, {9, 5}, {1, 5}};
  for (int i = 0; i < 4; ++i) {
    const double x = (*pts)[3 * i], y = (*pts)[3 * i + 1], z = (*pts)[3 * i + 2];
    CC_CHECK(near(z, 0.0));
    bool matched = false;
    for (auto& e : ex)
      if (near(x, e[0]) && near(y, e[1])) matched = true;
    CC_CHECK(matched);
  }
}

CC_TEST(offset_boundary_declines_growing_convex) {
  // A growing convex offset (distance < 0 → outward) is arc-rounded by OCCT →
  // native declines rather than emit sharp corners the oracle would not.
  Shape face = weldedQuad({0, 0, 0}, {10, 0, 0}, {10, 6, 0}, {0, 6, 0}, Dir3{0, 0, 1});
  CC_CHECK(!ref::offsetFaceBoundary(face, -1.0).has_value());
}

CC_TEST(offset_boundary_declines_non_planar) {
  const Shape cyl = makeCylinder(1.5, 5.0);
  CC_CHECK(!ref::offsetFaceBoundary(faceOf(cyl, 3), 0.2).has_value());  // cylinder face
}

CC_RUN_ALL()
