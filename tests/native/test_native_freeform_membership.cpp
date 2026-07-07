// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate (Gate 1, no OCCT) for the MOAT M2c / B3 point-in-freeform-
// solid classifier (`src/native/boolean/freeform_membership.h`).
//
// The two isolated geometry kernels (Möller–Trumbore, point-triangle distance)
// get pure unit tests; the classifier is then proven against CLOSED-FORM analytic
// truth on a genuinely-trimmed freeform-walled solid — the M0 `bumpCappedCylinder`
// keystone fixture, whose top wall is a Bézier "bump cap" `z = h + H·(1 − r²/R²)`.
// Its membership is analytic: IN ⇔ r < R ∧ z > 0 ∧ z < bumpZ(x,y). The solid is
// meshed WATERTIGHT by the landed M0 `SolidMesher` (consumed read-only), and the
// classifier is asserted to (a) match analytic truth on points comfortably away
// from the ON band, (b) resolve in-band points to `On`, (c) never emit a silent
// wrong crisp verdict across a random batch (crispWRONG == 0), and (d) decline a
// non-watertight mesh to `Unknown`.
//
// OCCT-FREE. Build (standalone, no CMake — same recipe as the tessellator tests):
//   clang++ -std=c++20 tests/native/test_native_freeform_membership.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_freeform_membership
//
#include "native/boolean/freeform_membership.h"
#include "native/tessellate/native_tessellate.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <random>
#include <vector>

using namespace cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace m = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kBumpH = 1.0;  // dome height above the rim (matches the M0 fixture)

// ── Fixture builders (mirror the M0 keystone in test_native_tessellate.cpp) ────
topo::Shape vertexAt(double x, double y, double z) {
  return topo::ShapeBuilder::makeVertex(m::Point3{x, y, z});
}
topo::Shape lineEdge(const topo::Shape& a, const topo::Shape& b) {
  topo::EdgeCurve c{}; c.kind = topo::EdgeCurve::Kind::Line;
  return topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, a, b);
}
topo::Shape circleEdge3d(double R, double z, const topo::Shape& v0, const topo::Shape& v1) {
  topo::EdgeCurve c{}; c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  c.radius = R;
  return topo::ShapeBuilder::makeEdgeWithVertices(c, 0.0, 2 * kPi, {v0, v1});
}
double bumpZ(double x, double y, double capZ, double rho) {
  return capZ + kBumpH * (1.0 - (x * x + y * y) / (rho * rho));
}
topo::FaceSurface bumpCapSurface(double capZ, double rho, bool rational) {
  const double k = kBumpH / (rho * rho), c0 = kBumpH / 2 - 0.25 * k, c1 = kBumpH / 2 + 0.25 * k;
  const double xc[3] = {-0.5, 0.0, 0.5}, fz[3] = {c0, c1, c0};
  topo::FaceSurface s{}; s.nPolesU = 3; s.nPolesV = 3;
  if (rational) {
    s.kind = topo::FaceSurface::Kind::BSpline; s.degreeU = 2; s.degreeV = 2;
    s.knotsU = {0, 0, 0, 1, 1, 1}; s.knotsV = {0, 0, 0, 1, 1, 1};
  } else {
    s.kind = topo::FaceSurface::Kind::Bezier;
  }
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      s.poles.push_back(m::Point3{xc[i], xc[j], capZ + fz[i] + fz[j]});
      if (rational) s.weights.push_back(1.0);
    }
  return s;
}
// Cylinder side (analytic) + flat bottom disk + a Bézier bump-cap top wall — a
// genuine trimmed-freeform-walled solid; the cap routes through the M0
// trimmed-free-form arm (`trimmedFreeformMesh`).
topo::Shape bumpCappedCylinderSolid(double R, double h) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface sideS{}; sideS.kind = topo::FaceSurface::Kind::Cylinder; sideS.frame = fr; sideS.radius = R;
  auto vb = vertexAt(R, 0, 0), vt = vertexAt(R, 0, h);
  topo::Shape botC = circleEdge3d(R, 0, vb, vb), topC = circleEdge3d(R, h, vt, vt);
  topo::Shape seam0 = lineEdge(vb, vt), seam1 = lineEdge(vb, vt);
  topo::Shape sf0 = topo::ShapeBuilder::makeFace(sideS, topo::Shape{});
  auto pcLine = [&](m::Point3 o, m::Vec3 d) {
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = o; pc.dir2d = d; return pc;
  };
  auto bS = topo::ShapeBuilder::addPCurve(botC, sf0.tshape(), pcLine({0, 0, 0}, {1, 0, 0}));
  auto tS = topo::ShapeBuilder::addPCurve(topC, sf0.tshape(), pcLine({0, h, 0}, {1, 0, 0}));
  auto s0 = topo::ShapeBuilder::addPCurve(seam0, sf0.tshape(), pcLine({0, 0, 0}, {0, h, 0}));
  auto s1 = topo::ShapeBuilder::addPCurve(seam1, sf0.tshape(), pcLine({2 * kPi, 0, 0}, {0, h, 0}));
  topo::Shape sideFace = topo::ShapeBuilder::makeFace(
      sideS, topo::ShapeBuilder::makeWire({bS, s1, tS.reversedShape(), s0.reversedShape()}));
  topo::FaceSurface disk{}; disk.kind = topo::FaceSurface::Kind::Plane; disk.frame = fr;
  topo::Shape kf0 = topo::ShapeBuilder::makeFace(disk, topo::Shape{});
  topo::PCurve pcd{}; pcd.kind = topo::EdgeCurve::Kind::Circle; pcd.origin2d = {0, 0, 0}; pcd.dir2d = {R, 0, 0};
  auto dOn = topo::ShapeBuilder::addPCurve(botC, kf0.tshape(), pcd);
  topo::Shape botCap = topo::ShapeBuilder::makeFace(disk, topo::ShapeBuilder::makeWire({dOn}), {},
                                                    topo::Orientation::Reversed);
  topo::FaceSurface cap = bumpCapSurface(h, R, /*rational=*/false);
  topo::Shape cf0 = topo::ShapeBuilder::makeFace(cap, topo::Shape{});
  topo::PCurve pcc{}; pcc.kind = topo::EdgeCurve::Kind::Circle; pcc.origin2d = {0.5, 0.5, 0}; pcc.dir2d = {R, 0, 0};
  auto cOn = topo::ShapeBuilder::addPCurve(topC, cf0.tshape(), pcc);
  topo::Shape capFace = topo::ShapeBuilder::makeFace(cap, topo::ShapeBuilder::makeWire({cOn}), {},
                                                     topo::Orientation::Forward);
  return topo::ShapeBuilder::makeSolid(
      {topo::ShapeBuilder::makeShell({sideFace, botCap, capFace})});
}

// Closed-form analytic membership + clearance for the bump-capped cylinder.
bool analyticInside(const m::Point3& p, double R, double h) {
  const double r2 = p.x * p.x + p.y * p.y;
  if (r2 >= R * R) return false;
  if (p.z <= 0.0) return false;
  return p.z < bumpZ(p.x, p.y, h, R);
}
// Min Euclidean distance to the solid's boundary (bottom plane, cylinder wall,
// bump surface, and the two rim circles), used to know if a sample sits in the band.
double analyticClearance(const m::Point3& p, double R, double h) {
  const double r = std::sqrt(p.x * p.x + p.y * p.y);
  double best = 1e300;
  if (r < R) best = std::min(best, std::fabs(p.z));               // bottom plane z=0
  if (p.z > 0 && p.z < h) best = std::min(best, std::fabs(r - R));  // cylinder wall
  {  // bump surface — perpendicular estimate |z−f| / sqrt(1+|∇f|²)
    const double gx = -2.0 * kBumpH * p.x / (R * R), gy = -2.0 * kBumpH * p.y / (R * R);
    const double vgap = std::fabs(p.z - bumpZ(p.x, p.y, h, R));
    best = std::min(best, vgap / std::sqrt(1.0 + gx * gx + gy * gy));
  }
  best = std::min(best, std::sqrt((r - R) * (r - R) + (p.z - h) * (p.z - h)));  // top rim
  best = std::min(best, std::sqrt((r - R) * (r - R) + p.z * p.z));              // bottom rim
  return best;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// §1.1 — Möller–Trumbore kernel unit tests.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(moller_trumbore_forward_interior_hit) {
  // Triangle in the z=1 plane; ray from origin along +z hits its interior.
  const m::Point3 a{0, 0, 1}, b{1, 0, 1}, c{0, 1, 1};
  auto hit = mollerTrumbore(m::Point3{0.25, 0.25, 0}, m::Vec3{0, 0, 1}, a, b, c);
  CC_CHECK(hit.has_value());
  CC_CHECK(std::fabs(hit->t - 1.0) < 1e-12);
  CC_CHECK(hit->u > 0 && hit->v > 0 && hit->u + hit->v < 1);  // strictly interior
}
CC_TEST(moller_trumbore_backward_miss) {
  // Same triangle at z=1 but the ray points AWAY (−z): no forward crossing.
  const m::Point3 a{0, 0, 1}, b{1, 0, 1}, c{0, 1, 1};
  CC_CHECK(!mollerTrumbore(m::Point3{0.25, 0.25, 0}, m::Vec3{0, 0, -1}, a, b, c).has_value());
}
CC_TEST(moller_trumbore_parallel_miss) {
  // Ray in the plane of the triangle (parallel) ⇒ nullopt.
  const m::Point3 a{0, 0, 1}, b{1, 0, 1}, c{0, 1, 1};
  CC_CHECK(!mollerTrumbore(m::Point3{0.25, 0.25, 1}, m::Vec3{1, 0, 0}, a, b, c).has_value());
}
CC_TEST(moller_trumbore_outside_barycentric_miss) {
  // Aim past the triangle (u+v > 1) ⇒ miss.
  const m::Point3 a{0, 0, 1}, b{1, 0, 1}, c{0, 1, 1};
  CC_CHECK(!mollerTrumbore(m::Point3{0.9, 0.9, 0}, m::Vec3{0, 0, 1}, a, b, c).has_value());
}

// ═════════════════════════════════════════════════════════════════════════════
// §1.2 — point-triangle distance kernel unit tests (vertex / edge / face regions).
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(point_triangle_distance_face_region) {
  const m::Point3 a{0, 0, 0}, b{1, 0, 0}, c{0, 1, 0};
  // Directly above the centroid ⇒ perpendicular distance = 2.
  CC_CHECK(std::fabs(pointTriangleDistance(m::Point3{0.25, 0.25, 2}, a, b, c) - 2.0) < 1e-12);
}
CC_TEST(point_triangle_distance_vertex_region) {
  const m::Point3 a{0, 0, 0}, b{1, 0, 0}, c{0, 1, 0};
  // Beyond vertex a in the −x,−y quadrant ⇒ closest point is a.
  CC_CHECK(std::fabs(pointTriangleDistance(m::Point3{-3, -4, 0}, a, b, c) - 5.0) < 1e-12);
}
CC_TEST(point_triangle_distance_edge_region) {
  const m::Point3 a{0, 0, 0}, b{1, 0, 0}, c{0, 1, 0};
  // Below the midpoint of edge ab (in −y), in-plane ⇒ distance to edge ab = 3.
  CC_CHECK(std::fabs(pointTriangleDistance(m::Point3{0.5, -3, 0}, a, b, c) - 3.0) < 1e-12);
}

// ═════════════════════════════════════════════════════════════════════════════
// §2 — the classifier, on a hand-built watertight unit cube (analytic truth,
// no mesher needed) — a fast smoke test of parity + ON-band before the freeform gate.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(classify_unit_cube_in_out_on) {
  tess::Mesh cube;
  const double c[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
  for (auto& q : c) cube.addVertex({q[0], q[1], q[2]});
  auto quad = [&](int a, int b, int d, int e) { cube.addTriangle(a, b, d); cube.addTriangle(a, d, e); };
  quad(0,3,2,1); quad(4,5,6,7); quad(0,1,5,4); quad(2,3,7,6); quad(1,2,6,5); quad(3,0,4,7);
  CC_CHECK(tess::isWatertight(cube));
  const Aabb bb = meshAabb(cube);
  MembershipTol tol;  // planar cube ⇒ pass deflection 0; ON-band = relTol·diag floor.
  CC_CHECK(classifyPointInMesh(cube, bb, 0.0, m::Point3{0.5, 0.5, 0.5}, tol) == Membership::In);
  CC_CHECK(classifyPointInMesh(cube, bb, 0.0, m::Point3{1.7, 0.5, 0.5}, tol) == Membership::Out);
  CC_CHECK(classifyPointInMesh(cube, bb, 0.0, m::Point3{-0.4, 0.5, 0.5}, tol) == Membership::Out);
  // A point on the +x face is within the band ⇒ ON (with a modest deflection band).
  CC_CHECK(classifyPointInMesh(cube, bb, 0.01, m::Point3{1.0, 0.5, 0.5}, tol) == Membership::On);
}

CC_TEST(classify_non_watertight_declines_unknown) {
  // An open patch (two triangles) is not watertight ⇒ Unknown, never a crisp guess.
  tess::Mesh open;
  open.addVertex({0, 0, 0}); open.addVertex({1, 0, 0}); open.addVertex({1, 1, 0}); open.addVertex({0, 1, 0});
  open.addTriangle(0, 1, 2); open.addTriangle(0, 2, 3);
  CC_CHECK(!tess::isWatertight(open));
  CC_CHECK(classifyPointInMesh(open, meshAabb(open), 0.01, m::Point3{0.5, 0.5, 0.5}) == Membership::Unknown);
}

// ═════════════════════════════════════════════════════════════════════════════
// §3 — HOST-ANALYTIC GATE: freeform-walled solid vs closed-form truth.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(freeform_solid_meshes_watertight) {
  const double R = 1.0, h = 2.0;
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(bumpCappedCylinderSolid(R, h));
  CC_CHECK(mesh.triangleCount() > 0);
  CC_CHECK(tess::isWatertight(mesh));            // R1 precondition: watertight substrate
  CC_CHECK(tess::boundaryEdgeCount(mesh) == 0);  // no open seam
}

// 3.3 — points COMFORTABLY away from the ON band match analytic IN/OUT exactly.
CC_TEST(freeform_membership_matches_analytic_truth_away_from_band) {
  const double R = 1.0, h = 2.0;
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(bumpCappedCylinderSolid(R, h));
  CC_CHECK(tess::isWatertight(mesh));
  const Aabb bb = meshAabb(mesh);
  MembershipTol tol;
  const double band = tol.bandDeflectionFactor * mp.deflection;  // relTol·diag negligible
  const double decideClear = 3.0 * band;  // "comfortably away"

  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> Ux(-1.25, 1.25), Uy(-1.25, 1.25), Uz(-0.3, 3.4);
  int total = 0, agree = 0, wrong = 0, declined = 0, inCount = 0, outCount = 0, tried = 0;
  while (total < 3000 && tried < 400000) {
    ++tried;
    m::Point3 p{Ux(rng), Uy(rng), Uz(rng)};
    if (analyticClearance(p, R, h) <= decideClear) continue;  // skip near-boundary
    ++total;
    const bool truth = analyticInside(p, R, h);
    (truth ? inCount : outCount)++;
    const Membership c = classifyPointInMesh(mesh, bb, mp.deflection, p, tol);
    if (c == Membership::Unknown || c == Membership::On) { ++declined; continue; }
    ((c == Membership::In) == truth) ? ++agree : ++wrong;
  }
  std::printf("  [away-from-band] sampled=%d (IN=%d OUT=%d) agree=%d WRONG=%d declined=%d\n",
              total, inCount, outCount, agree, wrong, declined);
  CC_CHECK(total >= 2000);      // fixture actually exercised on both sides
  CC_CHECK(inCount > 100);
  CC_CHECK(outCount > 100);
  CC_CHECK(wrong == 0);         // decidable-region agreement is exact
  CC_CHECK(declined == 0);      // comfortably-away points are never declined
}

// 3.4 — points placed INSIDE the band resolve to On (the honest ON contract).
CC_TEST(freeform_membership_in_band_resolves_on) {
  const double R = 1.0, h = 2.0;
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(bumpCappedCylinderSolid(R, h));
  const Aabb bb = meshAabb(mesh);
  MembershipTol tol;
  // Sample points exactly ON the true surfaces (bottom disk, cylinder wall, bump
  // cap) — well within the band; each must resolve On (never a crisp guess).
  std::mt19937 rng(777);
  std::uniform_real_distribution<double> Uang(0, 2 * kPi), U01(0, 1);
  int on = 0, notOn = 0;
  for (int i = 0; i < 600; ++i) {
    m::Point3 s;
    const int which = i % 3;
    const double a = Uang(rng), rr = U01(rng) * R * 0.9;
    if (which == 0) s = m::Point3{rr * std::cos(a), rr * std::sin(a), 0.0};             // bottom
    else if (which == 1) s = m::Point3{R * std::cos(a), R * std::sin(a), U01(rng) * h};  // wall
    else s = m::Point3{rr * std::cos(a), rr * std::sin(a), bumpZ(rr * std::cos(a), rr * std::sin(a), h, R)};
    (classifyPointInMesh(mesh, bb, mp.deflection, s, tol) == Membership::On) ? ++on : ++notOn;
  }
  std::printf("  [in-band] on-surface samples: On=%d notOn=%d\n", on, notOn);
  CC_CHECK(notOn == 0);  // every on-surface point honestly resolves On
}

// 3.5 — across a large random batch, ZERO silent wrong crisp verdicts (the core
// honesty invariant): every crisp IN/OUT the classifier emits agrees with analytic
// truth, regardless of clearance. In-band ambiguity surfaces as On/Unknown, not a lie.
CC_TEST(freeform_membership_zero_silent_wrong_over_batch) {
  const double R = 1.0, h = 2.0;
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(bumpCappedCylinderSolid(R, h));
  const Aabb bb = meshAabb(mesh);
  MembershipTol tol;
  std::mt19937 rng(2024);
  std::uniform_real_distribution<double> Ux(-1.25, 1.25), Uy(-1.25, 1.25), Uz(-0.3, 3.4);
  int crisp = 0, crispWrong = 0, onUnknown = 0;
  for (int i = 0; i < 40000; ++i) {
    m::Point3 p{Ux(rng), Uy(rng), Uz(rng)};
    const Membership c = classifyPointInMesh(mesh, bb, mp.deflection, p, tol);
    if (c == Membership::On || c == Membership::Unknown) { ++onUnknown; continue; }
    ++crisp;
    if ((c == Membership::In) != analyticInside(p, R, h)) ++crispWrong;
  }
  std::printf("  [batch] crisp=%d crispWRONG=%d on/unknown=%d\n", crisp, crispWrong, onUnknown);
  CC_CHECK(crisp > 20000);     // most points are decidable
  CC_CHECK(crispWrong == 0);   // *** the load-bearing invariant: no silent wrong ***
}

CC_RUN_ALL()
