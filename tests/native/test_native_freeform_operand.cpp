// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate (Gate a, no OCCT) for MOAT M2 / B1: the freeform operand
// DESCRIPTOR + `recogniseFreeformSolid` gate (`boolean/freeform_operand.h`).
//
// Proves, with ZERO OCCT:
//   §1 ADMISSION + ROUND-TRIP — a genuinely-trimmed freeform-walled solid (the M0
//      `bumpCappedCylinder` keystone: cylinder side + flat bottom + Bézier bump-cap
//      top) is admitted; its faces/kinds/roles/trims round-trip; `outwardN` points
//      materially outward; `bbox` is tight; `watertight` is true.
//   §2 DECLINE BATTERY — every non-admissible operand returns `nullopt` with the
//      RIGHT measured blocker (not-solid, multi-shell, unsupported kind (torus),
//      bare freeform, holed freeform, no freeform face, non-watertight).
//   §3 EXPOSED HANDLES — the descriptor hands the M2 verbs their inputs: the
//      operand `Shape` meshes watertight under M0; the `Aabb` scales B3
//      `classifyPointInMesh` (interior→In, exterior→Out); and the freeform `Face`
//      is B2 `splitFace`'s input — whose measured decline on this reachable wall is
//      the concrete assembly blocker recorded in the honest-out.
//
// OCCT-FREE. Standalone build (same recipe as the tessellator/membership tests):
//   clang++ -std=c++20 tests/native/test_native_freeform_operand.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_freeform_operand
//
#include "native/boolean/face_split.h"        // B2 splitFace (assembly-blocker witness)
#include "native/boolean/freeform_membership.h"  // B3 classifyPointInMesh
#include "native/boolean/freeform_operand.h"     // the B1 gate under test
#include "native/tessellate/native_tessellate.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace m = cybercad::native::math;
namespace ssi = cybercad::native::ssi;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kBumpH = 1.0;

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
topo::FaceSurface bumpCapSurface(double capZ, double rho) {
  const double k = kBumpH / (rho * rho), c0 = kBumpH / 2 - 0.25 * k, c1 = kBumpH / 2 + 0.25 * k;
  const double xc[3] = {-0.5, 0.0, 0.5}, fz[3] = {c0, c1, c0};
  topo::FaceSurface s{}; s.kind = topo::FaceSurface::Kind::Bezier; s.nPolesU = 3; s.nPolesV = 3;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) s.poles.push_back(m::Point3{xc[i], xc[j], capZ + fz[i] + fz[j]});
  return s;
}

// The keystone reachable operand: cylinder side + flat bottom disk + Bézier bump
// cap. `withBottom=false` drops the bottom cap → an OPEN (non-watertight) boundary.
topo::Shape bumpCappedCylinderSolid(double R, double h, bool withBottom = true) {
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
  topo::FaceSurface cap = bumpCapSurface(h, R);
  topo::Shape cf0 = topo::ShapeBuilder::makeFace(cap, topo::Shape{});
  topo::PCurve pcc{}; pcc.kind = topo::EdgeCurve::Kind::Circle; pcc.origin2d = {0.5, 0.5, 0}; pcc.dir2d = {R, 0, 0};
  auto cOn = topo::ShapeBuilder::addPCurve(topC, cf0.tshape(), pcc);
  topo::Shape capFace = topo::ShapeBuilder::makeFace(cap, topo::ShapeBuilder::makeWire({cOn}), {},
                                                     topo::Orientation::Forward);
  std::vector<topo::Shape> faces{sideFace, capFace};
  if (withBottom) faces.insert(faces.begin() + 1, botCap);
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

// ── Minimal single-face solids for the per-face decline battery ────────────────
topo::FaceSurface flatBezier() {
  topo::FaceSurface s{}; s.kind = topo::FaceSurface::Kind::Bezier; s.nPolesU = 3; s.nPolesV = 3;
  const double xc[3] = {-0.5, 0.0, 0.5};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) s.poles.push_back(m::Point3{xc[i], xc[j], 0.0});
  return s;
}
topo::Shape triWire() {  // a real (if trivial) wire — used only for its Wire count
  auto a = vertexAt(0, 0, 0), b = vertexAt(1, 0, 0), c = vertexAt(0, 1, 0);
  return topo::ShapeBuilder::makeWire({lineEdge(a, b), lineEdge(b, c), lineEdge(c, a)});
}
topo::Shape singleFaceSolid(const topo::Shape& face) {
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell({face})});
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// §1 — ADMISSION + ROUND-TRIP.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(freeform_operand_admits_bump_capped_cylinder) {
  const double R = 1.0, h = 2.0;
  OperandDecline why = OperandDecline::NotSolid;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h), &why);
  CC_CHECK(op.has_value());
  CC_CHECK(why == OperandDecline::Ok);
  CC_CHECK(op->watertight);
  CC_CHECK(op->faces.size() == 3);
  CC_CHECK(op->freeform.size() == 1);   // the Bézier bump cap
  CC_CHECK(op->analytic.size() == 2);   // cylinder side + planar bottom
}

CC_TEST(freeform_operand_faces_round_trip) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  // The freeform wall really is the Bézier cap; its stored surface re-evaluates
  // bit-identically to the face's own surface (faithful round-trip).
  const OperandFace& wall = op->faces[op->freeform[0]];
  CC_CHECK(wall.role == FaceRole::Freeform);
  CC_CHECK(wall.surface.kind == topo::FaceSurface::Kind::Bezier);
  const auto fs = topo::surfaceOf(wall.face);
  CC_CHECK(fs && fs->surface);
  tess::SurfaceEvaluator a(wall.surface, wall.location), b(*fs->surface, fs->location);
  const m::Point3 pa = a.value(0.5, 0.5), pb = b.value(0.5, 0.5);
  CC_CHECK(std::fabs(pa.x - pb.x) + std::fabs(pa.y - pb.y) + std::fabs(pa.z - pb.z) < 1e-12);
  // Every analytic face round-trips its kind too.
  for (std::size_t i : op->analytic) {
    const topo::FaceSurface::Kind k = op->faces[i].surface.kind;
    CC_CHECK(k == topo::FaceSurface::Kind::Cylinder || k == topo::FaceSurface::Kind::Plane);
    CC_CHECK(op->faces[i].role == FaceRole::AnalyticHalfSpace);
  }
}

CC_TEST(freeform_operand_outward_normals_point_out) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  bool sawBottom = false, sawCap = false, sawSide = false;
  for (const OperandFace& of : op->faces) {
    const m::Vec3 n = of.outwardN;
    CC_CHECK(m::norm(n) > 0.5);  // a real, unit-ish normal
    if (of.surface.kind == topo::FaceSurface::Kind::Plane) {   // bottom cap → −z
      sawBottom = true; CC_CHECK(n.z < -0.5);
    } else if (of.surface.kind == topo::FaceSurface::Kind::Bezier) {  // bump cap → +z
      sawCap = true; CC_CHECK(n.z > 0.3);
    } else {  // cylinder side → radial (mostly horizontal)
      sawSide = true; CC_CHECK(std::sqrt(n.x * n.x + n.y * n.y) > 0.7);
    }
  }
  CC_CHECK(sawBottom && sawCap && sawSide);
}

CC_TEST(freeform_operand_bbox_is_tight) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  const Aabb& bb = op->bbox;
  // Faithful boundary extent: x,y ∈ [−R,R], z ∈ [0, h] (the rim samples).
  CC_CHECK(std::fabs(bb.lo.x + R) < 1e-6 && std::fabs(bb.hi.x - R) < 1e-6);
  CC_CHECK(std::fabs(bb.lo.y + R) < 1e-6 && std::fabs(bb.hi.y - R) < 1e-6);
  CC_CHECK(std::fabs(bb.lo.z) < 1e-6 && bb.hi.z >= h - 1e-6);
  CC_CHECK(bb.diagonal() > R);  // a usable ON-band scale (never degenerate)
}

// ═════════════════════════════════════════════════════════════════════════════
// §2 — DECLINE BATTERY (each returns nullopt with the RIGHT measured blocker).
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(freeform_operand_declines_non_solid) {
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(topo::Shape{}, &why).has_value());
  CC_CHECK(why == OperandDecline::NotSolid);
  // A bare face is not a Solid either.
  CC_CHECK(!recogniseFreeformSolid(topo::ShapeBuilder::makeFace(flatBezier(), topo::Shape{}), &why)
                .has_value());
  CC_CHECK(why == OperandDecline::NotSolid);
}

CC_TEST(freeform_operand_declines_multi_shell) {
  const double R = 1.0, h = 2.0;
  // Two shells under one solid → not a single reachable operand.
  auto s1 = bumpCappedCylinderSolid(R, h);
  auto s2 = bumpCappedCylinderSolid(R, h);
  std::vector<topo::Shape> shells;
  for (topo::Explorer sh(s1, topo::ShapeType::Shell); sh.more(); sh.next()) shells.push_back(sh.current());
  for (topo::Explorer sh(s2, topo::ShapeType::Shell); sh.more(); sh.next()) shells.push_back(sh.current());
  topo::Shape multi = topo::ShapeBuilder::makeSolid(std::move(shells));
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(multi, &why).has_value());
  CC_CHECK(why == OperandDecline::MultiShell);
}

CC_TEST(freeform_operand_declines_torus_kind) {
  topo::FaceSurface tor{}; tor.kind = topo::FaceSurface::Kind::Torus; tor.radius = 2.0; tor.minorRadius = 0.5;
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(singleFaceSolid(topo::ShapeBuilder::makeFace(tor, triWire())), &why)
                .has_value());
  CC_CHECK(why == OperandDecline::UnsupportedSurfaceKind);
}

CC_TEST(freeform_operand_declines_bare_freeform) {
  // A freeform face with NO trimming wire = the natural rectangle → analytic-owned.
  topo::Shape bare = topo::ShapeBuilder::makeFace(flatBezier(), topo::Shape{});
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(singleFaceSolid(bare), &why).has_value());
  CC_CHECK(why == OperandDecline::BareFreeformFace);
}

CC_TEST(freeform_operand_declines_holed_freeform) {
  // A freeform face carrying an inner hole loop (outer + hole wire).
  topo::Shape holed =
      topo::ShapeBuilder::makeFace(flatBezier(), triWire(), std::vector<topo::Shape>{triWire()});
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(singleFaceSolid(holed), &why).has_value());
  CC_CHECK(why == OperandDecline::HoledFreeformFace);
}

CC_TEST(freeform_operand_declines_no_freeform_face) {
  // A purely-analytic solid → the analytic recogniseCurvedSolid path owns it.
  topo::FaceSurface pl{}; pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame = m::Ax3{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(singleFaceSolid(topo::ShapeBuilder::makeFace(pl, triWire())), &why)
                .has_value());
  CC_CHECK(why == OperandDecline::NoFreeformFace);
}

CC_TEST(freeform_operand_declines_non_watertight) {
  const double R = 1.0, h = 2.0;
  // The bump-capped cylinder with the bottom cap removed → the bottom rim edge has
  // only one incidence → open boundary → NotWatertight (never a leaky admit).
  OperandDecline why = OperandDecline::Ok;
  CC_CHECK(!recogniseFreeformSolid(bumpCappedCylinderSolid(R, h, /*withBottom=*/false), &why)
                .has_value());
  CC_CHECK(why == OperandDecline::NotWatertight);
}

// ═════════════════════════════════════════════════════════════════════════════
// §3 — EXPOSED HANDLES: the descriptor feeds the M2 verbs their inputs.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(freeform_operand_shape_meshes_watertight_under_m0) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(op->solid);   // the Shape handle → M0
  CC_CHECK(mesh.triangleCount() > 0);
  CC_CHECK(tess::isWatertight(mesh));
}

CC_TEST(freeform_operand_bbox_scales_b3_classifier) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  tess::MeshParams mp; mp.deflection = 0.02;
  tess::Mesh mesh = tess::SolidMesher{mp}.mesh(op->solid);
  const Aabb bb = op->bbox;  // the DESCRIPTOR's AABB drives B3's ON-band
  CC_CHECK(classifyPointInMesh(mesh, bb, mp.deflection, m::Point3{0, 0, 0.5}) == Membership::In);
  CC_CHECK(classifyPointInMesh(mesh, bb, mp.deflection, m::Point3{0, 0, 5.0}) == Membership::Out);
  CC_CHECK(classifyPointInMesh(mesh, bb, mp.deflection, m::Point3{5.0, 5.0, 0.5}) == Membership::Out);
}

// The freeform Face is B2 splitFace's input. On THIS reachable operand the sole
// freeform wall carries a smooth CLOSED (circular) trim; B2's first slice requires
// a convex straight-edged outer loop (≥3 boundary segments) and returns NoOuterLoop.
// This measured decline is the concrete end-to-end-assembly blocker recorded in the
// honest-out: the descriptor hands B2 a well-formed Face, but B2's admissible input
// (a polygon-trimmed wall) is not what the reachable freeform SOLID class presents.
CC_TEST(freeform_operand_face_is_b2_input_measured_blocker) {
  const double R = 1.0, h = 2.0;
  auto op = recogniseFreeformSolid(bumpCappedCylinderSolid(R, h));
  CC_CHECK(op.has_value());
  const topo::Shape wall = op->faces[op->freeform[0]].face;
  ssi::WLine dummy;  // NoOuterLoop is decided from the wire BEFORE the seam is read
  dummy.points.push_back(ssi::WLinePoint{});
  dummy.points.push_back(ssi::WLinePoint{});
  const SplitResult sr = splitFace(wall, dummy);
  CC_CHECK(!sr.ok());
  CC_CHECK(sr.decline == SplitDecline::NoOuterLoop);
  std::printf("  [assembly-blocker] B2 splitFace on the reachable freeform wall declines: %s\n",
              declineName(sr.decline));
}

CC_RUN_ALL()
