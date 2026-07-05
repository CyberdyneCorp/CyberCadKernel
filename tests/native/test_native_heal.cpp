// SPDX-License-Identifier: Apache-2.0
//
// test_native_heal.cpp — host (OCCT-FREE) unit suite for the native shape-healing
// slice (Phase 4 #4 `native-healing`).
//
// Builds deliberately-broken fixtures NATIVELY (topology::ShapeBuilder), heals them
// with cybercad::native::heal::healShell, and asserts the honest outcome:
//   * in-scope defects (soup / near-coincident verts / degenerate edge / sliver
//     face / flipped face) HEAL to a watertight, valid (enclosed volume > 0) unit-
//     cube solid with the expected merge/drop/flip metrics, and
//   * out-of-scope defects (missing face → open shell; gap beyond tolerance) report
//     UNHEALED with the input UNCHANGED and a measured maxResidualGap — never a
//     faked closure.
//
// This is gate 1 (host); the sim gate (native-vs-OCCT BRepBuilderAPI_Sewing /
// ShapeFix parity) lives in tests/sim/native_heal_parity.mm. No OCCT is linked here.
//
#include "harness.h"

#include "native/heal/native_heal.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace heal = cybercad::native::heal;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {

// ── Native face-soup builders (independent faces, NOT sharing vertex nodes) ─────

topo::Shape quadFace(const m::Point3& p0, const m::Point3& p1, const m::Point3& p2,
                     const m::Point3& p3, const m::Dir3& normal,
                     topo::Orientation orient = topo::Orientation::Forward) {
  const m::Vec3 ref = std::fabs(normal.z()) < 0.9 ? m::Vec3{0, 0, 1} : m::Vec3{1, 0, 0};
  const m::Ax3 frame = m::Ax3::fromAxisAndRef(p0, normal, m::Dir3{ref});
  const m::Point3 pts[4] = {p0, p1, p2, p3};
  topo::Shape v[4];
  for (int i = 0; i < 4; ++i) v[i] = topo::ShapeBuilder::makeVertex(pts[i]);
  auto toUV = [&](const m::Point3& p) -> m::Point3 {
    const m::Vec3 d = p - frame.origin;
    return m::Point3{m::dot(d, frame.x.vec()), m::dot(d, frame.y.vec()), 0.0};
  };
  std::vector<topo::Shape> edges;
  for (int i = 0; i < 4; ++i) {
    const m::Point3& a = pts[i];
    const m::Point3& b = pts[(i + 1) % 4];
    const m::Vec3 d = b - a;
    const double len = std::max(m::norm(d), 1e-12);
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = a;
    c.frame.x = m::norm(d) > 1e-12 ? m::Dir3{d} : m::Dir3{1, 0, 0};
    c.frame.z = frame.z;
    topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, v[i], v[(i + 1) % 4]);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    const m::Point3 uv0 = toUV(a), uv1 = toUV(b);
    pc.origin2d = uv0;
    pc.dir2d = (uv1 - uv0) / len;
    edges.push_back(topo::ShapeBuilder::addPCurve(e, e.tshape(), pc));
  }
  topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, wire, {}, orient);
}

topo::Shape triFace(const m::Point3& p0, const m::Point3& p1, const m::Point3& p2,
                    const m::Dir3& normal) {
  const m::Vec3 ref = std::fabs(normal.z()) < 0.9 ? m::Vec3{0, 0, 1} : m::Vec3{1, 0, 0};
  const m::Ax3 frame = m::Ax3::fromAxisAndRef(p0, normal, m::Dir3{ref});
  const m::Point3 pts[3] = {p0, p1, p2};
  topo::Shape v[3];
  for (int i = 0; i < 3; ++i) v[i] = topo::ShapeBuilder::makeVertex(pts[i]);
  auto toUV = [&](const m::Point3& p) -> m::Point3 {
    const m::Vec3 d = p - frame.origin;
    return m::Point3{m::dot(d, frame.x.vec()), m::dot(d, frame.y.vec()), 0.0};
  };
  std::vector<topo::Shape> edges;
  for (int i = 0; i < 3; ++i) {
    const m::Point3& a = pts[i];
    const m::Point3& b = pts[(i + 1) % 3];
    const m::Vec3 d = b - a;
    const double len = std::max(m::norm(d), 1e-12);
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = a;
    c.frame.x = m::norm(d) > 1e-12 ? m::Dir3{d} : m::Dir3{1, 0, 0};
    c.frame.z = frame.z;
    topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, v[i], v[(i + 1) % 3]);
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    const m::Point3 uv0 = toUV(a), uv1 = toUV(b);
    pc.origin2d = uv0;
    pc.dir2d = (uv1 - uv0) / len;
    edges.push_back(topo::ShapeBuilder::addPCurve(e, e.tshape(), pc));
  }
  topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, wire, {}, topo::Orientation::Forward);
}

struct Corners {
  m::Point3 c[8];
};
Corners cubeCorners(double s = 1.0) {
  return Corners{{m::Point3{0, 0, 0}, m::Point3{s, 0, 0}, m::Point3{s, s, 0}, m::Point3{0, s, 0},
                  m::Point3{0, 0, s}, m::Point3{s, 0, s}, m::Point3{s, s, s}, m::Point3{0, s, s}}};
}

// Six independent unit-cube faces; `jitter` displaces each face's own copy of every
// shared corner by a deterministic sub-value so no two faces share a vertex node.
std::vector<topo::Shape> cubeFaceSoup(double jitter) {
  const Corners k = cubeCorners();
  auto j = [&](const m::Point3& p, int face, int idx) -> m::Point3 {
    const double s = jitter;
    const double dx = s * (((face * 7 + idx * 3) % 5) - 2) / 2.0;
    const double dy = s * (((face * 3 + idx * 5) % 5) - 2) / 2.0;
    const double dz = s * (((face * 5 + idx * 7) % 5) - 2) / 2.0;
    return m::Point3{p.x + dx, p.y + dy, p.z + dz};
  };
  std::vector<topo::Shape> f;
  f.push_back(quadFace(j(k.c[0], 0, 0), j(k.c[3], 0, 1), j(k.c[2], 0, 2), j(k.c[1], 0, 3),
                       m::Dir3{0, 0, -1}));
  f.push_back(quadFace(j(k.c[4], 1, 0), j(k.c[5], 1, 1), j(k.c[6], 1, 2), j(k.c[7], 1, 3),
                       m::Dir3{0, 0, 1}));
  f.push_back(quadFace(j(k.c[0], 2, 0), j(k.c[1], 2, 1), j(k.c[5], 2, 2), j(k.c[4], 2, 3),
                       m::Dir3{0, -1, 0}));
  f.push_back(quadFace(j(k.c[3], 3, 0), j(k.c[7], 3, 1), j(k.c[6], 3, 2), j(k.c[2], 3, 3),
                       m::Dir3{0, 1, 0}));
  f.push_back(quadFace(j(k.c[0], 4, 0), j(k.c[4], 4, 1), j(k.c[7], 4, 2), j(k.c[3], 4, 3),
                       m::Dir3{-1, 0, 0}));
  f.push_back(quadFace(j(k.c[1], 5, 0), j(k.c[2], 5, 1), j(k.c[6], 5, 2), j(k.c[5], 5, 3),
                       m::Dir3{1, 0, 0}));
  return f;
}

const heal::HealOptions kOpts{1e-4};

// Verify a healed solid meshes watertight with the expected enclosed volume.
bool watertightVolumeNear(const topo::Shape& s, double expected, double tol) {
  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(s);
  return tess::isWatertight(mesh) && std::fabs(tess::enclosedVolume(mesh) - expected) < tol;
}

}  // namespace

// ── (A) soup-cube: tolerant sew + vertex unify → watertight unit cube ───────────
CC_TEST(heal_soup_cube_watertight) {
  auto f = cubeFaceSoup(1e-6);
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), kOpts);
  CC_CHECK(r.healed());
  CC_CHECK(r.metrics.watertight);
  CC_CHECK(r.metrics.valid);
  CC_CHECK_EQ(r.metrics.nMergedEdges, 12);       // a cube has 12 shared edges
  CC_CHECK_EQ(r.metrics.nMergedVerts, 16);       // 24 corner copies → 8 shared
  CC_CHECK(r.metrics.maxResidualGap == 0.0);     // fully closed
  CC_CHECK(watertightVolumeNear(r.shape, 1.0, 1e-6));
}

// ── (B) degenerate edge: zero-length side dropped, rest heals ───────────────────
CC_TEST(heal_degenerate_edge) {
  auto f = cubeFaceSoup(1e-6);
  // A tri face with a duplicated corner → a zero-length edge (degenerate).
  f.push_back(triFace(m::Point3{0.5, 0.5, 1.0}, m::Point3{0.5, 0.5, 1.0},
                      m::Point3{0.6, 0.5, 1.0}, m::Dir3{0, 0, 1}));
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), kOpts);
  CC_CHECK(r.healed());
  CC_CHECK(r.metrics.watertight);
  CC_CHECK(r.metrics.nDroppedDegenerate >= 1);   // the degenerate tri was removed
  CC_CHECK(watertightVolumeNear(r.shape, 1.0, 1e-6));
}

// ── (C) sliver face: near-zero-area needle dropped, cube heals ──────────────────
CC_TEST(heal_sliver_face) {
  auto f = cubeFaceSoup(1e-6);
  // A near-collinear tri (min height ~1e-7 ≪ tol) → a sliver face.
  f.push_back(triFace(m::Point3{0.2, 0.5, 1.0}, m::Point3{0.5, 0.5, 1.0},
                      m::Point3{0.8, 0.5000001, 1.0}, m::Dir3{0, 0, 1}));
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), kOpts);
  CC_CHECK(r.healed());
  CC_CHECK(r.metrics.nDroppedDegenerate >= 1);
  CC_CHECK(watertightVolumeNear(r.shape, 1.0, 1e-6));
}

// ── (D) flipped face: orientation flood-fill re-orients it outward ──────────────
CC_TEST(heal_flipped_face) {
  auto f = cubeFaceSoup(1e-6);
  f[1] = f[1].reversedShape();  // flip the +Z top face inward
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), kOpts);
  CC_CHECK(r.healed());
  CC_CHECK(r.metrics.valid);
  CC_CHECK(r.metrics.nFlipped >= 1);             // at least one face was re-oriented
  CC_CHECK(watertightVolumeNear(r.shape, 1.0, 1e-6));  // outward ⇒ +volume
}

// ── (D2) all-inward: consistent-but-inward → global enclosed-volume sign flip ───
CC_TEST(heal_all_inward_global_flip) {
  auto f = cubeFaceSoup(1e-6);
  for (auto& fc : f) fc = fc.reversedShape();  // every face wound inward
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), kOpts);
  CC_CHECK(r.healed());
  CC_CHECK(r.metrics.valid);
  CC_CHECK(watertightVolumeNear(r.shape, 1.0, 1e-6));  // sign flip makes it outward
}

// ── (E) near-coincident vertices: the unifier merges within-tol copies ──────────
CC_TEST(heal_vertex_unify_merges_within_tol) {
  heal::VertexUnifier vu(1e-4);
  const topo::Shape a = vu.vertexFor(m::Point3{1, 0, 0});
  const topo::Shape b = vu.vertexFor(m::Point3{1 + 5e-6, -5e-6, 0});  // within tol
  const topo::Shape c = vu.vertexFor(m::Point3{2, 0, 0});              // far → own node
  CC_CHECK(a.isSameGeometry(b));       // 5e-6 apart ⇒ unified
  CC_CHECK(!a.isSameGeometry(c));      // 1.0 apart ⇒ NOT merged (never fabricated)
  CC_CHECK_EQ(vu.mergedCount(), 1);
  CC_CHECK_EQ(vu.distinctCount(), 2);
}

// ── (E2) beyond-tol vertices are NEVER merged (honesty of the unifier) ──────────
CC_TEST(heal_vertex_unify_rejects_beyond_tol) {
  heal::VertexUnifier vu(1e-4);
  const topo::Shape a = vu.vertexFor(m::Point3{0, 0, 0});
  const topo::Shape b = vu.vertexFor(m::Point3{5e-4, 0, 0});  // 5·tol apart
  CC_CHECK(!a.isSameGeometry(b));
  CC_CHECK_EQ(vu.mergedCount(), 0);
}

// ── (F) UN-healable: a missing face (open shell) → UNHEALED, input unchanged ────
CC_TEST(heal_open_shell_unhealed) {
  auto f = cubeFaceSoup(1e-6);
  f.erase(f.begin() + 1);  // remove the +Z face → a genuine hole
  const topo::Shape input = topo::ShapeBuilder::makeShell(f);
  const heal::HealResult r = heal::healShell(input, kOpts);
  CC_CHECK(!r.healed());
  CC_CHECK(r.reason == heal::UnhealedReason::OpenShell);
  CC_CHECK(!r.metrics.watertight);
  CC_CHECK(r.shape.isSameGeometry(input));  // input returned UNCHANGED
}

// ── (F2) UN-healable: a gap beyond tolerance → UNHEALED with measured residual ──
CC_TEST(heal_beyond_tolerance_gap_unhealed) {
  auto f = cubeFaceSoup(1e-2);  // gaps ~1e-2 ≫ tol 1e-4
  const topo::Shape input = topo::ShapeBuilder::makeShell(f);
  const heal::HealResult r = heal::healShell(input, kOpts);
  CC_CHECK(!r.healed());
  CC_CHECK(r.reason == heal::UnhealedReason::GapBeyondTolerance);
  CC_CHECK(r.metrics.maxResidualGap > kOpts.tolerance);  // honest measured residual
  CC_CHECK(r.shape.isSameGeometry(input));               // input UNCHANGED
}

// ── (G) tolerance is never weakened: the same beyond-tol gap stays UNHEALED ─────
CC_TEST(heal_never_weakens_tolerance) {
  auto f = cubeFaceSoup(1e-2);
  const heal::HealResult r = heal::healShell(topo::ShapeBuilder::makeShell(f), heal::HealOptions{1e-4});
  CC_CHECK(!r.healed());  // a 1e-2 gap is NOT closed by silently widening 1e-4
}

CC_RUN_ALL()
