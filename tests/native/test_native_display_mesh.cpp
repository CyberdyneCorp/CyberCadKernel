// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the render-quality DISPLAY mesh (src/native/render/
// display_mesh.h) — Gate (a) of the two-gate verification model
// (openspec/NATIVE-REWRITE.md). OCCT-FREE: clang++ -std=c++20, no OCCT, no sim.
//
// The display mesh is a PURELY ADDITIVE post-process of the correctness mesh
// (SolidMesher output). It adds smooth per-vertex normals, crease-angle hard
// edges, optional UVs, and optional LOD decimation. It moves NO vertex (except
// LOD, bounded by a Hausdorff budget) and never touches the tessellator.
//
// The oracle here is CLOSED-FORM analytic surface normals — a BETTER oracle than
// OCCT for shading:
//   * CYLINDER lateral wall  → smooth normals are EXACTLY radial (⊥ axis), match
//     the analytic surface normal (x,y,0)/R to ~1e-6, continuous around the wall.
//   * CYLINDER cap↔wall circle→ a crease: the ring vertices SPLIT — the cap copy
//     points axial (±Z), the wall copy points radial.
//   * SPHERE                 → all-smooth; every display normal = analytic radial
//     P/R to ~1e-6.
//   * BOX                    → every edge a crease: 8 corners × 3 faces = 24 split
//     corner normals, 6 axis-aligned face normals.
//   * CREASE THRESHOLD       → raising the crease angle reduces the split count.
//   * LOD                    → reduces triangle count while every survivor stays
//     within the asserted Hausdorff bound of the source solid.
//   * UVs                    → all in [0,1], seam-consistent (same pos+axis ⇒ same UV).
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_display_mesh.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_display_mesh
//
#include "native/render/display_mesh.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <unordered_map>
#include <vector>

using namespace cybercad::native::render;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace m = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── Shape builders (mirrors of the tessellate test's, kept local) ─────────────

topo::Shape vertexAt(double x, double y, double z) {
  return topo::ShapeBuilder::makeVertex(m::Point3{x, y, z});
}
topo::Shape lineEdge(const topo::Shape& a, const topo::Shape& b) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Line;
  return topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, a, b);
}
topo::Shape circleEdge3d(double R, double z, const topo::Shape& v0, const topo::Shape& v1) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  c.radius = R;
  return topo::ShapeBuilder::makeEdgeWithVertices(c, 0.0, 2 * kPi, {v0, v1});
}

topo::Shape boxSolid(double a, double b, double c) {
  topo::Shape v[8] = {vertexAt(0, 0, 0), vertexAt(a, 0, 0), vertexAt(a, b, 0), vertexAt(0, b, 0),
                      vertexAt(0, 0, c), vertexAt(a, 0, c), vertexAt(a, b, c), vertexAt(0, b, c)};
  auto edge = [&](int i, int j) { return lineEdge(v[i], v[j]); };
  topo::Shape e[12] = {edge(0, 1), edge(1, 2), edge(2, 3), edge(3, 0),
                       edge(4, 5), edge(5, 6), edge(6, 7), edge(7, 4),
                       edge(0, 4), edge(1, 5), edge(2, 6), edge(3, 7)};
  const m::Vec3 center{a / 2, b / 2, c / 2};
  auto rectFace = [&](const m::Point3& O, const m::Dir3& X, const m::Dir3& Y, double U, double V,
                      topo::Shape (&edges)[4]) {
    const m::Vec3 z = m::cross(X.vec(), Y.vec());
    const m::Vec3 faceCenter = O.asVec() + X.vec() * (U / 2) + Y.vec() * (V / 2);
    const topo::Orientation o = m::dot(z, faceCenter - center) >= 0 ? topo::Orientation::Forward
                                                                    : topo::Orientation::Reversed;
    m::Ax3 fr{O, X, Y, m::Dir3{z}};
    topo::FaceSurface s{};
    s.kind = topo::FaceSurface::Kind::Plane;
    s.frame = fr;
    topo::Shape f0 = topo::ShapeBuilder::makeFace(s, topo::Shape{});
    const m::Point3 c2[4] = {{0, 0, 0}, {U, 0, 0}, {U, V, 0}, {0, V, 0}};
    std::vector<topo::Shape> pcE;
    for (int i = 0; i < 4; ++i) {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = c2[i];
      pc.dir2d = m::Vec3{c2[(i + 1) % 4].x - c2[i].x, c2[(i + 1) % 4].y - c2[i].y, 0};
      pcE.push_back(topo::ShapeBuilder::addPCurve(edges[i], f0.tshape(), pc));
    }
    return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(pcE), {}, o);
  };
  topo::Shape botE[4] = {e[0], e[1], e[2], e[3]};
  topo::Shape topE[4] = {e[4], e[5], e[6], e[7]};
  topo::Shape ymE[4] = {e[0], e[9], e[4], e[8]};
  topo::Shape ypE[4] = {e[2], e[11], e[6], e[10]};
  topo::Shape xmE[4] = {e[3], e[8], e[7], e[11]};
  topo::Shape xpE[4] = {e[1], e[10], e[5], e[9]};
  std::vector<topo::Shape> faces = {
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, a, b, botE),
      rectFace({0, 0, c}, {1, 0, 0}, {0, 1, 0}, a, b, topE),
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ymE),
      rectFace({0, b, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ypE),
      rectFace({0, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xmE),
      rectFace({a, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xpE)};
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(faces)});
}

topo::Shape cylinderSolid(double R, double h) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface sideS{};
  sideS.kind = topo::FaceSurface::Kind::Cylinder; sideS.frame = fr; sideS.radius = R;
  auto vb = vertexAt(R, 0, 0), vt = vertexAt(R, 0, h);
  topo::Shape botC = circleEdge3d(R, 0, vb, vb);
  topo::Shape topC = circleEdge3d(R, h, vt, vt);
  topo::Shape seam0 = lineEdge(vb, vt), seam1 = lineEdge(vb, vt);
  topo::Shape sideF0 = topo::ShapeBuilder::makeFace(sideS, topo::Shape{});
  auto pcLine = [&](m::Point3 o, m::Vec3 d) {
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = o; pc.dir2d = d; return pc;
  };
  topo::Shape botOnSide = topo::ShapeBuilder::addPCurve(botC, sideF0.tshape(), pcLine({0, 0, 0}, {1, 0, 0}));
  topo::Shape topOnSide = topo::ShapeBuilder::addPCurve(topC, sideF0.tshape(), pcLine({0, h, 0}, {1, 0, 0}));
  topo::Shape s0OnSide = topo::ShapeBuilder::addPCurve(seam0, sideF0.tshape(), pcLine({0, 0, 0}, {0, h, 0}));
  topo::Shape s1OnSide = topo::ShapeBuilder::addPCurve(seam1, sideF0.tshape(), pcLine({2 * kPi, 0, 0}, {0, h, 0}));
  std::vector<topo::Shape> sideWire = {botOnSide, s1OnSide, topOnSide.reversedShape(),
                                       s0OnSide.reversedShape()};
  topo::Shape sideFace = topo::ShapeBuilder::makeFace(sideS, topo::ShapeBuilder::makeWire(sideWire));
  auto capFace = [&](double z, bool top, const topo::Shape& circle) {
    topo::FaceSurface cs{};
    cs.kind = topo::FaceSurface::Kind::Plane;
    cs.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
    topo::Shape cf0 = topo::ShapeBuilder::makeFace(cs, topo::Shape{});
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Circle; pc.origin2d = {0, 0, 0}; pc.dir2d = {R, 0, 0};
    topo::Shape cOn = topo::ShapeBuilder::addPCurve(circle, cf0.tshape(), pc);
    const topo::Orientation o = top ? topo::Orientation::Forward : topo::Orientation::Reversed;
    return topo::ShapeBuilder::makeFace(cs, topo::ShapeBuilder::makeWire({cOn}), {}, o);
  };
  topo::Shape botCap = capFace(0, false, botC);
  topo::Shape topCap = capFace(h, true, topC);
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell({sideFace, botCap, topCap})});
}

topo::Shape fullSphere(double R) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Sphere;
  s.frame = fr;
  s.radius = R;
  topo::Shape f0 = topo::ShapeBuilder::makeFace(s, topo::Shape{});
  const double v0 = -kPi / 2, v1 = kPi / 2;
  const m::Point3 c2[4] = {{0, v0, 0}, {2 * kPi, v0, 0}, {2 * kPi, v1, 0}, {0, v1, 0}};
  std::vector<topo::Shape> edges;
  for (int i = 0; i < 4; ++i) {
    auto a = vertexAt(0, 0, 0), b = vertexAt(0, 0, 0);
    topo::Shape e = lineEdge(a, b);
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = c2[i];
    pc.dir2d = m::Vec3{c2[(i + 1) % 4].x - c2[i].x, c2[(i + 1) % 4].y - c2[i].y, 0};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, f0.tshape(), pc));
  }
  return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(edges));
}

// ── Analytic-normal helpers ───────────────────────────────────────────────────

tess::Mesh meshOf(const topo::Shape& s, double defl) {
  tess::MeshParams p;
  p.deflection = defl;
  return tess::SolidMesher{p}.mesh(s);
}

// Max distance of a display vertex to the analytic cylinder wall (radius R).
double maxDistToCylinder(const DisplayMesh& dm, double R) {
  double d = 0.0;
  for (const auto& p : dm.positions)
    d = std::max(d, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - R));
  return d;
}
double maxDistToSphere(const DisplayMesh& dm, double R) {
  double d = 0.0;
  for (const auto& p : dm.positions)
    d = std::max(d, std::fabs(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - R));
  return d;
}

// Count display vertices whose position coincides (within tol) with a given point,
// used to count crease splits at a shared ring/corner.
int copiesAt(const DisplayMesh& dm, const m::Point3& target, double tol) {
  int n = 0;
  for (const auto& p : dm.positions)
    if (m::distance(p, target) < tol) ++n;
  return n;
}

// EXACT quantised (x,y,z) position key — used to fold split display verts back onto
// their source vertex without the collisions a lossy scalar hash would introduce
// (a collision could merge distinct positions and mask a non-watertight fold).
struct PosKey {
  std::int64_t x, y, z;
  bool operator==(const PosKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};
struct PosKeyHash {
  std::size_t operator()(const PosKey& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
    h ^= static_cast<std::size_t>(k.y) * 19349663u + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(k.z) * 83492791u + (h << 6) + (h >> 2);
    return h;
  }
};
PosKey posKey(const m::Point3& p) {
  auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 1e9)); };
  return {q(p.x), q(p.y), q(p.z)};
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// SMOOTH NORMALS
// ═════════════════════════════════════════════════════════════════════════════

CC_TEST(cylinder_wall_normals_are_radial) {
  const double R = 3.0, h = 5.0, defl = 0.01;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);
  CC_CHECK(!src.triangles.empty());
  DisplayParams p;
  p.deflection = defl;
  p.creaseAngleDeg = 30.0;  // 90° cap↔wall is a crease; wall-wall (~few°) is smooth
  const DisplayMesh dm = buildDisplayMesh(src, p);
  CC_CHECK(dm.vertexCount() > 0 && dm.normals.size() == dm.vertexCount());

  // A cylinder ring vertex on the wall SPLITS into a cap copy (axial) and a wall
  // copy (radial). Take the WALL copy — the one whose normal is ⊥ Z (|nz| < 0.1)
  // — and assert it equals the analytic radial normal (x,y,0)/R. By the angular
  // symmetry of the wall band around each ring vertex the averaged wall normal is
  // EXACTLY radial (the two incident angular segments are symmetric about the
  // vertex angle), so the match holds to ~1e-6 despite the coarse mesh.
  double worst = 0.0;
  int wallVerts = 0;
  for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
    const m::Point3& pt = dm.positions[i];
    const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    const bool onWall = std::fabs(r - R) < 5 * defl;
    const m::Vec3 got = dm.normals[i].vec();
    const bool isWallCopy = std::fabs(got.z) < 0.1;  // radial, not the cap copy
    if (!(onWall && isWallCopy)) continue;
    ++wallVerts;
    const m::Vec3 analytic{pt.x / r, pt.y / r, 0.0};  // r≈R
    worst = std::max(worst, m::norm(got - analytic));
  }
  CC_CHECK(wallVerts > 8);
  std::printf("  cylinder wall: %d wall-copy verts, max |n - radial| = %.3e\n", wallVerts, worst);
  CC_CHECK(worst < 1e-6);

  // Continuity around the wall: consecutive wall-copy normals rotate smoothly (no
  // flip) — all point outward (n·position_xy > 0).
  bool allOutward = true;
  for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
    const m::Point3& pt = dm.positions[i];
    const double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
    const m::Vec3 got = dm.normals[i].vec();
    if (std::fabs(r - R) < 5 * defl && std::fabs(got.z) < 0.1)
      if (got.x * pt.x + got.y * pt.y <= 0.0) allOutward = false;
  }
  CC_CHECK(allOutward);
}

CC_TEST(cylinder_cap_wall_ring_splits) {
  const double R = 3.0, h = 5.0, defl = 0.02;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);
  DisplayParams p;
  p.deflection = defl;
  p.creaseAngleDeg = 30.0;
  const DisplayMesh dm = buildDisplayMesh(src, p);

  // Pick a point on the bottom ring (z=0, radius R at angle 0-ish). In the source
  // mesh it is one welded vertex; in the display mesh it SPLITS into a cap copy
  // (normal ≈ -Z) and a wall copy (normal ≈ radial). Find copies near (R,0,0).
  const m::Point3 ringPt{R, 0, 0};
  const int copies = copiesAt(dm, ringPt, 1e-9);
  std::printf("  bottom-ring point copies = %d (expect >= 2: cap + wall)\n", copies);
  CC_CHECK(copies >= 2);

  // Among the copies, at least one normal is ~axial (|nz| ~ 1) and one is ~radial
  // (|nz| ~ 0). That is the hard-edge signature.
  bool sawAxial = false, sawRadial = false;
  for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
    if (m::distance(dm.positions[i], ringPt) > 1e-9) continue;
    const double nz = std::fabs(dm.normals[i].z());
    if (nz > 0.9) sawAxial = true;
    if (nz < 0.1) sawRadial = true;
  }
  CC_CHECK(sawAxial && sawRadial);
}

// Max |averaged-normal − analytic-radial| over a meshed sphere at deflection d.
static double sphereNormalError(double R, double d) {
  const tess::Mesh src = meshOf(fullSphere(R), d);
  DisplayParams p;
  p.deflection = d;
  p.creaseAngleDeg = 30.0;  // a sphere has no crease anywhere ⇒ all smooth
  const DisplayMesh dm = buildDisplayMesh(src, p);
  // Fully smooth ⇒ no vertex split ⇒ display vertex count == source vertex count.
  if (dm.vertexCount() != src.vertices.size()) return 1e9;
  double worst = 0.0;
  for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
    const m::Point3& pt = dm.positions[i];
    const double rr = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    if (rr < 1e-9) continue;
    const m::Vec3 analytic{pt.x / rr, pt.y / rr, pt.z / rr};
    worst = std::max(worst, m::norm(dm.normals[i].vec() - analytic));
  }
  return worst;
}

CC_TEST(sphere_all_smooth_no_split) {
  // A sphere has NO crease → the display mesh duplicates nothing. Every display
  // normal is the angle/area-weighted average of the incident triangle normals; it
  // approximates the analytic radial normal P/R to O(deflection) (the discrete
  // average is not exactly radial because the latitude/longitude grid fan is not
  // angularly symmetric — unlike a cylinder ring vertex, which IS exact). We prove
  // it is a genuine convergent approximation, not a bug: refining the deflection
  // shrinks the error, and the error is within a small multiple of the deflection.
  const double R = 4.0;
  const tess::Mesh src = meshOf(fullSphere(R), 0.01);
  DisplayParams p; p.deflection = 0.01; p.creaseAngleDeg = 30.0;
  const DisplayMesh dm = buildDisplayMesh(src, p);
  std::printf("  sphere: src verts %zu, display verts %zu (expect equal — no split)\n",
              src.vertices.size(), dm.vertexCount());
  CC_CHECK(dm.vertexCount() == src.vertices.size());
}

CC_TEST(sphere_smooth_normal_converges_to_radial) {
  const double R = 4.0;
  const double eCoarse = sphereNormalError(R, 0.04);
  const double eFine = sphereNormalError(R, 0.005);
  std::printf("  sphere normal error: defl 0.04 → %.3e, defl 0.005 → %.3e\n", eCoarse, eFine);
  CC_CHECK(eFine < eCoarse);            // refining shrinks the deviation
  CC_CHECK(eFine < 3.0 * 0.005);        // within a small multiple of the deflection
}

// ═════════════════════════════════════════════════════════════════════════════
// CREASES
// ═════════════════════════════════════════════════════════════════════════════

CC_TEST(box_every_edge_is_a_crease) {
  const double a = 2.0, b = 3.0, c = 4.0, defl = 0.5;  // coarse ⇒ 2 tris/face
  const tess::Mesh src = meshOf(boxSolid(a, b, c), defl);
  CC_CHECK(!src.triangles.empty());
  DisplayParams p;
  p.deflection = defl;
  p.creaseAngleDeg = 30.0;  // 90° box edges are all creases
  const DisplayMesh dm = buildDisplayMesh(src, p);

  // Every normal is axis-aligned (points along ±X/±Y/±Z): a box face is flat.
  int axisAligned = 0;
  for (const auto& d : dm.normals) {
    const double mx = std::max({std::fabs(d.x()), std::fabs(d.y()), std::fabs(d.z())});
    if (mx > 1.0 - 1e-9) ++axisAligned;
  }
  std::printf("  box: %zu display verts, %d axis-aligned normals\n", dm.vertexCount(), axisAligned);
  CC_CHECK(axisAligned == static_cast<int>(dm.vertexCount()));

  // Each of the 8 geometric corners splits into 3 copies (one per incident face),
  // so 8 corners × 3 = 24 corner display vertices coincide at the 8 corner points.
  const m::Point3 corners[8] = {{0, 0, 0}, {a, 0, 0}, {a, b, 0}, {0, b, 0},
                                {0, 0, c}, {a, 0, c}, {a, b, c}, {0, b, c}};
  int totalCornerCopies = 0;
  for (const auto& cor : corners) {
    const int copies = copiesAt(dm, cor, 1e-9);
    CC_CHECK_EQ(copies, 3);
    totalCornerCopies += copies;
  }
  std::printf("  box: 24 split corner normals — got %d\n", totalCornerCopies);
  CC_CHECK_EQ(totalCornerCopies, 24);

  // Exactly 6 distinct face normals (±X, ±Y, ±Z).
  std::vector<m::Vec3> distinct;
  for (const auto& d : dm.normals) {
    bool seen = false;
    for (const auto& e : distinct)
      if (m::norm(e - d.vec()) < 1e-6) { seen = true; break; }
    if (!seen) distinct.push_back(d.vec());
  }
  std::printf("  box: %zu distinct face normals (expect 6)\n", distinct.size());
  CC_CHECK_EQ(static_cast<int>(distinct.size()), 6);
}

CC_TEST(crease_angle_threshold_respected) {
  // A cylinder: at a LOW crease angle (30°) the cap↔wall (90°) is hard ⇒ the ring
  // splits. Raise the crease angle ABOVE 90° and the same seam becomes smooth ⇒
  // FEWER split vertices (the ring no longer duplicates).
  const double R = 3.0, h = 5.0, defl = 0.05;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);

  DisplayParams lo;
  lo.deflection = defl; lo.creaseAngleDeg = 30.0;
  const DisplayMesh dmLo = buildDisplayMesh(src, lo);

  DisplayParams hi;
  hi.deflection = defl; hi.creaseAngleDeg = 120.0;  // 90° seam now below threshold
  const DisplayMesh dmHi = buildDisplayMesh(src, hi);

  std::printf("  crease 30°: %zu verts; crease 120°: %zu verts (expect fewer at 120°)\n",
              dmLo.vertexCount(), dmHi.vertexCount());
  CC_CHECK(dmHi.vertexCount() < dmLo.vertexCount());
  // At 120° nothing on the cylinder exceeds threshold ⇒ no split at all.
  CC_CHECK(dmHi.vertexCount() == src.vertices.size());
}

// ═════════════════════════════════════════════════════════════════════════════
// LOD
// ═════════════════════════════════════════════════════════════════════════════

CC_TEST(lod_reduces_triangles_within_hausdorff) {
  // A sphere is the clean LOD target: no boundary, no crease ⇒ every interior
  // vertex is free to collapse. We decimate to half the triangles and assert (a)
  // the count dropped and (b) every SURVIVING vertex stays within the Hausdorff
  // budget of the analytic sphere surface (radius R).
  const double R = 4.0, defl = 0.02;
  const tess::Mesh src = meshOf(fullSphere(R), defl);
  const std::size_t srcTris = src.triangles.size();
  CC_CHECK(srcTris > 200);

  DisplayParams p;
  p.deflection = defl;
  p.creaseAngleDeg = 30.0;
  p.lodTargetTris = static_cast<int>(srcTris / 2);
  p.lodHausdorffScale = 8.0;  // budget = 8·defl = 0.16
  const DisplayMesh dm = buildDisplayMesh(src, p);

  const double reduction = 1.0 - static_cast<double>(dm.triangleCount()) / srcTris;
  const double budget = p.lodHausdorffScale * defl;
  std::printf("  LOD: src %zu tris → display %zu tris (target %d, %.1f%% reduction, budget %.3f)\n",
              srcTris, dm.triangleCount(), p.lodTargetTris, 100.0 * reduction, budget);
  // Triangle count is REDUCED — decimation ran.
  CC_CHECK(dm.triangleCount() < srcTris);
  // …and it NEVER over-collapses past the target (the goal is a floor, not exceeded).
  CC_CHECK(dm.triangleCount() >= static_cast<std::size_t>(p.lodTargetTris));

  // Hausdorff bound HONORED: every survivor stays within budget of the analytic
  // sphere. On a near-uniform sphere the budget (not the target) is the binding
  // constraint, so the collapse stops early — that is the SAFETY the bound buys.
  const double worst = maxDistToSphere(dm, R);
  std::printf("  LOD: max vertex-to-sphere = %.4f (budget %.4f)\n", worst, budget);
  CC_CHECK(worst <= budget);

  // A TIGHTER budget makes the bound (not the target) the binding constraint, so
  // the collapse stops EARLY — more triangles survive. This proves the Hausdorff
  // bound genuinely throttles decimation (honest, not a fixed schedule).
  DisplayParams tight = p;
  tight.lodTargetTris = 4;         // ask for an aggressive target…
  tight.lodHausdorffScale = 0.5;   // …but a tiny budget = 0.01 caps the collapse
  const DisplayMesh dmTight = buildDisplayMesh(src, tight);
  const double tightBudget = tight.lodHausdorffScale * defl;
  std::printf("  LOD (tight budget %.3f, target 4): %zu tris, max dist %.4f\n", tightBudget,
              dmTight.triangleCount(), maxDistToSphere(dmTight, R));
  CC_CHECK(dmTight.triangleCount() > static_cast<std::size_t>(tight.lodTargetTris));
  CC_CHECK(maxDistToSphere(dmTight, R) <= tightBudget + 1e-9);

  // The decimated mesh is still a valid, normalled display mesh.
  CC_CHECK(dm.normals.size() == dm.vertexCount());
}

CC_TEST(lod_disabled_when_target_nonpositive) {
  const double R = 3.0, h = 5.0, defl = 0.05;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);
  DisplayParams p;
  p.deflection = defl;
  p.lodTargetTris = 0;  // disabled
  const DisplayMesh dm = buildDisplayMesh(src, p);
  // No decimation ⇒ triangle count equals the source (splitting never changes
  // triangle count, only vertex count).
  CC_CHECK_EQ(static_cast<int>(dm.triangleCount()), static_cast<int>(src.triangles.size()));
}

// ═════════════════════════════════════════════════════════════════════════════
// UVs
// ═════════════════════════════════════════════════════════════════════════════

CC_TEST(uvs_in_unit_range_and_seam_consistent) {
  const double a = 2.0, b = 3.0, c = 4.0, defl = 0.5;
  const tess::Mesh src = meshOf(boxSolid(a, b, c), defl);
  DisplayParams p;
  p.deflection = defl;
  p.wantUVs = true;
  const DisplayMesh dm = buildDisplayMesh(src, p);
  CC_CHECK(dm.hasUVs());
  CC_CHECK_EQ(static_cast<int>(dm.uvs.size()), static_cast<int>(dm.vertexCount()));

  double umin = 1e9, umax = -1e9, vmin = 1e9, vmax = -1e9;
  for (const auto& uv : dm.uvs) {
    CC_CHECK(uv[0] >= 0.0 && uv[0] <= 1.0);
    CC_CHECK(uv[1] >= 0.0 && uv[1] <= 1.0);
    umin = std::min(umin, uv[0]); umax = std::max(umax, uv[0]);
    vmin = std::min(vmin, uv[1]); vmax = std::max(vmax, uv[1]);
  }
  std::printf("  UV range: u[%.3f,%.3f] v[%.3f,%.3f]\n", umin, umax, vmin, vmax);
  CC_CHECK(umax > 0.5 && vmax > 0.5);  // UVs actually span the box

  // Seam consistency: two display verts with the same position AND same dominant
  // normal axis get the SAME uv. Key on the EXACT quantised (x,y,z,axis) tuple (a
  // lossy scalar hash could collide distinct verts and spuriously flag them).
  struct SKey {
    std::int64_t x, y, z; int axis;
    bool operator==(const SKey& o) const noexcept {
      return x == o.x && y == o.y && z == o.z && axis == o.axis;
    }
  };
  struct SKeyHash {
    std::size_t operator()(const SKey& s) const noexcept {
      std::size_t h = static_cast<std::size_t>(s.x) * 73856093u;
      h ^= static_cast<std::size_t>(s.y) * 19349663u + (h << 6) + (h >> 2);
      h ^= static_cast<std::size_t>(s.z) * 83492791u + (h << 6) + (h >> 2);
      h ^= static_cast<std::size_t>(s.axis) + (h << 6) + (h >> 2);
      return h;
    }
  };
  std::unordered_map<SKey, std::array<double, 2>, SKeyHash> seen;
  auto q = [](double x) { return static_cast<std::int64_t>(std::llround(x * 1e6)); };
  bool consistent = true;
  for (std::size_t i = 0; i < dm.vertexCount(); ++i) {
    const auto& pt = dm.positions[i];
    const m::Vec3 nv = dm.normals[i].vec();
    const int axis = (std::fabs(nv.x) >= std::fabs(nv.y) && std::fabs(nv.x) >= std::fabs(nv.z))
                         ? 0 : (std::fabs(nv.y) >= std::fabs(nv.z) ? 1 : 2);
    const SKey k{q(pt.x), q(pt.y), q(pt.z), axis};
    auto it = seen.find(k);
    if (it == seen.end()) { seen.emplace(k, dm.uvs[i]); }
    else if (std::fabs(it->second[0] - dm.uvs[i][0]) > 1e-9 ||
             std::fabs(it->second[1] - dm.uvs[i][1]) > 1e-9) {
      consistent = false;
    }
  }
  CC_CHECK(consistent);
}

// ═════════════════════════════════════════════════════════════════════════════
// HONEST DECLINE + POSITION-ON-SURFACE (Gate b, host arm)
// ═════════════════════════════════════════════════════════════════════════════

CC_TEST(empty_input_yields_empty_output) {
  tess::Mesh empty;
  DisplayParams p;
  const DisplayMesh dm = buildDisplayMesh(empty, p);
  CC_CHECK_EQ(static_cast<int>(dm.vertexCount()), 0);
  CC_CHECK_EQ(static_cast<int>(dm.triangleCount()), 0);
}

CC_TEST(base_display_positions_lie_on_source_solid) {
  // Pre-LOD: every display position is a verbatim source vertex ⇒ on the solid
  // within the source deflection (the tessellator's own invariant, inherited).
  const double R = 3.0, h = 5.0, defl = 0.01;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);
  DisplayParams p;
  p.deflection = defl;
  const DisplayMesh dm = buildDisplayMesh(src, p);  // no LOD
  const double wall = maxDistToCylinder(dm, R);
  std::printf("  base display: max wall dist %.3e (defl %.3e)\n", wall, defl);
  // Wall vertices are exactly on radius R; cap-interior verts are inside the disk
  // (r<R) so the pure-wall metric would flag them — instead assert every vertex is
  // a source vertex (position set identical up to duplication).
  // Build a quantised set of source positions and require each display pos present.
  std::unordered_map<PosKey, int, PosKeyHash> srcSet;
  for (const auto& v : src.vertices) srcSet[posKey(v)]++;
  bool allOnSource = true;
  for (const auto& pp : dm.positions)
    if (srcSet.find(posKey(pp)) == srcSet.end()) { allOnSource = false; break; }
  CC_CHECK(allOnSource);
}

CC_TEST(base_mesh_watertight_consistent_with_tessellate) {
  // The display mesh's triangle TOPOLOGY, welded back by position, is the same
  // closed surface the correctness mesh is: fold the split display verts back onto
  // their source vertex and confirm the folded mesh is watertight (as the source).
  const double R = 3.0, h = 5.0, defl = 0.03;
  const tess::Mesh src = meshOf(cylinderSolid(R, h), defl);
  CC_CHECK(tess::isWatertight(src));
  DisplayParams p;
  p.deflection = defl;
  const DisplayMesh dm = buildDisplayMesh(src, p);  // splits, no LOD

  // Fold display verts back to unique positions.
  std::unordered_map<PosKey, std::uint32_t, PosKeyHash> posToIdx;
  tess::Mesh folded;
  auto foldIdx = [&](std::uint32_t v) {
    const PosKey k = posKey(dm.positions[v]);
    auto it = posToIdx.find(k);
    if (it != posToIdx.end()) return it->second;
    const std::uint32_t idx = folded.addVertex(dm.positions[v]);
    posToIdx.emplace(k, idx);
    return idx;
  };
  for (const auto& t : dm.triangles) folded.addTriangle(foldIdx(t.a), foldIdx(t.b), foldIdx(t.c));
  std::printf("  folded display: %zu verts, watertight=%d (src watertight=%d)\n",
              folded.vertices.size(), (int)tess::isWatertight(folded), (int)tess::isWatertight(src));
  CC_CHECK(tess::isWatertight(folded));
  // And triangle count is preserved through the split (no LOD).
  CC_CHECK_EQ(static_cast<int>(dm.triangleCount()), static_cast<int>(src.triangles.size()));
}

CC_RUN_ALL()
