// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native tessellator (Phase 4, capability #3
// `native-tessellation`). OCCT-FREE — Gate 1 of the two-gate verification model
// (openspec/NATIVE-REWRITE.md): the native code compiles and unit-tests with
// clang++ -std=c++20, no OCCT, no simulator.
//
// Tessellation is an APPROXIMATION, so the tests assert TOLERANCE-BASED
// PROPERTIES (not triangle-identical output):
//   * a planar quad meshes to its EXACT area (a plane has zero deflection);
//   * every produced vertex lies ON the analytic surface (distance ≈ 0);
//   * a box solid meshes watertight; its area/volume equal the closed form
//     (planar ⇒ exact);
//   * a cylinder / sphere mesh is watertight and its area/volume converge to the
//     closed-form value as the deflection shrinks (finer ⇒ smaller error), and
//     stay within a deflection-derived bound;
//   * TRIMMING: a square face with a square hole drops the triangles inside the
//     hole (meshed area ≈ outer − hole);
//   * mesh property primitives (surfaceArea, enclosedVolume, isWatertight) are
//     correct on hand-built meshes.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_tessellate.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_tessellate
//
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace m = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── Shape builders ────────────────────────────────────────────────────────────

// A vertex handle at (x,y,z).
topo::Shape vertexAt(double x, double y, double z) {
  return topo::ShapeBuilder::makeVertex(m::Point3{x, y, z});
}

// A line edge between two vertices (curve geometry not needed for area/volume
// property checks — the face surface drives the mesh; edges carry pcurves for
// trimming, added separately where needed).
topo::Shape lineEdge(const topo::Shape& a, const topo::Shape& b) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Line;
  return topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, a, b);
}

// A planar rectangle face on the XY plane, [0,w]×[0,h], with a UV-line pcurve on
// each edge so trimming has a real outer polygon. The plane's (u,v) == (x,y).
topo::Shape planarRect(double w, double h) {
  const m::Ax3 xy{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface srf{};
  srf.kind = topo::FaceSurface::Kind::Plane;
  srf.frame = xy;

  auto v00 = vertexAt(0, 0, 0), v10 = vertexAt(w, 0, 0);
  auto v11 = vertexAt(w, h, 0), v01 = vertexAt(0, h, 0);
  topo::Shape e[4] = {lineEdge(v00, v10), lineEdge(v10, v11), lineEdge(v11, v01),
                      lineEdge(v01, v00)};

  // First build the face so its surface node exists, then re-lay pcurves onto it.
  // Simpler: build wire from edges carrying pcurves directly against the surface
  // node we will attach. We create the face node first via a temporary, then add
  // pcurves keyed to that node.
  topo::Shape wire0 = topo::ShapeBuilder::makeWire({e[0], e[1], e[2], e[3]});
  topo::Shape face = topo::ShapeBuilder::makeFace(srf, wire0);

  // Attach a UV line pcurve to each edge on this face's surface node, matching
  // the rectangle boundary in (u,v)=(x,y): the pcurves trace the outer loop.
  const m::Point3 corners[4] = {{0, 0, 0}, {w, 0, 0}, {w, h, 0}, {0, h, 0}};
  std::vector<topo::Shape> pcEdges;
  for (int i = 0; i < 4; ++i) {
    const m::Point3& p0 = corners[i];
    const m::Point3& p1 = corners[(i + 1) % 4];
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = m::Point3{p0.x, p0.y, 0};
    pc.dir2d = m::Vec3{p1.x - p0.x, p1.y - p0.y, 0};  // param t∈[0,1] over [first,last]
    pcEdges.push_back(topo::ShapeBuilder::addPCurve(e[i], face.tshape(), pc));
  }
  topo::Shape wire = topo::ShapeBuilder::makeWire(pcEdges);
  return topo::ShapeBuilder::makeFace(srf, wire);
}

// A planar square [0,S]×[0,S] with a centered square hole of side hS, both as UV
// line-pcurve loops. Outer CCW, hole CW (orientation drives sampling direction).
topo::Shape planarSquareWithHole(double S, double hS) {
  const m::Ax3 xy{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface srf{};
  srf.kind = topo::FaceSurface::Kind::Plane;
  srf.frame = xy;
  topo::Shape face0 = topo::ShapeBuilder::makeFace(srf, topo::Shape{});

  auto loop = [&](const m::Point3 (&c)[4]) {
    std::vector<topo::Shape> edges;
    for (int i = 0; i < 4; ++i) {
      auto a = vertexAt(c[i].x, c[i].y, 0), b = vertexAt(c[(i + 1) % 4].x, c[(i + 1) % 4].y, 0);
      topo::Shape e = lineEdge(a, b);
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = m::Point3{c[i].x, c[i].y, 0};
      pc.dir2d = m::Vec3{c[(i + 1) % 4].x - c[i].x, c[(i + 1) % 4].y - c[i].y, 0};
      edges.push_back(topo::ShapeBuilder::addPCurve(e, face0.tshape(), pc));
    }
    return topo::ShapeBuilder::makeWire(edges);
  };

  const m::Point3 outer[4] = {{0, 0, 0}, {S, 0, 0}, {S, S, 0}, {0, S, 0}};
  const double lo = (S - hS) / 2.0, hi = (S + hS) / 2.0;
  const m::Point3 hole[4] = {{lo, lo, 0}, {lo, hi, 0}, {hi, hi, 0}, {hi, lo, 0}};  // CW
  return topo::ShapeBuilder::makeFace(srf, loop(outer), {loop(hole)});
}

// A closed box solid [0,a]×[0,b]×[0,c] with shared vertices/edges — 6 planar
// faces oriented outward. Built bottom-up (Mäntylä order) so edges are shared
// between adjacent faces (the shared-node stitching path).
topo::Shape boxSolid(double a, double b, double c) {
  // 8 corner vertices.
  topo::Shape v[8] = {vertexAt(0, 0, 0), vertexAt(a, 0, 0), vertexAt(a, b, 0), vertexAt(0, b, 0),
                      vertexAt(0, 0, c), vertexAt(a, 0, c), vertexAt(a, b, c), vertexAt(0, b, c)};
  auto edge = [&](int i, int j) { return lineEdge(v[i], v[j]); };
  // 12 shared edges.
  topo::Shape e[12] = {edge(0, 1), edge(1, 2), edge(2, 3), edge(3, 0),  // bottom 0..3
                       edge(4, 5), edge(5, 6), edge(6, 7), edge(7, 4),  // top 4..7
                       edge(0, 4), edge(1, 5), edge(2, 6), edge(3, 7)}; // verticals 8..11

  // A planar face from a placement + four ordered edges (orientation chosen so
  // the outward normal = +axis; for area/volume the exact orientation per face is
  // not needed since enclosedVolume uses the signed sum with consistent outward
  // winding derived from face orientation).
  auto planeFace = [&](const m::Ax3& fr, std::initializer_list<int> edgeIdx,
                       topo::Orientation o) {
    std::vector<topo::Shape> es;
    for (int i : edgeIdx) es.push_back(e[i]);
    topo::FaceSurface s{};
    s.kind = topo::FaceSurface::Kind::Plane;
    s.frame = fr;
    return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(es), {}, o);
  };

  // Frames: origin + X + Y (+Z = outward normal). Untrimmed (no pcurves) ⇒ the
  // mesher uses each plane's natural bounds — but a plane's natural bounds are a
  // unit box, which would NOT span the face. So we give every face a trim polygon
  // via pcurves matching its rectangle. To keep the test focused, we instead trim
  // by attaching pcurves per face below.
  // For watertightness/volume we need each face to span its real rectangle. We
  // build faces WITH pcurves so buildRegion yields the true UV box.

  const m::Vec3 center{a / 2, b / 2, c / 2};

  // Helper to make a rectangular planar face spanning [0,U]x[0,V] in its own
  // frame, with UV line pcurves for the four edges (so trimming gives the right
  // box). The face orientation is chosen automatically so its EFFECTIVE outward
  // normal (frame Z if Forward, −Z if Reversed) points away from the box center —
  // giving the solid a consistent outward orientation without hand-reasoning each
  // face (the property enclosedVolume depends on).
  auto rectFace = [&](const m::Point3& O, const m::Dir3& X, const m::Dir3& Y, double U, double V,
                      topo::Shape (&edges)[4]) {
    const m::Vec3 z = m::cross(X.vec(), Y.vec());
    const m::Vec3 faceCenter = O.asVec() + X.vec() * (U / 2) + Y.vec() * (V / 2);
    const m::Vec3 outward = faceCenter - center;
    const topo::Orientation o =
        m::dot(z, outward) >= 0 ? topo::Orientation::Forward : topo::Orientation::Reversed;

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

  // Assemble the 6 faces sharing edges. Edge groups per face (in loop order):
  topo::Shape botE[4] = {e[0], e[1], e[2], e[3]};
  topo::Shape topE[4] = {e[4], e[5], e[6], e[7]};
  topo::Shape ymE[4] = {e[0], e[9], e[4], e[8]};   // y=0 face
  topo::Shape ypE[4] = {e[2], e[11], e[6], e[10]}; // y=b face
  topo::Shape xmE[4] = {e[3], e[8], e[7], e[11]};  // x=0 face
  topo::Shape xpE[4] = {e[1], e[10], e[5], e[9]};  // x=a face

  std::vector<topo::Shape> faces = {
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 1, 0}, a, b, botE),  // z=0
      rectFace({0, 0, c}, {1, 0, 0}, {0, 1, 0}, a, b, topE),  // z=c
      rectFace({0, 0, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ymE),   // y=0
      rectFace({0, b, 0}, {1, 0, 0}, {0, 0, 1}, a, c, ypE),   // y=b
      rectFace({0, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xmE),   // x=0
      rectFace({a, 0, 0}, {0, 1, 0}, {0, 0, 1}, b, c, xpE)};  // x=a

  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(faces)});
}

// A circle edge in 3D: center on the Z axis at height z, radius R, param u∈[0,2π].
// (Faithful curved boundary — the two-stage mesher discretizes it once, by its 3D
// curvature, and pins the face boundary to those samples. A dummy straight line
// would leave the u-direction unsubdivided at the boundary.)
topo::Shape circleEdge3d(double R, double z, const topo::Shape& v0, const topo::Shape& v1) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  c.radius = R;
  return topo::ShapeBuilder::makeEdgeWithVertices(c, 0.0, 2 * kPi, {v0, v1});
}

// A full cylinder side face (no caps) as a single trimmed cylindrical face:
// U∈[0,2π], v∈[0,height]. Radius R. The outer wire is bottom-circle (v=0) →
// seam up (u=2π) → top-circle (v=h) → seam down (u=0). The two CIRCLE edges are
// real 3D curves, so the shared per-edge discretization subdivides the u-direction
// of the boundary (the same discretization a cap would share in a full solid).
topo::Shape cylinderSide(double R, double h) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Cylinder;
  s.frame = fr;
  s.radius = R;
  topo::Shape f0 = topo::ShapeBuilder::makeFace(s, topo::Shape{});

  auto vb = vertexAt(R, 0, 0), vt = vertexAt(R, 0, h);
  topo::Shape botC = circleEdge3d(R, 0, vb, vb);
  topo::Shape topC = circleEdge3d(R, h, vt, vt);
  topo::Shape seam0 = lineEdge(vb, vt), seam1 = lineEdge(vb, vt);

  auto pcLine = [&](m::Point3 o, m::Vec3 d) {
    topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = o; pc.dir2d = d; return pc;
  };
  // Bottom circle → (u,0) with u∈[0,2π]; top → (u,h); seams → vertical lines.
  topo::Shape botOn = topo::ShapeBuilder::addPCurve(botC, f0.tshape(), pcLine({0, 0, 0}, {1, 0, 0}));
  topo::Shape topOn = topo::ShapeBuilder::addPCurve(topC, f0.tshape(), pcLine({0, h, 0}, {1, 0, 0}));
  topo::Shape s0On = topo::ShapeBuilder::addPCurve(seam0, f0.tshape(), pcLine({0, 0, 0}, {0, h, 0}));
  topo::Shape s1On = topo::ShapeBuilder::addPCurve(seam1, f0.tshape(), pcLine({2 * kPi, 0, 0}, {0, h, 0}));
  std::vector<topo::Shape> edges = {botOn, s1On, topOn.reversedShape(), s0On.reversedShape()};
  return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(edges));
}

// A CLOSED cylinder solid: two planar cap disks + one cylindrical side, SHARING
// the two circular edges (curved shared edges — the case that only welds watertight
// with the shared per-edge discretization; independent per-face grids left it open).
// This is the direct regression for the curved shared-edge stitch (Phase 4 #3).
topo::Shape cylinderSolid(double R, double h) {
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface sideS{};
  sideS.kind = topo::FaceSurface::Kind::Cylinder; sideS.frame = fr; sideS.radius = R;

  // Shared vertices and the two SHARED circular edges (one node each; both the side
  // and a cap reference the SAME edge node ⇒ one shared discretization).
  auto vb = vertexAt(R, 0, 0), vt = vertexAt(R, 0, h);
  topo::Shape botC = circleEdge3d(R, 0, vb, vb);
  topo::Shape topC = circleEdge3d(R, h, vt, vt);
  topo::Shape seam0 = lineEdge(vb, vt), seam1 = lineEdge(vb, vt);

  // SIDE face: cylinder surface, pcurves map the two circles to v=0 / v=h lines and
  // the seams to the u=0 / u=2π verticals.
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

  // CAP faces: planar disks; the shared circle edge's pcurve on a cap is a circle in
  // (u,v)=(x,y) (center 0, radius R). Orientation chosen so normals point outward.
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

  return topo::ShapeBuilder::makeSolid(
      {topo::ShapeBuilder::makeShell({sideFace, botCap, topCap})});
}

// Max distance of every mesh vertex to a plane through the origin with the given
// unit normal (for a plane face, all vertices should have ~0 signed distance).
double maxDistToPlaneZ0(const Mesh& mesh) {
  double d = 0.0;
  for (const auto& p : mesh.vertices) d = std::max(d, std::fabs(p.z));
  return d;
}

// Max distance of every mesh vertex to a cylinder of radius R about the Z axis.
double maxDistToCylinder(const Mesh& mesh, double R) {
  double d = 0.0;
  for (const auto& p : mesh.vertices)
    d = std::max(d, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - R));
  return d;
}

// A full sphere of radius R centered at the origin, as ONE spherical face over
// u∈[0,2π], v∈[-π/2,π/2]. The u-seam and the two poles are shared by welding,
// so the meshed sphere is closed (watertight) for the property checks.
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

double maxDistToSphere(const Mesh& mesh, double R) {
  double d = 0.0;
  for (const auto& p : mesh.vertices)
    d = std::max(d, std::fabs(std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - R));
  return d;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// Mesh property primitives.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(mesh_area_of_unit_square) {
  // Two triangles covering [0,1]² → area 1.
  Mesh m;
  m.addVertex({0, 0, 0}); m.addVertex({1, 0, 0}); m.addVertex({1, 1, 0}); m.addVertex({0, 1, 0});
  m.addTriangle(0, 1, 2);
  m.addTriangle(0, 2, 3);
  CC_CHECK(std::fabs(surfaceArea(m) - 1.0) < 1e-12);
}

CC_TEST(mesh_volume_of_unit_cube) {
  // A hand-built unit-cube surface mesh → enclosed volume 1, watertight.
  Mesh m;
  const double c[8][3] = {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}};
  for (auto& p : c) m.addVertex({p[0], p[1], p[2]});
  auto quad = [&](int a, int b, int d, int e) { m.addTriangle(a, b, d); m.addTriangle(a, d, e); };
  quad(0,3,2,1);  // bottom (-Z), outward CCW seen from below
  quad(4,5,6,7);  // top (+Z)
  quad(0,1,5,4);  // -Y
  quad(2,3,7,6);  // +Y
  quad(1,2,6,5);  // +X
  quad(3,0,4,7);  // -X
  CC_CHECK(isWatertight(m));
  CC_CHECK(isTwoManifold(m));
  CC_CHECK(std::fabs(enclosedVolume(m) - 1.0) < 1e-12);
  CC_CHECK(std::fabs(surfaceArea(m) - 6.0) < 1e-12);
}

CC_TEST(mesh_open_patch_is_manifold_not_watertight) {
  Mesh m;
  m.addVertex({0, 0, 0}); m.addVertex({1, 0, 0}); m.addVertex({1, 1, 0}); m.addVertex({0, 1, 0});
  m.addTriangle(0, 1, 2);
  m.addTriangle(0, 2, 3);
  CC_CHECK(isTwoManifold(m));
  CC_CHECK(!isWatertight(m));  // boundary edges used once
  CC_CHECK(boundaryEdgeCount(m) == 4);
}

// ═════════════════════════════════════════════════════════════════════════════
// Face mesher — planar face: exact area, vertices on the surface.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(planar_face_exact_area_and_on_surface) {
  MeshParams p; p.deflection = 0.05;
  Mesh mesh = FaceMesher{p}.mesh(planarRect(3.0, 2.0));
  CC_CHECK(mesh.triangleCount() >= 2);
  CC_CHECK(std::fabs(surfaceArea(mesh) - 6.0) < 1e-9);   // exact: plane has no deflection
  CC_CHECK(maxDistToPlaneZ0(mesh) < 1e-9);               // vertices on the plane
}

// A planar SQUARE face (side 10, no holes) meshes to its EXACT area 100 (a plane
// has zero deflection ⇒ exact regardless of grid), and every vertex is coplanar
// (on the z=0 plane the face lives in).
CC_TEST(planar_square_side10_exact_area_100) {
  MeshParams p; p.deflection = 0.05;
  Mesh mesh = FaceMesher{p}.mesh(planarRect(10.0, 10.0));
  CC_CHECK(mesh.triangleCount() >= 2);
  CC_CHECK(std::fabs(surfaceArea(mesh) - 100.0) < 1e-9);  // side² = 100, exact
  CC_CHECK(maxDistToPlaneZ0(mesh) < 1e-9);                // all vertices coplanar
}

// ═════════════════════════════════════════════════════════════════════════════
// Trimming — a square with a square hole drops the hole's triangles.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(holed_face_trims_the_hole) {
  MeshParams p; p.deflection = 0.02;
  Mesh mesh = FaceMesher{p}.mesh(planarSquareWithHole(10.0, 4.0));
  const double expected = 10.0 * 10.0 - 4.0 * 4.0;  // 100 - 16 = 84
  const double area = surfaceArea(mesh);
  // Centroid-based trim ⇒ the hole boundary is resolved to grid resolution; allow
  // a few percent for the stair-stepped hole edge, tightened by the grid density.
  CC_CHECK(area < 100.0);                 // hole removed some area
  CC_CHECK(std::fabs(area - expected) < 0.06 * expected);
  CC_CHECK(maxDistToPlaneZ0(mesh) < 1e-9);
}

// ═════════════════════════════════════════════════════════════════════════════
// Curved face — cylinder side: vertices within deflection, area converges.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(cylinder_vertices_within_deflection) {
  const double R = 5.0, h = 12.0;
  MeshParams p; p.deflection = 0.1;
  Mesh mesh = FaceMesher{p}.mesh(cylinderSide(R, h));
  // Every vertex is S(u,v) exactly ⇒ distance to the cylinder is ~0 (not just
  // within deflection): grid vertices lie ON the true surface.
  CC_CHECK(maxDistToCylinder(mesh, R) < 1e-9);
  // True lateral area = 2πRh. A grid inscribes the cylinder, so the mesh area is
  // slightly LESS; the gap shrinks with deflection and is bounded.
  const double truth = 2.0 * kPi * R * h;
  const double area = surfaceArea(mesh);
  CC_CHECK(area <= truth + 1e-6);
  CC_CHECK(area > truth * 0.98);  // within 2% at defl 0.1
}

CC_TEST(cylinder_area_converges_with_deflection) {
  const double R = 5.0, h = 12.0, truth = 2.0 * kPi * R * h;
  auto areaAt = [&](double defl) {
    MeshParams p; p.deflection = defl;
    return surfaceArea(FaceMesher{p}.mesh(cylinderSide(R, h)));
  };
  const double coarse = std::fabs(areaAt(0.5) - truth);
  const double fine = std::fabs(areaAt(0.02) - truth);
  CC_CHECK(fine < coarse);          // finer deflection ⇒ closer to truth
  CC_CHECK(fine < 0.005 * truth);   // and tight in absolute terms
}

// ═════════════════════════════════════════════════════════════════════════════
// Solid mesher — a box is watertight with the exact area/volume (planar ⇒ exact).
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(box_solid_is_watertight_exact_volume) {
  const double a = 10.0, b = 20.0, c = 30.0;
  MeshParams p; p.deflection = 0.1;
  Mesh mesh = SolidMesher{p}.mesh(boxSolid(a, b, c));
  CC_CHECK(mesh.triangleCount() >= 12);
  CC_CHECK(isTwoManifold(mesh));
  CC_CHECK(isWatertight(mesh));                      // welding closed the shared edges
  CC_CHECK(std::fabs(surfaceArea(mesh) - 2.0 * (a*b + b*c + a*c)) < 1e-6);
  // Outward orientation ⇒ POSITIVE signed enclosed volume (sign is meaningful).
  CC_CHECK(enclosedVolume(mesh) > 0.0);
  CC_CHECK(std::fabs(enclosedVolume(mesh) - a*b*c) < 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════════
// Doubly-curved closed surface — a sphere: watertight after welding the seam +
// poles; vertices on the surface; area/volume converge to the closed form.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(sphere_watertight_and_converges) {
  const double R = 7.0;
  MeshParams p; p.deflection = 0.05;
  Mesh mesh = SolidMesher{p}.mesh(fullSphere(R));
  CC_CHECK(maxDistToSphere(mesh, R) < 1e-9);   // vertices ON the sphere (S(u,v))
  CC_CHECK(isWatertight(mesh));                // seam + poles welded ⇒ closed
  const double truthA = 4.0 * kPi * R * R;
  const double truthV = 4.0 / 3.0 * kPi * R * R * R;
  const double area = surfaceArea(mesh);
  const double vol = std::fabs(enclosedVolume(mesh));
  CC_CHECK(area <= truthA + 1e-6);             // inscribed ⇒ under
  CC_CHECK(area > truthA * 0.98);              // within 2% at defl 0.05
  CC_CHECK(vol > truthV * 0.97 && vol < truthV * 1.001);
}

CC_TEST(sphere_area_converges_with_deflection) {
  const double R = 7.0, truthA = 4.0 * kPi * R * R;
  auto areaAt = [&](double defl) {
    MeshParams p; p.deflection = defl;
    return surfaceArea(SolidMesher{p}.mesh(fullSphere(R)));
  };
  CC_CHECK(std::fabs(areaAt(0.05) - truthA) < std::fabs(areaAt(0.5) - truthA));
}

// ═════════════════════════════════════════════════════════════════════════════
// REGRESSION (curved shared-edge stitch, Phase 4 #3): a CLOSED cylinder solid —
// two planar cap disks + a cylindrical side sharing the two CIRCULAR edges — meshes
// WATERTIGHT. This is the case that previously left the mesh open along the curved
// cap↔side seam (each face sampled the circle independently). With the shared
// per-edge discretization both faces place identical boundary vertices on the
// circle, so the seam welds closed. Area/volume converge to the closed form.
// ═════════════════════════════════════════════════════════════════════════════
CC_TEST(cylinder_solid_watertight_curved_seam) {
  const double R = 5.0, h = 12.0;
  MeshParams p; p.deflection = 0.05;
  Mesh mesh = SolidMesher{p}.mesh(cylinderSolid(R, h));
  CC_CHECK(mesh.triangleCount() > 0);
  CC_CHECK(isTwoManifold(mesh));
  CC_CHECK(isWatertight(mesh));              // curved cap↔side seam welded ⇒ closed
  CC_CHECK(boundaryEdgeCount(mesh) == 0);    // no open boundary (was ~0.12 fraction)
  // Vertices on the true surface (cylinder side OR either cap plane), within fp.
  double worst = 0.0;
  for (const auto& v : mesh.vertices) {
    const double dCyl = std::fabs(std::sqrt(v.x * v.x + v.y * v.y) - R);
    const double dz0 = std::fabs(v.z), dzh = std::fabs(v.z - h);
    worst = std::max(worst, std::min({dCyl, dz0, dzh}));
  }
  CC_CHECK(worst < 1e-9);
  const double truthA = 2.0 * kPi * R * h + 2.0 * kPi * R * R;
  const double truthV = kPi * R * R * h;
  const double area = surfaceArea(mesh);
  const double vol = std::fabs(enclosedVolume(mesh));
  CC_CHECK(area <= truthA + 1e-6);           // inscribed ⇒ under the true area
  CC_CHECK(area > truthA * 0.98);
  CC_CHECK(vol > truthV * 0.97 && vol < truthV * 1.001);
}

// The curved seam also converges: a finer deflection stays watertight and tightens
// the area toward the closed form.
CC_TEST(cylinder_solid_watertight_converges) {
  const double R = 5.0, h = 12.0, truthA = 2.0 * kPi * R * h + 2.0 * kPi * R * R;
  auto areaAt = [&](double defl) {
    MeshParams p; p.deflection = defl;
    Mesh mesh = SolidMesher{p}.mesh(cylinderSolid(R, h));
    CC_CHECK(isWatertight(mesh));  // watertight at every deflection
    return surfaceArea(mesh);
  };
  const double coarse = std::fabs(areaAt(0.2) - truthA);
  const double fine = std::fabs(areaAt(0.01) - truthA);
  CC_CHECK(fine < coarse);
}

CC_RUN_ALL()
