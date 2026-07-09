// SPDX-License-Identifier: Apache-2.0
//
// native_heal_parity.mm — native shape-healing vs OCCT-oracle parity harness
//                         (iOS simulator).
//
// Phase 4 capability #4 (`native-healing`), simulator verification gate 2 (see
// openspec/NATIVE-REWRITE.md). The native OCCT-FREE healer
// (cybercad::native::heal::healShell, src/native/heal/*) is compared, on IDENTICAL
// deliberately-broken face soups, against the OCCT healing oracle
// (BRepBuilderAPI_Sewing → ShapeFix_Shell → ShapeFix_Solid, via
// cyber::occt::sewAndFix). Healing is INTERNAL — no cc_* entry point — so parity is
// asserted at the C++/heal boundary, exactly like native-topology / native-ssi.
//
// ── The OCCT-free boundary ───────────────────────────────────────────────────
// The native library never sees OCCT. Each fixture is built TWICE, independently:
// once as a native topology::Shape face soup (nativeSoup*), once as a TopoDS
// compound of the same faces (occtSoup*). The native side heals via healShell; the
// OCCT side via sewAndFix (which lives in src/engine/occt/). Nothing under
// src/native gains an OCCT dependency.
//
// ── Comparisons (per fixture) ────────────────────────────────────────────────
//   * IN-SCOPE (soup / degenerate-edge / sliver / flipped): native HEALS to a
//     watertight valid solid whose enclosed volume matches OCCT's fixed-solid
//     volume within a mesh-deflection tolerance (both should be the unit cube V=1).
//   * UN-HEALABLE (missing face / gap beyond tol): native reports UNHEALED, and
//     OCCT ALSO leaves the shell open (free edges present / no valid closed solid)
//     at the SAME tolerance — the native UNHEALED verdict matches OCCT "needs more".
//
// Output: [NHEAL] PASS/FAIL lines, then "== N passed, M failed ==". Flushes and
// std::_Exit (OCCT static teardown in the trimmed static build is not exit-clean —
// same rationale as the sibling sim harnesses; every handle here is RAII-scoped).
//
// Build (see scripts/run-sim-native-heal.sh):
//   -DCYBERCAD_HAS_OCCT -std=c++20 for arm64-apple-ios-simulator, linking the
//   ModelingData/ModelingAlgorithms + TKShHealing slice of OCCT.
//
#include "native/heal/native_heal.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "engine/occt/occt_engine.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>

namespace topo = cybercad::native::topology;
namespace heal = cybercad::native::heal;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

static int g_pass = 0, g_fail = 0;
static void check(const char* name, bool ok, const char* detail = "") {
  std::printf("[NHEAL] %-40s %s %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (ok) ++g_pass; else ++g_fail;
}

// ── Fixture corner data (unit cube) — shared by both builders ──────────────────
struct Quad { m::Point3 p[4]; m::Dir3 n; };

static std::vector<Quad> cubeQuads(double jitter) {
  const double s = 1.0;
  const m::Point3 c[8] = {{0,0,0},{s,0,0},{s,s,0},{0,s,0},{0,0,s},{s,0,s},{s,s,s},{0,s,s}};
  auto j = [&](const m::Point3& p, int face, int idx) -> m::Point3 {
    const double dx = jitter * (((face*7+idx*3)%5)-2)/2.0;
    const double dy = jitter * (((face*3+idx*5)%5)-2)/2.0;
    const double dz = jitter * (((face*5+idx*7)%5)-2)/2.0;
    return m::Point3{p.x+dx,p.y+dy,p.z+dz};
  };
  std::vector<Quad> q;
  q.push_back({{j(c[0],0,0),j(c[3],0,1),j(c[2],0,2),j(c[1],0,3)}, m::Dir3{0,0,-1}});
  q.push_back({{j(c[4],1,0),j(c[5],1,1),j(c[6],1,2),j(c[7],1,3)}, m::Dir3{0,0,1}});
  q.push_back({{j(c[0],2,0),j(c[1],2,1),j(c[5],2,2),j(c[4],2,3)}, m::Dir3{0,-1,0}});
  q.push_back({{j(c[3],3,0),j(c[7],3,1),j(c[6],3,2),j(c[2],3,3)}, m::Dir3{0,1,0}});
  q.push_back({{j(c[0],4,0),j(c[4],4,1),j(c[7],4,2),j(c[3],4,3)}, m::Dir3{-1,0,0}});
  q.push_back({{j(c[1],5,0),j(c[2],5,1),j(c[6],5,2),j(c[5],5,3)}, m::Dir3{1,0,0}});
  return q;
}

// A perfect unit cube whose +Z (top) face is lifted by `g`: every top-ring corner is
// a near-miss seam of gap `g` (the M5 bounded-bridging fixture — see host test
// cubeTopSeam / gap_bridge.h). q[1] is the +Z face in cubeQuads order.
static std::vector<Quad> cubeTopSeamQuads(double g) {
  std::vector<Quad> q = cubeQuads(0.0);  // exact cube (no jitter) so a bridged heal is exact
  for (auto& p : q[1].p) p.z += g;       // lift only the top face → a uniform seam gap g
  return q;
}

// A unit cube MISSING its +Z face (index 1 in cubeQuads order) → a single planar
// square hole at z=1 (the missing-face fixture the cap slice closes).
static std::vector<Quad> cubeMissingTop(double jitter) {
  std::vector<Quad> q = cubeQuads(jitter);
  q.erase(q.begin() + 1);  // drop +Z
  return q;
}

// A unit cube MISSING two OPPOSITE faces (−Z and +Z) → an open tube with TWO disjoint
// planar boundary loops. Native caps exactly ONE simple hole, so this declines
// Unhealed{OpenShell}; OCCT sewing/ShapeFix never invents a missing face, so it ALSO
// leaves the shell open — honest parity of decline (≥ 2 holes stay OCCT's moat).
static std::vector<Quad> cubeMissingTwoOpposite(double jitter) {
  std::vector<Quad> q = cubeQuads(jitter);
  q.erase(q.begin() + 1);  // drop +Z (index 1)
  q.erase(q.begin() + 0);  // then drop −Z (index 0)
  return q;
}

// A unit cube MISSING two ADJACENT faces (+Z and +X). The two exclusively-shared corners
// (c5, c6) are orphaned (each left on a single face), so the sew measures a residual ≫ tol
// and the wrap-around boundary is non-planar — out of scope for BOTH the single- and
// multi-hole cap passes (native declines; OCCT cannot invent the missing faces either).
static std::vector<Quad> cubeMissingTwoAdjacent(double jitter) {
  std::vector<Quad> q = cubeQuads(jitter);
  q.erase(q.begin() + 5);  // drop +X (index 5)
  q.erase(q.begin() + 1);  // then drop +Z (index 1)
  return q;
}

// A hexagonal +Z top face whose first side c4→c5 (the y=0,z=1 edge) is split by a
// COLLINEAR short edge of length `seg` centred on its midpoint; the other five faces are
// plain quads. The neighbour −Y face carries the plain straight c4-c5 span, so the two
// interior split verts block sharing until the collinear short edge is collapsed (the M5
// short-edge-merge fixture — see host cubeTopShortEdge / short_edge.h). Returned as the
// six top corners for the native builder + as explicit point lists for OCCT.
struct HexTopSoup {
  std::vector<Quad> quads;     // the five plain quad faces (−Z,−Y,+Y,−X,+X)
  m::Point3 topHex[6];         // the split +Z face as a 6-corner loop (c4,B,C,c5,c6,c7)
  m::Dir3 topN{0, 0, 1};
};
static HexTopSoup cubeTopShortEdgeSoup(double seg) {
  const double s = 1.0;
  const m::Point3 c[8] = {{0,0,0},{s,0,0},{s,s,0},{0,s,0},{0,0,s},{s,0,s},{s,s,s},{0,s,s}};
  const m::Vec3 e = c[5] - c[4];
  const m::Vec3 u = e / m::norm(e);
  const m::Point3 mid = c[4] + e * 0.5;
  const m::Point3 B = mid - u * (seg * 0.5), C = mid + u * (seg * 0.5);  // collinear on c4-c5
  HexTopSoup h;
  h.quads.push_back({{c[0],c[3],c[2],c[1]}, m::Dir3{0,0,-1}});  // −Z
  h.quads.push_back({{c[0],c[1],c[5],c[4]}, m::Dir3{0,-1,0}});  // −Y (plain c4-c5)
  h.quads.push_back({{c[3],c[7],c[6],c[2]}, m::Dir3{0,1,0}});   // +Y
  h.quads.push_back({{c[0],c[4],c[7],c[3]}, m::Dir3{-1,0,0}});  // −X
  h.quads.push_back({{c[1],c[2],c[6],c[5]}, m::Dir3{1,0,0}});   // +X
  h.topHex[0]=c[4]; h.topHex[1]=B; h.topHex[2]=C; h.topHex[3]=c[5]; h.topHex[4]=c[6]; h.topHex[5]=c[7];
  return h;
}

// A pentagonal +Z top face whose first side c4→c5 (the y=0,z=1 edge) carries ONE extra
// vertex B at parameter `t` along the side, exactly on the line (a redundant COLLINEAR
// T-vertex); the other five faces are plain quads. The neighbour −Y face carries the plain
// straight c4-c5 span, so the extra vertex blocks sharing until it is removed. Unlike the
// short-edge soup this inserts a SINGLE vertex between two FULL-LENGTH sub-edges (the M5
// collinear-vertex fixture — see host cubeTopCollinearVert / collinear_vert.h).
struct PentaTopSoup {
  std::vector<Quad> quads;     // the five plain quad faces (−Z,−Y,+Y,−X,+X)
  m::Point3 topPenta[5];       // the +Z face as a 5-corner loop (c4,B,c5,c6,c7)
  m::Dir3 topN{0, 0, 1};
};
static PentaTopSoup cubeTopCollinearVertSoup(double t) {
  const double s = 1.0;
  const m::Point3 c[8] = {{0,0,0},{s,0,0},{s,s,0},{0,s,0},{0,0,s},{s,0,s},{s,s,s},{0,s,s}};
  const m::Point3 B = c[4] + (c[5] - c[4]) * t;  // exactly on the c4-c5 line
  PentaTopSoup h;
  h.quads.push_back({{c[0],c[3],c[2],c[1]}, m::Dir3{0,0,-1}});  // −Z
  h.quads.push_back({{c[0],c[1],c[5],c[4]}, m::Dir3{0,-1,0}});  // −Y (plain c4-c5)
  h.quads.push_back({{c[3],c[7],c[6],c[2]}, m::Dir3{0,1,0}});   // +Y
  h.quads.push_back({{c[0],c[4],c[7],c[3]}, m::Dir3{-1,0,0}});  // −X
  h.quads.push_back({{c[1],c[2],c[6],c[5]}, m::Dir3{1,0,0}});   // +X
  h.topPenta[0]=c[4]; h.topPenta[1]=B; h.topPenta[2]=c[5]; h.topPenta[3]=c[6]; h.topPenta[4]=c[7];
  return h;
}

// A native face from an N-corner planar loop (Line edges + pcurves), like nativeQuadFace.
static topo::Shape nativePolyFace(const m::Point3* pts, int n, const m::Dir3& normal) {
  const m::Vec3 ref = std::fabs(normal.z()) < 0.9 ? m::Vec3{0,0,1} : m::Vec3{1,0,0};
  const m::Ax3 frame = m::Ax3::fromAxisAndRef(pts[0], normal, m::Dir3{ref});
  std::vector<topo::Shape> v(n);
  for (int i = 0; i < n; ++i) v[i] = topo::ShapeBuilder::makeVertex(pts[i]);
  auto toUV = [&](const m::Point3& p) { const m::Vec3 d = p - frame.origin;
    return m::Point3{m::dot(d, frame.x.vec()), m::dot(d, frame.y.vec()), 0.0}; };
  std::vector<topo::Shape> edges;
  for (int i = 0; i < n; ++i) {
    const m::Point3 a = pts[i], b = pts[(i+1)%n];
    const m::Vec3 d = b - a; const double len = std::max(m::norm(d), 1e-12);
    topo::EdgeCurve cc; cc.kind = topo::EdgeCurve::Kind::Line; cc.frame.origin = a;
    cc.frame.x = m::norm(d) > 1e-12 ? m::Dir3{d} : m::Dir3{1,0,0}; cc.frame.z = frame.z;
    topo::Shape ed = topo::ShapeBuilder::makeEdge(cc, 0.0, len, v[i], v[(i+1)%n]);
    topo::PCurve pc; pc.kind = topo::EdgeCurve::Kind::Line;
    const m::Point3 uv0 = toUV(a), uv1 = toUV(b); pc.origin2d = uv0; pc.dir2d = (uv1-uv0)/len;
    edges.push_back(topo::ShapeBuilder::addPCurve(ed, ed.tshape(), pc));
  }
  topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  topo::FaceSurface fs; fs.kind = topo::FaceSurface::Kind::Plane; fs.frame = frame;
  return topo::ShapeBuilder::makeFace(fs, wire, {}, topo::Orientation::Forward);
}

// The OCCT short-edge soup: the SAME five plain quad faces + the split hexagonal +Z face
// (a 6-vertex polygon). BRepBuilderAPI_Sewing at a tolerance ≥ seg merges the split verts
// (its own small-edge handling), so sewAndFix closes it — the OCCT analogue of the native
// short-edge collapse; at a tolerance < seg it leaves the interior verts and stays open.
static TopoDS_Shape occtShortEdgeSoup(const HexTopSoup& h) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : h.quads) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < 4; ++i) poly.Add(gp_Pnt(q.p[i].x, q.p[i].y, q.p[i].z));
    poly.Close();
    if (!poly.IsDone()) continue;
    const gp_Pln plane(gp_Pnt(q.p[0].x, q.p[0].y, q.p[0].z), gp_Dir(q.n.x(), q.n.y(), q.n.z()));
    BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  BRepBuilderAPI_MakePolygon top;
  for (int i = 0; i < 6; ++i) top.Add(gp_Pnt(h.topHex[i].x, h.topHex[i].y, h.topHex[i].z));
  top.Close();
  if (top.IsDone()) {
    const gp_Pln plane(gp_Pnt(h.topHex[0].x, h.topHex[0].y, h.topHex[0].z),
                       gp_Dir(h.topN.x(), h.topN.y(), h.topN.z()));
    BRepBuilderAPI_MakeFace mf(plane, top.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  return comp;
}

// ── Native soup builder ────────────────────────────────────────────────────────
static topo::Shape nativeQuadFace(const Quad& q, bool reversed) {
  const m::Vec3 ref = std::fabs(q.n.z()) < 0.9 ? m::Vec3{0,0,1} : m::Vec3{1,0,0};
  const m::Ax3 frame = m::Ax3::fromAxisAndRef(q.p[0], q.n, m::Dir3{ref});
  topo::Shape v[4];
  for (int i = 0; i < 4; ++i) v[i] = topo::ShapeBuilder::makeVertex(q.p[i]);
  auto toUV = [&](const m::Point3& p) { const m::Vec3 d = p - frame.origin;
    return m::Point3{m::dot(d, frame.x.vec()), m::dot(d, frame.y.vec()), 0.0}; };
  std::vector<topo::Shape> edges;
  for (int i = 0; i < 4; ++i) {
    const m::Point3 a = q.p[i], b = q.p[(i+1)%4];
    const m::Vec3 d = b - a; const double len = std::max(m::norm(d), 1e-12);
    topo::EdgeCurve c; c.kind = topo::EdgeCurve::Kind::Line; c.frame.origin = a;
    c.frame.x = m::norm(d) > 1e-12 ? m::Dir3{d} : m::Dir3{1,0,0}; c.frame.z = frame.z;
    topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, len, v[i], v[(i+1)%4]);
    topo::PCurve pc; pc.kind = topo::EdgeCurve::Kind::Line;
    const m::Point3 uv0 = toUV(a), uv1 = toUV(b); pc.origin2d = uv0; pc.dir2d = (uv1-uv0)/len;
    edges.push_back(topo::ShapeBuilder::addPCurve(e, e.tshape(), pc));
  }
  topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  topo::FaceSurface s; s.kind = topo::FaceSurface::Kind::Plane; s.frame = frame;
  topo::Shape f = topo::ShapeBuilder::makeFace(s, wire, {}, topo::Orientation::Forward);
  return reversed ? f.reversedShape() : f;
}

static topo::Shape nativeSoup(const std::vector<Quad>& quads, int flipIndex = -1) {
  std::vector<topo::Shape> faces;
  for (int i = 0; i < (int)quads.size(); ++i)
    faces.push_back(nativeQuadFace(quads[i], i == flipIndex));
  return topo::ShapeBuilder::makeShell(faces);
}

// The native short-edge soup: five plain quad faces + the split hexagonal +Z face.
static topo::Shape nativeShortEdgeSoup(const HexTopSoup& h) {
  std::vector<topo::Shape> faces;
  for (const Quad& q : h.quads) faces.push_back(nativeQuadFace(q, false));
  faces.push_back(nativePolyFace(h.topHex, 6, h.topN));
  return topo::ShapeBuilder::makeShell(faces);
}

// The native collinear-vertex soup: five plain quad faces + the pentagonal +Z face.
static topo::Shape nativeCollinearVertSoup(const PentaTopSoup& h) {
  std::vector<topo::Shape> faces;
  for (const Quad& q : h.quads) faces.push_back(nativeQuadFace(q, false));
  faces.push_back(nativePolyFace(h.topPenta, 5, h.topN));
  return topo::ShapeBuilder::makeShell(faces);
}

// The OCCT collinear-vertex soup: the SAME five plain quad faces + the pentagonal +Z face
// (a 5-vertex polygon). BRepBuilderAPI_Sewing + ShapeFix (ShapeFix_Wire drops the collinear
// vertex / merges it) closes it — the OCCT analogue of the native collinear-vertex removal.
static TopoDS_Shape occtCollinearVertSoup(const PentaTopSoup& h) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : h.quads) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < 4; ++i) poly.Add(gp_Pnt(q.p[i].x, q.p[i].y, q.p[i].z));
    poly.Close();
    if (!poly.IsDone()) continue;
    const gp_Pln plane(gp_Pnt(q.p[0].x, q.p[0].y, q.p[0].z), gp_Dir(q.n.x(), q.n.y(), q.n.z()));
    BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  BRepBuilderAPI_MakePolygon top;
  for (int i = 0; i < 5; ++i) top.Add(gp_Pnt(h.topPenta[i].x, h.topPenta[i].y, h.topPenta[i].z));
  top.Close();
  if (top.IsDone()) {
    const gp_Pln plane(gp_Pnt(h.topPenta[0].x, h.topPenta[0].y, h.topPenta[0].z),
                       gp_Dir(h.topN.x(), h.topN.y(), h.topN.z()));
    BRepBuilderAPI_MakeFace mf(plane, top.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  return comp;
}

// ── OCCT soup builder (a compound of independent planar faces) ─────────────────
static TopoDS_Shape occtSoup(const std::vector<Quad>& quads) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : quads) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < 4; ++i) poly.Add(gp_Pnt(q.p[i].x, q.p[i].y, q.p[i].z));
    poly.Close();
    if (!poly.IsDone()) continue;
    // Build the face on a plane whose normal is the quad's OUTWARD normal, so the
    // OCCT soup carries the SAME coherent outward orientation as the native soup
    // (an apples-to-apples heal comparison — both get a well-formed unsewn soup).
    const gp_Pln plane(gp_Pnt(q.p[0].x, q.p[0].y, q.p[0].z),
                       gp_Dir(q.n.x(), q.n.y(), q.n.z()));
    BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  return comp;
}

// The OCCT REFERENCE CAP: the open-shell faces PLUS one cap face synthesized by
// BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire) over the four given boundary
// corners, on the plane through cap[0] with normal capN. sewAndFix then welds the
// completed soup (BRepBuilderAPI_Sewing + ShapeFix_Shell/Solid) — the OCCT analogue of
// the native cap. For a NON-PLANAR boundary the MakeFace on gp_Pln fails / yields a bad
// face and the sew leaves the shell open (the decline oracle).
static TopoDS_Shape occtSoupWithPlanarCap(const std::vector<Quad>& openQuads,
                                          const m::Point3 cap[4], const m::Dir3& capN) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : openQuads) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < 4; ++i) poly.Add(gp_Pnt(q.p[i].x, q.p[i].y, q.p[i].z));
    poly.Close();
    if (!poly.IsDone()) continue;
    const gp_Pln plane(gp_Pnt(q.p[0].x, q.p[0].y, q.p[0].z), gp_Dir(q.n.x(), q.n.y(), q.n.z()));
    BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  BRepBuilderAPI_MakePolygon capPoly;
  for (int i = 0; i < 4; ++i) capPoly.Add(gp_Pnt(cap[i].x, cap[i].y, cap[i].z));
  capPoly.Close();
  if (capPoly.IsDone()) {
    const gp_Pln capPlane(gp_Pnt(cap[0].x, cap[0].y, cap[0].z),
                          gp_Dir(capN.x(), capN.y(), capN.z()));
    BRepBuilderAPI_MakeFace capMf(capPlane, capPoly.Wire());
    if (capMf.IsDone()) bb.Add(comp, capMf.Face());
  }
  return comp;
}

// The OCCT MULTI reference: the open-shell faces PLUS one planar cap PER hole, each a
// BRepBuilderAPI_MakeFace(gp_Pln, boundaryWire) over its four boundary corners. sewAndFix
// then welds the completed soup — the OCCT 1:1 analogue of the native multi-hole cap.
static TopoDS_Shape occtSoupWithTwoPlanarCaps(const std::vector<Quad>& openQuads,
                                              const m::Point3 capA[4], const m::Dir3& nA,
                                              const m::Point3 capB[4], const m::Dir3& nB) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : openQuads) {
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < 4; ++i) poly.Add(gp_Pnt(q.p[i].x, q.p[i].y, q.p[i].z));
    poly.Close();
    if (!poly.IsDone()) continue;
    const gp_Pln plane(gp_Pnt(q.p[0].x, q.p[0].y, q.p[0].z), gp_Dir(q.n.x(), q.n.y(), q.n.z()));
    BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
    if (mf.IsDone()) bb.Add(comp, mf.Face());
  }
  auto addCap = [&](const m::Point3 cap[4], const m::Dir3& n) {
    BRepBuilderAPI_MakePolygon capPoly;
    for (int i = 0; i < 4; ++i) capPoly.Add(gp_Pnt(cap[i].x, cap[i].y, cap[i].z));
    capPoly.Close();
    if (!capPoly.IsDone()) return;
    const gp_Pln capPlane(gp_Pnt(cap[0].x, cap[0].y, cap[0].z), gp_Dir(n.x(), n.y(), n.z()));
    BRepBuilderAPI_MakeFace capMf(capPlane, capPoly.Wire());
    if (capMf.IsDone()) bb.Add(comp, capMf.Face());
  };
  addCap(capA, nA);
  addCap(capB, nB);
  return comp;
}

static double nativeVolume(const topo::Shape& s) {
  tess::MeshParams p; p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(s);
  return tess::isWatertight(mesh) ? tess::enclosedVolume(mesh) : 0.0;
}

int main() {
  const double tol = 1e-4;
  const heal::HealOptions opts{tol};

  // IN-SCOPE fixtures: native HEALS, OCCT sews to a valid solid, volumes agree ~1.
  struct InScope { const char* name; std::vector<Quad> quads; int flip; };
  std::vector<InScope> inScope = {
      {"soup-cube", cubeQuads(1e-6), -1},
      {"flipped-face", cubeQuads(1e-6), 1},
  };
  for (auto& fx : inScope) {
    const heal::HealResult nr = heal::healShell(nativeSoup(fx.quads, fx.flip), opts);
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtSoup(fx.quads), tol);
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[160];
    std::snprintf(buf, sizeof buf, "(native V=%.5f watertight=%d | OCCT V=%.5f valid=%d)",
                  nv, (int)nr.metrics.watertight, orr.volume, (int)orr.validSolid);
    const bool ok = nr.healed() && nr.metrics.watertight && orr.validSolid &&
                    std::fabs(nv - 1.0) < 1e-3 && std::fabs(std::fabs(orr.volume) - 1.0) < 1e-3 &&
                    std::fabs(nv - std::fabs(orr.volume)) < 1e-3;
    check(fx.name, ok, buf);
  }

  // UN-HEALABLE: gap beyond tol — native UNHEALED, OCCT also fails to close it.
  {
    const std::vector<Quad> quads = cubeQuads(1e-2);  // gaps ~1e-2 >> tol 1e-4
    const heal::HealResult nr = heal::healShell(nativeSoup(quads), opts);
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtSoup(quads), tol);
    char buf[160];
    std::snprintf(buf, sizeof buf, "(native UNHEALED reason=%d residual=%.4g | OCCT valid=%d watertight=%d)",
                  (int)nr.reason, nr.metrics.maxResidualGap, (int)orr.validSolid, (int)orr.watertight);
    // Native honestly UNHEALED with residual > tol; OCCT ALSO does not form a valid
    // closed solid at the same tolerance (needs more) — the verdicts agree.
    const bool ok = !nr.healed() && nr.metrics.maxResidualGap > tol && !orr.validSolid;
    check("beyond-tol-gap UNHEALED matches OCCT", ok, buf);
  }

  // UN-HEALABLE: missing face (open cube) — native UNHEALED(OpenShell), OCCT open.
  {
    std::vector<Quad> quads = cubeQuads(1e-6);
    quads.erase(quads.begin() + 1);  // drop the +Z face → a genuine hole
    const heal::HealResult nr = heal::healShell(nativeSoup(quads), opts);
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtSoup(quads), tol);
    char buf[160];
    std::snprintf(buf, sizeof buf, "(native UNHEALED reason=%d | OCCT valid=%d watertight=%d)",
                  (int)nr.reason, (int)orr.validSolid, (int)orr.watertight);
    const bool ok = !nr.healed() && !orr.watertight;  // both leave it open
    check("open-shell UNHEALED matches OCCT", ok, buf);
  }

  // ── M5 BOUNDED GAP BRIDGING (opt-in) ─────────────────────────────────────────
  // In-band: a near-miss seam gap g in (tol, budget] closes natively under the
  // bounded bridging pass and OCCT closes it by sewing at tolerance ≈ budget. Both
  // land a watertight/valid unit cube; the sewn-vertex position is implementation-
  // defined WITHIN the gap, so volumes agree only within the sewing tolerance (~g).
  {
    const double g = 5e-3, budget = 1e-2;
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeTopSeamQuads(g)), heal::HealOptions{tol, budget});
    const cyber::occt::SewFixResult orr =
        cyber::occt::sewAndFix(occtSoup(cubeTopSeamQuads(g)), budget);  // OCCT sews at ≈ budget
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native V=%.5f watertight=%d bridged=%d | OCCT V=%.5f valid=%d)", nv,
                  (int)nr.metrics.watertight, nr.metrics.nBridgedGaps, orr.volume,
                  (int)orr.validSolid);
    const bool ok = nr.healed() && nr.metrics.watertight && nr.metrics.nBridgedGaps > 0 &&
                    orr.validSolid && std::fabs(nv - 1.0) < budget &&
                    std::fabs(std::fabs(orr.volume) - 1.0) < budget &&
                    std::fabs(nv - std::fabs(orr.volume)) < budget;
    check("bridge-in-band matches OCCT sew@budget", ok, buf);
  }

  // Out-of-budget: a gap g > budget declines natively (GapBeyondBudget) and OCCT,
  // sewing at tolerance ≈ budget, also cannot close it — the verdicts agree.
  {
    const double g = 5e-2, budget = 1e-2;
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeTopSeamQuads(g)), heal::HealOptions{tol, budget});
    const cyber::occt::SewFixResult orr =
        cyber::occt::sewAndFix(occtSoup(cubeTopSeamQuads(g)), budget);
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native UNHEALED reason=%d residual=%.4g | OCCT valid=%d)", (int)nr.reason,
                  nr.metrics.maxResidualGap, (int)orr.validSolid);
    const bool ok = !nr.healed() && nr.reason == heal::UnhealedReason::GapBeyondBudget &&
                    nr.metrics.maxResidualGap > budget && !orr.validSolid;
    check("bridge-out-of-budget decline matches OCCT", ok, buf);
  }

  // ── M5 TAIL: SINGLE PLANAR-HOLE CAP (opt-in) ─────────────────────────────────
  // In-scope: a unit cube missing its +Z face. Native (capPlanarHoles = true)
  // synthesizes ONE cap on the free boundary; the OCCT reference cap is
  // BRepBuilderAPI_MakeFace(gp_Pln, topWire) added to the 5 open faces + ShapeFix.
  // Both land a watertight closed 6-face solid with volume ≈ 1.
  {
    const m::Point3 topCap[4] = {{0,0,1},{1,0,1},{1,1,1},{0,1,1}};  // free boundary at z=1
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeMissingTop(1e-6)), heal::HealOptions{tol, 0.0, true});
    const cyber::occt::SewFixResult orr =
        cyber::occt::sewAndFix(occtSoupWithPlanarCap(cubeMissingTop(1e-6), topCap, m::Dir3{0,0,1}), tol);
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native V=%.5f watertight=%d capped=%d | OCCT V=%.5f valid=%d)", nv,
                  (int)nr.metrics.watertight, nr.metrics.nCappedFaces, orr.volume,
                  (int)orr.validSolid);
    const bool ok = nr.healed() && nr.metrics.watertight && nr.metrics.nCappedFaces == 1 &&
                    orr.validSolid && std::fabs(nv - 1.0) < 1e-3 &&
                    std::fabs(std::fabs(orr.volume) - 1.0) < 1e-3 &&
                    std::fabs(nv - std::fabs(orr.volume)) < 1e-3;
    check("cap-single-planar-hole matches OCCT cap", ok, buf);
  }

  // Out-of-scope: TWO opposite missing faces (two disjoint boundary loops). Native
  // caps exactly one simple hole → Unhealed{OpenShell}, capped=0; OCCT sewing/ShapeFix
  // cannot invent the missing faces and leaves the shell open → parity of decline. (A
  // single mildly-non-planar hole is instead covered by the host planarity gate: OCCT's
  // MakeFace tolerates a near-planar wire and caps it, so native declining there is
  // native being MORE conservative and deferring — not a shared decline.)
  {
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeMissingTwoOpposite(1e-6)), heal::HealOptions{tol, 0.0, true});
    const cyber::occt::SewFixResult orr =
        cyber::occt::sewAndFix(occtSoup(cubeMissingTwoOpposite(1e-6)), tol);
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native UNHEALED reason=%d capped=%d residual=%.4g | OCCT valid=%d watertight=%d)",
                  (int)nr.reason, nr.metrics.nCappedFaces, nr.metrics.maxResidualGap,
                  (int)orr.validSolid, (int)orr.watertight);
    const bool ok = !nr.healed() && nr.reason == heal::UnhealedReason::OpenShell &&
                    nr.metrics.nCappedFaces == 0 && !orr.watertight;
    check("cap-two-hole decline matches OCCT", ok, buf);
  }

  // ── M5 TAIL: MULTI PLANAR-HOLE CAP (opt-in capMultiplePlanarHoles) ────────────
  // In-scope: a unit cube missing BOTH −Z and +Z faces → two disjoint planar square
  // holes. Native (capMultiplePlanarHoles = true) synthesizes ONE cap per hole; the OCCT
  // reference completes the SAME soup with the SAME two caps
  // (BRepBuilderAPI_MakeFace(gp_Pln, wire) ×2) + ShapeFix. Both land a watertight closed
  // 6-face solid with volume ≈ 1 — the multi-hole native win matches OCCT 1:1.
  {
    const m::Point3 topCap[4]    = {{0,0,1},{1,0,1},{1,1,1},{0,1,1}};  // z=1 hole (+Z outward)
    const m::Point3 bottomCap[4] = {{0,0,0},{0,1,0},{1,1,0},{1,0,0}};  // z=0 hole (−Z outward)
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeMissingTwoOpposite(1e-6)),
                        heal::HealOptions{tol, 0.0, false, true});
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(
        occtSoupWithTwoPlanarCaps(cubeMissingTwoOpposite(1e-6), topCap, m::Dir3{0,0,1},
                                  bottomCap, m::Dir3{0,0,-1}), tol);
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native V=%.5f watertight=%d capped=%d | OCCT V=%.5f valid=%d)", nv,
                  (int)nr.metrics.watertight, nr.metrics.nCappedFaces, orr.volume,
                  (int)orr.validSolid);
    const bool ok = nr.healed() && nr.metrics.watertight && nr.metrics.nCappedFaces == 2 &&
                    orr.validSolid && std::fabs(nv - 1.0) < 1e-3 &&
                    std::fabs(std::fabs(orr.volume) - 1.0) < 1e-3 &&
                    std::fabs(nv - std::fabs(orr.volume)) < 1e-3;
    check("cap-two-opposite-holes matches OCCT caps", ok, buf);
  }

  // Out-of-scope for the multi pass too: two ADJACENT missing faces orphan the two
  // exclusively-shared corners, so the boundary is a non-planar wrap (native declines,
  // capped=0). OCCT sewing the same open soup (no reference cap) cannot invent the
  // missing faces and leaves the shell open — parity of decline (the asymptotic tail).
  {
    const heal::HealResult nr =
        heal::healShell(nativeSoup(cubeMissingTwoAdjacent(1e-6)),
                        heal::HealOptions{tol, 0.0, false, true});
    const cyber::occt::SewFixResult orr =
        cyber::occt::sewAndFix(occtSoup(cubeMissingTwoAdjacent(1e-6)), tol);
    char buf[200];
    std::snprintf(buf, sizeof buf,
                  "(native UNHEALED reason=%d capped=%d residual=%.4g | OCCT valid=%d watertight=%d)",
                  (int)nr.reason, nr.metrics.nCappedFaces, nr.metrics.maxResidualGap,
                  (int)orr.validSolid, (int)orr.watertight);
    const bool ok = !nr.healed() && nr.metrics.nCappedFaces == 0 && !orr.watertight;
    check("cap-two-adjacent decline matches OCCT", ok, buf);
  }

  // ── M5 TAIL: SHORT-EDGE COLLAPSE (opt-in shortEdgeMergeLen) ───────────────────
  // In-scope: a unit cube whose +Z top face carries a COLLINEAR short edge (seg) an
  // exporter split into the y=0,z=1 boundary run. Native (shortEdgeMergeLen ≥ seg)
  // removes the redundant micro-edge and re-sews to a watertight unit cube. The OCCT
  // reference sews the SAME six-face soup (five quads + the split hexagon) at a tolerance
  // ≥ seg, so BRepBuilderAPI_Sewing's own small-edge handling merges the split verts and
  // ShapeFix closes it. Both land a watertight closed solid with volume ≈ 1.
  {
    const double seg = 5e-3, mergeLen = 1e-2, occtTol = 1e-2;  // occtTol ≥ seg swallows it
    const HexTopSoup h = cubeTopShortEdgeSoup(seg);
    const heal::HealResult nr =
        heal::healShell(nativeShortEdgeSoup(h), heal::HealOptions{tol, 0.0, false, false, mergeLen});
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtShortEdgeSoup(h), occtTol);
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[220];
    std::snprintf(buf, sizeof buf,
                  "(native V=%.5f watertight=%d collapsed=%d | OCCT V=%.5f valid=%d)", nv,
                  (int)nr.metrics.watertight, nr.metrics.nCollapsedShortEdges, orr.volume,
                  (int)orr.validSolid);
    // OCCT sewing at occtTol may reposition the merged vertex within seg, so volumes
    // agree only within the sew tolerance; native lands exactly 1 (collinear collapse).
    const bool ok = nr.healed() && nr.metrics.watertight && nr.metrics.nCollapsedShortEdges > 0 &&
                    orr.validSolid && std::fabs(nv - 1.0) < 1e-3 &&
                    std::fabs(std::fabs(orr.volume) - 1.0) < occtTol &&
                    std::fabs(nv - std::fabs(orr.volume)) < occtTol;
    check("short-edge-collapse matches OCCT sew@mergeLen", ok, buf);
  }

  // EQUAL-OR-MORE-CONSERVATIVE: with the flag OFF (shortEdgeMergeLen = 0) native declines
  // the SAME split soup honestly (GapBeyondTolerance, input unchanged, nothing collapsed).
  // OCCT sewing the same soup at a nominal tolerance BELOW seg is AGGRESSIVE — its sewer /
  // ShapeFix computes its own effective tolerance and closes the collinear split anyway
  // (observed here: valid=watertight=1 at V≈1). Native DEFERRING where OCCT aggressively
  // repairs is native being MORE conservative — the intended contract — NOT a wrong repair.
  // The bar: native must never emit a DIFFERENT watertight solid than the truth; a decline
  // is always safe. So we assert native declined + collapsed nothing, and — when OCCT DID
  // close — that OCCT's closure is the same honest unit cube (native lost no correctness by
  // deferring; the opt-in flag recovers exactly that win, proven by the check above).
  {
    const double seg = 5e-3, occtTol = 1e-4;  // occtTol < seg (OCCT still aggressively closes)
    const HexTopSoup h = cubeTopShortEdgeSoup(seg);
    const heal::HealResult nr = heal::healShell(nativeShortEdgeSoup(h), opts);  // flag OFF
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtShortEdgeSoup(h), occtTol);
    char buf[240];
    std::snprintf(buf, sizeof buf,
                  "(native UNHEALED reason=%d collapsed=%d residual=%.4g | OCCT valid=%d watertight=%d V=%.5f — native MORE conservative)",
                  (int)nr.reason, nr.metrics.nCollapsedShortEdges, nr.metrics.maxResidualGap,
                  (int)orr.validSolid, (int)orr.watertight, orr.volume);
    // Native declined honestly (never a wrong repair). If OCCT closed, its solid is the
    // honest unit cube (V≈1) — so native deferring cost no correctness, only recovered by
    // the opt-in flag. If OCCT also declined, that is straight parity of decline.
    const bool nativeConservative = !nr.healed() && nr.metrics.nCollapsedShortEdges == 0 &&
                                    nr.metrics.maxResidualGap > tol;
    const bool occtEitherDeclinesOrHonest =
        !orr.watertight || std::fabs(std::fabs(orr.volume) - 1.0) < occtTol;
    check("short-edge default-off: native more-conservative than OCCT",
          nativeConservative && occtEitherDeclinesOrHonest, buf);
  }

  // ── M5 TAIL: COLLINEAR-VERTEX REMOVAL (opt-in removeCollinearVerts) ────────────
  // In-scope: a unit cube whose +Z top face carries ONE redundant COLLINEAR vertex an
  // exporter dropped onto the y=0,z=1 boundary run at t=0.3 (both sub-edges full-length:
  // 0.3 and 0.7 — far above short_edge's ¼·neighbour bound, so pass 8 cannot reach it).
  // Native (removeCollinearVerts=true) drops the redundant vertex and re-sews to a
  // watertight unit cube EXACTLY (the span becomes the straight edge — no repositioning).
  // The OCCT reference sews the SAME six-face soup (five quads + the pentagon) at tol; its
  // sewer / ShapeFix_Wire drops the collinear vertex and closes it. Both land V ≈ 1.
  {
    const PentaTopSoup h = cubeTopCollinearVertSoup(0.3);
    const heal::HealResult nr = heal::healShell(
        nativeCollinearVertSoup(h), heal::HealOptions{tol, 0.0, false, false, 0.0, true});
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtCollinearVertSoup(h), tol);
    const double nv = nr.healed() ? nativeVolume(nr.shape) : 0.0;
    char buf[220];
    std::snprintf(buf, sizeof buf,
                  "(native V=%.5f watertight=%d removed=%d | OCCT V=%.5f valid=%d)", nv,
                  (int)nr.metrics.watertight, nr.metrics.nRemovedCollinearVerts, orr.volume,
                  (int)orr.validSolid);
    // Native lands EXACTLY 1 (collinear removal); OCCT closes the same honest unit cube.
    const bool ok = nr.healed() && nr.metrics.watertight && nr.metrics.nRemovedCollinearVerts > 0 &&
                    orr.validSolid && std::fabs(nv - 1.0) < 1e-3 &&
                    std::fabs(std::fabs(orr.volume) - 1.0) < 1e-3 &&
                    std::fabs(nv - std::fabs(orr.volume)) < 1e-3;
    check("collinear-vert-removal matches OCCT sew+fix", ok, buf);
  }

  // EQUAL-OR-MORE-CONSERVATIVE: with the flag OFF (removeCollinearVerts = false) native
  // declines the SAME soup honestly (GapBeyondTolerance, input unchanged, nothing removed)
  // — byte-identical to the landed slices. OCCT sewing the same soup still drops the
  // collinear vertex and closes it (aggressive). Native DEFERRING where OCCT repairs is
  // native being MORE conservative — the intended contract — and the opt-in flag recovers
  // exactly that win (proven by the check above). If OCCT closed, its solid is the honest
  // unit cube; if OCCT also declined, that is straight parity of decline.
  {
    const PentaTopSoup h = cubeTopCollinearVertSoup(0.3);
    const heal::HealResult nr = heal::healShell(nativeCollinearVertSoup(h), opts);  // flag OFF
    const cyber::occt::SewFixResult orr = cyber::occt::sewAndFix(occtCollinearVertSoup(h), tol);
    char buf[240];
    std::snprintf(buf, sizeof buf,
                  "(native UNHEALED reason=%d removed=%d residual=%.4g | OCCT valid=%d watertight=%d V=%.5f — native MORE conservative)",
                  (int)nr.reason, nr.metrics.nRemovedCollinearVerts, nr.metrics.maxResidualGap,
                  (int)orr.validSolid, (int)orr.watertight, orr.volume);
    const bool nativeConservative = !nr.healed() && nr.metrics.nRemovedCollinearVerts == 0 &&
                                    nr.metrics.maxResidualGap > tol;
    const bool occtEitherDeclinesOrHonest =
        !orr.watertight || std::fabs(std::fabs(orr.volume) - 1.0) < 1e-3;
    check("collinear-vert default-off: native more-conservative than OCCT",
          nativeConservative && occtEitherDeclinesOrHonest, buf);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
