// SPDX-License-Identifier: Apache-2.0
//
// native_healing_fuzz.mm — MOAT M6-breadth-14 (the COMPLETENESS BAR, FOURTEENTH domain):
// a SHAPE-HEALING differential-fuzzing harness (iOS simulator).
//
// This extends the landed M6 differential fuzzers (curved-boolean, STEP round-trip,
// construction, blend, wrap-emboss, mass-properties, geometry-services, transform,
// reference-geometry, direct-modeling, transformed-boolean, section, HLR) to a
// FOURTEENTH, independent native domain — the OCCT-FREE shape HEALER
// (cybercad::native::heal::healShell, src/native/heal/*). It is INFRASTRUCTURE (a test
// harness, not a geometry capability): OCCT (BRepBuilderAPI_Sewing → ShapeFix, via
// cyber::occt::sewAndFix) is the ORACLE, the bar is ZERO SILENT WRONG REPAIRS over a
// seeded batch, and an HONEST native DECLINE is a first-class outcome.
//
// ── THE THIRD ORACLE (why healing was previously deferred, and how this solves it) ──
// moat-m6h deferred a healing fuzzer because a heal's "correct" output is a heuristic
// agreement with no closed-form ground truth. This harness removes that objection by a
// single principle: INJECT DEFECTS INTO A SOLID WHOSE EXACT GEOMETRY IS KNOWN. Every
// base solid is built here (unit cube V=1, a random axis-aligned box, a random convex
// N-gon prism) so its enclosed volume + surface area are exact closed forms; every
// defect family is SHAPE-PRESERVING (it perturbs only the B-rep representation, never
// the intended solid). So a CORRECT heal reconstructs the ORIGINAL solid, and the
// closed-form volume+area is a THIRD, engine-independent arbiter. A native `Healed`
// solid is AGREED only when it matches that closed form (and OCCT concurs); a watertight
// native solid that misses the closed form is a genuine DISAGREE regardless of OCCT.
//
// ── THE DEFECT FAMILIES (each shape-preserving, reused from native_heal_parity.mm) ──
//   SEW-JITTER      coincident-within-tolerance soup (jitter ≤ tol)           → HEALS
//   FLIPPED-FACE    one face wound inward                                     → HEALS
//   SEAM-GAP-IN     a near-miss top-ring seam gap in (tol, budget]            → BRIDGES
//   SEAM-GAP-OUT    a seam gap > budget                                       → DECLINE (both)
//   SHORT-EDGE      a redundant collinear micro-edge (flag ON)                → COLLAPSES
//   SHORT-EDGE-OFF  the SAME split, flag OFF (native more-conservative)       → OCCT repairs
//   COLLINEAR-VERT  a redundant collinear T-vertex (flag ON)                  → REMOVES
//   COLL-VERT-OFF   the SAME vertex, flag OFF (native more-conservative)      → OCCT repairs
//   MISSING-1-HOLE  one missing planar face (capPlanarHoles)                  → CAPS
//   MISSING-2-OPP   two opposite missing faces (capMultiplePlanarHoles)       → CAPS×2
//   MISSING-2-ADJ   two adjacent missing faces (non-planar wrap)              → DECLINE (both)
//   BEYOND-TOL-GAP  jitter ≫ tol (a real gap)                                 → DECLINE (both)
//
// ── CLASSIFIER (exactly one bucket per trial; the equal-or-more-conservative contract) ──
//   AGREED            native Healed matches the closed-form truth AND OCCT concurs; OR
//                     native Unhealed while OCCT ALSO fails to close (parity of decline);
//                     OR native honestly declines a defect OCCT aggressively repairs to
//                     the SAME honest truth (native MORE conservative — a safe deferral).
//   HONESTLY-DECLINED native Unhealed (input unchanged, measured residual) → defer to OCCT.
//   DISAGREED         native Healed watertight but volume/area != the closed-form truth.
//                     A SILENT WRONG REPAIR — the failure this harness exists to catch.
//   ORACLE-INACCURATE native Healed matches exact math while OCCT does NOT (native
//                     vindicated). Logged, NOT a native fault, NOT a bar failure.
//   BOTH-DECLINED     a defect both engines refuse to close.
// The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, each base + defect family with ≥1
// non-error trial, across ≥2 seeds, N≥60/seed. The FIXED tolerances are NEVER widened.
//
// The RNG is a splitmix64-seeded xoshiro256** stream keyed ONLY by an explicit FUZZ_SEED
// (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//
// Build (see scripts/run-sim-native-healing-fuzz.sh): -DCYBERCAD_HAS_OCCT -std=c++20 for
// arm64-apple-ios-simulator, linking the native heal+math TUs + occt_shapefix.cpp oracle
// TU + TKShHealing/TKTopAlgo…TKernel. Own main(); std::_Exit (OCCT teardown not exit-clean).
//
#include "native/heal/native_heal.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "engine/occt/occt_engine.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream ───────────────────────
// Keyed ONLY by an explicit uint64 seed (argv/env). No clock, no rand(). Same seed →
// byte-identical batch.
struct Rng {
  uint64_t s[4];
  static uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  uint64_t next() {
    const uint64_t r = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

// ── base solid: an axis-aligned box (sx,sy,sz) at origin o. A UNIT cube = 1,1,1 @ 0. ──
struct Quad { m::Point3 p[4]; m::Dir3 n; };
struct Box { m::Point3 o; double sx, sy, sz; };

// Closed-form ground truth of the undamaged box.
static double boxVolume(const Box& b) { return b.sx * b.sy * b.sz; }
static double boxArea(const Box& b) {
  return 2.0 * (b.sx * b.sy + b.sy * b.sz + b.sz * b.sx);
}

// The eight corners of the box, in the SAME order the unit-cube fixtures use
// (c0..c7: the -Z ring then the +Z ring).
static void boxCorners(const Box& b, m::Point3 c[8]) {
  const double x0 = b.o.x, x1 = b.o.x + b.sx;
  const double y0 = b.o.y, y1 = b.o.y + b.sy;
  const double z0 = b.o.z, z1 = b.o.z + b.sz;
  c[0] = {x0, y0, z0}; c[1] = {x1, y0, z0}; c[2] = {x1, y1, z0}; c[3] = {x0, y1, z0};
  c[4] = {x0, y0, z1}; c[5] = {x1, y0, z1}; c[6] = {x1, y1, z1}; c[7] = {x0, y1, z1};
}

// The six box faces as outward-wound quads (identical winding to cubeQuads).
static std::vector<Quad> boxQuads(const Box& b, double jitter) {
  m::Point3 c[8]; boxCorners(b, c);
  auto j = [&](const m::Point3& p, int face, int idx) -> m::Point3 {
    const double dx = jitter * (((face * 7 + idx * 3) % 5) - 2) / 2.0;
    const double dy = jitter * (((face * 3 + idx * 5) % 5) - 2) / 2.0;
    const double dz = jitter * (((face * 5 + idx * 7) % 5) - 2) / 2.0;
    return m::Point3{p.x + dx, p.y + dy, p.z + dz};
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

// The +Z (top) face is q[1]; lift its ring by g → a uniform near-miss seam gap.
static std::vector<Quad> boxTopSeamQuads(const Box& b, double g) {
  std::vector<Quad> q = boxQuads(b, 0.0);
  for (auto& p : q[1].p) p.z += g;
  return q;
}
static std::vector<Quad> boxMissingTop(const Box& b) {
  std::vector<Quad> q = boxQuads(b, 1e-6);
  q.erase(q.begin() + 1);  // drop +Z
  return q;
}
static std::vector<Quad> boxMissingTwoOpposite(const Box& b) {
  std::vector<Quad> q = boxQuads(b, 1e-6);
  q.erase(q.begin() + 1);  // +Z
  q.erase(q.begin() + 0);  // −Z
  return q;
}
static std::vector<Quad> boxMissingTwoAdjacent(const Box& b) {
  std::vector<Quad> q = boxQuads(b, 1e-6);
  q.erase(q.begin() + 5);  // +X
  q.erase(q.begin() + 1);  // +Z
  return q;
}

// A box whose +Z top face's c4→c5 side carries a COLLINEAR short edge of length seg
// centred on its midpoint (the other five faces are plain quads carrying the plain span).
struct HexTopSoup { std::vector<Quad> quads; m::Point3 topHex[6]; m::Dir3 topN{0,0,1}; };
static HexTopSoup boxTopShortEdgeSoup(const Box& b, double seg) {
  m::Point3 c[8]; boxCorners(b, c);
  const m::Vec3 e = c[5] - c[4]; const m::Vec3 u = e / m::norm(e);
  const m::Point3 mid = c[4] + e * 0.5;
  const m::Point3 B = mid - u * (seg * 0.5), C = mid + u * (seg * 0.5);
  HexTopSoup h;
  h.quads.push_back({{c[0],c[3],c[2],c[1]}, m::Dir3{0,0,-1}});
  h.quads.push_back({{c[0],c[1],c[5],c[4]}, m::Dir3{0,-1,0}});
  h.quads.push_back({{c[3],c[7],c[6],c[2]}, m::Dir3{0,1,0}});
  h.quads.push_back({{c[0],c[4],c[7],c[3]}, m::Dir3{-1,0,0}});
  h.quads.push_back({{c[1],c[2],c[6],c[5]}, m::Dir3{1,0,0}});
  h.topHex[0]=c[4]; h.topHex[1]=B; h.topHex[2]=C; h.topHex[3]=c[5]; h.topHex[4]=c[6]; h.topHex[5]=c[7];
  return h;
}

// A box whose +Z top face's c4→c5 side carries ONE extra COLLINEAR vertex at parameter t.
struct PentaTopSoup { std::vector<Quad> quads; m::Point3 topPenta[5]; m::Dir3 topN{0,0,1}; };
static PentaTopSoup boxTopCollinearVertSoup(const Box& b, double t) {
  m::Point3 c[8]; boxCorners(b, c);
  const m::Point3 B = c[4] + (c[5] - c[4]) * t;
  PentaTopSoup h;
  h.quads.push_back({{c[0],c[3],c[2],c[1]}, m::Dir3{0,0,-1}});
  h.quads.push_back({{c[0],c[1],c[5],c[4]}, m::Dir3{0,-1,0}});
  h.quads.push_back({{c[3],c[7],c[6],c[2]}, m::Dir3{0,1,0}});
  h.quads.push_back({{c[0],c[4],c[7],c[3]}, m::Dir3{-1,0,0}});
  h.quads.push_back({{c[1],c[2],c[6],c[5]}, m::Dir3{1,0,0}});
  h.topPenta[0]=c[4]; h.topPenta[1]=B; h.topPenta[2]=c[5]; h.topPenta[3]=c[6]; h.topPenta[4]=c[7];
  return h;
}

// ── convex N-gon prism (planar, closed form) for the jitter / flipped / beyond-gap probes ──
struct Prism { m::Point3 o; int n; double r; double h; };  // regular n-gon radius r, height h @ o
static double prismVolume(const Prism& p) {
  const double A = 0.5 * p.n * p.r * p.r * std::sin(2.0 * M_PI / p.n);
  return A * p.h;
}
static double prismArea(const Prism& p) {
  const double A = 0.5 * p.n * p.r * p.r * std::sin(2.0 * M_PI / p.n);
  const double side = 2.0 * p.r * std::sin(M_PI / p.n);
  return 2.0 * A + p.n * side * p.h;
}
// Bottom ring b[i] then top ring t[i].
static void prismRings(const Prism& p, std::vector<m::Point3>& bot, std::vector<m::Point3>& top) {
  bot.resize(p.n); top.resize(p.n);
  for (int i = 0; i < p.n; ++i) {
    const double a = 2.0 * M_PI * i / p.n;
    const double x = p.o.x + p.r * std::cos(a), y = p.o.y + p.r * std::sin(a);
    bot[i] = {x, y, p.o.z}; top[i] = {x, y, p.o.z + p.h};
  }
}
// The prism faces as outward quads/polys: bottom (−Z), top (+Z), and n side quads.
// A jitter perturbs every corner deterministically (coincident-within-tol soup).
struct PolyFace { std::vector<m::Point3> p; m::Dir3 n; };
static std::vector<PolyFace> prismFaces(const Prism& p, double jitter) {
  std::vector<m::Point3> bot, top; prismRings(p, bot, top);
  auto jit = [&](m::Point3 q, int f, int i) {
    const double dx = jitter * (((f*7+i*3)%5)-2)/2.0, dy = jitter * (((f*3+i*5)%5)-2)/2.0,
                 dz = jitter * (((f*5+i*7)%5)-2)/2.0;
    return m::Point3{q.x+dx, q.y+dy, q.z+dz};
  };
  std::vector<PolyFace> faces;
  PolyFace bf; bf.n = m::Dir3{0,0,-1};
  for (int i = p.n - 1; i >= 0; --i) bf.p.push_back(jit(bot[i], 0, i));  // −Z, outward-down winding
  faces.push_back(bf);
  PolyFace tf; tf.n = m::Dir3{0,0,1};
  for (int i = 0; i < p.n; ++i) tf.p.push_back(jit(top[i], 1, i));       // +Z
  faces.push_back(tf);
  for (int i = 0; i < p.n; ++i) {
    const int j = (i + 1) % p.n;
    const m::Point3 edgeMid{(bot[i].x + bot[j].x) * 0.5, (bot[i].y + bot[j].y) * 0.5,
                            (bot[i].z + bot[j].z) * 0.5};
    const m::Vec3 mid = edgeMid - p.o;  // outward radial (z=0 component)
    const double rl = std::hypot(mid.x, mid.y);
    PolyFace sf; sf.n = m::Dir3{mid.x / rl, mid.y / rl, 0.0};
    sf.p.push_back(jit(bot[i], 2 + i, 0)); sf.p.push_back(jit(bot[j], 2 + i, 1));
    sf.p.push_back(jit(top[j], 2 + i, 2)); sf.p.push_back(jit(top[i], 2 + i, 3));
    faces.push_back(sf);
  }
  return faces;
}

// ── native builders (identical to native_heal_parity.mm) ───────────────────────────
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
static topo::Shape nativeQuadFace(const Quad& q, bool reversed) {
  topo::Shape f = nativePolyFace(q.p, 4, q.n);
  return reversed ? f.reversedShape() : f;
}
static topo::Shape nativeSoup(const std::vector<Quad>& quads, int flipIndex = -1) {
  std::vector<topo::Shape> faces;
  for (int i = 0; i < (int)quads.size(); ++i)
    faces.push_back(nativeQuadFace(quads[i], i == flipIndex));
  return topo::ShapeBuilder::makeShell(faces);
}
static topo::Shape nativeShortEdgeSoup(const HexTopSoup& h) {
  std::vector<topo::Shape> faces;
  for (const Quad& q : h.quads) faces.push_back(nativeQuadFace(q, false));
  faces.push_back(nativePolyFace(h.topHex, 6, h.topN));
  return topo::ShapeBuilder::makeShell(faces);
}
static topo::Shape nativeCollinearVertSoup(const PentaTopSoup& h) {
  std::vector<topo::Shape> faces;
  for (const Quad& q : h.quads) faces.push_back(nativeQuadFace(q, false));
  faces.push_back(nativePolyFace(h.topPenta, 5, h.topN));
  return topo::ShapeBuilder::makeShell(faces);
}
static topo::Shape nativePrismSoup(const std::vector<PolyFace>& faces, int flipIndex) {
  std::vector<topo::Shape> fs;
  for (int i = 0; i < (int)faces.size(); ++i) {
    topo::Shape f = nativePolyFace(faces[i].p.data(), (int)faces[i].p.size(), faces[i].n);
    fs.push_back(i == flipIndex ? f.reversedShape() : f);
  }
  return topo::ShapeBuilder::makeShell(fs);
}

// ── OCCT soup builders (a compound of independent planar faces) ────────────────────
static void occtAddPoly(BRep_Builder& bb, TopoDS_Compound& comp,
                        const m::Point3* pts, int n, const m::Dir3& nrm) {
  BRepBuilderAPI_MakePolygon poly;
  for (int i = 0; i < n; ++i) poly.Add(gp_Pnt(pts[i].x, pts[i].y, pts[i].z));
  poly.Close();
  if (!poly.IsDone()) return;
  const gp_Pln plane(gp_Pnt(pts[0].x, pts[0].y, pts[0].z), gp_Dir(nrm.x(), nrm.y(), nrm.z()));
  BRepBuilderAPI_MakeFace mf(plane, poly.Wire());
  if (mf.IsDone()) bb.Add(comp, mf.Face());
}
static TopoDS_Shape occtSoup(const std::vector<Quad>& quads) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : quads) occtAddPoly(bb, comp, q.p, 4, q.n);
  return comp;
}
static TopoDS_Shape occtPrismSoup(const std::vector<PolyFace>& faces) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const PolyFace& f : faces) occtAddPoly(bb, comp, f.p.data(), (int)f.p.size(), f.n);
  return comp;
}
static TopoDS_Shape occtShortEdgeSoup(const HexTopSoup& h) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : h.quads) occtAddPoly(bb, comp, q.p, 4, q.n);
  occtAddPoly(bb, comp, h.topHex, 6, h.topN);
  return comp;
}
static TopoDS_Shape occtCollinearVertSoup(const PentaTopSoup& h) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : h.quads) occtAddPoly(bb, comp, q.p, 4, q.n);
  occtAddPoly(bb, comp, h.topPenta, 5, h.topN);
  return comp;
}
static TopoDS_Shape occtSoupWithPlanarCap(const std::vector<Quad>& openQuads,
                                          const m::Point3 cap[4], const m::Dir3& capN) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : openQuads) occtAddPoly(bb, comp, q.p, 4, q.n);
  occtAddPoly(bb, comp, cap, 4, capN);
  return comp;
}
static TopoDS_Shape occtSoupWithTwoPlanarCaps(const std::vector<Quad>& openQuads,
                                              const m::Point3 capA[4], const m::Dir3& nA,
                                              const m::Point3 capB[4], const m::Dir3& nB) {
  BRep_Builder bb; TopoDS_Compound comp; bb.MakeCompound(comp);
  for (const Quad& q : openQuads) occtAddPoly(bb, comp, q.p, 4, q.n);
  occtAddPoly(bb, comp, capA, 4, nA);
  occtAddPoly(bb, comp, capB, 4, nB);
  return comp;
}

// ── native measurement (reproduces the healer's self-verify tessellation) ──────────
struct NativeMeas { bool watertight; double volume; double area; };
static NativeMeas nativeMeasure(const topo::Shape& s) {
  tess::MeshParams p; p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(s);
  if (!tess::isWatertight(mesh)) return {false, 0.0, 0.0};
  return {true, std::fabs(tess::enclosedVolume(mesh)), tess::surfaceArea(mesh)};
}

// ── classification ────────────────────────────────────────────────────────────────
enum Verdict { AGREED, HONESTLY_DECLINED, DISAGREED, ORACLE_INACCURATE, BOTH_DECLINED,
               ORACLE_UNRELIABLE, V_COUNT };
const char* verdictName(int v) {
  switch (v) {
    case AGREED: return "AGREED";
    case HONESTLY_DECLINED: return "HONESTLY-DECLINED";
    case DISAGREED: return "DISAGREED";
    case ORACLE_INACCURATE: return "ORACLE-INACCURATE";
    case BOTH_DECLINED: return "BOTH-DECLINED";
    case ORACLE_UNRELIABLE: return "ORACLE_UNRELIABLE";
  }
  return "?";
}

enum Base { B_UNIT_CUBE, B_BOX, B_PRISM, B_COUNT };
const char* baseName(int b) {
  switch (b) { case B_UNIT_CUBE: return "unit-cube"; case B_BOX: return "box";
               case B_PRISM: return "prism"; } return "?"; }

enum Defect { D_SEW_JITTER, D_FLIPPED, D_SEAM_IN, D_SEAM_OUT, D_SHORT_EDGE, D_SHORT_OFF,
              D_COLL_VERT, D_COLL_OFF, D_MISS1, D_MISS2_OPP, D_MISS2_ADJ, D_BEYOND, D_COUNT };
const char* defectName(int d) {
  switch (d) {
    case D_SEW_JITTER: return "sew-jitter";     case D_FLIPPED:    return "flipped-face";
    case D_SEAM_IN:    return "seam-gap-in";    case D_SEAM_OUT:   return "seam-gap-out";
    case D_SHORT_EDGE: return "short-edge";     case D_SHORT_OFF:  return "short-edge-off";
    case D_COLL_VERT:  return "collinear-vert"; case D_COLL_OFF:   return "coll-vert-off";
    case D_MISS1:      return "missing-1-hole"; case D_MISS2_OPP:  return "missing-2-opp";
    case D_MISS2_ADJ:  return "missing-2-adj";  case D_BEYOND:     return "beyond-tol-gap";
  }
  return "?";
}

// Which defects apply to which base. Prisms only take the base-agnostic families
// (jitter / flipped / beyond); the seam/short/collinear/missing families rely on the
// box's fixed face indexing and top-ring geometry, so run on cube/box only.
static bool defectAppliesTo(int base, int defect) {
  if (base == B_PRISM)
    return defect == D_SEW_JITTER || defect == D_FLIPPED || defect == D_BEYOND;
  return true;
}

struct Counters {
  long v[V_COUNT] = {0};
  long byBase[B_COUNT] = {0};
  long byDefect[D_COUNT] = {0};
  long agreeByBase[B_COUNT] = {0};
  long agreeByDefect[D_COUNT] = {0};
  long nonErrByBase[B_COUNT] = {0};
  long nonErrByDefect[D_COUNT] = {0};
};

static const double kTol = 1e-4;   // sew/weld tolerance (matches curated harness)
static const double kBand = 1e-3;  // exact-closure volume/area band (deflection-bounded)

// Classify one AGREE-expected in-scope HEAL: native must land the closed-form truth.
static int classifyHeal(const heal::HealResult& nr, const NativeMeas& nm,
                        const cyber::occt::SewFixResult& orr, double Vtruth, double Atruth,
                        double band) {
  const bool nativeMatch = nr.healed() && nm.watertight &&
                           std::fabs(nm.volume - Vtruth) < band &&
                           std::fabs(nm.area - Atruth) < band * (Atruth + 1.0);
  const bool nativeWrong = nr.healed() && nm.watertight &&
                           (std::fabs(nm.volume - Vtruth) >= band ||
                            std::fabs(nm.area - Atruth) >= band * (Atruth + 1.0));
  const bool occtMatch = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < band;
  if (nativeWrong) return DISAGREED;                       // silent wrong repair
  if (nativeMatch && occtMatch) return AGREED;             // both correct
  if (nativeMatch && !occtMatch) return ORACLE_INACCURATE; // native vindicated
  if (!nr.healed()) return HONESTLY_DECLINED;              // native deferred (safe)
  return ORACLE_UNRELIABLE;                                // native healed-not-truth impossible here
}

int main(int argc, char** argv) {
  uint64_t seed = 0x4845414C4Full;  // "HEALO"
  long N = 120;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = strtol(argv[2], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_N")) N = strtol(e, nullptr, 0);

  std::printf("[HFUZZ] seed=0x%llX N=%ld tol=%.1e band=%.1e\n",
              (unsigned long long)seed, N, kTol, kBand);

  Rng rng(seed);
  Counters cc;

  for (long i = 0; i < N; ++i) {
    // ── choose a base + a defect that applies to it ──
    int base = (int)rng.below(B_COUNT);
    int defect;
    do { defect = (int)rng.below(D_COUNT); } while (!defectAppliesTo(base, defect));

    // ── build the box / prism dimensions (unit cube is fixed 1,1,1 @ 0) ──
    Box b{{0, 0, 0}, 1.0, 1.0, 1.0};
    Prism pr{{0, 0, 0}, 4, 1.0, 1.0};
    if (base == B_BOX) {
      b = Box{{rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)},
              rng.range(0.6, 2.2), rng.range(0.6, 2.2), rng.range(0.6, 2.2)};
    } else if (base == B_PRISM) {
      pr = Prism{{rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)},
                 3 + (int)rng.below(6), rng.range(0.6, 1.6), rng.range(0.6, 2.0)};
    }

    // A random SUB-TOLERANCE sew jitter: the largest coincidence gap (2·jitter) stays
    // well under the weld tolerance so a coincident-within-tol soup heals reliably, while
    // still fuzzing the exact perturbation. Above ~0.4·tol the coincidence gap approaches
    // tol and the sew legitimately declines (that is the BEYOND-TOL family, not this one).
    const double jit = rng.range(1e-7, kTol * 0.2);

    heal::HealResult nr;
    NativeMeas nm{false, 0, 0};
    cyber::occt::SewFixResult orr;
    int verdict = HONESTLY_DECLINED;
    double Vtruth = 0, Atruth = 0;
    char detail[128] = {0};

    if (base == B_PRISM) {
      Vtruth = prismVolume(pr); Atruth = prismArea(pr);
      std::snprintf(detail, sizeof detail, "n=%d r=%.3f h=%.3f", pr.n, pr.r, pr.h);
      if (defect == D_SEW_JITTER) {
        const std::vector<PolyFace> f = prismFaces(pr, jit);
        nr = heal::healShell(nativePrismSoup(f, -1), heal::HealOptions{kTol});
        nm = nativeMeasure(nr.shape); orr = cyber::occt::sewAndFix(occtPrismSoup(f), kTol);
        verdict = classifyHeal(nr, nm, orr, Vtruth, Atruth, kBand);
      } else if (defect == D_FLIPPED) {
        const std::vector<PolyFace> f = prismFaces(pr, jit);
        const int flip = 1 + (int)rng.below((uint32_t)f.size() - 1);
        nr = heal::healShell(nativePrismSoup(f, flip), heal::HealOptions{kTol});
        nm = nativeMeasure(nr.shape); orr = cyber::occt::sewAndFix(occtPrismSoup(f), kTol);
        verdict = classifyHeal(nr, nm, orr, Vtruth, Atruth, kBand);
      } else {  // D_BEYOND
        const std::vector<PolyFace> f = prismFaces(pr, 1e-2);  // gaps ≫ tol
        nr = heal::healShell(nativePrismSoup(f, -1), heal::HealOptions{kTol});
        orr = cyber::occt::sewAndFix(occtPrismSoup(f), kTol);
        const bool nativeDeclined = !nr.healed() && nr.metrics.maxResidualGap > kTol;
        const bool occtOpen = !orr.validSolid;
        if (nativeDeclined && occtOpen) verdict = BOTH_DECLINED;
        else if (nativeDeclined) verdict = HONESTLY_DECLINED;
        else { nm = nativeMeasure(nr.shape); verdict = nm.watertight ? DISAGREED : HONESTLY_DECLINED; }
      }
    } else {  // B_UNIT_CUBE or B_BOX
      Vtruth = boxVolume(b); Atruth = boxArea(b);
      std::snprintf(detail, sizeof detail, "s=%.3f,%.3f,%.3f o=%.2f,%.2f,%.2f",
                    b.sx, b.sy, b.sz, b.o.x, b.o.y, b.o.z);
      m::Point3 c[8]; boxCorners(b, c);
      switch (defect) {
        case D_SEW_JITTER: {
          const std::vector<Quad> q = boxQuads(b, jit);
          nr = heal::healShell(nativeSoup(q), heal::HealOptions{kTol});
          nm = nativeMeasure(nr.shape); orr = cyber::occt::sewAndFix(occtSoup(q), kTol);
          verdict = classifyHeal(nr, nm, orr, Vtruth, Atruth, kBand);
        } break;
        case D_FLIPPED: {
          const std::vector<Quad> q = boxQuads(b, jit);
          const int flip = (int)rng.below(6);
          nr = heal::healShell(nativeSoup(q, flip), heal::HealOptions{kTol});
          nm = nativeMeasure(nr.shape); orr = cyber::occt::sewAndFix(occtSoup(q), kTol);
          verdict = classifyHeal(nr, nm, orr, Vtruth, Atruth, kBand);
        } break;
        case D_SEAM_IN: {
          const double budget = 1e-2, g = rng.range(kTol * 2, budget * 0.8);
          nr = heal::healShell(nativeSoup(boxTopSeamQuads(b, g)), heal::HealOptions{kTol, budget});
          nm = nativeMeasure(nr.shape);
          orr = cyber::occt::sewAndFix(occtSoup(boxTopSeamQuads(b, g)), budget);
          // Bridged vertex sits within the gap → volume/area agree within ~g, not kBand.
          const bool nativeMatch = nr.healed() && nm.watertight &&
                                   std::fabs(nm.volume - Vtruth) < budget * (Vtruth + 1.0) &&
                                   nr.metrics.nBridgedGaps > 0;
          const bool nativeWrong = nr.healed() && nm.watertight && !nativeMatch;
          const bool occtOk = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < budget * (Vtruth + 1.0);
          if (nativeWrong) verdict = DISAGREED;
          else if (nativeMatch && occtOk) verdict = AGREED;
          else if (nativeMatch) verdict = ORACLE_INACCURATE;
          else verdict = HONESTLY_DECLINED;
          std::snprintf(detail + strlen(detail), sizeof detail - strlen(detail), " g=%.4f", g);
        } break;
        case D_SEAM_OUT: {
          const double budget = 1e-2, g = rng.range(budget * 2, budget * 8);
          nr = heal::healShell(nativeSoup(boxTopSeamQuads(b, g)), heal::HealOptions{kTol, budget});
          orr = cyber::occt::sewAndFix(occtSoup(boxTopSeamQuads(b, g)), budget);
          const bool nativeDeclined = !nr.healed() &&
              nr.reason == heal::UnhealedReason::GapBeyondBudget && nr.metrics.maxResidualGap > budget;
          if (nativeDeclined && !orr.validSolid) verdict = BOTH_DECLINED;
          else if (nativeDeclined) verdict = HONESTLY_DECLINED;
          else { nm = nativeMeasure(nr.shape); verdict = nm.watertight ? DISAGREED : HONESTLY_DECLINED; }
        } break;
        case D_SHORT_EDGE: {
          const double seg = rng.range(kTol * 2, 8e-3), mergeLen = 1e-2, occtTol = 1e-2;
          const HexTopSoup h = boxTopShortEdgeSoup(b, seg);
          nr = heal::healShell(nativeShortEdgeSoup(h),
                               heal::HealOptions{kTol, 0.0, false, false, mergeLen});
          nm = nativeMeasure(nr.shape);
          orr = cyber::occt::sewAndFix(occtShortEdgeSoup(h), occtTol);
          const bool nativeMatch = nr.healed() && nm.watertight &&
                                   std::fabs(nm.volume - Vtruth) < kBand &&
                                   nr.metrics.nCollapsedShortEdges > 0;
          const bool nativeWrong = nr.healed() && nm.watertight && !nativeMatch;
          const bool occtOk = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < occtTol * (Vtruth + 1.0);
          if (nativeWrong) verdict = DISAGREED;
          else if (nativeMatch && occtOk) verdict = AGREED;
          else if (nativeMatch) verdict = ORACLE_INACCURATE;
          else verdict = HONESTLY_DECLINED;
          std::snprintf(detail + strlen(detail), sizeof detail - strlen(detail), " seg=%.4f", seg);
        } break;
        case D_SHORT_OFF: {
          // Flag OFF: native declines the split, OCCT aggressively repairs. Native being
          // MORE conservative is AGREED (safe deferral), provided OCCT's closure is the
          // honest truth (or OCCT also declined).
          const double seg = rng.range(kTol * 2, 8e-3);
          const HexTopSoup h = boxTopShortEdgeSoup(b, seg);
          nr = heal::healShell(nativeShortEdgeSoup(h), heal::HealOptions{kTol});  // OFF
          orr = cyber::occt::sewAndFix(occtShortEdgeSoup(h), kTol);
          const bool nativeConservative = !nr.healed() &&
              nr.metrics.nCollapsedShortEdges == 0 && nr.metrics.maxResidualGap > kTol;
          const bool occtHonest = !orr.watertight ||
              std::fabs(std::fabs(orr.volume) - Vtruth) < kBand;
          if (nr.healed()) { nm = nativeMeasure(nr.shape);
            verdict = (nm.watertight && std::fabs(nm.volume - Vtruth) >= kBand) ? DISAGREED : AGREED; }
          else if (nativeConservative && occtHonest) verdict = AGREED;  // more-conservative
          else verdict = HONESTLY_DECLINED;
          std::snprintf(detail + strlen(detail), sizeof detail - strlen(detail), " seg=%.4f", seg);
        } break;
        case D_COLL_VERT: {
          const double t = rng.range(0.2, 0.8);
          const PentaTopSoup h = boxTopCollinearVertSoup(b, t);
          nr = heal::healShell(nativeCollinearVertSoup(h),
                               heal::HealOptions{kTol, 0.0, false, false, 0.0, true});
          nm = nativeMeasure(nr.shape);
          orr = cyber::occt::sewAndFix(occtCollinearVertSoup(h), kTol);
          const bool nativeMatch = nr.healed() && nm.watertight &&
                                   std::fabs(nm.volume - Vtruth) < kBand &&
                                   nr.metrics.nRemovedCollinearVerts > 0;
          const bool nativeWrong = nr.healed() && nm.watertight && !nativeMatch;
          const bool occtOk = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < kBand;
          if (nativeWrong) verdict = DISAGREED;
          else if (nativeMatch && occtOk) verdict = AGREED;
          else if (nativeMatch) verdict = ORACLE_INACCURATE;
          else verdict = HONESTLY_DECLINED;
          std::snprintf(detail + strlen(detail), sizeof detail - strlen(detail), " t=%.3f", t);
        } break;
        case D_COLL_OFF: {
          const double t = rng.range(0.2, 0.8);
          const PentaTopSoup h = boxTopCollinearVertSoup(b, t);
          nr = heal::healShell(nativeCollinearVertSoup(h), heal::HealOptions{kTol});  // OFF
          orr = cyber::occt::sewAndFix(occtCollinearVertSoup(h), kTol);
          const bool nativeConservative = !nr.healed() &&
              nr.metrics.nRemovedCollinearVerts == 0 && nr.metrics.maxResidualGap > kTol;
          const bool occtHonest = !orr.watertight ||
              std::fabs(std::fabs(orr.volume) - Vtruth) < kBand;
          if (nr.healed()) { nm = nativeMeasure(nr.shape);
            verdict = (nm.watertight && std::fabs(nm.volume - Vtruth) >= kBand) ? DISAGREED : AGREED; }
          else if (nativeConservative && occtHonest) verdict = AGREED;  // more-conservative
          else verdict = HONESTLY_DECLINED;
          std::snprintf(detail + strlen(detail), sizeof detail - strlen(detail), " t=%.3f", t);
        } break;
        case D_MISS1: {
          const m::Point3 topCap[4] = {c[4], c[5], c[6], c[7]};  // the +Z boundary ring
          nr = heal::healShell(nativeSoup(boxMissingTop(b)),
                               heal::HealOptions{kTol, 0.0, true});
          nm = nativeMeasure(nr.shape);
          orr = cyber::occt::sewAndFix(
              occtSoupWithPlanarCap(boxMissingTop(b), topCap, m::Dir3{0, 0, 1}), kTol);
          const bool nativeMatch = nr.healed() && nm.watertight &&
                                   std::fabs(nm.volume - Vtruth) < kBand &&
                                   nr.metrics.nCappedFaces == 1;
          const bool nativeWrong = nr.healed() && nm.watertight && !nativeMatch;
          const bool occtOk = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < kBand;
          if (nativeWrong) verdict = DISAGREED;
          else if (nativeMatch && occtOk) verdict = AGREED;
          else if (nativeMatch) verdict = ORACLE_INACCURATE;
          else verdict = HONESTLY_DECLINED;
        } break;
        case D_MISS2_OPP: {
          const m::Point3 topCap[4]    = {c[4], c[5], c[6], c[7]};  // +Z hole
          const m::Point3 bottomCap[4] = {c[0], c[3], c[2], c[1]};  // −Z hole
          nr = heal::healShell(nativeSoup(boxMissingTwoOpposite(b)),
                               heal::HealOptions{kTol, 0.0, false, true});
          nm = nativeMeasure(nr.shape);
          orr = cyber::occt::sewAndFix(
              occtSoupWithTwoPlanarCaps(boxMissingTwoOpposite(b), topCap, m::Dir3{0, 0, 1},
                                        bottomCap, m::Dir3{0, 0, -1}), kTol);
          const bool nativeMatch = nr.healed() && nm.watertight &&
                                   std::fabs(nm.volume - Vtruth) < kBand &&
                                   nr.metrics.nCappedFaces == 2;
          const bool nativeWrong = nr.healed() && nm.watertight && !nativeMatch;
          const bool occtOk = orr.validSolid && std::fabs(std::fabs(orr.volume) - Vtruth) < kBand;
          if (nativeWrong) verdict = DISAGREED;
          else if (nativeMatch && occtOk) verdict = AGREED;
          else if (nativeMatch) verdict = ORACLE_INACCURATE;
          else verdict = HONESTLY_DECLINED;
        } break;
        case D_MISS2_ADJ: {
          nr = heal::healShell(nativeSoup(boxMissingTwoAdjacent(b)),
                               heal::HealOptions{kTol, 0.0, false, true});
          orr = cyber::occt::sewAndFix(occtSoup(boxMissingTwoAdjacent(b)), kTol);
          const bool nativeDeclined = !nr.healed() && nr.metrics.nCappedFaces == 0;
          if (nativeDeclined && !orr.watertight) verdict = BOTH_DECLINED;
          else if (nativeDeclined) verdict = HONESTLY_DECLINED;
          else { nm = nativeMeasure(nr.shape); verdict = nm.watertight ? DISAGREED : HONESTLY_DECLINED; }
        } break;
        case D_BEYOND: {
          const std::vector<Quad> q = boxQuads(b, 1e-2);  // gaps ~1e-2 ≫ tol
          nr = heal::healShell(nativeSoup(q), heal::HealOptions{kTol});
          orr = cyber::occt::sewAndFix(occtSoup(q), kTol);
          const bool nativeDeclined = !nr.healed() && nr.metrics.maxResidualGap > kTol;
          if (nativeDeclined && !orr.validSolid) verdict = BOTH_DECLINED;
          else if (nativeDeclined) verdict = HONESTLY_DECLINED;
          else { nm = nativeMeasure(nr.shape); verdict = nm.watertight ? DISAGREED : HONESTLY_DECLINED; }
        } break;
      }
    }

    // ── tally + log every noteworthy trial ──
    cc.v[verdict]++; cc.byBase[base]++; cc.byDefect[defect]++;
    if (verdict == AGREED) { cc.agreeByBase[base]++; cc.agreeByDefect[defect]++; }
    // A trial is "non-error" unless it is ORACLE_UNRELIABLE (a broken oracle). Every other
    // verdict — including BOTH_DECLINED, which is the intended outcome of the out-of-scope
    // decline probes — is a legitimate exercised trial and counts toward coverage.
    if (verdict != ORACLE_UNRELIABLE) { cc.nonErrByBase[base]++; cc.nonErrByDefect[defect]++; }
    if (verdict == DISAGREED || verdict == ORACLE_INACCURATE || verdict == ORACLE_UNRELIABLE ||
        getenv("HFUZZ_DEBUG")) {
      std::printf("[HFUZZ]  %-18s i=%ld base=%s defect=%s %s  "
                  "(native healed=%d wt=%d V=%.5f A=%.5f | truth V=%.5f A=%.5f | "
                  "OCCT valid=%d V=%.5f)\n",
                  verdictName(verdict), i, baseName(base), defectName(defect), detail,
                  (int)nr.healed(), (int)nm.watertight, nm.volume, nm.area, Vtruth, Atruth,
                  (int)orr.validSolid, orr.volume);
    }
  }

  // ── coverage summary ──
  std::printf("\n[HFUZZ] ── coverage (seed=0x%llX N=%ld) ──\n", (unsigned long long)seed, N);
  for (int v = 0; v < V_COUNT; ++v)
    std::printf("[HFUZZ]   %-18s %ld\n", verdictName(v), cc.v[v]);
  std::printf("[HFUZZ] ── per base family ──\n");
  bool baseOk = true;
  for (int b = 0; b < B_COUNT; ++b) {
    std::printf("[HFUZZ]   %-10s trials=%ld agreed=%ld nonErr=%ld\n",
                baseName(b), cc.byBase[b], cc.agreeByBase[b], cc.nonErrByBase[b]);
    if (cc.nonErrByBase[b] == 0) baseOk = false;
  }
  std::printf("[HFUZZ] ── per defect family ──\n");
  // HEAL-EXPECTED families must land at least one AGREED (native heals to the closed-form
  // truth, or safely-more-conservative-than-OCCT for the flag-OFF probes). DECLINE-PROBE
  // families (out-of-scope by construction) only need to be EXERCISED (≥1 non-error trial:
  // an honest / both-decline). Both require ≥1 trial to prove the family ran.
  auto healExpected = [](int d) {
    return d == D_SEW_JITTER || d == D_FLIPPED || d == D_SEAM_IN || d == D_SHORT_EDGE ||
           d == D_SHORT_OFF || d == D_COLL_VERT || d == D_COLL_OFF || d == D_MISS1 ||
           d == D_MISS2_OPP;
  };
  bool defectOk = true;
  for (int d = 0; d < D_COUNT; ++d) {
    const char* kind = healExpected(d) ? "heal " : "declines";
    std::printf("[HFUZZ]   %-16s (%s) trials=%ld agreed=%ld nonErr=%ld\n",
                defectName(d), kind, cc.byDefect[d], cc.agreeByDefect[d], cc.nonErrByDefect[d]);
    if (cc.byDefect[d] == 0) { defectOk = false; continue; }            // never exercised
    if (healExpected(d) && cc.agreeByDefect[d] == 0) defectOk = false;  // heal family never healed
    if (!healExpected(d) && cc.nonErrByDefect[d] == 0) defectOk = false;// probe never exercised cleanly
  }

  const bool barHolds = cc.v[DISAGREED] == 0 && cc.v[ORACLE_UNRELIABLE] == 0 &&
                        baseOk && defectOk;
  std::printf("\n[HFUZZ] HONEST SCOPE: defects are shape-preserving perturbations of a KNOWN "
              "solid; out-of-scope declines (beyond-tol-gap, two-adjacent missing faces, "
              "out-of-budget seam) are first-class HONESTLY-DECLINED/BOTH-DECLINED.\n");
  std::printf("[HFUZZ] BAR: DISAGREED=%ld ORACLE_UNRELIABLE=%ld baseCoverage=%d defectCoverage=%d "
              "→ %s\n", cc.v[DISAGREED], cc.v[ORACLE_UNRELIABLE], (int)baseOk, (int)defectOk,
              barHolds ? "PASS" : "FAIL");
  std::fflush(stdout);
  std::_Exit(barHolds ? 0 : 1);
}
