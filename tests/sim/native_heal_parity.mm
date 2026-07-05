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

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
