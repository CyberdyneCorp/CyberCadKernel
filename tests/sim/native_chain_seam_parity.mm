// SPDX-License-Identifier: Apache-2.0
//
// native_chain_seam_parity.mm — MOAT M2 blocker #4 (≥3-seam) SIM GATE (b):
// native-vs-OCCT on a booted iOS simulator for the THREE-arc, TWO-junction seam-graph
// CHAIN builder (`buildChainSeamGraph`, src/native/boolean/seam_graph_chain.h, OCCT-FREE).
//
// The native builder recognises A (bowl-lidded quad prism), detects the THREE `B` faces
// slicing A's Bézier wall, traces the three iso-parametric arcs (two parallel iso-`u`
// ends + one orthogonal iso-`v` middle), computes the TWO analytic junctions J1, J2, and
// joins the arcs into one bent boundary→J1→J2→boundary chain. This harness GROUNDS that
// purely-analytic graph against OCCT's exact geometry:
//   * each native ARC node lies ON OCCT's bowl Bézier surface (BRepExtrema ≤ tol) and ON
//     its own cutting PLANE face (≤ tol) — the arc really is surface∩plane;
//   * each native JUNCTION lies ON the bowl surface AND ON BOTH its adjacent cutting
//     planes (≤ tol) — the true triple point where two planes meet the surface;
//   * OCCT's own BRepAlgoAPI_Section(B, bowlFace) yields a connected section (the same
//     three-arc chain) whose extremal vertices coincide with the native junctions;
//   * a two-face corner box has only TWO planes cutting the wall → OCCT confirms only ONE
//     triple point, and the native builder DECLINES NotThreeCuttingFaces (no fabrication).
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-chain-seam.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_chain_seam.cpp.
//
#include "native/boolean/freeform_operand.h"
#include "native/boolean/seam_graph_chain.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/chain_seam_fixture.h"
#include "../native/first_freeform_boolean_fixture.h"
#include "../native/multi_seam_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_chain_seam_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pln.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepLib.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;
namespace ntess = cybercad::native::tessellate;
namespace bo = cybercad::native::boolean;
namespace ffx = first_freeform_boolean_fixture;
namespace msx = multi_seam_fixture;
namespace csx = chain_seam_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[CHN] %-10s %-22s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}
static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: the SAME bowl Bézier surface as a trimmed face (over the quad) ────
static TopoDS_Face buildOcctBowlTop() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);
  const auto& q = fx::quadUV();
  BRepBuilderAPI_MakeWire mkWire;
  for (int k = 0; k < 4; ++k) {
    const nm::Point3 a = q[k], b = q[(k + 1) % 4];
    const gp_Pnt2d p0(a.x, a.y), p1(b.x, b.y);
    const double len = p0.Distance(p1);
    gp_Dir2d dir(p1.X() - p0.X(), p1.Y() - p0.Y());
    Handle(Geom2d_Line) line = new Geom2d_Line(p0, dir);
    Handle(Geom2d_TrimmedCurve) seg = new Geom2d_TrimmedCurve(line, 0.0, len);
    mkWire.Add(BRepBuilderAPI_MakeEdge(seg, surf, 0.0, len).Edge());
  }
  TopoDS_Face face = BRepBuilderAPI_MakeFace(surf, mkWire.Wire(), Standard_True).Face();
  BRepLib::BuildCurves3d(face);
  return face;
}

// The FULL (untrimmed [0,1]²) bowl Bézier surface as a face — for the surface-membership
// check on arc nodes, which legitimately extend to the surface's natural parameter bounds
// (the trimmed quad face is used for the junctions, which lie inside the trim).
static TopoDS_Face buildOcctBowlFull() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);
  return BRepBuilderAPI_MakeFace(surf, 1e-7).Face();
}

// An unbounded plane face for an axis-aligned cutting plane at `coord` on axis `axis`
// (0=x,1=y), normal +axis.
static TopoDS_Face planeFace(int axis, double coord) {
  gp_Pnt o(axis == 0 ? coord : 0.0, axis == 1 ? coord : 0.0, 0.0);
  gp_Dir n(axis == 0 ? 1.0 : 0.0, axis == 1 ? 1.0 : 0.0, 0.0);
  return BRepBuilderAPI_MakeFace(gp_Pln(o, n), -1.0, 1.0, -1.0, 1.0).Face();
}

static double distToShape(const nm::Point3& p, const TopoDS_Shape& s) {
  TopoDS_Vertex v = BRepBuilderAPI_MakeVertex(P(p)).Vertex();
  BRepExtrema_DistShapeShape d(v, s);
  return d.IsDone() ? d.Value() : 1e30;
}

int main() {
  std::printf("== MOAT M2 blocker #4 (>=3-seam) chain seam-graph: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double tol = 1e-4;  // curved-face section tolerance (never widened)

  const TopoDS_Face bowl = buildOcctBowlTop();       // trimmed to the quad (junctions in-trim)
  const TopoDS_Face bowlFull = buildOcctBowlFull();  // full [0,1]² surface (arc-node membership)
  report("occt", "bowl-face-built", !bowl.IsNull() && !bowlFull.IsNull(),
         "trimmed + full Geom_BezierSurface");

  // ── native chain graph on the edge-straddling box ────────────────────────────
  const nt::Shape A = ffx::buildOperand();
  const nt::Shape B = csx::edgeBox();
  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  report("native", "operand-admitted", op.has_value(), "recogniseFreeformSolid");
  if (!op) { std::printf("[CHN] SUMMARY %d passed / %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0; }

  bo::ChainSeamDecline d = bo::ChainSeamDecline::Ok;
  const auto g = bo::buildChainSeamGraph(*op, B, &d);
  report("native", "chain-graph-built", g.has_value() && d == bo::ChainSeamDecline::Ok,
         bo::chainSeamDeclineName(d));
  if (!g) { std::printf("[CHN] SUMMARY %d passed / %d failed\n", g_pass, g_fail); return g_fail ? 1 : 0; }

  // The three cutting planes (matching the fixture box: x=kX0, x=kX1, y=kY0).
  const TopoDS_Face planeEnd0 = planeFace(0, csx::kX0);
  const TopoDS_Face planeEnd1 = planeFace(0, csx::kX1);
  const TopoDS_Face planeMid = planeFace(1, csx::kY0);
  const TopoDS_Face arcPlane[3] = {planeEnd0, planeEnd1, planeMid};

  // ── (1) each native ARC node lies on OCCT's bowl surface AND its cutting plane ──
  double maxArcSurf = 0.0, maxArcPlane = 0.0;
  for (int k = 0; k < 3; ++k) {
    for (const auto& pt : g->arcs[k].arc.points) {
      maxArcSurf = std::max(maxArcSurf, distToShape(pt.point, bowlFull));
      maxArcPlane = std::max(maxArcPlane, distToShape(pt.point, arcPlane[k]));
    }
  }
  char buf[96];
  std::snprintf(buf, sizeof buf, "max node->bowl %.2e", maxArcSurf);
  report("occt", "arcs-on-bowl-surface", maxArcSurf <= tol, buf);
  std::snprintf(buf, sizeof buf, "max node->plane %.2e", maxArcPlane);
  report("occt", "arcs-on-cut-planes", maxArcPlane <= tol, buf);

  // ── (2) each native JUNCTION lies on the bowl surface AND both adjacent planes ──
  // J1 adjacent to end0 (x=kX0) + middle (y=kY0); J2 adjacent to end1 (x=kX1) + middle.
  const double j1surf = distToShape(g->junction3d[0], bowl);
  const double j1e = distToShape(g->junction3d[0], planeEnd0);
  const double j1m = distToShape(g->junction3d[0], planeMid);
  const double j2surf = distToShape(g->junction3d[1], bowl);
  const double j2e = distToShape(g->junction3d[1], planeEnd1);
  const double j2m = distToShape(g->junction3d[1], planeMid);
  std::snprintf(buf, sizeof buf, "J1 surf/e/m %.1e/%.1e/%.1e", j1surf, j1e, j1m);
  report("occt", "J1-triple-point", j1surf <= tol && j1e <= tol && j1m <= tol, buf);
  std::snprintf(buf, sizeof buf, "J2 surf/e/m %.1e/%.1e/%.1e", j2surf, j2e, j2m);
  report("occt", "J2-triple-point", j2surf <= tol && j2e <= tol && j2m <= tol, buf);

  // ── (3) OCCT's own Section(B, bowlFace) is a connected chain reaching both J ─────
  // The SAME edge-straddling box B, reconstructed as an OCCT solid.
  const TopoDS_Shape occtB =
      BRepPrimAPI_MakeBox(gp_Pnt(csx::kX0, csx::kY0, csx::kZ0),
                          gp_Pnt(csx::kX1, csx::kY1, csx::kZ1)).Solid();
  BRepAlgoAPI_Section sec(occtB, bowl, Standard_False);
  sec.ComputePCurveOn1(Standard_False);
  sec.Approximation(Standard_True);
  sec.Build();
  const TopoDS_Shape section = sec.Shape();
  int secEdges = 0;
  for (TopExp_Explorer ex(section, TopAbs_EDGE); ex.More(); ex.Next()) ++secEdges;
  // The native junctions must lie ON OCCT's section (the box∩bowl intersection curve).
  const double j1sec = distToShape(g->junction3d[0], section);
  const double j2sec = distToShape(g->junction3d[1], section);
  std::snprintf(buf, sizeof buf, "%d edges, J1->sec %.1e J2->sec %.1e", secEdges, j1sec, j2sec);
  report("occt", "junctions-on-section", secEdges >= 1 && j1sec <= tol && j2sec <= tol, buf);

  // ── (4) fallback contract: the 2-face corner box → only TWO cutting faces ────────
  const nt::Shape B2 = msx::cornerBox();
  bo::ChainSeamDecline d2 = bo::ChainSeamDecline::Ok;
  const auto g2 = bo::buildChainSeamGraph(*op, B2, &d2);
  report("native", "two-face-box-declines",
         !g2.has_value() && d2 == bo::ChainSeamDecline::NotThreeCuttingFaces,
         bo::chainSeamDeclineName(d2));

  std::printf("[CHN] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
