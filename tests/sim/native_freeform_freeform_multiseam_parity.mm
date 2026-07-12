// SPDX-License-Identifier: Apache-2.0
//
// native_freeform_freeform_multiseam_parity.mm — L3-d STAGE 5 MULTI-SEAM
// freeform↔freeform CUT / COMMON SIM GATE (b): native-vs-OCCT on a booted iOS simulator.
//
// The native verb `freeformFreeformMultiSeamCut` (src/native/boolean/freeform_freeform_multiseam.h,
// OCCT-FREE) composes recognise[B1] → trace[M1] (≥2 closed loops) → split[multi-seam smooth
// trim] → classify[hole-respecting vote] → weld[M0] for TWO curved operands whose walls meet
// in MULTIPLE closed seams. It now WELDS the two-curved-side annulus↔annulus sew across BOTH
// seams (M0-WELD: the CDT hole-cull is a TOPOLOGICAL flood fill, so the two annuli triangulate
// the shared inner seam-as-hole strip IDENTICALLY and it welds 2-manifold). This harness
// GROUNDS the weld against OCCT:
//   * the native shared seams `A.wall ∩ B.wall` (the real S3 trace) are TWO closed loops, each
//     lying ON BOTH OCCT degree-4 Bézier surfaces (A's valley AND B's mirror dome),
//     BRepExtrema ≤ tol — the multi-seam intersection is the genuine curved↔curved one;
//   * OCCT's `BRepAlgoAPI_Common(A,B)` (the ORACLE) yields the closed-form annular-lens volume
//     V(A∩B) = π∫(z_B−z_A) over the ring between the two seams — the CORRECT answer OCCT owns;
//   * the native verb WELDS the annulus lens watertight (closed 2-manifold, boundaryEdges 0,
//     coherent) and its volume AGREES with OCCT `BRepAlgoAPI_Common` within the tessellation
//     band — DISAGREED=0 by AGREEMENT (native answers, OCCT confirms), never a leaky solid;
//   * the native MACHINERY tiles both walls exactly (2 seams, both walls split into 3 tiling
//     regions, lens survivors selected) before the shared-strip weld.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-freeform-freeform-multiseam.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_freeform_freeform_multiseam.cpp.
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include "../native/freeform_freeform_multiseam_fixture.h"

#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_freeform_freeform_multiseam_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libs"
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
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepLib.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;
namespace bo = cybercad::native::boolean;
namespace ssi = cybercad::native::ssi;
namespace ntess = cybercad::native::tessellate;
namespace ffx = freeform_freeform_multiseam_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[FFMS] %-8s %-30s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}
static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// The degree-4 valley/dome Bézier surface (5×5 poles) as a FULL [0,1]² face.
static TopoDS_Face bezierFull(const std::vector<nm::Point3>& poles) {
  TColgp_Array2OfPnt arr(1, 5, 1, 5);
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 5 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);
  return BRepBuilderAPI_MakeFace(surf, 1e-7).Face();
}

// The degree-4 cup (wall trimmed by the rim circle in (u,v), closed by a planar lid at
// z = lidZ, sewn into a solid) — the SAME cup the native fixture builds.
static TopoDS_Shape bezierCupSolid(const std::vector<nm::Point3>& poles, double lidZ,
                                   const nm::Point3& interior) {
  TColgp_Array2OfPnt arr(1, 5, 1, 5);
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 5 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);

  const std::vector<nm::Point3> uv = ffx::rimUV();
  const int n = static_cast<int>(uv.size());
  BRepBuilderAPI_MakeWire mkWire;
  for (int k = 0; k < n; ++k) {
    const nm::Point3 a = uv[k], b = uv[(k + 1) % n];
    const gp_Pnt2d p0(a.x, a.y), p1(b.x, b.y);
    const double len = p0.Distance(p1);
    gp_Dir2d dir(p1.X() - p0.X(), p1.Y() - p0.Y());
    Handle(Geom2d_Line) line = new Geom2d_Line(p0, dir);
    Handle(Geom2d_TrimmedCurve) seg = new Geom2d_TrimmedCurve(line, 0.0, len);
    mkWire.Add(BRepBuilderAPI_MakeEdge(seg, surf, 0.0, len).Edge());
  }
  const TopoDS_Wire wallWire = mkWire.Wire();
  TopoDS_Face wallFace = BRepBuilderAPI_MakeFace(surf, wallWire, Standard_True).Face();
  BRepLib::BuildCurves3d(wallFace);

  BRepBuilderAPI_MakeWire lidWireMk;
  for (TopExp_Explorer ex(wallWire, TopAbs_EDGE); ex.More(); ex.Next())
    lidWireMk.Add(TopoDS::Edge(ex.Current()));
  TopoDS_Face lidFace = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, lidZ), gp_Dir(0, 0, 1)),
                                                lidWireMk.Wire(), Standard_True).Face();

  BRepBuilderAPI_Sewing sew(1e-6);
  sew.Add(wallFace);
  sew.Add(lidFace);
  sew.Perform();
  TopoDS_Shape sewn = sew.SewedShape();
  TopoDS_Shell shell;
  for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) { shell = TopoDS::Shell(ex.Current()); break; }
  if (shell.IsNull()) return sewn;
  TopoDS_Solid solid = BRepBuilderAPI_MakeSolid(shell).Solid();
  BRepClass3d_SolidClassifier cl(solid);
  cl.Perform(gp_Pnt(interior.x, interior.y, interior.z), 1e-7);
  if (cl.State() != TopAbs_IN) solid.Reverse();
  return solid;
}

static double volumeOf(const TopoDS_Shape& s) {
  if (s.IsNull()) return 0.0;
  GProp_GProps props;
  BRepGProp::VolumeProperties(s, props);
  return std::fabs(props.Mass());
}
static double distToShape(const nm::Point3& p, const TopoDS_Shape& s) {
  TopoDS_Vertex v = BRepBuilderAPI_MakeVertex(P(p)).Vertex();
  BRepExtrema_DistShapeShape d(v, s);
  return d.IsDone() ? d.Value() : 1e30;
}

int main() {
  std::printf("== L3-d Stage 5 multi-seam freeform<->freeform CUT/COMMON: native-vs-OCCT ==\n");
  std::fflush(stdout);
  const double tol = 1e-4;   // curved-surface membership tolerance (never widened)
  const double vrel = 3e-2;  // OCCT-vs-closed-form volume band (curved-cup faceting)

  // ── OCCT oracle solids: the SAME two degree-4 mirror cups the native fixture builds ──
  const TopoDS_Shape occtA = bezierCupSolid(ffx::valleyPoles(), ffx::lidA(),
                                            nm::Point3{0, 0, ffx::lidA() - 0.01});
  const TopoDS_Shape occtB = bezierCupSolid(ffx::domePoles(), ffx::botB(),
                                            nm::Point3{0, 0, ffx::botB() + 0.01});
  report("occt", "cups-built", !occtA.IsNull() && !occtB.IsNull(), "sewn degree-4 wall + lid");

  const double vA = volumeOf(occtA), vB = volumeOf(occtB);
  {
    char buf[96];
    std::snprintf(buf, sizeof buf, "V(A)=%.5f cf=%.5f", vA, ffx::volA());
    report("occt", "VA-matches-closed-form", std::fabs(vA - ffx::volA()) / ffx::volA() < vrel, buf);
    std::snprintf(buf, sizeof buf, "V(B)=%.5f cf=%.5f", vB, ffx::volB());
    report("occt", "VB-matches-closed-form", std::fabs(vB - ffx::volB()) / ffx::volB() < vrel, buf);
  }

  // ── (1) the native trace returns TWO closed loops, each on BOTH OCCT surfaces ────────
  const TopoDS_Face wallA = bezierFull(ffx::valleyPoles());
  const TopoDS_Face wallB = bezierFull(ffx::domePoles());
  const std::vector<ssi::WLine> seams = ffx::closedSeams();
  char buf[160];
  bool twoOnBoth = seams.size() == 2;
  double maxOnA = 0, maxOnB = 0;
  for (const ssi::WLine& s : seams) {
    twoOnBoth = twoOnBoth && s.status == ssi::TraceStatus::Closed;
    for (const auto& p : s.points) {
      maxOnA = std::max(maxOnA, distToShape(p.point, wallA));
      maxOnB = std::max(maxOnB, distToShape(p.point, wallB));
    }
  }
  std::snprintf(buf, sizeof buf, "loops=%zu maxOnA=%.2e maxOnB=%.2e", seams.size(), maxOnA, maxOnB);
  report("occt", "two-seams-on-both-surfaces", twoOnBoth && maxOnA <= tol && maxOnB <= tol, buf);

  // ── (2) OCCT Common gives the closed-form annular-lens volume (the ORACLE answer) ────
  const TopoDS_Shape occtCom = BRepAlgoAPI_Common(occtA, occtB).Shape();
  const double vCom = volumeOf(occtCom);
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCom, ffx::volCommon());
  report("occt", "common-matches-closed-form", std::fabs(vCom - ffx::volCommon()) / ffx::volCommon() < vrel, buf);

  // ── (3) the native verb WELDS the annulus lens watertight AND agrees with OCCT ───────
  // The multi-seam annulus↔annulus sew now welds across BOTH seams (M0-WELD: the CDT
  // hole-cull is a TOPOLOGICAL flood fill, so the two annuli triangulate the shared inner
  // seam-as-hole strip IDENTICALLY and it welds 2-manifold). The native COMMON solid meshes
  // to a closed 2-manifold (boundaryEdges 0, χ correct, coherent) whose volume matches OCCT
  // `BRepAlgoAPI_Common` = the closed-form annular lens within the tessellation band —
  // DISAGREED=0 by AGREEMENT (native answers, OCCT confirms), not by abstention.
  const nt::Shape A = ffx::buildA();
  const nt::Shape B = ffx::buildB();
  bo::MultiSeamCutReport rep;
  const nt::Shape ncom = bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common,
                                                                   0.0025, &rep, ffx::volCommon());
  const bool weldedCleanly = !ncom.isNull() && rep.decline == bo::MultiSeamCutDecline::Ok &&
                             rep.watertight && rep.coherent && rep.boundaryEdges == 0;
  std::snprintf(buf, sizeof buf, "null=%d decline=%s seams=%d subA=%d subB=%d surv=%d be=%zu vol=%.6f",
                ncom.isNull(), bo::multiSeamCutDeclineName(rep.decline), rep.seamLoops,
                rep.subRegionsA, rep.subRegionsB, rep.survivorFaces, rep.boundaryEdges,
                rep.enclosedVolume);
  report("native", "welds-watertight-never-leaky",
         weldedCleanly && rep.seamLoops == 2 && rep.subRegionsA == 3 && rep.subRegionsB == 3 &&
             rep.survivorFaces >= 2,
         buf);

  // The native welded volume agrees with OCCT Common (= the closed-form lens) within band —
  // DISAGREED=0 by agreement.
  const double nrel = std::fabs(rep.enclosedVolume - vCom) / vCom;
  std::snprintf(buf, sizeof buf, "native=%.6f occt=%.6f rel=%.3f", rep.enclosedVolume, vCom, nrel);
  report("native", "common-vs-occt-agrees", !ncom.isNull() && nrel < 0.10, buf);

  // The machinery reached the weld: the split TILES both walls exactly (gap ≈ 0).
  std::snprintf(buf, sizeof buf, "tilingGapA=%.1e tilingGapB=%.1e", rep.tilingGapA, rep.tilingGapB);
  report("native", "multi-seam-split-tiles-exact", rep.tilingGapA < 1e-9 && rep.tilingGapB < 1e-9, buf);

  std::printf("[FFMS] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
