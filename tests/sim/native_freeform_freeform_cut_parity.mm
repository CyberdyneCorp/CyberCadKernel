// SPDX-License-Identifier: Apache-2.0
//
// native_freeform_freeform_cut_parity.mm — MOAT M2 freeform↔freeform CLOSED-SEAM
// CUT / COMMON SIM GATE (b): native-vs-OCCT on a booted iOS simulator.
//
// The native verb `freeformFreeformClosedSeamCut` (src/native/boolean/freeform_freeform_cut.h,
// OCCT-FREE) composes recognise[B1] → trace[M1] → split[B2 smooth-trim] → classify[B3] →
// weld[M0] for TWO curved operands over a shared CLOSED curved seam, and HONEST-DECLINES to
// NULL when the two-CURVED-side closed-seam weld (a byte-frozen M0 tessellator gate) cannot
// close. This harness GROUNDS the enabler + the honest fallthrough against OCCT:
//   * the native shared seam `A.wall ∩ B.wall` (the real S3 trace) lies ON BOTH OCCT
//     Bézier surfaces (A's UP bowl AND B's DOWN dome), BRepExtrema ≤ tol — the seam is
//     the genuine curved↔curved intersection, keyed correctly on both operands;
//   * OCCT's `BRepAlgoAPI_Cut(A,B)` and `Common(A,B)` (the ORACLE) yield the closed-form
//     volumes V(A−B) = V(A) − π·H²/(4a) and V(A∩B) = π·H²/(4a) — so the closed-form host
//     oracle is the CORRECT answer OCCT owns;
//   * the native verb DECLINES (returns NULL) for CUT and COMMON — the honest fallthrough
//     to OCCT is correct (the native path NEVER emits a leaky/partial/wrong solid), which
//     is the disciplined behaviour while the two-curved-side seam weld is tessellator-gated;
//   * a non-intersecting second operand → the native trace declines SeamUnusable and OCCT's
//     Cut is a no-op (V unchanged) — no fabrication either way.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-freeform-freeform-cut.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_freeform_freeform_cut.cpp.
//
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/freeform_freeform_cut_fixture.h"

#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_freeform_freeform_cut_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepAlgoAPI_Cut.hxx>
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
namespace ffx = freeform_freeform_cut_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[FF] %-8s %-26s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}
static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// The bowl/dome Bézier surface (3×3 poles) as a FULL [0,1]² face (for seam membership).
static TopoDS_Face bezierFull(const std::vector<nm::Point3>& poles) {
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);
  return BRepBuilderAPI_MakeFace(surf, 1e-7).Face();
}

// The bowl/dome Bézier surface trimmed by the rim circle (in (u,v)), closed by a planar
// lid at z = lidZ, sewn into a solid — the SAME bowl-cup the native fixture builds.
static TopoDS_Shape bezierCupSolid(const std::vector<nm::Point3>& poles, double lidZ,
                                   const nm::Point3& interior) {
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);

  const std::vector<nm::Point3> uv = ffx::rimUV();
  const int n = static_cast<int>(uv.size());
  // bowl face trimmed by the rim wire in (u,v)
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
  const TopoDS_Wire bowlWire = mkWire.Wire();
  TopoDS_Face bowlFace = BRepBuilderAPI_MakeFace(surf, bowlWire, Standard_True).Face();
  BRepLib::BuildCurves3d(bowlFace);

  // lid: a PLANAR face bounded by the bowl face's OWN 3-D rim edges (the rim is planar at
  // z = lidZ — the separable quadratic bowl gives z = a·R² constant on the r=R UV circle),
  // so the two faces SHARE the identical boundary curve and the sewn shell is exactly
  // closed (no straight-segment-vs-curve gap). BuildCurves3d above put a 3-D pcurve on
  // each bowl edge; rebuild the lid wire from those same edges.
  BRepBuilderAPI_MakeWire lidWireMk;
  for (TopExp_Explorer ex(bowlWire, TopAbs_EDGE); ex.More(); ex.Next())
    lidWireMk.Add(TopoDS::Edge(ex.Current()));
  TopoDS_Face lidFace = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, lidZ), gp_Dir(0, 0, 1)),
                                                lidWireMk.Wire(), Standard_True).Face();

  BRepBuilderAPI_Sewing sew(1e-6);
  sew.Add(bowlFace);
  sew.Add(lidFace);
  sew.Perform();
  TopoDS_Shape sewn = sew.SewedShape();
  TopoDS_Shell shell;
  for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next()) { shell = TopoDS::Shell(ex.Current()); break; }
  if (shell.IsNull()) return sewn;
  TopoDS_Solid solid = BRepBuilderAPI_MakeSolid(shell).Solid();
  // Orient the solid so a KNOWN deep-interior point classifies IN — a sewn shell can come
  // back with inward face normals, giving the COMPLEMENT solid (interior=outside) that the
  // boolean cannot intersect. `interior` is a point strictly inside the cup at r≈0.
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
  std::printf("== MOAT M2 freeform<->freeform closed-seam CUT/COMMON: native-vs-OCCT ==\n");
  std::fflush(stdout);
  const double tol = 1e-4;   // curved-surface membership tolerance (never widened)
  const double vrel = 2e-2;  // OCCT-vs-closed-form volume band (curved-cup faceting)

  // ── OCCT oracle solids: the SAME two bowl-cups the native fixture builds ─────────
  const TopoDS_Shape occtA = bezierCupSolid(ffx::upBowlPoles(), ffx::kA * ffx::kR * ffx::kR,
                                            nm::Point3{0, 0, 0.20});   // deep inside A (z∈[0,0.245])
  const TopoDS_Shape occtB = bezierCupSolid(ffx::downDomePoles(), ffx::kH - ffx::kA * ffx::kR * ffx::kR,
                                            nm::Point3{0, 0, 0.00});   // deep inside B (z∈[−0.085,0.16])
  report("occt", "cups-built", !occtA.IsNull() && !occtB.IsNull(), "sewn Bezier bowl + lid");

  const double vA = volumeOf(occtA), vB = volumeOf(occtB);
  {
    char buf[96]; std::snprintf(buf, sizeof buf, "V(A)=%.5f cf=%.5f", vA, ffx::volA());
    report("occt", "VA-matches-closed-form", std::fabs(vA - ffx::volA()) / ffx::volA() < vrel, buf);
    std::snprintf(buf, sizeof buf, "V(B)=%.5f cf=%.5f", vB, ffx::volA());
    report("occt", "VB-matches-closed-form", std::fabs(vB - ffx::volA()) / ffx::volA() < vrel, buf);
  }

  // ── (1) the native shared seam lies on BOTH OCCT Bezier surfaces ─────────────────
  const TopoDS_Face bowlA = bezierFull(ffx::upBowlPoles());
  const TopoDS_Face domeB = bezierFull(ffx::downDomePoles());
  const ssi::WLine seam = ffx::closedSeamWLine();
  double maxOnA = 0, maxOnB = 0;
  for (const auto& p : seam.points) {
    maxOnA = std::max(maxOnA, distToShape(p.point, bowlA));
    maxOnB = std::max(maxOnB, distToShape(p.point, domeB));
  }
  char buf[128];
  std::snprintf(buf, sizeof buf, "nodes=%zu maxOnA=%.2e maxOnB=%.2e", seam.points.size(), maxOnA, maxOnB);
  report("occt", "seam-on-both-surfaces",
         seam.status == ssi::TraceStatus::Closed && maxOnA <= tol && maxOnB <= tol, buf);

  // membership probe: (0,0,0.05) is strictly inside BOTH (A: 0≤0.05≤0.245; B: −0.085≤0.05≤0.155).
  {
    BRepClass3d_SolidClassifier ca(occtA); ca.Perform(gp_Pnt(0,0,0.05), 1e-7);
    BRepClass3d_SolidClassifier cb(occtB); cb.Perform(gp_Pnt(0,0,0.05), 1e-7);
    std::snprintf(buf, sizeof buf, "(0,0,0.05) inA=%d inB=%d", ca.State()==TopAbs_IN, cb.State()==TopAbs_IN);
    report("occt", "lens-point-in-both", ca.State()==TopAbs_IN && cb.State()==TopAbs_IN, buf);
  }

  // ── (2) OCCT Cut/Common give the closed-form volumes (the ORACLE answer) ─────────
  const TopoDS_Shape occtCut = BRepAlgoAPI_Cut(occtA, occtB).Shape();
  const TopoDS_Shape occtCom = BRepAlgoAPI_Common(occtA, occtB).Shape();
  const double vCut = volumeOf(occtCut), vCom = volumeOf(occtCom);
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCut, ffx::volCut());
  report("occt", "cut-matches-closed-form", std::fabs(vCut - ffx::volCut()) / ffx::volCut() < vrel, buf);
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCom, ffx::volCommon());
  report("occt", "common-matches-closed-form", std::fabs(vCom - ffx::volCommon()) / ffx::volCommon() < vrel, buf);

  // ── (3) the native verb HONEST-DECLINES CUT and COMMON (correct fallthrough) ─────
  // The two-CURVED-side closed-seam weld is gated on the byte-frozen M0 tessellator, so
  // the native path returns NULL → OCCT (NEVER a leaky/partial/wrong solid). This asserts
  // the disciplined fallthrough is what actually happens.
  const nt::Shape A = ffx::buildA();
  const nt::Shape B = ffx::buildB();
  for (double d : {0.01, 0.005}) {
    bo::FfCutDecline wc = bo::FfCutDecline::Ok, wm = bo::FfCutDecline::Ok;
    const nt::Shape ncut = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Cut, d, &wc);
    const nt::Shape ncom = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Common, d, &wm);
    std::snprintf(buf, sizeof buf, "d=%.3f CUT null=%d(%s)", d, ncut.isNull(), bo::ffCutDeclineName(wc));
    report("native", "cut-declines-to-null", ncut.isNull() && wc != bo::FfCutDecline::Ok, buf);
    std::snprintf(buf, sizeof buf, "d=%.3f COMMON null=%d(%s)", d, ncom.isNull(), bo::ffCutDeclineName(wm));
    report("native", "common-declines-to-null", ncom.isNull() && wm != bo::FfCutDecline::Ok, buf);
  }

  // ── (4) fallback contract: a non-intersecting operand → native SeamUnusable, OCCT no-op
  auto polesUp = ffx::upBowlPoles();
  for (auto& p : polesUp) p.z += 100.0;
  const nt::Shape Bfar = ffx::buildCup(polesUp, ffx::kA * ffx::kR * ffx::kR + 100.0);
  bo::FfCutDecline wf = bo::FfCutDecline::Ok;
  const nt::Shape nf = bo::freeformFreeformClosedSeamCut(A, Bfar, bo::FfOp::Cut, 0.005, &wf);
  const TopoDS_Shape occtBfar = bezierCupSolid(polesUp, ffx::kA * ffx::kR * ffx::kR + 100.0,
                                               nm::Point3{0, 0, 100.20});
  const double vNoOp = volumeOf(BRepAlgoAPI_Cut(occtA, occtBfar).Shape());
  std::snprintf(buf, sizeof buf, "native %s, OCCT V=%.5f (=VA)", bo::ffCutDeclineName(wf), vNoOp);
  report("both", "non-intersecting-clean",
         nf.isNull() && wf == bo::FfCutDecline::SeamUnusable &&
             std::fabs(vNoOp - vA) / vA < 1e-3, buf);

  std::printf("[FF] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
