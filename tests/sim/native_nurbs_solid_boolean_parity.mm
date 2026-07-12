// SPDX-License-Identifier: Apache-2.0
//
// native_nurbs_solid_boolean_parity.mm — BOOL-INT / LAYER 3 SIM GATE (b): the general
// two-freeform-solid NURBS boolean ORCHESTRATOR native-vs-OCCT on a booted iOS simulator.
//
// The native orchestrator `nurbsSolidBoolean(A, B, op)` (src/native/boolean/nurbs_solid_boolean.h,
// OCCT-FREE) composes the five landed L3 stage verbs (SSI → pcurve → split+heal →
// membership → sew) into ONE watertight result for op ∈ {Fuse, Cut, Common} over two
// freeform NURBS solids, or an honest decline to NULL. This harness GROUNDS all three ops
// against OCCT on the canonical single-transversal-seam bowl-cup:
//   * OCCT's BRepAlgoAPI_{Common,Cut,Fuse}(A,B) (the ORACLE) yield the closed-form volumes
//     V(A∩B)=π·H²/(4a), V(A−B)=V(A)−that, V(A∪B)=V(A)+V(B)−that;
//   * native `nurbsSolidBoolean` WELDS all three watertight (χ=2, coherent) with each
//     meshed volume matching the corresponding OCCT op within the deflection band and
//     CONVERGING (DISAGREED=0) — the native path NEVER emits a leaky/partial/wrong solid;
//   * the op-algebra V(fuse)+V(common)=V(A)+V(B) holds on OCCT's own volumes.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-nurbs-solid-boolean.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_nurbs_solid_boolean.cpp.
//
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/freeform_freeform_cut_fixture.h"

#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_nurbs_solid_boolean_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
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
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepLib.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;
namespace bo = cybercad::native::boolean;
namespace ntess = cybercad::native::tessellate;
namespace ffx = freeform_freeform_cut_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NSB] %-8s %-28s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}
static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// The bowl/dome Bézier bowl-cup, sewn into a solid — the SAME cup the native fixture builds.
static TopoDS_Shape bezierCupSolid(const std::vector<nm::Point3>& poles, double lidZ,
                                   const nm::Point3& interior) {
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
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
  const TopoDS_Wire bowlWire = mkWire.Wire();
  TopoDS_Face bowlFace = BRepBuilderAPI_MakeFace(surf, bowlWire, Standard_True).Face();
  BRepLib::BuildCurves3d(bowlFace);

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

// Mesh a native result solid, require watertight+coherent, return its volume (or 1e30).
static double nativeVol(const nt::Shape& s, double d, bool& watertight) {
  watertight = false;
  if (s.isNull()) return 1e30;
  ntess::MeshParams mp; mp.deflection = d;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(s);
  watertight = ntess::isWatertight(m) && ntess::isConsistentlyOriented(m);
  return std::fabs(ntess::enclosedVolume(m));
}

int main() {
  std::printf("== BOOL-INT L3 two-freeform-solid NURBS boolean orchestrator: native-vs-OCCT ==\n");
  std::fflush(stdout);
  const double vrel = 2e-2;

  // ── OCCT oracle solids: the SAME two bowl-cups the native fixture builds ──
  const TopoDS_Shape occtA = bezierCupSolid(ffx::upBowlPoles(), ffx::kA * ffx::kR * ffx::kR,
                                            nm::Point3{0, 0, 0.20});
  const TopoDS_Shape occtB = bezierCupSolid(ffx::downDomePoles(), ffx::kH - ffx::kA * ffx::kR * ffx::kR,
                                            nm::Point3{0, 0, 0.00});
  report("occt", "cups-built", !occtA.IsNull() && !occtB.IsNull(), "sewn Bezier bowl + lid");
  const double vA = volumeOf(occtA), vB = volumeOf(occtB);

  // ── OCCT Common/Cut/Fuse (the ORACLE) vs the closed form ──
  const TopoDS_Shape occtCom = BRepAlgoAPI_Common(occtA, occtB).Shape();
  const TopoDS_Shape occtCut = BRepAlgoAPI_Cut(occtA, occtB).Shape();
  const TopoDS_Shape occtFuse = BRepAlgoAPI_Fuse(occtA, occtB).Shape();
  const double vCom = volumeOf(occtCom), vCut = volumeOf(occtCut), vFuse = volumeOf(occtFuse);
  char buf[160];
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCom, ffx::volCommon());
  report("occt", "common-matches-closed-form", std::fabs(vCom - ffx::volCommon()) / ffx::volCommon() < vrel, buf);
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCut, ffx::volCut());
  report("occt", "cut-matches-closed-form", std::fabs(vCut - ffx::volCut()) / ffx::volCut() < vrel, buf);
  const double cfFuse = vA + vB - ffx::volCommon();
  std::snprintf(buf, sizeof buf, "V=%.6f cf~%.6f", vFuse, cfFuse);
  report("occt", "fuse-matches-closed-form", std::fabs(vFuse - cfFuse) / cfFuse < vrel, buf);
  // op-algebra on OCCT's own volumes: |A∪B|+|A∩B| = |A|+|B|.
  std::snprintf(buf, sizeof buf, "fuse+common=%.6f  A+B=%.6f", vFuse + vCom, vA + vB);
  report("occt", "op-algebra-incl-excl", std::fabs((vFuse + vCom) - (vA + vB)) / (vA + vB) < vrel, buf);

  // ── native `nurbsSolidBoolean` welds all three vs OCCT (DISAGREED=0, converging) ──
  const nt::Shape A = ffx::buildA();
  const nt::Shape B = ffx::buildB();
  double prevCom = 1.0, prevCut = 1.0, prevFuse = 1.0;
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::SolidBoolReport rc, ru, rf;
    const nt::Shape ncom = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Common, d, &rc, ffx::volCommon());
    const nt::Shape ncut = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Cut, d, &ru, ffx::volCut());
    const nt::Shape nfuse = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &rf, cfFuse);

    bool wtC, wtU, wtF;
    const double vc = nativeVol(ncom, d, wtC);
    const double vu = nativeVol(ncut, d, wtU);
    const double vf = nativeVol(nfuse, d, wtF);
    const double eC = std::fabs(vc - vCom) / vCom;
    const double eU = std::fabs(vu - vCut) / vCut;
    const double eF = std::fabs(vf - vFuse) / vFuse;

    std::snprintf(buf, sizeof buf, "d=%.4f native=%.6f OCCT=%.6f relErr=%.3f wt=%d", d, vc, vCom, eC, wtC);
    report("native", "common-welds-vs-occt", !ncom.isNull() && wtC && eC < 30.0 * d && eC < prevCom, buf);
    prevCom = eC;
    std::snprintf(buf, sizeof buf, "d=%.4f native=%.6f OCCT=%.6f relErr=%.3f wt=%d", d, vu, vCut, eU, wtU);
    report("native", "cut-welds-vs-occt", !ncut.isNull() && wtU && eU < 30.0 * d && eU < prevCut, buf);
    prevCut = eU;
    std::snprintf(buf, sizeof buf, "d=%.4f native=%.6f OCCT=%.6f relErr=%.3f wt=%d", d, vf, vFuse, eF, wtF);
    report("native", "fuse-welds-vs-occt", !nfuse.isNull() && wtF && eF < 30.0 * d && eF < prevFuse, buf);
    prevFuse = eF;
  }

  std::printf("[NSB] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
