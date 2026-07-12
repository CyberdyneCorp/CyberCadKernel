// SPDX-License-Identifier: Apache-2.0
//
// native_nurbs_freeform_split_parity.mm — NURBS roadmap LAYER 3, SLICE 3 (L3-S3): the
// THIRD (deepest) exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY ANOTHER
// FREEFORM NURBS face (BOTH operands arbitrary NURBS) — SIM GATE (b), native-vs-OCCT on
// a booted iOS simulator.
//
// The native split (src/native/boolean/nurbs_freeform_split.h, OCCT-FREE) composes
// NURBS-adapter ∩ NURBS-adapter trace[stage 1] → WLine-(u,v)-read fidelity gate on BOTH
// F and G[stage 2] → splitFaceSmoothTrim on BOTH walls[stage 3] → mesh-membership keep
// [stage 4] → curved-NURBS↔curved-NURBS orientation-coherent weld[stage 5] →
// watertight+volume self-verify, into ONE watertight COMMON (F ∩ G) lens solid from two
// genuine-NURBS-walled bowl-cups (each a Kind::BSpline degree-2 paraboloid wall trimmed
// by a rim + flat lid) meeting in ONE CLOSED CIRCULAR seam.
//
// This harness proves, against the OCCT ORACLE:
//   The SAME two operands are reconstructed in OCCT (each a Geom_BSplineSurface degree-2
//   paraboloid trimmed by the rim in (u,v) + a planar lid, sewn into a solid — bit-equal
//   to the native fixtures' Kind::BSpline walls). Native FfOp::Common (keep INSIDE BOTH =
//   the closed-form LENS) is the OCCT BRepAlgoAPI_Common(F, G). The native result (measured
//   by the native M0 tessellator) is compared to the OCCT boolean on:
//     * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel band); the
//                    COMMON leg is ALSO cross-checked against the closed-form lens π·H²/(4a).
//     * WATERTIGHT — the native mesh is a closed 2-manifold; the OCCT result is one solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-nurbs-freeform-split.sh.
//
#include "native/boolean/nurbs_freeform_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/nurbs_freeform_split_fixture.h"

#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_nurbs_freeform_split_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace fx = nurbs_freeform_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NFS] %-14s %-22s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct a NURBS bowl-cup operand ──────────────────────────────
// The wall is a Geom_BSplineSurface (degree 2, clamped knots {0,1} mult {3,3}, 3×3 poles
// — bit-equal to the native fixture's Kind::BSpline wall), trimmed by the rim circle in
// (u,v); the lid is a planar face on the SAME rim. Sew into one solid.
static Handle(Geom_BSplineSurface) buildOcctWall(const std::vector<nm::Point3>& poles) {
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  TColStd_Array1OfReal uk(1, 2); uk.SetValue(1, 0.0); uk.SetValue(2, 1.0);
  TColStd_Array1OfReal vk(1, 2); vk.SetValue(1, 0.0); vk.SetValue(2, 1.0);
  TColStd_Array1OfInteger um(1, 2); um.SetValue(1, 3); um.SetValue(2, 3);
  TColStd_Array1OfInteger vm(1, 2); vm.SetValue(1, 3); vm.SetValue(2, 3);
  return new Geom_BSplineSurface(arr, uk, vk, um, vm, 2, 2);
}

static TopoDS_Shape buildOcctCup(const std::vector<nm::Point3>& poles, double lidZ) {
  Handle(Geom_BSplineSurface) surf = buildOcctWall(poles);
  const std::vector<nm::Point3> uv = fx::rimUV();
  const int n = static_cast<int>(uv.size());

  BRepBuilderAPI_MakeWire bowlWire;
  for (int k = 0; k < n; ++k) {
    const int k1 = (k + 1) % n;
    const gp_Pnt2d p0(uv[k].x, uv[k].y), p1(uv[k1].x, uv[k1].y);
    const double len = p0.Distance(p1);
    gp_Dir2d dir(p1.X() - p0.X(), p1.Y() - p0.Y());
    Handle(Geom2d_Line) line = new Geom2d_Line(p0, dir);
    Handle(Geom2d_TrimmedCurve) seg = new Geom2d_TrimmedCurve(line, 0.0, len);
    bowlWire.Add(BRepBuilderAPI_MakeEdge(seg, surf, 0.0, len).Edge());
  }
  TopoDS_Face bowlFace = BRepBuilderAPI_MakeFace(surf, bowlWire.Wire(), Standard_True).Face();
  BRepLib::BuildCurves3d(bowlFace);

  BRepBuilderAPI_MakeWire lidWire;
  for (TopExp_Explorer ex(bowlFace, TopAbs_EDGE); ex.More(); ex.Next())
    lidWire.Add(TopoDS::Edge(ex.Current().Reversed()));

  const gp_Pln lidPln(gp_Pnt(0, 0, lidZ), gp_Dir(0, 0, 1));
  TopoDS_Face lidFace = BRepBuilderAPI_MakeFace(lidPln, lidWire.Wire(), Standard_True).Face();
  BRepLib::BuildCurves3d(lidFace);

  BRepBuilderAPI_Sewing sew(1e-4);
  sew.Add(bowlFace);
  sew.Add(lidFace);
  sew.Perform();
  TopoDS_Shape sewn = sew.SewedShape();
  BRepBuilderAPI_MakeSolid mk;
  for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next())
    mk.Add(TopoDS::Shell(ex.Current()));
  TopoDS_Solid solid = mk.Solid();
  GProp_GProps g; BRepGProp::VolumeProperties(solid, g);
  if (g.Mass() < 0.0) solid.Reverse();
  return solid;
}

static double occtVolume(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
static int occtSolidCount(const TopoDS_Shape& s) {
  int n = 0;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++n;
  return n;
}
static long eulerChar(const ntess::Mesh& m) {
  return static_cast<long>(m.vertices.size()) - static_cast<long>(ntess::edgeUseCounts(m).size()) +
         static_cast<long>(m.triangles.size());
}

int main() {
  std::printf("== L3-S3 exact-NURBS face split by FREEFORM NURBS face: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double relTol = 0.05;  // curved-tessellation volume band (never widened)
  const nt::Shape F = fx::buildF();
  const nt::Shape G = fx::buildG();

  // The OCCT cups: the SAME NURBS bowl (F, lid at z=a·R²) and dome (G, lid at z=H−a·R²).
  const TopoDS_Shape occtF = buildOcctCup(fx::upBowlPoles(), fx::kA * fx::kR * fx::kR);
  const TopoDS_Shape occtG = buildOcctCup(fx::downDomePoles(), fx::kH - fx::kA * fx::kR * fx::kR);
  report("occt-cups", "solids-built", occtSolidCount(occtF) == 1 && occtSolidCount(occtG) == 1,
         "two sewn 2-face NURBS cups");

  // OCCT reference COMMON = F ∩ G (the lens).
  BRepAlgoAPI_Common com(occtF, occtG); com.Build();
  report("COMMON", "occt-built", com.IsDone(), "F ∩ G");
  const TopoDS_Shape occtRes = com.Shape();
  const double vOcct = occtVolume(occtRes);
  const double vClosed = fx::volCommon();
  {
    const double relO = std::fabs(vOcct - vClosed) / vClosed;
    char buf[96]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
    report("COMMON", "occt-vs-closed", relO <= relTol, buf);
  }

  // Native COMMON via nurbsFaceFreeformSplit (the closed form passed for the two-sided gate).
  const double defl = 0.00125;
  const bo::NurbsFreeformSplitResult r =
      bo::nurbsFaceFreeformSplit(F, G, bo::FfOp::Common, defl, vClosed);
  {
    char buf[96]; std::snprintf(buf, sizeof(buf), "decline=%s", bo::nurbsFreeformSplitDeclineName(r.decline));
    report("COMMON", "native-composed", r.ok(), buf);
  }
  if (!r.ok()) {
    std::printf("\n== L3-S3 parity: %d PASS, %d FAIL ==\n", g_pass, g_fail);
    return 1;
  }
  report("COMMON", "native-fidelity",
         r.seamFidelityF < 1e-6 && r.seamFidelityG < 1e-6 && r.seamOnSurf < 1e-6,
         "seam on BOTH NURBS walls");

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(r.solid);
  report("COMMON", "native-watertight", ntess::isWatertight(m), "closed 2-manifold");
  report("COMMON", "native-oriented", ntess::isConsistentlyOriented(m), "coherent winding");
  { const long chi = eulerChar(m);
    char buf[48]; std::snprintf(buf, sizeof(buf), "euler=%ld (2)", chi);
    report("COMMON", "native-topology", chi == 2, buf); }
  { const double vN = r.enclosedVolume;
    const double relN = std::fabs(vN - vOcct) / std::max(vOcct, 1e-12);
    char buf[96]; std::snprintf(buf, sizeof(buf), "native=%.6f occt=%.6f rel=%.3e", vN, vOcct, relN);
    report("COMMON", "native-vs-occt-vol", relN <= relTol, buf); }

  std::printf("\n== L3-S3 parity: %d PASS, %d FAIL ==\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail == 0 ? 0 : 1;
}
