// SPDX-License-Identifier: Apache-2.0
//
// native_nurbs_curved_split_parity.mm — NURBS roadmap LAYER 3, SLICE 2 (L3-S2): the
// SECOND exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY A CURVED (analytic)
// FACE — SIM GATE (b), native-vs-OCCT on a booted iOS simulator.
//
// The native split (src/native/boolean/nurbs_curved_split.h, OCCT-FREE) composes
// NURBS-adapter ∩ sphere-adapter trace[stage 1] → WLine-(u,v)-read fidelity gate on BOTH
// F and G[stage 2] → splitFaceSmoothTrim[stage 3] → CURVED-solid membership keep[stage 4]
// → curved-G cap fan synth + M0 curved↔CURVED weld[stage 5] → watertight+volume self-
// verify, into ONE watertight keep-side solid from a NURBS-walled bowl (a genuine
// Kind::BSpline degree-2 paraboloid wall trimmed by a rim + flat top-lid) cut by a SPHERE.
//
// This harness proves, against the OCCT ORACLE:
//   The SAME operand is reconstructed in OCCT (a Geom_BSplineSurface degree-2 paraboloid
//   trimmed by the rim in (u,v) + a planar lid, sewn into a solid) and the SAME sphere is
//   a BRepPrimAPI_MakeSphere ball. Native KeepSide::Below (CUT, keep OUTSIDE the sphere =
//   the closed-form LENS) is the OCCT BRepAlgoAPI_Cut(cup, ball); native KeepSide::Above
//   (COMMON, keep INSIDE) is BRepAlgoAPI_Common(cup, ball). The native result (measured by
//   the native M0 tessellator) is compared to the OCCT boolean on:
//     * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel band);
//                    the CUT leg is ALSO cross-checked against the closed-form lens.
//     * WATERTIGHT — the native mesh is a closed 2-manifold; the OCCT result is one solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-nurbs-curved-split.sh.
//
#include "native/boolean/nurbs_curved_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/nurbs_curved_split_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_nurbs_curved_split_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
#include <gp_Ax2.hxx>
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
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace fx = nurbs_curved_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NCS] %-14s %-22s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the SAME NURBS bowl-cup operand ──────────────────────
// The bowl wall is a Geom_BSplineSurface (degree 2, clamped knots {0,1} mult {3,3},
// 3×3 poles — bit-equal to the native fixture's Kind::BSpline bowl), trimmed by the rim
// circle in (u,v); the top-lid is a planar face on the SAME rim. Sew into one solid.
static Handle(Geom_BSplineSurface) buildOcctBowlSurface() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();  // row-major, U outer, 3×3
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

static TopoDS_Shape buildOcctCup() {
  Handle(Geom_BSplineSurface) surf = buildOcctBowlSurface();
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

  const gp_Pln lidPln(gp_Pnt(0, 0, fx::kRimZ), gp_Dir(0, 0, 1));
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
  std::printf("== L3-S2 exact-NURBS face split by curved face: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double relTol = 0.04;  // curved-tessellation volume band (never widened)
  const fx::Operand op = fx::buildOperand();
  const nt::Shape sphere = fx::buildSphereCutter();

  const TopoDS_Shape occtCup = buildOcctCup();
  report("occt-cup", "solid-built", occtSolidCount(occtCup) == 1, "sewn 2-face NURBS cup");

  // The OCCT ball: BRepPrimAPI_MakeSphere at the SAME centre/radius as the native cutter.
  const TopoDS_Solid occtBall =
      BRepPrimAPI_MakeSphere(gp_Ax2(gp_Pnt(0, 0, fx::kZc), gp_Dir(0, 0, 1)), fx::kRs).Solid();

  // Each keep side: OCCT boolean vs native nurbsFaceCurvedSplit.
  auto leg = [&](const char* name, bo::KeepSide side, bool isCut, double vClosed) {
    // OCCT reference boolean.
    TopoDS_Shape occtRes;
    if (isCut) {
      BRepAlgoAPI_Cut cutter(occtCup, occtBall); cutter.Build();
      report(name, "occt-built", cutter.IsDone(), "cup - ball");
      occtRes = cutter.Shape();
    } else {
      BRepAlgoAPI_Common com(occtCup, occtBall); com.Build();
      report(name, "occt-built", com.IsDone(), "cup ∩ ball");
      occtRes = com.Shape();
    }
    const double vOcct = occtVolume(occtRes);
    if (isCut && vClosed > 0.0) {
      const double relO = std::fabs(vOcct - vClosed) / vClosed;
      char buf[96]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
      report(name, "occt-vs-closed", relO <= relTol, buf);
    }

    const double defl = 0.0005;
    const bo::NurbsCurvedSplitResult r =
        bo::nurbsFaceCurvedSplit(op.wall, op.base, sphere, side, defl);
    {
      char buf[96]; std::snprintf(buf, sizeof(buf), "decline=%s", bo::nurbsCurvedSplitDeclineName(r.decline));
      report(name, "native-composed", r.ok(), buf);
    }
    if (!r.ok()) return;
    report(name, "native-fidelity",
           r.seamFidelityF < 1e-6 && r.seamFidelityG < 1e-6 && r.seamOnSurf < 1e-6,
           "seam on BOTH curved surfaces");

    ntess::MeshParams mp; mp.deflection = defl;
    const ntess::Mesh m = ntess::SolidMesher(mp).mesh(r.solid);
    report(name, "native-watertight", ntess::isWatertight(m), "closed 2-manifold");
    { const long chi = eulerChar(m);
      char buf[48]; std::snprintf(buf, sizeof(buf), "euler=%ld (2)", chi);
      report(name, "native-topology", chi == 2, buf); }
    { const double vN = r.enclosedVolume;
      const double relN = std::fabs(vN - vOcct) / std::max(vOcct, 1e-12);
      char buf[96]; std::snprintf(buf, sizeof(buf), "native=%.6f occt=%.6f rel=%.3e", vN, vOcct, relN);
      report(name, "native-vs-occt-vol", relN <= relTol, buf); }
  };

  leg("CUT-LENS", bo::KeepSide::Below, true, fx::lensVolume());
  leg("COMMON", bo::KeepSide::Above, false, -1.0);

  std::printf("\n== L3-S2 parity: %d PASS, %d FAIL ==\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail == 0 ? 0 : 1;
}
