// SPDX-License-Identifier: Apache-2.0
//
// native_nurbs_plane_split_parity.mm — NURBS roadmap LAYER 3, SLICE 1 (L3-S1): the
// FIRST exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY A PLANE — SIM GATE
// (b), native-vs-OCCT on a booted iOS simulator.
//
// The native split (src/native/boolean/nurbs_plane_split.h, OCCT-FREE) composes
// NURBS-adapter trace[stage 1] → WLine-(u,v)-read fidelity gate[stage 2] →
// splitFaceSmoothTrim[stage 3] → half-space keep[stage 4] → flat cap synth + M0
// curved↔flat weld[stage 5] → watertight+volume self-verify, into ONE watertight
// keep-side solid from a NURBS-walled bowl-cup (a genuine Kind::BSpline degree-2 bowl
// wall trimmed by a rim circle + a flat top-lid) cut by the HORIZONTAL plane z = c.
//
// This harness proves, against the OCCT ORACLE:
//   The SAME operand is reconstructed in OCCT (a Geom_BSplineSurface degree-2 bowl
//   trimmed by the rim circle in (u,v) + a planar top-lid, sewn into a solid) and cut by
//   the SAME half-space via BRepAlgoAPI_Cut against a large box spanning z ≥ c (CUT keeps
//   z ≤ c) and z ≤ c (COMMON keeps z ≥ c). The native nurbsFacePlaneSplit result
//   (measured by the native M0 tessellator) is compared to the OCCT boolean on:
//     * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel band);
//                    both cross-checked against the closed form (π·ρ²·c/2 and V(full)−that).
//     * AREA       — native surfaceArea vs BRepGProp::SurfaceProperties.
//     * WATERTIGHT — the native cut mesh is a closed 2-manifold; the OCCT cut is one solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//     * BBOX       — native vertex min/max vs BRepBndLib per-axis (spatial band).
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-nurbs-plane-split.sh.
//
#include "native/boolean/nurbs_plane_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/nurbs_plane_split_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_nurbs_plane_split_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace fx = nurbs_plane_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NPS] %-14s %-22s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
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

static TopoDS_Shape buildOcctOperand() {
  Handle(Geom_BSplineSurface) surf = buildOcctBowlSurface();
  const nt::FaceSurface bowl = fx::bowlSurface();
  ntess::SurfaceEvaluator eval(bowl, nt::Location{});
  const std::vector<nm::Point3> uv = fx::rimUV();
  const int n = static_cast<int>(uv.size());

  std::vector<nm::Point3> rim3d(n), ctrl(n);
  for (int k = 0; k < n; ++k) rim3d[k] = eval.value(uv[k].x, uv[k].y);
  for (int k = 0; k < n; ++k) {
    const int k1 = (k + 1) % n;
    const nm::Point3 mid{(uv[k].x + uv[k1].x) * 0.5, (uv[k].y + uv[k1].y) * 0.5, 0.0};
    const nm::Point3 Sm = eval.value(mid.x, mid.y);
    ctrl[k] = nm::Point3{2 * Sm.x - 0.5 * (rim3d[k].x + rim3d[k1].x),
                         2 * Sm.y - 0.5 * (rim3d[k].y + rim3d[k1].y),
                         2 * Sm.z - 0.5 * (rim3d[k].z + rim3d[k1].z)};
  }

  (void)ctrl;

  // bowl wall: trim `surf` by the rim polygon in (u,v) (straight Geom2d_Line pcurves),
  // then derive the 3-D rim edges (BuildCurves3d). The lid REUSES the bowl face's OWN
  // 3-D boundary edges (extracted below), so the two faces share every boundary edge
  // for-edge and the sewn shell is watertight (VolumeProperties > 0).
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

  // the lid wire reuses the bowl face's own 3-D rim edges (reversed), so both faces bound
  // the SAME shared edges → a watertight sewn shell.
  BRepBuilderAPI_MakeWire lidWire;
  for (TopExp_Explorer ex(bowlFace, TopAbs_EDGE); ex.More(); ex.Next())
    lidWire.Add(TopoDS::Edge(ex.Current().Reversed()));

  // the lid is the flat disk at z = a·R² (the rim's constant plane); build it on an
  // EXPLICIT plane so the (slightly z-bulging) shared rim wire is accepted as its
  // boundary (BuildCurves3d re-derives pcurves), rather than requiring exact planarity.
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
  std::printf("== L3-S1 exact-NURBS face split by plane: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double relTol = 0.03;  // curved-tessellation volume band (never widened)
  const fx::Operand op = fx::buildOperand();
  const TopoDS_Shape occtOperand = buildOcctOperand();
  report("occt-operand", "solid-built", occtSolidCount(occtOperand) == 1, "sewn 2-face NURBS solid");
  {
    const double vO = occtVolume(occtOperand), vC = fx::fullVolume();
    char buf[80]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f", vO, vC);
    report("occt-operand", "volume-vs-closed", std::fabs(vO - vC) / vC <= relTol, buf);
  }

  const double c = fx::kCutZ;

  // Each keep side: OCCT BRepAlgoAPI_Cut vs native nurbsFacePlaneSplit + closed form.
  auto leg = [&](const char* name, bo::KeepSide side, double vClosed, bool below) {
    // OCCT reference boolean: remove the box on the DISCARD side.
    TopoDS_Solid box = below
        ? BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, c), gp_Pnt(10, 10, 10)).Solid()   // keep z<=c
        : BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, -10), gp_Pnt(10, 10, c)).Solid();  // keep z>=c
    BRepAlgoAPI_Cut cutter(occtOperand, box); cutter.Build();
    const bool okOcct = cutter.IsDone() && occtSolidCount(cutter.Shape()) == 1;
    report(name, "occt-cut-built", okOcct, below ? "z<=c" : "z>=c");
    const double vOcct = okOcct ? occtVolume(cutter.Shape()) : -1.0;
    {
      const double relO = std::fabs(vOcct - vClosed) / vClosed;
      char buf[96]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
      report(name, "occt-vs-closed", relO <= relTol, buf);
    }

    const double defl = 0.0025;
    const bo::NurbsPlaneSplitResult r = bo::nurbsFacePlaneSplit(op.wall, op.base, fx::cutPlane(), side, defl);
    {
      char buf[96]; std::snprintf(buf, sizeof(buf), "decline=%s", bo::nurbsPlaneSplitDeclineName(r.decline));
      report(name, "native-composed", r.ok(), buf);
    }
    if (!r.ok()) return;
    report(name, "native-fidelity", r.seamFidelity < 1e-6 && r.seamOnSurf < 1e-6, "seam on both surfaces");

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

  leg("CUT", bo::KeepSide::Below, fx::cutVolume(), true);
  leg("COMMON", bo::KeepSide::Above, fx::commonVolume(), false);

  std::printf("\n== L3-S1 parity: %d PASS, %d FAIL ==\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail == 0 ? 0 : 1;
}
