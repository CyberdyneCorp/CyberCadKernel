// SPDX-License-Identifier: Apache-2.0
//
// native_walled_bowl_midwall_parity.mm — MOAT M2 curved-wall freeform half-space CUT
// in the "walled bowl / dome cut MID-WALL" pose: SIM GATE (b), native-vs-OCCT on a
// booted iOS simulator.
//
// The native cut (src/native/boolean/curved_wall_cut.h, OCCT-FREE) composes the landed
// M2 substrate — B1 recognise → M1 seam trace → B2 SMOOTH-TRIM split (closed circular
// seam → disk + annulus-hole) → analytic WALL split (the mid-wall Kind::Split path) →
// flat ANNULAR cap synth (outer = wall-section chords, inner HOLE = the seam) → M0 weld
// → watertight+volume self-verify — into ONE watertight keep-side solid from a
// bowl-lidded convex-quad PRISM (a STEEP Bézier bowl over a convex quad + 4 PLANAR side
// walls + a flat base) cut by the HORIZONTAL plane z = c. Because the cut plane crosses
// the freeform bowl (closed interior circle) AND the 4 planar walls, the cross-section
// cap is an ANNULUS — distinct from the dome pose's simple disk cap
// (native_curved_wall_cut_parity.mm).
//
// This harness proves, against the OCCT ORACLE:
//   The SAME operand is reconstructed in OCCT (a Geom_BezierSurface bowl trimmed by the
//   convex quad + 4 planar walls with an exact degree-2 Bézier top edge + a planar base,
//   sewn into a solid) and cut by the SAME half-space via BRepAlgoAPI_Cut against a large
//   box spanning z ≥ c. The native curvedWallHalfSpaceCut result (measured by the native
//   M0 tessellator) is compared to the OCCT boolean on:
//     * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel band);
//                    both cross-checked against the closed form (H0+c)·A_Q − c·π·ρ²/2.
//     * AREA       — native surfaceArea vs BRepGProp::SurfaceProperties.
//     * WATERTIGHT — the native cut mesh is a closed 2-manifold; the OCCT cut is a single
//                    closed solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//     * BBOX       — native vertex min/max vs BRepBndLib per-axis (spatial band).
//     * HAUSDORFF  — max native→OCCT vertex distance via BRepExtrema (spatial band).
//
// The CUT (KeepSide::Below — the walled cup) is the landed keep-side: parity is asserted
// at MULTIPLE resonance-free deflections (the annular-cap weld is robust at each).
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-walled-bowl-midwall.sh.
//
#include "native/boolean/curved_wall_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/walled_bowl_midwall_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_walled_bowl_midwall_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepExtrema_DistShapeShape.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace mwx = walled_bowl_midwall_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[MWC] %-14s %-20s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the SAME steep bowl-lidded convex-quad prism operand ─────
// Mirrors walled_bowl_midwall_fixture::buildOperand — a Geom_BezierSurface bowl trimmed
// by the convex quad, four planar walls with an exact degree-2 Bézier top edge, and a
// planar bottom on z = −H0 — then sews the six faces into one solid.
static TopoDS_Face buildOcctBowlTop() {
  const std::vector<nm::Point3> poles = mwx::bowlPoles();  // row-major, U outer, 3×3
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
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

static TopoDS_Shape buildOcctOperand() {
  const nt::FaceSurface bowl = mwx::bowlSurface();
  ntess::SurfaceEvaluator eval(bowl, nt::Location{});
  const auto& q = fx::quadUV();
  std::array<nm::Point3, 4> T, B, ctrl;
  for (int k = 0; k < 4; ++k) {
    T[k] = eval.value(q[k].x, q[k].y);
    B[k] = nm::Point3{T[k].x, T[k].y, -mwx::kH0};
  }
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const nm::Point3 m{(q[k].x + q[k1].x) * 0.5, (q[k].y + q[k1].y) * 0.5, 0.0};
    const nm::Point3 S0 = T[k], S1 = T[k1], Sm = eval.value(m.x, m.y);
    ctrl[k] = nm::Point3{2 * Sm.x - 0.5 * (S0.x + S1.x), 2 * Sm.y - 0.5 * (S0.y + S1.y),
                         2 * Sm.z - 0.5 * (S0.z + S1.z)};
  }

  BRepBuilderAPI_Sewing sew(1e-6);
  sew.Add(buildOcctBowlTop());

  // four planar walls: B[k] → B[k1] → T[k1] → T[k] (exact degree-2 Bézier) → B[k].
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    TColgp_Array1OfPnt bp(1, 3);
    bp.SetValue(1, P(T[k1])); bp.SetValue(2, P(ctrl[k])); bp.SetValue(3, P(T[k]));
    Handle(Geom_BezierCurve) top = new Geom_BezierCurve(bp);
    BRepBuilderAPI_MakeWire w;
    w.Add(BRepBuilderAPI_MakeEdge(P(B[k]), P(B[k1])).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(P(B[k1]), P(T[k1])).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(top).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(P(T[k]), P(B[k])).Edge());
    sew.Add(BRepBuilderAPI_MakeFace(w.Wire(), Standard_True).Face());
  }
  // planar bottom on z = −H0.
  {
    BRepBuilderAPI_MakeWire w;
    for (int k = 0; k < 4; ++k)
      w.Add(BRepBuilderAPI_MakeEdge(P(B[k]), P(B[(k + 1) % 4])).Edge());
    sew.Add(BRepBuilderAPI_MakeFace(w.Wire(), Standard_True).Face());
  }

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
static double occtArea(const TopoDS_Shape& s) {
  GProp_GProps g; BRepGProp::SurfaceProperties(s, g); return g.Mass();
}
static int occtSolidCount(const TopoDS_Shape& s) {
  int n = 0;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++n;
  return n;
}

static long eulerChar(const ntess::Mesh& m) {
  const long V = static_cast<long>(m.vertices.size());
  const long F = static_cast<long>(m.triangles.size());
  const long E = static_cast<long>(ntess::edgeUseCounts(m).size());
  return V - E + F;
}
struct BBox { double lo[3], hi[3]; };
static BBox meshBBox(const ntess::Mesh& m) {
  BBox b{{1e30, 1e30, 1e30}, {-1e30, -1e30, -1e30}};
  for (const nm::Point3& v : m.vertices) {
    const double c[3] = {v.x, v.y, v.z};
    for (int i = 0; i < 3; ++i) { b.lo[i] = std::min(b.lo[i], c[i]); b.hi[i] = std::max(b.hi[i], c[i]); }
  }
  return b;
}

int main() {
  std::printf("== MOAT M2 walled-bowl MID-WALL freeform CUT: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double relTol = 0.02;  // curved-tessellation volume/area band (never widened)

  const nt::Shape operand = mwx::buildOperand();
  const TopoDS_Shape occtOperand = buildOcctOperand();
  report("occt-operand", "solid-built", occtSolidCount(occtOperand) == 1, "sewn 6-face solid");
  {
    const double vO = occtVolume(occtOperand), vC = mwx::fullVolume();
    char buf[80]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f", vO, vC);
    report("occt-operand", "volume-vs-closed", std::fabs(vO - vC) / vC <= relTol, buf);
  }

  // OCCT reference boolean: CUT keeps z ≤ c (remove the box spanning z ≥ c).
  const double c = mwx::kCutZ;
  TopoDS_Solid boxAbove = BRepPrimAPI_MakeBox(gp_Pnt(-10, -10, c), gp_Pnt(10, 10, 10)).Solid();
  BRepAlgoAPI_Cut cutter(occtOperand, boxAbove); cutter.Build();
  report("occt-cut", "built", cutter.IsDone() && occtSolidCount(cutter.Shape()) == 1, "z<=c");
  const TopoDS_Shape occtCut = cutter.Shape();
  const double vOcct = occtVolume(occtCut);
  const double aOcct = occtArea(occtCut);
  const double vClosed = mwx::cutVolume();
  {
    const double relO = std::fabs(vOcct - vClosed) / vClosed;
    char buf[88]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
    report("volume", "occt-vs-closed", relO <= relTol, buf);
  }
  Bnd_Box occtBB;
  BRepBndLib::AddOptimal(occtCut, occtBB, Standard_True, Standard_False);

  auto parityAt = [&](double defl) {
    char tag[24]; std::snprintf(tag, sizeof(tag), "d=%.4f", defl);
    const double spatialTol = 1.6 * defl;

    bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
    const nt::Shape cut = bo::curvedWallHalfSpaceCut(operand, mwx::cutPlane(), bo::KeepSide::Below, defl, &why);
    {
      char buf[96]; std::snprintf(buf, sizeof(buf), "%s decline=%s", tag, bo::curvedWallDeclineName(why));
      report("native-cut", "composed", why == bo::CurvedWallCutDecline::Ok && !cut.isNull(), buf);
    }
    if (cut.isNull()) return;

    // the mid-wall signature: 7 faces (disk + 4 wall trapezoids + base + ANNULAR cap).
    int nFaces = 0;
    for (nt::Explorer fxp(cut, nt::ShapeType::Face); fxp.more(); fxp.next()) ++nFaces;
    { char buf[48]; std::snprintf(buf, sizeof(buf), "%s faces=%d (7)", tag, nFaces);
      report("native-cut", "annular-topology", nFaces == 7, buf); }

    ntess::MeshParams mp; mp.deflection = defl;
    const ntess::Mesh m = ntess::SolidMesher(mp).mesh(cut);
    report("native-cut", "watertight", ntess::isWatertight(m), tag);
    { const long chi = eulerChar(m);
      char buf[48]; std::snprintf(buf, sizeof(buf), "%s euler=%ld (solid=2)", tag, chi);
      report("native-cut", "topology-solid", chi == 2, buf); }

    const double vNat = std::fabs(ntess::enclosedVolume(m));
    const double aNat = ntess::surfaceArea(m);
    const BBox bNat = meshBBox(m);
    {
      const double rel = std::fabs(vNat - vOcct) / vOcct;
      char buf[120]; std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, vNat, vOcct, rel);
      report("volume", "native-vs-occt", rel <= relTol, buf);
    }
    {
      const double rel = std::fabs(aNat - aOcct) / aOcct;
      char buf[104]; std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, aNat, aOcct, rel);
      report("area", "native-vs-occt", rel <= relTol, buf);
    }
    {
      double xo[3], xh[3];
      occtBB.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
      double worst = 0.0;
      for (int i = 0; i < 3; ++i) {
        worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
        worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
      }
      char buf[112]; std::snprintf(buf, sizeof(buf), "%s worst=%.3e tol=%.3e", tag, worst, spatialTol);
      report("bbox", "native-vs-occt", worst <= spatialTol, buf);
    }
    {
      double worst = 0.0;
      for (const nm::Point3& v : m.vertices) {
        TopoDS_Vertex ov = BRepBuilderAPI_MakeVertex(P(v)).Vertex();
        BRepExtrema_DistShapeShape d(ov, occtCut);
        if (d.IsDone() && d.NbSolution() > 0) worst = std::max(worst, static_cast<double>(d.Value()));
      }
      char buf[96]; std::snprintf(buf, sizeof(buf), "%s maxDist=%.3e tol=%.3e", tag, worst, spatialTol);
      report("hausdorff", "native->occt", worst <= spatialTol, buf);
    }
  };

  parityAt(0.02);
  parityAt(0.008);
  parityAt(0.005);

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
