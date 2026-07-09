// SPDX-License-Identifier: Apache-2.0
//
// native_curved_wall_cut_parity.mm — MOAT M2 curved-wall freeform half-space
// CUT / COMMON SIM GATE (b): native-vs-OCCT on a booted iOS simulator.
//
// The native cut (src/native/boolean/curved_wall_cut.h, OCCT-FREE) composes the landed
// M2 substrate — B1 recognise → M1 seam trace → B2 SMOOTH-TRIM split (closed circular
// seam → disk + annulus-hole) → analytic wall split → flat circular cap synth → M0 weld
// → watertight+volume self-verify — into ONE watertight keep-side solid from a bowl-cup
// operand (a STEEP Bézier bowl trimmed by a rim CIRCLE + a flat top-lid disk) cut by the
// HORIZONTAL plane z = c. This harness proves, against the OCCT ORACLE:
//
//   The SAME operand is reconstructed in OCCT (a Geom_BezierSurface bowl trimmed by the
//   rim circle + a planar lid disk on z = a·R², sewn into a solid) and cut by the SAME
//   half-space via BRepAlgoAPI_Cut (CUT, keep z≤c) / BRepAlgoAPI_Common (COMMON, keep
//   z≥c) against a large box spanning the kept half. The native
//   curvedWallHalfSpaceCut result (measured by the native M0 tessellator) is compared to
//   the OCCT boolean on:
//     * VOLUME     — native enclosedVolume vs BRepGProp::VolumeProperties (rel band);
//                    both cross-checked against the closed forms π·ρ²·c/2 and V(full)−that.
//     * AREA       — native surfaceArea vs BRepGProp::SurfaceProperties.
//     * WATERTIGHT — the native cut mesh is a closed 2-manifold; the OCCT cut is a single
//                    closed solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//     * BBOX       — native vertex min/max vs BRepBndLib per-axis (spatial band).
//     * HAUSDORFF  — max native→OCCT vertex distance via BRepExtrema (spatial band).
//
// The CUT (KeepSide::Below — the deep cup) and the COMMON (KeepSide::Above — the annulus +
// lid + cap) are BOTH robust keep-sides now: parity is asserted at MULTIPLE resonance-free
// deflections for each. The COMMON annulus↔lid OUTER CURVED-RIM weld (MOAT M0-rim) welds
// watertight across the full ladder — the curved-rim tessellator pin pins the flat lid's
// diverging rim samples to the bowl's canonical rim curve and drops the coarse-regime
// coincident sliver — so COMMON matches OCCT at every asserted deflection (no longer an
// honest decline). A case that still cannot weld returns NULL → OCCT, never a leaky solid.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-curved-wall-cut.sh.
//
#include "native/boolean/curved_wall_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/curved_wall_cut_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_curved_wall_cut_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
#include <Geom_Plane.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
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
#include <BRepAlgoAPI_Common.hxx>
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
namespace cwx = curved_wall_cut_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[CWC] %-14s %-20s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the SAME bowl-cup operand ───────────────────────────
// Mirrors curved_wall_cut_fixture::buildOperand — a Geom_BezierSurface bowl trimmed by
// the rim CIRCLE (a kRimSegs-gon of pcurve line segments, matching the native trim) +
// a planar lid disk on z = a·R² bounded by the SAME rim 3-D Bézier arcs — sewn to a solid.
static TopoDS_Face buildOcctBowl() {
  const std::vector<nm::Point3> poles = cwx::bowlPoles();  // row-major, U outer, 3×3
  TColgp_Array2OfPnt arr(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      arr.SetValue(i + 1, j + 1, P(poles[i * 3 + j]));
  Handle(Geom_BezierSurface) surf = new Geom_BezierSurface(arr);

  const std::vector<nm::Point3> uv = cwx::rimUV();
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
  TopoDS_Face face = BRepBuilderAPI_MakeFace(surf, mkWire.Wire(), Standard_True).Face();
  BRepLib::BuildCurves3d(face);
  return face;
}

static TopoDS_Shape buildOcctOperand() {
  const nt::FaceSurface bowl = cwx::bowlSurface();
  ntess::SurfaceEvaluator eval(bowl, nt::Location{});
  const std::vector<nm::Point3> uv = cwx::rimUV();
  const int n = static_cast<int>(uv.size());

  std::vector<nm::Point3> rim3d(n), ctrl(n);
  for (int k = 0; k < n; ++k) rim3d[k] = eval.value(uv[k].x, uv[k].y);
  for (int k = 0; k < n; ++k) {
    const int k1 = (k + 1) % n;
    const nm::Point3 m{(uv[k].x + uv[k1].x) * 0.5, (uv[k].y + uv[k1].y) * 0.5, 0.0};
    const nm::Point3 Sm = eval.value(m.x, m.y);
    ctrl[k] = nm::Point3{2 * Sm.x - 0.5 * (rim3d[k].x + rim3d[k1].x),
                         2 * Sm.y - 0.5 * (rim3d[k].y + rim3d[k1].y),
                         2 * Sm.z - 0.5 * (rim3d[k].z + rim3d[k1].z)};
  }

  // Sew tolerance ~1e-2 (the ORACLE's reconstruction tolerance, NOT a native pass-forcing
  // tolerance): the bowl's degree-2 rim edge (surface image of a straight UV chord) DIPS
  // ~6e-4 below the flat lid's straight rim chord between rim points — a fixture polygon-
  // approximation gap the oracle bridges to form the closed reference bowl-cup solid.
  BRepBuilderAPI_Sewing sew(1e-2);
  sew.Add(buildOcctBowl());

  // planar top-lid disk on the explicit plane z = a·R². The IDEAL bowl-cup rim is the
  // constant-z circle where the bowl (z=a·r²) reaches z=a·R²; the lid is bounded by the
  // rim as straight 3-D chords between rim points (all exactly at z=a·R², so the wire is
  // planar). The native fixture's rim degree-2 arcs DIP ≲1e-3 below the plane between rim
  // points — negligible against the parity bands; OCCT sewing bridges the tiny gap at the
  // shared rim vertices. This is the reference SOLID, not a native-topology mirror.
  {
    Handle(Geom_Plane) lidPln = new Geom_Plane(gp_Pnt(0, 0, cwx::kRimZ), gp_Dir(0, 0, 1));
    BRepBuilderAPI_MakeWire w;
    for (int k = 0; k < n; ++k) {
      const nm::Point3 a{rim3d[k].x, rim3d[k].y, cwx::kRimZ};
      const nm::Point3 b{rim3d[(k + 1) % n].x, rim3d[(k + 1) % n].y, cwx::kRimZ};
      w.Add(BRepBuilderAPI_MakeEdge(P(a), P(b)).Edge());
    }
    TopoDS_Face lid = BRepBuilderAPI_MakeFace(lidPln, w.Wire(), Standard_True).Face();
    BRepLib::BuildCurves3d(lid);
    sew.Add(lid);
  }

  sew.Perform();
  TopoDS_Shape sewn = sew.SewedShape();
  BRepBuilderAPI_MakeSolid mk;
  for (TopExp_Explorer ex(sewn, TopAbs_SHELL); ex.More(); ex.Next())
    mk.Add(TopoDS::Shell(ex.Current()));
  TopoDS_Solid solid = mk.Solid();
  // ensure OUTWARD orientation (a sewn shell may come out inside-out, which makes the
  // boolean treat the solid as the infinite complement). SIGNED volume < 0 ⇒ reverse.
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

// ── compare a native keep-side vs the OCCT boolean at a deflection ────────────────
static void parity(const char* tag, const nt::Shape& nativeCut, const TopoDS_Shape& occtCut,
                   double vClosed, double defl) {
  const double relTol = 0.10;             // volume/area band (coarse curved cup)
  const double spatialTol = 1.6 * defl;   // bbox / Hausdorff spatial band
  const double vOcct = occtVolume(occtCut);
  const double aOcct = occtArea(occtCut);
  {
    const double relO = std::fabs(vOcct - vClosed) / vClosed;
    char buf[96]; std::snprintf(buf, sizeof(buf), "%s occt=%.6f closed=%.6f rel=%.3e", tag, vOcct, vClosed, relO);
    report("volume", "occt-vs-closed", relO <= 0.03, buf);
  }
  Bnd_Box occtBB;
  BRepBndLib::AddOptimal(occtCut, occtBB, Standard_True, Standard_False);

  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(nativeCut);
  report("native", "watertight", ntess::isWatertight(m), tag);
  {
    const long chi = eulerChar(m);
    char buf[48]; std::snprintf(buf, sizeof(buf), "%s euler=%ld (solid=2)", tag, chi);
    report("native", "topology-solid", chi == 2, buf);
  }
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
}

int main() {
  std::printf("== MOAT M2 curved-wall freeform CUT/COMMON: native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const nt::Shape operand = cwx::buildOperand();
  const TopoDS_Shape occtOperand = buildOcctOperand();
  report("occt-operand", "solid-built", occtSolidCount(occtOperand) == 1, "sewn bowl+lid solid");
  {
    const double vO = occtVolume(occtOperand), vC = cwx::fullVolume();
    char buf[80]; std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f", vO, vC);
    report("occt-operand", "volume-vs-closed", std::fabs(vO - vC) / vC <= 0.02, buf);
  }

  // OCCT reference booleans (deflection-independent). CUT keeps z≤c; COMMON keeps z≥c.
  const double c = cwx::kCutZ;
  TopoDS_Solid boxBelow = BRepPrimAPI_MakeBox(gp_Pnt(-1, -1, -1), gp_Pnt(1, 1, c)).Solid();
  TopoDS_Solid boxAbove = BRepPrimAPI_MakeBox(gp_Pnt(-1, -1, c), gp_Pnt(1, 1, 1)).Solid();
  BRepAlgoAPI_Common cutBelow(occtOperand, boxBelow); cutBelow.Build();
  BRepAlgoAPI_Common cutAbove(occtOperand, boxAbove); cutAbove.Build();
  report("occt-cut", "below-built", cutBelow.IsDone() && occtSolidCount(cutBelow.Shape()) == 1, "z<=c");
  report("occt-cut", "above-built", cutAbove.IsDone() && occtSolidCount(cutAbove.Shape()) == 1, "z>=c");
  const TopoDS_Shape occtBelow = cutBelow.Shape();
  const TopoDS_Shape occtAbove = cutAbove.Shape();

  // ── CUT (Below) — the disk↔flat-cap CLOSED-SEAM keep-side, native-vs-OCCT across the
  // deflection ladder INCLUDING the fine d=0.004 that declined before the MOAT M0w
  // closed-seam tessellator pin (SeamPins): the seam now welds watertight there and
  // matches the OCCT BRepAlgoAPI_Common oracle on volume/area/topology.
  for (double d : {0.0102, 0.00532, 0.004, 0.00278}) {
    char tag[24]; std::snprintf(tag, sizeof(tag), "CUT d=%.4f", d);
    bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
    const nt::Shape cut = bo::curvedWallHalfSpaceCut(operand, cwx::cutPlane(), bo::KeepSide::Below, d, &why);
    char cb[64]; std::snprintf(cb, sizeof(cb), "%s decline=%s", tag, bo::curvedWallDeclineName(why));
    report("native-cut", "composed", !cut.isNull(), cb);
    if (!cut.isNull()) parity(tag, cut, occtBelow, cwx::cutVolume(), d);
  }

  // ── COMMON (Above) — the annulus↔lid OUTER CURVED-RIM keep-side, native-vs-OCCT across
  // the deflection ladder INCLUDING the fine d=0.004 that declined before the MOAT M0-rim
  // curved-rim tessellator pin (recordSeamChordPins on isCurvedSharedRim + the weld's
  // coincident-duplicate drop): the curved rim now welds watertight there and matches the
  // OCCT BRepAlgoAPI_Common oracle on volume/area/topology (no longer an honest decline).
  for (double d : {0.0102, 0.00532, 0.004, 0.00278}) {
    char tag[24]; std::snprintf(tag, sizeof(tag), "COMMON d=%.4f", d);
    bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
    const nt::Shape com = bo::curvedWallHalfSpaceCut(operand, cwx::cutPlane(), bo::KeepSide::Above, d, &why);
    char cb[64]; std::snprintf(cb, sizeof(cb), "%s decline=%s", tag, bo::curvedWallDeclineName(why));
    report("native-common", "composed", !com.isNull(), cb);
    if (!com.isNull()) parity(tag, com, occtAbove, cwx::commonVolume(), d);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
