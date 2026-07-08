// SPDX-License-Identifier: Apache-2.0
//
// native_first_freeform_boolean_parity.mm — MOAT M2-assembly / B4 (the FIRST
// end-to-end freeform↔analytic half-space CUT) SIM GATE (b): native-vs-OCCT on a
// booted iOS simulator.
//
// The native cut (src/native/boolean/half_space_cut.h, OCCT-FREE) composes the landed
// M2 substrate — B1 recognise, M1 seam trace (ssi::trace_intersection), B2 face split,
// B4 analytic-face split + cross-section cap weld, B3 confirm, M0 self-verify — into ONE
// watertight keep-side solid from a bowl-lidded convex-quad prism cut by the plane x=0
// (keep x≤0). This harness proves, against the OCCT ORACLE:
//
//   The SAME operand is reconstructed in OCCT (Geom_BezierSurface bowl top + four
//   planar Bézier-topped walls + planar bottom, sewn into a solid) and cut by the SAME
//   half-space via BRepAlgoAPI_Cut against a large box spanning x∈[0,+big]. The native
//   freeformHalfSpaceCut result (measured by the native M0 tessellator) is compared to
//   the OCCT cut on:
//     * VOLUME       — native enclosedVolume vs BRepGProp::VolumeProperties (rel ≤ 2e-2,
//                      the curved-tessellation deflection band); both cross-checked
//                      against the closed-form oracle ffx::cutVolume().
//     * AREA         — native surfaceArea vs BRepGProp::SurfaceProperties (rel ≤ 2e-2).
//     * WATERTIGHT   — the native cut mesh is a closed 2-manifold (every edge shared by
//                      exactly two triangles) and the OCCT cut is a single closed solid.
//     * TOPOLOGY     — the native cut mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//     * BBOX         — native vertex min/max vs BRepBndLib per-axis (abs ≤ 1.5·deflection).
//     * HAUSDORFF    — max over native mesh vertices of BRepExtrema_DistShapeShape to the
//                      OCCT cut solid (one-sided native→OCCT, spatial ≤ 1.5·deflection).
//
// OCCT is the ORACLE ONLY, never linked into src/native. The recognise/trace/split/cut
// /mesh are 100% native; OCCT builds the same operand and performs the reference cut.
//
// Build: scripts/run-sim-native-first-freeform-boolean.sh.
//
#include "native/boolean/half_space_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/first_freeform_boolean_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_first_freeform_boolean_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Dir2d.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Surface.hxx>
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
namespace ffx = first_freeform_boolean_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NFB] %-14s %-20s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the SAME bowl-lidded convex-quad prism operand ───────
// Mirrors first_freeform_boolean_fixture::buildOperand — a Geom_BezierSurface bowl top
// trimmed by the convex quad, four planar walls with an exact degree-2 Bézier top edge,
// and a planar bottom on z=−H0 — then sews the six faces into one solid.
static TopoDS_Face buildOcctBowlTop() {
  const std::vector<nm::Point3> poles = fx::bowlPoles();  // row-major, U outer, 3×3
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
  // T[k]/B[k]/ctrl[k] computed identically to the native fixture buildOperand.
  const nt::FaceSurface bowl = fx::bowlSurface();
  ntess::SurfaceEvaluator eval(bowl, nt::Location{});
  const auto& q = fx::quadUV();
  std::array<nm::Point3, 4> T, B, ctrl;
  for (int k = 0; k < 4; ++k) {
    T[k] = eval.value(q[k].x, q[k].y);
    B[k] = nm::Point3{T[k].x, T[k].y, -ffx::kH0};
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
  // planar bottom on z=−H0.
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
  return mk.Solid();
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

// ── native mesh measurement helpers ───────────────────────────────────────────────
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
  std::printf("== MOAT M2b/B4 first freeform boolean (half-space CUT): native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  const double defl = 0.01;      // matches the host GATE (a) deflection band
  const double relTol = 0.02;    // curved-tessellation volume/area band (never widened)
  const double spatialTol = 1.5 * defl;  // bbox / Hausdorff spatial band

  // ── native: build the operand and run the FIRST freeform half-space cut ──────
  const nt::Shape operand = ffx::buildOperand();
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const nt::Shape cut = bo::freeformHalfSpaceCut(operand, ffx::cutPlane(), bo::KeepSide::Below, defl, &why);
  {
    char buf[80];
    std::snprintf(buf, sizeof(buf), "decline=%s", bo::declineName(why));
    report("native-cut", "composed", why == bo::HalfSpaceCutDecline::Ok && !cut.isNull(), buf);
  }
  if (cut.isNull()) {
    std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
    std::fflush(stdout);
    std::_Exit(1);
  }

  // ── native: mesh the cut result (M0, unchanged) and measure ─────────────────
  ntess::MeshParams mp; mp.deflection = defl;
  const ntess::Mesh m = ntess::SolidMesher(mp).mesh(cut);
  report("native-cut", "meshes", !m.triangles.empty(), "M0 solid mesh");

  const bool wt = ntess::isWatertight(m);
  report("native-cut", "watertight", wt, "closed 2-manifold");
  {
    const long chi = eulerChar(m);
    char buf[48];
    std::snprintf(buf, sizeof(buf), "euler=%ld (solid=2)", chi);
    report("native-cut", "topology-solid", chi == 2, buf);
  }

  const double vNat = std::fabs(ntess::enclosedVolume(m));
  const double aNat = ntess::surfaceArea(m);
  const BBox bNat = meshBBox(m);

  // ── OCCT oracle: reconstruct the operand and cut it by the same half-space ──
  const TopoDS_Shape occtOperand = buildOcctOperand();
  report("occt-operand", "solid-built", occtSolidCount(occtOperand) == 1, "sewn 6-face solid");

  // Cut off the x>0 half with a large box spanning x∈[0,+big] (keep x≤0 = native Below).
  TopoDS_Solid box = BRepPrimAPI_MakeBox(gp_Pnt(0, -10, -10), gp_Pnt(10, 10, 10)).Solid();
  BRepAlgoAPI_Cut cutter(occtOperand, box);
  cutter.Build();
  report("occt-cut", "built", cutter.IsDone(), "BRepAlgoAPI_Cut");
  const TopoDS_Shape occtCut = cutter.Shape();
  report("occt-cut", "single-solid", occtSolidCount(occtCut) == 1, "one closed solid");

  const double vOcct = occtVolume(occtCut);
  const double aOcct = occtArea(occtCut);
  const double vClosed = ffx::cutVolume();

  // ── VOLUME parity (+ closed-form cross-check) ────────────────────────────────
  {
    const double rel = std::fabs(vNat - vOcct) / vOcct;
    char buf[112];
    std::snprintf(buf, sizeof(buf), "nat=%.6f occt=%.6f closed=%.6f rel=%.3e", vNat, vOcct, vClosed, rel);
    report("volume", "native-vs-occt", rel <= relTol, buf);
  }
  {
    const double relO = std::fabs(vOcct - vClosed) / vClosed;
    char buf[80];
    std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
    report("volume", "occt-vs-closed", relO <= relTol, buf);
  }

  // ── AREA parity ──────────────────────────────────────────────────────────────
  {
    const double rel = std::fabs(aNat - aOcct) / aOcct;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "nat=%.6f occt=%.6f rel=%.3e", aNat, aOcct, rel);
    report("area", "native-vs-occt", rel <= relTol, buf);
  }

  // ── BBOX parity (per-axis) ───────────────────────────────────────────────────
  {
    // AddOptimal gives the TIGHT box for the free-form Bézier face; plain Add returns
    // the loose control-point pole hull (x∈[-0.5,0.5]) which is not the trimmed extent.
    Bnd_Box bb; BRepBndLib::AddOptimal(occtCut, bb, Standard_True, Standard_False);
    double xo[3], xh[3];
    bb.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
      worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "natX[%.3f,%.3f] occtX[%.3f,%.3f] worst=%.3e tol=%.3e",
                  bNat.lo[0], bNat.hi[0], xo[0], xh[0], worst, spatialTol);
    report("bbox", "native-vs-occt", worst <= spatialTol, buf);
  }

  // ── HAUSDORFF (one-sided native vertices → OCCT cut solid) ───────────────────
  {
    double worst = 0.0;
    for (const nm::Point3& v : m.vertices) {
      TopoDS_Vertex ov = BRepBuilderAPI_MakeVertex(P(v)).Vertex();
      BRepExtrema_DistShapeShape d(ov, occtCut);
      if (d.IsDone() && d.NbSolution() > 0) worst = std::max(worst, static_cast<double>(d.Value()));
    }
    char buf[80];
    std::snprintf(buf, sizeof(buf), "maxDist=%.3e tol=%.3e", worst, spatialTol);
    report("hausdorff", "native->occt", worst <= spatialTol, buf);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
