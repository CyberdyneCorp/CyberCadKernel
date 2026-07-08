// SPDX-License-Identifier: Apache-2.0
//
// native_multi_seam_freeform_boolean_parity.mm — MOAT M2-multiseam SIM GATE (b):
// native-vs-OCCT on a booted iOS simulator for the MULTI-FACE corner-clip weld (the
// FIRST multi-seam two-operand freeform boolean).
//
// The native weld (src/native/boolean/multi_face_weld.h via freeformBooleanMultiSeam,
// OCCT-FREE) composes the landed substrate — B1 recognise, the two-arc seam graph
// (`seam_graph.h`), the junction-aware wall split (`junction_split.h`), the analytic
// face corner-clips + synthesised box caps / notched box faces — into ONE watertight
// result solid for CUT (A−B, the L-solid), COMMON (A∩B, the corner piece) and FUSE
// (A∪B), for the corner-box pose whose x=0 AND y=0 faces each slice A's Bézier wall.
//
// This harness reconstructs the SAME A in OCCT (Geom_BezierSurface bowl top + four
// planar Bézier-topped walls + planar bottom, sewn) and the SAME corner box B as a
// BRepPrimAPI box, runs BRepAlgoAPI_Cut/Common/Fuse (the ORACLE), and compares the
// native result (measured by the native M0 tessellator) on:
//   VOLUME (rel ≤ 2e-2, cross-checked vs the closed-form corner oracle), AREA (rel ≤ 2e-2),
//   WATERTIGHT (closed 2-manifold), TOPOLOGY (Euler χ = 2), BBOX + HAUSDORFF (≤ 1.5·defl),
//   and a CLASSIFY batch vs BRepClass3d_SolidClassifier (zero crisp IN↔OUT disagreements).
// Plus the fallback contract: a single-cut box DECLINES to NULL (→ OCCT), never a leaky solid.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-multi-seam-freeform-boolean.sh.
//
#include "native/boolean/multi_seam.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/first_freeform_boolean_fixture.h"
#include "../native/multi_seam_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_multi_seam_freeform_boolean_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Solid.hxx>
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
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepClass3d_SolidClassifier.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace msx = multi_seam_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[MSW] %-10s %-18s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}
static inline gp_Pnt P(const nm::Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT oracle: reconstruct the SAME bowl-lidded convex-quad prism operand A ──────
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

static TopoDS_Shape buildOcctOperand() {
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
  std::printf("== MOAT M2-multiseam multi-face corner-clip weld: native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02;

  const nt::Shape A = ffx::buildOperand();
  const nt::Shape Bx = msx::cornerBox();
  const TopoDS_Shape occtA = buildOcctOperand();
  report("occt", "operand-built", occtSolidCount(occtA) == 1, "sewn 6-face solid");
  TopoDS_Solid occtB = BRepPrimAPI_MakeBox(gp_Pnt(0.0, 0.0, -0.6), gp_Pnt(0.8, 0.6, 0.2)).Solid();

  struct OpCase {
    const char* name;
    bo::MultiSeamOp op;
    TopoDS_Shape occt;
    double closed;
  };
  BRepAlgoAPI_Cut cutter(occtA, occtB); cutter.Build();
  BRepAlgoAPI_Common commoner(occtA, occtB); commoner.Build();
  BRepAlgoAPI_Fuse fuser(occtA, occtB); fuser.Build();
  report("occt", "cut-built", cutter.IsDone(), "BRepAlgoAPI_Cut");
  report("occt", "common-built", commoner.IsDone(), "BRepAlgoAPI_Common");
  report("occt", "fuse-built", fuser.IsDone(), "BRepAlgoAPI_Fuse");

  const OpCase cases[3] = {
      {"CUT", bo::MultiSeamOp::Cut, cutter.Shape(), msx::volCut()},
      {"COMMON", bo::MultiSeamOp::Common, commoner.Shape(), msx::volCommon()},
      {"FUSE", bo::MultiSeamOp::Fuse, fuser.Shape(), msx::volUnion()}};

  auto parityAt = [&](const OpCase& c, double defl) {
    char tag[40];
    std::snprintf(tag, sizeof(tag), "%s d=%.3f", c.name, defl);
    const double spatialTol = 1.5 * defl;
    const double vOcct = occtVolume(c.occt);
    const double aOcct = occtArea(c.occt);
    Bnd_Box occtBB;
    BRepBndLib::AddOptimal(c.occt, occtBB, Standard_True, Standard_False);

    // OCCT vs closed-form corner oracle (independent cross-check).
    {
      const double relO = std::fabs(vOcct - c.closed) / c.closed;
      char buf[104];
      std::snprintf(buf, sizeof(buf), "%s occt=%.6f closed=%.6f rel=%.3e", tag, vOcct, c.closed, relO);
      report(c.name, "occt-vs-closed", relO <= relTol, buf);
    }

    bo::MultiSeamReport why;
    const nt::Shape r = bo::freeformBooleanMultiSeam(A, Bx, c.op, defl, &why);
    {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s decline=%s", tag, bo::multiSeamDeclineName(why.decline));
      report(c.name, "composed", why.decline == bo::MultiSeamDecline::Ok && !r.isNull(), buf);
    }
    if (r.isNull()) return;

    ntess::MeshParams mp; mp.deflection = defl;
    const ntess::Mesh m = ntess::SolidMesher(mp).mesh(r);
    report(c.name, "watertight", ntess::isWatertight(m), tag);
    {
      const long chi = eulerChar(m);
      char buf[56];
      std::snprintf(buf, sizeof(buf), "%s euler=%ld (solid=2)", tag, chi);
      report(c.name, "topology", chi == 2, buf);
    }
    const double vNat = std::fabs(ntess::enclosedVolume(m));
    const double aNat = ntess::surfaceArea(m);
    const BBox bNat = meshBBox(m);
    {
      const double rel = std::fabs(vNat - vOcct) / vOcct;
      char buf[120];
      std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, vNat, vOcct, rel);
      report(c.name, "volume", rel <= relTol, buf);
    }
    {
      const double rel = std::fabs(aNat - aOcct) / aOcct;
      char buf[120];
      std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, aNat, aOcct, rel);
      report(c.name, "area", rel <= relTol, buf);
    }
    {
      double xo[3], xh[3];
      occtBB.Get(xo[0], xo[1], xo[2], xh[0], xh[1], xh[2]);
      double worst = 0.0;
      for (int i = 0; i < 3; ++i) {
        worst = std::max(worst, std::fabs(bNat.lo[i] - xo[i]));
        worst = std::max(worst, std::fabs(bNat.hi[i] - xh[i]));
      }
      char buf[104];
      std::snprintf(buf, sizeof(buf), "%s worst=%.3e tol=%.3e", tag, worst, spatialTol);
      report(c.name, "bbox", worst <= spatialTol, buf);
    }
    {
      double worst = 0.0;
      for (const nm::Point3& v : m.vertices) {
        TopoDS_Vertex ov = BRepBuilderAPI_MakeVertex(P(v)).Vertex();
        BRepExtrema_DistShapeShape d(ov, c.occt);
        if (d.IsDone() && d.NbSolution() > 0) worst = std::max(worst, static_cast<double>(d.Value()));
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s maxDist=%.3e tol=%.3e", tag, worst, spatialTol);
      report(c.name, "hausdorff", worst <= spatialTol, buf);
    }
    // ── query-point batch vs OCCT solid classifier (zero crisp IN↔OUT disagreements) ─
    {
      const bo::Aabb bb = bo::meshAabb(m);
      const int N = 12;
      int agree = 0, disagree = 0, band = 0, total = 0;
      for (int ix = 0; ix <= N; ++ix)
        for (int iy = 0; iy <= N; ++iy)
          for (int iz = 0; iz <= N; ++iz) {
            const double x = -0.5 + 1.3 * (double)ix / N;
            const double y = -0.4 + 1.0 * (double)iy / N;
            const double z = -0.6 + 0.8 * (double)iz / N;
            const nm::Point3 qp{x, y, z};
            ++total;
            gp_Pnt op(x, y, z);
            BRepClass3d_SolidClassifier cls(c.occt, op, 1e-7);
            if (cls.State() == TopAbs_ON) { ++band; continue; }
            const bool occtIn = cls.State() == TopAbs_IN;
            const bo::Membership nv = bo::classifyPointInMesh(m, bb, defl, qp);
            if (nv != bo::Membership::In && nv != bo::Membership::Out) { ++band; continue; }
            ((nv == bo::Membership::In) == occtIn ? ++agree : ++disagree);
          }
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%s agree=%d disagree=%d band=%d/%d", tag, agree, disagree, band, total);
      report(c.name, "classify", disagree == 0 && agree > 0, buf);
    }
  };

  for (const OpCase& c : cases) { parityAt(c, 0.01); parityAt(c, 0.005); }

  // ── fallback contract: a single-cut box DECLINES to NULL (→ OCCT) ──────────────
  {
    // A box straddling only the x=0 plane (one cutting face) is the single-seam path's
    // job: the multi-seam builder DECLINES it (SeamGraph NotTwoCuttingFaces), never a solid.
    const nt::Shape oneCut = msx::buildCornerBox(0.0, 0.8, -0.6, 0.6, -0.6, 0.2);
    bo::MultiSeamReport why;
    const nt::Shape r = bo::freeformBooleanMultiSeam(A, oneCut, bo::MultiSeamOp::Cut, 0.01, &why);
    char buf[72];
    std::snprintf(buf, sizeof(buf), "decline=%s", bo::multiSeamDeclineName(why.decline));
    report("fallback", "declines-null",
           r.isNull() && why.decline == bo::MultiSeamDecline::SeamGraphDeclined, buf);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
