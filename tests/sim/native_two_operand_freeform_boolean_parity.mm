// SPDX-License-Identifier: Apache-2.0
//
// native_two_operand_freeform_boolean_parity.mm — MOAT M2-FUSE SIM GATE (b):
// native-vs-OCCT on a booted iOS simulator for the FIRST two-operand freeform boolean.
//
// The native FUSE (src/native/boolean/two_operand.h, OCCT-FREE) composes the landed
// substrate — B1 recognise, the inter-solid seam (M1 trace + B2 split + hscdetail cap
// loop), B3 membership confirm, the two-operand weld, and the M0 self-verify — into ONE
// watertight union solid of a bowl-lidded convex-quad prism A with a finite axis-aligned
// box B (whose only cut of A is the plane x=0). This harness proves, against the OCCT
// ORACLE:
//
//   The SAME A is reconstructed in OCCT (Geom_BezierSurface bowl top + four planar
//   Bézier-topped walls + planar bottom, sewn) and the SAME B as a BRepPrimAPI box, then
//   fused via BRepAlgoAPI_Fuse. The native FUSE (measured by the native M0 tessellator)
//   is compared to the OCCT fuse on:
//     * VOLUME     — native enclosedVolume vs BRepGProp (rel ≤ 2e-2); both cross-checked
//                    against the closed-form union V(B)+V(A∩{x≤0}).
//     * AREA       — native surfaceArea vs BRepGProp (rel ≤ 2e-2).
//     * WATERTIGHT — native mesh a closed 2-manifold; OCCT fuse a single closed solid.
//     * TOPOLOGY   — native mesh Euler χ = V−E+F = 2 (closed genus-0 solid).
//     * BBOX       — native vertex min/max vs BRepBndLib per-axis (abs ≤ 1.5·deflection).
//     * HAUSDORFF  — max native→OCCT vertex distance (spatial ≤ 1.5·deflection).
//     * CLASSIFY   — a query-point batch agrees with BRepClass3d_SolidClassifier on the
//                    native result with ZERO crisp IN↔OUT disagreements.
//   Plus the fallback contract: a box that never cuts A DECLINES to NULL (→ OCCT), never
//   a leaky solid.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-two-operand-freeform-boolean.sh.
//
#include "native/boolean/two_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/first_freeform_boolean_fixture.h"
#include "../native/two_operand_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_two_operand_freeform_boolean_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
namespace tox = two_operand_fixture;
namespace fx = face_split_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[NF2] %-14s %-20s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
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
  std::printf("== MOAT M2-FUSE first two-operand freeform boolean (FUSE): native-vs-OCCT parity ==\n");
  std::fflush(stdout);
  const double relTol = 0.02;

  // ── native operands ──────────────────────────────────────────────────────────
  const nt::Shape A = ffx::buildOperand();
  const nt::Shape Bx = tox::buildBoxB();

  // ── OCCT oracle: reconstruct A + B and fuse ──────────────────────────────────
  const TopoDS_Shape occtA = buildOcctOperand();
  report("occt-operand", "solid-built", occtSolidCount(occtA) == 1, "sewn 6-face solid");
  TopoDS_Solid occtB =
      BRepPrimAPI_MakeBox(gp_Pnt(tox::kX0, tox::kY0, tox::kZ0), gp_Pnt(tox::kX1, tox::kY1, tox::kZ1)).Solid();
  BRepAlgoAPI_Fuse fuser(occtA, occtB);
  fuser.Build();
  report("occt-fuse", "built", fuser.IsDone(), "BRepAlgoAPI_Fuse");
  const TopoDS_Shape occtFuse = fuser.Shape();
  report("occt-fuse", "single-solid", occtSolidCount(occtFuse) == 1, "one closed solid");

  const double vOcct = occtVolume(occtFuse);
  const double aOcct = occtArea(occtFuse);
  const double vClosed = tox::unionVolume();
  {
    const double relO = std::fabs(vOcct - vClosed) / vClosed;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "occt=%.6f closed=%.6f rel=%.3e", vOcct, vClosed, relO);
    report("volume", "occt-vs-closed", relO <= relTol, buf);
  }
  Bnd_Box occtBB;
  BRepBndLib::AddOptimal(occtFuse, occtBB, Standard_True, Standard_False);

  auto parityAt = [&](double defl) {
    char tag[24];
    std::snprintf(tag, sizeof(tag), "d=%.3f", defl);
    const double spatialTol = 1.5 * defl;

    bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
    const nt::Shape fused = bo::freeformBooleanTwoOperand(A, Bx, bo::TwoOperandOp::Fuse, defl, &why);
    {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s decline=%s", tag, bo::twoOperandDeclineName(why));
      report("native-fuse", "composed", why == bo::TwoOperandDecline::Ok && !fused.isNull(), buf);
    }
    if (fused.isNull()) return;

    ntess::MeshParams mp; mp.deflection = defl;
    const ntess::Mesh m = ntess::SolidMesher(mp).mesh(fused);
    report("native-fuse", "meshes", !m.triangles.empty(), tag);
    report("native-fuse", "watertight", ntess::isWatertight(m), tag);
    {
      const long chi = eulerChar(m);
      char buf[48];
      std::snprintf(buf, sizeof(buf), "%s euler=%ld (solid=2)", tag, chi);
      report("native-fuse", "topology-solid", chi == 2, buf);
    }

    const double vNat = std::fabs(ntess::enclosedVolume(m));
    const double aNat = ntess::surfaceArea(m);
    const BBox bNat = meshBBox(m);
    {
      const double rel = std::fabs(vNat - vOcct) / vOcct;
      char buf[120];
      std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, vNat, vOcct, rel);
      report("volume", "native-vs-occt", rel <= relTol, buf);
    }
    {
      const double rel = std::fabs(aNat - aOcct) / aOcct;
      char buf[104];
      std::snprintf(buf, sizeof(buf), "%s nat=%.6f occt=%.6f rel=%.3e", tag, aNat, aOcct, rel);
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
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%s worst=%.3e tol=%.3e", tag, worst, spatialTol);
      report("bbox", "native-vs-occt", worst <= spatialTol, buf);
    }
    {
      double worst = 0.0;
      for (const nm::Point3& v : m.vertices) {
        TopoDS_Vertex ov = BRepBuilderAPI_MakeVertex(P(v)).Vertex();
        BRepExtrema_DistShapeShape d(ov, occtFuse);
        if (d.IsDone() && d.NbSolution() > 0) worst = std::max(worst, static_cast<double>(d.Value()));
      }
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%s maxDist=%.3e tol=%.3e", tag, worst, spatialTol);
      report("hausdorff", "native->occt", worst <= spatialTol, buf);
    }
    // ── query-point batch: native mesh parity vs OCCT solid classifier ──────────
    // Native membership via the LANDED B3 classifier (multi-ray, grazing/ON-robust);
    // OCCT via BRepClass3d_SolidClassifier. Zero crisp IN↔OUT disagreements — a native
    // On/Unknown (its own fidelity band) or an OCCT ON abstains, never miscounts.
    {
      const bo::Aabb bb = bo::meshAabb(m);
      const int N = 12;
      int agree = 0, disagree = 0, band = 0, total = 0;
      for (int ix = 0; ix <= N; ++ix)
        for (int iy = 0; iy <= N; ++iy)
          for (int iz = 0; iz <= N; ++iz) {
            const double x = -0.5 + (tox::kX1 + 0.5) * (double)ix / N;
            const double y = tox::kY0 + (tox::kY1 - tox::kY0) * (double)iy / N;
            const double z = tox::kZ0 + (tox::kZ1 - tox::kZ0) * (double)iz / N;
            const nm::Point3 qp{x, y, z};
            ++total;
            gp_Pnt op(x, y, z);
            BRepClass3d_SolidClassifier cls(occtFuse, op, 1e-7);
            if (cls.State() == TopAbs_ON) { ++band; continue; }
            const bool occtIn = cls.State() == TopAbs_IN;
            const bo::Membership nv = bo::classifyPointInMesh(m, bb, defl, qp);
            if (nv != bo::Membership::In && nv != bo::Membership::Out) { ++band; continue; }
            ((nv == bo::Membership::In) == occtIn ? ++agree : ++disagree);
          }
      char buf[128];
      std::snprintf(buf, sizeof(buf), "%s agree=%d disagree=%d band=%d/%d", tag, agree, disagree, band, total);
      report("classify", "native-vs-occt", disagree == 0 && agree > 0, buf);
    }
  };

  parityAt(0.01);
  parityAt(0.005);

  // ── fallback contract: a box that never cuts A DECLINES to NULL (→ OCCT) ──────
  {
    auto p = [](double x, double y, double z) { return nm::Point3{x, y, z}; };
    std::vector<nt::Shape> faces;
    auto qf = [&](std::array<nm::Point3, 4> c, nm::Vec3 n) { faces.push_back(tox::quadFace(c, n)); };
    const double X0 = 10, X1 = 11, Y0 = -0.6, Y1 = 0.6, Z0 = -0.6, Z1 = 0.2;
    qf({p(X0, Y0, Z0), p(X0, Y0, Z1), p(X0, Y1, Z1), p(X0, Y1, Z0)}, {-1, 0, 0});
    qf({p(X1, Y0, Z0), p(X1, Y1, Z0), p(X1, Y1, Z1), p(X1, Y0, Z1)}, {1, 0, 0});
    qf({p(X0, Y0, Z0), p(X1, Y0, Z0), p(X1, Y0, Z1), p(X0, Y0, Z1)}, {0, -1, 0});
    qf({p(X0, Y1, Z0), p(X0, Y1, Z1), p(X1, Y1, Z1), p(X1, Y1, Z0)}, {0, 1, 0});
    qf({p(X0, Y0, Z0), p(X0, Y1, Z0), p(X1, Y1, Z0), p(X1, Y0, Z0)}, {0, 0, -1});
    qf({p(X0, Y0, Z1), p(X1, Y0, Z1), p(X1, Y1, Z1), p(X0, Y1, Z1)}, {0, 0, 1});
    const nt::Shape farBox = nt::ShapeBuilder::makeSolid({nt::ShapeBuilder::makeShell(std::move(faces))});
    bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
    const nt::Shape r = bo::freeformBooleanTwoOperand(A, farBox, bo::TwoOperandOp::Fuse, 0.01, &why);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "decline=%s", bo::twoOperandDeclineName(why));
    report("fallback", "declines-null", r.isNull() && why == bo::TwoOperandDecline::SeamDeclined, buf);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
