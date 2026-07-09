// SPDX-License-Identifier: Apache-2.0
//
// native_slab_disjoint_cut_parity.mm — MOAT M2b freeform↔analytic DISJOINT (MULTI-LUMP)
// CUT SIM GATE (b): native-vs-OCCT on a booted iOS simulator.
//
// The native verb `freeformSlabDisjointCut` (src/native/boolean/slab_disjoint_cut.h,
// OCCT-FREE) parts a bowl-lidded convex-quad prism `A` with a central axis-aligned slab
// `B` into TWO lumps, composing recognise[B1] → slab-pair → per-lump inter-solid-seam weld
// → disjoint-check → TWO-SIDED self-verify, and returns EITHER a `Compound` of two
// watertight `Solid`s (upper-bound mode) OR — with the closed-form volume supplied —
// HONEST-DECLINES to NULL when the byte-frozen keep-face machinery over-estimates the
// OFF-CENTRE cross-section volume. This harness GROUNDS both facets against OCCT:
//   * OCCT's `BRepAlgoAPI_Cut(A, B)` (the ORACLE) yields a compound of EXACTLY TWO solids
//     (the genuine disjoint parting) whose total volume matches the closed form
//     V(A∩{x≤−s}) + V(A∩{x≥+s});
//   * the native verb's DISJOINT MECHANISM matches OCCT's TOPOLOGY — a two-solid compound,
//     watertight, disjoint along the slab axis — the new outcome no landed native verb
//     produces;
//   * the native verb's TWO-SIDED self-verify HONEST-DECLINES (VolumeInconsistent) because
//     its meshed volume exceeds the OCCT/closed-form value beyond the deflection band, so
//     OCCT (the oracle) owns the correct-volume result; the native path NEVER emits a
//     wrong/leaky solid.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-slab-disjoint-cut.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_slab_disjoint_cut.cpp.
//
#include "native/boolean/slab_disjoint_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include "../native/slab_disjoint_cut_fixture.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_slab_disjoint_cut_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
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
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepLib.hxx>

namespace bo = cybercad::native::boolean;
namespace nt = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace fx = face_split_fixture;
namespace sfx = slab_disjoint_cut_fixture;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[SD] %-14s %-24s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
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
  if (s.IsNull()) return 0.0;
  GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
static int occtSolidCount(const TopoDS_Shape& s) {
  int n = 0;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++n;
  return n;
}
static int nativeSolidCount(const nt::Shape& s) {
  int n = 0;
  for (nt::Explorer ex(s, nt::ShapeType::Solid); ex.more(); ex.next()) ++n;
  return n;
}

int main() {
  std::printf("== MOAT M2b freeform<->analytic DISJOINT (multi-lump) CUT: native-vs-OCCT ==\n");
  std::fflush(stdout);
  const double vrel = 0.02;  // OCCT-vs-closed-form volume band (curved-cup faceting)

  // ── OCCT oracle: A (sewn 6-face prism) − B (central slab) ────────────────────────
  const TopoDS_Shape occtA = buildOcctOperand();
  report("occt", "operand-built", occtSolidCount(occtA) == 1, "sewn 6-face solid");
  TopoDS_Solid occtB =
      BRepPrimAPI_MakeBox(gp_Pnt(-sfx::kS, -sfx::kLat, -sfx::kLat),
                          gp_Pnt(sfx::kS, sfx::kLat, sfx::kLat)).Solid();

  const double vA = occtVolume(occtA);
  {
    char buf[96];
    std::snprintf(buf, sizeof buf, "V(A)=%.5f cf=%.5f", vA, ffx::fullVolume());
    report("occt", "VA-matches-closed-form", std::fabs(vA - ffx::fullVolume()) / ffx::fullVolume() < vrel, buf);
  }

  BRepAlgoAPI_Cut cutter(occtA, occtB);
  cutter.Build();
  report("occt", "cut-built", cutter.IsDone(), "BRepAlgoAPI_Cut");
  const TopoDS_Shape occtCut = cutter.Shape();

  // The ORACLE: the CUT parts A into EXACTLY TWO disjoint solids at the closed-form volume.
  char buf[160];
  const int occtN = occtSolidCount(occtCut);
  std::snprintf(buf, sizeof buf, "solids=%d (want 2)", occtN);
  report("occt", "cut-is-two-bodies", occtN == 2, buf);
  const double vCut = occtVolume(occtCut);
  const double cf = sfx::cutVolume();
  std::snprintf(buf, sizeof buf, "V=%.6f cf=%.6f", vCut, cf);
  report("occt", "cut-matches-closed-form", std::fabs(vCut - cf) / cf < vrel, buf);

  // ── native operands ─────────────────────────────────────────────────────────────
  const nt::Shape A = ffx::buildOperand();
  const nt::Shape Bslab = sfx::buildSlabB();

  for (double d : {0.01, 0.008, 0.006}) {
    std::snprintf(buf, sizeof buf, "d=%.3f", d);
    const char* tag = buf;

    // (1) DISJOINT MECHANISM (upper-bound mode) — matches OCCT's TWO-body topology,
    //     watertight, meshable.
    bo::SlabCutDecline wm = bo::SlabCutDecline::Ok;
    const nt::Shape mech = bo::freeformSlabDisjointCut(A, Bslab, d, &wm);  // no closed form
    char b2[160];
    std::snprintf(b2, sizeof b2, "%s decline=%s solids=%d", tag, bo::slabCutDeclineName(wm),
                  mech.isNull() ? -1 : nativeSolidCount(mech));
    const bool mechOk = !mech.isNull() && wm == bo::SlabCutDecline::Ok && nativeSolidCount(mech) == 2;
    report("native", "mechanism-two-bodies", mechOk, b2);
    if (mechOk) {
      ntess::MeshParams mp; mp.deflection = d;
      const ntess::Mesh m = ntess::SolidMesher(mp).mesh(mech);
      std::snprintf(b2, sizeof b2, "%s wt=%d v=%.5f occt=%.5f", tag, ntess::isWatertight(m),
                    std::fabs(ntess::enclosedVolume(m)), vCut);
      report("native", "mechanism-watertight", ntess::isWatertight(m), b2);
    }

    // (2) HONEST TWO-SIDED DECLINE — with the closed form supplied, the off-centre
    //     over-estimate is rejected → NULL → OCCT owns the correct-volume result.
    bo::SlabCutDecline wv = bo::SlabCutDecline::Ok;
    const nt::Shape verified = bo::freeformSlabDisjointCut(A, Bslab, d, &wv, cf);
    std::snprintf(b2, sizeof b2, "%s null=%d decline=%s", tag, verified.isNull(),
                  bo::slabCutDeclineName(wv));
    report("native", "two-sided-declines", verified.isNull() &&
               wv == bo::SlabCutDecline::VolumeInconsistent, b2);
  }

  // (3) the native over-estimate is a REAL blocker: the mechanism volume exceeds OCCT's
  //     correct CUT volume beyond the band (proving the decline is measured, not spurious).
  {
    bo::SlabCutDecline w = bo::SlabCutDecline::Ok;
    const nt::Shape mech = bo::freeformSlabDisjointCut(A, Bslab, 0.008, &w);
    double v = 0.0;
    if (!mech.isNull()) {
      ntess::MeshParams mp; mp.deflection = 0.008;
      v = std::fabs(ntess::enclosedVolume(ntess::SolidMesher(mp).mesh(mech)));
    }
    std::snprintf(buf, sizeof buf, "native=%.5f OCCT=%.5f over=%.1f%%", v, vCut,
                  100.0 * (v - vCut) / vCut);
    report("native", "overestimate-vs-occt", v > vCut * 1.10, buf);
  }

  std::printf("[SD] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
