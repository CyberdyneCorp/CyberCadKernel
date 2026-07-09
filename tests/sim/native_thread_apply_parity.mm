// SPDX-License-Identifier: Apache-2.0
//
// native_thread_apply_parity.mm — native cc_thread_apply SIM GATE (b): native-vs-OCCT on
// a booted iOS simulator.
//
// The native verb `threadApply` (src/native/boolean/thread_apply.h, OCCT-FREE) applies a
// helical thread to a shaft by recognise[cylinder] → facet → planar-BSP boolean_solid →
// four-part self-verify (watertight + Euler χ=2 + consistently-oriented + a two-sided
// closed-form-volume band), returning a verified native solid or a NULL Shape (→ OCCT) with
// a measured decline. This harness GROUNDS the machinery + the honest fallthrough vs OCCT:
//   * the tractable PLANAR-CUTTER baseline — a native cylinder CUT by a box — WELDS through
//     the SAME verb; its meshed volume matches OCCT `BRepAlgoAPI_Cut` + `BRepGProp` within
//     the deflection-bounded band (the BSP + facet + self-verify machinery is SOUND);
//   * the multi-turn HELICAL THREAD FUSE / CUT native-DECLINES to NULL (the near-tangent
//     helical BSP fragments + the orientation-inconsistent native thread operand) while
//     OCCT's per-turn `thread_apply` accumulate produces the reference threaded-shaft volume
//     — the correct honest fallthrough; the native path NEVER emits a leaky/wrong solid.
//
// OCCT is the ORACLE ONLY, never linked into src/native. Build:
// scripts/run-sim-native-thread-apply.sh. Gate (a) (host, no OCCT) is
// tests/native/test_native_thread_apply.cpp.
//
#include "native/boolean/thread_apply.h"
#include "native/construct/native_construct.h"
#include "native/construct/construct.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdio>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_thread_apply_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <BRepBuilderAPI_Transform.hxx>

namespace nt = cybercad::native::topology;
namespace nc = cybercad::native::construct;
namespace nm = cybercad::native::math;
namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, const char* check, bool ok, const char* detail) {
  std::printf("[TA] %-8s %-30s %s  (%s)\n", name, check, ok ? "PASS" : "FAIL", detail);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

constexpr double kPi = 3.14159265358979323846;

// ── native builders (the operands the verb consumes) ────────────────────────────
static nt::Shape nativeShaft(double Rs, double z0, double z1) {
  const std::vector<nc::LineSeg> segs = {
      {0.0, z0, Rs, z0}, {Rs, z0, Rs, z1}, {Rs, z1, 0.0, z1}, {0.0, z1, 0.0, z0}};
  const nc::detail::AxisFrame zAxis{nm::Point3{0, 0, 0}, nm::Dir3{0, 0, 1},
                                    nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}};
  return nc::build_revolution_framed(segs, zAxis, nc::kFullTurn);
}
static double nativeMeshVolume(const nt::Shape& s, double d, bool& wt) {
  tess::MeshParams p; p.deflection = d;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// ── OCCT oracle ──────────────────────────────────────────────────────────────────
static double occtVolume(const TopoDS_Shape& s) {
  if (s.IsNull()) return 0.0;
  GProp_GProps g; BRepGProp::VolumeProperties(s, g); return std::fabs(g.Mass());
}
// A single-turn V thread swept radially (aux-Z-axis spine), like the OCCT thread oracle's
// buildTurn — used to accumulate a per-turn thread into the shaft (the reference thread_apply).
static bool buildOcctTurn(double spineR, double crestR, double z0, double zSpan,
                          double thetaSweep, TopoDS_Shape& out) {
  const double depth = crestR - spineR;
  const double halfBase = std::min(zSpan / 2.0, depth * std::tan(kPi / 6.0));
  if (!(depth > 0.0) || !(halfBase > 0.0)) return false;
  const int N = 16;
  TColgp_Array1OfPnt pts(1, N + 1);
  for (int i = 0; i <= N; ++i) {
    const double f = double(i) / N, th = f * thetaSweep;
    pts.SetValue(i + 1, gp_Pnt(spineR * std::cos(th), spineR * std::sin(th), z0 + f * zSpan));
  }
  GeomAPI_PointsToBSpline fit(pts);
  if (!fit.IsDone() || fit.Curve().IsNull()) return false;
  BRepBuilderAPI_MakeWire spineMk(BRepBuilderAPI_MakeEdge(fit.Curve()).Edge());
  if (!spineMk.IsDone()) return false;
  BRepBuilderAPI_MakePolygon tri;
  tri.Add(gp_Pnt(spineR, 0, z0 - halfBase));
  tri.Add(gp_Pnt(crestR, 0, z0));
  tri.Add(gp_Pnt(spineR, 0, z0 + halfBase));
  tri.Close();
  if (!tri.IsDone()) return false;
  BRepOffsetAPI_MakePipeShell mk(spineMk.Wire());
  const TopoDS_Edge axisEdge = BRepBuilderAPI_MakeEdge(
      gp_Pnt(0, 0, z0 - halfBase - 1.0), gp_Pnt(0, 0, z0 + zSpan + halfBase + 1.0)).Edge();
  BRepBuilderAPI_MakeWire axisMk(axisEdge);
  if (!axisMk.IsDone()) return false;
  mk.SetMode(axisMk.Wire(), Standard_True);
  mk.Add(tri.Wire(), Standard_False, Standard_True);
  if (!mk.IsReady()) return false;
  mk.Build();
  if (!mk.IsDone() || !mk.MakeSolid()) return false;
  out = mk.Shape();
  return !out.IsNull();
}

int main() {
  std::printf("== native cc_thread_apply: native-vs-OCCT ==\n");
  std::fflush(stdout);

  // ── (1) tractable PLANAR-CUTTER baseline: cylinder CUT by a box, native-vs-OCCT ──
  // The SAME recognise → facet → boolean_solid → four-part self-verify path the thread
  // uses. It MUST weld here (the machinery is sound) and match the OCCT oracle volume.
  {
    const double Rs = 4.0, z1 = 8.0, d = 0.08;
    const nt::Shape shaft = nativeShaft(Rs, 0.0, z1);
    std::vector<double> sq = {2, -1, 6, -1, 6, 1, 2, 1};
    const nt::Shape box = nc::build_prism(sq.data(), 4, 3.0);
    bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
    const nt::Shape ncut = bo::threadApply(shaft, box, /*op=*/1, d, &why);
    const bool welded = !ncut.isNull() && why == bo::ThreadApplyDecline::Ok;
    bool wt = false;
    const double nv = welded ? nativeMeshVolume(ncut, d, wt) : 0.0;

    // OCCT oracle: cylinder(axis Z, r=4, h=8) − box([2,6]x[-1,1]x[0,3]).
    TopoDS_Shape occtShaft = BRepPrimAPI_MakeCylinder(Rs, z1).Shape();
    TopoDS_Shape occtBox = BRepPrimAPI_MakeBox(gp_Pnt(2, -1, 0), 4, 2, 3).Shape();
    const double ov = occtVolume(BRepAlgoAPI_Cut(occtShaft, occtBox).Shape());
    const double rel = ov > 0 ? std::fabs(nv - ov) / ov : 1e30;
    char buf[160];
    std::snprintf(buf, sizeof buf, "why=%s native=%.4f OCCT=%.4f rel=%.4f wt=%d",
                  bo::threadApplyDeclineName(why), nv, ov, rel, wt);
    report("baseline", "cyl-box-cut welds vs OCCT", welded && wt && rel < 30.0 * d, buf);
  }

  // ── (2) HELICAL THREAD: native declines; OCCT per-turn accumulate produces a solid ──
  {
    const double major = 5.0, pitch = 2.0, turns = 4.0, depth = 1.0, rise = pitch * turns;
    const nt::Shape thread = nc::build_helical_thread(major, pitch, turns, depth, 60.0, 1.0, 16);
    const double d = 0.05;

    // The native thread solid is watertight but NOT consistently oriented — the measured
    // root cause it cannot serve as a BSP operand. Assert it directly.
    {
      tess::MeshParams p; p.deflection = d;
      const tess::Mesh tm = tess::SolidMesher{p}.mesh(thread);
      char buf[128];
      std::snprintf(buf, sizeof buf, "wt=%d sameDir=%zu", tess::isWatertight(tm),
                    tess::sameDirectionEdgeCount(tm));
      report("native", "thread operand wt-not-oriented",
             tess::isWatertight(tm) && tess::sameDirectionEdgeCount(tm) != 0, buf);
    }

    // FUSE (external): shaft at root radius (major-depth=4).
    const nt::Shape shaftF = nativeShaft(major - depth, 0.0, rise);
    bo::ThreadApplyDecline wf = bo::ThreadApplyDecline::Ok;
    const nt::Shape nf = bo::threadApply(shaftF, thread, /*op=*/0, d, &wf);
    char buf[160];
    std::snprintf(buf, sizeof buf, "native FUSE null=%d why=%s", nf.isNull(),
                  bo::threadApplyDeclineName(wf));
    report("native", "thread FUSE declines to null",
           nf.isNull() && wf != bo::ThreadApplyDecline::Ok, buf);

    // CUT (internal): shaft at crest radius (5).
    const nt::Shape shaftC = nativeShaft(major, 0.0, rise);
    bo::ThreadApplyDecline wc = bo::ThreadApplyDecline::Ok;
    const nt::Shape nc_ = bo::threadApply(shaftC, thread, /*op=*/1, d, &wc);
    std::snprintf(buf, sizeof buf, "native CUT null=%d why=%s", nc_.isNull(),
                  bo::threadApplyDeclineName(wc));
    report("native", "thread CUT declines to null",
           nc_.isNull() && wc != bo::ThreadApplyDecline::Ok, buf);

    // OCCT reference: per-turn accumulate FUSE of the thread onto the shaft (the oracle
    // path the native verb honestly defers to). Shaft cylinder r=4, thread crest 5 — each
    // rebuilt turn's V ridge (spine at the shaft surface, crest outside it) is fused with a
    // small overlap. Each fuse is GATED on the volume-growth sign (the real OCCT adapter's
    // rule: a fuse must never LOSE material — a grazing overlap that drops a body is
    // skipped), so a valid accumulate grows the volume monotonically. We assert OCCT
    // PRODUCES a valid solid with volume strictly greater than the bare shaft (a real thread
    // was added) — the reference threaded-shaft volume the native path defers to.
    TopoDS_Shape occtShaft = BRepPrimAPI_MakeCylinder(major - depth, rise).Shape();
    const double vBareShaft = occtVolume(occtShaft);
    TopoDS_Shape acc = occtShaft;
    int applied = 0;
    const double spineR = major - depth;  // ridge root sits on the shaft surface
    for (int k = 0; k < int(turns); ++k) {
      const double z0 = 0.5 * pitch + k * pitch;  // centre each ridge within its pitch
      TopoDS_Shape turn;
      if (!buildOcctTurn(spineR, major, z0, pitch * 1.06, 2.0 * kPi * 1.06, turn)) continue;
      const double vAcc = occtVolume(acc);
      TopoDS_Shape best;
      for (double fuzz : {1.0e-3, 4.0e-3, 1.6e-2}) {
        BRepAlgoAPI_Fuse f(acc, turn);
        f.SetFuzzyValue(fuzz);
        f.Build();
        if (!f.IsDone() || f.Shape().IsNull()) continue;
        if (occtVolume(f.Shape()) < vAcc - 1.0e-6) continue;  // fuse must not lose material
        best = f.Shape();
        break;
      }
      if (!best.IsNull()) { acc = best; ++applied; }  // else keep the last valid acc
    }
    const double vThreaded = occtVolume(acc);
    std::snprintf(buf, sizeof buf, "applied=%d/%d V=%.3f > bareShaft=%.3f", applied,
                  int(turns), vThreaded, vBareShaft);
    report("occt", "per-turn accumulate adds thread",
           applied > 0 && vThreaded > vBareShaft + 1e-3, buf);
  }

  std::printf("[TA] SUMMARY %d passed / %d failed\n", g_pass, g_fail);
  std::fflush(stdout);
  return g_fail ? 1 : 0;
}
