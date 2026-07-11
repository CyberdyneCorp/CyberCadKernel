// SPDX-License-Identifier: Apache-2.0
//
// test_native_nurbs_plane_split.cpp — NURBS roadmap LAYER 3, SLICE 1 (L3-S1) host
// GATE: the OCCT-free closed-form proof of the FIRST exact-NURBS B-rep boolean —
// a genuine NURBS face SPLIT BY A PLANE (nurbs_plane_split.h).
//
// A NURBS-walled bowl-cup (a genuine Kind::BSpline degree-2 bowl wall trimmed by a rim
// circle + a flat top-lid) cut by the HORIZONTAL plane z=c has a CLOSED CIRCULAR seam
// on the NURBS wall. `nurbsFacePlaneSplit` composes NURBS-adapter trace[stage 1] →
// WLine-(u,v)-read fidelity gate[stage 2] → splitFaceSmoothTrim[stage 3] → half-space
// keep[stage 4] → flat cap synth + M0 curved↔flat weld[stage 5] → watertight+volume
// self-verify, into the CUT (Below) and COMMON (Above) keep sides.
//
// GATES (readiness doc):
//   1. CORRECTNESS vs a KNOWN oracle — the kept enclosed volume matches the closed
//      form (CUT π·ρ²·c/2, COMMON V(full)−that) as the mesh refines, with the
//      partition identity V(below)+V(above)=V(full).
//   2. DISAGREED==0 / no silent-wrong — the kept sub-face's boundary lies on the
//      intersection curve (seam fidelity S(u,v)==C AND on-both-surfaces ≤ tol), the
//      weld is watertight, and a case that can't be split/welded HONEST-DECLINES
//      (NULL + a measured reason), never a wrong result.
//
// The SIM leg vs OCCT BRepAlgoAPI_Cut is the sim GATE (b) (see the .mm sibling);
// this host leg carries the analytic ground truth so the slice is proven without OCCT.
//
// Exits 0 iff every gate holds; 1 on any failure. Requires CYBERCAD_HAS_NUMSCI (the
// M1 seam trace); with the substrate off it is a clean SKIP.
//
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

#include "native/math/bspline.h"
#include "native/math/vec.h"

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/boolean/nurbs_plane_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "nurbs_plane_split_fixture.h"
#endif

namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}
static void expectNear(double got, double want, double tol, const char* what) {
  ++g_checks;
  const double e = std::fabs(got - want);
  if (!(e <= tol)) {
    std::printf("FAIL: %s: got %.9g want %.9g |err|=%.3g > tol %.3g\n", what, got, want, e, tol);
    ++g_failures;
  } else {
    std::printf("  ok  %-42s got %.9g want %.9g |err|=%.3g\n", what, got, want, e);
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)
namespace bool_ = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace fx = nurbs_plane_split_fixture;

// Evaluate the fixture's NURBS bowl surface at (u,v) via native-math (non-rational).
static math::Point3 evalBowl(double u, double v) {
  const topo::FaceSurface s = fx::bowlSurface();
  math::SurfaceGrid grid{std::span<const math::Point3>(s.poles.data(), s.poles.size()),
                         s.nPolesU, s.nPolesV};
  return math::surfacePoint(s.degreeU, s.degreeV, grid, {s.knotsU.data(), s.knotsU.size()},
                            {s.knotsV.data(), s.knotsV.size()}, u, v);
}

// Gate 0 — ORACLE SANITY: the genuine NURBS (BSpline) surface reproduces the paraboloid
// z = a·((u−½)²+(v−½)²) EXACTLY (a clamped degree-2 B-spline reproduces a quadratic), so
// the closed-form volume oracle is exact on THIS surface (not a fit).
static void gate0_surface_is_exact_paraboloid() {
  std::printf("\n== GATE 0: NURBS surface reproduces the exact paraboloid ==\n");
  double maxDev = 0.0;
  for (int i = 0; i <= 10; ++i)
    for (int j = 0; j <= 10; ++j) {
      const double u = i / 10.0, v = j / 10.0;
      const math::Point3 p = evalBowl(u, v);
      const double x = u - 0.5, y = v - 0.5;
      const double zExact = fx::kA * (x * x + y * y);
      maxDev = std::max(maxDev, std::fabs(p.z - zExact));
      maxDev = std::max(maxDev, std::hypot(p.x - x, p.y - y));
    }
  std::printf("  NURBS-vs-paraboloid max dev = %.3g\n", maxDev);
  expectTrue(maxDev < 1e-12, "NURBS BSpline surface reproduces the paraboloid to machine eps");
}

// A oracle-mesh enclosed volume of the FULL bowl-cup (sanity that the operand is a
// well-formed closed solid whose meshed volume converges to π·a·R⁴/2).
static double meshedVolume(const topo::Shape& solid, double defl) {
  tess::MeshParams mp;
  mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  return tess::isWatertight(m) ? tess::enclosedVolume(m) : -1.0;
}

// The core L3-S1 gate: CUT (Below) + COMMON (Above), each against its closed-form
// volume, watertight, seam on the curve, DISAGREED=0, and the partition identity.
static void gate1_cut_and_common() {
  std::printf("\n== GATE 1: exact-NURBS face split by plane — CUT + COMMON ==\n");
  const fx::Operand op = fx::buildOperand();
  const math::Plane P = fx::cutPlane();

  // Oracle sanity: the full operand meshes watertight and its volume tracks π·a·R⁴/2.
  const double vFullMesh = meshedVolume(op.solid, 0.004);
  std::printf("  full bowl-cup meshed volume = %.6g (closed form %.6g)\n", vFullMesh,
              fx::fullVolume());
  expectTrue(vFullMesh > 0.0, "full bowl-cup operand meshes watertight");
  expectNear(vFullMesh, fx::fullVolume(), 0.06 * fx::fullVolume(), "full volume ~ pi*a*R^4/2");

  const double defl = 0.0025;

  // ── CUT (KeepSide::Below): keep the cup below z=c. V = π·ρ²·c/2. ──────────────
  const bool_::NurbsPlaneSplitResult cut =
      bool_::nurbsFacePlaneSplit(op.wall, op.base, P, bool_::KeepSide::Below, defl);
  std::printf("  CUT   decline=%s seamNodes=%d fidelity=%.3g onSurf=%.3g tiling=%.3g wt=%d V=%.6g\n",
              bool_::nurbsPlaneSplitDeclineName(cut.decline), cut.seamNodes, cut.seamFidelity,
              cut.seamOnSurf, cut.tilingGap, cut.watertight ? 1 : 0, cut.enclosedVolume);
  expectTrue(cut.ok(), "CUT (Below) returns a verified keep-side solid");
  expectTrue(cut.watertight, "CUT weld is watertight (M0 closed 2-manifold)");
  // DISAGREED=0: the kept sub-face's boundary lies on the intersection curve (both F and P).
  expectTrue(cut.seamFidelity < 1e-6, "CUT seam pcurve round-trips on the NURBS face (S(u,v)==C)");
  expectTrue(cut.seamOnSurf < 1e-6, "CUT seam lies on BOTH the NURBS face and the plane");
  expectTrue(cut.tilingGap < 1e-6, "CUT smooth-trim tiles (areaInside+areaOutside==parent)");
  expectNear(cut.enclosedVolume, fx::cutVolume(), 0.06 * fx::cutVolume(),
             "CUT enclosed volume ~ pi*rho^2*c/2 (closed form)");

  // ── COMMON (KeepSide::Above): keep the bowl above z=c + lid. V = V(full)−cut. ─
  const bool_::NurbsPlaneSplitResult com =
      bool_::nurbsFacePlaneSplit(op.wall, op.base, P, bool_::KeepSide::Above, defl);
  std::printf("  COMMON decline=%s seamNodes=%d fidelity=%.3g onSurf=%.3g tiling=%.3g wt=%d V=%.6g\n",
              bool_::nurbsPlaneSplitDeclineName(com.decline), com.seamNodes, com.seamFidelity,
              com.seamOnSurf, com.tilingGap, com.watertight ? 1 : 0, com.enclosedVolume);
  expectTrue(com.ok(), "COMMON (Above) returns a verified keep-side solid");
  expectTrue(com.watertight, "COMMON weld is watertight");
  expectTrue(com.seamFidelity < 1e-6, "COMMON seam pcurve round-trips on the NURBS face");
  expectTrue(com.seamOnSurf < 1e-6, "COMMON seam lies on both the NURBS face and the plane");
  expectNear(com.enclosedVolume, fx::commonVolume(), 0.06 * fx::commonVolume(),
             "COMMON enclosed volume ~ V(full)-cut (closed form)");

  // ── PARTITION IDENTITY: V(below) + V(above) = V(full), from the SAME meshes. ──
  if (cut.ok() && com.ok()) {
    const double part = cut.enclosedVolume + com.enclosedVolume;
    expectNear(part, fx::fullVolume(), 0.06 * fx::fullVolume(),
               "partition closure V(below)+V(above)=V(full)");
  }
}

// Gate 2 — HONEST DECLINE (no silent-wrong): cases that cannot be split/welded must
// return a NULL solid with a MEASURED reason, never a fabricated result.
static void gate2_honest_declines() {
  std::printf("\n== GATE 2: honest declines (NULL + measured reason, no silent-wrong) ==\n");
  const fx::Operand op = fx::buildOperand();

  // (a) A plane ABOVE the whole bowl-cup (z > a·R²): no seam on the wall → SeamUnusable.
  {
    math::Ax3 fr;
    fr.origin = math::Point3{0, 0, fx::kRimZ + 0.5};  // above the entire cup
    fr.z = math::Dir3{math::Vec3{0, 0, 1}};
    const math::Plane hi{fr};
    const bool_::NurbsPlaneSplitResult res =
        bool_::nurbsFacePlaneSplit(op.wall, op.base, hi, bool_::KeepSide::Below, 0.01);
    std::printf("  above-cup plane: decline=%s solid-null=%d\n",
                bool_::nurbsPlaneSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    // No-silent-wrong: a plane that does not cut the wall in a usable closed interior
    // seam must return NULL with a measured reason (SeamUnusable if nothing traced, or
    // SmoothSplitFailed if a spurious/boundary WLine failed the closed-loop split) —
    // NEVER a fabricated solid. Either honest decline is correct.
    expectTrue(!res.ok() && res.solid.isNull(), "above-cup plane HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsPlaneSplitDecline::SeamUnusable ||
                   res.decline == bool_::NurbsPlaneSplitDecline::SmoothSplitFailed ||
                   res.decline == bool_::NurbsPlaneSplitDecline::SeamOffSurface,
               "above-cup decline is a measured no-seam reason (never a wrong solid)");
  }

  // (b) Swapping the wall for the flat base (a non-NURBS wall) → WallNotNurbs decline.
  {
    const math::Plane P = fx::cutPlane();
    const bool_::NurbsPlaneSplitResult res =
        bool_::nurbsFacePlaneSplit(op.base, op.base, P, bool_::KeepSide::Below, 0.01);
    std::printf("  non-NURBS wall: decline=%s solid-null=%d\n",
                bool_::nurbsPlaneSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "non-NURBS wall HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsPlaneSplitDecline::WallNotNurbs,
               "non-NURBS wall decline is WallNotNurbs");
  }
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("# L3-S1 EXACT-NURBS FACE SPLIT BY PLANE — host closed-form gate (OCCT-free)\n");
#if defined(CYBERCAD_HAS_NUMSCI)
  gate0_surface_is_exact_paraboloid();
  gate1_cut_and_common();
  gate2_honest_declines();
  std::printf("\n# checks=%d failures=%d\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("# RESULT: L3-S1 exact-NURBS face split by plane — ALL GATES GREEN\n");
    return 0;
  }
  std::printf("# RESULT: %d FAILURE(S)\n", g_failures);
  return 1;
#else
  std::printf("# SKIPPED (CYBERCAD_HAS_NUMSCI OFF): the M1 seam trace is substrate-gated.\n");
  return 0;
#endif
}
