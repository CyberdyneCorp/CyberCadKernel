// SPDX-License-Identifier: Apache-2.0
//
// test_native_nurbs_curved_split.cpp — NURBS roadmap LAYER 3, SLICE 2 (L3-S2) host
// GATE: the OCCT-free closed-form proof of the SECOND exact-NURBS B-rep boolean —
// a genuine NURBS face SPLIT BY A CURVED (analytic) FACE (nurbs_curved_split.h).
//
// A NURBS-walled bowl (a genuine Kind::BSpline degree-2 paraboloid wall trimmed by a
// rim circle + a flat top-lid) cut by a SPHERE has a CLOSED CIRCULAR seam on the NURBS
// wall — a curve on BOTH curved surfaces. `nurbsFaceCurvedSplit` composes NURBS-adapter
// ∩ sphere-adapter trace[stage 1] → WLine-(u,v)-read fidelity gate on BOTH F and G
// [stage 2] → splitFaceSmoothTrim[stage 3] → CURVED-solid membership keep[stage 4] →
// curved-G cap fan synth + M0 curved↔CURVED weld[stage 5] → watertight+volume self-
// verify, into the CUT (Below, the closed-form LENS) and COMMON (Above) keep sides.
//
// GATES (readiness doc):
//   1. CORRECTNESS vs a KNOWN oracle — the CUT (Below) kept enclosed volume matches the
//      closed-form lens V_lens = 2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}] as
//      the mesh refines (a deflection-bounded curved cap, O(deflection) convergence).
//   2. DISAGREED==0 / no silent-wrong — the seam lies on BOTH curved surfaces (fidelity
//      S_F(u,v)==C AND S_G(u,v)==C AND on-both-surfaces ≤ tol), the curved↔curved weld is
//      watertight (Euler χ=2), and a case that can't be split/welded HONEST-DECLINES
//      (NULL + a measured reason), never a wrong result.
//
// The SIM leg vs OCCT BRepAlgoAPI_Cut is the sim GATE (b) (see the .mm sibling); this
// host leg carries the analytic ground truth so the slice is proven without OCCT.
//
// Exits 0 iff every gate holds; 1 on any failure. Requires CYBERCAD_HAS_NUMSCI (the M1
// seam trace); with the substrate off it is a clean SKIP.
//
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

#include "native/math/bspline.h"
#include "native/math/vec.h"

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/boolean/nurbs_curved_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "nurbs_curved_split_fixture.h"
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
    std::printf("  ok  %-46s got %.9g want %.9g |err|=%.3g\n", what, got, want, e);
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)
namespace bool_ = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace fx = nurbs_curved_split_fixture;

// Evaluate the fixture's NURBS bowl surface at (u,v) via native-math (non-rational).
static math::Point3 evalBowl(double u, double v) {
  const topo::FaceSurface s = fx::bowlSurface();
  math::SurfaceGrid grid{std::span<const math::Point3>(s.poles.data(), s.poles.size()),
                         s.nPolesU, s.nPolesV};
  return math::surfacePoint(s.degreeU, s.degreeV, grid, {s.knotsU.data(), s.knotsU.size()},
                            {s.knotsV.data(), s.knotsV.size()}, u, v);
}

// Euler characteristic of a closed mesh (χ = V − E + F = 2 for a genus-0 solid).
static long eulerChar(const tess::Mesh& m) {
  return static_cast<long>(m.vertices.size()) - static_cast<long>(tess::edgeUseCounts(m).size()) +
         static_cast<long>(m.triangles.size());
}

// Gate 0 — ORACLE SANITY: the genuine NURBS (BSpline) surface reproduces the paraboloid
// z = a·(x²+y²), x=(u−½)·2H, y=(v−½)·2H EXACTLY (a clamped degree-2 B-spline reproduces a
// quadratic), so the closed-form lens oracle is exact on THIS surface (not a fit).
static void gate0_surface_is_exact_paraboloid() {
  std::printf("\n== GATE 0: NURBS surface reproduces the exact paraboloid ==\n");
  double maxDev = 0.0;
  for (int i = 0; i <= 10; ++i)
    for (int j = 0; j <= 10; ++j) {
      const double u = i / 10.0, v = j / 10.0;
      const math::Point3 p = evalBowl(u, v);
      const double x = (u - 0.5) * 2.0 * fx::kH, y = (v - 0.5) * 2.0 * fx::kH;
      const double zExact = fx::kA * (x * x + y * y);
      maxDev = std::max(maxDev, std::fabs(p.z - zExact));
      maxDev = std::max(maxDev, std::hypot(p.x - x, p.y - y));
    }
  std::printf("  NURBS-vs-paraboloid max dev = %.3g\n", maxDev);
  expectTrue(maxDev < 1e-12, "NURBS BSpline surface reproduces the paraboloid to machine eps");
}

// The core L3-S2 gate: CUT (Below, the LENS) against its closed-form volume as the mesh
// refines, watertight χ=2, seam on BOTH curved surfaces, DISAGREED=0; plus COMMON (Above)
// as a verified watertight complementary keep side.
static void gate1_lens_cut_and_common() {
  std::printf("\n== GATE 1: exact-NURBS face split by a SPHERE — LENS (CUT) + COMMON ==\n");
  const fx::Operand op = fx::buildOperand();
  const topo::Shape sphere = fx::buildSphereCutter();
  std::printf("  sphere centre z=%.6f radius=%.3f  closed-form lens V=%.8f\n", fx::kZc, fx::kRs,
              fx::lensVolume());

  // ── CUT (KeepSide::Below): keep the disk OUTSIDE the sphere = the LENS. ────────
  // Refine the mesh; the deflection-bounded curved cap makes the meshed volume converge
  // to the closed form (O(deflection) faceting). We assert the FINEST band + monotone
  // approach, mirroring L3-S1's closed-form gate but with a CURVED cap.
  const double Vlens = fx::lensVolume();
  double relPrev = 1e9;
  double vFine = 0.0;
  bool converging = true;
  for (double defl : {0.004, 0.002, 0.001, 0.0005}) {
    const bool_::NurbsCurvedSplitResult cut =
        bool_::nurbsFaceCurvedSplit(op.wall, op.base, sphere, bool_::KeepSide::Below, defl);
    const double rel = std::fabs(cut.enclosedVolume - Vlens) / Vlens;
    std::printf("  CUT   defl=%.4f decline=%s seamN=%d fidF=%.3g fidG=%.3g onSurf=%.3g "
                "tiling=%.3g wt=%d V=%.8f rel=%.3e\n",
                defl, bool_::nurbsCurvedSplitDeclineName(cut.decline), cut.seamNodes,
                cut.seamFidelityF, cut.seamFidelityG, cut.seamOnSurf, cut.tilingGap,
                cut.watertight ? 1 : 0, cut.enclosedVolume, rel);
    expectTrue(cut.ok(), "CUT (Below) returns a verified keep-side solid");
    expectTrue(cut.watertight, "CUT curved↔curved weld is watertight");
    // DISAGREED=0: the seam lies on BOTH the NURBS wall AND the analytic sphere.
    expectTrue(cut.seamFidelityF < 1e-6, "CUT seam pcurve round-trips on the NURBS wall (S_F==C)");
    expectTrue(cut.seamFidelityG < 1e-6, "CUT seam pcurve round-trips on the sphere (S_G==C)");
    expectTrue(cut.seamOnSurf < 1e-6, "CUT seam lies on BOTH the NURBS wall and the sphere");
    expectTrue(cut.tilingGap < 1e-6, "CUT smooth-trim tiles (areaInside+areaOutside==parent)");
    if (rel > relPrev + 1e-6) converging = false;
    relPrev = rel;
    vFine = cut.enclosedVolume;
  }
  expectTrue(converging, "CUT lens volume converges MONOTONELY toward the closed form");
  // Finest band: within the curved-tessellation band of the exact lens volume.
  expectNear(vFine, Vlens, 0.02 * Vlens, "CUT lens volume ~ V_lens (closed form, finest defl)");

  // Euler χ=2 at a representative deflection.
  {
    const bool_::NurbsCurvedSplitResult cut =
        bool_::nurbsFaceCurvedSplit(op.wall, op.base, sphere, bool_::KeepSide::Below, 0.001);
    tess::MeshParams mp;
    mp.deflection = 0.001;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(cut.solid);
    const long chi = eulerChar(m);
    std::printf("  CUT euler χ = %ld (want 2)\n", chi);
    expectTrue(chi == 2, "CUT weld is a closed genus-0 solid (Euler χ=2)");
  }

  // ── COMMON (KeepSide::Above): keep the annulus INSIDE the sphere. ─────────────
  const bool_::NurbsCurvedSplitResult com =
      bool_::nurbsFaceCurvedSplit(op.wall, op.base, sphere, bool_::KeepSide::Above, 0.001);
  std::printf("  COMMON decline=%s seamN=%d fidF=%.3g fidG=%.3g wt=%d V=%.8f\n",
              bool_::nurbsCurvedSplitDeclineName(com.decline), com.seamNodes, com.seamFidelityF,
              com.seamFidelityG, com.watertight ? 1 : 0, com.enclosedVolume);
  expectTrue(com.ok(), "COMMON (Above) returns a verified keep-side solid");
  expectTrue(com.watertight, "COMMON curved↔curved weld is watertight");
  expectTrue(com.seamFidelityF < 1e-6 && com.seamFidelityG < 1e-6,
             "COMMON seam lies on both the NURBS wall and the sphere");
  expectTrue(com.enclosedVolume > 0.0, "COMMON enclosed volume is positive");
}

// Gate 2 — HONEST DECLINE (no silent-wrong): cases that cannot be split/welded must
// return a NULL solid with a MEASURED reason, never a fabricated result.
static void gate2_honest_declines() {
  std::printf("\n== GATE 2: honest declines (NULL + measured reason, no silent-wrong) ==\n");
  const fx::Operand op = fx::buildOperand();
  const topo::Shape sphere = fx::buildSphereCutter();

  // (a) A NON-CURVED cutter (the flat base) → CutterNotCurved decline.
  {
    const bool_::NurbsCurvedSplitResult res =
        bool_::nurbsFaceCurvedSplit(op.wall, op.base, op.base, bool_::KeepSide::Below, 0.01);
    std::printf("  non-curved cutter: decline=%s solid-null=%d\n",
                bool_::nurbsCurvedSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "non-curved cutter HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsCurvedSplitDecline::CutterNotCurved,
               "non-curved cutter decline is CutterNotCurved");
  }

  // (b) A non-NURBS wall (the flat base as wall) → WallNotNurbs decline.
  {
    const bool_::NurbsCurvedSplitResult res =
        bool_::nurbsFaceCurvedSplit(op.base, op.base, sphere, bool_::KeepSide::Below, 0.01);
    std::printf("  non-NURBS wall: decline=%s solid-null=%d\n",
                bool_::nurbsCurvedSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "non-NURBS wall HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsCurvedSplitDecline::WallNotNurbs,
               "non-NURBS wall decline is WallNotNurbs");
  }

  // (c) A far-away sphere that does not cut the wall → a measured no-seam / no-solid
  // decline (SeamUnusable / SeamOffSurface / SmoothSplitFailed) — never a wrong solid.
  {
    const topo::Shape farSphere = fx::buildSphereCutter().located(
        topo::Location{math::Transform::translationOf(math::Vec3{0, 0, 10.0})});
    const bool_::NurbsCurvedSplitResult res =
        bool_::nurbsFaceCurvedSplit(op.wall, op.base, farSphere, bool_::KeepSide::Below, 0.01);
    std::printf("  far sphere: decline=%s solid-null=%d\n",
                bool_::nurbsCurvedSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "far sphere HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsCurvedSplitDecline::SeamUnusable ||
                   res.decline == bool_::NurbsCurvedSplitDecline::SeamOffSurface ||
                   res.decline == bool_::NurbsCurvedSplitDecline::SmoothSplitFailed ||
                   res.decline == bool_::NurbsCurvedSplitDecline::KeepFaceUnusable,
               "far sphere decline is a measured no-seam reason (never a wrong solid)");
  }
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("# L3-S2 EXACT-NURBS FACE SPLIT BY CURVED FACE — host closed-form gate (OCCT-free)\n");
#if defined(CYBERCAD_HAS_NUMSCI)
  gate0_surface_is_exact_paraboloid();
  gate1_lens_cut_and_common();
  gate2_honest_declines();
  std::printf("\n# checks=%d failures=%d\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("# RESULT: L3-S2 exact-NURBS face split by curved face — ALL GATES GREEN\n");
    return 0;
  }
  std::printf("# RESULT: %d FAILURE(S)\n", g_failures);
  return 1;
#else
  std::printf("# SKIPPED (CYBERCAD_HAS_NUMSCI OFF): the M1 seam trace is substrate-gated.\n");
  return 0;
#endif
}
