// SPDX-License-Identifier: Apache-2.0
//
// test_native_nurbs_freeform_split.cpp — NURBS roadmap LAYER 3, SLICE 3 (L3-S3) host
// GATE: the OCCT-free closed-form proof of the THIRD (deepest) exact-NURBS B-rep boolean
// — a genuine NURBS face SPLIT BY ANOTHER FREEFORM NURBS face, welded watertight
// (nurbs_freeform_split.h). BOTH operands are arbitrary NURBS (the general
// freeform↔freeform sew, the stage-5 deep-tail wall).
//
// TWO genuine-NURBS-walled bowl-cups (each a Kind::BSpline degree-2 paraboloid wall
// trimmed by a rim + a flat lid) — an UP bowl F and a DOWN dome G — meet in ONE CLOSED
// CIRCULAR seam, a curve on BOTH NURBS walls. `nurbsFaceFreeformSplit` composes
// NURBS-adapter ∩ NURBS-adapter trace[stage 1] → WLine-(u,v)-read fidelity gate on BOTH
// F and G[stage 2] → splitFaceSmoothTrim on BOTH walls[stage 3] → mesh-membership keep
// [stage 4] → curved-NURBS↔curved-NURBS orientation-coherent weld[stage 5] →
// watertight+volume self-verify, into the COMMON (F ∩ G) lens.
//
// GATES (readiness doc):
//   1. CORRECTNESS vs a KNOWN oracle — the COMMON lens enclosed volume matches the
//      closed-form V_lens = π·H²/(4a) as the mesh refines (a deflection-bounded curved
//      cap, O(deflection) convergence), verified TWO-SIDED (the closed form is passed in,
//      so a too-small orientation-collapsed volume can never pass).
//   2. DISAGREED==0 / no silent-wrong — the seam lies on BOTH NURBS walls (fidelity
//      S_F(u,v)==C AND S_G(u,v)==C AND on-both-surfaces ≤ tol), the curved↔curved weld is
//      watertight (Euler χ=2) AND consistently oriented, and a case that can't be
//      split/welded HONEST-DECLINES (NULL + a measured reason), never a wrong result.
//
// The SIM leg vs OCCT BRepAlgoAPI_Common is the sim GATE (b) (see the .mm sibling); this
// host leg carries the analytic ground truth so the slice is proven without OCCT.
//
// Exits 0 iff every gate holds; 1 on any failure. Requires CYBERCAD_HAS_NUMSCI (the M1
// seam trace); with the substrate off it is a clean SKIP.
//
#include <cmath>
#include <cstdio>

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/boolean/nurbs_freeform_split.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "nurbs_freeform_split_fixture.h"
#endif

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
namespace fx = nurbs_freeform_split_fixture;

// Euler characteristic of a closed mesh (χ = V − E + F = 2 for a genus-0 solid).
static long eulerChar(const tess::Mesh& m) {
  return static_cast<long>(m.vertices.size()) - static_cast<long>(tess::edgeUseCounts(m).size()) +
         static_cast<long>(m.triangles.size());
}

// Gate 0 — the shared seam is a CLOSED curved circle interior to BOTH NURBS walls, on
// both surfaces (the real M1 trace between two Kind::BSpline surfaces).
static void gate0_shared_seam_is_closed_on_both_nurbs_walls() {
  std::printf("\n== GATE 0: shared seam is a closed circle on BOTH NURBS walls ==\n");
  const bool_::ssi::WLine seam = fx::closedSeamWLine();
  expectTrue(seam.points.size() >= 8, "shared seam has >= 8 nodes");
  expectTrue(seam.status == bool_::ssi::TraceStatus::Closed, "shared seam is a CLOSED loop");
  double maxF = 0, maxG = 0, maxSurf = 0, maxZ = 0;
  for (const auto& p : seam.points) {
    maxF = std::max(maxF, std::fabs(std::hypot(p.u1 - 0.5, p.v1 - 0.5) - fx::rho()));
    maxG = std::max(maxG, std::fabs(std::hypot(p.u2 - 0.5, p.v2 - 0.5) - fx::rho()));
    maxSurf = std::max(maxSurf, p.onSurfResidual);
    maxZ = std::max(maxZ, std::fabs(p.point.z - fx::seamZ()));
  }
  std::printf("  seamN=%zu radiusResid F=%.3g G=%.3g onSurf=%.3g zResid=%.3g\n",
              seam.points.size(), maxF, maxG, maxSurf, maxZ);
  expectTrue(maxF < 1e-3, "seam radius ρ on F's (u,v)");
  expectTrue(maxG < 1e-3, "seam radius ρ on G's (u,v) — BOTH sides NURBS");
  expectTrue(maxSurf < 1e-6, "seam node lies on BOTH NURBS surfaces");
  expectTrue(maxZ < 1e-6, "seam height z* = H/2");
  expectTrue(fx::rho() < fx::kR, "seam interior to both rim trims");
}

// Gate 0b — the closed-form oracles tile exactly, and the lens is a substantial fraction.
static void gate0b_closed_form_partition() {
  std::printf("\n== GATE 0b: closed-form oracle self-consistency ==\n");
  expectTrue(std::fabs((fx::volCut() + fx::volCommon()) - fx::volF()) < fx::volF() * 1e-12,
             "V(F−G)+V(F∩G)==V(F) (oracle partition closes)");
  expectTrue(fx::volCommon() > 0.01 * fx::volF(), "the lens is a SUBSTANTIAL discriminating volume");
}

// Gate 1 — the core L3-S3 gate: COMMON (the lens) WELDS watertight at the closed-form
// volume π·H²/(4a) as the mesh refines, χ=2, consistently oriented, seam on BOTH NURBS
// walls, DISAGREED=0. The curved-NURBS↔curved-NURBS sew — the deep-tail wall — resolved.
static void gate1_common_lens_welds_at_closed_form() {
  std::printf("\n== GATE 1: exact-NURBS face split by FREEFORM NURBS face — COMMON lens ==\n");
  const topo::Shape F = fx::buildF();
  const topo::Shape G = fx::buildG();
  const double cf = fx::volCommon();  // π·H²/(4a)
  std::printf("  closed-form lens V=%.8f\n", cf);

  double relPrev = 1e9;
  double vFine = 0.0;
  bool converging = true;
  for (double defl : {0.01, 0.005, 0.0025, 0.00125}) {
    const bool_::NurbsFreeformSplitResult com =
        bool_::nurbsFaceFreeformSplit(F, G, bool_::FfOp::Common, defl, cf);
    const double rel = std::fabs(com.enclosedVolume - cf) / cf;
    std::printf("  COMMON defl=%.5f decline=%s seamN=%d fidF=%.3g fidG=%.3g onSurf=%.3g "
                "tileF=%.3g tileG=%.3g wt=%d V=%.8f rel=%.3e\n",
                defl, bool_::nurbsFreeformSplitDeclineName(com.decline), com.seamNodes,
                com.seamFidelityF, com.seamFidelityG, com.seamOnSurf, com.tilingGapF,
                com.tilingGapG, com.watertight ? 1 : 0, com.enclosedVolume, rel);
    expectTrue(com.ok(), "COMMON returns a verified keep-side solid");
    expectTrue(com.watertight, "COMMON curved-NURBS↔curved-NURBS weld is watertight");
    // DISAGREED=0: the seam lies on BOTH NURBS walls.
    expectTrue(com.seamFidelityF < 1e-6, "COMMON seam round-trips on F's NURBS wall (S_F==C)");
    expectTrue(com.seamFidelityG < 1e-6, "COMMON seam round-trips on G's NURBS wall (S_G==C)");
    expectTrue(com.seamOnSurf < 1e-6, "COMMON seam lies on BOTH NURBS walls");
    expectTrue(com.tilingGapF < 1e-6, "COMMON F wall tiles (areaInside+areaOutside==parent)");
    expectTrue(com.tilingGapG < 1e-6, "COMMON G wall tiles (areaInside+areaOutside==parent)");
    if (com.ok()) {
      if (rel > relPrev + 1e-6) converging = false;
      relPrev = rel;
      vFine = com.enclosedVolume;
    }
  }
  expectTrue(converging, "COMMON lens volume converges MONOTONELY toward the closed form");
  expectNear(vFine, cf, 0.04 * cf, "COMMON lens volume ~ V_lens (closed form, finest defl)");

  // Euler χ=2 + consistently oriented at a representative deflection.
  {
    const bool_::NurbsFreeformSplitResult com =
        bool_::nurbsFaceFreeformSplit(F, G, bool_::FfOp::Common, 0.0025, cf);
    expectTrue(com.ok(), "COMMON (repr defl) ok for χ check");
    if (com.ok()) {
      tess::MeshParams mp;
      mp.deflection = 0.0025;
      const tess::Mesh m = tess::SolidMesher(mp).mesh(com.solid);
      const long chi = eulerChar(m);
      std::printf("  COMMON euler χ = %ld (want 2), consistent=%d\n", chi,
                  tess::isConsistentlyOriented(m) ? 1 : 0);
      expectTrue(chi == 2, "COMMON weld is a closed genus-0 solid (Euler χ=2)");
      expectTrue(tess::isConsistentlyOriented(m), "COMMON weld is consistently oriented");
    }
  }
}

// Gate 2 — HONEST DECLINE (no silent-wrong): cases that cannot be split/welded must
// return a NULL solid with a MEASURED reason, never a fabricated result.
static void gate2_honest_declines() {
  std::printf("\n== GATE 2: honest declines (NULL + measured reason, no silent-wrong) ==\n");
  const topo::Shape F = fx::buildF();
  const topo::Shape G = fx::buildG();

  // (a) A null operand F → NotAdmittedF.
  {
    const bool_::NurbsFreeformSplitResult res =
        bool_::nurbsFaceFreeformSplit(topo::Shape{}, G, bool_::FfOp::Common, 0.005);
    std::printf("  null F: decline=%s solid-null=%d\n",
                bool_::nurbsFreeformSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "null operand F HONEST-DECLINES to NULL");
    expectTrue(res.decline == bool_::NurbsFreeformSplitDecline::NotAdmittedF,
               "null operand F decline is NotAdmittedF");
  }

  // (b) Two non-intersecting operands (G lifted far above F) → SeamUnusable.
  {
    auto polesUp = fx::upBowlPoles();
    for (auto& p : polesUp) p.z += 100.0;
    const topo::Shape farG = fx::buildCup(polesUp, fx::kA * fx::kR * fx::kR + 100.0);
    const bool_::NurbsFreeformSplitResult res =
        bool_::nurbsFaceFreeformSplit(F, farG, bool_::FfOp::Common, 0.005);
    std::printf("  non-intersecting: decline=%s solid-null=%d\n",
                bool_::nurbsFreeformSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "non-intersecting operands HONEST-DECLINE to NULL");
    expectTrue(res.decline == bool_::NurbsFreeformSplitDecline::SeamUnusable,
               "non-intersecting decline is SeamUnusable");
  }

  // (c) The CUT leg now WELDS watertight. Its survivor membership RESOLVES honestly (robust
  // interior-UV winding, not a fragile apex sample), and its annulus↔disk shared seam-as-hole
  // sew now welds through the M0-WELD shared-strip cull (uv_triangulate.h ConstrainedDelaunay:
  // the CDT hole-cull is a TOPOLOGICAL flood fill so the annulus and the disk triangulate the
  // shared seam strip identically). The CUT keep-side solid meshes to a closed 2-manifold
  // (weldOpenEdges 0) whose volume passes the two-sided closed-form self-verify — a real weld,
  // not a decline.
  {
    const bool_::NurbsFreeformSplitResult res =
        bool_::nurbsFaceFreeformSplit(F, G, bool_::FfOp::Cut, 0.005);
    std::printf("  CUT: decline=%s solid-null=%d membershipResolved=%d weldOpenEdges=%d wt=%d V=%.6f\n",
                bool_::nurbsFreeformSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0,
                res.cutMembershipResolved ? 1 : 0, res.weldOpenEdges, res.watertight ? 1 : 0,
                res.enclosedVolume);
    expectTrue(res.ok() && !res.solid.isNull(), "CUT WELDS watertight (never leaky, never NULL)");
    expectTrue(res.cutMembershipResolved, "CUT survivor membership RESOLVES (robust winding, not apex)");
    expectTrue(res.decline == bool_::NurbsFreeformSplitDecline::Ok, "CUT welds (decline Ok)");
    expectTrue(res.watertight, "CUT welded solid is watertight");
    expectTrue(res.weldOpenEdges == 0, "CUT weld leaves NO unpaired boundary edges (shared-strip weld)");
    // The CUT keep-side volume V(F)−lens matches the closed form within the tessellation band.
    const double cf = fx::volCut();
    expectTrue(res.enclosedVolume > 0.0 && std::fabs(res.enclosedVolume - cf) / cf < 0.15,
               "CUT welded volume ~ V(F)-lens (closed form) within band");
  }
}

// Gate 3 — MULTI-CROSSING honest decline: a laterally-offset dome whose shared seam is no
// longer a single simple interior loop (the trim curve enters/exits F's rim more than once)
// must HONEST-DECLINE with a MEASURED reason — never fabricate a single-loop partition or a
// mis-membered region. Exercises the "classify each region independently / decline the
// multi-crossing" requirement without widening any tolerance.
static void gate3_multi_crossing_honest_decline() {
  std::printf("\n== GATE 3: multi-crossing (offset seam) honest decline ==\n");
  const topo::Shape F = fx::buildF();
  // dx = 0.45 shifts the dome so far that the seam leaves F's trimmed rim (a boundary-
  // crossing / re-entrant seam), the multi-crossing regime that is out of the single-loop
  // smooth-trim envelope.
  const topo::Shape Goff = fx::buildOffsetDome(0.45);
  for (bool_::FfOp op : {bool_::FfOp::Common, bool_::FfOp::Cut}) {
    const bool_::NurbsFreeformSplitResult res = bool_::nurbsFaceFreeformSplit(F, Goff, op, 0.005);
    std::printf("  multi-crossing op=%s: decline=%s null=%d\n", op == bool_::FfOp::Cut ? "Cut" : "Common",
                bool_::nurbsFreeformSplitDeclineName(res.decline), res.solid.isNull() ? 1 : 0);
    expectTrue(!res.ok() && res.solid.isNull(), "multi-crossing HONEST-DECLINES to NULL (never wrong)");
    expectTrue(res.decline == bool_::NurbsFreeformSplitDecline::SeamUnusable ||
                   res.decline == bool_::NurbsFreeformSplitDecline::SmoothSplitFailedF ||
                   res.decline == bool_::NurbsFreeformSplitDecline::SmoothSplitFailedG ||
                   res.decline == bool_::NurbsFreeformSplitDecline::ClassifyAmbiguous,
               "multi-crossing decline is a measured seam/split/membership reason");
  }
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("# L3-S3 EXACT-NURBS FACE SPLIT BY FREEFORM NURBS FACE — host closed-form gate (OCCT-free)\n");
#if defined(CYBERCAD_HAS_NUMSCI)
  gate0_shared_seam_is_closed_on_both_nurbs_walls();
  gate0b_closed_form_partition();
  gate1_common_lens_welds_at_closed_form();
  gate2_honest_declines();
  gate3_multi_crossing_honest_decline();
  std::printf("\n# checks=%d failures=%d\n", g_checks, g_failures);
  if (g_failures == 0) {
    std::printf("# RESULT: L3-S3 exact-NURBS face split by freeform NURBS face — ALL GATES GREEN\n");
    return 0;
  }
  std::printf("# RESULT: %d FAILURE(S)\n", g_failures);
  return 1;
#else
  std::printf("# SKIPPED (CYBERCAD_HAS_NUMSCI OFF): the M1 seam trace is substrate-gated.\n");
  return 0;
#endif
}
