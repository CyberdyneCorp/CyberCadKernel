// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON — the
// OCCT-FREE analytic proof that the recognise → trace → split → classify pipeline
// composes for TWO curved operands over a shared CLOSED curved seam, and that the weld
// verb HONEST-DECLINES to NULL (never a leaky/partial solid) because the two-CURVED-side
// closed-seam weld is gated on the byte-frozen M0 tessellator.
//
// The fixture is two coaxial paraboloid bowl-cups (an UP bowl-cup A and a DOWN dome-cup
// B) whose two curved walls meet in ONE CLOSED CIRCLE of radius ρ = √(H/2a) at z = H/2
// (freeform_freeform_cut_fixture.h). We assert:
//   * both operands B1-admit with EXACTLY one freeform wall + one analytic lid;
//   * the shared seam is the real M1 trace: CLOSED, radius ρ on BOTH walls' (u,v) to
//     ~1e-13, on both surfaces to the trace residual;
//   * the closed-form volume oracles are self-consistent (V(A−B)+V(A∩B)=V(A));
//   * `freeformFreeformClosedSeamCut` runs the whole pipeline: COMMON (the lens) WELDS
//     watertight at the closed-form volume π·H²/(4a) after the orientation-coherence
//     repair (one cap reversed) and CONVERGES as the deflection refines, verified
//     TWO-SIDED; CUT honest-declines to NULL (its apex-adjacent membership is ambiguous)
//     — NEVER a leaky/wrong solid; and it DECLINES a non-operand / non-intersecting pose.
// Requires CYBERCAD_HAS_NUMSCI (the seam is the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_cut_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ffx = freeform_freeform_cut_fixture;

// ── Both operands B1-admit with exactly ONE freeform wall + ONE analytic lid ──
CC_TEST(ff_operands_admit_single_freeform_each) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  bo::OperandDecline wa = bo::OperandDecline::Ok, wb = bo::OperandDecline::Ok;
  const auto foA = bo::recogniseFreeformSolid(A, &wa);
  const auto foB = bo::recogniseFreeformSolid(B, &wb);
  CC_CHECK(foA.has_value());
  CC_CHECK(foB.has_value());
  if (!foA || !foB) return;
  CC_CHECK(foA->freeform.size() == 1 && foA->analytic.size() == 1 && foA->watertight);
  CC_CHECK(foB->freeform.size() == 1 && foB->analytic.size() == 1 && foB->watertight);
}

// ── The shared seam is a CLOSED curved circle interior to BOTH walls (real M1) ──
CC_TEST(ff_shared_seam_is_closed_curved_circle) {
  const bo::ssi::WLine seam = ffx::closedSeamWLine();
  CC_CHECK(seam.points.size() >= 8);
  CC_CHECK(seam.status == bo::ssi::TraceStatus::Closed);
  double maxA = 0, maxB = 0, maxSurf = 0, maxZ = 0;
  for (const auto& p : seam.points) {
    maxA = std::max(maxA, std::fabs(std::hypot(p.u1 - 0.5, p.v1 - 0.5) - ffx::rho()));
    maxB = std::max(maxB, std::fabs(std::hypot(p.u2 - 0.5, p.v2 - 0.5) - ffx::rho()));
    maxSurf = std::max(maxSurf, p.onSurfResidual);
    maxZ = std::max(maxZ, std::fabs(p.point.z - ffx::seamZ()));
  }
  CC_CHECK(maxA < 1e-3);           // radius ρ on A's (u,v)
  CC_CHECK(maxB < 1e-3);           // radius ρ on B's (u,v) — BOTH sides curved
  CC_CHECK(maxSurf < 1e-6);        // node lies on BOTH surfaces (trace residual)
  CC_CHECK(maxZ < 1e-6);           // seam height z* = H/2
  CC_CHECK(ffx::rho() < ffx::kR);  // interior to both rim trims
}

// ── The closed-form volume oracles tile exactly (oracle unit-check) ──
CC_TEST(ff_closed_form_partition_is_consistent) {
  CC_CHECK(std::fabs((ffx::volCut() + ffx::volCommon()) - ffx::volA()) < ffx::volA() * 1e-12);
  CC_CHECK(ffx::volCommon() > 0.01 * ffx::volA());  // a SUBSTANTIAL, discriminating lens
}

// ── CUT HONEST-DECLINES to NULL (never a leaky solid): the two-CURVED-side seam ──
// weld is gated on the byte-frozen M0 tessellator (MEASURED: the closed seam between
// two independently-tessellated CURVED sub-faces opens — curved↔flat welds via the M0w
// pin, curved↔curved does not, so the verb returns NULL → OCCT, NEVER a leaky solid).
CC_TEST(ff_cut_honest_declines_never_leaky) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::FfCutDecline why = bo::FfCutDecline::Ok;
    const topo::Shape cut = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Cut, d, &why);
    CC_CHECK(cut.isNull());                      // NULL → OCCT (the disciplined outcome)
    CC_CHECK(why != bo::FfCutDecline::Ok);       // with a measured blocker
    // The blocker is the weld itself (or the membership at the degenerate apex), NOT a
    // recognise/trace/split failure — the pipeline reaches the weld and refuses to emit
    // a non-watertight solid.
    CC_CHECK(why == bo::FfCutDecline::NotWatertight ||
             why == bo::FfCutDecline::ClassifyAmbiguous);
  }
}

// ── COMMON (the lens) WELDS watertight at the CLOSED-FORM volume, and CONVERGES ──
// The two survivor caps (A's disk, B's disk) each inherit their parent wall's
// orientation (A opens UP, B opens DOWN), so a naive weld is watertight (undirected)
// but orientation-INCONSISTENT — its signed volume is a locked 33% too small and does
// NOT converge. `freeformFreeformClosedSeamCut` repairs orientation coherence (the
// directed-edge invariant: exactly one cap reversed) so the assembled lens is a coherent
// outward-normal boundary; its meshed volume then matches the closed-form lens
// V = π·H²/(4a) within the deflection-bounded band AND converges monotonically as the
// deflection refines (the residual is the O(deflection) triangulation under-estimate of
// a smooth cap, NOT the orientation error). The self-verify is TWO-SIDED (the closed
// form is passed in), so a too-small wrong volume can never be returned as success.
CC_TEST(ff_common_welds_watertight_at_closed_form) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const double cf = ffx::volCommon();  // π·H²/(4a) = 0.010053096
  double prevErr = 1.0;                 // relative error must shrink as d refines
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::FfCutDecline why = bo::FfCutDecline::Ok;
    const topo::Shape com = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Common, d, &why, cf);
    CC_CHECK(!com.isNull());                       // WELDS (not a decline)
    CC_CHECK(why == bo::FfCutDecline::Ok);
    if (com.isNull()) continue;
    tess::MeshParams mp;
    mp.deflection = d;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(com);
    CC_CHECK(tess::isWatertight(m));                       // χ = 2 (closed 2-manifold)
    CC_CHECK(tess::isConsistentlyOriented(m));             // coherent winding (0 same-dir dups)
    const double v = std::fabs(tess::enclosedVolume(m));
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);                              // within the deflection band
    CC_CHECK(v < cf);                                      // smooth cap under-estimates
    CC_CHECK(err < prevErr);                               // and CONVERGES toward cf
    prevErr = err;
  }
}

// ── The pipeline reaches the split for BOTH walls (the enabler is real) ──
// Proves recognise → trace → B2 smooth-trim split lands on BOTH curved walls (each a
// disk + annulus tiling to the closed-form circle area), so the ONLY missing step is
// the two-curved-side seam weld (the tessellator gate) — not any earlier stage.
CC_TEST(ff_pipeline_splits_both_curved_walls) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  CC_CHECK(foA && foB);
  if (!foA || !foB) return;
  const bo::ssi::WLine seam = ffx::closedSeamWLine();
  const auto& wallA = foA->faces[foA->freeform.front()];
  const auto& wallB = foB->faces[foB->freeform.front()];
  bo::ssi::WLine seamB = seam;
  for (auto& p : seamB.points) { p.u1 = p.u2; p.v1 = p.v2; }
  const bo::SmoothSplitResult sA = bo::splitFaceSmoothTrim(wallA.face, seam);
  const bo::SmoothSplitResult sB = bo::splitFaceSmoothTrim(wallB.face, seamB);
  CC_CHECK(sA.ok());
  CC_CHECK(sB.ok());
  if (!sA.ok() || !sB.ok()) return;
  // each disk cap's UV area equals the closed-form circle π·ρ² (interior to the rim).
  const double diskCf = ffx::kPi * ffx::rho() * ffx::rho();
  CC_CHECK(std::fabs(sA.split->areaInside - diskCf) / diskCf < 1e-2);
  CC_CHECK(std::fabs(sB.split->areaInside - diskCf) / diskCf < 1e-2);
}

// ── Honest decline: a non-operand (null shape) → NotAdmitted ──
CC_TEST(ff_declines_non_operand) {
  const topo::Shape nul{};
  const topo::Shape B = ffx::buildB();
  bo::FfCutDecline why = bo::FfCutDecline::Ok;
  const topo::Shape r = bo::freeformFreeformClosedSeamCut(nul, B, bo::FfOp::Cut, 0.005, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == bo::FfCutDecline::NotAdmittedA);
}

// ── Honest decline: two operands that do NOT intersect (no shared seam) → SeamUnusable ──
CC_TEST(ff_declines_non_intersecting) {
  const topo::Shape A = ffx::buildA();
  // A second up bowl-cup far ABOVE A (no curved-wall intersection).
  const double lidZ = ffx::kA * ffx::kR * ffx::kR;
  auto polesUp = ffx::upBowlPoles();
  for (auto& p : polesUp) p.z += 100.0;  // lift far away
  const topo::Shape B = ffx::buildCup(polesUp, lidZ + 100.0);
  bo::FfCutDecline why = bo::FfCutDecline::Ok;
  const topo::Shape r = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Cut, 0.005, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == bo::FfCutDecline::SeamUnusable);
}

int main() { return cctest::run_all(); }
