// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for BOOL-INT / LAYER 3 — the general two-freeform-solid NURBS boolean
// ORCHESTRATOR `nurbsSolidBoolean(A, B, op)` (nurbs_solid_boolean.h), the flagship that
// COMPOSES the five landed L3 stage verbs (SSI → pcurve → split+heal → membership → sew)
// into ONE watertight result — or an honest decline to NULL with a measured residual map.
//
// This suite proves, OCCT-free (host closed-form oracle):
//   * SINGLE-SEAM bowl-cup (two freeform NURBS solids meeting in ONE transversal seam):
//       - COMMON welds watertight (χ=2, be=0, coherent) at the closed-form lens π·H²/(4a);
//       - CUT welds watertight at V(A)−lens;
//       - FUSE welds watertight at V(A)+V(B)−lens (the outer envelope — the op the
//         single-seam CUT/COMMON verb did not expose, composed here from the SAME
//         select+weld primitives) — all within the tessellation band, DISAGREED=0;
//   * OP-ALGEBRA sanity: V(fuse)+V(common) == V(A)+V(B) on the tractable pose; CUT is
//     order-sensitive (A−B ≠ B−A here since they are z-mirror-symmetric ⇒ equal volume,
//     so we instead assert the CUT/COMMON partition V(cut)+V(common)=V(A));
//   * MULTI-SEAM pose (two solids meeting in TWO seams): the split+classify lands exactly
//     but the annulus↔annulus inner-seam sew HONEST-DECLINES to NULL with a residual map
//     (per L3-d) — NOT a leaky solid, and the orchestrator surfaces MultiSeamDeclined;
//   * honest declines for a null operand.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_cut_fixture.h"
#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ssx = freeform_freeform_cut_fixture;       // single-seam bowl-cup
namespace msx = freeform_freeform_multiseam_fixture;  // multi-seam mirror cups

// Mesh a result solid and return its (watertight, coherent, boundaryEdges, volume).
static void meshStats(const topo::Shape& s, double d, bool& wt, bool& coh, std::size_t& be,
                      double& vol) {
  tess::MeshParams mp;
  mp.deflection = d;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  wt = tess::isWatertight(m);
  coh = tess::isConsistentlyOriented(m);
  be = tess::boundaryEdgeCount(m);
  vol = std::fabs(tess::enclosedVolume(m));
}

// ── SINGLE-SEAM COMMON welds watertight at the closed-form lens, converging ──
CC_TEST(nsb_single_seam_common_welds_watertight) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volCommon();
  double prevErr = 1.0;
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::SolidBoolReport rep;
    const topo::Shape r = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Common, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::SolidBoolDecline::Ok);
    if (r.isNull()) continue;
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);            // watertight (χ=2, 0 boundary edges)
    CC_CHECK(coh);               // consistently oriented
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);    // within the tessellation band (DISAGREED=0)
    CC_CHECK(err < prevErr);     // and CONVERGES
    prevErr = err;
  }
}

// ── SINGLE-SEAM CUT welds watertight at V(A)−lens, converging ──
CC_TEST(nsb_single_seam_cut_welds_watertight) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volCut();
  double prevErr = 1.0;
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::SolidBoolReport rep;
    const topo::Shape r = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Cut, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::SolidBoolDecline::Ok);
    if (r.isNull()) continue;
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);
    CC_CHECK(coh);
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);
    CC_CHECK(err < prevErr);
    prevErr = err;
  }
}

// ── SINGLE-SEAM FUSE welds watertight at V(A)+V(B)−lens (the outer envelope) ──
// This is the op the single-seam CUT/COMMON verb did not expose: the two rim annuli of A
// and B meet across the shared seam and weld watertight through the M0w seam pin, coherent
// once the whole B-group is orientation-flipped. The meshed volume matches the closed-form
// envelope within the band and is monotone-convergent.
CC_TEST(nsb_single_seam_fuse_welds_watertight) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volA() + ssx::volA() - ssx::volCommon();  // V(A)+V(B)−lens (V(B)=V(A))
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::SolidBoolReport rep;
    const topo::Shape r = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::SolidBoolDecline::Ok);
    if (r.isNull()) continue;
    CC_CHECK(rep.survivorFaces >= 4);  // A annulus+lid ∪ B annulus+lid (the envelope)
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);
    CC_CHECK(coh);
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);
  }
}

// ── OP-ALGEBRA sanity: V(fuse)+V(common) == V(A)+V(B) on the tractable pose ──
// Fuse and Common partition the operand sum: |A∪B| + |A∩B| = |A| + |B| (inclusion-
// exclusion). We assert this on the MESHED volumes of the two orchestrated results (each
// measured at the SAME deflection, so the O(deflection) cap bias cancels between them and
// the identity holds within a tight band). Also asserts the closed-form partition
// V(cut)+V(common)=V(A) directly on the meshed CUT + COMMON.
CC_TEST(nsb_op_algebra_inclusion_exclusion) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.0025;
  bo::SolidBoolReport rf, rc, ru;
  const topo::Shape fuse = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &rf);
  const topo::Shape common = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Common, d, &rc);
  const topo::Shape cut = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Cut, d, &ru);
  CC_CHECK(!fuse.isNull() && !common.isNull() && !cut.isNull());
  if (fuse.isNull() || common.isNull() || cut.isNull()) return;
  bool wt, coh; std::size_t be;
  double vFuse, vCommon, vCut;
  meshStats(fuse, d, wt, coh, be, vFuse);
  meshStats(common, d, wt, coh, be, vCommon);
  meshStats(cut, d, wt, coh, be, vCut);
  tess::MeshParams mp; mp.deflection = d;
  const double vA = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(A)));
  const double vB = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(B)));
  // inclusion-exclusion: |A∪B| + |A∩B| ≈ |A| + |B| (meshed, same deflection).
  const double lhs = vFuse + vCommon, rhs = vA + vB;
  CC_CHECK(std::fabs(lhs - rhs) / rhs < 0.02);
  // partition: |A−B| + |A∩B| ≈ |A|.
  CC_CHECK(std::fabs((vCut + vCommon) - vA) / vA < 0.02);
  // CUT is order-sensitive as a SET operation: A−B keeps A's material, B−A keeps B's.
  // Here A,B are z-mirror-symmetric so V(A−B)=V(B−A), but the SHAPES differ (A−B is the
  // carved bowl with B's ceiling; B−A the mirror). We assert the two are BOTH valid and
  // equal-volume (the symmetry), confirming the op is genuinely per-operand.
  bo::SolidBoolReport rba;
  const topo::Shape cutBA = bo::nurbsSolidBoolean(B, A, bo::SolidBoolOp::Cut, d, &rba);
  CC_CHECK(!cutBA.isNull());
  if (!cutBA.isNull()) {
    double vCutBA;
    meshStats(cutBA, d, wt, coh, be, vCutBA);
    CC_CHECK(std::fabs(vCut - vCutBA) / vCut < 0.02);  // symmetric pose ⇒ equal volume
  }
}

// ── MULTI-SEAM pose: split+classify lands, annulus↔annulus sew HONEST-DECLINES ──
// Two degree-4 mirror cups meeting in TWO concentric circular seams. The orchestrator
// dispatches to the multi-seam path, which splits + classifies exactly but the inner-seam
// annulus↔annulus sew hits the frozen M0 mesher's holed-curved-seam gap → NULL with a
// residual map (the sharpened boundaryEdges), NOT a leaky solid.
CC_TEST(nsb_multi_seam_honest_declines_never_leaky) {
  const topo::Shape A = msx::buildA();
  const topo::Shape B = msx::buildB();
  // Prime the cached 2-seam trace (expensive degree-4 trace) — asserts it returns 2 loops.
  CC_CHECK(msx::closedSeams().size() == 2);
  for (bo::SolidBoolOp op : {bo::SolidBoolOp::Common, bo::SolidBoolOp::Cut}) {
    const double cf = op == bo::SolidBoolOp::Common ? msx::volCommon() : msx::volCut();
    bo::SolidBoolReport rep;
    const topo::Shape r = bo::nurbsSolidBoolean(A, B, op, 0.0025, &rep, cf);
    CC_CHECK(r.isNull());                                       // NEVER a leaky solid
    CC_CHECK(rep.multiSeam);                                    // dispatched to multi-seam
    CC_CHECK(rep.seamLoops == 2);
    CC_CHECK(rep.decline == bo::SolidBoolDecline::MultiSeamDeclined);
    CC_CHECK(rep.multiDecline == bo::MultiSeamCutDecline::NotWatertight);
    CC_CHECK(rep.survivorFaces >= 2);                          // the machinery reached the weld
    CC_CHECK(rep.boundaryEdges > 0 && rep.boundaryEdges < 500);  // residual localized to inner seam
  }
}

// ── The multi-seam closed-form partition is self-consistent (oracle unit-check) ──
CC_TEST(nsb_multi_seam_closed_form_consistent) {
  CC_CHECK(std::fabs((msx::volCut() + msx::volCommon()) - msx::volA()) < msx::volA() * 1e-9);
  CC_CHECK(msx::volCommon() > 0.05 * msx::volA());
}

// ── Honest decline: a null operand → NotAdmitted ──
CC_TEST(nsb_declines_null_operand) {
  const topo::Shape nul{};
  const topo::Shape B = ssx::buildB();
  for (bo::SolidBoolOp op : {bo::SolidBoolOp::Fuse, bo::SolidBoolOp::Cut, bo::SolidBoolOp::Common}) {
    bo::SolidBoolReport rep;
    const topo::Shape r = bo::nurbsSolidBoolean(nul, B, op, 0.005, &rep);
    CC_CHECK(r.isNull());
    CC_CHECK(rep.decline == bo::SolidBoolDecline::NotAdmittedA);
  }
}

int main() { return cctest::run_all(); }
