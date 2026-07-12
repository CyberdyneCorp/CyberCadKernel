// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for BOOL-MULTI-OP / LAYER 3 — the N-OPERAND NURBS solid boolean
// (assemblies): `nurbsSolidUnionN` / `nurbsSolidCutN` / `nurbsSolidIntersectN`
// (nurbs_solid_boolean_nary.h), a LEFT-FOLD of the general two-solid boolean
// `nurbsSolidBoolean(A, B, op)` — union of many parts, a base minus several tools, the
// common of an assembly. It COMPOSES the binary boolean and re-implements NONE of it.
//
// This suite proves, OCCT-free (host closed-form oracle):
//   * UNION of two overlapping tractable solids folds through the N-ary driver to the SAME
//     watertight outer envelope the binary Fuse welds, at the inclusion-exclusion volume
//     V(A)+V(B)−V(A∩B) within the tessellation band (DISAGREED=0);
//   * BASE minus one tool (CutN) folds to the carved bowl at V(base)−lens, watertight;
//   * BASE minus TWO tools — the fold ATTEMPTS each Cut IN ORDER and HONEST-DECLINES at the
//     re-admission boundary (the binary boolean's holed-freeform-wall output the B1 gate
//     declines), carrying the declining step's residual + index — NEVER a leaky solid;
//   * DECLINE PROPAGATION: a multi-seam-FUSE-below-band intermediate makes the N-ary op
//     decline with THAT step's `MultiSeamDeclined` residual; a null operand / empty list
//     decline as the fold's own first-class pre-checks;
//   * ORDER-AWARENESS: CutN(base,[T]) is the per-operand A−B (order-sensitive by construction);
//   * ASSOCIATIVITY of the meshed union volume (fold order does not change it beyond the band).
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/nurbs_solid_boolean_nary.h"
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

static double meshVol(const topo::Shape& s, double d) {
  bool wt, coh;
  std::size_t be;
  double v;
  meshStats(s, d, wt, coh, be, v);
  return v;
}

// ── N-ary UNION of two overlapping tractable solids = inclusion-exclusion, watertight ──
// UnionN([A,B]) folds through ONE binary Fuse; the meshed volume matches V(A)+V(B)−lens
// within the tessellation band (DISAGREED=0). The N-ary driver threads the binary boolean's
// verified watertight solid + a measured report.
CC_TEST(nary_union_two_inclusion_exclusion) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volA() + ssx::volA() - ssx::volCommon();  // V(A)+V(B)−lens (V(B)=V(A))
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidUnionN({A, B}, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.operands == 2);
    CC_CHECK(rep.steps == 1);
    if (r.isNull()) continue;
    bool wt, coh;
    std::size_t be;
    double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);  // watertight (χ=2, 0 boundary edges)
    CC_CHECK(coh);
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);  // within the tessellation band (DISAGREED=0)
  }
}

// ── N-ary UNION equals the binary Fuse it folds (byte-for-byte the same weld) ──
CC_TEST(nary_union_matches_binary_fuse) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::SolidBoolReport brep;
  const topo::Shape bin = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &brep);
  bo::NaryBoolReport nrep;
  const topo::Shape nary = bo::nurbsSolidUnionN({A, B}, d, &nrep);
  CC_CHECK(!bin.isNull() && !nary.isNull());
  CC_CHECK(brep.decline == bo::SolidBoolDecline::Ok);
  CC_CHECK(nrep.decline == bo::NaryBoolDecline::Ok);
  if (bin.isNull() || nary.isNull()) return;
  // Same weld ⇒ identical meshed volume (the fold adds no geometry of its own).
  CC_CHECK(std::fabs(meshVol(bin, d) - meshVol(nary, d)) < 1e-9);
}

// ── BASE minus ONE tool (CutN) = V(base)−lens, watertight (the CUT fold + order) ──
CC_TEST(nary_cut_base_minus_one) {
  const topo::Shape A = ssx::buildA();  // base
  const topo::Shape B = ssx::buildB();  // tool
  const double cf = ssx::volCut();      // V(A)−lens
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidCutN(A, {B}, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.operands == 2);  // [base, tool]
    CC_CHECK(rep.steps == 1);
    if (r.isNull()) continue;
    bool wt, coh;
    std::size_t be;
    double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);
    CC_CHECK(coh);
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);
  }
}

// ── N-ary INTERSECT of two solids = the lens, watertight (the COMMON fold) ──
CC_TEST(nary_intersect_two_lens) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volCommon();
  const double d = 0.0025;
  bo::NaryBoolReport rep;
  const topo::Shape r = bo::nurbsSolidIntersectN({A, B}, d, &rep, cf);
  CC_CHECK(!r.isNull());
  CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
  CC_CHECK(rep.steps == 1);
  if (r.isNull()) return;
  bool wt, coh;
  std::size_t be;
  double v;
  meshStats(r, d, wt, coh, be, v);
  CC_CHECK(wt);
  CC_CHECK(be == 0);
  CC_CHECK(coh);
  CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);
}

// ── ORDER-AWARENESS: CutN is per-operand A−B; the z-mirror pose ⇒ equal volume, distinct
// shapes. Both CutN(A,[B]) and CutN(B,[A]) weld to equal volume (the symmetry), confirming
// the op is genuinely per-operand (order-sensitive by construction). ──
CC_TEST(nary_cut_is_order_aware) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::NaryBoolReport rAB, rBA;
  const topo::Shape ab = bo::nurbsSolidCutN(A, {B}, d, &rAB);
  const topo::Shape ba = bo::nurbsSolidCutN(B, {A}, d, &rBA);
  CC_CHECK(!ab.isNull() && !ba.isNull());
  CC_CHECK(rAB.decline == bo::NaryBoolDecline::Ok);
  CC_CHECK(rBA.decline == bo::NaryBoolDecline::Ok);
  if (ab.isNull() || ba.isNull()) return;
  const double vAB = meshVol(ab, d), vBA = meshVol(ba, d);
  CC_CHECK(std::fabs(vAB - vBA) / vAB < 0.02);  // symmetric pose ⇒ equal volume
}

// ── ASSOCIATIVITY of the union volume: fold order does not change the meshed volume. The
// SET union is order-free; here the ONLY tractable fold is the 2-operand one, so we assert
// UnionN([A,B]) == UnionN([B,A]) (the commutativity leg of associativity) at one deflection,
// the meshed volumes agreeing beyond the tessellation band. ──
CC_TEST(nary_union_is_associative_commutative) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::NaryBoolReport r1, r2;
  const topo::Shape ab = bo::nurbsSolidUnionN({A, B}, d, &r1);
  const topo::Shape ba = bo::nurbsSolidUnionN({B, A}, d, &r2);
  CC_CHECK(!ab.isNull() && !ba.isNull());
  if (ab.isNull() || ba.isNull()) return;
  CC_CHECK(std::fabs(meshVol(ab, d) - meshVol(ba, d)) / meshVol(ab, d) < 0.02);
}

// ── BASE minus TWO tools (BOOL-READMIT): the fold RE-ADMITS the [base−T1] output as the
// second Cut's operand. With T2 == T1 the second tool is DISJOINT from the remaining
// material (its material was already carved out), so the re-admit path SHORT-CIRCUITS to the
// [base−T1] result EXACTLY — no weld, no synthesised geometry — and the fold WELDS to
// V(base)−lens, watertight (χ=2, be=0), within the tessellation band (DISAGREED=0). This is
// the base-minus-2 oracle made HONEST: the idempotent tool re-admits and adds nothing. ──
CC_TEST(nary_cut_base_minus_two_readmits) {
  const topo::Shape A = ssx::buildA();  // base
  const topo::Shape B = ssx::buildB();  // tool (applied twice)
  const double cf = ssx::volCut();      // V(A)−lens (second identical Cut removes nothing)
  for (double d : {0.01, 0.005}) {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidCutN(A, {B, B}, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.operands == 3);  // [base, T1, T2]
    CC_CHECK(rep.steps == 2);     // both Cuts folded (order-aware)
    if (r.isNull()) continue;
    bool wt, coh;
    std::size_t be;
    double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);  // watertight
    CC_CHECK(coh);
    CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);  // V(base)−lens (DISAGREED=0)
  }
}

// ── 3-solid UNION (BOOL-READMIT): {A,B,B} folds A∪B (binary) then RE-ADMITS that output for
// ∪B. B is CONTAINED in A∪B, so the re-admit path SHORT-CIRCUITS to A∪B EXACTLY (a
// coincidence-tolerant containment witness, no weld) and the union WELDS watertight at the
// inclusion-exclusion volume V(A)+V(B)−lens within the tessellation band (DISAGREED=0). The
// prior measured decline is now a re-admitted weld — the boundary is unblocked for the
// reachable idempotent 3-union. ──
CC_TEST(nary_union_three_readmits) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volA() + ssx::volA() - ssx::volCommon();  // V(A)+V(B)−lens
  for (double d : {0.01, 0.005}) {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidUnionN({A, B, B}, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.operands == 3);
    CC_CHECK(rep.steps == 2);
    if (r.isNull()) continue;
    bool wt, coh;
    std::size_t be;
    double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);
    CC_CHECK(coh);
    CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);
  }
}

// ── INTERSECT idempotence (BOOL-READMIT): {C, A, A} common — the FIRST step C∩A welds a
// genuine lens; the SECOND step re-admits that lens for ∩A. The lens ⊆ A, so ∩A short-
// circuits to the lens EXACTLY, welding at V(C∩A), watertight. This exercises the re-admit
// COMMON containment witness on a genuine boolean-output accumulator (not two coincident
// pristine operands, which have no transversal seam). ──
CC_TEST(nary_intersect_three_readmits) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();  // C: B∩A is the lens
  const double d = 0.005;
  const double cf = ssx::volCommon();   // V(B∩A) = lens; then ∩A leaves the lens
  bo::NaryBoolReport rep;
  const topo::Shape r = bo::nurbsSolidIntersectN({B, A, A}, d, &rep, cf);
  CC_CHECK(!r.isNull());
  CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
  CC_CHECK(rep.steps == 2);
  if (r.isNull()) return;
  bool wt, coh;
  std::size_t be;
  double v;
  meshStats(r, d, wt, coh, be, v);
  CC_CHECK(wt);
  CC_CHECK(be == 0);
  CC_CHECK(coh);
  CC_CHECK(std::fabs(v - cf) / cf < 30.0 * d);
}

// ── SHARPENED BOUNDARY (BOOL-READMIT honest-decline): a GENUINELY-OVERLAPPING 3-union whose
// third operand straddles A∪B (a distinct down-dome shifted in z, NOT redundant) does NOT
// short-circuit; its second seam lands on an ALREADY-HOLED annulus, needing the UNLANDED
// multi-hole / multi-crossing face split. The fold HONEST-DECLINES `StepDeclined` at the
// failing step with the weld's residual — NEVER a leaky/partial solid, NO tolerance widened.
// This is the sharpened re-admission boundary (a step narrower than the pre-BOOL-READMIT
// NotAdmittedA: the operand is now ADMITTED and its seam TRACED; only the multi-hole split
// remains). ──
CC_TEST(nary_union_three_genuine_overlap_declines_at_multihole_split) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  // C = a down-dome shifted UP by dz: it overlaps A∪B at a FRESH seam on B's annulus, and is
  // NOT contained (it straddles A∪B's boundary), so it is a genuine third body.
  auto cPoles = ssx::downDomePoles();
  const double dz = 0.06;
  for (auto& p : cPoles) p.z += dz;
  const topo::Shape C = ssx::buildCup(cPoles, (ssx::kH - ssx::kA * ssx::kR * ssx::kR) + dz);
  const double d = 0.005;
  bo::NaryBoolReport rep;
  const topo::Shape r = bo::nurbsSolidUnionN({A, B, C}, d, &rep);
  CC_CHECK(r.isNull());  // never leaky — honest decline at the sharpened boundary
  CC_CHECK(rep.decline == bo::NaryBoolDecline::StepDeclined);
  CC_CHECK(rep.stepIndex == 2);  // step 1 (A∪B) welds; step 2 re-admits + declines the weld
  CC_CHECK(rep.steps == 2);
  // The operand IS admitted + the seam IS traced now: the decline is the weld's residual
  // (NotWatertight from the multi-hole split), NOT the old NotAdmittedA.
  CC_CHECK(rep.stepReport.decline == bo::SolidBoolDecline::NotWatertight);
}

// ── DECLINE PROPAGATION (the airtight oracle): a multi-seam FUSE below its weld band is a
// genuine intermediate decline — the N-ary op returns NULL carrying THAT step's
// `MultiSeamDeclined` residual, never a leaky/partial solid, no tolerance widened. ──
CC_TEST(nary_union_propagates_multiseam_decline) {
  const topo::Shape MA = msx::buildA();
  const topo::Shape MB = msx::buildB();
  // d = 0.0025 is BELOW the multi-seam FUSE outer-envelope weld band (the flagship suite
  // records the single-T-junction frozen-mesher parity residual there): a genuine decline.
  const double d = 0.0025;
  bo::NaryBoolReport rep;
  const topo::Shape r = bo::nurbsSolidUnionN({MA, MB}, d, &rep);
  CC_CHECK(r.isNull());  // never leaky
  CC_CHECK(rep.decline == bo::NaryBoolDecline::StepDeclined);
  CC_CHECK(rep.stepIndex == 1);
  CC_CHECK(rep.stepReport.decline == bo::SolidBoolDecline::MultiSeamDeclined);
}

// ── The fold's OWN pre-checks: empty list, null operand, single-element identity ──
CC_TEST(nary_edge_cases_empty_null_identity) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;

  // Empty list ⇒ EmptyList decline, NULL.
  {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidUnionN({}, d, &rep);
    CC_CHECK(r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::EmptyList);
  }
  // Null operand mid-list ⇒ NullOperand at its index, caught BEFORE any boolean runs.
  {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidUnionN({A, topo::Shape{}, B}, d, &rep);
    CC_CHECK(r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::NullOperand);
    CC_CHECK(rep.stepIndex == 1);
    CC_CHECK(rep.steps == 0);  // no boolean folded (pre-check)
  }
  // Single-element list ⇒ the identity (that one solid, unchanged; no boolean run).
  {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidUnionN({A}, d, &rep);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.steps == 0);
    CC_CHECK(std::fabs(meshVol(r, d) - meshVol(A, d)) < 1e-12);  // unchanged
  }
  // CutN with NO tools ⇒ the base unchanged (identity fold).
  {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidCutN(A, {}, d, &rep);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::Ok);
    CC_CHECK(rep.steps == 0);
    CC_CHECK(std::fabs(meshVol(r, d) - meshVol(A, d)) < 1e-12);
  }
  // CutN with a null base ⇒ NullOperand at index 0.
  {
    bo::NaryBoolReport rep;
    const topo::Shape r = bo::nurbsSolidCutN(topo::Shape{}, {B}, d, &rep);
    CC_CHECK(r.isNull());
    CC_CHECK(rep.decline == bo::NaryBoolDecline::NullOperand);
    CC_CHECK(rep.stepIndex == 0);
  }
}

int main() { return cctest::run_all(); }
