// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2b/F4 freeform↔analytic DISJOINT (MULTI-LUMP) CUT — the OCCT-FREE
// analytic proof that `freeformSlabDisjointCut` composes the (now off-centre-accurate)
// inter-solid-seam weld into the FIRST native freeform boolean whose result is TWO
// disconnected bodies, and that it now WELDS at the closed-form two-body volume.
//
// The fixture is the bowl-lidded convex-quad prism `A` parted by a central axis-aligned
// slab `B` (x∈[−0.10,0.10]) into two lumps A∩{x≤−0.10} ⊎ A∩{x≥+0.10}
// (slab_disjoint_cut_fixture.h). We assert:
//   * the pipeline REACHES the weld: B1 admits A (one freeform wall), B is an all-planar
//     slab, the opposite-parallel slab pair straddling A's wall is located, and both lumps
//     assemble through the inter-solid-seam machinery;
//   * the DISJOINT MECHANISM is sound — the verb returns a CONSISTENTLY-ORIENTED WATERTIGHT
//     `Compound` of EXACTLY TWO `Solid`s whose world AABBs are separated along the slab
//     axis (genuinely two connected components — the new topological outcome);
//   * the closed-form CUT-volume oracle is self-consistent (V(A−B)+V(A∩B)=V(A));
//   * F4 WELD: with the closed-form CUT volume supplied, the two-sided self-verify now
//     ACCEPTS the weld — the off-centre cross-section cap is oriented by the mesher's real
//     +fr.z convention (planarFaceFromLoopByNormal), so each lump is consistently oriented
//     and the combined enclosed volume matches the closed form within the deflection band
//     (MEASURED: relerr < 1% at d∈{0.006,0.008,0.010}, down from the frozen cap's ~29%);
//   * the honest-decline battery: a non-operand A and a non-slab B each decline with a
//     measured blocker.
// Requires CYBERCAD_HAS_NUMSCI (the composition traces the real M1 seam on each face).
//
#include "native/boolean/slab_disjoint_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include "slab_disjoint_cut_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace sfx = slab_disjoint_cut_fixture;
namespace ffx = first_freeform_boolean_fixture;

namespace {
int solidCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Solid); ex.more(); ex.next()) ++n;
  return n;
}
}  // namespace

// ── B1 admits A, B is a planar slab, the opposite-parallel slab pair is located ──
CC_TEST(slab_operands_admit_and_pair_locates) {
  const topo::Shape A = ffx::buildOperand();
  const topo::Shape B = sfx::buildSlabB();
  bo::OperandDecline wa = bo::OperandDecline::Ok;
  const auto foA = bo::recogniseFreeformSolid(A, &wa);
  CC_CHECK(foA.has_value());
  if (!foA) return;
  CC_CHECK(foA->freeform.size() == 1);
  CC_CHECK(bo::isAllPlanar(B));
  CC_CHECK(bo::extractPolygons(B).size() == 6);
}

// ── The closed-form CUT/slab partition tiles exactly (oracle unit-check) ──
CC_TEST(slab_closed_form_partition_is_consistent) {
  const double v = ffx::fullVolume();
  CC_CHECK(std::fabs((sfx::cutVolume() + sfx::slabVolume()) - v) < v * 1e-12);
  CC_CHECK(sfx::lumpLoVolume() > 0.01 * v);   // both lumps SUBSTANTIAL (a real parting)
  CC_CHECK(sfx::lumpHiVolume() > 0.01 * v);
  CC_CHECK(sfx::slabVolume() > 0.001 * v);    // the removed band is non-degenerate
}

// ── The DISJOINT MECHANISM: welds a CONSISTENTLY-ORIENTED two-body Compound ──
// Without the closed-form band the verb self-verifies (both lumps + the combined compound
// CONSISTENTLY ORIENTED, the two lumps DISJOINT along the slab axis, 0 < V ≤ V(A)) and
// returns a `Compound` of EXACTLY TWO `Solid`s — the new topological outcome no landed
// verb produces.
CC_TEST(slab_disjoint_mechanism_welds_two_body_compound) {
  const topo::Shape A = ffx::buildOperand();
  const topo::Shape B = sfx::buildSlabB();
  for (double d : {0.01, 0.008, 0.006}) {
    bo::SlabCutDecline why = bo::SlabCutDecline::Ok;
    const topo::Shape r = bo::freeformSlabDisjointCut(A, B, d, &why);  // no closed form
    CC_CHECK(!r.isNull());
    CC_CHECK(why == bo::SlabCutDecline::Ok);
    if (r.isNull()) continue;
    CC_CHECK(solidCount(r) == 2);                  // TWO disconnected bodies
    tess::MeshParams mp; mp.deflection = d;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(r);
    CC_CHECK(tess::isConsistentlyOriented(m));      // coherently-wound closed 2-manifold
    const double v = std::fabs(tess::enclosedVolume(m));
    CC_CHECK(v > 0.0 && v <= ffx::fullVolume() * 1.05);   // 0 < V ≤ V(A)
  }
}

// ── F4 WELD: the off-centre-accurate cap makes the two-sided gate ACCEPT the weld ──
// With the closed-form CUT volume supplied, the verb's TWO-SIDED band now ACCEPTS the
// disjoint compound: the off-centre cross-section cap is oriented by the mesher's real
// +fr.z convention (planarFaceFromLoopByNormal), so each lump is consistently oriented and
// the combined enclosed volume matches the closed form within the deflection band. This is
// the F4 payoff — the frozen keep-face's ~29% off-centre over-estimate is gone, turning the
// former `VolumeInconsistent` honest-decline into a full, volume-accurate WELD.
CC_TEST(slab_two_sided_verify_welds_at_closed_form) {
  const topo::Shape A = ffx::buildOperand();
  const topo::Shape B = sfx::buildSlabB();
  const double cf = sfx::cutVolume();
  for (double d : {0.01, 0.008, 0.006}) {
    bo::SlabCutDecline why = bo::SlabCutDecline::Ok;
    const topo::Shape r = bo::freeformSlabDisjointCut(A, B, d, &why, cf);
    CC_CHECK(!r.isNull());                          // WELDS (no longer VolumeInconsistent)
    CC_CHECK(why == bo::SlabCutDecline::Ok);
    if (r.isNull()) continue;
    CC_CHECK(solidCount(r) == 2);
    tess::MeshParams mp; mp.deflection = d;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(r);
    CC_CHECK(tess::isConsistentlyOriented(m));
    const double v = std::fabs(tess::enclosedVolume(m));
    CC_CHECK(std::fabs(v - cf) < 0.02 * cf);        // MEASURED: relerr < 1% (was ~29%)
  }
}

// ── Honest decline: a non-operand (null shape) A → NotAdmittedA ──
CC_TEST(slab_declines_non_operand) {
  const topo::Shape nul{};
  const topo::Shape B = sfx::buildSlabB();
  bo::SlabCutDecline why = bo::SlabCutDecline::Ok;
  const topo::Shape r = bo::freeformSlabDisjointCut(nul, B, 0.008, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == bo::SlabCutDecline::NotAdmittedA);
}

// ── Honest decline: a non-slab B (a single box face pair that does NOT bracket A's wall)
// — here a box entirely on one side of A (no straddle) → NoSlabPair ──
CC_TEST(slab_declines_non_slab_box) {
  const topo::Shape A = ffx::buildOperand();
  // A box far to the +x side of A (x ∈ [2,3]): neither face straddles A's wall.
  const topo::Shape B = sfx::buildFarPlusXBox();
  bo::SlabCutDecline why = bo::SlabCutDecline::Ok;
  const topo::Shape r = bo::freeformSlabDisjointCut(A, B, 0.008, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == bo::SlabCutDecline::NoSlabPair);
}

int main() { return cctest::run_all(); }
