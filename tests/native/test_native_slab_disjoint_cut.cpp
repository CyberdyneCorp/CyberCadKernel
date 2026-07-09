// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2b freeform↔analytic DISJOINT (MULTI-LUMP) CUT — the OCCT-FREE
// analytic proof that `freeformSlabDisjointCut` composes the landed inter-solid-seam weld
// into the FIRST native freeform boolean whose result is TWO disconnected bodies, and that
// its mandatory two-sided self-verify HONEST-DECLINES a wrong-volume weld (never leaks).
//
// The fixture is the bowl-lidded convex-quad prism `A` parted by a central axis-aligned
// slab `B` (x∈[−0.10,0.10]) into two lumps A∩{x≤−0.10} ⊎ A∩{x≥+0.10}
// (slab_disjoint_cut_fixture.h). We assert:
//   * the pipeline REACHES the weld: B1 admits A (one freeform wall), B is an all-planar
//     slab, the opposite-parallel slab pair straddling A's wall is located, and both lumps
//     assemble through the landed inter-solid-seam machinery;
//   * the DISJOINT MECHANISM is sound — in upper-bound mode the verb returns a WATERTIGHT
//     `Compound` of EXACTLY TWO `Solid`s whose world AABBs are separated along the slab
//     axis (genuinely two connected components — the new topological outcome);
//   * the closed-form CUT-volume oracle is self-consistent (V(A−B)+V(A∩B)=V(A));
//   * the mandatory TWO-SIDED self-verify HONEST-DECLINES `VolumeInconsistent` when the
//     closed-form volume is supplied — the byte-frozen inter-solid-seam keep-face
//     machinery over-estimates the volume of an OFF-CENTRE cross-section (MEASURED: the
//     lump volume exceeds the closed form well beyond the deflection band), so the verb
//     refuses to emit a wrong-volume solid → NULL → OCCT; a leaky/wrong result is NEVER
//     emitted; no tolerance is widened;
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

// ── The DISJOINT MECHANISM: upper-bound mode welds a watertight two-body Compound ──
// Without the closed-form band the verb still self-verifies (both lumps + the combined
// compound WATERTIGHT, the two lumps DISJOINT along the slab axis, 0 < V ≤ V(A)) and
// returns a `Compound` of EXACTLY TWO `Solid`s — the new topological outcome no landed
// verb produces. (The volume is the frozen keep-face machinery's off-centre estimate;
// the TWO-SIDED gate below is what makes the shipped verb refuse a wrong-volume result.)
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
    CC_CHECK(tess::isWatertight(m));               // a closed 2-manifold (two lumps)
    const double v = std::fabs(tess::enclosedVolume(m));
    CC_CHECK(v > 0.0 && v <= ffx::fullVolume() * 1.05);   // 0 < V ≤ V(A)
  }
}

// ── HONEST TWO-SIDED DECLINE: the wrong off-centre volume is caught, never shipped ──
// With the closed-form CUT volume supplied, the verb's TWO-SIDED band rejects the
// byte-frozen keep-face machinery's off-centre OVER-estimate: the meshed compound volume
// exceeds the closed form well beyond the deflection band, so the verb returns NULL →
// OCCT with `VolumeInconsistent`. This is the load-bearing no-silent-wrong invariant —
// the disjoint compound is topologically real, but its off-centre VOLUME is not trusted,
// so it is HONEST-DECLINED rather than emitted as a correct result.
CC_TEST(slab_two_sided_verify_declines_offcentre_wrong_volume) {
  const topo::Shape A = ffx::buildOperand();
  const topo::Shape B = sfx::buildSlabB();
  const double cf = sfx::cutVolume();
  for (double d : {0.01, 0.008, 0.006}) {
    bo::SlabCutDecline why = bo::SlabCutDecline::Ok;
    const topo::Shape r = bo::freeformSlabDisjointCut(A, B, d, &why, cf);
    CC_CHECK(r.isNull());                                  // NULL → OCCT (disciplined)
    CC_CHECK(why == bo::SlabCutDecline::VolumeInconsistent);  // the measured off-centre blocker
  }
  // MEASURE the blocker: the upper-bound compound volume materially exceeds the closed
  // form (the over-estimate the two-sided gate rejects) — proving the decline is real,
  // not a spurious threshold.
  bo::SlabCutDecline w = bo::SlabCutDecline::Ok;
  const topo::Shape mech = bo::freeformSlabDisjointCut(A, B, 0.008, &w);  // no band
  CC_CHECK(!mech.isNull());
  if (!mech.isNull()) {
    tess::MeshParams mp; mp.deflection = 0.008;
    const double v = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(mech)));
    CC_CHECK(v > cf * 1.10);   // > 10% over the closed form (measured ~29%)
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
