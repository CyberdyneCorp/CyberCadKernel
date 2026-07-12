// SPDX-License-Identifier: Apache-2.0
//
// Host GATE for BOOL-READMIT / LAYER 3 — making the binary boolean's WELDED OUTPUT
// re-admissible as a boolean INPUT (boolean_readmit.h), so the N-operand fold welds for
// ≥3 operands instead of declining at the re-admission boundary.
//
// This suite proves, OCCT-free (host closed-form + membership oracles):
//   * the MORE-PERMISSIVE recogniser `recogniseFreeformSolidReadmit` ADMITS a binary
//     boolean's holed / multi-freeform-wall output that the byte-frozen B1 gate
//     `recogniseFreeformSolid` deliberately declines (`HoledFreeformFace`) — the operand is
//     now admissible (the FIRST measured blocker cleared);
//   * `nurbsSolidBooleanReadmit` with a PRISTINE single-wall accumulator is BIT-IDENTICAL to
//     the frozen `nurbsSolidBoolean` (2-operand folds UNREGRESSED — it defers verbatim);
//   * the REDUNDANT-operand short-circuit welds the reachable idempotent re-admit folds
//     EXACTLY (a contained UNION operand, a disjoint CUT tool) at the inclusion-exclusion
//     volume, watertight (χ=2, be=0), COINCIDENCE-TOLERANT — no synthesised geometry, so
//     DISAGREED=0 is structural;
//   * a GENUINELY-OVERLAPPING third body HONEST-DECLINES at the sharpened multi-hole-split
//     boundary (never a leaky/partial solid; no tolerance widened).
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/boolean_readmit.h"
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_cut_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ssx = freeform_freeform_cut_fixture;

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

// ── The frozen gate DECLINES the boolean output; the readmit gate ADMITS it (blocker 1). ──
CC_TEST(readmit_recogniser_admits_holed_multiwall_output) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::SolidBoolReport rf;
  const topo::Shape AB = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &rf);
  CC_CHECK(!AB.isNull());
  CC_CHECK(rf.decline == bo::SolidBoolDecline::Ok);

  // The frozen gate declines the boolean output (HoledFreeformFace) — the measured blocker.
  bo::OperandDecline whyFrozen = bo::OperandDecline::Ok;
  const auto frozen = bo::recogniseFreeformSolid(AB, &whyFrozen);
  CC_CHECK(!frozen.has_value());
  CC_CHECK(whyFrozen == bo::OperandDecline::HoledFreeformFace);

  // The readmit gate ADMITS it, exposing its (multiple) freeform walls + lids.
  bo::OperandDecline whyReadmit = bo::OperandDecline::Ok;
  const auto readmit = bo::recogniseFreeformSolidReadmit(AB, &whyReadmit);
  CC_CHECK(readmit.has_value());
  CC_CHECK(whyReadmit == bo::OperandDecline::Ok);
  if (readmit) {
    CC_CHECK(readmit->freeform.size() == 2);  // A's annulus + B's annulus (both holed)
    CC_CHECK(readmit->analytic.size() == 2);  // A's lid + B's lid
    CC_CHECK(readmit->watertight);
  }
}

// ── PRISTINE-accumulator re-admit == the frozen binary boolean (2-operand UNREGRESSED). ──
CC_TEST(readmit_pristine_acc_is_binary_identical) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  for (bo::SolidBoolOp op : {bo::SolidBoolOp::Fuse, bo::SolidBoolOp::Cut, bo::SolidBoolOp::Common}) {
    bo::SolidBoolReport rb, rr;
    const topo::Shape bin = bo::nurbsSolidBoolean(A, B, op, d, &rb);
    const topo::Shape rdm = bo::nurbsSolidBooleanReadmit(A, B, op, d, &rr);
    CC_CHECK(!bin.isNull() && !rdm.isNull());
    CC_CHECK(rb.decline == bo::SolidBoolDecline::Ok);
    CC_CHECK(rr.decline == bo::SolidBoolDecline::Ok);
    if (bin.isNull() || rdm.isNull()) continue;
    // Same weld ⇒ identical meshed volume (readmit deferred verbatim, no new geometry).
    CC_CHECK(std::fabs(meshVol(bin, d) - meshVol(rdm, d)) < 1e-12);
  }
}

// ── Redundant UNION operand contained in acc ⇒ short-circuit to acc EXACTLY, watertight. ──
CC_TEST(readmit_redundant_union_operand_short_circuits) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::SolidBoolReport r1;
  const topo::Shape AB = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &r1);
  const double vAB = meshVol(AB, d);
  // B ⊆ A∪B ⇒ (A∪B) ∪ B = A∪B (no weld synthesised; the containment witness resolves it).
  bo::SolidBoolReport r2;
  const topo::Shape r = bo::nurbsSolidBooleanReadmit(AB, B, bo::SolidBoolOp::Fuse, d, &r2, vAB);
  CC_CHECK(!r.isNull());
  CC_CHECK(r2.decline == bo::SolidBoolDecline::Ok);
  if (r.isNull()) return;
  bool wt, coh;
  std::size_t be;
  double v;
  meshStats(r, d, wt, coh, be, v);
  CC_CHECK(wt);
  CC_CHECK(be == 0);
  CC_CHECK(coh);
  CC_CHECK(std::fabs(v - vAB) < 1e-9);  // EXACTLY acc (short-circuit, not a re-weld)
}

// ── Redundant CUT tool disjoint from remaining material ⇒ short-circuit to acc EXACTLY. ──
CC_TEST(readmit_redundant_cut_tool_short_circuits) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double d = 0.005;
  bo::SolidBoolReport r1;
  const topo::Shape AmB = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Cut, d, &r1);
  const double vAmB = meshVol(AmB, d);
  // (A−B) ∩ B = ∅ ⇒ (A−B) − B = A−B (the second identical tool removes nothing).
  bo::SolidBoolReport r2;
  const topo::Shape r = bo::nurbsSolidBooleanReadmit(AmB, B, bo::SolidBoolOp::Cut, d, &r2, vAmB);
  CC_CHECK(!r.isNull());
  CC_CHECK(r2.decline == bo::SolidBoolDecline::Ok);
  if (r.isNull()) return;
  bool wt, coh;
  std::size_t be;
  double v;
  meshStats(r, d, wt, coh, be, v);
  CC_CHECK(wt);
  CC_CHECK(be == 0);
  CC_CHECK(coh);
  CC_CHECK(std::fabs(v - vAmB) < 1e-9);
}

// ── A GENUINELY-OVERLAPPING third body does NOT short-circuit and HONEST-DECLINES at the
// sharpened multi-hole-split boundary (never leaky). ──
CC_TEST(readmit_genuine_overlap_honest_declines) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  auto cPoles = ssx::downDomePoles();
  const double dz = 0.06;
  for (auto& p : cPoles) p.z += dz;
  const topo::Shape C = ssx::buildCup(cPoles, (ssx::kH - ssx::kA * ssx::kR * ssx::kR) + dz);
  const double d = 0.005;
  bo::SolidBoolReport r1;
  const topo::Shape AB = bo::nurbsSolidBoolean(A, B, bo::SolidBoolOp::Fuse, d, &r1);
  bo::SolidBoolReport r2;
  const topo::Shape r = bo::nurbsSolidBooleanReadmit(AB, C, bo::SolidBoolOp::Fuse, d, &r2);
  CC_CHECK(r.isNull());  // never a leaky/partial solid
  CC_CHECK(r2.decline != bo::SolidBoolDecline::Ok);
  // The operand IS admitted + the seam IS traced (the decline is the weld residual, NOT the
  // old pre-readmit NotAdmittedA): the sharpened boundary is the multi-hole split.
  CC_CHECK(r2.decline == bo::SolidBoolDecline::NotWatertight);
}

int main() { return cctest::run_all(); }
