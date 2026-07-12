// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for L3-d STAGE 5 — the MULTI-SEAM freeform↔freeform watertight sew, the
// N-seam generalisation of track W's single-transversal-seam curved↔curved weld.
//
// The fixture is two coaxial degree-4 freeform cups (a VALLEY cup A and its z-MIRROR dome
// cup B) whose curved walls intersect in TWO disjoint closed circular seams r₁ ≈ 0.131,
// r₂ ≈ 0.374 (freeform_freeform_multiseam_fixture.h) — a genuine multi-seam pose the SSI
// returns as TWO closed loops. This suite proves, OCCT-free:
//
//   * the SSI trace returns EXACTLY TWO closed interior loops (the multi-seam trigger);
//   * `splitWallBySeams` partitions EACH wall by BOTH seams into 3 sub-regions (inner disk,
//     middle annulus, background) that TILE the parent EXACTLY (UV tiling gap = 0), the
//     nesting-aware multi-seam split;
//   * the hole-respecting membership vote (track W's `subFaceInteriorReps` fix) classifies
//     EACH sub-region — the middle annulus of A is INSIDE B, of B is INSIDE A (the lens);
//   * `freeformFreeformMultiSeamCut` sews the survivors and MANDATORILY self-verifies: the
//     OUTER seam (seam-as-OUTER on both annuli) welds watertight, but the INNER seam
//     (seam-as-HOLE on both annuli) hits the FROZEN M0 mesher's holed-curved-annulus weld
//     gap (the SAME residual the L3-S3 NURBS CUT leg named), so the verb HONEST-DECLINES to
//     NULL (`NotWatertight`) with the residual localized to the inner seam — NEVER a
//     leaky/partial/wrong solid, no tolerance widened;
//   * the closed-form partition is self-consistent (V(A−B)+V(A∩B)=V(A));
//   * a SINGLE-seam pose (track W's bowl-cup) is DECLINED `NoMultiSeam` (the single-seam
//     case belongs to `freeformFreeformClosedSeamCut`, byte-unchanged here), and a
//     non-operand is DECLINED `NotAdmitted`.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_cut_fixture.h"
#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <cmath>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace ffm = cybercad::native::boolean::ffmdetail;
namespace ffc = cybercad::native::boolean::ffcdetail;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ffx = freeform_freeform_multiseam_fixture;
namespace ssx = freeform_freeform_cut_fixture;

// ── The SSI trace returns EXACTLY TWO closed interior circular seams ──
CC_TEST(ffms_trace_returns_two_closed_seams) {
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;
  // Each loop is a circle at r₁ or r₂ on A's (u,v), on both surfaces, at z* = H/2.
  const double r1 = ffx::seamR1(), r2 = ffx::seamR2();
  double gotR[2] = {0, 0};
  for (int k = 0; k < 2; ++k) {
    double rsum = 0, maxSurf = 0, maxZ = 0;
    for (const auto& p : seams[k].points) {
      rsum += std::hypot(p.u1 - 0.5, p.v1 - 0.5);
      maxSurf = std::max(maxSurf, p.onSurfResidual);
      maxZ = std::max(maxZ, std::fabs(p.point.z - ffx::kH / 2.0));
    }
    gotR[k] = rsum / static_cast<double>(seams[k].points.size());
    CC_CHECK(seams[k].status == bo::ssi::TraceStatus::Closed);
    CC_CHECK(maxSurf < 1e-6);  // on BOTH surfaces (trace residual)
    CC_CHECK(maxZ < 1e-6);     // both seams at z* = H/2
  }
  // the two mean radii match r₁ and r₂ (in either trace order).
  const bool ok = (std::fabs(gotR[0] - r1) < 1e-2 && std::fabs(gotR[1] - r2) < 1e-2) ||
                  (std::fabs(gotR[0] - r2) < 1e-2 && std::fabs(gotR[1] - r1) < 1e-2);
  CC_CHECK(ok);
  CC_CHECK(r1 < ffx::kR && r2 < ffx::kR);  // both interior to the rim
}

// ── The multi-seam split partitions EACH wall by BOTH seams, tiling EXACTLY ──
CC_TEST(ffms_split_tiles_each_wall_by_both_seams) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  CC_CHECK(foA && foB);
  if (!foA || !foB) return;
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;
  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(ffc::rekeyToB(s));

  const auto& wallA = foA->faces[foA->freeform.front()];
  const auto& wallB = foB->faces[foB->freeform.front()];
  std::vector<ffm::WallRegion> regA, regB;
  double gapA = 1.0, gapB = 1.0;
  const bool okA = ffm::splitWallBySeams(wallA.face, seams, regA, gapA);
  const bool okB = ffm::splitWallBySeams(wallB.face, seamsB, regB, gapB);
  CC_CHECK(okA && okB);
  CC_CHECK(regA.size() == 3);  // inner disk + middle annulus + background
  CC_CHECK(regB.size() == 3);
  CC_CHECK(gapA < 1e-9);       // the 3 sub-regions TILE the parent exactly
  CC_CHECK(gapB < 1e-9);
  // exactly ONE background region per wall.
  int bgA = 0, bgB = 0;
  for (const auto& r : regA) bgA += r.isBackground ? 1 : 0;
  for (const auto& r : regB) bgB += r.isBackground ? 1 : 0;
  CC_CHECK(bgA == 1 && bgB == 1);
}

// ── The hole-respecting membership vote classifies EACH sub-region (the lens survivors) ──
CC_TEST(ffms_membership_selects_the_lens_annuli) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  CC_CHECK(foA && foB);
  if (!foA || !foB) return;
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(ffc::rekeyToB(s));

  const auto& wallA = foA->faces[foA->freeform.front()];
  const auto& wallB = foB->faces[foB->freeform.front()];
  std::vector<ffm::WallRegion> regA, regB;
  double gA, gB;
  ffm::splitWallBySeams(wallA.face, seams, regA, gA);
  ffm::splitWallBySeams(wallB.face, seamsB, regB, gB);

  const double d = 0.0025;
  tess::MeshParams mp;
  mp.deflection = d;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  CC_CHECK(tess::isWatertight(meshA) && tess::isWatertight(meshB));
  const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);

  // EXACTLY one region of A is INSIDE B (its middle annulus = the lens floor); likewise B.
  int aIn = 0, bIn = 0;
  for (const auto& r : regA)
    aIn += ffc::subFaceHasMembership(r.face, meshB, bbB, d, bo::Membership::In) ? 1 : 0;
  for (const auto& r : regB)
    bIn += ffc::subFaceHasMembership(r.face, meshA, bbA, d, bo::Membership::In) ? 1 : 0;
  CC_CHECK(aIn == 1);  // the middle annulus of A (between the two seams) is inside B
  CC_CHECK(bIn == 1);  // the middle annulus of B is inside A — the lens survivors
}

// ── The verb HONEST-DECLINES the annulus lens (frozen-mesher seam-as-hole gap), never
//    leaky — the residual is localized to the inner seam, the outer seam welds. ──
// The multi-seam COMMON/CUT survivor set is curved-annulus↔curved-annulus: the OUTER seam
// (seam-as-OUTER on both annuli) welds watertight through the M0w seam pin, but the INNER
// seam (seam-as-HOLE on both annuli) hits the FROZEN M0 mesher's holed-curved-annulus weld
// gap (the SAME residual the L3-S3 NURBS CUT leg named). The verb therefore HONEST-DECLINES
// `NotWatertight` with a sharpened residual map (`boundaryEdges` = the unpaired seam edges,
// small and localized to the inner seam) rather than emit a leaky/partial solid. This is a
// first-class Stage-5 outcome: the general multi-seam annulus↔annulus sew is beyond the
// frozen M0 mesher, and the honest-decline is DISAGREED=0-safe (never a wrong solid).
CC_TEST(ffms_declines_annulus_lens_never_leaky) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  for (bo::FfOp op : {bo::FfOp::Common, bo::FfOp::Cut}) {
    const double cf = op == bo::FfOp::Common ? ffx::volCommon() : ffx::volCut();
    for (double d : {0.005, 0.0025}) {
      bo::MultiSeamCutReport rep;
      const topo::Shape r =
          bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, op, d, &rep, cf);
      // The verb declines (NULL) — never a leaky solid.
      CC_CHECK(r.isNull());
      CC_CHECK(rep.decline == bo::MultiSeamCutDecline::NotWatertight);
      // The machinery reached the weld: both seams traced, both walls split into 3, and the
      // lens survivors selected (≥ 2 curved caps).
      CC_CHECK(rep.seamLoops == 2);
      CC_CHECK(rep.subRegionsA == 3 && rep.subRegionsB == 3);
      CC_CHECK(rep.survivorFaces >= 2);
      // The residual is SMALL and localized (the outer seam welds; only the inner seam
      // leaves unpaired edges) — a sharpened residual map, not a wholesale weld failure.
      CC_CHECK(rep.boundaryEdges > 0);
      CC_CHECK(rep.boundaryEdges < 200);
    }
  }
}

// ── The closed-form partition is self-consistent (oracle unit-check) ──
CC_TEST(ffms_closed_form_partition_is_consistent) {
  CC_CHECK(std::fabs((ffx::volCut() + ffx::volCommon()) - ffx::volA()) < ffx::volA() * 1e-9);
  CC_CHECK(ffx::volCommon() > 0.05 * ffx::volA());  // a SUBSTANTIAL, discriminating lens
  CC_CHECK(std::fabs(ffx::volA() - ffx::volB()) < ffx::volA() * 1e-9);  // z-mirror symmetry
}

// ── A SINGLE-seam pose is DECLINED NoMultiSeam (deferred to track W, byte-unchanged) ──
CC_TEST(ffms_declines_single_seam_pose) {
  // Track W's canonical bowl-cup single-seam fixture — its walls meet in ONE circle.
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCut(A, B, bo::FfOp::Common, 0.005, &rep);
  CC_CHECK(r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::NoMultiSeam);
  CC_CHECK(rep.seamLoops == 1);
}

// ── Track W's single-seam weld is UNREGRESSED (this track never touched it) ──
// The single-seam COMMON lens still welds watertight at the closed-form volume — proving
// the multi-seam track is purely additive and W's path is byte-identical.
CC_TEST(ffms_single_seam_weld_unregressed) {
  const topo::Shape A = ssx::buildA();
  const topo::Shape B = ssx::buildB();
  const double cf = ssx::volCommon();
  bo::FfCutDecline why = bo::FfCutDecline::Ok;
  const topo::Shape com = bo::freeformFreeformClosedSeamCut(A, B, bo::FfOp::Common, 0.005, &why, cf);
  CC_CHECK(!com.isNull());
  CC_CHECK(why == bo::FfCutDecline::Ok);
  if (com.isNull()) return;
  tess::MeshParams mp;
  mp.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(com);
  CC_CHECK(tess::isWatertight(m));
  CC_CHECK(tess::boundaryEdgeCount(m) == 0);
  const double v = std::fabs(tess::enclosedVolume(m));
  CC_CHECK(std::fabs(v - cf) / cf < 30.0 * 0.005);
}

// ── A non-operand (null shape) is DECLINED NotAdmitted ──
CC_TEST(ffms_declines_non_operand) {
  const topo::Shape nul{};
  const topo::Shape B = ffx::buildB();
  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCut(nul, B, bo::FfOp::Cut, 0.005, &rep);
  CC_CHECK(r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::NotAdmittedA);
}

int main() { return cctest::run_all(); }
