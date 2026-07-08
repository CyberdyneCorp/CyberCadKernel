// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2-multiseam — the FIRST multi-seam two-operand freeform
// boolean, OCCT-FREE. On the two-face corner-junction pose (A = bowl-lidded convex-quad
// prism; B = the corner box x∈[0,0.8], y∈[0,0.6], z∈[−0.6,0.2] whose x=0 AND y=0 faces
// each slice A's wall) we prove, against the mesh-free closed-form CORNER oracle:
//
//   * buildSeamGraph ADMITS the pose: EXACTLY two cutting faces, the other four contain
//     A, the two arcs are orthogonal iso-parametric curves on the wall, the junction J is
//     computed ANALYTICALLY and lies on BOTH cutting planes inside the trimmed wall, and
//     the two arcs join at J into one bent boundary→J→boundary seam (design §2, §9 L3);
//   * the seam graph is B2-CONSISTENT: EACH full arc individually splits the wall through
//     the byte-frozen B2 splitFace (the two single-seam partitions the weld composes);
//   * the closed-form corner oracle identities hold to machine precision:
//     V(A∩B)+V(A−B)=V(A) and V(A∪B)=V(A)+V(B)−V(A∩B), with V(A∩B)≈0.051275,
//     V(A−B)≈0.145035, V(A∪B)≈0.529035 (design §1, §6);
//   * the junction-aware WALL SPLIT now LANDS (this wave): where the byte-frozen B2
//     splitFace declines RebuildMismatch on the bent joined seam (its fixed-density
//     reflatten shortcuts the sharp interior kink at J, losing ~1e-5·parentArea),
//     splitFaceJunction introduces J as an EXACT shared valence-3 vertex — two seam edges
//     meeting at J — so each straight-in-UV half reflattens to MACHINE PRECISION under the
//     SAME strict rebuild tolerance (never weakened). The wall partitions into the corner
//     sub-face + the L-shaped survivor, tilingGap and rebuildResidual ~1e-16, and the
//     corner UV area equals the closed-form Q∩{u≥½,v≥½} projection to ~1e-16;
//   * the MEASURED next blocker is now the full multi-FACE corner-clip weld: the box
//     straddles A's footprint quad, so the x=0/y=0 planes also corner-clip A's flat BOTTOM
//     and the two side walls, and two box cap faces must be synthesized + the shell welded
//     across multiple junctions. freeformBooleanMultiSeam builds+proves the graph AND the
//     junction-aware wall split, then returns NULL → OCCT with MultiFaceWeldUnreachable;
//   * the honest-decline envelope: a single-cut box and a non-freeform operand each
//     decline to a NULL Shape — never a leaky/partial/wrong solid.
// Requires CYBERCAD_HAS_NUMSCI (the graph traces the real M1 seam).
//
#include "native/boolean/freeform_operand.h"
#include "native/boolean/junction_split.h"
#include "native/boolean/multi_seam.h"
#include "native/boolean/seam_graph.h"

#include "native/first_freeform_boolean_fixture.h"
#include "native/multi_seam_fixture.h"
#include "native/two_operand_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace msx = multi_seam_fixture;
namespace tox = two_operand_fixture;

// ── The seam-graph builder: admits the pose, computes J on both planes, joins arcs ──
CC_TEST(seam_graph_builds_closed_two_arc_one_junction_graph) {
  const auto A = ffx::buildOperand();
  const auto B = msx::cornerBox();

  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;

  bo::SeamGraphDecline sgd = bo::SeamGraphDecline::Ok;
  const auto g = bo::buildSeamGraph(*op, B, &sgd);
  CC_CHECK(sgd == bo::SeamGraphDecline::Ok);
  CC_CHECK(g.has_value());
  if (!g) return;

  CC_CHECK(g->cutIdx[0] != g->cutIdx[1]);                 // two distinct cutting faces
  CC_CHECK(g->arcs[0].uConst != g->arcs[1].uConst);       // orthogonal iso-parametric arcs
  CC_CHECK(g->arcs[0].arc.points.size() >= std::size_t{2});
  CC_CHECK(g->arcs[1].arc.points.size() >= std::size_t{2});

  // J is computed analytically and lies on BOTH cutting planes (the graph-closes proof).
  const double diag = op->bbox.diagonal();
  const double weldTol = 1e-7 * (diag > 1.0 ? diag : 1.0);
  CC_CHECK(g->junctionPlaneResidual <= weldTol);

  // The bent joined seam runs boundary→J→boundary with J an interior shared vertex.
  CC_CHECK(g->jointSeam.points.size() >= std::size_t{3});
  bool hasJ = false;
  for (const auto& p : g->jointSeam.points)
    if (std::fabs(p.u1 - g->junctionUV.u) < 1e-12 && std::fabs(p.v1 - g->junctionUV.v) < 1e-12)
      hasJ = true;
  CC_CHECK(hasJ);
}

// ── B2-consistency (isolation proof): each full arc splits the wall via frozen B2 ──
CC_TEST(seam_graph_arcs_each_split_the_wall_via_frozen_b2) {
  const auto A = ffx::buildOperand();
  const auto B = msx::cornerBox();
  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;
  bo::SeamGraphDecline sgd = bo::SeamGraphDecline::Ok;
  const auto g = bo::buildSeamGraph(*op, B, &sgd);
  CC_CHECK(g.has_value());
  if (!g) return;

  const bo::OperandFace& wall = op->faces[op->freeform.front()];
  const bo::SplitResult s0 = bo::splitFace(wall.face, g->arcs[0].arc);
  const bo::SplitResult s1 = bo::splitFace(wall.face, g->arcs[1].arc);
  CC_CHECK(s0.ok());
  CC_CHECK(s1.ok());
  // Each single-arc partition tiles the wall exactly (the landed single-seam invariant).
  CC_CHECK(s0.crossings == 2);
  CC_CHECK(s1.crossings == 2);
}

// ── Closed-form corner-clip oracle: the exact inclusion–exclusion identities ──────
CC_TEST(closed_form_corner_oracle_identities) {
  const double vA = msx::volFull();
  const double vB = msx::kBoxVolume;
  const double vCommon = msx::volCommon();
  const double vCut = msx::volCut();
  const double vUnion = msx::volUnion();

  // Exact partition + inclusion–exclusion (mesh-free, no OCCT), to machine precision.
  CC_CHECK(std::fabs((vCommon + vCut) - vA) <= 1e-12);
  CC_CHECK(std::fabs(vUnion - (vA + vB - vCommon)) <= 1e-12);

  // The DIAGNOSE reference values (design §1), to five decimals.
  CC_CHECK(std::fabs(vA - 0.196310) <= 5e-5);
  CC_CHECK(std::fabs(vCommon - 0.051275) <= 5e-5);
  CC_CHECK(std::fabs(vCut - 0.145035) <= 5e-5);
  CC_CHECK(std::fabs(vUnion - 0.529035) <= 5e-5);
  CC_CHECK(std::fabs(vB - 0.384000) <= 1e-9);
}

// ── The junction-aware WALL SPLIT lands where byte-frozen B2 declines ─────────────
CC_TEST(junction_aware_split_lands_where_byte_frozen_b2_declines) {
  const auto A = ffx::buildOperand();
  const auto B = msx::cornerBox();

  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;
  bo::SeamGraphDecline sgd = bo::SeamGraphDecline::Ok;
  const auto g = bo::buildSeamGraph(*op, B, &sgd);
  CC_CHECK(g.has_value());
  if (!g) return;
  const bo::OperandFace& wall = op->faces[op->freeform.front()];

  // BASELINE: the byte-frozen B2 splitFace REACHES a geometrically-exact partition of the
  // joined bent seam (crossings==2, tilingGap≈0) but DECLINES RebuildMismatch — its fixed-
  // density reflatten shortcuts the interior valence-3 kink at J. (B2 stays byte-frozen.)
  const bo::SplitResult b2 = bo::splitFace(wall.face, g->jointSeam);
  CC_CHECK(b2.crossings == 2);
  CC_CHECK(b2.tilingGap <= 1e-9 * (b2.measuredArea > 1.0 ? b2.measuredArea : 1.0));
  CC_CHECK(!b2.ok());
  CC_CHECK(b2.decline == bo::SplitDecline::RebuildMismatch);

  // THE ADVANCE: the junction-aware split introduces J as an EXACT shared valence-3 vertex
  // and SUCCEEDS at the SAME strict rebuild tolerance (never weakened).
  const bo::JunctionSplitResult js =
      bo::splitFaceJunction(wall.face, g->jointSeam, g->junctionUV, g->junction3d);
  CC_CHECK(js.ok());
  CC_CHECK(js.decline == bo::JunctionDecline::Ok);
  CC_CHECK(js.crossings == 2);
  if (!js.ok()) return;
  const auto& s = *js.split;

  // Partition is EXACT to machine precision: tiling gap ≈ 0, reflatten residual ≈ 0.
  const double parea = s.parentArea > 1.0 ? s.parentArea : 1.0;
  CC_CHECK(js.tilingGap <= 1e-9 * parea);
  CC_CHECK(s.rebuildResidual <= 1e-6 * parea);   // B2's own rebuildTolFrac, satisfied EXACTLY
  CC_CHECK(s.rebuildResidual <= 1e-12);          // in fact machine-precision (straight-in-UV halves)
  CC_CHECK(std::fabs(s.parentArea - (s.areaCorner + s.areaSurvivor)) <= 1e-12);

  // The corner sub-face UV area equals the closed-form Q∩{u≥½,v≥½} projection (the removed
  // quadrant) to machine precision — grounds the partition against the mesh-free oracle.
  CC_CHECK(std::fabs(s.areaCorner - msx::uvCornerArea()) <= 1e-12);
  CC_CHECK(s.areaCorner > 0.0 && s.areaSurvivor > s.areaCorner);

  // J is the EXACT shared interior vertex: bit-identical junctionUV inside the seam chord.
  CC_CHECK(s.jIdx > 0 && s.jIdx + 1 < static_cast<int>(s.seam.size()));
  CC_CHECK(s.seam[s.jIdx].u == g->junctionUV.u && s.seam[s.jIdx].v == g->junctionUV.v);
}

// ── The full multi-FACE corner-clip weld is the measured next blocker ─────────────
CC_TEST(multi_face_weld_is_the_measured_next_blocker) {
  const auto A = ffx::buildOperand();
  const auto B = msx::cornerBox();
  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;

  // The new blocker is MEASURED, not asserted: the box straddles A's footprint quad, so
  // the x=0/y=0 planes corner-clip A's flat bottom + side walls too — a MULTI-FACE cut.
  CC_CHECK(msx::footprintStraddlesBothPlanes());

  // The entry point builds + proves the graph AND lands the junction-aware wall split,
  // then honestly DECLINES the multi-face weld → OCCT (never a leaky/partial solid).
  bo::MultiSeamReport rep;
  const auto r = bo::freeformBooleanMultiSeam(A, B, bo::MultiSeamOp::Fuse, 0.01, &rep);
  CC_CHECK(r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamDecline::MultiFaceWeldUnreachable);
  CC_CHECK(rep.graphBuilt);
  CC_CHECK(rep.arcsSplitOk);
  CC_CHECK(rep.wallJunctionSplitOk);                       // the wall split LANDED
  CC_CHECK(rep.junctionDecline == bo::JunctionDecline::Ok);
  CC_CHECK(rep.junctionCrossings == 2);
  CC_CHECK(rep.junctionRebuildResidual <= 1e-12);
  CC_CHECK(std::fabs(rep.cornerArea - msx::uvCornerArea()) <= 1e-12);
  const double diag = op->bbox.diagonal();
  CC_CHECK(rep.junctionPlaneResidual <= 1e-7 * (diag > 1.0 ? diag : 1.0));

  // CUT and COMMON take the same honest-out (the L-shape / corner is not a half-space of
  // A, so unlike the single-seam slice they cannot delegate to freeformHalfSpaceCut).
  bo::MultiSeamReport repC, repI;
  const auto rc = bo::freeformBooleanMultiSeam(A, B, bo::MultiSeamOp::Cut, 0.01, &repC);
  const auto ri = bo::freeformBooleanMultiSeam(A, B, bo::MultiSeamOp::Common, 0.01, &repI);
  CC_CHECK(rc.isNull() && repC.decline == bo::MultiSeamDecline::MultiFaceWeldUnreachable);
  CC_CHECK(ri.isNull() && repI.decline == bo::MultiSeamDecline::MultiFaceWeldUnreachable);
}

// ── Honest-decline envelope: never emit a leaky/partial/wrong solid ───────────────
CC_TEST(single_cut_box_declines_to_the_single_seam_path) {
  // The single-curved-cut box (only x=0 slices A's wall) is the single-seam path's job:
  // the multi-seam builder DECLINES it (exactly ONE cutting face, not two).
  const auto A = ffx::buildOperand();
  const auto B = tox::buildBoxB();
  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;

  bo::SeamGraphDecline sgd = bo::SeamGraphDecline::Ok;
  const auto g = bo::buildSeamGraph(*op, B, &sgd);
  CC_CHECK(!g.has_value());
  CC_CHECK(sgd == bo::SeamGraphDecline::NotTwoCuttingFaces);

  bo::MultiSeamReport rep;
  const auto r = bo::freeformBooleanMultiSeam(A, B, bo::MultiSeamOp::Fuse, 0.01, &rep);
  CC_CHECK(r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamDecline::SeamGraphDeclined);
  CC_CHECK(rep.subDecline == bo::SeamGraphDecline::NotTwoCuttingFaces);
}

CC_TEST(non_freeform_operand_declines_null) {
  const auto v = cybercad::native::topology::ShapeBuilder::makeVertex(tmath::Point3{0, 0, 0});
  const auto B = msx::cornerBox();
  bo::MultiSeamReport rep;
  const auto r = bo::freeformBooleanMultiSeam(v, B, bo::MultiSeamOp::Fuse, 0.01, &rep);
  CC_CHECK(r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamDecline::NotAdmittedA);
}

CC_RUN_ALL()
