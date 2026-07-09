// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2 blocker #4 (≥3-seam seam graph) — the OCCT-FREE proof that
// the THREE-arc, TWO-junction seam-graph CHAIN builder (`buildChainSeamGraph`) lands in
// ISOLATION. On the reachable edge-straddling box fixture (`A` = the bowl-lidded quad
// prism; `B` = the box straddling one edge, three faces slicing `A`'s Bézier wall):
//   * exactly THREE `B` faces straddle the wall, the other three contain `A`;
//   * the three traced arcs are iso-parametric — two PARALLEL iso-`u` end arcs + one
//     ORTHOGONAL iso-`v` middle arc (the chain topology);
//   * both junctions `J1`, `J2` are computed ANALYTICALLY, lie INSIDE the trimmed wall
//     and ON BOTH of their adjacent cutting planes (residual < weldTol);
//   * the three arcs clip + join into ONE bent `chainSeam` (boundary→J1→J2→boundary)
//     with `J1`, `J2` as exact interior vertices, closing within tol;
//   * the strip-clip closed-form volume oracle is self-consistent (partition identity
//     V(A∩B)+V(A−B)=V(A), a strictly-interior discriminating strip).
// A non-chain box (the two-face corner box) and a non-freeform operand DECLINE.
// Requires CYBERCAD_HAS_NUMSCI (the arcs are the real S3 trace).
//
#include "native/boolean/freeform_operand.h"
#include "native/boolean/seam_graph_chain.h"
#include "native/tessellate/surface_eval.h"

#include "native/chain_seam_fixture.h"
#include "native/first_freeform_boolean_fixture.h"
#include "native/multi_seam_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace topo = cybercad::native::topology;
namespace tmath = cybercad::native::math;
namespace tess = cybercad::native::tessellate;
namespace ffx = first_freeform_boolean_fixture;
namespace msx = multi_seam_fixture;
namespace csx = chain_seam_fixture;

// ── The chain seam-graph builder: three iso-arcs, two analytic junctions, joined ──
CC_TEST(chain_seam_graph_builds_three_arc_two_junction_chain) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();

  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;

  bo::ChainSeamDecline d = bo::ChainSeamDecline::Ok;
  const auto g = bo::buildChainSeamGraph(*op, B, &d);
  CC_CHECK(d == bo::ChainSeamDecline::Ok);
  CC_CHECK(g.has_value());
  if (!g) return;

  // three distinct cutting faces.
  CC_CHECK(g->cutIdx[0] != g->cutIdx[1]);
  CC_CHECK(g->cutIdx[0] != g->cutIdx[2]);
  CC_CHECK(g->cutIdx[1] != g->cutIdx[2]);

  // chain topology: two parallel iso-u end arcs + one orthogonal iso-v middle arc.
  CC_CHECK(g->arcs[0].uConst == g->arcs[1].uConst);       // ends parallel
  CC_CHECK(g->arcs[0].uConst != g->arcs[2].uConst);       // middle orthogonal
  CC_CHECK(g->arcs[0].isoVal < g->arcs[1].isoVal);        // canonical order end0 < end1
  for (int k = 0; k < 3; ++k)
    CC_CHECK(g->arcs[k].arc.points.size() >= std::size_t{2});

  // both junctions on BOTH planes, inside the wall (residual << weldTol).
  const double diag = op->bbox.diagonal();
  const double weldTol = 1e-7 * (diag > 1.0 ? diag : 1.0);
  CC_CHECK(g->junctionPlaneResidual <= weldTol);
  CC_CHECK(g->junctionPlaneResidual < 1e-9);              // measured ~5e-13
  // J1 at (u(end0), v(mid)); J2 at (u(end1), v(mid)); distinct.
  CC_CHECK(tmath::distance(g->junction3d[0], g->junction3d[1]) > 0.1);

  // the chain closes: bent boundary→J1→J2→boundary with J1,J2 as interior vertices.
  CC_CHECK(g->junctionJoinGap <= 0.02 * diag);
  CC_CHECK(g->chainSeam.points.size() >= std::size_t{5});
  // J1 and J2 appear as exact interior nodes (shared position, on both planes).
  bool sawJ1 = false, sawJ2 = false;
  for (const auto& p : g->chainSeam.points) {
    if (tmath::distance(p.point, g->junction3d[0]) < 1e-12) sawJ1 = true;
    if (tmath::distance(p.point, g->junction3d[1]) < 1e-12) sawJ2 = true;
  }
  CC_CHECK(sawJ1 && sawJ2);
}

// ── The middle arc is bounded by BOTH junctions; the end arcs each by ONE + boundary ──
CC_TEST(chain_seam_middle_arc_spans_both_junctions) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  CC_CHECK(op.has_value());
  if (!op) return;
  const auto g = bo::buildChainSeamGraph(*op, B);
  CC_CHECK(g.has_value());
  if (!g) return;

  // The middle arc (arcs[2], iso-v) fixes v; its two junctions differ in u (the ends'
  // iso-u values). The two end arcs (iso-u) fix distinct u; both junctions share v.
  const bool endUConst = g->arcs[0].uConst;  // true for this pose
  CC_CHECK(endUConst);
  CC_CHECK(std::fabs(g->junctionUV[0].v - g->junctionUV[1].v) < 1e-9);  // same v (middle)
  CC_CHECK(std::fabs(g->junctionUV[0].u - g->arcs[0].isoVal) < 1e-6);
  CC_CHECK(std::fabs(g->junctionUV[1].u - g->arcs[1].isoVal) < 1e-6);
  CC_CHECK(std::fabs(g->junctionUV[0].v - g->arcs[2].isoVal) < 1e-6);
}

// ── The strip-clip closed-form oracle: a strictly-interior discriminating removal ──
CC_TEST(chain_seam_strip_oracle_partition_identity) {
  const double vFull = csx::volFull();
  const double vCommon = csx::volCommon();  // removed strip
  const double vCut = csx::volCut();
  CC_CHECK(vCommon > 0.05 * vFull);         // discriminating (strip removes a real chunk)
  CC_CHECK(vCommon < 0.6 * vFull);          // but not the whole solid (survivor remains)
  CC_CHECK(std::fabs((vCommon + vCut) - vFull) <= 1e-12);   // partition identity
  CC_CHECK(std::fabs(csx::volUnion() - (vFull + csx::kBoxVolume - vCommon)) <= 1e-12);
}

// ── A two-face corner box is NOT the chain pose (only 2 cutting faces) → decline ──
CC_TEST(chain_seam_two_face_box_declines) {
  const auto A = ffx::buildOperand();
  const auto B = msx::cornerBox();  // the landed 2-face corner box
  const auto op = bo::recogniseFreeformSolid(A);
  CC_CHECK(op.has_value());
  if (!op) return;
  bo::ChainSeamDecline d = bo::ChainSeamDecline::Ok;
  const auto g = bo::buildChainSeamGraph(*op, B, &d);
  CC_CHECK(!g.has_value());
  CC_CHECK(d == bo::ChainSeamDecline::NotThreeCuttingFaces);
}

// ── A non-freeform operand declines (no single freeform wall) ──
CC_TEST(chain_seam_non_freeform_operand_declines) {
  const auto v = topo::ShapeBuilder::makeVertex(tmath::Point3{0, 0, 0});
  const auto op = bo::recogniseFreeformSolid(v);
  CC_CHECK(!op.has_value());
}

CC_RUN_ALL()
