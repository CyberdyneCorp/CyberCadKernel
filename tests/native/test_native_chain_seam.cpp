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
#include "native/boolean/face_split.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/multi_face_strip_weld.h"
#include "native/boolean/seam_graph_chain.h"
#include "native/boolean/strip_split.h"
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

// ── The 2-junction WALL SPLIT lands where byte-frozen B2 declines on the chain seam ──
CC_TEST(strip_split_lands_where_byte_frozen_b2_declines_two_junction) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  CC_CHECK(op.has_value());
  if (!op) return;
  const auto g = bo::buildChainSeamGraph(*op, B);
  CC_CHECK(g.has_value());
  if (!g) return;
  const bo::OperandFace& wall = op->faces[op->freeform.front()];

  // BASELINE: byte-frozen B2 splitFace finds the two boundary crossings (crossings==2)
  // but DECLINES on the bent chain seam — both crossings land on ONE boundary edge (the
  // strip pokes through the top edge), so B2's whole-vertex boundary-arc walk cannot
  // separate the strip from the survivor (its two sub-loops collapse to the same strip
  // area, a TilingGap). B2 stays byte-frozen; the strip verb resolves this by
  // construction (a full-ring wrap + J1/J2 as exact vertices).
  const bo::SplitResult b2 = bo::splitFace(wall.face, g->chainSeam);
  CC_CHECK(b2.crossings == 2);
  CC_CHECK(!b2.ok());
  CC_CHECK(b2.decline == bo::SplitDecline::TilingGap);

  // THE ADVANCE: the two-junction split introduces J1, J2 as EXACT shared valence-3
  // vertices (three seam edges) and SUCCEEDS at the SAME strict rebuild tolerance.
  const bo::StripSplitResult ss =
      bo::splitFaceStrip(wall.face, g->chainSeam, g->junctionUV[0], g->junction3d[0],
                         g->junctionUV[1], g->junction3d[1]);
  CC_CHECK(ss.ok());
  CC_CHECK(ss.decline == bo::StripSplitDecline::Ok);
  CC_CHECK(ss.crossings == 2);
  if (!ss.ok()) return;
  const auto& s = *ss.split;

  const double parea = s.parentArea > 1.0 ? s.parentArea : 1.0;
  CC_CHECK(ss.tilingGap <= 1e-9 * parea);
  CC_CHECK(s.rebuildResidual <= 1e-6 * parea);   // B2's own rebuildTolFrac, satisfied EXACTLY
  CC_CHECK(s.rebuildResidual <= 1e-12);          // in fact machine-precision (straight-in-UV parts)
  CC_CHECK(std::fabs(s.parentArea - (s.areaStrip + s.areaSurvivor)) <= 1e-12);

  // The strip sub-face UV area equals the closed-form Q∩{u0≤u≤u1,v≥v0} projection.
  CC_CHECK(std::fabs(s.areaStrip - csx::uvStripArea()) <= 1e-12);
  CC_CHECK(s.areaStrip > 0.0 && s.areaSurvivor > s.areaStrip);

  // J1, J2 are EXACT shared interior vertices in the seam chord.
  CC_CHECK(s.j1Idx > 0 && s.j1Idx < s.j2Idx && s.j2Idx + 1 < static_cast<int>(s.seam.size()));
  CC_CHECK(s.seam[s.j1Idx].u == g->junctionUV[0].u && s.seam[s.j1Idx].v == g->junctionUV[0].v);
  CC_CHECK(s.seam[s.j2Idx].u == g->junctionUV[1].u && s.seam[s.j2Idx].v == g->junctionUV[1].v);
}

// ── The 2-junction STRIP WELD lands watertight CUT/COMMON at the closed-form volumes ──
CC_TEST(strip_weld_lands_watertight_cut_common_at_closed_form_volumes) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  CC_CHECK(op.has_value());
  if (!op) return;
  const auto g = bo::buildChainSeamGraph(*op, B);
  CC_CHECK(g.has_value());
  if (!g) return;
  const bo::OperandFace& wall = op->faces[op->freeform.front()];
  const bo::StripSplitResult ss =
      bo::splitFaceStrip(wall.face, g->chainSeam, g->junctionUV[0], g->junction3d[0],
                         g->junctionUV[1], g->junction3d[1]);
  CC_CHECK(ss.ok());
  if (!ss.ok()) return;

  struct Case { bo::StripWeldOp op; double oracle; const char* name; };
  const Case cases[3] = {{bo::StripWeldOp::Cut, csx::volCut(), "CUT"},
                         {bo::StripWeldOp::Common, csx::volCommon(), "COMMON"},
                         {bo::StripWeldOp::Fuse, csx::volUnion(), "FUSE"}};
  for (const Case& c : cases) {
    bo::StripWeldReport rep;
    const auto r = bo::multiFaceStripClip(*op, *g, *ss.split, c.op, 0.01, &rep);
    // A real watertight result solid (never NULL for this reachable strip pose).
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::StripWeldDecline::Ok);
    CC_CHECK(rep.watertight);
    CC_CHECK(rep.faceCount >= 5);
    // Enclosed volume matches the CLOSED-FORM strip oracle to the curved-tessellation band
    // (deflection 0.01); no tolerance weakened — the mesh converges from above.
    CC_CHECK(std::fabs(rep.volume - c.oracle) <= 2e-2 * c.oracle);
  }
}

// ── The 2-junction STRIP-WELD FUSE lands watertight at V(A∪B), fixing the two-attach- ──
//    column box notch (the prior wave's measured next blocker: the naive 3-notched-box
//    FUSE left open + non-manifold edges at the J1/J2 vertical columns). The MIDDLE cutting
//    face's cap spans the full box width and attaches along BOTH junction columns, so it is
//    split into a TOP + BOTTOM piece (`splitMiddleBoxFace`); the two END faces keep the
//    single-column corner-clip notch. Additive: CUT/COMMON are byte-unchanged.
CC_TEST(strip_weld_fuse_lands_watertight_at_union_volume) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  CC_CHECK(op.has_value());
  if (!op) return;
  const auto g = bo::buildChainSeamGraph(*op, B);
  CC_CHECK(g.has_value());
  if (!g) return;
  const bo::OperandFace& wall = op->faces[op->freeform.front()];
  const bo::StripSplitResult ss =
      bo::splitFaceStrip(wall.face, g->chainSeam, g->junctionUV[0], g->junction3d[0],
                         g->junctionUV[1], g->junction3d[1]);
  CC_CHECK(ss.ok());
  if (!ss.ok()) return;

  bo::StripWeldReport rep;
  const auto r = bo::multiFaceStripClip(*op, *g, *ss.split, bo::StripWeldOp::Fuse, 0.01, &rep);
  CC_CHECK(!r.isNull());                                 // the FUSE now welds (was NULL before)
  CC_CHECK(rep.decline == bo::StripWeldDecline::Ok);
  CC_CHECK(rep.watertight);                              // the J1/J2 columns are now closed 2-manifold
  // V(A∪B) is the DISCRIMINATING union: strictly larger than both operands (it is NOT a
  // subset op), and equals the closed-form V(A) + V(box) − V(A∩B).
  CC_CHECK(csx::volUnion() > rep.volA + 1e-6);
  CC_CHECK(csx::volUnion() > rep.volB + 1e-6);
  CC_CHECK(std::fabs(rep.volume - csx::volUnion()) <= 2e-2 * csx::volUnion());
  // and it sits inside the inclusion–exclusion bound max(V(A),V(B)) ≤ V ≤ V(A)+V(B).
  CC_CHECK(rep.volume >= std::max(rep.volA, rep.volB) - 1e-6);
  CC_CHECK(rep.volume <= rep.volA + rep.volB + 1e-6);
}

// ── The FUSE union volume converges to the closed-form oracle as deflection → 0 ──
CC_TEST(strip_weld_fuse_volume_converges_across_deflection) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  if (!op) { CC_CHECK(false); return; }
  const auto g = bo::buildChainSeamGraph(*op, B);
  if (!g) { CC_CHECK(false); return; }
  const bo::OperandFace& wall = op->faces[op->freeform.front()];
  const bo::StripSplitResult ss =
      bo::splitFaceStrip(wall.face, g->chainSeam, g->junctionUV[0], g->junction3d[0],
                         g->junctionUV[1], g->junction3d[1]);
  if (!ss.ok()) { CC_CHECK(false); return; }
  double prevErr = 1e30;
  for (double d : {0.02, 0.01, 0.005}) {
    bo::StripWeldReport rep;
    const auto r = bo::multiFaceStripClip(*op, *g, *ss.split, bo::StripWeldOp::Fuse, d, &rep);
    CC_CHECK(!r.isNull() && rep.watertight);
    const double err = std::fabs(rep.volume - csx::volUnion());
    CC_CHECK(err <= prevErr + 1e-9);   // monotone-converging (curved tessellation)
    prevErr = err;
  }
  CC_CHECK(prevErr <= 1e-2 * csx::volUnion());
}

// ── The strip-weld CUT volume converges monotonically to the closed-form oracle ──
CC_TEST(strip_weld_cut_volume_converges_across_deflection) {
  const auto A = ffx::buildOperand();
  const auto B = csx::edgeBox();
  const auto op = bo::recogniseFreeformSolid(A);
  if (!op) { CC_CHECK(false); return; }
  const auto g = bo::buildChainSeamGraph(*op, B);
  if (!g) { CC_CHECK(false); return; }
  const bo::OperandFace& wall = op->faces[op->freeform.front()];
  const bo::StripSplitResult ss =
      bo::splitFaceStrip(wall.face, g->chainSeam, g->junctionUV[0], g->junction3d[0],
                         g->junctionUV[1], g->junction3d[1]);
  if (!ss.ok()) { CC_CHECK(false); return; }
  double prevErr = 1e30;
  for (double d : {0.02, 0.01, 0.005}) {
    bo::StripWeldReport rep;
    const auto r = bo::multiFaceStripClip(*op, *g, *ss.split, bo::StripWeldOp::Cut, d, &rep);
    CC_CHECK(!r.isNull() && rep.watertight);
    const double err = std::fabs(rep.volume - csx::volCut());
    CC_CHECK(err <= prevErr + 1e-9);   // monotone-converging (curved tessellation)
    prevErr = err;
  }
  CC_CHECK(prevErr <= 1e-2 * csx::volCut());
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
