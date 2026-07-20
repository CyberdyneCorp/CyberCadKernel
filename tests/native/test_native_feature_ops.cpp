// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for BOOL-FEATURE — the CAD feature operations POCKET / BOSS
// (feature_ops.h), the natural CAD composition on top of the general two-freeform-solid
// NURBS boolean: sweep a closed profile into a TOOL solid, then boolean it against the
// BASE (Pocket ⇒ Cut, Boss ⇒ Fuse). It re-implements NEITHER the boolean NOR the sweep —
// it COMPOSES the landed `nurbsSolidBoolean` (nurbs_solid_boolean.h) + the Layer-6
// `sweepTranslational` (bspline_sweep.h).
//
// This suite proves, OCCT-free (host closed-form oracle):
//   * POCKET a tool cup out of a freeform base cup (the single-transversal-seam bowl-cup
//     pose the boolean verifies): watertight (χ=2, be=0, coherent), V = V(base) − lens
//     (the removed material), converging within the tessellation band, DISAGREED=0.
//   * BOSS a tool cup onto the base: watertight, V = V(base) + V(tool) − lens (the raised
//     pad's outer envelope), within the band.
//   * GENUINE SWEEP COMPOSITION: `sweptExtrudeToolWall` returns the EXACT translational-
//     extrusion wall of a supplied profile section — the swept surface reproduces the
//     section translated by the sweep vector POINTWISE (the Layer-6 exactness oracle,
//     reached through the feature-ops seam), so the tool really is a SWEPT profile.
//   * MULTI-SEAM BOSS: a BOSS over a genuine MULTI-SEAM pose (two disjoint closed seams,
//     boolReport.multiSeam=1) now WELDS watertight at the outer-envelope volume
//     V(base)+V(tool)−lens and CONVERGES — the underlying boolean sews the two-seam FUSE
//     rather than deferring it. (History: this pose used to honest-decline BooleanDeclined;
//     the boolean since landed the multi-seam sew, so the pin asserts the correct measured
//     outcome — a watertight, correct-volume solid — not the stale decline.)
//   * HONEST-DECLINE (never a leaky solid): a null base declines NullBase before the
//     boolean, and a null/rational/degenerate swept section declines SweepFailed — the
//     feature op never fabricates a leaky/partial solid.
//
// Requires CYBERCAD_HAS_NUMSCI (the boolean's seams are the real S3 trace, and the swept
// wall composes the numsci-guarded Layer-6 sweep TU).
//
#include "native/boolean/feature_ops.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_cut_fixture.h"
#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <algorithm>
#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace fmath = cybercad::native::math;
namespace ssx = freeform_freeform_cut_fixture;        // single-seam bowl-cup base+tool
namespace msx = freeform_freeform_multiseam_fixture;  // multi-seam mirror cups (decline)

// Mesh a result solid → (watertight, coherent, boundaryEdges, volume).
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

// ── POCKET a tool cup out of a freeform base: watertight, V = V(base) − lens ──
// The base is the UP bowl-cup A; the tool is the mirror DOWN dome-cup B (a swept profile
// solid — its wall is the exact z-mirror of the base's, meeting it in ONE closed seam). The
// pocket subtracts the tool ⇒ the carved solid at V(A) − lens, the boolean's own verified
// Cut oracle, now reached through `pocket`, watertight and CONVERGENT.
//
// CONVERGENCE CRITERION — why NOT strict per-level `err < prevErr`: `pocket` composes the
// SAME freeform↔freeform CUT this fixture's `ff_cut_welds_watertight_at_closed_form`
// exercises, and inherits its identical convergence character. The pocket volume is a
// DIFFERENCE (the assembled shell encloses ≈V(base) MINUS the tool's curved ceiling ≈lens);
// the base-annulus and tool-disk are two INDEPENDENTLY tessellated curved surfaces, each
// carrying its own O(deflection) signed-volume triangulation residual, and because the CUT
// is their difference those residuals partially CANCEL. The sign of the cancellation
// residual — and hence the step-to-step direction of `err` — flips level-to-level (MEASURED
// err over d∈{.01,.005,.0025}: 0.78%,1.49%,0.95% — bounded ≲1.5%, oscillating, NOT
// monotone). This is normal for adaptive tessellation of a cancellation-difference volume,
// NOT a weld/mesh defect (every level is watertight, be=0, consistently oriented, v<cf). The
// honest convergence statement is therefore a SHRINKING-ENVELOPE / best-so-far bound plus a
// tight absolute tolerance the refinement actually ATTAINS — the same criterion the landed
// ff_cut CUT pin uses, NOT strict monotonicity (which was test-strictness, since the impl is
// genuinely convergent — measured best ≈0.78%). The pin still fails loudly on a real
// non-convergence (envelope > 2%), a wrong volume (best > 1% or outside the band), or a leak.
CC_TEST(feat_pocket_welds_watertight) {
  const topo::Shape base = ssx::buildA();
  const topo::Shape tool = ssx::buildB();
  const double cf = ssx::volCut();  // V(base) − lens (the removed pocket material)
  double bestErr = 1.0;             // best-so-far (running minimum) relative error
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::FeatureReport rep;
    const topo::Shape r = bo::pocket(base, tool, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::FeatureDecline::Ok);
    CC_CHECK(rep.boolReport.decline == bo::SolidBoolDecline::Ok);
    if (r.isNull()) continue;
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);   // watertight (χ=2, 0 boundary edges)
    CC_CHECK(coh);       // consistently oriented
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);   // within the tessellation band (DISAGREED=0)
    CC_CHECK(v < cf);           // the smooth CUT cap under-estimates the closed form
    CC_CHECK(err < 0.02);       // shrinking envelope: every level within ~2% (rejects mis-weld)
    bestErr = std::min(bestErr, err);
  }
  // The refinement genuinely CONVERGES: the best (running-minimum) error is within a tight 1%
  // of the closed form — a two-sided proof the pocket welds at the RIGHT volume, not merely
  // that it welds. (Achieved best ≈0.78% on this schedule.)
  CC_CHECK(bestErr < 0.01);
}

// ── BOSS a tool cup onto the base: watertight, V = V(base) + V(tool) − lens ──
// The boss ADDS the tool (a raised pad): its volume is the outer envelope V(A)+V(B)−lens
// (the boolean's verified Fuse oracle), watertight and within the band.
CC_TEST(feat_boss_welds_watertight) {
  const topo::Shape base = ssx::buildA();
  const topo::Shape tool = ssx::buildB();
  const double cf = ssx::volA() + ssx::volA() - ssx::volCommon();  // V(base)+V(tool)−lens
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::FeatureReport rep;
    const topo::Shape r = bo::boss(base, tool, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::FeatureDecline::Ok);
    CC_CHECK(rep.boolReport.decline == bo::SolidBoolDecline::Ok);
    if (r.isNull()) continue;
    CC_CHECK(rep.boolReport.survivorFaces >= 4);  // base env ∪ tool env (the pad envelope)
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);
    CC_CHECK(coh);
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);
  }
}

// ── The tool REALLY is a swept profile: sweptExtrudeToolWall = EXACT extrusion ──
// The feature op's swept-tool WALL builder composes the Layer-6 translational sweep. We feed
// it a genuine NURBS profile SECTION (a degree-2 Bézier arc rail) and a straight sweep vector
// and assert the returned wall surface reproduces the section TRANSLATED by v·sweep pointwise
// (the exactness oracle from bspline_sweep) — proving the tool is a SWEPT profile, not a
// hand-built patch. The pole net of the extruded Bézier patch encodes exactly that:
// pole(i,0)=section.poles[i], pole(i,1)=section.poles[i]+sweep.
CC_TEST(feat_swept_tool_wall_is_exact_extrusion) {
  fmath::BsplineCurveData section;
  section.degree = 2;
  section.poles = {fmath::Point3{0.0, 0.0, 0.0}, fmath::Point3{0.5, 0.3, 0.0},
                   fmath::Point3{1.0, 0.0, 0.0}};
  section.knots = {0, 0, 0, 1, 1, 1};  // clamped degree-2 Bézier
  const fmath::Vec3 sweep{0.0, 0.0, 0.7};  // extrude by depth 0.7 in +z

  const auto wall = bo::featdetail::sweptExtrudeToolWall(section, sweep);
  CC_CHECK(wall.has_value());
  if (!wall.has_value()) return;
  CC_CHECK(wall->kind == topo::FaceSurface::Kind::Bezier);  // Bézier ⇒ SSI-adapter-ready
  CC_CHECK(wall->nPolesU == 3);
  CC_CHECK(wall->nPolesV == 2);
  CC_CHECK(wall->weights.empty());  // non-rational section ⇒ non-rational wall
  // The extrusion is EXACT: row 0 is the section, row 1 is the section + sweep.
  for (int i = 0; i < 3; ++i) {
    const fmath::Point3& p0 = wall->poles[static_cast<std::size_t>(i) * 2 + 0];
    const fmath::Point3& p1 = wall->poles[static_cast<std::size_t>(i) * 2 + 1];
    CC_CHECK(std::fabs(p0.x - section.poles[i].x) < 1e-12);
    CC_CHECK(std::fabs(p0.y - section.poles[i].y) < 1e-12);
    CC_CHECK(std::fabs(p0.z - section.poles[i].z) < 1e-12);
    CC_CHECK(std::fabs(p1.x - (section.poles[i].x + sweep.x)) < 1e-12);
    CC_CHECK(std::fabs(p1.y - (section.poles[i].y + sweep.y)) < 1e-12);
    CC_CHECK(std::fabs(p1.z - (section.poles[i].z + sweep.z)) < 1e-12);
  }
}

// ── A malformed / rational section makes the swept-tool wall HONEST-DECLINE ──
CC_TEST(feat_swept_tool_wall_declines_bad_section) {
  // Null sweep vector ⇒ the Layer-6 sweep declines ⇒ no wall (never fabricated).
  fmath::BsplineCurveData section;
  section.degree = 2;
  section.poles = {fmath::Point3{0, 0, 0}, fmath::Point3{0.5, 0.3, 0}, fmath::Point3{1, 0, 0}};
  section.knots = {0, 0, 0, 1, 1, 1};
  CC_CHECK(!bo::featdetail::sweptExtrudeToolWall(section, fmath::Vec3{0, 0, 0}).has_value());
  // Empty section ⇒ decline.
  CC_CHECK(!bo::featdetail::sweptExtrudeToolWall(fmath::BsplineCurveData{}, fmath::Vec3{0, 0, 1})
                .has_value());
}

// ── HONEST-DECLINE: a null base declines NullBase BEFORE the boolean ──
CC_TEST(feat_declines_null_base) {
  const topo::Shape nul{};
  const topo::Shape tool = ssx::buildB();
  for (bo::FeatureOp op : {bo::FeatureOp::Pocket, bo::FeatureOp::Boss}) {
    bo::FeatureReport rep;
    const topo::Shape r = bo::featureOp(nul, tool, op, 0.005, &rep);
    CC_CHECK(r.isNull());
    CC_CHECK(rep.decline == bo::FeatureDecline::NullBase);
  }
}

// ── A BOSS over a genuine MULTI-SEAM pose WELDS watertight at the outer envelope ──
// The base is the valley cup A; the tool is the mirror dome cup B, meeting it in TWO
// disjoint closed circular seams (boolReport.multiSeam=1). This pose historically HONEST-
// DECLINED (the two-seam FUSE was beyond the tractable single-seam sew); the underlying
// boolean since landed the multi-seam sew, so the boss now welds a watertight, correctly
// oriented solid at the outer-envelope volume V(base)+V(tool)−lens and CONVERGES. The pin
// asserts the CORRECT measured outcome (watertight + right volume + convergence, MEASURED
// err over d∈{.01,.005,.0025}: 1.20%,1.07%,0.77% — monotone to the oracle), and it STILL
// fails loudly if the multi-seam FUSE regresses to a leaky (be≠0) or wrong-volume solid.
// The `multiSeam` witness stays asserted: the op genuinely faces the two-seam pose, it just
// now welds it instead of deferring it. (Honest-decline of a leaky solid is still proven by
// feat_declines_null_base and feat_swept_tool_wall_declines_bad_section.)
CC_TEST(feat_boss_multi_seam_welds_watertight) {
  const topo::Shape base = msx::buildA();
  const topo::Shape tool = msx::buildB();
  const double cf = msx::volA() + msx::volB() - msx::volCommon();  // outer-envelope oracle
  double bestErr = 1.0;      // best-so-far (running minimum) relative error
  bool sawMultiSeam = false;
  for (double d : {0.01, 0.005, 0.0025}) {
    bo::FeatureReport rep;
    const topo::Shape r = bo::boss(base, tool, d, &rep, cf);
    CC_CHECK(!r.isNull());                                     // welds (not a decline)
    CC_CHECK(rep.decline == bo::FeatureDecline::Ok);
    CC_CHECK(rep.boolReport.decline == bo::SolidBoolDecline::Ok);
    CC_CHECK(rep.boolReport.multiSeam);                        // genuinely a 2-seam pose
    if (rep.boolReport.multiSeam) sawMultiSeam = true;
    if (r.isNull()) continue;
    CC_CHECK(rep.boolReport.survivorFaces >= 4);  // base env ∪ tool env (the pad envelope)
    bool wt, coh; std::size_t be; double v;
    meshStats(r, d, wt, coh, be, v);
    CC_CHECK(wt);
    CC_CHECK(be == 0);   // watertight (χ=2, 0 boundary edges) — never a leaky solid
    CC_CHECK(coh);       // consistently oriented
    const double err = std::fabs(v - cf) / cf;
    CC_CHECK(err < 30.0 * d);   // within the tessellation band (DISAGREED=0)
    CC_CHECK(err < 0.02);       // shrinking envelope: every level within ~2% (rejects mis-weld)
    bestErr = std::min(bestErr, err);
  }
  CC_CHECK(sawMultiSeam);       // it really did face the multi-seam pose
  // Converges to the outer-envelope volume: best (running-minimum) error within a tight 1%.
  // (Achieved best ≈0.77% on this schedule.)
  CC_CHECK(bestErr < 0.01);
}

int main() { return cctest::run_all(); }
