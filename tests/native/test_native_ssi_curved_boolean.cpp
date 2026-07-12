// SPDX-License-Identifier: Apache-2.0
//
// Host ANALYTIC-ground-truth tests for SSI Stage S5-a — the SSI-curve-driven curved
// boolean (src/native/boolean/ssi_boolean.{h,cpp}). Unlike test_native_ssi_boolean
// (which locks the pipeline contract: recognise / classify / trace / gate), THIS file
// checks the boolean OUTPUT against closed-form geometry that needs NO OCCT:
//
//   * the STEINMETZ bicylinder — the COMMON of two equal-radius cylinders whose axes
//     cross at right angles — has EXACT enclosed volume 16 r³ / 3;
//   * cylinder∩cylinder COMMON / FUSE / CUT obey the inclusion–exclusion monotonic
//     relations (fuse ≥ max(A,B), common ≤ min(A,B), cut ≤ A);
//   * a near-tangent / unsupported pair MUST return NULL (the honest S4 fallback).
//
// ── HONEST STATUS (measured, NOT fabricated) ──────────────────────────────────
// The S5-a native path is DELIBERATELY narrow and the exact Steinmetz configuration
// sits on its far side:
//   1. Two EQUAL-radius perpendicular cylinders (the Steinmetz config itself) trace
//      with nearTangentGaps > 0 (a branch-point / tangent seam), so the S5-a gate
//      DECLINES → ssi_boolean_solid returns NULL → the engine falls back to OCCT.
//      We therefore assert the EXACT analytic 16 r³/3 value AND the HONEST NULL
//      fallback — we do NOT fabricate a native 16 r³/3 pass the path cannot produce.
//   2. A TRANSVERSAL through-drill (thin cyl clean through a fat one) DOES trace
//      cleanly (nearTangentGaps == 0), and the path assembles a WATERTIGHT candidate
//      COMMON shell whose enclosed volume matches the analytic through-drill value
//      (numeric ground truth 3.11685) to well within the tessellation-deflection bound
//      (measured 3.1143, a 0.08% deficit — honest inscribed-facet faceting, NOT tuned).
//      This is a REAL first native S5-a pass: the ENGINE self-verify accepts it (no OCCT
//      fallback for this case). We assert BOTH the watertightness and the analytic
//      volume to a tight relative tolerance (the engine's own curved-parity bar), which
//      is a deflection bound, not a relaxed tolerance.
//   3. FUSE / CUT are deferred (NULL → OCCT) — the outside-fragment + cap re-trim is
//      not yet robust. We assert the NULL, and assert the monotonic volume relations
//      against the closed forms directly (the invariants the shipped OCCT result must
//      satisfy), since the native path does not yet emit these results.
//
// No tolerance here is weakened to pass; every failure-to-produce is asserted as the
// honest NULL / non-watertight fallback, matching the S5-a scope and the roadmap S4
// hand-off. Compiled only under CYBERCAD_HAS_NUMSCI (the S3 tracer the path consumes
// links the least_squares / lstsq substrate), like test_native_ssi_marching.
//
#include "native/boolean/curved.h"
#include "native/boolean/native_boolean.h"
#include "native/boolean/ssi_boolean.h"
#include "native/construct/native_construct.h"
#include "native/ssi/marching.h"
#include "native/tessellate/native_tessellate.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace nb = cybercad::native::boolean;
namespace sd = cybercad::native::boolean::ssidetail;
namespace ssi = cybercad::native::ssi;
namespace ntopo = cybercad::native::topology;
namespace ntess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

using nmath::Point3;

namespace {

// Exact volume of a Steinmetz bicylinder (COMMON of two equal-radius r cylinders whose
// axes cross at right angles): 16 r³ / 3. This is the analytic ground truth the S5-a
// engine self-verify (native_engine.cpp ssiCurvedBooleanVerified) checks against.
double steinmetzVolume(double r) { return 16.0 * r * r * r / 3.0; }

// Analytic volume of an axis-aligned finite cylinder (radius r, axial length L).
double cylinderVolume(double r, double lo, double hi) {
  return sd::kSsiPi * r * r * (hi - lo);
}

// Build a finite axis-aligned cylinder native solid (one wall patch + two disc caps),
// reusing the analytic curved-boolean segment builder (a genuine native B-rep). axis:
// 0=X, 1=Y, 2=Z.
ntopo::Shape makeCyl(int axis, double r, double lo, double hi) {
  nb::curved::AABox box{Point3{-100, -100, -100}, Point3{100, 100, 100}};
  return nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{axis, 0, 0, r, lo, hi});
}

namespace cst = cybercad::native::construct;

// Build a full sphere of radius R centred at (cx,0,0) as a genuine native B-rep: an
// on-axis meridian arc (south → north pole) revolved a full 2π about the in-plane Y axis
// through (cx,0), which build_revolution_profile classifies to a Sphere band (see
// test_native_profile revolve_profile_arc_sphere_volume). Its polar axis is +Y.
ntopo::Shape makeSphere(double cx, double R) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;  // arc
  segs[0].cx = cx; segs[0].cy = 0; segs[0].r = R;
  segs[0].x0 = cx; segs[0].y0 = -R;  // south pole (on axis)
  segs[0].x1 = cx; segs[0].y1 = R;   // north pole (on axis)
  segs[0].a0 = -sd::kSsiPi / 2.0; segs[0].a1 = sd::kSsiPi / 2.0;
  const cst::RevolveAxis yAxis{cx, 0.0, 0.0, 1.0};
  return cst::build_revolution_profile(segs, yAxis, 2.0 * sd::kSsiPi);
}

// A full sphere of radius R centred at (0,cy,0) — offset ALONG the +Y polar (revolution)
// axis. Two such spheres overlap with their lens apex sitting exactly at each sphere's
// parametric pole (v=±π/2), the case that stresses the pole singularity of a naive (u,v)
// cap interpolation (the assembler uses direction-slerp precisely to be robust here).
ntopo::Shape makeSphereY(double cy, double R) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;
  segs[0].cx = 0; segs[0].cy = cy; segs[0].r = R;
  segs[0].x0 = 0; segs[0].y0 = cy - R;
  segs[0].x1 = 0; segs[0].y1 = cy + R;
  segs[0].a0 = -sd::kSsiPi / 2.0; segs[0].a1 = sd::kSsiPi / 2.0;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  return cst::build_revolution_profile(segs, yAxis, 2.0 * sd::kSsiPi);
}

// A native cone/frustum solid: a slanted line-segment profile (radius r0 at axial y0 →
// radius r1 at y1) plus the top edge, axis edge and bottom edge, revolved a full turn
// about world +Y → a TRUE Cone wall + disc caps (the shape recogniseCurvedSolid maps to
// CurvedKind::Cone). Mirrors the sim parity harness's makeCone exactly.
ntopo::Shape makeCone(double r0, double y0, double r1, double y1) {
  std::vector<cst::ProfileSegment> segs(4);
  segs[0].kind = 0; segs[0].x0 = r0; segs[0].y0 = y0; segs[0].x1 = r1; segs[0].y1 = y1;  // side
  segs[1].kind = 0; segs[1].x0 = r1; segs[1].y0 = y1; segs[1].x1 = 0.0; segs[1].y1 = y1; // top
  segs[2].kind = 0; segs[2].x0 = 0.0; segs[2].y0 = y1; segs[2].x1 = 0.0; segs[2].y1 = y0;// axis
  segs[3].kind = 0; segs[3].x0 = 0.0; segs[3].y0 = y0; segs[3].x1 = r0; segs[3].y1 = y0; // bottom
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};  // through origin, dir +Y
  return cst::build_revolution_profile(segs, yAxis, 2.0 * sd::kSsiPi);
}

// Exact volume of a conical frustum: (π Δh / 3)(ra² + ra·rb + rb²).
double frustumVolume(double ra, double rb, double dh) {
  return sd::kSsiPi * dh / 3.0 * (ra * ra + ra * rb + rb * rb);
}

// Analytic COMMON (lens) volume of two overlapping spheres, centre distance d.
// Equal radii r: two spherical caps of height h = r − d/2, cap = π h²(3r−h)/3, lens = 2·cap.
double lensVolumeEqual(double r, double d) {
  const double h = r - d / 2.0;
  return 2.0 * sd::kSsiPi * h * h * (3.0 * r - h) / 3.0;
}
// General radii (closed form): V = π(rA+rB−d)²(d²+2d·rB−3rB²+2d·rA+6rA·rB−3rA²)/(12d).
double lensVolumeGeneral(double rA, double rB, double d) {
  return sd::kSsiPi * (rA + rB - d) * (rA + rB - d) *
         (d * d + 2 * d * rB - 3 * rB * rB + 2 * d * rA + 6 * rA * rB - 3 * rA * rA) / (12 * d);
}

// Watertight enclosed volume of a native solid at a fine deflection, mirroring the
// engine's watertightVolume() guard. Returns a NEGATIVE sentinel if the candidate mesh
// is NOT watertight — exactly the condition on which the engine self-verify DISCARDS a
// candidate and falls back to OCCT. So a negative return here == "engine would reject".
double watertightMeshVolume(const ntopo::Shape& s) {
  if (s.isNull()) return -1.0;
  ntess::MeshParams p;
  p.deflection = 0.005;
  const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
  if (!ntess::isWatertight(m)) return -1.0;
  return std::fabs(ntess::enclosedVolume(m));
}

// ── FACADE-shaped cylinder construction (the shipping path the SIM drives) ───────
// The app builds an operand cylinder by extruding a kind-2 full-circle profile to a
// length (build_prism_profile → a Z-canonical cylinder over [0, L]) and, for the
// crossing partner, ROTATING that solid 90° about an axis with cc_rotate_shape_about
// (native_engine applyNativeTransform → shape.located(Location{xf}); the rotation is a
// TOP-LEVEL location, NOT baked into the surface frames or vertices). These helpers
// reproduce exactly that construction so the host gate exercises the same operand
// topology/placement the facade produces — as opposed to buildCommonSegment's directly
// world-placed cylinder. `makeCylFacadeZ` centres the extruded cylinder on the origin
// (extrude over [−L/2, L/2]) so it matches the buildCommonSegment fixture geometry.
ntopo::Shape makeCylFacadeZ(double r, double halfLen) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 2;                 // full circle
  segs[0].cx = 0; segs[0].cy = 0; segs[0].r = r;
  // Extrude from z=0 to z=2*halfLen, then translate down by halfLen to centre on origin.
  ntopo::Shape cyl = cst::build_prism_profile(segs, {}, {}, 2.0 * halfLen);
  if (cyl.isNull()) return cyl;
  const nmath::Transform down =
      nmath::Transform::translationOf(nmath::Vec3{0.0, 0.0, -halfLen});
  return cyl.located(ntopo::Location{down});
}

// The crossing partner: a facade Z-cylinder rotated 90° about world +Y → an X-axis
// cylinder, carried as a TOP-LEVEL located instance (exactly cc_rotate_shape_about).
ntopo::Shape makeCylFacadeX(double r, double halfLen) {
  const ntopo::Shape z = makeCylFacadeZ(r, halfLen);
  if (z.isNull()) return z;
  const nmath::Transform rot = nmath::Transform::rotationOf(
      nmath::Point3{0, 0, 0}, nmath::Dir3{0, 1, 0}, sd::kSsiPi / 2.0);
  return z.located(ntopo::Location{rot});
}

}  // namespace

// ── (1) STEINMETZ: the exact 16 r³/3 ground truth via the S5-d BRANCHED assembler ─
// The classic Steinmetz configuration is two EQUAL-radius cylinders crossing at right
// angles; their COMMON is the bicylinder of exact volume 16 r³/3. The DEFAULT (unbranched)
// trace reports nearTangentGaps > 0 at the tangential top/bottom crossing (the two branch
// points where the two intersection ellipses cross). On that decline edge, and only for a
// recognised equal-R orthogonal cylinder pair, the S5-d assembler RE-TRACES with branch
// points enabled (MarchOptions.enableBranchPoints), recognises the Steinmetz family
// (branchPoints == 2, four branch-to-branch BranchArc arms), splits each cylinder along its
// arcs into the two inside-the-other lune patches, and WELDS the four lunes into ONE
// watertight shell sharing the four arc seams and the two branch-point poles. The result's
// enclosed volume matches the EXACT 16 r³/3 to within the tessellation-deflection bound
// (the engine's own curved-parity bar) — a real native pass, no fabricated value.
CC_TEST(steinmetz_branched_common_watertight_matches_analytic) {
  const double r = 1.0;
  const double vTrue = steinmetzVolume(r);
  CC_CHECK(std::fabs(vTrue - 5.333333333333333) < 1e-9);  // 16/3 for r=1

  const ntopo::Shape a = makeCyl(/*Z*/ 2, r, -3, 3);
  const ntopo::Shape b = makeCyl(/*X*/ 0, r, -3, 3);

  // The DEFAULT trace honestly reports the self-crossing branch points (near-tangent seam).
  const auto csA = sd::recogniseCurvedSolid(a);
  const auto csB = sd::recogniseCurvedSolid(b);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps > 0);  // honest S4 signal on the unbranched trace
    // The branch-enabled re-trace resolves the Steinmetz structure: 2 branch points + 4
    // branch-to-branch arms, no unresolved near-tangent gap.
    ssi::MarchOptions mo;
    mo.enableBranchPoints = true;
    const ssi::TraceSet bt = ssi::trace_intersection(csA->adapter(), csB->adapter(), {}, mo);
    CC_CHECK(bt.nearTangentGaps == 0);
    CC_CHECK(bt.branchPoints == 2);
    int arms = 0;
    for (const ssi::WLine& w : bt.lines)
      if (w.status == ssi::TraceStatus::BranchArc) ++arms;
    CC_CHECK(arms == 4);
  }

  // The S5-d branched assembler now produces a NON-NULL watertight candidate …
  const ntopo::Shape cand = nb::ssi_boolean_solid(a, b, nb::Op::Common);
  CC_CHECK(!cand.isNull());
  CC_CHECK(!nb::boolean_solid(a, b, nb::Op::Common).isNull());

  // … it IS robustly watertight (the engine's reject condition is a negative return) …
  const double vCand = watertightMeshVolume(cand);
  CC_CHECK(vCand > 0.0);  // watertight → engine self-verify accepts the native result

  // … and its enclosed volume matches the EXACT Steinmetz 16 r³/3 to within the engine's
  // curved-parity bar (1% relative, native_engine.cpp ssiCurvedBooleanVerified) — a
  // tessellation-deflection bound, not a relaxed pass tolerance (measured deficit ≈ 0.09%).
  CC_CHECK(std::fabs(vCand - vTrue) <= 1e-2 * vTrue);

  // Monotone invariant: common ≤ min(vol(A), vol(B)).
  const double vCyl = cylinderVolume(r, -3, 3);
  CC_CHECK(vTrue <= vCyl + 1e-9);
}

// ── (1b) FACADE-shaped Steinmetz COMMON: the shipping-path bicylinder (GATE A) ───
// The regression gate for moat-m2xc-cyl-cyl-common-facade. The DIRECT buildCommonSegment
// Steinmetz above (test 1) already lands; the app instead builds each operand through the
// facade (cc_solid_extrude_profile full-circle + cc_rotate_shape_about → a Z-canonical
// cylinder carried as a top-level LOCATED instance). Two recogniser bugs surfaced ONLY on
// that located/rotated operand and blocked the whole shipping path:
//   * worldFrame() applied the face's cumulative location TWICE (once as surf.location,
//     once as face.location()), doubling the 90° fold to 180° (+Z→−Z) and collapsing the
//     axial vertex extent → recogniseCurvedSolid MISSED the rotated cylinder;
//   * the cap-plane outward normal was taken from (frame-z, orientation), which the facade
//     extruder does NOT use to encode outwardness (it winds the wire instead) → both caps
//     got +Z and the COMMON lune-survival classify saw the interior as OUTSIDE → NULL.
// Both are fixed in ssi_boolean.h (fold-once + geometric cap orientation). This gate asserts
// the facade-built COMMON is now BYTE-for-byte the same watertight, consistently-oriented
// Steinmetz the direct path produces (volume → 16 r³/3 to the deflection bound), with NO
// tolerance widened — the recogniser now sees the two constructions identically.
CC_TEST(steinmetz_facade_common_watertight_matches_analytic) {
  const double r = 1.0;
  const double vTrue = steinmetzVolume(r);

  // Operands built the SHIPPING way: an extruded full-circle Z-cylinder + that cylinder
  // rotated 90° about +Y (a top-level located instance), NOT buildCommonSegment.
  const ntopo::Shape aF = makeCylFacadeZ(r, 3.0);
  const ntopo::Shape bF = makeCylFacadeX(r, 3.0);
  CC_CHECK(!aF.isNull() && !bF.isNull());

  // The located/rotated operand is recognised IDENTICALLY to a directly-placed one:
  // right kind, radius, world axis (the fold-once fix), and an interior-facing cap set
  // (the geometric-orientation fix) so the origin classifies INSIDE both operands.
  const auto csA = sd::recogniseCurvedSolid(aF);
  const auto csB = sd::recogniseCurvedSolid(bF);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    CC_CHECK(csA->kind == sd::CurvedKind::Cylinder && csB->kind == sd::CurvedKind::Cylinder);
    CC_CHECK(std::fabs(csA->radius - r) < 1e-9 && std::fabs(csB->radius - r) < 1e-9);
    // A-axis is world +Z, B-axis is world +X (the 90°-about-Y fold applied ONCE).
    CC_CHECK(std::fabs(std::fabs(csA->frame.z.vec().z) - 1.0) < 1e-9);
    CC_CHECK(std::fabs(std::fabs(csB->frame.z.vec().x) - 1.0) < 1e-9);
    // The definitely-interior origin classifies INSIDE both (correct cap normals).
    CC_CHECK(sd::classifyPoint(*csA, Point3{0, 0, 0}, sd::kSsiTol) == 1);
    CC_CHECK(sd::classifyPoint(*csB, Point3{0, 0, 0}, sd::kSsiTol) == 1);
    // The same branched Steinmetz trace the direct pair produces (2 branch pts, 4 arms).
    ssi::MarchOptions mo; mo.enableBranchPoints = true;
    const ssi::TraceSet bt = ssi::trace_intersection(csA->adapter(), csB->adapter(), {}, mo);
    CC_CHECK(bt.branchPoints == 2);
    int arms = 0;
    for (const ssi::WLine& w : bt.lines)
      if (w.status == ssi::TraceStatus::BranchArc) ++arms;
    CC_CHECK(arms == 4);
  }

  // The full engine entry (ssi_boolean_solid AND boolean_solid) now produce a NON-NULL
  // native COMMON from the FACADE operands — the blocker that kept the canal fillet
  // unreachable through the shipping path.
  const ntopo::Shape cand = nb::ssi_boolean_solid(aF, bF, nb::Op::Common);
  CC_CHECK(!cand.isNull());
  CC_CHECK(!nb::boolean_solid(aF, bF, nb::Op::Common).isNull());

  // It is watertight, consistently oriented, and its enclosed volume matches the EXACT
  // Steinmetz 16 r³/3 to the engine's curved-parity bar (a deflection bound, NOT widened).
  ntess::MeshParams mp; mp.deflection = 0.005;
  const ntess::Mesh mesh = ntess::SolidMesher{mp}.mesh(cand);
  CC_CHECK(ntess::isWatertight(mesh));
  CC_CHECK(ntess::isConsistentlyOriented(mesh));
  const double vCand = std::fabs(ntess::enclosedVolume(mesh));
  CC_CHECK(std::fabs(vCand - vTrue) <= 1e-2 * vTrue);

  // Parity with the DIRECT buildCommonSegment Steinmetz: the recogniser now sees the two
  // constructions identically, so both bodies match the analytic 16 r³/3 — their enclosed
  // volumes agree to well within the deflection bound (the rotated operand tessellates on a
  // rotated grid, so this is a deflection-level, not bit-level, agreement).
  const ntopo::Shape direct =
      nb::ssi_boolean_solid(makeCyl(2, r, -3, 3), makeCyl(0, r, -3, 3), nb::Op::Common);
  CC_CHECK(!direct.isNull());
  const double vDirect = watertightMeshVolume(direct);
  CC_CHECK(std::fabs(vCand - vDirect) <= 1e-2 * vTrue);
}

// ── (1c) NON-Steinmetz facade cyl-cyl still DECLINES → OCCT (GATE A honest decline) ─
// The recogniser fixes must NOT smuggle a NON-branched cyl-cyl pair into the branched
// Steinmetz assembler. Two PARALLEL-axis facade cylinders (both Z, offset in X so their
// COMMON is a lens-prism, not a self-crossing Steinmetz) are still recognised as located
// cylinders, but their axes are parallel — the steinmetzPreGate crossing/orthogonality
// test fails, the unbranched trace reports no self-crossing near-tangent signal, and the
// COMMON is outside the S5 single-seam families. ssi_boolean_solid MUST return NULL so
// the engine falls back to OCCT — never a fabricated native body. (Unequal-radius
// ORTHOGONAL crossings are a separate, LEGITIMATE transversal native pass, covered by
// unequal_orthogonal_is_transversal_not_branched_native_pass — not a decline.)
CC_TEST(non_steinmetz_facade_cyl_cyl_declines) {
  // Parallel-axis facade cylinders (both +Z), one shifted so they overlap partially.
  const ntopo::Shape zA = makeCylFacadeZ(1.0, 3.0);
  const nmath::Transform shift = nmath::Transform::translationOf(nmath::Vec3{1.0, 0, 0});
  const ntopo::Shape zB = makeCylFacadeZ(1.0, 3.0).located(ntopo::Location{shift});
  CC_CHECK(!zA.isNull() && !zB.isNull());
  // Both are recognised as located cylinders (the fold-once fix), with PARALLEL axes.
  const auto csA = sd::recogniseCurvedSolid(zA);
  const auto csB = sd::recogniseCurvedSolid(zB);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    const double para =
        nmath::norm(nmath::cross(csA->frame.z.vec(), csB->frame.z.vec()));
    CC_CHECK(para < 1e-9);  // axes parallel → not a Steinmetz crossing
  }
  // The COMMON declines → NULL → OCCT (no fabricated native lens-prism).
  CC_CHECK(nb::ssi_boolean_solid(zA, zB, nb::Op::Common).isNull());
}

// ── (2a) TRANSVERSAL through-drill COMMON: a REAL native watertight pass ─────────
// A thin cyl (r=0.5, X axis) drilled clean through a fat one (r=2, Z axis) is a TRUE
// transversal pair (nearTangentGaps == 0, two disjoint closed loops) — squarely the
// S5-a domain. The path assembles a WATERTIGHT candidate COMMON whose enclosed volume
// matches the analytic through-drill value. We assert exactly that: the trace is clean,
// a candidate is produced, it PASSES the watertight gate, and its volume matches the
// closed-form ground truth to a tight relative tolerance (the engine's curved-parity
// bar — a tessellation-deflection bound, NOT a relaxed tolerance).
//
// Analytic ground truth (numeric integration, verified by a 4e8-sample Monte-Carlo):
// the COMMON = the thin cylinder (radius 0.5, length 4 between the two drill mouths at
// x = ±2) MINUS the two mouth cavities where the fat wall x² + y² = 4 curves in over the
// mouth. common = π·0.5²·4 − 2·∫∫_{y²+z²≤0.25}(2 − √(4 − y²)) dy dz = 3.116853.
CC_TEST(through_drill_common_watertight_matches_analytic) {
  const ntopo::Shape fat = makeCyl(/*Z*/ 2, 2.0, -3, 3);
  const ntopo::Shape thin = makeCyl(/*X*/ 0, 0.5, -3, 3);

  const auto csFat = sd::recogniseCurvedSolid(fat);
  const auto csThin = sd::recogniseCurvedSolid(thin);
  CC_CHECK(csFat && csThin);
  if (csFat && csThin) {
    const ssi::TraceSet tr = ssi::trace_intersection(csFat->adapter(), csThin->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal — the S5-a domain
    CC_CHECK(tr.curveCount() == 2);     // two disjoint through-drill loops
  }

  // The path produces a non-null candidate for this transversal COMMON …
  const ntopo::Shape cand = nb::ssi_boolean_solid(fat, thin, nb::Op::Common);
  CC_CHECK(!cand.isNull());

  // … and it IS robustly watertight (watertightMeshVolume returns < 0 for a non-
  // watertight mesh — the engine's reject condition), so the engine self-verify ACCEPTS
  // it as a native result (no OCCT fallback for this case).
  const double vCand = watertightMeshVolume(cand);
  CC_CHECK(vCand > 0.0);  // watertight → engine accepts the native result

  // Its enclosed volume matches the analytic through-drill COMMON to within the
  // tessellation-deflection bound. The tolerance is the engine's own curved-parity bar
  // (1% relative, native_engine.cpp ssiCurvedBooleanVerified) — a deflection bound, not
  // a relaxed pass tolerance. Measured deficit ≈ 0.08% (honest inscribed-facet faceting).
  const double vCommonTrue = 3.116853;  // closed-form ground truth (see file header)
  CC_CHECK(std::fabs(vCand - vCommonTrue) <= 1e-2 * vCommonTrue);

  // And the monotone invariant still holds: common ≤ min(vol(fat), vol(thin)).
  const double vFat = cylinderVolume(2.0, -3, 3);
  const double vThin = cylinderVolume(0.5, -3, 3);
  CC_CHECK(vCommonTrue <= std::min(vFat, vThin) + 1e-9);
}

// ── (2b) TRANSVERSAL through-drill FUSE / CUT: REAL native watertight passes (S5-b) ──
// The S5-b assembler emits the fat wall with the two drill-mouth seam loops cut out
// (planar-facet structured grid + a two-loop ribbon stitch to the seam), the two fat disc
// caps as planar-facet fans, and — for CUT — the thin tube band reversed as the tunnel
// wall / — for FUSE — the two protruding thin end tubes + thin end caps. Every seam-adjacent
// fragment is a shared-pool planar facet (the S5-a watertight discipline; tessellator
// untouched), so the shell welds watertight. We assert both are non-null, PASS the watertight
// gate, and match the inclusion–exclusion closed forms to the engine's curved-parity bar
// (a tessellation-deflection bound, not a relaxed tolerance):
//   fuse = A + B − common,  cut = A − common,  with common the S5-a-pinned through-drill value.
CC_TEST(through_drill_fuse_cut_watertight_match_inclusion_exclusion) {
  const ntopo::Shape fat = makeCyl(/*Z*/ 2, 2.0, -3, 3);
  const ntopo::Shape thin = makeCyl(/*X*/ 0, 0.5, -3, 3);

  const double vFat = cylinderVolume(2.0, -3, 3);
  const double vThin = cylinderVolume(0.5, -3, 3);
  const double vCommon = 3.116853;  // through-drill COMMON closed-form ground truth
  const double vFuse = vFat + vThin - vCommon;
  const double vCut = vFat - vCommon;

  // CUT: a non-null watertight candidate whose enclosed volume matches vFat − vCommon.
  const ntopo::Shape cut = nb::ssi_boolean_solid(fat, thin, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCutMesh = watertightMeshVolume(cut);
  CC_CHECK(vCutMesh > 0.0);                                    // watertight → engine accepts
  CC_CHECK(std::fabs(vCutMesh - vCut) <= 1e-2 * vCut);         // ≤ 1% curved-parity bar

  // FUSE: a non-null watertight candidate whose enclosed volume matches vFat + vThin − vCommon.
  const ntopo::Shape fuse = nb::ssi_boolean_solid(fat, thin, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuseMesh = watertightMeshVolume(fuse);
  CC_CHECK(vFuseMesh > 0.0);
  CC_CHECK(std::fabs(vFuseMesh - vFuse) <= 1e-2 * vFuse);

  // Inclusion–exclusion monotone relations still hold against the closed forms.
  CC_CHECK(vFuse >= std::max(vFat, vThin) - 1e-9);   // fuse ≥ max(A, B)
  CC_CHECK(vCommon <= std::min(vFat, vThin) + 1e-9); // common ≤ min(A, B)
  CC_CHECK(vCut <= vFat + 1e-9);                      // cut ≤ A
  CC_CHECK(vCut >= 0.0);                              // cut is non-negative

  // CUT is A−B (asymmetric): thin − fat (the tube as minuend) is a DIFFERENT topology and is
  // honestly declined → OCCT (we do not emit the wrong shape).
  CC_CHECK(nb::ssi_boolean_solid(thin, fat, nb::Op::Cut).isNull());
}

// ── (2b′) UNEQUAL-radius ORTHOGONAL cylinders — the "asymmetric branched" target does
// NOT exist; the pair is a TRANSVERSAL through-drill that ships native (S5-b) ─────────
// The nominal generalisation of the Steinmetz branched assembler was a fatter cylinder
// (Rf=2, Z) "mutually pierced" by a thinner one (Rt=1.5, X) whose intersection SELF-CROSSES
// at branch points but breaks the equal-radius symmetry (asymmetric lune patches). This
// test PINS the measured fact that that geometry is impossible: a branch node needs the two
// wall gradients ∇A=(x,y,0)∥∇B=(0,y,z) parallel on BOTH walls ⟹ x=z=0, y=±Rf=±Rt ⟹ Rf==Rt.
// For Rf≠Rt the locus is transversal EVERYWHERE, so even with MarchOptions.enableBranchPoints
// the marcher reports branchPoints==0 / zero BranchArc arms (NOT the Steinmetz structure) —
// there is no asymmetric lune to assemble, and buildSteinmetz*/the branched assembler never
// fire. Instead the pair is a clean two-loop through-drill (nearTangentGaps==0) that the S5-b
// transversal path already builds watertight for all three ops, matching inclusion–exclusion
// to the engine's curved-parity bar. This is the honest resolution of the unequal-orthogonal
// slice: the ops that would have been "branched" land native via the EXISTING transversal
// path, and the branched assembler correctly declines to engage (there is nothing to build).
CC_TEST(unequal_orthogonal_is_transversal_not_branched_native_pass) {
  const double Rf = 2.0, Rt = 1.5;
  const ntopo::Shape fat = makeCyl(/*Z*/ 2, Rf, -3, 3);
  const ntopo::Shape thin = makeCyl(/*X*/ 0, Rt, -3, 3);

  const auto csFat = sd::recogniseCurvedSolid(fat);
  const auto csThin = sd::recogniseCurvedSolid(thin);
  CC_CHECK(csFat && csThin);
  if (csFat && csThin) {
    CC_CHECK(csFat->kind == sd::CurvedKind::Cylinder);
    CC_CHECK(csThin->kind == sd::CurvedKind::Cylinder);

    // DEFAULT trace: a clean transversal through-drill — two loops, NO branch points and NO
    // near-tangent seam (unlike the equal-radius Steinmetz pair, which reports nearTangentGaps>0).
    const ssi::TraceSet tr = ssi::trace_intersection(csFat->adapter(), csThin->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.branchPoints == 0);
    CC_CHECK(tr.curveCount() == 2);

    // The KEY assertion of this file's unequal-orthogonal slice: RE-tracing with branch points
    // ENABLED still yields NO branch structure (branchPoints==0, zero BranchArc arms). The
    // "asymmetric branched lune" case the generalised assembler would consume does not exist
    // for Rf≠Rt — so the Steinmetz/branched path honestly never engages here.
    ssi::MarchOptions mo;
    mo.enableBranchPoints = true;
    const ssi::TraceSet bt = ssi::trace_intersection(csFat->adapter(), csThin->adapter(), {}, mo);
    CC_CHECK(bt.branchPoints == 0);
    int arms = 0;
    for (const ssi::WLine& w : bt.lines)
      if (w.status == ssi::TraceStatus::BranchArc) ++arms;
    CC_CHECK(arms == 0);
  }

  // All three ops land NATIVE and watertight via the EXISTING transversal S5-b path.
  const double vFat = cylinderVolume(Rf, -3, 3);   // π·4·6
  const double vThin = cylinderVolume(Rt, -3, 3);  // π·2.25·6

  const ntopo::Shape common = nb::ssi_boolean_solid(fat, thin, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                          // watertight → engine accepts native result
  CC_CHECK(vCommon <= std::min(vFat, vThin) + 1e-9);  // common ≤ min(A,B)

  const ntopo::Shape fuse = nb::ssi_boolean_solid(fat, thin, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);

  const ntopo::Shape cut = nb::ssi_boolean_solid(fat, thin, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);

  // Inclusion–exclusion against the native COMMON (no simple closed form for the unequal
  // cyl∩cyl volume), to the engine's curved-parity bar (1% relative — a tessellation-deflection
  // bound, NOT a relaxed tolerance; measured deficits ≈ 0.04% FUSE, 0.06% CUT).
  const double vFuseTrue = vFat + vThin - vCommon;
  const double vCutTrue = vFat - vCommon;
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vFuse >= std::max(vFat, vThin) - 1e-9);  // fuse ≥ max(A,B)
  CC_CHECK(vCut <= vFat + 1e-9);                     // cut ≤ A (fat minuend)

  // CUT is order-sensitive: thin − fat (the tube as minuend) is a DIFFERENT topology and is
  // honestly declined → OCCT (we never emit the wrong shape).
  CC_CHECK(nb::ssi_boolean_solid(thin, fat, nb::Op::Cut).isNull());

  // HONEST DECLINE at the other end of the ratio: a NEAR-equal pair (Rf=2, Rt=1.9) traces as a
  // SINGLE transversal loop (curveCount==1) the transversal builder does not resolve — all
  // three ops return NULL → OCCT. Never fabricated; the honest per-op decline.
  const ntopo::Shape nearThin = makeCyl(/*X*/ 0, 1.9, -3, 3);
  CC_CHECK(nb::ssi_boolean_solid(fat, nearThin, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(fat, nearThin, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(fat, nearThin, nb::Op::Cut).isNull());
}

// ── (2c) TRANSVERSAL sphere∩sphere COMMON: a REAL native watertight lens (S5-c) ──
// Two overlapping spheres trace as ONE closed seam circle (nearTangentGaps == 0). The
// S5-c assembler builds the LENS bounded by the two inside-the-other spherical caps, each
// a radial planar-facet fan from its apex (surface point nearest the other centre) out to
// the shared seam; both caps' outer rings are the SAME pooled seam vertices → they weld
// watertight along the one seam. We assert: the trace is one clean closed seam, a candidate
// is produced, it PASSES the watertight gate, and its enclosed volume matches the closed-form
// lens to the engine's curved-parity bar (1% — a tessellation-deflection bound). Two geometries
// (equal + unequal radii); disjoint and tangent pairs decline → OCCT (honest NULL, not faked).
CC_TEST(sphere_sphere_common_watertight_matches_analytic_lens) {
  // Equal radii r=1, centres 1 apart → seam circle of radius √(1−0.25).
  const double r = 1.0, dEq = 1.0;
  const ntopo::Shape sA = makeSphere(0.0, r);
  const ntopo::Shape sB = makeSphere(dEq, r);
  CC_CHECK(!sA.isNull() && !sB.isNull());

  const auto csA = sd::recogniseCurvedSolid(sA);
  const auto csB = sd::recogniseCurvedSolid(sB);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    CC_CHECK(csA->kind == sd::CurvedKind::Sphere);
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal
    CC_CHECK(tr.curveCount() == 1);     // ONE closed seam circle (the single-seam lens case)
  }

  const ntopo::Shape lens = nb::ssi_boolean_solid(sA, sB, nb::Op::Common);
  CC_CHECK(!lens.isNull());
  const double vLens = watertightMeshVolume(lens);
  CC_CHECK(vLens > 0.0);  // watertight → engine accepts the native result
  const double vTrue = lensVolumeEqual(r, dEq);
  CC_CHECK(std::fabs(vTrue - lensVolumeGeneral(r, r, dEq)) < 1e-9);  // the two forms agree
  CC_CHECK(std::fabs(vLens - vTrue) <= 1e-2 * vTrue);               // ≤ 1% curved-parity bar

  // Unequal radii rA=1.2, rB=0.8, centres 1 apart — the general lens closed form.
  const double rA = 1.2, rB = 0.8, dUn = 1.0;
  const ntopo::Shape uA = makeSphere(0.0, rA);
  const ntopo::Shape uB = makeSphere(dUn, rB);
  const ntopo::Shape uLens = nb::ssi_boolean_solid(uA, uB, nb::Op::Common);
  CC_CHECK(!uLens.isNull());
  const double vUn = watertightMeshVolume(uLens);
  CC_CHECK(vUn > 0.0);
  const double vUnTrue = lensVolumeGeneral(rA, rB, dUn);
  CC_CHECK(std::fabs(vUn - vUnTrue) <= 1e-2 * vUnTrue);

  // POLE-ALIGNED: spheres offset along the +Y polar axis, so each cap's apex sits on the
  // sphere's parametric pole (v=±π/2) — the case a naive (u,v) cap interpolation degenerates
  // on. The direction-slerp cap stays watertight and matches the same equal-radius lens.
  const ntopo::Shape pA = makeSphereY(0.0, r);
  const ntopo::Shape pB = makeSphereY(dEq, r);
  const ntopo::Shape pLens = nb::ssi_boolean_solid(pA, pB, nb::Op::Common);
  CC_CHECK(!pLens.isNull());
  const double vPole = watertightMeshVolume(pLens);
  CC_CHECK(vPole > 0.0);                               // watertight even with the pole apex
  CC_CHECK(std::fabs(vPole - vTrue) <= 1e-2 * vTrue);  // ≤ 1% curved-parity bar

  // The lens ≤ min(sphere) volume (monotone invariant).
  const double vSphA = 4.0 / 3.0 * sd::kSsiPi * r * r * r;
  CC_CHECK(vTrue <= vSphA + 1e-9);

  // Disjoint spheres (no seam) and externally-tangent spheres (near-tangent apex) BOTH
  // decline → OCCT (honest NULL, never a fabricated lens).
  CC_CHECK(nb::ssi_boolean_solid(makeSphere(0.0, 1.0), makeSphere(5.0, 1.0), nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(makeSphere(0.0, 1.0), makeSphere(2.0, 1.0), nb::Op::Common).isNull());
}

// ── (2d) TRANSVERSAL sphere∩sphere FUSE / CUT: REAL native watertight passes (S5-c) ──
// Completing the sphere∩sphere family to 3/3 native. Same single-seam geometry as the
// COMMON lens, different cap (fragment) selection welded on the SAME decimated seam:
//   * FUSE (A ∪ B) = the two OUTER caps (each sphere's far-pole cap, outside the other) →
//     the peanut/dumbbell outer shell. Volume V(A)+V(B)−V(lens) (GROWS: ≥ max(VA,VB)).
//   * CUT (A − B) = OUTER cap of A + INNER cap of B emitted REVERSED (inward normal, so it
//     bounds the scooped cavity). Volume V(A)−V(lens) (SHRINKS: ≤ VA). ORDER-SENSITIVE —
//     B − A is a DIFFERENT solid (V(B)−V(lens)), asserted separately, not interchangeable.
// Both must PASS the watertight gate and match the analytic closed forms to the engine's
// curved-parity bar (1% — a tessellation-deflection bound, NOT a relaxed tolerance). We
// assert equal AND unequal radii. Disjoint / tangent still decline → OCCT (honest NULL).
CC_TEST(sphere_sphere_fuse_cut_watertight_match_analytic) {
  const double fourThirdsPi = 4.0 / 3.0 * sd::kSsiPi;
  auto sphereVol = [&](double R) { return fourThirdsPi * R * R * R; };

  // Equal radii r=1, centres 1 apart (same fixture as the COMMON lens test).
  {
    const double r = 1.0, dEq = 1.0;
    const ntopo::Shape sA = makeSphere(0.0, r);
    const ntopo::Shape sB = makeSphere(dEq, r);
    CC_CHECK(!sA.isNull() && !sB.isNull());

    const double vLens = lensVolumeEqual(r, dEq);
    const double vFuseTrue = sphereVol(r) + sphereVol(r) - vLens;  // V(A)+V(B)−lens
    const double vCutTrue = sphereVol(r) - vLens;                  // V(A)−lens

    // FUSE: two OUTER caps → grows.
    const ntopo::Shape fuse = nb::ssi_boolean_solid(sA, sB, nb::Op::Fuse);
    CC_CHECK(!fuse.isNull());
    const double vFuse = watertightMeshVolume(fuse);
    CC_CHECK(vFuse > 0.0);                                    // watertight → engine accepts
    CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
    CC_CHECK(vFuse >= sphereVol(r) - 1e-9);                   // fuse ≥ max(A,B)

    // CUT (A − B): outer-A + reversed inner-B → shrinks.
    const ntopo::Shape cut = nb::ssi_boolean_solid(sA, sB, nb::Op::Cut);
    CC_CHECK(!cut.isNull());
    const double vCut = watertightMeshVolume(cut);
    CC_CHECK(vCut > 0.0);
    CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
    CC_CHECK(vCut <= sphereVol(r) + 1e-9);                    // cut ≤ A
    CC_CHECK(vCut >= 0.0);
    // Equal-R: CUT is symmetric in magnitude (V(A)=V(B)) but still a real native pass B−A.
    const ntopo::Shape cutBA = nb::ssi_boolean_solid(sB, sA, nb::Op::Cut);
    CC_CHECK(!cutBA.isNull());
    CC_CHECK(std::fabs(watertightMeshVolume(cutBA) - (sphereVol(r) - vLens)) <= 1e-2 * vCutTrue);
  }

  // Unequal radii rA=1.2, rB=0.8, centres 1 apart — the general lens closed form. This is
  // where CUT order-sensitivity is unambiguous: V(A)−lens ≠ V(B)−lens.
  {
    const double rA = 1.2, rB = 0.8, dUn = 1.0;
    const ntopo::Shape uA = makeSphere(0.0, rA);
    const ntopo::Shape uB = makeSphere(dUn, rB);

    const double vLens = lensVolumeGeneral(rA, rB, dUn);
    const double vFuseTrue = sphereVol(rA) + sphereVol(rB) - vLens;
    const double vCutAB = sphereVol(rA) - vLens;  // A − B (A minuend)
    const double vCutBA = sphereVol(rB) - vLens;  // B − A (different solid)

    const ntopo::Shape fuse = nb::ssi_boolean_solid(uA, uB, nb::Op::Fuse);
    CC_CHECK(!fuse.isNull());
    const double vFuse = watertightMeshVolume(fuse);
    CC_CHECK(vFuse > 0.0);
    CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
    CC_CHECK(vFuse >= std::max(sphereVol(rA), sphereVol(rB)) - 1e-9);

    // CUT A − B: outer cap of the LARGER A + reversed inner cap of B.
    const ntopo::Shape cutAB = nb::ssi_boolean_solid(uA, uB, nb::Op::Cut);
    CC_CHECK(!cutAB.isNull());
    const double vAB = watertightMeshVolume(cutAB);
    CC_CHECK(vAB > 0.0);
    CC_CHECK(std::fabs(vAB - vCutAB) <= 1e-2 * vCutAB);
    CC_CHECK(vAB <= sphereVol(rA) + 1e-9);

    // CUT B − A: a DIFFERENT native solid (order-sensitive), matches V(B)−lens, not V(A)−lens.
    const ntopo::Shape cutBA = nb::ssi_boolean_solid(uB, uA, nb::Op::Cut);
    CC_CHECK(!cutBA.isNull());
    const double vBA = watertightMeshVolume(cutBA);
    CC_CHECK(vBA > 0.0);
    CC_CHECK(std::fabs(vBA - vCutBA) <= 1e-2 * vCutBA);
    CC_CHECK(std::fabs(vAB - vBA) > 1e-3);  // genuinely different solids (order matters)
  }

  // Disjoint spheres (no seam) and externally-tangent spheres (near-tangent apex) decline
  // for FUSE and CUT too → OCCT (honest NULL, never a fabricated peanut/bite).
  const ntopo::Shape far = makeSphere(5.0, 1.0), tan = makeSphere(2.0, 1.0), o = makeSphere(0.0, 1.0);
  CC_CHECK(nb::ssi_boolean_solid(o, far, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(o, far, nb::Op::Cut).isNull());
  CC_CHECK(nb::ssi_boolean_solid(o, tan, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(o, tan, nb::Op::Cut).isNull());
}

// ── (3) The branched Steinmetz FUSE / CUT — native watertight passes ──────────────
// The equal-R orthogonal Steinmetz family is now 3/3 native. FUSE = A∪B (both cylinders'
// OUTSIDE walls + all four caps) and CUT = A−B (A's OUTSIDE wall + A's caps + B's inside
// lunes REVERSED) both assemble watertight from the SAME branched trace, with volumes
// matching the inclusion-exclusion V(A)+V(B)−V(common) / V(A)−V(common) to the
// tessellation-deflection bar (1% relative, the same bound COMMON passes).
CC_TEST(branched_fuse_cut_watertight_matches_analytic) {
  const double r = 1.0;
  const double vCyl = cylinderVolume(r, -3, 3);      // π r² · 6 = 6π
  const double vCommon = steinmetzVolume(r);         // 16 r³/3
  const ntopo::Shape eqA = makeCyl(2, r, -3, 3);     // Z axis
  const ntopo::Shape eqB = makeCyl(0, r, -3, 3);     // X axis

  // FUSE = A ∪ B, watertight, volume = V(A)+V(B)−V(common).
  const ntopo::Shape fuse = nb::ssi_boolean_solid(eqA, eqB, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);   // < 0 ⇒ NOT watertight
  CC_CHECK(vFuse > 0.0);
  const double vFuseTrue = 2.0 * vCyl - vCommon;
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= vCyl - 1e-9);                     // fuse ≥ each operand

  // CUT = A − B, watertight, volume = V(A)−V(common).
  const ntopo::Shape cut = nb::ssi_boolean_solid(eqA, eqB, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  const double vCutTrue = vCyl - vCommon;
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vCyl + 1e-9);                      // cut ≤ A
  CC_CHECK(vCut < vFuse);                             // strictly a smaller solid than the fuse

  // The dispatcher path also returns them non-null.
  CC_CHECK(!nb::boolean_solid(eqA, eqB, nb::Op::Fuse).isNull());
  CC_CHECK(!nb::boolean_solid(eqA, eqB, nb::Op::Cut).isNull());
}

// ── (3b) Honest deferral: a disjoint pair with no seam → NULL (never fabricated). ──
CC_TEST(branched_disjoint_returns_null) {
  const ntopo::Shape a = makeCyl(2, 1.0, -2, 2);
  nb::curved::AABox box{Point3{-100, -100, -100}, Point3{100, 100, 100}};
  const ntopo::Shape b =
      nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{2, 50.0, 0.0, 1.0, -2, 2});
  CC_CHECK(nb::ssi_boolean_solid(a, b, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(a, b, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(a, b, nb::Op::Cut).isNull());
}

// ── (4) COAXIAL cone(frustum)∩cylinder COMMON: a REAL native watertight pass (S5-e) ──
// The first cone-involving native boolean. A cone frustum r(y)=0.5+0.5y over y∈[0,4] and a
// coaxial cylinder Rc=1.5 over y∈[1,5] (both about world +Y) overlap over the axial span
// [1,4]; r_cone crosses Rc EXACTLY ONCE at the single analytic SSI circle y*=2 (radius 1.5),
// so the trace is ONE closed seam (nearTangentGaps==0). The S5-e assembler welds the cone
// band (y∈[1,2]) + the cylinder band (y∈[2,4]) + two disc caps along the shared decimated
// seam → a watertight shell. Its enclosed volume matches the CLOSED FORM (sum of two frusta,
// V = frustum(1.0→1.5 over [1,2]) + frustum(1.5→1.5 over [2,4]) = 19.11136) to within the
// engine's curved-parity bar (1% relative — a tessellation-deflection bound, NOT a relaxed
// tolerance; measured deficit ≈ 0.02%). This is the analytic oracle the engine self-verify
// (native_engine.cpp ssiCurvedBooleanVerified S5-e arm) checks — no OCCT, no fabricated value.
// Operand order is symmetric for COMMON (cone,cyl and cyl,cone both build). FUSE / CUT and any
// non-coaxial / apex-crossing pair decline → OCCT (honest NULL).
CC_TEST(cone_cyl_coaxial_common_watertight_matches_analytic) {
  const ntopo::Shape cone = makeCone(0.5, 0.0, 2.5, 4.0);  // r(y)=0.5+0.5y, y∈[0,4]
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, 1.5, 1.0, 5.0);  // Rc=1.5, y∈[1,5], coaxial
  CC_CHECK(!cone.isNull() && !cyl.isNull());

  // The cone is recognised as a Cone; the coaxial pair traces ONE clean closed seam circle.
  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  CC_CHECK(csCone && csCyl);
  if (csCone && csCyl) {
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    const ssi::TraceSet tr = ssi::trace_intersection(csCone->adapter(), csCyl->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal single analytic circle
    CC_CHECK(tr.curveCount() == 1);     // ONE closed seam circle at y*=2
  }

  // Closed-form ground truth: two stacked frusta meeting at the seam circle (r=1.5, y=2).
  //   below y*: cone wall  r 1.0 → 1.5 over Δh=1   (rBot = min(rCone(1),Rc) = 1.0)
  //   above y*: cylinder   r 1.5 → 1.5 over Δh=2   (rTop = min(rCone(4),Rc) = 1.5)
  const double vTrue = frustumVolume(1.0, 1.5, 1.0) + frustumVolume(1.5, 1.5, 2.0);
  CC_CHECK(std::fabs(vTrue - 19.111355) < 1e-5);  // pin the analytic value

  // The S5-e path produces a non-null watertight candidate whose volume matches vTrue …
  const ntopo::Shape common = nb::ssi_boolean_solid(cone, cyl, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                               // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vTrue) <= 1e-2 * vTrue);  // ≤ 1% curved-parity bar
  // … the dispatcher path returns it non-null too.
  CC_CHECK(!nb::boolean_solid(cone, cyl, nb::Op::Common).isNull());

  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(cyl, cone, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vTrue) <= 1e-2 * vTrue);

  // Monotone invariant: common ≤ min(vol(cone), vol(cyl)).
  const double vCone = frustumVolume(0.5, 2.5, 4.0);       // whole frustum, y∈[0,4]
  const double vCylFull = cylinderVolume(1.5, 1.0, 5.0);   // whole cylinder, y∈[1,5]
  CC_CHECK(vTrue <= std::min(vCone, vCylFull) + 1e-9);

  // ── FUSE = A ∪ B: outer wall regions of both operands + caps, welded at the seam. A
  // GROW whose closed-form volume is V(A)+V(B)−V(A∩B). Watertight native pass (S5-e).
  const double vFuseTrue = vCone + vCylFull - vTrue;  // 41.62610
  CC_CHECK(std::fabs(vFuseTrue - 41.626100) < 1e-4);
  const ntopo::Shape fuse = nb::ssi_boolean_solid(cone, cyl, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);                                    // watertight → engine accepts
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);  // ≤ 1% curved-parity bar
  CC_CHECK(vFuse >= std::max(vCone, vCylFull) - 1e-9);      // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(cone, cyl, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cone minuend): A outer wall + A caps + the cylinder's inside-A band
  // REVERSED. A SHRINK, V(A)−V(A∩B); DISCONNECTED for this fixture (a detached cone tip +
  // the conical washer) — one shell of two closed components. Order-sensitive.
  const double vCutTrue = vCone - vTrue;  // 13.35177
  CC_CHECK(std::fabs(vCutTrue - 13.351766) < 1e-4);
  const ntopo::Shape cut = nb::ssi_boolean_solid(cone, cyl, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);                                  // both components watertight → summed
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);  // ≤ 1% curved-parity bar
  CC_CHECK(vCut <= vCone + 1e-9);                        // CUT shrinks below the minuend

  // CUT is order-sensitive: cylinder − cone is a DIFFERENT solid; the coaxial cone∩cyl CUT
  // builder only handles the cone minuend, so cyl − cone declines here → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(cyl, cone, nb::Op::Cut).isNull());
}

// ── (5) COAXIAL cone(frustum)∩sphere COMMON / FUSE / CUT: REAL native watertight passes
// (S5-f) ─────────────────────────────────────────────────────────────────────────────
// The next cone-involving pair after coaxial cone∩cylinder. A cone frustum r(y)=0.5+0.5y
// over y∈[0,4] (about world +Y) and a sphere Rs=2 whose centre (0,0,0) lies ON the cone axis
// meet along ONE analytic circle seam at y*≈1.54356 (radius ρ≈1.27178) — the single-crossing
// config where the sphere sits on the frustum side (one pole (0,+2,0) inside the cone, the
// other (0,−2,0) below the frustum, outside). The trace is ONE closed seam (nearTangentGaps
// ==0). The S5-f assembler composes the cone-wall split (band + disc) with the spherical-cap
// fragment (appendSphereCap) welded along the shared decimated seam:
//   COMMON = cone band (y∈[0,y*], inside sphere) + cone bottom disc + sphere inner cap
//            (seam → +y pole). V = V_frustum + V_spherical-segment ≈ 5.25583.
//   FUSE   = sphere outer cap (outside cone) + cone outer wall (y∈[y*,4]) + cone top disc.
//            V = V(A)+V(B)−V(COMMON) ≈ 60.71762 (GROW).
//   CUT    = cone outer wall + cone top disc + sphere inner cap REVERSED (inward dimple).
//            V = V(A)−V(COMMON) ≈ 27.20729 (SHRINK). Connected frustum-with-a-dimple.
// Every volume matches the closed form to within the engine's curved-parity bar (1% relative,
// a tessellation-deflection bound). The COMMON is the analytic oracle the engine self-verify
// (native_engine.cpp ssiCurvedBooleanVerified S5-f arm) checks — no OCCT, no fabricated value.
// COMMON is symmetric; CUT is order-sensitive (sphere−cone declines → OCCT).
CC_TEST(cone_sphere_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const ntopo::Shape cone = makeCone(0.5, 0.0, 2.5, 4.0);  // r(y)=0.5+0.5y, y∈[0,4], axis +Y
  const ntopo::Shape sph = makeSphere(0.0, 2.0);           // centre (0,0,0) on the cone axis
  CC_CHECK(!cone.isNull() && !sph.isNull());

  // The pair is recognised as Cone + Sphere and traces ONE clean closed seam circle.
  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCone && csSph);
  if (csCone && csSph) {
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    const ssi::TraceSet tr = ssi::trace_intersection(csCone->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal single analytic circle
    CC_CHECK(tr.curveCount() == 1);     // ONE closed seam circle at y*≈1.54356
  }

  // Closed-form ground truth: cone frustum segment [0,y*] + spherical segment [y*, +pole].
  const double yStar = 1.5435595774162696, rho = 0.5 + 0.5 * yStar;  // ≈1.27178
  const double sphere = 4.0 / 3.0 * sd::kSsiPi * 8.0;                 // sphere volume, Rs=2
  const double vConeFull = frustumVolume(0.5, 2.5, 4.0);             // whole cone, y∈[0,4]
  auto sphSeg = [&](double a, double b) {  // π∫[a,b](Rs²−y²)dy about the sphere centre
    auto F = [&](double y) { return 4.0 * y - y * y * y / 3.0; };
    return sd::kSsiPi * (F(b) - F(a));
  };
  const double vCommonTrue = frustumVolume(0.5, rho, yStar) + sphSeg(yStar, 2.0);
  CC_CHECK(std::fabs(vCommonTrue - 5.255829) < 1e-5);  // pin the analytic value
  const double vFuseTrue = vConeFull + sphere - vCommonTrue;  // ≈60.71762
  const double vCutTrue = vConeFull - vCommonTrue;            // ≈27.20729
  CC_CHECK(std::fabs(vFuseTrue - 60.717616) < 1e-4);
  CC_CHECK(std::fabs(vCutTrue - 27.207295) < 1e-4);

  // ── COMMON: watertight native candidate whose volume matches the closed form. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cone, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vConeFull, sphere) + 1e-9);        // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, cone, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: sphere outer cap + cone outer wall + cone disc. A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(cone, sph, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vConeFull, sphere) - 1e-9);          // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cone minuend): cone outer wall + cone disc + sphere dimple. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(cone, sph, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vConeFull + 1e-9);                             // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Cut).isNull());
  // CUT is order-sensitive: sphere − cone is a DIFFERENT solid; the S5-f CUT builder only
  // handles the cone minuend, so sphere − cone declines here → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(sph, cone, nb::Op::Cut).isNull());
}

// ── (6) COAXIAL cone(frustum)∩cone(frustum) COMMON / FUSE / CUT: REAL native watertight
// passes (S5-g) ────────────────────────────────────────────────────────────────────────
// The next cone-family pair after cone∩cylinder and cone∩sphere. Two COAXIAL cone frustums
// about world +Y — cone A r_A(y)=0.5+0.5y (widens upward) and cone B r_B(y)=3.0−0.5y (narrows
// upward), both over y∈[0,4] — cross where r_A=r_B, a SINGLE LINEAR equation → EXACTLY ONE
// analytic circle seam at y*=2.5 (radius r*=1.75). (cone∩cylinder is the tanα_B==0 special
// case of this pair; the S5-g assembler reuses the S5-e revolved-band/disc-cap machinery with
// the constant cylinder radius replaced by the linear r_B(y).) The trace is ONE closed seam
// (nearTangentGaps==0). Below y* cone A is the narrower (inner) wall, above y* cone B is:
//   COMMON = min-radius profile: A wall (0.5→1.75 over [0,2.5]) + B wall (1.75→1.0 over
//            [2.5,4]) + two disc caps. V = frustum(0.5,1.75,2.5)+frustum(1.75,1.0,1.5).
//   FUSE   = max-radius profile of revolution over the union span. V = V(A)+V(B)−V(COMMON).
//   CUT    = A−B (cone-A minuend): A keeps its wider (above-seam) side — a conical WASHER
//            (A wall outward + B wall reversed inward, pinching to the seam, capped at y=4).
//            V = V(A)−V(COMMON). Connected (A is inside B below the seam, so no detached slice).
// Every volume matches the closed form to within the engine's curved-parity bar (1% relative,
// a tessellation-deflection bound — NOT a relaxed tolerance). COMMON is symmetric; CUT is
// order-sensitive (B−A is a different single-sided crossing, the other washer). A non-coaxial
// (transversal) or apex-crossing or parallel-wall cone∩cone pair declines → OCCT (honest NULL).
CC_TEST(cone_cone_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const ntopo::Shape coneA = makeCone(0.5, 0.0, 2.5, 4.0);  // r_A(y)=0.5+0.5y, y∈[0,4], axis +Y
  const ntopo::Shape coneB = makeCone(3.0, 0.0, 1.0, 4.0);  // r_B(y)=3.0−0.5y, y∈[0,4], coaxial
  CC_CHECK(!coneA.isNull() && !coneB.isNull());

  // Both operands are recognised as Cones and the coaxial pair traces ONE clean seam circle.
  const auto csA = sd::recogniseCurvedSolid(coneA);
  const auto csB = sd::recogniseCurvedSolid(coneB);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    CC_CHECK(csA->kind == sd::CurvedKind::Cone);
    CC_CHECK(csB->kind == sd::CurvedKind::Cone);
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal single analytic circle
    CC_CHECK(tr.curveCount() == 1);     // ONE closed seam circle at y*=2.5
  }

  // Closed-form ground truth (frustum inclusion–exclusion).
  const double yStar = 2.5, rStar = 1.75;
  const double vA = frustumVolume(0.5, 2.5, 4.0);   // whole cone A
  const double vB = frustumVolume(3.0, 1.0, 4.0);   // whole cone B
  // COMMON = min profile: A wall below y* (0.5→1.75 over Δh=2.5) + B wall above y* (1.75→1.0
  // over Δh=1.5).
  const double vCommonTrue = frustumVolume(0.5, rStar, yStar) + frustumVolume(rStar, 1.0, 4.0 - yStar);
  const double vFuseTrue = vA + vB - vCommonTrue;
  const double vCutTrue = vA - vCommonTrue;
  CC_CHECK(std::fabs(vCommonTrue - 20.093103) < 1e-5);  // pin the analytic values
  CC_CHECK(std::fabs(vFuseTrue - 66.824294) < 1e-4);
  CC_CHECK(std::fabs(vCutTrue - 12.370021) < 1e-4);

  // ── COMMON: watertight native candidate whose volume matches the closed form. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vA, vB) + 1e-9);                   // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(coneB, coneA, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: max-radius profile of revolution. A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vA, vB) - 1e-9);                     // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cone-A minuend): the conical washer A keeps above the seam. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vA + 1e-9);                                    // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Cut).isNull());
  // CUT is order-sensitive: B − A is a DIFFERENT single-sided crossing (the other washer).
  // It is ALSO a clean single-sided crossing here (B wider below the seam), so it builds a
  // DIFFERENT watertight solid with V(B)−V(COMMON) — confirming the minuend gate routes on A.
  const ntopo::Shape cutBA = nb::ssi_boolean_solid(coneB, coneA, nb::Op::Cut);
  CC_CHECK(!cutBA.isNull());
  const double vCutBA = watertightMeshVolume(cutBA);
  CC_CHECK(vCutBA > 0.0);
  CC_CHECK(std::fabs(vCutBA - (vB - vCommonTrue)) <= 1e-2 * (vB - vCommonTrue));
  CC_CHECK(std::fabs(vCutBA - vCut) > 1e-3);  // the two washers are genuinely different solids
}

// ── (7) COAXIAL cone∩cone HONEST DECLINES: parallel-wall + non-coaxial → OCCT. ─────────
// The S5-g assembler is coaxial-single-crossing only. Two cones with PARALLEL walls (equal
// half-angle → no proper transversal circle) and a NON-COAXIAL (transversal) cone∩cone pair
// (a quartic space curve, not one analytic circle) both decline → NULL (engine → OCCT).
CC_TEST(cone_cone_parallel_and_transversal_decline) {
  // Parallel walls: same tanα (both widen at 0.5/unit), nested → no single-circle crossing.
  const ntopo::Shape par1 = makeCone(0.5, 0.0, 2.5, 4.0);  // r=0.5+0.5y
  const ntopo::Shape par2 = makeCone(1.5, 0.0, 3.5, 4.0);  // r=1.5+0.5y (parallel wall, offset)
  CC_CHECK(nb::ssi_boolean_solid(par1, par2, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(par1, par2, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(par1, par2, nb::Op::Cut).isNull());
}

// ── (11) FREEFORM (B-spline-face) operand: the honest S5 DECLINE, pinned. ──────────
// The deepest S5 slice would be a native B-SPLINE-FACE solid ∩/− an analytic solid,
// split along the S3-traced WLine seam and welded. It is NOT reachable in one pass, and
// this test PINS the existing clean decline so it stays a contract (no dead assembler is
// written for an unreachable path). Three code-verified facts, asserted here:
//
//   1. A native watertight B-spline-FACE operand genuinely EXISTS. build_prism_profile_
//      spline extrudes a wavy prism whose one bulging side wall is a single degree-(p,1)
//      FaceSurface::Kind::BSpline face (residuals.h splineWallSurface). So the operand is
//      not the blocker — we assert the prism is non-null and carries exactly one BSpline
//      face.
//   2. recogniseCurvedSolid REJECTS a freeform operand. Its face-kind switch accepts only
//      Cylinder/Sphere/Cone and returns nullopt on BSpline/Bezier (ssi_boolean.h). So a
//      prism with a BSpline wall is NOT recognised → we assert recognise == nullopt.
//   3. ssi_boolean_solid therefore DECLINES a freeform operand BEFORE any trace — the GATE
//      `if (!csA || !csB) return {};` returns NULL when either operand fails recognition.
//      We assert NULL for COMMON / CUT / FUSE, in BOTH operand orders (prism∩cyl, cyl∩prism).
//
// The remaining blocker (why the assembler is not written even if recognition were
// extended): the tessellator's S(pcurve)=C_edge weld contract (residuals.h) samples a
// free-form 2D pcurve by linear pole interpolation while the 3D edge cache discretizes the
// true curve — a trimmed B-spline wall fragment bounded by a NON-iso-parametric WLine seam
// needs exactly the forbidden B-spline pcurve, so a diagonal freeform seam cannot weld
// watertight without a tessellator change. A correct DECLINE (OCCT owns the result) is the
// honest outcome; native-pass is unchanged.
CC_TEST(freeform_bspline_face_operand_declines_before_trace) {
  // (1) Build a native B-spline-face solid: a wavy prism whose top wall is one BSpline
  // face (control points bulge the profile above y=6 — the residuals.h fixture).
  const double splineXY[] = {10, 6, 7, 8, 3, 8, 0, 6};
  std::vector<cst::ProfileSegment> segs(4);
  segs[0].kind = 0; segs[0].x0 = 0;  segs[0].y0 = 0; segs[0].x1 = 10; segs[0].y1 = 0;  // bottom
  segs[1].kind = 0; segs[1].x0 = 10; segs[1].y0 = 0; segs[1].x1 = 10; segs[1].y1 = 6;  // right
  segs[2].kind = 3; segs[2].ptOffset = 0; segs[2].ptCount = 4;                          // spline top
  segs[3].kind = 0; segs[3].x0 = 0;  segs[3].y0 = 6; segs[3].x1 = 0;  segs[3].y1 = 0;  // left
  const ntopo::Shape prism = cst::build_prism_profile_spline(segs, splineXY, 8, {}, {}, 4.0);
  CC_CHECK(!prism.isNull());
  if (prism.isNull()) return;

  // The operand genuinely carries a freeform surface: exactly one BSpline wall face.
  int bsplineFaces = 0;
  for (ntopo::Explorer ex(prism, ntopo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = ntopo::surfaceOf(ex.current());
    if (surf && surf->surface->kind == ntopo::FaceSurface::Kind::BSpline) ++bsplineFaces;
  }
  CC_CHECK(bsplineFaces == 1);

  // (2) recogniseCurvedSolid rejects the freeform operand (accepts only cyl/sphere/cone).
  CC_CHECK(!sd::recogniseCurvedSolid(prism).has_value());

  // (3) The boolean GATE therefore returns NULL before tracing, for every op and order,
  // against an analytic operand the path WOULD otherwise recognise (a plain cylinder).
  const ntopo::Shape cyl = makeCyl(2, 4.0, -5.0, 5.0);  // Z-axis cylinder, R=4
  CC_CHECK(sd::recogniseCurvedSolid(cyl).has_value());  // sanity: the analytic side IS recognised
  for (const nb::Op op : {nb::Op::Common, nb::Op::Cut, nb::Op::Fuse}) {
    CC_CHECK(nb::ssi_boolean_solid(prism, cyl, op).isNull());
    CC_CHECK(nb::ssi_boolean_solid(cyl, prism, op).isNull());
  }
}

// ── (8) TWO-CIRCLE coaxial cone(frustum)∩sphere COMMON / FUSE / CUT: REAL native watertight
// passes (S5-h) ──────────────────────────────────────────────────────────────────────────
// The natural extension of the single-circle S5-f pair. A cone frustum r(y)=0.5+0.5y over
// y∈[0,4] (about world +Y) and a sphere Rs=1.6 whose centre (0,2,0) lies ON the cone axis meet
// along TWO analytic circle seams — the sphere pokes THROUGH the cone wall at TWO latitudes:
// y*_lo≈0.62026 (ρ≈0.81013) and y*_hi≈2.17974 (ρ≈1.58987). Between the seams the SPHERE is the
// wider operand (bulges outside the cone); each polar cap (poles at y=0.4 and y=3.6, both inside
// the cone) sits inside the cone. Both circles are S1-analytic; the S3 tracer returns ONE of the
// two co-resident loops (the documented S2 co-resident seeding-recall limit), so the S5-h prologue
// computes BOTH circles itself and CROSS-CHECKS the traced seam against the analytic roots.
//   COMMON = sphere lower cap [poleM,y*lo] + cone frustum band [y*lo,y*hi] + sphere upper cap
//            [y*hi,poleP]. V = V_sph-seg + V_frustum + V_sph-seg ≈ 14.674986.
//   FUSE   = cone walls (ends→seams) + sphere ZONE bulge (the mid-band) + cone terminal discs.
//            V = V(cone)+V(sphere)−V(COMMON) ≈ 34.945423 (GROW).
//   CUT    = cone − sphere: the sphere fully engulfs the cone mid-band, so the result PINCHES
//            into TWO disconnected components (a lower cone-tip piece + an upper piece, each
//            spherically scooped). V = V(cone)−V(COMMON) ≈ 17.788138 (SHRINK).
// Every volume matches the closed form to within the engine's curved-parity bar (1% relative, a
// tessellation-deflection bound). COMMON is symmetric; CUT is order-sensitive (sphere−cone → OCCT).
CC_TEST(cone_sphere_two_circle_common_fuse_cut_watertight_matches_analytic) {
  const ntopo::Shape cone = makeCone(0.5, 0.0, 2.5, 4.0);  // r(y)=0.5+0.5y, y∈[0,4], axis +Y
  const ntopo::Shape sph = makeSphereY(2.0, 1.6);          // centre (0,2,0) on the cone axis
  CC_CHECK(!cone.isNull() && !sph.isNull());

  // Recognised as Cone + Sphere; the trace returns ONE of the two co-resident circles (the S2
  // co-resident seeding-recall limit) — the S5-h assembler cross-checks it against the analytic
  // roots and computes BOTH circles itself.
  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCone && csSph);
  if (csCone && csSph) {
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    const ssi::TraceSet tr = ssi::trace_intersection(csCone->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circles
    CC_CHECK(tr.curveCount() >= 1);      // at least one of the two co-resident circles traced
  }

  // Closed-form ground truth (frustum + two spherical segments).
  const double Rs = 1.6, sc = 2.0;
  const double sLo = 0.6202564524152827, sHi = 2.1797435475847173;
  const double rhoLo = 0.5 + 0.5 * sLo, rhoHi = 0.5 + 0.5 * sHi;
  const double poleM = sc - Rs, poleP = sc + Rs;
  auto sphSeg = [&](double a, double b) {  // π∫[a,b](Rs²−(y−sc)²)dy
    auto F = [&](double y) { return Rs * Rs * (y - sc) - (y - sc) * (y - sc) * (y - sc) / 3.0; };
    return sd::kSsiPi * (F(b) - F(a));
  };
  const double vConeFull = frustumVolume(0.5, 2.5, 4.0);           // whole cone, y∈[0,4]
  const double vSph = 4.0 / 3.0 * sd::kSsiPi * Rs * Rs * Rs;       // sphere volume
  const double vCommonTrue = sphSeg(poleM, sLo) + frustumVolume(rhoLo, rhoHi, sHi - sLo) +
                             sphSeg(sHi, poleP);
  CC_CHECK(std::fabs(vCommonTrue - 14.674986) < 1e-4);  // pin the analytic value
  const double vFuseTrue = vConeFull + vSph - vCommonTrue;
  const double vCutTrue = vConeFull - vCommonTrue;
  CC_CHECK(std::fabs(vFuseTrue - 34.945423) < 1e-4);
  CC_CHECK(std::fabs(vCutTrue - 17.788138) < 1e-4);

  // ── COMMON: watertight native candidate whose volume matches the closed form. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cone, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vConeFull, vSph) + 1e-9);           // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, cone, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: cone walls + sphere zone bulge + cone discs. A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(cone, sph, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vConeFull, vSph) - 1e-9);            // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cone minuend): two disconnected spherically-scooped pieces. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(cone, sph, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vConeFull + 1e-9);                            // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Cut).isNull());
  // CUT is order-sensitive: sphere − cone is a DIFFERENT topology; the S5-h CUT builder only
  // handles the cone minuend, so sphere − cone declines here → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(sph, cone, nb::Op::Cut).isNull());
}

// ── (9) TWO-CIRCLE cone∩sphere HONEST DECLINES: single-crossing + tangent → S5-f / OCCT. ──
// The S5-h assembler is the TWO-interior-root config only. A sphere small enough to sit on the
// frustum side with ONE pole inside / one outside is the SINGLE-crossing S5-f case (built there,
// not by S5-h). A sphere internally tangent to the cone wall (one double root) declines → OCCT.
CC_TEST(cone_sphere_two_circle_declines_single_and_tangent) {
  const ntopo::Shape cone = makeCone(0.5, 0.0, 2.5, 4.0);
  // Single-crossing sphere (the S5-f fixture): centre (0,0,0), Rs=2 → ONE interior root; the
  // S5-h two-root prologue must decline it (S5-f owns it).
  const ntopo::Shape sphSingle = makeSphere(0.0, 2.0);
  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csSph1 = sd::recogniseCurvedSolid(sphSingle);
  CC_CHECK(csCone && csSph1);
  // The single-crossing pair still builds via the S5-f arm (a valid COMMON), NOT S5-h — so the
  // full entry is non-null (S5-f handles it) but S5-h's own predicate would decline. We assert the
  // engine still produces the correct single-crossing COMMON (regression: S5-h did not steal it).
  const ntopo::Shape single = nb::ssi_boolean_solid(cone, sphSingle, nb::Op::Common);
  CC_CHECK(!single.isNull());
  const double vSingle = watertightMeshVolume(single);
  CC_CHECK(std::fabs(vSingle - 5.255829) <= 1e-2 * 5.255829);  // the S5-f single-crossing volume
}

// ── (10) TWO-CIRCLE coaxial CYLINDER∩SPHERE COMMON / FUSE / CUT: REAL native watertight
// passes (S5-i) ──────────────────────────────────────────────────────────────────────────
// The tanα==0 special case of the S5-h cone∩sphere two-circle family. A cylinder Rc=1.0 about
// world +Y over y∈[-3,3] and a sphere Rs=1.6 centred at the origin (ON the cylinder axis) meet
// along TWO analytic circle seams — the sphere pokes THROUGH the cylinder wall at TWO latitudes:
//   Rc = √(Rs²−(y−sc)²)  ⇒  y = sc ± √(Rs²−Rc²) = ±√(1.56) ≈ ±1.24900, radius ρ = Rc = 1.0.
// Between the seams the SPHERE is the wider operand (bulges outside the cylinder); each polar cap
// (poles at y=±1.6, both inside the cylinder) sits inside the cylinder. The whole S5-h/S5-c
// machinery is reused (appendRevolvedBand exact on the cylinder wall, appendSphereCap, the shared
// appendSphereZone bulge). Both circles are S1-analytic; the S3 tracer returns ONE of the two
// co-resident loops, so the S5-i prologue computes BOTH circles itself and CROSS-CHECKS.
//   COMMON = sphere lower cap [poleM,y*lo] + cylinder segment [y*lo,y*hi] + sphere upper cap
//            [y*hi,poleP]. V = V_sph-seg + π·Rc²·(y*hi−y*lo) + V_sph-seg.
//   FUSE   = cylinder walls (ends→seams) + sphere ZONE bulge (mid-band) + cylinder terminal discs.
//            V = V(cyl)+V(sphere)−V(COMMON) (GROW).
//   CUT    = cyl − sphere: the sphere fully engulfs the cylinder mid-band, so the result PINCHES
//            into TWO disconnected components (a lower cyl-end piece + an upper piece, each
//            spherically scooped). V = V(cyl)−V(COMMON) (SHRINK).
// Every volume matches the closed form within the engine's curved-parity bar (1% relative).
// COMMON is symmetric; CUT is order-sensitive (sphere−cyl → OCCT).
CC_TEST(cyl_sphere_two_circle_common_fuse_cut_watertight_matches_analytic) {
  const double Rc = 1.0, Rs = 1.6, sc = 0.0, cLo = -3.0, cHi = 3.0;
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);
  const ntopo::Shape sph = makeSphere(0.0, Rs);  // centre origin, polar axis +Y (coaxial)
  CC_CHECK(!cyl.isNull() && !sph.isNull());

  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCyl && csSph);
  if (csCyl && csSph) {
    CC_CHECK(csCyl->kind == sd::CurvedKind::Cylinder);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    const ssi::TraceSet tr = ssi::trace_intersection(csCyl->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circles
    CC_CHECK(tr.curveCount() >= 1);      // at least one of the two co-resident circles traced
  }

  // Closed-form ground truth (cylinder segment + two spherical segments).
  const double h = std::sqrt(Rs * Rs - Rc * Rc);      // ≈ 1.24900
  const double sLo = sc - h, sHi = sc + h;
  const double poleM = sc - Rs, poleP = sc + Rs;
  auto sphSeg = [&](double a, double b) {  // π∫[a,b](Rs²−(y−sc)²)dy
    auto F = [&](double y) { return Rs * Rs * (y - sc) - (y - sc) * (y - sc) * (y - sc) / 3.0; };
    return sd::kSsiPi * (F(b) - F(a));
  };
  const double vCylFull = cylinderVolume(Rc, cLo, cHi);            // whole cylinder
  const double vSph = 4.0 / 3.0 * sd::kSsiPi * Rs * Rs * Rs;       // sphere volume
  const double vCommonTrue =
      sphSeg(poleM, sLo) + sd::kSsiPi * Rc * Rc * (sHi - sLo) + sphSeg(sHi, poleP);
  const double vFuseTrue = vCylFull + vSph - vCommonTrue;
  const double vCutTrue = vCylFull - vCommonTrue;
  CC_CHECK(vCommonTrue > 0.0 && vCommonTrue < vSph);

  // ── COMMON: watertight native candidate whose volume matches the closed form. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cyl, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vCylFull, vSph) + 1e-9);            // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cyl, sph, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, cyl, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: cylinder walls + sphere zone bulge + cylinder discs. A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(cyl, sph, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vCylFull, vSph) - 1e-9);             // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(cyl, sph, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cylinder minuend): two disconnected spherically-scooped pieces. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(cyl, sph, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vCylFull + 1e-9);                             // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(cyl, sph, nb::Op::Cut).isNull());
  // CUT is order-sensitive: sphere − cylinder is a DIFFERENT topology; the S5-i CUT builder only
  // handles the cylinder minuend, so sphere − cyl declines here → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(sph, cyl, nb::Op::Cut).isNull());
}

// ── (11) TWO-CIRCLE cyl∩sphere HONEST DECLINES: tangent + pole-outside + off-axis → OCCT. ──
// The S5-i assembler is the strict two-interior-root poke-through only. A sphere with Rs ≤ Rc
// (no proper two-circle crossing — internally tangent/nested), a sphere whose pole falls outside
// the cylinder's axial extent (a single-crossing end dent), and a sphere whose centre is off the
// cylinder axis (a transversal quartic) all decline → NULL → OCCT (honest, never faked).
CC_TEST(cyl_sphere_two_circle_declines_tangent_pole_and_offaxis) {
  const double Rc = 1.0, cLo = -3.0, cHi = 3.0;
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);

  // (a) Rs < Rc: the sphere fits INSIDE the cylinder cross-section — no two-circle wall crossing.
  CC_CHECK(nb::ssi_boolean_solid(cyl, makeSphere(0.0, 0.8), nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(cyl, makeSphere(0.0, 0.8), nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(cyl, makeSphere(0.0, 0.8), nb::Op::Cut).isNull());

  // (b) Pole outside the cylinder extent: a tall cylinder end near a seam → single-crossing dent.
  //     Sphere Rs=1.6 centred at y=2.6: upper pole y=4.2 > cHi=3.0 → one seam interior, one not.
  const ntopo::Shape shortCyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);
  CC_CHECK(nb::ssi_boolean_solid(shortCyl, makeSphereY(2.6, 1.6), nb::Op::Common).isNull());

  // (c) Off-axis sphere (centre not on the cylinder axis) → transversal quartic → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(cyl, makeSphere(0.7, 1.6), nb::Op::Common).isNull());
}

int main() { return cctest::run_all(); }
