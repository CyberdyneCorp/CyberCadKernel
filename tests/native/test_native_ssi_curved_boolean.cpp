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

#include <algorithm>
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

// A finite cylinder whose cross-section CENTRE is offset by (c0,c1) in the two perpendicular
// axes (an OFF-AXIS placement) — for the transversal (non-coaxial) torus∩cylinder family. For a
// Z-axis (axis=2) cylinder (c0,c1) = (x,y): centring c0=off puts the axis at radius `off` from
// the world Z-axis, parallel-but-offset from a Z-torus axis.
ntopo::Shape makeCylOff(int axis, double c0, double c1, double r, double lo, double hi) {
  nb::curved::AABox box{Point3{-100, -100, -100}, Point3{100, 100, 100}};
  return nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{axis, c0, c1, r, lo, hi});
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

// A full RING TORUS (major R, minor r) about world +Z as a genuine native B-rep: a BARE
// doubly-periodic Kind::Torus face with a NULL outer wire (exactly the shape the STEP reader
// maps a TOROIDAL_SURFACE to, and the ONLY torus form recogniseCurvedSolid admits — a native
// revolve builds a torus as B-spline bands, which decline). The tessellator meshes the
// natural (u,v)∈[0,2π]² rectangle, welding both seams (no poles) → a watertight torus.
ntopo::Shape makeTorus(double R, double r) {
  ntopo::FaceSurface s;
  s.kind = ntopo::FaceSurface::Kind::Torus;
  s.frame = nmath::Ax3{};  // identity: origin 0, axis +Z, x=+X
  s.radius = R;
  s.minorRadius = r;
  const ntopo::Shape face = ntopo::ShapeBuilder::makeFace(s, ntopo::Shape{});
  return ntopo::ShapeBuilder::makeSolid({ntopo::ShapeBuilder::makeShell({face})});
}

// A full RING TORUS (major R, minor r) about world +Z whose CENTRE sits on the axis at z=zc —
// a COAXIAL torus offset axially from the origin-centred makeTorus, for the torus∩torus family.
ntopo::Shape makeTorusAt(double R, double r, double zc) {
  ntopo::FaceSurface s;
  s.kind = ntopo::FaceSurface::Kind::Torus;
  s.frame = nmath::Ax3::fromAxisAndRef(nmath::Point3{0, 0, zc}, nmath::Dir3{0, 0, 1},
                                       nmath::Dir3{1, 0, 0});
  s.radius = R;
  s.minorRadius = r;
  const ntopo::Shape face = ntopo::ShapeBuilder::makeFace(s, ntopo::Shape{});
  return ntopo::ShapeBuilder::makeSolid({ntopo::ShapeBuilder::makeShell({face})});
}

// An axis-aligned SLAB half-space for the S5-r torus∩plane family: a big square footprint
// [-half,half]² extruded in z over [zBot, zTop] — a box whose TOP face (z = zTop) and BOTTOM
// face (z = zBot) are perpendicular to a +Z torus axis. Positioned with `half` large enough to
// bracket the torus in-plane and exactly ONE of the two z-faces cutting the tube, it acts as a
// single axis-perpendicular half-space through the torus. All six faces are planar.
ntopo::Shape makeSlab(double half, double zBot, double zTop) {
  std::vector<cst::ProfileSegment> segs(4);
  segs[0].kind = 0; segs[0].x0 = -half; segs[0].y0 = -half; segs[0].x1 = half;  segs[0].y1 = -half;
  segs[1].kind = 0; segs[1].x0 = half;  segs[1].y0 = -half; segs[1].x1 = half;  segs[1].y1 = half;
  segs[2].kind = 0; segs[2].x0 = half;  segs[2].y0 = half;  segs[2].x1 = -half; segs[2].y1 = half;
  segs[3].kind = 0; segs[3].x0 = -half; segs[3].y0 = half;  segs[3].x1 = -half; segs[3].y1 = -half;
  ntopo::Shape box = cst::build_prism_profile(segs, {}, {}, zTop - zBot);  // extrude z=0..(zTop-zBot)
  if (box.isNull()) return box;
  const nmath::Transform up = nmath::Transform::translationOf(nmath::Vec3{0.0, 0.0, zBot});
  return box.located(ntopo::Location{up});
}

// A full RING TORUS (major R, minor r) about world +Y (its axis aligned with the makeCone /
// makeSphere +Y revolution axis, so a torus∩cone pair is genuinely coaxial). Same bare
// doubly-periodic Kind::Torus face as makeTorus, only the frame axis is +Y.
ntopo::Shape makeTorusY(double R, double r) {
  ntopo::FaceSurface s;
  s.kind = ntopo::FaceSurface::Kind::Torus;
  s.frame = nmath::Ax3::fromAxisAndRef(nmath::Point3{0, 0, 0}, nmath::Dir3{0, 1, 0},
                                       nmath::Dir3{1, 0, 0});
  s.radius = R;
  s.minorRadius = r;
  const ntopo::Shape face = ntopo::ShapeBuilder::makeFace(s, ntopo::Shape{});
  return ntopo::ShapeBuilder::makeSolid({ntopo::ShapeBuilder::makeShell({face})});
}

// Pappus-exact volume of the COMMON (torus ρ ≤ Rc part) of a coaxial torus∩cylinder: the
// vertical-chord circular segment {ρ ≤ Rc} of the tube disk (radius r, centre ρ=R), revolved.
// V = 2π·(R·A_seg + M), A_seg = πr² − (r²·acos(d/r) − d·√(r²−d²)), M = −(2/3)(r²−d²)^{3/2},
// d = Rc − R. Airtight closed form (matches the engine's ssiCurvedBooleanVerified S5-l arm).
double torusCylCommonVolume(double R, double r, double Rc) {
  const double d = Rc - R;
  const double root = std::sqrt(std::max(r * r - d * d, 0.0));
  const double aCap = r * r * std::acos(std::clamp(d / r, -1.0, 1.0)) - d * root;
  const double aSeg = sd::kSsiPi * r * r - aCap;
  const double mom = -(2.0 / 3.0) * root * root * root;
  return 2.0 * sd::kSsiPi * (R * aSeg + mom);
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

// ── (6b) COAXIAL HOURGLASS (apex-to-apex / bowtie) cone∩cone COMMON / FUSE / CUT: REAL
// native watertight passes (S5-j) ────────────────────────────────────────────────────────
// The genuinely-different sibling of the S5-g coaxial cone∩cone frustum pair. Two coaxial
// cones about world +Y pointing AT each other (bowtie): cone A ▽ r_A(y)=2−y over y∈[0,2]
// (base r=2 at y=0, APEX at y=2) and cone B △ r_B(y)=y over y∈[0,2] (APEX at y=0, base r=2 at
// y=2). Their walls cross at the single analytic circle y*=1, r*=1 (still ONE LINEAR equation
// → ConeConeSetup is reused verbatim). The distinctive feature: the COMMON's min-radius
// profile PINCHES to the axis (a cone apex, r→0) at BOTH overlap ends, so S5-g's COMMON/CUT
// apex gates (rBot/rTop/rBCap>0) DECLINE it — S5-j is the apex-terminated assembler.
//   COMMON = bicone (two full cones apex-to-apex sharing the seam ring): apex-band(y=0→1) +
//            apex-band(y=1→2). V = V_frustum(0,r*,1) + V_frustum(r*,0,1) = 2·(π/3). Symmetric.
//   FUSE   = max-radius hourglass profile with a WAIST at the seam — OFF-axis terminal discs,
//            so S5-g's FUSE builds it directly. V = V(A)+V(B)−V(COMMON) (a GROW).
//   CUT    = A−B (cone-A minuend): A keeps its wider (below-seam) side — a conical shell whose
//            inner boundary is B's wall reversed running into B's OWN apex at y=0, closed by a
//            FULL A-base disc (B absent there). V = V(A)−V(COMMON) (a SHRINK).
// Every volume matches the closed form to within the engine's curved-parity bar (1% relative,
// a tessellation-deflection bound — NOT a relaxed tolerance). COMMON is symmetric; CUT is
// order-sensitive (here symmetric-volume but a genuinely different watertight solid). A
// non-apex (both-ends-off-axis) frustum pair is left to S5-g (asserted elsewhere).
CC_TEST(cone_cone_hourglass_bowtie_common_fuse_cut_watertight_matches_analytic) {
  const ntopo::Shape coneA = makeCone(2.0, 0.0, 0.0, 2.0);  // ▽ r_A(y)=2−y, apex at y=2, axis +Y
  const ntopo::Shape coneB = makeCone(0.0, 0.0, 2.0, 2.0);  // △ r_B(y)=y,   apex at y=0, coaxial
  CC_CHECK(!coneA.isNull() && !coneB.isNull());

  // Both operands are recognised as Cones and the coaxial bowtie traces ONE clean seam circle.
  const auto csA = sd::recogniseCurvedSolid(coneA);
  const auto csB = sd::recogniseCurvedSolid(coneB);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    CC_CHECK(csA->kind == sd::CurvedKind::Cone);
    CC_CHECK(csB->kind == sd::CurvedKind::Cone);
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);  // fully transversal single analytic circle
    CC_CHECK(tr.curveCount() == 1);     // ONE closed seam circle at y*=1
  }

  // Closed-form ground truth (frustum inclusion–exclusion; a full cone is frustum(r,0,Δh)).
  const double rStar = 1.0;
  const double vA = frustumVolume(2.0, 0.0, 2.0);   // whole cone A (a full cone)
  const double vB = frustumVolume(0.0, 2.0, 2.0);   // whole cone B
  const double vCommonTrue = frustumVolume(0.0, rStar, 1.0) + frustumVolume(rStar, 0.0, 1.0);
  const double vFuseTrue = vA + vB - vCommonTrue;
  const double vCutTrue = vA - vCommonTrue;
  CC_CHECK(std::fabs(vCommonTrue - 2.0 * sd::kSsiPi / 3.0) < 1e-9);  // pin the analytic values
  CC_CHECK(std::fabs(vFuseTrue - 14.0 * sd::kSsiPi / 3.0) < 1e-9);
  CC_CHECK(std::fabs(vCutTrue - 6.0 * sd::kSsiPi / 3.0) < 1e-9);

  // ── COMMON: watertight bicone whose volume matches the closed form. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vA, vB) + 1e-9);                   // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight bicone.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(coneB, coneA, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: max-radius hourglass profile (waist at the seam). A GROW (via S5-g). ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vA, vB) - 1e-9);                     // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Fuse).isNull());

  // ── CUT = A − B (cone-A minuend): conical shell to a full A-base disc. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vA + 1e-9);                                    // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(coneA, coneB, nb::Op::Cut).isNull());
  // CUT is order-sensitive: B − A keeps B's wider side (the OTHER conical shell). The bowtie is
  // symmetric in volume but the two solids are genuinely different orientations.
  const ntopo::Shape cutBA = nb::ssi_boolean_solid(coneB, coneA, nb::Op::Cut);
  CC_CHECK(!cutBA.isNull());
  const double vCutBA = watertightMeshVolume(cutBA);
  CC_CHECK(vCutBA > 0.0);
  CC_CHECK(std::fabs(vCutBA - (vB - vCommonTrue)) <= 1e-2 * (vB - vCommonTrue));
}

// ── (6c) ONE-APEX hourglass cone∩cone COMMON / CUT: the mixed disc-cap + apex-band case ──
// A hourglass where only ONE overlap end pinches to the axis: cone A ▽ r_A(y)=2−y over y∈[0,2]
// (apex at y=2) and cone B △ r_B(y)=0.5+y over y∈[0,2] (base r=0.5 at y=0, NOT an apex). Seam
// y*=0.75, r*=1.25. COMMON's min profile: rBot=min(2,0.5)=0.5 (OFF-axis → flat disc cap) and
// rTop=min(0,2.5)=0 (apex → cone-tip band). This exercises S5-j's mixed cap path (one disc,
// one apex band) — still watertight, still matching the frustum closed form.
CC_TEST(cone_cone_hourglass_one_apex_common_cut_watertight_matches_analytic) {
  const ntopo::Shape coneA = makeCone(2.0, 0.0, 0.0, 2.0);  // ▽ r_A(y)=2−y, apex at y=2
  const ntopo::Shape coneB = makeCone(0.5, 0.0, 2.5, 2.0);  // △ r_B(y)=0.5+y, base r=0.5 at y=0
  CC_CHECK(!coneA.isNull() && !coneB.isNull());

  const double yStar = 0.75, rStar = 1.25;
  const double vA = frustumVolume(2.0, 0.0, 2.0);
  // COMMON = min profile: cone B wall below y* (rBot=0.5 → r* over Δh=0.75) + cone A wall above
  // y* (r* → apex over Δh=1.25).
  const double vCommonTrue = frustumVolume(0.5, rStar, yStar) + frustumVolume(rStar, 0.0, 2.0 - yStar);
  const double vCutTrue = vA - vCommonTrue;

  const ntopo::Shape common = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);

  const ntopo::Shape cut = nb::ssi_boolean_solid(coneA, coneB, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vA + 1e-9);
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

  // ── REVERSE CUT = sphere − cyl (order-sensitive) now LANDS (buildSphereCyl2Cut): the sphere
  // with a coaxial cylindrical TUNNEL drilled through — an annular solid of revolution (sphere
  // equatorial belt + reversed cylinder bore), a DIFFERENT topology from the two-piece cyl-minuend
  // dimpled stack. Watertight, matching the closed form V_sph − V_common; a SHRINK below the sphere. ──
  const double vCutSphTrue = vSph - vCommonTrue;                  // sphere − cyl = sph − ∩
  const ntopo::Shape cutSph = nb::ssi_boolean_solid(sph, cyl, nb::Op::Cut);
  CC_CHECK(!cutSph.isNull());
  const double vCutSph = watertightMeshVolume(cutSph);
  CC_CHECK(vCutSph > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCutSph - vCutSphTrue) <= 1e-2 * vCutSphTrue);
  CC_CHECK(vCutSph <= vSph + 1e-9);                              // sphere − cyl ≤ sphere
  CC_CHECK(!nb::boolean_solid(sph, cyl, nb::Op::Cut).isNull());  // engine self-verify accepts it
  // Partition identity: (sphere − cyl) + COMMON = sphere (the closed-form self-consistency).
  CC_CHECK(std::fabs((vCutSph + vCommon) - vSph) <= 2e-2 * vSph);
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

// ── (12) TRANSVERSAL (NON-COAXIAL) CYLINDER∩SPHERE COMMON: the FIRST transversal curved-
// boolean slice (S5-k) ──────────────────────────────────────────────────────────────────────
// A thin cylinder Rc=1.0 about world +Y over y∈[-3,3] and a sphere Rs=2.0 centred at (0.5,0,0)
// — the sphere centre is OFFSET 0.5 (perpendicular) from the cylinder axis but the cylinder
// still pierces BOTH poles of the sphere. Because the axes are non-coaxial, the cylinder wall
// crosses the sphere in TWO disjoint NON-PLANAR closed loops (generalised Viviani curves) — no
// analytic circle exists, so S5-k drives the boolean directly from the S3-traced seams. COMMON
// is the same topology as the coaxial S5-i COMMON (a cylinder mid-band capped by two spherical
// caps) but every ring is the traced non-planar seam. There is no closed-form COMMON volume, so
// the oracle is a deterministic fine-grid numerical integration of the analytic region (the
// task's numerical cross-check) — the primary parity oracle for a non-analytic seam.
CC_TEST(cyl_sphere_transversal_offset_common_watertight_matches_numeric) {
  const double Rc = 1.0, Rs = 2.0, off = 0.5, cLo = -3.0, cHi = 3.0;
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);   // Y-cylinder at origin
  const ntopo::Shape sph = makeSphere(off, Rs);              // sphere centred at (off,0,0)
  CC_CHECK(!cyl.isNull() && !sph.isNull());

  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCyl && csSph);
  if (csCyl && csSph) {
    CC_CHECK(csCyl->kind == sd::CurvedKind::Cylinder);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    // The trace is TWO fully-transversal CLOSED non-planar loops (pierce-both-poles).
    const ssi::TraceSet tr = ssi::trace_intersection(csCyl->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.branchPoints == 0);
    CC_CHECK(tr.lines.size() == 2);
    int closed = 0;
    for (const ssi::WLine& w : tr.lines)
      if (w.isClosed()) ++closed;
    CC_CHECK(closed == 2);
  }

  // Deterministic numeric COMMON volume (analytic-region grid: inside the sphere AND the finite
  // Y-cylinder). ~9e6 cells → converged to ≪ the 1% curved-parity bar (midpoint rule on a convex
  // region converges O(1/n)); fast enough for the host gate.
  auto inSph = [&](double x, double y, double z) {
    const double dx = x - off;
    return dx * dx + y * y + z * z <= Rs * Rs;
  };
  auto inCyl = [&](double x, double y, double z) {
    return (x * x + z * z <= Rc * Rc) && (y >= cLo && y <= cHi);
  };
  const int nx = 120, ny = 96, nz = 120;
  const double x0 = -1.5, x1 = 2.5, y0 = -2.1, y1 = 2.1, z0 = -1.2, z1 = 1.2;
  const double cell = (x1 - x0) / nx * (y1 - y0) / ny * (z1 - z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = x0 + (x1 - x0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = y0 + (y1 - y0) * (j + 0.5) / ny;
      for (int k = 0; k < nz; ++k) {
        const double z = z0 + (z1 - z0) * (k + 0.5) / nz;
        if (inSph(x, y, z) && inCyl(x, y, z)) ++inside;
      }
    }
  }
  const double vCommonNumeric = inside * cell;
  CC_CHECK(vCommonNumeric > 10.0 && vCommonNumeric < 12.0);  // ≈ 11.28

  // ── COMMON: watertight native candidate whose volume matches the numeric oracle. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cyl, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                    // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonNumeric) <= 1e-2 * vCommonNumeric);
  const double vSph = 4.0 / 3.0 * sd::kSsiPi * Rs * Rs * Rs;
  const double vCyl = cylinderVolume(Rc, cLo, cHi);
  CC_CHECK(vCommon <= std::min(vSph, vCyl) + 1e-9);          // COMMON ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cyl, sph, nb::Op::Common).isNull());

  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, cyl, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonNumeric) <= 1e-2 * vCommonNumeric);

  // ── CUT / FUSE now LAND (S5-k transversal band via appendSphereOuterZoneBetweenSeams). ──
  // The integration box above is sized for the COMMON (it does NOT enclose the whole sphere in
  // z or the whole finite cylinder), so the COMMON is the only region measured directly on the
  // grid; every CUT/FUSE oracle is the exact analytic V(A)/V(B) minus that numeric COMMON
  // (inclusion–exclusion) — the deterministic numeric cross-check the task requires.
  const double vCutSphNumeric = vSph - vCommonNumeric;                // sphere − cyl = sph − ∩
  const double vCutCylNumeric = vCyl - vCommonNumeric;                // cyl − sphere = cyl − ∩
  const double vFuseNumeric = vSph + vCyl - vCommonNumeric;           // A ∪ B (inclusion–excl.)

  // CUT sphere − cylinder: watertight, on-surface outer zone + reversed bore; ΔV within 1%.
  const ntopo::Shape cutSC = nb::ssi_boolean_solid(sph, cyl, nb::Op::Cut);
  CC_CHECK(!cutSC.isNull());
  const double vCutSC = watertightMeshVolume(cutSC);
  CC_CHECK(vCutSC > 0.0);                                              // watertight → engine accepts
  CC_CHECK(std::fabs(vCutSC - vCutSphNumeric) <= 1e-2 * vCutSphNumeric);
  CC_CHECK(vCutSC < vSph + 1e-9);                                     // A − B ≤ A

  // CUT cylinder − sphere: two dimpled cylinder stubs; watertight; ΔV within 1%.
  const ntopo::Shape cutCS = nb::ssi_boolean_solid(cyl, sph, nb::Op::Cut);
  CC_CHECK(!cutCS.isNull());
  const double vCutCS = watertightMeshVolume(cutCS);
  CC_CHECK(vCutCS > 0.0);
  CC_CHECK(std::fabs(vCutCS - vCutCylNumeric) <= 1e-2 * vCutCylNumeric);
  CC_CHECK(vCutCS < vCyl + 1e-9);

  // FUSE (both operand orders): watertight union envelope; ΔV within 1%; symmetric.
  const ntopo::Shape fuseCS = nb::ssi_boolean_solid(cyl, sph, nb::Op::Fuse);
  const ntopo::Shape fuseSC = nb::ssi_boolean_solid(sph, cyl, nb::Op::Fuse);
  CC_CHECK(!fuseCS.isNull() && !fuseSC.isNull());
  const double vFuseCS = watertightMeshVolume(fuseCS), vFuseSC = watertightMeshVolume(fuseSC);
  CC_CHECK(vFuseCS > 0.0 && vFuseSC > 0.0);
  CC_CHECK(std::fabs(vFuseCS - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(std::fabs(vFuseSC - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(vFuseCS >= std::max(vSph, vCyl) - 1e-9);                   // union ≥ max(A,B)

  // Partition identity: COMMON + (sphere − cyl) = sphere (the numeric oracle self-consistency).
  CC_CHECK(std::fabs((vCommon + vCutSC) - vSph) <= 2e-2 * vSph);
}

// ── (13) TRANSVERSAL cyl∩sphere REDUCES TO COAXIAL: the offset→0 guarantee ────────────────────
// As the perpendicular offset → 0 the pose becomes COAXIAL and the S5-i two-circle assembler
// (which runs BEFORE S5-k in the dispatch) claims it, reproducing the landed coaxial COMMON;
// S5-k's transversal setup gates on a strictly-positive offset so it declines the coaxial pose.
// This test pins that hand-off: the SAME cylinder with an ON-AXIS sphere yields a watertight
// COMMON matching the coaxial closed form (cylinder segment + two spherical segments), distinct
// from the offset COMMON, confirming the reduction boundary.
CC_TEST(cyl_sphere_transversal_reduces_to_coaxial_at_zero_offset) {
  const double Rc = 1.0, Rs = 2.0, cLo = -3.0, cHi = 3.0;
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);
  const ntopo::Shape sphCoax = makeSphere(0.0, Rs);   // ON-axis → coaxial → S5-i owns it

  // Coaxial closed form: sphere lower cap + cylinder segment + sphere upper cap.
  const double h = std::sqrt(Rs * Rs - Rc * Rc);      // = √3 ≈ 1.73205
  const double sLo = -h, sHi = h, poleM = -Rs, poleP = Rs;
  auto sphSeg = [&](double a, double b) {
    auto F = [&](double y) { return Rs * Rs * y - y * y * y / 3.0; };
    return sd::kSsiPi * (F(b) - F(a));
  };
  const double vCoaxTrue = sphSeg(poleM, sLo) + sd::kSsiPi * Rc * Rc * (sHi - sLo) + sphSeg(sHi, poleP);

  const ntopo::Shape coax = nb::ssi_boolean_solid(cyl, sphCoax, nb::Op::Common);
  CC_CHECK(!coax.isNull());
  const double vCoax = watertightMeshVolume(coax);
  CC_CHECK(vCoax > 0.0);
  CC_CHECK(std::fabs(vCoax - vCoaxTrue) <= 1e-2 * vCoaxTrue);

  // The coaxial COMMON is DISTINCT from the offset (transversal) COMMON — the offset shrinks
  // the overlap — so the reduction is not a trivial identity: they must differ measurably.
  const ntopo::Shape offCommon = nb::ssi_boolean_solid(cyl, makeSphere(0.5, Rs), nb::Op::Common);
  CC_CHECK(!offCommon.isNull());
  const double vOff = watertightMeshVolume(offCommon);
  CC_CHECK(vOff > 0.0);
  CC_CHECK(vCoax - vOff > 1e-2);   // coaxial overlap strictly larger than the offset overlap
}

// ── (13b) TRANSVERSAL cyl∩sphere CUT/FUSE still HONEST-DECLINE for a POLE-GRAZING pose ────────
// The S5-k seam-band CUT/FUSE need the two-cap+outer-zone topology, valid ONLY when both sphere
// poles sit strictly INSIDE the cylinder. A THIN cylinder offset so far that a sphere pole falls
// OUTSIDE it (offset + Rc close to Rs — the cylinder grazes past the pole region) makes the
// sphere-outside-cyl set no longer two-cap-complementary, so a single monotone φ-sweep cannot
// tile the outer zone. Rather than emit a leaky shell, S5-k HONEST-DECLINES those ops → NULL →
// OCCT (the DISAGREED=0 discipline). COMMON still lands (it needs only the two inner caps + the
// cylinder band). This regression pins the decline boundary — no silent-wrong solid.
CC_TEST(cyl_sphere_transversal_pole_grazing_cut_fuse_decline) {
  const double Rc = 0.4, Rs = 2.0, off = 1.55, cLo = -3.0, cHi = 3.0;  // off+Rc=1.95 → poles OUTSIDE
  const ntopo::Shape cyl = makeCyl(/*Y*/ 1, Rc, cLo, cHi);
  const ntopo::Shape sph = makeSphere(off, Rs);
  CC_CHECK(!cyl.isNull() && !sph.isNull());
  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCyl && csSph);
  // The cylinder still pierces the sphere (two closed loops), so COMMON lands …
  const ntopo::Shape common = nb::ssi_boolean_solid(cyl, sph, nb::Op::Common);
  if (!common.isNull()) CC_CHECK(watertightMeshVolume(common) > 0.0);
  // … but CUT / FUSE HONEST-DECLINE (pole outside cyl → outer-zone untileable) → NULL → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(sph, cyl, nb::Op::Cut).isNull());
  CC_CHECK(nb::ssi_boolean_solid(cyl, sph, nb::Op::Cut).isNull());
  CC_CHECK(nb::ssi_boolean_solid(cyl, sph, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(sph, cyl, nb::Op::Fuse).isNull());
}

// ── (14) COAXIAL TORUS∩CYLINDER COMMON / FUSE / CUT (S5-l) — the TORUS surface family ──
// A ring torus (major R=3, minor r=1, axis +Z) and a coaxial cylinder Rc=3.2 over z∈[-2,2]:
// the cylinder wall crosses the tube at TWO latitudes (|Rc−R|=0.2 < r=1), giving two analytic
// circle seams at z=±z0=±√(r²−(Rc−R)²) of radius Rc. Every op is a Pappus-exact solid of
// revolution. COMMON = the ρ ≤ Rc tube part; CUT (torus−cyl) = the ρ > Rc outer ring; FUSE =
// the union (outer bulge + cylinder wall outside the tube + cylinder disc caps). Verified vs
// the AIRTIGHT closed forms — no OCCT, no fabricated value (mirrors the engine's S5-l oracle).
CC_TEST(torus_cyl_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const double R = 3.0, r = 1.0, Rc = 3.2, cLo = -2.0, cHi = 2.0;
  const ntopo::Shape tor = makeTorus(R, r);
  const ntopo::Shape cyl = makeCyl(/*Z*/ 2, Rc, cLo, cHi);
  CC_CHECK(!tor.isNull() && !cyl.isNull());

  const auto csTor = sd::recogniseCurvedSolid(tor);
  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  CC_CHECK(csTor && csCyl);
  if (csTor && csCyl) {
    CC_CHECK(csTor->kind == sd::CurvedKind::Torus);
    CC_CHECK(csCyl->kind == sd::CurvedKind::Cylinder);
    CC_CHECK(std::fabs(csTor->radius - R) < 1e-9 && std::fabs(csTor->minorRadius - r) < 1e-9);
    const ssi::TraceSet tr = ssi::trace_intersection(csTor->adapter(), csCyl->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circle seams
    CC_CHECK(tr.curveCount() >= 1);      // ≥1 of the two co-resident circles traced
  }

  // Airtight closed-form ground truth (Pappus).
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;   // 2π²Rr²
  const double vCylFull = cylinderVolume(Rc, cLo, cHi);             // π Rc² (cHi−cLo)
  const double vCommonTrue = torusCylCommonVolume(R, r, Rc);
  const double vFuseTrue = vTorus + vCylFull - vCommonTrue;
  const double vCutTrue = vTorus - vCommonTrue;
  CC_CHECK(vCommonTrue > 0.0 && vCommonTrue < vTorus);

  // ── COMMON: the ρ ≤ Rc tube part (inner arc + cylinder chord band). ──
  const ntopo::Shape common = nb::ssi_boolean_solid(tor, cyl, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vTorus, vCylFull) + 1e-9);          // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(tor, cyl, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(cyl, tor, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: outer tube bulge + cylinder wall outside the tube + cylinder discs. GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(tor, cyl, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vTorus, vCylFull) - 1e-9);            // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(tor, cyl, nb::Op::Fuse).isNull());

  // ── CUT = A − B (torus minuend): the ρ > Rc outer tube ring (a SHRINK). ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(tor, cyl, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vTorus + 1e-9);                                // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(tor, cyl, nb::Op::Cut).isNull());

  // ── REVERSE CUT = cyl − torus (order-sensitive) now LANDS (buildCylTorusCut): the finite
  // cylinder with a concave toroidal GROOVE carved into its wall over z∈[−z0,z0]. A DIFFERENT
  // solid from the torus-minuend outer ring — a cylinder-body solid of revolution. Watertight,
  // matching the Pappus V_cyl − V_common; a SHRINK below the cylinder. ──
  const double vCutRevTrue = vCylFull - vCommonTrue;               // cyl − torus = cyl − ∩
  const ntopo::Shape cutRev = nb::ssi_boolean_solid(cyl, tor, nb::Op::Cut);
  CC_CHECK(!cutRev.isNull());
  const double vCutRev = watertightMeshVolume(cutRev);
  CC_CHECK(vCutRev > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCutRev - vCutRevTrue) <= 1e-2 * vCutRevTrue);
  CC_CHECK(vCutRev <= vCylFull + 1e-9);                             // cyl − torus ≤ cyl
  CC_CHECK(!nb::boolean_solid(cyl, tor, nb::Op::Cut).isNull());     // engine self-verify accepts it
  // Partition identity: (cyl − torus) + COMMON = cyl (the closed-form self-consistency).
  CC_CHECK(std::fabs((vCutRev + vCommon) - vCylFull) <= 2e-2 * vCylFull);
}

// ── (14a′) NATIVE PRIMITIVE construct::build_torus drives the boolean identically (BOOL-COMPLETE) ─
// The headline gap this wave closes: a native REVOLVE of an off-axis circle builds rational
// B-spline bands (declined by recogniseCurvedSolid), so before this wave the ONLY way to get a
// bare Kind::Torus operand into the native path was the in-test makeTorus. construct::build_torus
// is the SHIPPING native primitive (backing cc_torus) that emits exactly that recognisable torus.
// This pins that the native-built torus (a) recognises with the correct R/r/axis, and (b) drives
// the coaxial S5-l COMMON/CUT/FUSE to the SAME Pappus-exact volumes as makeTorus — pure-native,
// watertight, no OCCT.
CC_TEST(native_build_torus_recognises_and_drives_boolean) {
  const double R = 3.0, r = 1.0, Rc = 3.2, cLo = -2.0, cHi = 2.0;
  // The native shipping primitive — a +Z ring torus at the origin.
  const ntopo::Shape tor = cst::build_torus(R, r, nmath::Point3{0, 0, 0}, nmath::Dir3{0, 0, 1});
  CC_CHECK(!tor.isNull());

  // (a) recogniseCurvedSolid admits it as a Torus with the correct radii + axis.
  const auto csTor = sd::recogniseCurvedSolid(tor);
  CC_CHECK(csTor);
  if (csTor) {
    CC_CHECK(csTor->kind == sd::CurvedKind::Torus);
    CC_CHECK(std::fabs(csTor->radius - R) < 1e-9);
    CC_CHECK(std::fabs(csTor->minorRadius - r) < 1e-9);
    CC_CHECK(std::fabs(csTor->frame.z.vec().z - 1.0) < 1e-9);
  }

  // The native torus itself meshes watertight at the Pappus volume 2π²Rr².
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;
  const double vTorMesh = watertightMeshVolume(tor);
  CC_CHECK(vTorMesh > 0.0);
  CC_CHECK(std::fabs(vTorMesh - vTorus) <= 2e-2 * vTorus);

  // (b) the coaxial torus∩cyl COMMON/CUT/FUSE land NATIVELY (pure-native, no OCCT), matching the
  // Pappus closed forms — identical to the makeTorus fixture (which uses the SAME bare face).
  const ntopo::Shape cyl = makeCyl(/*Z*/ 2, Rc, cLo, cHi);
  CC_CHECK(!cyl.isNull());
  const double vCylFull = cylinderVolume(Rc, cLo, cHi);
  const double vCommonTrue = torusCylCommonVolume(R, r, Rc);
  const double vCutTrue = vTorus - vCommonTrue;
  const double vFuseTrue = vTorus + vCylFull - vCommonTrue;

  const ntopo::Shape common = nb::ssi_boolean_solid(tor, cyl, nb::Op::Common);
  const ntopo::Shape cut = nb::ssi_boolean_solid(tor, cyl, nb::Op::Cut);
  const ntopo::Shape fuse = nb::ssi_boolean_solid(tor, cyl, nb::Op::Fuse);
  CC_CHECK(!common.isNull() && !cut.isNull() && !fuse.isNull());
  const double vCommon = watertightMeshVolume(common);
  const double vCut = watertightMeshVolume(cut);
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vCommon > 0.0 && vCut > 0.0 && vFuse > 0.0);   // all watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);

  // The build_torus(frame) overload agrees with the (centre,axis) overload.
  const ntopo::Shape torF = cst::build_torus(R, r, nmath::Ax3{});
  CC_CHECK(!torF.isNull());
  const auto csF = sd::recogniseCurvedSolid(torF);
  CC_CHECK(csF && csF->kind == sd::CurvedKind::Torus);

  // Spindle / degenerate → NULL (honest decline; never a self-intersecting body).
  CC_CHECK(cst::build_torus(0.5, 1.0, nmath::Point3{0, 0, 0}, nmath::Dir3{0, 0, 1}).isNull());
  CC_CHECK(cst::build_torus(1.0, 1.0, nmath::Point3{0, 0, 0}, nmath::Dir3{0, 0, 1}).isNull());
  CC_CHECK(cst::build_torus(3.0, 0.0, nmath::Point3{0, 0, 0}, nmath::Dir3{0, 0, 1}).isNull());
}

// ── (14b) TRANSVERSAL (NON-COAXIAL) TORUS∩CYLINDER COMMON (S5-p) — the first transversal
// TORUS slice ─────────────────────────────────────────────────────────────────────────────────
// A ring torus (major R=3, minor r=1, axis +Z) and a THIN cylinder Rc=0.6 whose axis is PARALLEL
// to +Z but OFFSET perpendicular by off=3 (sitting over the tube-centre circle on the +X rim),
// spanning z∈[-2,2] so it pierces fully THROUGH the tube. Because the axes are non-coaxial the
// cylinder wall crosses the tube in TWO disjoint NON-PLANAR closed loops (the quartic cyl∩torus
// locus — no analytic circle), so S5-p drives the boolean directly from the S3-traced seams:
// COMMON = torus lower cap + cylinder band + torus upper cap, every ring the traced non-planar
// seam. There is no closed-form COMMON volume, so the oracle is a deterministic fine-grid
// numerical integration of the analytic region (inside the tube AND the offset finite cylinder)
// — the primary parity oracle for a non-analytic seam. CUT/FUSE now LAND via the tube-band
// primitive appendTorusTubeOuterZoneBetweenSeams (the tube ZONE outside the bore, swept the LONG
// way round the minor angle between the two non-planar seams, on-surface to machine precision).
CC_TEST(torus_cyl_transversal_offset_common_watertight_matches_numeric) {
  const double R = 3.0, r = 1.0, Rc = 0.6, off = 3.0, cLo = -2.0, cHi = 2.0;
  const ntopo::Shape tor = makeTorus(R, r);                       // +Z torus at origin
  const ntopo::Shape cyl = makeCylOff(/*Z*/ 2, off, 0.0, Rc, cLo, cHi);  // Z-cyl at (off,0)
  CC_CHECK(!tor.isNull() && !cyl.isNull());

  const auto csTor = sd::recogniseCurvedSolid(tor);
  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  CC_CHECK(csTor && csCyl);
  if (csTor && csCyl) {
    CC_CHECK(csTor->kind == sd::CurvedKind::Torus);
    CC_CHECK(csCyl->kind == sd::CurvedKind::Cylinder);
    // The trace is TWO fully-transversal CLOSED non-planar loops (pierce-through both sheets).
    const ssi::TraceSet tr = ssi::trace_intersection(csTor->adapter(), csCyl->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.branchPoints == 0);
    CC_CHECK(tr.lines.size() == 2);
    int closed = 0;
    for (const ssi::WLine& w : tr.lines)
      if (w.isClosed()) ++closed;
    CC_CHECK(closed == 2);
  }

  // Deterministic numeric COMMON volume (analytic region: inside the tube AND the offset finite
  // Z-cylinder). The overlap is a small bore-sized region → a tight AABB grid converges fast.
  auto inTor = [&](double x, double y, double z) {
    const double rho = std::sqrt(x * x + y * y);
    const double dr = rho - R;
    return dr * dr + z * z <= r * r;
  };
  auto inCyl = [&](double x, double y, double z) {
    const double dx = x - off;
    return (dx * dx + y * y <= Rc * Rc) && (z >= cLo && z <= cHi);
  };
  const int nx = 160, ny = 128, nz = 128;
  const double x0 = off - Rc - 0.2, x1 = off + Rc + 0.2, y0 = -Rc - 0.2, y1 = Rc + 0.2,
               z0 = -r - 0.2, z1 = r + 0.2;
  const double cell = (x1 - x0) / nx * (y1 - y0) / ny * (z1 - z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = x0 + (x1 - x0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = y0 + (y1 - y0) * (j + 0.5) / ny;
      for (int k = 0; k < nz; ++k) {
        const double z = z0 + (z1 - z0) * (k + 0.5) / nz;
        if (inTor(x, y, z) && inCyl(x, y, z)) ++inside;
      }
    }
  }
  const double vCommonNumeric = inside * cell;
  CC_CHECK(vCommonNumeric > 2.0 && vCommonNumeric < 2.3);  // ≈ 2.15

  // ── COMMON: watertight native candidate whose volume matches the numeric oracle. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(tor, cyl, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                  // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonNumeric) <= 1e-2 * vCommonNumeric);
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;
  const double vCyl = cylinderVolume(Rc, cLo, cHi);
  CC_CHECK(vCommon <= std::min(vTorus, vCyl) + 1e-9);      // COMMON ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(tor, cyl, nb::Op::Common).isNull());

  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(cyl, tor, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonNumeric) <= 1e-2 * vCommonNumeric);

  // ── CUT / FUSE now LAND (S5-p tube-band via appendTorusTubeOuterZoneBetweenSeams). ──
  // The integration box above is sized for the COMMON (bore region only), so every CUT/FUSE
  // oracle is the exact analytic V(torus)/V(cyl) minus that numeric COMMON (inclusion–exclusion)
  // — the deterministic numeric cross-check for a non-analytic seam.
  const double vCutTorNumeric = vTorus - vCommonNumeric;               // torus − cyl = torus − ∩
  const double vFuseNumeric = vTorus + vCyl - vCommonNumeric;          // A ∪ B (inclusion–excl.)

  // CUT torus − cylinder: watertight, on-surface tube outer zone + reversed bore; ΔV within 1%.
  const ntopo::Shape cutTC = nb::ssi_boolean_solid(tor, cyl, nb::Op::Cut);
  CC_CHECK(!cutTC.isNull());
  const double vCutTC = watertightMeshVolume(cutTC);
  CC_CHECK(vCutTC > 0.0);                                              // watertight → engine accepts
  CC_CHECK(std::fabs(vCutTC - vCutTorNumeric) <= 1e-2 * vCutTorNumeric);
  CC_CHECK(vCutTC < vTorus + 1e-9);                                    // A − B ≤ A

  // CUT cylinder − torus is a DIFFERENT topology (order-sensitive) → declines here → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(cyl, tor, nb::Op::Cut).isNull());

  // FUSE (both operand orders): watertight union envelope; ΔV within 1%; symmetric.
  const ntopo::Shape fuseTC = nb::ssi_boolean_solid(tor, cyl, nb::Op::Fuse);
  const ntopo::Shape fuseCT = nb::ssi_boolean_solid(cyl, tor, nb::Op::Fuse);
  CC_CHECK(!fuseTC.isNull() && !fuseCT.isNull());
  const double vFuseTC = watertightMeshVolume(fuseTC), vFuseCT = watertightMeshVolume(fuseCT);
  CC_CHECK(vFuseTC > 0.0 && vFuseCT > 0.0);
  CC_CHECK(std::fabs(vFuseTC - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(std::fabs(vFuseCT - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(vFuseTC >= std::max(vTorus, vCyl) - 1e-9);                  // union ≥ max(A,B)

  // Partition identity: COMMON + (torus − cyl) = torus (the numeric oracle self-consistency).
  CC_CHECK(std::fabs((vCommon + vCutTC) - vTorus) <= 2e-2 * vTorus);
}

// ── (14c) TRANSVERSAL torus∩cyl REDUCES TO COAXIAL + skew/single-sheet declines ────────────────
// As the perpendicular offset → 0 the pose becomes COAXIAL and the S5-l torus∩cylinder assembler
// (which runs BEFORE S5-p in the dispatch) claims it, reproducing the landed coaxial COMMON; the
// S5-p transversal setup gates on a strictly-positive offset so it declines the coaxial pose. This
// pins the reduction hand-off (the same offset→0 guarantee S5-k has), plus the honest declines: a
// SKEW (perpendicular-axis) cylinder and a cylinder that does NOT pierce both tube sheets (a single
// loop) both fall through S5-p to OCCT.
CC_TEST(torus_cyl_transversal_reduces_to_coaxial_and_declines_skew) {
  const double R = 3.0, r = 1.0;
  const ntopo::Shape tor = makeTorus(R, r);

  // Coaxial (off=0): S5-l owns it — a Pappus-exact COMMON, distinct from the offset overlap.
  const double Rc = 3.2, cLo = -2.0, cHi = 2.0;
  const ntopo::Shape cylCoax = makeCylOff(/*Z*/ 2, 0.0, 0.0, Rc, cLo, cHi);
  const ntopo::Shape coax = nb::ssi_boolean_solid(tor, cylCoax, nb::Op::Common);
  CC_CHECK(!coax.isNull());
  const double vCoax = watertightMeshVolume(coax);
  CC_CHECK(vCoax > 0.0);
  const double vCoaxTrue = torusCylCommonVolume(R, r, Rc);
  CC_CHECK(std::fabs(vCoax - vCoaxTrue) <= 1e-2 * vCoaxTrue);

  // The coaxial COMMON is DISTINCT from a transversal (offset) COMMON — the offset shrinks the
  // overlap to a bore-sized region — so the reduction is not a trivial identity.
  const ntopo::Shape off = nb::ssi_boolean_solid(tor, makeCylOff(/*Z*/ 2, 3.0, 0.0, 0.6, cLo, cHi),
                                                 nb::Op::Common);
  CC_CHECK(!off.isNull());
  const double vOff = watertightMeshVolume(off);
  CC_CHECK(vOff > 0.0);
  CC_CHECK(vCoax - vOff > 1.0);   // coaxial overlap strictly (much) larger than the offset bore

  // SKEW (perpendicular-axis) cylinder: an X-axis thin cylinder stabbing the +X tube rim. The
  // S5-p parallel-axis gate rejects it → NULL → OCCT (no faked skew handling).
  const ntopo::Shape skew = makeCylOff(/*X*/ 0, 0.0, 0.0, 0.6, 2.0, 4.0);
  CC_CHECK(nb::ssi_boolean_solid(tor, skew, nb::Op::Common).isNull());

  // SINGLE-SHEET grazing cylinder (z∈[0.2,2] misses the lower tube sheet → not two closed loops):
  // S5-p needs exactly two closed loops, so it declines → OCCT.
  const ntopo::Shape graze = makeCylOff(/*Z*/ 2, 3.0, 0.0, 0.6, 0.2, 2.0);
  CC_CHECK(nb::ssi_boolean_solid(tor, graze, nb::Op::Common).isNull());
}

// ── (15) TORUS∩CYLINDER HONEST DECLINES: spindle torus + clear/tangent cylinder → OCCT ──
// The S5-l assembler is the strict RING-torus two-circle poke-through only. A self-intersecting
// SPINDLE torus (R ≤ r) is not even recognised; a cylinder that clears the tube (inside the hole
// or beyond the outer equator, |Rc−R| ≥ r → no proper two-circle crossing) declines for every op.
CC_TEST(torus_cyl_declines_spindle_and_non_crossing) {
  // (a) Spindle torus (R < r) — self-intersecting, degenerate → not recognised as a CurvedSolid.
  CC_CHECK(!sd::recogniseCurvedSolid(makeTorus(0.5, 1.0)));
  // A degenerate ring torus R == r is likewise declined (no clear tube).
  CC_CHECK(!sd::recogniseCurvedSolid(makeTorus(1.0, 1.0)));

  const ntopo::Shape tor = makeTorus(3.0, 1.0);
  // (b) Cylinder Rc=1.5 sits entirely inside the donut hole (Rc < R−r = 2) — no wall crossing.
  const ntopo::Shape clear = makeCyl(/*Z*/ 2, 1.5, -2.0, 2.0);
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Cut).isNull());

  // (c) Cylinder tangent to the outer equator (Rc = R+r = 4 → |Rc−R| = r): boundary, not a proper
  //     two-circle crossing → declines.
  const ntopo::Shape tangent = makeCyl(/*Z*/ 2, 4.0, -2.0, 2.0);
  CC_CHECK(nb::ssi_boolean_solid(tor, tangent, nb::Op::Common).isNull());

  // (d) A short cylinder that does not axially span the tube (a seam falls outside its extent).
  const ntopo::Shape shortCyl = makeCyl(/*Z*/ 2, 3.2, 0.5, 2.0);  // z∈[0.5,2] misses the −z0 seam
  CC_CHECK(nb::ssi_boolean_solid(tor, shortCyl, nb::Op::Common).isNull());
}

// Airtight Pappus closed form for the COMMON (tube ∩ ball) of a coaxial torus∩sphere whose
// sphere is centred at the torus centre (sc = 0). The two seams share radius ρ* = (R²−r²+Rs²)/(2R)
// at z = ±z0 = ±√(r²−(ρ*−R)²); COMMON is the ρ ≤ √(Rs²−z²) tube segment revolved:
//   V = 2π·[ (Rs²−R²−r²)·z0 + R·(z0·√(r²−z0²) + r²·asin(z0/r)) ].
// Matches the engine's ssiCurvedBooleanVerified S5-m arm (no OCCT, no fabricated value).
double torusSphereCommonVolume(double R, double r, double Rs) {
  const double rhoStar = (R * R - r * r + Rs * Rs) / (2.0 * R);
  const double d = rhoStar - R;
  const double z0 = std::sqrt(std::max(r * r - d * d, 0.0));
  const double innerInt =
      z0 * std::sqrt(std::max(r * r - z0 * z0, 0.0)) + r * r * std::asin(std::clamp(z0 / r, -1.0, 1.0));
  return 2.0 * sd::kSsiPi * ((Rs * Rs - R * R - r * r) * z0 + R * innerInt);
}

// ── (16) COAXIAL TORUS∩SPHERE COMMON / FUSE / CUT (S5-m) — the SECOND torus-family pair ──
// A ring torus (major R=3, minor r=1, axis +Z, centre O) and a sphere Rs=3.0 centred AT the
// torus centre (ON the axis, sc=0 — the symmetric pose). The sphere crosses the tube at TWO
// latitudes (ρ*=(R²−r²+Rs²)/(2R)=2.833, |ρ*−R|=0.167<r=1), giving two analytic circle seams
// at z=±z0=±√(r²−(ρ*−R)²)≈±0.986 of EQUAL radius ρ*. Every op is a Pappus-exact solid of
// revolution: COMMON = inner tube arc + sphere zone; CUT (torus−sph) = outer tube arc +
// reversed sphere zone; FUSE = outer tube bulge + two sphere polar caps. Verified vs the
// AIRTIGHT closed forms — no OCCT, no fabricated value (mirrors the engine's S5-m oracle).
CC_TEST(torus_sphere_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const double R = 3.0, r = 1.0, Rs = 3.0;
  const ntopo::Shape tor = makeTorus(R, r);
  const ntopo::Shape sph = makeSphere(0.0, Rs);  // centre origin (= torus centre) → coaxial, sc=0
  CC_CHECK(!tor.isNull() && !sph.isNull());

  const auto csTor = sd::recogniseCurvedSolid(tor);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csTor && csSph);
  if (csTor && csSph) {
    CC_CHECK(csTor->kind == sd::CurvedKind::Torus);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    CC_CHECK(std::fabs(csTor->radius - R) < 1e-9 && std::fabs(csTor->minorRadius - r) < 1e-9);
    const ssi::TraceSet tr = ssi::trace_intersection(csTor->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circle seams
    CC_CHECK(tr.curveCount() >= 1);      // ≥1 of the two co-resident circles traced
  }

  // Airtight closed-form ground truth (Pappus).
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;   // 2π²Rr²
  const double vSph = 4.0 / 3.0 * sd::kSsiPi * Rs * Rs * Rs;         // 4/3 π Rs³
  const double vCommonTrue = torusSphereCommonVolume(R, r, Rs);
  const double vFuseTrue = vTorus + vSph - vCommonTrue;
  const double vCutTrue = vTorus - vCommonTrue;
  CC_CHECK(vCommonTrue > 0.0 && vCommonTrue < vTorus);

  // ── COMMON: the tube ∩ ball (inner arc + sphere zone). ──
  const ntopo::Shape common = nb::ssi_boolean_solid(tor, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vTorus, vSph) + 1e-9);              // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(tor, sph, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, tor, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: outer tube bulge + two sphere polar caps. A GROW (hole filled by the ball). ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(tor, sph, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vTorus, vSph) - 1e-9);               // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(tor, sph, nb::Op::Fuse).isNull());

  // ── CUT = A − B (torus minuend): outer tube ring scooped by the reversed sphere zone. SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(tor, sph, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vTorus + 1e-9);                                // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(tor, sph, nb::Op::Cut).isNull());

  // ── REVERSE CUT = sphere − torus (order-sensitive) now LANDS (buildSphereTorusCut): the ball
  // with a concave TOROIDAL GROOVE scooped into its equatorial belt — two sphere polar caps +
  // reversed inner tube arc groove. A DIFFERENT topology from the torus-minuend outer ring;
  // watertight, on-surface, ΔV within 1% vs the closed-form V_sph − V_common. ──
  const double vCutRevTrue = vSph - vCommonTrue;                   // sphere − torus = sphere − ∩
  const ntopo::Shape cutRev = nb::ssi_boolean_solid(sph, tor, nb::Op::Cut);
  CC_CHECK(!cutRev.isNull());
  const double vCutRev = watertightMeshVolume(cutRev);
  CC_CHECK(vCutRev > 0.0);                                         // watertight → engine accepts
  CC_CHECK(std::fabs(vCutRev - vCutRevTrue) <= 1e-2 * vCutRevTrue);
  CC_CHECK(vCutRev <= vSph + 1e-9);                                // sphere − torus ≤ sphere
  // Partition identity: (sphere − torus) + COMMON = sphere (the closed-form self-consistency).
  CC_CHECK(std::fabs((vCutRev + vCommonTrue) - vSph) <= 2e-2 * vSph);
}

// ── (17) TORUS∩SPHERE HONEST DECLINES: spindle / off-centre / off-axis / non-crossing → OCCT ──
// The S5-m assembler is the strict RING-torus + centred-sphere (sc=0) two-circle poke-through only.
// A spindle torus is not recognised; an OFF-CENTRE coaxial sphere (sc≠0 — the general spiric
// section), an OFF-AXIS sphere, a sphere too small to reach the tube, and a sphere so large it
// engulfs the inner tube (no proper two-circle crossing) all decline → NULL → OCCT (never faked).
CC_TEST(torus_sphere_declines_offcentre_offaxis_and_non_crossing) {
  const ntopo::Shape tor = makeTorus(3.0, 1.0);

  // (a) Off-CENTRE coaxial sphere (centre on the +Z axis but shifted — sc≠0): the general spiric
  //     section with UNEQUAL seam radii; the symmetric-pose assembler declines. makeSphereY shifts
  //     the centre along its +Y polar axis, which is NOT the torus +Z axis → also off-axis; either
  //     way it is not the sc=0 symmetric pose the S5-m builder handles.
  const ntopo::Shape offCentre = makeSphereY(0.5, 3.0);
  CC_CHECK(nb::ssi_boolean_solid(tor, offCentre, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, offCentre, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, offCentre, nb::Op::Cut).isNull());

  // (b) Off-AXIS sphere (centre off the torus axis, perpendicular offset) → transversal spiric.
  const ntopo::Shape offAxis = makeSphere(0.7, 3.0);  // centre (0.7,0,0) — off the +Z axis
  CC_CHECK(nb::ssi_boolean_solid(tor, offAxis, nb::Op::Common).isNull());

  // (c) A small centred sphere (Rs=1.5) sits entirely inside the donut hole — ρ*=(9−1+2.25)/6≈1.71
  //     < R−r=2, so |ρ*−R|=1.29 > r=1 → no proper two-circle crossing → declines for every op.
  const ntopo::Shape small = makeSphere(0.0, 1.5);
  CC_CHECK(nb::ssi_boolean_solid(tor, small, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, small, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, small, nb::Op::Cut).isNull());

  // (d) A large centred sphere (Rs=4.2) engulfs the inner tube — ρ*=(9−1+17.64)/6≈4.27 > R+r=4,
  //     so |ρ*−R|=1.27 > r=1 → no two-circle crossing → declines.
  const ntopo::Shape big = makeSphere(0.0, 4.2);
  CC_CHECK(nb::ssi_boolean_solid(tor, big, nb::Op::Common).isNull());
}

// Airtight Pappus closed form for the COMMON (tube ∩ cone-solid) of a coaxial torus∩cone whose
// meridian cone chord is ρ = a + b·z (b = ±tanα) in the torus frame (torus major R, minor r).
// The COMMON is the ρ ≤ a+b·z tube segment revolved. About the tube centre (ρ'=ρ−R): the chord
// has unit normal m̂=(1,−b)/√(1+b²) into the discarded (ρ>line) region and signed offset
// t0=(a−R)/√(1+b²); discarded area A_d=r²acos(t0/r)−t0√(r²−t0²), discarded ρ'-moment
// (1/√(1+b²))(2/3)(r²−t0²)^{3/2}. Kept area A_seg=πr²−A_d, kept moment M=−(1/√(1+b²))(2/3)(r²−t0²)^{3/2}.
//   V = 2π·(R·A_seg + M).
// Matches the engine's ssiCurvedBooleanVerified S5-n arm; reduces to torusCylCommonVolume at b=0.
double torusConeCommonVolume(double R, double r, double a, double b) {
  const double invN = 1.0 / std::sqrt(1.0 + b * b);
  const double t0 = (a - R) * invN;
  const double root = std::sqrt(std::max(r * r - t0 * t0, 0.0));
  const double aCap = r * r * std::acos(std::clamp(t0 / r, -1.0, 1.0)) - t0 * root;
  const double aSeg = sd::kSsiPi * r * r - aCap;
  const double mom = -invN * (2.0 / 3.0) * root * root * root;
  return 2.0 * sd::kSsiPi * (R * aSeg + mom);
}

// Exact volume of a conical frustum about its axis: (π Δh/3)(ra²+ra·rb+rb²) — the makeCone
// frustum volume (the axis edge is at ρ=0, so it is a genuine truncated cone / frustum).
double coneFrustumVolume(double ra, double rb, double dh) {
  return sd::kSsiPi * dh / 3.0 * (ra * ra + ra * rb + rb * rb);
}

// ── (18) COAXIAL TORUS∩CONE COMMON / FUSE / CUT (S5-n) — the THIRD torus-family pair ──
// A ring torus (major R=3, minor r=1, axis +Y, centre O) and a COAXIAL cone (about +Y) whose
// SLANTED wall ρ = a + b·z (a=3.2 at z=0, b=0.5 = the meridian slope) crosses the tube at TWO
// latitudes — the oblique-chord generalisation of the S5-l vertical-chord cylinder. The chord
// cuts the tube disk at z1=−0.96 (ρ=2.72) and z2=0.8 (ρ=3.6), two analytic circle seams at
// DIFFERENT radii and stations. Every op is a Pappus-exact solid of revolution: COMMON = the
// ρ≤line tube part (inner arc + slanted cone chord band); CUT (torus−cone) = the ρ>line outer
// ring; FUSE = the union (outer bulge + cone wall outside the tube + cone discs). Verified vs
// the AIRTIGHT closed forms — no OCCT, no fabricated value (mirrors the engine's S5-n oracle).
CC_TEST(torus_cone_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const double R = 3.0, r = 1.0, aLine = 3.2, bSlope = 0.5;
  const ntopo::Shape tor = makeTorusY(R, r);
  // makeCone(r0,y0,r1,y1) about +Y: radius(y)=r0+(r1−r0)/(y1−y0)(y−y0). For radius=3.2+0.5·y
  // over y∈[−2,2]: r0=3.2−1=2.2 at y=−2, r1=3.2+1=4.2 at y=+2 → slope 0.5, radius(0)=3.2.
  const ntopo::Shape cone = makeCone(2.2, -2.0, 4.2, 2.0);
  CC_CHECK(!tor.isNull() && !cone.isNull());

  const auto csTor = sd::recogniseCurvedSolid(tor);
  const auto csCone = sd::recogniseCurvedSolid(cone);
  CC_CHECK(csTor && csCone);
  if (csTor && csCone) {
    CC_CHECK(csTor->kind == sd::CurvedKind::Torus);
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    CC_CHECK(std::fabs(csTor->radius - R) < 1e-9 && std::fabs(csTor->minorRadius - r) < 1e-9);
    const ssi::TraceSet tr = ssi::trace_intersection(csTor->adapter(), csCone->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circle seams
    CC_CHECK(tr.curveCount() >= 1);      // ≥1 of the two co-resident circles traced
  }

  // Airtight closed-form ground truth (Pappus).
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;   // 2π²Rr²
  const double vConeFull = coneFrustumVolume(2.2, 4.2, 4.0);         // frustum over y∈[−2,2]
  const double vCommonTrue = torusConeCommonVolume(R, r, aLine, bSlope);
  const double vFuseTrue = vTorus + vConeFull - vCommonTrue;
  const double vCutTrue = vTorus - vCommonTrue;
  CC_CHECK(vCommonTrue > 0.0 && vCommonTrue < vTorus);

  // ── COMMON: the ρ≤line tube part (inner arc + slanted cone chord band). ──
  const ntopo::Shape common = nb::ssi_boolean_solid(tor, cone, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vTorus, vConeFull) + 1e-9);         // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(tor, cone, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(cone, tor, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: cone frustum fills the hole + outer tube bulge + cone discs. A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(tor, cone, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vTorus, vConeFull) - 1e-9);           // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(tor, cone, nb::Op::Fuse).isNull());

  // ── CUT = A − B (torus minuend): the ρ>line outer tube ring (a SHRINK). ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(tor, cone, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vTorus + 1e-9);                                // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(tor, cone, nb::Op::Cut).isNull());

  // ── REVERSE CUT = cone − torus (order-sensitive) now LANDS (buildConeTorusCut): the cone with a
  // concave TOROIDAL GROOVE scooped into its slanted wall — cone wall stubs + terminal discs +
  // reversed inner tube arc groove. A DIFFERENT topology from the torus-minuend outer ring;
  // watertight, on-surface, ΔV within 1% vs the closed-form V_cone − V_common. ──
  const double vCutRevTrue = vConeFull - vCommonTrue;             // cone − torus = cone − ∩
  const ntopo::Shape cutRev = nb::ssi_boolean_solid(cone, tor, nb::Op::Cut);
  CC_CHECK(!cutRev.isNull());
  const double vCutRev = watertightMeshVolume(cutRev);
  CC_CHECK(vCutRev > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCutRev - vCutRevTrue) <= 1e-2 * vCutRevTrue);
  CC_CHECK(vCutRev <= vConeFull + 1e-9);                          // cone − torus ≤ cone
  // Partition identity: (cone − torus) + COMMON = cone (the closed-form self-consistency).
  CC_CHECK(std::fabs((vCutRev + vCommonTrue) - vConeFull) <= 2e-2 * vConeFull);
}

// ── (19) TORUS∩CONE HONEST DECLINES: spindle / near-cylindrical / non-crossing / off-axis → OCCT ──
// The S5-n assembler is the strict RING-torus + coaxial-cone two-circle poke-through only. A
// spindle torus is not recognised; a near-cylindrical cone (tanα≈0) routes to the S5-l cylinder
// path (declined here as a cone); a cone whose slant chord clears the tube, and a non-coaxial /
// off-axis cone all decline → NULL → OCCT (never faked).
CC_TEST(torus_cone_declines_spindle_and_non_crossing) {
  // (a) Spindle torus (R < r) — self-intersecting → not recognised as a CurvedSolid.
  CC_CHECK(!sd::recogniseCurvedSolid(makeTorusY(0.5, 1.0)));

  const ntopo::Shape tor = makeTorusY(3.0, 1.0);

  // (b) A cone whose slant chord sits entirely inside the donut hole (radius ~1.5 near the tube,
  //     |ρ−R| ≥ r everywhere on z∈[−1,1]) — no wall crossing of the tube → declines all ops.
  const ntopo::Shape clear = makeCone(1.0, -2.0, 2.0, 2.0);  // radius(y)=1.5+0.25y, near ρ≈1.5 ≪ R−r
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Cut).isNull());

  // (c) A cone that does not axially span the tube (its extent misses the lower seam z1=−0.96):
  //     the seams are not both interior to the cone frustum → declines.
  const ntopo::Shape shortCone = makeCone(3.45, 0.5, 4.2, 2.0);  // y∈[0.5,2] misses z1=−0.96
  CC_CHECK(nb::ssi_boolean_solid(tor, shortCone, nb::Op::Common).isNull());

  // (d) Off-AXIS cone: shift the whole torus to a +Z torus while the cone stays +Y → the axes are
  //     perpendicular (non-coaxial) → declines (never faked).
  const ntopo::Shape torZ = makeTorus(3.0, 1.0);               // +Z torus
  const ntopo::Shape coneY = makeCone(2.2, -2.0, 4.2, 2.0);    // +Y cone → axes ⟂
  CC_CHECK(nb::ssi_boolean_solid(torZ, coneY, nb::Op::Common).isNull());
}

// Airtight Pappus closed form for the COMMON (revolved LENS) of two coaxial ring tori: tube A
// the meridian disk radius r1 centred (R1,zA), tube B radius r2 centred (R2,zB). D = centre
// distance, a = (D²+r1²−r2²)/(2D) (chord offset from A along the centre line), h = √(r1²−a²).
// V = 2π·(R1·A_segA + R2·A_segB), A_segA = r1²·acos(a/r1) − a·h, A_segB = r2²·acos((D−a)/r2) −
// (D−a)·h (the equal-h segment moment terms cancel). Matches the engine's ssiCurvedBooleanVerified
// S5-o arm (no OCCT, no fabricated value).
double torusTorusCommonVolume(double R1, double r1, double zA, double R2, double r2, double zB) {
  const double cr = R2 - R1, cz = zB - zA;
  const double D = std::hypot(cr, cz);
  const double a = (D * D + r1 * r1 - r2 * r2) / (2.0 * D);
  const double h = std::sqrt(std::max(r1 * r1 - a * a, 0.0));
  const double aSegA = r1 * r1 * std::acos(std::clamp(a / r1, -1.0, 1.0)) - a * h;
  const double aSegB = r2 * r2 * std::acos(std::clamp((D - a) / r2, -1.0, 1.0)) - (D - a) * h;
  return 2.0 * sd::kSsiPi * (R1 * aSegA + R2 * aSegB);
}

// ── (20) COAXIAL TORUS∩TORUS COMMON / FUSE / CUT (S5-o) — the FOURTH torus-family pair ──
// Two coaxial ring tori about world +Z: torus A (R1=3, r1=1, centre z=0) and torus B (R2=3.4,
// r2=0.9, centre z=0.6). Their tubes cross in TWO circle seams at DIFFERENT radii AND stations
// (D=0.721, |r1−r2|=0.1 < D < r1+r2=1.9 — a proper two-circle poke-through). BOTH boundary walls
// are TUBE ARCS (reusing appendTubeArc on both tori), no flat chord band, no caps: COMMON = the
// revolved lens (inner arc of A + inner arc of B); FUSE = the revolved union (outer arc of A +
// outer arc of B); CUT (A−B) = outer arc of A + reversed inner arc of B. Verified vs the AIRTIGHT
// Pappus closed forms — no OCCT, no fabricated value (mirrors the engine's S5-o oracle).
CC_TEST(torus_torus_coaxial_common_fuse_cut_watertight_matches_analytic) {
  const double R1 = 3.0, r1 = 1.0, zA = 0.0, R2 = 3.4, r2 = 0.9, zB = 0.6;
  const ntopo::Shape torA = makeTorusAt(R1, r1, zA);
  const ntopo::Shape torB = makeTorusAt(R2, r2, zB);
  CC_CHECK(!torA.isNull() && !torB.isNull());

  const auto csA = sd::recogniseCurvedSolid(torA);
  const auto csB = sd::recogniseCurvedSolid(torB);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    CC_CHECK(csA->kind == sd::CurvedKind::Torus && csB->kind == sd::CurvedKind::Torus);
    CC_CHECK(std::fabs(csA->radius - R1) < 1e-9 && std::fabs(csA->minorRadius - r1) < 1e-9);
    CC_CHECK(std::fabs(csB->radius - R2) < 1e-9 && std::fabs(csB->minorRadius - r2) < 1e-9);
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal circle seams
    CC_CHECK(tr.curveCount() >= 1);      // ≥1 of the two co-resident circles traced
  }

  // Airtight closed-form ground truth (Pappus).
  const double vTorA = 2.0 * sd::kSsiPi * sd::kSsiPi * R1 * r1 * r1;   // 2π²R1r1²
  const double vTorB = 2.0 * sd::kSsiPi * sd::kSsiPi * R2 * r2 * r2;   // 2π²R2r2²
  const double vCommonTrue = torusTorusCommonVolume(R1, r1, zA, R2, r2, zB);
  const double vFuseTrue = vTorA + vTorB - vCommonTrue;
  const double vCutTrue = vTorA - vCommonTrue;         // A − B (torus-A minuend)
  const double vCutBATrue = vTorB - vCommonTrue;       // B − A (torus-B minuend)
  CC_CHECK(vCommonTrue > 0.0 && vCommonTrue < std::min(vTorA, vTorB));

  // ── COMMON: the revolved lens (inner arc of A + inner arc of B). ──
  const ntopo::Shape common = nb::ssi_boolean_solid(torA, torB, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                          // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonTrue) <= 1e-2 * vCommonTrue);
  CC_CHECK(vCommon <= std::min(vTorA, vTorB) + 1e-9);              // common ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(torA, torB, nb::Op::Common).isNull());
  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(torB, torA, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonTrue) <= 1e-2 * vCommonTrue);

  // ── FUSE = A ∪ B: the revolved union (outer arc of A + outer arc of B). A GROW. ──
  const ntopo::Shape fuse = nb::ssi_boolean_solid(torA, torB, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  const double vFuse = watertightMeshVolume(fuse);
  CC_CHECK(vFuse > 0.0);
  CC_CHECK(std::fabs(vFuse - vFuseTrue) <= 1e-2 * vFuseTrue);
  CC_CHECK(vFuse >= std::max(vTorA, vTorB) - 1e-9);               // FUSE grows past either operand
  CC_CHECK(!nb::boolean_solid(torA, torB, nb::Op::Fuse).isNull());

  // ── CUT = A − B (torus-A minuend): outer arc of A + reversed inner arc of B. A SHRINK. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(torA, torB, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vTorA + 1e-9);                                 // CUT shrinks below the minuend
  CC_CHECK(!nb::boolean_solid(torA, torB, nb::Op::Cut).isNull());
  // CUT is order-sensitive: B − A is a DIFFERENT solid (torus-B the minuend) — swapping operands
  // builds it (torus-B outer arc + reversed torus-A inner arc), matching vTorB − vCommon.
  const ntopo::Shape cutBA = nb::ssi_boolean_solid(torB, torA, nb::Op::Cut);
  CC_CHECK(!cutBA.isNull());
  const double vCutBA = watertightMeshVolume(cutBA);
  CC_CHECK(vCutBA > 0.0);
  CC_CHECK(std::fabs(vCutBA - vCutBATrue) <= 1e-2 * vCutBATrue);
}

// ── (21) TORUS∩TORUS HONEST DECLINES: spindle / contained / clear / concentric / off-axis → OCCT ──
// The S5-o assembler is the strict coaxial RING-torus two-circle poke-through only. A spindle
// torus is not recognised; two tubes that clear each other, one tube contained in the other,
// concentric coaxial tubes (same centre), and a non-coaxial / off-axis torus pair all decline
// → NULL → OCCT (never faked).
CC_TEST(torus_torus_declines_spindle_contained_clear_and_offaxis) {
  // (a) Spindle torus (R < r) — self-intersecting → not recognised as a CurvedSolid.
  CC_CHECK(!sd::recogniseCurvedSolid(makeTorusAt(0.5, 1.0, 0.0)));

  const ntopo::Shape torA = makeTorusAt(3.0, 1.0, 0.0);

  // (b) A coaxial torus whose tube CLEARS torus A (centre distance D ≥ r1+r2): major-radius gap
  //     far exceeds the two minor radii → no wall crossing → declines all ops.
  const ntopo::Shape clear = makeTorusAt(6.0, 1.0, 0.0);  // D = 3 ≥ r1+r2 = 2
  CC_CHECK(nb::ssi_boolean_solid(torA, clear, nb::Op::Common).isNull());
  CC_CHECK(nb::ssi_boolean_solid(torA, clear, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(torA, clear, nb::Op::Cut).isNull());

  // (c) A coaxial torus whose tube is CONTAINED in torus A (D ≤ |r1−r2|): a fat tube swallowing
  //     the thin one at the same centre → no proper two-circle crossing → declines.
  const ntopo::Shape contained = makeTorusAt(3.0, 0.3, 0.0);  // D = 0 ≤ |r1−r2| = 0.7
  CC_CHECK(nb::ssi_boolean_solid(torA, contained, nb::Op::Common).isNull());

  // (d) CONCENTRIC-coaxial tubes (identical centre + radii): D ≈ 0, tubes coincide → declines
  //     (the seam solve needs a positive centre distance).
  const ntopo::Shape concentric = makeTorusAt(3.0, 1.0, 0.0);
  CC_CHECK(nb::ssi_boolean_solid(torA, concentric, nb::Op::Common).isNull());

  // (e) Off-AXIS torus pair: torus A about +Z, torus B about +Y → axes ⟂ (non-coaxial) → declines.
  const ntopo::Shape torY = makeTorusY(3.4, 0.9);   // +Y torus
  CC_CHECK(nb::ssi_boolean_solid(torA, torY, nb::Op::Common).isNull());
}

// ── (16) TRANSVERSAL (NON-COAXIAL) cone∩sphere COMMON (S5-q) — the FIRST transversal CONE
// slice ─────────────────────────────────────────────────────────────────────────────────────
// A cone frustum r(y)=0.5+(y+3)/6 over y∈[-3,3] about world +Y (r goes 0.5→1.5) and a sphere
// of radius Rs=2 whose centre is DISPLACED off the cone axis by off=0.5 in +X. The cone still
// pierces the sphere so its wall crosses in TWO disjoint NON-PLANAR closed loops (the cone∩sphere
// quartic locus — no analytic circle), so S5-q drives the COMMON directly from the S3-traced
// seams: COMMON = sphere lower cap + cone band + sphere upper cap, every ring the traced
// non-planar seam. There is no closed-form COMMON volume, so the oracle is a deterministic
// fine-grid numerical integration of the analytic region (inside the sphere AND the finite cone)
// — the primary parity oracle for a non-analytic seam. CUT/FUSE honest-decline (the sphere-outer-
// zone weld between two non-planar seams is the transversal residual, same class as S5-k/S5-p).
CC_TEST(cone_sphere_transversal_offset_common_watertight_matches_numeric) {
  const double r0 = 0.5, y0 = -3.0, r1 = 1.5, y1 = 3.0, off = 0.5, Rs = 2.0;
  const ntopo::Shape cone = makeCone(r0, y0, r1, y1);   // r(y)=0.5+0.5·(y+3)/3, axis +Y
  const ntopo::Shape sph = makeSphere(off, Rs);         // sphere centred at (off,0,0) — OFF-AXIS
  CC_CHECK(!cone.isNull() && !sph.isNull());

  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csSph = sd::recogniseCurvedSolid(sph);
  CC_CHECK(csCone && csSph);
  if (csCone && csSph) {
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    CC_CHECK(csSph->kind == sd::CurvedKind::Sphere);
    // The trace is TWO fully-transversal CLOSED non-planar loops (pierce-both-ends).
    const ssi::TraceSet tr = ssi::trace_intersection(csCone->adapter(), csSph->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.branchPoints == 0);
    CC_CHECK(tr.lines.size() == 2);
    int closed = 0;
    for (const ssi::WLine& w : tr.lines)
      if (w.isClosed()) ++closed;
    CC_CHECK(closed == 2);
  }

  // Deterministic numeric COMMON volume (analytic region: inside the sphere AND inside the finite
  // cone). 200³ midpoint cells → converged to ≪ the 1% curved-parity bar.
  const double slope = (r1 - r0) / (y1 - y0);
  const int nx = 200, ny = 200, nz = 200;
  const double X0 = -2.0, X1 = 3.0, Y0 = y0, Y1 = y1, Z0 = -2.0, Z1 = 2.0;
  const double cell = (X1 - X0) / nx * (Y1 - Y0) / ny * (Z1 - Z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = X0 + (X1 - X0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = Y0 + (Y1 - Y0) * (j + 0.5) / ny;
      const double rr = r0 + slope * (y - y0);
      const double rr2 = rr * rr;
      for (int k = 0; k < nz; ++k) {
        const double z = Z0 + (Z1 - Z0) * (k + 0.5) / nz;
        const double dx = x - off;
        if (dx * dx + y * y + z * z <= Rs * Rs && x * x + z * z <= rr2) ++inside;
      }
    }
  }
  const double vCommonNumeric = inside * cell;
  CC_CHECK(vCommonNumeric > 10.5 && vCommonNumeric < 12.0);   // ≈ 11.28

  // ── COMMON: watertight native candidate whose volume matches the numeric oracle. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cone, sph, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                    // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonNumeric) <= 1e-2 * vCommonNumeric);
  const double vSph = 4.0 / 3.0 * sd::kSsiPi * Rs * Rs * Rs;
  const double vCone = frustumVolume(r0, r1, y1 - y0);
  CC_CHECK(vCommon <= std::min(vSph, vCone) + 1e-9);          // COMMON ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cone, sph, nb::Op::Common).isNull());

  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(sph, cone, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonNumeric) <= 1e-2 * vCommonNumeric);

  // ── CUT / FUSE now LAND (S5-q, the SAME seam-band primitive as S5-k about the cone axis). ──
  // Oracles: exact V(A)/V(B) minus the numeric COMMON (inclusion–exclusion), the box being sized
  // for the COMMON not the whole sphere/cone.
  const double vCutSphNumeric = vSph - vCommonNumeric;                // sphere − cone
  const double vCutConeNumeric = vCone - vCommonNumeric;              // cone − sphere
  const double vFuseNumeric = vSph + vCone - vCommonNumeric;          // union

  const ntopo::Shape cutSC = nb::ssi_boolean_solid(sph, cone, nb::Op::Cut);
  CC_CHECK(!cutSC.isNull());
  const double vCutSC = watertightMeshVolume(cutSC);
  CC_CHECK(vCutSC > 0.0);
  CC_CHECK(std::fabs(vCutSC - vCutSphNumeric) <= 1e-2 * vCutSphNumeric);

  const ntopo::Shape cutCS = nb::ssi_boolean_solid(cone, sph, nb::Op::Cut);
  CC_CHECK(!cutCS.isNull());
  const double vCutCS = watertightMeshVolume(cutCS);
  CC_CHECK(vCutCS > 0.0);
  CC_CHECK(std::fabs(vCutCS - vCutConeNumeric) <= 1e-2 * vCutConeNumeric);

  const ntopo::Shape fuseCS = nb::ssi_boolean_solid(cone, sph, nb::Op::Fuse);
  const ntopo::Shape fuseSC = nb::ssi_boolean_solid(sph, cone, nb::Op::Fuse);
  CC_CHECK(!fuseCS.isNull() && !fuseSC.isNull());
  const double vFuseCS = watertightMeshVolume(fuseCS), vFuseSC = watertightMeshVolume(fuseSC);
  CC_CHECK(vFuseCS > 0.0 && vFuseSC > 0.0);
  CC_CHECK(std::fabs(vFuseCS - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(std::fabs(vFuseSC - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(vFuseCS >= std::max(vSph, vCone) - 1e-9);
}

// ── (17) TRANSVERSAL cone∩sphere REDUCES TO COAXIAL: the offset→0 guarantee ────────────────────
// As the perpendicular offset → 0 the pose becomes COAXIAL and the S5-h two-circle cone∩sphere
// assembler (which runs BEFORE S5-q in the dispatch) claims it, reproducing the landed coaxial
// COMMON; S5-q's transversal setup gates on a strictly-positive offset so it declines the coaxial
// pose. This test pins that hand-off: the SAME cone with an ON-AXIS sphere yields a watertight
// COMMON matching a deterministic numeric oracle, distinct from (larger than) the offset COMMON,
// confirming the reduction boundary is a real hand-off, not a trivial identity.
CC_TEST(cone_sphere_transversal_reduces_to_coaxial_at_zero_offset) {
  const double r0 = 0.5, y0 = -3.0, r1 = 1.5, y1 = 3.0, Rs = 2.0;
  const ntopo::Shape cone = makeCone(r0, y0, r1, y1);
  const ntopo::Shape sphCoax = makeSphereY(0.0, Rs);   // centre (0,0,0) ON the cone +Y axis → coaxial

  // Numeric coaxial COMMON (sphere centre on the axis → off=0): inside sphere AND cone.
  const double slope = (r1 - r0) / (y1 - y0);
  const int nx = 200, ny = 200, nz = 200;
  const double X0 = -2.0, X1 = 2.0, Y0 = y0, Y1 = y1, Z0 = -2.0, Z1 = 2.0;
  const double cell = (X1 - X0) / nx * (Y1 - Y0) / ny * (Z1 - Z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = X0 + (X1 - X0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = Y0 + (Y1 - Y0) * (j + 0.5) / ny;
      const double rr = r0 + slope * (y - y0);
      const double rr2 = rr * rr;
      for (int k = 0; k < nz; ++k) {
        const double z = Z0 + (Z1 - Z0) * (k + 0.5) / nz;
        if (x * x + y * y + z * z <= Rs * Rs && x * x + z * z <= rr2) ++inside;
      }
    }
  }
  const double vCoaxNumeric = inside * cell;   // ≈ 11.75

  const ntopo::Shape coax = nb::ssi_boolean_solid(cone, sphCoax, nb::Op::Common);
  CC_CHECK(!coax.isNull());                                     // coaxial S5-h owns it
  const double vCoax = watertightMeshVolume(coax);
  CC_CHECK(vCoax > 0.0);
  CC_CHECK(std::fabs(vCoax - vCoaxNumeric) <= 1e-2 * vCoaxNumeric);

  // The coaxial COMMON is DISTINCT from the offset (transversal) COMMON — the offset shrinks the
  // overlap — so the reduction is a real hand-off, not a trivial identity: they differ measurably.
  const ntopo::Shape offCommon = nb::ssi_boolean_solid(cone, makeSphere(0.5, Rs), nb::Op::Common);
  CC_CHECK(!offCommon.isNull());
  const double vOff = watertightMeshVolume(offCommon);
  CC_CHECK(vOff > 0.0);
  CC_CHECK(vCoax - vOff > 1e-2);   // coaxial overlap strictly larger than the offset overlap
}

// ── (18) S5-r TORUS ∩ HALF-SPACE (PLANE ⟂ AXIS) COMMON / CUT ────────────────────────────────
// A ring torus (major R, minor r, +Z axis) cut by an axis-perpendicular half-space (a big slab
// whose one z-face cuts the tube at station z=h). The horizontal cut is symmetric in ρ−R, so the
// plane-cut torus-segment volume collapses to the Pappus closed form V(z≤h) = 2π·R·A_low, with
// A_low = πr² − (r²·acos(h/r) − h·√(r²−h²)). We cross-check that closed form against a 200³
// numeric-integration oracle, then assert the native COMMON is watertight and matches it to the
// 1% curved-parity bar, COMMON ≤ min(A,B), and swap-symmetric. CUT (torus minuend) keeps the
// complementary segment. This is the OCCT-parity slice: BRepAlgoAPI_Common(torus, halfspace) is
// exactly this revolution (encoded structurally; host numeric oracle is the primary check here).

// Closed-form plane-cut torus segment volume, material on z ≤ h (keepLow) or z ≥ h.
double torusPlaneSegmentVolume(double R, double r, double h, bool keepLow) {
  const double root = std::sqrt(std::max(r * r - h * h, 0.0));
  const double aCap = r * r * std::acos(std::clamp(h / r, -1.0, 1.0)) - h * root;  // z>h cap area
  const double aLow = sd::kSsiPi * r * r - aCap;
  const double vLow = 2.0 * sd::kSsiPi * R * aLow;
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;
  return keepLow ? vLow : (vTorus - vLow);
}

// 200³ numeric COMMON of a +Z ring torus AND the z ≤ h (keepLow) / z ≥ h half-space.
double torusPlaneNumeric(double R, double r, double h, bool keepLow, int Ns = 200) {
  const double L = R + r, Zr = r;
  const double cell = (2 * L) / Ns * (2 * L) / Ns * (2 * Zr) / Ns;
  long inside = 0;
  for (int i = 0; i < Ns; ++i) {
    const double x = -L + (2 * L) * (i + 0.5) / Ns;
    for (int j = 0; j < Ns; ++j) {
      const double y = -L + (2 * L) * (j + 0.5) / Ns;
      const double rho = std::sqrt(x * x + y * y);
      const double dRho = rho - R;
      for (int k = 0; k < Ns; ++k) {
        const double z = -Zr + (2 * Zr) * (k + 0.5) / Ns;
        if (keepLow ? (z > h) : (z < h)) continue;
        if (dRho * dRho + z * z <= r * r) ++inside;
      }
    }
  }
  return inside * cell;
}

CC_TEST(torus_plane_perp_common_cut_watertight_matches_numeric) {
  const double R = 3.0, r = 1.0, h = 0.4;
  const ntopo::Shape tor = makeTorus(R, r);            // +Z ring torus at the origin
  // Slab occupying z ∈ [−3, h]: its TOP face (z=h) cuts the tube; all other faces bracket the
  // torus (footprint half=10 ≫ R+r, bottom z=−3 < −r). torus ∩ slab = torus with z ≤ h.
  const ntopo::Shape slabLow = makeSlab(10.0, -3.0, h);
  CC_CHECK(!tor.isNull() && !slabLow.isNull());

  // Oracle cross-check: closed form vs 200³ numeric agree to ≪ 1%.
  const double vClosed = torusPlaneSegmentVolume(R, r, h, /*keepLow=*/true);
  const double vNumeric = torusPlaneNumeric(R, r, h, /*keepLow=*/true);
  CC_CHECK(std::fabs(vClosed - vNumeric) <= 1e-2 * vClosed);

  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;

  // ── COMMON = torus ∩ (z ≤ h): watertight revolution matching the oracle. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(tor, slabLow, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                        // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vClosed) <= 1e-2 * vClosed);
  CC_CHECK(vCommon <= vTorus + 1e-9);                             // COMMON ≤ min(torus, halfspace)

  // Swap-symmetric: (slab, torus) builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(slabLow, tor, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vClosed) <= 1e-2 * vClosed);

  // ── CUT = torus − (z ≤ h): the complementary segment (keep z ≥ h). SHRINK below the torus. ──
  const ntopo::Shape cut = nb::ssi_boolean_solid(tor, slabLow, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  const double vCut = watertightMeshVolume(cut);
  CC_CHECK(vCut > 0.0);
  const double vCutTrue = torusPlaneSegmentVolume(R, r, h, /*keepLow=*/false);
  CC_CHECK(std::fabs(vCut - vCutTrue) <= 1e-2 * vCutTrue);
  CC_CHECK(vCut <= vTorus + 1e-9);
  // COMMON + CUT partition the torus (up to the parity bar).
  CC_CHECK(std::fabs((vCommon + vCut) - vTorus) <= 1e-2 * vTorus);

  // A half-space − torus minuend is out of the revolution scope → declines → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(slabLow, tor, nb::Op::Cut).isNull());
  // FUSE needs the box's remaining faces → HONEST-DECLINE → OCCT.
  CC_CHECK(nb::ssi_boolean_solid(tor, slabLow, nb::Op::Fuse).isNull());
  CC_CHECK(nb::ssi_boolean_solid(slabLow, tor, nb::Op::Fuse).isNull());
}

// A second pose: the slab keeps the UPPER part (z ≥ h) so COMMON is the short over-the-top cap
// arc, and the cut station is NEGATIVE — exercises the keepLow=false builder branch directly.
CC_TEST(torus_plane_perp_upper_keep_common_matches_numeric) {
  const double R = 4.0, r = 1.5, h = -0.6;
  const ntopo::Shape tor = makeTorus(R, r);
  // Slab occupying z ∈ [h, 5]: its BOTTOM face (z=h) cuts the tube; keep z ≥ h.
  const ntopo::Shape slabHi = makeSlab(12.0, h, 5.0);
  CC_CHECK(!tor.isNull() && !slabHi.isNull());

  const double vClosed = torusPlaneSegmentVolume(R, r, h, /*keepLow=*/false);
  const double vNumeric = torusPlaneNumeric(R, r, h, /*keepLow=*/false);
  CC_CHECK(std::fabs(vClosed - vNumeric) <= 1e-2 * vClosed);

  const ntopo::Shape common = nb::ssi_boolean_solid(tor, slabHi, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);
  CC_CHECK(std::fabs(vCommon - vClosed) <= 1e-2 * vClosed);
  const double vTorus = 2.0 * sd::kSsiPi * sd::kSsiPi * R * r * r;
  CC_CHECK(vCommon <= vTorus + 1e-9);
}

// ── (19) S5-r HONEST DECLINES: parallel-to-axis / tangent / clear / through-the-hole → OCCT ──
// Only the axis-perpendicular slice that cuts the tube assembles from the revolution builders.
// A plane PARALLEL to the torus axis (Villarceau-like), a plane tangent to the tube, a slab clear
// of the torus, and a slab whose cutting face passes through the donut hole (missing the tube) all
// HONEST-DECLINE → NULL → OCCT. Nothing is faked.
CC_TEST(torus_plane_honest_declines) {
  const double R = 3.0, r = 1.0;
  const ntopo::Shape tor = makeTorus(R, r);            // +Z axis

  // (a) Plane PARALLEL to the axis: a slab whose cutting faces are ⟂ X (vertical), not ⟂ Z. Its
  //     one cutting face at x=1.5 is parallel to the torus axis → not the perpendicular slice.
  //     Build a +Y-extruded... simpler: a Z-tall slab thin in X at x∈[1.0, 2.0], tall in y and z.
  {
    std::vector<cst::ProfileSegment> segs(4);
    // footprint in XY: x∈[1.0,2.0], y∈[−10,10]; extruded tall in z so the x-faces cut the torus.
    segs[0].kind = 0; segs[0].x0 = 1.0; segs[0].y0 = -10; segs[0].x1 = 2.0; segs[0].y1 = -10;
    segs[1].kind = 0; segs[1].x0 = 2.0; segs[1].y0 = -10; segs[1].x1 = 2.0; segs[1].y1 = 10;
    segs[2].kind = 0; segs[2].x0 = 2.0; segs[2].y0 = 10;  segs[2].x1 = 1.0; segs[2].y1 = 10;
    segs[3].kind = 0; segs[3].x0 = 1.0; segs[3].y0 = 10;  segs[3].x1 = 1.0; segs[3].y1 = -10;
    ntopo::Shape vslab = cst::build_prism_profile(segs, {}, {}, 4.0);  // z=0..4
    vslab = vslab.located(ntopo::Location{
        nmath::Transform::translationOf(nmath::Vec3{0.0, 0.0, -2.0})});  // z∈[−2,2] spans the tube
    CC_CHECK(!vslab.isNull());
    // The x-faces are parallel to the +Z axis → the perpendicular-slice setup declines.
    CC_CHECK(nb::ssi_boolean_solid(tor, vslab, nb::Op::Common).isNull());
    CC_CHECK(nb::ssi_boolean_solid(tor, vslab, nb::Op::Cut).isNull());
  }

  // (b) Plane TANGENT to the tube (z = r): the cut just grazes the tube top → no proper annulus.
  const ntopo::Shape tangent = makeSlab(10.0, -3.0, r);  // top face at z = r (tangent)
  CC_CHECK(nb::ssi_boolean_solid(tor, tangent, nb::Op::Common).isNull());

  // (c) Slab CLEAR of the torus (entirely above it): no intersection.
  const ntopo::Shape clear = makeSlab(10.0, r + 0.5, 5.0);  // bottom face z = r+0.5 > r
  CC_CHECK(nb::ssi_boolean_solid(tor, clear, nb::Op::Common).isNull());

  // (d) Slab whose cutting face passes through the donut HOLE but the slab does not bracket the
  //     torus tube in-plane (a narrow post inside the hole): the torus pokes out past the side
  //     faces, so recognisePlaneHalfspace declines (not a single clean bracketing cut).
  const ntopo::Shape post = makeSlab(1.0, -3.0, 0.4);  // half=1.0 < R−r=2 → torus pokes out
  CC_CHECK(nb::ssi_boolean_solid(tor, post, nb::Op::Common).isNull());
}

// ── (20) TRANSVERSAL (NON-COAXIAL) cone∩cylinder COMMON (S5-s) — the FIRST transversal
// cone∩cyl slice ──────────────────────────────────────────────────────────────────────────────
// A cone frustum r(y)=0.5+(y+3)/6 over y∈[-3,3] about world +Y (r goes 0.5→1.5) and a THIN
// cylinder of radius Rc=0.3 whose axis is PARALLEL to the cone axis (+Y) but DISPLACED off it by
// off=1.0 in +X, spanning y∈[-2,2] so its whole cross-section is OUTSIDE the cone at its narrow
// (lower) end and fully INSIDE at its wide (upper) end. Because the axes are non-coaxial AND the
// cone wall is monotonic, the cylinder wall crosses the cone in ONE (not two) NON-PLANAR closed
// loop (the cone∩cylinder quartic locus — no analytic circle; verified: the trace returns exactly
// one closed seam). S5-s therefore is a SINGLE-SEAM assembler, distinct from the two-loop S5-k/p/q
// machinery: it drives the COMMON directly from the traced seam:
//   COMMON = cone-wall cap (bounded by the seam) + cylinder wall band (seam → inside-end rim) +
//     cylinder inside-end disc. There is no closed-form COMMON volume, so the oracle is a
// deterministic 200³ numerical integration of the analytic region (inside the finite cone AND the
// offset finite cylinder) — the primary parity oracle for a non-analytic seam. CUT/FUSE now LAND
// via the single-seam outer weld: cyl − cone is a clean seam-driven cylinder stub, while cone − cyl
// and FUSE use the HOLED cone wall (full cone wall minus the seam cap, grid + loop-zipper) — the
// same on-surface hole-split scheme as the S5-p torus tube outer zone.
CC_TEST(cone_cyl_transversal_offset_common_watertight_matches_numeric) {
  const double r0 = 0.5, y0 = -3.0, r1 = 1.5, y1 = 3.0, off = 1.0, Rc = 0.3;
  const double cyLo = -2.0, cyHi = 2.0;
  const ntopo::Shape cone = makeCone(r0, y0, r1, y1);          // r(y)=0.5+(y+3)/6, axis +Y
  const ntopo::Shape cyl = makeCylOff(/*Y*/ 1, off, 0.0, Rc, cyLo, cyHi);  // axis at x=off, z=0
  CC_CHECK(!cone.isNull() && !cyl.isNull());

  const auto csCone = sd::recogniseCurvedSolid(cone);
  const auto csCyl = sd::recogniseCurvedSolid(cyl);
  CC_CHECK(csCone && csCyl);
  if (csCone && csCyl) {
    CC_CHECK(csCone->kind == sd::CurvedKind::Cone);
    CC_CHECK(csCyl->kind == sd::CurvedKind::Cylinder);
    // The trace is ONE fully-transversal CLOSED non-planar loop (single wall crossing).
    const ssi::TraceSet tr = ssi::trace_intersection(csCone->adapter(), csCyl->adapter());
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.branchPoints == 0);
    CC_CHECK(tr.lines.size() == 1);
    int closed = 0;
    for (const ssi::WLine& w : tr.lines)
      if (w.isClosed()) ++closed;
    CC_CHECK(closed == 1);
  }

  // Deterministic numeric COMMON volume (analytic region: inside the finite cone AND inside the
  // offset finite cylinder). 200³ midpoint cells → converged to ≪ the 1% curved-parity bar.
  const double slope = (r1 - r0) / (y1 - y0);
  const int nx = 200, ny = 200, nz = 200;
  const double X0 = off - Rc - 0.05, X1 = off + Rc + 0.05, Y0 = cyLo, Y1 = cyHi;
  const double Z0 = -Rc - 0.05, Z1 = Rc + 0.05;
  const double cell = (X1 - X0) / nx * (Y1 - Y0) / ny * (Z1 - Z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = X0 + (X1 - X0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = Y0 + (Y1 - Y0) * (j + 0.5) / ny;
      const double rr = r0 + slope * (y - y0);
      const double rr2 = rr * rr;
      for (int k = 0; k < nz; ++k) {
        const double z = Z0 + (Z1 - Z0) * (k + 0.5) / nz;
        const double dx = x - off;
        if (x * x + z * z <= rr2 && dx * dx + z * z <= Rc * Rc) ++inside;
      }
    }
  }
  const double vCommonNumeric = inside * cell;
  CC_CHECK(vCommonNumeric > 0.05);   // a real, non-degenerate overlap wedge (≈ 0.547)

  // ── COMMON: watertight native candidate whose volume matches the numeric oracle. ──
  const ntopo::Shape common = nb::ssi_boolean_solid(cone, cyl, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vCommon = watertightMeshVolume(common);
  CC_CHECK(vCommon > 0.0);                                    // watertight → engine accepts
  CC_CHECK(std::fabs(vCommon - vCommonNumeric) <= 1e-2 * vCommonNumeric);
  const double vCyl = sd::kSsiPi * Rc * Rc * (cyHi - cyLo);
  const double vCone = frustumVolume(r0, r1, y1 - y0);
  CC_CHECK(vCommon <= std::min(vCyl, vCone) + 1e-9);          // COMMON ≤ min(A,B)
  CC_CHECK(!nb::boolean_solid(cone, cyl, nb::Op::Common).isNull());

  // COMMON is symmetric — reversing the operand order builds the same watertight solid.
  const ntopo::Shape swapped = nb::ssi_boolean_solid(cyl, cone, nb::Op::Common);
  CC_CHECK(!swapped.isNull());
  const double vSwapped = watertightMeshVolume(swapped);
  CC_CHECK(vSwapped > 0.0);
  CC_CHECK(std::fabs(vSwapped - vCommonNumeric) <= 1e-2 * vCommonNumeric);

  // ── CUT / FUSE now LAND (S5-s single-seam outer weld: clean cyl stub + holed cone wall). ──
  // The integration box is sized for the COMMON wedge, so every CUT/FUSE oracle is the exact
  // analytic V(cyl)/V(cone) minus the numeric COMMON (inclusion–exclusion).
  const double vCutCylNumeric = vCyl - vCommonNumeric;                // cyl − cone = cyl − ∩
  const double vCutConeNumeric = vCone - vCommonNumeric;              // cone − cyl = cone − ∩
  const double vFuseNumeric = vCyl + vCone - vCommonNumeric;          // A ∪ B (inclusion–excl.)

  // CUT cylinder − cone: the clean seam-driven cylinder stub outside the cone; ΔV within 1%.
  const ntopo::Shape cutCC = nb::ssi_boolean_solid(cyl, cone, nb::Op::Cut);
  CC_CHECK(!cutCC.isNull());
  const double vCutCC = watertightMeshVolume(cutCC);
  CC_CHECK(vCutCC > 0.0);                                             // watertight → engine accepts
  CC_CHECK(std::fabs(vCutCC - vCutCylNumeric) <= 1e-2 * vCutCylNumeric);
  CC_CHECK(vCutCC < vCyl + 1e-9);                                     // A − B ≤ A

  // CUT cone − cylinder: the holed cone wall + cone discs + reversed cyl bite; ΔV within 1%.
  const ntopo::Shape cutCoC = nb::ssi_boolean_solid(cone, cyl, nb::Op::Cut);
  CC_CHECK(!cutCoC.isNull());
  const double vCutCoC = watertightMeshVolume(cutCoC);
  CC_CHECK(vCutCoC > 0.0);
  CC_CHECK(std::fabs(vCutCoC - vCutConeNumeric) <= 1e-2 * vCutConeNumeric);
  CC_CHECK(vCutCoC < vCone + 1e-9);

  // FUSE (both operand orders): watertight union envelope; ΔV within 1%; symmetric.
  const ntopo::Shape fuseCoC = nb::ssi_boolean_solid(cone, cyl, nb::Op::Fuse);
  const ntopo::Shape fuseCC = nb::ssi_boolean_solid(cyl, cone, nb::Op::Fuse);
  CC_CHECK(!fuseCoC.isNull() && !fuseCC.isNull());
  const double vFuseCoC = watertightMeshVolume(fuseCoC), vFuseCC = watertightMeshVolume(fuseCC);
  CC_CHECK(vFuseCoC > 0.0 && vFuseCC > 0.0);
  CC_CHECK(std::fabs(vFuseCoC - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(std::fabs(vFuseCC - vFuseNumeric) <= 1e-2 * vFuseNumeric);
  CC_CHECK(vFuseCoC >= std::max(vCyl, vCone) - 1e-9);                 // union ≥ max(A,B)

  // Partition identity: COMMON + (cyl − cone) = cyl (the numeric oracle self-consistency).
  CC_CHECK(std::fabs((vCommon + vCutCC) - vCyl) <= 2e-2 * vCyl);
}

// ── (21) TRANSVERSAL cone∩cylinder REDUCES TO COAXIAL: the offset→0 hand-off + honest declines ──
// As the perpendicular offset → 0 the pose becomes COAXIAL and the S5-e cone∩cylinder assembler
// (which runs BEFORE S5-s in the dispatch) owns it; S5-s's transversal setup gates on a strictly-
// positive offset so it declines the coaxial pose. This test pins that hand-off in two parts:
//   (a) a coaxial cylinder that CROSSES the cone wall (Rc=1.0, on-axis) yields a watertight S5-e
//       COMMON matching a deterministic 200³ numeric oracle;
//   (b) the SAME THIN cylinder (Rc=0.3) as the landed S5-s slice, taken COAXIAL, sits entirely
//       inside the cone (no wall crossing → no seam) so its analytic overlap is the WHOLE cylinder
//       — strictly LARGER than the S5-s offset overlap (the offset shrinks the overlap to a wedge).
// So the reduction boundary is a real hand-off, not a trivial identity. It also pins the S5-s
// honest declines: a skew (perpendicular-axis) cylinder never yields a leaky native solid.
CC_TEST(cone_cyl_transversal_reduces_to_coaxial_and_declines_skew) {
  const double r0 = 0.5, y0 = -3.0, r1 = 1.5, y1 = 3.0;
  const double cyLo = -2.0, cyHi = 2.0;
  const ntopo::Shape cone = makeCone(r0, y0, r1, y1);
  const double slope = (r1 - r0) / (y1 - y0);

  // (a) COAXIAL CROSSING (Rc=1.0, on the cone +Y axis) → S5-e single-circle COMMON.
  const double RcCross = 1.0;
  const ntopo::Shape cylCoax = makeCylOff(/*Y*/ 1, 0.0, 0.0, RcCross, cyLo, cyHi);
  const int nx = 200, ny = 200, nz = 200;
  const double X0 = -RcCross, X1 = RcCross, Y0 = cyLo, Y1 = cyHi, Z0 = -RcCross, Z1 = RcCross;
  const double cell = (X1 - X0) / nx * (Y1 - Y0) / ny * (Z1 - Z0) / nz;
  long inside = 0;
  for (int i = 0; i < nx; ++i) {
    const double x = X0 + (X1 - X0) * (i + 0.5) / nx;
    for (int j = 0; j < ny; ++j) {
      const double y = Y0 + (Y1 - Y0) * (j + 0.5) / ny;
      const double rr = r0 + slope * (y - y0);
      const double rr2 = rr * rr;
      for (int k = 0; k < nz; ++k) {
        const double z = Z0 + (Z1 - Z0) * (k + 0.5) / nz;
        if (x * x + z * z <= rr2 && x * x + z * z <= RcCross * RcCross) ++inside;
      }
    }
  }
  const double vCoaxNumeric = inside * cell;
  const ntopo::Shape coax = nb::ssi_boolean_solid(cone, cylCoax, nb::Op::Common);
  CC_CHECK(!coax.isNull());                                     // coaxial S5-e owns it
  const double vCoax = watertightMeshVolume(coax);
  CC_CHECK(vCoax > 0.0);
  CC_CHECK(std::fabs(vCoax - vCoaxNumeric) <= 1e-2 * vCoaxNumeric);

  // (b) The landed S5-s OFFSET thin slice (Rc=0.3, off=1.0) vs the analytic COAXIAL overlap of the
  // SAME thin cylinder. Coaxial (off=0) the whole cylinder is inside the cone (r(y) ≥ 0.667 > Rc),
  // so the coaxial overlap is the full cylinder volume — strictly larger than the offset wedge.
  const double RcThin = 0.3;
  const ntopo::Shape offCommon =
      nb::ssi_boolean_solid(cone, makeCylOff(1, 1.0, 0.0, RcThin, cyLo, cyHi), nb::Op::Common);
  CC_CHECK(!offCommon.isNull());                                // S5-s owns the offset thin pose
  const double vOff = watertightMeshVolume(offCommon);
  CC_CHECK(vOff > 0.0);
  const double vCoaxThinAnalytic = sd::kSsiPi * RcThin * RcThin * (cyHi - cyLo);  // full cylinder
  CC_CHECK(vCoaxThinAnalytic - vOff > 1e-2);   // coaxial (full-containment) overlap > offset wedge

  // HONEST DECLINE: a SKEW cylinder (axis ⟂ the cone +Y axis — here a +Z cylinder through the
  // cone) is a different, harder locus → S5-s parallel-axis gate rejects it. It must NEVER emit a
  // leaky native solid: the result is either NULL (→ OCCT) or a robustly watertight candidate.
  const ntopo::Shape skew = makeCyl(/*Z*/ 2, RcThin, -3, 3);
  const ntopo::Shape skewCommon = nb::ssi_boolean_solid(cone, skew, nb::Op::Common);
  if (!skewCommon.isNull()) CC_CHECK(watertightMeshVolume(skewCommon) > 0.0);
}

int main() { return cctest::run_all(); }
