// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the Tier-1 + Tier-2#4 native-geometry COMPLETION batch
// ("construct-residuals", src/native/construct/residuals.h): the two profile-builder
// residuals that were OCCT-fallthrough after Tier-A/B/C/D (see
// openspec/NATIVE-REWRITE.md #4b):
//
//   1. kind-3 SPLINE profile edges — a B-spline curve edge (native-math NURBS) in a
//      TYPED profile, extruded into a prism. build_prism_profile_spline fits a clamped
//      B-spline through the supplied control points, then expands it into a dense
//      deflection-bounded polyline so the whole profile routes through the proven
//      straight-edge watertight extrude path (no free-form face meshing). The spline
//      SHAPE is the fitted NURBS; the B-rep stores the polyline.
//
//   2. OFF-AXIS circular arc revolve → a TORUS surface of revolution (native-math
//      Torus, src/native/math/torus.h). An arc whose circle centre sits at a NON-ZERO
//      distance from the revolution axis sweeps a torus, emitted as EXACT rational-
//      quadratic B-spline surface-of-revolution bands. A FULL 360° revolve of a FULL
//      circle meridian gives a ring torus; its analytic volume is 2·π²·R·r².
//
// OCCT-FREE — Gate 1 (host, analytic) of the two-gate model. Each native result is
// validated with the native TESSELLATOR (watertight / enclosed volume / area) and the
// topology EXPLORER (surface/curve kinds). SELF-INTERSECTING / partial-turn / spindle
// cases are asserted to return the NOT-SUPPORTED signal (a NULL Shape) so the engine
// falls through to OCCT — never a faked or leaky solid.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_residuals.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_residuals
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {
constexpr double kPi = 3.14159265358979323846;

int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}

// Watertight at the given deflection; returns the mesh's |enclosed volume|.
double meshVolumeIfWatertight(const topo::Shape& s, double defl, bool& watertight) {
  tess::MeshParams p;
  p.deflection = defl;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(s);
  watertight = tess::isWatertight(mesh);
  return std::fabs(tess::enclosedVolume(mesh));
}

// A straight LINE profile segment.
cst::ProfileSegment line(double x0, double y0, double x1, double y1) {
  cst::ProfileSegment s;
  s.kind = 0;
  s.x0 = x0; s.y0 = y0; s.x1 = x1; s.y1 = y1;
  return s;
}
}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// RESIDUAL #1 — SPLINE-edge profile extrude (build_prism_profile_spline)
// ═════════════════════════════════════════════════════════════════════════════

// ── A closed profile with a kind-3 SPLINE top edge → a watertight prism ──────────
// Outer boundary: a 10×6 rectangle whose TOP edge (10,6)→(0,6) is replaced by a
// spline that bulges UP through control points (10,6)(7,8)(3,8)(0,6). The residual
// builder fits the clamped B-spline, expands it into a dense polyline, and extrudes
// the whole profile depth 4 into a prism.
//
// Asserted (analytic, no OCCT):
//   * the solid is WATERTIGHT (boundaryEdges==0) at coarse AND fine deflection — the
//     spline edge welds to the flat caps and side walls via the shared polyline;
//   * the SPLINE EDGE is PRESENT and non-trivial: the outer cap loop has many more
//     edges than the 4 of a plain rectangle (the spline expands into a curved run of
//     line segments), so the face count far exceeds the 6 of a straight box prism;
//   * the volume is SANE: strictly greater than the base 10×6 rectangle prism
//     (60·4 = 240), because the spline bulges the profile OUTWARD above y=6, and
//     bounded above by the bounding-box prism (10·8·4 = 320).
CC_TEST(spline_extrude_bulging_top_is_watertight_with_spline_edge) {
  // Spline control points for the top edge (x,y pairs on z=0), bulging up to y=8.
  const double splineXY[] = {10, 6, 7, 8, 3, 8, 0, 6};
  std::vector<cst::ProfileSegment> segs = {
      line(0, 0, 10, 0),   // bottom
      line(10, 0, 10, 6),  // right
      {},                  // spline top — filled below
      line(0, 6, 0, 0),    // left (closes)
  };
  segs[2].kind = 3;         // kind-3 SPLINE segment
  segs[2].ptOffset = 0;     // control points start at splineXY[0]
  segs[2].ptCount = 4;      // 4 control points

  const topo::Shape solid =
      cst::build_prism_profile_spline(segs, splineXY, 8, {}, {}, 4.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  // The spline top edge becomes ONE B-spline side FACE (matching OCCT's single spline
  // face); with three line walls + two caps the prism has exactly 6 faces. (This is the
  // face-count-parity representation: a spline segment contributes exactly one wall,
  // unlike the earlier polyline expansion that emitted a curved run of quads.)
  const int faceCount = countSub(solid, topo::ShapeType::Face);
  CC_CHECK(faceCount == 6);  // 3 line walls + 1 spline wall + 2 caps

  // Watertight at BOTH a coarse and a fine deflection (the spline↔cap seam welds via
  // the shared B-spline rim edge / canonical anchors).
  bool wtCoarse = false, wtFine = false;
  const double volCoarse = meshVolumeIfWatertight(solid, 0.05, wtCoarse);
  const double volFine = meshVolumeIfWatertight(solid, 0.01, wtFine);
  CC_CHECK(wtCoarse);
  CC_CHECK(wtFine);

  // Sane volume: above the base 10×6 rectangle prism, below the bbox prism. The fitted
  // spline INTERPOLATES its control points (passes through (7,8)/(3,8)), so the profile
  // bulges OUTWARD above y=6 (volume strictly above the rectangle prism).
  const double baseRect = 60.0 * 4.0;   // 240
  const double bboxPrism = 10.0 * 8.0 * 4.0;  // 320
  CC_CHECK(volFine > baseRect);
  CC_CHECK(volFine < bboxPrism);
  // The two deflections agree to the curved-face deflection bound (the spline wall is a
  // true B-spline surface, so a coarser mesh chords the curve slightly more).
  CC_CHECK(std::fabs(volCoarse - volFine) / volFine < 2e-2);
}

// ── A profile with NO spline segment delegates to the plain typed extrude ────────
// build_prism_profile_spline forwards a line/arc-only profile to profile.h's typed
// path (this module only owns the residual spline case). A 10×10 square (4 lines) →
// a 10×10×5 box: 6 faces, watertight, volume 500 EXACT.
CC_TEST(spline_extrude_no_spline_delegates_to_typed_box) {
  std::vector<cst::ProfileSegment> segs = {
      line(0, 0, 10, 0), line(10, 0, 10, 10), line(10, 10, 0, 10), line(0, 10, 0, 0)};
  const topo::Shape solid = cst::build_prism_profile_spline(segs, nullptr, 0, {}, {}, 5.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;
  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 6);
  bool wt = false;
  const double vol = meshVolumeIfWatertight(solid, 0.1, wt);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(vol - 500.0) < 1e-6);
}

// ── DEFERRAL: a degenerate spline segment / bad depth → NULL (OCCT fallthrough) ──
CC_TEST(spline_extrude_degenerate_returns_null) {
  const double splineXY[] = {0, 0, 1, 1, 2, 2, 3, 3};
  std::vector<cst::ProfileSegment> segs = {line(0, 0, 10, 0), line(10, 0, 10, 6), {},
                                           line(0, 6, 0, 0)};
  segs[2].kind = 3; segs[2].ptOffset = 0; segs[2].ptCount = 4;
  // Zero / negative depth → NULL.
  CC_CHECK(cst::build_prism_profile_spline(segs, splineXY, 8, {}, {}, 0.0).isNull());
  CC_CHECK(cst::build_prism_profile_spline(segs, splineXY, 8, {}, {}, -1.0).isNull());
  // A spline segment whose ptCount exceeds the supplied buffer → NULL (bounds guard;
  // splineN is the number of DOUBLES = 2× the point count, per cc_kernel.h).
  std::vector<cst::ProfileSegment> bad = {line(0, 0, 10, 0), line(10, 0, 10, 6), {},
                                          line(0, 6, 0, 0)};
  bad[2].kind = 3; bad[2].ptOffset = 0; bad[2].ptCount = 4;
  CC_CHECK(cst::build_prism_profile_spline(bad, splineXY, 4, {}, {}, 4.0).isNull());  // only 2 pts fit
}

// ═════════════════════════════════════════════════════════════════════════════
// RESIDUAL #2 — OFF-AXIS ARC revolve → a TORUS (build_revolution_profile_spline)
// ═════════════════════════════════════════════════════════════════════════════

// ── A FULL off-axis circle revolved 360° → a ring TORUS, watertight, vol 2π²Rr² ──
// The meridian is a FULL circle (arc a0=0, a1=2π) of radius r=1.5 whose centre sits
// at distance R=5 from the Y revolution axis. Revolving it a full turn sweeps a ring
// torus. Its analytic volume is 2·π²·R·r² and its analytic surface area is 4·π²·R·r.
//
// The torus is meshed via the rational-B-spline surface-of-revolution bands. Both the
// angular (revolution) direction AND the meridian tube circle are polyline/chord
// approximations at a finite deflection, so the meshed volume converges FROM BELOW to
// the analytic value — we assert watertight + a deflection bound (< 4%), and that a
// FINER deflection is at least as close as a coarser one (monotone convergence).
CC_TEST(offaxis_full_circle_revolve_is_watertight_torus) {
  const double R = 5.0, r = 1.5;
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;              // arc
  segs[0].cx = R; segs[0].cy = 0.0; segs[0].r = r;  // centre OFF the axis (distance R)
  segs[0].x0 = R + r; segs[0].y0 = 0.0;             // full circle: endpoints coincide
  segs[0].x1 = R + r; segs[0].y1 = 0.0;
  segs[0].a0 = 0.0; segs[0].a1 = 2.0 * kPi;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};

  const topo::Shape torus =
      cst::build_revolution_profile_spline(segs, nullptr, 0, yAxis, 2.0 * kPi);
  CC_CHECK(!torus.isNull());
  if (torus.isNull()) return;

  // Full-turn revolve of a closed meridian ⇒ no planar caps; a genus-1 (toroidal)
  // closed surface tiled into rational bands. It must be WATERTIGHT.
  bool wtCoarse = false, wtFine = false;
  const double volCoarse = meshVolumeIfWatertight(torus, 0.05, wtCoarse);
  const double volFine = meshVolumeIfWatertight(torus, 0.01, wtFine);
  CC_CHECK(wtCoarse);
  CC_CHECK(wtFine);

  const double expected = 2.0 * kPi * kPi * R * r * r;  // 2π²Rr² ≈ 222.07
  // Deflection-bounded: within 4%, converging from below.
  CC_CHECK(std::fabs(volFine - expected) / expected < 0.04);
  CC_CHECK(volFine < expected);  // chord under-fills a convex tube → from below
  // Finer deflection is no worse than the coarser one (monotone convergence).
  CC_CHECK(std::fabs(volFine - expected) <= std::fabs(volCoarse - expected) + 1e-9);

  // The surface has no spurious planar caps — a torus is all curved bands. There is at
  // least one face and no cap that would betray a mis-closed (partial-turn) solid: a
  // full ring torus is a single toroidal shell of curved (BSpline) bands.
  CC_CHECK(countSub(torus, topo::ShapeType::Face) >= 3);  // ≥ 120° spans → ≥ 3 bands
}

// ── The torus volume TRACKS 2π²Rr² across radii (analytic scaling) ───────────────
// A larger tube radius r and larger major radius R must scale the meshed volume as
// 2π²Rr²: doubling r quadruples the tube area, doubling R doubles the sweep. We assert
// the meshed volume of two different tori is within the same 4% deflection bound of
// their analytic values — the formula is the geometry, not a fit.
CC_TEST(offaxis_torus_volume_tracks_analytic_across_radii) {
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  auto torusVol = [&](double R, double r, bool& wt) {
    std::vector<cst::ProfileSegment> segs(1);
    segs[0].kind = 1;
    segs[0].cx = R; segs[0].cy = 0.0; segs[0].r = r;
    segs[0].x0 = R + r; segs[0].y0 = 0.0; segs[0].x1 = R + r; segs[0].y1 = 0.0;
    segs[0].a0 = 0.0; segs[0].a1 = 2.0 * kPi;
    const topo::Shape s =
        cst::build_revolution_profile_spline(segs, nullptr, 0, yAxis, 2.0 * kPi);
    if (s.isNull()) { wt = false; return 0.0; }
    return meshVolumeIfWatertight(s, 0.01, wt);
  };

  bool wtA = false, wtB = false;
  const double R1 = 4.0, r1 = 1.0, R2 = 8.0, r2 = 2.0;
  const double vA = torusVol(R1, r1, wtA);
  const double vB = torusVol(R2, r2, wtB);
  CC_CHECK(wtA);
  CC_CHECK(wtB);
  const double eA = 2.0 * kPi * kPi * R1 * r1 * r1;
  const double eB = 2.0 * kPi * kPi * R2 * r2 * r2;
  CC_CHECK(std::fabs(vA - eA) / eA < 0.04);
  CC_CHECK(std::fabs(vB - eB) / eB < 0.04);
  // vB/vA analytic ratio = (R2·r2²)/(R1·r1²) = (8·4)/(4·1) = 8. The meshed ratio must
  // match within twice the per-torus deflection bound.
  CC_CHECK(std::fabs((vB / vA) - 8.0) / 8.0 < 0.08);
}

// ── DEFERRAL: a SPINDLE torus (R < r, arc crosses the axis) → NULL (Tier-4 SSI) ──
// When the arc centre distance R is LESS than the tube radius r the tube would sweep
// through the revolution axis — a self-intersecting "spindle" torus. That is genuine
// surface-surface-intersection territory (Tier 4), NOT attempted here: the builder
// returns the not-supported signal (NULL) so the engine falls through to OCCT.
CC_TEST(spindle_torus_self_intersecting_returns_null) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;
  segs[0].cx = 1.0; segs[0].cy = 0.0; segs[0].r = 3.0;  // R=1 < r=3 → spindle
  segs[0].x0 = 4.0; segs[0].y0 = 0.0; segs[0].x1 = 4.0; segs[0].y1 = 0.0;
  segs[0].a0 = 0.0; segs[0].a1 = 2.0 * kPi;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  CC_CHECK(cst::build_revolution_profile_spline(segs, nullptr, 0, yAxis, 2.0 * kPi).isNull());
}

// ── DEFERRAL: a PARTIAL-turn off-axis arc revolve → NULL (needs meridian caps) ──--
// A partial turn needs two planar meridian end caps that weld to the curved rational
// bands along the meridian rim — a curved band↔planar-cap seam the residual builder
// does not robustly weld — so a partial-turn torus FALLS THROUGH to OCCT (verified,
// never a leaky solid). The FULL-turn case above is fully native.
CC_TEST(offaxis_torus_partial_turn_returns_null) {
  const double R = 5.0, r = 1.5;
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;
  segs[0].cx = R; segs[0].cy = 0.0; segs[0].r = r;
  segs[0].x0 = R + r; segs[0].y0 = 0.0; segs[0].x1 = R + r; segs[0].y1 = 0.0;
  segs[0].a0 = 0.0; segs[0].a1 = 2.0 * kPi;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  CC_CHECK(cst::build_revolution_profile_spline(segs, nullptr, 0, yAxis, kPi).isNull());
}

CC_RUN_ALL()
