// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 3, Stage 3 — the TOLERANT-TOPOLOGY
// SPLIT-HEALING PRE-PASS (src/native/boolean/split_healing.h). OCCT-FREE. The oracles
// are airtight and closed-form, mirroring the Stage-3 readiness verdict:
//
//   1. CLEAN LOOP → NO-OP — a clean rectangle heals to a byte-identical pass-through
//      (0 gaps closed, arcs unchanged, anyChanged=false), signed area preserved exactly.
//   2. SMALL-GAP LOOP → CLOSED + SPLIT-READY — a rectangle whose closing endpoints miss
//      by < tol is welded closed (1 gap), the output is a valid simple loop whose signed
//      area equals the input's within 1e-12, and it passes the split's own readiness
//      predicates (simple, non-self-crossing, area above the degeneracy floor) so it can
//      be fed to boolean/face_split.h. An OVER-tolerance gap on the same fixture DECLINES.
//   3. PINCHED LOOP → SIMPLE SUB-LOOPS — a figure-eight (self-touching at one vertex) is
//      resolved (by composing G5's splitAtPinches) into two valid simple sub-loops whose
//      SIGNED areas sum to the input's, region-preserving.
//   4. OVER-TOLERANCE GAP → HONEST DECLINE — a gap ≫ tol is NEVER force-welded; the loop
//      declines with blocker=LargeGap, the whole set's ok=false. A tolerance is never
//      widened to force a heal.
//
// It composes ONLY the Wave-G/G5 healing already in topology/trimmed_nurbs (never
// duplicated) and adds a signed-area-preservation gate. Header-only over boolean +
// topology; no numsci link (the composed primitives — healTrimLoop / splitTrimLoopAtPinches
// / flattenTrimLoop — are always-compiled), so this gate lives in the always-on suite.
//
#include <cstdio>

#include "native/boolean/split_healing.h"
#include "native/topology/trimmed_nurbs.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::topology;
namespace bln = cybercad::native::boolean;
namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectEq(int a, int b, const char* what) {
  ++g_checks;
  if (a != b) {
    std::printf("FAIL %-46s got %d want %d\n", what, a, b);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixtures — the SAME arc-loop builders the Layer-8 trimmed_nurbs gate uses (a
// straight-line pcurve segment; a rectangle; a gapped rectangle; a figure-eight).
// ─────────────────────────────────────────────────────────────────────────────

// A straight-line pcurve segment from (u0,v0) to (u1,v1).
static PcurveSegment lineSeg(double u0, double v0, double u1, double v1) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Line;
  const double du = u1 - u0, dv = v1 - v0;
  const double L = std::sqrt(du * du + dv * dv);
  seg.curve.origin2d = math::Point3{u0, v0, 0.0};
  seg.curve.dir2d = L > 0 ? math::Vec3{du / L, dv / L, 0.0} : math::Vec3{1, 0, 0};
  seg.first = 0.0;
  seg.last = L;
  seg.reversed = false;
  return seg;
}

// A rectangular loop in UV: [uLo,uHi]×[vLo,vHi], CCW (closes exactly).
static TrimLoop rectLoop(double uLo, double uHi, double vLo, double vHi) {
  TrimLoop loop;
  loop.push_back(lineSeg(uLo, vLo, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));
  return loop;
}

// A rectangle whose first segment's start is shifted by (du,dv), opening a gap of
// ‖(du,dv)‖ at the closing join. Every other join stays exact.
static TrimLoop rectLoopGapped(double uLo, double uHi, double vLo, double vHi,
                               double du, double dv) {
  TrimLoop loop;
  loop.push_back(lineSeg(uLo + du, vLo + dv, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));
  return loop;
}

// The figure-eight: two triangle lobes touching ONLY at the pinch vertex (0.5,0.5).
static TrimLoop figureEightLoop() {
  TrimLoop loop;
  loop.push_back(lineSeg(0.5, 0.5, 0.2, 0.7));
  loop.push_back(lineSeg(0.2, 0.7, 0.2, 0.3));
  loop.push_back(lineSeg(0.2, 0.3, 0.5, 0.5));
  loop.push_back(lineSeg(0.5, 0.5, 0.8, 0.3));
  loop.push_back(lineSeg(0.8, 0.3, 0.8, 0.7));
  loop.push_back(lineSeg(0.8, 0.7, 0.5, 0.5));
  return loop;
}

// ─────────────────────────────────────────────────────────────────────────────
// Split-readiness: replicate the predicates boolean/face_split.h self-verifies on a
// sub-loop it accepts — a simple (non-self-crossing) closed polygon with |area| above a
// scale-relative floor. If a healed loop passes these, the split machinery can consume it.
// ─────────────────────────────────────────────────────────────────────────────
static double shoelace(const bln::HealedLoop& p) {
  const std::size_t n = p.size();
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += p[j].u * p[i].v - p[i].u * p[j].v;
  return 0.5 * a;
}

// Proper (interior) crossing of open segments a0a1 and b0b1 (shared endpoints excluded).
static bool segsProperlyCross(const ParamPoint& a0, const ParamPoint& a1, const ParamPoint& b0,
                              const ParamPoint& b1) {
  auto orient = [](const ParamPoint& p, const ParamPoint& q, const ParamPoint& r) {
    return (q.u - p.u) * (r.v - p.v) - (q.v - p.v) * (r.u - p.u);
  };
  const double d1 = orient(a0, a1, b0);
  const double d2 = orient(a0, a1, b1);
  const double d3 = orient(b0, b1, a0);
  const double d4 = orient(b0, b1, a1);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && std::fabs(d1) > 1e-14 &&
         std::fabs(d2) > 1e-14 && std::fabs(d3) > 1e-14 && std::fabs(d4) > 1e-14;
}

// A healed loop is SPLIT-READY iff it is a simple closed polygon with area above the floor.
static bool splitReady(const bln::HealedLoop& p, double areaFloor) {
  const std::size_t n = p.size();
  if (n < 3) return false;
  if (std::fabs(shoelace(p)) < areaFloor) return false;
  for (std::size_t i = 0; i < n; ++i) {
    const ParamPoint& a0 = p[i];
    const ParamPoint& a1 = p[(i + 1) % n];
    for (std::size_t k = i + 1; k < n; ++k) {
      if (k == i || (k + 1) % n == i || i == (k + 1) % n) continue;  // adjacent (shared vertex)
      const ParamPoint& b0 = p[k];
      const ParamPoint& b1 = p[(k + 1) % n];
      if (segsProperlyCross(a0, a1, b0, b1)) return false;
    }
  }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// (1) CLEAN LOOP → NO-OP (byte-identical, 0 heals, area preserved exactly).
// ─────────────────────────────────────────────────────────────────────────────
static void testNoOp() {
  std::vector<TrimLoop> loops{rectLoop(0.2, 0.8, 0.2, 0.8)};
  const bln::HealTrimLoopsReport r = bln::healTrimLoops(loops);

  expectTrue(r.ok, "clean loop: report ok");
  expectTrue(!r.anyChanged, "clean loop: nothing changed (no-op)");
  expectEq(r.loopsNoOp, 1, "clean loop: 1 no-op");
  expectEq(r.loopsHealed, 0, "clean loop: 0 healed");
  expectEq(r.loopsDeclined, 0, "clean loop: 0 declined");
  expectTrue(r.loops.size() == 1 && r.loops[0].noOp, "clean loop: loop marked no-op");
  expectTrue(r.loops[0].arcsUnchanged, "clean loop: arcs byte-identical pass-through");
  expectEq(r.loops[0].gapsClosed, 0, "clean loop: 0 gaps closed");
  expectTrue(r.areaPreserved, "clean loop: area preserved");
  // No-op ⇒ signed area is the input's exactly (the same flattened polyline).
  expectLE(r.areaResidual, 0.0, "clean loop: area residual exactly 0");
  // The no-op still yields a split-ready polyline.
  const std::vector<bln::HealedLoop> hl = r.healedLoops();
  expectEq(static_cast<int>(hl.size()), 1, "clean loop: 1 healed loop out");
  expectTrue(splitReady(hl[0], 1e-9), "clean loop: output is split-ready");
}

// ─────────────────────────────────────────────────────────────────────────────
// (2) SMALL-GAP LOOP → CLOSED + SPLIT-READY (area preserved ≤ 1e-12).
// ─────────────────────────────────────────────────────────────────────────────
static void testGapCloseThenSplitReady() {
  // Rectangle [0.2,0.8]² with the closing endpoints missing by ‖(1e-9, 1e-9)‖ ≈ 1.4e-9,
  // well within the default relative gapTol (1e-6 × UV extent ~0.6 ⇒ ~6e-7). The weld's
  // area perturbation is O(gap²) ≈ 1e-18 ≪ 1e-12 (the area oracle bar).
  std::vector<TrimLoop> loops{rectLoopGapped(0.2, 0.8, 0.2, 0.8, 1e-9, 1e-9)};
  const bln::HealTrimLoopsReport r = bln::healTrimLoops(loops);

  expectTrue(r.ok, "small-gap: report ok");
  expectTrue(r.anyChanged, "small-gap: a heal happened");
  expectEq(r.loopsHealed, 1, "small-gap: 1 healed");
  expectEq(r.loopsDeclined, 0, "small-gap: 0 declined");
  expectTrue(r.loops[0].healed && !r.loops[0].declined, "small-gap: loop healed");
  expectTrue(r.loops[0].gapsClosed >= 1, "small-gap: at least one gap welded");
  expectTrue(r.loops[0].maxGapClosed > 1e-10, "small-gap: a real (above-noise) gap welded");
  expectEq(r.loops[0].subLoops, 1, "small-gap: one simple sub-loop out");

  // AREA PRESERVED ≤ 1e-12 (the closed loop's area == the reference rectangle's area).
  expectTrue(r.areaPreserved, "small-gap: area preserved");
  expectLE(r.areaResidual, 1e-12, "small-gap: area residual ≤ 1e-12");

  // The closed loop is SPLIT-READY: a valid simple polygon face_split would accept.
  const std::vector<bln::HealedLoop> hl = r.healedLoops();
  expectEq(static_cast<int>(hl.size()), 1, "small-gap: 1 healed loop out");
  expectTrue(splitReady(hl[0], 1e-9), "small-gap: closed loop is split-ready");
}

// ─────────────────────────────────────────────────────────────────────────────
// (3) PINCHED LOOP → simple sub-loops (area preserved, region-preserving via G5).
// ─────────────────────────────────────────────────────────────────────────────
static void testPinchResolve() {
  std::vector<TrimLoop> loops{figureEightLoop()};
  const bln::HealTrimLoopsReport r = bln::healTrimLoops(loops);

  expectTrue(r.ok, "pinch: report ok");
  expectTrue(r.anyChanged, "pinch: a heal happened");
  expectEq(r.loopsHealed, 1, "pinch: 1 healed");
  expectEq(r.pinchesResolved, 1, "pinch: 1 pinch resolved");
  expectTrue(r.loops[0].pinchResolved, "pinch: loop pinchResolved");
  expectTrue(r.loops[0].subLoops >= 2, "pinch: split into ≥2 sub-loops");

  // SIGNED-area preserved: Σ signedArea(sub-loops) == signedArea(figure-eight) ≤ 1e-12.
  expectTrue(r.areaPreserved, "pinch: signed area preserved");
  expectLE(r.areaResidual, 1e-12, "pinch: area residual ≤ 1e-12");

  // Each emitted sub-loop is a valid simple loop (split-ready).
  const std::vector<bln::HealedLoop> hl = r.healedLoops();
  expectTrue(hl.size() >= 2, "pinch: ≥2 healed loops out");
  bool allReady = true;
  for (const bln::HealedLoop& L : hl)
    if (!splitReady(L, 1e-9)) allReady = false;
  expectTrue(allReady, "pinch: every sub-loop is split-ready");
}

// ─────────────────────────────────────────────────────────────────────────────
// (4) OVER-TOLERANCE GAP → HONEST DECLINE (never fabricate a closure).
// ─────────────────────────────────────────────────────────────────────────────
static void testOverGapDecline() {
  // A gap of ‖(0.05, 0.05)‖ ≈ 0.07 ≫ the relative gapTol (~6e-7 for this UV extent).
  std::vector<TrimLoop> loops{rectLoopGapped(0.2, 0.8, 0.2, 0.8, 0.05, 0.05)};
  const bln::HealTrimLoopsReport r = bln::healTrimLoops(loops);

  expectTrue(!r.ok, "over-gap: whole set declines (ok=false)");
  expectEq(r.loopsHealed, 0, "over-gap: 0 healed");
  expectEq(r.loopsDeclined, 1, "over-gap: 1 declined");
  expectTrue(r.loops[0].declined && !r.loops[0].healed, "over-gap: loop declined");
  expectTrue(r.loops[0].blocker == bln::HealBlocker::LargeGap, "over-gap: blocker=LargeGap");
  expectTrue(r.loops[0].residualGap > 0.01, "over-gap: residual gap witnessed");
  expectTrue(r.loops[0].outLoops.empty(), "over-gap: no fabricated closure emitted");

  // Tightening the tolerance never widens to force a heal — with a LARGER tol that STILL
  // does not span the 0.07 gap the loop still declines (a monotone witness the tol is not
  // being widened to pass): even gapTol=1e-3 (× extent ~0.6 ⇒ ~6e-4) < 0.07 still declines.
  bln::HealTrimLoopsOptions strict;
  strict.gapTol = 1e-3;
  const bln::HealTrimLoopsReport r2 = bln::healTrimLoops(loops, strict);
  expectTrue(!r2.ok, "over-gap: still declines under a wider (but < gap) tol");
  expectTrue(r2.loops[0].blocker == bln::HealBlocker::LargeGap, "over-gap: still LargeGap");
}

// ─────────────────────────────────────────────────────────────────────────────
// (5) MIXED SET — a clean loop + a gapped loop heal together; the set is split-ready and
// its TOTAL signed area is the sum of the two input areas (composition sanity).
// ─────────────────────────────────────────────────────────────────────────────
static void testMixedSet() {
  std::vector<TrimLoop> loops{rectLoop(0.1, 0.4, 0.1, 0.4),
                              rectLoopGapped(0.6, 0.9, 0.6, 0.9, 1e-9, -1e-9)};
  const bln::HealTrimLoopsReport r = bln::healTrimLoops(loops);
  expectTrue(r.ok, "mixed: report ok");
  expectEq(r.loopsNoOp, 1, "mixed: 1 no-op");
  expectEq(r.loopsHealed, 1, "mixed: 1 healed");
  expectEq(r.loopsDeclined, 0, "mixed: 0 declined");
  // Whole-SET area preservation uses the header's scale-relative gate (the honest
  // invariant across a multi-loop set): residual ≤ areaAbsTol + areaRelTol·|Σ area|.
  expectTrue(r.areaPreserved, "mixed: total area preserved (scale-relative gate)");
  expectLE(r.areaResidual, r.areaTolerance, "mixed: residual within the applied tolerance");
  const std::vector<bln::HealedLoop> hl = r.healedLoops();
  expectEq(static_cast<int>(hl.size()), 2, "mixed: 2 split-ready loops out");
  for (const bln::HealedLoop& L : hl) expectTrue(splitReady(L, 1e-9), "mixed: loop split-ready");
}

int main() {
  testNoOp();
  testGapCloseThenSplitReady();
  testPinchResolve();
  testOverGapDecline();
  testMixedSet();

  std::printf("\nsplit_healing host gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
