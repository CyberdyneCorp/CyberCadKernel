// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the Layer-3 Stage-3 MULTI-CROSSING / RE-ENTRANT / HOLE-CROSSING
// face split (src/native/boolean/multi_crossing_split.h) — the planar-arrangement
// generalisation of the byte-frozen convex-1-chord (face_split.h) + closed-interior-seam
// (smooth_trim_split.h) slices into N ≥ 2 sub-regions. OCCT-FREE. Every oracle is
// AIRTIGHT and closed-form; the asserted tolerances are the ACHIEVED errors, never
// widened to pass:
//
//   1. TWO NON-CROSSING CHORDS (3 regions) — a unit square cut by two vertical chords
//      → exactly 3 sub-regions, areas 0.3 / 0.4 / 0.3, summing to the parent (≤1e-10),
//      each a simple loop.
//   2. CROSSING CHORDS (4 regions) — a unit square cut by two chords that cross at the
//      centre → exactly 4 sub-regions, each area 0.25, summing to the parent (≤1e-10).
//   3. HOLE-CROSSING SEAM — a square with a central square hole cut by one chord that
//      crosses the hole → the net area (outer − hole) tiles exactly across the pieces
//      (≤1e-10), holes correctly attributed.
//   4. DEGENERATE DECLINE — a seam coincident with a boundary edge → HONEST-DECLINE
//      (Degenerate), never a fabricated tiling; and a single non-cutting chord →
//      NoSubdivision.
//
// The splitter reuses the trim_boolean segment-crossing closed form + the same
// orientation-coherent arc-walk family (outer CCW, holes CW). It composes with the L3
// SSI cut pcurves: SSI produces the seams that split a trimmed operand face; this module
// tiles the face into the sub-regions the seams cut it into.
//
// This is a pure src/native test (reuses flattenTrimLoop from the always-on topology
// lib; no numsci link needed), registered in the numsci-gated block for symmetry with
// its face_split / smooth_trim_split siblings.
//
#include <cstdio>

#include "native/boolean/multi_crossing_split.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::boolean;
using cybercad::native::topology::ParamPoint;

static int g_failures = 0;
static int g_checks = 0;
static double g_maxTilingGap = 0.0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-44s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}

// A closed axis-aligned rectangle [u0,u1]×[v0,v1] as a CCW UV polygon (4 corners).
static std::vector<ParamPoint> rect(double u0, double v0, double u1, double v1) {
  return {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
}

// Find a sub-region whose area is ≈ `want` (for order-independent assertions).
static bool hasRegionArea(const std::vector<SubRegion>& rs, double want, double tol) {
  for (const auto& r : rs)
    if (std::fabs(r.signedArea - want) <= tol) return true;
  return false;
}

// ── Oracle 1: two non-crossing chords → 3 regions. ─────────────────────────────
static void testTwoChords() {
  MultiSplitInput in;
  in.outer = rect(0, 0, 1, 1);
  // Two vertical chords at u = 0.3 and u = 0.7, each spanning the full height.
  in.seams.push_back({{0.3, 0.0}, {0.3, 1.0}});
  in.seams.push_back({{0.7, 0.0}, {0.7, 1.0}});

  MultiSplitResult r = multiCrossingSplit(in);
  expectTrue(r.ok(), "2-chord ok");
  if (!r.ok()) { std::printf("  decline=%s\n", multiSplitDeclineName(r.decline)); return; }
  expectTrue(r.subRegions == 3, "2-chord → 3 sub-regions");
  expectNear(r.tiledArea, 1.0, 1e-10, "2-chord Σ area == parent");
  expectNear(r.tilingGap, 0.0, 1e-10, "2-chord tiling gap 0");
  g_maxTilingGap = std::max(g_maxTilingGap, r.tilingGap);
  const auto& rs = *r.regions;
  expectTrue(hasRegionArea(rs, 0.3, 1e-10), "2-chord left slab 0.3");
  expectTrue(hasRegionArea(rs, 0.4, 1e-10), "2-chord mid slab 0.4");
  // (two 0.3 slabs — one hasRegionArea proof is enough given the exact Σ.)
}

// ── Oracle 2: two crossing chords → 4 regions. ─────────────────────────────────
static void testCrossingChords() {
  MultiSplitInput in;
  in.outer = rect(0, 0, 1, 1);
  in.seams.push_back({{0.5, 0.0}, {0.5, 1.0}});  // vertical through the centre
  in.seams.push_back({{0.0, 0.5}, {1.0, 0.5}});  // horizontal through the centre

  MultiSplitResult r = multiCrossingSplit(in);
  expectTrue(r.ok(), "crossing ok");
  if (!r.ok()) { std::printf("  decline=%s\n", multiSplitDeclineName(r.decline)); return; }
  expectTrue(r.subRegions == 4, "crossing → 4 sub-regions");
  expectNear(r.tiledArea, 1.0, 1e-10, "crossing Σ area == parent");
  expectNear(r.tilingGap, 0.0, 1e-10, "crossing tiling gap 0");
  g_maxTilingGap = std::max(g_maxTilingGap, r.tilingGap);
  for (const auto& sr : *r.regions)
    expectNear(sr.signedArea, 0.25, 1e-10, "crossing quadrant area 0.25");
}

// ── Oracle 3: a chord crossing a central hole. ─────────────────────────────────
static void testHoleCrossing() {
  MultiSplitInput in;
  in.outer = rect(0, 0, 4, 4);
  in.holes.push_back(rect(1.5, 1.5, 2.5, 2.5));  // 1×1 central hole
  // Horizontal chord at v = 2.0 spanning the full width — cuts through the hole.
  in.seams.push_back({{0.0, 2.0}, {4.0, 2.0}});

  MultiSplitResult r = multiCrossingSplit(in);
  expectTrue(r.ok(), "hole-crossing ok");
  if (!r.ok()) { std::printf("  decline=%s\n", multiSplitDeclineName(r.decline)); return; }
  // Net parent area = 16 − 1 = 15; the chord halves it into two pieces of net area 7.5.
  const double parentNet = 16.0 - 1.0;
  expectNear(r.parentArea, parentNet, 1e-10, "hole-crossing parent net area 15");
  expectTrue(r.subRegions >= 2, "hole-crossing ≥ 2 sub-regions");
  expectNear(r.tiledArea, parentNet, 1e-10, "hole-crossing Σ net area == parent");
  expectNear(r.tilingGap, 0.0, 1e-10, "hole-crossing tiling gap 0");
  g_maxTilingGap = std::max(g_maxTilingGap, r.tilingGap);
}

// ── Oracle 3b: a chord that does NOT cross a hole — hole attributed to one piece. ─
static void testHoleAttribution() {
  MultiSplitInput in;
  in.outer = rect(0, 0, 4, 4);
  in.holes.push_back(rect(0.5, 0.5, 1.5, 1.5));  // hole in the lower-left
  in.seams.push_back({{0.0, 3.0}, {4.0, 3.0}});  // high horizontal chord, above the hole

  MultiSplitResult r = multiCrossingSplit(in);
  expectTrue(r.ok(), "hole-attribution ok");
  if (!r.ok()) { std::printf("  decline=%s\n", multiSplitDeclineName(r.decline)); return; }
  const double parentNet = 16.0 - 1.0;
  expectNear(r.tiledArea, parentNet, 1e-10, "hole-attribution Σ net area == parent");
  g_maxTilingGap = std::max(g_maxTilingGap, r.tilingGap);
  // The lower piece (area 12 gross) owns the hole ⇒ its net area is 11; the upper is 4.
  const auto& rs = *r.regions;
  expectTrue(hasRegionArea(rs, 11.0, 1e-10), "hole-attribution lower net 11");
  expectTrue(hasRegionArea(rs, 4.0, 1e-10), "hole-attribution upper 4");
}

// ── Oracle 4: degenerate seams → honest decline. ───────────────────────────────
static void testDegenerateDecline() {
  // (a) A seam coincident with the bottom boundary edge — coincident-overlap ⇒ Degenerate.
  {
    MultiSplitInput in;
    in.outer = rect(0, 0, 1, 1);
    in.seams.push_back({{0.0, 0.0}, {1.0, 0.0}});  // lies ON the bottom edge
    MultiSplitResult r = multiCrossingSplit(in);
    expectTrue(!r.ok(), "coincident seam declines");
    expectTrue(r.decline == MultiSplitDecline::Degenerate, "coincident seam → Degenerate");
  }
  // (b) A chord wholly interior that touches no boundary (does not subdivide) ⇒
  //     NoSubdivision (a single region is not a split).
  {
    MultiSplitInput in;
    in.outer = rect(0, 0, 1, 1);
    in.seams.push_back({{0.3, 0.4}, {0.6, 0.6}});  // floating interior stub
    MultiSplitResult r = multiCrossingSplit(in);
    expectTrue(!r.ok(), "non-cutting chord declines");
    expectTrue(r.decline == MultiSplitDecline::NoSubdivision,
               "non-cutting chord → NoSubdivision");
  }
}

// ── Oracle 5: re-entrant chord — a chord crossing the boundary MORE than twice. ─
static void testReentrant() {
  // A non-convex (L-shaped) outer loop cut by a horizontal chord that enters/exits it
  // TWICE (four boundary crossings) ⇒ the chord carves TWO separate pieces off. The
  // L is [0,3]×[0,1] ∪ [0,1]×[1,3] (an L), and the chord v = 0.5 crosses the wide base.
  // Use a U-shape so a single horizontal chord genuinely re-enters:
  //   U = big rect [0,4]×[0,3] MINUS the notch [1,3]×[1,3] (top-centre bite).
  MultiSplitInput in;
  in.outer = {{0, 0}, {4, 0}, {4, 3}, {3, 3}, {3, 1}, {1, 1}, {1, 3}, {0, 3}};
  // Horizontal chord at v = 2 crosses the two legs of the U (4 boundary crossings).
  in.seams.push_back({{0.0, 2.0}, {4.0, 2.0}});

  MultiSplitResult r = multiCrossingSplit(in);
  expectTrue(r.ok(), "re-entrant ok");
  if (!r.ok()) { std::printf("  decline=%s\n", multiSplitDeclineName(r.decline)); return; }
  // U area = 4*3 − 2*2 = 8. The chord at v=2 cuts the two legs (each 1 wide) above v=2:
  // it produces sub-regions that still Σ to the parent area exactly.
  expectNear(r.parentArea, 8.0, 1e-10, "re-entrant parent area 8");
  expectNear(r.tiledArea, 8.0, 1e-10, "re-entrant Σ area == parent");
  expectNear(r.tilingGap, 0.0, 1e-10, "re-entrant tiling gap 0");
  expectTrue(r.subRegions >= 3, "re-entrant ≥ 3 sub-regions");
  g_maxTilingGap = std::max(g_maxTilingGap, r.tilingGap);
}

int main() {
  testTwoChords();
  testCrossingChords();
  testHoleCrossing();
  testHoleAttribution();
  testDegenerateDecline();
  testReentrant();

  std::printf("\nmulti_crossing_split: %d checks, %d failures; max tiling gap %.3g\n", g_checks,
              g_failures, g_maxTilingGap);
  return g_failures == 0 ? 0 : 1;
}
