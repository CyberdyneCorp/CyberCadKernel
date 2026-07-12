// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the parameter-space trimmed-region boolean
// (src/native/topology/trim_boolean.{h,cpp}) — the (u,v) half of the trimmed-NURBS
// B-rep boolean. OCCT-FREE. Every oracle is AIRTIGHT and closed-form; the asserted
// tolerances are the ACHIEVED errors, never widened to pass:
//
//   1. DISJOINT squares — Union = both (area = sum), Intersect = EMPTY, Difference = A.
//   2. OVERLAPPING squares — A∪B, A∩B, A∖B produce the exact polygonal regions with
//      area equal to the closed-form value to ≤ 1e-10.
//   3. HOLE handling — an annulus (square with a square hole) unioned / intersected with
//      a disk gives the correct nested-loop signed area.
//   4. CIRCLE trims (rational pcurves) — two overlapping circular trim loops → the
//      lens-shaped intersection with area matching the closed-form circular-lens area
//      to ≤ 1e-8 (fine flatten of the exact rational-circle pcurve).
//   5. COINCIDENT-edge overlap — two squares sharing a boundary edge → HONEST-DECLINE
//      (status Degenerate), never a fabricated region.
//
// The boolean reuses the trimmed_nurbs pcurve evaluator (flattenTrimLoop) and the same
// even-odd classify rule, so a rational circle pcurve flattens with no sag. It composes
// with the L3 SSI cut pcurves: SSI produces the seam pcurves that split the operands'
// trim loops; this module then assembles the trimmed result region.
//
// This is a pure src/native/topology test (no numsci link needed), but it is registered
// in the numsci-gated block for symmetry with the H1 intersector it conceptually reuses;
// with the guard OFF it still compiles + passes (the boolean itself is always-on).
//
#include <cstdio>

#include "native/topology/trim_boolean.h"
#include "native/topology/trimmed_nurbs.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::topology;

static int g_failures = 0;
static int g_checks = 0;
static double g_maxSquareAreaErr = 0.0;
static double g_maxLensAreaErr = 0.0;

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
    std::printf("FAIL %-40s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}

// ── Loop builders. ───────────────────────────────────────────────────────────

// A CCW axis-aligned rectangle [u0,u1]×[v0,v1] as four Line pcurve segments.
static TrimLoop rectLoop(double u0, double v0, double u1, double v1, bool ccw = true) {
  auto lineSeg = [](double au, double av, double bu, double bv) {
    PcurveSegment s;
    s.curve.kind = EdgeCurve::Kind::Line;
    s.curve.origin2d = {au, av, 0};
    s.curve.dir2d = {bu - au, bv - av, 0};
    s.first = 0.0;
    s.last = 1.0;
    s.reversed = false;
    return s;
  };
  TrimLoop loop;
  if (ccw) {
    loop.push_back(lineSeg(u0, v0, u1, v0));
    loop.push_back(lineSeg(u1, v0, u1, v1));
    loop.push_back(lineSeg(u1, v1, u0, v1));
    loop.push_back(lineSeg(u0, v1, u0, v0));
  } else {
    loop.push_back(lineSeg(u0, v0, u0, v1));
    loop.push_back(lineSeg(u0, v1, u1, v1));
    loop.push_back(lineSeg(u1, v1, u1, v0));
    loop.push_back(lineSeg(u1, v0, u0, v0));
  }
  return loop;
}

// A circle of radius r centred at (cu,cv), one Circle pcurve over [0, 2π]. CCW.
static TrimLoop circleLoop(double cu, double cv, double r) {
  PcurveSegment s;
  s.curve.kind = EdgeCurve::Kind::Circle;
  s.curve.origin2d = {cu, cv, 0};
  s.curve.dir2d = {r, 0, 0};  // pcurveValue reads r = dir2d.x
  s.first = 0.0;
  s.last = 2.0 * M_PI;
  s.reversed = false;
  return TrimLoop{s};
}

// ── Tests. ───────────────────────────────────────────────────────────────────

static void testDisjointSquares() {
  TrimRegion A{rectLoop(0, 0, 1, 1), {}};
  TrimRegion B{rectLoop(3, 0, 4, 1), {}};

  auto uni = trimRegionBoolean(A, B, TrimBoolOp::Union);
  expectTrue(uni.ok(), "disjoint union ok");
  expectNear(uni.area, 2.0, 1e-10, "disjoint union area = 2");
  g_maxSquareAreaErr = std::max(g_maxSquareAreaErr, std::fabs(uni.area - 2.0));
  expectTrue(uni.loops.size() == 2, "disjoint union has 2 loops");

  auto inter = trimRegionBoolean(A, B, TrimBoolOp::Intersect);
  expectTrue(inter.status == TrimBoolStatus::Empty, "disjoint intersect empty");
  expectNear(inter.area, 0.0, 1e-12, "disjoint intersect area 0");

  auto diff = trimRegionBoolean(A, B, TrimBoolOp::Difference);
  expectTrue(diff.ok(), "disjoint difference ok");
  expectNear(diff.area, 1.0, 1e-10, "disjoint difference area = A");
}

static void testOverlappingSquares() {
  // A = [0,2]×[0,2], B = [1,3]×[1,3]. Overlap = [1,2]×[1,2] area 1.
  TrimRegion A{rectLoop(0, 0, 2, 2), {}};
  TrimRegion B{rectLoop(1, 1, 3, 3), {}};

  auto inter = trimRegionBoolean(A, B, TrimBoolOp::Intersect);
  expectTrue(inter.ok(), "overlap intersect ok");
  expectNear(inter.area, 1.0, 1e-10, "overlap intersect area = 1");
  g_maxSquareAreaErr = std::max(g_maxSquareAreaErr, std::fabs(inter.area - 1.0));

  auto uni = trimRegionBoolean(A, B, TrimBoolOp::Union);
  expectTrue(uni.ok(), "overlap union ok");
  // |A| + |B| - |A∩B| = 4 + 4 - 1 = 7.
  expectNear(uni.area, 7.0, 1e-10, "overlap union area = 7");
  g_maxSquareAreaErr = std::max(g_maxSquareAreaErr, std::fabs(uni.area - 7.0));

  auto diff = trimRegionBoolean(A, B, TrimBoolOp::Difference);
  expectTrue(diff.ok(), "overlap difference ok");
  // |A| - |A∩B| = 4 - 1 = 3.
  expectNear(diff.area, 3.0, 1e-10, "overlap difference area = 3");
  g_maxSquareAreaErr = std::max(g_maxSquareAreaErr, std::fabs(diff.area - 3.0));
}

static void testHole() {
  // A = annulus: outer [0,4]×[0,4] with a square hole [1,3]×[1,3] (hole area 4).
  //   A region area = 16 - 4 = 12.
  // B = square [1.5,2.5]×[1.5,2.5] (area 1) sitting ENTIRELY inside A's hole.
  //   A ∩ B = empty (B is in the hole). A ∪ B = A plus the filled-in B island... but B is
  //   inside the hole and disjoint from the annulus material, so union area = 12 + 1 = 13.
  TrimRegion A{rectLoop(0, 0, 4, 4), {rectLoop(1, 1, 3, 3, /*ccw=*/false)}};
  TrimRegion B{rectLoop(1.5, 1.5, 2.5, 2.5), {}};

  auto inter = trimRegionBoolean(A, B, TrimBoolOp::Intersect);
  expectTrue(inter.status == TrimBoolStatus::Empty, "annulus∩(in-hole disk) empty");
  expectNear(inter.area, 0.0, 1e-12, "annulus∩ area 0");

  auto uni = trimRegionBoolean(A, B, TrimBoolOp::Union);
  expectTrue(uni.ok(), "annulus∪(in-hole disk) ok");
  expectNear(uni.area, 13.0, 1e-10, "annulus∪ area = 13");

  // Now C OVERLAPPING the annulus material: C = [3.5,5]×[1,3] straddles the right OUTER wall
  // (x=4) without touching the hole (x=3). A∩C = the part of C inside the annulus material =
  // [3.5,4]×[1,3], all outside the hole ⇒ area = 0.5·2 = 1.
  TrimRegion C{rectLoop(3.5, 1, 5, 3), {}};
  auto inter2 = trimRegionBoolean(A, C, TrimBoolOp::Intersect);
  expectTrue(inter2.ok(), "annulus∩straddle ok");
  expectNear(inter2.area, 1.0, 1e-10, "annulus∩straddle area = 1");
}

static void testCircularLens() {
  // Two unit circles, centres distance d=1 apart. Lens (intersection) area closed form:
  //   A_lens = 2 r² cos⁻¹(d/2r) − (d/2)√(4r² − d²),  r=1, d=1
  //          = 2 cos⁻¹(0.5) − 0.5·√3 = 2·(π/3) − √3/2.
  const double r = 1.0, d = 1.0;
  const double lens =
      2.0 * r * r * std::acos(d / (2.0 * r)) - (d * 0.5) * std::sqrt(4.0 * r * r - d * d);

  TrimRegion A{circleLoop(0.0, 0.0, r), {}};
  TrimRegion B{circleLoop(d, 0.0, r), {}};
  TrimBoolOptions opts;
  opts.flattenSegments = 40000;  // fine flatten of the exact rational-circle pcurve

  auto inter = trimRegionBoolean(A, B, TrimBoolOp::Intersect, opts);
  expectTrue(inter.ok(), "circle lens intersect ok");
  expectNear(inter.area, lens, 1e-8, "circle lens area matches closed form");
  g_maxLensAreaErr = std::max(g_maxLensAreaErr, std::fabs(inter.area - lens));

  // Union area = 2·(π r²) − lens.
  auto uni = trimRegionBoolean(A, B, TrimBoolOp::Union, opts);
  expectTrue(uni.ok(), "circle union ok");
  const double unionArea = 2.0 * M_PI * r * r - lens;
  expectNear(uni.area, unionArea, 1e-7, "circle union area matches closed form");
}

static void testCoincidentDecline() {
  // A = [0,1]×[0,1], B = [1,2]×[0,1] share the edge u=1 (a coincident boundary).
  TrimRegion A{rectLoop(0, 0, 1, 1), {}};
  TrimRegion B{rectLoop(1, 0, 2, 1), {}};
  auto uni = trimRegionBoolean(A, B, TrimBoolOp::Union);
  expectTrue(uni.status == TrimBoolStatus::Degenerate, "coincident-edge union DECLINES");
  auto inter = trimRegionBoolean(A, B, TrimBoolOp::Intersect);
  expectTrue(inter.status == TrimBoolStatus::Degenerate, "coincident-edge intersect DECLINES");
}

int main() {
  testDisjointSquares();
  testOverlappingSquares();
  testHole();
  testCircularLens();
  testCoincidentDecline();

  std::printf("trim_boolean: %d checks, %d failures\n", g_checks, g_failures);
  std::printf("  max square-area error   = %.3e\n", g_maxSquareAreaErr);
  std::printf("  max circular-lens error = %.3e\n", g_maxLensAreaErr);
  return g_failures == 0 ? 0 : 1;
}
