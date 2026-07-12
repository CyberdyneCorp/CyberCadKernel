// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — surface G1/G2 CONTINUITY JOIN across a shared
// edge (src/native/math/bspline_join.{h,cpp}). OCCT-FREE. Given two already-built adjacent
// tensor-product NURBS patches sharing a boundary edge (C0), reposition the near-boundary
// control rows so they meet G1 (tangent-plane) or G2 (curvature) with minimal movement, the
// shared boundary curve frozen. Airtight, closed-form oracles:
//
//   1. COPLANAR NO-OP — two co-planar patches already meeting G-infinity → join is a no-op
//      (maxMovement == 0, residual already below tol).
//   2. G1 ENFORCED — two patches meeting only C0 (a crease) → after joinG1 the unit normal is
//      continuous across the shared edge at sampled stations (≤ 1e-7 rad) and the boundary
//      curve is unchanged (≤ 1e-12).
//   3. G2 ENFORCED — after joinG2 the normal curvature is continuous (relative ≤ 1e-5) AND G1
//      still holds; boundary unchanged.
//   4. ANALYTIC CYLINDER — two halves of a cylinder split along a knot line already meet G2 →
//      join is a no-op (movement 0). An over-cap demand HONEST-DECLINES.
//
// Under CYBERCAD_HAS_NUMSCI (like the rest of the Layer-6 surfacing gates). Guard OFF ⇒ pass.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_join.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;
static void fail(const char* what) { std::printf("FAIL %s\n", what); ++g_failures; }
static void expectTrue(bool c, const char* what) { ++g_checks; if (!c) fail(what); }
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) { std::printf("FAIL %-56s %.6g <= %.6g violated\n", what, a, b); ++g_failures; }
}

// ── Evaluators ──────────────────────────────────────────────────────────────────────
static Dir3 surfNormal(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfaceNormal(s.degreeU, s.degreeV, g, {}, s.knotsU, s.knotsV, u, v);
}
static Point3 surfPt(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}
static double normalMismatch(const Dir3& a, const Dir3& b) {
  const double ang = a.angle(b);
  return std::min(ang, M_PI - ang);
}

// ── Surface builders ────────────────────────────────────────────────────────────────
// A bicubic (degree 3×3) B-spline patch from a 4x4 control net on a clamped [0,1]² domain.
static BsplineSurfaceData bicubic(const std::vector<Point3>& net16) {
  BsplineSurfaceData s;
  s.degreeU = 3; s.degreeV = 3; s.nPolesU = 4; s.nPolesV = 4;
  s.poles = net16;
  s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};
  return s;
}

// ── 1. COPLANAR NO-OP ─────────────────────────────────────────────────────────────────
// Two flat 4x4 patches in the z=0 plane, meeting along x=1 (A's V1 col? we use A's U1 row).
// A spans x in [0,1], B spans x in [1,2]; both flat. They already meet G-infinity → no-op.
static BsplineSurfaceData flatPatch(double x0, double x1, double y0, double y1) {
  std::vector<Point3> net(16);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      net[i * 4 + j] = {x0 + (x1 - x0) * i / 3.0, y0 + (y1 - y0) * j / 3.0, 0.0};
  return bicubic(net);
}

// ── 2/3. CREASE PATCHES ───────────────────────────────────────────────────────────────
// A: flat in z=0, x∈[0,1]. B: shares the x=1 edge but tilts UP (a crease). We join along
// A's U1 (x=1) row and B's U0 (x=1) row. Along-edge is the V (=y) direction, same orientation.
static BsplineSurfaceData creaseA() {
  std::vector<Point3> net(16);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      net[i * 4 + j] = {static_cast<double>(i) / 3.0, static_cast<double>(j) / 3.0, 0.0};
  return bicubic(net);
}
static BsplineSurfaceData creaseB() {
  // B's U0 row (i=0) must coincide with A's U1 row (i=3): x=1, y=j/3, z=0.
  // Interior rows rise in z (a tilted-up flap) → a crease at the shared edge.
  std::vector<Point3> net(16);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double x = 1.0 + static_cast<double>(i) / 3.0;
      const double y = static_cast<double>(j) / 3.0;
      const double z = 0.6 * static_cast<double>(i) / 3.0;  // rises with i → tilt up
      net[i * 4 + j] = {x, y, z};
    }
  return bicubic(net);
}

// Max unit-normal mismatch across the A.U1 / B.U0 shared edge, sampled along y.
static double edgeNormalMismatch(const BsplineSurfaceData& A, const BsplineSurfaceData& B, int nS = 40) {
  double worst = 0.0;
  for (int k = 0; k <= nS; ++k) {
    const double t = static_cast<double>(k) / nS;
    const Dir3 nA = surfNormal(A, 1.0, t);   // A's U1 edge is u=1
    const Dir3 nB = surfNormal(B, 0.0, t);   // B's U0 edge is u=0
    if (nA.valid() && nB.valid()) worst = std::max(worst, normalMismatch(nA, nB));
  }
  return worst;
}
// Max boundary-curve displacement across the shared edge (must stay ~0 after a join).
static double edgeBoundaryDev(const BsplineSurfaceData& A, const BsplineSurfaceData& B, int nS = 40) {
  double worst = 0.0;
  for (int k = 0; k <= nS; ++k) {
    const double t = static_cast<double>(k) / nS;
    worst = std::max(worst, distance(surfPt(A, 1.0, t), surfPt(B, 0.0, t)));
  }
  return worst;
}
// Normal-curvature (cross-boundary, u-direction) at the shared edge; relative mismatch.
static double edgeCurvatureRel(const BsplineSurfaceData& A, const BsplineSurfaceData& B, int nS = 40) {
  auto kappa = [](const BsplineSurfaceData& s, double u, double v) {
    SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
    std::vector<Vec3> d(9);
    surfaceDerivs(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v, 2, d);
    const Vec3 Su = d[3], Sv = d[1], Suu = d[6];
    const Vec3 nrm = cross(Su, Sv);
    if (isNull(nrm, 1e-14)) return 0.0;
    const Dir3 n = Dir3(nrm);
    return dot(Suu, n.vec()) / std::max(dot(Su, Su), 1e-30);
  };
  double worst = 0.0;
  for (int k = 0; k <= nS; ++k) {
    const double t = static_cast<double>(k) / nS;
    const double ka = kappa(A, 1.0, t), kb = kappa(B, 0.0, t);
    const double sc = std::max({std::abs(ka), std::abs(kb), 1e-9});
    worst = std::max(worst, std::abs(ka - kb) / sc);
  }
  return worst;
}

// ── 4. CYLINDER HALVES ────────────────────────────────────────────────────────────────
// Two halves of one smooth surface, obtained by splitting a single clamped cubic-in-u B-spline
// wall at its interior knot line u=0.5 into two Bézier segments via splitSurface. Because both
// halves are the SAME C-infinity surface restricted to complementary spans, they already meet
// G2 across the split line → any join must be a no-op. We RE-CLAMP the split output to the
// canonical clamped [0,1] representation the join API expects (splitSurface may leave the high
// piece on its own [0.5,1] sub-domain / with a non-normalised pole count).
static BsplineSurfaceData reclampBezierU(const BsplineSurfaceData& seg) {
  // Reduce to the pure clamped cubic Bézier on [0,1] (4 U-poles). We rebuild from 4 evaluated
  // Bézier poles: for a clamped cubic Bézier the poles ARE the control net, so just take the
  // first/last-clamped 4 U-poles when the segment is already a single Bézier span; otherwise
  // keep as-is. Here every split segment is a single cubic Bézier span in u.
  BsplineSurfaceData s = seg;
  // Normalise the U knot domain to [0,1].
  const double a = s.knotsU.front(), b = s.knotsU.back();
  for (double& k : s.knotsU) k = (k - a) / (b - a);
  return s;
}
static BsplineSurfaceData cylinderFull() {
  // Degree 3 in u (arc-like), degree 1 in v (straight axis), 5 poles in u (one interior knot
  // at 0.5), 2 in v. Poles trace a smooth curve in the xz-plane, extruded along y.
  BsplineSurfaceData s;
  s.degreeU = 3; s.degreeV = 1; s.nPolesU = 5; s.nPolesV = 2;
  s.knotsU = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  const double R = 2.0;
  const double ang[5] = {0.0, 0.25, 0.5, 0.8, 1.1};
  s.poles.resize(10);
  for (int i = 0; i < 5; ++i) {
    const double x = R * std::cos(ang[i]);
    const double z = R * std::sin(ang[i]);
    s.poles[i * 2 + 0] = {x, 0.0, z};
    s.poles[i * 2 + 1] = {x, 3.0, z};
  }
  return s;
}

int main() {
  // ═══ 1. COPLANAR NO-OP ════════════════════════════════════════════════════════════════
  {
    BsplineSurfaceData A = flatPatch(0, 1, 0, 1);   // x∈[0,1]
    BsplineSurfaceData B = flatPatch(1, 2, 0, 1);   // x∈[1,2], shares x=1 edge
    EdgeSpec e; e.edgeA = SurfaceEdge::U1; e.edgeB = SurfaceEdge::U0; e.reversed = false;

    JoinResult r1 = joinG1(A, B, e);
    expectTrue(r1.ok, "coplanar joinG1 ok");
    expectTrue(r1.noop, "coplanar joinG1 is a no-op");
    expectLE(r1.maxMovement, 0.0, "coplanar joinG1 moves nothing");
    expectLE(r1.continuityResidual, 1e-7, "coplanar joinG1 residual already G1");

    JoinResult r2 = joinG2(A, B, e);
    expectTrue(r2.ok && r2.noop, "coplanar joinG2 is a no-op");
    expectLE(r2.maxMovement, 0.0, "coplanar joinG2 moves nothing");
  }

  // ═══ 2. G1 ENFORCED across a crease ═══════════════════════════════════════════════════
  {
    BsplineSurfaceData A = creaseA();
    BsplineSurfaceData B = creaseB();
    EdgeSpec e; e.edgeA = SurfaceEdge::U1; e.edgeB = SurfaceEdge::U0; e.reversed = false;

    const double creaseAng = edgeNormalMismatch(A, B);
    expectTrue(creaseAng > 1e-3, "crease patches start NON-G1 (a real crease to fix)");

    JoinResult r = joinG1(A, B, e);
    expectTrue(r.ok, "joinG1 succeeds on the crease");
    expectTrue(!r.noop, "joinG1 actually moved poles (was a crease)");
    expectLE(edgeNormalMismatch(r.A, r.B), 1e-7, "joinG1 → unit normal continuous (≤1e-7 rad)");
    expectLE(r.continuityResidual, 1e-7, "joinG1 reported residual ≤1e-7");
    expectLE(edgeBoundaryDev(r.A, r.B), 1e-12, "joinG1 boundary curve unchanged (≤1e-12)");
    expectLE(r.boundaryDev, 1e-12, "joinG1 reported boundary dev ≤1e-12");
  }

  // ═══ 3. G2 ENFORCED across a crease (curvature continuous + G1 holds) ═════════════════
  {
    BsplineSurfaceData A = creaseA();
    BsplineSurfaceData B = creaseB();
    EdgeSpec e; e.edgeA = SurfaceEdge::U1; e.edgeB = SurfaceEdge::U0; e.reversed = false;

    JoinResult r = joinG2(A, B, e);
    expectTrue(r.ok, "joinG2 succeeds on the crease");
    expectLE(edgeNormalMismatch(r.A, r.B), 1e-7, "joinG2 keeps G1 (normal continuous)");
    expectLE(edgeCurvatureRel(r.A, r.B), 1e-5, "joinG2 → normal curvature continuous (rel ≤1e-5)");
    expectLE(r.continuityResidual, 1e-5, "joinG2 reported curvature residual ≤1e-5");
    expectLE(edgeBoundaryDev(r.A, r.B), 1e-12, "joinG2 boundary curve unchanged (≤1e-12)");
  }

  // ═══ 4. CYLINDER HALVES no-op + over-cap decline ══════════════════════════════════════
  {
    const BsplineSurfaceData full = cylinderFull();
    SurfaceSplit sp = splitSurface(full, ParamDir::U, 0.5);
    BsplineSurfaceData A = reclampBezierU(sp.low), B = reclampBezierU(sp.high);  // split line = shared edge
    EdgeSpec e; e.edgeA = SurfaceEdge::U1; e.edgeB = SurfaceEdge::U0; e.reversed = false;

    // The split halves already meet C-infinity → both joins are no-ops.
    JoinResult r1 = joinG1(A, B, e);
    expectTrue(r1.ok && r1.noop, "cylinder-split joinG1 is a no-op");
    expectLE(r1.maxMovement, 0.0, "cylinder-split joinG1 moves nothing");

    JoinResult r2 = joinG2(A, B, e);
    expectTrue(r2.ok && r2.noop, "cylinder-split joinG2 is a no-op");
    expectLE(r2.maxMovement, 0.0, "cylinder-split joinG2 moves nothing");

    // Over-cap: enforce G1 on the crease with a punishingly small cap → HONEST DECLINE.
    BsplineSurfaceData ca = creaseA(), cb = creaseB();
    JoinResult over = joinG1(ca, cb, e, /*maxMovementCap=*/1e-6);
    expectTrue(!over.ok, "over-cap joinG1 honest-declines");
    expectTrue(over.maxMovement > 1e-6, "over-cap decline reports the required movement");
    expectTrue(!over.reason.empty(), "over-cap decline carries a reason");
  }

  std::printf("test_native_nurbs_join: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else   // CYBERCAD_HAS_NUMSCI not defined

int main() {
  std::printf("test_native_nurbs_join: CYBERCAD_HAS_NUMSCI off — trivially pass\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
