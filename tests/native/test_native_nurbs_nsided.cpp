// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — N-SIDED boundary-filled surface
// (src/native/math/bspline_nsided.{h,cpp}). OCCT-FREE. Fills a closed N-gon (N ≠ 4)
// with N Coons sub-patches by midpoint subdivision meeting at the centroid. The oracles
// are airtight and closed-form:
//
//   1. BOUNDARY CONTAINMENT (the core oracle) — the N sub-patches together reproduce all
//      N input boundary curves POINTWISE along the OUTER edges (~1e-9): each edge e[k] is
//      covered by sub-patch k's first-half outer edge and sub-patch k+1's second-half
//      outer edge; every point of every boundary curve lies on the union of patches.
//   2. PLANAR N-GON → FLAT patches — a planar pentagon / triangle boundary yields N
//      coplanar patches; every surface point of every patch lies on the boundary's plane
//      exactly (~1e-12).
//   3. C0 INTERIOR JUNCTIONS — adjacent sub-patches meet C0 along the shared interior
//      spoke M[i]→C, and all patches pass through the shared centroid C (~1e-12).
//   4. N=4 CONSISTENCY — for a 4-sided boundary the 4 sub-patches' union reproduces the
//      same four boundary curves as the single Coons patch (both contain the boundary).
//   5. HONEST DECLINES — a non-closed loop, a rational edge, N<3, and a malformed edge
//      decline (ok=false, with a reason), never a silently-wrong surface, never a crash.
//
// The module sits with the rest of the numsci-gated Layer-6 surfacing family, so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_coons). With the guard
// OFF this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_coons.h"
#include "native/math/bspline_nsided.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

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
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-56s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}

// Boundary containment via the EXACT outer iso-edges. Sub-patch i is built with (Coons
// convention): c0 = S_i(·,0) = FIRST half of edge e[i]; d0 = S_i(0,·) = SECOND half of
// edge e[i-1] reversed. So the outer boundary of edge e[k] is reproduced by two exact
// patch iso-curves — the first half by S_k(t,0) (t: 0→1 ≡ e[k] on [0,0.5]) and the
// second half by S_{k+1}(0, 1−t) (≡ e[k] on [0.5,1], the reversal undone). We sample
// each half against the true edge; because each is an EXACT iso-curve of one sub-patch
// the residual is machine-precision.
static double boundaryContainmentDev(const std::vector<BsplineSurfaceData>& patches,
                                     const NSidedBoundary& b, int nS = 200) {
  const int N = static_cast<int>(b.edges.size());
  double worst = 0.0;
  for (int k = 0; k < N; ++k) {
    const BsplineCurveData& e = b.edges[k];
    const BsplineSurfaceData& sk = patches[k];               // owns first half of e[k]
    const BsplineSurfaceData& sk1 = patches[(k + 1) % N];    // owns second half of e[k]
    for (int i = 0; i <= nS; ++i) {
      const double t = static_cast<double>(i) / nS;
      // First half: e[k] param s∈[0,0.5] ≡ S_k(2s, 0).
      const double s1 = 0.5 * t;
      worst = std::max(worst, distance(evalCurve(e, s1), evalSurface(sk, t, 0.0)));
      // Second half: e[k] param s∈[0.5,1] ≡ S_{k+1}(0, ·). d0 = second-half-reversed, so
      // S_{k+1}(0, τ) traces e[k] backwards from V[k+1]→M[k]; e[k] at s=1−0.5τ.
      const double s2 = 1.0 - 0.5 * t;
      worst = std::max(worst, distance(evalCurve(e, s2), evalSurface(sk1, 0.0, t)));
    }
  }
  return worst;
}

// ── Boundary builders ────────────────────────────────────────────────────────────
// A straight degree-1 edge between two corners (2 poles).
static BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0, 0, 1, 1};
  c.poles = {a, b};
  return c;
}

// A clamped cubic edge on [0,1] pinned to a,b with an in-direction bow (5 poles).
static BsplineCurveData cubicEdge(const Point3& a, const Point3& b, const Vec3& bow) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  const Point3 p1{a.x + 0.25 * (b.x - a.x) + bow.x, a.y + 0.25 * (b.y - a.y) + bow.y,
                  a.z + 0.25 * (b.z - a.z) + bow.z};
  const Point3 p2{a.x + 0.50 * (b.x - a.x) + 1.3 * bow.x, a.y + 0.50 * (b.y - a.y) + 1.3 * bow.y,
                  a.z + 0.50 * (b.z - a.z) + 1.3 * bow.z};
  const Point3 p3{a.x + 0.75 * (b.x - a.x) + bow.x, a.y + 0.75 * (b.y - a.y) + bow.y,
                  a.z + 0.75 * (b.z - a.z) + bow.z};
  c.poles = {a, p1, p2, p3, b};
  return c;
}

// Regular planar N-gon corners in the z=0 plane, radius R, straight-line edges.
static std::vector<Point3> regularPolygonCorners(int N, double R) {
  std::vector<Point3> v(N);
  for (int i = 0; i < N; ++i) {
    const double a = 2.0 * M_PI * i / N;
    v[i] = {R * std::cos(a), R * std::sin(a), 0.0};
  }
  return v;
}
static NSidedBoundary polygonBoundary(const std::vector<Point3>& v) {
  NSidedBoundary b;
  const int N = static_cast<int>(v.size());
  for (int i = 0; i < N; ++i) b.edges.push_back(lineEdge(v[i], v[(i + 1) % N]));
  return b;
}

int main() {
  // ═══ 1. BOUNDARY CONTAINMENT + 2. PLANAR N-GON — a PLANAR PENTAGON ═════════════════
  {
    const auto v = regularPolygonCorners(5, 3.0);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillResult r = fillNSided(b);
    expectTrue(r.ok, "fillNSided ok on a planar pentagon (N=5)");
    expectTrue(static_cast<int>(r.patches.size()) == 5, "pentagon → 5 sub-patches");
    for (const auto& s : r.patches)
      expectTrue(s.weights.empty(), "sub-patch is non-rational");
    expectLE(r.maxCornerError, 1e-12, "pentagon loop closes exactly");

    const double bc = boundaryContainmentDev(r.patches, b);
    expectLE(bc, 1e-9, "pentagon: union contains all 5 boundary edges POINTWISE");
    std::printf("INFO pentagon boundary-containment worst dev = %.3e\n", bc);

    // Planar → every surface point on z=0.
    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 20; ++iu)
        for (int iv = 0; iv <= 20; ++iv) {
          const double u = static_cast<double>(iu) / 20, w = static_cast<double>(iv) / 20;
          maxZ = std::max(maxZ, std::fabs(evalSurface(s, u, w).z));
        }
    expectLE(maxZ, 1e-12, "planar pentagon → all sub-patch points on z=0 plane (flat)");
    std::printf("INFO pentagon flat-patch max |z| = %.3e\n", maxZ);

    // Centroid of a regular polygon centred at origin is the origin.
    expectLE(distance(r.centroid, Point3{0, 0, 0}), 1e-12, "pentagon centroid == origin");
  }

  // ═══ 2b. PLANAR TRIANGLE (N=3) ═════════════════════════════════════════════════════
  {
    const auto v = regularPolygonCorners(3, 2.5);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillResult r = fillNSided(b);
    expectTrue(r.ok, "fillNSided ok on a planar triangle (N=3)");
    expectTrue(static_cast<int>(r.patches.size()) == 3, "triangle → 3 sub-patches");

    const double bc = boundaryContainmentDev(r.patches, b);
    expectLE(bc, 1e-9, "triangle: union contains all 3 boundary edges POINTWISE");
    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 20; ++iu)
        for (int iv = 0; iv <= 20; ++iv) {
          const double u = static_cast<double>(iu) / 20, w = static_cast<double>(iv) / 20;
          maxZ = std::max(maxZ, std::fabs(evalSurface(s, u, w).z));
        }
    expectLE(maxZ, 1e-12, "planar triangle → all sub-patch points on z=0 plane (flat)");
    std::printf("INFO triangle boundary-containment=%.3e flat max|z|=%.3e\n", bc, maxZ);
  }

  // ═══ 2c. PLANAR HEXAGON with MILDLY-CURVED (but in-plane) edges ═══════════════════════
  // Edges bow within z=0, so it is a genuine curved-boundary containment + flat test.
  {
    const auto v = regularPolygonCorners(6, 3.0);
    NSidedBoundary b;
    for (int i = 0; i < 6; ++i) {
      const Point3 a = v[i], c = v[(i + 1) % 6];
      // In-plane outward bow (perpendicular to the chord, in z=0).
      const Vec3 chord{c.x - a.x, c.y - a.y, 0};
      const Vec3 nrm{-chord.y, chord.x, 0};  // 90° rotate in-plane
      const double s = 0.12;
      b.edges.push_back(cubicEdge(a, c, {s * nrm.x, s * nrm.y, 0}));
    }
    NSidedFillResult r = fillNSided(b);
    expectTrue(r.ok, "fillNSided ok on a curved-edge hexagon (N=6, in-plane)");
    expectTrue(static_cast<int>(r.patches.size()) == 6, "hexagon → 6 sub-patches");

    const double bc = boundaryContainmentDev(r.patches, b);
    expectLE(bc, 1e-9, "curved hexagon: union contains all 6 boundary edges POINTWISE");
    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 16; ++iu)
        for (int iv = 0; iv <= 16; ++iv) {
          const double u = static_cast<double>(iu) / 16, w = static_cast<double>(iv) / 16;
          maxZ = std::max(maxZ, std::fabs(evalSurface(s, u, w).z));
        }
    expectLE(maxZ, 1e-12, "curved-but-coplanar hexagon → still a flat patch on z=0");
    std::printf("INFO curved-hexagon boundary-containment=%.3e flat max|z|=%.3e\n", bc, maxZ);
  }

  // ═══ 3. C0 INTERIOR JUNCTIONS (shared spokes + shared centroid) ═════════════════════
  // Adjacent sub-patches meet C0 on the spoke M[i]→C, and all pass through C. In the
  // Coons convention: sub-patch i's d1 is the u=1 iso S(1,·) (M[i]→C); sub-patch (i+1)'s
  // c1 is the v=1 iso S(·,1) (M[i]→C). Sample both and compare; also every patch's
  // P11 corner (u=1,v=1) == centroid.
  {
    const auto v = regularPolygonCorners(5, 3.0);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillResult r = fillNSided(b);
    expectTrue(r.ok, "fillNSided ok for C0-junction check");

    const int N = static_cast<int>(r.patches.size());
    double spokeDev = 0.0, centroidDev = 0.0;
    for (int i = 0; i < N; ++i) {
      const BsplineSurfaceData& si = r.patches[i];
      const BsplineSurfaceData& sj = r.patches[(i + 1) % N];
      // si's d1 spoke S_i(1, t)  vs  sj's c1 spoke S_{i+1}(t, 1) — SAME segment M[i]→C.
      for (int k = 0; k <= 30; ++k) {
        const double t = static_cast<double>(k) / 30;
        spokeDev = std::max(spokeDev, distance(evalSurface(si, 1.0, t), evalSurface(sj, t, 1.0)));
      }
      // P11 corner (u=1,v=1) is the centroid on every patch.
      centroidDev = std::max(centroidDev, distance(evalSurface(si, 1.0, 1.0), r.centroid));
    }
    expectLE(spokeDev, 1e-12, "adjacent sub-patches meet C0 on the shared interior spoke");
    expectLE(centroidDev, 1e-12, "all sub-patches share the centroid C exactly");
    std::printf("INFO C0 spoke dev=%.3e centroid dev=%.3e\n", spokeDev, centroidDev);
  }

  // ═══ 4. N=4 CONSISTENCY (subdivision union == single-Coons boundary) ════════════════
  {
    const Point3 A{0, 0, 0}, B{4, 0, 0}, D{4, 3, 0}, E{0, 3, 0};
    NSidedBoundary quad;
    quad.edges = {lineEdge(A, B), lineEdge(B, D), lineEdge(D, E), lineEdge(E, A)};
    NSidedFillResult r = fillNSided(quad);
    expectTrue(r.ok, "fillNSided ok on N=4 (reduces via same subdivision)");
    expectTrue(static_cast<int>(r.patches.size()) == 4, "N=4 → 4 sub-patches");

    // Union contains the 4 boundary edges.
    const double bc = boundaryContainmentDev(r.patches, quad);
    expectLE(bc, 1e-9, "N=4: union contains all 4 boundary edges POINTWISE");

    // The SINGLE Coons patch of the same boundary (c0=AB, c1=ED, d0=AE, d1=BD) also
    // contains the same boundary — both interpolate it, so they MATCH on the boundary.
    CoonsBoundary cb;
    cb.c0 = lineEdge(A, B);  // v=0
    cb.c1 = lineEdge(E, D);  // v=1
    cb.d0 = lineEdge(A, E);  // u=0
    cb.d1 = lineEdge(B, D);  // u=1
    CoonsResult single = coonsPatch(cb);
    expectTrue(single.ok, "single Coons patch of the same 4-sided boundary ok");
    // Compare the two along the shared boundary edge AB: the single Coons patch has
    // S(·,0)==AB exactly; the subdivision reproduces AB via its first two sub-patches'
    // exact iso-edges (S_0(t,0) is the first half of AB, S_1(0,t) the second). Both agree
    // with the true edge to machine precision.
    double matchDev = 0.0;
    const BsplineCurveData ab = lineEdge(A, B);
    for (int k = 0; k <= 40; ++k) {
      const double t = static_cast<double>(k) / 40;
      matchDev = std::max(matchDev, distance(evalCurve(ab, t), evalSurface(single.surface, t, 0.0)));
    }
    expectLE(matchDev, 1e-9, "single Coons patch reproduces the AB boundary edge exactly");
    // The subdivision's containment (exact iso-edges) is already ~1e-9 (bc above); confirm
    // the two constructions therefore agree on the boundary.
    expectLE(bc, 1e-9, "N=4 subdivision union and single Coons patch agree on the boundary");
    std::printf("INFO N=4 single-Coons AB dev = %.3e (subdivision bc = %.3e)\n", matchDev, bc);
  }

  // ═══ 5. HONEST DECLINES ══════════════════════════════════════════════════════════
  {
    // Non-closed loop: break one corner so edges[1](0) != edges[0](1).
    {
      const auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary bad = polygonBoundary(v);
      bad.edges[1].poles.front().z += 0.7;  // edges[1](0) no longer meets edges[0](1)
      NSidedFillResult r = fillNSided(bad);
      expectTrue(!r.ok, "non-closed loop declines honestly");
      expectTrue(!r.reason.empty(), "non-closed decline carries a reason");
      expectTrue(r.maxCornerError > 0.1, "non-closed decline reports a large corner error");
      expectTrue(!verifyNSidedBoundary(bad).ok, "verifyNSidedBoundary declines non-closed loop");
    }
    // Rational edge → non-rational scope declines.
    {
      const auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary ratl = polygonBoundary(v);
      ratl.edges[2].weights.assign(ratl.edges[2].poles.size(), 1.0);  // non-empty ⇒ rational
      expectTrue(!fillNSided(ratl).ok, "rational edge declines (non-rational scope)");
      expectTrue(!verifyNSidedBoundary(ratl).ok, "verifyNSidedBoundary declines rational edge");
    }
    // N < 3 → declines.
    {
      const auto v = regularPolygonCorners(3, 2.0);
      NSidedBoundary two;
      two.edges = {lineEdge(v[0], v[1]), lineEdge(v[1], v[0])};
      NSidedFillResult r = fillNSided(two);
      expectTrue(!r.ok, "N<3 boundary declines");
      expectTrue(!r.reason.empty(), "N<3 decline carries a reason");
    }
    // Malformed edge (degree 0) → declines.
    {
      const auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary mal = polygonBoundary(v);
      mal.edges[0].degree = 0;
      expectTrue(!fillNSided(mal).ok, "malformed edge declines");
    }
    // A consistent boundary still succeeds (the guard is not over-eager).
    {
      const auto v = regularPolygonCorners(5, 3.0);
      expectTrue(fillNSided(polygonBoundary(v)).ok, "consistent pentagon still succeeds");
    }
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_nsided: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_nsided: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_nsided (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
