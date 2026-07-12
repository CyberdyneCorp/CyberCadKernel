// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — RATIONAL N-SIDED boundary-filled surface
// (src/native/math/bspline_nsided.{h,cpp}, fillNSidedRational). OCCT-FREE. The additive
// rational analogue of fillNSided: the entire midpoint subdivision + per-corner Coons
// boolean-sum runs in HOMOGENEOUS (wx, wy, wz, w) space so a RATIONAL boundary curve (an
// exact circular arc) is reproduced EXACTLY, not approximated polynomially. Oracles:
//
//   1. RATIONAL BOUNDARY EXACTNESS (the core oracle) — an N-gon whose edges are exact
//      rational quarter-circle arcs (a rounded frame): each rational boundary edge is
//      reproduced EXACTLY by the sub-patch outer iso-curves (≤1e-12), measured against the
//      true rational arc, NOT a polynomial fit. A degree-2 polynomial CANNOT trace a circle,
//      so a large residual here would prove the fill fell back to a polynomial — it does not.
//   2. NON-RATIONAL REDUCTION — a boundary with all weights 1 (or empty) reproduces the
//      existing fillNSided result POINTWISE (≤1e-12).
//   3. PLANAR RATIONAL CONTAINMENT — rational arcs in the z=0 plane yield sub-patch points
//      ON that plane (≤1e-10).
//   4. HONEST DECLINES — non-closed loop, non-positive weight, N<3, malformed edge decline
//      (ok=false with a reason), never a silently-wrong surface, never a crash.
//
// Under CYBERCAD_HAS_NUMSCI (like the C0 nsided gate). Guard OFF → trivial pass.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
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
static Point3 evalCurveR(const BsplineCurveData& c, double u) {
  if (c.weights.empty()) return curvePoint(c.degree, c.poles, c.knots, u);
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
static Point3 evalSurfaceR(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

// Boundary containment: sub-patch k owns the FIRST half of edge e[k] as its c0 = S_k(t,0),
// and sub-patch k+1 owns the SECOND half reversed as its d0 = S_{k+1}(0,t). Sample each half
// against the TRUE (rational) edge. Because each is an exact iso-curve of one homogeneous
// Coons sub-patch, the residual is machine-precision even for a genuine circular arc.
static double boundaryContainmentDev(const std::vector<BsplineSurfaceData>& patches,
                                     const NSidedBoundary& b, int nS = 200) {
  const int N = static_cast<int>(b.edges.size());
  double worst = 0.0;
  for (int k = 0; k < N; ++k) {
    const BsplineCurveData& e = b.edges[k];
    const BsplineSurfaceData& sk = patches[k];
    const BsplineSurfaceData& sk1 = patches[(k + 1) % N];
    for (int i = 0; i <= nS; ++i) {
      const double t = static_cast<double>(i) / nS;
      const double s1 = 0.5 * t;                 // e[k] on [0,0.5] ≡ S_k(t,0)
      worst = std::max(worst, distance(evalCurveR(e, s1), evalSurfaceR(sk, t, 0.0)));
      const double s2 = 1.0 - 0.5 * t;           // e[k] on [0.5,1] ≡ S_{k+1}(0,t)
      worst = std::max(worst, distance(evalCurveR(e, s2), evalSurfaceR(sk1, 0.0, t)));
    }
  }
  return worst;
}

// ── Boundary builders ────────────────────────────────────────────────────────────
static BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0, 0, 1, 1};
  c.poles = {a, b};
  return c;
}

// An exact rational quarter-circle arc, centre `ctr`, from angle a0 to a0+90° (CCW), radius R,
// in the z=0 plane. Standard NURBS circular arc: degree 2, 3 poles, middle weight cos(45°).
static BsplineCurveData quarterArc(const Point3& ctr, double R, double a0) {
  const double a1 = a0 + M_PI / 2.0;
  const Point3 p0{ctr.x + R * std::cos(a0), ctr.y + R * std::sin(a0), ctr.z};
  const Point3 p2{ctr.x + R * std::cos(a1), ctr.y + R * std::sin(a1), ctr.z};
  // Middle control point = intersection of the two end tangents (apex of the bounding wedge).
  const double am = a0 + M_PI / 4.0;
  const double w = std::cos(M_PI / 4.0);         // = 1/sqrt(2)
  // Apex lies along the bisector at distance R/cos(45°) from the centre.
  const Point3 p1{ctr.x + (R / w) * std::cos(am), ctr.y + (R / w) * std::sin(am), ctr.z};
  BsplineCurveData c;
  c.degree = 2;
  c.knots = {0, 0, 0, 1, 1, 1};
  c.poles = {p0, p1, p2};
  c.weights = {1.0, w, 1.0};
  return c;
}

// Regular planar N-gon corners in the z=0 plane.
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
  // ═══ 1. RATIONAL BOUNDARY EXACTNESS — a rounded frame of 4 quarter-circle arcs ═══════
  // Four exact quarter-circles centred on the origin, each spanning one 90° sector, form a
  // closed circle (a topological 4-gon whose edges are genuine rational arcs). The fill must
  // reproduce each arc EXACTLY. Because a polynomial of any degree cannot trace a circular
  // arc, a residual near machine-eps here proves the construction stayed rational throughout.
  {
    const Point3 O{0, 0, 0};
    const double R = 2.0;
    NSidedBoundary b;
    for (int k = 0; k < 4; ++k) b.edges.push_back(quarterArc(O, R, k * M_PI / 2.0));

    const NSidedFillRationalResult r = fillNSidedRational(b);
    expectTrue(r.ok, "fillNSidedRational ok on a 4-arc rounded frame");
    expectTrue(static_cast<int>(r.patches.size()) == 4, "4 arcs → 4 sub-patches");
    expectLE(r.maxCornerError, 1e-12, "arc frame loop closes exactly");

    // Confirm at least one sub-patch is genuinely rational (carries weights ≠ 1).
    bool anyRational = false;
    for (const auto& s : r.patches)
      if (!s.weights.empty()) anyRational = true;
    expectTrue(anyRational, "rational-arc fill produces genuinely rational sub-patches");

    const double exact = boundaryContainmentDev(r.patches, b);
    expectLE(exact, 1e-12, "RATIONAL arcs reproduced EXACTLY (not a polynomial approx)");
    std::printf("INFO rational-arc exactness worst dev = %.3e\n", exact);

    // Sanity: sampled boundary points really are on the circle of radius R.
    double radDev = 0.0;
    for (int k = 0; k < 4; ++k)
      for (int i = 0; i <= 50; ++i) {
        const Point3 p = evalCurveR(b.edges[k], static_cast<double>(i) / 50);
        radDev = std::max(radDev, std::fabs(std::hypot(p.x, p.y) - R));
      }
    expectLE(radDev, 1e-12, "boundary arcs lie on the exact circle (oracle is a true circle)");
  }

  // ═══ 2. NON-RATIONAL REDUCTION — weights-all-1 reproduces fillNSided pointwise ════════
  {
    const auto v = regularPolygonCorners(5, 3.0);
    const NSidedBoundary b = polygonBoundary(v);        // straight edges, non-rational

    const NSidedFillResult base = fillNSided(b);
    const NSidedFillRationalResult rat = fillNSidedRational(b);
    expectTrue(base.ok && rat.ok, "both C0 and rational fills succeed on the pentagon");
    expectTrue(base.patches.size() == rat.patches.size(), "same sub-patch count");

    double dev = 0.0;
    for (std::size_t p = 0; p < base.patches.size(); ++p)
      for (int iu = 0; iu <= 24; ++iu)
        for (int iv = 0; iv <= 24; ++iv) {
          const double u = static_cast<double>(iu) / 24, w = static_cast<double>(iv) / 24;
          dev = std::max(dev, distance(evalSurfaceR(base.patches[p], u, w),
                                       evalSurfaceR(rat.patches[p], u, w)));
        }
    expectLE(dev, 1e-12, "non-rational reduction: rational fill == fillNSided pointwise");
    std::printf("INFO non-rational reduction worst dev = %.3e\n", dev);

    // Explicit all-weights-1 (non-empty) input must also reduce identically.
    NSidedBoundary bw = b;
    for (auto& e : bw.edges) e.weights.assign(e.poles.size(), 1.0);
    const NSidedFillRationalResult rw = fillNSidedRational(bw);
    expectTrue(rw.ok, "explicit weights-all-1 boundary fills");
    double dev2 = 0.0;
    for (std::size_t p = 0; p < base.patches.size(); ++p)
      for (int iu = 0; iu <= 16; ++iu)
        for (int iv = 0; iv <= 16; ++iv) {
          const double u = static_cast<double>(iu) / 16, w = static_cast<double>(iv) / 16;
          dev2 = std::max(dev2, distance(evalSurfaceR(base.patches[p], u, w),
                                         evalSurfaceR(rw.patches[p], u, w)));
        }
    expectLE(dev2, 1e-12, "explicit weights-all-1 also reduces to fillNSided");
  }

  // ═══ 3. PLANAR RATIONAL CONTAINMENT — arcs in z=0 → fill points on z=0 ════════════════
  {
    const Point3 O{0, 0, 0};
    NSidedBoundary b;
    for (int k = 0; k < 4; ++k) b.edges.push_back(quarterArc(O, 1.5, k * M_PI / 2.0));
    const NSidedFillRationalResult r = fillNSidedRational(b);
    expectTrue(r.ok, "planar rational-arc fill ok");

    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 20; ++iu)
        for (int iv = 0; iv <= 20; ++iv) {
          const double u = static_cast<double>(iu) / 20, w = static_cast<double>(iv) / 20;
          maxZ = std::max(maxZ, std::fabs(evalSurfaceR(s, u, w).z));
        }
    expectLE(maxZ, 1e-10, "planar rational arcs → all fill points on z=0 plane");
    std::printf("INFO planar rational containment max|z| = %.3e\n", maxZ);
  }

  // ═══ 4. HONEST DECLINES ══════════════════════════════════════════════════════════
  {
    const Point3 O{0, 0, 0};
    // Non-closed loop.
    {
      NSidedBoundary bad;
      for (int k = 0; k < 4; ++k) bad.edges.push_back(quarterArc(O, 2.0, k * M_PI / 2.0));
      bad.edges[1].poles.front().z += 0.7;  // break a shared corner
      const NSidedFillRationalResult r = fillNSidedRational(bad);
      expectTrue(!r.ok, "non-closed rational loop declines honestly");
      expectTrue(!r.reason.empty(), "non-closed rational decline carries a reason");
      expectTrue(r.maxCornerError > 0.1, "non-closed decline reports a large corner error");
      expectTrue(!verifyNSidedBoundaryRational(bad).ok, "verify declines non-closed loop");
    }
    // Non-positive weight → declines (never a faked rational net).
    {
      NSidedBoundary badw;
      for (int k = 0; k < 4; ++k) badw.edges.push_back(quarterArc(O, 2.0, k * M_PI / 2.0));
      badw.edges[2].weights[1] = 0.0;  // zero weight is dishonest
      expectTrue(!fillNSidedRational(badw).ok, "non-positive weight declines");
      expectTrue(!verifyNSidedBoundaryRational(badw).ok, "verify declines non-positive weight");
    }
    // N < 3.
    {
      NSidedBoundary two;
      two.edges = {quarterArc(O, 2.0, 0.0), quarterArc(O, 2.0, M_PI)};
      expectTrue(!fillNSidedRational(two).ok, "N<3 rational boundary declines");
    }
    // Malformed edge (degree 0).
    {
      NSidedBoundary mal;
      for (int k = 0; k < 4; ++k) mal.edges.push_back(quarterArc(O, 2.0, k * M_PI / 2.0));
      mal.edges[0].degree = 0;
      expectTrue(!fillNSidedRational(mal).ok, "malformed rational edge declines");
    }
    // A consistent arc frame still succeeds (the guard is not over-eager).
    {
      NSidedBoundary ok;
      for (int k = 0; k < 4; ++k) ok.edges.push_back(quarterArc(O, 2.0, k * M_PI / 2.0));
      expectTrue(fillNSidedRational(ok).ok, "consistent arc frame still succeeds");
    }
  }

  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_nsided_rational: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_nsided_rational: %d failures / %d checks\n",
                g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_nsided_rational (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
