// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided.cpp — NURBS roadmap Layer 6 (N-SIDED boundary-filled surface) impl.
//
// Fill a closed N-gon boundary (N ≠ 4) with N tensor-product B-spline Coons sub-patches
// by Catmull-Clark-style MIDPOINT subdivision: split each boundary edge at its midpoint,
// take the polygon centroid as the single interior hub, and build one Coons quad per
// original corner (bounded by two boundary half-edges and two straight spokes to the
// centroid). The UNION reproduces every input boundary curve pointwise. It COMPOSES the
// exact Layer-1 ops (splitCurve / reparamCurve / knot-reversal to halve and orient the
// boundary edges) and bspline_coons (the per-quad fill). No linear solve — but the TU is
// guarded by CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated
// Layer-6 surfacing family (and because it composes bspline_coons, which is itself
// numsci-gated). With the guard OFF the TU is inert and the functions are absent.
//
#include "native/math/bspline_nsided.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint (corner / midpoint evaluation)
#include "native/math/bspline_coons.h"  // CoonsBoundary / coonsPatch (per-quad fill)
#include "native/math/bspline_ops.h"    // splitCurve / reparamCurve (halve the edges)

#include <cmath>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

bool wellFormed(const BsplineCurveData& c) {
  return c.weights.empty() && c.degree >= 1 && !c.poles.empty() &&
         static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 2;
}

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

// Reverse a curve's parametrization: C_rev(t) = C(1 − t) on the same [0,1] domain.
// Poles are reversed; the flat knot vector is mirrored about its span so the clamped
// [0,1] domain is preserved. Geometry-exact (the curve traced backwards).
BsplineCurveData reverseCurve(const BsplineCurveData& c) {
  BsplineCurveData r;
  r.degree = c.degree;
  r.poles.assign(c.poles.rbegin(), c.poles.rend());
  const double a = c.knots.front();
  const double b = c.knots.back();
  r.knots.resize(c.knots.size());
  for (std::size_t i = 0; i < c.knots.size(); ++i)
    r.knots[c.knots.size() - 1 - i] = a + b - c.knots[i];  // mirror + reverse order
  return r;  // non-rational in, non-rational out (weights left empty)
}

// The first half of edge e (e on [0,0.5]), reparametrized onto [0,1]. Runs V→M.
BsplineCurveData firstHalf(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reparamCurve(s.left, 0.0, 1.0);
}

// The second half of edge e (e on [0.5,1]), reparametrized onto [0,1] then REVERSED so
// it runs from the NEXT corner back to the midpoint (M_prev → … no: V → M direction the
// quad needs). splitCurve(e,0.5).right runs M→V(next); reversed it runs V(next)→M.
BsplineCurveData secondHalfReversed(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reverseCurve(reparamCurve(s.right, 0.0, 1.0));
}

// A straight degree-1 edge between two points (2 poles), on [0,1].
BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0.0, 0.0, 1.0, 1.0};
  c.poles = {a, b};
  return c;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// N-sided boundary consistency.
// ─────────────────────────────────────────────────────────────────────────────

NSidedBoundaryCheck verifyNSidedBoundary(const NSidedBoundary& b, double tol) {
  NSidedBoundaryCheck r;
  r.n = static_cast<int>(b.edges.size());

  if (r.n < 3) { r.reason = "N-sided fill needs at least 3 boundary edges"; return r; }

  for (int i = 0; i < r.n; ++i) {
    if (!wellFormed(b.edges[i])) {
      r.reason = "edge " + std::to_string(i) + " is rational or malformed";
      return r;
    }
  }

  // Consecutive corners must meet: edges[i](1) == edges[(i+1)%N](0). This is the
  // closed-loop precondition (the N-gon actually closes on shared corners).
  double worst = 0.0;
  for (int i = 0; i < r.n; ++i) {
    const int j = (i + 1) % r.n;
    const Point3 end_i = evalCurve(b.edges[i], 1.0);
    const Point3 start_j = evalCurve(b.edges[j], 0.0);
    worst = std::max(worst, distance(end_i, start_j));
  }
  r.maxCornerError = worst;

  if (worst > tol) {
    r.reason = "boundary is not a closed loop (consecutive corners do not coincide)";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// N-sided fill (midpoint subdivision → N Coons sub-patches).
// ─────────────────────────────────────────────────────────────────────────────

NSidedFillResult fillNSided(const NSidedBoundary& b, double tol) {
  NSidedFillResult r;

  // Step 1 — verify the boundary (declines honestly on non-closed / rational / degenerate).
  const NSidedBoundaryCheck chk = verifyNSidedBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  const int N = chk.n;

  // Step 2 — corners V[i] = edges[i](0), edge midpoints M[i] = edges[i](0.5),
  // and the centroid C = mean(V[i]) (the single interior hub all sub-patches share).
  std::vector<Point3> V(N), M(N);
  Vec3 sum{0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    V[i] = evalCurve(b.edges[i], 0.0);
    M[i] = evalCurve(b.edges[i], 0.5);
    sum = sum + Vec3{V[i].x, V[i].y, V[i].z};
  }
  const Point3 C{sum.x / N, sum.y / N, sum.z / N};
  r.centroid = C;

  // Guard: a degenerate (zero-length) spoke to the centroid would collapse a sub-quad.
  for (int i = 0; i < N; ++i) {
    if (distance(M[i], C) <= tol) {
      r.reason = "degenerate N-gon: an edge midpoint coincides with the centroid";
      return r;
    }
  }

  // Step 3 — one Coons quad per corner V[i]. Coons convention corners:
  //   P00 = V[i], P10 = M[i], P01 = M[i-1], P11 = C.
  //   c0 (v=0, V[i]→M[i])   = first half of e[i]                 (boundary),
  //   d0 (u=0, V[i]→M[i-1]) = second half of e[i-1] reversed     (boundary),
  //   c1 (v=1, M[i-1]→C)    = straight spoke M[i-1] → C,
  //   d1 (u=1, M[i]→C)      = straight spoke M[i]   → C.
  r.patches.reserve(N);
  for (int i = 0; i < N; ++i) {
    const int prev = (i + N - 1) % N;

    CoonsBoundary quad;
    quad.c0 = firstHalf(b.edges[i]);                 // V[i] → M[i]     (on boundary)
    quad.d0 = secondHalfReversed(b.edges[prev]);     // V[i] → M[i-1]   (on boundary)
    quad.c1 = lineEdge(M[prev], C);                  // M[i-1] → C      (interior spoke)
    quad.d1 = lineEdge(M[i], C);                     // M[i]   → C      (interior spoke)

    const CoonsResult cr = coonsPatch(quad, tol);
    if (!cr.ok) {
      r.patches.clear();
      r.reason = "sub-quad " + std::to_string(i) + " Coons fill declined: " + cr.reason;
      return r;
    }
    r.patches.push_back(cr.surface);
  }

  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
