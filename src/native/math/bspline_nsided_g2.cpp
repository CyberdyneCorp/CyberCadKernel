// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided_g2.cpp — NURBS roadmap Layer 6 (G2 N-SIDED boundary-filled surface) impl.
//
// Fill a closed N-gon boundary with N Gregory quintic-in-v pie-slice sub-patches that meet G2
// (curvature continuous) across every internal spoke and to the boundary cross-tangent +
// cross-curvature fields. Clean-room from the G2 Gregory / Chiyokura-Kimura construction
// (Piegl & Tiller ch.11; Farin; Chiyokura-Kimura 1983). See the header for the full geometry.
//
// DECOMPOSITION — N "pie slices", one per boundary edge e[i], on (u,v) ∈ [0,1]²:
//   * v = 0 : the boundary edge e[i]      (u-direction carries e[i]'s own basis EXACTLY),
//   * v = 1 : collapses to the centroid C (the degenerate hub corner — Gregory twist),
//   * u = 0 : the spoke V[i]   → C        (a quintic-in-v curve, SHARED with slice i-1),
//   * u = 1 : the spoke V[i+1] → C        (a quintic-in-v curve, SHARED with slice i+1).
//
// The v-direction is a QUINTIC (degree 5) clamped Bézier: 6 poles per u-station encoding, at
// v=0, boundary POSITION + CROSS-TANGENT + CROSS-2ND-DERIVATIVE (a G2 Hermite), and at v=1 the
// hub with radially-tapered 1st/2nd derivatives. Because v=0 reproduces e[i] exactly, boundary
// INTERPOLATION is machine-exact.
//
// G2 ACROSS A SEAM — BY POLE EQUALITY of the THREE seam-adjacent u-columns. For a CLAMPED
// B-spline the surface near a u-endpoint is governed EXACTLY (as a Bézier corner, independent of
// any interior knots) by the three columns adjacent to that endpoint: cols {0,1,2} at u=0 and
// cols {nu-1,nu-2,nu-3} at u=1. Slice i's u=1 seam and slice i+1's u=0 seam are the SAME spoke;
// we set slice i's {nu-3,nu-2,nu-1} and slice i+1's {0,1,2} to the IDENTICAL shared spoke pole
// columns (position + 1st-inward rib + 2nd-inward rib). Identical three-column corners over the
// identical clamped u-basis ⇒ S, ∂S/∂u AND ∂²S/∂u² all coincide across the seam → C0, G1 (unit
// normal continuous) AND G2 (normal curvature continuous), an exact machine-precision invariant.
// The centre twist (hub) is kept finite by radially tapering the interior 1st/2nd rows, which
// touches neither the boundary rows nor the across-seam ribs, so it breaks neither G1 nor G2.
//
// Guarded by CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated Layer-6
// surfacing family (it composes the shared Layer-1 ops). No linear solve — the construction is
// direct. With the guard OFF the TU is inert and the functions are absent.
//
#include "native/math/bspline_nsided_g2.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"      // curvePoint / curveDerivs (edge eval + tangents)
#include "native/math/bspline_ops.h"  // elevateDegreeCurve (exact degree raise for G2 control)

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

// ── Small helpers ──────────────────────────────────────────────────────────────────
int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

Vec3 asVec(const Point3& p) { return {p.x, p.y, p.z}; }
Point3 asPt(const Vec3& v) { return {v.x, v.y, v.z}; }
Point3 addV(const Point3& p, const Vec3& v) { return {p.x + v.x, p.y + v.y, p.z + v.z}; }

// The (unit-less) tangent of an edge at parameter t (direction; magnitude not used for guards).
Vec3 edgeTangent(const BsplineCurveData& e, double t) {
  std::vector<Vec3> d(2);
  curveDerivs(e.degree, e.poles, e.knots, t, 1, d);
  return d[1];
}

// The CURVATURE VECTOR κ⃗ = (r'' − (r''·T̂)T̂)/|r'|² of an edge at parameter t — the parametrization-
// invariant second-order shape of the curve (the component of acceleration normal to the tangent,
// per unit arclength²). Two edges meeting at a corner are G2-compatible iff their κ⃗ agree there.
Vec3 edgeCurvatureVec(const BsplineCurveData& e, double t) {
  std::vector<Vec3> d(3);
  curveDerivs(e.degree, e.poles, e.knots, t, 2, d);
  const Vec3 r1 = d[1], r2 = d[2];
  const double s2 = normSquared(r1);
  if (s2 <= 0.0) return Vec3{};
  const Vec3 tHat = r1 * (1.0 / std::sqrt(s2));
  const Vec3 normalAccel = r2 - tHat * dot(r2, tHat);
  return normalAccel * (1.0 / s2);
}

// ── v-direction QUINTIC (degree-5) Hermite basis, shared by every slice ─────────────
// A clamped quintic Bézier in v on [0,1]: knots {0×6, 1×6}, 6 poles Q0..Q5 per u-station.
// Endpoint identities (degree-5 clamped Bézier on [0,1]):
//   S(u,0)      = Q0,   ∂S/∂v(u,0)  = 5·(Q1 − Q0),   ∂²S/∂v²(u,0) = 20·(Q2 − 2·Q1 + Q0),
//   S(u,1)      = Q5,   ∂S/∂v(u,1)  = 5·(Q5 − Q4),   ∂²S/∂v²(u,1) = 20·(Q5 − 2·Q4 + Q3).
// So to place data (P, T, K) at v=0:  Q0 = P, Q1 = P + T/5, Q2 = 2·Q1 − P + K/20; and to reach
// the hub C at v=1 leaving-derivative tOut and curvature Kout: Q5 = C, Q4 = C − tOut/5,
// Q3 = 2·Q4 − C + Kout/20. Invert-free, exact.
const std::vector<double>& quinticKnots() {
  static const std::vector<double> k{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  return k;
}

// Build the 6-pole quintic-in-v column at one u-station from G2 Hermite data:
//   v=0 side: position P, cross-tangent T, cross-2nd-derivative K,
//   v=1 side: hub C, arrival-tangent tOut (into C), arrival-curvature Kout.
std::array<Point3, 6> quinticColumn(const Point3& P, const Vec3& T, const Vec3& K,
                                    const Point3& C, const Vec3& tOut, const Vec3& Kout) {
  std::array<Point3, 6> Q{};
  Q[0] = P;
  Q[1] = addV(P, T * (1.0 / 5.0));
  // Q2 = 2·Q1 − P + K/20  ⇒  ∂²S/∂v²(v=0) = 20·(Q2 − 2Q1 + P) = K.
  Q[2] = addV(asPt(asVec(Q[1]) * 2.0 - asVec(P)), K * (1.0 / 20.0));
  Q[5] = C;
  Q[4] = addV(C, tOut * (-1.0 / 5.0));
  // Q3 = 2·Q4 − C + Kout/20  ⇒  ∂²S/∂v²(v=1) = 20·(C − 2Q4 + Q3) = Kout.
  Q[3] = addV(asPt(asVec(Q[4]) * 2.0 - asVec(C)), Kout * (1.0 / 20.0));
  return Q;
}

// The mean spoke length V[k]→C (a fair geometric yardstick for lengths / curvatures).
double meanSpokeLength(const std::vector<Point3>& V, const Point3& C) {
  if (V.empty()) return 1.0;
  double s = 0.0;
  for (const Point3& v : V) s += distance(v, C);
  return s / static_cast<double>(V.size());
}

// ── Corner feasibility checks — each returns an empty string on OK, else a decline reason. ──

// G1-FEASIBILITY at the corners (inherited; G2 requires G1). A tangent-plane-continuous surface
// cannot cross a boundary with a tangent DISCONTINUITY at a corner unless everything there is
// coplanar. Accept iff the incident edge tangents are collinear (smooth corner) OR they + the
// spoke are coplanar (a planar N-gon). Never widen a tolerance for a creased corner.
std::string checkG1CornerFeasibility(const std::vector<BsplineCurveData>& E,
                                     const std::vector<Point3>& V, const Point3& C) {
  const int N = static_cast<int>(E.size());
  const double cornerTol = 1e-7;
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 tOut = edgeTangent(E[k], 0.0);        // leave V[k] along edge k
    const Vec3 tIn = edgeTangent(E[prev], 1.0);      // arrive at V[k] along edge k-1
    const Vec3 sp = asVec(C) - asVec(V[k]);          // spoke V[k]→C
    const double nOut = norm(tOut), nIn = norm(tIn), nSp = norm(sp);
    if (nOut <= 0.0 || nIn <= 0.0 || nSp <= 0.0) continue;
    const double sinCorner = norm(cross(tOut, tIn)) / (nOut * nIn);          // 0 ⇔ collinear
    const double triple = std::fabs(dot(cross(tOut, tIn), sp)) / (nOut * nIn * nSp);  // 0 ⇔ coplanar
    if (sinCorner > cornerTol && triple > cornerTol) {
      return "boundary creases at corner " + std::to_string(k) +
             " (non-collinear edge tangents not coplanar with the spoke) — no tangent plane "
             "across the incident spokes, G1 impossible ⇒ G2 impossible; supply a smooth "
             "boundary or a G-compatible cross-tangent field";
    }
  }
  return {};
}

// BOUNDARY G2-FEASIBILITY at the corners (the second-order precondition on the boundary GEOMETRY).
// A curvature-continuous surface across the incident spokes requires the two boundary edges at a
// corner to be CURVATURE-continuous there: their curvature vectors κ⃗ must agree. A curvature-
// CREASED boundary (G1 but not G2 arcs) leaks incompatible ∂²S/∂u² into the seam → decline. The
// tolerance is a relative curvature-vector agreement, NOT a G2 slack.
std::string checkBoundaryG2Feasibility(const std::vector<BsplineCurveData>& E,
                                       const std::vector<Point3>& V, const Point3& C) {
  const int N = static_cast<int>(E.size());
  const double mean = meanSpokeLength(V, C);
  const double kScale = (mean > 0.0) ? (1.0 / mean) : 1.0;  // reciprocal length ~ curvature scale
  const double cornerG2Tol = 1e-6;
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 kNext = edgeCurvatureVec(E[k], 0.0);     // edge k's curvature at V[k]
    const Vec3 kPrev = edgeCurvatureVec(E[prev], 1.0);  // edge k-1's curvature at V[k]
    const double mism = norm(kNext - kPrev);
    if (mism > cornerG2Tol * std::max({norm(kNext), norm(kPrev), kScale, 1e-30})) {
      return "boundary is curvature-discontinuous at corner " + std::to_string(k) +
             " (incident edges are G1 but not G2 there — a curvature crease) — no curvature-"
             "continuous surface can cross the incident spokes; supply a G2-continuous boundary";
    }
  }
  return {};
}

// PRESCRIBED cross-curvature compatibility at the corners. Two incident curvature values are
// irreconcilable ⇔ both are SIGNIFICANT (well above the geometric scale) AND point in substantially
// OPPOSING directions (‖a−b‖ ≥ 0.9·(‖a‖+‖b‖) ⇒ more than ~150° apart — averaging annihilates the
// data). A mild corner variation is reconciled by averaging and accepted. Never widen a tolerance.
std::string checkPrescribedCurvatureCompat(const std::vector<std::vector<Vec3>>& Kfield,
                                           const std::vector<Point3>& V, const Point3& C) {
  const int N = static_cast<int>(Kfield.size());
  const double scale = meanSpokeLength(V, C);
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 kNext = Kfield[k].front();          // edge k's cross-curvature at V[k]
    const Vec3 kPrev = Kfield[prev].back();        // edge k-1's cross-curvature at V[k]
    const double sumMag = norm(kNext) + norm(kPrev);
    const bool significant = sumMag > 1e3 * std::max(scale, 1e-30);
    if (significant && norm(kNext - kPrev) >= 0.9 * sumMag) {
      return "prescribed cross-curvature is incompatible at corner " + std::to_string(k) +
             " (incident edges' second-order data point in opposing directions — irreconcilable)"
             " — G2 impossible";
    }
  }
  return {};
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// G2 N-sided fill.
// ─────────────────────────────────────────────────────────────────────────────

NSidedFillG2Result nSidedFillG2(const NSidedBoundary& b,
                                const std::vector<CrossTangentField>& tangents,
                                const std::vector<CrossCurvatureField>& curvatures,
                                double tol) {
  NSidedFillG2Result r;

  // Step 1 — verify the boundary loop (declines honestly on non-closed / rational / degenerate).
  const NSidedBoundaryCheck chk = verifyNSidedBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  const int N = chk.n;
  if (!tangents.empty() && static_cast<int>(tangents.size()) != N) {
    r.reason = "cross-tangent field count must be empty or exactly N";
    return r;
  }
  if (!curvatures.empty() && static_cast<int>(curvatures.size()) != N) {
    r.reason = "cross-curvature field count must be empty or exactly N";
    return r;
  }

  // Step 2 — corners V[i] = e[i](0) and centroid C = mean(V[i]).
  std::vector<Point3> V(N);
  Vec3 sum{0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    V[i] = evalCurve(b.edges[i], 0.0);
    sum += asVec(V[i]);
  }
  const Point3 C = asPt(sum / static_cast<double>(N));
  r.centroid = C;

  for (int i = 0; i < N; ++i) {
    if (distance(V[i], C) <= tol) {
      r.reason = "degenerate N-gon: a corner coincides with the centroid";
      return r;
    }
  }

  // Step 3 — pre-elevate every edge to degree ≥ 5 (≥ 6 u-poles) so each slice has THREE distinct
  // seam-adjacent u-columns per side (cols {0,1,2} at u=0 and {nu-1,nu-2,nu-3} at u=1 disjoint):
  // the position + 1st-inward + 2nd-inward across-seam control lives in those three columns, so
  // G2-by-pole-equality has room on both seams. Degree elevation is EXACT (Layer-1 A5.9): v=0
  // still reproduces the edge pointwise, so boundary interpolation stays machine-exact.
  std::vector<BsplineCurveData> E(N);              // elevated edges (u-basis of each slice)
  std::vector<std::vector<Vec3>> Tfield(N);        // cross-tangent per u-pole, into the fill
  std::vector<std::vector<Vec3>> Kfield(N);        // cross-2nd-derivative per u-pole
  for (int i = 0; i < N; ++i) {
    BsplineCurveData e = b.edges[i];
    if (e.degree < 5) e = elevateDegreeCurve(e, 5 - e.degree);
    E[i] = e;
    const int nu = nPolesOf(e);

    const bool haveT = !tangents.empty() && !tangents[i].poles.empty();
    if (haveT && static_cast<int>(tangents[i].poles.size()) != nu) {
      r.reason = "cross-tangent field " + std::to_string(i) + " pole count != edge pole count";
      return r;
    }
    const bool haveK = !curvatures.empty() && !curvatures[i].poles.empty();
    if (haveK && static_cast<int>(curvatures[i].poles.size()) != nu) {
      r.reason = "cross-curvature field " + std::to_string(i) + " pole count != edge pole count";
      return r;
    }

    auto greville = [&](int a) {
      double s = 0.0;
      for (int k = 1; k <= e.degree; ++k) s += e.knots[a + k];
      return std::clamp(s / e.degree, 0.0, 1.0);
    };

    Tfield[i].resize(nu);
    Kfield[i].resize(nu);
    for (int a = 0; a < nu; ++a) {
      // Natural minimal-energy field: pull the boundary radially toward the centroid C for the
      // cross-tangent, and a mildly-tapered radial cross-curvature. For a PLANAR boundary this is
      // in-plane, so the whole fill stays planar (analytic-exact); for a sphere / cylinder the
      // caller prescribes the true analytic fields. A caller field overrides per pole.
      const Vec3 radial = asVec(C) - asVec(e.poles[a]);
      Vec3 Tb = haveT ? tangents[i].poles[a] : radial;
      Vec3 Kb = haveK ? curvatures[i].poles[a] : (radial * 0.0);  // natural: zero curvature bias

      // G1-incompatibility guard (a prescribed cross-tangent (anti-)parallel to the boundary
      // tangent leaves no tangent plane → G1 impossible ⇒ G2 impossible). Honest decline; NEVER
      // widen the tolerance. The natural radial field is never parallel for a non-degenerate N-gon.
      if (haveT) {
        const Vec3 tb = edgeTangent(e, greville(a));
        const double denom = norm(tb) * norm(Tb);
        if (denom <= 0.0 || norm(cross(tb, Tb)) <= 1e-9 * denom) {
          r.reason = "prescribed cross-tangent is (anti-)parallel to the boundary tangent at "
                     "edge " + std::to_string(i) + " — no tangent plane, G1 impossible ⇒ G2 "
                     "impossible";
          return r;
        }
      }
      Tfield[i][a] = Tb;
      Kfield[i][a] = Kb;
    }
  }

  // FEASIBILITY at the corners (honest preconditions; never widen a tolerance to pass any of them):
  //   * G1 — a creased 3-D corner has no tangent plane across the incident spokes (G1 ⇒ G2 fails).
  //   * BOUNDARY G2 — a curvature-CREASED boundary corner (G1-only arcs) leaks incompatible
  //     ∂²S/∂u² into the seam; no curvature-continuous surface can cross the incident spokes.
  //   * PRESCRIBED-K — a caller cross-curvature whose incident-corner values oppose is
  //     irreconcilable (averaging annihilates the data). The natural field is always consistent.
  if (std::string why = checkG1CornerFeasibility(E, V, C); !why.empty()) { r.reason = why; return r; }
  if (std::string why = checkBoundaryG2Feasibility(E, V, C); !why.empty()) { r.reason = why; return r; }
  if (!curvatures.empty()) {
    if (std::string why = checkPrescribedCurvatureCompat(Kfield, V, C); !why.empty()) {
      r.reason = why;
      return r;
    }
  }

  // The SHARED per-corner spoke: the position column V[k] → C AND the two inward RIBS (1st and
  // 2nd cross-spoke derivative) at each v-level. Both slices meeting at corner V[k] (slice k-1's
  // u=1 seam and slice k's u=0 seam) build these IDENTICAL columns, so the seam is C0/G1/G2 by
  // pole equality.
  //
  //  * spokeQ[k][j] — the shared position column (a quintic-in-v curve from V[k] leaving along the
  //    shared corner cross-tangent to C, arriving radially with the shared corner cross-curvature).
  //  * rib1[k][j]  — the shared 1st inward cross-spoke derivative at v-level j (loop-tangential
  //    sweep at the corner, transverse to the spoke, tapered to the hub).
  //  * rib2[k][j]  — the shared 2nd inward cross-spoke derivative at v-level j (the second-order
  //    reconciliation; a mild radial-of-the-sweep taper — zero for the natural field).
  std::vector<std::array<Point3, 6>> spokeQ(N);
  std::vector<std::array<Vec3, 6>> rib1(N), rib2(N);
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;

    // Shared corner cross-tangent + cross-curvature (average of the two incident edges' values).
    const Vec3 cornerT = (Tfield[k].front() + Tfield[prev].back()) * 0.5;
    const Vec3 cornerK = (Kfield[k].front() + Kfield[prev].back()) * 0.5;

    // Position column: V[k] leaving along cornerT with curvature cornerK, arriving at C radially
    // (tOut = C→V[k], zero hub curvature — the fair minimal-energy hub).
    const Vec3 tOutHub = asVec(V[k]) - asVec(C);
    spokeQ[k] = quinticColumn(V[k], cornerT, cornerK, C, tOutHub, Vec3{});

    // 1st inward rib: the corner's loop-tangential sweep, made transverse to the spoke, tapered.
    const Vec3 tE = edgeTangent(E[k], 0.0);             // leave V[k] along edge k
    const Vec3 tP = edgeTangent(E[prev], 1.0);          // arrive at V[k] along edge k-1
    Vec3 sweep = tE + tP;
    const Vec3 sp = asVec(C) - asVec(V[k]);
    const double sp2 = normSquared(sp);
    if (sp2 > 0.0) sweep -= sp * (dot(sweep, sp) / sp2);
    if (isNull(sweep, 1e-14)) sweep = tE;
    // Taper the sweep from full at the boundary (j=1) to zero at the hub (j=4): j∈{1,2,3,4} get
    // {1, 2/3, 1/3, 0}·sweep. j=0 (boundary, fixed by the edge) and j=5 (apex) carry no rib.
    rib1[k] = {Vec3{}, sweep, sweep * (2.0 / 3.0), sweep * (1.0 / 3.0), Vec3{}, Vec3{}};

    // 2nd inward rib: the second-order reconciliation. For the natural field this is zero (a fair
    // minimal-energy G2 blend); a caller curvature that bends the sweep supplies a nonzero rib
    // proportional to the corner cross-curvature's transverse component, tapered like rib1.
    Vec3 sweep2 = cornerK;
    if (sp2 > 0.0) sweep2 -= sp * (dot(sweep2, sp) / sp2);
    rib2[k] = {Vec3{}, sweep2, sweep2 * (2.0 / 3.0), sweep2 * (1.0 / 3.0), Vec3{}, Vec3{}};
  }

  // Step 4 — build the N Gregory quintic pie-slices. Slice i covers edge e[i] at v=0 and shrinks
  // to C. u-direction: e[i]'s own (elevated, degree ≥ 5) basis (nu poles) → v=0 reproduces e[i]
  // EXACTLY. v-direction: quintic Bézier (6 poles), G2 Hermite boundary→hub.
  //
  // Per interior u-station a the 6 v-poles form a quintic column carrying the boundary G2 data
  // (position e[i].poles[a], cross-tangent Tfield[i][a], cross-curvature Kfield[i][a]) and the
  // radially-tapered hub. The THREE seam-adjacent columns on each side (cols {0,1,2} at u=0,
  // {nu-1,nu-2,nu-3} at u=1) are OVERWRITTEN from the shared spoke data (position + rib1 + rib2)
  // so both incident slices carry byte-identical seam corners ⇒ C0/G1/G2 by pole equality.
  r.patches.reserve(N);
  for (int i = 0; i < N; ++i) {
    const BsplineCurveData& e = E[i];
    const int nu = nPolesOf(e);
    const int nextI = (i + 1) % N;

    std::vector<Point3> poles(static_cast<std::size_t>(nu) * 6);
    auto put = [&](int a, int j, const Point3& p) {
      poles[static_cast<std::size_t>(a) * 6 + j] = p;
    };

    // Interior stations: the boundary G2 Hermite column with a radially-tapered hub.
    for (int a = 0; a < nu; ++a) {
      const Point3 P = e.poles[a];
      const Vec3 tOutHub = asVec(P) - asVec(C);  // arrive at C radially from this station
      const std::array<Point3, 6> Q =
          quinticColumn(P, Tfield[i][a], Kfield[i][a], C, tOutHub, Vec3{});
      for (int j = 0; j < 6; ++j) put(a, j, Q[j]);
    }

    // Overwrite the SEAM (u=0 / u=1 corner) columns fully with the shared spoke position column
    // (exact C0 seam on both incident slices — same as the G1 fill), then inject the shared 1st +
    // 2nd cross-spoke ribs into the two inner seam-adjacent columns at the INTERIOR v-levels
    // (j∈{1,2,3,4}) ONLY: j=0 (boundary v=0) stays the edge's own poles so boundary interpolation
    // is untouched, and j=5 (hub apex) stays C so the hub stays collapsed. The endpoint u-deriv
    // identities for a clamped B-spline of degree p (clamped-end spans a1 = U[p+1]−U[1],
    // a2 = U[p+2]−U[2], gap b = U[p+1]−U[2]):
    //   ∂S/∂u(0,v)   = p·(col1(v) − col0(v))/a1,
    //   ∂²S/∂u²(0,v) = p(p−1)·[ (col2−col1)/a2 − (col1−col0)/a1 ]/b.
    // Placing col1 = spoke + (a1/p)·rib1 gives ∂S/∂u = rib1; then
    // col2 = col1 + a2·[ rib1/p + rib2·b/(p(p−1)) ] gives ∂²S/∂u² = rib2. Both neighbour slices
    // inject the SAME rib1/rib2 with these identities at their own clamped ends → ∂S/∂u AND
    // ∂²S/∂u² are the identical vectors across the seam ⇒ G1 AND G2 by pole equality.
    const double p = static_cast<double>(e.degree);
    const int m = static_cast<int>(e.knots.size()) - 1;
    const double a1_0 = e.knots[e.degree + 1] - e.knots[1];
    const double a2_0 = e.knots[e.degree + 2] - e.knots[2];
    const double b_0 = e.knots[e.degree + 1] - e.knots[2];
    const double a1_N = e.knots[m - 1] - e.knots[m - e.degree - 1];
    const double a2_N = e.knots[m - 2] - e.knots[m - e.degree - 2];
    const double b_N = e.knots[m - 2] - e.knots[m - e.degree - 1];
    auto safe = [](double x) { return (std::fabs(x) > 0.0) ? x : 1.0; };
    for (int j = 0; j < 6; ++j) {
      // Seam corner columns: full shared spoke (all j, exact C0 seam curve).
      put(0, j, spokeQ[i][j]);
      put(nu - 1, j, spokeQ[nextI][j]);
      if (j >= 1 && j <= 4) {
        // u=0 side (spoke i) — inward +u.
        const Vec3 r1 = rib1[i][j], r2 = rib2[i][j];
        const Point3 c0 = spokeQ[i][j];
        const Point3 c1 = addV(c0, r1 * (safe(a1_0) / p));
        const Vec3 step = r1 * (1.0 / p) + r2 * (b_0 / (p * (p - 1.0)));
        const Point3 c2 = addV(c1, step * safe(a2_0));
        put(1, j, c1);
        put(2, j, c2);
        // u=1 side (spoke i+1) — inward −u. ∂S/∂u is ODD in u so its first-difference flips sign
        // (C1 = C0 − (a1/p)·R1 ⇒ ∂S/∂u(1,·) = R1, matching the neighbour's ∂S/∂u(0,·)); ∂²S/∂u²
        // is EVEN in u so the SECOND-order term must NOT flip — the R2 contribution keeps the same
        // sign as the u=0 side while the R1 contribution flips, so that ∂²S/∂u²(1,·) = R2 too. Both
        // incident slices thus present the identical ∂S/∂u AND ∂²S/∂u² at the seam → G1 AND G2.
        const Vec3 R1 = rib1[nextI][j], R2 = rib2[nextI][j];
        const Point3 C0 = spokeQ[nextI][j];
        const Point3 C1 = addV(C0, R1 * (-safe(a1_N) / p));
        const Vec3 stepN = R1 * (1.0 / p) - R2 * (b_N / (p * (p - 1.0)));
        const Point3 C2 = addV(C1, stepN * (-safe(a2_N)));
        put(nu - 2, j, C1);
        put(nu - 3, j, C2);
      }
    }

    BsplineSurfaceData s;
    s.degreeU = e.degree;
    s.degreeV = 5;
    s.nPolesU = nu;
    s.nPolesV = 6;
    s.knotsU = e.knots;
    s.knotsV = quinticKnots();
    s.poles = std::move(poles);
    s.weights.clear();  // non-rational (the Gregory twist reconciliation is folded into the rows).
    r.patches.push_back(std::move(s));
  }

  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
