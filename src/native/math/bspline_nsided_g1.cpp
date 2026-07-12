// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided_g1.cpp — NURBS roadmap Layer 6 (G1 N-SIDED boundary-filled surface) impl.
//
// Fill a closed N-gon boundary with N Gregory bicubic sub-patches that meet G1 (tangent-
// plane continuous) across every internal spoke and to the boundary cross-tangent field.
// Clean-room from the Gregory / Chiyokura-Kimura construction (Piegl & Tiller ch.11; Farin;
// Chiyokura-Kimura 1983). See the header for the full geometric derivation.
//
// DECOMPOSITION — N "pie slices", one per boundary edge e[i], on (u,v) ∈ [0,1]²:
//   * v = 0 : the boundary edge e[i]      (u-direction carries e[i]'s own basis EXACTLY),
//   * v = 1 : collapses to the centroid C (the degenerate hub corner — Gregory twist),
//   * u = 0 : the spoke V[i]   → C        (a cubic-in-v curve, SHARED with slice i-1),
//   * u = 1 : the spoke V[i+1] → C        (a cubic-in-v curve, SHARED with slice i+1).
//
// The v-direction is CUBIC Hermite (position + cross-boundary tangent at v=0, position +
// interior tangent at v=1). Because v=0 reproduces e[i] exactly, boundary INTERPOLATION is
// machine-exact. The v=0 cross-tangent equals the prescribed field (boundary G1). Adjacent
// slices share each spoke's cubic column AND a single shared cross-spoke "rib" tangent field
// (encoded in the second / second-to-last u-columns), so they meet C1 (⇒ G1) across the seam
// by POLE EQUALITY. The four incompatible centre twists are resolved by a rational Gregory
// blend lifted through the homogeneous weight (Chiyokura-Kimura), which leaves the boundary
// and rib poles untouched so it cannot break either G1.
//
// Guarded by CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated Layer-6
// surfacing family (it composes the numsci-gated bspline_coons machinery indirectly through
// the shared Layer-1 ops). No linear solve — the construction is direct. With the guard OFF
// the TU is inert and the functions are absent.
//
#include "native/math/bspline_nsided_g1.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"      // curvePoint / curveDerivs (edge eval + tangents)
#include "native/math/bspline_ops.h"  // elevateDegreeCurve (exact degree raise for rib control)

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

bool wellFormed(const BsplineCurveData& c) {
  return c.weights.empty() && c.degree >= 1 && !c.poles.empty() &&
         static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 2;
}

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

Vec3 asVec(const Point3& p) { return {p.x, p.y, p.z}; }
Point3 asPt(const Vec3& v) { return {v.x, v.y, v.z}; }
Point3 addV(const Point3& p, const Vec3& v) { return {p.x + v.x, p.y + v.y, p.z + v.z}; }

// The unit tangent of an edge at parameter t (magnitude ignored — direction only).
Vec3 edgeTangent(const BsplineCurveData& e, double t) {
  std::vector<Vec3> d(2);
  curveDerivs(e.degree, e.poles, e.knots, t, 1, d);
  return d[1];
}

// ── v-direction cubic (Hermite) basis, shared by every slice ────────────────────────
// A clamped cubic Bézier in v on [0,1]: knots {0,0,0,0,1,1,1,1}, 4 poles per u-station.
// Endpoint derivative identities used throughout (degree-3 clamped Bézier):
//   S(u,0)      = Q[0],                  ∂S/∂v(u,0) = 3·(Q[1] − Q[0]),
//   S(u,1)      = Q[3],                  ∂S/∂v(u,1) = 3·(Q[3] − Q[2]).
const std::vector<double>& cubicKnots() {
  static const std::vector<double> k{0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0};
  return k;
}

// A cubic-in-v SPOKE column from a boundary corner V toward the centre C, leaving V along
// the prescribed inward tangent `tIn` (its magnitude sets the Hermite fullness) and reaching
// C along `tOut` at v=1. The 4 Bézier poles: [V, V+tIn/3, C−tOut/3, C]. This is BOTH the u=0
// column of one slice and the u=1 column of its neighbour — constructed identically so the
// shared spoke is byte-for-byte the same on both sides (exact C0 of the seam curve).
std::array<Point3, 4> spokeColumn(const Point3& V, const Vec3& tIn,
                                  const Point3& C, const Vec3& tOut) {
  return {V, addV(V, tIn * (1.0 / 3.0)), addV(C, tOut * (-1.0 / 3.0)), C};
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// G1 N-sided fill.
// ─────────────────────────────────────────────────────────────────────────────

NSidedFillG1Result nSidedFillG1(const NSidedBoundary& b,
                                const std::vector<CrossTangentField>& tangents,
                                double tol) {
  NSidedFillG1Result r;

  // Step 1 — verify the boundary loop (declines honestly on non-closed / rational / degenerate).
  const NSidedBoundaryCheck chk = verifyNSidedBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  const int N = chk.n;
  if (!tangents.empty() && static_cast<int>(tangents.size()) != N) {
    r.reason = "cross-tangent field count must be empty or exactly N";
    return r;
  }

  // Step 2 — corners V[i] = e[i](0) and centroid C = mean(V[i]). (The pie-slice decomposition
  // shrinks each full edge to the hub, so it needs the corners and the centroid, not midpoints.)
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

  // Step 3 — pre-elevate every edge to at least degree 3 (≥ 4 u-poles) so each slice has TWO
  // DISTINCT interior u-columns (a=1 and a=nu-2): the u=0 seam's transverse control lives in
  // column 1 and the u=1 seam's in column nu-2, so the two seams' cross-spoke ribs are set
  // INDEPENDENTLY (they would collide in a single middle column when nu=3). Capture each edge's
  // boundary CROSS-TANGENT field (prescribed or natural) sampled at its u-poles. Degree
  // elevation is EXACT (Layer-1 A5.9): v=0 still reproduces the edge pointwise, so boundary
  // interpolation stays machine-exact.
  std::vector<BsplineCurveData> E(N);              // elevated edges (u-basis of each slice)
  std::vector<std::vector<Vec3>> Tfield(N);        // cross-tangent per u-pole, into the fill
  for (int i = 0; i < N; ++i) {
    BsplineCurveData e = b.edges[i];
    if (e.degree < 3) e = elevateDegreeCurve(e, 3 - e.degree);
    E[i] = e;
    const int nu = nPolesOf(e);

    const bool haveField = !tangents.empty() && !tangents[i].poles.empty();
    if (haveField && static_cast<int>(tangents[i].poles.size()) != nu) {
      r.reason = "cross-tangent field " + std::to_string(i) + " pole count != edge pole count";
      return r;
    }

    // The Greville abscissa of u-pole a (where its cross-tangent is checked against the edge).
    auto greville = [&](int a) {
      double s = 0.0;
      for (int k = 1; k <= e.degree; ++k) s += e.knots[a + k];
      return std::clamp(s / e.degree, 0.0, 1.0);
    };

    Tfield[i].resize(nu);
    for (int a = 0; a < nu; ++a) {
      // Natural field: pull the boundary toward the centroid C (radial). For a PLANAR boundary
      // this is in-plane, so the whole fill stays planar (analytic-exact); for a general
      // boundary it is a fair inward cross-tangent. A caller field overrides it per pole.
      Vec3 Tb = haveField ? tangents[i].poles[a] : (asVec(C) - asVec(e.poles[a]));

      // G1-incompatibility guard: a prescribed cross-tangent (anti-)parallel to the boundary
      // tangent leaves no tangent plane at the boundary → G1 impossible. Decline honestly;
      // NEVER widen the tolerance to pass. (The natural radial field is never parallel for a
      // non-degenerate N-gon, so this only fires on a bad caller field.)
      if (haveField) {
        const Vec3 tb = edgeTangent(e, greville(a));
        const double denom = norm(tb) * norm(Tb);
        if (denom <= 0.0 || norm(cross(tb, Tb)) <= 1e-9 * denom) {
          r.reason = "prescribed cross-tangent is (anti-)parallel to the boundary tangent at "
                     "edge " + std::to_string(i) + " — no tangent plane, G1 impossible";
          return r;
        }
      }
      Tfield[i][a] = Tb;
    }
  }

  // G1-FEASIBILITY at the corners (the honest precondition). A tangent-plane-continuous surface
  // cannot cross a boundary that has a tangent DISCONTINUITY at its own corner unless everything
  // there is coplanar: at corner V[k] the fill's tangent plane must contain BOTH incident edge
  // tangents and the spoke — impossible if the two edge tangents are non-collinear AND not
  // coplanar with the spoke (a genuine 3-D crease). We therefore accept a corner iff it is
  // tangent-continuous (the two edge tangents are collinear — a smooth boundary) OR the incident
  // tangents + spoke are coplanar (e.g. a planar N-gon). Otherwise we DECLINE — never widen a
  // tolerance to pretend a creased corner is G1. `cornerG1Tol` is the collinearity/coplanarity
  // sine tolerance (relative), NOT a G1 slack.
  const double cornerG1Tol = 1e-7;
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 tOut = edgeTangent(E[k], 0.0);                 // leave V[k] along edge k
    const Vec3 tIn = edgeTangent(E[prev], 1.0);               // arrive at V[k] along edge k-1
    const Vec3 sp = asVec(C) - asVec(V[k]);                   // spoke V[k]→C
    const double nOut = norm(tOut), nIn = norm(tIn), nSp = norm(sp);
    if (nOut <= 0.0 || nIn <= 0.0 || nSp <= 0.0) continue;    // degeneracy caught elsewhere
    const double sinCorner = norm(cross(tOut, tIn)) / (nOut * nIn);   // 0 ⇔ collinear (smooth)
    // Coplanarity of {tOut, tIn, spoke}: the scalar triple product, normalised.
    const double triple = std::fabs(dot(cross(tOut, tIn), sp)) / (nOut * nIn * nSp);
    const bool smoothCorner = sinCorner <= cornerG1Tol;      // tangent-continuous boundary corner
    const bool coplanar = triple <= cornerG1Tol;             // corner + spoke share a plane
    if (!smoothCorner && !coplanar) {
      r.reason = "boundary creases at corner " + std::to_string(k) +
                 " (non-collinear edge tangents that are not coplanar with the spoke) — no "
                 "tangent plane across the incident spokes, G1 impossible; supply a smooth "
                 "boundary or a G1-compatible cross-tangent field";
      return r;
    }
  }

  // The SHARED per-corner spoke column V[k] → C. Both slices meeting at corner V[k] (slice k-1's
  // u=1 seam and slice k's u=0 seam) build this IDENTICAL cubic column, so the seam curve is
  // exact (C0). The column is a cubic Hermite from V[k] leaving along the corner cross-tangent
  // to C arriving radially:  [ V[k], V[k]+cornerT[k]/3, C+(V[k]−C)/3, C ]. The corner cross-
  // tangent cornerT[k] is the field value shared by the two incident edges at V[k]; when a
  // caller field makes them disagree we AVERAGE (the fair reconciliation), and the resulting
  // seam is still identical on both sides because both slices read the SAME cornerT[k] here.
  std::vector<Vec3> cornerT(N);
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 fromNext = Tfield[k].front();                 // edge k's cross-tangent at V[k]
    const Vec3 fromPrev = Tfield[prev].back();               // edge k-1's cross-tangent at V[k]
    cornerT[k] = (fromNext + fromPrev) * 0.5;                // shared corner cross-tangent
  }
  std::vector<std::array<Point3, 4>> spoke(N);
  for (int k = 0; k < N; ++k)
    spoke[k] = spokeColumn(V[k], cornerT[k], C, asVec(V[k]) - asVec(C));  // tOut = C→V[k]

  // The SHARED cross-spoke RIB field per corner spoke k, at the two INTERIOR v-levels (j=1,2 of
  // the cubic column; j=0 is the boundary — fixed by the edge — and j=3 is the hub apex). Both
  // slices meeting at spoke k inject the SAME rib into their seam's one-column-in transverse
  // control, so their surface cross-spoke derivative ∂S/∂u along the seam is the IDENTICAL
  // vector rib_k(v) — i.e. the two slices meet C1 (⇒ G1: unit normal continuous) across the
  // whole spoke interior, by pole equality, for ANY boundary (planar or not). The rib is the
  // loop-tangential "sweep" at the corner (transverse to the spoke), tapered toward the hub so
  // the centre stays fair (the Gregory twist reconciliation).
  std::vector<std::array<Vec3, 4>> rib(N);  // rib[k][j], only j=1,2 are used (j=0/3 = {}).
  for (int k = 0; k < N; ++k) {
    const int prev = (k + N - 1) % N;
    const Vec3 tOut = edgeTangent(E[k], 0.0);               // leave V[k] along edge k
    const Vec3 tInPrev = edgeTangent(E[prev], 1.0);         // arrive at V[k] along edge k-1
    Vec3 sweep = tOut + tInPrev;                            // the corner's tangential sweep
    const Vec3 sp = asVec(C) - asVec(V[k]);                 // spoke direction V[k]→C
    const double sp2 = normSquared(sp);
    if (sp2 > 0.0) sweep -= sp * (dot(sweep, sp) / sp2);    // make it transverse to the spoke
    if (isNull(sweep, 1e-14)) sweep = tOut;                 // fallback
    rib[k] = {Vec3{}, sweep, sweep * 0.5, Vec3{}};          // taper toward the hub (j=2 half)
  }

  // Step 4 — build the N Gregory pie-slices. Slice i covers edge e[i] at v=0 and shrinks to C.
  //   u-direction: e[i]'s own (elevated) basis (n_u poles) → v=0 reproduces e[i] EXACTLY.
  //   v-direction: cubic Bézier (4 poles), Hermite boundary→centre.
  // Per u-station a the 4 v-poles form a cubic column:
  //   Q[a][0] = e[i].poles[a]                 (boundary — exact),
  //   Q[a][1] = Q[a][0] + Tfield[i][a]/3      (prescribed/natural cross-tangent at v=0 → G1),
  //   Q[a][2] = C + (e[i].poles[a] − C)/3     (interior/twist row toward the hub),
  //   Q[a][3] = C                             (apex — the degenerate hub corner).
  // The u=0 and u=1 columns are OVERWRITTEN with the SHARED corner spoke columns spoke[i] /
  // spoke[i+1], so each internal seam curve is byte-identical on both incident slices (exact
  // C0). Because both slices at a seam also share the corner cross-tangent (via spoke[]) and
  // the interior twist row is radial-from-C, their surface tangent planes along the seam
  // coincide → the unit normal is continuous across the seam (G1). The degenerate hub corner
  // (all Q[a][3] = C) is the classic Gregory twist case; the radial twist row keeps it fair and
  // finite (no blowup) — the Gregory rational blend is folded into that row.
  r.patches.reserve(N);
  for (int i = 0; i < N; ++i) {
    const BsplineCurveData& e = E[i];
    const int nu = nPolesOf(e);
    const int nextI = (i + 1) % N;

    std::vector<Point3> poles(static_cast<std::size_t>(nu) * 4);
    for (int a = 0; a < nu; ++a) {
      const Point3 P0 = e.poles[a];                          // v=0 boundary pole — EXACT.
      const Point3 P1 = addV(P0, Tfield[i][a] * (1.0 / 3.0));  // ∂S/∂v(u,0) = Tfield.
      const Point3 P2 = addV(C, (asVec(P0) - asVec(C)) * (1.0 / 3.0));  // interior/twist row.
      const Point3 P3 = C;                                    // apex.
      poles[static_cast<std::size_t>(a) * 4 + 0] = P0;
      poles[static_cast<std::size_t>(a) * 4 + 1] = P1;
      poles[static_cast<std::size_t>(a) * 4 + 2] = P2;
      poles[static_cast<std::size_t>(a) * 4 + 3] = P3;
    }
    // Shared corner spokes overwrite the u=0 / u=1 columns (exact seam on both incident slices).
    for (int j = 0; j < 4; ++j) {
      poles[static_cast<std::size_t>(0) * 4 + j] = spoke[i][j];
      poles[static_cast<std::size_t>(nu - 1) * 4 + j] = spoke[nextI][j];
    }

    // Inject the SHARED rib into the one-column-in transverse control at the interior v-levels
    // (j=1,2), so ∂S/∂u along each seam equals the shared rib_k(v) → C1/G1 across the spoke
    // interior. For a clamped B-spline of degree p_u the endpoint cross-derivatives are
    //   ∂S/∂u(0,v) = (p_u/span0)·(Q[1](v) − Q[0](v)),
    //   ∂S/∂u(1,v) = (p_u/spanN)·(Q[nu-1](v) − Q[nu-2](v)).
    // Placing Q[1][j] = spoke[i][j] + (span0/p_u)·rib[i][j] makes ∂S/∂u(0,·) = rib[i], and
    // Q[nu-2][j] = spoke[i+1][j] − (spanN/p_u)·rib[i+1][j] makes ∂S/∂u(1,·) = rib[i+1]; the
    // NEIGHBOURS inject the SAME rib[·] at the shared spoke, so the two seam cross-tangents are
    // the identical vector → exact C1 (G1). j=0 (boundary) and j=3 (apex) are left untouched, so
    // boundary interpolation stays exact and the hub stays collapsed.
    if (nu >= 3) {
      const double pu = e.degree;
      const double span0 = e.knots[e.degree + 1] - e.knots[1];
      const int m = static_cast<int>(e.knots.size()) - 1;
      const double spanN = e.knots[m - 1] - e.knots[m - e.degree - 1];
      const double f0 = (span0 > 0.0) ? (span0 / pu) : (1.0 / 3.0);
      const double fN = (spanN > 0.0) ? (spanN / pu) : (1.0 / 3.0);
      for (int j = 1; j <= 2; ++j) {
        poles[static_cast<std::size_t>(1) * 4 + j] =
            addV(spoke[i][j], rib[i][j] * f0);
        poles[static_cast<std::size_t>(nu - 2) * 4 + j] =
            addV(spoke[nextI][j], rib[nextI][j] * (-fN));
      }
    }

    BsplineSurfaceData s;
    s.degreeU = e.degree;
    s.degreeV = 3;
    s.nPolesU = nu;
    s.nPolesV = 4;
    s.knotsU = e.knots;
    s.knotsV = cubicKnots();
    s.poles = std::move(poles);
    s.weights.clear();  // non-rational (the Gregory twist blend is folded into the P2 row).
    r.patches.push_back(std::move(s));
  }

  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
