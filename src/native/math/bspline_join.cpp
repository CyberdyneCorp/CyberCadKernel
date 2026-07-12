// SPDX-License-Identifier: Apache-2.0
//
// bspline_join.cpp — NURBS roadmap Layer 6 (surface G1/G2 continuity JOIN) implementation.
//
// Reposition the near-boundary control rows of two already-built adjacent NURBS patches so
// they meet G1 (tangent-plane) or G2 (curvature) across a shared edge, with MINIMAL control-
// point movement and the shared boundary curve frozen. Clean-room from the cross-boundary
// continuity conditions (Piegl & Tiller ch.10; Farin). See the header for the full derivation.
//
// APPROACH (per along-edge station i, after makeCompatibleAlongEdge aligns the along-edge
// degree + knots so A and B share the same boundary control-point count):
//   * Extract three control rows parallel to the shared edge: row 0 (boundary, frozen),
//     row 1 (drives the cross-boundary tangent), row 2 (drives the cross-boundary 2nd deriv).
//   * Offsets into A's interior:  A1off = A_row1 - P0,   A2off = A_row2 - P0.
//   * G1 SHARED RIBBON — set B_row1 = P0 - s·A1off so B's cross-tangent is antiparallel-
//     collinear to A's ⇒ the two tangent planes coincide ⇒ unit normal continuous. The single
//     global s>0 is the closed-form least-squares minimiser of the total row-1 movement.
//   * G2 SHARED RIBBON — set B_row2 = P0 + s²·A2off - 2s(s+1)·A1off so the cross-boundary
//     second difference of B equals s²·(that of A) ⇒ ∂²S/∂n² continuous, G1 preserved.
//   * NO-OP GUARD — measure the achieved residual FIRST; if already below tol, return with
//     movement 0 (the coplanar / cylinder-split analytic cases, exact by construction).
//   * MOVEMENT CAP — if the required max displacement exceeds the caller cap, HONEST-DECLINE
//     (would distort the surface); never widen a tolerance to pass.
//
// Guarded by CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated Layer-6
// surfacing family (it composes the numsci-gated Layer-1 ops). No linear solve — the row
// placement is direct + one scalar LS. With the guard OFF the TU is inert.
//
#include "native/math/bspline_join.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"      // surfaceNormal / surfaceDerivs (residual measurement)
#include "native/math/bspline_ops.h"  // elevateDegreeSurface / refineKnotSurface (compat)

#include <algorithm>
#include <cmath>
#include <span>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

// ── Surface accessors ────────────────────────────────────────────────────────────────
Point3 pole(const BsplineSurfaceData& s, int i, int j) {
  return s.poles[static_cast<std::size_t>(i) * s.nPolesV + j];
}
void setPole(BsplineSurfaceData& s, int i, int j, const Point3& p) {
  s.poles[static_cast<std::size_t>(i) * s.nPolesV + j] = p;
}

bool wellFormed(const BsplineSurfaceData& s) {
  if (s.degreeU < 1 || s.degreeV < 1 || s.nPolesU < 2 || s.nPolesV < 2) return false;
  if (static_cast<int>(s.poles.size()) != s.nPolesU * s.nPolesV) return false;
  if (static_cast<int>(s.knotsU.size()) != s.nPolesU + s.degreeU + 1) return false;
  if (static_cast<int>(s.knotsV.size()) != s.nPolesV + s.degreeV + 1) return false;
  if (!s.weights.empty() && static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV)
    return false;
  return true;
}

Vec3 sub(const Point3& a, const Point3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Point3 addV(const Point3& p, const Vec3& v) { return {p.x + v.x, p.y + v.y, p.z + v.z}; }

// The along-edge parameter direction for a given shared edge: U-edges (U0/U1) run along V;
// V-edges (V0/V1) run along U. Return the along-edge pole count.
bool edgeIsUDir(SurfaceEdge e) { return e == SurfaceEdge::U0 || e == SurfaceEdge::U1; }
int alongCount(const BsplineSurfaceData& s, SurfaceEdge e) {
  return edgeIsUDir(e) ? s.nPolesV : s.nPolesU;
}
int alongDegree(const BsplineSurfaceData& s, SurfaceEdge e) {
  return edgeIsUDir(e) ? s.degreeV : s.degreeU;
}
// The number of control ROWS available in the cross direction (perpendicular to the edge).
int crossCount(const BsplineSurfaceData& s, SurfaceEdge e) {
  return edgeIsUDir(e) ? s.nPolesU : s.nPolesV;
}

// Fetch the pole on cross-row `r` (0 = boundary, 1 = second, …) at along-edge station `i`.
// `r` is measured INWARD from the shared boundary.
Point3 rowPole(const BsplineSurfaceData& s, SurfaceEdge e, int r, int i) {
  switch (e) {
    case SurfaceEdge::U0: return pole(s, r, i);                     // rows go +U inward
    case SurfaceEdge::U1: return pole(s, s.nPolesU - 1 - r, i);     // rows go -U inward
    case SurfaceEdge::V0: return pole(s, i, r);                     // cols go +V inward
    case SurfaceEdge::V1: return pole(s, i, s.nPolesV - 1 - r);     // cols go -V inward
  }
  return {};
}
void setRowPole(BsplineSurfaceData& s, SurfaceEdge e, int r, int i, const Point3& p) {
  switch (e) {
    case SurfaceEdge::U0: setPole(s, r, i, p); return;
    case SurfaceEdge::U1: setPole(s, s.nPolesU - 1 - r, i, p); return;
    case SurfaceEdge::V0: setPole(s, i, r, p); return;
    case SurfaceEdge::V1: setPole(s, i, s.nPolesV - 1 - r, p); return;
  }
}

// Map B's along-edge station to A's, honouring a reversed parameter direction.
int mapStation(int i, int n, bool reversed) { return reversed ? (n - 1 - i) : i; }

// ── Residual measurement (unit-normal + normal-curvature mismatch across the edge) ──────
// Evaluate the surface derivatives at a boundary station and return (normal, kappa_n along
// the edge). For G1 we compare unit normals; for G2 we compare the normal curvature of the
// cross-boundary curve. Non-rational path (the common join case).

struct EdgeSample {
  Dir3 normal;
  double kappaN = 0.0;   // normal curvature of the cross-boundary iso-curve
  bool valid = false;
};

// (u,v) of a boundary station on edge `e` at along-parameter t in [0,1].
void edgeParam(const BsplineSurfaceData& s, SurfaceEdge e, double t, double& u, double& v) {
  const auto& kU = s.knotsU; const auto& kV = s.knotsV;
  const double u0 = kU.front(), u1 = kU.back(), v0 = kV.front(), v1 = kV.back();
  switch (e) {
    case SurfaceEdge::U0: u = u0; v = v0 + t * (v1 - v0); return;
    case SurfaceEdge::U1: u = u1; v = v0 + t * (v1 - v0); return;
    case SurfaceEdge::V0: u = v = 0; u = u0 + t * (u1 - u0); v = v0; return;
    case SurfaceEdge::V1: u = u0 + t * (u1 - u0); v = v1; return;
  }
}

EdgeSample sampleEdge(const BsplineSurfaceData& s, SurfaceEdge e, double t) {
  double u, v; edgeParam(s, e, t, u, v);
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  const int md = 2;
  std::vector<Vec3> d(static_cast<std::size_t>(md + 1) * (md + 1));
  surfaceDerivs(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v, md, d);
  auto D = [&](int k, int l) { return d[static_cast<std::size_t>(k) * (md + 1) + l]; };
  const Vec3 Su = D(1, 0), Sv = D(0, 1);
  const Vec3 nrm = cross(Su, Sv);
  EdgeSample out;
  if (isNull(nrm, 1e-14)) return out;
  const Dir3 n = Dir3(nrm);
  out.normal = n;
  // Normal curvature of the CROSS-boundary direction (into the surface, perpendicular to the
  // shared edge). For a U-edge the cross direction is U (use S_uu); for a V-edge it is V
  // (S_vv). kappa_n = (S_cc · n) / (S_c · S_c).
  Vec3 Sc, Scc;
  if (edgeIsUDir(e)) { Sc = Su; Scc = D(2, 0); } else { Sc = Sv; Scc = D(0, 2); }
  const double denom = dot(Sc, Sc);
  out.kappaN = (denom > 1e-30) ? dot(Scc, n.vec()) / denom : 0.0;
  out.valid = true;
  return out;
}

// Worst unit-normal mismatch (rad, sign-insensitive) across the edge over nS stations.
double measureG1Residual(const BsplineSurfaceData& A, const BsplineSurfaceData& B,
                         const EdgeSpec& edge, int nS = 40) {
  double worst = 0.0;
  for (int k = 0; k <= nS; ++k) {
    const double tA = static_cast<double>(k) / nS;
    const double tB = edge.reversed ? (1.0 - tA) : tA;
    const EdgeSample sa = sampleEdge(A, edge.edgeA, tA);
    const EdgeSample sb = sampleEdge(B, edge.edgeB, tB);
    if (!sa.valid || !sb.valid) continue;
    const double ang = sa.normal.angle(sb.normal);
    worst = std::max(worst, std::min(ang, M_PI - ang));
  }
  return worst;
}

// Worst RELATIVE normal-curvature mismatch across the edge.
double measureG2Residual(const BsplineSurfaceData& A, const BsplineSurfaceData& B,
                         const EdgeSpec& edge, int nS = 40) {
  double worst = 0.0;
  for (int k = 0; k <= nS; ++k) {
    const double tA = static_cast<double>(k) / nS;
    const double tB = edge.reversed ? (1.0 - tA) : tA;
    const EdgeSample sa = sampleEdge(A, edge.edgeA, tA);
    const EdgeSample sb = sampleEdge(B, edge.edgeB, tB);
    if (!sa.valid || !sb.valid) continue;
    const double scale = std::max({std::abs(sa.kappaN), std::abs(sb.kappaN), 1e-9});
    worst = std::max(worst, std::abs(sa.kappaN - sb.kappaN) / scale);
  }
  return worst;
}

// Shared-boundary coincidence deviation (max distance A_boundary[i] ↔ B_boundary[mapped]).
double boundaryCoincidence(const BsplineSurfaceData& A, const BsplineSurfaceData& B,
                           const EdgeSpec& edge, int n) {
  double worst = 0.0;
  for (int i = 0; i < n; ++i) {
    const int ai = mapStation(i, n, edge.reversed);
    worst = std::max(worst, distance(rowPole(A, edge.edgeA, 0, ai), rowPole(B, edge.edgeB, 0, i)));
  }
  return worst;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// makeCompatibleAlongEdge — knot-merge + degree-match along the shared edge.
// ─────────────────────────────────────────────────────────────────────────────
EdgeCompatResult makeCompatibleAlongEdge(const BsplineSurfaceData& A0,
                                         const BsplineSurfaceData& B0,
                                         const EdgeSpec& edge, double tol) {
  EdgeCompatResult r;
  if (!wellFormed(A0)) { r.reason = "A malformed"; return r; }
  if (!wellFormed(B0)) { r.reason = "B malformed"; return r; }
  if (A0.weights.empty() != B0.weights.empty()) {
    r.reason = "rationality mismatch (one patch rational, one not)"; return r;
  }
  BsplineSurfaceData A = A0, B = B0;

  const ParamDir dirA = edgeIsUDir(edge.edgeA) ? ParamDir::V : ParamDir::U;
  const ParamDir dirB = edgeIsUDir(edge.edgeB) ? ParamDir::V : ParamDir::U;

  // 1. Degree-match along the edge (elevate the lower-degree side; exact).
  const int dA = alongDegree(A, edge.edgeA), dB = alongDegree(B, edge.edgeB);
  const int dShared = std::max(dA, dB);
  if (dA < dShared) A = elevateDegreeSurface(A, dirA, dShared - dA);
  if (dB < dShared) B = elevateDegreeSurface(B, dirB, dShared - dB);

  // 2. Knot-merge along the edge: refine each side by the OTHER side's interior knots so both
  //    carry the union knot vector (exact — the geometry is unchanged). Along-edge knots must
  //    live on the same [0,1]-normalised domain for the merge to be meaningful; we compare on
  //    the shared normalised parameter, mapping B's if reversed is not required here because a
  //    refine only inserts values, geometry-preserving.
  auto alongKnots = [](const BsplineSurfaceData& s, SurfaceEdge e) {
    return edgeIsUDir(e) ? s.knotsV : s.knotsU;
  };
  const std::vector<double> kA = alongKnots(A, edge.edgeA);
  const std::vector<double> kB = alongKnots(B, edge.edgeB);
  const double a0 = kA.front(), a1 = kA.back(), b0 = kB.front(), b1 = kB.back();
  const double aSpan = a1 - a0, bSpan = b1 - b0;
  if (!(aSpan > 0) || !(bSpan > 0)) { r.reason = "degenerate along-edge knot domain"; return r; }

  // Map B's interior knots onto A's domain (affine), insert into A; and vice-versa.
  const int dS = alongDegree(A, edge.edgeA);
  auto interior = [dS](const std::vector<double>& k) {
    std::vector<double> in;
    for (int i = dS + 1; i + dS + 1 < static_cast<int>(k.size()); ++i) in.push_back(k[i]);
    return in;
  };
  const std::vector<double> inA = interior(kA);
  const std::vector<double> inB = interior(kB);
  // Insert B's interior (remapped to A's domain) that A lacks, into A; symmetric for B.
  auto remap = [](double t, double s0, double s1, double d0, double d1) {
    return d0 + (t - s0) / (s1 - s0) * (d1 - d0);
  };
  auto missing = [&](const std::vector<double>& have, const std::vector<double>& want) {
    std::vector<double> add;
    for (double w : want) {
      int cntHave = 0, cntWant = 0;
      for (double h : have) if (std::abs(h - w) <= tol * std::max(1.0, std::abs(w))) ++cntHave;
      for (double x : want) if (std::abs(x - w) <= tol * std::max(1.0, std::abs(w))) ++cntWant;
      int already = 0;
      for (double x : add) if (std::abs(x - w) <= tol * std::max(1.0, std::abs(w))) ++already;
      if (cntHave + already < cntWant) add.push_back(w);
    }
    return add;
  };
  std::vector<double> inBonA;
  for (double t : inB) inBonA.push_back(remap(t, b0, b1, a0, a1));
  std::vector<double> inAonB;
  for (double t : inA) inAonB.push_back(remap(t, a0, a1, b0, b1));
  const std::vector<double> addToA = missing(inA, inBonA);
  const std::vector<double> addToB = missing(inB, inAonB);
  if (!addToA.empty()) {
    std::vector<double> s = addToA; std::sort(s.begin(), s.end());
    A = refineKnotSurface(A, dirA, s);
  }
  if (!addToB.empty()) {
    std::vector<double> s = addToB; std::sort(s.begin(), s.end());
    B = refineKnotSurface(B, dirB, s);
  }

  // 3. Verify along-edge pole counts now match and the shared boundary is C0-coincident.
  const int nA = alongCount(A, edge.edgeA), nB = alongCount(B, edge.edgeB);
  if (nA != nB) { r.reason = "along-edge pole counts irreconcilable after knot merge"; return r; }
  if (boundaryCoincidence(A, B, edge, nA) > std::max(tol, 1e-7)) {
    r.reason = "shared boundary not coincident (patches do not meet C0 along the edge)"; return r;
  }
  r.ok = true; r.A = std::move(A); r.B = std::move(B);
  r.sharedDegree = dShared; r.sharedPoles = nA;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// joinG1 — minimal-movement tangent-plane join.
// ─────────────────────────────────────────────────────────────────────────────
JoinResult joinG1(const BsplineSurfaceData& A0, const BsplineSurfaceData& B0,
                  const EdgeSpec& edge, double maxMovementCap, bool adjustBoth, double tol) {
  JoinResult out;
  EdgeCompatResult comp = makeCompatibleAlongEdge(A0, B0, edge, tol);
  if (!comp.ok) { out.reason = comp.reason; return out; }
  BsplineSurfaceData A = comp.A, B = comp.B;
  const int n = comp.sharedPoles;

  // Need a second cross-row on both sides to carry the tangent.
  if (crossCount(A, edge.edgeA) < 2 || crossCount(B, edge.edgeB) < 2) {
    out.reason = "a patch has no second control row (cross-degree < 1)"; return out;
  }

  out.boundaryDev = boundaryCoincidence(A, B, edge, n);

  // NO-OP GUARD: already G1 within tol ⇒ nothing to move (coplanar / analytic split cases).
  const double res0 = measureG1Residual(A, B, edge);
  if (res0 <= std::max(tol, 1e-7)) {
    out.ok = true; out.noop = true; out.A = std::move(A); out.B = std::move(B);
    out.continuityResidual = res0; out.maxMovement = 0.0;
    return out;
  }

  // Closed-form minimal-movement scalar s>0 for the shared ribbon B1off = -s·A1off.
  // Minimise Σ|B_row1[i] - (P0 - s·A1off[i])|² over s ⇒ s = Σ(d·g)/Σ(g·g), d=B1cur-P0, g=-A1off.
  double num = 0.0, den = 0.0, dd = 0.0;
  for (int i = 0; i < n; ++i) {
    const int ai = mapStation(i, n, edge.reversed);
    const Point3 P0 = rowPole(A, edge.edgeA, 0, ai);
    const Vec3 A1off = sub(rowPole(A, edge.edgeA, 1, ai), P0);
    const Vec3 g = A1off * -1.0;
    const Vec3 d = sub(rowPole(B, edge.edgeB, 1, i), P0);
    num += dot(d, g); den += dot(g, g); dd += dot(d, d);
  }
  if (!(den > 0)) { out.reason = "degenerate cross-tangent ribbon on A (zero row-1 offset)"; return out; }
  double s = num / den;
  if (!(s > 1e-9)) s = std::sqrt(std::max(dd, 0.0) / den);  // magnitude-match fallback (>0)
  if (!(s > 1e-12)) { out.reason = "cannot form a positive tangent proportionality"; return out; }

  // Place B's row 1 (and optionally split the move with A) and track max displacement.
  double maxMove = 0.0;
  for (int i = 0; i < n; ++i) {
    const int ai = mapStation(i, n, edge.reversed);
    const Point3 P0 = rowPole(A, edge.edgeA, 0, ai);
    const Vec3 A1off = sub(rowPole(A, edge.edgeA, 1, ai), P0);
    const Point3 Bcur = rowPole(B, edge.edgeB, 1, i);
    const Point3 Btarget = addV(P0, A1off * -s);
    if (!adjustBoth) {
      maxMove = std::max(maxMove, distance(Bcur, Btarget));
      setRowPole(B, edge.edgeB, 1, i, Btarget);
    } else {
      // Split: move B halfway to a common ribbon and A the other half, keeping collinearity.
      // Common offset r chosen so A_row1 = P0 + r, B_row1 = P0 - s·r with r = 0.5(A1off + (-1/s)(Bcur-P0))·?
      // Simpler symmetric scheme: average the two implied ribbons then re-impose collinearity.
      const Vec3 rB = (Btarget - P0);                 // desired B offset (= -s·A1off)
      const Vec3 Acur = sub(rowPole(A, edge.edgeA, 1, ai), P0);
      const Vec3 rAnew = (Acur + (rB * (-1.0 / s))) * 0.5;   // blended A offset
      const Point3 Anew = addV(P0, rAnew);
      const Point3 Bnew = addV(P0, rAnew * -s);
      maxMove = std::max({maxMove, distance(rowPole(A, edge.edgeA, 1, ai), Anew),
                          distance(Bcur, Bnew)});
      setRowPole(A, edge.edgeA, 1, ai, Anew);
      setRowPole(B, edge.edgeB, 1, i, Bnew);
    }
  }

  if (maxMove > maxMovementCap) {
    out.reason = "G1 needs control movement beyond cap (would distort the surface)";
    out.maxMovement = maxMove; return out;
  }

  out.ok = true; out.A = std::move(A); out.B = std::move(B);
  out.maxMovement = maxMove;
  out.continuityResidual = measureG1Residual(out.A, out.B, edge);
  out.boundaryDev = boundaryCoincidence(out.A, out.B, edge, n);
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// joinG2 — additionally match the second cross-derivative.
// ─────────────────────────────────────────────────────────────────────────────
JoinResult joinG2(const BsplineSurfaceData& A0, const BsplineSurfaceData& B0,
                  const EdgeSpec& edge, double maxMovementCap, bool adjustBoth, double tol) {
  JoinResult out;
  // Need a third cross-row on both sides (along-edge cross-degree ≥ 2) to carry curvature.
  {
    EdgeCompatResult pc = makeCompatibleAlongEdge(A0, B0, edge, tol);
    if (!pc.ok) { out.reason = pc.reason; return out; }
    if (crossCount(pc.A, edge.edgeA) < 3 || crossCount(pc.B, edge.edgeB) < 3) {
      out.reason = "a patch has no third control row (cross-degree < 2 cannot carry G2)";
      return out;
    }
  }

  // NO-OP GUARD: already G2 (and G1) within tol ⇒ nothing to move (analytic cylinder split).
  {
    EdgeCompatResult pc = makeCompatibleAlongEdge(A0, B0, edge, tol);
    const double g1res = measureG1Residual(pc.A, pc.B, edge);
    const double g2res = measureG2Residual(pc.A, pc.B, edge);
    if (g1res <= std::max(tol, 1e-7) && g2res <= 1e-5) {
      out.ok = true; out.noop = true; out.A = pc.A; out.B = pc.B;
      out.continuityResidual = g2res; out.maxMovement = 0.0;
      out.boundaryDev = boundaryCoincidence(pc.A, pc.B, edge, pc.sharedPoles);
      return out;
    }
  }

  // First enforce G1 (row 1 → collinear ribbon, scalar s). Reuse joinG1 with the same cap.
  JoinResult g1 = joinG1(A0, B0, edge, maxMovementCap, adjustBoth, tol);
  if (!g1.ok) { out.reason = g1.reason; return out; }
  BsplineSurfaceData A = g1.A, B = g1.B;
  const int n = static_cast<int>(edgeIsUDir(edge.edgeA) ? A.nPolesV : A.nPolesU);

  // Recover the s actually used from the placed row 1 (B1off = -s·A1off) as a robust average.
  double num = 0.0, den = 0.0;
  for (int i = 0; i < n; ++i) {
    const int ai = mapStation(i, n, edge.reversed);
    const Point3 P0 = rowPole(A, edge.edgeA, 0, ai);
    const Vec3 A1off = sub(rowPole(A, edge.edgeA, 1, ai), P0);
    const Vec3 B1off = sub(rowPole(B, edge.edgeB, 1, i), P0);
    num += -dot(B1off, A1off); den += dot(A1off, A1off);
  }
  double s = (den > 0) ? num / den : 1.0;
  if (!(s > 1e-12)) s = 1.0;

  // Row 2 shared ribbon: B2off = s²·A2off - 2s(s+1)·A1off (see header derivation). Preserves
  // G1 (row 1 untouched) and C0 (row 0 frozen).
  double maxMove = g1.maxMovement;
  for (int i = 0; i < n; ++i) {
    const int ai = mapStation(i, n, edge.reversed);
    const Point3 P0 = rowPole(A, edge.edgeA, 0, ai);
    const Vec3 A1off = sub(rowPole(A, edge.edgeA, 1, ai), P0);
    const Vec3 A2off = sub(rowPole(A, edge.edgeA, 2, ai), P0);
    const Vec3 B2off = A2off * (s * s) + A1off * (-2.0 * s * (s + 1.0));
    const Point3 Bcur = rowPole(B, edge.edgeB, 2, i);
    const Point3 Btarget = addV(P0, B2off);
    maxMove = std::max(maxMove, distance(Bcur, Btarget));
    setRowPole(B, edge.edgeB, 2, i, Btarget);
  }

  if (maxMove > maxMovementCap) {
    out.reason = "G2 needs control movement beyond cap (would distort the surface)";
    out.maxMovement = maxMove; return out;
  }

  out.ok = true; out.A = std::move(A); out.B = std::move(B);
  out.maxMovement = maxMove;
  out.continuityResidual = measureG2Residual(out.A, out.B, edge);
  out.boundaryDev = boundaryCoincidence(out.A, out.B, edge, n);
  return out;
}

}  // namespace cybercad::native::math

#else   // CYBERCAD_HAS_NUMSCI not defined — TU inert.
namespace cybercad::native::math {}
#endif  // CYBERCAD_HAS_NUMSCI
