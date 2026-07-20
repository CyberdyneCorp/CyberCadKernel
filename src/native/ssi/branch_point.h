// SPDX-License-Identifier: Apache-2.0
//
// branch_point.h — SSI Stage S4-d branch-point LOCALIZATION + ARM ENUMERATION.
//
// A BRANCH POINT is where the intersection LOCUS of two surfaces self-crosses: several
// real curve arms meet at one point B (the canonical case is the Steinmetz bicylinder,
// two equal orthogonal cylinders, whose intersection is TWO ellipses crossing at two
// points). The S3+S4-c marcher DETECTS such a point — approaching B the transversality
// sine ‖nA×nB‖ collapses toward 0 and the raw cross-product tangent FLIPS (two arms
// meet) — and today DEFERS (an honest NearTangent stop). S4-d turns that same detection
// into: LOCALIZE B, ENUMERATE the real outgoing arm directions, and hand them back so
// the marcher can ROUTE each arm.
//
// THIS HEADER IS THE DETECTOR + ENUMERATOR (steps 1–2 of the design); the ROUTING /
// ASSEMBLY (steps 3–4) stays in marching.cpp, which owns the corrector and the WLine
// assembly. Kept OUT of marching.cpp so the branch mathematics (a 1-D sine minimization
// + a relative-second-fundamental-form quadratic) is isolated and separately readable —
// the design flags the branch handler as the one place complexity is allowed to sit.
//
// HONESTY CORE (the reason S4-d cannot fabricate arms). The outgoing arm directions are
// the TANGENT CONE of the locus at B: in the shared tangent plane they solve a quadratic
// whose coefficients come from the RELATIVE second fundamental form H = II_A − II_B
// (each surface's normal-curvature form, projected on the shared normal). Its discriminant
// sign IS the S4-b point-vs-cross invariant:
//   * Δ > 0  (H indefinite)  ⇒ TWO distinct real tangent lines ⇒ up to FOUR outgoing rays
//                               — a genuine transversal self-crossing (route the arms).
//   * Δ ≤ 0  (H definite / semi-definite) ⇒ NO real distinct lines ⇒ NOT a transversal
//                               branch. Definite = the isolated TangentPoint (S4-b): the
//                               curve ENDS, we return NO arms. A double root (Δ==0) is a
//                               cusp — also no distinct arms — defer.
// So an isolated TangentPoint can NEVER sprout arms: the same quadratic that yields four
// rays for the bicylinder yields ZERO for two spheres touching at a point.
//
// Header-only, OCCT-FREE. Uses src/native/math + src/native/numerics (the 1-D minimize
// and the 4-vector least_squares re-projection), so — like marching.cpp — it is only
// meaningful under CYBERCAD_HAS_NUMSCI; the callers are already gated there.
//
#ifndef CYBERCAD_NATIVE_SSI_BRANCH_POINT_H
#define CYBERCAD_NATIVE_SSI_BRANCH_POINT_H

#include "native/math/vec.h"
#include "native/numerics/numerics.h"
#include "native/ssi/patch_bounds.h"  // SurfaceAdapter

#include <algorithm>  // std::max/std::min over an initializer_list
#include <array>
#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::ssi {

namespace branchpt {

namespace nn = cybercad::native::numerics;
using math::Dir3;
using math::Point3;
using math::Vec3;

// Params of an on-both-surfaces point, mirroring marching's State (kept independent so
// this header does not depend on marching internals).
struct BPState {
  double u1, v1, u2, v2;
  Point3 p;
};

// The transversality sine ‖nA×nB‖ at a state (the branch witness: → 0 at B).
inline double sineAt(const SurfaceAdapter& A, const SurfaceAdapter& B, const BPState& s) {
  const Vec3 c = math::cross(A.normal(s.u1, s.v1).vec(), B.normal(s.u2, s.v2).vec());
  return math::norm(c);
}

// Re-project a world guess onto BOTH surfaces (r = A.point − B.point → 0), seeded at the
// given params and CLAMPED to both domains — the same fixed-point re-projection the S4-c
// corrector uses. The 4-DOF / 3-residual system has an along-curve null space that would
// otherwise SLIDE the solution back to `anchor`; when `pin` is supplied we add the S4-c
// fixed-plane residual dot(A.point − anchor, pin.dir) − pin.dist so the solution is held
// at that signed distance along `pin.dir` (a hyperplane cut the curve crosses even where
// the local tangent degenerates). With no pin we drop the along-DOF (the full re-project
// of the localized minimum, which we WANT to settle onto the nearest common point B).
// Returns nullopt if it does not land on both surfaces within `onSurfTol`.
struct AlongPin {
  Vec3 dir{};
  double dist = 0.0;
};

inline std::optional<BPState> reproject(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                        const BPState& seed, double onSurfTol,
                                        const Point3& anchor,
                                        const std::optional<AlongPin>& pin) {
  const ParamBox& da = A.domain;
  const ParamBox& db = B.domain;
  auto clampd = [](double x, double lo, double hi) { return x < lo ? lo : (x > hi ? hi : x); };
  const bool aPU = A.uPeriod > 0.0, aPV = A.vPeriod > 0.0;
  const bool bPU = B.uPeriod > 0.0, bPV = B.vPeriod > 0.0;
  auto clampX = [&](const nn::Vector& x) {
    return std::array<double, 4>{aPU ? x[0] : clampd(x[0], da.u0, da.u1),
                                 aPV ? x[1] : clampd(x[1], da.v0, da.v1),
                                 bPU ? x[2] : clampd(x[2], db.u0, db.u1),
                                 bPV ? x[3] : clampd(x[3], db.v0, db.v1)};
  };
  nn::VecFn resid = [&](const nn::Vector& x) -> nn::Vector {
    const auto c = clampX(x);
    const Point3 pa = A.point(c[0], c[1]);
    const Vec3 gap = pa - B.point(c[2], c[3]);
    if (pin) return {gap.x, gap.y, gap.z, math::dot(pa - anchor, pin->dir) - pin->dist};
    return {gap.x, gap.y, gap.z};
  };
  const nn::Vector x0{seed.u1, seed.v1, seed.u2, seed.v2};
  const nn::SolveResult r = nn::least_squares(resid, x0);
  const auto c = clampX(r.x);
  const Point3 pa = A.point(c[0], c[1]);
  const Point3 pb = B.point(c[2], c[3]);
  if (math::distance(pa, pb) > onSurfTol) return std::nullopt;
  return BPState{c[0], c[1], c[2], c[3], pa};
}

// ── LOCALIZE (design S4-d-1) ─────────────────────────────────────────────────────
//
// The marcher hits the branch region and stalls with the sine collapsing (its last
// accepted node `stall` sits just off B, in the band). B is the point on the intersection
// where the sine reaches its minimum (≈ 0). We minimize the sine along a short arc through
// `stall`, sweeping a signed distance `d` off `stall` along the last-good tangent `tStar`
// (each trial re-projected onto both surfaces so it stays on the locus), then FULLY
// re-project the minimizer onto both surfaces. Accept B only if it lands on both surfaces
// AND its sine dropped to a small fraction of the entering sine (a real branch collapses;
// a false trigger does not). Returns nullopt otherwise ⇒ the caller keeps deferring.
struct Localized {
  BPState point;      ///< the branch point B, on both surfaces
  double sine = 0.0;  ///< ‖nA×nB‖ at B (≈ 0 for a true branch)
};

inline std::optional<Localized> localize(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                         const BPState& stall, const Vec3& tStar,
                                         double enterSine, double h0, double onSurfTol) {
  // 1-D objective: signed distance d along tStar off `stall`. The trial is re-projected
  // onto both surfaces PINNED at distance d along tStar from `stall` (the S4-c fixed-plane
  // cut) so it stays on the locus at that offset instead of sliding back to the stall; then
  // we read the sine there. A trial that will not re-project is penalized (steer away).
  auto sineAlong = [&](double d) -> double {
    BPState g = stall;
    g.p = stall.p + tStar * d;
    const AlongPin pin{tStar, d};
    if (const auto s = reproject(A, B, g, onSurfTol * 64.0, stall.p, pin)) return sineAt(A, B, *s);
    return 1.0;  // off-surface trial → large sine, steer away
  };
  // The re-projected on-locus position at signed offset d (for the final settle seed).
  auto stateAlong = [&](double d) -> std::optional<BPState> {
    BPState g = stall;
    g.p = stall.p + tStar * d;
    return reproject(A, B, g, onSurfTol * 64.0, stall.p, AlongPin{tStar, d});
  };
  // Bracket B within ±reach of the stall (it entered the band after overshooting B, so B
  // is within a couple of steps either side); seed the BFGS at the better coarse sample.
  const double reach = h0 * 2.0;
  double d0 = 0.0, best = sineAlong(0.0);
  const int nCoarse = 32;
  for (int k = -nCoarse; k <= nCoarse; ++k) {
    const double d = reach * (static_cast<double>(k) / nCoarse);
    const double s = sineAlong(d);
    if (s < best) { best = s; d0 = d; }
  }
  nn::ObjFn f = [&](const nn::Vector& x) { return sineAlong(x[0]); };
  const nn::SolveResult m = nn::minimize(f, nn::Vector{d0});
  const double dStar = m.x.empty() ? d0 : m.x[0];

  // Full re-project of the minimizer onto both surfaces → the branch point B. Seed from the
  // PINNED on-locus position at dStar (so we start next to B, not at the stall), then drop
  // the pin so the corrector settles onto the nearest common point — the branch point.
  const auto atMin = stateAlong(dStar);
  BPState g = atMin ? *atMin : stall;
  if (!atMin) g.p = stall.p + tStar * dStar;
  const auto Bopt = reproject(A, B, g, onSurfTol, g.p, std::nullopt);
  if (!Bopt) return std::nullopt;
  const double sB = sineAt(A, B, *Bopt);
  // A genuine branch: the sine at B has collapsed to a small fraction of the entering
  // transversality (the two arms truly meet). A shallow dip that never reached ≈0 is not
  // a branch — defer. 0.1·enterSine sits well below any crossable graze's bounded dip.
  if (!(sB <= 0.1 * enterSine)) return std::nullopt;
  return Localized{*Bopt, sB};
}

// ── ENUMERATE ARMS (design S4-d-2) ───────────────────────────────────────────────
//
// At B the intersection locus has a TANGENT CONE. In the shared tangent plane (spanned by
// e1,e2 ⟂ the mean normal N) a curve direction d = cosθ·e1 + sinθ·e2 is tangent to the
// locus iff the two surfaces' NORMAL CURVATURES agree along d — i.e. the RELATIVE second
// fundamental form H(d,d) = II_A(d,d) − II_B(d,d) VANISHES. Writing m = tanθ this is the
// quadratic  c·m² + 2b·m + a = 0  with a=H(e1,e1), b=H(e1,e2), c=H(e2,e2). Its discriminant
// Δ = b² − a·c:
//   Δ > 0 ⇒ two distinct real roots ⇒ two tangent LINES ⇒ four rays (±T1, ±T2).
//   Δ ≤ 0 ⇒ no distinct real lines ⇒ NOT a transversal branch (definite = isolated
//           TangentPoint → curve ends; double root = cusp) ⇒ return EMPTY.
// The Hessians are taken by CENTRAL FINITE DIFFERENCES of each surface's signed height
// over its own tangent frame, projected on the shared normal N — no analytic 2nd
// derivatives are needed from the adapter (it exposes only point/normal).
//
// Returns the outgoing unit rays (4 for a transversal self-crossing, empty otherwise).
// The shared tangent frame at B: the mean unit normal N (nA ∥ ±nB at a tangency) and an
// orthonormal tangent-plane basis e1,e2 ⟂ N. Returns false if the mean normal degenerates.
struct TangentFrame {
  Vec3 N, e1, e2;
};
inline bool sharedTangentFrame(const SurfaceAdapter& A, const SurfaceAdapter& B,
                               const BPState& s, TangentFrame& out) {
  const Vec3 nA = A.normal(s.u1, s.v1).vec();
  Vec3 nB = B.normal(s.u2, s.v2).vec();
  if (math::dot(nA, nB) < 0.0) nB = -nB;  // flip nB to agree with nA
  Vec3 Nv = nA + nB;
  const double nlen = math::norm(Nv);
  if (!(nlen > 1e-12)) return false;
  Nv = Nv / nlen;
  const Vec3 seed = std::fabs(Nv.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 e1 = seed - Nv * math::dot(seed, Nv);
  e1 = e1 / math::norm(e1);
  out.N = Nv;
  out.e1 = e1;
  out.e2 = math::cross(Nv, e1);
  return true;
}

// Signed height of surface S OFF the shared tangent plane at B, a distance `delta` along a
// tangent direction. The surface bows off its tangent plane by ≈ ½·κ_n·δ² along N; we get
// the on-surface height by re-projecting the tangent-plane target B + δ·t onto S (two
// residuals pin the tangent-plane position, leaving S free to bow along N) and read the
// N-component of (foot − B).
inline double surfHeight(const SurfaceAdapter& S, const BPState& s, const TangentFrame& F,
                         double su, double sv, const Vec3& tanDir, double delta) {
  const ParamBox& d = S.domain;
  auto clampd = [](double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); };
  const bool pu = S.uPeriod > 0.0, pv = S.vPeriod > 0.0;
  const Point3 target = s.p + tanDir * delta;
  nn::VecFn resid = [&](const nn::Vector& xx) -> nn::Vector {
    const double cu = pu ? xx[0] : clampd(xx[0], d.u0, d.u1);
    const double cv = pv ? xx[1] : clampd(xx[1], d.v0, d.v1);
    const Vec3 off = S.point(cu, cv) - target;
    return {math::dot(off, F.e1), math::dot(off, F.e2)};
  };
  const nn::SolveResult r = nn::least_squares(resid, nn::Vector{su, sv});
  const double cu = pu ? r.x[0] : clampd(r.x[0], d.u0, d.u1);
  const double cv = pv ? r.x[1] : clampd(r.x[1], d.v0, d.v1);
  return math::dot(S.point(cu, cv) - s.p, F.N);
}

// The relative second fundamental form H = II_A − II_B in the tangent frame, as (a,b,c) with
// a=H(e1,e1), b=H(e1,e2), c=H(e2,e2). Each κ ≈ (h(+δ)+h(−δ))/δ² (central; linear term
// cancels); b is recovered from the diagonal κ: H(diag,diag)=½(a+c)+b ⇒ b = H_diag − ½(a+c).
struct RelForm {
  double a, b, c;
};
inline RelForm relativeSecondForm(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                  const BPState& s, const TangentFrame& F, double scale) {
  const double hstep = std::max(scale * 1e-3, 1e-7);
  auto kappa = [&](const Vec3& t) {
    const double hA = surfHeight(A, s, F, s.u1, s.v1, t, +hstep) + surfHeight(A, s, F, s.u1, s.v1, t, -hstep);
    const double hB = surfHeight(B, s, F, s.u2, s.v2, t, +hstep) + surfHeight(B, s, F, s.u2, s.v2, t, -hstep);
    return (hA - hB) / (hstep * hstep);  // relative normal curvature along t
  };
  RelForm h;
  h.a = kappa(F.e1);
  h.c = kappa(F.e2);
  Vec3 diag = F.e1 + F.e2;
  diag = diag / math::norm(diag);
  h.b = kappa(diag) - 0.5 * (h.a + h.c);
  return h;
}

// Solve the tangent-cone quadratic c·m² + 2b·m + a = 0 (m = tanθ from e1) for the outgoing
// arm rays. Δ = b² − ac must be POSITIVE by a clear margin (the S4-b point-vs-cross gate): a
// near-zero / negative Δ means the relative form is definite (isolated TangentPoint → curve
// ends) or a double root (cusp → defer) — NO arms. Δ > 0 ⇒ two distinct tangent lines ⇒ the
// four rays ±T1, ±T2. Returns empty when there is no genuine transversal self-crossing.
inline std::vector<Vec3> solveTangentCone(const RelForm& h, const TangentFrame& F) {
  const double disc = h.b * h.b - h.a * h.c;
  const double kscale = std::max({std::fabs(h.a), std::fabs(h.b), std::fabs(h.c), 1e-30});
  if (!(disc > 1e-6 * kscale * kscale)) return {};
  const double sq = std::sqrt(disc);
  std::array<Vec3, 2> lines{};
  int n = 0;
  auto addLine = [&](const Vec3& raw) {
    const double len = math::norm(raw);
    if (len > 1e-12) lines[n++] = raw / len;
  };
  if (std::fabs(h.c) > 1e-12 * kscale) {
    addLine(F.e1 + F.e2 * ((-h.b + sq) / h.c));
    addLine(F.e1 + F.e2 * ((-h.b - sq) / h.c));
  } else {                                              // c≈0: root m→∞ (dir e2) + m=−a/2b
    addLine(F.e2);
    if (std::fabs(h.b) > 1e-12 * kscale) addLine(F.e1 + F.e2 * (-h.a / (2.0 * h.b)));
  }
  if (n < 2) return {};                                 // could not form two distinct lines
  return {lines[0], lines[0] * -1.0, lines[1], lines[1] * -1.0};
}

inline std::vector<Vec3> enumerateArms(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                       const Localized& L, double scale) {
  TangentFrame F;
  if (!sharedTangentFrame(A, B, L.point, F)) return {};
  const RelForm h = relativeSecondForm(A, B, L.point, F, scale);
  return solveTangentCone(h, F);
}

}  // namespace branchpt

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_BRANCH_POINT_H
