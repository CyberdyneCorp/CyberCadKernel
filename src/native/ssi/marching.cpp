// SPDX-License-Identifier: Apache-2.0
//
// marching.cpp — SSI Stage S3 predictor-corrector marching-line tracer.
//
// See marching.h for the method + honest-scope contract. Clean-room from
// SSI-ROADMAP S3; OCCT IntWalk_PWalking / IntPatch WLine as the reference oracle only
// (the deflection step, two-surface Newton corrector and loop/boundary termination
// are the same scheme, re-derived, never copied).
//
// The whole file is under CYBERCAD_HAS_NUMSCI — the corrector (least_squares) and the
// B-spline fit (lstsq) both call the native-numerics substrate. With the guard OFF the
// declarations in marching.h stay visible but this TU is not compiled (like
// seeding.cpp's refine half), so a kernel built without NumPP/SciPP will not resolve
// these symbols by design.
//
#include "native/ssi/marching.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/numerics/numerics.h"
#include "native/ssi/branch_point.h"    // S4-d: branch-point localize + arm enumerate
#include "native/ssi/tangent_seeded.h"  // S4-b: type a near-tangent stop

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace cybercad::native::ssi {

namespace {

namespace nn = cybercad::native::numerics;

using math::Dir3;
using math::Point3;
using math::Vec3;

inline double clampd(double x, double lo, double hi) noexcept {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ─────────────────────────────────────────────────────────────────────────────
// Resolved (scale-scaled) marching parameters — the sentinel MarchOptions expanded
// once at call time so the hot loop reads plain doubles.
// ─────────────────────────────────────────────────────────────────────────────
struct Tuned {
  double h0, minStep, maxStep, onSurfTol, maxDeflection, tangentSinTol, loopClose;
  double bandEnterSin, bandExitSin;   ///< S4-c crossing-band thresholds (tangentSinTol-derived)
  double minCrossSine;                ///< S4-c floor: sine below this mid-crossing ⇒ genuine tangency/branch, defer
  int maxPoints, crossMaxSteps;
  bool enableBranchPoints = false;    ///< S4-d: capture branch stalls for localization + arm routing
};

Tuned tune(const MarchOptions& o, double scale) {
  Tuned t;
  t.h0            = o.initialStep  > 0 ? o.initialStep  : scale * (1.0 / 64.0);
  t.minStep       = o.minStep      > 0 ? o.minStep      : scale * 1e-6;
  t.maxStep       = o.maxStep      > 0 ? o.maxStep      : scale * (1.0 / 8.0);
  t.onSurfTol     = o.onSurfTol    > 0 ? o.onSurfTol    : scale * 1e-7;
  t.maxDeflection = o.maxDeflection> 0 ? o.maxDeflection: scale * 1e-3;
  t.tangentSinTol = o.tangentSinTol;
  t.loopClose     = std::max(1.0, o.loopCloseFrac);
  // S4-c band: ENTER the fixed-plane crossing exactly where the S3 gate would STOP
  // (sine < tangentSinTol) — the crossing takes over the same near-tangent region S3
  // truncates — and EXIT once the far side recovers safely above that stop threshold
  // (1.5·tangentSinTol). Both derive from tangentSinTol — no independent tolerance is
  // introduced or weakened; the band only decides WHERE the fixed-plane corrector runs.
  t.bandEnterSin  = o.bandEnterSin > 0 ? o.bandEnterSin : o.tangentSinTol;
  t.bandExitSin   = o.bandExitSin  > 0 ? o.bandExitSin  : 1.5 * o.tangentSinTol;
  if (t.bandExitSin <= t.bandEnterSin) t.bandExitSin = t.bandEnterSin * 1.5;
  // A CROSSABLE single-branch graze keeps a bounded minimum sine (it grazes but never
  // truly touches — its dip is a fair fraction of the band-enter threshold); a genuine
  // tangency / BRANCH POINT drives sine → 0. So a band whose MINIMUM sine falls below this
  // fraction of the enter threshold is NOT a crossable graze — defer (honest S4-d/tangency
  // handoff). 0.3·enter sits below a real graze's dip yet far above a true tangency's 0.
  t.minCrossSine  = 0.3 * t.bandEnterSin;
  t.maxPoints     = std::max(16, o.maxPoints);
  t.crossMaxSteps = std::max(1, o.crossMaxSteps);
  t.enableBranchPoints = o.enableBranchPoints;
  return t;
}

// A point on the intersection being walked: 3D position + params on both surfaces.
struct State {
  double u1, v1, u2, v2;
  Point3 p;
};

State toState(const WLinePoint& n) noexcept { return {n.u1, n.v1, n.u2, n.v2, n.point}; }

WLinePoint toNode(const State& s, double resid) noexcept {
  WLinePoint n;
  n.u1 = s.u1; n.v1 = s.v1; n.u2 = s.u2; n.v2 = s.v2;
  n.point = s.p; n.onSurfResidual = resid;
  return n;
}

// ── the intersection tangent  t = normalize(nA × nB) ────────────────────────────
//
// The cross product of the two surface normals is tangent to both tangent planes,
// hence tangent to the intersection curve. `sine` = ‖nA × nB‖ is the transversality
// witness: near zero ⇒ near-tangent (an S4 stop). `valid` is false when it degenerates.
struct Tangent {
  Vec3 dir{};    // unit tangent (undefined if !valid)
  double sine = 0.0;
  bool valid = false;
};

Tangent intersectionTangent(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& s) {
  const Dir3 nA = A.normal(s.u1, s.v1);
  const Dir3 nB = B.normal(s.u2, s.v2);
  const Vec3 c = math::cross(nA.vec(), nB.vec());
  const double sine = math::norm(c);
  Tangent t;
  t.sine = sine;
  if (sine <= 1e-14) return t;  // degenerate; caller decides tangentSinTol
  t.dir = c / sine;
  t.valid = true;
  return t;
}

// ── param-space prediction: advance each surface's (u,v) by the world step h·t ──
//
// Solve the surface's own tangent-plane 2×2 [dU dV]ᵀ[dU dV] (Δu,Δv) = [dU dV]ᵀ (h·t)
// (normal equations of the 3×2 system) so the corrector starts near the answer. A
// finite-difference tangent frame is used (works for every adapter — it only exposes
// point/normal, not analytic dU/dV). Falls back to no param move if the frame is
// degenerate (the corrector still fixes it from the 3D guess).
void advanceParams(const SurfaceAdapter& S, double u, double v, double period,
                   const Vec3& worldStep, const ParamBox& dom, double& uOut, double& vOut) {
  const double hu = std::max(dom.du() * 1e-6, 1e-9);
  const double hv = std::max(dom.dv() * 1e-6, 1e-9);
  const Point3 p0 = S.point(u, v);
  const Vec3 dU = (S.point(std::min(u + hu, dom.u1), v) - p0) / hu;
  const Vec3 dV = (S.point(u, std::min(v + hv, dom.v1)) - p0) / hv;
  const double a = math::dot(dU, dU), b = math::dot(dU, dV), d = math::dot(dV, dV);
  const double det = a * d - b * b;
  double du = 0.0, dv = 0.0;
  if (std::fabs(det) > 1e-18) {
    const double ru = math::dot(dU, worldStep), rv = math::dot(dV, worldStep);
    du = (d * ru - b * rv) / det;
    dv = (a * rv - b * ru) / det;
  }
  // Periodic (angular) directions wrap; non-periodic clamp to the domain.
  uOut = period > 0.0 ? u + du : clampd(u + du, dom.u0, dom.u1);
  vOut = clampd(v + dv, dom.v0, dom.v1);
  (void)period;
}

// ── the two-surface corrector ────────────────────────────────────────────────
//
// Re-project onto BOTH surfaces AND advance ≈ h along the tangent, in one solve. The
// residual is 4-vector (m=n=4, well-posed):
//   r0..2 = A.point(u1,v1) − B.point(u2,v2)          (land on the intersection)
//   r3    = dot(A.point(u1,v1) − Pprev, t) − h       (advance one step along t)
// so the along-curve DOF that left the S2 refine rank-deficient is pinned — the
// corrector moves forward by h instead of sliding back to Pprev. Params are clamped to
// both domains inside the residual (an angular direction is left unclamped so a loop
// can wrap the seam). Returns the corrected State, whether it converged on both
// surfaces, and the corrector strain (nfev) for the step controller.
struct CorrectorOut {
  State s{};
  double resid = 0.0;   // ‖A.point − B.point‖ at the solution
  int nfev = 0;
  bool ok = false;
};

CorrectorOut correct(const SurfaceAdapter& A, const SurfaceAdapter& B,
                     const State& prev, const Vec3& tdir, double h,
                     const State& guess, double onSurfTol) {
  const ParamBox& da = A.domain;
  const ParamBox& db = B.domain;
  const bool aPeriodU = A.uPeriod > 0.0, aPeriodV = A.vPeriod > 0.0;
  const bool bPeriodU = B.uPeriod > 0.0, bPeriodV = B.vPeriod > 0.0;
  auto clampX = [&](const nn::Vector& x) {
    return std::array<double, 4>{
        aPeriodU ? x[0] : clampd(x[0], da.u0, da.u1),
        aPeriodV ? x[1] : clampd(x[1], da.v0, da.v1),
        bPeriodU ? x[2] : clampd(x[2], db.u0, db.u1),
        bPeriodV ? x[3] : clampd(x[3], db.v0, db.v1)};
  };
  nn::VecFn resid = [&](const nn::Vector& x) -> nn::Vector {
    const auto c = clampX(x);
    const Point3 pa = A.point(c[0], c[1]);
    const Point3 pb = B.point(c[2], c[3]);
    const Vec3 gap = pa - pb;
    const double along = math::dot(pa - prev.p, tdir) - h;
    return {gap.x, gap.y, gap.z, along};
  };
  const nn::Vector x0{guess.u1, guess.v1, guess.u2, guess.v2};
  const nn::SolveResult r = nn::least_squares(resid, x0);
  const auto c = clampX(r.x);
  const Point3 pa = A.point(c[0], c[1]);
  const Point3 pb = B.point(c[2], c[3]);
  CorrectorOut out;
  out.s = {c[0], c[1], c[2], c[3], pa};
  out.resid = math::distance(pa, pb);
  out.nfev = r.nfev;
  out.ok = out.resid <= onSurfTol;
  return out;
}

// Chord/arc deflection of a step: how far the true intersection midpoint bows off the
// straight prev→next chord. Approximated by re-projecting the chord midpoint's world
// position onto the intersection and measuring the offset. Cheap and monotone in
// curvature·h², exactly what the step controller needs (cf. IntWalk TestDeflection).
double stepDeflection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                     const State& prev, const State& next, const Vec3& tdir, double onSurfTol) {
  const Point3 chordMid = prev.p + (next.p - prev.p) * 0.5;
  // one corrector half-step from the chord midpoint (no advance term: h≈0 pins it near
  // the midpoint's foot on the curve).
  const CorrectorOut m = correct(A, B, prev, tdir, 0.0,
                                 State{prev.u1, prev.v1, prev.u2, prev.v2, chordMid}, onSurfTol);
  if (!m.ok) return std::numeric_limits<double>::infinity();  // can't measure → force shrink
  return math::distance(chordMid, m.s.p);
}

// ── one marching direction ──────────────────────────────────────────────────────
//
// Walk from `start` in the sign(+1/−1) sense of the seed tangent until the curve
// closes on the seed, exits a boundary, or hits a near-tangent/divergent region.
// Appends the accepted nodes (NOT including `start`) to `out`. Reports the ending
// reason so march_branch can classify the WLine and stitch the two halves.
enum class DirEnd { Boundary, LoopClosed, NearTangent };

// Did the corrected params land on (or past) a NON-periodic domain edge? That is a
// boundary exit — the intersection leaves the surface patch there.
bool onBoundary(const SurfaceAdapter& S, double u, double v, bool periodU, bool periodV) {
  const ParamBox& d = S.domain;
  // Edge band = a small fraction of the domain. The corrector's along-step constraint
  // lands a boundary-crossing node slightly SHORT of the exact edge (it can only step
  // ≈ h at a time), so a hair-thin band would make the march crawl the last stretch in
  // ever-smaller steps and mis-flag it near-tangent. 1e-4 of the domain is well below
  // one marching step yet safely above the corrector's short-fall.
  const double eu = std::max(d.du() * 1e-4, 1e-12);
  const double ev = std::max(d.dv() * 1e-4, 1e-12);
  const bool uEdge = !periodU && (u <= d.u0 + eu || u >= d.u1 - eu);
  const bool vEdge = !periodV && (v <= d.v0 + ev || v >= d.v1 - ev);
  return uEdge || vEdge;
}

struct DirResult {
  DirEnd end = DirEnd::Boundary;
  double sineAtStop = 0.0;
  State stopState{};   ///< the on-curve state where the march stopped (for S4-b typing)
  int crossed = 0;             ///< S4-c: near-tangent grazes marched through this direction
  double maxCrossResid = 0.0;  ///< worst on-both-surfaces residual over the crossed arcs

  // S4-d: set only when enableBranchPoints AND this direction stopped at a BRANCH signature
  // (steep sine collapse / raw-tangent flip — two arms meet). Carries the last-good on-curve
  // state + forward tangent so the caller can localize the branch point and enumerate arms.
  bool branchStall = false;
  State branchState{};      ///< last-good on-curve state entering the branch band
  Vec3 branchTStar{};       ///< last-good FORWARD unit tangent at that state
  double branchEnterSine = 0.0;  ///< ‖nA×nB‖ just before the collapse
};

// One PREDICTED-then-CORRECTED step with deflection-driven shrink. Halves `h` (in/out)
// until the corrector converges to an on-both-surfaces point whose chord/arc
// deflection is within budget AND that actually advanced (didn't slide back), or `h`
// falls below minStep. Returns the accepted CorrectorOut and sets `ok`; on `!ok` the
// step could not be taken transversally (a near-tangent / divergent stop). Isolating
// the shrink loop keeps marchDir a flat guarded dispatch.
CorrectorOut tryStep(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& cur,
                    const Vec3& dir, const Tuned& t, double& h, bool& ok) {
  ok = false;
  CorrectorOut c;
  while (h >= t.minStep) {
    State guess = cur;
    guess.p = cur.p + dir * h;
    advanceParams(A, cur.u1, cur.v1, A.uPeriod, dir * h, A.domain, guess.u1, guess.v1);
    advanceParams(B, cur.u2, cur.v2, B.uPeriod, dir * h, B.domain, guess.u2, guess.v2);
    c = correct(A, B, cur, dir, h, guess, t.onSurfTol);
    const double defl = c.ok ? stepDeflection(A, B, cur, c.s, dir, t.onSurfTol)
                             : std::numeric_limits<double>::infinity();
    const bool tooFar = !c.ok || defl > t.maxDeflection ||
                        math::distance(cur.p, c.s.p) < 0.25 * h;  // corrector slid back
    if (!tooFar) { ok = true; return c; }
    h *= 0.5;  // shrink and retry
  }
  return c;
}

// ── S4-c: curvature-aware predictor ─────────────────────────────────────────────
//
// Across a sharp near-tangent bend the first-order guess P + h·t★ overshoots off the
// curve, so the fixed-plane corrector can start out of basin. Bend it by the DISCRETE
// CURVATURE of the last two accepted nodes: with unit tangents t_{k-1}, t_k and node
// spacing Δs, κ̂·N̂ ≈ (t_k − t_{k-1})/Δs, and the second-order guess is
// P + h·t★ + ½·h²·(κ̂·N̂). Falls back to first-order when there are < 2 prior nodes, Δs
// is degenerate, or κ̂ is non-finite — never worse than the S3 predictor.
Point3 curvaturePredict(const State* prevPrev, const State& prev, double h, const Vec3& tStar) {
  const Point3 first = prev.p + tStar * h;
  if (prevPrev == nullptr) return first;
  const double ds = math::distance(prev.p, prevPrev->p);
  if (!(ds > 1e-12)) return first;
  const Vec3 seg = (prev.p - prevPrev->p) / ds;              // chord tangent of the last step
  const Vec3 kN = (tStar - seg) / ds;                        // (t_k − t_{k-1})/Δs ≈ κ̂·N̂
  const Vec3 bend = kN * (0.5 * h * h);
  const Point3 guess = first + bend;
  if (!(std::isfinite(guess.x) && std::isfinite(guess.y) && std::isfinite(guess.z))) return first;
  return guess;
}

// ── S4-c: fine look-ahead scan of the near-tangent band minimum sine ────────────
//
// The marching step is far coarser than the near-tangent region, so a genuine
// (measure-zero) tangency / branch point — where sine → 0 — can be LEAPT OVER by one
// step and mistaken for a graze. Before committing to a crossing we SCAN the band along
// the fixed crossing direction `dirStar` in FINE increments (fixed-plane cuts), tracking
// the minimum sine until it recovers above `bandExitSin` or a bounded look-ahead is
// exhausted. A crossable graze bottoms out at a bounded sine; a tangency/branch dips to
// ~0. Returns that minimum (∞ if the scan never lands on both surfaces — treated as
// non-crossable). Pure look-ahead — appends nothing; the caller decides on the result.
double bandMinSine(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& stall,
                   const Vec3& dirStar, const Tuned& t) {
  const double hFine = std::max(t.minStep, t.h0 / 64.0);     // fine sampling step
  const double reach = t.h0 * 4.0;                           // bounded look-ahead distance
  const int maxScan = 512;
  double minSine = intersectionTangent(A, B, stall).sine;
  // Scan BOTH ways along the crossing direction: the marcher enters the band only after
  // OVERSHOOTING the minimum (its last-good node was above the enter threshold, then one
  // step dropped below), so the sine → 0 dip of a true tangency sits BEHIND the stall as
  // often as ahead. Sampling ±dirStar captures it either way.
  for (int dir = -1; dir <= 1; dir += 2) {
    const Vec3 step = dirStar * (hFine * static_cast<double>(dir));
    State cur = stall;
    double travelled = 0.0;
    for (int i = 0; i < maxScan && travelled < reach; ++i) {
      State guess = cur;
      guess.p = cur.p + step;
      advanceParams(A, cur.u1, cur.v1, A.uPeriod, step, A.domain, guess.u1, guess.v1);
      advanceParams(B, cur.u2, cur.v2, B.uPeriod, step, B.domain, guess.u2, guess.v2);
      // reproject onto both surfaces holding the fine advance along `step` (no branch pin)
      const CorrectorOut c = correct(A, B, cur, dirStar, hFine * static_cast<double>(dir),
                                     guess, t.onSurfTol);
      if (!c.ok) break;  // cannot resolve this side → stop scanning this direction
      const Tangent tn = intersectionTangent(A, B, c.s);
      minSine = std::min(minSine, tn.sine);
      travelled += math::distance(cur.p, c.s.p);
      cur = c.s;
      if (tn.sine >= t.bandExitSin && travelled > hFine) break;  // recovered on this side
    }
  }
  return minSine;
}

// ── S4-c: the crossable-graze driver (fixed-plane cut) ──────────────────────────
//
// Called from marchDir when the march reaches the crossing band (sine < bandEnterSin).
// Attempts to MARCH THROUGH the near-tangency, appending verified nodes to `out`, and
// returns true iff it crossed to the far side (sine ≥ bandExitSin with the tangent still
// consistent with t★). On ANY failure — a genuine tangency / undecided jet at the stall,
// a branch flip, a node off either surface, a non-monotone step, or the step budget /
// minStep floor exhausted — it appends NOTHING new (the caller keeps only the pre-band
// nodes) and returns false so the caller STOPS + defers. Never fabricates a point.
//
// COMPLEXITY NOTE: this is the isolated near-tangent corrector the design flags as a
// systems-band function (guarded crossing loop + verification); it is deliberately kept
// in one place rather than spread through marchDir.
struct CrossOut {
  bool crossed = false;
  State end{};          ///< the far-side state to resume the normal march from
  int count = 0;        ///< nodes appended (for nearTangentCrossed accounting)
  double maxResid = 0.0;
  // S4-d: the crossing was refused because the near-tangency is a BRANCH signature — the
  // transversality sine collapsed toward 0 (steep-collapse witness or band-minimum below
  // the crossable floor). This is exactly the sine→0 dip of a self-crossing (Steinmetz
  // saddle), distinct from a bounded-dip crossable graze. The caller (with branch points
  // enabled) hands this to the S4-d localizer instead of a blind defer.
  bool branchSignature = false;
};

// Per-node crossability verdict for a corrected crossing node with tangent `tanNew`.
// Two guards, EITHER of which rejects (→ discard + defer):
//  * GENUINE-TANGENCY / BRANCH floor — a crossable graze keeps a bounded minimum sine; a
//    true tangency / branch point drives sine → 0. A node whose sine collapsed below the
//    floor is not a single-branch graze.
//  * BRANCH-FLIP witness — the RAW cross-product tangent field is smooth along a single
//    branch (successive raw tangents stay aligned once continuity-oriented); at a branch
//    point it turns sharply / reverses. A ≥60° turn in one step, or a continuity-oriented
//    tangent that no longer heads the crossing way, means two branches meet (S4-d).
// On accept, `rawOut` is the continuity-oriented raw tangent to carry to the next step.
bool crossNodeCrossable(const Tangent& tanNew, const Vec3& rawPrev, const Vec3& dirStar,
                        const Tuned& t, Vec3& rawOut) {
  if (!tanNew.valid || tanNew.sine < t.minCrossSine) return false;
  Vec3 rawNew = tanNew.dir;
  const double contDot = math::dot(rawNew, rawPrev);
  if (std::fabs(contDot) < 0.5) return false;              // turned ≥60° in one step
  if (contDot < 0.0) rawNew = rawNew * -1.0;               // orient by continuity
  if (math::dot(rawNew, dirStar) <= 0.0) return false;     // U-turn off the crossing way
  rawOut = rawNew;
  return true;
}

// `tStarFwd` is the last-good FORWARD unit tangent (already oriented toward the march
// direction), used directly as the fixed crossing direction — the raw cross-product sign
// is unreliable through the near-tangency, so orientation is carried in, not re-derived.
// `lastGoodSine` is ‖nA×nB‖ at the last pre-band node (the transversality just before the
// dip) — the crossable-vs-branch discriminator below.
CrossOut crossNearTangent(const SurfaceAdapter& A, const SurfaceAdapter& B,
                          const State& stall, const Vec3& tStarFwd, double lastGoodSine,
                          const Tuned& t, double scale, std::vector<WLinePoint>& out) {
  CrossOut r;

  // Crossable GATE (S4-b): only a NearTangentTransversal graze may be crossed.
  const Dir3 nA = A.normal(stall.u1, stall.v1);
  const Dir3 nB = B.normal(stall.u2, stall.v2);
  const double sine0 = math::norm(math::cross(nA.vec(), nB.vec()));
  const TangentContact tc = classify_tangent_contact_seeded(
      A, B, stall.u1, stall.v1, stall.u2, stall.v2, stall.p, nA, nB, sine0, scale);
  if (tc.type != TangentContactType::NearTangentTransversal) return r;  // tangent/undecided → defer

  const Vec3 dirStar = tStarFwd;        // oriented crossing direction (fixed for the whole band)

  // BRANCH / GENUINE-TANGENCY reject — the honesty core. Two independent witnesses, EITHER
  // of which forces a defer:
  //
  //  (1) STEEP COLLAPSE. A crossable graze bends GENTLY — the transversality sine eases
  //      down from its last-good value to a bounded dip (the sphere/graze drops only
  //      0.266 → 0.247 into the band, a ~0.9 ratio). A true tangency / S4-d branch point
  //      COLLAPSES sine by orders of magnitude in one step (the equal-cylinder saddle
  //      drops 0.031 → 3e-5, a ~0.001 ratio). So if the stall sine fell to a small
  //      fraction of the last-good sine, the region is a tangency/branch → defer.
  //  (2) BAND-MINIMUM FLOOR. A fine look-ahead scan of the band (finer than the marching
  //      step, so it cannot leap over a measure-zero dip) must keep its minimum sine at or
  //      above a fraction of the enter threshold; a sine → 0 dip ⇒ defer.
  const bool steepCollapse = lastGoodSine > 0.0 && sine0 < 0.25 * lastGoodSine;
  if (steepCollapse) { r.branchSignature = true; return r; }

  const double bandMin = bandMinSine(A, B, stall, dirStar, t);
  if (!(bandMin >= t.minCrossSine)) { r.branchSignature = true; return r; }  // (also catches inf / NaN → defer)

  const std::size_t base = out.size();  // rollback marker: discard all crossing nodes on failure
  State cur = stall;
  State prev = stall;
  bool havePrev = false;                // a within-band prior node feeds the curvature predictor
  Vec3 rawPrev = tStarFwd;              // previous CONTINUITY-oriented raw tangent (flip witness)
  // Enter with a FINE step so the crossing RESOLVES the near-tangent region instead of
  // leaping over it. A big step from the stall would land straight on the far side and
  // look transversal even at a true tangency/branch (the sine → 0 dip skipped); a fine
  // step samples the dip so the sine floor + tangent-flip guards below can see it. The
  // cap is generous enough that a genuine graze still crosses in a bounded node count.
  const double hCrossCap = std::max(t.minStep, t.h0 / 16.0);
  double h = hCrossCap;

  for (int i = 0; i < t.crossMaxSteps; ++i) {
    // Curvature-aware guess along the FIXED t★, then fixed-plane-cut correct: the
    // corrector's advance residual uses dirStar (well-posed as local sine → 0), NOT the
    // degenerating local tangent.
    State guess = cur;
    guess.p = curvaturePredict(havePrev ? &prev : nullptr, cur, h, dirStar);
    advanceParams(A, cur.u1, cur.v1, A.uPeriod, dirStar * h, A.domain, guess.u1, guess.v1);
    advanceParams(B, cur.u2, cur.v2, B.uPeriod, dirStar * h, B.domain, guess.u2, guess.v2);
    const CorrectorOut c = correct(A, B, cur, dirStar, h, guess, t.onSurfTol);

    // VERIFY: on both surfaces, advanced along t★, and the chord/arc DEFLECTION is within
    // budget. The deflection cap is what forces the crossing to RESOLVE the near-tangent
    // region rather than leap across it: at a true tangency / branch point the curve bows
    // sharply, so a big step has huge deflection → shrink until it SAMPLES the sine → 0
    // dip, where the floor/flip guards below abort. A genuine graze bows gently and is
    // accepted at a healthy step.
    const double advance = math::dot(c.s.p - cur.p, dirStar);
    const bool advanced = advance >= 0.25 * h;
    const double defl = c.ok ? stepDeflection(A, B, cur, c.s, dirStar, t.onSurfTol)
                             : std::numeric_limits<double>::infinity();
    if (!c.ok || !advanced || defl > t.maxDeflection) {
      if (h <= t.minStep) { out.resize(base); return r; }  // stuck at the floor → defer
      h *= 0.5;                                            // shrink and retry the same node
      continue;
    }

    const Tangent tanNew = intersectionTangent(A, B, c.s);
    Vec3 rawNew{};
    if (!crossNodeCrossable(tanNew, rawPrev, dirStar, t, rawNew)) { out.resize(base); return r; }

    out.push_back(toNode(c.s, c.resid));
    prev = cur; havePrev = true;
    cur = c.s;
    rawPrev = rawNew;
    r.maxResid = std::max(r.maxResid, c.resid);
    ++r.count;

    // Recovered on the far side: transversality back above the exit threshold → crossed.
    if (tanNew.sine >= t.bandExitSin) {
      r.crossed = true;
      r.end = cur;
      return r;
    }
    // still deep in the band: keep the step fine (bounded by the crossing cap / minStep).
    h = std::max(t.minStep, std::min(h, hCrossCap));
  }

  out.resize(base);  // budget exhausted without recovering → discard, defer
  return r;
}

// The mutable state carried through one direction's walk. Bundled so the walk loop can
// hand it to small helpers (band entry, node acceptance) instead of inlining every branch
// — keeping marchDir a flat dispatch (see the S4-c helpers below).
struct Walk {
  State cur;
  double h;
  double sign;
  Vec3 tStar{};              // last-good FORWARD unit tangent (fixed-plane normal on a band entry)
  double lastGoodSine = 0.0; // ‖nA×nB‖ at the last pre-band node (crossable discriminator)
  bool haveStar = false;
  bool crossedAny = false;   // has this direction marched through a graze?
  Vec3 seedFwd{};            // seed's outgoing FORWARD tangent (closure-direction gate)
  bool haveSeedFwd = false;
};

// Loop-closure proximity. In a pure S3 transversal trace this is exactly `loopClose·h`
// (bit-identical to S3). Once a graze has been CROSSED the step h collapses to near
// minStep through the band, which would make `loopClose·h` too tight to detect the loop
// returning to the seed (the seed can sit right at a graze); after a crossing we floor
// the radius at the NOMINAL step so closure stays detectable.
double closeRadius(const Walk& w, const Tuned& t) {
  return w.crossedAny ? t.loopClose * std::max(w.h, 0.5 * t.h0) : t.loopClose * w.h;
}
// DIRECTION-CONSISTENT closure (only meaningful once a graze was crossed and the radius
// floored). A near-tangent loop can pass CLOSE TO the seed at a mid-loop junction while
// running the OTHER way; the inflated radius would false-close there. Require the current
// forward tangent to agree with the seed's outgoing tangent. Pure S3 (crossedAny==false)
// keeps its exact tight-radius behaviour (returns true unconditionally).
bool closeAligned(const Walk& w, const Vec3& fwdNow) {
  if (!w.crossedAny || !w.haveSeedFwd) return true;
  return math::dot(fwdNow, w.seedFwd) > 0.0;
}
bool atBoundary(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& s) {
  return onBoundary(A, s.u1, s.v1, A.uPeriod > 0.0, A.vPeriod > 0.0) ||
         onBoundary(B, s.u2, s.v2, B.uPeriod > 0.0, B.vPeriod > 0.0);
}

// S4-c band-entry handler: at a near-tangent stall (sine < bandEnterSin, with a last-good
// tangent), try to MARCH THROUGH via crossNearTangent. On success it updates `w` to the
// recovered far side and returns nullopt (the caller CONTINUES the walk after re-checking
// closure/boundary here); on a non-crossable region it returns a NearTangent DirResult
// (the caller STOPS + defers). Isolating this keeps marchDir flat.
std::optional<DirResult> tryBandEntry(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                      const State& start, int step, const Tuned& t,
                                      double scale, Walk& w, std::vector<WLinePoint>& out,
                                      DirResult& res) {
  const CrossOut cx = crossNearTangent(A, B, w.cur, w.tStar, w.lastGoodSine, t, scale, out);
  if (!cx.crossed) {  // genuine tangency / branch / unverifiable → honest S4 stop
    DirResult stop;
    stop.end = DirEnd::NearTangent;
    stop.sineAtStop = intersectionTangent(A, B, w.cur).sine;
    stop.stopState = w.cur;
    // S4-d: a BRANCH signature (sine collapsed toward 0 — two arms meet) is CAPTURED so the
    // caller can localize the branch point + route arms, instead of a blind defer. Only the
    // branch case is captured; an ordinary tangency (TangentPoint/Curve/Undecided) is NOT —
    // it keeps deferring, so an isolated tangent point can never sprout arms.
    if (t.enableBranchPoints && cx.branchSignature) {
      stop.branchStall = true;
      stop.branchState = w.cur;
      stop.branchTStar = w.tStar;
      stop.branchEnterSine = w.lastGoodSine > 0.0 ? w.lastGoodSine : t.bandEnterSin;
    }
    return stop;
  }
  res.crossed += cx.count;
  res.maxCrossResid = std::max(res.maxCrossResid, cx.maxResid);
  const Vec3 fwd = cx.end.p - w.cur.p;               // forward chord across the graze
  w.cur = cx.end;
  w.crossedAny = true;
  w.h = std::max(t.minStep, 0.5 * t.h0);             // resume with a modest step
  // Re-derive the forward SIGN so `tan.dir · sign` keeps going the way the crossing went
  // (the raw cross-product sign may have flipped through the near-tangency), refreshing
  // the FORWARD tStar / lastGoodSine for any subsequent band entry.
  const Tangent rec = intersectionTangent(A, B, w.cur);
  double recSine = 0.0;
  if (rec.valid) {
    const bool fwdAligned = math::dot(rec.dir, fwd) >= 0.0;
    w.sign = fwdAligned ? 1.0 : -1.0;
    w.tStar = fwdAligned ? rec.dir : rec.dir * -1.0;
    w.lastGoodSine = rec.sine;
    w.haveStar = true;
    recSine = rec.sine;
  }
  if (step > 2 && math::distance(w.cur.p, start.p) <= closeRadius(w, t) && closeAligned(w, w.tStar)) {
    DirResult done; done.end = DirEnd::LoopClosed; done.crossed = res.crossed;
    done.maxCrossResid = res.maxCrossResid; done.sineAtStop = recSine; return done;
  }
  if (atBoundary(A, B, w.cur)) {
    DirResult done; done.end = DirEnd::Boundary; done.crossed = res.crossed;
    done.maxCrossResid = res.maxCrossResid; done.sineAtStop = recSine; return done;
  }
  return std::nullopt;  // crossed → continue the walk
}

DirResult marchDir(const SurfaceAdapter& A, const SurfaceAdapter& B,
                  const State& start, double sign, const Tuned& t, double scale,
                  std::vector<WLinePoint>& out) {
  Walk w{start, t.h0, sign};
  DirResult res;
  for (int step = 0; step < t.maxPoints; ++step) {
    const Tangent tan = intersectionTangent(A, B, w.cur);

    // Loop closure at the current node (checked before a possible band entry so a loop
    // whose seed lies inside a graze still closes rather than crossing forever).
    if (step > 2 && w.crossedAny && math::distance(w.cur.p, start.p) <= closeRadius(w, t) &&
        closeAligned(w, w.tStar)) {
      res.end = DirEnd::LoopClosed; res.sineAtStop = tan.sine; return res;
    }

    // S4-c crossing band: try to MARCH THROUGH a near-tangency instead of stopping — only
    // with a valid last-good tangent and if the S4-b classification + single-branch checks
    // pass (see tryBandEntry / crossNearTangent).
    if ((!tan.valid || tan.sine < t.bandEnterSin) && w.haveStar) {
      if (auto done = tryBandEntry(A, B, start, step, t, scale, w, out, res)) return *done;
      continue;  // crossed → resume the normal walk on the far side
    }
    if (!tan.valid || tan.sine < t.tangentSinTol) {  // near-tangent, no last-good tangent → S4 stop
      res.end = DirEnd::NearTangent; res.sineAtStop = tan.sine; res.stopState = w.cur; return res;
    }
    const Vec3 dir = tan.dir * w.sign;

    bool ok = false;
    const CorrectorOut c = tryStep(A, B, w.cur, dir, t, w.h, ok);
    if (!ok) {  // corrector could not take even a minStep transversally → S4 stop
      res.end = DirEnd::NearTangent; res.sineAtStop = intersectionTangent(A, B, w.cur).sine;
      res.stopState = w.cur; return res;
    }

    // Forward tangent (oriented by the step just taken) — the raw cross-product sign is
    // not reliable across a coming near-tangency, so a later band entry crosses this way.
    const Vec3 chord = c.s.p - w.cur.p;
    Vec3 fwd = dir;
    if (math::norm(chord) > 1e-14 && math::dot(dir, chord) < 0.0) fwd = dir * -1.0;

    // Loop closure: back near the seed after real progress, moving consistently (a crossed
    // loop must also return in the seed's outgoing direction — see closeAligned).
    if (step > 2 && math::distance(c.s.p, start.p) <= closeRadius(w, t) && closeAligned(w, fwd)) {
      out.push_back(toNode(c.s, c.resid));
      res.end = DirEnd::LoopClosed; res.sineAtStop = tan.sine; return res;
    }

    w.tStar = fwd; w.haveStar = true;
    w.lastGoodSine = tan.sine;                                  // transversality just before any dip
    if (!w.haveSeedFwd) { w.seedFwd = fwd; w.haveSeedFwd = true; }
    out.push_back(toNode(c.s, c.resid));
    w.cur = c.s;

    if (atBoundary(A, B, w.cur)) {  // boundary exit (non-periodic edge reached)
      res.end = DirEnd::Boundary; res.sineAtStop = tan.sine; return res;
    }
    if (c.nfev <= 40) w.h = std::min(w.h * 1.5, t.maxStep);  // smooth + cheap → grow h (bounded)
  }
  res.end = DirEnd::Boundary;  // ran out of budget → treat as an (unfinished) open end
  return res;
}

// A clamped uniform flat knot vector for `nPoles` control points of `degree`
// (multiplicity degree+1 at both ends, evenly spaced interior knots), on [0,1].
// Length nPoles+degree+1 — the flat convention the native evaluator expects.
std::vector<double> clampedUniformKnots(int degree, int nPoles) {
  const int nInterior = nPoles - degree - 1;
  std::vector<double> knots;
  knots.reserve(static_cast<std::size_t>(nPoles + degree + 1));
  for (int i = 0; i <= degree; ++i) knots.push_back(0.0);
  for (int i = 1; i <= nInterior; ++i) knots.push_back(double(i) / (nInterior + 1));
  for (int i = 0; i <= degree; ++i) knots.push_back(1.0);
  return knots;
}

// Chord-length parameters in [0,1] for the polyline nodes. Returns empty if the
// polyline has zero total length (all coincident) — no valid parametrization.
std::vector<double> chordLengthParams(const std::vector<WLinePoint>& pts) {
  const int m = static_cast<int>(pts.size());
  std::vector<double> tval(m, 0.0);
  double total = 0.0;
  for (int i = 1; i < m; ++i) { total += math::distance(pts[i].point, pts[i - 1].point); tval[i] = total; }
  if (total <= 0.0) return {};
  for (int i = 0; i < m; ++i) tval[i] /= total;
  return tval;
}

// ── B-spline fit through the traced polyline (native-math + lstsq) ──────────────
//
// Chord-length-parametrized least-squares fit to a clamped uniform B-spline. With
// fewer target poles than samples this SMOOTHS the polyline; if there are too few
// samples for the degree it degrades the degree, and with ≤1 sample returns empty.
// The polyline stays the ground truth; the fit is a convenience curve. Solves the
// normal-equations column-by-column (x,y,z independently) via lstsq.
FittedBSpline fitBSpline(const std::vector<WLinePoint>& pts, int degree, int maxPoles) {
  FittedBSpline out;
  const int m = static_cast<int>(pts.size());
  if (m < 2) return out;
  degree = std::min(degree, m - 1);
  if (degree < 1) return out;

  const std::vector<double> tval = chordLengthParams(pts);
  if (tval.empty()) return out;

  int nPoles = (maxPoles <= 0) ? m : std::min(maxPoles, m);
  nPoles = std::clamp(nPoles, degree + 1, m);
  if (nPoles < degree + 1) return out;

  std::vector<double> knots = clampedUniformKnots(degree, nPoles);

  // basis matrix N (m × nPoles), row i = non-zero basis of t[i] scattered into columns.
  std::vector<double> N(static_cast<std::size_t>(m) * nPoles, 0.0);
  std::vector<double> basis(static_cast<std::size_t>(degree) + 1);
  for (int i = 0; i < m; ++i) {
    const int span = math::findSpan(nPoles - 1, degree, tval[i], knots);
    math::basisFuns(span, tval[i], degree, knots, basis);
    for (int j = 0; j <= degree; ++j)
      N[static_cast<std::size_t>(i) * nPoles + (span - degree + j)] = basis[j];
  }

  // Solve min‖N·P − X‖ for each coordinate; lstsq handles the m≥nPoles overdetermined
  // (and the interpolating m==nPoles square) case.
  auto solveCoord = [&](int axis) {
    std::vector<double> rhs(m);
    for (int i = 0; i < m; ++i) rhs[i] = pts[i].point[static_cast<std::size_t>(axis)];
    return nn::lstsq(N, m, nPoles, rhs);
  };
  const std::vector<double> px = solveCoord(0), py = solveCoord(1), pz = solveCoord(2);
  if (static_cast<int>(px.size()) != nPoles) return out;  // solver failed → no fit

  out.degree = degree;
  out.knots = std::move(knots);
  out.poles.resize(static_cast<std::size_t>(nPoles));
  for (int j = 0; j < nPoles; ++j) out.poles[j] = {px[j], py[j], pz[j]};

  // report the worst deviation of the polyline from the fitted curve.
  double err = 0.0;
  for (int i = 0; i < m; ++i) {
    const Point3 cv = math::curvePoint(out.degree, out.poles, out.knots, tval[i]);
    err = std::max(err, math::distance(cv, pts[i].point));
  }
  out.maxFitError = err;
  return out;
}

}  // namespace

// S4-b: type a near-tangent STOP with the seeded differential-geometry classifier. The
// tracer STILL STOPS here (never steps through the tangency); this only records WHAT the
// degeneracy is (TangentPoint/Curve resolved, or NearTangentTransversal/Undecided handed
// to S4-c → OCCT). Types whichever end stopped near-tangent, preferring the flatter one.
TangentContact typeNearTangentStop(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                   const DirResult& f, const DirResult& b, double scale) {
  const bool fNT = f.end == DirEnd::NearTangent, bNT = b.end == DirEnd::NearTangent;
  const bool useF = fNT && (!bNT || f.sineAtStop <= b.sineAtStop);
  const DirResult& sel = useF ? f : b;
  const State& s = sel.stopState;
  const Dir3 nA = A.normal(s.u1, s.v1);
  const Dir3 nB = B.normal(s.u2, s.v2);
  return classify_tangent_contact_seeded(A, B, s.u1, s.v1, s.u2, s.v2, s.p, nA, nB,
                                         sel.sineAtStop, scale);
}

// ─────────────────────────────────────────────────────────────────────────────
// march_branch — forward + backward from the seed, stitch, classify, fit.
//
// Internal impl returns the traced WLine AND (S4-d) any BRANCH STALLS the two directions
// hit — the last-good on-curve state + forward tangent + entering sine where a branch
// signature (sine → 0, two arms meeting) forced the near-tangent stop. `trace_from_seeds`
// uses those to localize branch points + route arms when enableBranchPoints. The public
// march_branch drops the stalls (unchanged signature / behaviour).
// ─────────────────────────────────────────────────────────────────────────────
namespace {

struct BranchStall {
  State state{};
  Vec3 tStar{};
  double enterSine = 0.0;
};

WLine march_branch_impl(const SurfaceAdapter& A, const SurfaceAdapter& B, const Seed& seed,
                        const MarchOptions& opts, std::vector<BranchStall>& stalls) {
  const double scale = std::max(A.modelScale, B.modelScale);
  const Tuned t = tune(opts, scale);
  const State seedState{seed.u1, seed.v1, seed.u2, seed.v2, seed.point};

  std::vector<WLinePoint> fwd, bwd;
  const DirResult f = marchDir(A, B, seedState, +1.0, t, scale, fwd);
  if (f.branchStall) stalls.push_back({f.branchState, f.branchTStar, f.branchEnterSine});

  WLine line;
  line.branchId = seed.branchId;

  if (f.end == DirEnd::LoopClosed) {
    // A closed loop: forward march wrapped back to the seed. Points = seed + forward.
    line.points.push_back(toNode(seedState, seed.onSurfResidual));
    line.points.insert(line.points.end(), fwd.begin(), fwd.end());
    line.status = TraceStatus::Closed;
    line.nearTangentCrossed = f.crossed;                       // S4-c: grazes marched through
    line.crossMaxResidual   = f.maxCrossResid;
  } else {
    // Open (or truncated forward). March the other way too and stitch:
    //   [reversed backward half]  seed  [forward half].
    const DirResult b = marchDir(A, B, seedState, -1.0, t, scale, bwd);
    if (b.branchStall) stalls.push_back({b.branchState, b.branchTStar, b.branchEnterSine});
    for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) line.points.push_back(*it);
    line.points.push_back(toNode(seedState, seed.onSurfResidual));
    line.points.insert(line.points.end(), fwd.begin(), fwd.end());
    line.nearTangentCrossed = f.crossed + b.crossed;           // S4-c: grazes marched through
    line.crossMaxResidual   = std::max(f.maxCrossResid, b.maxCrossResid);
    // Failed: neither direction advanced past the seed (no real curve). NearTangent: at
    // least one end stopped at a tangency (traced up to it — honest S4 gap). Otherwise
    // both ends left a domain boundary → a clean open branch.
    if (fwd.empty() && bwd.empty())
      line.status = TraceStatus::Failed;
    else if (f.end == DirEnd::NearTangent || b.end == DirEnd::NearTangent) {
      line.status = TraceStatus::NearTangent;
      line.stopReason = typeNearTangentStop(A, B, f, b, scale);  // S4-b: type the stop
    } else
      line.status = TraceStatus::BoundaryExit;
  }

  for (const auto& n : line.points)
    line.onSurfResidual = std::max(line.onSurfResidual, n.onSurfResidual);
  line.curve = fitBSpline(line.points, opts.fitDegree, opts.fitMaxPoles);
  return line;
}

}  // namespace

WLine march_branch(const SurfaceAdapter& A, const SurfaceAdapter& B, const Seed& seed,
                   const MarchOptions& opts) {
  std::vector<BranchStall> ignored;
  return march_branch_impl(A, B, seed, opts, ignored);
}

// ─────────────────────────────────────────────────────────────────────────────
// trace_from_seeds — march every seed, dedup retraced branches, tally.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Does WLine `w` retrace an already-kept line? True if any node of `w` is within
// `radius` of any node of a kept line AND they run along the same locus (a second seed
// dropped onto an already-traced branch produces a near-identical polyline). Cheap
// node-proximity test — sufficient to drop duplicate seeds on one branch.
bool retraces(const WLine& w, const std::vector<WLine>& kept, double radius) {
  if (w.points.empty()) return false;
  // sample a few probe nodes of w; if the MAJORITY sit within radius of some kept
  // line's nodes, w duplicates it.
  for (const WLine& k : kept) {
    if (k.points.empty()) continue;
    int hits = 0, probes = 0;
    const int stride = std::max<int>(1, static_cast<int>(w.points.size()) / 8);
    for (std::size_t i = 0; i < w.points.size(); i += stride) {
      ++probes;
      double best = std::numeric_limits<double>::infinity();
      for (const auto& kn : k.points)
        best = std::min(best, math::distance(w.points[i].point, kn.point));
      if (best <= radius) ++hits;
    }
    if (probes > 0 && hits * 2 >= probes) return true;  // majority coincide → duplicate
  }
  return false;
}

// Distance from point p to segment [a,b].
double pointSegDist(const Point3& p, const Point3& a, const Point3& b) {
  const Vec3 ab = b - a;
  const double len2 = math::normSquared(ab);
  double s = len2 > 1e-30 ? math::dot(p - a, ab) / len2 : 0.0;
  s = s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s);
  return math::distance(p, a + ab * s);
}

// S4-d arc dedup: does arc `w` lie on the SAME LOCUS as a kept line? Unlike retraces()
// (node-to-node proximity, tuned for identical polylines from duplicate seeds), two arms
// tracing the same arc from different branch points sample it at DIFFERENT positions, so we
// test each probe node of `w` against the kept line's POLYLINE SEGMENTS (point-to-segment)
// and require the MAJORITY to lie on it within `radius`. Symmetric-enough for arcs that
// share a locus but were walked in opposite directions / with different steps.
bool sameLocus(const WLine& w, const std::vector<WLine>& kept, double radius) {
  if (w.points.size() < 2) return false;
  for (const WLine& k : kept) {
    if (k.points.size() < 2) continue;
    int hits = 0, probes = 0;
    const int stride = std::max<int>(1, static_cast<int>(w.points.size()) / 16);
    for (std::size_t i = 0; i < w.points.size(); i += stride) {
      ++probes;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t j = 1; j < k.points.size(); ++j)
        best = std::min(best, pointSegDist(w.points[i].point, k.points[j - 1].point, k.points[j].point));
      if (best <= radius) ++hits;
    }
    if (probes > 0 && hits * 4 >= probes * 3) return true;  // ≥75% of w lies on k's locus
  }
  return false;
}

// Tally a kept WLine into the TraceSet counters. Shared by the backbone loop and the
// S4-d routed arms so both report identically.
void tallyLine(TraceSet& res, WLine&& w) {
  res.nearTangentCrossed += w.nearTangentCrossed;
  switch (w.status) {
    case TraceStatus::Closed:       ++res.closedCurves; ++res.tracedBranches; break;
    case TraceStatus::BoundaryExit: ++res.openCurves;   ++res.tracedBranches; break;
    case TraceStatus::NearTangent:  ++res.nearTangentGaps; break;
    case TraceStatus::BranchArc:    ++res.openCurves;   ++res.tracedBranches; break;
    case TraceStatus::Failed:       break;
  }
  res.lines.push_back(std::move(w));
}

// ── S4-d: localize + route branch arms (design steps 1–4) ─────────────────────────
//
// From the branch STALLS captured by the backbone marches, LOCALIZE each distinct branch
// point B (branchpt::localize — minimize the transversality sine along the approach, then
// re-project onto both surfaces), ENUMERATE its real outgoing arm rays (branchpt::
// enumerateArms — the relative-second-form quadratic; empty ⇒ NOT a transversal branch ⇒
// no arms), then ROUTE each ray: step off B by a small arm step, re-project back onto the
// curve, seed a fresh march there, and keep the arc if it is genuinely NEW (not a retrace
// of the backbone or an already-routed arm). Branch points that localize AND yield arms
// become BranchNodes with the branchIds of every arc meeting at B. A stall that will not
// localize / yields no arms leaves the backbone's honest NearTangent gap as-is.
//
// COMPLEXITY NOTE: this is the isolated S4-d branch handler the design flags — the one
// place branch-routing complexity is allowed to sit (kept out of the hot marchDir loop).
// Record `lineId` on the BranchNode(s) this arc's ENDPOINTS reach — an arc between two
// branch points connects both. `endpoints` are the arc's first/last world points.
void connectArm(TraceSet& res, int lineId, const Point3& p0, const Point3& p1, double mergeRadius) {
  for (BranchNode& node : res.branchNodes) {
    if (math::distance(node.point, p0) <= mergeRadius || math::distance(node.point, p1) <= mergeRadius) {
      if (std::find(node.armLineIds.begin(), node.armLineIds.end(), lineId) == node.armLineIds.end())
        node.armLineIds.push_back(lineId);
    }
  }
}

// A localized branch point + its enumerated outgoing arm rays.
struct LocalBP { branchpt::Localized L; std::vector<Vec3> arms; };

// PHASE 1 — from the raw stalls, LOCALIZE the distinct branch points and enumerate each
// one's arms. A stall that does not localize, or whose tangent-cone quadratic yields no real
// arms (definite second form = isolated TangentPoint → curve ends; cusp → defer), is dropped:
// it never becomes a branch point (no fabrication). Duplicate stalls onto one B are merged.
std::vector<LocalBP> localizeBranchPoints(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                          double scale, double h0, double onSurfTol,
                                          double mergeRadius, const std::vector<BranchStall>& stalls) {
  std::vector<LocalBP> bps;
  for (const BranchStall& st : stalls) {
    const branchpt::BPState stall{st.state.u1, st.state.v1, st.state.u2, st.state.v2, st.state.p};
    const auto L = branchpt::localize(A, B, stall, st.tStar, st.enterSine, h0, onSurfTol);
    if (!L) continue;
    bool dup = false;
    for (const LocalBP& e : bps)
      if (math::distance(e.L.point.p, L->point.p) <= mergeRadius) { dup = true; break; }
    if (dup) continue;
    std::vector<Vec3> arms = branchpt::enumerateArms(A, B, *L, scale);
    if (arms.empty()) continue;  // NOT a transversal branch → no arms (curve ends / defer)
    bps.push_back({*L, std::move(arms)});
  }
  return bps;
}

// ROUTE one outgoing ray of a branch point: step off B along `ray`, re-project onto the
// curve, seed a fresh march (branch points OFF so it stops honestly at the far branch and
// never recurses). Returns the traced arc, or nullopt if the arm won't verify / made no
// progress / wound past the cap (an honest drop — never a fabricated fragment).
std::optional<WLine> routeArm(const SurfaceAdapter& A, const SurfaceAdapter& B,
                              const MarchOptions& armOpts, const branchpt::Localized& bp,
                              const Vec3& ray, double armStep, double onSurfTol, int armId) {
  branchpt::BPState off = bp.point;
  off.p = bp.point.p + ray * armStep;
  const auto seedState = branchpt::reproject(A, B, off, onSurfTol, bp.point.p,
                                             branchpt::AlongPin{ray, armStep});
  if (!seedState) return std::nullopt;                            // won't verify on both surfaces
  if (math::distance(seedState->p, bp.point.p) < 0.25 * armStep) return std::nullopt;  // no progress

  Seed armSeed;
  armSeed.u1 = seedState->u1; armSeed.v1 = seedState->v1;
  armSeed.u2 = seedState->u2; armSeed.v2 = seedState->v2;
  armSeed.point = seedState->p;
  armSeed.onSurfResidual = math::distance(A.point(seedState->u1, seedState->v1),
                                          B.point(seedState->u2, seedState->v2));
  armSeed.branchId = armId;

  std::vector<BranchStall> ignored;
  WLine w = march_branch_impl(A, B, armSeed, armOpts, ignored);
  if (w.points.size() < 2 || w.status == TraceStatus::Failed) return std::nullopt;
  // A capped arm (reached the node budget) is winding, not a clean branch-to-branch arc → drop.
  if (static_cast<int>(w.points.size()) >= 2 * armOpts.maxPoints - 4) return std::nullopt;
  return w;
}

// RECLASSIFY branch-terminated arcs. An arc whose NearTangent ends both sit on a LOCALIZED
// branch point is NOT an unresolved S4 gap — it is a resolved junction (the arcs meet at B,
// recorded in the BranchNode). Convert it to BranchArc (a complete arc of the multi-arm
// locus) and take it out of `nearTangentGaps`. This drives Steinmetz to nearTangentGaps == 0.
void reclassifyBranchArcs(TraceSet& res, double mergeRadius) {
  auto atBranch = [&](const Point3& p) {
    for (const BranchNode& n : res.branchNodes)
      if (math::distance(n.point, p) <= mergeRadius) return true;
    return false;
  };
  for (WLine& w : res.lines) {
    if (w.status != TraceStatus::NearTangent || w.points.size() < 2) continue;
    if (atBranch(w.points.front().point) && atBranch(w.points.back().point)) {
      w.status = TraceStatus::BranchArc;
      w.stopReason.reset();
      --res.nearTangentGaps;
      ++res.openCurves;
      ++res.tracedBranches;
    }
  }
}

void routeBranches(const SurfaceAdapter& A, const SurfaceAdapter& B, const MarchOptions& opts,
                   double scale, const std::vector<BranchStall>& stalls, TraceSet& res) {
  if (stalls.empty()) return;
  const Tuned t = tune(opts, scale);
  const double h0 = t.h0;
  const double armStep = h0 / 8.0;                                   // step off B along a ray
  const double mergeRadius = (opts.branchMergeFrac > 0 ? opts.branchMergeFrac : 1e-3) * scale;
  const double dedupRadius = (opts.dedupFrac > 0 ? opts.dedupFrac : 1e-4) * scale;

  const std::vector<LocalBP> bps =
      localizeBranchPoints(A, B, scale, h0, t.onSurfTol, mergeRadius, stalls);
  if (bps.empty()) return;

  for (const LocalBP& e : bps) {
    BranchNode node;
    node.point = e.L.point.p;
    node.branchSine = e.L.sine;
    node.onSurfResidual = math::distance(A.point(e.L.point.u1, e.L.point.v1),
                                         B.point(e.L.point.u2, e.L.point.v2));
    res.branchNodes.push_back(node);
  }
  // Backbone arcs already passing through a branch point (they stopped there) are its arms.
  for (const WLine& w : res.lines)
    if (!w.points.empty())
      connectArm(res, w.branchId, w.points.front().point, w.points.back().point, mergeRadius);

  // ROUTE each branch point's rays. Branch points OFF for the arm marches (stop at the far
  // branch, no recursion / runaway). Bound arm length so a winding arm is dropped, not kept.
  MarchOptions armOpts = opts;
  armOpts.enableBranchPoints = false;
  const int backboneCap = std::max(64, static_cast<int>(16.0 * scale / std::max(h0, 1e-12)));
  armOpts.maxPoints = std::min(opts.maxPoints, backboneCap);
  int nextArmId = 10000;  // routed-arm branchIds live above the seed branchIds
  for (const LocalBP& e : bps)
    for (const Vec3& ray : e.arms) {
      auto w = routeArm(A, B, armOpts, e.L, ray, armStep, t.onSurfTol, nextArmId);
      if (!w) continue;
      // Dedup against every kept line by LOCUS (point-to-polyline): an arm tracing an already
      // kept arc from a different branch point samples it elsewhere, so retraces() would miss it.
      if (sameLocus(*w, res.lines, std::max(dedupRadius, mergeRadius))) { ++res.dedupedRetraces; continue; }
      connectArm(res, w->branchId, w->points.front().point, w->points.back().point, mergeRadius);
      ++res.routedArms;
      ++nextArmId;
      tallyLine(res, std::move(*w));
    }

  res.branchPoints = static_cast<int>(res.branchNodes.size());
  reclassifyBranchArcs(res, mergeRadius);
}

}  // namespace

TraceSet trace_from_seeds(const SurfaceAdapter& A, const SurfaceAdapter& B,
                          const SeedSet& seeds, const MarchOptions& opts) {
  TraceSet res;
  res.seededBranches = seeds.branchCount();
  res.deferredTangent = seeds.deferredTangent;
  const double scale = std::max(A.modelScale, B.modelScale);
  const double dedupRadius = (opts.dedupFrac > 0 ? opts.dedupFrac : 1e-4) * scale;

  std::vector<BranchStall> stalls;  // S4-d: branch stalls captured by the backbone marches
  for (const Seed& s : seeds.seeds) {
    std::vector<BranchStall> seedStalls;
    WLine w = march_branch_impl(A, B, s, opts, seedStalls);
    // Capture branch stalls even when the WLine itself is deduped/failed — the branch point
    // is a property of the geometry, not of which seed reached it.
    if (opts.enableBranchPoints)
      stalls.insert(stalls.end(), seedStalls.begin(), seedStalls.end());
    if (w.points.size() < 2 || w.status == TraceStatus::Failed) continue;  // no real curve
    if (retraces(w, res.lines, dedupRadius)) { ++res.dedupedRetraces; continue; }
    tallyLine(res, std::move(w));
  }

  // S4-d: localize the captured branch points and route their outgoing arms. Additive —
  // with enableBranchPoints off, `stalls` is empty and this is a no-op, so the transversal
  // and crossable-graze results above are byte-identical to before.
  if (opts.enableBranchPoints) routeBranches(A, B, opts, scale, stalls, res);

  return res;
}

TraceSet trace_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                            const SeedOptions& seedOpts, const MarchOptions& marchOpts) {
  const SeedSet seeds = seed_intersection(A, B, seedOpts);
  return trace_from_seeds(A, B, seeds, marchOpts);
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_HAS_NUMSCI
