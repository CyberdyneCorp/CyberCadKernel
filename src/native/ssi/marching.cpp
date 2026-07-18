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
#include "native/ssi/branch_point.h"       // S4-d: branch-point localize + arm enumerate
#include "native/ssi/chart_singularity.h"  // S4-e: single-surface chart-collapse witness + pole map
#include "native/ssi/completeness_critic.h" // S4-f: coverage map + uncovered re-seed targets
#include "native/ssi/tangent_seeded.h"     // S4-b: type a near-tangent stop

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
  double closureTangentCos;           ///< S4-f: close only if dot(fwdNow,seedFwd) ≥ this (BLOCK a false-close)
  bool enableSelfIntersection = false;///< S4-f: record + trace through single-arm self-crossings
  double selfIntersectRadius;         ///< S4-f: self-cross coincidence radius (resolved from selfIntersectRadiusFrac)
  double selfIntersectCoincDist;      ///< S4-f: TRUE-crossing coincidence gate (resolved from selfIntersectCoincFrac·h0)
  int maxPoints, crossMaxSteps;
  bool adaptiveCrossReanchor = false; ///< S4-c deep tail: re-anchor crossing plane to local curve tangent (M1d)
  double reanchorBlend = 1.0;         ///< blend weight local-tangent↔t★ per crossing step (1 = full local tangent)
  bool reanchorIncrementalOrientation = false; ///< M1e: orient re-anchored tangent by continuity, not frozen t★
  bool enableBranchPoints = false;    ///< S4-d: capture branch stalls for localization + arm routing
  // ── S4-e chart-singularity knobs (single-surface ‖dU‖ collapse; sentinel-resolved) ──
  bool enableChartSingularities = false;  ///< S4-e: step across a sphere pole / cone apex
  double chartCollapseFrac;                ///< ‖dU‖ < this·‖dV‖ AND < this·scale ⇒ chart collapse
  double chartStep;                        ///< FINE crossing step off the singular point (chartStepFrac·h0)
  int chartMaxSteps;                       ///< max fine point-based steps crossing one pole/apex
  // S4-e general: non-circular (elliptical / lumpy) collapsed-row pole detection. Near a genuine
  // degenerate v-edge (chartsing::degenerateVEdge — the whole edge row collapses to one point),
  // flag a collapse once ‖dU‖ < chartEdgeCollapseFrac·‖dV‖ within chartEdgeApproachV of the edge.
  // Looser than the pointwise chartCollapseFrac so the SLOW minor-axis collapse fires before the
  // boundary; still a proper rank-drop signal, and the crossing verifies every emitted node.
  double chartEdgeApproachV;               ///< fraction of the v-domain within which the edge-pole band applies
  double chartEdgeCollapseFrac;            ///< ‖dU‖ < this·‖dV‖ near a degenerate v-edge ⇒ pole approach
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
  // S4-f TRUE-RETURN closure: a close requires the return heading to be tangent-continuous
  // with the seed's outgoing tangent — dot(fwdNow, seedFwd) ≥ closureTangentCos. Default 0.5
  // refuses headings turned more than 60° from the outgoing tangent: a genuine smooth loop
  // returns to the seed heading essentially the way it left (dot ≈ +1), so every truly-closing
  // control still closes; a curve merely SWEEPING PAST the seed mid-loop (or its own earlier
  // node) is at a large tangent angle and is REFUSED. It is a necessary-condition tightening —
  // it can only REFUSE a close, never MAKE one — so no tolerance is weakened. A caller may set
  // a positive closureTangentCos to override; ≤ 0 uses the 0.5 default.
  t.closureTangentCos = o.closureTangentCos > 0.0 ? o.closureTangentCos : 0.5;
  // S4-f self-intersection coincidence radius (only used when the guard is on). Default
  // 2·loopClose·h0 matches the loop-closure window scale; sentinel-resolved.
  t.enableSelfIntersection = o.enableSelfIntersection;
  t.selfIntersectRadius = (o.selfIntersectRadiusFrac > 0 ? o.selfIntersectRadiusFrac
                                                         : 2.0 * t.loopClose) * t.h0;
  // COINCIDENCE gate: a genuine self-crossing has the two passes essentially coincident (dist ≈ 0);
  // a small CONVEX loop's near-by chords stay a substantial fraction of a step apart. Tighten
  // acceptance to selfIntersectCoincFrac·h0 (default 0.25) — a necessary condition that only
  // REJECTS false positives, never adds a crossing (so the guard OFF stays byte-identical and every
  // truly-coincident figure-eight crossing is still recorded). See MarchOptions.selfIntersectCoincFrac.
  t.selfIntersectCoincDist = (o.selfIntersectCoincFrac > 0.0 ? o.selfIntersectCoincFrac : 0.25) * t.h0;
  t.maxPoints     = std::max(16, o.maxPoints);
  t.crossMaxSteps = std::max(1, o.crossMaxSteps);
  t.adaptiveCrossReanchor = o.adaptiveCrossReanchor;
  t.reanchorBlend = o.reanchorBlend > 0.0 ? clampd(o.reanchorBlend, 0.0, 1.0) : 1.0;
  // Only meaningful under the re-anchor path; AND-ed here so the crossing loop's fast path
  // stays a single flag test and the option can never act on the frozen-t★ path.
  t.reanchorIncrementalOrientation = o.adaptiveCrossReanchor && o.reanchorIncrementalOrientation;
  t.enableBranchPoints = o.enableBranchPoints;
  // S4-e: chart-collapse threshold (‖dU‖ ≪ ‖dV‖·scale) and the FINE crossing step. Both
  // sentinel-resolved; neither weakens a solve tolerance — they only decide WHERE the
  // point-based crossing engages and how finely it resolves the pole/apex.
  t.enableChartSingularities = o.enableChartSingularities;
  t.chartCollapseFrac = o.chartCollapseFrac > 0 ? o.chartCollapseFrac : 1e-3;
  t.chartStep = std::max(t.minStep, (o.chartStepFrac > 0 ? o.chartStepFrac : (1.0 / 16.0)) * t.h0);
  t.chartMaxSteps = std::max(1, o.chartMaxSteps);
  // S4-e general edge-pole band: within 2% of the v-domain of a degenerate edge, treat a ‖dU‖
  // that has dropped below 5% of ‖dV‖ as a collapse. Both are loose relative to the pointwise
  // chartCollapseFrac (default 1e-3) — they only make a slow (elliptical) collapse fire before
  // the boundary; the crossing's on-both-surfaces verification remains the honesty gate.
  t.chartEdgeApproachV    = o.chartEdgeApproachV    > 0 ? o.chartEdgeApproachV    : 0.02;
  t.chartEdgeCollapseFrac = o.chartEdgeCollapseFrac > 0 ? o.chartEdgeCollapseFrac : 0.05;
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
  int chartCrossed = 0;        ///< S4-e: chart singularities (pole/apex) stepped across this direction

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
//
// `reanchor` (M1d deep tail): on the curve-following reanchor path the intersection curve
// legitimately turns a LARGE TOTAL angle across a tighter graze — more than 90° away from
// the FROZEN entry t★ by the far side. The per-step ≥60° branch-flip guard (a branch point
// flips the raw tangent sharply in ONE step) and the sine floor still hold and still catch
// a genuine branch / tangency; only the "U-turn vs the FROZEN t★" test is relaxed to
// continuity against the PREVIOUS raw tangent (rawPrev), because t★ is a stale reference
// once the curve has turned. Honesty is preserved: a real branch still trips the per-step
// turn guard, the sine floor, or the anti-orbit arc cap. On the frozen-t★ path (reanchor
// false) the behaviour is byte-identical.
bool crossNodeCrossable(const Tangent& tanNew, const Vec3& rawPrev, const Vec3& dirStar,
                        const Tuned& t, Vec3& rawOut, bool reanchor = false) {
  if (!tanNew.valid || tanNew.sine < t.minCrossSine) return false;
  Vec3 rawNew = tanNew.dir;
  const double contDot = math::dot(rawNew, rawPrev);
  if (std::fabs(contDot) < 0.5) return false;              // turned ≥60° in one step
  if (contDot < 0.0) rawNew = rawNew * -1.0;               // orient by continuity
  if (reanchor) {
    // Continuity is against the PREVIOUS step's raw tangent (already enforced by the ≥60°
    // guard above once oriented) — the curve may have turned well past 90° from the frozen
    // t★ and that is a legitimate graze turn, not a U-turn.
    if (math::dot(rawNew, rawPrev) <= 0.0) return false;   // net reversal step-to-step
  } else {
    if (math::dot(rawNew, dirStar) <= 0.0) return false;   // U-turn off the crossing way
  }
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

  // ANTI-ORBIT arc cap (deep-tail reanchor only). When a graze is WIDE (a large fraction
  // of the intersection loop stays near-tangent) the curve-following crossing legitimately
  // traverses a loop-scale arc before the far side recovers — so the cap must admit a full
  // loop. It is a TERMINATION SAFETY (never a correctness gate: far-side recovery + the
  // per-node on-both-surfaces / floor / branch-flip guards decide the crossing). A run that
  // exceeds a few loop-scales is genuinely orbiting → discard + defer, never fabricate.
  // Scaled to the domain (loopClose·h0 is the loop-closure window; ×crossMaxSteps admits a
  // full loop-sized traversal). Only active on the reanchor path; frozen-t★ is byte-identical.
  const double crossArcCap =
      std::max(hCrossCap, t.loopClose * t.h0) * static_cast<double>(t.crossMaxSteps);
  double crossArc = 0.0;
  int recoveredRun = 0;  // consecutive recovered nodes (reanchor hand-back stability)
  // M1e incremental orientation reference: the last ACCEPTED step direction. Seeded to the
  // frozen t★ so the first step is identical either way; thereafter it tracks the curve.
  Vec3 prevStepDir = dirStar;

  for (int i = 0; i < t.crossMaxSteps; ++i) {
    // ADVANCE DIRECTION for this step. By default it is the FROZEN t★ (the shipped S4-c
    // fixed-plane cut — well-posed as local sine → 0, byte-identical when the deep-tail
    // flag is off). With `adaptiveCrossReanchor` on, RE-ANCHOR it toward the LOCAL
    // intersection tangent at the current node so the advance plane FOLLOWS the curve's
    // turn through a tighter graze instead of slicing it far from the guess. Inside a
    // CROSSABLE graze the pair sine stays bounded above the floor, so t_local = nA×nB is a
    // well-defined curve direction (unlike a raw node secant, which is noisy at fine h). It
    // is CONTINUITY-ORIENTED toward the crossing way and only adopted (blended, then a bit)
    // when it still heads the CROSSING way (dot(t_local, t★) > 0); otherwise the frozen t★
    // is kept — the honesty anchor never rotates past a U-turn.
    //
    // M1e: with `reanchorIncrementalOrientation` the ORIENTATION REFERENCE for both the sign
    // test and the adoption gate is the PREVIOUS ACCEPTED step direction rather than the
    // frozen t★. Continuity is monotone across an arbitrarily large accumulated turn; the
    // frozen reference inverts the true forward tangent the moment the turn passes 90° and
    // traps the march in a 2-cycle. Both tests use the same `ref` — re-referencing only the
    // sign test relocates the failure rather than fixing it. t★ stays the honesty anchor in
    // `crossNodeCrossable` (sine floor + ≥60° branch-flip) and in the band-minimum scan.
    Vec3 stepDir = dirStar;
    if (t.adaptiveCrossReanchor) {
      const Tangent tl = intersectionTangent(A, B, cur);
      if (tl.valid) {
        const Vec3& ref = t.reanchorIncrementalOrientation ? prevStepDir : dirStar;
        Vec3 loc = tl.dir;
        if (math::dot(loc, ref) < 0.0) loc = loc * -1.0;  // orient toward the crossing way
        if (math::dot(loc, ref) > 0.0) {
          const double a = t.reanchorBlend;
          Vec3 blended = loc * a + dirStar * (1.0 - a);
          const double bl = math::norm(blended);
          if (bl > 1e-300 && math::dot(blended, ref) > 0.0) stepDir = blended * (1.0 / bl);
        }
      }
    }
    // Curvature-aware guess along stepDir, then fixed-plane-cut correct: the corrector's
    // advance residual uses stepDir (well-posed as local sine → 0), NOT the degenerating
    // local tangent.
    State guess = cur;
    guess.p = curvaturePredict(havePrev ? &prev : nullptr, cur, h, stepDir);
    advanceParams(A, cur.u1, cur.v1, A.uPeriod, stepDir * h, A.domain, guess.u1, guess.v1);
    advanceParams(B, cur.u2, cur.v2, B.uPeriod, stepDir * h, B.domain, guess.u2, guess.v2);
    const CorrectorOut c = correct(A, B, cur, stepDir, h, guess, t.onSurfTol);

    // VERIFY: on both surfaces, advanced along t★, and the chord/arc DEFLECTION is within
    // budget. The deflection cap is what forces the crossing to RESOLVE the near-tangent
    // region rather than leap across it: at a true tangency / branch point the curve bows
    // sharply, so a big step has huge deflection → shrink until it SAMPLES the sine → 0
    // dip, where the floor/flip guards below abort. A genuine graze bows gently and is
    // accepted at a healthy step.
    // NET progress is measured against the STEP DIRECTION actually taken (stepDir): the
    // curve advances along its OWN tangent through the turn, so on the frozen-t★ path
    // stepDir == dirStar and this is the original along-t★ test (byte-identical); on the
    // reanchor path it lets a legitimately TURNING graze keep advancing (a step whose
    // t★-projection dips below 0.25·h at the pinch is NOT a stall — it is the curve
    // turning). The anti-orbit arc cap below bounds the total path so a non-traversing
    // orbit still terminates + defers. Deflection is measured in the plane the step was
    // actually taken in (stepDir) so a re-anchored step is not spuriously flagged.
    const double advance = math::dot(c.s.p - cur.p, stepDir);
    const bool advanced = advance >= 0.25 * h;
    const double defl = c.ok ? stepDeflection(A, B, cur, c.s, stepDir, t.onSurfTol)
                             : std::numeric_limits<double>::infinity();
    if (!c.ok || !advanced || defl > t.maxDeflection) {
      if (h <= t.minStep) { out.resize(base); return r; }  // stuck at the floor → defer
      h *= 0.5;                                            // shrink and retry the same node
      continue;
    }

    const Tangent tanNew = intersectionTangent(A, B, c.s);
    Vec3 rawNew{};
    if (!crossNodeCrossable(tanNew, rawPrev, dirStar, t, rawNew, t.adaptiveCrossReanchor)) { out.resize(base); return r; }

    crossArc += math::distance(cur.p, c.s.p);
    out.push_back(toNode(c.s, c.resid));
    prev = cur; havePrev = true;
    cur = c.s;
    rawPrev = rawNew;
    prevStepDir = stepDir;  // M1e: orientation reference for the next step
    r.maxResid = std::max(r.maxResid, c.resid);
    ++r.count;

    // ANTI-ORBIT: a crossable graze traverses in a short bounded arc; if the reanchored
    // curve-following step has run past the arc cap without recovering on the far side it
    // is orbiting (not traversing) → discard + defer. Never fabricates a close.
    if (t.adaptiveCrossReanchor && crossArc > crossArcCap) {
      out.resize(base);
      return r;
    }

    // M1e NET-TRANSPORT guard (reanchor + incremental orientation only). `crossArcCap` above
    // is derived from crossMaxSteps, so it scales WITH the budget and can never bind — raising
    // crossMaxSteps 64× leaves the orbit arc growing linearly and exit F silent. And the
    // per-step `advanced` test is structurally blind (the corrector pins advance/h to exactly
    // 1). This measures the one thing neither can see: ARC SPENT vs NET DISPLACEMENT from the
    // band entry. A traversing crossing is near-ballistic (measured ratio 1.19 at the crossing
    // pose dx=0.590); an orbit is not (17.8 and 20.4 at dx=0.593/0.595) — a >14× separation, so
    // the 4× threshold is nowhere near delicate. Floored by hCrossCap so the first few steps,
    // where net displacement is legitimately tiny, cannot trip it. Termination safety only: it
    // turns a residual orbit into an immediate HONEST defer instead of a burned budget, and can
    // never fabricate a close.
    if (t.reanchorIncrementalOrientation) {
      const double net = math::distance(stall.p, cur.p);
      if (crossArc > 4.0 * std::max(net, hCrossCap)) { out.resize(base); return r; }
    }

    // Recovered on the far side → hand back to the normal S3 march. Frozen-t★ path uses the
    // exit hysteresis bandExitSin (1.5·enter) — byte-identical. Reanchor path: a WIDE graze's
    // inter-pinch stretch may top out only modestly above the ENTER threshold (below the
    // exit hysteresis), yet it is a genuine transversal recovery — sine steadily ≥ bandEnter
    // where S3 itself is comfortable. Hand back at bandEnter with a stability requirement (two
    // consecutive recovered nodes) so a single lucky sample can't trigger a premature exit; if
    // S3 dips near-tangent again it simply re-enters the crossing. No tolerance is weakened —
    // the recovered region is handed to the SAME S3 corrector that runs everywhere else.
    const double recoverSin = t.adaptiveCrossReanchor ? t.bandEnterSin : t.bandExitSin;
    if (tanNew.sine >= recoverSin) {
      if (!t.adaptiveCrossReanchor || recoveredRun >= 1) {  // reanchor: require 2 in a row
        r.crossed = true;
        r.end = cur;
        return r;
      }
      ++recoveredRun;
    } else {
      recoveredRun = 0;
    }
    // STEP CONTROL. Frozen-t★ path: keep the step fine through the whole band (bounded by
    // the crossing cap / minStep) — byte-identical to before. Reanchor path: a WIDE graze
    // has TRANSVERSAL stretches between pinches (the loop recovers to sine≈1 there); pinning
    // the step fine crawls those in ~loop-scale arc and exhausts the node budget. Because
    // reanchoring follows the true curve tangent (and every node is still verified on both
    // surfaces + gated), the step may GROW back toward the normal maxStep as sine climbs
    // above the enter threshold, then SHRINK to the fine cap as it re-approaches a pinch —
    // so the crossing spends its fine steps where the graze actually is. This changes only
    // WHERE the resolution is spent, never a tolerance: the sine-floor / branch-flip / arc
    // guards are unchanged, so a genuine tangency still defers.
    if (t.adaptiveCrossReanchor) {
      if (tanNew.sine >= t.bandEnterSin) h = std::min(t.maxStep, h * 1.5);  // transversal stretch → speed up
      else                               h = std::max(t.minStep, std::min(h, hCrossCap));  // near a pinch → resolve fine
    } else {
      h = std::max(t.minStep, std::min(h, hCrossCap));
    }
  }

  out.resize(base);  // budget exhausted without recovering → discard, defer
  return r;
}

// ── S4-e: single-surface chart-singularity detection + point-based crossing ──────────
//
// A CHART SINGULARITY is ONE surface's own (u,v) parametrization degenerating (‖dU‖ → 0 at
// a sphere pole v=±π/2 or a cone apex, signed radius = 0) while its 3D point + normal stay
// FINITE. The intersection can be perfectly TRANSVERSAL through it (the pair sine need not
// collapse), yet advanceParams' single-surface 2×2 goes rank-deficient there, so S3 either
// spuriously BoundaryExits (the pole sits on a non-periodic v edge) or step-crawls the node
// budget (the apex). The witness is the single-surface Jacobian rank-drop — computed from ONE
// surface's ‖dU‖/‖dV‖, DISTINCT from the S4-c pair sine ‖n₁×n₂‖ and the S4-d locus flip
// (chart_singularity.h). Which surface (if any) collapsed at the current node.
enum class ChartSurf { None, A, B };

struct ChartHit {
  ChartSurf surf = ChartSurf::None;
  chartsing::ChartCond cond{};  ///< the collapsed surface's conditioning (‖dU‖, ‖dV‖, finite normal)
  // TRANSPOSE-SYMMETRIC axis (S4-e transposed slice). Which of the collapsed surface's two
  // parametric directions ran out: false = the U chart collapsed (‖dU‖→0 at a v-edge, the
  // original sphere-of-revolution case), true = the V chart collapsed (‖dV‖→0 at a u-edge, a
  // TRANSPOSED authoring). The crossing uses the SAME point-based corrector either way; only the
  // far-side chart re-seed + the pole-edge classification transpose on this flag.
  bool axisV = false;
};

// A single surface's chart condition at a node, AUGMENTED with the S4-e-general
// degenerate-v-edge test. The pointwise `chartConditionAt` collapse (‖dU‖ ≪ ‖dV‖·frac) fires
// reliably on a CIRCULAR pole (‖dU‖ → 0 uniformly along the row) but only in a razor-thin band on
// a NON-CIRCULAR (elliptical / lumpy) collapsed-row pole — ‖dU‖ collapses SLOWLY along the minor
// axis, so a meridian marched there reaches the non-periodic v edge and spuriously BoundaryExits
// before the pointwise ratio ever crosses `chartCollapseFrac`. To catch that, ALSO flag a collapse
// when the surface is APPROACHING a genuine degenerate v-edge (chartsing::degenerateVEdge — the
// whole edge row collapses to one point, a SURFACE property, FALSE at a finite v-cap) AND ‖dU‖ has
// already dropped well below ‖dV‖ (the looser edge-approach band). This never fires at a finite
// boundary (the edge row is a full circle there, so degenerateVEdge is false) and never at a cone
// APEX (interior v, not near a v-edge — the pointwise test still owns that case). Additive: a
// circular pole already trips the pointwise test at the same or a nearer node, so the sphere / cone
// / freeform-sphere fixtures are unchanged.
chartsing::ChartCond chartCondAugmented(const SurfaceAdapter& S, double u, double v, double scale,
                                        const Tuned& t) {
  chartsing::ChartCond c = chartsing::chartConditionAt(S, u, v, scale, t.chartCollapseFrac);
  if (c.collapsed || c.collapsedV || !c.normalFinite) return c;  // collapsed (U or V), or NaN → leave it
  // FREEFORM only (uPeriod == 0). An ANALYTIC surface (sphere, uPeriod = 2π) crosses via the exact
  // poleContinuationU u+π jump, whose small-h pin needs the witness to fire RIGHT AT the pole — so
  // firing it early (in the edge band) would break the analytic crossing. Analytic circular poles
  // already trip the pointwise test reliably (‖dU‖ → 0 uniformly), so they need no augmentation and
  // stay bit-identical. The augmentation is exactly for the FREEFORM non-circular pole the pointwise
  // test misses (and whose crossing uses the wide reflected pin below).
  if (S.uPeriod > 0.0) return c;
  double edgeV;
  if (chartsing::degenerateVEdge(S, v, scale, t.chartCollapseFrac, edgeV)) {
    // Near the degenerate edge (within the fine approach band) AND ‖dU‖ already ≪ ‖dV‖ (the row is
    // measurably collapsing here) ⇒ a NON-CIRCULAR pole approach the pointwise test would miss.
    const double distToEdge = std::fabs(v - edgeV);
    const double band = t.chartEdgeApproachV * std::fabs(S.domain.dv());
    if (distToEdge <= band && c.dU < t.chartEdgeCollapseFrac * c.dV) c.collapsed = true;
  }
  return c;
}

// Evaluate the chart witness on BOTH surfaces at a node; report the collapsed one (the one
// with the SMALLER ‖dU‖/‖dV‖ ratio if both somehow collapse). Independent of the S4-c/S4-d
// seams: reads only single-surface finite-difference dU/dV, never the pair normal cross.
ChartHit chartCondition(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& s,
                        double scale, const Tuned& t) {
  const chartsing::ChartCond ca = chartCondAugmented(A, s.u1, s.v1, scale, t);
  const chartsing::ChartCond cb = chartCondAugmented(B, s.u2, s.v2, scale, t);
  ChartHit h;
  // A collapse is present on a surface if EITHER its U or its V chart ran out (transpose-symmetric).
  const bool aColl = ca.collapsed || ca.collapsedV;
  const bool bColl = cb.collapsed || cb.collapsedV;
  // The collapse SEVERITY on a surface is the smaller-over-larger tangent ratio (the collapsed
  // direction against the healthy one), directionless so U- and V-collapses compare on equal
  // footing when both surfaces somehow collapse.
  auto sev = [](const chartsing::ChartCond& c) {
    return std::min(c.dU, c.dV) / std::max(std::max(c.dU, c.dV), 1e-300);
  };
  const double ra = sev(ca), rb = sev(cb);
  if (aColl && (!bColl || ra <= rb)) { h.surf = ChartSurf::A; h.cond = ca; h.axisV = ca.collapsedV; }
  else if (bColl)                    { h.surf = ChartSurf::B; h.cond = cb; h.axisV = cb.collapsedV; }
  return h;
}

// True once the chart U direction has RECOVERED on BOTH surfaces (‖dU‖ back above the
// collapse threshold) — the signal to hand back to the normal S3 march.
bool chartRecovered(const SurfaceAdapter& A, const SurfaceAdapter& B, const State& s,
                    double scale, const Tuned& t) {
  // Use the SAME augmented condition the witness uses, so a non-circular pole that fired via the
  // degenerate-v-edge band is recognised as RECOVERED once v steps back out of that band (‖dU‖
  // recovering OR the node having left the edge approach band). Bit-identical to the raw test for
  // a circular pole / cone apex (those never trip the edge-augmentation).
  auto anyCollapse = [&](const SurfaceAdapter& S, double u, double v) {
    const chartsing::ChartCond c = chartCondAugmented(S, u, v, scale, t);
    return c.collapsed || c.collapsedV;  // recovered ⇔ NEITHER chart direction still collapsed
  };
  return !anyCollapse(A, s.u1, s.v1) && !anyCollapse(B, s.u2, s.v2);
}

// Seed the SINGULAR surface's far-side (u,v) LOOSELY from chart continuity, using only the
// finite point + normal (never the degenerate dU). The corrector then confirms it; a wrong pick
// simply fails verification and the march defers (no fabrication). Two removable-singularity
// cases:
//  * SPHERE POLE. A great arc through the pole continues on the OPPOSITE meridian: the longitude
//    jumps by half a turn (poleContinuationU, u_in+π) while the latitude REFLECTS about the pole
//    (v stays near ±π/2, then decreases back into the domain) — so the far-side v seed is the
//    SAME magnitude, kept just inside the pole edge. The corrector re-lands the exact v. This is
//    the PERIODIC (uPeriod>0) analytic path; it stays bit-identical.
//  * FREEFORM POLE. A collapsed B-spline/NURBS control ROW (‖dU‖→0, finite point+normal — the
//    spline analog of the sphere pole) carries uPeriod==0, so no analytic meridian jump exists.
//    The far-side (u,v) is re-seeded by a POINT-ONLY nearest-(u,v) inversion of the continued 3D
//    tangent target (chartsing::freeformChartInvert) — the same removable-singularity crossing,
//    generalized. The corrector confirms it on both surfaces exactly as for the analytic pole.
//  * CONE APEX. The apex is a single 3D point; the curve passes straight through to the far
//    nappe, whose signed radius (hence v) has the OPPOSITE sign. Flip v; the longitude is
//    unchanged (the straight line keeps its azimuth). The corrector confirms the far nappe.
// `s` is the singular surface's (u,v) IN the collapsed node (near the pole/apex); returns the
// far-side (uOut,vOut) seed. `poleCase` selects the sphere-pole reflect vs the apex sign-flip.
// A chart singularity is a POLE (sphere) when the collapse sits on a v DOMAIN EDGE (v = ±π/2 —
// the latitude runs out there), and an APEX (cone) when it sits in the v INTERIOR (signed radius
// crosses zero away from either v-edge). Distinguished by the collapsed v's proximity to a
// v-edge — periods alone cannot tell them apart (both have periodic u, non-periodic v).
bool isPoleEdge(const SurfaceAdapter& S, double v) {
  const ParamBox& d = S.domain;
  const double ev = std::max(d.dv() * 1e-3, 1e-9);
  return v <= d.v0 + ev || v >= d.v1 - ev;
}
// Transposed pole (V collapse): the pole sits on a u-EDGE (the v column runs out there). The
// mirror of isPoleEdge with U/V swapped.
bool isPoleEdgeU(const SurfaceAdapter& S, double u) {
  const ParamBox& d = S.domain;
  const double eu = std::max(d.du() * 1e-3, 1e-9);
  return u <= d.u0 + eu || u >= d.u1 - eu;
}

void chartFarUV(const SurfaceAdapter& S, bool poleCase, bool axisV, double u, double v,
                const Point3& target, double& uOut, double& vOut) {
  if (poleCase && !axisV) {
    // U-COLLAPSE pole (the original sphere-of-revolution case). Sphere pole: the great arc
    // CONTINUES on the OPPOSITE meridian — jump the longitude by half a turn (poleContinuationU,
    // u_in+π) and KEEP the latitude (the far arc runs back DOWN from the same pole edge, v
    // unchanged; there is no v beyond ±π/2). The corrector re-lands v.
    if (S.uPeriod > 0.0) {
      uOut = chartsing::poleContinuationU(u, S.uPeriod);
    } else {
      // FREEFORM pole (uPeriod == 0 — a collapsed B-spline/NURBS control row): no analytic u+π
      // meridian jump exists, so recover the far LONGITUDE numerically from the far-side `target`.
      // `target` is the point the crossing wants to reach: for a CIRCULAR pole it is the fine-step
      // point (the witness fires AT the pole, so that is already across it); for a NON-CIRCULAR
      // (elliptical) pole crossChartSingularity supplies the REFLECTED far point (2·P_pole − anchor)
      // so the inversion picks the correct FAR meridian even though the witness fired a finite
      // distance before the pole. Point-only (never the degenerate dU); the corrector verifies the
      // pick on both surfaces, so a wrong longitude simply fails and the march defers.
      uOut = chartsing::freeformChartInvert(S, target, v);
    }
    vOut = v;  // KEEP the latitude (analytic reflect and freeform share this — far arc runs back down)
  } else if (poleCase && axisV) {
    // TRANSPOSED (V-COLLAPSE) pole: the roles of U and V swap. The pole sits on a u-EDGE, the
    // "longitude" is now v (revolution on V) and the collapsing "latitude" is u. Recover the far
    // LATITUDE v by the transposed point-only inversion at the fixed near-pole u; KEEP u (the far
    // arc runs back from the same u-edge). Freeform-only in practice (a transposed authoring is a
    // freeform NURBS, uPeriod == 0); the corrector verifies the pick, else the march defers.
    vOut = chartsing::freeformChartInvertV(S, target, u);
    uOut = u;
  } else {
    uOut = u;       // apex: azimuth unchanged (the straight line keeps its longitude)
    vOut = -v;      // far nappe: signed radius (hence v) flips sign through the apex
  }
}

// The result of one chart-singularity crossing (mirrors CrossOut).
struct ChartCrossOut {
  bool crossed = false;
  State end{};          ///< the far-side state to resume the normal march from
  int count = 0;        ///< nodes appended
  double maxResid = 0.0;
};

// ── S4-e crossing driver (point-based fixed-plane cut) ───────────────────────────
//
// Called from marchDir when a chart collapse is detected (enableChartSingularities on). Makes a
// bounded sequence of POINT-BASED JUMPS along the fixed last-good tangent t★ to STEP ACROSS the
// singular point (sphere pole / cone apex). The corrector is the S4-c / branch_point.h FIXED-
// PLANE cut: it drives the 3D residual A.point − B.point → 0 with an along-t★ hyperplane advance
// (dot(A.point − anchor, t★) = d) — NEITHER residual touches the degenerate single-surface dU,
// so it stays well-posed exactly where advanceParams failed. The singular surface's far-side
// (u,v) are re-seeded LOOSELY from chart continuity (chartFarUV — pole meridian jump / apex v
// sign flip); the corrector confirms them, and a node is EMITTED only if it verifies on BOTH
// surfaces ≤ onSurfTol. Once ‖dU‖ has RECOVERED on both surfaces (we are clear of the pole/apex)
// and the far side made real progress, resume the normal S3 march. On ANY failure DISCARD the
// whole band (roll back `out`) and return crossed=false so the caller STOPS + defers → OCCT.
// Never fabricates a pole/apex point: the singular point itself is never emitted; the crossing
// steps FROM a last-good pre-singular node TO verified far-side nodes only.
//
// COMPLEXITY NOTE: the isolated S4-e crossing handler the design flags — the one place the chart
// crossing loop + verification sits, kept out of the hot marchDir dispatch (parallels
// crossNearTangent for S4-c).
ChartCrossOut crossChartSingularity(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                    const State& stall, const Vec3& tStarFwd, ChartSurf which,
                                    bool axisV, const Tuned& t, double scale,
                                    std::vector<WLinePoint>& out) {
  ChartCrossOut r;
  const std::size_t base = out.size();  // rollback marker: discard all crossing nodes on failure
  const Vec3 dirStar = tStarFwd;        // fixed crossing direction (last-good forward tangent)
  const bool singularIsA = which == ChartSurf::A;
  const SurfaceAdapter& singular = singularIsA ? A : B;
  // POLE (sphere, collapse on a v edge — continue on the opposite meridian) vs APEX (cone,
  // collapse in the v interior — pass through to the far nappe, v sign flips).
  const double vSing = singularIsA ? stall.v1 : stall.v2;
  const double uSing = singularIsA ? stall.u1 : stall.u2;
  // POLE vs APEX. A pole sits at a genuine degenerate EDGE (the whole edge row/column collapses to
  // one point); an apex is an INTERIOR collapse (cone signed radius crosses zero away from an edge).
  // isPoleEdge's thin (1e-3 of domain) band works for a CIRCULAR pole (the witness fires right at
  // the edge), but the NON-CIRCULAR (elliptical) witness fires up to chartEdgeApproachV (2%) BEFORE
  // the edge, well outside that thin band — so classify by the SURFACE property (degenerate edge
  // row/column collapsed) as well. Either signal ⇒ pole; neither ⇒ apex. TRANSPOSE-SYMMETRIC: a
  // V-collapse (axisV) pole sits on a u-EDGE, so test the u-edge witnesses; a U-collapse on a v-edge.
  double poleEdgeV, poleEdgeU;
  const bool poleCase =
      axisV ? (isPoleEdgeU(singular, uSing) ||
               chartsing::degenerateUEdge(singular, uSing, scale, 1e-2, poleEdgeU))
            : (isPoleEdge(singular, vSing) ||
               chartsing::degenerateVEdge(singular, vSing, scale, 1e-2, poleEdgeV));

  State cur = stall;
  const double h = t.chartStep;         // FINE, fixed step across the band (resolve, don't leap)
  bool crossed = false;                 // have we applied the far-side chart map yet?

  for (int i = 0; i < t.chartMaxSteps; ++i) {
    const Point3 anchor = cur.p;
    const ChartHit hitCur = chartCondition(A, B, cur, scale, t);
    // The along-t★ pin distance for this reproject. Normally the FINE h (resolve the band, don't
    // leap). But the CROSSING step of a NON-CIRCULAR freeform pole is a genuine DISCRETE jump: the
    // witness fired a finite distance before the pole (‖dU‖ collapses slowly on the minor axis), so
    // the far meridian is ~2·(pole−anchor) away — a tiny-h pin cannot reach it and the reproject
    // fails. On that one step we pin at the ACTUAL along-t★ distance to the reflected far point
    // (2·P_pole − anchor). Circular poles keep h (the witness fires at the pole, jump ≈ h); the pin
    // never weakens onSurfTol — it only sets WHERE along t★ the verified far node sits.
    double pinDist = h;
    Point3 target = anchor + dirStar * h;
    const bool crossingStep = !crossed && hitCur.surf != ChartSurf::None;
    if (crossingStep && poleCase) {
      const SurfaceAdapter& sing = (hitCur.surf == ChartSurf::A) ? A : B;
      if (sing.uPeriod <= 0.0) {  // freeform pole → reflect through the pole point for the far target
        const double vS = (hitCur.surf == ChartSurf::A) ? cur.v1 : cur.v2;
        const double uS = (hitCur.surf == ChartSurf::A) ? cur.u1 : cur.u2;
        // Locate the collapsed edge point. U-collapse ⇒ a v-edge row (S.point(uS, edgeV));
        // transposed V-collapse ⇒ a u-edge column (S.point(edgeU, vS)).
        double edge;
        bool onEdge = false;
        Point3 pole{};
        if (axisV) {
          onEdge = chartsing::degenerateUEdge(sing, uS, scale, 1e-2, edge);
          if (onEdge) pole = sing.point(edge, vS);
        } else {
          onEdge = chartsing::degenerateVEdge(sing, vS, scale, 1e-2, edge);
          if (onEdge) pole = sing.point(uS, edge);
        }
        if (onEdge) {
          const Point3 far{2.0 * pole.x - anchor.x, 2.0 * pole.y - anchor.y, 2.0 * pole.z - anchor.z};
          const double d = math::dot(far - anchor, dirStar);
          if (d > h) { pinDist = d; target = far; }  // only widen (never shrink below the fine h)
        }
      }
    }
    // Seed the corrector at the CONTINUITY guess. The point-based reproject needs only a nearby
    // (u,v) seed — it does NOT use the degenerate dU — so we advance the NON-singular surface by
    // its (well-conditioned) tangent plane and carry the singular surface's (u,v) by continuity.
    // At the collapse itself (once, when the singular chart's ‖dU‖ has collapsed) we FLIP to the
    // FAR chart via chartFarUV (pole meridian jump / apex v sign flip); after that the far chart
    // advances by continuity as v moves away from the pole/apex until ‖dU‖ recovers.
    branchpt::BPState seed{cur.u1, cur.v1, cur.u2, cur.v2, target};
    const SurfaceAdapter& reg = singularIsA ? B : A;   // the well-conditioned (regular) surface
    if (singularIsA) advanceParams(reg, cur.u2, cur.v2, reg.uPeriod, dirStar * pinDist, reg.domain, seed.u2, seed.v2);
    else             advanceParams(reg, cur.u1, cur.v1, reg.uPeriod, dirStar * pinDist, reg.domain, seed.u1, seed.v1);
    if (crossingStep) {
      double uO, vO;
      if (singularIsA) { chartFarUV(A, poleCase, axisV, cur.u1, cur.v1, target, uO, vO); seed.u1 = uO; seed.v1 = vO; }
      else             { chartFarUV(B, poleCase, axisV, cur.u2, cur.v2, target, uO, vO); seed.u2 = uO; seed.v2 = vO; }
      crossed = true;
    }

    // Point-based fixed-plane reproject: land on both surfaces, held at distance pinDist along t★
    // from the anchor (the well-posed-as-dU→0 cut). Never uses the degenerate single-surface dU.
    const auto landed = branchpt::reproject(A, B, seed, t.onSurfTol, anchor,
                                            branchpt::AlongPin{dirStar, pinDist});
    if (!landed) { out.resize(base); return r; }  // will not verify on both surfaces → defer
    const State next{landed->u1, landed->v1, landed->u2, landed->v2, landed->p};

    // Real progress along t★ (not sliding back to the anchor).
    const double advance = math::dot(next.p - anchor, dirStar);
    if (!(advance >= 0.25 * h)) { out.resize(base); return r; }

    const double resid = math::distance(A.point(next.u1, next.v1), B.point(next.u2, next.v2));
    out.push_back(toNode(next, resid));
    r.maxResid = std::max(r.maxResid, resid);
    ++r.count;
    cur = next;

    // Recovered on the far side: we have applied the far-chart map AND ‖dU‖ is back above the
    // collapse threshold on BOTH surfaces (clear of the pole/apex). Resume the normal S3 march.
    if (crossed && chartRecovered(A, B, cur, scale, t)) {
      r.crossed = true;
      r.end = cur;
      return r;
    }
  }

  out.resize(base);  // budget exhausted without recovering → discard, defer (honest gap)
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
  bool crossedChart = false; // S4-e: has this direction stepped across a chart singularity?
  Vec3 seedFwd{};            // seed's outgoing FORWARD tangent (closure-direction gate)
  bool haveSeedFwd = false;
  double arcLen = 0.0;       // S4-f: accumulated arc length from the seed (TRUE-RETURN gate)
};

// Loop-closure proximity. In a pure S3 transversal trace this is exactly `loopClose·h`
// (bit-identical to S3). Once a graze has been CROSSED the step h collapses to near
// minStep through the band, which would make `loopClose·h` too tight to detect the loop
// returning to the seed (the seed can sit right at a graze); after a crossing we floor
// the radius at the NOMINAL step so closure stays detectable.
double closeRadius(const Walk& w, const Tuned& t) {
  return (w.crossedAny || w.crossedChart) ? t.loopClose * std::max(w.h, 0.5 * t.h0)
                                          : t.loopClose * w.h;
}
// S4-f TRUE-RETURN arc gate. A genuine loop closes only after the march has actually
// TRAVELLED a full circuit and come BACK — its accumulated arc length must exceed the
// diameter of the closure window (2·closeRadius). A curve that is merely still WITHIN the
// window after a few steps (the false-close an inflated proximity radius admits — the march
// never left) has arcLen ≤ 2·closeRadius and is REFUSED. This is a necessary condition — a
// real loop's perimeter is far larger than its detection window, so every truly-closing
// control passes trivially; only the not-yet-departed near-pass is blocked. Combined with
// the tangent-continuity gate (closeAligned), closure is a TRUE-RETURN test, not proximity.
bool closeReturned(const Walk& w, const Tuned& t) {
  return w.arcLen > 2.0 * closeRadius(w, t);
}
// S4-f TRUE-RETURN direction gate — a NECESSARY-CONDITION tightening applied to ALL
// closures. A loop closes only when the march comes back near the seed HEADING THE WAY IT
// LEFT: dot(fwdNow, seedFwd) ≥ closureTangentCos. A curve merely PASSING near its seed (or
// an earlier node) while heading the OTHER way — the false-close the pure-proximity S3 test
// admits — is now REFUSED, so the march continues to its true termination. This can only
// REFUSE a close, never MAKE one: a genuinely closing loop returns roughly antiparallel-free
// (dot ≈ +1 ≫ −0.5), so every truly-closing control still closes byte-identically.
//
// The gate needs the seed's captured outgoing tangent; before it is captured (haveSeedFwd
// false — only possible in the first couple of steps, below the step > 2 closure guard) it
// cannot refuse, so it returns true. Once a graze/pole was crossed the raw cross-product sign
// is unreliable, so we compare with the previously-derived FORWARD tangent (w.tStar), which
// tryBandEntry/tryChartBand already re-orient by the crossing chord.
bool closeAligned(const Walk& w, const Vec3& fwdNow, const Tuned& t) {
  if (!w.haveSeedFwd) return true;  // seed tangent not captured yet → cannot refuse
  const double nf = math::norm(fwdNow), ns = math::norm(w.seedFwd);
  if (nf <= 1e-14 || ns <= 1e-14) return true;  // degenerate heading → do not refuse
  return math::dot(fwdNow, w.seedFwd) / (nf * ns) >= t.closureTangentCos;
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
  w.arcLen += math::distance(w.cur.p, cx.end.p);     // S4-f: crossed arc counts toward the return gate
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
  if (step > 2 && math::distance(w.cur.p, start.p) <= closeRadius(w, t) &&
      closeReturned(w, t) && closeAligned(w, w.tStar, t)) {
    DirResult done; done.end = DirEnd::LoopClosed; done.crossed = res.crossed;
    done.maxCrossResid = res.maxCrossResid; done.sineAtStop = recSine; return done;
  }
  if (atBoundary(A, B, w.cur)) {
    DirResult done; done.end = DirEnd::Boundary; done.crossed = res.crossed;
    done.maxCrossResid = res.maxCrossResid; done.sineAtStop = recSine; return done;
  }
  return std::nullopt;  // crossed → continue the walk
}

// ── S4-e chart-band entry (parallels tryBandEntry) ───────────────────────────────
//
// At a detected single-surface chart collapse (‖dU‖ → 0 on `which`, finite normal), try to
// STEP ACROSS the pole/apex via crossChartSingularity. On success it updates `w` to the
// recovered far side and returns nullopt (the caller CONTINUES the walk, re-checking closure/
// boundary here); on a non-crossable / unverifiable singularity it returns a NearTangent
// DirResult (the caller STOPS + defers → OCCT, reporting the measured gap). Requires a valid
// last-good forward tangent (w.haveStar): the crossing needs a fixed t★ for the point-based cut.
// Isolating this keeps marchDir a flat dispatch.
std::optional<DirResult> tryChartBand(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                      const State& start, int step, ChartSurf which, bool axisV,
                                      const Tuned& t, double scale, Walk& w,
                                      std::vector<WLinePoint>& out, DirResult& res) {
  const ChartCrossOut cx =
      crossChartSingularity(A, B, w.cur, w.tStar, which, axisV, t, scale, out);
  if (!cx.crossed) {  // chart collapse the point-based cut could not verify across → honest stop
    DirResult stop;
    stop.end = DirEnd::NearTangent;
    stop.sineAtStop = intersectionTangent(A, B, w.cur).sine;
    stop.stopState = w.cur;
    return stop;
  }
  res.chartCrossed += cx.count > 0 ? 1 : 0;  // one pole/apex stepped across (count = nodes)
  res.maxCrossResid = std::max(res.maxCrossResid, cx.maxResid);
  const Vec3 fwd = cx.end.p - w.cur.p;  // forward chord across the singular band
  w.arcLen += math::distance(w.cur.p, cx.end.p);  // S4-f: crossed arc counts toward the return gate
  w.cur = cx.end;
  w.h = std::max(t.minStep, 0.5 * t.h0);  // resume with a modest step
  // Re-derive the forward sign so the resumed march keeps going the crossing way (the raw
  // cross-product sign is reliable again once ‖dU‖ recovered), refreshing tStar / lastGoodSine.
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
  w.crossedChart = true;
  if (step > 2 && math::distance(w.cur.p, start.p) <= closeRadius(w, t) &&
      closeReturned(w, t) && closeAligned(w, w.tStar, t)) {
    DirResult done; done.end = DirEnd::LoopClosed; done.crossed = res.crossed;
    done.chartCrossed = res.chartCrossed; done.maxCrossResid = res.maxCrossResid;
    done.sineAtStop = recSine; return done;
  }
  if (atBoundary(A, B, w.cur)) {
    DirResult done; done.end = DirEnd::Boundary; done.crossed = res.crossed;
    done.chartCrossed = res.chartCrossed; done.maxCrossResid = res.maxCrossResid;
    done.sineAtStop = recSine; return done;
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
        closeReturned(w, t) && closeAligned(w, w.tStar, t)) {
      res.end = DirEnd::LoopClosed; res.sineAtStop = tan.sine; return res;
    }

    // S4-e chart singularity: a SINGLE-surface parametrization collapse (‖dU‖ → 0 at a sphere
    // pole / cone apex) with a finite point + normal. Checked BEFORE the S4-c near-tangent and
    // the boundary branches so a pole/apex (which sits on a non-periodic v edge AND ill-
    // conditions advanceParams) routes to the point-based crossing instead of a spurious
    // BoundaryExit or the step-crawl. Independent of the pair sine — computed from single-
    // surface ‖dU‖/‖dV‖. Requires a last-good tangent for the fixed-plane cut.
    if (t.enableChartSingularities && w.haveStar) {
      const ChartHit hit = chartCondition(A, B, w.cur, scale, t);
      if (hit.surf != ChartSurf::None) {
        if (auto done = tryChartBand(A, B, start, step, hit.surf, hit.axisV, t, scale, w, out, res))
          return *done;
        continue;  // crossed → resume the normal walk on the far side
      }
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

    // Loop closure (S4-f TRUE-RETURN): back near the seed AFTER actually travelling a full
    // circuit (closeReturned — arcLen ≫ window, so the march really came back, not merely
    // never left) AND heading the way it left (closeAligned — tangent-continuous). Either
    // condition failing REFUSES the close; neither can MANUFACTURE one, so a truly-closing
    // loop is byte-identical while a near-pass no longer false-closes.
    if (step > 2 && math::distance(c.s.p, start.p) <= closeRadius(w, t) &&
        closeReturned(w, t) && closeAligned(w, fwd, t)) {
      out.push_back(toNode(c.s, c.resid));
      res.end = DirEnd::LoopClosed; res.sineAtStop = tan.sine; return res;
    }

    w.tStar = fwd; w.haveStar = true;
    w.lastGoodSine = tan.sine;                                  // transversality just before any dip
    if (!w.haveSeedFwd) { w.seedFwd = fwd; w.haveSeedFwd = true; }
    w.arcLen += math::distance(w.cur.p, c.s.p);                 // S4-f: accumulate travelled arc
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

// Single-shot least-squares B-spline fit through the polyline with a FIXED pole count.
FittedBSpline fitBSplineFixed(const std::vector<WLinePoint>& pts,
                              const std::vector<double>& tval, int degree, int nPoles) {
  FittedBSpline out;
  const int m = static_cast<int>(pts.size());
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

  // report the worst deviation of the polyline from the fitted curve (at the node params).
  double err = 0.0;
  for (int i = 0; i < m; ++i) {
    const Point3 cv = math::curvePoint(out.degree, out.poles, out.knots, tval[i]);
    err = std::max(err, math::distance(cv, pts[i].point));
  }
  out.maxFitError = err;
  return out;
}

// ── DENSIFY-AND-REFIT (S3 follow-up: a too-loose fit no longer truncates a curve) ──────
//
// The fitted B-spline is a CONVENIENCE curve through the on-both-surfaces polyline — the
// polyline is the ground truth. At the default pole cap (`fitMaxPoles`, 64) a least-squares
// fit SMOOTHS the polyline, so when a loop has MORE nodes than the cap the fit cannot ride
// them all: it deviates from the on-locus nodes by a fit-resolution amount that grows with
// the loop's curvature (measured up to ~5e-3 on the freeform fuzzer's densest high-curvature
// loops — 1000+ on-locus nodes squeezed onto 64 poles — just over the 1e-3 curve-coverage
// budget the fitted spline, not the polyline, is compared at downstream). That deviation is a
// pole-COUNT artifact, not a corrector error: every node is on both surfaces and on the true
// locus (≤ onSurfTol); only the under-resolved smoothing curve cuts the corner between them.
//
// The fix: when the fit's worst deviation from the on-locus NODES (`maxFitError`, measured at
// the node params) exceeds the target, refit ONCE at a HIGHER pole count sized to the loop, so
// the curve rides the nodes it under-fit. This is a pure FIT-QUALITY improvement — the polyline
// is unchanged and NO on-curve / on-surface tolerance is touched or widened; more poles can
// only pull the convenience curve CLOSER to the already-on-locus nodes, never fabricate
// geometry. Cost-bounded to ONE extra solve at a hard-capped pole count (`kDensifyMaxPoles`):
// a global least-squares B-spline fit is O(m·poles²), so an iterative doubling to interpolation
// on a many-thousand-node loop would be prohibitively expensive for a mere convenience curve —
// a single refit at a bounded pole count keeps the extra work affordable. Keying off
// `maxFitError` (node deviation, already computed) makes it both cheap and correct: it fires
// ONLY when the pole cap genuinely under-resolves the nodes (a smooth loop whose 64-pole fit
// already rides the nodes within the target is a NO-OP — byte-identical single fit), never on
// a loop that does not need it. `fitTarget ≤ 0` disables densification entirely.
FittedBSpline fitBSpline(const std::vector<WLinePoint>& pts, int degree, int maxPoles,
                         double fitTarget) {
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

  out = fitBSplineFixed(pts, tval, degree, nPoles);
  // Densify ONLY if requested, the initial fit is valid, and it genuinely UNDER-RESOLVES the
  // nodes (maxFitError over the target) with room to add poles. A smooth loop whose 64-pole fit
  // already rides the nodes skips the refit entirely (byte-identical to the single fit).
  if (fitTarget <= 0.0 || !out.valid() || out.maxFitError <= fitTarget || nPoles >= m) return out;

  // NODE-COUNT GUARD (cost + no-benefit): the refit is a dense O(m·poles²) least-squares solve,
  // so on a VERY dense loop (thousands of nodes — e.g. a chart-singularity pole-crossing loop
  // that circulates many times, or a pathological parametrization) the single 200-pole solve
  // would be prohibitively expensive AND fruitless (poles ≪ nodes cannot follow such a loop to
  // the target anyway). Skip densification there — the initial fit stands; the polyline remains
  // the ground truth and the honest coverage figure is reported unchanged. The fit-density
  // declines this recovers are moderate-node high-curvature loops (≈ 1e3 nodes), which are well
  // under this guard; a 20k-node loop is out of scope for a convenience-curve refit.
  constexpr int kMaxDensifyNodes = 2000;
  if (m > kMaxDensifyNodes) return out;

  // ONE bounded refit: raise the pole count to a loop-sized target, hard-capped so the single
  // O(m·poles²) solve stays affordable. Accept the refit only if it is valid AND strictly
  // tighter than the initial fit (defensive — never a regression).
  constexpr int kDensifyMaxPoles = 200;
  const int target = std::min(m, kDensifyMaxPoles);
  if (target > nPoles) {
    FittedBSpline cand = fitBSplineFixed(pts, tval, degree, target);
    if (cand.valid() && cand.maxFitError < out.maxFitError) out = std::move(cand);
  }
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

// Closest approach between two 3D segments [p0,p1] and [q0,q1]: the minimum distance and the
// clamped parameters s,u ∈ [0,1] of the closest points (Ericson, Real-Time Collision
// Detection §5.1.9). Used to find a TRUE polyline self-crossing (two non-adjacent segments
// whose closest approach ≈ 0), not mere node proximity.
struct SegClosest { double dist; Point3 pOnA; Point3 pOnB; };
SegClosest segSegClosest(const Point3& p0, const Point3& p1, const Point3& q0, const Point3& q1) {
  const Vec3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
  const double a = math::dot(d1, d1), e = math::dot(d2, d2), f = math::dot(d2, r);
  double s = 0.0, u = 0.0;
  if (a <= 1e-18 && e <= 1e-18) { /* both degenerate → points */ }
  else if (a <= 1e-18) { u = clampd(f / e, 0.0, 1.0); }
  else {
    const double c = math::dot(d1, r);
    if (e <= 1e-18) { s = clampd(-c / a, 0.0, 1.0); }
    else {
      const double b = math::dot(d1, d2), den = a * e - b * b;
      s = den > 1e-18 ? clampd((b * f - c * e) / den, 0.0, 1.0) : 0.0;
      u = (b * s + f) / e;
      if (u < 0.0) { u = 0.0; s = clampd(-c / a, 0.0, 1.0); }
      else if (u > 1.0) { u = 1.0; s = clampd((b - c) / a, 0.0, 1.0); }
    }
  }
  const Point3 cA = p0 + d1 * s, cB = q0 + d2 * u;
  return {math::distance(cA, cB), cA, cB};
}

// S4-f: find GENUINE self-crossings of the stitched arm — two NON-ADJACENT polyline segments
// that actually CROSS in 3D (closest approach ≤ a tight touch radius) at a TRANSVERSE angle
// (segment directions more than ~45° apart — not a retrace / (anti)parallel doubling back).
// This is the geometric definition of a self-intersection (a segment-segment crossing), so it
// fires ONCE per real crossing and NOT along parallel-running arcs — the over-count a loose
// node-proximity scan produces on a folded curve. Hits clustered around one physical crossing
// are deduped by spatial proximity of the crossing point. Reported as DATA; the polyline is
// UNCHANGED. DISTINCT from an S4-d BranchNode (a locus flip, ‖nA×nB‖→0, that spawns arms) — a
// self-crossing keeps ONE arm, so a WLine with selfIntersectionCount > 0 has branchPoints 0.
void detectStitchedSelfIntersections(WLine& line, const Tuned& t) {
  const auto& pts = line.points;
  const int n = static_cast<int>(pts.size());
  if (n < 4) { line.selfIntersectionCount = 0; return; }
  // Touch radius: a genuine crossing has the two passes essentially COINCIDENT. Tie it to a
  // small fraction of the nominal step (the segment scale), NOT the loose closure window, so
  // parallel-running arcs a node-proximity test would merge are not counted.
  const double touch = std::max(0.5 * t.h0, t.selfIntersectRadius * 0.25);
  const double mergeR = std::max(t.h0, touch * 2.0);  // dedup radius for one physical crossing
  constexpr double kCrossCosMax = 0.70;               // > ~45° apart ⇒ transverse (not a retrace)
  for (int i = 0; i + 1 < n; ++i) {
    const Vec3 di = pts[static_cast<std::size_t>(i) + 1].point - pts[static_cast<std::size_t>(i)].point;
    const double ni = math::norm(di);
    if (ni <= 1e-14) continue;
    for (int j = i + 2; j + 1 < n; ++j) {  // j ≥ i+2 skips the segment sharing node i+1
      const Vec3 dj = pts[static_cast<std::size_t>(j) + 1].point - pts[static_cast<std::size_t>(j)].point;
      const double nj = math::norm(dj);
      if (nj <= 1e-14) continue;
      const SegClosest cc = segSegClosest(
          pts[static_cast<std::size_t>(i)].point, pts[static_cast<std::size_t>(i) + 1].point,
          pts[static_cast<std::size_t>(j)].point, pts[static_cast<std::size_t>(j) + 1].point);
      if (cc.dist > touch) continue;                 // in the loose candidate neighbourhood?
      // TRUE-CROSSING coincidence gate: the two passes must actually COINCIDE (closest approach ≤ a
      // small fraction of the step), not merely fall inside the loose neighbourhood window. A genuine
      // transverse self-crossing coincides to ≈ 0.2·h; a small CONVEX loop's near-by chords stay a
      // larger fraction of a step apart and would otherwise be FALSELY reported (a circle cannot cross
      // itself). Necessary-condition tightening — only REJECTS, never adds — so real figure-eight
      // crossings are unchanged and the guard OFF is byte-identical. (S4-f false-positive guard.)
      if (cc.dist > t.selfIntersectCoincDist) continue;
      const double cos = math::dot(di, dj) / (ni * nj);
      if (std::fabs(cos) >= kCrossCosMax) continue;  // (anti)parallel → retrace, not a crossing
      const Point3 X = cc.pOnA + (cc.pOnB - cc.pOnA) * 0.5;  // crossing = midpoint of closest approach
      bool dup = false;
      for (const auto& s : line.selfIntersections)
        if (math::distance(s.point, X) < mergeR) { dup = true; break; }
      if (dup) continue;
      const WLinePoint& e = pts[static_cast<std::size_t>(i)];
      const WLinePoint& l = pts[static_cast<std::size_t>(j)];
      SelfIntersection si;
      si.point = X;
      si.u1a = e.u1; si.v1a = e.v1; si.u1b = l.u1; si.v1b = l.v1;
      si.nodeA = i; si.nodeB = j;
      si.tangentCos = cos;
      line.selfIntersections.push_back(si);
    }
  }
  line.selfIntersectionCount = static_cast<int>(line.selfIntersections.size());
}

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
    line.chartSingularCrossed = f.chartCrossed;                // S4-e: poles/apexes stepped across
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
    line.chartSingularCrossed = f.chartCrossed + b.chartCrossed;  // S4-e: poles/apexes stepped across
    line.crossMaxResidual   = std::max(f.maxCrossResid, b.maxCrossResid);
    // Failed: neither direction advanced past the seed (no real curve). NearTangent: at
    // least one end stopped at a tangency (traced up to it — honest S4 gap). Otherwise
    // both ends left a domain boundary → a clean open branch.
    if (fwd.empty() && bwd.empty())
      line.status = TraceStatus::Failed;
    else if (f.end == DirEnd::NearTangent || b.end == DirEnd::NearTangent) {
      line.status = TraceStatus::NearTangent;
      line.stopReason = typeNearTangentStop(A, B, f, b, scale);  // S4-b: type the stop
      // Record WHICH end stalled: points = [reversed backward half] seed [forward half], so
      // points.front() is the backward-march terminus (b) and points.back() the forward (f).
      line.frontNearTangent = (b.end == DirEnd::NearTangent);
      line.backNearTangent = (f.end == DirEnd::NearTangent);
    } else
      line.status = TraceStatus::BoundaryExit;
  }

  for (const auto& n : line.points)
    line.onSurfResidual = std::max(line.onSurfResidual, n.onSurfResidual);

  // S4-f: scan the stitched arm for single-arm self-crossings (default off → no-op, so the
  // WLine is byte-identical to S3/S4-c/S4-d/S4-e). Recorded as DATA; the arm is unchanged.
  if (opts.enableSelfIntersection) detectStitchedSelfIntersections(line, t);

  // DENSIFY-AND-REFIT the convenience curve so it does not deviate from the on-locus polyline
  // when a high-curvature loop has more nodes than the pole cap (the fit-density decline the
  // freeform fuzzer sees: every node on both surfaces, but the under-resolved smoothed fit —
  // which downstream coverage is sampled from, not the polyline — reads over the curve-coverage
  // budget on a dense tight loop). Target the fit's node deviation at a fraction of model scale
  // that sits BELOW the downstream ~1e-3 on-curve budget yet ABOVE the fit error a smooth loop
  // already achieves at the default pole cap — so the (bounded, one-shot) refit fires ONLY on a
  // genuinely under-resolved high-curvature loop and is a NO-OP on the common smooth loop.
  // Raising poles only tightens the convenience curve toward the already-on-locus nodes, never
  // widens a tolerance or moves a node. The polyline stays the ground truth; deterministic; no
  // caller knob.
  const double fitTarget = scale * 2e-4;
  line.curve = fitBSpline(line.points, opts.fitDegree, opts.fitMaxPoles, fitTarget);
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
  res.singularitiesCrossed += w.chartSingularCrossed;  // S4-e: poles/apexes stepped across
  res.selfIntersections += w.selfIntersectionCount;    // S4-f: single-arm self-crossings recorded
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

// RECLASSIFY branch-terminated arcs. A NearTangent arc is a resolved arm of the self-crossing
// locus — NOT an unresolved S4 gap — when EVERY end that stalled at a near-tangency sits on a
// LOCALIZED branch point, and at least one end does. Convert it to BranchArc (a complete arc of
// the multi-arm locus) and take it out of `nearTangentGaps`.
//
// TWO topologies are covered by the same honesty rule:
//   * CLOSED NETWORK (Steinmetz): both ends are near-tangent stalls sitting on the TWO branch
//     points — a branch-to-branch arc. Drives Steinmetz to nearTangentGaps == 0 (unchanged).
//   * OPEN ARM (a general/freeform X-crossing on a FINITE patch, e.g. a B-spline saddle tangent
//     to a plane through its saddle point): ONE end is the near-tangent stall on the single
//     localized branch point, the OTHER end a clean domain-boundary exit. The four arms radiate
//     branch-to-boundary; each is a complete arm, not a residual gap.
//
// HONESTY GATE (why this never hides a real S4 gap). An end is UNRESOLVED when it stalled at a
// near-tangency (frontNearTangent / backNearTangent) that is NOT within mergeRadius of any
// localized branch point. If either end is unresolved, the arc is left as a NearTangent gap —
// only ends whose stall IS a genuine, arm-enumerated branch point (localizeBranchPoints already
// gated Δ>0: no arms ⇒ no BranchNode) are treated as resolved. A boundary/loop end (not a stall)
// is a clean terminus and never blocks reclassification.
void reclassifyBranchArcs(TraceSet& res, double mergeRadius) {
  auto atBranch = [&](const Point3& p) {
    for (const BranchNode& n : res.branchNodes)
      if (math::distance(n.point, p) <= mergeRadius) return true;
    return false;
  };
  for (WLine& w : res.lines) {
    if (w.status != TraceStatus::NearTangent || w.points.size() < 2) continue;
    const bool frontAtBr = atBranch(w.points.front().point);
    const bool backAtBr = atBranch(w.points.back().point);
    // A near-tangent END that is not on a branch point is a genuine, still-open S4 gap → defer.
    if ((w.frontNearTangent && !frontAtBr) || (w.backNearTangent && !backAtBr)) continue;
    if (!(frontAtBr || backAtBr)) continue;  // no branch end ⇒ not a branch arc
    w.status = TraceStatus::BranchArc;
    w.stopReason.reset();
    --res.nearTangentGaps;
    ++res.openCurves;
    ++res.tracedBranches;
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

namespace {

// March every seed of `seeds`, appending each genuinely-new (non-retracing) branch to `res`
// and collecting any S4-d branch stalls. Returns the number of NEW branches kept (a retrace
// or a Failed/too-short march adds none). Shared by the initial trace and the S4-f critic
// re-seed rounds so both dedup identically against ALL kept curves.
int marchSeedsInto(const SurfaceAdapter& A, const SurfaceAdapter& B, const SeedSet& seeds,
                   const MarchOptions& opts, double dedupRadius, TraceSet& res,
                   std::vector<BranchStall>& stalls, bool locusDedup = false,
                   double locusRadius = 0.0) {
  int added = 0;
  for (const Seed& s : seeds.seeds) {
    std::vector<BranchStall> seedStalls;
    WLine w = march_branch_impl(A, B, s, opts, seedStalls);
    // Capture branch stalls even when the WLine itself is deduped/failed — the branch point
    // is a property of the geometry, not of which seed reached it.
    if (opts.enableBranchPoints)
      stalls.insert(stalls.end(), seedStalls.begin(), seedStalls.end());
    if (w.points.size() < 2 || w.status == TraceStatus::Failed) continue;  // no real curve
    // The initial trace uses the tight node-proximity retrace test (byte-identical to before).
    // The S4-f critic re-seed uses a LOCUS test (point-to-polyline): a re-seed at a finer
    // resolution re-traces an already-found loop with DIFFERENT node sampling, so its probe
    // nodes sit BETWEEN the kept line's nodes — a node-proximity test would miss the duplicate
    // and OVER-PRODUCE, while the locus test correctly recognizes the same curve.
    const bool dup = locusDedup ? sameLocus(w, res.lines, locusRadius)
                                : retraces(w, res.lines, dedupRadius);
    if (dup) { ++res.dedupedRetraces; continue; }
    tallyLine(res, std::move(w));
    ++added;
  }
  return added;
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
  marchSeedsInto(A, B, seeds, opts, dedupRadius, res, stalls);

  // S4-d: localize the captured branch points and route their outgoing arms. Additive —
  // with enableBranchPoints off, `stalls` is empty and this is a no-op, so the transversal
  // and crossable-graze results above are byte-identical to before.
  if (opts.enableBranchPoints) routeBranches(A, B, opts, scale, stalls, res);

  return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// S4-f ADAPTIVE COMPLETENESS CRITIC — bounded loop-until-dry re-seed.
//
// After the initial fixed-resolution seed + trace, re-subdivide FINER (minPatchFrac *=
// criticRefineFactor per round) in the param regions NO traced curve covers, refine each
// new candidate at the SAME onSurfTol (a candidate that does not land on BOTH surfaces is
// DISCARDED by the seeder — never a fabricated seed), march it, and keep it only if it does
// NOT retrace an already-kept curve. Repeat until `criticDryRounds` (K) consecutive rounds
// find NO new branch, or the cost cap (criticMaxRounds / criticMaxCandidates) is hit.
//
// HONEST: this RAISES the recall floor (criticFloorFrac = the finest minPatchFrac reached);
// it is NOT a proof. Below the finest round a smaller loop can still be missed, so
// completenessResidual stays true. NEVER fabricates a seed or a loop.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

void runCompletenessCritic(const SurfaceAdapter& A, const SurfaceAdapter& B,
                           const SeedOptions& seedOpts, const MarchOptions& marchOpts,
                           double dedupRadius, TraceSet& res) {
  const double refine = seedOpts.criticRefineFactor > 0.0 && seedOpts.criticRefineFactor < 1.0
                            ? seedOpts.criticRefineFactor : 0.5;
  const int maxRounds = std::max(0, seedOpts.criticMaxRounds);
  const int dryK = std::max(1, seedOpts.criticDryRounds);
  const int candCap = std::max(0, seedOpts.criticMaxCandidates);

  double frac = seedOpts.minPatchFrac > 0 ? seedOpts.minPatchFrac : (1.0 / 32.0);
  int candTotal = 0;
  int dryStreak = 0;
  std::vector<BranchStall> stalls;  // critic marches keep branch points OFF (see below)

  // The critic re-seed marches with the SAME options but WITHOUT branch routing (a re-seed is
  // for RECOVERING a missed transversal loop, not re-running S4-d) so it never perturbs the
  // branch-point controls; self-intersection recording follows the caller's flag as-is.
  MarchOptions critOpts = marchOpts;
  critOpts.enableBranchPoints = false;

  for (int round = 0; round < maxRounds; ++round) {
    frac *= refine;                                   // finer resolution this round
    res.criticFloorFrac = frac;                       // report the floor reached (updated each round)

    // Where is nothing traced yet? A coarse coverage grid over A's domain (resolution tied to
    // the CURRENT frac so the grid tracks the re-seed scale). No uncovered cell ⇒ nothing to
    // look at at this grid resolution (NOT a completeness proof — see the header).
    const int gridN = std::max(2, static_cast<int>(std::lround(1.0 / std::max(frac, 1e-6))));
    const critic::Coverage cov = critic::coverageOf(res.lines, A.domain, gridN);
    const std::vector<ParamBox> uncovered = critic::uncoveredBoxes(cov);
    if (uncovered.empty()) { ++dryStreak; if (dryStreak >= dryK) { res.criticStoppedDry = true; break; } continue; }

    // Re-seed at the finer resolution. The seeder already prunes/dedups internally; we then
    // dedup the traced NEW branches vs ALL kept curves, so only genuinely new loops survive.
    SeedOptions ro = seedOpts;
    ro.completenessCritic = false;   // no recursion
    ro.criticTargetedReseed = false; // consumed here — the seeder itself never re-targets
    ro.minPatchFrac = frac;
    SeedSet rs;
    if (seedOpts.criticTargetedReseed) {
      // M1c TARGETED re-seed: seed ONLY the uncovered A-cells (A clamped to the cell) vs B's
      // full domain and accumulate. Recovers the second loop of a twice-piercing pose (merged
      // into one topological cluster at the coarse grid → one representative seed) without
      // re-subdividing the already-covered region. Never fabricates a seed (each cell's
      // candidate must still land on BOTH surfaces) and never widens a tolerance.
      SurfaceAdapter Asub = A;
      const int cellCap = std::max(1, seedOpts.criticMaxCells);
      int usedCells = 0;
      for (const ParamBox& cell : uncovered) {
        if (usedCells >= cellCap) break;
        Asub.domain = cell;
        SeedOptions rc = ro;
        rc.initialGridU = 1;  // the cell IS the pre-split; recurse within it
        rc.initialGridV = 1;
        const SeedSet part = seed_intersection(Asub, B, rc);
        for (const Seed& sd : part.seeds) rs.seeds.push_back(sd);
        rs.candidateRegions += part.candidateRegions;
        ++usedCells;
      }
    } else {
      rs = seed_intersection(A, B, ro);
    }
    candTotal += rs.candidateRegions;

    // Dedup a re-seed's traced branch by LOCUS (point-to-polyline) at a step-scaled radius, so
    // a finer re-trace of an already-found loop is recognized as a duplicate (not over-produced).
    const double scale = std::max(A.modelScale, B.modelScale);
    const double h0 = marchOpts.initialStep > 0 ? marchOpts.initialStep : scale * (1.0 / 64.0);
    const double locusRadius = std::max(dedupRadius, 2.0 * h0);
    const int added = marchSeedsInto(A, B, rs, critOpts, dedupRadius, res, stalls,
                                     /*locusDedup=*/true, locusRadius);
    res.criticRecoveredLoops += added;
    ++res.criticRounds;

    if (added > 0) dryStreak = 0;
    else { ++dryStreak; if (dryStreak >= dryK) { res.criticStoppedDry = true; break; } }

    if (candCap > 0 && candTotal >= candCap) break;  // cost cap (not dry) → stop, residual stands
  }
}

}  // namespace

TraceSet trace_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                            const SeedOptions& seedOpts, const MarchOptions& marchOpts) {
  const SeedSet seeds = seed_intersection(A, B, seedOpts);
  TraceSet res = trace_from_seeds(A, B, seeds, marchOpts);

  // S4-f: the adaptive completeness critic (default OFF → byte-identical to the fixed-
  // resolution trace above). It recovers small loops the fixed floor missed, loop-until-dry,
  // and reports the recall floor + residual — never a fabricated branch.
  if (seedOpts.completenessCritic) {
    const double scale = std::max(A.modelScale, B.modelScale);
    const double dedupRadius = (marchOpts.dedupFrac > 0 ? marchOpts.dedupFrac : 1e-4) * scale;
    runCompletenessCritic(A, B, seedOpts, marchOpts, dedupRadius, res);
  }
  return res;
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_HAS_NUMSCI
