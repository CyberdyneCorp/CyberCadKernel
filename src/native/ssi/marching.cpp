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
  int maxPoints;
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
  t.maxPoints     = std::max(16, o.maxPoints);
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

DirResult marchDir(const SurfaceAdapter& A, const SurfaceAdapter& B,
                  const State& start, double sign, const Tuned& t,
                  std::vector<WLinePoint>& out) {
  State cur = start;
  double h = t.h0;
  DirResult res;
  for (int step = 0; step < t.maxPoints; ++step) {
    const Tangent tan = intersectionTangent(A, B, cur);
    if (!tan.valid || tan.sine < t.tangentSinTol) {  // near-tangent → S4 stop
      res.end = DirEnd::NearTangent; res.sineAtStop = tan.sine; return res;
    }
    const Vec3 dir = tan.dir * sign;

    bool ok = false;
    const CorrectorOut c = tryStep(A, B, cur, dir, t, h, ok);
    if (!ok) {  // corrector could not take even a minStep transversally → S4 stop
      res.end = DirEnd::NearTangent; res.sineAtStop = intersectionTangent(A, B, cur).sine;
      return res;
    }

    // Loop closure: back near the seed after real progress, moving consistently.
    if (step > 2 && math::distance(c.s.p, start.p) <= t.loopClose * h) {
      out.push_back(toNode(c.s, c.resid));
      res.end = DirEnd::LoopClosed; res.sineAtStop = tan.sine; return res;
    }

    out.push_back(toNode(c.s, c.resid));
    cur = c.s;

    // Boundary exit (non-periodic edge reached).
    if (onBoundary(A, cur.u1, cur.v1, A.uPeriod > 0.0, A.vPeriod > 0.0) ||
        onBoundary(B, cur.u2, cur.v2, B.uPeriod > 0.0, B.vPeriod > 0.0)) {
      res.end = DirEnd::Boundary; res.sineAtStop = tan.sine; return res;
    }

    // Smooth + cheap step → grow h (bounded). Cheap ≈ few corrector evals.
    if (c.nfev <= 40) h = std::min(h * 1.5, t.maxStep);
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

// ─────────────────────────────────────────────────────────────────────────────
// march_branch — forward + backward from the seed, stitch, classify, fit.
// ─────────────────────────────────────────────────────────────────────────────
WLine march_branch(const SurfaceAdapter& A, const SurfaceAdapter& B, const Seed& seed,
                   const MarchOptions& opts) {
  const double scale = std::max(A.modelScale, B.modelScale);
  const Tuned t = tune(opts, scale);
  const State seedState{seed.u1, seed.v1, seed.u2, seed.v2, seed.point};

  std::vector<WLinePoint> fwd, bwd;
  const DirResult f = marchDir(A, B, seedState, +1.0, t, fwd);

  WLine line;
  line.branchId = seed.branchId;

  if (f.end == DirEnd::LoopClosed) {
    // A closed loop: forward march wrapped back to the seed. Points = seed + forward.
    line.points.push_back(toNode(seedState, seed.onSurfResidual));
    line.points.insert(line.points.end(), fwd.begin(), fwd.end());
    line.status = TraceStatus::Closed;
  } else {
    // Open (or truncated forward). March the other way too and stitch:
    //   [reversed backward half]  seed  [forward half].
    const DirResult b = marchDir(A, B, seedState, -1.0, t, bwd);
    for (auto it = bwd.rbegin(); it != bwd.rend(); ++it) line.points.push_back(*it);
    line.points.push_back(toNode(seedState, seed.onSurfResidual));
    line.points.insert(line.points.end(), fwd.begin(), fwd.end());
    // Failed: neither direction advanced past the seed (no real curve). NearTangent: at
    // least one end stopped at a tangency (traced up to it — honest S4 gap). Otherwise
    // both ends left a domain boundary → a clean open branch.
    if (fwd.empty() && bwd.empty())
      line.status = TraceStatus::Failed;
    else if (f.end == DirEnd::NearTangent || b.end == DirEnd::NearTangent)
      line.status = TraceStatus::NearTangent;
    else
      line.status = TraceStatus::BoundaryExit;
  }

  for (const auto& n : line.points)
    line.onSurfResidual = std::max(line.onSurfResidual, n.onSurfResidual);
  line.curve = fitBSpline(line.points, opts.fitDegree, opts.fitMaxPoles);
  return line;
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

}  // namespace

TraceSet trace_from_seeds(const SurfaceAdapter& A, const SurfaceAdapter& B,
                          const SeedSet& seeds, const MarchOptions& opts) {
  TraceSet res;
  res.seededBranches = seeds.branchCount();
  res.deferredTangent = seeds.deferredTangent;
  const double scale = std::max(A.modelScale, B.modelScale);
  const double dedupRadius = (opts.dedupFrac > 0 ? opts.dedupFrac : 1e-4) * scale;

  for (const Seed& s : seeds.seeds) {
    WLine w = march_branch(A, B, s, opts);
    if (w.points.size() < 2 || w.status == TraceStatus::Failed) continue;  // no real curve
    if (retraces(w, res.lines, dedupRadius)) { ++res.dedupedRetraces; continue; }
    switch (w.status) {
      case TraceStatus::Closed:       ++res.closedCurves; ++res.tracedBranches; break;
      case TraceStatus::BoundaryExit: ++res.openCurves;   ++res.tracedBranches; break;
      case TraceStatus::NearTangent:  ++res.nearTangentGaps; break;
      case TraceStatus::Failed:       break;  // filtered above
    }
    res.lines.push_back(std::move(w));
  }
  return res;
}

TraceSet trace_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                            const SeedOptions& seedOpts, const MarchOptions& marchOpts) {
  const SeedSet seeds = seed_intersection(A, B, seedOpts);
  return trace_from_seeds(A, B, seeds, marchOpts);
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_HAS_NUMSCI
