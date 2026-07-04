// SPDX-License-Identifier: Apache-2.0
//
// marching.h — SSI Stage S3: predictor-corrector marching-line tracer (WLine).
//
// S3 turns the S2 SeedSet (one point per transversal intersection branch) into the
// FULL intersection curves. From each seed it WALKS the intersection of two native
// surfaces A, B — a curve where A.point(u1,v1) = B.point(u2,v2) — producing a
// polyline of on-both-surfaces points (a "WLine", the walking-line of OCCT's
// IntWalk/IntPatch) and a fitted B-spline through it.
//
// METHOD (clean-room, SSI-ROADMAP S3; OCCT IntWalk_PWalking / IntPatch WLine as the
// reference ORACLE ONLY — the deflection-driven step, the two-surface Newton
// corrector, and the loop/boundary termination are the same *scheme*, re-derived, not
// copied):
//
//   PREDICTOR. At the current on-curve point P with surface params (u1,v1,u2,v2), the
//     intersection tangent is t = normalize(nA × nB) (both surface normals; their
//     cross product is tangent to BOTH surfaces, hence to the intersection curve).
//     Oriented by the march direction, the next guess is P_guess = P + h·t for the
//     current step h. The param guesses on each surface are advanced through the
//     surface's own tangent plane (solve the 2×2 [dU dV]·(Δu,Δv)=h·t for each side),
//     so the corrector starts close.
//
//   CORRECTOR. Re-project P_guess back onto BOTH surfaces at once: native-numerics
//     least_squares drives r(u1,v1,u2,v2) = A.point − B.point → 0, seeded at the
//     predicted params and CLAMPED to both param boxes, landing exactly on the
//     intersection. The extra along-curve DOF (m=3 residuals, n=4 params) is pinned by
//     an added residual that holds the step to ≈ h along t, so the corrector advances
//     rather than sliding back to the previous point. A converged, on-both-surfaces,
//     TRANSVERSAL result is the next curve point.
//
//   ADAPTIVE STEP (deflection control, cf. IntWalk_StatusDeflection). After each
//     accepted step we measure the chord/arc DEFLECTION (how far the true midpoint
//     bows off the P→P_next chord) and the corrector strain (iterations / residual).
//     Too much deflection or a strained corrector ⇒ SHRINK h and retry; a smooth,
//     cheap step ⇒ GROW h (bounded). This is the curvature-adaptive step the roadmap
//     calls for.
//
//   TERMINATION. Stop when the march (a) CLOSES — comes back within a proximity of the
//     seed moving in a consistent direction ⇒ Closed loop; (b) EXITS a BOUNDARY — a
//     clamped param lands on a domain edge ⇒ BoundaryExit (an open curve ending on that
//     boundary); or (c) hits a NEAR-TANGENT / degenerate region — ‖nA × nB‖ → 0 or the
//     corrector diverges ⇒ STOP and flag NearTangent (an honest S4 gap). We NEVER force
//     a bogus point past a tangency. Open curves march BOTH directions from the seed
//     and are stitched (reverse of the backward half + forward half).
//
// HONEST SCOPE (S3 = TRANSVERSAL). Near-tangent / coincident / degenerate marching,
// branch-point splitting and self-intersection resolution are DEFERRED to S4. If a
// march enters a near-tangent region it traces UP TO it and reports the remainder as a
// NearTangent gap — it does not fabricate curve points or claim a full trace that
// stopped short. Coverage is reported per WLine (Closed / BoundaryExit / NearTangent),
// not asserted complete.
//
// SSI is INTERNAL — no cc_* entry point, no ABI change. Verified at the
// cybercad::native::ssi C++ boundary (host: sampled WLine points on both surfaces ≤
// tol, loop/open classification; sim: length/shape vs OCCT IntPatch on non-tangent
// freeform pairs).
//
// SUBSTRATE GUARD. The corrector (least_squares) and the B-spline fit (lstsq) are
// native-numerics, so the marching ENTRY POINTS are compiled only under
// CYBERCAD_HAS_NUMSCI (the declarations stay visible, like seeding.h). Uses
// src/native/math + src/native/numerics + the S2 SeedSet. clang++ -std=c++20, fp64,
// deterministic.
//
#ifndef CYBERCAD_NATIVE_SSI_MARCHING_H
#define CYBERCAD_NATIVE_SSI_MARCHING_H

#include "native/math/vec.h"
#include "native/ssi/patch_bounds.h"  // SurfaceAdapter (point/normal/domain/period)
#include "native/ssi/seed.h"          // Seed / SeedSet (S3 input contract)
#include "native/ssi/seeding.h"       // SeedOptions (reused for the seeding pass)
#include "native/ssi/tangent_contact.h"  // TangentContact (S4-b: typed stop reason)

#include <optional>
#include <vector>

namespace cybercad::native::ssi {

// ─────────────────────────────────────────────────────────────────────────────
// WLinePoint — one traced point on the intersection curve, with its params on BOTH
// surfaces (the walking-line node OCCT stores as IntSurf_PntOn2S).
// ─────────────────────────────────────────────────────────────────────────────
struct WLinePoint {
  double u1 = 0.0, v1 = 0.0;   ///< params on surface A
  double u2 = 0.0, v2 = 0.0;   ///< params on surface B
  math::Point3 point{};        ///< A.point(u1,v1) ≈ B.point(u2,v2)
  double onSurfResidual = 0.0; ///< ‖A.point − B.point‖ at this node (≤ tol)
};

/// How a traced WLine ended — the honest per-curve coverage report (design.md
/// TraceStatus). A well-formed transversal branch is Closed or BoundaryExit; a march
/// that ran into a tangency is NearTangent (traced up to it, remainder an S4 gap); a
/// seed the corrector could not advance from at all is Failed (reported, no curve).
enum class TraceStatus {
  Closed,        ///< the march returned to the seed → a closed loop
  BoundaryExit,  ///< the curve runs boundary-to-boundary (both ends on a domain edge)
  NearTangent,   ///< stopped at a near-tangent / divergent region → S4 gap (up-to-here only)
  Failed,        ///< corrector could not advance from the seed at all (no curve emitted)
};

// ─────────────────────────────────────────────────────────────────────────────
// FittedBSpline — the B-spline fitted through a WLine polyline (native-math
// representation: non-rational, clamped uniform knots). Evaluate with
// math::curvePoint(degree, poles, knots, t) over [knots.front(), knots.back()].
// Empty (degree 0, no poles) when the polyline was too short to fit; the polyline is
// always the ground truth and is retained regardless.
// ─────────────────────────────────────────────────────────────────────────────
struct FittedBSpline {
  int degree = 0;
  std::vector<math::Point3> poles{};
  std::vector<double> knots{};  ///< flat, clamped; length = poles.size() + degree + 1
  double maxFitError = 0.0;     ///< max ‖polyline sample − curve‖ over the fit samples

  bool valid() const noexcept { return degree >= 1 && poles.size() >= 2; }
};

// ─────────────────────────────────────────────────────────────────────────────
// WLine — one traced intersection branch: the polyline, its status, and the fitted
// B-spline. `branchId` echoes the seed's branch id. A NearTangent WLine carries the
// points traced UP TO the tangency (never fabricated past it) — the remainder is an
// S4 gap.
// ─────────────────────────────────────────────────────────────────────────────
struct WLine {
  std::vector<WLinePoint> points{};   ///< corrected march nodes in order (design's `polyline`, + params)
  TraceStatus status = TraceStatus::Failed;
  FittedBSpline curve{};              ///< B-spline fitted through the polyline (Geom-quality)
  int branchId = 0;                   ///< echoes the S2 seed branch id
  double onSurfResidual = 0.0;        ///< max ‖node − surface‖ over BOTH surfaces (≤ tol)

  /// S4-b: WHY the march stopped, TYPED, when it stopped at a tangency
  /// (`status == NearTangent`). Carries the classified `TangentContact` at the stop
  /// point (TangentPoint / TangentCurve / NearTangentTransversal / Undecided) so the
  /// caller knows the SHAPE of the degeneracy the tracer ran into, not just that it did.
  /// The tracer STILL STOPS at the tangency — this field types the stop, it does NOT
  /// mean the march stepped through. Absent (nullopt) for Closed / BoundaryExit / Failed.
  std::optional<TangentContact> stopReason{};

  bool isClosed() const noexcept { return status == TraceStatus::Closed; }
  bool truncated() const noexcept { return status == TraceStatus::NearTangent; }
};

// ─────────────────────────────────────────────────────────────────────────────
// MarchOptions — the predictor/corrector/step/termination knobs. Sentinel (≤ 0)
// values are resolved from the operands' model scale at call time.
// ─────────────────────────────────────────────────────────────────────────────
struct MarchOptions {
  double initialStep = -1.0;   ///< first predictor step h (≤ 0 → scale · 1/64)
  double minStep = -1.0;       ///< smallest h before declaring the corrector stuck (≤ 0 → scale · 1e-6)
  double maxStep = -1.0;       ///< largest h a grow may reach (≤ 0 → scale · 1/8)
  double onSurfTol = -1.0;     ///< corrector accept residual ‖A−B‖ (≤ 0 → scale · 1e-7)
  double maxDeflection = -1.0; ///< max chord/arc bow per step before shrinking (≤ 0 → scale · 1e-3)
  double tangentSinTol = 1e-3; ///< ‖nA×nB‖ below this ⇒ near-tangent → truncate (S4)
  double loopCloseFrac = 2.0;  ///< loop-closure proximity = loopCloseFrac · current step
  int maxPoints = 20000;       ///< hard cap on nodes per direction (termination safety)
  int fitDegree = 3;           ///< B-spline fit degree (cubic default)
  int fitMaxPoles = 64;        ///< max poles the least-squares fit uses (0 → interpolate every node)
  double dedupFrac = -1.0;     ///< two WLines duplicate if a node is within dedupFrac·scale (≤ 0 → 1e-4)
};

/// The full S3 result (design.md TraceSet): one WLine per distinct traced branch plus
/// the honest diagnostics. `TraceSet` is the S5 curved-boolean contract — each WLine
/// splits a curved face; `nearTangentGaps > 0` is the honest S4 signal (a branch traced
/// up to a tangency, remainder deferred).
struct TraceSet {
  std::vector<WLine> lines{};   ///< one WLine per distinct traced transversal branch

  int tracedBranches = 0;       ///< WLines produced (Closed | BoundaryExit)
  int nearTangentGaps = 0;      ///< marches stopped at a near-tangent region → S4 (reported)
  int dedupedRetraces = 0;      ///< seeds whose march retraced an already-traced branch

  // Extra diagnostics (kept for verification/reporting; not in the minimal contract).
  int seededBranches = 0;       ///< branches the S2 seeder handed us (SeedSet.branchCount)
  int deferredTangent = 0;      ///< S2 near-tangent branches never seeded (echoed S4 gap)
  int closedCurves = 0;         ///< WLines that closed into a loop
  int openCurves = 0;           ///< WLines running boundary-to-boundary

  int curveCount() const noexcept { return static_cast<int>(lines.size()); }
};

// ─────────────────────────────────────────────────────────────────────────────
// march_branch — trace ONE intersection branch from a seed (both directions).
//
// Runs the predictor-corrector loop forward and backward from `seed`, stitches the
// two halves, classifies the result (Closed / Open / NearTangentTruncated) and fits a
// B-spline through the polyline. The returned WLine is honest: a truncated march
// carries only the points reached before the tangency/divergence.
//
// GUARD: the DECLARATIONS below are always visible (like seed_intersection), but the
// DEFINITIONS (marching.cpp) are compiled only under CYBERCAD_HAS_NUMSCI (the
// corrector calls least_squares, the fit calls lstsq). A TU that links a kernel built
// without the substrate will not resolve these symbols — by design.
// ─────────────────────────────────────────────────────────────────────────────
WLine march_branch(const SurfaceAdapter& A, const SurfaceAdapter& B, const Seed& seed,
                   const MarchOptions& opts = {});

// ─────────────────────────────────────────────────────────────────────────────
// trace_intersection — the S3 entry point: seed (S2) → march each seed → dedup
// retraced branches → fit each WLine. Returns the curves + per-curve status counts.
//
// `seedOpts` is forwarded to the S2 seeder; `marchOpts` controls the tracer.
// ─────────────────────────────────────────────────────────────────────────────
TraceSet trace_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                               const SeedOptions& seedOpts = {},
                               const MarchOptions& marchOpts = {});

/// Trace directly from an already-computed SeedSet (skips the S2 pass — for callers
/// that seeded separately or want to reuse a SeedSet). Same dedup + fit as above.
TraceSet trace_from_seeds(const SurfaceAdapter& A, const SurfaceAdapter& B,
                             const SeedSet& seeds, const MarchOptions& opts = {});

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_MARCHING_H
