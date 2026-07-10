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
// HONEST SCOPE (S3 = TRANSVERSAL; S4-c = near-tangent MARCHING, first slice). A pure
// transversal march is bit-identical to S3. When a march reaches a near-tangent region
// (‖nA×nB‖ < tangentSinTol) the S4-c logic (CYBERCAD_HAS_NUMSCI) tries to MARCH THROUGH
// it — but only when it is genuinely CROSSABLE:
//
//   FIXED-PLANE-CUT CORRECTOR. The S3 corrector pins the step with an ALONG-t advance
//     residual r₃ = dot(A.point−Pprev, t) − h, where t = normalize(nA×nB) ILL-CONDITIONS
//     as sine→0 — the root cause of the near-tangent stop. Inside the crossing band S4-c
//     replaces t in that residual with the LAST-GOOD (pre-band) tangent t★: a hyperplane
//     perpendicular to t★ at arc-distance h that the curve crosses transversally even
//     where the local surface tangent degenerates, so the least_squares solve stays
//     well-posed. Outside the band the corrector is the S3 along-local-t solve, unchanged.
//
//   CROSSABLE GATE (the honesty core). A crossing is ATTEMPTED only when S4-b
//     classify_tangent_contact_seeded types the stall NearTangentTransversal AND it is a
//     genuine SINGLE-branch graze, decided by two witnesses: the transversality sine must
//     NOT steeply collapse into the band (a stall sine < ¼ of the last-good sine signals a
//     tangency/branch driving sine→0), and a FINE look-ahead scan's minimum sine must stay
//     ≥ a fraction of the enter threshold (a measure-zero dip a coarse step would leap is
//     resolved). Per crossing step the corrected node must stay on both surfaces, advance
//     monotonically along t★, keep sine above the floor, and keep its raw tangent aligned
//     by continuity (no ≥60° turn / reversal — a flip means two branches meet, S4-d). A
//     genuine tangency (TangentPoint / TangentCurve), an Undecided jet, a branch crossing,
//     non-convergence at minStep, or any failed verification ⇒ the arc is DISCARDED and the
//     march STILL STOPS + classifies + defers (→ OCCT). No point is ever fabricated past a
//     degeneracy; a crossed arc is emitted only if every node verified on both surfaces
//     ≤ onSurfTol.
//
// Branch-point splitting (S4-d), surface-singularity (S4-e), self-intersection (S4-f)
// and whole-tangent-seam / coincident-region marching stay DEFERRED. Coverage is
// reported per WLine (Closed / BoundaryExit / NearTangent) with nearTangentCrossed for a
// crossed graze; a region not robustly crossable is still an honest NearTangent gap.
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

// ─────────────────────────────────────────────────────────────────────────────
// SelfIntersection (S4-f) — a point where a SINGLE traced arm crosses ITSELF: the
// curve returns to the neighbourhood of an EARLIER, non-adjacent node with a TRANSVERSE
// tangent (the two passes head in materially different directions — |dot| well below 1).
// This is a figure-eight-style self-crossing of ONE branch — DISTINCT from an S4-d
// BranchNode (where the intersection LOCUS itself branches: two DISTINCT arms meet,
// ‖nA×nB‖ → 0), and DISTINCT from a retrace (an arm running back over itself, dot ≈ ±1,
// which is a dedup/periodic-seam artifact, not reported). Reported as DATA — the arm is
// NOT stopped and NOT closed at the crossing; it marches through. `nodeA`/`nodeB` index
// the two WLine.points that coincide (the earlier and the later pass).
//
// HONEST SCOPE: S4-f DETECTS + REPORTS + traces THROUGH a self-crossing. It does NOT
// split the arm into sub-arcs or repair topology (that is S5/S6 assembler work). A false
// positive is a spurious COUNT, never a wrong curve (the polyline is unchanged).
// ─────────────────────────────────────────────────────────────────────────────
struct SelfIntersection {
  math::Point3 point{};        ///< the crossing point (≈ both passes' node position)
  double u1a = 0.0, v1a = 0.0; ///< params on A at the EARLIER pass (nodeA)
  double u1b = 0.0, v1b = 0.0; ///< params on A at the LATER pass (nodeB)
  int nodeA = 0;               ///< index of the earlier WLine.points node
  int nodeB = 0;               ///< index of the later WLine.points node
  double tangentCos = 0.0;     ///< dot(unit tangent at nodeA, unit tangent at nodeB) (|·| ≪ 1 ⇒ transverse)
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
  BranchArc,     ///< S4-d: a complete arc of a self-crossing locus, running branch-to-branch
                 ///< (both ends meet LOCALIZED branch points). A resolved junction, NOT an S4
                 ///< gap — the arcs meeting at each branch point are recorded in TraceSet.branchNodes.
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

  /// S4-c: how many NEAR-TANGENT TRANSVERSAL grazes this branch MARCHED THROUGH (a
  /// crossable graze recognized `NearTangentTransversal` by S4-b and verified single-
  /// branch — see marching.cpp). 0 for a pure S3 transversal trace. A branch with
  /// `nearTangentCrossed > 0` that is `Closed`/`BoundaryExit` is a FULL curve the S3
  /// marcher would have truncated. `crossMaxResidual` is the worst on-both-surfaces
  /// residual over the crossed arc(s) (≤ onSurfTol — a crossing is accepted only if
  /// every node verifies), an honest witness the crossing stayed on both surfaces.
  int nearTangentCrossed = 0;
  double crossMaxResidual = 0.0;

  /// S4-e: how many CHART SINGULARITIES (sphere parametric poles v=±π/2 / cone apexes,
  /// signed radius = 0) this branch STEPPED ACROSS with the point-based corrector — a
  /// single-surface parametrization degeneracy (‖dU‖ → 0 with a finite point + normal),
  /// DISTINCT from the S4-c pair graze and the S4-d locus branch. 0 for a pure S3 / S4-c /
  /// S4-d trace. Every crossed node verified on both surfaces ≤ onSurfTol (see marching.cpp
  /// crossChartSingularity); a branch with `chartSingularCrossed > 0` that is
  /// `Closed`/`BoundaryExit` is a FULL curve the S3 marcher would have truncated at the
  /// pole/apex. A chart singularity that could NOT be crossed is an honest NearTangent gap.
  int chartSingularCrossed = 0;

  /// S4-b: WHY the march stopped, TYPED, when it stopped at a tangency
  /// (`status == NearTangent`). Carries the classified `TangentContact` at the stop
  /// point (TangentPoint / TangentCurve / NearTangentTransversal / Undecided) so the
  /// caller knows the SHAPE of the degeneracy the tracer ran into, not just that it did.
  /// The tracer STILL STOPS at the tangency — this field types the stop, it does NOT
  /// mean the march stepped through. Absent (nullopt) for Closed / BoundaryExit / Failed.
  std::optional<TangentContact> stopReason{};

  /// S4-f: SELF-INTERSECTIONS this arm crossed THROUGH (a single arm returning to an
  /// earlier non-adjacent node with a TRANSVERSE tangent — a figure-eight self-crossing).
  /// Detected + recorded ONLY when MarchOptions.enableSelfIntersection is on (off → this
  /// stays empty and the trace is byte-identical to the S3/S4-c/S4-d/S4-e behaviour). Each
  /// entry is a typed crossing, verified on both surfaces ≤ onSurfTol; the arm was NOT
  /// stopped or closed at the crossing (it is reported as DATA, never a fabricated closure).
  /// DISTINCT from an S4-d BranchNode (`branchPoints`): a self-crossing does NOT flip the
  /// locus, so a WLine with `selfIntersectionCount > 0` has NO associated branch point.
  int selfIntersectionCount = 0;
  std::vector<SelfIntersection> selfIntersections{};

  /// S4-d (OPEN-ARM ASSEMBLY): whether the FRONT / BACK end of `points` stopped at a
  /// near-tangency (an S4 stall) as opposed to a clean domain-boundary exit or loop close.
  /// `points.front()` is the backward-march terminus, `points.back()` the forward-march
  /// terminus (see marching.cpp assembly). Only meaningful when `status == NearTangent`
  /// (at least one is true then); both false for Closed / BoundaryExit. Used by
  /// reclassifyBranchArcs to tell a resolved open branch arm (its near-tangent end sits on
  /// a LOCALIZED branch point, its other end a genuine boundary) from an unresolved S4 gap
  /// (a near-tangent end NOT on any branch point) — so an X-crossing on a freeform patch,
  /// whose four arms each run branch-to-boundary, is reported as resolved BranchArcs rather
  /// than four residual nearTangentGaps. Additive: default false ⇒ every non-branch trace
  /// (S3 / S4-c / S4-e / S4-f) is byte-identical.
  bool frontNearTangent = false;
  bool backNearTangent = false;

  bool isClosed() const noexcept { return status == TraceStatus::Closed; }
  bool truncated() const noexcept { return status == TraceStatus::NearTangent; }
};

// ─────────────────────────────────────────────────────────────────────────────
// BranchNode (S4-d) — a localized branch point where the intersection LOCUS self-crosses
// (multiple real curve arms meet at one point). `point` is the branch point B on BOTH
// surfaces; `branchSine` = ‖nA×nB‖ there (≈ 0 — the transversality that collapsed);
// `armLineIds` are the branchId's of the WLines whose ends meet at B (the connected arms).
// A BranchNode is emitted ONLY for a GENUINE transversal self-crossing (the tangent-cone
// quadratic had two distinct real roots ⇒ real outgoing arms) — never for an isolated
// TangentPoint (definite second form ⇒ no arms ⇒ the curve ends).
// ─────────────────────────────────────────────────────────────────────────────
struct BranchNode {
  math::Point3 point{};        ///< the branch point B (on both surfaces)
  double branchSine = 0.0;     ///< ‖nA×nB‖ at B (≈ 0)
  double onSurfResidual = 0.0; ///< ‖A.point − B.point‖ at B (≤ onSurfTol)
  std::vector<int> armLineIds{}; ///< branchId of each WLine arm meeting at B
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
  double tangentSinTol = 1e-3; ///< ‖nA×nB‖ below this ⇒ near-tangent → truncate/cross (S4)
  double loopCloseFrac = 2.0;  ///< loop-closure proximity = loopCloseFrac · current step

  // ── S4-f ROBUST CLOSURE + SELF-INTERSECTION (completeness / loop-robustness slice) ──
  //
  // TRUE-RETURN CLOSURE is ALWAYS ON and is a NECESSARY-CONDITION tightening of the S3
  // proximity close: a loop now closes only when the march returns near the seed AND its
  // heading is TANGENT-CONTINUOUS with the seed's outgoing tangent (it comes back the way
  // it left). This can only REFUSE a false-close (a curve passing near the seed / an
  // earlier node while heading the OTHER way) — it can never MAKE a close — so a
  // truly-closing curve (every transversal control) is byte-identical. `closureTangentCos`
  // sets how ANTIPARALLEL a return heading must be to BLOCK the close: dot(fwdNow, seedFwd)
  // below it refuses the stop. ≤ 0 → −0.5 (a generous block threshold: only headings turned
  // more than 120° from the seed's outgoing tangent are refused, so a genuine loop — whose
  // return heading points roughly the same way it left — always closes).
  double closureTangentCos = -1.0;  ///< close only if dot(fwdNow,seedFwd) ≥ this (≤ 0 → −0.5)

  // SELF-INTERSECTION GUARD (default OFF → byte-identical S3/S4-c/S4-d/S4-e). When on, a
  // crossing of an EARLIER non-seed, non-adjacent node of THIS arm with a TRANSVERSE tangent
  // is recorded as a typed WLine.selfIntersection (data), and the arm CONTINUES through it
  // (never stopped, never closed). `selfIntersectRadiusFrac` scales the coincidence radius
  // (≤ 0 → 2·loopClose, so the self-cross window matches the loop-closure window).
  bool enableSelfIntersection = false;
  double selfIntersectRadiusFrac = -1.0;  ///< self-cross coincidence radius = this·h (≤ 0 → 2·loopClose·h)

  // ── S4-c near-tangent MARCHING band (all tangentSinTol-derived; no tolerance is
  // weakened — these only decide WHERE the fixed-plane crossing corrector engages) ──
  double bandEnterSin = -1.0;  ///< sine below this ⇒ enter the crossing band (≤ 0 → tangentSinTol)
  double bandExitSin  = -1.0;  ///< sine above this on the far side ⇒ band exit, resume S3 (≤ 0 → 1.5·tangentSinTol)
  int    crossMaxSteps = 256;  ///< max fixed-plane steps spent crossing one graze before deferring

  // ── S4-c DEEP near-tangent breadth (M1d, ADDITIVE, default OFF → byte-identical) ──
  //
  // The shipped S4-c crossing freezes ONE reference tangent t★ (the last-good pre-band
  // tangent) as BOTH the crossability anchor AND the fixed-plane advance direction for
  // the whole band. On a TIGHTER graze (transversality sine dipping ≈ 0.10–0.17, below
  // the ≈ 0.17 the frozen plane still resolves) the intersection curve TURNS materially
  // through the pinch, so the frozen advance plane slices the curve far from the guess
  // and the two-surface corrector fails to converge (`c.ok == false`) — an honest defer,
  // but a RESOLVABLE one: the graze is a genuine single transversal branch (band-min
  // sine well above the 0.3·enter floor, no steep collapse).
  //
  // `adaptiveCrossReanchor` (default OFF) lets the crossing corrector RE-ANCHOR its
  // fixed-plane advance direction toward the LOCAL intersection tangent (normalize(nA×nB),
  // continuity-oriented) as it steps — a higher-order (curve-following) plane that stays
  // transverse to the turning curve, so the corrector keeps landing on both surfaces deeper
  // into the band (a raw node-secant was tried first and rejected as too noisy at the fine
  // crossing step). The HONESTY anchors are UNCHANGED: crossability is still decided against
  // the frozen t★ (`crossNodeCrossable` sine floor + per-step branch-flip guard, band-min
  // floor, steep-collapse witness), the re-anchored direction must still head the crossing
  // way (`dot(t_local, t★) > 0`) or the frozen t★ is kept, every node is still verified on
  // BOTH surfaces at the SAME onSurfTol, and a genuine tangency / branch (sine → 0) still
  // defers. It only follows the curve's turn — it never widens a tolerance and never
  // fabricates a point.
  bool   adaptiveCrossReanchor = false;  ///< S4-c deep tail: re-anchor the crossing plane to the local curve tangent
  double reanchorBlend = -1.0;           ///< blend weight local-tangent↔t★ per step (≤ 0 → 1.0 = full local tangent); clamped [0,1]
  int maxPoints = 20000;       ///< hard cap on nodes per direction (termination safety)
  int fitDegree = 3;           ///< B-spline fit degree (cubic default)
  int fitMaxPoles = 64;        ///< max poles the least-squares fit uses (0 → interpolate every node)
  double dedupFrac = -1.0;     ///< two WLines duplicate if a node is within dedupFrac·scale (≤ 0 → 1e-4)

  // ── S4-d BRANCH POINTS (branch-point localization + arm routing). OFF by default so
  // every S3 transversal trace and every S4-c crossable-graze trace is BYTE-IDENTICAL to
  // before; a caller opts in to route the arms of a genuine self-crossing (Steinmetz). ──
  bool enableBranchPoints = false;  ///< S4-d: localize branch points + route the outgoing arms
  double branchMergeFrac = -1.0;    ///< arms meeting within branchMergeFrac·scale of one B share a node (≤ 0 → 1e-3)

  // ── S4-e CHART SINGULARITIES (sphere parametric pole / cone apex crossing). OFF by
  // default so every S3 transversal trace, every S4-c crossable-graze trace, and every S4-d
  // branch trace is BYTE-IDENTICAL to before; a caller opts in to STEP ACROSS a single-
  // surface parametrization degeneracy (‖dU‖ → 0 with a finite point + normal). The witness
  // is the single-surface Jacobian rank-drop — DISTINCT from the S4-c pair sine and the S4-d
  // locus flip (chart_singularity.h). Neither knob weakens a tolerance; they only decide the
  // chart-collapse threshold and the fine crossing step. ──
  bool enableChartSingularities = false;  ///< S4-e master switch (off → S3/S4-c/S4-d behaviour, byte-identical)
  double chartCollapseFrac = -1.0;  ///< ‖dU‖ < chartCollapseFrac·‖dV‖ (and ·scale) ⇒ chart collapse (≤ 0 → 1e-3)
  double chartStepFrac = -1.0;      ///< fine step off the singular point when crossing = chartStepFrac·h0 (≤ 0 → 1/16)
  int chartMaxSteps = 256;          ///< max fine point-based steps spent crossing one pole/apex before deferring
};

/// The full S3 result (design.md TraceSet): one WLine per distinct traced branch plus
/// the honest diagnostics. `TraceSet` is the S5 curved-boolean contract — each WLine
/// splits a curved face; `nearTangentGaps > 0` is the honest S4 signal (a branch traced
/// up to a tangency, remainder deferred).
struct TraceSet {
  std::vector<WLine> lines{};   ///< one WLine per distinct traced transversal branch

  int tracedBranches = 0;       ///< WLines produced (Closed | BoundaryExit)
  int nearTangentGaps = 0;      ///< marches stopped at a near-tangent region that could NOT be crossed → S4 (reported)
  int nearTangentCrossed = 0;   ///< S4-c: near-tangent TRANSVERSAL grazes MARCHED THROUGH (would have truncated in S3)
  int singularitiesCrossed = 0; ///< S4-e: chart poles/apexes STEPPED ACROSS + verified across all branches (would have truncated in S3). `nearTangentGaps` keeps counting ONLY singularities that could NOT be crossed (deferred → OCCT)
  int dedupedRetraces = 0;      ///< seeds whose march retraced an already-traced branch

  // Extra diagnostics (kept for verification/reporting; not in the minimal contract).
  int seededBranches = 0;       ///< branches the S2 seeder handed us (SeedSet.branchCount)
  int deferredTangent = 0;      ///< S2 near-tangent branches never seeded (echoed S4 gap)
  int closedCurves = 0;         ///< WLines that closed into a loop
  int openCurves = 0;           ///< WLines running boundary-to-boundary

  // ── S4-d branch-point diagnostics (0 unless MarchOptions.enableBranchPoints) ──
  std::vector<BranchNode> branchNodes{};  ///< localized branch points + their arm connectivity
  int branchPoints = 0;          ///< #branch points localized + routed (== branchNodes.size())
  int routedArms = 0;            ///< #arm WLines routed off branch points and kept (not deduped)

  // ── S4-f SELF-INTERSECTION + COMPLETENESS-CRITIC diagnostics ──
  //
  // HONEST ASYMPTOTIC FRAMING (must not be overclaimed): the completeness critic RAISES the
  // recall FLOOR by re-seeding finer in uncovered regions until dry; it is NOT a proof. Below
  // ANY fixed subdivision resolution a smaller loop can still be missed, so `completenessResidual`
  // is ALWAYS true. `criticFloorFrac` is the finest minPatchFrac reached; recall against OCCT
  // stays a MEASURED figure (see RecallReport.recall()), never a blind 1.0.
  int selfIntersections = 0;      ///< S4-f: total self-crossings recorded over all arms (0 unless enableSelfIntersection)
  int criticRounds = 0;           ///< S4-f: adaptive-critic re-seed rounds run (0 unless completenessCritic)
  int criticRecoveredLoops = 0;   ///< S4-f: NEW branches the critic recovered beyond the initial fixed-resolution trace
  double criticFloorFrac = 0.0;   ///< S4-f: finest minPatchFrac the critic reached (the honest recall floor; 0 if critic off)
  bool criticStoppedDry = false;  ///< S4-f: true if the critic stopped after K dry rounds (vs hitting the cost cap)
  bool completenessResidual = true;  ///< S4-f: ALWAYS true — a loop below criticFloorFrac can still exist (never a proof)

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
