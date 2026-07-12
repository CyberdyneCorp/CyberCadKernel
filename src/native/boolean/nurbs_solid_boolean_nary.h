// SPDX-License-Identifier: Apache-2.0
//
// nurbs_solid_boolean_nary.h — BOOL-MULTI-OP / LAYER 3: the N-OPERAND NURBS solid
// boolean (assemblies). `nurbsSolidUnionN` / `nurbsSolidCutN` / `nurbsSolidIntersectN`
// fold the general two-solid boolean `nurbsSolidBoolean(A, B, op)` (nurbs_solid_boolean.h)
// over a LIST of freeform NURBS solids — a union of many parts, a base minus several tools
// (a part with multiple pockets), the common of an assembly.
//
// ── WHAT THIS IS (a LEFT-FOLD, not a new boolean) ─────────────────────────────────
// The binary orchestrator already welds two freeform NURBS solids into ONE watertight-or-
// honest-decline result. Real CAD needs the N-operand generalisation:
//   * UNION(s₀, …, sₙ)     = (((s₀ ∪ s₁) ∪ s₂) … ∪ sₙ)          — fuse a list of solids.
//   * CUT(base, t₀, …, tₙ) = (((base − t₀) − t₁) … − tₙ)          — a base minus tools.
//   * INTERSECT(s₀, …, sₙ) = (((s₀ ∩ s₁) ∩ s₂) … ∩ sₙ)          — the common of all.
// Each is a LEFT-FOLD of the SAME binary boolean; this header COMPOSES it and re-implements
// NONE of it — exactly the compose pattern of `feature_ops.h` (pocket/boss on top of the
// binary boolean). Order-aware where the op is order-sensitive: CUT applies the tools in the
// GIVEN order (A − B ≠ B − A as SETS); UNION / INTERSECT are commutative/associative so the
// meshed volume is fold-order-invariant beyond the tessellation band.
//
// ── HONESTY (DISAGREED=0 SACRED) ──────────────────────────────────────────────────
// The fold is HONEST-DECLINE-SHORT-CIRCUITING: the moment ANY intermediate binary boolean
// declines (a null/unadmitted operand, no usable seam, a multi-seam sew that abstains, a
// beyond-tractable pose), the N-ary op returns a NULL Shape carrying THAT intermediate's
// measured residual — the exact `SolidBoolReport` of the step that failed, plus the fold
// index it failed at. It NEVER returns a leaky/partial/wrong solid, and NO tolerance is
// ever widened (the fold merely threads the binary boolean's own gates). An empty list is a
// first-class decline; a single-element list is the identity (that one solid, unchanged).
//
// ── RE-ADMISSION (BOOL-READMIT — the fold now re-admits the boolean's output) ───────
// A fold of MORE than two freeform operands re-feeds the prior binary boolean's OUTPUT solid
// as the next step's operand `acc`. That output is NOT a pristine single-wall bowl-cup: it is
// a MULTI-freeform-wall solid whose walls are SEAM-SPLIT ANNULI (the seam is an interior HOLE
// loop), which the byte-frozen B1 gate `recogniseFreeformSolid` declines on two counts
// (`HoledFreeformFace` + its exactly-one-simply-trimmed-wall rule). `boolean_readmit.h`
// (additive; does NOT edit the frozen gate) makes it re-admissible: `nurbsSolidBooleanReadmit`
// admits the holed/multi-wall output through a MORE-PERMISSIVE sibling recogniser, and:
//   * step 1 (pristine acc) DEFERS bit-identically to `nurbsSolidBoolean` — 2-operand folds
//     stay UNREGRESSED;
//   * a REDUNDANT operand (a re-applied part contained in the union, a tool disjoint from the
//     remaining material, an intersect operand containing the acc) SHORT-CIRCUITS to `acc`
//     EXACTLY by a coincidence-tolerant membership witness — no weld, no synthesised
//     geometry, so DISAGREED=0 is structural. This welds the reachable idempotent ≥3 folds
//     (`{A,B,B}` union, `(base−T)−T` cut) watertight at the inclusion-exclusion volume.
// The residual boundary is now SHARPER and NARROWER: a GENUINELY-OVERLAPPING ≥3 fold whose
// second seam lands on an ALREADY-HOLED annulus needs a MULTI-HOLE / multi-crossing face split
// (splitting an annulus that already carries a seam-hole by a second seam) — the readiness
// doc's UNLANDED §4 multi-crossing split. That sub-case HONEST-DECLINES `StepDeclined` with
// the weld's residual map at the failing fold index — NOT a leaky/partial solid; NO tolerance
// widened. When that multi-hole split lands, the SAME fold extends with no change here.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_NARY_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_NARY_H

#include "native/boolean/boolean_readmit.h"
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/topology/native_topology.h"

#include <cstddef>
#include <limits>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// The measured N-ary decline. `Ok` iff a verified watertight solid is returned; every
// non-Ok is an HONEST decline to NULL (never a leaky/partial/wrong solid). `EmptyList`
// is the N-ary op's OWN pre-check; `StepDeclined` defers to the underlying binary boolean's
// measured `SolidBoolReport` (carried verbatim in `stepReport`) at fold index `stepIndex`.
// ─────────────────────────────────────────────────────────────────────────────
enum class NaryBoolDecline {
  Ok,
  EmptyList,     ///< the operand list was empty (nothing to fold)
  NullOperand,   ///< a list entry was a null Shape (caught before the binary boolean ran)
  StepDeclined   ///< an intermediate binary boolean honest-declined (see `stepReport`)
};

inline const char* naryBoolDeclineName(NaryBoolDecline d) noexcept {
  switch (d) {
    case NaryBoolDecline::Ok: return "Ok";
    case NaryBoolDecline::EmptyList: return "EmptyList";
    case NaryBoolDecline::NullOperand: return "NullOperand";
    case NaryBoolDecline::StepDeclined: return "StepDeclined";
  }
  return "?";
}

/// The honest witnesses of an N-ary fold (for the host gate / caller log). `stepReport` is
/// the DECLINING intermediate binary boolean's measured report verbatim (its `decline`,
/// watertight/coherent witnesses, residual `boundaryEdges` map, sub-verb reasons) when a
/// step declined; `finalReport` is the LAST successful step's report (the result's own
/// witnesses) on success. `steps` counts the binary booleans actually run.
struct NaryBoolReport {
  NaryBoolDecline decline = NaryBoolDecline::Ok;
  SolidBoolOp op = SolidBoolOp::Fuse;
  int operands = 0;                 ///< length of the input list
  int steps = 0;                    ///< binary booleans actually folded (= operands − 1 on Ok)
  int stepIndex = -1;               ///< fold index of the declining step (−1 ⇒ none / n/a)
  SolidBoolReport stepReport;       ///< the declining step's report (valid iff StepDeclined)
  SolidBoolReport finalReport;      ///< the last successful step's report (valid iff steps > 0)
};

namespace nsbnary {

/// The shared LEFT-FOLD driver. Folds `nurbsSolidBoolean(acc, solids[i], op)` from
/// `solids[0]` across the tail, short-circuiting to an HONEST decline the instant any
/// intermediate boolean returns NULL / a non-Ok report — carrying that step's residual and
/// index. An empty list declines `EmptyList`; a single-element list is the identity. A null
/// entry declines `NullOperand` BEFORE the binary boolean is invoked (the binary boolean
/// would itself decline NotAdmitted, but the pre-check names it crisply and cheaply).
///
/// `analyticFold` (optional, NaN ⇒ unknown): the closed-form volume of the WHOLE folded
/// result, forwarded to the FINAL step's two-sided volume band only — intermediate steps
/// keep NaN (their partial volumes are not the caller's closed form). This keeps the last
/// weld gated against the true op-volume without over-constraining the partial folds.
inline topo::Shape foldBoolean(const std::vector<topo::Shape>& solids, SolidBoolOp op,
                               double deflection, NaryBoolReport& rep, double analyticFold) {
  rep.op = op;
  rep.operands = static_cast<int>(solids.size());
  auto emit = [&](topo::Shape s) -> topo::Shape { return s; };
  auto fail = [&](NaryBoolDecline d, int idx) -> topo::Shape {
    rep.decline = d;
    rep.stepIndex = idx;
    return emit({});
  };

  if (solids.empty()) return fail(NaryBoolDecline::EmptyList, -1);
  for (std::size_t i = 0; i < solids.size(); ++i)
    if (solids[i].isNull()) return fail(NaryBoolDecline::NullOperand, static_cast<int>(i));

  // Single-element list — the identity (that one solid, unchanged; no boolean to run).
  if (solids.size() == 1) {
    rep.decline = NaryBoolDecline::Ok;
    rep.steps = 0;
    return emit(solids.front());
  }

  topo::Shape acc = solids.front();
  const std::size_t last = solids.size() - 1;
  for (std::size_t i = 1; i < solids.size(); ++i) {
    // Only the FINAL fold step is gated against the caller's whole-result closed form.
    const double stepAnalytic =
        (i == last) ? analyticFold : std::numeric_limits<double>::quiet_NaN();
    SolidBoolReport srep;
    // Fold via the RE-ADMITTING boolean: step 1 (pristine acc) defers bit-identically to
    // `nurbsSolidBoolean` (2-operand folds UNREGRESSED); steps ≥ 2 re-admit the prior
    // boolean's holed/multi-wall output — a redundant operand short-circuits to `acc`
    // exactly, a genuine-overlap ≥3 case honest-declines at the measured multi-hole-split
    // boundary (never a leaky solid).
    const topo::Shape next =
        nurbsSolidBooleanReadmit(acc, solids[i], op, deflection, &srep, stepAnalytic);
    ++rep.steps;
    if (next.isNull() || srep.decline != SolidBoolDecline::Ok) {
      rep.stepReport = srep;
      return fail(NaryBoolDecline::StepDeclined, static_cast<int>(i));
    }
    rep.finalReport = srep;
    acc = next;
  }
  rep.decline = NaryBoolDecline::Ok;
  return emit(acc);
}

}  // namespace nsbnary

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidUnionN — fuse a LIST of freeform NURBS solids: (((s₀ ∪ s₁) ∪ s₂) … ∪ sₙ). A
// commutative/associative left-fold of `nurbsSolidBoolean(acc, sᵢ, Fuse)`. Returns the
// welded watertight union or an HONEST decline to NULL with a measured `NaryBoolReport`
// carrying the declining step's residual (never a leaky/partial/wrong solid; no tolerance
// widened). `analyticUnion` (optional) gates the FINAL weld against the closed-form union
// volume (inclusion-exclusion of the parts).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidUnionN(const std::vector<topo::Shape>& solids,
                                    double deflection = 0.005, NaryBoolReport* report = nullptr,
                                    double analyticUnion =
                                        std::numeric_limits<double>::quiet_NaN()) {
  NaryBoolReport rep;
  const topo::Shape r = nsbnary::foldBoolean(solids, SolidBoolOp::Fuse, deflection, rep,
                                             analyticUnion);
  if (report) *report = rep;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidCutN — subtract a LIST of tool solids from a base: (((base − t₀) − t₁) … − tₙ).
// An ORDER-SENSITIVE left-fold of `nurbsSolidBoolean(acc, tᵢ, Cut)` (the base is folded with
// each tool IN THE GIVEN ORDER; Cut is per-operand so the order defines which material each
// tool removes). The fold list is [base, t₀, …, tₙ]; an empty tool list is the identity
// (the base, unchanged). Returns the carved watertight solid or an HONEST decline to NULL
// with the declining step's residual. `analyticCut` (optional) gates the FINAL weld.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidCutN(const topo::Shape& base, const std::vector<topo::Shape>& tools,
                                  double deflection = 0.005, NaryBoolReport* report = nullptr,
                                  double analyticCut =
                                      std::numeric_limits<double>::quiet_NaN()) {
  std::vector<topo::Shape> solids;
  solids.reserve(tools.size() + 1);
  solids.push_back(base);
  for (const topo::Shape& t : tools) solids.push_back(t);
  NaryBoolReport rep;
  const topo::Shape r = nsbnary::foldBoolean(solids, SolidBoolOp::Cut, deflection, rep, analyticCut);
  if (report) *report = rep;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidIntersectN — the common of a LIST of freeform NURBS solids:
// (((s₀ ∩ s₁) ∩ s₂) … ∩ sₙ). A commutative/associative left-fold of
// `nurbsSolidBoolean(acc, sᵢ, Common)`. Returns the watertight common or an HONEST decline
// to NULL with the declining step's residual. `analyticIntersect` (optional) gates the FINAL
// weld against the closed-form common volume.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidIntersectN(const std::vector<topo::Shape>& solids,
                                        double deflection = 0.005, NaryBoolReport* report = nullptr,
                                        double analyticIntersect =
                                            std::numeric_limits<double>::quiet_NaN()) {
  NaryBoolReport rep;
  const topo::Shape r = nsbnary::foldBoolean(solids, SolidBoolOp::Common, deflection, rep,
                                             analyticIntersect);
  if (report) *report = rep;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_NARY_H
