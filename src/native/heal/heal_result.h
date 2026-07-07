// SPDX-License-Identifier: Apache-2.0
//
// heal_result.h — the typed result of a healing attempt (Phase 4 #4
// `native-healing`, first slice).
//
// A heal attempt has exactly two outcomes, and the caller must be able to tell
// them apart WITHOUT trusting a fabricated closure:
//
//   * Healed   — the four sub-operations produced a candidate that SELF-VERIFIED
//                watertight + valid (enclosed volume > 0) with every merge inside
//                the stated tolerance. `shape` is the repaired shell/solid.
//   * Unhealed — the input is outside the coincident-within-tolerance / degenerate
//                / orientation family this slice heals (a gap beyond tolerance, a
//                genuinely open shell, a non-manifold input, a self-verify failure,
//                or an out-of-scope defect). `shape` is the INPUT, UNCHANGED, and
//                `metrics.maxResidualGap` carries the measured residual. The engine
//                must route an Unhealed shape to OCCT `BRepBuilderAPI_Sewing` +
//                `ShapeFix` (the deferral seam — see native_heal.h).
//
// `Unhealed` is DATA, not an error (exactly like SSI's `NotAnalytic`): it is the
// honest first-class outcome that keeps the healer from ever claiming a closure it
// did not achieve. The tolerance is NEVER auto-relaxed to force a Healed result.
//
// OCCT-FREE. Includes only src/native/topology + src/native/math. clang++
// -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_HEAL_RESULT_H
#define CYBERCAD_NATIVE_HEAL_HEAL_RESULT_H

#include "native/math/native_math.h"
#include "native/topology/shape.h"

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

/// The two outcomes of a heal attempt.
enum class HealStatus {
  Healed,    ///< self-verified watertight + valid; caller uses `shape`
  Unhealed,  ///< out of scope / gap > tol / still open → DEFER to OCCT ShapeFix
};

/// Why a heal was deferred. Meaningful only when `status == Unhealed`.
enum class UnhealedReason {
  None,                ///< status == Healed
  GapBeyondTolerance,  ///< a shared boundary is farther apart than `tolerance`
  OpenShell,           ///< still has boundary edges after sewing (a real hole)
  NonManifold,         ///< an edge is shared by 3+ faces (bad input, not a soup)
  SelfVerifyFailed,    ///< candidate did not tessellate watertight / volume sign wrong
  OutOfScope,          ///< missing pcurve, self-intersecting wire, freeform re-approx
  GapBeyondBudget,     ///< a near-miss gap exceeds the opt-in bridge budget / feature cap
};

/// Measured facts about the heal (all counts are of the operations actually
/// applied; maxResidualGap is the honest residual — 0 ⇒ fully closed).
struct HealMetrics {
  bool watertight = false;      ///< self-verified closed 2-manifold mesh
  bool valid = false;           ///< consistent outward orientation (enclosed vol > 0)
  int nMergedVerts = 0;         ///< near-coincident vertices unified (input − output)
  int nMergedEdges = 0;         ///< coincident-within-tol edge pairs stitched
  int nDroppedDegenerate = 0;   ///< zero-length edges + sliver/zero-area faces removed
  int nFlipped = 0;             ///< faces re-oriented by the flood-fill
  int nBridgedGaps = 0;         ///< boundary corners closed by the opt-in bridging pass
  double maxBridgedGap = 0.0;   ///< largest near-miss gap bridged (≤ the effective bound)
  double maxResidualGap = 0.0;  ///< largest surviving boundary gap (0 ⇒ closed)
};

/// The result handed back by `healShell`.
struct HealResult {
  HealStatus status = HealStatus::Unhealed;
  UnhealedReason reason = UnhealedReason::None;  ///< meaningful when Unhealed
  topo::Shape shape;                             ///< Healed ⇒ repaired; Unhealed ⇒ input UNCHANGED
  HealMetrics metrics;

  bool healed() const noexcept { return status == HealStatus::Healed; }
};

/// Options controlling a heal. The tolerance is the SINGLE sew/weld/merge
/// tolerance and is NEVER widened by the healer to force a pass.
struct HealOptions {
  double tolerance = 1e-6;  ///< sew/weld/merge tolerance (model units)

  /// Opt-in budget (model units) for the bounded beyond-tolerance gap-bridging
  /// pass: the largest near-miss gap the healer may close ABOVE `tolerance`. It is
  /// a SEPARATE, caller-supplied bound — the primary weld `tolerance` is never
  /// widened by it. Each candidate gap is additionally capped at a quarter of the
  /// shortest edge incident to the corner (the local-feature-size cap, see
  /// gap_bridge.h), so a gap comparable to a real edge can never be bridged
  /// regardless of how large this is set. `0.0` (default) DISABLES bridging, making
  /// `healShell` byte-identical to the landed slice.
  double gapBridgeBudget = 0.0;
};

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_HEAL_RESULT_H
