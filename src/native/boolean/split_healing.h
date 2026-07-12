// SPDX-License-Identifier: Apache-2.0
//
// split_healing.h — NURBS roadmap Layer 3, Stage 3 residual: the TOLERANT-TOPOLOGY
// SPLIT-HEALING PRE-PASS. Repairs a RAW trim-loop set (SSI-derived, with small gaps,
// near-coincident vertices, and pinch points) into split-ready VALID simple loops the
// face-split machinery (boolean/face_split.h, boolean/smooth_trim_split.h) can consume.
//
// ── ROLE ──────────────────────────────────────────────────────────────────────
// The split machinery assumes CLEAN input loops: a single simple closed loop per
// region, endpoints exactly coincident, no self-touch. Real SSI-derived trim loops do
// NOT satisfy this — a marched seam's arc endpoints miss by a marching tolerance (a
// GAP), two arcs land on near-coincident vertices (a SNAP), or a loop folds back on
// itself (a PINCH). This header is the bounded PRE-PASS that turns such a raw loop set
// into split-ready loops OR declines honestly — it never fabricates a closure across a
// real gap and never force-splits an ambiguous pinch.
//
// ── COMPOSES (never re-implements) the Wave-G/G5 healing in topology/trimmed_nurbs.h ──
//   * topology::splitTrimLoopAtPinches — flatten (join-gap-aware) + weld small gaps +
//     resolve pinches (N-way / crossing) into simple sub-loops, in ONE call. It is
//     region- AND signed-area-preserving (the trimmed_nurbs header proof) and honest-
//     declines a genuine large gap (healTriageLargeGap) or an unresolvable pinch. This
//     is the single primitive that already does gap-close + snap + pinch-resolve; this
//     header only ORCHESTRATES it over a loop SET and reports what was healed.
//   * topology::healTrimLoop — the diagnosis (gapsClosed / pinch / largeGap) used to
//     tell a genuine NO-OP (0 heals, no pinch, no large gap) from a loop that needed
//     work. A no-op loop is passed through BYTE-IDENTICALLY (the original arcs), so a
//     clean input set is returned unchanged with 0 heals — never re-flattened.
//   * topology::flattenTrimLoop — the SAME seam-consistent pcurve flattener the split
//     ray-casts, used to measure the input signed area for the preservation guard.
//
// ── SIGNED-AREA / REGION PRESERVATION (the load-bearing invariant) ─────────────
// Every heal is region- and SIGNED-AREA-preserving by the trimmed_nurbs proofs (a weld
// moves a vertex ≤ tol/2; a pinch re-route only re-partitions the same directed edges
// into cycles). This header ADDS a host-checkable guard: the sum of the OUTPUT loops'
// signed areas must equal the sum of the INPUT loops' signed areas within a scale-
// relative tolerance. A mismatch → the whole set DECLINES (ok=false) rather than emit a
// silently-wrong region. This is the "area preserved ≤ 1e-12" oracle, promoted to a gate.
//
// ── OUTPUT ─────────────────────────────────────────────────────────────────────
// Healed loops are returned as FLATTENED UV polylines (the form the split ray-casts and
// that flattenTrimLoop / face_split's shoelace already consume). A no-op loop is instead
// returned as its ORIGINAL arcs (byte-identical) AND its flattened polyline, so a caller
// wanting the split-ready polyline always has one, while the byte-identical no-op is
// observable (report.anyChanged == false, report.arcsUnchanged[i] == true).
//
// OCCT-FREE. Uses ONLY src/native/topology (which is math + topology). Header-only.
// clang++ -std=c++20. fp64, deterministic. Additive: modifies NO existing file.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SPLIT_HEALING_H
#define CYBERCAD_NATIVE_BOOLEAN_SPLIT_HEALING_H

#include "native/topology/trimmed_nurbs.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;

// ─────────────────────────────────────────────────────────────────────────────
// A healed loop = a flattened, split-ready UV polyline. `ParamPoint` is the same
// (u,v) point the split machinery uses (via flattenTrimLoop). Implicitly closed.
// ─────────────────────────────────────────────────────────────────────────────
using HealedLoop = std::vector<topo::ParamPoint>;

// ─────────────────────────────────────────────────────────────────────────────
// Per-input-loop heal outcome. Exactly one of {noOp, healed, declined}:
//   * noOp     — the loop was already clean (0 gaps closed, no pinch, no large gap);
//                the ORIGINAL arcs pass through byte-identically (arcsUnchanged=true).
//   * healed   — a small gap was welded / a near-coincident vertex snapped / a pinch
//                resolved; `outLoops` carries the repaired simple sub-loop polyline(s).
//   * declined — a genuine gap > tol, or an ambiguous (3+-way / crossing) pinch that
//                G5 could not resolve. `outLoops` is empty; `blocker` says why.
// ─────────────────────────────────────────────────────────────────────────────
enum class HealBlocker : std::uint8_t {
  None = 0,       ///< healed or no-op (not a decline)
  LargeGap,       ///< a join gap exceeded the scale-relative tolerance (never force-welded)
  AmbiguousPinch, ///< a pinch G5 could not cleanly resolve (non-manifold touch)
  Degenerate,     ///< the loop collapsed to < 3 distinct points after welding
};

inline const char* blockerName(HealBlocker b) noexcept {
  switch (b) {
    case HealBlocker::None: return "None";
    case HealBlocker::LargeGap: return "LargeGap";
    case HealBlocker::AmbiguousPinch: return "AmbiguousPinch";
    case HealBlocker::Degenerate: return "Degenerate";
  }
  return "?";
}

struct LoopHeal {
  bool noOp = false;            ///< the loop was clean → original arcs unchanged
  bool healed = false;         ///< a gap/snap/pinch was repaired
  bool declined = false;       ///< an honest decline (see blocker)
  bool arcsUnchanged = true;   ///< true ⇔ the original arcs were passed through byte-identically
  bool pinchResolved = false;  ///< true ⇔ a pinch was split into simple sub-loops (G5 splitAtPinches)
  int gapsClosed = 0;          ///< small gaps / near-coincident pairs welded (from healTrimLoop)
  int subLoops = 0;            ///< number of simple sub-loops this input loop produced
  double maxGapClosed = 0.0;   ///< largest gap actually welded (UV space)
  double residualGap = 0.0;    ///< the remaining un-welded gap that forced a large-gap decline
  double signedAreaIn = 0.0;   ///< signed area of the input loop (flattened)
  double signedAreaOut = 0.0;  ///< summed signed area of the output sub-loop(s)
  HealBlocker blocker = HealBlocker::None;
  std::vector<HealedLoop> outLoops;  ///< the healed split-ready polyline(s) (empty iff declined)
};

// ─────────────────────────────────────────────────────────────────────────────
// The whole-set heal report. `ok` iff EVERY input loop healed or was a no-op AND the
// total signed area is preserved. On ANY loop decline OR an area mismatch, ok=false
// (honest decline of the whole set — a split fed a partially-healed set could leak).
// ─────────────────────────────────────────────────────────────────────────────
struct HealTrimLoopsReport {
  bool ok = false;              ///< true ⇔ all loops healed/no-op AND area preserved → split-ready
  bool anyChanged = false;      ///< true ⇔ any loop was welded / snapped / pinch-split
  bool areaPreserved = false;   ///< true ⇔ Σ signedArea(out) == Σ signedArea(in) within tol
  int loopsHealed = 0;          ///< loops that needed (and got) a repair
  int loopsNoOp = 0;            ///< loops already clean (byte-identical pass-through)
  int loopsDeclined = 0;        ///< loops honest-declined
  int pinchesResolved = 0;      ///< loops whose pinch was split into simple sub-loops
  double totalAreaIn = 0.0;     ///< Σ signed area of input loops
  double totalAreaOut = 0.0;    ///< Σ signed area of output sub-loops
  double areaResidual = 0.0;    ///< |totalAreaOut − totalAreaIn|
  double areaTolerance = 0.0;   ///< the scale-relative area tolerance actually applied
  std::vector<LoopHeal> loops;  ///< per-input-loop outcome (parallel to the input vector)
  /// The flattened split-ready loops, in order (only populated when ok). A single input
  /// loop may contribute several (pinch → sub-loops); a no-op loop contributes one.
  std::vector<HealedLoop> healedLoops() const {
    std::vector<HealedLoop> out;
    for (const LoopHeal& lh : loops)
      for (const HealedLoop& hl : lh.outLoops) out.push_back(hl);
    return out;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Tuning. `gapTol` is the SAME scale-relative gap/snap/pinch band the G5 healer uses
// (× the loop's UV extent) — NEVER widened here to force a heal. `flatten` is the per-
// segment sample count (matches the split's flatten density). `areaAbsTol` / `areaRelTol`
// bound the signed-area-preservation guard (tol = areaAbsTol + areaRelTol·|areaIn|).
// ─────────────────────────────────────────────────────────────────────────────
struct HealTrimLoopsOptions {
  double gapTol = 1e-6;      ///< relative gap/snap/pinch band (× loop UV extent); G5 HealOptions::gapTol
  int flatten = 48;          ///< per-segment flatten sample count (matches the split)
  double areaAbsTol = 1e-12; ///< absolute floor of the signed-area-preservation tolerance
  double areaRelTol = 1e-9;  ///< relative term (× |input signed area|) of that tolerance
  double noOpGapFloor = 1e-12;///< a "gap" welded below this is FLOATING-POINT NOISE (a clean
                              ///< closed loop's corner sampled by two adjacent segments differs
                              ///< by ~1 ULP): it is NOT a real heal, so the loop is a genuine
                              ///< NO-OP. A weld above this floor is a real gap-close / snap.
};

namespace shdetail {

// Signed area (shoelace) of a flattened closed polyline — the SAME formula face_split's
// detail::shoelace and trimmed_nurbs's signedArea use, so the preservation guard speaks
// the split's own area language.
inline double signedArea(const HealedLoop& p) noexcept {
  const std::size_t n = p.size();
  if (n < 3) return 0.0;
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += p[j].u * p[i].v - p[i].u * p[j].v;
  return 0.5 * a;
}

}  // namespace shdetail

// ─────────────────────────────────────────────────────────────────────────────
// healTrimLoops — the Stage-3 tolerant-topology PRE-PASS.
//
// For each raw input loop, in order:
//   1. DIAGNOSE with topology::healTrimLoop → {gapsClosed, pinch, largeGap}. A loop with
//      0 heals, no pinch, no large gap is a NO-OP: its ORIGINAL arcs pass through byte-
//      identically (flattened once for the split-ready polyline + the area guard).
//   2. Otherwise REPAIR by composing topology::splitTrimLoopAtPinches (ONE call: flatten
//      join-gap-aware → triage a genuine large gap (decline) → weld small gaps / snap
//      near-coincident vertices → resolve pinches N-way into simple sub-loops). Its
//      MultiSplitReport is region- and signed-area-preserving.
//   3. A genuine large gap (report.largeGap) OR an unresolvable pinch (report.ambiguous)
//      OR a degenerate collapse → this loop DECLINES honestly (blocker set); the whole
//      set's ok becomes false. A split must never consume a partially-healed set.
//   4. GUARD: Σ signedArea(output sub-loops) == Σ signedArea(input loops) within the
//      scale-relative area tolerance. A mismatch → ok=false (never a silent-wrong region).
//
// Returns a full report; `report.ok` ⇔ every loop healed/no-op AND area preserved, i.e.
// `report.healedLoops()` is a split-ready valid simple-loop set.
// ─────────────────────────────────────────────────────────────────────────────
inline HealTrimLoopsReport healTrimLoops(const std::vector<topo::TrimLoop>& loops,
                                         const HealTrimLoopsOptions& opts = {}) {
  HealTrimLoopsReport rep;
  rep.loops.reserve(loops.size());

  const topo::HealOptions g5{opts.gapTol};  // the SAME G5 scale-relative band; never widened
  double totalAreaIn = 0.0, totalAreaOut = 0.0;
  bool anyDecline = false;

  for (const topo::TrimLoop& loop : loops) {
    LoopHeal lh;

    // (0) Flatten once with the split's seam-consistent evaluator — the input polyline is
    // the area reference and the byte-identical no-op's split-ready form.
    const HealedLoop flatIn = topo::flattenTrimLoop(loop, opts.flatten);
    lh.signedAreaIn = shdetail::signedArea(flatIn);
    totalAreaIn += lh.signedAreaIn;

    // (1) Diagnose: distinguish a genuine NO-OP from a loop that needs work.
    const topo::HealReport diag = topo::healTrimLoop(loop, g5, opts.flatten);
    lh.maxGapClosed = diag.maxGapClosed;

    // A genuine NO-OP welded no REAL gap: any weld it performed was below the FP-noise floor
    // (a clean closed loop's corner sampled by two adjacent segments differs by ~1 ULP, which
    // healWeldGaps counts as a "gap closed" — that is not a real heal). gapsClosed above the
    // floor, a pinch, or a large gap all mean the loop needs work. `gapsClosed` in the report
    // counts ONLY real (above-floor) welds, so a no-op reports 0.
    const bool realGapClosed = diag.gapsClosed > 0 && diag.maxGapClosed > opts.noOpGapFloor;
    lh.gapsClosed = realGapClosed ? diag.gapsClosed : 0;
    const bool needsWork = realGapClosed || diag.pinch || diag.largeGap;
    if (!needsWork && diag.healed) {
      // NO-OP: pass the original arcs through byte-identically; the flattened polyline is
      // the split-ready form. 0 real heals reported.
      lh.noOp = true;
      lh.arcsUnchanged = true;
      lh.subLoops = 1;
      lh.signedAreaOut = lh.signedAreaIn;
      lh.outLoops.push_back(flatIn);
      totalAreaOut += lh.signedAreaOut;
      rep.loopsNoOp += 1;
      rep.loops.push_back(std::move(lh));
      continue;
    }

    // (2) Repair by composing the G5 gap-close + snap + N-way pinch resolver (one call).
    const topo::MultiSplitReport ms = topo::splitTrimLoopAtPinches(loop, g5, opts.flatten);
    lh.arcsUnchanged = false;

    if (!ms.ok) {
      // (3) Honest decline — classify the blocker from the G5 diagnosis (large gap wins,
      // then an ambiguous pinch, else a degenerate collapse). NEVER a fabricated closure.
      lh.declined = true;
      if (diag.largeGap) {
        lh.blocker = HealBlocker::LargeGap;
        lh.residualGap = diag.residualGap;
      } else if (ms.ambiguous || diag.pinch) {
        lh.blocker = HealBlocker::AmbiguousPinch;
      } else {
        lh.blocker = HealBlocker::Degenerate;
      }
      anyDecline = true;
      rep.loopsDeclined += 1;
      rep.loops.push_back(std::move(lh));
      continue;
    }

    // Healed: collect the simple sub-loop polylines (1 for a gap-close, ≥2 for a pinch).
    lh.healed = true;
    lh.pinchResolved = ms.pinch;
    lh.subLoops = static_cast<int>(ms.loops.size());
    for (const std::vector<topo::ParamPoint>& s : ms.loops) {
      lh.outLoops.push_back(s);
      lh.signedAreaOut += shdetail::signedArea(s);
    }
    totalAreaOut += lh.signedAreaOut;
    rep.loopsHealed += 1;
    if (ms.pinch) rep.pinchesResolved += 1;
    rep.anyChanged = true;
    rep.loops.push_back(std::move(lh));
  }

  rep.totalAreaIn = totalAreaIn;
  rep.totalAreaOut = totalAreaOut;
  rep.areaResidual = std::fabs(totalAreaOut - totalAreaIn);
  rep.areaTolerance = opts.areaAbsTol + opts.areaRelTol * std::fabs(totalAreaIn);
  rep.areaPreserved = rep.areaResidual <= rep.areaTolerance;

  // The set is split-ready iff no loop declined AND the total signed area is preserved.
  // A partial heal (some loop declined) or an area drift is a whole-set honest decline.
  rep.ok = !anyDecline && rep.areaPreserved && !loops.empty();
  return rep;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SPLIT_HEALING_H
