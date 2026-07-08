// SPDX-License-Identifier: Apache-2.0
//
// multi_seam.h — MOAT M2-multiseam: the entry point for the FIRST *multi-seam*
// two-operand freeform boolean, and the honest boundary of what the byte-frozen
// single-seam machinery can reach.
//
// ── WHAT THIS ADDS OVER THE LANDED SINGLE-SEAM TWO-OPERAND BOOLEAN ────────────────
// `two_operand.h` (`freeformBooleanTwoOperand`) handles the pose where EXACTLY ONE of
// `B`'s faces slices `A`'s freeform wall (one closed seam loop). This header generalises
// the CALLER to the SEAM GRAPH: a corner box `B` positioned so TWO adjacent faces slice
// `A`'s wall in two arcs that meet at a junction vertex `J` (design §1). It composes:
//
//   recognise[B1] → `buildSeamGraph` (two-arc, one-junction graph, `seam_graph.h`) →
//   the per-arc freeform split[B2] the junction weld composes → (junction weld) →
//   self-verify → OCCT fallback.
//
// ── THE REACHED LEVEL + THE MEASURED NEXT BLOCKER (design §9) ─────────────────────
// The seam-graph builder is LANDED and proven in isolation (level 3): the two arcs are
// traced (byte-unchanged M1), the junction `J` is computed ANALYTICALLY and verified to
// lie on both cutting planes inside the trimmed wall, the arcs are clipped at `J` and
// joined into one bent boundary→J→boundary seam, and EACH arc individually splits the
// wall through the byte-frozen B2 `splitFace` (the two single-seam partitions the weld
// composes). The junction WALL SPLIT — partitioning the wall into the corner sub-face
// (`A ∩ {x≥0, y≥0}`) and the L-shaped survivor at the shared valence-3 vertex `J` — is
// now LANDED (this wave) via `splitFaceJunction` (`junction_split.h`): where the byte-
// frozen B2 declines `RebuildMismatch` (its fixed-density reflatten shortcuts the sharp
// interior kink at `J`, losing ~1e-5·parentArea), the junction-aware verb introduces `J`
// as an EXACT shared valence-3 vertex — two seam edges (arc0-half E→J, arc1-half J→X)
// meeting at `J`. Because the two arcs are ORTHOGONAL iso-parametric curves (u-const /
// v-const → straight lines in UV), the only bend is the right-angle at `J`, so once `J`
// is an edge endpoint each half reflattens to its combinatorial loop to MACHINE PRECISION
// and the SAME strict rebuild tolerance (never weakened) is satisfied. The wall partition
// is exact (`tilingGap`, `rebuildResidual` ~1e-16; the corner UV area equals the closed-
// form `Q ∩ {u≥½, v≥½}` projection to 7e-17).
//
// The MEASURED NEXT BLOCKER is now the full multi-FACE corner-clip weld: the corner box
// straddles the corner of `A`'s footprint quad `Q`, so the `x=0`/`y=0` planes also corner-
// clip `A`'s flat BOTTOM quad and the TWO side walls over the two `Q` edges they cross —
// AND two box CAP faces (the `x=0`/`y=0` planes bounded inside `A`) must be synthesized,
// then the whole shell welded across MULTIPLE junction vertices (`J` on the wall, `J'` on
// the bottom, the wall/plane pierce points) and self-verified (watertight + closed-form
// volume). That multi-face family assembly is beyond this slice, so the entry point
// returns a NULL `Shape` → OCCT `BRepAlgoAPI_*`, recording `MultiFaceWeldUnreachable`.
//
// NO wrong/leaky/partial solid is EVER emitted; NO tolerance is weakened; the seam-graph
// builder AND the junction-aware wall split are REAL (independently proven by the host
// gate) and CONSUMED here (their success + each per-arc B2 split gate the caller's log).
//
// ── CONSUMES (byte-identical) ─────────────────────────────────────────────────────
// B1 `recogniseFreeformSolid`, `buildSeamGraph` (`seam_graph.h`), B2 `splitFace`,
// `splitFaceJunction` (`junction_split.h`, additive sibling of B2), and the landed
// `hscdetail::`/`isdetail::` primitives. Edits none of them, nor the landed single-seam
// `two_operand.h` / `inter_solid_seam.h` path.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_MULTI_SEAM_H
#define CYBERCAD_NATIVE_BOOLEAN_MULTI_SEAM_H

#include "native/boolean/face_split.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/junction_split.h"
#include "native/boolean/seam_graph.h"
#include "native/topology/native_topology.h"

#include <cstddef>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;

/// The requested multi-seam two-operand operator.
enum class MultiSeamOp { Fuse, Cut, Common };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight multi-seam result solid is returned.
enum class MultiSeamDecline {
  Ok,
  NotAdmittedA,             ///< B1 declined operand `A`
  SeamGraphDeclined,        ///< `buildSeamGraph` declined (see `subDecline` for the reason)
  ArcSplitFailed,           ///< a single-arc B2 split of the wall failed (graph not B2-consistent)
  WallJunctionSplitFailed,  ///< the junction-aware wall split declined (unexpected for a valid pose)
  MultiFaceWeldUnreachable  ///< wall split LANDED; the multi-face corner-clip weld is the next blocker
};

inline const char* multiSeamDeclineName(MultiSeamDecline d) noexcept {
  switch (d) {
    case MultiSeamDecline::Ok: return "Ok";
    case MultiSeamDecline::NotAdmittedA: return "NotAdmittedA";
    case MultiSeamDecline::SeamGraphDeclined: return "SeamGraphDeclined";
    case MultiSeamDecline::ArcSplitFailed: return "ArcSplitFailed";
    case MultiSeamDecline::WallJunctionSplitFailed: return "WallJunctionSplitFailed";
    case MultiSeamDecline::MultiFaceWeldUnreachable: return "MultiFaceWeldUnreachable";
  }
  return "?";
}

/// A measured record of a multi-seam attempt (for the caller's log / the host gate).
/// Populated on every call; `decline == Ok` never happens in this slice (the multi-face
/// weld is not yet reachable), but the fields prove HOW FAR the assembly progressed — in
/// particular that the junction-aware WALL SPLIT now LANDS (the prior wave's blocker).
struct MultiSeamReport {
  MultiSeamDecline decline = MultiSeamDecline::Ok;
  SeamGraphDecline subDecline = SeamGraphDecline::Ok;   ///< the seam-graph reason (if declined there)
  bool graphBuilt = false;                              ///< `buildSeamGraph` succeeded
  bool arcsSplitOk = false;                             ///< both single arcs split the wall via B2
  bool wallJunctionSplitOk = false;                     ///< the junction-aware wall split LANDED
  JunctionDecline junctionDecline = JunctionDecline::Ok;///< `splitFaceJunction` verdict (Ok when landed)
  int junctionCrossings = -1;                           ///< the joined seam's boundary-crossing count
  double junctionTilingGap = 0.0;                       ///< the junction-split tiling gap (≈ 0)
  double junctionRebuildResidual = 0.0;                 ///< junction-aware reflatten residual (≈ 0)
  double cornerArea = 0.0;                              ///< the removed-quadrant UV area (vs the oracle)
  double junctionPlaneResidual = 0.0;                   ///< |signedDist(Pk, J)| — J lies on both planes
};

// ─────────────────────────────────────────────────────────────────────────────
// freeformBooleanMultiSeam — the entry point. `A` is a freeform-walled solid, `B` a
// finite all-planar solid (a corner box) whose two adjacent faces slice `A`'s wall.
// Returns the self-verified multi-seam FUSE / CUT / COMMON solid, or a NULL `Shape`
// (→ OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`) with a MEASURED decline. Never emits a
// leaky/overlapping/wrong-volume solid; never weakens a tolerance.
//
// For this wave the junction-aware WALL SPLIT lands (the corner + L-shaped survivor at
// the exact valence-3 vertex J), but the full multi-FACE corner-clip weld (bottom quad +
// two side walls also cut, two box cap faces synthesized, multi-junction shell weld) is
// not yet assembled, so a valid pose returns NULL with `MultiFaceWeldUnreachable` after
// building + proving the seam graph AND the junction-aware wall partition.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformBooleanMultiSeam(const topo::Shape& A, const topo::Shape& B,
                                            MultiSeamOp /*op*/, double /*deflection*/ = 0.01,
                                            MultiSeamReport* report = nullptr) {
  MultiSeamReport rep;
  auto fail = [&](MultiSeamDecline d) -> topo::Shape {
    rep.decline = d;
    if (report) *report = rep;
    return {};
  };

  OperandDecline b1 = OperandDecline::Ok;
  const auto opA = recogniseFreeformSolid(A, &b1);
  if (!opA) return fail(MultiSeamDecline::NotAdmittedA);

  SeamGraphDecline sgd = SeamGraphDecline::Ok;
  const auto graph = buildSeamGraph(*opA, B, &sgd);
  rep.subDecline = sgd;
  if (!graph) return fail(MultiSeamDecline::SeamGraphDeclined);
  rep.graphBuilt = true;
  rep.junctionPlaneResidual = graph->junctionPlaneResidual;

  // The two single-seam partitions the junction weld composes: EACH full arc must split
  // the wall through the byte-frozen B2 (proves the graph is B2-consistent).
  const OperandFace& wall = opA->faces[opA->freeform.front()];
  const SplitResult s0 = splitFace(wall.face, graph->arcs[0].arc);
  const SplitResult s1 = splitFace(wall.face, graph->arcs[1].arc);
  if (!s0.ok() || !s1.ok()) return fail(MultiSeamDecline::ArcSplitFailed);
  rep.arcsSplitOk = true;

  // The junction-aware WALL SPLIT (the prior wave's named enabler, now LANDED): partition
  // the wall into the corner sub-face (`A ∩ {x≥0, y≥0}`) + the L-shaped survivor at the
  // EXACT shared valence-3 vertex J. Where the byte-frozen B2 declines `RebuildMismatch`
  // (its fixed-density reflatten shortcuts the kink at J), `splitFaceJunction` makes J an
  // edge endpoint so each straight-in-UV half reflattens to machine precision under the
  // SAME strict rebuild tolerance (never weakened).
  const JunctionSplitResult js =
      splitFaceJunction(wall.face, graph->jointSeam, graph->junctionUV, graph->junction3d);
  rep.junctionDecline = js.decline;
  rep.junctionCrossings = js.crossings;
  rep.junctionTilingGap = js.tilingGap;
  if (!js.ok()) return fail(MultiSeamDecline::WallJunctionSplitFailed);
  rep.wallJunctionSplitOk = true;
  rep.junctionRebuildResidual = js.split->rebuildResidual;
  rep.cornerArea = js.split->areaCorner;

  // The wall now partitions at the exact valence-3 J. The FULL multi-seam result solid
  // additionally needs the OTHER cut faces (the corner box straddles A's footprint quad,
  // so the x=0/y=0 planes also corner-clip A's flat BOTTOM and the TWO side walls over the
  // Q edges they cross), the two box CAP faces synthesized inside A, and the whole shell
  // welded across MULTIPLE junctions + self-verified (watertight + closed-form volume).
  // That multi-face family assembly is the measured NEXT blocker → NULL → OCCT.
  return fail(MultiSeamDecline::MultiFaceWeldUnreachable);
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_MULTI_SEAM_H
