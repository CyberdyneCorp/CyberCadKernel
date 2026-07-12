// SPDX-License-Identifier: Apache-2.0
//
// trim_boolean.h — NURBS roadmap Layer 8: the PARAMETER-SPACE half of the trimmed-
// NURBS B-rep boolean. Given two sets of trimming loops (pcurves) on the SAME surface
// domain (u,v), compute the 2-D boolean of the enclosed regions — Union, Intersect,
// Difference — as a NEW set of trim loops.
//
// WHERE THIS SITS
// ───────────────
// A trimmed-face boolean has two halves. The 3-D half (Layer-3 SSI, src/native/ssi +
// src/native/boolean) intersects the two operand SURFACES and produces the CUT curves,
// then their pcurves on each face (constructPcurve). The PARAMETER-SPACE half — this
// file — takes those pcurves PLUS the operands' original trim loops, all now living in
// ONE face's (u,v) domain, and assembles the trimmed result: it computes, in 2-D, the
// union / intersection / difference of the two enclosed trim REGIONS and emits the
// output loops (correctly nested outer/holes, correctly oriented). It is tractable and
// airtight INDEPENDENT of the 3-D mesher: it is pure planar computational geometry on
// the (u,v) plane, verified against closed-form polygon / circular-lens areas.
//
// So the composition is:
//   L3 SSI  →  cut curves  →  constructPcurve  →  cut pcurves in (u,v)  ┐
//   operand original trim loops (already pcurves in (u,v))              ┼→ trimRegionBoolean
//                                                                       ┘   → result trim loops
// This module is the last box: given the loops (the operands' trims split/joined by the
// SSI cut pcurves at the seam), it does the region arithmetic that yields the trimmed
// face of the boolean result.
//
// METHOD (standard planar region boolean — Greiner–Hormann / arc-walk):
//   1. FLATTEN each loop's pcurve segments into a UV polyline (join-gap-aware, reusing
//      the trimmed_nurbs flatten). Rational pcurves (circles/ellipses) are lifted to
//      homogeneous form by the existing pcurveValue evaluator, so a rational trim edge
//      flattens with no sag.
//   2. INTERSECT the two loops' edges pairwise in (u,v) to find crossing points, and
//      SPLIT both loops at every crossing into arcs. (The pairwise edge intersection is
//      the polyline specialisation of the H1 curve↔curve intersector reused on the
//      lifted pcurves; for the airtight polygonal oracles the crossings are the exact
//      closed-form segment–segment points.)
//   3. CLASSIFY each arc midpoint inside/outside the OTHER region (even-odd ray-cast,
//      the same test trimmed_nurbs::classify uses).
//   4. WALK the arcs to assemble the output loops for the requested op, preserving
//      orientation: an OUTER boundary loop is emitted CCW (positive signed area), a HOLE
//      loop CW (negative signed area).
//
// HONESTY CONTRACT (hard invariant, mirrors the rest of the kernel): a DEGENERATE
// overlap — coincident boundary edges (the two loops share a boundary segment) or a
// TANGENTIAL-only touch (boundaries meet at a point with no transversal crossing) — is
// AMBIGUOUS for a region boolean and is HONEST-DECLINED (status Degenerate, no output),
// NEVER resolved into a fabricated (and possibly wrong) region. No tolerance is ever
// widened to force a crossing.
//
// OCCT-FREE. Uses ONLY src/native/math + src/native/topology (reuses trimmed_nurbs
// flatten + classify + pcurveValue). clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_TRIM_BOOLEAN_H
#define CYBERCAD_NATIVE_TOPOLOGY_TRIM_BOOLEAN_H

#include "native/topology/trimmed_nurbs.h"

#include <cstdint>
#include <vector>

namespace cybercad::native::topology {

// ─────────────────────────────────────────────────────────────────────────────
// The boolean operation on the two trim REGIONS (each region = the area enclosed by
// its outer loop and outside its holes).
// ─────────────────────────────────────────────────────────────────────────────
enum class TrimBoolOp : std::uint8_t {
  Union = 0,       ///< A ∪ B  (points in A OR B)
  Intersect = 1,   ///< A ∩ B  (points in A AND B)
  Difference = 2,  ///< A ∖ B  (points in A AND NOT B)
};

// ─────────────────────────────────────────────────────────────────────────────
// Result status. Mirrors the kernel's honest-decline discipline.
// ─────────────────────────────────────────────────────────────────────────────
enum class TrimBoolStatus : std::uint8_t {
  Ok = 0,          ///< a valid region was computed (loops may be empty ⇒ empty region)
  Empty = 1,       ///< the result region is EMPTY (e.g. Intersect of disjoint inputs)
  Degenerate = 2,  ///< honest decline: coincident-boundary / tangential-only overlap —
                   ///< ambiguous for a region boolean, NOT resolved into a fake region
  Invalid = 3,     ///< an input was malformed (empty / non-closeable / self-touching loop)
};

// ─────────────────────────────────────────────────────────────────────────────
// A result loop, expressed as a closed UV polyline (the arc-walk output). Orientation
// is encoded by signed area: `outer` (CCW, area > 0) vs a hole (CW, area < 0).
//
// The boolean produces loops as polylines (the arcs it walked are polyline arcs). A
// caller that needs pcurve segments back can fit these polylines; the airtight oracles
// operate directly on the polylines (exact vertices + signed area).
// ─────────────────────────────────────────────────────────────────────────────
struct ResultLoop {
  std::vector<ParamPoint> poly;  ///< closed UV polyline (first ≠ last; implicitly closed)
  bool outer = true;             ///< true ⇔ CCW outer boundary; false ⇔ CW hole
  double signedArea = 0.0;       ///< signed area (shoelace); > 0 CCW, < 0 CW
};

struct TrimBoolResult {
  TrimBoolStatus status = TrimBoolStatus::Invalid;
  std::vector<ResultLoop> loops;  ///< output region loops (valid iff status == Ok)
  double area = 0.0;              ///< total signed area of the region (Σ loop areas)

  bool ok() const noexcept { return status == TrimBoolStatus::Ok; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Options controlling the flatten resolution + geometric tolerances.
// ─────────────────────────────────────────────────────────────────────────────
struct TrimBoolOptions {
  int flattenSegments = 256;  ///< polyline samples per pcurve segment (higher ⇒ tighter
                              ///< circular-lens area; polygonal oracles are exact at any N)
  double tol = 1e-10;         ///< relative crossing / coincidence tolerance (× UV extent)
};

// ─────────────────────────────────────────────────────────────────────────────
// trimRegionBoolean — the parameter-space region boolean.
//
// `regionA` / `regionB` are two regions in the SAME surface (u,v) domain, each given as
// an outer loop + hole loops (a TrimmedNurbsFace supplies exactly this — pass its outer
// and holes). `op` selects Union / Intersect / Difference. Returns the result region's
// loops (correctly nested + oriented) and its total area, OR an honest Degenerate /
// Invalid decline (see TrimBoolStatus). Coincident-boundary and tangential-only overlaps
// DECLINE (Degenerate) rather than emit a wrong region.
// ─────────────────────────────────────────────────────────────────────────────
struct TrimRegion {
  TrimLoop outer;               ///< outer loop (bounds the kept region)
  std::vector<TrimLoop> holes;  ///< hole loops (bound removed regions)
};

TrimBoolResult trimRegionBoolean(const TrimRegion& regionA, const TrimRegion& regionB,
                                 TrimBoolOp op, const TrimBoolOptions& opts = {});

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_TRIM_BOOLEAN_H
