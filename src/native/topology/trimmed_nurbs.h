// SPDX-License-Identifier: Apache-2.0
//
// trimmed_nurbs.h — NURBS roadmap Layer 8: the trimmed-NURBS B-rep data model +
// pcurve robustness. The foundational prerequisite for the eventual Layer-3
// exact-NURBS B-rep boolean (docs/NURBS-SCOPE.md §4 Layer-8 row).
//
// WHAT A TRIMMED NURBS FACE IS
// ─────────────────────────────
// A trimmed NURBS face is a surface S(u,v) (analytic Plane/Cylinder/Cone/Sphere/
// Torus, or a free-form BSpline/Bezier grid — a topology::FaceSurface) TOGETHER
// with trimming LOOPS that bound the valid region. Each loop is an ordered list of
// PCURVES — 2-D curves in the surface's (u,v) parameter plane (topology::PCurve).
// The OUTER loop bounds the kept region; each INNER (hole) loop bounds a removed
// region. The face's valid domain is: inside the outer loop AND outside every hole.
//
// This header is the ROBUST operations layer ON TOP of the existing shape.h data
// model (FaceSurface + PCurve + EdgePCurve are ALREADY the storage). It adds NOTHING
// to shape.h and changes no existing consumer; it provides three things a NURBS
// B-rep boolean (L3) needs and that the mesher-oriented tessellate/trim.h does NOT
// promise (that layer's point-in-polygon deliberately classifies boundary points
// "either way", which a boolean cannot tolerate):
//
//   1. TrimmedNurbsFace  — a self-contained (surface + location + outer/hole loops)
//      aggregate, buildable from an existing topology face Shape (reusing its stored
//      pcurves) or assembled directly. Stores enough to answer "is (u,v) inside?".
//
//   2. classify(u,v) → {In, Out, OnBoundary, Unknown} — HONEST even-odd ray-cast in
//      PARAMETER space with robust on-edge / vertex-hit / degenerate handling. A point
//      within `onEdgeTol` of any loop edge is OnBoundary; a ray that grazes a loop
//      vertex or a degenerate/empty/open loop returns OnBoundary / Unknown rather than
//      a fabricated In/Out verdict (mirrors the kernel's honest-decline discipline).
//
//   3. pcurveFidelity(edgeCurve, pcurve, ...) — verify S(pcurve(t)) == C(t) on a dense
//      sample within a scale-relative tolerance, reporting the max deviation; a pcurve
//      NOT on S is DETECTED (large deviation flagged), never passed. And (numsci-gated)
//      constructPcurve — CONSTRUCT a pcurve for a 3-D edge curve lying on S by
//      projecting sampled points to (u,v) via numerics::closest_point_on_surface and
//      fitting a 2-D B-spline (bspline_fit), then round-trip-verifying fidelity.
//
// The pcurve FIDELITY property (S(pcurve(t)) == C(t)) is the load-bearing robustness
// invariant: without it a NURBS B-rep boolean's shared seams crack. It is the same
// seam-weld contract the tessellator relies on (tessellate/trim.h note), promoted here
// to a first-class, densely-verified guard with an honest deviation report.
//
// TOLERANT-TOPOLOGY HEALING (this slice adds a bounded first pass).
//   Common near-valid loops are HEALED into valid ones before classification, every heal
//   SCALE-RELATIVE and REGION-PRESERVING (a heal never flips an interior/exterior point's
//   In/Out verdict — see healLoop / HealReport):
//     * GAP CLOSING — consecutive segment endpoints within a scale-relative tol but not
//       coincident are SNAPPED coincident (the loop is welded closed). A loop with a
//       genuinely LARGE gap (beyond tol) still DECLINES (Unknown) — never force-welded.
//     * NEAR-COINCIDENT PCURVE SNAP — two loop vertices within tol are snapped to a shared
//       location so adjacent pcurves share the boundary exactly.
//     * PINCH-POINT detection — a loop that self-touches at an interior point is DETECTED
//       and reported (the healed loop is declined honestly, not silently repaired into a
//       different region).
//   healLoop() is the primitive; classify() runs it (opt-in via ClassifyOptions::heal).
//
// SCOPE / RESIDUALS. This is a data-model + robustness slice, not a heavy algorithm:
//   * Healing covers the common near-valid cases above (small gaps, near-coincident
//     vertices, pinch detection). GENERAL non-manifold repair — resolving a detected
//     pinch by SPLITTING the loop into two valid faces, healing across surface seams,
//     re-parametrizing a badly-drifting pcurve — remains a documented residual: those
//     cases are declined honestly (Unknown / pinch reported), not silently repaired.
//   * constructPcurve is non-rational (bspline_fit is non-rational); a genuinely
//     rational edge is fitted as a non-rational approximation and its true fidelity
//     deviation is reported (never a widened/faked tolerance) — the rational-residual.
//
// OCCT-FREE. Uses ONLY src/native/math + src/native/topology + (guarded)
// src/native/numerics. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_TRIMMED_NURBS_H
#define CYBERCAD_NATIVE_TOPOLOGY_TRIMMED_NURBS_H

#include "native/math/native_math.h"
#include "native/topology/shape.h"

#include <cstdint>
#include <optional>
#include <vector>

namespace cybercad::native::topology {

namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// A point in the surface parameter plane. (2-D; independent of tessellate::UV so
// this header does not depend on the tessellate module.)
// ─────────────────────────────────────────────────────────────────────────────
struct ParamPoint {
  double u = 0.0;
  double v = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// PcurveSegment — one oriented pcurve of a trimming loop, in (u,v).
//
// The 2-D curve is a topology::PCurve (the existing shape.h storage); [first,last]
// is its parameter range and `reversed` flips the traversal direction so the loop
// is traced consistently (mirrors an edge's orientation within a wire).
// ─────────────────────────────────────────────────────────────────────────────
struct PcurveSegment {
  PCurve curve;
  double first = 0.0;
  double last = 1.0;
  bool reversed = false;
};

/// A closed trimming loop = ordered pcurve segments in (u,v). Implicitly closed
/// (the last segment's end joins the first segment's start).
using TrimLoop = std::vector<PcurveSegment>;

// ─────────────────────────────────────────────────────────────────────────────
// Classification verdict of a (u,v) point against the trimmed region.
// ─────────────────────────────────────────────────────────────────────────────
enum class Containment : std::uint8_t {
  In = 0,          ///< strictly inside the outer loop and outside every hole
  Out = 1,         ///< outside the outer loop, or inside a hole
  OnBoundary = 2,  ///< within tolerance of some loop edge
  Unknown = 3,     ///< honest decline: degenerate/open/self-touching loop, or a
                   ///< ray-cast ambiguity (vertex graze) that cannot be resolved
};

// ─────────────────────────────────────────────────────────────────────────────
// TrimmedNurbsFace — surface + placement + outer loop + hole loops.
//
// Built from an existing topology face Shape (reusing its stored pcurves, so a
// STEP-read trimmed B-spline face becomes a TrimmedNurbsFace with no re-derivation)
// or assembled directly for tests / synthesis. All classification is done in the
// surface's (u,v) parameter plane; the surface + location are kept so fidelity /
// construction can evaluate S(u,v) in world coordinates.
// ─────────────────────────────────────────────────────────────────────────────
struct TrimmedNurbsFace {
  FaceSurface surface;      ///< S(u,v)
  Location location{};      ///< world placement of the surface
  TrimLoop outer;           ///< outer trimming loop (bounds the kept region)
  std::vector<TrimLoop> holes;  ///< inner (hole) loops (bound removed regions)

  bool hasOuter() const noexcept { return !outer.empty(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Building a TrimmedNurbsFace from an existing topology Shape.
//
// The face's child 0 wire is the outer loop, the rest are holes. Each edge's
// pcurve ON THIS FACE (topo::pcurveOf, with the single-pcurve fallback) supplies a
// PcurveSegment; the edge's [first,last] range and orientation set the segment's
// bounds/direction. Returns nullopt if the shape is not a face, has no surface, or
// its outer wire yields no usable pcurve (honest decline — no fabricated loop).
// ─────────────────────────────────────────────────────────────────────────────
std::optional<TrimmedNurbsFace> makeTrimmedFace(const Shape& face);

// ─────────────────────────────────────────────────────────────────────────────
// Point-in-trimmed-region classification.
//
// `flattenSegments` sets how many polyline points each pcurve segment is sampled
// into for the raycast (higher = tighter on curved loops). `onEdgeTol` is the
// UV-space distance under which a point is reported OnBoundary; it is scaled by the
// loop's UV extent so it is scale-relative.
// ─────────────────────────────────────────────────────────────────────────────
struct ClassifyOptions {
  int flattenSegments = 48;  ///< polyline samples per pcurve segment
  double onEdgeTol = 1e-9;   ///< relative on-edge band (× loop UV extent)
  bool heal = true;          ///< run the tolerant-topology healing pre-pass (see healLoop)
  double healGapTol = 1e-6;  ///< relative gap-close band (× loop UV extent); see HealOptions
};

/// Classify (u,v) against the trimmed region. Robust even-odd ray-cast in UV with
/// honest OnBoundary / Unknown handling (see Containment). With opts.heal (default on)
/// each loop is run through healLoop() first: small gaps / near-coincident vertices are
/// welded closed; a genuinely large gap or a self-touching pinch still declines Unknown.
Containment classify(const TrimmedNurbsFace& face, const ParamPoint& p,
                     const ClassifyOptions& opts = {});

// ─────────────────────────────────────────────────────────────────────────────
// TOLERANT-TOPOLOGY HEALING — the bounded first pass (see the file header).
//
// healLoop() operates on the FLATTENED loop polyline (the same polyline classify()
// ray-casts), because that is where a gap between two adjacent pcurve segments, or a
// near-coincident pair of vertices, actually shows up. Every heal is SCALE-RELATIVE
// (tolerances × the loop's UV extent) and REGION-PRESERVING: a vertex is only moved by
// less than the gap tolerance, so no interior/exterior test point farther than that
// band from the boundary can change its In/Out verdict — a heal never flips the region.
//
// The three cases, in order:
//   1. GAP CLOSING / vertex snap — if two consecutive polyline vertices (including the
//      implicit closing edge) are within `gapTol` but not exactly coincident, they are
//      merged to their midpoint (welded). This closes a small inter-segment gap AND
//      snaps two near-coincident pcurve endpoints to a shared location.
//   2. LARGE-GAP decline — if, after welding the small gaps, the loop's closing edge (or
//      any inter-vertex step that was NOT a within-tol join) still spans more than
//      `gapTol`, that is a GENUINE gap: healed=false, decline honestly (do NOT force-weld
//      an arbitrarily large gap into a fabricated region).
//   3. PINCH detection — if the welded polyline has a repeated NON-adjacent vertex, the
//      loop self-touches (a pinch); pinch=true is reported and the loop is declined
//      (split-or-decline: this slice declines honestly, it does not fabricate a split).
// ─────────────────────────────────────────────────────────────────────────────
struct HealOptions {
  double gapTol = 1e-6;  ///< relative gap/snap band (× loop UV extent) — welds gaps ≤ this
};

struct HealReport {
  bool healed = false;       ///< true ⇔ the loop is valid (possibly after welding); safe to classify
  bool changed = false;      ///< true ⇔ any vertex was moved/merged by a heal
  bool pinch = false;        ///< true ⇔ a self-touching pinch point was DETECTED (declined honestly)
  bool largeGap = false;     ///< true ⇔ a gap beyond gapTol remained (declined honestly)
  int gapsClosed = 0;        ///< number of small gaps / near-coincident pairs welded
  double maxGapClosed = 0.0; ///< largest gap distance actually welded (UV space)
  double residualGap = 0.0;  ///< the remaining (un-welded) gap that forced a large-gap decline
  double tolerance = 0.0;    ///< the scale-relative gap tolerance actually applied
};

/// Heal a flattened loop polyline in place: weld small gaps / near-coincident vertices,
/// decline a genuine large gap, detect a pinch. Returns the report; `poly` is the healed
/// polyline (valid to ray-cast iff report.healed). Region-preserving (see above).
///
/// `joinGaps` (optional) carries the raw gap distance at each segment JOIN (segment k's
/// end → segment k+1's start, plus the closing join). It lets healLoop distinguish a real
/// inter-segment GAP from an ordinary long shape edge: a join gap ≤ tol is welded; a join
/// gap > tol is a GENUINE large gap → largeGap=true, healed=false (honest decline, never
/// force-welded). Pass empty to heal purely from consecutive-vertex proximity.
HealReport healLoop(std::vector<ParamPoint>& poly,
                    const std::vector<double>& joinGaps = {},
                    const HealOptions& opts = {});

/// Flatten `loop` (join-gap-aware) and heal it. Returns the report (the healed polyline is
/// discarded — use when only the diagnosis / heal ops are wanted). This is the entry that
/// distinguishes small inter-segment gaps (welded) from genuine large gaps (declined) and
/// self-touching pinches (reported). `flatten` is the per-segment sample count.
HealReport healTrimLoop(const TrimLoop& loop, const HealOptions& opts = {}, int flatten = 48);

// ─────────────────────────────────────────────────────────────────────────────
// Pcurve fidelity guard — the load-bearing robustness property.
//
// Verify S(p(t)) == C(t) on a dense sample of the shared parameter range: for each
// sample t, evaluate the pcurve p(t) → (u,v), the surface S(u,v) (world-placed),
// and the 3-D edge curve C(t) (world-placed), and measure ‖S(p(t)) − C(t)‖. The
// deviation is compared against a SCALE-RELATIVE tolerance tol = absTol + relTol·L,
// where L is the 3-D length scale of the sampled edge (so a large part is not held
// to an absolute-µm bar it can never meet, and a tiny part is not passed loosely).
// ─────────────────────────────────────────────────────────────────────────────
struct FidelityReport {
  bool ok = false;         ///< true ⇔ maxDeviation ≤ tolerance on every sample
  double maxDeviation = 0.0;   ///< max ‖S(p(t)) − C(t)‖ over the samples
  double meanDeviation = 0.0;  ///< mean deviation
  double tolerance = 0.0;      ///< the scale-relative tolerance actually applied
  int samples = 0;             ///< number of samples evaluated
  double atParam = 0.0;        ///< the t achieving maxDeviation
};

struct FidelityOptions {
  int samples = 64;        ///< dense sample count over [first,last]
  double absTol = 1e-9;    ///< absolute floor of the tolerance
  double relTol = 1e-9;    ///< relative term (× edge length scale)
};

/// Verify the pcurve of an edge is faithful to the edge's 3-D curve on `surface`.
/// `edgeLoc` / `surfLoc` place the edge curve / surface in world coordinates (pass
/// identity if already world). `first`/`last` bound the shared parameter range.
FidelityReport pcurveFidelity(const FaceSurface& surface, const Location& surfLoc,
                              const EdgeCurve& edgeCurve, const Location& edgeLoc,
                              const PCurve& pcurve, double first, double last,
                              const FidelityOptions& opts = {});

// ─────────────────────────────────────────────────────────────────────────────
// Pcurve construction (numsci-gated) — CONSTRUCT a pcurve for a 3-D edge curve
// that lies on S, by projecting sampled edge points to (u,v) via
// numerics::closest_point_on_surface and fitting a 2-D B-spline (bspline_fit), then
// round-trip-verifying fidelity.
//
// Declared unconditionally for documentation; the DEFINITION is compiled only under
// CYBERCAD_HAS_NUMSCI (it depends on the numerics facade + bspline_fit, exactly like
// src/native/math/bspline_fit.cpp). With the guard OFF the function is absent.
// ─────────────────────────────────────────────────────────────────────────────
struct PcurveConstruction {
  bool ok = false;             ///< true ⇔ a pcurve was constructed AND round-trips
  PCurve pcurve;               ///< the constructed 2-D B-spline pcurve
  double projMaxDistance = 0.0;///< max ‖projected surface point − edge point‖ (edge-on-S residual)
  FidelityReport fidelity;     ///< round-trip S(pcurve(t)) == C(t) verification
};

struct ConstructOptions {
  int samples = 40;      ///< 3-D edge points to sample+project+fit
  int fitDegree = 3;     ///< degree of the fitted 2-D B-spline (clamped to samples−1)
  int surfSamplesU = 24; ///< closest_point multi-start grid density in U
  int surfSamplesV = 24; ///< closest_point multi-start grid density in V
  FidelityOptions fidelity{};  ///< round-trip fidelity tolerance
};

/// Construct the pcurve of `edgeCurve` (which is expected to lie on `surface`) by
/// projection + fit, then verify S(pcurve) reproduces C. `u0,u1,v0,v1` bound the
/// surface parameter domain searched by the projection. Returns ok=false honestly
/// when the edge does not lie on S (projection residual too large), the fit fails,
/// or the round-trip fidelity is not met.
PcurveConstruction constructPcurve(const FaceSurface& surface, const Location& surfLoc,
                                   const EdgeCurve& edgeCurve, const Location& edgeLoc,
                                   double first, double last,
                                   double u0, double u1, double v0, double v1,
                                   const ConstructOptions& opts = {});

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_TRIMMED_NURBS_H
