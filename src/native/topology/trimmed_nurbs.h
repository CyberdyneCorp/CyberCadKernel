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
//     * PINCH-POINT detection + SPLITTING — a loop that self-touches at an interior point is
//       DETECTED (healLoop). A CLEAN 2-way pinch (a figure-eight self-touching at exactly one
//       vertex) is then SPLIT into two independent, region-preserving sub-loops (splitAtPinch:
//       the union of the two sub-loops classifies every point identically to the original
//       pinched loop). A 3+-way / crossing pinch that cannot be cleanly split is DECLINED
//       honestly (never force-split into a different region).
//   healLoop() is the primitive; classify() runs it (opt-in via ClassifyOptions::heal), and
//   splitAtPinch() is the opt-in pinch-resolution step (ClassifyOptions::splitPinch).
//
// SCOPE / RESIDUALS. This is a data-model + robustness slice, not a heavy algorithm:
//   * Healing covers the common near-valid cases above (small gaps, near-coincident
//     vertices, pinch detection) PLUS clean 2-way pinch SPLITTING (splitAtPinch: a
//     figure-eight self-touching at one vertex becomes two region-preserving sub-loops)
//     PLUS GENERAL N-way / crossing pinch splitting (splitAtPinches: a vertex where 3+
//     strands meet, or two pinch points that cross, is decomposed by CCW-adjacency into
//     the union of its simple sub-loops, iterated to a fixpoint — region- and area-
//     preserving) PLUS SEAM-CROSSING HEALING (surfacePeriod / healSeamLoop / classifySeam:
//     a trim loop that wraps a PERIODIC surface's u-seam — a cylinder/cone/sphere/torus — is
//     unwrapped into ONE closed seam-crossing loop with u=0 ≡ u=uPeriod identified, and
//     classified on the seamed domain; a full wrap encloses the whole u-band). A seam-tangent
//     graze, a non-simple wrap, and a CLOSED FREE-FORM surface's (unmodelled) seam are declined
//     honestly. Re-parametrizing a badly-drifting pcurve remains a documented residual; a pinch
//     whose strands do not alternate around the vertex (a non-manifold touch) is still declined.
//   * constructPcurve builds a RATIONAL pcurve when the 3-D edge is rational (a circle /
//     ellipse / rational NURBS): the edge is lifted to homogeneous form, its feet are
//     projected into (u,v), and a WEIGHTED interpolation (interpolateRationalCurve) fits an
//     EXACT rational pcurve — so a circular trim edge's pcurve reproduces the 3-D curve to
//     ~1e-9, where the old polynomial fit had a measurable sag. A non-rational edge still
//     fits non-rationally (the fit is exact there); the residual narrows to weight-unknown
//     rational edges recovered only approximately (Ma–Kruth), reported never faked.
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
  bool splitPinch = false;   ///< OPT-IN: split a clean 2-way pinch (figure-eight) into two
                             ///< sub-loops and classify the union (see splitAtPinch). Default
                             ///< OFF preserves the honest pinch→Unknown decline; when ON a
                             ///< cleanly-splittable pinch classifies (region-preserving). With
                             ///< splitNWay OFF a 3+-way / crossing pinch still declines Unknown.
  bool splitNWay = false;    ///< OPT-IN (requires splitPinch): when the 2-way split declines,
                             ///< fall back to the GENERAL N-way / crossing-pinch resolver
                             ///< (splitAtPinches) — a vertex where 3+ strands meet, or two
                             ///< pinch points that cross, is decomposed into the union of its
                             ///< simple sub-loops (region-preserving). Default OFF: byte-
                             ///< unchanged; a genuinely-ambiguous pinch still declines Unknown.
};

/// Flatten a single trim loop into a closed UV polyline (implicitly closed; consecutive
/// exact-duplicate join vertices dropped). `segsPerSegment` polyline points sample each
/// pcurve segment (higher ⇒ tighter on curved / rational loops — a rational circle/ellipse
/// pcurve flattens with no sag via the homogeneous evaluator). This is the SAME polyline
/// classify() ray-casts; exposed so the parameter-space region boolean (trim_boolean.h) can
/// flatten loops with the identical, seam-consistent evaluator instead of re-deriving one.
std::vector<ParamPoint> flattenTrimLoop(const TrimLoop& loop, int segsPerSegment = 48);

/// Classify (u,v) against the trimmed region. Robust even-odd ray-cast in UV with
/// honest OnBoundary / Unknown handling (see Containment). With opts.heal (default on)
/// each loop is run through healLoop() first: small gaps / near-coincident vertices are
/// welded closed; a genuinely large gap or a self-touching pinch still declines Unknown.
/// With opts.splitPinch (default OFF) a cleanly 2-way pinched outer loop is instead SPLIT
/// into two sub-loops (splitAtPinch) and the point is In iff inside EITHER sub-loop —
/// region-preserving; a 3+-way / crossing pinch still declines Unknown.
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
// PINCH-SPLITTING — resolve a detected 2-way pinch into two valid sub-loops.
//
// healLoop() DETECTS a pinch (a repeated non-adjacent vertex, where a loop self-touches)
// and declines honestly. splitAtPinch() is the next, opt-in step: when a welded loop
// self-touches at EXACTLY ONE pinch vertex — i.e. one interior vertex k that coincides
// (within tol) with exactly one earlier interior vertex i, the two touching only there —
// the loop is a figure-eight and SPLITS cleanly into two independent sub-loops:
//
//     sub-loop A = poly[i .. k]              (the first lobe, closed at the pinch)
//     sub-loop B = poly[k .. n-1] + poly[0 .. i]   (the second lobe, closed at the pinch)
//
// Both sub-loops share only the pinch vertex, so the two lobes are disjoint regions that
// meet at a point. This is REGION-PRESERVING by construction: for a non-crossing
// self-touching loop the original even-odd ray-cast reports a point In iff it is inside
// EITHER lobe (each disjoint lobe contributes an odd crossing count on its own), so the
// UNION of the two split sub-loops classifies every interior/exterior point IDENTICALLY to
// the original pinched loop. A pinch that is NOT a clean 2-way split — three-or-more
// coincident vertices (a 3+-way pinch), or a pinch whose lobes would overlap/self-cross —
// is AMBIGUOUS and DECLINED honestly (split=false): the split is never forced.
//
// This narrows the "general non-manifold repair" residual from "any pinch declines" to
// "only 3+-way / crossing pinches decline"; the common self-touching figure-eight is now
// repaired into a valid multi-loop region rather than thrown away.
// ─────────────────────────────────────────────────────────────────────────────
struct SplitReport {
  bool split = false;        ///< true ⇔ a clean 2-way pinch was found AND split into two valid loops
  bool pinch = false;        ///< true ⇔ a self-touching pinch was detected at all (splittable or not)
  bool ambiguous = false;    ///< true ⇔ a pinch was found but is NOT cleanly 2-way splittable (declined)
  int pinchCount = 0;        ///< number of distinct non-adjacent coincident vertex pairs found
  double tolerance = 0.0;    ///< the scale-relative pinch tolerance actually applied
  std::vector<ParamPoint> loopA;  ///< first sub-loop polyline (valid to ray-cast iff `split`)
  std::vector<ParamPoint> loopB;  ///< second sub-loop polyline (valid to ray-cast iff `split`)
};

/// Split a flattened loop polyline at its pinch vertex into two sub-loops. `poly` is a
/// WELDED polyline (run healWeldGaps first, as healLoop / preparedLoop do). Returns a
/// SplitReport: on a clean 2-way pinch, `split=true` with `loopA` / `loopB` set; a 3+-way
/// or crossing pinch sets `ambiguous=true` and `split=false` (honest decline); a loop with
/// no pinch sets `pinch=false`, `split=false`. Region-preserving (see above). `opts.gapTol`
/// scales the pinch-coincidence tolerance exactly as healLoop's weld tolerance.
SplitReport splitAtPinch(const std::vector<ParamPoint>& poly, const HealOptions& opts = {});

/// Flatten `loop` (join-gap-aware), weld small gaps, then split at a clean 2-way pinch.
/// Returns the SplitReport with the two healed sub-loop polylines (valid iff `split`). Use
/// to turn a self-touching TrimLoop into two ray-castable loops. A large gap, a degeneracy,
/// or a non-2-way pinch declines (`split=false`).
SplitReport splitTrimLoopAtPinch(const TrimLoop& loop, const HealOptions& opts = {},
                                 int flatten = 48);

// ─────────────────────────────────────────────────────────────────────────────
// N-WAY / CROSSING PINCH SPLITTING — the general figure-eight-of-figure-eights.
//
// splitAtPinch handles a CLEAN 2-way pinch. splitAtPinches GENERALIZES it to a loop that
// self-touches at ANY number of vertices, and at a vertex where N≥3 loop strands meet:
//
//   * At each pinch vertex P the loop arrives along several strands and departs along the
//     same number. The correct region-preserving decomposition pairs each INCOMING strand
//     with an OUTGOING strand by ANGULAR (CCW) adjacency around P — the planar-subdivision
//     "next edge in the rotational order" rule. This un-crosses the vertex into locally
//     non-crossing strands and re-routes the loop's successor links so that tracing them
//     yields a set of SIMPLE sub-loops that share only pinch points.
//   * Two pinch points that CROSS (a figure-8-of-figure-8) are resolved by ITERATING the
//     single-vertex resolution to a FIXPOINT: each pass resolves one pinch cluster and the
//     resulting sub-loops are re-fed until none self-touches.
//
// REGION-PRESERVING by the same even-odd argument as splitAtPinch: the CCW-adjacency
// pairing preserves the parity of boundary crossings for every query point, so the UNION of
// the resulting simple sub-loops classifies every interior/exterior point IDENTICALLY to the
// original self-touching loop. Total signed area is preserved (the reconnection only
// re-partitions the same directed edges into cycles). A genuinely AMBIGUOUS configuration —
// a vertex whose incoming/outgoing strands do not alternate around P (an odd, non-manifold
// touch that no CCW pairing resolves into simple loops), or a sub-loop that still self-
// touches after the fixpoint — is DECLINED honestly (ok=false), never force-split.
// ─────────────────────────────────────────────────────────────────────────────
struct MultiSplitReport {
  bool ok = false;        ///< true ⇔ the loop was split into a set of valid SIMPLE sub-loops
  bool pinch = false;     ///< true ⇔ any self-touching pinch was detected
  bool ambiguous = false; ///< true ⇔ a pinch was detected but is NOT cleanly resolvable (declined)
  int pinchVertices = 0;  ///< number of DISTINCT pinch locations resolved (clusters)
  int maxWays = 0;        ///< largest strand-count N at any single resolved pinch vertex
  int iterations = 0;     ///< fixpoint passes taken (>1 ⇔ crossing pinches were present)
  double tolerance = 0.0; ///< the scale-relative pinch tolerance actually applied
  std::vector<std::vector<ParamPoint>> loops;  ///< the simple sub-loops (valid iff `ok`)
};

/// Split a WELDED polyline that self-touches at any number of vertices (each N≥2-way) into a
/// set of SIMPLE sub-loops by CCW-adjacency re-routing, iterated to a fixpoint for crossing
/// pinches. Returns the sub-loops in `loops` (valid iff `ok`). A loop with NO pinch returns
/// ok=true with the single input loop; an unresolvable pinch returns ok=false, ambiguous=true
/// (honest decline). Region- and area-preserving (see above). `opts.gapTol` scales the pinch
/// tolerance exactly as healLoop's weld tolerance.
MultiSplitReport splitAtPinches(const std::vector<ParamPoint>& poly, const HealOptions& opts = {});

/// Flatten `loop` (join-gap-aware), weld small gaps, then split at ALL pinch vertices (N-way /
/// crossing) into simple sub-loops. Returns the MultiSplitReport (valid iff `ok`). A large
/// gap, degeneracy, or unresolvable pinch declines (`ok=false`).
MultiSplitReport splitTrimLoopAtPinches(const TrimLoop& loop, const HealOptions& opts = {},
                                        int flatten = 48);

// ─────────────────────────────────────────────────────────────────────────────
// SEAM-CROSSING HEALING — a trim loop that wraps a PERIODIC surface's parametric seam.
//
// On a periodic / closed surface (a full cylinder, cone, sphere — u ∈ [0, 2π) with
// u=0 ≡ u=2π the same physical curve; a torus — periodic in BOTH u and v) a trim loop
// such as a full cross-section circle CROSSES the u-seam: its (u,v) polyline leaves the
// u=uPeriod boundary and re-enters at u=0 (or the reverse). The always-on classify /
// heal above treat the seam as a HARD boundary, so a wrapped circle is mis-handled —
// split into two open arcs, or classified wrong because a full wrap should enclose the
// WHOLE u-band, not a thin slab.
//
// This slice adds SEAM-AWARE healing, purely additive (the existing entry points are
// byte-unchanged; every seam verb is a NEW function and every seam option DEFAULTS OFF):
//   1. DETECT the surface is periodic (surfacePeriod) and that a loop crosses the seam
//      (loopCrossesSeam — a u-jump of ~one full period between consecutive polyline
//      vertices, or a loop whose unwrapped u-extent is ≥ one full period ⇒ a full wrap).
//   2. STITCH across the seam (healSeamLoop): UNWRAP the loop's u so u=0 and u=1·period
//      are identified — a wrapped circle becomes ONE continuous monotone polyline in
//      unwrapped-u space, i.e. ONE closed seam-crossing loop, not two arcs. The net
//      winding (Δu / period, rounded) records how many times the loop wraps.
//   3. CLASSIFY on the seamed domain (classifySeam): a point's u is reduced modulo the
//      period and the raycast is done against the unwrapped loop — a full wrap encloses
//      the whole u-band (every u classifies In within the v-band); a half-wrap encloses
//      exactly its u-arc.
//   4. HONEST-DECLINE genuinely-ambiguous seam topology: a NON-periodic surface (a plane
//      / open freeform) is a NO-OP (seam=false, byte-identical to the non-seam path); a
//      loop that only TOUCHES the seam tangentially (grazes u=0 without truly crossing)
//      is DECLINED (ambiguous) rather than mis-wrapped into a fabricated full band.
//
// SCALE-RELATIVE and REGION-PRESERVING: the unwrap only ADDS ±period to a u already at
// the seam (an exact identity on the periodic surface — u and u+period are the SAME
// physical point), so no interior/exterior verdict on the cylinder is flipped; a
// non-crossing loop is left byte-identical. A tolerance is NEVER widened.
// ─────────────────────────────────────────────────────────────────────────────

/// Periodicity of a FaceSurface's parameter domain. For the analytic quadrics the
/// u-direction is the angular sweep (period 2π): Cylinder / Cone / Sphere are periodic
/// in u; a Torus is periodic in BOTH u and v. Plane and (this slice) free-form BSpline /
/// Bezier are reported non-periodic — a closed free-form surface's seam is an honest
/// residual, declined rather than guessed.
struct SurfacePeriod {
  bool periodicU = false;   ///< true ⇔ u=0 ≡ u=uPeriod is the same physical curve (a seam)
  bool periodicV = false;   ///< true ⇔ v=0 ≡ v=vPeriod is the same physical curve (a seam)
  double uPeriod = 0.0;     ///< the u-period (2π for the analytic angular sweep); 0 if not periodic
  double vPeriod = 0.0;     ///< the v-period (2π for a torus minor sweep); 0 if not periodic
};

/// Report the parametric periodicity of `surface`. A NON-periodic surface returns
/// periodicU=periodicV=false (the seam-healing path is then a NO-OP — honest decline).
SurfacePeriod surfacePeriod(const FaceSurface& surface) noexcept;

// ─────────────────────────────────────────────────────────────────────────────
// SeamHealReport — the diagnosis + result of unwrapping a seam-crossing loop.
// ─────────────────────────────────────────────────────────────────────────────
struct SeamHealReport {
  bool healed = false;      ///< true ⇔ the loop was successfully unwrapped into one seam-crossing loop
  bool crossesSeam = false; ///< true ⇔ the loop was DETECTED to cross the periodic seam
  bool ambiguous = false;   ///< true ⇔ a seam-tangent / undecidable crossing → honest decline
  bool fullWrap = false;    ///< true ⇔ the loop wraps the FULL u-period (|winding| ≥ 1)
  int winding = 0;          ///< net wraps of the loop around the seam (Δu / period, rounded)
  double period = 0.0;      ///< the u-period applied
  double uSpan = 0.0;       ///< unwrapped u-extent of the loop (max−min of the unwrapped u)
  int seamCrossings = 0;    ///< number of consecutive vertices that jumped ~one full period
  double tolerance = 0.0;   ///< the scale-relative seam-jump tolerance actually applied
  std::vector<ParamPoint> loop;  ///< the UNWRAPPED loop polyline (valid to ray-cast iff healed)
};

/// Does `loop` (already flattened to a UV polyline) cross the u-seam of a period-`period`
/// surface? Detects a consecutive u-jump of ~`period` (a seam crossing) OR an unwrapped
/// u-extent ≥ one full period (a full wrap). Returns false for a loop that stays inside
/// one period (no seam involvement — the plain classify path is correct and used).
bool loopCrossesSeam(const std::vector<ParamPoint>& loop, double period) noexcept;

/// Unwrap a seam-crossing loop into ONE continuous polyline. Walks the flattened polyline
/// and, whenever consecutive u jump by ~±period (a seam crossing), ADDS ∓period to keep u
/// continuous — so a wrapped circle becomes a single monotone-in-u closed loop. Returns a
/// SeamHealReport: `healed` with the unwrapped `loop` on success; `crossesSeam=false` (NO-OP,
/// the input echoed) when the loop does not touch the seam; `ambiguous` (honest decline) for
/// a seam-tangent loop that grazes u=0 without a clean crossing. Region-preserving: the
/// unwrap only ADDS an exact multiple of the period (u and u+period are the same physical
/// point on the periodic surface). `period` must be > 0 (a non-periodic surface never calls this).
SeamHealReport healSeamLoop(const std::vector<ParamPoint>& loop, double period);

/// Flatten `loop` and heal it across the seam of `surface` (if periodic). A non-periodic
/// surface, or a loop that does not cross the seam, is a NO-OP (`crossesSeam=false`, the
/// plain flattened loop echoed) — byte-identical to the non-seam path. `flatten` is the
/// per-segment sample count. Uses the u-period (analytic angular sweep) as the seam period.
SeamHealReport healTrimLoopSeam(const FaceSurface& surface, const TrimLoop& loop, int flatten = 48);

// ─────────────────────────────────────────────────────────────────────────────
// classifySeam — point-in-trimmed-region WITH seam identification.
//
// Identical to classify() for a NON-periodic surface or a loop that does not cross the
// seam (byte-for-byte the same verdict — the seam path is a strict superset that no-ops
// on the common case). For a periodic surface whose outer loop CROSSES the seam, the
// loop is first unwrapped (healSeamLoop); the query point's u is reduced modulo the
// period into the loop's unwrapped u-window before the raycast, so a full wrap correctly
// classifies EVERY u inside the v-band as In (the whole u-band is enclosed), and a
// half-wrap encloses exactly its u-arc. A seam-tangent (ambiguous) loop declines Unknown.
//
// TORUS (periodic in BOTH u and v): the outer loop is checked against BOTH seams.
//   * crosses only the U-seam → the u-unwrap above.
//   * crosses only the V-seam (the minor sweep) → the loop + query point are TRANSPOSED
//     (u↔v swap) into the u-seam frame and classified by the identical proven u-machinery
//     with vPeriod; In/Out/OnBoundary are invariant under a consistent u↔v swap, so the
//     verdict is exact. Holes crossing the v-seam are handled the same way.
//   * crosses BOTH seams at once (a doubly-wrapped torus region) → genuinely ambiguous for
//     a single-axis unwrap → DECLINED honestly (Unknown), never faked (a documented residual).
// ─────────────────────────────────────────────────────────────────────────────
Containment classifySeam(const TrimmedNurbsFace& face, const ParamPoint& p,
                         const ClassifyOptions& opts = {});

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
  bool rational = false;       ///< true ⇔ the constructed pcurve is RATIONAL (weights non-empty)
  PCurve pcurve;               ///< the constructed 2-D B-spline / rational pcurve
  double projMaxDistance = 0.0;///< max ‖projected surface point − edge point‖ (edge-on-S residual)
  FidelityReport fidelity;     ///< round-trip S(pcurve(t)) == C(t) verification
};

struct ConstructOptions {
  int samples = 40;      ///< 3-D edge points to sample+project+fit
  int fitDegree = 3;     ///< degree of the fitted 2-D B-spline (clamped to samples−1)
  int surfSamplesU = 24; ///< closest_point multi-start grid density in U
  int surfSamplesV = 24; ///< closest_point multi-start grid density in V
  bool rational = true;  ///< when the 3-D edge is RATIONAL (a circle/ellipse/rational NURBS),
                         ///< build a RATIONAL pcurve (homogeneous projection + weighted
                         ///< interpolation) so a circular trim edge's pcurve is EXACT, not
                         ///< polygonal. OFF forces the legacy non-rational fit (the documented
                         ///< rational-residual). A non-rational edge always fits non-rationally.
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
