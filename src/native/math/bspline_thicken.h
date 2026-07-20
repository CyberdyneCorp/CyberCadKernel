// SPDX-License-Identifier: Apache-2.0
//
// bspline_thicken.h — NURBS roadmap Layer 5: SOLID THICKEN / SHELL (a NURBS
// surface → a CLOSED, watertight thickened solid).
//
// Given an OPEN tensor-product NURBS surface S(u,v) and a signed thickness d,
// construct the CLOSED SOLID that S bounds when offset by d:
//
//     solid ∂ = S  ∪  O  ∪  (four ruled SIDE WALLS)
//     where  O = offsetSurface(S, d)  is the Layer-5 offset surface (S + d·N),
//     and each side wall is the RULED strip connecting a boundary edge of S to
//     the corresponding boundary edge of O.
//
// The two "cap" panels (S and O) plus the four ruled walls sew into a single
// closed 2-manifold shell with NO boundary edges — a watertight solid. This is
// the construction underneath SHELL / THICKEN / HOLLOW (Shapr3D's shell
// workflow): the offset SURFACE (bspline_offset.h) is one panel; this module
// assembles the offset panel, the original panel, and the ruled walls into the
// closed solid.
//
// OUTPUT REPRESENTATION — a tessellated CLOSED SHELL (`tessellate::Mesh`, the
// kernel's existing triangle-solid carrier from src/native/tessellate/mesh.h:
// fp64 Point3 vertices + indexed Triangles, with `isWatertight` / `enclosedVolume`
// / `boundaryEdgeCount` / `isConsistentlyOriented` verification primitives). The
// exact offset O is NOT a NURBS (its normal carries a square root), so the honest
// closed-solid output for a fitted-offset shell is a tessellation whose closure
// is PROVEN, not asserted by fiat: the panels share EXACT boundary vertices, so
// the shell is watertight BY CONSTRUCTION (every seam edge is used by exactly two
// triangles), and the module verifies χ = 2 / zero boundary edges / consistent
// outward orientation before returning `ok`. A shell that fails closure is
// DECLINED — never returned as a valid solid.
//
// This module sits ABOVE and COMPOSES landed layers, modifying none of them:
//   * bspline_offset.h — the Layer-5 offset surface O = S + d·N (and, crucially,
//     its 2nd-fundamental-form SELF-INTERSECTION (fold) guard: a thicken past a
//     principal radius of curvature DECLINES because its offset panel does).
//   * bspline.h        — surfacePoint / nurbsSurfacePoint / surfaceNormal for the
//     panel tessellation (rational-aware evaluation of S).
//   * bspline_ops.h    — the Layer-1 BsplineSurfaceData carrier for S and O.
//   * tessellate/mesh.h— the closed-shell carrier + watertight/volume checks.
//
// HONEST DEGENERACY GUARDS (never emit a non-closed or self-intersecting solid):
//   * SELF-INTERSECTION — a thicken whose |d| meets/exceeds a principal radius of
//     curvature folds the offset panel onto itself. This is DETECTED by the
//     offset layer (`OffsetStatus::SelfIntersection`) and the thicken DECLINES
//     with `ThickenStatus::SelfIntersection` — it is NOT returned folded.
//   * DEGENERATE / NON-ORIENTABLE — a patch whose normal is near-null anywhere
//     (no defined offset direction), or a degenerate/malformed input, DECLINES.
//   * NOT-CLOSED — if the assembled shell is (unexpectedly) not watertight, the
//     module DECLINES rather than returning an open or leaky solid.
//
// SCOPE — a SINGLE non-rational NURBS patch thickened into a closed six-panel
// (2 caps + 4 walls) box-topology shell. The input S may be rational (its weights
// are honoured through nurbsSurfacePoint / surfaceNormal), but the offset panel is
// fitted non-rationally (bspline_offset). ROBUST self-intersecting-shell recovery
// (trim the fold rather than decline), MULTI-FACE shells (thickening a whole
// multi-patch B-rep with mitred/rounded corners), and a rational offset RESIDUAL
// are documented residuals for later slices — this module never fakes them. See
// docs/NURBS-SCOPE.md Layer-5 row.
//
// GUARD — the offset-bearing routine is compiled only when CYBERCAD_HAS_NUMSCI is
// defined (it composes offsetSurface, whose fit is the sole linear-algebra
// dependency), exactly like bspline_offset.cpp / bspline_fit.cpp. With the guard
// OFF the implementation TU is inert and the function is absent; the declaration
// remains visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_THICKEN_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_THICKEN_H

#include "bspline_ops.h"               // BsplineSurfaceData (Layer-1 data type)
#include "native/tessellate/mesh.h"    // tessellate::Mesh (closed-shell carrier + checks)

#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Thicken result.
// ─────────────────────────────────────────────────────────────────────────────

/// Why a thicken request was declined (`ok == false`).
enum class ThickenStatus {
  Ok = 0,                 ///< a valid CLOSED watertight solid was produced.
  DegenerateInput,        ///< malformed / empty / degree < 1 input surface.
  DegenerateNormal,       ///< the surface has a near-zero normal somewhere (no offset dir).
  SelfIntersection,       ///< |d| meets/exceeds a principal radius of curvature (offset folds).
  OffsetFailed,           ///< the underlying offset surface could not be fitted (see offset layer).
  NotClosed,              ///< the assembled shell is not watertight (declined, never returned open).
  ZeroThickness,          ///< |d| below the linear tolerance — no solid to build.
};

/// Result of a solid-thicken construction. On `ok` the `solid` is a CLOSED,
/// watertight, consistently-oriented triangle shell (χ = 2, no boundary edges).
struct ThickenResult {
  bool ok = false;                          ///< true ⇔ a closed watertight solid within tol.
  ThickenStatus status = ThickenStatus::DegenerateInput;
  tessellate::Mesh solid;                   ///< the closed thickened shell (empty on decline).

  // ── Closure invariants (all verified before `ok` is set) ──
  bool watertight = false;                  ///< isWatertight(solid): every edge used exactly twice.
  bool consistentlyOriented = false;        ///< isConsistentlyOriented(solid): coherent outward wind.
  std::size_t boundaryEdges = 0;            ///< boundaryEdgeCount(solid); 0 ⇔ closed.
  int eulerCharacteristic = 0;              ///< V − E + F; 2 for a closed genus-0 shell.

  // ── Geometry metrics ──
  double enclosedVolume = 0.0;              ///< signed enclosed volume (divergence theorem).
  double surfaceAreaMid = 0.0;             ///< area of the original panel S (the mid-surface area).
  double offsetError = 0.0;                 ///< the offset panel's achieved deviation from S + d·N.
  double minCurvatureRadius = 0.0;          ///< smallest principal radius of curvature on the
                                            ///< offsetting side (the self-intersection bound; 0 flat).
  int gridU = 0, gridV = 0;                 ///< tessellation resolution per direction on each panel.

  // ── Additive fields (default-valued; the non-trim thickenSurface path never sets
  //    them, so existing callers see the historical struct values) ──
  bool trimmed = false;        ///< true ⇔ the interpenetrating region was TRIMMED away and the
                               ///< solid built over just the fold-free sub-domain (thickenTrimmed
                               ///< only). false ⇒ the full domain was used (no interpenetration).
  double keptU0 = 0.0, keptU1 = 0.0;  ///< the kept (fold-free) parameter rectangle in the INPUT
  double keptV0 = 0.0, keptV1 = 0.0;  ///< surface's (u,v) coords; equals the full domain when
                                      ///< `trimmed == false`. The complement is the interpenetrating
                                      ///< region cut away so the solid is self-intersection-free.
};

// ─────────────────────────────────────────────────────────────────────────────
// Solid thicken / shell.
// ─────────────────────────────────────────────────────────────────────────────

/// THICKEN the open NURBS surface `surface` by signed distance `d` into a CLOSED,
/// watertight solid (NURBS-SCOPE Layer 5). The solid is bounded by the original
/// surface S, its offset surface O = S + d·N (from `offsetSurface`), and four
/// ruled side walls connecting their boundary loops. Positive `d` thickens along
/// +N (the surface normal ∂S/∂u × ∂S/∂v, normalized), negative along −N.
///
/// Algorithm:
///   1. Guard the input (well-formed, degree ≥ 1, |d| above tolerance).
///   2. Construct the OFFSET panel O = `offsetSurface(S, d, tol)`. This carries the
///      HONEST guards: a degenerate normal → decline (DegenerateNormal); a |d| past
///      a principal radius of curvature → the offset FOLDS → decline
///      (SelfIntersection). A thicken NEVER returns a folded solid.
///   3. Tessellate S and O on a SHARED (nu × nv) parameter grid into two cap panels;
///      build four ruled side-wall strips joining S's four boundary edges to O's,
///      REUSING the exact shared boundary vertices so every seam edge is used by
///      exactly two triangles (watertight by construction). Orient every panel so
///      the shell's outward normal is consistent (the volume sign is positive).
///   4. VERIFY closure: isWatertight (χ = 2, zero boundary edges) and
///      isConsistentlyOriented. If either fails → decline (NotClosed) — an open or
///      leaky shell is never returned as a valid solid.
///
/// The closed-form GUARANTEES the host gate checks: the solid is WATERTIGHT; its
/// enclosed volume ≈ (mid-surface area)·|d| for thin |d| and matches the exact
/// closed form for a planar/analytic patch (a flat rectangle thickened by d is a
/// box, volume = area·|d| exactly); the offset panel matches `offsetSurface` at
/// distance |d|; and a thicken past the curvature radius DECLINES (fold guard).
///
/// `tol` is passed through to the offset fit (the target max deviation of the
/// offset panel from the true locus S + d·N). `gridU`/`gridV` set the tessellation
/// resolution of each cap panel (≥ 2 per direction; the wall strips inherit the
/// boundary resolution). Declines (ok=false, empty solid) rather than ever
/// returning a non-closed or self-intersecting solid.
ThickenResult thickenSurface(const BsplineSurfaceData& surface, double d,
                             double tol = 1e-4, int gridU = 24, int gridV = 24);

// ─────────────────────────────────────────────────────────────────────────────
// SELF-INTERSECTION-TRIMMED thicken (additive; existing thickenSurface byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// THICKEN `surface` by signed distance `d`, but when the inward offset would
/// INTERPENETRATE (the offset panel O = S + d·N folds back through S over PART of the
/// domain, producing a self-intersecting / non-manifold solid) TRIM the interpenetrating
/// portion away and return the CLOSED, watertight, self-intersection-FREE solid over the
/// valid (fold-free) region — rather than declining the whole request as thickenSurface
/// does.
///
/// The interpenetration is the SAME fold-locus the offset layer detects: the offset map's
/// principal Jacobian factors (1 + d·κᵢ) go non-positive where the offset crosses a centre
/// of curvature. thickenTrimmed reuses that machinery (offsetSurfaceTrimmed) to find the
/// maximal fold-free axis-aligned parameter rectangle, then assembles the closed six-panel
/// (2 caps + 4 walls) shell over JUST that sub-domain, re-closing the shell so it is
/// watertight and non-self-intersecting. The reported `enclosedVolume` is the TRUE trimmed
/// volume of the kept region, not the naive full-domain value.
///
/// Behaviour contract:
///   * NO INTERPENETRATION — when the offset is fold-free over the WHOLE domain (a gently
///     curved face thickened by a safe |d|), the kept rectangle IS the full domain and the
///     result is BYTE-IDENTICAL to thickenSurface(surface, d, tol, gridU, gridV): the trim
///     path is a no-op (`trimmed == false`, keptU0..keptV1 == full domain).
///   * PARTIAL INTERPENETRATION — the folded sub-region is cut at the self-intersection
///     locus; the solid is rebuilt over the fold-free rectangle, closed and watertight, and
///     `trimmed == true` with the kept rectangle reported. A self-intersecting solid is
///     NEVER returned as valid.
///   * FULLY DEGENERATE — when no fold-free region of meaningful area remains (the whole
///     face interpenetrates), HONEST-DECLINE (SelfIntersection, ok=false, empty solid).
///
/// Same guards as thickenSurface for degenerate input / degenerate normal / zero thickness.
/// `tol`, `gridU`, `gridV` have the same meaning. Declines (ok=false, empty solid) rather
/// than ever returning a non-closed or self-intersecting solid.
ThickenResult thickenTrimmed(const BsplineSurfaceData& surface, double d,
                             double tol = 1e-4, int gridU = 24, int gridV = 24);

// ─────────────────────────────────────────────────────────────────────────────
// MULTI-REGION self-intersection-trimmed thicken (additive; thickenTrimmed byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// THICKEN `surface` by signed distance `d`, recovering a CLOSED watertight solid over EVERY
/// meaningful fold-free parameter rectangle — not just the single maximal one thickenTrimmed
/// keeps.
///
/// When the inward offset would INTERPENETRATE as a BAND crossing the domain (a ridge/valley
/// whose offset folds through S along a strip), the fold-free parameter space splits into TWO
/// (or more) disjoint rectangles on either side of the band. thickenTrimmed keeps only the
/// single largest side and discards the rest — up to ~half the part. thickenMultiTrimmed
/// instead thickens EACH meaningful fold-free rectangle into its own closed six-panel shell
/// (via the SAME offsetSurfaceMultiTrimmed decomposition) and returns them as a set of
/// DISJOINT closed solids — the honest result of thickening a face a fold cuts in two.
///
/// Behaviour contract:
///   * NO INTERPENETRATION — returns a SINGLE solid byte-identical to thickenSurface(surface,
///     d, tol, gridU, gridV) (`trimmed == false`, full-domain kept rectangle).
///   * SINGLE fold-free region — returns a single solid identical to thickenTrimmed.
///   * BAND / MULTIPLE fold-free regions — returns ONE closed watertight solid per region, in
///     descending area order; each individually verified (watertight, χ = 2, zero boundary
///     edges, consistently oriented) before inclusion.
///   * FULLY DEGENERATE — returns an EMPTY vector (honest-decline; a self-intersecting solid
///     is NEVER returned). A degenerate-normal / malformed input likewise returns empty.
///
/// Same guards as thickenSurface for degenerate input / normal / zero thickness. Every
/// returned solid is a valid closed shell; a region that fails closure is dropped, never
/// returned open. Never widens tolerance; never emits a self-intersecting solid.
std::vector<ThickenResult> thickenMultiTrimmed(const BsplineSurfaceData& surface, double d,
                                               double tol = 1e-4, int gridU = 24,
                                               int gridV = 24);

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-LOCUS-following thicken (additive; every routine above stays byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// THICKEN `surface` by signed distance `d`, trimming to the actual FOLD LOCUS rather than to
/// an axis-aligned rectangle staircase.
///
/// When the inward offset interpenetrates as a DIAGONAL or CURVED band, the fold-free space on
/// each side is a TRIANGULAR / curved-boundary region. thickenMultiTrimmed covers each side
/// with an axis-aligned rectangle inscribed in the triangle — recovering only a fraction of
/// its area (the "staircase" residual). thickenFoldTrim instead thickens EACH fold-free region
/// over the COLUMN-BAND that FOLLOWS the fold locus (via offsetSurfaceFoldTrim's per-u
/// fold-free v-interval), sewing a closed six-panel (2 caps + 4 walls) shell whose two side
/// walls along the fold trace the diagonal / curved fold boundary. The recovered solid volume
/// exceeds the rectangle-staircase solid's on a diagonal-fold fixture. A CLOSED fold loop (a
/// fold disk around a dome crest) leaves ONE fold-free region wrapping around the fold;
/// offsetSurfaceFoldTrim's scanline decomposition splits it into several simple bands
/// (left/right of the loop + below/above it) and EACH becomes its own closed solid here —
/// where the rectangle staircase recovers only inscribed slabs (or nothing).
///
/// Behaviour contract mirrors offsetSurfaceFoldTrim:
///   * FOLD-FREE everywhere — returns a SINGLE solid byte-identical to thickenSurface.
///   * DIAGONAL / CURVED / CLOSED fold — returns ONE closed watertight solid per simple band
///     (a component wrapping a closed fold loop yields several; one whose bands split/merge
///     around several loops also yields the bifurcation ARMS), each `trimmed == true`, in
///     descending area order; each verified (watertight, χ = 2, zero boundary edges,
///     consistently oriented) before inclusion.
///   * FULLY FOLDING / degenerate — returns an EMPTY vector (honest-decline; a self-intersecting
///     solid is NEVER returned). A degenerate-normal / malformed input likewise returns empty.
///
/// Every returned solid is a valid closed shell over a provably fold-free region; a region that
/// fails closure is dropped, never returned open. Additionally, every band shell passes a
/// DISCRETE EMBEDDING guard (triangle-pair piercing scan): at a large enough |d| a near-fold
/// band can tessellate into a shell that is watertight and χ = 2 yet BUCKLED (self-piercing
/// between samples — the node-wise (1 + d·κ) fold guard is necessary but not sufficient for
/// the discrete panels); such a band is SKIPPED, never emitted. Never widens tolerance; never
/// emits a self-intersecting solid. `tol`, `gridU`, `gridV` have the same meaning as
/// thickenSurface.
std::vector<ThickenResult> thickenFoldTrim(const BsplineSurfaceData& surface, double d,
                                           double tol = 1e-4, int gridU = 24, int gridV = 24);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_THICKEN_H
