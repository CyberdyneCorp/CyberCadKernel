// SPDX-License-Identifier: Apache-2.0
//
// bspline_shell.h — NURBS roadmap Layer 5: MULTI-FACE SOLID THICKEN / SHELL (a
// small B-rep of 2+ edge-adjacent NURBS faces → ONE closed watertight solid).
//
// bspline_thicken.h thickens a SINGLE open NURBS patch into a closed six-panel
// (2 caps + 4 walls) box shell. This module generalises that to a MULTI-PATCH
// set of edge-adjacent faces: offset EACH face by d, and along a SHARED interior
// edge the two INNER (S) faces MEET DIRECTLY (welded at the shared model edge) —
// there is NO S-to-O side wall on the interior seam. On the OFFSET (O) side, if
// the two faces are coplanar/tangent the offset caps ALSO weld directly (no
// filler); at a DIHEDRAL corner the two offset caps diverge, so the offset-side
// gap is closed by a MITRE strip bridging the two offset edges. Full side walls
// (S→O) are built ONLY on the OUTER boundary of the assembled patch set. The
// result is ONE closed 2-manifold shell (χ = 2, no boundary edges), not two
// adjacent solids sharing a wall.
//
//     ∂(shell) = ⋃ᶠ Sᶠ  ∪  ⋃ᶠ Oᶠ  ∪  (ruled side walls on OUTER boundary edges only)
//     where Oᶠ = offsetSurface(Sᶠ, d) is the Layer-5 offset of face f.
//
// This is the multi-face SHELL / THICKEN of a freeform B-rep: a shelled body
// whose walls appear only on the free boundary, while faces welded across a
// shared model edge join into one continuous inner + one continuous outer skin.
//
// CONSTRUCTION — the shell is assembled by SPATIAL WELD. Every face's S-cap and
// O-cap is sampled on a shared (nu × nv) grid; a shared model edge between two
// faces is sampled to the SAME point sequence on both faces (the adjacency record
// gives the matched edge + orientation), so the two faces' boundary samples are
// coincident. A tolerance-bucketed vertex weld deduplicates coincident samples to
// ONE vertex. Then:
//   * an interior shared S-edge is used by the S-cap triangles of BOTH faces →
//     used twice → interior, NO S→O wall (the two inner faces meet directly);
//   * on the offset side, a shared O-edge either WELDS (coplanar/tangent — the two
//     offset caps meet directly) or, at a dihedral corner, is bridged by a MITRE
//     strip closing the offset-side gap between the two offset edges;
//   * an OUTER boundary edge is used by ONE face's cap → a ruled side wall is
//     built there, joining the S-cap boundary to the O-cap boundary.
// A vertex where 3+ faces/walls meet (a shared model VERTEX) welds to a single
// vertex, so the corner closes with no gap. Every seam edge ends up used by
// exactly two triangles → watertight by construction, then VERIFIED (χ = 2, zero
// boundary edges, coherent orientation) before `ok` is set.
//
// OUTPUT — a tessellated CLOSED SHELL (`tessellate::Mesh`), exactly as
// bspline_thicken: closure is PROVEN, not asserted. A shell that fails closure is
// DECLINED — never returned as a valid solid.
//
// This module sits ABOVE and COMPOSES landed layers, modifying none of them:
//   * bspline_offset.h — the Layer-5 offset surface Oᶠ = Sᶠ + d·N and its
//     2nd-fundamental-form SELF-INTERSECTION (fold) guard (a face thickened past a
//     principal radius of curvature DECLINES the whole shell — no folded panel).
//   * bspline.h        — nurbsSurfacePoint / surfacePoint / surfaceNormal.
//   * bspline_ops.h    — the Layer-1 BsplineSurfaceData carrier for each face.
//   * tessellate/mesh.h— the closed-shell carrier + watertight/volume checks.
//
// HONEST DEGENERACY GUARDS (never emit a non-closed / self-intersecting / double-
// walled shell):
//   * SELF-INTERSECTION — any face whose |d| meets/exceeds a principal radius of
//     curvature folds; DETECTED by the offset layer → the shell DECLINES.
//   * DEGENERATE / NON-ORIENTABLE — a face with a near-null normal, a malformed
//     input, or an adjacency record whose two edges do not sample to coincident
//     points (a bad/inconsistent B-rep) DECLINES.
//   * NOT-CLOSED — an assembled shell that is (unexpectedly) not watertight, or
//     that welds to a NON-MANIFOLD seam (≥ 3 caps on one edge — a fold in the
//     model, not a valid 2-face shared edge), DECLINES rather than returning open.
//
// SCOPE — 2+ NON-RATIONAL-offset edge-adjacent NURBS faces thickened into ONE
// closed shell, side walls on the OUTER boundary only. Faces may be rational (the
// offset panel is fitted non-rationally, inherited from bspline_offset). ROBUST
// self-intersecting recovery (trim the fold rather than decline), and a rational
// offset RESIDUAL, are documented residuals — this module never fakes them. See
// docs/NURBS-SCOPE.md Layer-5 row.
//
// GUARD — compiled only when CYBERCAD_HAS_NUMSCI is defined (it composes
// offsetSurface, whose fit is the sole linear-algebra dependency), exactly like
// bspline_thicken.cpp. With the guard OFF the TU is inert and the function absent;
// the declaration remains visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_SHELL_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_SHELL_H

#include "bspline_ops.h"               // BsplineSurfaceData (Layer-1 data type)
#include "native/tessellate/mesh.h"    // tessellate::Mesh (closed-shell carrier + checks)

#include <cstddef>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Adjacency record for a multi-face B-rep.
// ─────────────────────────────────────────────────────────────────────────────

/// The four parametric boundary edges of a tensor-product patch, keyed for the
/// adjacency record. `U0` is the edge u = u_lo (v varies), `U1` is u = u_hi,
/// `V0` is v = v_lo (u varies), `V1` is v = v_hi.
enum class PatchEdge { U0 = 0, U1 = 1, V0 = 2, V1 = 3 };

/// One SHARED interior edge between two faces of the B-rep. `faceA`/`faceB` index
/// into the `faces` array; `edgeA`/`edgeB` name which parametric boundary edge of
/// each face is shared. `reversed` is true when the two edges run in OPPOSITE
/// parametric directions along the shared curve (so face B's samples must be
/// reversed to line up with face A's) — the usual case for two faces meeting at a
/// seam with consistent outward orientation. The two edges MUST sample to
/// coincident 3-D points on the ORIGINAL (S) surfaces — the shared model edge
/// (the module verifies this; an inconsistent B-rep declines). The OFFSET edges
/// need NOT coincide: coplanar/tangent faces weld their offsets, a dihedral corner
/// gets a mitre strip bridging the two offset edges.
struct SharedEdge {
  std::size_t faceA = 0;
  std::size_t faceB = 0;
  PatchEdge edgeA = PatchEdge::U0;
  PatchEdge edgeB = PatchEdge::U0;
  bool reversed = true;
};

// ─────────────────────────────────────────────────────────────────────────────
// Shell result.
// ─────────────────────────────────────────────────────────────────────────────

/// Why a multi-face shell request was declined (`ok == false`).
enum class ShellStatus {
  Ok = 0,                 ///< a valid CLOSED watertight solid was produced.
  DegenerateInput,        ///< empty face set, malformed face, or malformed adjacency index.
  DegenerateNormal,       ///< a face has a near-zero normal somewhere (no offset dir).
  SelfIntersection,       ///< some face's |d| meets/exceeds a principal radius (offset folds).
  OffsetFailed,           ///< an underlying offset surface could not be fitted.
  AdjacencyMismatch,      ///< a shared-edge record's two edges do not sample to coincident points.
  NonManifold,            ///< a welded seam carries ≥ 3 caps (a model fold, not a 2-face edge).
  NotClosed,              ///< the assembled shell is not watertight (declined, never returned open).
  ZeroThickness,          ///< |d| below the linear tolerance — no solid to build.
};

/// Result of a multi-face shell construction. On `ok` the `solid` is a CLOSED,
/// watertight, consistently-oriented triangle shell (χ = 2, no boundary edges)
/// spanning ALL input faces with NO interior double-wall.
struct ShellResult {
  bool ok = false;                          ///< true ⇔ a closed watertight solid within tol.
  ShellStatus status = ShellStatus::DegenerateInput;
  tessellate::Mesh solid;                   ///< the closed multi-face shell (empty on decline).

  // ── Closure invariants (all verified before `ok` is set) ──
  bool watertight = false;                  ///< isWatertight(solid): every edge used exactly twice.
  bool consistentlyOriented = false;        ///< isConsistentlyOriented(solid): coherent outward wind.
  std::size_t boundaryEdges = 0;            ///< boundaryEdgeCount(solid); 0 ⇔ closed.
  int eulerCharacteristic = 0;              ///< V − E + F; 2 for a closed genus-0 shell.

  // ── Geometry metrics ──
  double enclosedVolume = 0.0;              ///< signed enclosed volume (divergence theorem).
  double surfaceAreaMid = 0.0;              ///< Σ per-face mid-surface (S) area (the total mid area).
  double maxOffsetError = 0.0;              ///< worst per-face offset panel deviation from S + d·N.
  double minCurvatureRadius = 0.0;          ///< smallest principal radius seen across all faces.
  std::size_t interiorSharedEdges = 0;      ///< count of shared-edge records welded WITHOUT a wall.
  std::size_t wallEdges = 0;                ///< count of OUTER boundary grid-edges given a side wall.
  std::size_t mitreEdges = 0;               ///< count of interior grid-segments bridged by a MITRE
                                            ///< strip (a dihedral shared edge whose offset caps do
                                            ///< NOT weld — the mitre closes the offset-side gap; 0
                                            ///< for a coplanar/tangent seam where the offsets weld).
  int gridU = 0, gridV = 0;                 ///< per-face tessellation resolution per direction.
};

// ─────────────────────────────────────────────────────────────────────────────
// Multi-face solid thicken / shell.
// ─────────────────────────────────────────────────────────────────────────────

/// THICKEN a set of edge-adjacent NURBS `faces` by signed distance `d` into ONE
/// CLOSED, watertight solid (NURBS-SCOPE Layer 5, multi-face). Each face is offset
/// by `d` (via `offsetSurface`, inheriting its fold + degenerate guards). Along
/// every SHARED interior edge named in `adjacency` the two offset faces MEET
/// DIRECTLY — no side wall. Side walls are built only on the OUTER boundary edges
/// (grid-edges that no adjacency record claims). The whole set assembles into a
/// single closed 2-manifold shell.
///
/// Algorithm:
///   1. Guard the inputs (≥ 1 well-formed face, degree ≥ 1, |d| above tolerance,
///      every adjacency index in range).
///   2. Offset each face O = offsetSurface(Sᶠ, d, tol); a degenerate normal or a
///      self-intersecting (fold) offset propagates as a decline — never a folded
///      panel. Record the worst offset error and smallest curvature radius.
///   3. Sample each face's S-cap and O-cap on a shared (nu × nv) grid. Emit S-cap
///      and O-cap triangles for every face into ONE mesh, then WELD coincident
///      vertices (tolerance bucket). A shared MODEL edge, sampled to the same S
///      points on both faces, welds — so the interior S-edge is used by two S-caps
///      (no S→O wall). VERIFY each adjacency record's two S-edges coincide (else
///      AdjacencyMismatch). On the offset side the shared O-edge either welds
///      (coplanar/tangent) or, at a dihedral corner, is bridged by a MITRE strip
///      closing the offset-side gap between the two offset edges.
///   4. Build a ruled S→O side wall on every OUTER boundary grid-edge (an S-cap
///      edge used by exactly one triangle after the weld), joining the S boundary
///      to the O boundary and reusing the welded boundary vertices.
///   5. Orient coherently (BFS across shared edges; a ≥ 3-cap seam → NonManifold
///      decline), fix global inside/out by the signed-volume sign, then VERIFY
///      closure (isWatertight, χ = 2, zero boundary edges, isConsistentlyOriented).
///      A shell that fails closure is DECLINED (NotClosed) — never returned open.
///
/// The closed-form GUARANTEES the host gate checks: the solid is WATERTIGHT with
/// NO interior double-wall (the shared edge carries no wall); its enclosed volume
/// equals Σ (per-face area·|d|) minus the shared-edge double-count for a flat
/// multi-patch (two coplanar rectangles sharing an edge, thickened = one box,
/// exact); and a thicken past any face's curvature radius DECLINES.
///
/// `tol` is passed through to each offset fit. `gridU`/`gridV` set the per-face
/// tessellation resolution (≥ 2 per direction). `weldTol` is the absolute distance
/// under which two sampled vertices are treated as the same welded vertex (default
/// scales with the model; a shared edge must sample within it). Declines (ok=false,
/// empty solid) rather than ever returning a non-closed / double-walled /
/// self-intersecting solid.
ShellResult thickenPatches(const std::vector<BsplineSurfaceData>& faces,
                           const std::vector<SharedEdge>& adjacency, double d,
                           double tol = 1e-4, int gridU = 24, int gridV = 24,
                           double weldTol = 1e-7);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_SHELL_H
