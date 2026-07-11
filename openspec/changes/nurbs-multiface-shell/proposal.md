# Proposal — nurbs-multiface-shell (NURBS roadmap Layer 5)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 5 landed the
**surface OFFSET** (`bspline_offset`) and the **single-patch SOLID THICKEN / SHELL**
(`bspline_thicken` — one open NURBS patch → a closed six-panel box shell). Both explicitly list
**multi-face shells** (thickening a whole multi-patch B-rep with mitred corners into ONE closed
solid) as a documented residual.

This change lands that residual as its own bounded slice: **MULTI-FACE SOLID THICKEN / SHELL**. Given
a small B-rep of 2+ edge-adjacent NURBS faces and a signed thickness `d`, offset EACH face by `d` and
assemble ONE closed watertight solid — with the crucial property that along a SHARED interior edge the
two inner (S) faces MEET DIRECTLY (welded at the shared model edge, NO interior double-wall), while
full side walls are built only on the OUTER boundary. On the offset side, coplanar/tangent faces weld
their offsets directly; a dihedral corner is joined by a MITRE (the two offset planes extended to their
meeting corner). This is the shell of a freeform B-rep body: walls only on the free boundary, faces
welded across shared model edges into one continuous inner and one continuous outer skin.

It is worth building **now** because it (a) composes the landed offset + evaluators + the closed-shell
carrier `tessellate::Mesh`, adding only a tolerance-bucketed spatial weld and a mitre join, (b)
inherits the offset layer's honest fold guard for free (any face thickened past its curvature radius
declines the whole shell), and (c) has an **airtight, closed-form oracle**: a 2-patch coplanar pair or
L-shape thickens to ONE closed watertight solid (χ = 2, no interior double-wall), and two coplanar
rectangles sharing an edge thicken to EXACTLY one box (volume = total_area·|d| = Σ per-face area·|d|,
the shared edge not double-counted).

## What

A new OCCT-free module `src/native/math/bspline_shell.{h,cpp}` (namespace `cybercad::native::math`,
beside `bspline_thicken`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, like `bspline_thicken.cpp`) because
it composes `offsetSurface`. It takes a `std::vector<BsplineSurfaceData>` (the faces, which may be
rational) plus a `std::vector<SharedEdge>` adjacency record and produces a CLOSED shell as a
`tessellate::Mesh`.

`thickenPatches(faces, adjacency, d, tol, gridU, gridV, weldTol)`:

1. **Guards first** — ≥ 1 well-formed face, `|d|` above the linear tolerance, every adjacency index in
   range. Offset EACH face `O = offsetSurface(Sᶠ, d, tol)`; a degenerate normal or a self-intersecting
   (fold) offset on ANY face propagates as a decline — the shell NEVER contains a folded panel.
2. **Sample + weld** — sample each face's S-cap and O-cap on a shared `(nu × nv)` grid; a
   tolerance-bucketed spatial weld deduplicates coincident samples, so a shared MODEL edge (sampled to
   the same S points on both faces) collapses to ONE vertex chain and a corner where 3+ faces meet
   welds to one vertex. VERIFY each adjacency record's two S-edges coincide (else `AdjacencyMismatch`).
3. **Mitre the offset side** — where the shared O-edges also weld (coplanar/tangent) the offset caps
   meet directly; at a dihedral corner they diverge and a MITRE strip extends both offset planes to
   their meeting apex, sealing each outer corner to the adjacent walls (a chamfer fallback when no
   finite apex).
4. **Wall the outer boundary only** — a ruled S→O side wall on every OUTER boundary grid-edge (an S-cap
   edge used once after the weld); an interior shared S-edge is used twice → NO wall.
5. **Orient + verify** — coherent winding by BFS across shared edges (a ≥ 3-cap seam → `NonManifold`
   decline), global inside/out by the signed-volume sign, then VERIFY closure (`isWatertight`, χ = 2,
   zero boundary edges, `isConsistentlyOriented`). A shell that fails closure DECLINES (`NotClosed`).

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_shell.cpp`:
  1. **Watertight** — a 2-patch coplanar pair, an L-shaped base+wall pair, and a two-strip curved pair
     each thicken to ONE closed 2-manifold (`isWatertight`, χ = 2, zero boundary edges,
     `isConsistentlyOriented`) at both signs of `d`, with exactly one interior shared edge welded.
  2. **Volume** — two coplanar rectangles sharing an edge thicken to EXACTLY one box: enclosed volume
     == total_area·|d| == Σ per-face area·|d| to ~1e-9 (shared edge not double-counted); the L-shape
     encloses a positive finite volume (its exact union closed form needs face trimming — a residual).
  3. **No interior wall** — the shared interior edge carries NO side wall: the reported wall-edge count
     equals ONLY the outer perimeter, and every offset-side seam edge is used twice (interior) — the
     two offset faces meet directly.
  4. **Fold / degenerate / adjacency guards** — a face thickened past its curvature radius declines the
     whole shell (`SelfIntersection`, empty solid); a degenerate near-null-normal face declines; an
     inconsistent (non-coincident) adjacency record declines (`AdjacencyMismatch`); an out-of-range
     adjacency index and a zero thickness decline.

## Scope

- Adds `src/native/math/bspline_shell.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob.
- Adds `tests/native/test_native_nurbs_shell.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_thicken`.
- Only `#include`s `bspline.h`, `bspline_ops.h` (Layer 1), `bspline_offset.h` (Layer 5), and
  `tessellate/mesh.h` — it does NOT modify them, and does NOT touch `bspline_thicken` (single-patch
  stays as-is) or `bspline_fit` / ssi / blend / boolean / topology.
- **`cc_*` ABI unchanged.** Layer 5 multi-face shell is an internal geometry-algorithm library; no ABI
  is added until a consumer needs it (demand-driven policy).

## Non-goals

- **No robust self-intersecting-shell recovery** — a folded face still DECLINES the whole shell
  honestly (never returns folded geometry). A documented residual.
- **No exact volume for interpenetrating dihedral slabs** — a right-angle L-shape's thin slabs
  interpenetrate near the seam (one face's S-cap passes through the other's slab), so its enclosed
  volume is not the simple union closed form without FACE TRIMMING. The shell is still watertight; the
  EXACT volume oracle is the coplanar (box) case. Face trimming at face–face intersections is a
  documented residual.
- **No rational offset panel** — the offset panel is fitted non-rationally (inherited from
  `bspline_offset`); input faces may be rational.
- No new `cc_*` ABI; no change to STEP admission, the boolean/blend/ssi/topology modules, or any
  evaluator signature.
