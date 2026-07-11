# Design — nurbs-solid-thicken

## Placement & conventions

New module `src/native/math/bspline_thicken.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_offset.{h,cpp}`. Reuses `math::Point3` / `Dir3` (`native/math/vec.h`), the evaluators
`surfacePoint` / `nurbsSurfacePoint` / `surfaceNormal` (`bspline.h`), the **Layer-1 data type**
`BsplineSurfaceData`, the **Layer-5 offset** `offsetSurface` (`bspline_offset.h`), and the kernel's
existing **closed-shell carrier** `tessellate::Mesh` (`native/tessellate/mesh.h` — fp64 `Point3`
vertices + indexed `Triangle`s with `isWatertight` / `enclosedVolume` / `boundaryEdgeCount` /
`isConsistentlyOriented` primitives). OCCT-free, fp64, deterministic.

**Not re-exported from `native_math.h`.** `bspline_thicken.h` includes `tessellate/mesh.h`, which
itself includes the `native_math.h` aggregate — re-exporting thicken from the aggregate would create
a circular include (the aggregate → thicken → mesh → aggregate, with `tessellate::Mesh` undefined on
the second pass). Thicken is therefore a HIGHER-level module (it sits above tessellate) and consumers
include it directly, exactly as consumers of the tessellator do. A one-paragraph note in
`native_math.h` records this.

**numsci gate.** Thicken composes `offsetSurface` (whose fit solves through `numerics::lin_solve`),
so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI`, exactly like `bspline_offset.cpp`: the header
declares everything; with the guard OFF the implementation TU is inert and the function is absent.
`CYBERCAD_HAS_NUMSCI` is defined library-wide, so `bspline_thicken.cpp` — in the default `src/native`
glob — sees it when the option is ON.

## Output representation — a tessellated CLOSED SHELL, closure PROVEN not asserted

The exact offset `O = S + d·N` is not a NURBS (its normal carries a square root), so the honest
closed-solid output for a fitted-offset shell is a TESSELLATION whose closure is proven by
construction, not asserted by fiat. This mirrors the kernel's existing native solids: a
`tessellate::Mesh` is exactly what the native solid mesher produces, and it carries the same
watertight / volume verification vocabulary. The shell is built from three panel kinds that share
EXACT boundary vertices:

- the ORIGINAL cap `S(u,v)` on a `(nu × nv)` grid;
- the OFFSET cap `O = S + d·N` on the SAME grid (the true offset locus — sampling the locus directly,
  not re-evaluating the fitted `offsetSurface`, keeps every offset-cap vertex exactly `|d|` from `S`
  along `N` AND exactly coincident with the wall endpoints);
- four RULED SIDE WALLS, each a quad band joining a boundary edge of the `S` cap to the corresponding
  boundary edge of the `O` cap, REUSING the shared boundary vertex indices.

Because the seam vertices are shared (identical indices, not merely coincident coordinates), every
seam edge is used by exactly one cap triangle and exactly one wall triangle — the shell is watertight
by construction, with no floating-point vertex-welding step to get wrong.

## Why the offset panel is sampled but the fold guard uses the fitted offset

`offsetSurface(S, d, tol)` is called for TWO things: (1) its honest guards — a degenerate normal or a
`|d|` past a principal radius of curvature (fold) DECLINES the offset, and the thicken propagates that
decline (a thicken past the curvature radius returns `SelfIntersection`, never a folded solid); and
(2) the reported achieved offset error + minimum curvature radius. The offset-cap GEOMETRY, however,
is the true locus `S + d·N` sampled on the tessellation grid — this is the ground truth the fitted
`offsetSurface` approximates, so the cap vertices are exactly `|d|` from `S` (the host gate checks
this to ~1e-9) and exactly matched to the walls. The fitted surface's fit error is reported for
transparency; it does not perturb the closed-shell vertices.

## Orientation — coherent winding then outward

Seam-sharing guarantees watertightness regardless of per-panel winding, but the panels may not agree
on a single orientation. A standard BFS across shared edges makes the shell coherently oriented: for
each triangle, any neighbour that traverses the shared edge the SAME way (rather than reversed) is
flipped. A non-manifold seam (≥ 3 triangles on an edge — impossible for this construction, but
checked) cannot be coherently oriented and DECLINES. A single signed-volume sign then fixes global
inside/out: if the enclosed volume is negative, every triangle is reversed (preserving watertightness
+ coherence), giving the positive enclosed volume `enclosedVolume()` needs to be meaningful. Finally
the module VERIFIES `isWatertight` (χ = 2, zero boundary edges) and `isConsistentlyOriented`; a shell
that fails is DECLINED (`NotClosed`).

## Volume oracle

For a flat rectangular patch `[0,Lx]×[0,Ly]` thickened by `d`, the solid is an exact box and the
divergence-theorem `enclosedVolume()` equals `Lx·Ly·|d|` to ~1e-9 (bilinear patch → exact
tessellation). For a curved patch the thin-slab identity `V ≈ A·|d|` (with `A` the mid-surface area)
holds with relative error `O(|d|·curvature)`, so shrinking `|d|` shrinks the discrepancy — a monotone
convergence oracle. For a 90° cylinder wedge thickened outward by `d` (radii `r..r+d`, height `h`),
`V = (π/4)·((r+d)² − r²)·h`, matched to a modest relative bound at the tessellation resolution (the
faceted arc slightly under-integrates the area).

## Guards and honest declines

`ThickenStatus`: `Ok`, `DegenerateInput`, `DegenerateNormal`, `SelfIntersection`, `OffsetFailed`,
`NotClosed`, `ZeroThickness`. The self-intersection and degenerate-normal declines are inherited from
the offset layer's second-fundamental-form guard; `NotClosed` is the construction's own safety net (a
shell that is not watertight / coherently oriented is never returned as valid). In no case is a folded
or non-closed solid returned as valid, and in no case does the routine crash.

## Residuals (documented, never faked)

- **Robust self-intersecting-shell recovery** — trimming the folded region to recover a valid thicken
  (rather than declining) is materially harder; this module DECLINES.
- **Multi-face shells** — thickening a whole multi-patch B-rep with mitred / rounded corners into one
  closed solid is a distinct construction; this module thickens a SINGLE patch into a six-panel
  (2 caps + 4 walls) box-topology shell.
- **Rational offset panel** — inherited from `bspline_offset` (non-rational fit; the input may be
  rational).
