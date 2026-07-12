# Proposal — nurbs-thicken-shell-trim

## Why

The landed Layer-5 solid thicken (`src/native/math/bspline_thicken.{h,cpp}`,
`thickenSurface`) and multi-face shell (`src/native/math/bspline_shell.{h,cpp}`,
`thickenPatches`) assemble a CLOSED, watertight solid from a face's original cap, its offset
cap `O = S + d·N`, and ruled side walls / mitred corners. Both DECLINE wholesale when
thickening a high-curvature or self-approaching input:

1. **Single-face interpenetration.** When a face's inward/large offset FOLDS over PART of the
   domain, the offset cap crosses back through the base — a self-intersecting, non-manifold
   solid. `thickenSurface` declines the whole request (`SelfIntersection`) rather than
   keeping the valid remainder. This is the same fold-locus the Wave-E offset layer already
   detects and trims (`offsetSurfaceTrimmed`, Jacobian factors `1 + d·κᵢ`).

2. **Adjacent-slab overlap.** At a SHARP dihedral corner the two mitred slabs can OVERLAP:
   the extend-to-meet mitre apex spikes past the faces' own extent and the mitre panel
   interpenetrates the offset caps. `thickenPatches` builds this overlapping mitre (watertight
   by vertex-weld construction, but geometrically self-intersecting).

Both are documented residuals ("ROBUST self-intersecting-shell recovery — trim the fold
rather than decline") in `bspline_thicken.h` / `bspline_shell.h` and `docs/NURBS-SCOPE.md`.

## What changes

- **Additive API** in `bspline_thicken.h` (existing `thickenSurface` byte-unchanged):
  - `thickenTrimmed(S, d, tol, gridU, gridV)` — when the offset would INTERPENETRATE (fold
    back through `S`) over PART of the domain, reuses the Wave-E `offsetSurfaceTrimmed`
    fold-analysis to find the maximal fold-free parameter rectangle, then assembles the
    closed six-panel (2 caps + 4 walls) shell over just that sub-domain — re-closing it
    watertight and self-intersection-free. BYTE-IDENTICAL to `thickenSurface` when nothing
    interpenetrates (passthrough no-op). HONEST-DECLINES only when no fold-free region
    remains. Reports the TRUE trimmed volume of the kept region.
  - `ThickenResult` gains additive fields (`trimmed`, `keptU0/1`, `keptV0/1`) — default-valued
    so existing callers are unaffected.

- **Additive API** in `bspline_shell.h` (existing `thickenPatches` byte-unchanged):
  - `shellTrimmed(faces, adjacency, d, tol, gridU, gridV, weldTol)` — detects adjacent-slab
    overlap at each dihedral seam with a slab-vs-slab test (the extend-to-meet apex extension
    `|d|/(1 + nA·nB)` versus the faces' extent from the seam), re-closes an overlapping seam
    as a CLEAN bisector-chamfer mitre (whose ridge lies between the two offset caps and cannot
    spike out), and VERIFIES the finished solid is self-intersection-free (no two non-adjacent
    triangles pierce) before returning it valid. BYTE-IDENTICAL to `thickenPatches` when no
    seam overlaps (passthrough). HONEST-DECLINES (`SelfIntersecting`) when a DEEP
    cap-through-cap interpenetration cannot be re-closed to a clean solid.
  - `ShellResult` gains additive fields (`trimmed`, `trimmedSeams`, `selfIntersectionFree`);
    `ShellStatus` gains `SelfIntersecting` — additive, default paths unaffected.

- **No OCCT.** `src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK references in the changed
  files). Both routines remain under `CYBERCAD_HAS_NUMSCI` (they compose the offset fit).

## Impact

- **API (additive only).** No existing signature or ABI byte changes; `cc_*` façade untouched.
  Existing `thickenSurface` / `thickenPatches` outputs are bit-for-bit unchanged (regression:
  the passthrough scenarios assert byte-identity).
- **Tests.** `tests/native/test_native_nurbs_thicken.cpp` and `..._shell.cpp` gain trim
  scenarios: passthrough byte-identity, interpenetration-trimmed watertight +
  self-intersection-free, slab-overlap trimmed to a clean mitre, and honest-decline on a
  fully-degenerate / deep-interpenetration input.

## Honest residual

A DEEP concave-wedge cap-through-cap interpenetration (the two offset caps physically cross,
not merely a mitre-panel overshoot) requires trimming the caps themselves at their
intersection curve — a CSG-scale operation. `shellTrimmed` HONEST-DECLINES those
(`SelfIntersecting`, empty solid) rather than emitting a self-intersecting solid; cap-level
trimming remains a documented residual.
