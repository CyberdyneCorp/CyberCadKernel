# native-math

## ADDED Requirements

### Requirement: Self-intersecting single-face thicken is trimmed to the valid region

The native math library SHALL provide, additively alongside the existing `thickenSurface`
(which stays byte-unchanged), a routine `thickenTrimmed` in the same OCCT-free,
`CYBERCAD_HAS_NUMSCI`-gated module. When thickening a face by signed distance `d` would cause
the offset cap `O = S + d·N` to INTERPENETRATE (fold back through the base `S`) over PART of
the parameter domain — a self-intersecting, non-manifold solid — `thickenTrimmed` SHALL cut
the interpenetrating portion at the self-intersection (fold) locus and return the CLOSED,
watertight, self-intersection-FREE solid over the remaining valid region, rather than
declining the whole request as `thickenSurface` does.

The fold-free region SHALL be determined by reusing the offset layer's Jacobian-regularity
fold analysis (`offsetSurfaceTrimmed`: principal factors `1 + d·κᵢ` on a dense grid). The
returned solid's `enclosedVolume` SHALL be the TRUE trimmed volume of the kept region, not the
naive full-domain value. `ThickenResult` SHALL carry additive fields `trimmed`,
`keptU0/keptU1/keptV0/keptV1` (default-valued so existing callers are unaffected). It SHALL
honour the same degenerate-input, degenerate-normal, and zero-thickness guards as
`thickenSurface`. It SHALL NEVER return a self-intersecting solid as valid.

#### Scenario: No-interpenetration passthrough is byte-identical to thickenSurface

- GIVEN a gently curved (or flat) face and a safe signed distance `d` whose offset is fold-free over the whole domain
- WHEN the face is thickened with `thickenTrimmed(S, d, tol, gridU, gridV)`
- THEN the result SHALL report `trimmed == false`, the kept rectangle SHALL equal the full parameter domain, and the returned solid mesh SHALL be BYTE-IDENTICAL (vertices and triangle indices) to `thickenSurface(S, d, tol, gridU, gridV)`, with an identical enclosed volume

#### Scenario: A partially interpenetrating thicken is trimmed to a watertight, self-intersection-free solid

- GIVEN a face whose offset folds (interpenetrates) over only PART of the domain, such that `thickenSurface` declines with `SelfIntersection`
- WHEN the face is thickened with `thickenTrimmed`
- THEN the routine SHALL return `ok == true` with `trimmed == true`, and the returned solid SHALL be watertight (closed, zero boundary edges, Euler characteristic `χ = 2`), consistently oriented, and self-intersection-free (no two non-adjacent triangles cross), enclosing a positive volume, with the kept rectangle a strict interior sub-region of the full domain

#### Scenario: A fully-interpenetrating thicken honest-declines

- GIVEN a face whose offset folds over the WHOLE domain (no fold-free region of meaningful area remains)
- WHEN the face is thickened with `thickenTrimmed`
- THEN the routine SHALL DECLINE (`ok == false`) with an empty solid, never returning a self-intersecting solid as valid

### Requirement: Adjacent-slab overlap in a multi-face shell is trimmed to a clean mitre

The native math library SHALL provide, additively alongside the existing `thickenPatches`
(which stays byte-unchanged), a routine `shellTrimmed` in the same OCCT-free,
`CYBERCAD_HAS_NUMSCI`-gated module. When two adjacent mitred slabs at a SHARP dihedral seam
would OVERLAP — the extend-to-meet mitre apex spiking past the faces' own extent so the mitre
panel interpenetrates the offset caps — `shellTrimmed` SHALL TRIM the overlap by re-closing
the offending seam as a clean bisector-chamfer mitre (whose ridge lies between the two offset
caps and cannot spike out) and return a CLOSED, watertight, self-intersection-FREE solid.

Overlap SHALL be detected with a slab-vs-slab test (the apex extension
`|d|/(1 + nA·nB)·‖nA+nB‖` versus each face's extent from the seam). The finished solid SHALL be
VERIFIED self-intersection-free (no two non-adjacent triangles pierce) before it is returned
valid. `ShellResult` SHALL carry additive fields `trimmed`, `trimmedSeams`,
`selfIntersectionFree`, and `ShellStatus` SHALL gain an additive `SelfIntersecting` value
(default paths unaffected). When a DEEP cap-through-cap interpenetration cannot be re-closed to
a clean solid, `shellTrimmed` SHALL honest-decline (`SelfIntersecting`, empty solid) rather
than emit a self-intersecting solid.

#### Scenario: No-overlap passthrough is byte-identical to thickenPatches

- GIVEN a face set whose dihedral seams all mitre cleanly (a coplanar pair, or a right-angle L-shape at a safe thickness)
- WHEN the set is thickened with `shellTrimmed(faces, adjacency, d, …)`
- THEN the result SHALL report `trimmed == false` and `trimmedSeams == 0`, and the returned solid mesh SHALL be BYTE-IDENTICAL to `thickenPatches(faces, adjacency, d, …)`

#### Scenario: An overlapping adjacent-slab mitre is trimmed to a clean watertight solid

- GIVEN an L-shaped two-face set thickened by a distance so large that the extend-to-meet mitre apex overshoots the faces' extent (the two slabs would overlap)
- WHEN the set is thickened with `shellTrimmed`
- THEN the routine SHALL return `ok == true` with `trimmed == true` and at least one re-closed seam, and the returned solid SHALL be watertight (`χ = 2`, zero boundary edges), consistently oriented, and self-intersection-free (verified: no two non-adjacent triangles pierce), enclosing a positive finite volume

#### Scenario: A deep concave-wedge interpenetration honest-declines

- GIVEN a sharp concave two-face wedge (small interior angle) thickened so the two offset caps deeply interpenetrate (a cap-through-cap crossing the mitre choice cannot resolve)
- WHEN the set is thickened with `shellTrimmed`
- THEN the routine SHALL DECLINE (`ok == false`, status `SelfIntersecting`) with an empty solid, never returning a self-intersecting solid as valid
