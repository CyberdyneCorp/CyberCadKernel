# native-math

## ADDED Requirements

### Requirement: NURBS multi-face shell into one closed watertight solid

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_shell.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `thickenPatches` that thickens a set of edge-adjacent
tensor-product NURBS faces by a signed distance `d` into ONE CLOSED, watertight SOLID. Each face is
offset by `d` (via the Layer-5 `offsetSurface`); along every SHARED interior edge named in the
adjacency record the two INNER (original) faces SHALL meet directly (welded at the shared model edge,
with NO interior double-wall), and full ruled SIDE WALLS SHALL be built ONLY on the OUTER boundary of
the assembled face set. On the offset side, coplanar/tangent faces SHALL weld their offset caps
directly and a dihedral corner SHALL be joined by a MITRE (the two offset planes extended to their
meeting corner). The solid SHALL be returned as a triangle shell (`tessellate::Mesh`) assembled so
that all panel kinds share EXACT welded vertices (watertight by construction). Input faces MAY be
rational (weights honoured through `nurbsSurfacePoint` / `surfaceNormal`); the offset panels are
non-rational (inherited from `offsetSurface`).

#### Scenario: A 2-patch coplanar or L-shaped thicken is one closed watertight solid

- GIVEN two edge-adjacent well-formed NURBS faces (a coplanar pair, or an L-shaped base + right-angle wall) and a fold-free signed thickness `d`
- WHEN the shell is constructed and accepted (`ok = true`)
- THEN the returned shell SHALL be a single closed 2-manifold — every undirected edge shared by exactly two triangles (watertight), zero boundary edges, Euler characteristic `χ = 2`, and consistently oriented — for both signs of `d`, with exactly one interior shared edge welded

#### Scenario: The shared interior edge carries no side wall

- GIVEN an accepted 2-patch shell whose two faces share one interior edge
- WHEN the assembled shell is inspected
- THEN the shared interior edge SHALL carry NO S→O side wall (the two inner faces meet directly), the reported wall-edge count SHALL equal only the OUTER perimeter, and every offset-side seam edge SHALL be used by exactly two triangles (interior — the two offset faces meet directly or via the mitre)

#### Scenario: A non-closed or non-manifold assembled shell is declined, never returned

- GIVEN any multi-face thicken whose assembled shell is not watertight, not consistently oriented, or whose welded seam carries three or more caps (a model fold, not a two-face edge)
- WHEN closure is verified
- THEN the routine SHALL return `ok = false` with a not-closed / non-manifold status and no solid — it SHALL NOT return an open, leaky, or double-walled shell as a valid solid

### Requirement: Multi-face enclosed volume matches the closed form for coplanar patches

The enclosed volume of the multi-face shell SHALL be the divergence-theorem volume of its watertight
boundary. For two COPLANAR rectangles that share an edge, thickened by `d`, the solid SHALL be one box
and its volume SHALL equal `total_area·|d|` (equivalently `Σ per-face area·|d|`, the shared edge NOT
double-counted) to ~1e-9, with the reported summed mid-surface area equal to the summed rectangle
area. For a dihedral (e.g. right-angle) face pair the shell SHALL still be watertight and enclose a
positive finite volume; its exact union closed form is NOT claimed (interpenetrating thin slabs need
face trimming — a documented residual).

#### Scenario: Two coplanar rectangles sharing an edge thicken to exactly one box

- GIVEN two coplanar rectangular NURBS patches sharing an edge, of total planar extent `total_area`, and a signed thickness `d`
- WHEN the shell is constructed
- THEN its enclosed volume SHALL equal `total_area·|d|` (== `Σ per-face area·|d|`) to ~1e-9 and its reported summed mid-surface area SHALL equal the summed rectangle area

### Requirement: Multi-face shell inherits the offset fold and degeneracy guards

Any face thickened past a principal radius of curvature folds its offset panel; the module SHALL
detect this through `offsetSurface` and DECLINE the WHOLE shell (`SelfIntersection`, `ok = false`,
empty solid) — it SHALL NOT return a folded panel. A face with a near-null surface normal SHALL
decline; an adjacency record whose two edges do not sample to coincident original-surface points SHALL
decline (`AdjacencyMismatch`); an out-of-range adjacency index and a thickness at or below the linear
tolerance SHALL decline, all without crashing.

#### Scenario: An over-radius or degenerate face declines the whole shell

- GIVEN a face set containing a tightly-curved face and a thickness `|d|` past its minimum radius of curvature, OR a face with a near-null normal
- WHEN the shell is requested
- THEN the routine SHALL return `ok = false` with a self-intersection / degenerate status and an empty solid, never a folded or degenerate shell

#### Scenario: An inconsistent or out-of-range adjacency record declines

- GIVEN an adjacency record naming two edges that do not sample to coincident points, OR an adjacency index outside the face array, OR a zero thickness
- WHEN the shell is requested
- THEN the routine SHALL return `ok = false` with an adjacency-mismatch / degenerate-input / zero-thickness status and no solid, without crashing

### Requirement: Multi-face non-rational-offset scope with recovery/rational/exact-dihedral-volume residuals

The multi-face shell module SHALL produce a CLOSED shell for a small B-rep of 2+ edge-adjacent NURBS
faces, welding shared interior edges (no double-wall) and walling only the outer boundary. It SHALL
NOT claim robust self-intersecting-shell recovery (trimming a folded region rather than declining), a
rational offset panel, or an exact union volume for interpenetrating dihedral slabs (which needs face
trimming at face–face intersections). These are documented residuals recorded in
`docs/NURBS-SCOPE.md`.

#### Scenario: Multi-face output is a single closed welded shell, not a trimmed fold or a rational offset

- GIVEN any accepted multi-face shell
- WHEN the result is inspected
- THEN it SHALL be a single closed watertight `tessellate::Mesh` with interior shared edges welded (no double-wall) and non-rational offset panels, and the module SHALL NOT emit a trimmed self-intersecting shell or a rational offset, nor claim an exact union volume for interpenetrating dihedral slabs
