# Design — nurbs-multiface-shell

## Placement & conventions

New module `src/native/math/bspline_shell.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_thicken.{h,cpp}`. Reuses `math::Point3` / `Dir3` / `Vec3` (`native/math/vec.h`), the
evaluators `surfacePoint` / `nurbsSurfacePoint` / `surfaceNormal` (`bspline.h`), the **Layer-1 data
type** `BsplineSurfaceData`, the **Layer-5 offset** `offsetSurface` (`bspline_offset.h`), and the
kernel's existing **closed-shell carrier** `tessellate::Mesh` (`native/tessellate/mesh.h` — fp64
`Point3` vertices + indexed `Triangle`s with `isWatertight` / `enclosedVolume` / `boundaryEdgeCount` /
`isConsistentlyOriented` primitives). OCCT-free, fp64, deterministic. It does NOT touch
`bspline_thicken` (the single-patch path stays as-is); the two coexist.

**Not re-exported from `native_math.h`.** Like `bspline_thicken.h`, this header includes
`tessellate/mesh.h`, which includes the `native_math.h` aggregate — re-exporting from the aggregate
would create a circular include. Consumers include `bspline_shell.h` directly.

**numsci gate.** `thickenPatches` composes `offsetSurface` (whose fit solves through
`numerics::lin_solve`), so the whole `bspline_shell.cpp` TU is under `CYBERCAD_HAS_NUMSCI` (mirroring
`bspline_thicken.cpp` / `bspline_offset.cpp`). With the guard OFF the TU is inert and the function is
absent; the declaration stays visible for documentation.

## Adjacency record

`SharedEdge{faceA, faceB, edgeA, edgeB, reversed}` names a shared interior model edge: which
parametric boundary edge (`PatchEdge::{U0,U1,V0,V1}`) of each face is shared, and whether the two run
in opposite parametric directions along the shared curve. The caller supplies the B-rep topology; the
module VERIFIES it geometrically (the two S-edges must sample to coincident points, welding to one
vertex chain) and declines an inconsistent record.

## Assembly by spatial weld — the watertight-by-construction core

The shell is three panel kinds sharing EXACT vertices via a tolerance-bucketed spatial weld:

- **S-caps + O-caps.** Each face's S-cap `Sᶠ` and O-cap `Oᶠ = Sᶠ + d·N` is sampled on a shared
  `(nu × nv)` grid; every sample is welded into ONE global vertex list (a snapped-bucket hash map with
  a 3×3×3-neighbourhood probe within `weldTol`). A shared MODEL edge, sampled to the same S points on
  both faces, collapses to ONE boundary chain — so its S-edge is used by BOTH faces' S-caps (used
  twice = interior, NO S→O wall). A corner where 3+ faces meet welds to a single vertex.
- **Side walls — OUTER boundary only.** After the weld, an S-cap boundary grid-edge used by exactly
  ONE cap triangle is OUTER; a ruled S→O wall is raised there (reusing the welded S/O vertices). An
  interior shared edge is used twice → skipped. This is the "walls only on the free boundary" property.
- **Mitre — the offset-side join at dihedral corners.** On the offset side the shared O-edge either
  WELDS (coplanar/tangent faces → the two offset caps meet directly, closed) or DIVERGES (a dihedral
  corner). A divergent seam is joined by a MITRE: for each seam node the two offset planes are extended
  to their meeting apex `X = S + d/(1+nA·nB)·(nA+nB)` (the point at signed distance `d` from BOTH
  faces), and two ruled panels bridge `oA → apex` and `apex → oB`. Each OUTER-corner endpoint of the
  mitre is sealed to the two adjacent side walls with corner triangles (found via the walls' S→O
  rails). A degenerate (near-anti-parallel) seam with no finite apex falls back to a direct chamfer.

Every seam edge then ends up used by exactly two triangles → **watertight by construction**. The
module then makes the winding COHERENT (BFS across shared edges, flipping inconsistent neighbours;
a ≥ 3-cap seam is non-manifold → decline) and fixes global inside/out by the signed-volume sign.

## Verification before `ok`

`isWatertight` (every undirected edge used exactly twice → χ = 2, zero boundary edges) +
`isConsistentlyOriented` (no directed half-edge traversed twice the same way). Any failure DECLINES
(`NotClosed`) — the module never returns an open, leaky, or double-walled shell as valid.

## Volume oracle rationale

For two COPLANAR rectangles sharing an edge, the offset caps weld (no mitre), the two S-caps form one
flat plane, and the result is EXACTLY one box: enclosed volume = total_area·|d| = Σ per-face area·|d|,
with the shared edge contributing NO wall and NO double-count. This is the airtight closed-form oracle
(exact to ~1e-9 on a bilinear patch). For a DIHEDRAL L-shape the shell is watertight, but the two thin
slabs interpenetrate near the seam (one face's S-cap passes through the other's slab), so its enclosed
volume is NOT the simple union closed form without face trimming — hence the L-shape asserts
watertightness + positive finite volume only, and face trimming is a documented residual.

## Complexity note

`thickenPatches` is a linear pipeline of well-named phases (guard → offset-all → sample+weld →
adjacency+mitre-classify → caps → walls → mitre-emit → orient → verify), with the two most separable
phases extracted (`sampleOneFaceCaps`, `emitMitreSeams`) and the coherent-orientation pass shared with
the single-patch thicken's proven `orientCoherently`. Its cyclomatic count is dominated by
guard-clause early returns (each `if (bad) return` counts a path but keeps cognitive load low); the
residual is genuine for a multi-face + mitre assembly and is flagged rather than mangled.

## Alternatives considered

- **Extend `bspline_thicken` in place** — rejected as more invasive: the single-patch box-topology path
  is proven and its host gate is airtight; a separate module keeps that untouched and the two
  constructions independently verifiable.
- **Chamfer-only offset join** — rejected: a chamfer chops the offset corner and loses volume even for
  a clean convex mitre. The extend-to-meet apex is the correct join; chamfer is kept only as the
  degenerate (no-finite-apex) fallback.
