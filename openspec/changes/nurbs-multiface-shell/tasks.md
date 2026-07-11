# Tasks — nurbs-multiface-shell

## 1. Foundation — module + result type
- [x] 1.1 Create `src/native/math/bspline_shell.h` — `PatchEdge`, `SharedEdge` (adjacency record),
      `ShellStatus`, `ShellResult` (carrying the closed `tessellate::Mesh solid` + closure invariants +
      geometry metrics incl. `interiorSharedEdges` / `wallEdges` / `mitreEdges`), and the
      `thickenPatches(faces, adjacency, d, tol, gridU, gridV, weldTol)` declaration. Reuses the Layer-1
      `BsplineSurfaceData` inputs and the `tessellate::Mesh` closed-shell carrier. NOT re-exported from
      `native_math.h` (same tessellate→native_math cycle avoidance as `bspline_thicken.h`).
- [x] 1.2 Guards — ≥ 1 well-formed face, `|d|` above the linear tolerance (`ZeroThickness` else),
      every adjacency `faceA`/`faceB` index in range and distinct (`DegenerateInput` else).

## 2. Compose the offset + weld the multi-face shell
- [x] 2.1 Offset EACH face via `offsetSurface(Sᶠ, d, tol)` — propagate its honest declines
      (`DegenerateNormal` / `SelfIntersection` / fit failure) onto `ShellStatus`; report the worst
      offset error + smallest curvature radius across faces. NEVER build a folded panel.
- [x] 2.2 Sample each face's S-cap and O-cap on a shared `(nu × nv)` grid (rational-aware `evalS` +
      `surfaceNormal`), welding coincident vertices into ONE global vertex list via a tolerance-bucketed
      spatial weld (the 3×3×3-neighbourhood probe collapses a shared model edge / a 3+-face corner to one
      vertex). `sampleOneFaceCaps` helper keeps the per-face loop out of the driver.
- [x] 2.3 VERIFY each adjacency record — the two S-edges MUST weld to the same vertex chain (else
      `AdjacencyMismatch`); classify the offset side (welds → coplanar/tangent; diverges → dihedral
      needing a mitre), computing the mitre apex where the two offset planes meet.
- [x] 2.4 Emit S-cap + O-cap triangles (opposite windings); build a ruled S→O side wall on every OUTER
      boundary grid-edge (S-edge used once after weld) and NONE on interior shared edges (used twice).
- [x] 2.5 Emit MITRE panels for dihedral seams (`emitMitreSeams`): extend both offset planes to the
      apex (or chamfer if no finite apex) and seal each OUTER corner to the adjacent walls via the
      side-wall rails.
- [x] 2.6 Orient coherently — BFS across shared edges flipping any inconsistent neighbour (declining a
      non-manifold ≥ 3-cap seam as `NonManifold`), then fix global inside/out by the signed-volume sign.
- [x] 2.7 VERIFY closure — `isWatertight` (χ = 2, zero boundary edges) + `isConsistentlyOriented`; a
      shell that fails closure DECLINES (`NotClosed`) — never returned open/leaky. Report the enclosed
      volume + summed mid-surface area.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_shell.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_thicken`; keep the CMake `if/endif` balanced).
- [x] 3.2 WATERTIGHT — a 2-patch coplanar pair, an L-shaped base+wall pair, and a two-strip curved pair
      each thicken to ONE closed 2-manifold (χ = 2, zero boundary edges, consistently oriented) at both
      signs of `d`, with exactly one interior shared edge.
- [x] 3.3 VOLUME — two coplanar rectangles sharing an edge == EXACTLY one box (volume == total_area·|d|
      == Σ per-face area·|d| to ~1e-9, shared edge not double-counted); L-shape encloses a positive
      finite volume.
- [x] 3.4 NO INTERIOR WALL — wall-edge count == outer perimeter only; every offset-side seam edge is
      used twice (interior) — the two offset faces meet directly.
- [x] 3.5 GUARDS — over-radius face declines the shell (`SelfIntersection`, empty solid); degenerate
      face declines; inconsistent adjacency declines (`AdjacencyMismatch`); out-of-range index + zero
      thickness decline.

## 4. Docs + validation
- [x] 4.1 Update `docs/NURBS-SCOPE.md` Layer-5 row(s): surface offset + single-patch + multi-face
      thicken landed; self-intersecting recovery + rational offset + dihedral-slab exact volume (face
      trimming) residual.
- [x] 4.2 `openspec validate nurbs-multiface-shell --strict` passes; full host `ctest` green
      (no regression).

## 5. Honest residuals (recorded, never faked)
- [x] 5.1 Robust self-intersecting-shell recovery (trim the fold rather than decline whole shell).
- [x] 5.2 Exact union volume for interpenetrating dihedral slabs (needs face trimming at face–face
      intersections); the coplanar box is the exact oracle, the L-shape is watertight only.
- [x] 5.3 Rational offset panel (inherited from `bspline_offset`).
