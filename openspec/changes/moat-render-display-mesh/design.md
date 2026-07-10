# Design — moat-render-display-mesh

## The hard constraint that makes this parallel-safe

The correctness tessellator is byte-frozen (mesh hashes asserted across the
suites). This change adds a SEPARATE consumer, `src/native/render/display_mesh.h`,
whose sole geometry input is the `(vertices, triangles)` the existing mesher
already produced. Proof the tessellator is untouched: `git diff` on
`src/native/tessellate/**` is empty and the existing mesh-dependent suites
(`test_native_tessellate`, `test_native_engine`, `test_native_stl`, the sim mesh
hashes) stay green.

## Why crease detection from the mesh, not from face IDs

A crease is where two triangles meet at a dihedral angle above threshold. The
dihedral is computable from the two triangles' face normals alone — no topology,
no per-face grouping needed. This is both simpler AND more OCCT-free than routing
face IDs through the facade: the display module never sees the B-rep. A box's 90°
edges, a cylinder's cap↔wall 90° seam, and a sphere's zero-crease surface all fall
out of the mesh geometry directly.

## Smooth normals + crease split

For each undirected mesh edge shared by exactly two triangles, if
`dot(n0, n1) >= cos(creaseAngle)` the edge is SMOOTH and the two triangles' corner
slots on each shared endpoint are UNIONED (union-find over `3·triangle+corner`
slots). Each connected component at an original vertex is a SMOOTHING GROUP and
becomes one display vertex carrying the angle-weighted (× area) average of its
group's face normals. A vertex in more than one group is duplicated — that IS the
hard edge.

Key oracle property proven in Gate A: on a cylinder wall, each ring vertex's two
incident angular segments are SYMMETRIC about the vertex angle, so the averaged
wall normal is EXACTLY radial (matches the analytic surface normal to ~1e-6 /
machine precision). A sphere's lat/long fan is NOT angularly symmetric, so its
averaged normal approximates the analytic radial to O(deflection) and CONVERGES as
deflection refines — a genuine, honest approximation, asserted as convergence
rather than a false 1e-6 claim.

## UVs

A box/planar projection: each vertex projects onto the plane whose axis is its
normal's dominant component, and the whole-mesh projected extent is normalised into
`[0,1]`. Deterministic and seam-consistent (same position + same dominant axis ⇒
same UV). This is a material-preview UV, explicitly NOT a full unwrap — documented
as such so no one mistakes it for a seam-minimised atlas.

## LOD (quadric-error edge collapse)

Garland-Heckbert quadrics per vertex from incident triangle planes. Greedy:
collapse the cheapest legal edge to the midpoint (or onto a locked endpoint if
exactly one side is locked) until the target triangle count is reached.

Three guards keep it honest:

1. **Lock** boundary (edge used ≠ 2), non-manifold, and position-shared (crease-
   split) vertices — silhouettes and hard edges never collapse.
2. **Hausdorff (geometric, not travel-distance).** The merged quadric's RMS
   perpendicular deviation of the target from the incident planes must stay within
   `budget = lodHausdorffScale · deflection`. This bounds how far the decimated
   surface departs from the SOURCE surface — the correct Hausdorff metric. (A
   naive "how far did the vertex move" guard over-rejects every on-surface
   tangential move and stalls the decimator; that was fixed.)
3. **Flip guard.** Reject any collapse that reverses an incident triangle's
   winding. A rejected edge is skipped for the round and re-tried after the next
   successful collapse (never permanently locked, which would cascade-stall).

The Hausdorff bound is the BINDING constraint on a near-uniform sphere: the
collapse stops early to honour the budget, and a looser budget collapses further —
proven in Gate A. `lodTargetTris <= 0` skips decimation entirely.

## Facade wiring (engine-agnostic)

`cc_display_mesh` calls `active_engine()->tessellate(body, deflection)` for the
correctness mesh, converts the flat `MeshData` to `tess::Mesh`, and runs
`buildDisplayMesh`. Because it consumes the ENGINE's tessellation, it serves under
both the OCCT and the native engine with NO engine-interface change (no new
virtual). The result is copied into C-owned malloc buffers freed by
`cc_display_mesh_free`.

## Honest decline

Empty body / empty tessellation / post-process yielding no triangles ⇒ return 0
with `*out` zeroed and `cc_last_error` set. No fabricated mesh, ever. Every display
POSITION (pre-LOD) is a verbatim source vertex (on the solid within deflection);
LOD moves vertices only within the asserted Hausdorff budget.

## Alternatives considered

- *Analytic normals from the B-rep surface* would give exact 1e-6 normals on a
  sphere too, but requires the display module to see topology/surfaces — breaking
  the "pure mesh consumer, OCCT-free, no tessellator coupling" property. Averaged
  normals with a proven convergence bound are the honest, decoupled choice.
- *A new engine virtual `display_mesh`* was rejected: consuming the existing
  `tessellate` output keeps the change additive with zero interface churn and
  works identically under both engines.
