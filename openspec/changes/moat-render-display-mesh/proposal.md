# Proposal — moat-render-display-mesh (render-quality display mesh)

## Why

The native tessellator (`src/native/tessellate`) produces a CORRECTNESS mesh:
watertight, consistently oriented, every vertex ON the analytic surface within a
deflection bound. That is exactly what the verification model needs, and it is
BYTE-FROZEN — the mesh hashes are asserted across the host and sim suites, so it
must not change.

A real-time viewport (the app's iPad canvas) and higher-quality exchange
(glTF / USDZ) want SHADING attributes the correctness mesh deliberately omits:

- **Smooth per-vertex normals** so a curved wall shades continuously instead of
  faceted;
- **Hard edges** at real creases (a box corner, a cap↔wall seam) so sharp
  features stay sharp;
- **Texture coordinates** for material preview;
- **Level of detail** so a distant / low-power view can drop triangle count.

Today the app has none of these from the kernel. This is ANTICIPATORY value: it
is the render-quality geometry path the app will need as it adopts the native
engine, and it is a bounded, purely-additive slice that touches nothing on the
correctness path.

## What Changes

1. **A new header-only, OCCT-free module `src/native/render/display_mesh.h`** in
   namespace `cybercad::native::render`, consuming ONLY `src/native/math` and
   `src/native/tessellate/mesh.h` (the `Mesh` type). It is a pure CONSUMER of the
   existing correctness mesh — it never meshes a B-rep and never touches the
   tessellator:
   - `smoothNormalsWithCreases(mesh, creaseAngleDeg)` — per-vertex angle/area-
     weighted smooth normals, with crease-based vertex SPLITTING: incident
     triangles are partitioned into smoothing groups by a union-find over shared
     edges whose dihedral is below the crease angle; a vertex touched by more than
     one group is duplicated (one copy per group) so hard edges stay sharp.
   - `assignBoxUVs(displayMesh)` — per-vertex UVs by dominant-axis planar (box)
     projection, normalised into `[0,1]`, seam-consistent.
   - `decimate(displayMesh, targetTris, hausdorff, creaseAngleDeg)` — greedy
     quadric-error edge-collapse LOD that LOCKS boundary and crease vertices and
     rejects any collapse whose geometric deviation exceeds a Hausdorff budget or
     that would flip a triangle.
   - `buildDisplayMesh(mesh, params)` — the driver: smooth-normal split → optional
     LOD → optional UVs. Empty input ⇒ empty output (HONEST DECLINE).

2. **A new ADDITIVE `cc_*` op** `cc_display_mesh(body, deflection, creaseAngleDeg,
   lodTargetTris, wantUVs, out)` + `cc_display_mesh_free` + the POD
   `CCDisplayMesh` (positions / unit normals / optional uvs / triangles). It is
   NOT part of the mirrored `KernelBridgeAPI.h` ABI (styled like the other
   phase-additive ops). It consumes the active engine's tessellation
   (`active_engine()->tessellate`) — so it works under BOTH the OCCT and native
   engines with no engine-interface change — and post-processes it OCCT-free.
   Returns 0 with `*out` zeroed on an empty/unknown body or a mesh that cannot be
   produced; never a fabricated mesh.

3. **No change to the correctness path.** `cc_tessellate` / `cc_face_meshes` and
   every file under `src/native/tessellate/**` are BYTE-IDENTICAL (empty
   `git diff`). The existing mesh-hash suites stay green — that is the proof.

## Impact

- **Affected specs:** `native-tessellation` (ADDED requirements only — the
  display-mesh post-process; the existing tessellation requirements are
  untouched).
- **Affected code:** NEW `src/native/render/display_mesh.h`; ADDITIVE additions to
  `include/cybercadkernel/cc_kernel.h` (struct + two prototypes) and
  `src/facade/cc_kernel.cpp` (the op + free). NEW host test
  `tests/native/test_native_display_mesh.cpp` + CMake registration. NEW sim
  harness `tests/sim/native_display_mesh_parity.mm` (Gate b, SKIP-guarded).
- **ABI:** additive-only. No existing signature changes. `test_abi` stays green.
- **Discipline:** `src/native/**` stays OCCT-free; the byte-frozen tessellator is
  untouched.
