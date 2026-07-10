# Tasks — moat-render-display-mesh

## 1. Native display-mesh module (OCCT-free, header-only, pure consumer)
- [x] 1.1 `src/native/render/display_mesh.h` in `cybercad::native::render`,
      including only `src/native/math` + `src/native/tessellate/mesh.h` (zero OCCT
      symbols; the byte-frozen tessellator is READ, never modified).
- [x] 1.2 `smoothNormalsWithCreases` — angle/area-weighted per-vertex normals with
      union-find smoothing groups over sub-crease shared edges; crease vertices
      SPLIT (one display vertex per group). Moves no vertex.
- [x] 1.3 `assignBoxUVs` — dominant-axis planar UV projection, normalised to
      `[0,1]`, seam-consistent.
- [x] 1.4 `decimate` — quadric-error edge-collapse LOD; boundary/crease/position-
      shared vertices LOCKED; geometric-deviation Hausdorff guard + flip guard;
      recomputes smooth normals on the survivors.
- [x] 1.5 `buildDisplayMesh` driver — smooth split → optional LOD → optional UVs;
      empty input ⇒ empty output (HONEST DECLINE).

## 2. ABI + facade (ADDITIVE, no cc_* signature change)
- [x] 2.1 `CCDisplayMesh` POD + `cc_display_mesh` + `cc_display_mesh_free` in
      `include/cybercadkernel/cc_kernel.h` (documented, ADDITIVE, not part of the
      mirrored `KernelBridgeAPI.h` ABI).
- [x] 2.2 `cc_display_mesh` in `src/facade/cc_kernel.cpp`: pull the active engine's
      tessellation (OCCT or native), convert to `tess::Mesh`, run
      `buildDisplayMesh`, emit C-owned buffers. Decline → 0 + `cc_last_error`.
- [x] 2.3 `cc_tessellate` / `cc_face_meshes` and `src/native/tessellate/**` are
      BYTE-IDENTICAL (empty `git diff`).

## 3. Gate A — host analytic (no OCCT)
- [x] 3.1 `tests/native/test_native_display_mesh.cpp` + CMake registration.
- [x] 3.2 Cylinder lateral-wall smooth normals EXACTLY radial (~1e-6 — machine
      precision by angular symmetry); cap↔wall ring SPLITS (axial + radial copies).
- [x] 3.3 Sphere all-smooth (no split); averaged normal CONVERGES to analytic
      radial as deflection refines, within a small multiple of deflection.
- [x] 3.4 Box: 24 split corner normals + 6 axis-aligned face normals; every normal
      axis-aligned.
- [x] 3.5 Crease-angle threshold respected (raise it → fewer / no splits).
- [x] 3.6 LOD reduces triangles toward target within the asserted Hausdorff bound;
      a tighter budget throttles the collapse; `lodTargetTris<=0` disables LOD.
- [x] 3.7 UVs in `[0,1]`, seam-consistent; empty input ⇒ empty output; base
      (pre-LOD) positions lie on the source solid and fold back to a watertight
      mesh consistent with `cc_tessellate`.

## 4. Gate B — sim native-vs-engine (SKIP-guarded harness)
- [x] 4.1 `tests/sim/native_display_mesh_parity.mm` — own `main()` + runner + SKIP
      entry; drives `cc_display_mesh` through the engine and re-asserts the
      closed-form-normal oracle + the on-surface / watertight-fold cross-checks on
      the same shapes.

## 5. Docs
- [x] 5.1 `openspec/MOAT-ROADMAP.md` — new "tessellate — render-quality display
      mesh" entry.
- [x] 5.2 `openspec validate --strict` passes for this change.
