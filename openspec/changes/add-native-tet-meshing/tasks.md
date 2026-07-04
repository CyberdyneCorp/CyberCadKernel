# Tasks — add-native-tet-meshing (issue #1)

Order: additive ABI PODs/prototypes → always-on native quality → gated TetGen
adapter → facade wiring → CMake gating + build script → tests + docs. All new native
code stays OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::mesh`. No `cc_*` ABI change (additive entry points only). The
default MIT build MUST compile and link ZERO AGPL code, and `cc_tet_mesh*` MUST
return a clean "unavailable" error when the flag is OFF. TetGen sources are never
vendored (referenced by absolute path only) and `/build-tet/` is git-ignored. Keep
per-function cognitive complexity in the systems band (split `tetrahedralize_surface`
into small marshal / switches / linear-read / C3D10-build helpers; factor each
quality metric into its own function).

## 1. Additive ABI (`include/cybercadkernel/cc_kernel.h`)

- [ ] 1.1 Add PODs `CCTetMesh` (nodes + counts, connectivity + counts,
      `nodesPerElement` 4/10, `order`), `CCVolumeMeshOptions` (`order`,
      `target_element_size`, `grading`, `min_scaled_jacobian`), and `CCQualityReport`
      (`min/max_dihedral_angle`, `min/mean_scaled_jacobian`, `max_aspect_ratio`,
      `elements_below_threshold`, `flagged_elements`, `valid`) in the POD block,
      carrying the "ADDITIVE — NOT part of the mirrored KernelBridgeAPI.h ABI" comment.
- [ ] 1.2 Add prototypes `cc_tet_mesh`, `cc_tet_mesh_surface`, `cc_tet_mesh_free`,
      `cc_mesh_quality`, `cc_quality_report_free`, documenting the unavailable-when-OFF
      contract for the two mesh entries and the always-on nature of quality. Field
      names mirror CalculiX++ `VolumeMeshOptions` / `QualityReport`. No existing
      signature changed.

## 2. Always-on native quality (`src/native/mesh/quality.{h,cpp}`)

- [ ] 2.1 SPDX `Apache-2.0`; NO TetGen reference; compiled by the default glob.
      `QualityResult` struct + `quality(nodes, nodeCount, elements, elementCount,
      nodesPerElement, minScaledJacobian)` using only the 4 corner nodes per element
      (works identically for C3D4 and C3D10).
- [ ] 2.2 Signed volume `V = (1/6) det[e12,e13,e14]`; regular edge L ⇒ `V = L³/(6√2)`.
- [ ] 2.3 Scaled Jacobian (Verdict / Knupp): `√2 · min` over the 4 corners of the
      triple product of the 3 outgoing UNIT edge vectors. Regular → 1, sliver → 0,
      inverted < 0. Cite Stimpson et al. Verdict SAND2007-1751 and Knupp 2001.
- [ ] 2.4 Six dihedral angles from 4 outward face normals (Shewchuk 2002), stored in
      DEGREES; regular tet ⇒ all six = `acos(1/3) = 70.5288°`.
- [ ] 2.5 Aspect ratio = Verdict radius ratio `R·S/(9|V|)`; regular → 1, degenerate →
      ∞. Cite Liu & Joe 1994 and Verdict SAND2007-1751.
- [ ] 2.6 Aggregate: min/max dihedral over all elements×6 edges; min/mean scaled
      Jacobian; max aspect ratio; `flagged = {e : scaledJ_e < minScaledJacobian}`,
      `elements_below_threshold = flagged.size()`; `valid = false` on empty/degenerate
      input (any |V| ≈ 0 or non-finite metric).

## 3. Gated TetGen adapter (`src/native/mesh/tet_mesher.{h,cpp}`)

- [ ] 3.1 SPDX `AGPL-3.0` on both files. `tet_mesher.h` is TetGen-header-free: declares
      `MeshOrder`, `VolumeMeshOptions`, `TetMesh`, `TetMeshResult`, and
      `tetrahedralize_surface(verts, tris, opts)` using only `std::vector`. All
      `tetgenio` usage lives in `tet_mesher.cpp`, the SOLE `#include <tetgen.h>` TU.
- [ ] 3.2 Validate up front (before allocation): reject empty / mis-sized `V`,`T`,
      `< 4` points or triangles, and any out-of-range triangle index → `fail(msg)`.
- [ ] 3.3 Marshal into stack `tetgenio in,out` with `new[]` arrays (TetGen frees with
      `delete[]`): `in.firstnumber=0`, `pointlist` from `V`, one facet of one 3-vertex
      polygon per triangle, zeroed `facetmarkerlist`.
- [ ] 3.4 Build switches locale-safely: always `p` + `q<ratio>` (from
      `radius_edge_ratio`/`grading`, clamped ≥ 1.0) + `Q` (quiet); `a<maxvol>` with
      `maxvol = h³/(6√2)` only when `target_element_size h > 0`; append `Y` (freeze the
      input boundary) ONLY on the unsized pure-fill path — never with `a`, since `Y`
      forbids boundary Steiner points and silently defeats the max-volume cap
      (unit cube stays ~6 tets at any size); never `o2`.
- [ ] 3.5 Drive `tetrahedralize` inside a try/catch (open / non-watertight surface →
      throw or 0 tets → clean `fail`, no partial mesh). Guard empty output.
- [ ] 3.6 Read LINEAR tets from `out.pointlist` (Steiner points included); per tet fix
      corner order to POSITIVE signed volume. Linear path ⇒ 4 nodes/elem.
- [ ] 3.7 Native C3D10 path: dedup mid-edge nodes via a packed `edge_key` map appended
      after all corner/Steiner nodes; emit in CalculiX `shape10tet` order (5=mid(1,2),
      6=mid(2,3), 7=mid(3,1), 8=mid(1,4), 9=mid(2,4), 10=mid(3,4)); 10 nodes/elem.

## 4. Facade wiring (`src/facade/cc_kernel.cpp`)

- [ ] 4.1 Include `native/mesh/native_mesh.h`; add an `empty_tet_mesh()` helper.
- [ ] 4.2 Shared `tet_mesh_from_surface(verts, tris, opts)`: under
      `CYBERCAD_HAS_TETGEN` marshal options, call `tetrahedralize_surface`, copy into
      a `CCTetMesh` on success or set `cc_last_error` and return empty on failure;
      under `#else` set `cc_last_error` to a "tet meshing unavailable (build with
      CYBERCAD_HAS_TETGEN=ON — optional, external AGPL TetGen backend)" message and
      return empty.
- [ ] 4.3 `cc_tet_mesh(body, deflection, opts)`: guard, tessellate via the active
      engine, funnel `MeshData` through `tet_mesh_from_surface` (mirrors
      `cc_stl_export`).
- [ ] 4.4 `cc_tet_mesh_surface(...)`: guard, build vectors from the raw pointers, same
      shared helper.
- [ ] 4.5 `cc_mesh_quality(mesh, thr)`: NOT `#ifdef`'d — guard, call
      `native::mesh::quality`, copy scalars + `flagged_elements`, set `valid`, and set
      `cc_last_error` on invalid input. Add `cc_tet_mesh_free` /
      `cc_quality_report_free` (mirror `cc_mesh_free`).

## 5. CMake gating + build script

- [ ] 5.1 `CMakeLists.txt`: add `option(CYBERCAD_HAS_TETGEN … OFF)`,
      `CYBERCAD_TETGEN_DIR`, `CYBERCAD_TETGEN_SRC_DIR` (mirroring the NumSci block).
- [ ] 5.2 Exclude `src/native/mesh/tet_mesher.cpp` from the default source glob
      (basename-anchored); leave `quality.cpp` and the umbrella globbed.
- [ ] 5.3 Gated `if(CYBERCAD_HAS_TETGEN)` block: `target_sources` the adapter with
      per-source `COMPILE_DEFINITIONS "CYBERCAD_HAS_TETGEN=1;TETLIBRARY"` and per-source
      `INCLUDE_DIRECTORIES` = `CYBERCAD_TETGEN_SRC_DIR`; `FATAL_ERROR` if
      `CYBERCAD_TETGEN_DIR` unset or no `libtetgen_*.a`; link the archive; and define
      `CYBERCAD_HAS_TETGEN=1` on the target so the facade call-site `#ifdef` compiles
      (keep `TETLIBRARY` per-source only).
- [ ] 5.4 Register `test_native_quality` on the unconditional `CYBERCAD_TESTS` list +
      `_SRC`; register `test_native_tet` + `_SRC` inside a gated `if(CYBERCAD_HAS_TETGEN)`
      test block.
- [ ] 5.5 `scripts/build-tetgen.sh` (chmod +x) mirroring `scripts/build-numsci.sh`:
      compile `/home/leonardo/work/tetgen` (predicates.cxx `-O0`, tetgen.cxx `-O2`,
      both `-DTETLIBRARY`) into `build-tet/host/libtetgen_host.a` by absolute path,
      never copying sources. Add an explicit `/build-tet/` line to `.gitignore`.

## 6. Tests + docs

- [ ] 6.1 Always-on `tests/native/test_native_quality.cpp` driving `cc_mesh_quality`:
      regular-tet baseline (dihedral ≈ 70.5288°, scaledJ ≈ 1, aspect ≈ 1, V > 0,
      valid); sliver flagged below threshold; inverted tet scaledJ < 0; C3D10 mid-node
      consistency (quality == C3D4 corners, mid-nodes at exact midpoints);
      empty/degenerate → valid == 0.
- [ ] 6.2 Gated `tests/native/test_native_tet.cpp` driving `cc_tet_mesh_surface` on a
      hardcoded unit cube (8 pts + 12 facets, `pq1.4a…`): C3D4 watertight (every V > 0,
      scaledJ > 0); C3D10 mid-nodes at midpoints in CalculiX order; volume conservation
      (Σ|V_e| == enclosed surface volume within tolerance); face manifoldness (boundary
      faces == input surface); quality gate via `cc_mesh_quality`.
- [ ] 6.3 README / ROADMAP / STATUS: add the TetGen build+test recipe and the new
      module / option / tests, stating honestly that TetGen is optional, external,
      AGPL-3.0, default-OFF; the default MIT build never links it; `cc_tet_mesh*` return
      "unavailable" when OFF; shipping a closed app with TetGen needs a commercial
      license; and CalculiX++ `CadMesher` wiring is a follow-up.
- [ ] 6.4 Verify: default (OFF) host build + CTest green — `test_native_quality`
      passes, no new failure vs the pre-existing `test_native_boolean` baseline; TetGen
      ON build + CTest runs `test_native_tet`; `openspec validate add-native-tet-meshing
      --strict`; AGPL git-status check (no `tetgen.*` / `predicates.*` staged).
