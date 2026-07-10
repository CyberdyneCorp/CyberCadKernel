# Tasks — add-python-binding

Verification: **build** = `scripts/build-macos-dylib.sh` (CMake,
`CYBERCAD_MACOS_OCCT=ON`) produces `build-mac/libcybercadkernel.dylib` from
current source (Homebrew OCCT 7.9.x, Metal excluded); the standalone
`python/build_dylib.sh` codifies the same recipe with a manual `clang++` loop as
a fallback. **py** = `python -m pytest python/tests` on the desktop interpreter
with the real engine (`cc_brep_available() == 1`). The acceptance bar for this
change is the pytest suite asserting REAL geometry through Python.

Latest verified run (macOS arm64, Python 3.11.5, pytest 7.4.4):
**35 passed, 1 skipped** — the skip is the offscreen GL render
(`test_viz.py::test_render_png_gl_offscreen_or_skip`, no GL context / pyglet);
the matplotlib PNG fallback and STL round-trip are asserted unconditionally.
Live smoke: `brep_available=True`, box volume `999.9999999999998`.

## 1. Desktop build target
- [x] 1.1 Codify the proven macOS build recipe in `python/build_dylib.sh`:
  compile every `src/**/*.cpp` EXCEPT `src/compute/metal/*` with
  `-std=c++20 -DCYBERCAD_HAS_OCCT -arch arm64 -fPIC -O2` and the `include`/`src`/
  Homebrew-OCCT (`/opt/homebrew/opt/opencascade/include/opencascade`) roots; link
  a dylib against the `TK*` libs with `-rpath` to the OCCT lib dir. (**build**)
  <br>Uses `find … | while read` (zsh does not word-split unquoted vars) and
  flattened object names; `CLEAN=1` rebuild from scratch = 21 TUs, links clean.
- [x] 1.2 Metal excluded on macOS (no `-DCYBERCAD_HAS_METAL`); `cc_set_gpu_tessellation`
  is a safe no-op and `cc_gpu_tessellation_enabled()` returns 0 on this build. (**py**)
  <br>Asserted via the pythonic `Kernel.set_gpu_tessellation` /
  `gpu_tessellation_enabled` controls.
- [x] 1.3 Library discovery: `$CYBERCADKERNEL_DYLIB`, then `build-mac/` relative to
  the repo root, then bare name. (**py**) — `_ffi._candidate_paths`.

## 2. Low-level binding (`_ffi.py`)
- [x] 2.1 Declare every POD struct (`CCMesh`, `CCProfileSeg`, `CCMassProps`,
  `CCEdgePolyline`, `CCFaceMesh`) and `CCShapeId` as ctypes types mirroring the
  header. (**py**)
- [x] 2.2 One signature table applying `argtypes`/`restype` to all `cc_*` symbols,
  incl. by-value struct returns (`CCMesh`, `CCMassProps`) and `T**`
  out-parameters. (**py**)
- [x] 2.3 ABI-layout guard: `test_ffi.py` asserts each struct's `ctypes.sizeof`
  against values cross-checked with a C `sizeof` of the same header
  (`CCMesh 32`, `CCMassProps 48`, `CCProfileSeg 88`, `CCEdgePolyline 24`,
  `CCFaceMesh 40`, `CCShapeId 8`). (**py**)

## 3. Pythonic API
- [x] 3.1 `Kernel` facade: availability (`brep_available`/`require_brep`),
  `last_error`, additive controls (`set_parallel`/`parallel_enabled`,
  `set_gpu_tessellation`/`gpu_tessellation_enabled`), and construction
  (`extrude`/`revolve`/`box`, STEP/IGES import). (**py**)
- [x] 3.2 `Shape`: RAII wrapper over `CCShapeId`, context manager releasing via
  `cc_shape_release` on `__exit__` (+ `__del__` backstop, idempotent
  double-release, use-after-release guard). (**py**)
- [x] 3.3 Operations return NEW `Shape`s and leave operands valid (functional
  ABI): boolean (`fuse`/`cut`/`common`), features (`fillet`/`chamfer`/`shell`),
  transforms (`translate`/`scale`), queries (mass props/bbox/moments/subshape
  ids), exchange (`export_step`/`export_iges`). (**py**)
- [x] 3.4 Failures raise `KernelError` carrying `cc_last_error()` (and
  `BRepUnavailableError` for a stub build) instead of returning `0`/`valid==0`. (**py**)
- [x] 3.5 NumPy meshes: `mesh.py` copies C-owned buffers into owned `(N,3)`
  float64 vertices + `(M,3)` int32 triangles BEFORE freeing the C buffer. (**py**)

## 4. Tests (real geometry through Python)
- [x] 4.1 Exact mass properties: box `10^3` → volume `1000`, area `600`, centroid
  `(5,5,5)`. (**py**)
- [x] 4.2 Booleans by volume: cut `1000-125 = 875`, fuse of two overlapping boxes
  `1500`, common `500`. (**py**)
- [x] 4.3 Exact B-rep bounding box; revolve → cylinder `πr²h`; fillet strictly
  reduces volume; translate moves the bbox; scale cubes the volume. (**py**)
- [x] 4.4 Watertightness: box tessellation is a closed surface (trimesh
  `is_watertight`) with mesh volume `1000`; `face_meshes` cover 6 faces, 12 edges,
  8 vertices. (**py**)
- [x] 4.5 STEP + IGES round-trip preserve volume within tolerance. (**py**)
- [x] 4.6 Lifetime/errors: context-manager release, use-after-release raises,
  degenerate profile raises `KernelError`, operands survive a boolean. (**py**)
  <br>Result: 35 passed, 1 skipped (offscreen GL render — no GL context; the
  matplotlib PNG fallback and STL round-trip still assert real geometry).
- [x] 4.7 A stub build (no engine) SKIPS the geometry tests loudly rather than
  passing a trivially-true check (`conftest._require_real_engine`). (**py**)

## 5. Visualization
- [x] 5.1 `viz.to_trimesh` / `export_mesh` (STL/OBJ/PLY) — lazy trimesh import,
  clear error if missing. (**py**)
- [x] 5.2 Offscreen PNG render smoke test; skips when no GL context, while the STL
  round-trip test asserts real geometry (reload → volume `1000`). (**py**)

## 6. Docs & roadmap
- [x] 6.1 `python/README.md` (build recipe, usage, test) + `docs/python.md`
  (full guide) linked from `docs/README.md`; root README `Python (desktop)`
  subsection.
- [x] 6.2 `openspec/ROADMAP.md` **Tooling & bindings** section + change-index row
  referencing this change + capability `python-binding` (updated to the true
  state: 35 passed / 1 skipped, `scripts/build-macos-dylib.sh`, fuse 1875 /
  common 125).
- [x] 6.3 `openspec validate add-python-binding --strict` green.

## 7. Deferred (out of scope for this change)
- [ ] 7.1 **pybind11 variant — DEFERRED (intentional).** The binding is
  `ctypes`-only. Rationale: the ctypes binding is now COMPLETE (all 111 `cc_*`
  symbols) and working (75 passed / 1 skipped, real geometry). A pybind11 layer
  would be a parallel *reimplementation* of the same C ABI for marginal gain —
  slightly faster per-call dispatch and richer compiled type stubs — at the cost
  of a C++ build-toolchain dependency in the wheel and a second surface to keep in
  lockstep with the header. The `cc_*` ABI is a plain-C boundary that ctypes
  consumes with zero build step and copies buffers into NumPy already; there is no
  functional capability pybind11 would unlock here. Revisit only if profiling shows
  ctypes dispatch is a real bottleneck for a hot loop.
- [~] 7.2 **Interactive `pyvista` render — PARTIAL (offscreen provided).** An
  offscreen PNG render helper usable for an examples gallery is implemented in
  `viz.render_png` (trimesh GL path with a headless matplotlib `Poly3DCollection`
  fallback, `PNG_MAGIC`-validated). Full interactive `pyvista` viewing stays
  optional/out of scope — the offscreen path covers the gallery/CI need without a
  live GL context.
- [x] 7.3 **Wheel packaging — DONE.** `python/setup.py` (`build_py` hook) bundles
  the prebuilt `libcybercadkernel.dylib` into `cybercadkernel/lib/` as package
  data; `_ffi`/`_cffi` discovery probes that bundled path first. `python -m build
  --wheel` produces `cybercadkernel-0.1.0-py3-none-any.whl` with the dylib inside;
  a clean-venv `pip install` of the wheel imports and runs real geometry (box
  volume `1000`, `brep_available=True`) WITHOUT `CYBERCADKERNEL_DYLIB` set.

## 8. Completion — full `cc_*` API coverage + packaging
Verification: `CYBERCADKERNEL_DYLIB=…/build-mac/libcybercadkernel.dylib
python -m pytest python/tests -q` → **75 passed, 1 skipped** (the one skip is the
offscreen GL render; up from 35 passed). `comm -23` of the header symbols against
the bound set is EMPTY: **111 / 111** `cc_*` symbols bound (was 74).

- [x] 8.1 **Coverage closed to 100%.** All 37 previously-missing symbols bound in
  BOTH low-level tables (`_cffi.py` primary + `_ffi.py` parallel, kept in lockstep)
  with correct `argtypes`/`restype`, by-value struct returns, `T**` out-params, and
  the matching `_free` counterparts. `test_ffi.py::test_all_abi_symbols_are_bound`
  guards the count at 111. (**py**)
- [x] 8.2 **New POD structs mirrored field-for-field** with `ctypes.sizeof` guards
  cross-checked against a C `sizeof` compiled from the header
  (`test_ffi.py::test_new_struct_layouts_match_abi`): `CCProjection` 40,
  `CCInterference` 112, `CCValidityReport` 32, `CCDisplayMesh` 48, `CCTetMesh` 40,
  `CCVolumeMeshOptions` 32, `CCQualityReport` 64, `CCPmiSummary` 32,
  `CCDrawingSegment` 32, `CCDrawing` 32, `CCHlrOptions` 24, `CCSectionLoop` 32,
  `CCSection` 32. (**py**)
- [x] 8.3 **Pythonic wrappers** in `api.py` for every group, following the existing
  RAII / `KernelError`-from-`cc_last_error` / copy-then-`_free` patterns: loft
  family (`loft_circles`, `loft_circle_wire`, `loft_typed`, `loft_along_rails`,
  `loft_sections`), variable/guided-orientation sweeps, `draft_faces`,
  `chamfer_edges_asym`, sheet metal (`sheet_base_flange`/`sheet_edge_flange`/
  `sheet_unfold`), `fill_ngon`, `section_plane`, `hlr_project`,
  `project_point_on_face`, `check_solid`, `interference`, `display_mesh`,
  `mesh_quality`, `tet_mesh`/`tet_mesh_surface`, `gltf_export`/`usdz_export`,
  engine select (`Kernel.engine`/`set_engine`), solid enumeration
  (`solid_count`/`solid_at`/`solids`), `step_pmi_scan`, and the measurement queries
  (`measure_distance`/`measure_angle`/`surface_curvature`/`edge_curvature`). Typed
  result dataclasses added: `DisplayMesh`, `TetMesh`, `QualityReport`,
  `ValidityReport`, `Interference`, `Projection`, `PmiSummary`, `Section`/
  `SectionLoop`, `Drawing`. (**py**)
- [x] 8.4 **Real-geometry tests** for the new features (`test_full_api.py`,
  `test_engine_and_declines.py`): loft circles → exact cylinder/frustum volume;
  loft of three square rings → prism `2000`; variable sweep circle→circle → cone;
  draft strictly reduces volume; asym chamfer removes stock; sheet base flange
  volume `3000`, edge flange then unfold → flat-pattern area strictly larger;
  n-gon fill → patch mesh area `50`; section of a box at z=5 → one loop, area `100`,
  perimeter `40`; HLR of a box → 4 visible + 4 hidden segments; point projection →
  distance `40` onto z=10; `check_solid` valid box; interference clash overlap `125`
  / clear gap `5`; `solid_count` of a two-lump fuse == `2` (each lump `1000`); glTF
  `.glb`/`.gltf` + USDZ export write valid magic-byte files; display mesh has unit
  normals + UVs; native tet-mesh quality of a regular tet (min dihedral `70.53`,
  scaled J `1`) and flags a sliver. (**py**)
- [x] 8.5 **Honest-decline contract asserted** for features this MIT desktop build
  does not link: measurement/curvature (`CYBERCAD_HAS_NUMSCI` off) and tet meshing
  (`CYBERCAD_HAS_TETGEN` off) RAISE `CyberCadError` carrying the engine's
  `cc_last_error` message rather than fabricating a value — so the bindings are
  verified against a real engine response, not a stub. (**py**)
- [x] 8.6 **Wheel packaging** (see 7.3). (**build** + **py**)
- [x] 8.7 **Docs**: `python/README.md` + `docs/python.md` updated to "all 111
  `cc_*` bound" with the wheel-build recipe and the native-engine / build-flag
  caveats (native-only section+sheet-metal+PMI; measurement/tet unavailable in the
  MIT build). `openspec validate --all --strict` green.

> **Engine bug found (reported, NOT worked around).** Building a solid under the
> NATIVE engine (`cc_set_engine(1)`) and then CONSUMING it under the OCCT engine —
> `cc_set_engine(0)` then `cc_step_export` / `cc_mass_properties` on that native
> body — SEGFAULTS. Reproducible on the raw C ABI with pure ctypes (no binding
> involvement): the OCCT path is handed a native body it cannot read, contradicting
> the header's "the engine NEVER hands a native body to OCCT" guarantee. This is a
> kernel/engine defect, out of scope for the (correct) binding; the Python tests
> keep engine usage consistent (native bodies built AND consumed under the native
> engine, then restored to OCCT) as the header prescribes, and never trigger it.
