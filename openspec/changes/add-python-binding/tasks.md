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
- [ ] 7.1 **pybind11 variant** — the binding is `ctypes`-only; a pybind11 layer
  (zero-copy NumPy views, richer type stubs) is not implemented.
- [ ] 7.2 **Interactive `pyvista` render** — only offscreen PNG (trimesh GL /
  matplotlib fallback) is provided; no interactive `pyvista` viewer.
- [ ] 7.3 **Wheel packaging** — editable/`pip install -e` only; no binary wheel
  bundling the dylib is built or published.
