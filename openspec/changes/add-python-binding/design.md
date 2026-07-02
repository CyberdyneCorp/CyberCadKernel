## Context

CyberCadKernel is a C++20 geometry kernel behind a stable, plain-C ABI (`cc_*`,
`include/cybercadkernel/cc_kernel.h`): opaque integer `CCShapeId` handles and POD
structs cross the boundary — no C++ or OCCT type does. The kernel is built for
iOS, and until now the only way to exercise it was the C/C++ harnesses
cross-compiled for the iOS simulator. That makes interactive use, quick
regression scripting, and *seeing* what a `cc_*` call produced awkward.

The ABI is the ideal seam for a desktop binding: because it is plain C, a thin
binding maps directly onto it and stays valid as the engine behind the facade
evolves (Phases 1–4). The prerequisite is a **desktop build of the kernel with a
real B-rep engine** — otherwise Python only sees the host stub
(`cc_brep_available() == 0`) and gets no solids.

A manual build has already proven the recipe works end-to-end: it produced
`build-mac/libcybercadkernel.dylib` and ran real geometry through it
(`cc_brep_available() == 1`, box volume `1000`, box-minus-corner cut `875`). This
change codifies that build and layers a Python package on top; it does not
rediscover the recipe and does not touch the ABI.

Constraints:
- **ABI is frozen here.** The binding is a pure consumer of `cc_*`; no signature
  or POD struct layout changes. The ctypes declarations must match the header
  byte-for-byte (verified against a C `sizeof`).
- **Real geometry only.** Tests must assert exact/analytic properties (volumes,
  mass properties, watertightness, STEP round-trip) through Python, never a
  trivially-true check; on a stub build they SKIP loudly, not pass.
- **Tooling, not shipping.** The package targets the desktop for development,
  testing, and visualization. It is not part of the iOS app path.
- **License.** Linking macOS OCCT (LGPL-2.1 + exception) applies to the desktop
  dylib exactly as it does to iOS; the binding adds no new obligation.

## Goals / Non-Goals

Goals:
- A **desktop (macOS arm64) build** of the kernel as `libcybercadkernel.dylib`
  linking Homebrew OCCT — the prerequisite that gives Python real solids.
- A **low-level 1:1 binding** of every `cc_*` function + POD struct.
- A **pythonic API** — `Kernel` / `Shape` object model, context-managed handle
  lifetime, NumPy meshes, exceptions from `cc_last_error`.
- A **pytest suite** asserting real geometry through Python.
- **Visualization** helpers (trimesh) with an offscreen render smoke test.

Non-Goals:
- No new `cc_*` and no ABI change (pure consumer).
- No shipping to iOS; no packaging to PyPI in this change.
- No wrapper for a Windows/Linux OCCT build here (macOS Homebrew only, though the
  loader and code are written to not preclude it).
- No coverage of every one of the 57 entry points in the *pythonic* layer this
  change — the low-level layer binds all of them; the object model wraps the
  common subset (construct/boolean/feature/transform/query/tessellate/exchange)
  and exposes the rest through `_ffi` until demand warrants sugar.

## Decisions

- **macOS OCCT via Homebrew.** Prefix `/opt/homebrew/opt/opencascade` (v7.9.x),
  headers under `include/opencascade`, libs under `lib`. Codified in
  `python/build_dylib.sh`: compile every `src/**/*.cpp` **except**
  `src/compute/metal/*` with `-std=c++20 -DCYBERCAD_HAS_OCCT -arch arm64 -fPIC
  -O2` and the three include roots (`include`, `src`, the OCCT headers), then link
  a dylib against the `TK*` libraries with an `-rpath` to the OCCT lib dir. The
  script finds sources with `find … | while read` (not an unquoted glob) because
  zsh does not word-split unquoted variables, and it flattens object names so
  same-basename files in different dirs do not collide.
- **Metal excluded on macOS.** No `-DCYBERCAD_HAS_METAL`; the Metal `.mm` TUs are
  skipped. `cc_set_gpu_tessellation` is a safe no-op and
  `cc_gpu_tessellation_enabled()` returns 0 on this build — matching the kernel's
  documented non-Metal behavior. OCCT tessellation is exact and complete.
- **ctypes over pybind11.** ctypes gives a **zero-build-step** binding over the C
  ABI: no compiler, no extension module, no version-coupled `.so` to rebuild when
  Python changes. Since the boundary is already plain C with integer handles and
  POD structs, pybind11 would add a build dependency and a second ABI (the C++
  glue) for no expressive gain. ctypes marshals the by-value struct returns
  (`CCMesh`, `CCMassProps`) and the `T**` out-parameters directly. A single
  signature table in `_ffi.py` applies `argtypes`/`restype` so the mapping is
  auditable against the header in one place.
- **Handle lifetime via context managers.** `Shape` owns a `CCShapeId` and calls
  `cc_shape_release` on `__exit__` (with a `__del__` GC backstop and idempotent
  double-release). This scopes handles deterministically (`with kernel.box(...)
  as b:`) instead of leaking them, mirroring the kernel's own explicit-release
  registry model. The ABI is functional — an operation returns a *new* body and
  never mutates its operands — so `a.cut(b)` yields a new `Shape` and leaves `a`,
  `b` valid.
- **NumPy for meshes.** `cc_tessellate` / `cc_face_meshes` return C-owned pointer
  buffers; the mesh layer **copies** them into owned `(N,3)` float64 vertices and
  `(M,3)` int32 triangles *before* freeing the C buffer, so a `Mesh` has no
  lifetime coupling to the kernel and is safe to keep and export.
- **Exceptions from `cc_last_error`.** The C ABI signals failure with `0` /
  `valid == 0`. The pythonic layer raises `KernelError` carrying the thread-local
  `cc_last_error()` message instead of returning a sentinel;
  `BRepUnavailableError` distinguishes a stub build.
- **Viz via trimesh (optional).** `viz.py` never imports trimesh at module load;
  each entry point imports it lazily and raises a clear message if missing, so
  the core binding is usable with only NumPy. It converts a `Mesh` to
  `trimesh.Trimesh`, exports STL/OBJ/PLY, and renders an offscreen PNG.
- **Tooling-only, not shipped to iOS.** The package lives under `python/`, builds
  against the desktop dylib, and is documented as a development artifact.

## Risks / Trade-offs

- **ABI drift.** If a `cc_*` signature or POD layout changed, ctypes would
  silently mis-marshal. Mitigation: `test_ffi.py` asserts every struct's
  `ctypes.sizeof` against values cross-checked with a C `sizeof` of the same
  header, and the whole signature table sits in one reviewable block.
- **`CCShapeId` is `long`.** 64-bit on macOS/Linux LP64 (correct here); a hostile
  ILP32 target would differ. Documented; out of scope for the macOS tooling.
- **Ownership bugs.** Forgetting `cc_*_free` leaks C memory. Mitigation: every
  buffer-returning call frees the C buffer in a `finally` immediately after
  copying into NumPy; `Shape` releases its handle deterministically.
- **trimesh dependency friction.** trimesh imports SciPy at load; a NumPy-2
  environment needs `scipy>=1.13`. Mitigation: viz is optional and its tests use
  `importorskip`; the offscreen PNG render skips when no GL context exists, while
  the STL-export test still asserts real geometry (reloads and checks volume).
- **Homebrew coupling.** The build hard-codes the Homebrew prefix (overridable via
  `OCCT_PREFIX`). A from-source OCCT would need a different prefix; documented.

## Migration Plan

1. `brew install opencascade`; run `python/build_dylib.sh` to produce
   `build-mac/libcybercadkernel.dylib` (21 TUs, Metal excluded).
2. Land the package under `python/` (`_ffi` low-level binding, `kernel`/`mesh`/
   `viz` pythonic layer) that loads the dylib via `$CYBERCADKERNEL_DYLIB` or the
   `build-mac/` default.
3. Run `python -m pytest python/tests` — real-geometry assertions pass against
   `cc_brep_available() == 1`; on a stub build they skip loudly.
4. Document build + usage in `python/README.md`; record the change in
   `openspec/ROADMAP.md` under **Tooling & bindings** (capability
   `python-binding`).

No app or iOS change; additive `python/` tree only.

## Open Questions

- Whether to later grow the pythonic wrappers to all 57 entry points or keep the
  long tail on `_ffi` (demand-driven).
- Whether a Linux/Windows OCCT build and a PyPI wheel are worth adding once the
  desktop workflow settles (out of scope here).
