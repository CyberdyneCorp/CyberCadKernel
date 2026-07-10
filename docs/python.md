# Python binding (desktop)

`cybercadkernel` is a **desktop (macOS arm64) Python package** that drives the
kernel through its stable plain-C ABI (`cc_*`, declared in
`include/cybercadkernel/cc_kernel.h`). It is a pure *consumer* of the ABI — it
never changes it — and it loads a shared library built against **Homebrew
OpenCASCADE**, so Python exercises the *real* B-rep engine
(`cc_brep_available() == 1`), not the CPU-only host stub.

It is a **development / tooling** artifact: for driving, testing, and
visualizing the kernel on the desktop. It is **not shipped to iOS**.

## What it provides

- **Low-level 1:1 `ctypes` binding** (`cybercadkernel/_cffi.py`) — **all 111
  `cc_*` functions** and every POD struct (`CCMesh`, `CCMassProps`,
  `CCProfileSeg`, `CCEdgePolyline`, `CCFaceMesh`, `CCShapeId`, plus the
  full-coverage additions `CCProjection`, `CCInterference`, `CCValidityReport`,
  `CCDisplayMesh`, `CCTetMesh`, `CCVolumeMeshOptions`, `CCQualityReport`,
  `CCPmiSummary`, `CCDrawing`/`CCDrawingSegment`/`CCHlrOptions`,
  `CCSection`/`CCSectionLoop`) with `argtypes`/`restype` set, by-value struct
  returns, and `T**` out-parameters. An ABI-layout guard (`test_ffi.py`) asserts
  each struct's `ctypes.sizeof` against a C `sizeof` of the same header, and
  `test_all_abi_symbols_are_bound` pins the bound count at **111 / 111** — the
  whole header, no gaps. (`_ffi.py` is a parallel table kept in lockstep.)
- **Pythonic object model** (`cybercadkernel/api.py`):
  - `Kernel` — facade over engine state (`brep_available`, `require_brep`,
    `last_error`, the additive `set_parallel` / `set_gpu_tessellation` toggles)
    and the shape factories (`extrude`, `revolve`, `box`, lofts/sweeps/threads,
    STEP/IGES import, reference-geometry helpers).
  - `Shape` — a body wrapping a `CCShapeId` with a **context-managed, GC-safe
    lifetime** (auto `cc_shape_release` on `__exit__`, `__del__` backstop,
    idempotent double-release, use-after-release guard). Booleans
    (`fuse`/`cut`/`common`), features (`fillet_edges`/`chamfer_edges`/`shell`),
    transforms (`translate`/`scale`), queries (mass properties / bbox / moments
    / subshape ids), tessellation, and STEP/IGES export live here. Every op
    returns a **new** `Shape` and leaves the operands valid (functional ABI).
  - `Mesh` — a tessellation result as **owned** NumPy arrays: vertices
    `(N, 3)` float64, triangles `(M, 3)` int32. C-owned buffers are copied into
    Python-owned arrays *before* the C buffer is freed.
  - `MassProps`, `BoundingBox`, `ReferencePlane`, `ReferenceAxis`, `FaceMesh`,
    `EdgePolyline` — query results as dataclasses.
  - `CyberCadError` (alias `KernelError`) — raised from `cc_last_error()`
    whenever a call would return `0` / invalid, and `KernelLibraryNotFound`
    when the dylib cannot be located.
- **Visualization helpers** (`cybercadkernel/viz.py`) — see below.

## Install / build

The binding needs the macOS OCCT dylib. Build it with Homebrew OCCT and the
CMake-driven build script:

```bash
brew install opencascade                 # OCCT 7.9.x
scripts/build-macos-dylib.sh             # → build-mac/libcybercadkernel.dylib
# CLEAN=1 scripts/build-macos-dylib.sh   # wipe build-mac/ and rebuild
```

`scripts/build-macos-dylib.sh` configures CMake with `-DCYBERCAD_MACOS_OCCT=ON`
and builds the `cybercadkernel` shared target: every `src/**/*.cpp` **except**
the Metal TUs (`src/compute/metal/*`), compiled `-std=c++20`
`-DCYBERCAD_HAS_OCCT` `-arch arm64` against the Homebrew OCCT headers, then
linked into a dylib against the `TK*` libraries with an rpath so OCCT resolves
at load time. Metal is excluded on macOS (no `-DCYBERCAD_HAS_METAL`);
`cc_set_gpu_tessellation` is a safe no-op there and `cc_gpu_tessellation_enabled()`
returns `0`.

> A standalone `python/build_dylib.sh` codifies the same recipe with a manual
> `clang++` loop (no CMake) and is kept as a fallback / reference.

Install the package (editable) and its test/viz extras:

```bash
pip install -e "python/[test]"           # numpy, pytest, trimesh, matplotlib
```

The library is discovered at import time via `$CYBERCADKERNEL_DYLIB`, then
`build-mac/` relative to the repo root, then the bare library name.

## Usage — extrude → boolean → mass props → tessellate → export

```python
from cybercadkernel import Kernel

k = Kernel()                             # raises unless a B-rep engine is linked
assert k.brep_available                  # property, not a call

def box(dx, dy, dz):                     # extrude takes (x, y) profile points
    return k.extrude([(0, 0), (dx, 0), (dx, dy), (0, dy)], dz)

# 10×10×10 box, cut by a 5×5×10 corner tool → 875 mm³.
with box(10, 10, 10) as big, box(5, 5, 5) as small:
    with big.cut(small) as part:
        mp = part.mass_properties()
        print(mp.volume)                 # 875.0
        print(mp.centroid)               # exact B-rep centroid

        part.step_export("part.step")    # STEP round-trips (preserves volume)

        mesh = part.tessellate(0.1)      # deflection in mm
        print(mesh.vertices.shape)       # (N, 3) float64
        print(mesh.triangles.shape)      # (M, 3) int32

        from cybercadkernel import viz
        viz.export_stl(mesh, "part.stl")     # binary STL (also .ply/.glb/.obj)
        viz.render_png(mesh, "part.png")     # offscreen PNG (headless-safe)

# Reference geometry: a datum plane through 3 points → origin + unit normal.
plane = k.ref_plane_from_points((0, 0, 0), (1, 0, 0), (0, 1, 0))
assert plane.normal == (0.0, 0.0, 1.0)
```

Failures never cross the boundary as C error codes: a failed `cc_*` call
(returning `0` / `valid == 0`) raises `CyberCadError` carrying the
`cc_last_error()` message, and a stub build raises `BRepUnavailableError`.

## Visualization helpers (`cybercadkernel.viz`)

`trimesh` and `matplotlib` are **optional** — importing `viz` pulls in neither;
each entry point raises a clear error only if the backend it needs is missing.

- `to_trimesh(mesh)` — wrap a `Mesh` (or a raw `(vertices, triangles)` pair) as
  a `trimesh.Trimesh` (`process=False`, so geometry is preserved verbatim).
- `export_mesh(mesh, path)` — export by extension; plus explicit
  `export_stl` / `export_ply` / `export_glb`.
- `render_png(mesh, path, resolution=(512, 512), prefer="auto")` — render an
  **offscreen** PNG. `prefer="auto"` tries trimesh's GL renderer and, if no GL
  context is available (the common headless case), degrades gracefully to a
  pure-matplotlib `Poly3DCollection` raster on the `Agg` backend.
  `prefer="matplotlib"` forces the always-headless-safe fallback; `prefer="gl"`
  requires a GL context and raises otherwise.

## Run the tests

```bash
CYBERCADKERNEL_DYLIB="$PWD/build-mac/libcybercadkernel.dylib" \
  python -m pytest python/tests -q
```

`conftest.py` builds/loads the dylib and **skips** with an actionable message if
it cannot (no error). Every geometry test then gates on the real engine
(`brep_available`) and asserts exact numbers — a stub build **skips loudly**
rather than passing a trivially-true check.

Verified suite (macOS arm64, pytest): **75 passed, 1 skipped** (up from 35 — the
new `test_full_api.py` and `test_engine_and_declines.py` cover the full-coverage
additions). The single skip is `test_viz.py::test_render_png_gl_offscreen_or_skip`
— the offscreen **GL** render path (no GL context / pyglet available); the
matplotlib PNG fallback and the STL round-trip are asserted unconditionally (both
headless-safe). Geometry asserted through Python (exact, `abs=1e-6` unless
noted):

| Assertion | Value |
|---|---|
| box 10×10×10 volume / area / centroid | `1000.0` / `600.0` / `(5, 5, 5)` |
| boolean CUT (a − b, 5×5×10 overlap) | `875.0` |
| boolean FUSE (two overlapping boxes) | `1875.0` |
| boolean COMMON | `125.0` |
| operands survive a boolean | `a = 1000`, `b = 125` |
| 3×4×5 box: volume / area / centroid | `60.0` / `94.0` / `(1.5, 2, 2.5)` |
| 3×4×5 box bbox | min `(0,0,0)` max `(3,4,5)` |
| cube principal moments | `i1 == i2 == i3 > 0` |
| revolved cylinder volume | matches `πr²h` (rel `1e-3`) |
| tessellation | watertight, float64 `(N,3)` verts, int32 `(M,3)` tris; 6 box face-meshes |
| STEP / IGES round-trip | preserves volume |
| fillet / translate / scale | volume down / bbox moves / volume cubes |
| loft circles (equal r) / (taper) | cylinder `πr²h` / frustum volume |
| loft of 3 square rings | prism `2000.0` |
| variable sweep circle→circle | cone / frustum (rel `2e-2`) |
| draft / asym chamfer | strictly reduce volume |
| sheet base flange 50×30×2 | `3000.0` |
| edge flange → unfold | flat-pattern area > base area |
| n-gon fill (10/10 triangle) | patch mesh area `50.0` |
| section of box at z=5 | 1 loop, area `100.0`, perimeter `40.0` |
| HLR of box (front view) | 4 visible + 4 hidden segments |
| point projection onto z=10 | distance `40.0`, foot `(5,5,10)` |
| `check_solid` of a box | valid, decided, `first_failure == 0` |
| interference clash / clear | overlap `125.0` / min distance `5.0` |
| `solid_count` of a two-lump fuse | `2` (each lump `1000`) |
| glTF `.glb` / `.gltf` / USDZ export | valid magic-byte files |
| display mesh | unit per-vertex normals + UVs |
| tet-mesh quality (regular tet) | min dihedral `70.53`, scaled J `1.0` |

The measurement / curvature and tet-meshing wrappers are asserted to **honestly
decline** (raise `CyberCadError`) on this MIT desktop build, since it links
neither `CYBERCAD_HAS_NUMSCI` nor `CYBERCAD_HAS_TETGEN`.

Live smoke through Python: `brep_available = True`, box volume
`999.9999999999998`.

## Wheel packaging

`python -m build` produces a **self-contained wheel** with the native dylib
inside, so an install needs no `build-mac/` checkout or env var at runtime:

```bash
python/build_dylib.sh                    # → build-mac/libcybercadkernel.dylib
cd python && python -m build --wheel      # → dist/cybercadkernel-*.whl (dylib bundled)
pip install dist/cybercadkernel-*.whl     # imports + runs real geometry as-is
```

`setup.py`'s `build_py` hook copies the dylib (from `$CYBERCADKERNEL_DYLIB`, else
`build-mac/`) into `cybercadkernel/lib/` as package data; discovery probes that
bundled path first. A clean-venv install of the wheel imports and runs real
geometry (box volume `1000`, `brep_available = True`) with no env var set.

## Deferred

- **pybind11 variant** — **deferred by design.** The `ctypes` binding is complete
  (all 111 `cc_*`) and passing on real geometry; a pybind11 layer would only be a
  parallel reimplementation of the same plain-C ABI for marginal per-call speed
  and compiled type stubs, at the cost of a C++ build-toolchain dependency in the
  wheel. No functional capability is gated on it. Revisit only if ctypes dispatch
  is shown to be a real hot-loop bottleneck.
- **Interactive `pyvista` render** — partial. An offscreen PNG helper
  (`viz.render_png`, trimesh GL + headless matplotlib fallback) is provided and
  covers the examples-gallery / CI need; a live interactive `pyvista` viewer stays
  optional.

## Known engine defect (not a binding issue)

Building a solid under the **native** engine (`kernel.engine = "native"`) and then
consuming it under the **OCCT** engine — e.g. `kernel.engine = "occt"` followed by
`step_export` / `mass_properties` on that native body — **segfaults**. This
reproduces on the raw C ABI with pure `ctypes` (no binding involvement): the OCCT
path is handed a native body it cannot read, contradicting the header's guarantee
that the engine never hands a native body to OCCT. It is a kernel/engine defect,
outside the (correct) binding. Keep engine usage consistent — build *and* consume
a native body under the native engine, then restore OCCT — as the header
prescribes; the test suite does exactly this and never trips it.
