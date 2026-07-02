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

- **Low-level 1:1 `ctypes` binding** (`cybercadkernel/_cffi.py`) — every `cc_*`
  function and POD struct (`CCMesh`, `CCMassProps`, `CCProfileSeg`,
  `CCEdgePolyline`, `CCFaceMesh`, `CCShapeId`) with `argtypes`/`restype` set,
  by-value struct returns, and `T**` out-parameters. An ABI-layout guard
  (`test_ffi.py`) asserts each struct's `ctypes.sizeof` against a C `sizeof` of
  the same header.
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

Verified suite (macOS arm64, Python 3.11, pytest 7.4): **35 passed, 1 skipped**.
The single skip is `test_viz.py::test_render_png_gl_offscreen_or_skip` — the
offscreen **GL** render path (no GL context / pyglet available); the matplotlib
PNG fallback and the STL round-trip are asserted unconditionally (both
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

Live smoke through Python: `brep_available = True`, box volume
`999.9999999999998`.

## Deferred

- **pybind11 variant** — the current binding is `ctypes`-only. A pybind11
  layer (for zero-copy NumPy views and richer type stubs) is not implemented.
- **Interactive `pyvista` render** — only offscreen PNG (trimesh GL /
  matplotlib) is provided; an interactive `pyvista` viewer is not wired up.
- **Wheel packaging** — the package is editable/`pip install -e` only; no
  binary wheel bundling the dylib is built or published.
