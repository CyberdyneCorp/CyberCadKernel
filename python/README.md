# cybercadkernel (Python binding)

A desktop (macOS arm64) Python binding over the CyberCadKernel **`cc_*` C ABI**.
It is a pure *consumer* of the ABI in `include/cybercadkernel/cc_kernel.h` — it
does not change the ABI. It loads a shared library built against **Homebrew
OpenCASCADE**, so Python drives the *real* geometry engine
(`cc_brep_available() == 1`), not the host stub.

This is a **development / tooling** artifact: it is for driving, testing, and
visualizing the kernel on the desktop. It is **not shipped to iOS**.

**Coverage: all 111 `cc_*` symbols are bound** (the complete
`include/cybercadkernel/cc_kernel.h` surface), each exercised by a real-geometry
pytest assertion.

## Layout

- `cybercadkernel/_cffi.py` — low-level 1:1 ctypes binding (**every one of the
  111 `cc_*` functions** + all POD structs) and the shared-library loader.
  `_ffi.py` is a parallel table kept in lockstep (used by the legacy `kernel.py`).
- `cybercadkernel/api.py` — pythonic `Kernel` / `Shape` object model (context-
  managed handle lifetime, exceptions from `cc_last_error`), plus result
  dataclasses: `Mesh` / `MassProps` / `BoundingBox` / `ReferencePlane` /
  `ReferenceAxis` and the full-coverage additions `DisplayMesh` / `TetMesh` /
  `QualityReport` / `ValidityReport` / `Interference` / `Projection` /
  `PmiSummary` / `Section` / `Drawing`.
- `cybercadkernel/viz.py` — optional `trimesh` export (STL/PLY/GLB) + offscreen
  PNG render (trimesh GL, with a headless matplotlib fallback).
- `tests/` — pytest suite asserting real geometry.
- `build_dylib.sh` — codifies the proven macOS build recipe.

## Build the library

```bash
brew install opencascade          # OCCT 7.9.x
python/build_dylib.sh             # → build-mac/libcybercadkernel.dylib
```

The compile step builds every `src/**/*.cpp` **except** `src/compute/metal/*`
with `-DCYBERCAD_HAS_OCCT` (C++20, arm64) against Homebrew OCCT headers, then
links a dylib against the `TK*` libraries with an rpath. Metal is excluded on
macOS (no `-DCYBERCAD_HAS_METAL`); `cc_set_gpu_tessellation` is a safe no-op
there.

## Use

```python
from cybercadkernel import Kernel

k = Kernel()                        # raises unless a B-rep engine is linked
assert k.brep_available             # property, not a call

def box(dx, dy, dz):                # extrude takes (x, y) pairs
    return k.extrude([(0, 0), (dx, 0), (dx, dy), (0, dy)], dz)

with box(10, 10, 10) as big, box(5, 5, 5) as small:
    with big.cut(small) as part:
        print(part.mass_properties().volume)   # 875.0
        part.step_export("part.step")
        mesh = part.tessellate(0.1)             # NumPy (N,3) verts + (M,3) tris

# datum plane through 3 points → origin + unit normal (XY plane here)
plane = k.ref_plane_from_points((0, 0, 0), (1, 0, 0), (0, 1, 0))
assert plane.normal == (0.0, 0.0, 1.0)
```

The library is discovered via `$CYBERCADKERNEL_DYLIB`, then a `lib/` folder
bundled *inside* the installed package (see the wheel below), then `build-mac/`
relative to the repo root, then the bare name on the loader path.

### Engine selection & build-flag caveats

The default build is **OCCT** (`kernel.engine == "occt"`). A few capabilities are
served only by the **native** C++20 engine — switch with `kernel.engine =
"native"` (or `kernel.set_engine(True)`), and build *and* consume a native body
under the same engine:

- **native-only**: `section_plane`, sheet metal (`sheet_base_flange` /
  `sheet_edge_flange` / `sheet_unfold`), `fill_ngon`, `step_pmi_scan`.
- **build-flag-gated (unavailable in the MIT desktop build)**: the measurement /
  curvature queries (`measure_distance` / `measure_angle` / `surface_curvature` /
  `edge_curvature`) need `CYBERCAD_HAS_NUMSCI`; tet meshing (`tet_mesh` /
  `tet_mesh_surface`) needs the optional AGPL `CYBERCAD_HAS_TETGEN` backend. When
  a flag is off the call *honestly declines* — it raises `CyberCadError` with the
  engine's message rather than returning a fabricated value. (`mesh_quality` is
  pure geometry and always available.)

## Build a wheel

`python -m build` bundles the prebuilt dylib into the wheel so the install is
self-contained (no `build-mac/` or env var needed at runtime):

```bash
python/build_dylib.sh                        # → build-mac/libcybercadkernel.dylib
cd python && python -m build --wheel         # → dist/cybercadkernel-*.whl (dylib inside)
pip install dist/cybercadkernel-*.whl        # imports + runs real geometry as-is
```

`setup.py`'s `build_py` hook copies `libcybercadkernel.dylib` (from
`$CYBERCADKERNEL_DYLIB`, else `build-mac/`) into `cybercadkernel/lib/` as package
data. Build the dylib first for a fully self-contained wheel; otherwise the wheel
ships without it and falls back to the env-var / `build-mac` discovery at runtime.

## Test

```bash
pip install -e "python/[test]"      # numpy, pytest, trimesh, matplotlib
python -m pytest python/tests -q
```

The suite builds/loads the dylib in `conftest.py` and **skips** with an
actionable message if it cannot (no error). Every test then gates on the real
engine (`brep_available`) and asserts exact geometry — box volume 1000 / area
600, boolean cut/fuse/common = 875 / 1875 / 125, watertight tessellation, STEP &
IGES round-trips, and a reference-plane normal. The offscreen **GL** render test
skips when no GL context is available; the STL export and the matplotlib PNG
render are asserted unconditionally (they are headless-safe).

If `trimesh` fails to import (e.g. a NumPy-2 vs SciPy/shapely ABI mismatch),
the viz tests `importorskip` cleanly rather than failing the run.
