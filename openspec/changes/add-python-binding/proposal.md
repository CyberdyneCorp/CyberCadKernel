## Why

CyberCadKernel is a C++20 library behind a plain-C ABI (`cc_*`) built for iOS.
Today the only way to exercise it is the C/C++ test harnesses cross-compiled for
the iOS simulator. That makes it hard to *use* the kernel interactively, to write
tests quickly, or to **see** what a `cc_*` call produced. A Python binding over
the stable `cc_*` surface gives a fast, ergonomic way to drive the kernel, script
regression tests, and visualize the resulting meshes — on the desktop, where
iteration is quick.

The ABI is the ideal seam: it is plain C (integer handles + POD structs, no C++
or engine type crosses it), so a thin binding (`ctypes`/`pybind11`) maps directly
onto it and stays valid as the engine behind the facade evolves.

## What Changes

- Add a **desktop (macOS arm64) build** of the kernel as a shared library
  (`libcybercadkernel.dylib`) linking a **macOS OCCT** — the prerequisite that
  turns the host build from the stub engine into a real geometry engine, so
  Python gets real solids (not `cc_brep_available()==0`).
- Add a **Python package** (`cybercadkernel`) that loads the dylib and exposes:
  - a low-level 1:1 binding of every `cc_*` function + POD struct;
  - a **pythonic API** — a `Kernel`/`Shape` object model with context-managed
    handle lifetime (auto `cc_shape_release`), NumPy arrays for meshes/points,
    and exceptions raised from `cc_last_error` instead of `0`/`nil` returns.
- Add a **pytest suite** asserting real geometric properties through Python
  (extrude/boolean volumes, mass properties, STEP round-trip, tessellation).
- Add **visualization** helpers: convert a `CCMesh` to NumPy and render/export it
  (e.g. `pyvista`/`trimesh`), with an offscreen smoke test that writes an image.
- Document build + usage in `docs/python.md`; the package builds against the
  desktop dylib and is a **development/tooling** artifact (not shipped to iOS).

No change to the `cc_*` ABI; the binding is a pure consumer of it.

## Capabilities

### New Capabilities
- `python-binding`: a desktop Python package over the `cc_*` ABI (low-level
  binding + pythonic object model + NumPy mesh interop + visualization), backed
  by a macOS OCCT build of the kernel, with a pytest suite verifying real
  geometry through Python.

## Impact

- **New desktop build target**: macOS arm64 shared library linking macOS OCCT
  (Homebrew `opencascade` or a from-source build). This is additive — the iOS
  and host-stub builds are unchanged.
- **New `python/` tree**: the `cybercadkernel` package, tests, and viz helpers.
- **Tooling only**: the binding is for development, testing, and visualization on
  the desktop; it is not part of the iOS app shipping path.
- **License**: linking macOS OCCT (LGPL-2.1 + exception) applies to the desktop
  build the same way it does to iOS.
