# python-binding

## ADDED Requirements

### Requirement: Desktop shared library over the cc_* ABI (macOS OCCT)
The project SHALL provide a build (`python/build_dylib.sh`) that produces a
desktop macOS arm64 shared library `build-mac/libcybercadkernel.dylib` from the
current sources by compiling every `src/**/*.cpp` EXCEPT the Metal backend
(`src/compute/metal/*`) with `-std=c++20 -DCYBERCAD_HAS_OCCT -arch arm64 -fPIC`
against Homebrew OpenCASCADE and linking against the `TK*` libraries. The library
SHALL report `cc_brep_available() == 1` (a real B-rep engine), and the Metal GPU
tessellation path SHALL be excluded so that `cc_gpu_tessellation_enabled()`
returns `0` and `cc_set_gpu_tessellation` is a safe no-op. The build SHALL NOT
modify the `cc_*` ABI.

#### Scenario: Build produces a real-geometry dylib
- GIVEN a machine with Homebrew OpenCASCADE installed at `/opt/homebrew/opt/opencascade`
- WHEN `python/build_dylib.sh` is run
- THEN it SHALL compile 21 translation units (all `src/**/*.cpp` except `src/compute/metal/*`) and link `build-mac/libcybercadkernel.dylib`
- AND loading that library and calling `cc_brep_available()` SHALL return `1`

#### Scenario: Metal is excluded on the desktop build
- GIVEN the desktop dylib built without `-DCYBERCAD_HAS_METAL`
- WHEN `cc_set_gpu_tessellation(1)` is called through the binding
- THEN `cc_gpu_tessellation_enabled()` SHALL return `0` and tessellation SHALL use the OCCT path

### Requirement: Low-level 1:1 ctypes binding of the cc_* ABI
The package SHALL expose a low-level module that declares every POD struct
(`CCMesh`, `CCProfileSeg`, `CCMassProps`, `CCEdgePolyline`, `CCFaceMesh`) and the
`CCShapeId` handle type as ctypes types matching `include/cybercadkernel/cc_kernel.h`,
and SHALL apply `argtypes`/`restype` to every `cc_*` function — including the
by-value struct returns (`CCMesh`, `CCMassProps`) and the pointer-to-pointer
out-parameters. The binding SHALL be a pure consumer: it SHALL NOT add, remove,
or alter any `cc_*` signature or POD struct layout.

#### Scenario: Struct layouts match the C ABI
- GIVEN the ctypes struct declarations in the low-level module
- WHEN `ctypes.sizeof` is taken of `CCMesh`, `CCMassProps`, `CCProfileSeg`, `CCEdgePolyline`, `CCFaceMesh`, and `CCShapeId`
- THEN each SHALL equal the value a C `sizeof` of the same header yields on the platform (`32`, `48`, `88`, `24`, `40`, `8` on arm64 macOS)

#### Scenario: A raw call returns real geometry
- GIVEN the loaded library through the low-level binding
- WHEN a closed 10×10 square profile is extruded by depth 10 via `cc_solid_extrude` and `cc_mass_properties` is queried
- THEN the returned `CCMassProps.valid` SHALL be `1` and `volume` SHALL equal `1000` within `1e-6`

### Requirement: Pythonic object model with context-managed handle lifetime
The package SHALL expose a `Kernel` facade and a `Shape` object model over the
low-level binding. A `Shape` SHALL own a `CCShapeId` and SHALL release it via
`cc_shape_release` when used as a context manager (on `__exit__`), with a
garbage-collection backstop and an idempotent, crash-free double release. Because
the ABI is functional, an operation SHALL return a NEW `Shape` and SHALL leave
its operand shapes valid. Using a shape after release SHALL raise rather than
call into the library with a stale handle.

#### Scenario: Context manager releases the handle
- GIVEN `with kernel.box(1, 1, 1) as b:`
- WHEN the `with` block exits
- THEN `b`'s handle SHALL have been released via `cc_shape_release`
- AND calling a query on `b` afterward SHALL raise `KernelError`

#### Scenario: Operands survive a boolean
- GIVEN two boxes `a` (10³) and `b` (5³)
- WHEN `a.cut(b)` is evaluated
- THEN the result SHALL be a new `Shape` with volume `875` within `1e-6`
- AND `a.volume()` SHALL still be `1000` and `b.volume()` SHALL still be `125`

#### Scenario: Double release is safe
- GIVEN a `Shape` that has already been released once
- WHEN `release()` is called again
- THEN it SHALL be a no-op and SHALL NOT crash

### Requirement: NumPy mesh interop
The package SHALL convert `cc_tessellate` / `cc_face_meshes` results into owned
NumPy arrays — vertices as an `(N, 3)` float64 array and triangles as an `(M, 3)`
int32 array — by COPYING the C-owned buffers before freeing them via the matching
`cc_*_free` call, so the resulting mesh has no lifetime dependency on the kernel.

#### Scenario: Box tessellation yields owned watertight geometry
- GIVEN a 10×10×10 box shape
- WHEN it is tessellated at deflection `0.1` and the mesh is converted to NumPy
- THEN `vertices` SHALL be an `(N, 3)` float64 array and `triangles` an `(M, 3)` int32 array with `N, M > 0`
- AND the mesh SHALL be watertight and its computed volume SHALL equal `1000` within `1e-6`
- AND the underlying C mesh buffer SHALL have been freed after the copy

#### Scenario: Per-face meshes cover the box topology
- GIVEN a 10×10×10 box shape
- WHEN `face_meshes` is queried
- THEN it SHALL return exactly 6 face meshes, and the shape SHALL report 6 faces, 12 edges, and 8 vertices

### Requirement: Exceptions raised from cc_last_error
The pythonic API SHALL raise `KernelError` carrying the thread-local
`cc_last_error()` message when a `cc_*` call fails (returns `0`, an invalid
handle, or a struct with `valid == 0`), instead of returning a sentinel. When a
B-rep operation is attempted on a build with no engine (`cc_brep_available() == 0`),
it SHALL raise `BRepUnavailableError`.

#### Scenario: A failing construction raises with the kernel message
- GIVEN a degenerate profile that cannot form a solid
- WHEN it is passed to `Kernel.extrude`
- THEN a `KernelError` SHALL be raised (not a `0` handle returned)

#### Scenario: An unknown boolean op is rejected
- GIVEN two valid shapes
- WHEN a boolean is requested with an operation name other than `fuse`/`cut`/`common`
- THEN a `ValueError` SHALL be raised before any `cc_*` call

### Requirement: Real-geometry test suite (honest verification)
The package SHALL ship a pytest suite that asserts REAL geometric properties
through Python — exact volumes, mass properties, exact bounding boxes,
watertightness, and STEP/IGES round-trip — not trivially-true checks. On a build
without a real engine (`cc_brep_available() == 0`) the geometry tests SHALL SKIP
loudly rather than pass.

#### Scenario: Suite asserts exact volumes and round-trips
- GIVEN the desktop dylib with `cc_brep_available() == 1`
- WHEN `python -m pytest python/tests` is run
- THEN tests SHALL assert box volume `1000`, boolean cut `875`, fuse `1500`, common `500`, a revolved cylinder `πr²h`, and STEP/IGES export→import preserving volume within tolerance
- AND the suite SHALL pass (with the offscreen-render test allowed to skip when no GL context is available)

#### Scenario: Stub build skips loudly
- GIVEN a build where `cc_brep_available() == 0`
- WHEN the geometry tests run
- THEN they SHALL be reported as SKIPPED with a reason, NOT passed

### Requirement: Mesh visualization helpers
The package SHALL provide visualization helpers that convert a mesh to a
`trimesh.Trimesh`, export it to a mesh file (e.g. STL/OBJ/PLY), and render an
offscreen PNG. The `trimesh` dependency SHALL be optional: importing the viz
module SHALL NOT import trimesh, and each helper SHALL raise a clear error if
trimesh is unavailable. The offscreen render MAY skip when no GL context is
available, but the file-export path SHALL work headlessly.

#### Scenario: STL export round-trips a real solid headlessly
- GIVEN a 10×10×10 box mesh
- WHEN it is exported to an `.stl` file and reloaded
- THEN the file SHALL be non-empty and the reloaded mesh volume SHALL equal `1000` within `1e-6`

#### Scenario: Offscreen render is a skippable smoke test
- GIVEN a box mesh and an environment without a GL context
- WHEN an offscreen PNG render is attempted
- THEN it SHALL either write a valid PNG file or skip with a reason, and SHALL NOT fail the suite

### Requirement: Tooling artifact, not shipped to iOS, ABI unchanged
The Python binding SHALL be a development/tooling artifact that lives under
`python/` and builds against the desktop dylib; it SHALL NOT be part of the iOS
app shipping path and SHALL NOT change the `cc_*` ABI. Linking macOS OCCT
(LGPL-2.1 + exception) SHALL apply to the desktop build the same way it does to
iOS, and the binding SHALL add no new license obligation.

#### Scenario: The binding adds no ABI surface
- GIVEN the change as landed
- WHEN the `cc_*` header and POD structs are compared before and after
- THEN they SHALL be unchanged, and the binding SHALL exist only under `python/` plus the additive `build-mac/` output
