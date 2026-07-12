# nurbs-cc-facade

## ADDED Requirements

### Requirement: Additive NURBS geometry handle types
The kernel SHALL add opaque `cc_curve` and `cc_surface` integer handle types,
registry-backed like `CCShapeId`, declared in a new `cc_kernel_nurbs.h` that
`cc_kernel.h` includes. The addition SHALL be purely additive: every one of the
existing `cc_*` symbols and every existing POD struct SHALL remain byte-unchanged,
so `test_abi` and the `ios-app-compat` "ABI is a verified superset of the app's
bridge header" guarantee continue to hold. Curve/surface handles SHALL be released
by `cc_curve_release` / `cc_surface_release`, which SHALL be idempotent and
crash-free on double release, and using a released handle SHALL fail safely rather
than dereference a stale slot.

#### Scenario: New handles are additive to the ABI
- GIVEN the kernel built with `cc_kernel_nurbs.h`
- WHEN the exported `cc_*` symbol set and every existing POD struct layout are compared against the prior ABI
- THEN all previously-existing symbols and struct `sizeof`s SHALL be unchanged, and only new symbols / new handle types / new accessor structs SHALL be added

#### Scenario: Handle lifetime is safe
- GIVEN a `cc_surface` handle created from data
- WHEN `cc_surface_release` is called twice on it and then an accessor is called on the released handle
- THEN the second release SHALL be a safe no-op and the accessor SHALL return an error (not crash or read a stale slot)

### Requirement: POD accessors read back exact NURBS data
The kernel SHALL expose accessors that return a curve/surface's degree, control
count, knot count, and rationality through `CCCurveInfo` / `CCSurfaceInfo`, and
buffer-fill calls that copy the knot vector and the control points into
caller-owned storage. Control points SHALL be emitted in homogeneous `(x, y, z, w)`
form (surfaces row-major), so the rational and non-rational cases are uniform; a
non-rational input SHALL report `rational = 0` with all weights `1`. A buffer-fill
call given too small a buffer SHALL return a negative count and write nothing out
of bounds.

#### Scenario: Round-trip of control data
- GIVEN a rational `cc_curve` created from a known knot vector and homogeneous control net
- WHEN `cc_curve_info` then `cc_curve_knots` and `cc_curve_poles` are read back
- THEN the info SHALL report the correct degree / counts / `rational = 1`, and the returned knots and homogeneous poles SHALL equal the input to ≤ 1e-12

#### Scenario: Undersized buffer is rejected safely
- GIVEN a `cc_surface` whose pole buffer needs N doubles
- WHEN `cc_surface_poles` is called with a buffer smaller than N
- THEN it SHALL return a negative count and SHALL NOT write past the caller buffer

### Requirement: Exact evaluation through the handle
The kernel SHALL evaluate a point on a `cc_curve` / `cc_surface` at given
parameters via `cc_curve_eval` / `cc_surface_eval`, returning the point on the
exact rational geometry. The evaluated point SHALL match the native evaluator to
≤ 1e-12, and for a handle built from an analytic primitive it SHALL lie on the
true primitive.

#### Scenario: Evaluating an exact circle
- GIVEN a `cc_curve` produced by `cc_nurbs_circle(center, radius, plane)`
- WHEN `cc_curve_eval` is sampled at several parameters
- THEN each returned point SHALL lie on the true circle to ≤ 1e-12 (the rational quadratic is exact, not an approximation)

### Requirement: Display tessellation bridge, not a watertight weld
The kernel SHALL provide `cc_surface_tessellate` (→ `CCMesh`) and
`cc_curve_polyline` (→ `CCEdgePolyline`) for **single-surface display**
visualization. This SHALL be documented and treated as display tessellation only:
it SHALL NOT be represented as, and the facade SHALL NOT claim, a watertight
multi-face curved-seam weld into a solid B-rep (the frozen-mesher wall). A closed
single surface MAY tessellate to a watertight mesh; a multi-face assembly SHALL be
shown as a set of faces, not as a sewn watertight solid.

#### Scenario: A closed surface tessellates for display
- GIVEN a `cc_surface` that is a closed revolved sphere
- WHEN `cc_surface_tessellate` is called and the mesh is loaded
- THEN it SHALL be a renderable triangle mesh approximating the sphere within the requested tolerance

#### Scenario: No watertight-solid claim for multi-face assemblies
- GIVEN several curved NURBS faces meeting at shared seams
- WHEN they are tessellated for display
- THEN the facade SHALL expose them as individual face meshes and SHALL NOT return them as a single watertight sewn solid

### Requirement: Honest-decline error model at the ABI
Every NURBS constructor and feature wrapper SHALL return a `0` (invalid) handle and
set `cc_last_error` when it cannot produce a correct result — the over-radius
freeform fillet, the over-constrained fit, the G2-infeasible creased vertex blend,
the coincident-loop trim boolean. It SHALL NOT widen any tolerance at the boundary
to convert a decline into a success, and it SHALL NOT return a plausible-but-wrong
handle. This is the ABI-level expression of the kernel's `DISAGREED = 0`
discipline.

#### Scenario: An infeasible operation declines, not fabricates
- GIVEN two freeform faces and a rolling-ball radius larger than the local concave curvature allows
- WHEN `cc_nurbs_fillet_freeform_g2` is called
- THEN it SHALL return a `0` handle and set `cc_last_error`, and SHALL NOT return a self-intersecting fillet surface

### Requirement: Feature wrappers over the Wave D–I native modules
The kernel SHALL expose `cc_nurbs_*` wrappers over the landed native modules —
fitting/reverse-engineering, analytic↔NURBS conversion, curve/surface intersection,
surfacing (skin/Gordon/Coons/N-sided/sweep/revolve/join), blend (freeform G2
fillet, vertex blend, chamfer), offset/thicken (rational offset, fold-trim,
self-intersection-trimmed thicken/shell), and the parameter-space trim boolean.
Each SHALL return a `cc_curve` / `cc_surface` / value result (or an honest
decline), preserving the correctness guarantees proven natively (e.g. a
self-intersection-trimmed thicken SHALL never yield a self-intersecting result).

#### Scenario: Constructing a surface through a wrapper
- GIVEN N boundary curves passed as `cc_curve` handles
- WHEN `cc_nurbs_nsided_fill` is called in G2 mode
- THEN it SHALL return a `cc_surface` whose points interpolate the boundaries and whose spokes are curvature-continuous, or an honest decline if the configuration is G2-infeasible

#### Scenario: Recognition returns general for freeform input
- GIVEN a `cc_surface` that is a genuinely freeform bicubic bump
- WHEN `cc_nurbs_recognize_surface` is called
- THEN it SHALL report "general" and SHALL NOT report a spurious analytic primitive

### Requirement: Python object model over the NURBS handles
The Python package SHALL expose `Curve` and `Surface` RAII classes over the new
handles (context-managed release + GC backstop + stale-handle guard), NumPy
accessors for knots and homogeneous control points, `.eval`, and `.tessellate`
into the existing mesh interop, plus a `nurbs` submodule surfacing every
`cc_nurbs_*` wrapper following the existing RAII / `KernelError`-from-`cc_last_error`
pattern. An honest ABI decline SHALL raise `KernelError`, not return a degenerate
object. The low-level binding tables SHALL bind every new symbol and guard the new
struct `sizeof`s against a C `sizeof` of the header.

#### Scenario: A NURBS surface is built and shown from Python
- GIVEN the desktop package with the real engine
- WHEN a `Surface` is produced by `nurbs.skin([...])`, evaluated, and tessellated
- THEN the evaluated points SHALL match the analytic expectation and `.tessellate()` SHALL yield a renderable mesh, with the handle released on context exit

#### Scenario: A declining operation raises
- GIVEN an over-constrained constrained-fit request through the `nurbs` submodule
- WHEN it is called
- THEN it SHALL raise `KernelError` carrying the engine's `cc_last_error` message rather than returning an invalid `Curve`

### Requirement: Example gallery showcasing each feature
The project SHALL provide runnable example scripts, one per Wave D–I feature
family, that each construct geometry through the `nurbs` Python API and render a
validated PNG, extending the existing example gallery. Each script SHALL be a
commented usage sample, and the gallery index SHALL state which feature each piece
exercises.

#### Scenario: A gallery piece renders
- GIVEN the example script for the N-sided-filled boss
- WHEN it is run against the desktop package
- THEN it SHALL construct the surface via `nurbs.nsided_fill`, tessellate it, and write a valid (magic-byte-checked) PNG to the gallery output
