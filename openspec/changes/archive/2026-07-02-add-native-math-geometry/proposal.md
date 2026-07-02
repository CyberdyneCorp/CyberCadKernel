## Why

Phase 4 drops OCCT one capability at a time until it can be unlinked entirely
(`openspec/NATIVE-REWRITE.md`). Every higher capability — topology, tessellation,
swept solids, booleans, blends — ultimately reduces to evaluating geometry:
points, derivatives, and normals on curves and surfaces, expressed in a small
algebra of fp64 value types and transforms. Nothing native can be built until
that math foundation exists, OCCT-free and host-buildable, so this is capability
`#1` in the locked dependency order.

Making the math layer OCCT-free is what unlocks the two-gate verification model:
because it links no OCCT and needs no simulator, it can be unit-tested on the host
with `clang++ -std=c++20` against analytic known values, and it can *also* be
compared numerically against the OCCT oracle (`gp_*`, `BSplCLib`, `BSplSLib`,
`PLib`, `ElSLib`) on the simulator. Two independent gates over the same code give
high confidence in the numerics before anything is wired into the engine.

This change delivers the foundation library and its verification ONLY. It does
NOT touch the `cc_*` ABI and does NOT wire native code into the active engine —
that begins with `add-native-brep-topology` / `add-native-swept-solids`. The app
keeps running unchanged on OCCT throughout.

## What Changes

- Add a new **OCCT-free C++20 math library** under `src/native/math/` (no OCCT
  include anywhere; compiles with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`
  with no OCCT), implemented **clean-room** from first principles and public
  references (*The NURBS Book*: FindSpan A2.1, BasisFuns A2.2, CurvePoint A3.1,
  CurveDerivs A3.2, SurfacePoint A3.5, SurfaceDerivs A3.6; de Casteljau for
  Bézier). OCCT source is consulted only as a numeric/convention **oracle**, never
  copied.
- **Value types** (fp64, deterministic): `vec3`, `point3`, `dir3` (unit-normalized
  direction), and a 4×4 affine `transform` supporting compose, invert, and
  application to a point (translated), a vector (not translated), and a direction
  (rotated + re-normalized). Clean, `constexpr`-friendly where natural.
- **Curve evaluation**: Bézier point + derivatives via de Casteljau; B-spline /
  NURBS point + derivatives via basis functions / de Boor, with rational weights
  handled by evaluating in homogeneous coordinates and applying the quotient rule
  for derivatives.
- **Surface evaluation**: tensor-product Bézier / B-spline / NURBS point + first
  partial derivatives (`dS/du`, `dS/dv`) + unit normal, rational weights handled in
  homogeneous form.
- **Elementary surfaces**: plane, cylinder, cone, sphere — point + unit normal
  from their closed-form parametrizations (oracle: `ElSLib`).
- **Verification harness**: for every requirement, (a) a host analytic unit test
  (known Bézier/NURBS points, transform identities, exact elementary-surface
  normals) built with `clang++ -std=c++20` and no OCCT, AND (b) a native-vs-OCCT
  numeric parity test on the iOS simulator that samples inputs and asserts a match
  within a tight fp64 tolerance against the OCCT oracle.

No public C ABI change. No engine wiring. Determinism: fp64 throughout, fixed
evaluation order, reproducible.

## Capabilities

### New Capabilities
- `native-math`: OCCT-free, host-buildable C++20 fp64 math + geometry-evaluation
  foundation — value types (`vec3`/`point3`/`dir3`/4×4 transform), curve evaluation
  (Bézier/B-spline/NURBS point + derivatives), surface evaluation (tensor-product
  Bézier/B-spline/NURBS point + partials + normal), and elementary surfaces
  (plane/cylinder/cone/sphere) point + normal. Clean-room from *The NURBS Book* +
  de Casteljau; verified by host analytic unit tests AND native-vs-OCCT numeric
  parity on the simulator within a tight fp64 tolerance. Depends on nothing (it is
  the base of the native stack); it is the foundation that `native-topology` and
  every later native capability build on.

### Modified Capabilities
<!-- none — native-math is purely additive: a new OCCT-free library plus its
     tests. No cc_* signature or POD struct changes, and it is NOT wired into the
     active engine in this change (engine wiring begins with native-topology /
     native-construction). -->

## Impact

- **Contract / ABI**: none. No `cc_*` signature or POD struct layout changes; the
  library is not yet reachable through the facade.
- **Engine**: none. `NativeEngine` does not consume this yet; the OCCT engine
  remains the sole active implementation. Wiring begins in `native-topology`.
- **Build**: new OCCT-free `src/native/math/` sources; a host CTest target
  (`clang++ -std=c++20`, no OCCT, no simulator) and a simulator native-vs-OCCT
  parity target (OCCT linked only in the parity test, never in the library).
- **Determinism / precision**: fp64 throughout with a fixed, reproducible
  evaluation order — exact modeling precision, matching the OCCT oracle within a
  tight tolerance.
- **Risk**: numeric divergence from OCCT conventions (knot multiplicity, weight
  normalization, parameter ranges, normal orientation) — bounded by the
  native-vs-OCCT parity gate; the host analytic tests independently pin the math
  to known closed-form values so a convention mismatch cannot pass silently.
