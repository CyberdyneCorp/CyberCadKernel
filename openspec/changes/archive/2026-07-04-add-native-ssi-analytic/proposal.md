## Why

Surface-Surface Intersection (SSI) is the keystone of the drop-OCCT endgame:
`SSI-ROADMAP.md` stages it analytic-first (S1) → seeding (S2) → marching (S3) →
tangent robustness (S4) → the curved-booleans payoff (S5). **S1 is the highest
bang-for-buck slice**: for the elementary-surface pairs that dominate CAD
primitives (plane / cylinder / cone / sphere / torus), the intersection is a
**closed-form conic** — a line, circle, ellipse, parabola, hyperbola, or (for
plane∩torus) the roots of a low-degree planar polynomial. No marching, no
seeding, no substrate root-finder for the simple pairs: pure clean-room geometry.

We already ship one instance of this (the axis-aligned box∩cylinder analytic
curved slice in `src/native/boolean/`). S1 generalizes it into a proper native
SSI module so that S5 can build **elementary-pair curved booleans** on top of
exact curves, with the same discipline every prior native tier used: a narrow
**verified** slice with an **explicit OCCT fallback** for everything out of scope.

SSI is an **internal** capability — it is consumed by native booleans/blends, not
exposed on the `cc_*` C ABI. So there is **no ABI change** and it is verified at
the SSI-function level (native curves vs OCCT), exactly like native-math and
native-topology parity, not through a `cc_*` entry point.

## What Changes

- Add a native, OCCT-free **analytic SSI module** under `src/native/ssi/` that
  takes two native analytic surfaces (`math::{Plane,Cylinder,Cone,Sphere,Torus}`)
  and returns native intersection curve(s) — `Line` / `Circle` / `Ellipse` /
  conic (parabola / hyperbola) built from native-math primitives — that provably
  lie on **both** surfaces, for the **closed-form conic family** only.
- Add a **pair-dispatch / classifier** that inspects the two surface types and
  their relative placement (coaxial / parallel / perpendicular / oblique) and
  routes to a closed-form handler, or returns a typed **`NOT_ANALYTIC`** result
  (with a reason) for out-of-scope pairs. NOT-ANALYTIC pairs are the deferral
  seam to S2/S3 marching or OCCT — **never faked**.
- Supported closed-form handlers (S1 scope):
  - **plane ∩ plane** → line (or coincident / parallel-none).
  - **plane ∩ sphere** → circle (or tangent point / none).
  - **plane ∩ cylinder** → parallel lines / circle / ellipse, by plane orientation
    vs the cylinder axis (perpendicular → circle, parallel → line pair, oblique →
    ellipse).
  - **plane ∩ cone** → circle / ellipse / parabola / hyperbola / degenerate
    (point / line pair through the apex), by the classic conic-section rule.
  - **plane ∩ torus** → the intersection is a planar quartic; solve it with the
    native `numerics` polynomial-root substrate (e.g. the Villarceau / axis-perp
    circle families and the general in-plane quartic). The **general oblique
    plane∩torus** may return NOT-ANALYTIC (deferred) if it does not reduce to a
    closed-form curve family — honestly, not faked.
  - **sphere ∩ sphere** → circle / tangent point / none.
  - **sphere ∩ cylinder, COAXIAL** → up to two circles / tangent circle / none.
  - **sphere ∩ cone, COAXIAL** → up to two circles / tangent circle / none.
  - **cylinder ∩ cylinder, COAXIAL or PARALLEL** → coincident / two parallel lines
    (equal-radius parallel) / none; coaxial equal-radius = coincident, coaxial
    unequal = none.
  - **cylinder ∩ cone, COAXIAL** → circle(s) (the cone crosses the cylinder radius
    at up to two heights) / tangent circle / none.
- **Explicitly OUT of S1 (return NOT-ANALYTIC → defer, never fake):** general /
  skew cylinder∩cylinder (a quartic space curve), general cone∩cone, general
  cone∩cylinder (non-coaxial), torus∩curved (anything but the plane∩torus above),
  ANY NURBS / Bézier / B-spline / freeform surface, and any **near-tangent /
  coincident** configuration where the closed-form branch is numerically unsafe.
- **Self-verify each analytic result:** sample the returned curve(s) and confirm
  every sample lies on both surfaces within tolerance before returning; if the
  check fails, downgrade to NOT-ANALYTIC rather than emit a wrong curve.
- Native code stays **OCCT-free**: it may use the `src/native/numerics` substrate
  (NumPP/SciPP polynomial roots / `solve`) for the plane∩torus quartic; the simple
  pairs need **no solver at all**.

**No `cc_*` ABI change.** SSI is internal. The only surface it exposes is the
native `cybercad::native::ssi` C++ API, consumed by native booleans (S5). The
public C facade is untouched.

## Capabilities

### New Capabilities
- `native-ssi`: native, OCCT-free, closed-form analytic surface-surface
  intersection (SSI Stage S1) for the elementary-surface conic family, returning
  native curve(s) that lie on both surfaces and match OCCT `GeomAPI_IntSS`, plus a
  pair-dispatch that returns NOT-ANALYTIC for out-of-scope pairs so they defer to
  S2/S3/OCCT. Consumes `native-math` (surface + curve primitives) and, for the
  plane∩torus quartic only, `native-numerics` (polynomial roots). Verified at the
  SSI-function level; the on-ramp to curved booleans (S5). No `cc_*` change.

### Modified Capabilities
<!-- none — native-ssi is a new internal module. It does not modify any cc_*
     signature, POD struct, or existing native capability; it only consumes
     native-math (and native-numerics for the plane∩torus quartic). -->

## Impact

- **ABI**: none. SSI is an internal native capability; no `cc_*` entry point,
  signature, or POD struct changes.
- **Build**: adds `src/native/ssi/` (OCCT-free, header-heavy math). The
  plane∩torus quartic path depends on `native-numerics` and so is compiled only
  under `CYBERCAD_HAS_NUMSCI`; the solver-free elementary pairs build without it.
- **Verification**: two gates from `SSI-ROADMAP.md` — **host analytic** (known
  conics; every sampled curve point lies on both surfaces within tol; no OCCT) +
  **sim native-vs-OCCT** (`GeomAPI_IntSS` parity on the elementary pairs). This is
  the same internal parity discipline as native-math / native-topology.
- **Roadmap**: implements `SSI-ROADMAP.md` **S1** and is the on-ramp to S5 curved
  booleans (elementary-face restricted). S2–S4 (freeform seeding / marching /
  tangent robustness) are explicitly future stages that consume the same
  dispatch's NOT-ANALYTIC deferral seam.
- **Risk**: honest scope — the closed-form family is finite and each result is
  self-verified; anything unsafe or out-of-family returns NOT-ANALYTIC and defers,
  so S1 can never emit a wrong or leaky curved result.
