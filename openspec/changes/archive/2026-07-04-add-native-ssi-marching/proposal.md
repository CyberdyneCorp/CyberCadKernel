## Why

Surface-Surface Intersection is staged analytic-first in `SSI-ROADMAP.md`: S1
(closed-form conics, **shipped**) → S2 seeding (**shipped**) → **S3 marching** → S4
tangent robustness → S5 curved booleans. For the pairs S1's dispatch returns
`NotAnalytic` for — **freeform (NURBS / Bézier / B-spline)** and the
**non-closed-form quadric** pairs (skew cylinder∩cylinder, general cone∩cone,
non-coaxial cone∩cylinder / sphere∩cylinder / sphere∩cone, oblique plane∩torus,
torus∩curved) — there is **no closed form**: the intersection curve must be traced
numerically. S2 supplied a **seed point on every transversal branch** (the
`SeedSet`, one seed per branch). But a seed is a single point, not a curve — and S5
curved booleans need the **whole curve** to split a curved face. The missing
capability is exactly **tracing**: walking the full intersection curve out from each
seed. That is S3.

S3 is the second SSI stage to lean on the numeric substrate, in the way the roadmap
earmarks: the eval is explicit that local **re-projection** (Newton/LM back onto
both surfaces) is *provided* by `native-numerics`, and that is precisely the S3
**corrector**. What the substrate does not buy — a robust global tracer that
predicts the next point and knows when to stop — S3 builds on top: a
predictor-corrector march (tangent predictor + substrate corrector + adaptive step +
loop/boundary termination) followed by a B-spline fit, producing one WLine per seed.

SSI is an **internal** capability (consumed by S5, not the `cc_*` C ABI), so there
is **no ABI change**; it is verified at the `cybercad::native::ssi` C++ boundary,
exactly like S1, S2, and native-math parity.

## What Changes

- Extend the native, OCCT-free `native-ssi` module (`cybercad::native::ssi`) with a
  **marching-line tracer** that, given the S2 `SeedSet` and the two native surfaces
  (elementary + torus + freeform via their `point` / `dU` / `dV` / `normal`
  evaluators from `src/native/math/`), **traces the full intersection curve** of
  each **transversal** branch and returns one traced **WLine** per seed — a polyline
  fitted to a B-spline — whose sampled points provably lie on **both** surfaces
  within tolerance.
- **Predictor (tangent step).** At the current curve point, the intersection tangent
  is `t = normalize(n₁ × n₂)` where `n₁, n₂` are the two surface unit normals
  (`native-math` `dU × dV`). Step `P + h·t` to a predicted next point, with an
  **adaptive** step `h`: shrink where the curve bends sharply or the corrector
  strains (many iterations / large correction), grow where the walk is easy — bounded
  by a scale/tol-derived min and max step.
- **Corrector (substrate re-projection).** Re-project the predicted point back onto
  **both** surfaces by driving `A.point(u1,v1) − B.point(u2,v2) = 0` with
  `native-numerics` `least_squares` (seeded by `closest_point_on_surface` on each
  operand), landing the point on the intersection curve and advancing
  `(u1,v1,u2,v2)`. A corrector that **converges** with on-both-surfaces residual
  ≤ tol advances the march; a corrector that **fails** (does not converge, or
  `‖n₁ × n₂‖ → 0` near-tangent) **STOPS** the march — it does **not** force a point.
- **Termination + dedup.** Stop the walk on **loop closure** (the point returns
  within a tol-scaled radius of the seed / walk start, closing the curve) or on a
  **boundary exit** (a corrected param leaves `[u0,u1]×[v0,v1]` on either surface —
  the branch runs off the patch). March in **both directions** from the seed for an
  open branch. **Dedup** seeds whose march retraces an already-traced branch (a
  second S2 seed that lands on the same curve) so the output is one WLine per
  distinct branch.
- **B-spline fit.** Fit a B-spline through the traced polyline via `native-math`
  (the existing B-spline fitting), yielding a single `Geom`-quality curve per branch
  with a `value(t)` evaluator, self-verified on both surfaces before it is returned.
- **Honest transversal-only scope.** S3 traces **transversal** branches only.
  **Near-tangent / coincident / degenerate** marching (`n₁ × n₂ → 0`), branch-point
  splitting, and self-intersection resolution are **deferred to S4**: a march that
  enters a near-tangent region is traced **up to the tangent**, and the untraced
  remainder is **flagged as an S4 gap** (reported), never faked. S3 NEVER fabricates
  curve points nor claims a full trace that stopped short.

**No `cc_*` ABI change.** SSI is internal. The only surface exposed is the native
`cybercad::native::ssi` marching API, consumed by S5 curved booleans. The public C
facade is untouched. The corrector and B-spline fit depend on `native-numerics` /
`native-math`, so the tracer is compiled under `CYBERCAD_HAS_NUMSCI`; native code
stays OCCT-free and uses only the NumPP/SciPP substrate.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the existing native-ssi capability (added at S1, extended at S2). -->

### Modified Capabilities
- `native-ssi`: extend the S1 closed-form / S2 seeding module with **SSI Stage S3
  marching** — a predictor-corrector tracer that, from each S2 seed, walks the full
  intersection curve of a **transversal** branch (tangent `n₁ × n₂` predictor +
  substrate `least_squares` re-projection corrector + adaptive step + loop / boundary
  termination + B-spline fit), producing one WLine per seed whose sampled points lie
  on both surfaces within tol AND coincide with OCCT `IntPatch` / `GeomAPI_IntSS`
  (points / length / branch-count) within tolerance. Near-tangent / coincident /
  degenerate marching is deferred to S4 (traced up to the tangent, flagged, not
  faked). Consumes the S2 `SeedSet`, `native-math` (point/dU/dV/normal + B-spline
  fit) and `native-numerics` (`least_squares`, closest-point). No `cc_*` change. The
  output WLines are the input contract for S5 curved booleans.

## Impact

- **ABI**: none. SSI is an internal native capability; no `cc_*` entry point,
  signature, or POD struct changes.
- **Build**: adds the tracer to `src/native/ssi/` (predictor step + termination +
  dedup are OCCT-free header/TU code). The **corrector** (re-projection) and the
  **B-spline fit** call `native-numerics` / `native-math`, so the S3 marching entry
  point is compiled only under `CYBERCAD_HAS_NUMSCI` (like the S2 seeder), gated the
  same way `native-numerics` itself is.
- **Verification**: two gates from `SSI-ROADMAP.md` — **host** (known-shape native
  pairs: every sampled WLine point lies on both surfaces ≤ tol; loop branches close;
  open branches exit at a boundary; near-tangent fixtures trace up to the tangent and
  flag an S4 gap, never fabricate a point) + **sim native-vs-OCCT** (curve parity vs
  OCCT `IntPatch` / `GeomAPI_IntSS`: branch count, curve length, and sampled
  point-to-curve distance within tol on transversal freeform + non-coaxial quadric
  pairs). Same internal parity discipline as S1 / S2 / native-math.
- **Roadmap**: implements `SSI-ROADMAP.md` **S3** and is the input contract for S5
  curved booleans (each traced WLine splits a curved face). S2's `SeedSet` is the
  entry point (one WLine per seed); S1's `NotAnalytic` routed those pairs here.
- **Risk (honest)**: the near-tangent moat — a march that enters `n₁ × n₂ → 0` is
  stopped and the remainder flagged as an S4 gap, never faked; branch-point splitting
  and self-intersection are out of transversal scope and deferred to S4. The
  corrector can converge off-branch (onto a different sheet) — the on-both-surfaces
  self-check plus continuity-with-the-previous-point guard reject a jump. Step-size
  mis-tuning trades trace cost against fidelity (adaptive `h` bounded by scale/tol);
  the B-spline fit residual is self-verified on both surfaces before the curve is
  returned. Whatever S3 cannot robustly trace falls back to OCCT and is reported with
  the measured gap.
