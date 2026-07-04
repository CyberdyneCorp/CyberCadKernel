## Why

Surface-Surface Intersection is staged analytic-first in `SSI-ROADMAP.md`: S1
(closed-form conics, **shipped**) → **S2 seeding** → S3 marching → S4 tangent
robustness → S5 curved booleans. S1's dispatch returns an honest `NotAnalytic` for
everything outside the degree-≤2 closed-form family — **freeform (NURBS / Bézier /
B-spline) pairs** and the **non-closed-form quadric pairs** (skew cylinder∩cylinder,
general cone∩cone, non-coaxial cone∩cylinder / sphere∩cylinder / sphere∩cone,
oblique plane∩torus, torus∩curved). Those pairs are exactly what a general kernel
must intersect, and they have **no closed form**: the curve must be traced
numerically (S3). But marching cannot start until it has a **seed point on every
branch** — and the roadmap's substrate eval is explicit that generic `fsolve` from
a naive guess fails to *find* intersection points on freeform surfaces (0/7). So the
missing capability is exactly **seeding**: robustly producing ≥1 seed per branch to
hand to S3. That is S2.

S2 is the first SSI stage to lean on the numeric substrate: the roadmap's whole
premise is that local re-projection (Newton/LM) is *provided* by `native-numerics`
but *finding* a first point is not — S2 supplies the "finding" via recursive
subdivision + AABB-overlap pruning, then uses the substrate's `least_squares` to
refine each candidate region to a real intersection point on both surfaces.

SSI is an **internal** capability (consumed by S3/S5, not the `cc_*` C ABI), so
there is **no ABI change**; it is verified at the `cybercad::native::ssi` C++
boundary, exactly like S1 and native-math parity.

## What Changes

- Extend the native, OCCT-free `native-ssi` module (`cybercad::native::ssi`) with a
  **subdivision seeder** that, given two native surfaces (elementary + torus +
  freeform NURBS/Bézier/B-spline via their `point` / `dU` / `dV` / `normal`
  evaluators from `src/native/math/`), returns a set of **seed points** — one per
  distinct **transversal** intersection branch/loop — each with its parameters on
  both surfaces `(u1,v1,u2,v2)` and a proof that the point lies on **both** surfaces
  within tolerance.
- **Recursive patch subdivision + AABB-overlap pruning.** Split each surface's
  `[u,v]` domain into patches; bound each patch by an AABB — from the **control-net
  convex hull** for B-spline / Bézier / NURBS (the poles bound the surface), and an
  analytic or sampled bound for the elementary surfaces + torus. Prune every patch
  pair whose AABBs are disjoint; recurse on overlapping pairs down to a size / depth
  threshold, yielding a set of small **candidate regions** that bracket the
  intersection.
- **Substrate refine.** For each surviving candidate region, seed
  `least_squares` (from `native-numerics`) with the region-center parameters and
  drive the residual `S1(u1,v1) − S2(u2,v2) = 0` (a 3-vector, 4 unknowns) to a point
  on both surfaces, **clamping** `(u1,v1,u2,v2)` to each surface's param range. A
  converged refine with on-both-surfaces residual ≤ tol is a valid seed; a refine
  that fails to converge or ill-conditions is dropped (not faked).
- **Branch dedup (spatial clustering).** Cluster the refined seeds by 3D proximity
  (and parameter proximity) so seeds landing on the same branch collapse to ~one
  representative seed per branch. The output is the deduped seed set — the input
  contract for S3 marching.
- **Honest transversal-only scope.** S2 targets **transversal** intersections
  (surfaces cross cleanly; the refine is well-conditioned). **Near-tangent /
  coincident / degenerate** seeding — where `n₁ × n₂ → 0` and `least_squares`
  ill-conditions — is **deferred to S4**: such a branch is **reported as a known S4
  gap**, never assigned a fabricated seed.
- **Completeness is a measured figure, not a claim.** Too-shallow subdivision can
  miss a small loop; S2 reports **branch recall** (seeds found ÷ true branches) vs
  OCCT `GeomAPI_IntSS` on the sim, rather than asserting 100% blindly. Missing a
  small loop is the acknowledged honest failure mode.

**No `cc_*` ABI change.** SSI is internal. The only surface exposed is the native
`cybercad::native::ssi` seeding API, consumed by S3 marching (and eventually S5).
The public C facade is untouched. The refine path depends on `native-numerics`, so
the seeder is compiled under `CYBERCAD_HAS_NUMSCI`; native code stays OCCT-free and
uses only the NumPP/SciPP substrate.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the existing native-ssi capability (added at S1). -->

### Modified Capabilities
- `native-ssi`: extend the S1 closed-form module with **SSI Stage S2 subdivision
  seeding** — recursive patch-AABB-overlap subdivision + `native-numerics`
  `least_squares` refine that finds ≥1 seed point per **transversal** intersection
  branch for the freeform (NURBS) and non-closed-form quadric pairs S1 defers, each
  seed on both surfaces within tol with its `(u1,v1,u2,v2)`, deduped to ~one seed
  per branch. Branch **recall** is measured vs OCCT `GeomAPI_IntSS`;
  near-tangent/coincident/degenerate seeding is deferred to S4 (reported, not
  faked); small-loop completeness is a measured recall figure. Consumes
  `native-math` (surface point/dU/dV/normal + control nets) and `native-numerics`
  (`least_squares`, closest-point). No `cc_*` change. The output seeds are the input
  contract for S3 marching.

## Impact

- **ABI**: none. SSI is an internal native capability; no `cc_*` entry point,
  signature, or POD struct changes.
- **Build**: adds the seeder to `src/native/ssi/` (recursive subdivision + AABB
  pruning + dedup are OCCT-free header/TU code). The **refine** path calls
  `native-numerics` `least_squares` and so is compiled only under
  `CYBERCAD_HAS_NUMSCI`; the subdivision/pruning geometry needs no substrate to
  build, but a *useful* seeder requires the refine, so the S2 seeding entry point is
  gated on `CYBERCAD_HAS_NUMSCI`.
- **Verification**: two gates from `SSI-ROADMAP.md` — **host** (known-branch-count
  test pairs: every seed lies on both surfaces ≤ tol; ≥1 seed per known transversal
  branch; dedup collapses to the expected branch count; no OCCT) + **sim
  native-vs-OCCT** (branch **recall** vs `GeomAPI_IntSS` on freeform + non-coaxial
  quadric pairs). Same internal parity discipline as S1 / native-math.
- **Roadmap**: implements `SSI-ROADMAP.md` **S2** and is the input contract for S3
  marching (each seed → one walked WLine). S1's `NotAnalytic` deferral seam is the
  entry point that routes pairs into S2.
- **Risk (honest)**: completeness — a too-shallow subdivision can miss a small loop;
  reported as a measured recall figure, not hidden. Near-tangent/coincident/
  degenerate branches are out of transversal scope and deferred to S4 with the
  measured gap, never faked. Refine can converge to a spurious/duplicate point;
  clustering + the on-both-surfaces self-check guard against emitting a bad seed.
