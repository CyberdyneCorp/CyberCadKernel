# Design — add-native-ssi-seeding (SSI Stage S2)

## Context

`SSI-ROADMAP.md` stages SSI analytic-first. **S1** (shipped) returns closed-form
conics for the elementary-pair family and an honest `NotAnalytic` for everything
else. **S2** is the next stage: for the pairs S1 defers — **freeform** (NURBS /
Bézier / B-spline) and the **non-closed-form quadric** pairs (skew cyl∩cyl, general
cone∩cone, non-coaxial cone∩cyl / sphere∩cyl / sphere∩cone, oblique plane∩torus,
torus∩curved) — there is no closed form, so the intersection curve must be traced
numerically. Tracing (S3) needs a **start point on every branch**. The roadmap's
substrate eval is blunt about the gap: local re-projection (Newton/LM) is provided,
but *finding* a first point from a naive seed fails on freeform (generic `fsolve`
0/7). **S2 supplies the finding**: subdivision + AABB-overlap pruning brackets the
intersection, then the substrate's `least_squares` refines each bracket to a real
point on both surfaces.

The method is **locked to CLEAN-ROOM**: recursive subdivision + AABB pruning +
substrate refine derived from `SSI-ROADMAP.md`, using OCCT (`IntPatch` PrmPrm /
`IntWalk` seeding, `IntPolyh`) strictly as a verification **oracle** for branch
recall, never copied.

SSI is an **internal** capability (consumed by S3/S5, not the `cc_*` ABI), verified
at the `cybercad::native::ssi` C++ boundary — the same discipline as S1 and
native-math parity — with **no ABI change**.

## Goals / Non-Goals

**Goals**
- Given two native surfaces, return ≥1 **seed** per distinct **transversal**
  intersection branch/loop, each with `(u1,v1,u2,v2)` and on both surfaces ≤ tol.
- Recursive patch subdivision + per-patch AABB (control-net convex hull for
  freeform; analytic/sampled for elementary+torus) + disjoint-AABB pruning.
- Refine each surviving candidate region with `native-numerics` `least_squares`
  driving `S1(u1,v1) − S2(u2,v2) = 0`, clamped to param ranges.
- **Dedup** seeds on the same branch by spatial clustering → ~one seed per branch.
- **Measure branch recall** vs OCCT; report it, do not claim 100%.

**Non-Goals (deferred — never faked here)**
- **Tracing** the full curve from a seed → **S3** (marching / WLine).
- **Near-tangent / coincident / degenerate** seeding (the refine ill-conditions when
  `n₁ × n₂ → 0`) → **S4**: reported as a known gap, not assigned a seed.
- Guaranteed 100% completeness — a too-shallow subdivision may miss a small loop;
  that is a measured recall figure, not a promise.
- Any `cc_*` facade entry point or ABI change.

## Module shape

```
src/native/ssi/
  seeding.h / seeding.cpp   // SSI Stage S2: subdivide → prune → refine → dedup   [CYBERCAD_HAS_NUMSCI]
  patch_bounds.h            // per-surface param-domain patch + AABB (control-net hull / analytic / sampled)
  seed.h                    // Seed { u1,v1,u2,v2; Point3 p; residual } + SeedSet + recall report struct
```

The seeder consumes the **same** native-math surface interface S1 uses — every
surface exposes `point(u,v)` / `dU` / `dV` / `normal(u,v)` — via a small
`SurfaceEval`-style adapter (an evaluator + its `[u0,u1]×[v0,v1]` param box + an
AABB provider), so elementary, torus, Bézier, B-spline and NURBS all flow through
the same subdivision code. It reuses `curve.h` (frames / tolerance scale) and
`tolerance.h` from S1, and `native-numerics` `least_squares` / closest-point for the
refine.

## Result type

```cpp
struct Seed {                 // one point on ONE transversal branch, on BOTH surfaces
  double u1, v1;              // params on surface A  (clamped to A's range)
  double u2, v2;              // params on surface B  (clamped to B's range)
  math::Point3 point;         // A.point(u1,v1) ≈ B.point(u2,v2)
  double onSurfResidual;      // max ‖point − surface‖ over both surfaces (≤ tol)
};

struct SeedSet {
  std::vector<Seed> seeds;    // deduped: ~one seed per distinct transversal branch
  int candidateRegions = 0;   // surviving AABB-overlap pairs before refine (diagnostic)
  int deferredTangent = 0;    // candidate regions dropped as near-tangent/degenerate → S4
};
```

`SeedSet` is the S3 contract: S3 walks one WLine per `Seed`. `deferredTangent > 0`
is the honest S4 signal — those branches were seen but not safely seeded.

## Algorithm (clean-room)

### 1. Per-patch AABB (`patch_bounds.h`)
A **patch** is a sub-box of a surface's `[u0,u1]×[v0,v1]` param domain. Its AABB
bounds the surface *over that sub-box*:
- **B-spline / Bézier / NURBS** — the **control-net convex hull** bounds the patch:
  the poles influencing the sub-box (after conceptual refinement to the sub-box)
  bound the surface by the convex-hull property; the AABB is the min/max of those
  poles. For NURBS the rational poles `Pᵢ` (already the projected control points)
  bound the surface for `wᵢ > 0`. This gives a **tight, sound** bound without
  sampling.
- **Elementary (plane / cylinder / cone / sphere)** and **torus** — an
  analytic bound where available, else a **sampled** bound with a safety margin
  (sample `point(u,v)` on a grid over the sub-box, take min/max, inflate by a
  Lipschitz/derivative-based margin so the AABB is conservative). Elementary
  surfaces are cheap to bound analytically over a param sub-box.

### 2. Recursive subdivision + pruning (`seeding.cpp`)
Start with the two full param domains as a single patch pair. Repeatedly:
1. If the two patches' AABBs are **disjoint** → discard the pair (no intersection in
   this region — the prune step; this is where the work is saved).
2. Else if **both** patches are below the size/depth threshold → emit as a
   **candidate region**.
3. Else **split** the larger patch (or both) along its longer param direction and
   recurse on the (up to 4) sub-pairs.

The recursion is bounded by a max depth and a min patch size (both derived from the
operands' scale + tol), so it always terminates. Depth controls the recall/cost
trade-off: deeper subdivision recovers smaller loops at more cost. **Cognitive
complexity**: the recursion is one guard-clause function (prune / emit / split) in
the systems band; the split geometry and AABB test are isolated helpers.

### 3. Refine (`least_squares`) 
For each candidate region, take its center `(u1,v1,u2,v2)` as `x0` and call
`native-numerics` `least_squares` with the residual
`r(x) = A.point(u1,v1) − B.point(u2,v2)` (m=3, n=4; the extra DOF is the tangential
freedom along the curve — LM handles the rank-deficient-by-one Jacobian). After
convergence, **clamp** `(u1,v1,u2,v2)` into each surface's param box and re-check the
residual. A region whose refine converges with `onSurfResidual ≤ tol` yields a
`Seed`. A region whose refine **fails to converge** or whose Jacobian is
**near-tangent / ill-conditioned** (‖n₁ × n₂‖ ≈ 0 at the solution) is **not** turned
into a seed — it increments `deferredTangent` (an S4 gap), never a fabricated seed.

### 4. Dedup (spatial clustering)
Many candidate regions on the same branch refine to nearby points. Cluster the
converged seeds by 3D proximity (a tolerance-scaled radius; ties broken by parameter
proximity) and keep one representative per cluster, so the output is ~one seed per
distinct branch/loop. This is what makes "≥1 seed per branch" a *clean* set rather
than a cloud.

## Transversal-vs-deferred scope (honest)

| Configuration | S2 behavior |
|---|---|
| **Transversal** branch (surfaces cross, `n₁ × n₂ ≠ 0`) | seed it — refine is well-conditioned; this is the S2 target |
| Small loop under a coarse subdivision | may miss → counts against **recall** (measured, reported) |
| **Near-tangent** branch (`n₁ × n₂ → 0`) | **deferred to S4** — refine ill-conditions; increment `deferredTangent`, no seed |
| **Coincident / overlapping** surfaces | **deferred to S4** — not a discrete branch; reported, no seed |
| **Degenerate** (cusp / singular param) | **deferred to S4** — reported, no seed |

## Verification model (two gates, per SSI-ROADMAP §Verification model)

- **Host (no OCCT) — known-branch-count pairs.** Construct native pairs whose
  transversal branch count is known analytically (e.g. two NURBS bumps that cross in
  1 loop; two skew cylinders → 1 or 2 branches; a sphere piercing a freeform patch).
  Assert: (a) **every** returned seed lies on both surfaces ≤ tol; (b) **≥1** seed
  per known transversal branch; (c) dedup collapses to the expected branch count;
  (d) near-tangent fixtures report `deferredTangent` rather than emit a seed. No OCCT.
- **Sim native-vs-OCCT — recall parity.** Build the same operands as OCCT
  `Geom_*Surface`, run `GeomAPI_IntSS`, count its branches, and report native
  **branch recall** = (native branches with ≥1 seed) ÷ (OCCT transversal branches),
  plus each native seed's on-both-surfaces residual. Recall is a **reported figure**
  (with the deferred-tangent count called out), not asserted to be 1.0 blindly.
  Compared at the SSI C++ boundary; no `cc_*` call.

## Decisions

- **Control-net hull for freeform AABBs.** The convex-hull property makes the pole
  bounding box a **sound** (conservative) surface bound with no sampling — the right
  primitive for prune-based subdivision (mirrors OCCT `IntPolyh`'s bounding role,
  clean-room).
- **`least_squares` (LM), not `fsolve`.** The residual is 3 equations in 4 unknowns
  (rank-deficient by the along-curve DOF); LM's damping handles the extra DOF and the
  near-rank-deficiency gracefully, where a square `fsolve` would be ill-posed. This
  is exactly the substrate routine the roadmap earmarks for the refine.
- **`deferredTangent` is data, not an error.** A near-tangent region is a first-class
  "seen but not safely seeded → S4" outcome, reported in the `SeedSet`, matching S1's
  `NotAnalytic`-is-data stance and the roadmap's degeneracy discipline.
- **Recall is measured, not promised.** Completeness of subdivision is fundamentally
  a depth/cost trade-off; we surface it as a number so callers (and S5) know the
  honest coverage rather than trusting a false 100%.
- **Substrate-gated.** Subdivision + pruning + dedup are OCCT-free and substrate-free
  to build, but a useful seed *requires* the refine, so the S2 entry point is under
  `CYBERCAD_HAS_NUMSCI` (like `native-numerics` itself).

## Risks / Trade-offs

- **Completeness (small loops).** The dominant honest failure mode: a loop smaller
  than the min patch size is missed. Mitigation: tolerance-scaled min patch size +
  reported recall; deeper subdivision recovers more at more cost. Accepted, measured.
- **Refine to a spurious/duplicate point.** LM may converge off-branch or to a point
  another region already found. Mitigation: on-both-surfaces self-check (drop
  off-branch) + spatial-clustering dedup (collapse duplicates).
- **Near-tangent boundary classification.** Deciding "transversal vs near-tangent"
  uses a `‖n₁ × n₂‖` threshold; borderline branches route to `deferredTangent` (S4)
  rather than risk a fragile seed — matching the roadmap's fallback stance.
- **AABB tightness for sampled elementary bounds.** A sampled+inflated bound is
  looser than the analytic one and can over-produce candidate regions (cost, not
  correctness — dedup cleans the output). Analytic bounds used where derivable.
