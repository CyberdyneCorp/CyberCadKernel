# Tasks — add-native-ssi-seeding (SSI Stage S2)

Verification levels: **host** = OCCT-free host CTest (known-branch-count native
pairs: every seed on both surfaces ≤ tol; ≥1 seed per known transversal branch;
dedup → expected branch count; near-tangent fixtures report `deferredTangent`, not a
seed); **sim** = native-vs-OCCT branch **recall** vs `GeomAPI_IntSS` on the
simulator (recall figure + per-seed on-surface residual + deferred-tangent count).
SSI is INTERNAL — no `cc_*` entry point is added or exercised; the seeder is asserted
at the `cybercad::native::ssi` C++ boundary, exactly like S1 / native-math. The
refine (and thus a useful seeder) is under **`CYBERCAD_HAS_NUMSCI`**.

## 1. Seed types + surface adapter
- [x] 1.1 Add `seed.h` — `Seed { u1,v1,u2,v2; Point3 point; onSurfResidual }`,
  `SeedSet { seeds; candidateRegions; deferredTangent }`, and a recall-report struct
  for the sim gate. OCCT-free. (**host**)
- [x] 1.2 Small surface adapter over the native-math `point`/`dU`/`dV`/`normal`
  interface + its `[u0,u1]×[v0,v1]` param box, so elementary / torus / Bézier /
  B-spline / NURBS all flow through one subdivision path. (**host**)

## 2. Per-patch AABB bounds (`patch_bounds.h`)
- [x] 2.1 **Freeform** (B-spline / Bézier / NURBS): AABB from the **control-net
  convex hull** — min/max of the poles influencing a param sub-box (sound bound by
  the convex-hull property; rational poles for NURBS, wᵢ > 0). (**host**)
- [x] 2.2 **Elementary + torus**: analytic AABB over a param sub-box where derivable,
  else a **sampled** bound inflated by a derivative/Lipschitz safety margin so the
  box stays conservative. (**host**)
- [x] 2.3 Disjoint-AABB test (with tolerance) used by the prune step. (**host**)

## 3. Recursive subdivision + pruning (`seeding.cpp`)
- [x] 3.1 Recursion over patch **pairs**: prune disjoint-AABB pairs; emit pairs below
  the size/depth threshold as **candidate regions**; else split the larger patch on
  its longer param direction and recurse. Bounded by max depth + min patch size
  (scale/tol-derived) so it always terminates. (**host**)
- [x] 3.2 Depth / min-patch-size are tolerance-scaled parameters that trade recall
  for cost; documented as the completeness knob. (**host**)

## 4. Refine each candidate (substrate `least_squares`)  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 For each candidate region, seed `least_squares` (from `native-numerics`)
  at the region center with residual `r = A.point(u1,v1) − B.point(u2,v2)` (m=3,
  n=4), **clamp** `(u1,v1,u2,v2)` to each surface's range, re-check the residual, and
  emit a `Seed` when `onSurfResidual ≤ tol`. (**host**)
- [x] 4.2 A region whose refine fails to converge OR whose Jacobian is near-tangent /
  ill-conditioned (‖n₁ × n₂‖ ≈ 0 at the solution) is **NOT** seeded — increment
  `deferredTangent` (an S4 gap), never fabricate a seed. (**host**)

## 5. Branch dedup (spatial clustering)
- [x] 5.1 Cluster converged seeds by 3D proximity (tol-scaled radius; ties by param
  proximity) and keep one representative per cluster → ~one seed per distinct
  transversal branch/loop. (**host**)

## 6. Transversal-only scope + honest deferral (S4 gaps, never faked)
- [x] 6.1 **Transversal** branches (`n₁ × n₂ ≠ 0`) are the S2 target and get a seed.
  Near-tangent / coincident / degenerate configurations are **deferred to S4** —
  counted in `deferredTangent` and reported, never assigned a fabricated seed.
  (**host**)
- [x] 6.2 Documented that the `SeedSet` is the input contract for **S3** marching
  (one WLine per seed) and that `deferredTangent > 0` is the honest S4 signal
  (`native_ssi.h` namespace doc / `seeding.h` header + SSI-ROADMAP S2 entry). (**host**)

## 7. Completeness = measured recall (not a blind 100% claim)
- [x] 7.1 Report **branch recall** = (native transversal branches with ≥1 seed) ÷
  (true transversal branches). Small-loop misses from too-shallow subdivision reduce
  recall and are reported, not hidden. (**host** + **sim**)

## 8. Verification (two gates)
- [x] 8.1 Host known-branch-count suite: for each native pair with a known
  transversal branch count (NURBS bumps crossing in a loop; skew cyl∩cyl;
  sphere piercing a freeform patch; non-coaxial cone∩cyl), assert every seed lies on
  both surfaces ≤ tol, ≥1 seed per known branch, dedup → expected count, and
  near-tangent fixtures report `deferredTangent`. No OCCT. (**host**)
- [x] 8.2 Sim native-vs-OCCT recall parity: build the same operands as OCCT
  `Geom_*Surface`, run `GeomAPI_IntSS`, count branches, and report native branch
  **recall** + per-seed on-surface residual + deferred-tangent count at the SSI C++
  boundary. Recall is reported (target: high on transversal freeform + non-coaxial
  quadric pairs), not asserted 1.0 blindly. (**sim**)
- [x] 8.3 `openspec validate add-native-ssi-seeding --strict` green; S2 marked in
  progress / done and the S3 marching on-ramp (seeds → WLines) noted in
  `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md`. (**host**)

## Deferred to S3 / S4 / OCCT (NOT in S2 seeding scope — honest)

- [ ] **Tracing** the full intersection curve from a seed (marching / WLine, adaptive
  step + re-project + B-spline fit) → **S3**.
- [ ] **Near-tangent** seeding (`n₁ × n₂ → 0`: higher-order predictor / step control)
  → **S4**; S2 reports these as `deferredTangent`, never faked.
- [ ] **Coincident / overlapping-surface** detection and **degenerate** (cusp /
  singular param) seeding → **S4** + OCCT fallback.
- [ ] Closing the completeness gap (guaranteeing every small loop is found) — S2 only
  **measures** recall; hardening subdivision depth/heuristics is ongoing.
