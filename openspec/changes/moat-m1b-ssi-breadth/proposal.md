# Proposal — moat-m1b-ssi-breadth (MOAT M1 SSI breadth: general non-coaxial / skew analytic quadric intersections)

## Why

The MOAT-ROADMAP M1 track ("General freeform surface–surface intersection robustness")
records **skew cylinder∩cylinder** as a `NotAnalytic` decline and names, as its highest-value
tractable breadth slices, "skew (non-parallel, non-coaxial) cylinder∩cylinder", "cylinder∩cone /
cone∩cone general relative pose", and "sphere∩cylinder / sphere∩cone off-axis".

The decline is precise but narrow: it is an **S1 (closed-form) decline only**. S1's analytic
dispatch has no closed form for a skew cyl∩cyl quartic (correct — none exists), so it honestly
returns `NotAnalytic`. But the **S2 seeder + S3 marching tracer already trace these general
poses** — empirically confirmed on host (this change's probe, later folded into the regression
suite): skew cyl∩cyl (gap + oblique tilt → a single connected quartic loop), cyl∩cone off-axis
oblique, cone∩cone general, sphere∩cyl off-axis, sphere∩cone off-axis all trace to a closed /
boundary-exit WLine with a maximum on-both-surfaces residual ≈ 1e-11.

The gap is **verification and regression coverage**, not tracer capability. The existing S3
parity suite (`tests/sim/native_ssi_marching_parity.mm`) covers exactly ONE skew-quadric pose —
the symmetric orthogonal-**intersecting**-axis cyl∩cyl that splits into two loops — plus
freeform and S4-c/d/e cases. The genuinely general poses (nonzero axis-to-axis **gap** AND an
**oblique** angle, and the mixed cone/sphere non-coaxial families) have **no host regression
test and no native-vs-OCCT parity gate**. Until they are locked against the `GeomAPI_IntSS`
oracle, a future marcher/seeder edit could silently regress them and the roadmap decline would
stay overstated.

This change lands the breadth as a **verified, regression-locked family**: the general
non-coaxial / skew analytic quadric intersections, each traced by the already-shipped S3 tracer
and asserted native-vs-OCCT (curve count/type, densely-sampled on-curve + on-both-surfaces
deltas, arc-length) against `GeomAPI_IntSS`. It is **test + spec only**; `src/native/**` stays
byte-identical (no tracer change is needed — the capability is already there).

## What

- **Host gate (Gate A, OCCT-free).** New `tests/native/test_native_ssi_marching.cpp` cases that
  trace each general-pose family and assert the S3 CONTRACT self-consistently: every traced node
  lies on BOTH native surfaces to ≤ 1e-9, the branch closes / exits cleanly, `nearTangentGaps == 0`,
  and the traced count matches the analytically-known component count (one connected quartic loop
  for a fully-penetrating skew pose; the documented count otherwise).

- **Sim gate (Gate B, native-vs-OCCT parity).** New `pair*()` cases in the existing
  `tests/sim/native_ssi_marching_parity.mm`, registered in `main()`, reusing the shipped
  `reportPair` harness (welds OCCT `GeomAPI_IntSS` branches into transversal components, matches
  count/closed-count/length + densely-sampled on-curve/on-surface deltas). **Decisively landed**
  (both gates green, matching count/type + ≈1e-5 deltas):
  - **skew cyl∩cyl (general)** — axes with a nonzero minimum distance AND an oblique 60° tilt → one
    connected quartic loop (distinct from the existing orthogonal-intersecting two-loop case);
    bounded by the smaller finite cylinder's single penetration region.
  - **sphere∩cone off-axis** — the finite sphere admits the near cone nappe once → a single loop.

  **Honestly declined tail** (measured on the oracle, no fake, no weakened tolerance): general
  cone∩cone, off-axis cyl∩cone, and off-axis sphere∩cyl — poses where an UNBOUNDED OCCT quadric
  pierces the other operand more than once along its infinite extent (`Geom_ConicalSurface` /
  `Geom_CylindricalSurface` are infinite; the native adapters are finite patches), so the finite
  native trace cannot match the infinite-locus oracle without domain-clipped oracle surfaces,
  and the second loop is a seeding-recall miss at practical seed densities.

- **Honest-decline preservation.** The S1 analytic dispatch STILL returns `NotAnalytic` for skew
  cyl∩cyl and the other non-closed-form pairs — that decline is CORRECT (no closed form) and is
  unchanged. This change verifies the S2/S3 *marching* path that the S1 decline defers to; it does
  NOT add a fake closed form. The existing `NotAnalytic` unit assertions (`test_native_ssi.cpp`)
  stay green.

## Impact

- `src/native/**` — **byte-identical** (verified by `git diff src/native`). No tracer/seeder edit
  is required; the capability is already shipped.
- Tests — additive: new host cases + new sim parity cases (existing cases frozen).
- `cc_*` ABI — **unchanged** (SSI is internal; asserted at the C++ boundary, no facade entry point).
- Tessellator (`src/native/tessellate/`) — **UNTOUCHED**.
- Spec — one new requirement in `native-ssi` (general non-coaxial / skew analytic quadric
  breadth, verified vs OCCT).
- Roadmap — M1 status updated: the skew-cyl decline is re-scoped to "S1-closed-form only; the
  S2/S3 marcher traces general poses, now verified vs OCCT and regression-locked".
