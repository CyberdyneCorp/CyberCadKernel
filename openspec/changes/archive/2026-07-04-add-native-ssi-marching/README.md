# add-native-ssi-marching

SSI Stage **S3** — native, OCCT-free **marching-line tracer** (WLine) for
surface-surface intersection. Given the **S2 `SeedSet`** (one seed per distinct
**transversal** branch, each with its `(u1,v1,u2,v2)` and proof it lies on both
surfaces), **trace the full intersection curve** of each branch and fit a B-spline
through the traced polyline — one WLine per seed.

The method (locked to **clean-room** predictor-corrector marching, OCCT
`IntWalk`/`IntPatch` `WLine` as the reference **oracle** only):

- **Predictor.** At the current point compute the intersection tangent
  `t = normalize(n₁ × n₂)` from the two surface normals (`native-math` `dU`/`dV` →
  `normal`); step `P + h·t` with an **adaptive** `h` (shrink on high curvature /
  corrector strain, grow when the walk is easy).
- **Corrector.** Re-project the predicted point back onto **both** surfaces with the
  `native-numerics` substrate (`least_squares` on `A.point(u1,v1) − B.point(u2,v2)`,
  seeded by closest-point) so it lands **on the intersection**. If the corrector
  **fails to converge** — near-tangent, `‖n₁ × n₂‖ → 0` — **STOP** the march and
  flag the remainder as a near-tangent **S4 gap**; never force a bogus point.
- **Termination.** Stop on **loop closure** (the walk returns near its start) or on a
  **boundary exit** (a param leaves a surface's range). **Dedup** marches that
  retrace the same branch (a second seed on an already-traced curve).
- **Fit.** Fit a **B-spline** through the traced polyline via `native-math`, so each
  branch is a single `Geom`-quality WLine curve.

**Honest scope.** S3 traces **transversal** branches only. **Near-tangent /
coincident / degenerate** marching, branch-point splitting, and self-intersection
resolution are **deferred to S4** — a march that enters a near-tangent region is
traced up to the tangent, and the remainder is **flagged as an S4 gap**, never
faked. S3 NEVER fabricates curve points nor claims a full trace that stopped short.

SSI is an **internal** kernel capability: **no `cc_*` ABI change**. Native code
stays OCCT-free (uses the NumPP/SciPP substrate behind `native-numerics`); the S3
marching module is compiled under `CYBERCAD_HAS_NUMSCI`. OCCT (`IntWalk`,
`IntPatch` `WLine`/`ALine`, `GeomAPI_IntSS`) is used strictly as a verification
**oracle**, never copied. These traced curves are the input contract for **S5**
curved booleans (they split the curved faces).

Reference: [../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S3 scope + verification
model). Extends the `native-ssi` capability from S1's closed-form dispatch and S2's
seeding.
