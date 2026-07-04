## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness. S4-a/S4-b landed
the CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`) but explicitly
**did not march through a tangency**. S4-c is the HARD CORE of that moat: when the
intersection curve GRAZES a near-tangent region **but genuinely continues on the same
branch**, the tracer must MARCH THROUGH it and emit the FULL curve, verified vs OCCT —
instead of stopping short.

Today the S3 marcher STOPS the moment the intersection tangent ill-conditions:

- `marching.cpp intersectionTangent` computes `t = normalize(nₐ × n_b)`; the transversality
  witness `sine = ‖nₐ × n_b‖` drops toward zero in a near-tangent region. `marchDir` flags
  `DirEnd::NearTangent` and STOPS the walk the moment `sine < tangentSinTol` (default 1e-3)
  OR the corrector cannot take even a `minStep`.
- The ROOT CAUSE is the corrector's along-tangent advance constraint
  `r₃ = dot(A.point − Pprev, t) − h` (`marching.cpp correct`). As `sine → 0` the tangent
  `t` becomes ill-defined, so this constraint degenerates and the 2×2/4×4 Newton
  diverges — the marcher cannot advance and honestly truncates.

Diagnosed on the current marcher (Diagnose phase, host build, `CYBERCAD_HAS_NUMSCI` ON):
two EQUAL crossing cylinders (R=1, axes crossing at 90°, i.e. the Steinmetz configuration)
trace only **167 of the curve's points** and stop with
`TraceStatus::NearTangent`, `stopReason = NearTangentTransversal` (`crossingSine ≈ 4.5e-4`),
`nearTangentGaps = 1` — the walk halts at the saddle `(0, ±1, 0)` where the two cylinder
normals go antiparallel, leaving the Steinmetz curve incomplete even though it genuinely
continues through the grazing region. The same truncation reproduces across tilted-axis
crossing cylinders (`NearTangentTransversal`, `nearTangentGaps = 1`, sine ≈ 1e-4…1e-3).
Meanwhile a GENUINE tangency — two spheres at `d = R₁+R₂`, and a sphere tangent to a
cylinder — is correctly deferred (`deferredTangent = 1`, no curve fabricated), which S4-c
MUST preserve.

This change adds the FIRST HONEST S4-c SLICE: march THROUGH a near-tangent region **only
when the S4-b classification says it is crossable** (`NearTangentTransversal` **and** a
single-branch graze), producing the full curve; a GENUINE tangency (`TangentPoint` /
`TangentCurve`), a branch crossing, or anything not robustly crossable STILL STOPS +
classifies + defers → OCCT. It NEVER fabricates a point past a degeneracy and NEVER
weakens a tolerance.

## What Changes

- **(S4-c-1) ROBUST CORRECTOR near tangency — a fixed-plane cut.** Replace the
  along-tangent advance constraint (which needs a well-defined `t`, degenerating as
  `sine → 0`) with a constraint that stays well-defined THROUGH the low-sine band: pin the
  new node to the **hyperplane perpendicular to the LAST-GOOD tangent** `t★` (the last
  well-conditioned intersection tangent) at arc-distance `h` from the previous node —
  `r₃ = dot(A.point − Pprev, t★) − h`. The curve crosses this fixed cut plane
  TRANSVERSALLY even where the LOCAL surface tangent degenerates, so the residual is
  well-posed across the grazing region. Solve as a constrained residual-minimization over
  the native-numerics substrate (`nn::least_squares` on `‖A.point − B.point‖²` subject to
  the plane cut, or `nn::minimize` of `‖A.point − B.point‖` with the cut as a penalty/
  constraint), NOT the 2×2 tangent-plane Newton that diverges. Outside the low-sine band
  the existing along-`t` corrector is unchanged (bit-identical trace on all currently
  passing S3 fixtures).
- **(S4-c-2) CURVATURE-AWARE PREDICTOR.** Use the change in tangent over the last two
  accepted nodes (a discrete curvature estimate) to BEND the first-order `P + h·t★` guess
  toward the curve, so the corrector starts inside its basin of convergence across the
  grazing region. Falls back to the first-order predictor when fewer than two prior nodes
  exist or the curvature estimate is ill-conditioned.
- **(S4-c-3) STEP CONTROL through the low-sine band.** As `sine` drops below a
  `bandEnterSin` threshold, SHRINK `h` (bounded below by `minStep`) to cross the
  minimum-clearance region in small steps; once `sine` recovers above a `bandExitSin`
  threshold, resume normal deflection-driven step growth. If `h` hits `minStep` and the
  fixed-plane corrector still cannot converge, STOP honestly (an S4-c gap, deferred).
- **(S4-c-4) CROSSABLE GATE (the honesty core).** Attempt to cross ONLY when the S4-b
  classifier at the stall returns `NearTangentTransversal` (the relative second
  fundamental form is INDEFINITE — the surfaces graze but CROSS) **and** the local
  configuration is a SINGLE-BRANCH graze (the curve continues on the same branch, not a
  branch crossing). `TangentPoint` / `TangentCurve` (a genuine measure-zero/grazing-contact
  tangency where the curve does NOT continue), a detected branch crossing (multiple curve
  branches meeting — S4-d), an `Undecided` jet, or a corrector that will not converge at
  `minStep` ⇒ STOP + classify + defer. A successful crossing is VERIFIED (the emitted
  points lie on BOTH surfaces ≤ `onSurfTol` and advance monotonically past the band); a
  crossing that cannot be verified is discarded and the march truncates honestly.
- **Marching result carries the crossing outcome (additive).** `marching.h` gains, per
  `WLine`, a count of near-tangent regions CROSSED and the residual through the band; a
  crossed region no longer increments `TraceSet.nearTangentGaps` (it is now a completed
  arc), while an uncrossable region still does. No `cc_*` ABI change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the FIRST S4-c
near-tangent MARCHING-THROUGH slice (robust fixed-plane corrector + curvature-aware
predictor + low-sine step control + crossable gate). It adds no new capability spec and no
cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: add (S4-c) a native, OCCT-free **near-tangent marching-through** capability
  that CROSSES a near-tangent region and emits the FULL intersection curve **only when the
  S4-b classification is `NearTangentTransversal` and the region is a single-branch graze
  the curve genuinely continues through**, via a robust FIXED-PLANE-CUT constrained
  corrector (well-posed as `sine → 0`), a curvature-aware predictor, and low-sine step
  control. A GENUINE tangency (`TangentPoint` / `TangentCurve`), a branch crossing (S4-d),
  an `Undecided` jet, or a non-convergent corrector at `minStep` STILL STOPS + classifies +
  defers → OCCT. The tracer NEVER fabricates a point past a degeneracy and NEVER weakens a
  tolerance; a curve that still truncates is an honestly reported gap. No `cc_*` ABI change;
  `src/native/**` stays OCCT-free; the new marching parts are compiled under
  `CYBERCAD_HAS_NUMSCI` like S2/S3.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change.
  Additive only; the tessellator (`src/native/tessellate`) and the CyberCad app are
  untouched.
- **Build**: extends `src/native/ssi/marching.{h,cpp}` (the fixed-plane corrector, the
  curvature-aware predictor, the low-sine step controller, the crossable gate, and the
  additive crossing outcome on `WLine` / `TraceSet`) under `CYBERCAD_HAS_NUMSCI` like the
  S3 marcher; consumes the existing S4-b `tangent_seeded.h` classifier (`classify_tangent_
  contact_seeded`) unchanged for the crossable gate. No change to `src/native/tessellate`;
  no new substrate routine (reuses `nn::least_squares` / `nn::minimize`).
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4c_near_tangent.cpp`): the equal crossing-cylinder
  Steinmetz fixture that S3 currently truncates (`nearTangentGaps = 1`) is now FULLY traced
  (`nearTangentGaps = 0`, the two ellipse branches complete), every emitted node on BOTH
  surfaces ≤ `onSurfTol`; the genuine-tangency pairs (spheres at `d = R₁+R₂`; sphere tangent
  to a cylinder) STILL stop + classify (`TangentPoint` / `TangentCurve`, deferred, no curve
  fabricated across the tangency); a detected branch crossing STILL stops (deferred, not
  crossed). Full CTest green NUMSCI ON and OFF (the S4-c assertions absent with NUMSCI off,
  like S2/S3). No OCCT linked; no tolerance weakened. **Sim native-vs-OCCT** — extend
  `scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm`:
  assert the currently-truncating near-tangent curve is now FULLY traced
  (`nearTangentGaps → 0`) with the full curve matching OCCT `IntPatch` / `GeomAPI_IntSS`
  (every native WLine point on the OCCT curve ≤ `onCurveTol`; native arc length ≈ OCCT arc
  length within the deflection/step tolerance; closed/branch counts agree); AND a
  genuine-tangency pair STILL stops (reported, never crossed). `xcrun simctl list devices
  booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4 from CLASSIFICATION (S4-a/S4-b) to the FIRST
  S4-c near-tangent MARCHING slice — a truncating grazing-but-continuous curve now fully
  traced vs OCCT. **Explicitly a first slice:** branch points (S4-d), singularities (S4-e),
  and self-intersection (S4-f) remain OUT OF SCOPE, and any near-tangent region not
  robustly crossable still truncates + defers → OCCT.
- **Risk (honest)**: (a) the fixed-plane-cut corrector can still fail to converge in a
  DEEP low-sine band (the surfaces are nearly coincident over a stretch, not just at a
  point) — mitigated by the `minStep` floor + `Undecided`/defer fallback (report, don't
  fabricate) and the sim on-surface/arc-length parity gate; (b) the crossable gate can
  MISREAD a branch crossing (S4-d) as a single-branch graze — mitigated by the branch-
  crossing detection (multiple valid tangent directions / multiple nearby seeds at the
  stall) forcing a defer, and by the crossing being VERIFIED on both surfaces before it is
  accepted; (c) the low-sine step control must not regress the transversal S3 trace —
  mitigated by leaving the along-`t` corrector and deflection controller bit-identical
  OUTSIDE the low-sine band, pinned by the existing S3 host + sim fixtures staying green.
  Whatever does not cross robustly still truncates + defers → OCCT and is reported with the
  measured gap; no case is faked, hand-tuned, or weakened to pass.
