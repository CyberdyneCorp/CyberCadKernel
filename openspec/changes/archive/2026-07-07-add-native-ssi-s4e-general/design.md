# Design — add-native-ssi-s4e-general (SSI Stage S4-e, second slice: FREEFORM pole + curve-cusp assessment)

## Context

The archived `add-native-ssi-s4e-singularities` landed the FIRST S4-e slice for two ANALYTIC
removable chart singularities — a SPHERE parametric pole (`v = ±π/2`, `‖dU‖ = R·cos v → 0`) and a
CONE apex (signed radius `R₀ + v·sin α = 0`) — with:

- `chart_singularity.h`: `chartConditionAt` (the single-surface `‖dU‖`/`‖dV‖` collapse witness) +
  `poleContinuationU` (the `u_in + π` meridian jump).
- `marching.cpp`: `chartCondition` (the per-step witness on both surfaces), `crossChartSingularity`
  (the point-based fixed-plane crossing driver), `chartFarUV` (the far-side chart re-seed —
  pole meridian jump / apex `v` sign flip), `isPoleEdge` (pole-vs-apex discrimination), and
  `tryChartBand` (the `marchDir` hook that routes a chart collapse to the crossing instead of a
  spurious `BoundaryExit` / step-crawl).

The crossing is a POINT-BASED fixed-plane cut (`branchpt::reproject`) whose residuals use only the
surface POINT and the last-good tangent `t★` — NEVER the degenerate `dU`, NEVER the normal. That is
exactly why it is well-posed where the single-surface Jacobian drops rank.

**This change extends S4-e to the GENERAL singularities the archived change deferred**: a FREEFORM
parametric pole (§ Slice A — LANDS narrow) and a CURVE CUSP (§ Slice B — honest DECLINE).

## What is TRUE in the current code (verified, re-derived clean-room)

Confirmed by reading `branch_point.h`, `chart_singularity.h`, `marching.cpp`, `bspline.cpp`,
`seeding.cpp`, `vec.h` (OCCT is the verification ORACLE only):

1. **`branchpt::reproject` is normal-free.** Its residual is `{A.point − B.point, dot(A.point −
   anchor, t★) − d}`. It calls `A.point` / `B.point` and uses `t★` / `anchor` — it NEVER calls
   `A.normal` / `B.normal`. So the crossing corrector is well-posed at a freeform pole where ONLY
   the normal degenerates (the SAME reason it works at the cone apex).
2. **The chart witness fires for a freeform pole.** `chartConditionAt` computes `‖dU‖` / `‖dV‖` by
   finite differences of `S.point` (finite across a collapsed row) and checks `std::isfinite` on
   `S.normal(u,v).vec()`. At a collapsed U-row the freeform normal `normalize(Sᵤ × Sᵥ)` → a
   near-zero vector; `Dir3::normalizeFrom` (in `vec.h`) sets `v_ = d` (the raw near-zero vector) and
   `valid_ = false` when the input is below tolerance — a FINITE value, never NaN. So
   `normalFinite` is TRUE and `collapsed` fires: `‖dU‖ ≈ 0 < chartCollapseFrac·‖dV‖` and
   `< chartCollapseFrac·scale`. The freeform pole is DETECTED with NO change to the witness.
3. **Freeform pairs are marched.** `trace_intersection` / `trace_from_seeds` already run on
   `makeBSplineAdapter` / `makeBezierAdapter` adapters (the dome ∩ sheet and saddle ∩ plane
   fixtures in `test_native_ssi_marching.cpp`). A degenerate-pole B-spline adapter is a marchable
   `SurfaceAdapter`; `surfacePoint` on a collapsed-row grid is a convex combination — always finite.
4. **Freeform adapters carry `uPeriod = 0`.** `freeformAdapter` (`seeding.cpp`) sets `domain`,
   `point`, `normal`, `bound`, `modelScale` — it does NOT set `uPeriod` / `vPeriod`, and
   `SurfaceAdapter` defaults them to `0.0`. Elementary adapters set `uPeriod = 2π`. So the meridian
   jump `poleContinuationU` (which needs a period) does NOT apply to a freeform pole — this is the
   ONE gap the freeform-pole slice must close.

## Slice A — FREEFORM PARAMETRIC POLE (lands, narrow)

### The singularity

A B-spline / NURBS surface with a DEGENERATE control row: every pole in one U-row is the SAME 3D
point (a collapsed row / spline cone-tip). The whole U-line at that `v`-edge collapses to one 3D
point, so `‖∂S/∂u‖ → 0` there while `S.point` is a well-defined single point and `‖∂S/∂v‖` stays
`O(scale)`. **Topologically this is the SPHERE POLE, not the cone apex**: the collapse sits on a
`v` DOMAIN EDGE (the boundary row), the whole `u` circle maps to one point, and a curve through the
tip continues on the "opposite side" of the tip. `isPoleEdge` (collapse on a `v`-edge) already
classifies it as `poleCase = true`.

The DIFFERENCE from the sphere pole: the sphere's normal stays a true finite radial unit vector AT
the pole; the freeform normal degenerates to a near-zero `Dir3` AT the collapsed row (finite, but
not a unit normal). Since the crossing NEVER emits the exact pole and the corrector is normal-free,
this difference does not block the crossing — the nodes are all stepped just OFF the collapsed row,
where the freeform normal is a true finite unit normal.

### The one additive piece — a period-free far-side re-seed

`chartFarUV`'s pole branch today:

```
uOut = S.uPeriod > 0.0 ? poleContinuationU(u_in, S.uPeriod) : u_in;   // needs a period
vOut = v_in;                                                          // v unchanged at a pole edge
```

For a freeform pole `S.uPeriod == 0`, so `uOut = u_in` — the far-side seed stays on the SAME
meridian and the corrector re-lands on the near side (fails to cross) → defer. The fix: when
`uPeriod == 0`, recover the far LONGITUDE from the CONTINUED 3D TANGENT — the SAME idea the cone
apex already uses ("re-seed `(u,v)` from the continued 3D tangent"), generalized to the pole:

```
target = anchor + t★ · h            // the world point just past the collapsed row, along t★
uOut   = freeformChartInvert(S, target, vFix = v_in)   // far longitude at the SAME latitude
vOut   = v_in                                          // KEEP the latitude (as the analytic reflect)
```

`freeformChartInvert(S, target, vFix)` is a POINT-ONLY 1-D search for the far LONGITUDE: the `u` at
the fixed near-pole latitude `vFix` that MINIMIZES `‖S.point(u, vFix) − target‖` (a coarse `u`-scan
over the domain + a shrinking local refine, using `S.point` only — never `dU`, never the normal).

**Why FIXED-latitude, 1-D, not a full 2-D nearest-`(u,v)` solve** (the empirical refinement over the
initial sketch): a 2-D nearest-point search collapses onto the degenerate pole TIP itself — every
`u` at the pole edge maps to the SAME point, so the tip is a spurious flat minimum a 2-D descent
cannot leave. Holding `v = vFix` just INSIDE the pole edge (exactly as `poleContinuationU` keeps the
latitude and only jumps the longitude `u_in + π`) keeps the tiny parallel ring non-degenerate, so
the far vs near meridian is cleanly distinguished by `u`. This mirrors the analytic path one-for-one
— the freeform case only replaces the closed-form `u_in + π` with a numeric far-longitude search at
the same latitude. The corrector (`branchpt::reproject`) then CONFIRMS the far-side node; a wrong
pick simply fails the on-both-surfaces verification and the march defers. `freeformChartInvert` lives
in `chart_singularity.h` (OCCT-free, header-only, alongside `poleContinuationU`).

When `uPeriod > 0` (the analytic sphere pole) the existing `u_in + π` jump is UNCHANGED, so the
sphere-pole and cone-apex crossings stay BIT-IDENTICAL.

### The crossing (reused unchanged)

`crossChartSingularity` is reused verbatim: enter the band FINE (`chartStep`), point-based
fixed-plane cut (`branchpt::reproject` — no `dU`, no normal), verify every node on BOTH surfaces
`≤ onSurfTol`, resume the normal march once `‖dU‖` recovers on both surfaces. Only the far-side
re-seed (`chartFarUV` → `freeformChartInvert` when `uPeriod == 0`) differs.

### Honest guard (reused)

Emit the freeform-pole crossing ONLY if every node verifies on both surfaces `≤ onSurfTol` AND the
far side makes real progress off the tip. Otherwise DISCARD the arc + STOP + defer (truncated
`NearTangent` / `BoundaryExit` + typed reason → OCCT), counted in `nearTangentGaps`, reporting the
measured gap. An ASYMMETRIC freeform pole whose continued-tangent inversion does not verify is the
"must-still-defer" control. NEVER a fabricated freeform-pole-crossing point.

## Slice B — CURVE CUSP (honest DECLINE, no dead code)

### Why a curve cusp is NOT a reachable S4-e chart witness

The S4-e witness is a SINGLE-surface Jacobian rank-drop (`‖dU‖ → 0` with a finite normal). A CURVE
CUSP is a DIFFERENT quantity: the intersection CURVE's own velocity `→ 0`. The intersection curve's
tangent direction is `t = normalize(n_A × n_B)`. A geometric cusp (the curve's tangent reverses /
its velocity vanishes) requires `n_A × n_B → 0` — i.e. the surfaces are TANGENT there. **By the
implicit function theorem, if both charts are regular AND `n_A × n_B ≠ 0`, the intersection is a
SMOOTH REGULAR curve locally** — a cusp with "both charts regular AND healthy pair sine" is the
EMPTY SET. Therefore a curve cusp ALWAYS coincides with the PAIR-TANGENCY regime, which is:

- **S4-c** — a `NearTangentTransversal` single-branch graze (`‖n_A × n_B‖ → 0`, the curve
  continues): the existing S4-c fixed-plane crossing MARCHES THROUGH it (a cusp where the branch
  continues is exactly this).
- **S4-d** — a branch point (the locus self-crosses): the existing S4-d machinery localizes + routes
  the arms.
- **A genuine tangential endpoint / tacnode / isolated contact** (the curve truly ends): S4-c / S4-d
  already DEFER it honestly → OCCT.

There is NO reachable configuration for a STANDALONE single-surface-chart cusp witness distinct from
these. Adding a cusp detector + crossing to S4-e would be UNREACHABLE DEAD CODE — precisely what the
honesty discipline forbids.

### What this change does for curve cusps

NOTHING mechanistic — and that is the correct outcome. A curve cusp routes to the EXISTING
S4-c graze crossing (if the branch continues), the S4-d branch machinery (if the locus self-crosses),
or an honest deferral → OCCT (a genuine tangential endpoint the point-based step cannot continue).
The DECLINE is documented in `marching.h` / `chart_singularity.h` with this specific blocker (the
IFT argument). No cusp field, no cusp detector, no cusp corrector is added.

## Goals / Non-Goals

**Goals**
- (S4-e-g-1) Confirm the existing single-surface chart witness FIRES at a freeform degenerate pole
  (collapsed U-row): `‖dU‖ → 0`, `‖dV‖` finite, `normalFinite` TRUE (finite near-zero `Dir3`).
- (S4-e-g-2) Add a PERIOD-FREE far-side pole re-seed (`freeformChartInvert` from the continued 3D
  tangent) so a freeform pole (`uPeriod == 0`) can be crossed; keep the `uPeriod > 0` sphere-pole
  `u_in + π` jump bit-identical.
- (S4-e-g-3) Cross a degenerate-pole B-spline ∩ plane through the tip, reusing the normal-free
  point-based corrector, verified on both surfaces `≤ onSurfTol` vs OCCT.
- (S4-e-g-4) DECLINE the curve cusp with the specific IFT blocker; add NO dead code.
- (S4-e-g-5) Honest guard: verify-or-defer; never fabricate; never weaken a tolerance.

**Non-Goals (deferred — never faked here)**
- **Asymmetric freeform poles** whose continued-tangent inversion does not verify on both surfaces
  → DEFER → OCCT.
- **Curve cusps as a standalone S4-e mechanism** — they coincide with the pair-tangency S4-c/S4-d
  regime; no standalone cusp code is added.
- **Higher-order / edge / seam singularities**, ridge singularities, and **S4-f self-intersection
  completeness**.
- **A full brep degenerate-pole B-spline SOLID through the boolean pipeline** — the slice is at the
  marcher level (hand-seeded `trace_from_seeds`), the SAME envelope as the archived sphere-pole /
  cone-apex fixtures.
- **Any change to `src/native/tessellate`, the `cc_*` ABI, the analytic sphere-pole / cone-apex
  crossings, the S3 transversal trace, the S4-c graze crossing, or the S4-d branch machinery.**
- **Weakening `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, or any tolerance to "pass".**

## Module shape

```
src/native/ssi/chart_singularity.h  [extend — OCCT-free, header-only]
  namespace chartsing {
    // ... existing chartConditionAt / poleContinuationU unchanged ...
    // freeformChartInvert(S, target, vFix) → uOut
    //   a POINT-ONLY 1-D search for the far LONGITUDE: the u at the FIXED near-pole latitude vFix
    //   minimizing ‖S.point(u, vFix) − target‖ (coarse u-scan + shrinking refine). Fixed-latitude
    //   (like poleContinuationU keeps v and jumps u) so it never collapses onto the degenerate tip.
    //   Re-seeds the FAR side of a freeform pole from the continued 3D tangent when the surface
    //   carries NO analytic uPeriod (every freeform adapter). Uses S.point only — no dU, no normal.
  }

src/native/ssi/marching.cpp         [extend — the period-free pole re-seed, CYBERCAD_HAS_NUMSCI]
  // chartFarUV(S, poleCase, u, v, target, &uOut, &vOut):   // target = anchor + t★·h (continued)
  //   pole branch: if S.uPeriod > 0  → uOut = poleContinuationU(u, uPeriod)         (UNCHANGED)
  //                else (freeform)    → uOut = freeformChartInvert(S, target, vFix=v)
  //                vOut = v   (KEEP the latitude — analytic reflect and freeform share this)
  //   apex branch: uOut = u, vOut = −v   (UNCHANGED)
  // crossChartSingularity / chartCondition / isPoleEdge / tryChartBand: UNCHANGED (reused).
  // (No cusp code — the curve cusp is a documented DECLINE, routed to S4-c/S4-d/OCCT.)
```

`src/native/**` stays OCCT-free. The only new code is the header-only `freeformChartInvert` and the
`chartFarUV` pole-branch generalization — both point-based, neither weakening a tolerance. No new
result field (freeform-pole crossings reuse `singularitiesCrossed` / `chartSingularCrossed`).

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_s4e_singularities.cpp`, under
  `CYBERCAD_HAS_NUMSCI`:
  - **Freeform pole now crossed.** A bicubic B-spline whose top U-row is COLLAPSED to a single 3D
    tip point (a spline cone-tip) ∩ a plane through the tip, forced through marching
    (`trace_from_seeds` with a hand seed on the intersection away from the tip). OFF: the march
    truncates at the tip (a spurious `BoundaryExit` / one-sided arc that never reaches the far side
    of the tip). ON: it CROSSES the freeform pole — the curve continues onto the far side,
    `singularitiesCrossed ≥ 1`, `nearTangentGaps == 0`, every node on BOTH surfaces `≤ onSurfTol`
    (`surfacePoint` on the collapsed-row grid; `planeValue`).
  - **Asymmetric freeform pole still defers.** A freeform pole whose continued-tangent re-seed does
    NOT verify on both surfaces STILL truncates + defers (`singularitiesCrossed == 0`,
    `nearTangentGaps ≥ 1`) — the "must-still-defer" control (no fabricated crossing).
  - **Analytic crossings unchanged.** The sphere-pole great circle still fully traces
    (`singularitiesCrossed ≥ 2`, `status == Closed`); the cone-apex line still crosses
    (`singularitiesCrossed ≥ 1`); the genuine-boundary cylinder cap still exits `BoundaryExit`
    (`singularitiesCrossed == 0`).
  - **Regression.** The 5 transversal S3 pairs trace `nt == 0`; the S4-c graze still crosses; the
    S4-d Steinmetz still traces (`branchPoints == 2`).
  - Full CTest green NUMSCI ON and OFF (S4-e-g assertions absent with NUMSCI off). No OCCT; no
    tolerance weakened.
- **Sim native-vs-OCCT (booted simulator).** Extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`: build the degenerate-pole B-spline as an OCCT
  `Geom_BSplineSurface` with the collapsed row + a `Geom_Plane`; assert the native freeform-pole
  crossing matches OCCT `GeomAPI_IntSS` / `IntPatch` — every sampled native node on the OCCT locus
  `≤ onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces `≤ onSurfTol`; the native
  curve passes through the OCCT tip point to `tol`; the far side of the tip is reached in both. Also
  assert the analytic sphere-pole / cone-apex, the genuine-boundary, and the transversal + S4-c +
  S4-d cases are UNCHANGED. Report per-pair crossed vs still-deferred. Run via
  `xcrun simctl spawn <booted udid>` (`xcrun simctl list devices booted`).

## Decisions

- **Reuse the normal-free point-based corrector — the freeform pole needs no new corrector.**
  `branchpt::reproject` touches only `A.point` / `B.point` / `t★`, so it is well-posed at a freeform
  pole where only the normal degenerates. The freeform-pole slice adds NO corrector; it reuses the
  S4-c fixed-plane cut, exactly as the archived analytic slice does.
- **The ONE additive piece is a period-free far-side re-seed.** Because freeform adapters carry no
  `uPeriod`, the analytic `u_in + π` meridian jump does not apply. Re-seed the far side from the
  CONTINUED 3D TANGENT via a clamped nearest-`(u,v)` inversion — the SAME apex-style re-seed,
  generalized to a pole. Gated on `uPeriod == 0` so the analytic crossings stay bit-identical.
- **The curve cusp is DECLINED by construction (no dead code).** A cusp with regular charts and
  healthy pair sine is impossible (IFT); a curve cusp always coincides with the pair-tangency
  S4-c/S4-d regime. So NO standalone cusp witness / corrector is added — it would be unreachable.
  Curve cusps route to S4-c (graze), S4-d (branch), or an honest deferral → OCCT.
- **Emit ONLY verified crossings; otherwise DISCARD + DEFER.** A freeform pole whose continued-
  tangent re-seed does not verify on both surfaces is rolled back and the march defers — never a
  fabricated pole point. The asymmetric-pole control pins this.
- **Marcher-level fixtures (hand-seeded), the SAME envelope as the archived slice.** No new native
  degenerate-pole brep SOLID is required; the freeform-pole adapter + `trace_from_seeds` mirrors the
  sphere-pole / cone-apex fixtures. A full brep degenerate-pole solid is OUT OF SCOPE.
- **Additive, ABI-stable, tessellator-untouched.** Only `freeformChartInvert` + the `chartFarUV`
  pole-branch generalization; no new result field, no `cc_*` change; `src/native/tessellate`
  untouched; under `CYBERCAD_HAS_NUMSCI` like the rest of S4.

## Risks / Trade-offs

- **Wrong far-side inversion on an asymmetric freeform pole.** Mitigation: VERIFY the first far-side
  node on both surfaces `≤ onSurfTol` before accepting; a mis-inverted node does not verify →
  defer. The asymmetric-pole control is the "must-still-defer" pin. Accepted.
- **No native degenerate-pole brep solid flows through the boolean pipeline yet.** Mitigation: the
  slice is scoped at the marcher level (hand-seeded), the SAME as the archived sphere-pole /
  cone-apex fixtures; a full brep solid is explicitly OUT OF SCOPE. If NO freeform-pole fixture
  verifies even at the marcher level, the honest outcome is DECLINE with the measured gap — no dead
  code shipped. Accepted.
- **Curve cusp mistaken for a crossable S4-e singularity.** Mitigation: no cusp code is added; a
  cusp routes to S4-c/S4-d (pair tangency) or defers → OCCT, by the IFT argument. Accepted.
- **Perturbing the analytic sphere-pole / cone-apex crossings.** Mitigation: the period-free re-seed
  is gated on `uPeriod == 0` (freeform only); `uPeriod > 0` keeps the exact `u_in + π` jump, so the
  analytic crossings and the transversal / S4-c / S4-d traces are bit-identical, pinned green by
  their existing assertions. Accepted. Whatever does not resolve robustly still stops + defers →
  OCCT and is reported with the measured gap; no point is faked, hand-tuned, or weakened to pass.
