## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness. The archived
`add-native-ssi-s4e-singularities` landed the FIRST S4-e slice: it DETECTS a single-surface
CHART singularity (`‖dU‖` collapse vs `‖dV‖·scale` with a finite normal — the single-surface
Jacobian rank-drop, DISTINCT from the S4-c pair sine and the S4-d locus flip) and STEPS across it
with a POINT-BASED fixed-plane cut (`branchpt::reproject` — residuals use only the surface POINT
and the last-good tangent `t★`, NEVER the degenerate `dU`), for two **ANALYTIC** removable
singularities: a SPHERE parametric pole (`v = ±π/2`, `‖dU‖ = R·cos v → 0`) and a CONE apex
(signed radius `R₀ + v·sin α = 0`). Those crossings are green (`singularitiesCrossed ≥ 2` for the
great circle, `≥ 1` for the apex line).

**This change extends S4-e beyond the analytic cases**, honestly, to the two GENERAL singularities
the archived change explicitly deferred:

- **(a) A FREEFORM PARAMETRIC POLE** — a B-spline / NURBS surface with a DEGENERATE control row
  (a collapsed U-row / a spline cone-tip): the whole U-line collapses to one 3D point at a v-edge,
  so `‖∂S/∂u‖ → 0` while the 3D point stays finite — the FREEFORM ANALOG of the sphere pole.
- **(b) A CURVE CUSP** — where the intersection CURVE's own velocity `→ 0` (the marcher step
  degenerates) even though BOTH surfaces are regular and their charts are fine.

**Diagnosis on the CURRENT stack (host, `CYBERCAD_HAS_NUMSCI` ON), re-derived clean-room; OCCT
is the ORACLE only:**

1. **The point-based corrector is already normal-free.** `branchpt::reproject`
   (`src/native/ssi/branch_point.h`) solves `r₀..₂ = A.point − B.point`, `r₃ = dot(A.point −
   anchor, t★) − d` — it touches `A.point`/`B.point` and `t★` ONLY, never a surface normal. So it
   is well-posed at a freeform pole where ONLY the normal degenerates, exactly as at the cone
   apex.
2. **The chart witness already FIRES for a freeform pole.** `chartConditionAt`
   (`chart_singularity.h`) reads `‖dU‖` / `‖dV‖` by finite differences of `S.point` (finite across
   a collapsed row) and checks `std::isfinite` on `S.normal`. At a collapsed row the freeform
   normal `normalize(Sᵤ × Sᵥ)` degenerates to a near-zero vector, but `Dir3::normalizeFrom`
   returns that raw near-zero vector FINITE (`valid_ = false`, never NaN — `vec.h`), so
   `normalFinite` is TRUE and `collapsed` fires. The freeform pole is DETECTED by the existing
   witness with no change.
3. **Freeform pairs are already marched.** `trace_intersection` / `trace_from_seeds` run on
   `makeBSplineAdapter` / `makeBezierAdapter` adapters today (the B-spline dome ∩ sheet and the
   B-spline saddle ∩ plane fixtures in `test_native_ssi_marching.cpp`). A degenerate-pole B-spline
   adapter (a collapsed top U-row) ∩ a plane through the tip, hand-seeded through `trace_from_seeds`
   exactly like the sphere-pole / cone-apex fixtures, is a REACHABLE marched pair.
4. **The ONE real gap for a freeform pole.** `chartFarUV`'s pole branch re-seeds the far side as
   `uOut = S.uPeriod > 0 ? poleContinuationU(u_in) : u` — the meridian jump `u_in + π` needs a
   `uPeriod`. Elementary adapters carry `uPeriod = 2π`; **freeform adapters carry `uPeriod = 0`**
   (`freeformAdapter` never sets it; `SurfaceAdapter` defaults to 0). So for a freeform pole the
   far-side longitude is NOT jumped and the corrector re-seeds on the SAME side — it fails to
   cross and (honestly) defers. The freeform-pole crossing needs a PERIOD-FREE far-side re-seed:
   derive the far-side `(u,v)` from the CONTINUED 3D TANGENT (project `anchor + t★·h` back onto the
   freeform chart by a nearest-`(u,v)` inversion seeded from the pre-pole node) — the SAME
   "re-seed from the continued 3D tangent" the cone apex already uses, generalized to a pole with
   no analytic period. Everything else (detector, point-based cut, honest verify-or-defer) is
   reused UNCHANGED.
5. **The curve cusp is NOT an independent S4-e witness (honest DECLINE).** A genuine cusp of an
   intersection curve — curve velocity `→ 0` — can occur ONLY where the pair transversality sine
   `‖n_A × n_B‖ → 0`: by the implicit function theorem, if the charts are regular AND
   `‖n_A × n_B‖ ≠ 0` the intersection is a SMOOTH REGULAR curve locally, so a cusp with "both
   charts regular AND healthy pair sine" is the EMPTY SET. A curve cusp therefore coincides with
   the PAIR-TANGENCY regime already owned by the S4-c graze crossing (if the branch continues
   through) and the S4-d branch machinery (if the locus self-crosses), or it is a genuine
   tangential endpoint / tacnode that S4-c/S4-d already DEFER honestly → OCCT. There is NO reachable
   configuration for a separate single-surface-chart cusp witness — building one would be
   UNREACHABLE DEAD CODE. This change therefore ADDS NO cusp mechanism; it DECLINES curve cusps
   with that specific blocker and routes them to the existing S4-c/S4-d/OCCT path.

This change adds the SECOND, NARROW S4-e slice: the FREEFORM parametric-pole crossing (reusing the
normal-free point-based corrector + a period-free far-side re-seed), with the CURVE CUSP an honest
DECLINE (no dead code). Anything not robustly steppable + verified across the singularity STILL
DEFERS (honest truncated stop + typed reason → OCCT). It NEVER fabricates a point across a
singularity and NEVER weakens a tolerance.

## What Changes

- **(S4-e-g-1) FREEFORM-POLE DETECTION (reuse, confirm).** Confirm the existing single-surface
  chart witness (`chartConditionAt` / `chartCondition`) FIRES at a freeform degenerate pole (a
  collapsed U-row): `‖dU‖ → 0` from the finite-difference of `S.point` across the collapsed row,
  `‖dV‖` finite, and `normalFinite` TRUE (the degenerate freeform normal is a FINITE near-zero
  `Dir3`, not NaN). No new witness; the freeform pole is caught by the SAME single-surface
  Jacobian rank-drop as the sphere pole. It STAYS DISTINCT from the S4-c pair sine and the S4-d
  locus flip.
- **(S4-e-g-2) PERIOD-FREE FAR-SIDE POLE RE-SEED (the one additive piece).** Generalize
  `chartFarUV`'s pole branch so a pole with NO analytic `uPeriod` (every freeform adapter) re-seeds
  the far-side `(u,v)` from the CONTINUED 3D TANGENT rather than an `u_in + π` meridian jump: take
  the continued world target `anchor + t★·h` on the far side of the collapsed row and invert it to
  the freeform chart by a nearest-`(u,v)` projection seeded from the pre-pole node (a short Newton /
  least-squares on `‖S(u,v) − target‖`, clamped to the domain). This is the SAME apex-style
  "re-seed from the continued tangent" already in the code, extended to the pole case when
  `uPeriod == 0`. When `uPeriod > 0` (the analytic sphere pole) the existing `u_in + π` jump is
  UNCHANGED — the sphere-pole / cone-apex crossings stay bit-identical.
- **(S4-e-g-3) POINT-BASED CROSSING (reuse UNCHANGED).** The crossing itself is the existing
  `crossChartSingularity` driver: enter the band FINE (`chartStep`), point-based fixed-plane cut
  (`branchpt::reproject` — no `dU`, no normal), verify every node on BOTH surfaces `≤ onSurfTol`,
  resume the normal `(u,v)` march once `‖dU‖` recovers on both surfaces. Only the far-side
  re-seed (S4-e-g-2) differs for a freeform pole. Reuses the S4-c corrector; adds no new corrector.
- **(S4-e-g-4) CURVE CUSP — HONEST DECLINE (no dead code).** Do NOT add a curve-cusp mechanism.
  A curve cusp coincides with the pair-tangency regime (see Why §5), so it is handled by the
  existing S4-c graze crossing (if the branch continues), the S4-d branch machinery (if the locus
  self-crosses), or an honest deferral → OCCT (a genuine tangential endpoint / tacnode the
  point-based step cannot continue). This change documents the blocker and adds NO unreachable
  code for a standalone cusp witness.
- **(S4-e-g-5) HONEST GUARD (reuse).** A freeform-pole crossing is emitted ONLY if every node
  verifies on BOTH surfaces `≤ onSurfTol` AND the far side makes real progress off the pole. If the
  period-free re-seed cannot land a verified far-side node (an asymmetric freeform pole where the
  continued-tangent inversion misses; a pole the point-based cut cannot resolve), the arc is
  DISCARDED and the march STILL STOPS + defers (truncated `NearTangent` / `BoundaryExit` + typed
  reason → OCCT), counted in `nearTangentGaps`, reporting the MEASURED gap. NEVER a fabricated
  freeform-pole-crossing point; NEVER a weakened tolerance.
- **Result reuse (no new field).** The freeform-pole crossings reuse the existing
  `TraceSet.singularitiesCrossed` / `WLine.chartSingularCrossed` counters (a freeform pole is a
  chart singularity crossed, same as a sphere pole). No new result field; no `cc_*` ABI change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the SECOND S4-e
chart-singularity slice (the FREEFORM parametric-pole crossing, reusing the normal-free
point-based corrector + a period-free far-side re-seed) + an honest curve-cusp DECLINE. It adds
no new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: extend the S4-e chart-singularity capability to a FREEFORM parametric pole — a
  B-spline / NURBS surface with a DEGENERATE control row (a collapsed U-row / spline cone-tip)
  where `‖∂S/∂u‖ → 0` while the 3D point stays finite. The marcher DETECTS the freeform pole via
  the SAME single-surface Jacobian rank-drop (`‖dU‖` collapse vs `‖dV‖·scale`, finite normal — the
  degenerate freeform normal is a finite near-zero `Dir3`), STEPS across the singular band with the
  EXISTING point-based fixed-plane corrector (which uses only the finite POINT + the last-good
  tangent, never the degenerate `dU` or the normal), re-seeding the far-side `(u,v)` from the
  CONTINUED 3D TANGENT (because freeform adapters carry no analytic `uPeriod`, so the analytic
  `u_in + π` meridian jump does not apply), and RESUMES the normal `(u,v)` march on the far side —
  reporting the crossing in `singularitiesCrossed`. A freeform pole that cannot be robustly
  verified on both surfaces STILL STOPS + defers → OCCT (never a fabricated pole-crossing point).
  A CURVE CUSP (intersection-curve velocity `→ 0`) is EXPLICITLY DECLINED: it coincides with the
  pair-tangency regime already owned by S4-c / S4-d (a cusp with regular charts and healthy pair
  sine is impossible by the implicit function theorem), so it routes to the existing
  S4-c/S4-d/OCCT path and NO standalone cusp mechanism is added (no unreachable dead code). The
  analytic sphere-pole / cone-apex crossings, the transversal S3 trace, the S4-c crossable graze,
  and the S4-d branch trace are UNTOUCHED. The tracer NEVER fabricates a point across a singularity
  and NEVER weakens a tolerance. No `cc_*` ABI change; `src/native/**` stays OCCT-free; the S4-e-g
  machinery is compiled under `CYBERCAD_HAS_NUMSCI` like the rest of S2/S3/S4.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change.
  Additive only; the tessellator (`src/native/tessellate`) and the CyberCad app are untouched.
- **Build**: extends `src/native/ssi/marching.cpp` — generalize `chartFarUV`'s pole branch (the
  period-free far-side re-seed from the continued 3D tangent when `uPeriod == 0`) and, if needed, a
  small `freeformChartInvert(S, target, seedU, seedV)` helper (a clamped nearest-`(u,v)` Newton on
  `‖S(u,v) − target‖`) added to `chart_singularity.h` (OCCT-free, header-only, like
  `poleContinuationU`). No change to the detector (`chartConditionAt` already fires), the crossing
  driver (`crossChartSingularity` reused), the S4-c corrector (`branchpt::reproject` reused), or the
  result structs (`singularitiesCrossed` / `chartSingularCrossed` reused). Under
  `CYBERCAD_HAS_NUMSCI` like the S3/S4-c/S4-d/S4-e marcher. No new tolerance beyond the existing
  chart discriminators (`chartCollapseFrac`, `chartStep`); NO weakening of `onSurfTol` /
  `tangentSinTol` / `minStep` / `maxDeflection`.
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_s4e_singularities.cpp`, under `CYBERCAD_HAS_NUMSCI`: a
  DEGENERATE-POLE B-spline (a collapsed top U-row = a spline cone-tip) ∩ a plane through the tip,
  forced through marching (`trace_from_seeds` with a hand seed away from the tip). OFF: the march
  truncates at the pole (a spurious `BoundaryExit` / one-sided arc). ON: it CROSSES the freeform
  pole — the curve continues onto the far side of the tip, `singularitiesCrossed ≥ 1`,
  `nearTangentGaps == 0`, every node on BOTH surfaces `≤ onSurfTol` (via `surfacePoint` /
  `planeValue`), the far side of the tip is reached. A control: an ASYMMETRIC freeform pole whose
  continued-tangent re-seed does NOT verify STILL defers (`singularitiesCrossed == 0`,
  `nearTangentGaps ≥ 1`, truncated) — honest. Regression: the analytic sphere-pole great circle
  still fully traces (`singularitiesCrossed ≥ 2`), the cone-apex line still crosses
  (`singularitiesCrossed ≥ 1`), the genuine-boundary cylinder cap still exits `BoundaryExit`
  (`singularitiesCrossed == 0`), the 5 transversal pairs trace `nt == 0`, the S4-c graze still
  crosses, the S4-d Steinmetz still traces (`branchPoints == 2`). Full CTest green NUMSCI ON and
  OFF (S4-e-g assertions absent with NUMSCI off). No OCCT; no tolerance weakened.
  **Sim native-vs-OCCT (booted sim)** — extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`: the same degenerate-pole B-spline ∩ plane, built as an
  OCCT `Geom_BSplineSurface` with the collapsed row and a `Geom_Plane`; assert the native
  freeform-pole crossing matches OCCT `GeomAPI_IntSS` / `IntPatch` — every sampled native node on
  the OCCT locus `≤ onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces `≤ onSurfTol`;
  the native curve passes through the OCCT tip point to `tol`; the far side of the tip is reached in
  both. Report per-pair crossed vs still-deferred. `xcrun simctl list devices booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4-e from the ANALYTIC sphere-pole / cone-apex slice to the
  FREEFORM parametric-pole slice. **Explicitly a narrow slice:** the freeform pole reachable by the
  continued-tangent re-seed lands; asymmetric freeform poles that do not verify, CURVE CUSPS (which
  coincide with the pair-tangency S4-c/S4-d regime), higher-order / edge singularities, seam
  singularities, and S4-f self-intersection completeness remain OUT OF SCOPE and DEFER → OCCT.
- **Risk (honest)**: (a) the continued-tangent re-seed could invert to the WRONG far-side `(u,v)`
  on an asymmetric freeform pole — mitigated by VERIFYING the first far-side node on both surfaces
  `≤ onSurfTol` before accepting; a mis-inverted node does not verify and the crossing defers
  (never faked). (b) The freeform pole could be reached ONLY through a hand-seeded test adapter
  (no native degenerate-pole B-spline SOLID flows through the boolean pipeline yet) — that is the
  SAME reachability envelope as the archived sphere-pole / cone-apex fixtures (all hand-seeded
  through `trace_from_seeds`), so the marcher-level fixture is the established, honest scope; a full
  brep degenerate-pole solid is OUT OF SCOPE. If NO freeform-pole fixture verifies even at the
  marcher level, the honest outcome is DECLINE with the measured gap — no dead code is shipped.
  (c) The curve cusp could be mistaken for a crossable S4-e singularity — mitigated by construction:
  no cusp witness is added; a curve cusp routes to S4-c/S4-d (pair tangency) or defers → OCCT.
  (d) The freeform-pole path must not perturb the analytic sphere-pole / cone-apex crossings — the
  period-free re-seed is gated on `uPeriod == 0` (freeform only); `uPeriod > 0` keeps the exact
  `u_in + π` jump, so the analytic crossings stay bit-identical. Whatever does not resolve robustly
  still stops + defers → OCCT and is reported with the measured gap; no point is faked, hand-tuned,
  or weakened to pass.
