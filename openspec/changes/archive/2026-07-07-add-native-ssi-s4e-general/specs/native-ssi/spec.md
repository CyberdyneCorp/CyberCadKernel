# native-ssi

Extend the SSI Stage **S4-e CHART-SINGULARITY** capability (`openspec/SSI-ROADMAP.md` S4 — the
moat, degeneracy robustness) from the ANALYTIC removable singularities (sphere parametric pole,
cone apex) that the archived `add-native-ssi-s4e-singularities` landed, to the GENERAL cases:

- **S4-e (freeform pole)** — a native, OCCT-free extension that crosses a FREEFORM parametric pole:
  a B-spline / NURBS surface with a DEGENERATE control row (a collapsed U-row / spline cone-tip)
  where `‖∂S/∂u‖ → 0` while the 3D point stays finite — the FREEFORM ANALOG of the sphere pole. The
  marcher DETECTS it with the SAME single-surface Jacobian rank-drop witness (`‖dU‖` collapse vs
  `‖dV‖·scale`, finite normal), STEPS across it with the EXISTING point-based fixed-plane corrector
  (which uses only the finite POINT + the last-good tangent, never the degenerate `dU` or the
  normal), re-seeding the far-side `(u,v)` from the CONTINUED 3D TANGENT (because freeform adapters
  carry no analytic `uPeriod`, so the `u_in + π` meridian jump does not apply), and RESUMES the
  normal march on the far side.

This change is **additive** to the S3/S4-c/S4-d/S4-e marcher. The analytic sphere-pole / cone-apex
crossings, the transversal S3 trace, the S4-c crossable-graze crossing, and the S4-d branch-point
trace are UNCHANGED — the freeform-pole path is gated on a freeform pole (`uPeriod == 0`) and reuses
the existing detector, corrector, and result counters. A freeform pole that cannot be robustly
CROSSED (the continued-tangent-re-seeded far node does not re-project on both surfaces within
tolerance; the crossing makes no far-side progress) STILL STOPS + defers → OCCT. A CURVE CUSP (the
intersection curve's own velocity `→ 0`) is EXPLICITLY DECLINED and routed to the existing
S4-c/S4-d/OCCT path: a cusp with regular charts and a healthy pair transversality sine is impossible
by the implicit function theorem, so a curve cusp coincides with the pair-tangency regime and no
standalone single-surface-chart cusp mechanism is added (no unreachable dead code). The tracer NEVER
fabricates a point across a singularity and NEVER weakens a tolerance; a singularity it cannot cross
is an honestly reported gap. SSI is INTERNAL: **no `cc_*` ABI change**; `src/native/**` stays
OCCT-free; the S4-e-g parts are compiled under `CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e).
Asymmetric freeform poles that do not verify, higher-order / edge / seam singularities, a full brep
degenerate-pole solid through the boolean pipeline, and self-intersection completeness (S4-f) remain
OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native freeform parametric-pole crossing (S4-e general)

The kernel SHALL extend the native, **OCCT-free** chart-singularity capability in
`cybercad::native::ssi` to cross a FREEFORM parametric pole — a B-spline / NURBS surface with a
DEGENERATE control row (a collapsed U-row / spline cone-tip) where the surface's own `‖∂S/∂u‖`
collapses to zero at a `v` domain edge while the 3D point remains well-defined. When the
S3/S4-c/S4-d/S4-e marcher reaches such a freeform pole, it SHALL DETECT the chart collapse via the
SAME single-surface Jacobian rank-drop witness used for the analytic sphere pole and cone apex —
`‖dU‖` collapsing relative to `‖dV‖` and the model scale on that surface while the surface normal
evaluates FINITE (the degenerate freeform normal being a finite near-zero direction, never a
non-finite value) — and this detection SHALL remain INDEPENDENT of the S4-c pair transversality sine
and the S4-d locus tangent flip. It SHALL STEP across the singular band with the EXISTING
POINT-BASED fixed-plane corrector, whose residuals use only the surface point and the last-good
tangent and therefore do NOT depend on the degenerate `∂S/∂u` or the surface normal. Because a
freeform surface adapter carries no analytic longitude period, the marcher SHALL re-seed the
far-side `(u,v)` for a freeform pole from the CONTINUED 3D TANGENT — keeping the near-pole LATITUDE
`v` fixed (as the analytic reflect does) and recovering the far LONGITUDE `u` by a point-only search
for the `u` at that fixed latitude whose surface point is nearest the continued world point — rather
than the analytic `u_in + π` meridian jump used for the sphere pole; when the surface DOES carry an
analytic longitude period (the sphere pole) the marcher SHALL keep the existing meridian jump
unchanged. The marcher
SHALL resume the normal `(u,v)` march once `‖dU‖` recovers on both surfaces, and SHALL report the
freeform pole it steps across and verifies in the existing chart-singularities-crossed count (a
per-`TraceSet` count and a per-`WLine` count), NOT in `nearTangentGaps`. Outside a detected freeform
pole the analytic sphere-pole and cone-apex crossings, the transversal S3 trace, the S4-c
crossable-graze crossing, and the S4-d branch-point trace SHALL be unchanged. `src/native/**` SHALL
NOT link OCCT; no `cc_*` entry point, signature, or POD struct SHALL be added or changed; the
marching entry points SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the freeform degenerate-pole curve is now crossed

- GIVEN a B-spline surface whose top U-row of control points is COLLAPSED to a single 3D tip point
  (a spline cone-tip, so `‖∂S/∂u‖` collapses to zero at that `v` edge while the tip point is
  well-defined) and a plane through the tip, whose intersection is a curve passing through the tip,
  forced through marching with a hand seed on the curve away from the tip, whose S3 trace today
  truncates at the tip (a boundary exit / one-sided arc that never reaches the far side of the tip),
  with `CYBERCAD_HAS_NUMSCI` built and chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at the tip, STEP across it with the point-based corrector,
  re-seed the far-side `(u,v)` from the continued 3D tangent, and RESUME the march on the far side of
  the tip — reporting at least one chart singularity crossed and yielding `nearTangentGaps == 0` for
  the traced curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the analytic sphere-pole and cone-apex crossings are unchanged

- GIVEN the unit-sphere ∩ plane great circle (which the S4-e analytic slice traces as a full closed
  loop crossing both poles) and the double-cone ∩ plane apex-crossing line (which the S4-e analytic
  slice traces across the apex spanning both nappes)
- WHEN the S4-e marcher traces them with chart-singularity handling enabled
- THEN the sphere-pole great circle SHALL still be fully traced (at least two chart singularities
  crossed, closed status) and the cone-apex line SHALL still be crossed (at least one chart
  singularity crossed) — each identical to the analytic S4-e result, because the freeform-pole
  re-seed engages ONLY when the surface carries no analytic longitude period

#### Scenario: the transversal march, the S4-c graze, and the S4-d branch trace are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a chart singularity, a surface
  pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c slice marches
  through, and the Steinmetz bicylinder whose intersection self-crosses at two branch points the
  S4-d slice localizes and routes
- WHEN the S4-e marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same status,
  same counts), the graze SHALL still be MARCHED THROUGH by the S4-c crossing, and the Steinmetz
  SHALL still be assembled by the S4-d branch machinery (`branchPoints == 2`) — each with zero chart
  singularities crossed, because the chart machinery SHALL engage ONLY at a detected single-surface
  chart collapse

### Requirement: Freeform-pole honesty and curve-cusp decline (S4-e general)

The S4-e marcher SHALL cross a freeform parametric pole only when the crossing is robustly verified,
and SHALL DECLINE the curve cusp without adding an unreachable mechanism. A freeform-pole crossing
SHALL be emitted only if every node across the singular band verifies on BOTH surfaces within
`onSurfTol`, the far side makes real progress off the tip, and the far-side node re-seeded from the
continued 3D tangent verifies on both surfaces. When a freeform pole's far-side node cannot be
re-projected onto both surfaces within `onSurfTol` (an asymmetric freeform pole whose continued-
tangent re-seed misses), when the crossing makes no real far-side progress, or when the point-based
cut cannot resolve the pole, the marcher SHALL DISCARD the crossing arc, STOP, record the typed stop
reason, count the region in `nearTangentGaps`, and DEFER it (→ OCCT) — reporting the measured gap.
For a CURVE CUSP (the intersection curve's own velocity collapsing to zero), the marcher SHALL NOT
introduce a standalone single-surface-chart cusp witness or corrector: because a cusp of the
intersection curve requires the pair transversality sine to collapse (a cusp with regular charts and
a healthy pair sine is impossible by the implicit function theorem), a curve cusp coincides with the
pair-tangency regime and SHALL be handled by the existing S4-c graze crossing (when the branch
continues through), the S4-d branch machinery (when the locus self-crosses), or an honest deferral →
OCCT (a genuine tangential endpoint the point-based step cannot continue). The marcher SHALL NEVER
fabricate a point across a singularity and SHALL NEVER weaken a tolerance to force a crossing; a
singularity that cannot be robustly crossed SHALL remain an honestly reported gap.

#### Scenario: an asymmetric freeform pole that cannot be verified defers honestly

- GIVEN a freeform degenerate pole whose far-side `(u,v)`, re-seeded from the continued 3D tangent,
  does NOT re-project onto both surfaces within `onSurfTol` (an asymmetric tip where the continued
  tangent misses the far side), with chart-singularity handling enabled
- WHEN the S4-e marcher attempts to step across it
- THEN it SHALL DISCARD the crossing arc, STOP at the pole, record the typed stop reason, and count
  the region in `nearTangentGaps` (deferred → OCCT), reporting zero chart singularities crossed for
  that region — it SHALL NOT fabricate a point across the pole and SHALL NOT weaken a tolerance to
  force the crossing

#### Scenario: a curve cusp is declined and routed to the pair-tangency path, never fabricated

- GIVEN an intersection whose curve has a genuine CUSP (the curve velocity collapses to zero), which
  necessarily coincides with the pair transversality sine collapsing
- WHEN the S4-e marcher reaches the cusp
- THEN it SHALL NOT apply a standalone single-surface-chart cusp crossing, but SHALL let the existing
  S4-c graze crossing march through it when the branch continues, the S4-d branch machinery route it
  when the locus self-crosses, or the march STOP and DEFER it → OCCT when it is a genuine tangential
  endpoint the point-based step cannot continue — never fabricating a continuation across the cusp
