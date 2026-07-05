## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness. S4-a/S4-b landed
the CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`); S4-c landed the
FIRST MARCHING-CORE slice — it MARCHES THROUGH a `NearTangentTransversal` **single-branch
graze** the S3 marcher used to truncate, and emits the full curve. But S4-c deliberately
STOPS + defers the moment the near-tangent stall is NOT a single continuing branch but a
**branch point** — where the intersection LOCUS itself crosses / splits (multiple curve
arms meet at one point). **S4-d is the hardest SSI piece**: at that same detection, the
marcher must instead LOCALIZE the branch point, ENUMERATE the outgoing arms, ROUTE down
each, and ASSEMBLE the multi-arm curve.

The canonical target is the **STEINMETZ bicylinder** — two equal-radius cylinders (R=1)
whose axes cross orthogonally at the origin. Its intersection is **two ellipses that CROSS
each other** at TWO branch points (the saddles `(0, ±1, 0)` where all four elliptical arms
meet). S3+S4-c currently DEFER there.

Diagnosed on the current marcher (host build, `CYBERCAD_HAS_NUMSCI` ON,
`trace_intersection` on two equal orthogonal `makeCylinderAdapter` cylinders R=1, axes Z
and X crossing at the origin):

| Fixture | S3+S4-c result today | Verdict |
|---|---|---|
| Steinmetz — equal cylinders R=1, axes crossing at 90° | one WLine, `status = NearTangent`, **229 pts**, `stopReason = NearTangentTransversal` (sine ≈ 9.2e-4), `tracedBranches = 0`, `nearTangentGaps = 1`, `nearTangentCrossed = 0` | **DEFERS at the branch point** — the four arms are never assembled |
| Two spheres at `d = R₁+R₂` (external tangent) | `deferredTangent`/`TangentPoint`, no curve fabricated | **isolated tangent point — must STILL END** |
| Sphere grazed by an offset cylinder (S4-c graze) | `nearTangentGaps = 0`, `nearTangentCrossed ≥ 1`, full loop | **crossable graze — must stay CROSSED** |
| 5 transversal pairs | `nt = 0`, bit-identical | **must stay UNTOUCHED** |

The S4-c gate reaches the branch point precisely because its honesty witnesses fire there:
the transversality sine **STEEPLY COLLAPSES** (orders of magnitude toward zero — a true
tangency/branch drives `sine → 0`) and the raw intersection tangent **FLIPS** (successive
raw tangents stop heading the crossing way — two branches meet). S4-c uses that
collapse+flip to ROLL BACK and defer. **S4-d reuses exactly that detector** but, instead of
deferring, LOCALIZES the branch point and ROUTES the arms.

This change adds the FIRST HONEST S4-d SLICE: at a detected branch point of an **elementary
transversal self-crossing** (the Steinmetz family), localize the branch point B, enumerate
the REAL outgoing arm directions from the local second-order structure, route the normal
march down each arm, dedup arms already traced, and assemble the multi-arm curve into the
`TraceSet` with a reported `BranchPoint` count. Anything not robustly localizable /
enumerable / routable STILL DEFERS (honest `NearTangent` stop + typed reason → OCCT). An
ISOLATED `TangentPoint` (S4-b: definite relative second form — the curve ENDS there) STILL
ENDS; it NEVER sprouts fabricated arms. It NEVER fabricates arms or points, and NEVER
weakens a tolerance.

## What Changes

- **(S4-d-1) BRANCH-POINT DETECTION (reuse the S4-c seam).** At the S4-c
  collapse+flip detection (`crossNearTangent` / `crossNodeCrossable` — the steep sine
  collapse below `minCrossSine` AND the raw tangent flip that today force a defer),
  DISCRIMINATE a genuine branch point from a plain tangency: a branch point is where the
  intersection locus self-crosses — the transversality sine reaches a near-zero MINIMUM at
  an INTERIOR point of both surfaces (not a boundary), and MULTIPLE real curve arms emanate
  from it. A `TangentPoint` (S4-b: sign-definite relative second form — isolated, no
  continuation) is NOT a branch point and MUST still end; a `TangentCurve` / `Undecided` is
  handed on unchanged.
- **(S4-d-2) BRANCH-POINT LOCALIZATION.** From the S4-c collapse, refine the exact param
  point where the transversality sine `‖nA × nB‖` reaches its minimum (≈ 0) along the
  approach — solve for the on-both-surfaces point B where sine is minimized (a 1-D minimize
  of `sine(s)` along the approach direction bracketed by the last-good node and the
  overshoot node, then a full re-project of B onto both surfaces). B lies on BOTH surfaces
  within `onSurfTol` and its sine is at/near the floor. If the minimum cannot be robustly
  bracketed and re-projected on both surfaces, DEFER (no fabricated B).
- **(S4-d-3) ARM ENUMERATION — the tangent cone (real roots only).** At B the intersection
  locus has a TANGENT CONE: the arm directions are the real solutions of a QUADRATIC formed
  from the local second-order structure of the two surfaces at B (the relative second
  fundamental form / the Hessian of the contact restricted to the shared tangent plane).
  For a transversal self-crossing this quadratic has TWO distinct real roots ⇒ two tangent
  lines ⇒ UP TO FOUR outgoing rays (±each line). Return ONLY the REAL, distinct directions.
  If the quadratic has NO real distinct roots (definite form) it is NOT a transversal branch
  — it is a tangent point / curve ⇒ END / DEFER, NEVER fabricate arms. A double root (one
  line) is a cusp/degenerate branch ⇒ DEFER (out of scope).
- **(S4-d-4) ARM ROUTING.** Step off B a small distance along each enumerated ray (using the
  S4-c fixed-plane corrector to land back ON the intersection curve, verified on both
  surfaces ≤ `onSurfTol`), then continue the NORMAL S3 march down that arm to its natural
  termination (loop closure / boundary / another branch point). Each routed arm is a march
  seeded at the branch-adjacent point in the ray direction.
- **(S4-d-5) DEDUP + CONNECTIVITY + ASSEMBLE.** Extend `retraces()` so an arm that retraces
  an already-traced arm (the same locus arriving at the branch point from the other side) is
  deduped, and arms meeting at the SAME branch point B are recorded as CONNECTED at a
  branch-point node in the assembled `TraceSet`. The `TraceSet` gains a `branchPoints` count
  and, per branch point, the list of arm endpoints that meet there. A branch that cannot be
  robustly localized / enumerated / routed ⇒ DEFER that structure (counted in
  `nearTangentGaps`, typed reason), NEVER a partial fake.
- **Marching result carries the branch-point outcome (additive).** `marching.h` gains a
  per-`TraceSet` `branchPoints` count and a `BranchNode` list (the localized point + the
  connected arm ids); a branch point that is localized + routed no longer increments
  `nearTangentGaps`. No `cc_*` ABI change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the FIRST S4-d
branch-point slice (localization + tangent-cone arm enumeration + arm routing + dedup /
connectivity assembly). It adds no new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: add (S4-d) a native, OCCT-free **branch-point** capability that, at a
  detected self-crossing of the intersection LOCUS (multiple real curve arms meeting at one
  point — the Steinmetz-family transversal self-crossing S3+S4-c currently DEFER),
  LOCALIZES the branch point, ENUMERATES the outgoing arms from the local second-order
  structure (the tangent cone — REAL roots only), ROUTES the normal march down each arm,
  and ASSEMBLES the multi-arm curve into the `TraceSet` with a reported `branchPoints`
  count. An ISOLATED `TangentPoint` (S4-b definite form — the curve ENDS) STILL ENDS and
  NEVER sprouts arms; a branch that cannot be robustly localized / enumerated (no real
  distinct tangent-cone roots) / routed STILL STOPS + classifies + defers → OCCT. The
  transversal S3 trace and the S4-c crossable graze are UNTOUCHED. The tracer NEVER
  fabricates an arm or a point and NEVER weakens a tolerance; a branch it cannot resolve is
  an honestly reported gap. No `cc_*` ABI change; `src/native/**` stays OCCT-free; the new
  branch machinery is compiled under `CYBERCAD_HAS_NUMSCI` like S2/S3/S4-c.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change.
  Additive only; the tessellator (`src/native/tessellate`) and the CyberCad app are
  untouched.
- **Build**: extends `src/native/ssi/marching.{h,cpp}` (branch-point detection at the S4-c
  seam, `localizeBranchPoint`, `enumerateArms` (tangent cone), `routeArm`, extended
  `retraces` / `trace_from_seeds` assembly, and the additive `BranchNode` / `branchPoints`
  on `TraceSet`) under `CYBERCAD_HAS_NUMSCI` like the S3/S4-c marcher; consumes the existing
  S4-b `classify_tangent_contact_seeded` (`tangent_seeded.h`) unchanged to reject a
  `TangentPoint` / `TangentCurve` / `Undecided`. Branch-point localization (minimize sine
  along the approach) and arm enumeration (real roots of the tangent-cone quadratic) use the
  native-numerics substrate (`nn::minimize` / a closed-form quadratic + `nn::least_squares`
  for the re-projection). No change to `src/native/tessellate`; no new substrate routine; no
  new tolerance beyond the branch-point discriminators (which are `tangentSinTol` /
  `minCrossSine`-derived, documented, and never weaken a tolerance).
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4d_branch_points.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  the Steinmetz fixture (two equal R=1 orthogonal cylinders) that S3+S4-c today truncate
  (one `NearTangent` WLine, ~229 pts, `nearTangentGaps = 1`, `branchPoints = 0`) is now
  FULLY traced — the two ellipses assembled, BOTH branch points localized (each on both
  surfaces ≤ `onSurfTol`, at/near sine floor) + routed, `branchPoints = 2`, every emitted
  node on BOTH cylinders ≤ `onSurfTol`, the assembled arms matching the analytic Steinmetz
  ellipses within the deflection tolerance; an ISOLATED `TangentPoint` (two spheres at
  `d = R₁+R₂`) STILL ENDS with NO fabricated arms (`branchPoints = 0`); the S4-c crossable
  graze STILL crosses (`nearTangentCrossed ≥ 1`, `nearTangentGaps = 0`, `branchPoints = 0`);
  the 5 transversal pairs trace bit-identically (`nt = 0`). Full CTest green NUMSCI ON and
  OFF (the S4-d assertions absent with NUMSCI off, like S2/S3/S4-c). No OCCT linked; no
  tolerance weakened. **Sim native-vs-OCCT (booted sim)** — extend
  `scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm` (or a
  new `scripts/run-sim-native-ssi-s4d.sh`): assert the Steinmetz intersection is now FULLY
  traced natively — all arms, both branch points localized + routed — matching OCCT
  `IntPatch` / `GeomAPI_IntSS`: every sampled native node on the OCCT locus ≤ `onCurveTol`
  (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤ `onSurfTol`; the native arm / loop
  count reconciles with the OCCT branch count; the localized native branch points match the
  OCCT branch points to `tol`. Report per-pair the localized-and-routed vs still-deferred
  count. `xcrun simctl list devices booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4 from the S4-c near-tangent MARCHING slice to the
  FIRST S4-d BRANCH-POINT slice — the Steinmetz self-crossing S3+S4-c deferred is now fully
  traced vs OCCT. **Explicitly a first slice:** general / freeform branch points,
  singularities (S4-e), and self-intersection completeness (S4-f) remain OUT OF SCOPE, and
  any branch point not robustly localizable / enumerable / routable still defers → OCCT.
- **Risk (honest)**: (a) the branch-point localization can converge to a false minimum
  (a plain graze rather than a true self-crossing) — mitigated by requiring the enumeration
  quadratic to have TWO DISTINCT REAL roots (a genuine tangent cone) before any arm is
  routed, by re-projecting B on both surfaces ≤ `onSurfTol`, and by the sim on-locus /
  branch-point-match parity gate; a suspected branch that fails either check DEFERS. (b) The
  arm enumeration can produce a spurious ray (the second-order form is noisy at the
  near-zero sine) — mitigated by routing an arm ONLY if the first routed step lands back on
  the intersection ≤ `onSurfTol` AND the march makes real progress, and by `retraces` dedup
  folding a spurious arm that coincides with a real one; an arm whose first step cannot be
  verified is DROPPED (not a fabricated arm). (c) A `TangentPoint` could be misread as a
  branch point and sprout arms — mitigated by the S4-b classifier gate (a definite relative
  second form ⇒ `TangentPoint` ⇒ END) AND the no-real-distinct-roots reject in the tangent
  cone; the two-sphere isolated-tangent fixture is the "must-still-end" pin. (d) Routing an
  arm must not perturb the transversal S3 trace or the S4-c graze — mitigated by leaving the
  S3 corrector / deflection controller and the S4-c crossing driver bit-identical, engaging
  the branch machinery ONLY at a detected branch point, pinned by the 5 transversal pairs +
  the S4-c graze staying green. Whatever does not resolve robustly still stops + defers →
  OCCT and is reported with the measured gap; no arm or point is faked, hand-tuned, or
  weakened to pass.
