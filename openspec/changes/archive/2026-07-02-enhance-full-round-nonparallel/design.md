## Context

`src/engine/occt/occt_full_round_fillet.cpp` builds a face-consuming rolling-ball
full round NATIVELY (OCCT has no "consume this face and blend its neighbours"
call, and `BRepFilletAPI_MakeFillet` refuses the critical merge radius). The
construction is:

1. `resolveAuto` / `resolveFromFaces` find the two seam edges (`eL`, `eR`), the two
   neighbour side faces (`left`, `right`), the middle face's outward normal
   (`nMiddleOut`), the perpendicular strip `width`, the strip/axis direction
   (`axisDir = edgeDirection(eL)`), and the seam midpoint (`seamMid`).
2. `rollingBallEligible` gates the native path.
3. `buildRollingBall` places a solid cylinder of radius `r = width/2` with its axis
   on the strip mid-plane, tangent to both walls, then carves `body − (slab −
   cylinder)` so the two top corners round into the cylinder wall (the blend), and
   the middle face is provably gone.
4. On ineligibility or an invalid boolean, `fallbackEdgeFillet` returns a valid
   standard edge fillet and the case is recorded deferred.

The gate today (`rollingBallEligible`) is:

```
if (!isPlanar(left) || !isPlanar(right)) return false;   // planar only
if (width < 1e-6 || seamLen < 1e-6) return false;
nL = faceOutwardNormal(left);  nR = faceOutwardNormal(right);
return nL.Dot(nR) < -0.98;                                // (anti-)PARALLEL only
```

and `buildRollingBall` derives the axis from the (anti-)parallel assumption:

```
r      = 0.5 * width;
inward = -r * nMiddleOut;
axisPt = seamMid + inward;          // ON the strip mid-plane  (parallel-only)
axisDir= edgeDirection(eL);         // along the strip
```

This is correct ONLY when `nL ≈ -nR` (walls parallel, strip constant-radius). Two
planar walls meeting at any other dihedral angle are rejected and silently take the
lower-fidelity fallback — even though a constant-radius rolling ball exists for
them. This change closes exactly that gap, planar-only.

## Goals / Non-Goals

Goals:
- Handle NON-PARALLEL PLANAR neighbour walls with the SAME native tangent-cylinder
  full round: a valid watertight solid, middle face consumed, G1-tangent to both
  neighbours at the seams.
- Preserve the parallel path bit-for-bit (same eligibility outcome, same expected
  volume/axis for the existing rib fixture).
- Verify on the simulator with REAL asserted properties, never a trivially-true
  check.

Non-Goals:
- CURVED (non-planar) neighbours. Out of scope; keep the valid fallback + honest
  defer with the measured gap.
- Variable-radius / non-constant strips, multi-face chains, and any `cc_*` ABI or
  POD-struct change.

## The dihedral tangent cylinder (the real algorithm)

Let the two neighbour planes be `P1` (through a point `a1` on seam `eL`, unit
outward normal `n1`) and `P2` (through `a2` on seam `eR`, unit outward normal
`n2`). A ball of radius `r` tangent to both planes has its centre `c` at signed
perpendicular distance `r` from each plane, on the INTERIOR (valley / material)
side. Rolling that ball keeps `c` on the line

    L(t) = c0 + t · d,   d = normalize(n1 × n2)          (the crease direction)

so the swept tangent surface is a CYLINDER of radius `r` with:

- **axis DIRECTION** `d = normalize(n1 × n2)` — the intersection direction of the
  two planes (for two side walls of a rib, this is exactly the strip direction
  `axisDir` the parallel path already uses; the generalization agrees there);
- **axis LOCATION** `c0` — the point on the interior dihedral bisector at
  perpendicular distance `r` from BOTH planes.

**Solving for `c0`.** Work in the plane spanned by `n1, n2` (the cross-section
perpendicular to `d`). Interior offset directions are `-n1` and `-n2` (into the
material, since normals are outward). The centre must satisfy, for a point `p0` on
the crease line (any solution of the two plane equations, e.g. the near-point of
`P1 ∩ P2`):

    (c0 − a1) · n1 = −r        (distance r on the interior side of P1)
    (c0 − a2) · n2 = −r        (distance r on the interior side of P2)
    (c0 − p0) · d  = 0         (c0 in the cross-section through p0)

Three linear equations, one unique `c0` (the 3×3 system `[n1; n2; d]` is
non-singular exactly when `n1 × n2 ≠ 0`, i.e. the non-parallel branch). Equivalently
and more robustly for implementation: the interior bisector direction is
`b = normalize(−n1 − n2)` (points into the valley), the half-dihedral is
`θ = ½·angle(n1, n2)` measured on the material side, and

    c0 = p0 + b · (r / sin θ)

places the centre so its perpendicular foot on each plane is at distance `r`. Both
forms give the same point; the implementation SHOULD solve the linear system (it
degrades gracefully and needs no separate `sin θ`).

**Choosing `r`.** `r` is fixed by the strip geometry exactly as the parallel case
picks it: the seam-edge offset / half the perpendicular strip width. Concretely
`r` is chosen so the tangent points on `P1` and `P2` fall on (or just outside) the
two seam edges `eL`, `eR` — i.e. the cylinder spans the strip. The parallel case's
`r = width/2` is the `θ = 90°` specialization of the general
`r = (strip half-width) · sin θ` relation; the implementation derives `r` from the
seam geometry so both branches share one expression.

**Parallel case is the limit.** As `n2 → −n1`, `n1 × n2 → 0` along the strip
direction and the bisector plane → the strip mid-plane, so `c0 →` the existing
`seamMid − r·nMiddleOut`. The two branches meet continuously; we keep the explicit
anti-parallel branch (its `d` is taken from `edgeDirection(eL)`, since `n1 × n2` is
numerically unreliable there) and add the dihedral branch for everything in
between.

**Carve.** With `(c0, d, r)` in hand, `buildRollingBall` is otherwise unchanged:
build the solid cylinder along `d` centred to span `seamLen`, build the covering
corner slab, and cut `body − (slab − cylinder)`. The slab's frame generalizes from
`(Z = nMiddleOut, X = axisDir)` to `(Z = −b outward = the mean of n1,n2, X = d)`
so it still covers the whole strip top for a splayed wall. Result gated on
`BRepCheck_Analyzer::IsValid`; on failure, the existing fallback runs.

## Eligibility (widened, still planar-only)

```
bool rollingBallEligible(info):
    if !isPlanar(left) || !isPlanar(right):   return false     // curved → fallback
    if width < 1e-6 || seamLen < 1e-6:        return false
    d = n1 × n2
    if |d| >= kCreaseFloor:                   return true       // NON-PARALLEL dihedral (new)
    return n1·n2 < -0.98                                        // (anti-)PARALLEL (existing)
```

`kCreaseFloor` (e.g. `sin(1°)`) separates the two branches; between the floor and
anti-parallel, the dihedral branch owns it. Convex vs. concave crease is
disambiguated by orienting the bisector toward the body interior using the body
centroid (the same trick `faceOutwardNormal` already uses), so the ball is placed
in the valley, not on the ridge.

## Decision: extend the existing native path, do not add a new op

- **No new C ABI / capability.** The two entry points already exist and their
  contract ("consume the middle face into a blend tangent to both neighbours") is
  agnostic to the dihedral angle; only the internal construction was too narrow.
  This keeps the change additive-internal and `tests/test_abi.cpp` untouched.
- **One axis solver, two callers.** Extract the `(c0, d, r)` derivation into a
  small `planarDihedralAxis(info, ...)` helper so `buildRollingBall` stays a thin
  carve. The parallel branch calls the same helper (anti-parallel special case) so
  there is a single source of truth for the tangent geometry.
- **Cognitive complexity.** This is a systems/geometry TU (target band 25–35). The
  solver is a short linear solve; `buildRollingBall` stays a straight-line carve;
  `rollingBallEligible` is guard clauses. Keep the solver isolated and documented
  rather than inlining the plane math into the carve.

## Honesty / verification

A check MUST assert a REAL property. For the non-parallel rib the on-sim check
asserts (all via the `cc_*` facade, since the check TU cannot include OCCT):

- **valid + watertight**: `cc_mass_properties(out).valid == 1`, positive volume.
- **middle face consumed**: the flat top face at the original strip height is gone
  (`flat_top_remains` is false) — equivalently, the middle face id no longer
  resolves to a face in the rebuilt body.
- **blend is the rolling cylinder along the crease**: `find_blend_cylinder`
  succeeds and the cylinder axis direction matches `normalize(n1 × n2)` (the strip
  direction) within tolerance.
- **G1 tangency to BOTH neighbours**: the blend-cylinder axis is at perpendicular
  distance `r` from BOTH neighbour planes AND the blend surface normal at each seam
  contact equals the corresponding neighbour outward normal (`dot ≥ cos(1°)`).

If the dihedral blend genuinely cannot be built for a sub-case (e.g. a degenerate
crease, or an invalid boolean), the op returns the VALID standard-fillet fallback
and the check records `ctx.defer(...)` with the measured tangency gap and dihedral
angle — it never asserts a full round that isn't tangent, and never a
trivially-true check. Curved neighbours remain deferred by construction.

## Risks / Trade-offs

- **Convex vs. concave crease sign.** Mitigated by orienting the bisector toward
  the body centroid; the sim check's watertight + tangency assertions catch a
  wrong-side placement (it would fail, not silently pass).
- **Very shallow dihedral (near-flat crease).** `|n1 × n2|` small but non-zero →
  large `r/sin θ`; if the boolean becomes invalid the fallback + defer covers it
  honestly. `kCreaseFloor` keeps such cases out of the dihedral branch when the
  crease is numerically unreliable.
- **Regression on the parallel path.** Guarded by keeping the anti-parallel branch
  explicit and re-running the existing parallel-rib check with unchanged expected
  volume (`720 + 20π`) and axis (`x=0, dir ±z`).
