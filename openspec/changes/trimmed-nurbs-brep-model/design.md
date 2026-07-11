# Design — trimmed-nurbs-brep-model (NURBS roadmap Layer 8)

## Placement decision — new module, not a `FaceSurface` extension

The existing `shape.h` data model ALREADY stores a trimmed NURBS face: `FaceSurface` (the surface,
including `Kind::BSpline`), the face node's child wires (child 0 = outer, rest = holes), and each
edge's `PCurve` on the face (`TShape::EdgePCurve`, read via `topo::pcurveOf`). `step_reader.cpp`
already populates all of this for STEP-read B-spline faces + trims.

Therefore the least-invasive placement is a **new operations module**
`src/native/topology/trimmed_nurbs.{h,cpp}` that reads the existing storage — NOT an extension of
`FaceSurface` or `PCurve`. This:
- adds NOTHING to `shape.h`, so every existing `FaceSurface`/`PCurve` consumer
  (`step_reader`, `tessellate`, `blend`, `boolean`, …) is byte-for-byte unchanged;
- keeps the robust, honest-declining boolean-grade operations SEPARATE from the mesher-oriented
  `tessellate/trim.h` (whose point-in-polygon deliberately classifies boundary points "either way"
  — acceptable for meshing, unacceptable for a boolean, which needs a truthful `OnBoundary`).

`TrimmedNurbsFace` is a self-contained aggregate (surface + location + loops) so a face can be
classified/verified without walking the topology graph on every query; `makeTrimmedFace(Shape)`
snapshots an existing topology face into it (reusing the stored pcurves), and tests assemble it
directly.

## Point-in-region — robust even-odd ray-cast with honest declines

Classification is in PARAMETER space:
1. Flatten each loop's pcurve segments into a `(u,v)` polyline (dense per segment; consecutive
   duplicate join vertices dropped; the closing duplicate removed so the implicit-close edge is
   unique). Pcurve evaluation mirrors `tessellate::pcurveValue` (analytic Line/Circle/Ellipse via
   `origin2d`/`dir2d`/radii; free-form B-spline via `curvePoint`/`nurbsCurvePoint` when a proper
   knot vector is present, else a pole-lerp fallback) — the SAME seam-weld contract, duplicated
   locally to keep `topology` from depending on `tessellate`.
2. **On-edge band FIRST**: a point within a scale-relative `onEdgeTol` (× the loop's UV extent) of
   ANY loop edge is `OnBoundary`. This is checked before parity so a boundary point is never
   mis-bucketed as In/Out.
3. **Parity**: the Franklin PNPOLY half-open rule `(a.v > p.v) != (b.v > p.v)`. This treats a
   shared vertex consistently (one incident edge's half-open v-interval includes it, the other
   excludes it), so a ray grazing a polygon vertex — which happens constantly for an axis-aligned
   rectangle sampled uniformly — is handled deterministically and CORRECTLY without a per-vertex
   veto (an earlier veto-based attempt spuriously declined every rectangle; rejected).
4. **Honest decline**: a loop with < 3 distinct points (empty/open) or a repeated non-adjacent
   vertex (self-touch / pinch) fails `loopWellFormed` → `Unknown`. Holes are checked the same way;
   any degenerate hole makes the whole query `Unknown`.

Keep rule: `In` iff inside the outer loop AND outside every hole; a point inside a hole → `Out`.

## Pcurve fidelity — the load-bearing invariant

`pcurveFidelity` samples `[first,last]` densely; at each `t` it evaluates `p(t) → (u,v)`,
`S(u,v)` (world-placed) and `C(t)` (world-placed) and measures `‖S(p(t)) − C(t)‖`. The tolerance
is **scale-relative**: `tol = absTol + relTol · L` where `L` is the summed chord length of the
sampled 3-D edge — a large part is not held to an absolute-µm bar, a tiny part is not passed
loosely. The report carries `maxDeviation`, `meanDeviation`, the applied `tolerance`, and the `t`
that achieved the worst deviation. A pcurve not on `S` produces a large `maxDeviation` and `ok=false`.

## Pcurve construction (numsci-gated)

`constructPcurve` samples the 3-D edge, projects each point to `(u,v)` with
`numerics::closest_point_on_surface` (multi-start, robust), and — if the projection residual is
within a small fraction of the edge's world extent (else the edge is NOT on `S` → honest decline)
— fits a 2-D B-spline through the projected feet with `bspline_fit::interpolateCurve`. The fitted
curve is parametrized on `[0,1]`; its knots are reparametrized onto the edge's `[first,last]` so
`pcurve(t)` is evaluated at the SAME parameter as `C(t)` (the fidelity contract). It then
round-trip-verifies via `pcurveFidelity` and returns `ok` iff the round-trip is met. Non-rational
(`bspline_fit` scope); the true deviation is always reported.

## Verification (host, no OCCT — the airtight gate)

`tests/native/test_native_trimmed_nurbs.cpp`:
1. **Containment** — a rectangular sub-region of a bicubic B-spline patch with a circular hole:
   provable inside/outside classify correctly; boundary → `OnBoundary`; hole interior → `Out`;
   removing the hole makes the former hole center `In`.
2. **Fidelity** — an exact iso-curve of the patch (`S(u0,·)` extracted by collapsing the u-basis)
   has `S(pcurve(t)) == C(t)` to ~1e-9; a wrong (drifting) pcurve is DETECTED (deviation > 1e-3).
   The planar case is machine-exact.
3. **Construction round-trip** (numsci) — a 3-D edge that lies on `S` (an interpolating B-spline
   through dense `S`-path samples) is projected+fitted; the round-trip fidelity holds within the
   fit tol and the reconstructed pcurve stays in the path's `(u,v)` box; an off-surface edge
   declines honestly.
4. **Degenerate guards** — empty / open / self-touching outer loops, and a degenerate hole →
   `Unknown`.

Numsci wiring mirrors `test_native_nurbs_fit`: the gate is always built (its containment /
fidelity / degenerate legs are substrate-free); the construction leg is `#ifdef
CYBERCAD_HAS_NUMSCI` and the TU gets the macro (no substrate include trees needed — it calls only
always-visible topology declarations).
