# Proposal — moat-m4-step-bspline-admission (MOAT M4, first slice)

## Why

M0 (the keystone) LANDED the native trimmed-freeform mesher: a genuinely-trimmed
curved B-spline/NURBS face now meshes watertight through
`src/native/tessellate/face_mesher.h::trimmedFreeformMesh` + the constrained-Delaunay
interior triangulator (`uv_triangulate.h::triangulateConstrained` /
`ConstrainedDelaunay`). But M0 explicitly **DEFERRED** the one piece that feeds that
mesher from a foreign file: the **STEP-reader admission** of a foreign
`B_SPLINE_SURFACE_WITH_KNOTS` face. It was deferred because it can only be validated
against the sim-side `BRepMesh` oracle (watertight + area/volume vs OCCT) and the STEP
round-trip suites, none runnable in the OCCT-OFF host worktree M0 landed in
(`openspec/MOAT-ROADMAP.md` M0 "Deferred (needs the OCCT-linked simulator)"). That
gate is now runnable, so this is M4's first slice.

The concrete gap is in `src/native/exchange/step_reader.cpp`. A trimmed
`B_SPLINE_SURFACE_WITH_KNOTS` face (`Kind::BSpline`, real `EDGE_LOOP`, not a full
revolution) already routes to `buildFaceWithPCurves`, but the ONLY B-spline-surface
path in `pcurveFor` is the fall-through **"Generic (should not reach here …)"** linear
arm (lines ~1912-1916): it projects the two edge endpoints and joins them by a **UV
Line** with **no faithfulness check**. That is correct ONLY for an edge that is
straight in `(u,v)` (a rim / seam / isoparametric trim). A **genuinely curved**
boundary edge — whose `(u,v)` image is not a straight segment — gets a WRONG linear
pcurve, so `S_face(pcurve(t)) ≠ C_edge(t)`, the trim is off the true boundary, and the
patch is not faithfully reconstructed. There is also **no** `Kind::BSpline` case in the
guard evaluator `surfaceValue` (line ~2028 returns `frame.origin`), so nothing in the
reader can even verify a B-spline face today. The result is that a foreign curved-edge
trimmed patch is admitted **unfaithfully** and left for the engine's downstream
watertight/volume self-verify to (maybe) catch — not an explicit, measured decline.

This slice closes that: a faithful curved-edge B-spline-surface pcurve arm PLUS an
explicit `S_face(pcurve(t)) = C_edge(t)` reconstruction guard that DECLINES → OCCT on
ANY unfaithful edge, so exactly ONE foreign trimmed B-spline patch is admitted and
meshed watertight (consuming the landed M0 mesher, untouched), verified native-vs-OCCT
on the simulator — or declined honestly with the measured gap.

## What Changes

1. **A faithful B-spline-surface arm in `pcurveFor`** (`step_reader.cpp`), taken for a
   `Kind::BSpline` face surface, replacing the blind generic-linear fall-through:
   - a **straight-in-`(u,v)`** edge (rim / seam / isoparametric trim) → project both
     endpoints via the existing `projectBSplineUV` surface inversion → a UV `Line`
     (byte-identical to what the generic arm produces today for the straight case, so
     the already-passing native B-spline-wall round-trips are unchanged);
   - a **genuinely curved** boundary edge → densify N samples along the 3D edge,
     invert each to `(u,v)` (`projectBSplineUV`), and emit a UV `B_SPLINE` pcurve
     (`poles2d`, `degree`/`knots` preserved from the 3D edge curve) that the landed
     tessellator already evaluates (`trim.h::pcurveValue` case `K::BSpline`).
2. **A faithful-reconstruction GUARD** in `buildFaceWithPCurves` / `pcurveFor`: for
   EVERY boundary edge, re-evaluate `S_face(pcurve(t)) = C_edge(t)` at several `t`
   across `[first,last]` and require the gap ≤ a **scale-relative** tolerance (the same
   `1e-6 · max(1,scale)` form as the existing `pointOnSurface` guard — never weakened).
   Any edge failing ⇒ `decline()` the face (NULL → OCCT). This needs a `Kind::BSpline`
   case added to the guard evaluator `surfaceValue` (rational-aware, via
   `math::nurbsSurfacePoint` over `s.weights`, mirroring the existing `revolutionValue`
   helper) so the guard can evaluate the admitted patch.
3. **Admission of ONE foreign trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face**: with (1)+(2)
   in place, `advancedFace` → `buildFaceWithPCurves` builds the trimmed `Kind::BSpline`
   face and the landed M0 mesher meshes it watertight. The tessellator is **NOT** edited
   (M0 already shipped it); this change is reader-only.
4. **The honest-out is preserved end-to-end.** The engine's mandatory watertight +
   volume/area self-verify remains the final arbiter: an admitted native trimmed patch
   whose mesh is not watertight or whose volume/area does not match the OCCT oracle is
   DISCARDED → OCCT. No tolerance is weakened; `src/native/**` stays OCCT-free; the
   `cc_*` ABI is unchanged (internal reader behaviour only).

## Capabilities

### Modified Capabilities

- `native-exchange`: ADDS the sim-verified STEP-reader admission M0 deferred — a
  faithful curved-edge B-spline-surface pcurve arm in `pcurveFor` plus an explicit
  `S_face(pcurve(t)) = C_edge(t)` faithful-reconstruction guard — so ONE foreign
  trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face (rational or not) whose boundary pcurves
  reconstruct faithfully is admitted and meshed watertight by the landed M0 tessellator,
  matching OCCT `STEPControl_Reader` re-import + `BRepMesh` on count / volume / area /
  watertightness / topology; any unfaithful edge DECLINES → OCCT.

## Impact

- `src/native/exchange/step_reader.cpp` — `pcurveFor` gains an explicit `Kind::BSpline`
  surface arm (straight-UV `Line` + curved-edge UV `B_SPLINE` via densified
  `projectBSplineUV` samples); `buildFaceWithPCurves` gains the per-edge
  `S_face(pcurve(t)) = C_edge(t)` guard (→ `decline()` on any unfaithful edge);
  `surfaceValue` gains a rational-aware `Kind::BSpline` case for the guard. The existing
  Plane / Cylinder / Cone / Sphere pcurve arms, the bare-periodic full-sphere/torus and
  `isFullRevolutionBSpline` paths, and every existing `decline()` precedent are
  UNTOUCHED. Cognitive complexity kept in the systems band by delegating the curved-edge
  fit and the guard to small helpers.
- `src/native/tessellate/**` — **NOT modified.** This change CONSUMES the landed M0
  mesher (`trimmedFreeformMesh`, `triangulateConstrained`, `trim.h::pcurveValue`
  `K::BSpline`) exactly as shipped.
- **Two gates (both mandatory):** (a) HOST ANALYTIC — a native trimmed `Kind::BSpline`
  face with a closed-form curved boundary is built with NO OCCT linked; the guard is
  proven to ACCEPT the faithful pcurve (`S(pcurve(t)) = C_edge(t)` within tol at many
  `t`) and REJECT a deliberately off-surface edge, and the meshed solid is watertight
  with the closed-form volume. (b) SIM native-vs-OCCT — on a booted iOS simulator the
  foreign trimmed-B-spline STEP fixture imports to a native solid whose volume / area /
  watertight / triangle envelope / topology match OCCT `STEPControl_Reader` re-import +
  `BRepMesh`, OR the reader declines and the file round-trips through OCCT unchanged.
- **Zero-regression discipline (mandatory).** The straight-UV arm reproduces the current
  generic-linear pcurve byte-for-byte, so every existing STEP round-trip stays green:
  `run-sim-suite` 221/221, STEP import 77/77, and the tessellation-sensitive suites are
  unaffected (no tessellator edit). Proven, not assumed.
- **Out of scope (declines, documented not faked):** curved boundary pcurves that do not
  reconstruct faithfully (non-converging inversion, off-surface fit, beyond-tolerance
  gap); self-intersecting trim wires; multi-patch foreign assemblies; AP242 PMI
  semantics; general STEP breadth beyond this ONE trimmed patch (the rest of M4). No
  `cc_*` ABI change; no CyberCad app change; no OCCT linked into `src/native/**`.
