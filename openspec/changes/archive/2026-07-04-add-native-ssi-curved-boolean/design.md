# Design — add-native-ssi-curved-boolean (SSI Stage S5-a)

## Context

`SSI-ROADMAP.md` frames analytic SSI as the **enabler** and **general curved
booleans** as the **payoff**. S1 / S2 / S3 are shipped: the S3 tracer
(`src/native/ssi/marching.h`) returns a **`TraceSet`** — one `WLine` per distinct
**transversal** intersection branch, each `WLinePoint` carrying its 3D point AND its
`(u1,v1,u2,v2)` on **both** surfaces, plus a fitted B-spline and a per-curve
`TraceStatus` (Closed / BoundaryExit / NearTangent / Failed) and `nearTangentGaps`.

Today `src/native/boolean/` handles two families and neither consumes SSI:
- **Planar BSP-CSG** (`bsp.h` / `polygon.h` / `assemble.h`): flatten every face to an
  oriented planar `Polygon`, do clip/invert set algebra on the polygon soup, weld back
  to a `Solid`. Curved faces cannot be flattened, so this path gates on `isAllPlanar`.
- **Analytic box∩cylinder** (`curved.h`): PATTERN-MATCHED — `recogniseBox` /
  `recogniseCylinder` + `buildCutHole` / `buildBlindHole` / `buildFuseBoss` build the
  result analytically **per primitive**. It does **not** read SSI curves and does not
  generalize beyond the axis-aligned box∩axis-parallel-cylinder family.

**S5-a is the general, SSI-curve-driven path.** It drives the split from the S3
`TraceSet` WLines, so it generalizes across the **transversal elementary** curved-face
family (cylinder / sphere / cone / plane) instead of hand-matching each primitive. The
method is **clean-room**: OCCT `BRepAlgoAPI` / `BOPAlgo` is the verification **oracle**
only, never copied.

Scope is **honestly narrow**: transversal elementary pairs where S3 produced a fully
transversal trace (`nearTangentGaps == 0`). Near-tangent / coincident / branch-point /
freeform pairs are **deferred to S4 + OCCT fallback**.

## Goals / Non-Goals

**Goals**
- Consume the S3 `TraceSet` and compute native `fuse` / `cut` / `common` for
  **transversal elementary** curved pairs (cyl∩cyl, sphere∩box, cone∩box, general
  cyl∩box), returning a watertight curved-faced `Solid` with the correct set-algebra
  volume/area, or NULL (→ OCCT) when out of scope.
- **Split** each curved face along its `WLine` using the WLine's on-surface `(u,v)`
  track as the split polyline and the fitted B-spline as the shared seam edge.
- **Classify** fragments INSIDE / OUTSIDE / ON the other solid with a **curved
  point-in-solid** test (the `bsp.h` classification idea generalized to curved faces).
- **Weld** the surviving fragments into one closed watertight `Solid`, sharing the
  WLine seam between the two faces it splits (extend `assemble.h`'s corner-weld to
  curved faces + the curved seam).
- Wire the engine opt-in behind the mandatory **watertight + correct-volume**
  self-verify → OCCT `BRepAlgoAPI` fallback.
- Report native-vs-OCCT parity (volume / area / watertight / validity), with the
  near-tangent gap boundary called out.

**Non-Goals (deferred — never faked here)**
- **Near-tangent / coincident / degenerate** pairs (any pair where S3 reports
  `nearTangentGaps > 0`, tangent faces, coincident/overlapping surfaces) → **S4 + OCCT
  fallback**. Not consumed.
- **Branch-point / self-intersecting** seams (multi-branch singular crossings) → **S4**.
- **Freeform** (NURBS / Bézier) operand faces → deferred (elementary family only in
  S5-a); OCCT fallback.
- The **tracing** itself — that is S3, S5-a's input.
- Any `cc_*` facade entry point or ABI change; any change to the planar BSP-CSG or the
  analytic `curved.h` paths (S5-a is a sibling path).

## Module shape

```
src/native/boolean/
  ssi_curved.h / .cpp   // SSI Stage S5-a: split → classify → weld, driven by ssi::TraceSet   [CYBERCAD_HAS_NUMSCI]
  native_boolean.h      // boolean_solid tries: analytic curved.h → SSI-driven ssi_curved → planar BSP-CSG
```

`ssi_curved` reuses: `native-ssi` `TraceSet` (the WLines — the input contract), `bsp.h`
(the inside/outside classification **idea**, generalized), `assemble.h` (weld → shell →
`Solid`), `native-math` (surface `point`/`dU`/`dV`/`normal` + B-spline eval, `topology`
face/edge/shell), and `native-numerics` (closest-point / containment). It is OCCT-free.

## Pipeline (split → classify → weld, consuming the TraceSet)

### 0. Gate + trace
`boolean_solid` first tries the analytic `curved::tryBoxCylinder` (unchanged). On NULL,
if either operand carries an elementary curved face (Cylinder / Sphere / Cone) it enters
the SSI path: for each pair of intersecting faces `(fA, fB)` it builds the two
`SurfaceAdapter`s and calls `ssi::trace_intersection` (or `trace_from_seeds`). If the
`TraceSet` reports **`nearTangentGaps > 0`**, or any consumed `WLine.status ==
NearTangent | Failed`, the pair is **not transversal-elementary** → return NULL → OCCT.
Only a fully transversal `TraceSet` (Closed / BoundaryExit WLines) proceeds.

### 1. Split — cut each curved face along its WLine
Each `WLine` records, per node, the `(u,v)` on **surface A** and the `(u,v)` on
**surface B**. For face `fA`, the ordered `(u1,v1)` track is the **split polyline in
fA's UV domain**; likewise `(u2,v2)` for `fB`. Cutting a face along that UV polyline is
the **curved analogue of `bsp.h`'s `splitPolygon`**: it partitions the face's trimmed
UV region into fragments on either side of the seam (a Closed WLine encloses a UV loop →
inner/outer fragments; a BoundaryExit WLine runs edge-to-edge → two side fragments). The
fragment's boundary is the existing face trim edges plus the **new seam edge**, whose 3D
geometry is the WLine's **fitted B-spline** and whose pcurve on each face is that face's
`(u,v)` track. Each fragment keeps its parent face's exact surface (Cylinder / Sphere /
Cone / Plane) — nothing is faceted.

### 2. Classify — curved point-in-solid at an interior sample
For each fragment, take an **interior sample** `(u*,v*)` (a UV point strictly inside the
fragment, away from the seam), evaluate `P* = face.point(u*,v*)`, and classify `P*`
INSIDE / OUTSIDE / ON the **other** solid with a **curved point-in-solid** test: this
reuses the `bsp.h` classification **idea** (which side of the boundary the point falls)
generalized to curved faces — a containment test against the other operand's exact faces
(signed distance / ray-crossing parity against Cylinder / Sphere / Cone / Plane faces,
via `native-math` closest-point). A sample within tol of a face (**ON**) is a coincident
/ near-tangent signal → the pair is out of scope → NULL → OCCT (never a guessed side).
Survivors are selected per the op's face-survival rule — the **same set algebra** as
`booleanPolygons`:
- **fuse** `A ∪ B`: fragments of A **outside** B + fragments of B **outside** A;
- **cut** `A − B`: A **outside** B + B **inside** A, orientation reversed;
- **common** `A ∩ B`: A **inside** B + B **inside** A.

### 3. Weld — watertight curved shell
Sew the surviving fragments into one `Solid`, extending `assemble.h`: weld coincident
corners → shared `Vertex`; the **WLine seam edge** is shared by exactly the two
fragments it split (one from each operand), so the two faces meet watertight along the
curved seam — the same **curved-seam weld** the tessellator (`src/native/tessellate`)
already performs. Fragments carry true curved face kinds, so the result tessellates
watertight by the identical mesher path and carries the exact analytic surface geometry.

### 4. Self-verify → OCCT fallback (ENGINE, not the library)
`ssi_curved` returns the assembled `Solid` or NULL — it does **not** decide shippability.
The ENGINE (`native_engine.cpp`) runs the mandatory guard: (a) **watertight** closed
2-manifold at every deflection in the mesher's ladder (`boundaryEdgeCount == 0`, every
edge shared by exactly two faces, positive enclosed volume) AND (b) **correct
set-algebra volume** sign and magnitude for the op (within a relative tolerance sized to
the curved-face tessellation deflection). If EITHER fails, the engine **DISCARDS** the
native result → OCCT `BRepAlgoAPI` (an OCCT operand) or an honest error (both native).
The guard lives in the engine, next to the OCCT fallback; the library stays OCCT-free.

## Curved point-in-solid classification (the crux)

The planar `bsp.h` classifies a point by which side of a plane it falls. For a curved
solid the analogue is a **containment test against the exact faces**: for elementary
solids (box, cylinder, sphere, cone) an **inside/outside signed test** per bounding face
is closed-form (a cylinder is `r(P) < R` within the axial slab; a sphere `‖P−c‖ < R`; a
box the six half-spaces; a cone the apex-angle test within the height slab). Compose them
per the solid's boolean structure — reusing the `bsp.h` clip/invert idea to combine
half-space results — and use `native-numerics` closest-point to measure the ON-band. A
sample in the ON-band aborts the native path (out of scope → OCCT), never a coin-flip.

## Transversal-vs-deferred scope (honest)

| Configuration | S5-a behavior |
|---|---|
| **Transversal elementary** pair, S3 `nearTangentGaps == 0`, all WLines Closed/BoundaryExit | split → classify → weld → self-verify → native `Solid` (or discard → OCCT) |
| S3 reports **`nearTangentGaps > 0`** (branch traced up to a tangent) | **NOT consumed** → NULL → OCCT (S4 seam), reported |
| A consumed WLine is **NearTangent / Failed** | out of scope → NULL → OCCT |
| Fragment interior sample lands **ON** the other solid (coincident / tangent face) | out of scope → NULL → OCCT (never a guessed side) |
| **Coincident / overlapping** curved faces (no discrete transversal seam) | **deferred to S4** → NULL → OCCT |
| **Branch-point / self-intersecting** seam | **deferred to S4** → NULL → OCCT |
| **Freeform** (NURBS / Bézier) operand face | deferred (elementary only in S5-a) → NULL → OCCT |
| Self-verify (watertight or volume) fails on the assembled candidate | engine **DISCARDS** → OCCT, reported |
| Either operand not a native body | → OCCT (needs native B-rep of both) |

## Verification model (two gates)

- **Host (no OCCT) — analytic oracle where one exists.** Two equal-radius cylinders
  (radius `r`) crossing at right angles → the **Steinmetz solid**: `common` enclosed
  volume `= 16 r³ / 3` (exact), asserted within the curved-face deflection band; the
  through-`cut` volume is the box/parent complement. Plus, on every host case:
  watertight (`boundaryEdgeCount == 0`, every edge shared by exactly two faces), correct
  volume **sign**, positive enclosed volume, and every WLine seam node on **both**
  surfaces ≤ tol. A near-tangent fixture (equal coaxial cylinders / tangent sphere) must
  return NULL (deferred), emitting no native solid. No OCCT linked.
- **Sim native-vs-OCCT — BRepAlgoAPI parity.** Build the same operands as OCCT
  `BRepPrimAPI` solids, run `BRepAlgoAPI_{Fuse,Cut,Common}`, and compare **volume**,
  **surface area**, **watertightness** (closed shell), and **shape validity**
  (`BRepCheck`) on cyl∩cyl (Steinmetz + cut + fuse), sphere∩box, cone∩box. Modelled on
  `scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm` —
  a new `native_ssi_curved_boolean_parity.mm` harness built against the SDK sysroot,
  linked with `libnumsci_full_iossim_arm64.a` + the OCCT-for-iOS libs, run via `xcrun
  simctl spawn <booted udid>`. Parity is a **reported** figure; the near-tangent gap
  count (pairs deferred to OCCT) is called out. Whatever S5-a cannot compute falls back
  to OCCT and is reported with the measured gap.

## Decisions

- **Split driven by the WLine `(u,v)` track, not per-primitive.** Using the S3 curve's
  on-surface params as the UV split polyline is what makes S5-a **general** across the
  elementary family — the exact reason it is a sibling to, not an extension of, the
  hand-matched `curved.h`. The fitted B-spline is the shared seam geometry so the two
  faces meet on identical 3D points.
- **Reuse the `bsp.h` classification idea, generalized to curved faces.** The
  inside/outside/on decision and the fuse/cut/common survival rules are the proven
  planar set algebra; only the point-in-solid primitive changes (plane half-space →
  curved containment). This keeps the op mapping identical to `booleanPolygons`.
- **Weld via `assemble.h` extended to curved seams.** The corner-weld → shared-vertex →
  shell → Solid path already produces watertight solids the mesher can read; extending it
  to carry curved faces + the curved seam reuses the same watertight guarantee.
- **Self-verify stays in the ENGINE.** As with the planar and analytic paths, the
  library returns a candidate and the engine runs the mandatory watertight +
  correct-volume guard and owns the OCCT fallback — keeping the library OCCT-free and
  single-purpose.
- **`nearTangentGaps > 0` is the fallback boundary, not an error.** A non-transversal
  trace is honest S3 data marking the S4 seam; S5-a respects it by declining (→ OCCT),
  never by consuming a truncated trace or fabricating the seam remainder.
- **Substrate-gated.** The path consumes the S3 tracer (`least_squares` corrector +
  B-spline fit), so it is compiled under `CYBERCAD_HAS_NUMSCI`, like `native-ssi`.

## Risks / Trade-offs

- **The near-tangent / coincident / branch-point moat.** The dominant honest limit: any
  pair S3 cannot trace transversally is not consumed. Mitigation: gate on
  `nearTangentGaps == 0` + per-WLine status; decline → OCCT, reported. Accepted.
- **Fragment misclassification near the seam.** An interior sample near-ON the other
  solid can flip inside/outside. Mitigation: an ON-band abort (→ OCCT) + the mandatory
  engine self-verify (watertight + correct volume) DISCARDS a mis-welded candidate → OCCT
  — a misclassification never ships as a leaky/wrong solid.
- **Hairline seam gap on high curvature.** The curved-seam weld can leave a gap where the
  fitted B-spline seam and the two faces disagree beyond the deflection band. Mitigation:
  the watertight check runs at every deflection in the mesher's ladder; a gap fails →
  discard → OCCT.
- **Volume-oracle tolerance.** Curved faces are deflection-bounded, not fp-exact; the
  correct-volume check uses a relative tolerance sized to the tessellation deflection
  (like the analytic box∩cylinder guard), with the Steinmetz `16 r³ / 3` analytic value
  as the host ground truth and OCCT `BRepAlgoAPI` volume as the sim ground truth.
- **Interior-sample selection.** Picking a UV point strictly inside a curved fragment
  (away from seam + trim edges) must be robust for annular / crescent fragments.
  Mitigation: sample the fragment's UV centroid, reject-and-reseed if it lands in the
  ON-band or outside the trimmed region; persistent failure declines → OCCT.
