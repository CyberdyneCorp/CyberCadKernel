## Why

`SSI-ROADMAP.md` stages SSI analytic-first as the **enabler**; **general curved
booleans are the payoff** — the user-visible feature the whole SSI ladder exists to
unlock. S1 (closed-form conics), S2 (seeding) and S3 (marching) are **shipped**: S3
now hands back a **`TraceSet`** — one `WLine` per distinct **transversal** branch,
every node carrying its 3D point AND its `(u1,v1,u2,v2)` on **both** surfaces, with a
fitted B-spline and an honest per-curve status (Closed / BoundaryExit / NearTangent).
Those WLines are exactly the geometry a curved boolean needs to **split** a curved
face. But nothing consumes them yet: `src/native/boolean/` today does **planar
BSP-CSG** (`bsp.h`) plus a **hand-matched, analytic** box∩axis-parallel-cylinder slice
(`curved.h`) that recognises primitives and builds the answer per-primitive — it does
**not** read SSI curves, so it does not generalize to cyl∩cyl, sphere∩box, or cone∩box.

The missing capability is the **general, SSI-curve-driven** boolean path: take the S3
`TraceSet`, split the curved operand faces along each `WLine`, classify the resulting
fragments inside/outside the other solid, and weld the survivors into a watertight
shell. This change (`S5-a`) delivers exactly that slice — **transversal elementary
curved pairs** — and wires it into the engine behind the mandatory self-verify → OCCT
fallback. It is the first stage of S5; the near-tangent / coincident / branch-point
tail (any pair for which S3 reports `nearTangentGaps > 0`) stays **deferred to S4** and
falls back to OCCT, honestly reported.

## What Changes

- Add a native, **OCCT-free**, **SSI-curve-driven** curved-boolean path to
  `src/native/boolean/` (`cybercad::native::boolean::ssi_curved`) that consumes the S3
  `ssi::TraceSet` for each intersecting curved face pair and computes `fuse` / `cut` /
  `common` for **transversal elementary curved pairs** (cyl∩cyl, sphere∩box,
  cone∩box, cyl∩box general orientation). It is a **new path alongside** the planar
  BSP-CSG and the hand-matched analytic `curved.h` — it does **not** replace them and
  does **not** add more hand-matched primitives; the split is **driven by the WLines**,
  so it generalizes across the elementary-face family.
- **Split.** For each curved face crossed by a `WLine`, cut the face along the curve's
  on-surface parameter track `(u,v)` into fragments (a curved analogue of `bsp.h`'s
  `splitPolygon`: the WLine's per-node surface params are the split polyline in that
  face's UV domain; the WLine's fitted B-spline is the shared **seam edge** geometry).
- **Classify.** Tag each fragment INSIDE / OUTSIDE / ON the **other** solid with a
  **curved point-in-solid** test at the fragment's interior sample — reusing the
  `bsp.h` ray/clip classification **idea** generalized to curved faces (evaluate the
  face at an interior `(u,v)`, test containment against the other operand's exact
  faces). Select survivors per the op's face-survival rule (same set-algebra mapping as
  `booleanPolygons`: fuse = outside∪outside; cut = outside(A)∪inside(B)-reversed;
  common = inside∩inside).
- **Weld.** Sew the surviving curved + planar fragments into one closed watertight
  `Solid`, sharing the WLine seam edge between the two faces it splits — extending
  `assemble.h`'s weld-coincident-corners → shared-vertex → face → shell → Solid to
  carry curved faces and the curved seam (the same curved-seam weld the tessellator
  uses). The result carries **true** curved face kinds (Cylinder / Sphere / Cone /
  Plane) so it tessellates watertight by the identical mesher path.
- **Engine opt-in with self-verify → OCCT fallback.** `boolean_solid` tries the SSI
  path when an operand has a curved face outside the analytic `curved.h` family and the
  S3 trace is fully transversal (`nearTangentGaps == 0`); the ENGINE
  (`native_engine.cpp`) runs the mandatory **watertight + correct-volume** self-verify
  and **DISCARDS** a bad native result, falling through to the OCCT `BRepAlgoAPI` oracle.
  Any pair S3 cannot trace transversally (`nearTangentGaps > 0`), or that fails
  self-verify, returns NULL from native → OCCT (labelled, verified, never faked).
- **Honest scope.** S5-a covers **transversal elementary** curved pairs only.
  Near-tangent / coincident / branch-point pairs (`nearTangentGaps > 0`), freeform
  (NURBS/Bézier) faces, and multi-branch / self-intersecting seams are **DEFERRED to
  S4 + OCCT fallback** — reported with the measured gap, never faked or hand-tuned.

**No `cc_*` ABI change.** The path is invoked behind the existing `cc_boolean` op
codes (`0` fuse / `1` cut / `2` common) through the same `boolean_solid` entry the
planar/analytic paths already use. `src/native/**` stays OCCT-free; the SSI-driven path
depends on `native-ssi` / `native-numerics` / `native-math`, so it is compiled under
`CYBERCAD_HAS_NUMSCI` (like the S3 tracer). Additive only.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-booleans capability (planar +
analytic box∩cylinder) with an SSI-curve-driven curved path, and EXTENDS native-ssi by
declaring the TraceSet as the consumed S5 curved-boolean input contract. -->

### Modified Capabilities
- `native-booleans`: extend the planar-polyhedron + analytic box∩cylinder boolean with
  an **SSI-curve-driven curved boolean** for **transversal elementary curved pairs** —
  split each curved face along the S3 `WLine` (its `(u,v)` track + fitted B-spline
  seam), classify fragments with a curved point-in-solid test, weld the survivors into
  a watertight curved-faced `Solid` with the correct set-algebra volume/area. The
  engine's mandatory watertight + correct-volume self-verify guards it and DISCARDS a
  bad candidate → OCCT `BRepAlgoAPI`. Near-tangent (`nearTangentGaps > 0`), freeform,
  and degenerate pairs are DEFERRED to S4 + OCCT fallback, reported not faked. No
  `cc_*` change.
- `native-ssi`: declare the S3 `TraceSet` (WLines with `(u1,v1,u2,v2)` per node + fitted
  B-spline + per-curve status) as the **consumed input contract for S5-a curved
  booleans** — each transversal `WLine` splits a curved face — and confirm
  `nearTangentGaps > 0` as the honest S4 hand-off boundary the boolean path respects
  (falls back to OCCT rather than consume a non-transversal trace).

## Impact

- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry
  point, signature, or POD struct changes. Additive only.
- **Build**: adds the SSI-curve-driven path to `src/native/boolean/` (e.g.
  `ssi_curved.h` — split + curved point-in-solid classify + curved-seam weld), reusing
  `bsp.h` (classification idea), `assemble.h` (weld/shell/solid), `native-ssi`
  (`TraceSet`), `native-math` (surface eval + B-spline) and `native-numerics`
  (closest-point / containment). Because it consumes the S3 tracer it is compiled under
  **`CYBERCAD_HAS_NUMSCI`**, gated the same way `native-ssi` and `native-numerics` are.
  `src/native/**` stays OCCT-free.
- **Verification**: two gates. **Host (no OCCT)** — an analytic oracle where one
  exists: two equal-radius cylinders crossing at right angles → the **Steinmetz solid**,
  `common` volume `16 r³ / 3` (and the through-`cut` its complement in the box); plus
  watertightness (`boundaryEdgeCount == 0`, every edge shared by exactly two faces),
  correct volume **sign**, and every seam node on both surfaces ≤ tol. **Sim
  native-vs-OCCT** — `BRepAlgoAPI_{Fuse,Cut,Common}` parity (volume, surface area,
  watertight closed shell, shape validity) on cyl∩cyl, sphere∩box, cone∩box, modelled on
  `scripts/run-sim-native-ssi-marching.sh` + a new `native_ssi_curved_boolean_parity.mm`
  harness on a booted iOS simulator. Whatever S5-a cannot handle falls back to OCCT and
  is reported with the measured gap.
- **Roadmap**: implements `SSI-ROADMAP.md` **S5** (the payoff) as the transversal
  elementary slice **S5-a**; consumes the shipped S3 `TraceSet`. Unlocks curved blends
  (#6) and curved wrap-emboss (#7) to compose on top. S4 robustness (near-tangent moat)
  remains the long tail and the fallback boundary.
- **Risk (honest)**: the near-tangent / coincident / branch-point moat — any pair S3
  reports `nearTangentGaps > 0` is NOT consumed; native returns NULL → OCCT, reported.
  A curved fragment whose interior sample sits near-ON the other solid can misclassify —
  the mandatory watertight + correct-volume self-verify in the engine DISCARDS a bad
  weld → OCCT, so a misclassification never ships as a leaky/wrong solid. The
  curved-seam weld can leave a hairline gap on a high-curvature seam — caught by the
  watertight check at the mesher's deflection ladder → discard → OCCT. Whatever S5-a
  cannot robustly compute falls back to OCCT and is reported with the measured gap; no
  case is faked, stubbed, or hand-tuned to pass.
