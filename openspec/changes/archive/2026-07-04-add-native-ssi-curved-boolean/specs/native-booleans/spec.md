# native-booleans

Extend the native planar-polyhedron + analytic box∩cylinder boolean with an
**SSI-curve-driven curved boolean** for **transversal elementary curved pairs** (SSI
Stage **S5-a**, `openspec/SSI-ROADMAP.md`). The new path consumes the S3 `ssi::TraceSet`
— one `WLine` per transversal branch, each node carrying `(u1,v1,u2,v2)` on both
surfaces and a fitted B-spline — and computes `fuse` / `cut` / `common` by **splitting**
each curved face along its `WLine`, **classifying** fragments inside/outside the other
solid with a curved point-in-solid test, and **welding** the survivors into a watertight
curved-faced `Solid`. It is a SIBLING to the planar BSP-CSG and the hand-matched analytic
`curved.h` — driven by the SSI curves, not by per-primitive matching. The engine's
mandatory watertight + correct-volume self-verify guards it and DISCARDS a bad candidate
→ OCCT `BRepAlgoAPI`. Near-tangent (`nearTangentGaps > 0`), coincident, branch-point,
and freeform pairs are DEFERRED to S4 + OCCT fallback, reported not faked. Internal:
**no `cc_*` ABI change** — invoked behind the existing `cc_boolean` op codes.

## ADDED Requirements

### Requirement: SSI-curve-driven curved boolean for transversal elementary pairs

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse
(`A ∪ B`), `op = 1` cut (`A − B`), `op = 2` common (`A ∩ B`) — NATIVELY for
**transversal elementary curved pairs** (a pair where at least one operand has a
Cylinder / Sphere / Cone face, outside the analytic axis-aligned box∩cylinder family,
whose S3 intersection trace is fully transversal) by consuming the native, OCCT-free S3
`cybercad::native::ssi` `TraceSet`. The builder SHALL be a SIBLING path to the planar
BSP-CSG and the analytic `curved.h`, invoked through the same `boolean_solid` entry
behind the `cc_boolean` op codes, and SHALL:

- **Gate.** For each pair of intersecting curved faces, build the two `SurfaceAdapter`s
  and obtain the S3 `TraceSet`; PROCEED only when the trace is fully transversal
  (`nearTangentGaps == 0` and every consumed `WLine.status` is `Closed` or
  `BoundaryExit`). Otherwise return a NULL `Shape` (→ OCCT fallback).
- **Split.** Cut each curved face along its `WLine` using the WLine's per-node `(u,v)`
  track on that face as the split polyline in the face's UV domain (the curved analogue
  of the planar `splitPolygon`), partitioning the trimmed face into fragments; the shared
  **seam edge** SHALL take its 3D geometry from the WLine's fitted B-spline and its
  pcurve on each face from that face's `(u,v)` track, and each fragment SHALL retain its
  parent face's exact surface kind (Cylinder / Sphere / Cone / Plane) — nothing faceted.
- **Classify.** Tag each fragment INSIDE / OUTSIDE / ON the OTHER solid via a curved
  point-in-solid test at an interior UV sample of the fragment (the planar
  side-of-boundary classification idea generalized to curved faces), and select the
  surviving fragments per the op's face-survival rule — the SAME set algebra as the
  planar path (fuse: `A` outside `B` + `B` outside `A`; cut: `A` outside `B` + `B` inside
  `A` reversed; common: `A` inside `B` + `B` inside `A`), oriented outward. A sample that
  is robustly ON the other solid (coincident / tangent) SHALL abort the native path
  (NULL → OCCT), never a guessed side.
- **Weld.** Sew the surviving curved + planar fragments into one closed `Solid`, welding
  coincident corners to shared vertices and sharing the WLine seam edge between exactly
  the two fragments it split (one from each operand), so the two faces meet watertight
  along the curved seam (the tessellator's curved-seam weld).

The result SHALL be a native `topology::Shape` of type `Solid` carrying true curved face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
the exact set-algebra value for the op within a relative tolerance sized to the
curved-face tessellation deflection. The builder SHALL remain OCCT-free and reference no
OCCT / `IEngine` / `EngineShape` type, and — because it consumes the S3 tracer — its
SSI-driven entry point SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry
point, signature, or POD struct SHALL be added or changed.

#### Scenario: Equal-radius crossing cylinders common is the Steinmetz solid (host)
- GIVEN two equal-radius cylinders of radius `r` crossing at right angles, built as
  native curved solids on the host with no OCCT, and their S3 `TraceSet` (fully
  transversal)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated by
  `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) with every
  edge shared by exactly two faces AND its enclosed volume SHALL equal the analytic
  Steinmetz value `16 · r³ / 3` within the curved-face deflection tolerance

#### Scenario: A curved cut / fuse is watertight with the correct set-algebra volume (host)
- GIVEN a transversal elementary curved pair (e.g. sphere∩box or cone∩box) built as
  native curved solids on the host with no OCCT, and its fully-transversal S3 `TraceSet`
- WHEN the corresponding `cc_boolean` op (cut or fuse) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` with every seam node on
  BOTH surfaces within tolerance AND its enclosed volume SHALL have the correct
  set-algebra sign and magnitude for the op within the curved-face deflection tolerance

#### Scenario: The split is driven by the WLine, not by per-primitive matching (host)
- GIVEN two distinct transversal elementary curved pairs (e.g. cyl∩cyl and sphere∩box)
  and their S3 `TraceSet`s
- WHEN each pair's curved faces are split along their `WLine` `(u,v)` tracks
- THEN the SAME split → classify → weld driver SHALL handle both pairs by consuming their
  WLines, with no hand-matched per-primitive result builder for either pair

#### Scenario: A verified native curved boolean is read back by the native paths (host)
- GIVEN a native SSI-driven curved boolean result that PASSES the engine self-verify
  (watertight 2-manifold with the correct set-algebra volume)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no fallback call

### Requirement: SSI curved boolean self-verify guard (discard and fall through)

The engine SHALL accept a native SSI-driven curved boolean result as native ONLY when it
PASSES the mandatory self-verify: the candidate SHALL be (a) a **closed watertight
2-manifold** (closed at every deflection in the mesher's deflection ladder, positive
enclosed volume, every edge shared by exactly two faces) AND (b) have the **correct
set-algebra volume sign and magnitude** for the op — `Vr ≈ |A| + |B| − |A ∩ B|` (fuse),
`Vr ≈ |A| − |A ∩ B|` (cut), or `Vr ≈ |A ∩ B|` (common) — within a RELATIVE tolerance
sized to the curved-face tessellation deflection (curved faces are deflection-bounded,
not fp-exact), using the operands' native volumes and their native intersection volume
(the Steinmetz `16 r³ / 3` for the equal-cyl common serves as the host analytic oracle).
If EITHER check fails, the engine SHALL **DISCARD** the native result. The engine SHALL
NEVER emit an unverified, leaky, or wrong SSI-driven curved boolean; when either operand
is OCCT-built the discarded case SHALL fall through to OCCT `BRepAlgoAPI`, and when both
operands are native voids OCCT cannot read, the engine SHALL report an honest error.
The self-verify guard SHALL live in the engine (next to the OCCT fallback), not in the
OCCT-free builder library.

#### Scenario: A bad native SSI curved boolean result is discarded (host)
- GIVEN a native SSI-driven curved boolean candidate that is open / non-manifold OR whose
  enclosed volume is outside the deflection-sized band for its op, built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; a case with an OCCT operand
  falls through to OCCT)

#### Scenario: A verified equal-cyl common passes the Steinmetz analytic oracle (host)
- GIVEN a native equal-radius right-angle cylinder∩cylinder `common` candidate whose
  enclosed volume matches the analytic `16 · r³ / 3` within the deflection band, built on
  the host
- WHEN the self-verify guard is applied
- THEN the analytic-volume oracle SHALL accept the candidate AND it SHALL be served
  natively with no fallback call

### Requirement: Near-tangent, coincident, branch-point, and freeform curved pairs fall through to OCCT

The SSI-driven curved boolean builder SHALL DECLINE (return a NULL `Shape`) for any case
outside the transversal-elementary slice: (1) the S3 `TraceSet` for an intersecting face
pair reports **`nearTangentGaps > 0`**, or any consumed `WLine.status` is `NearTangent`
or `Failed` (the branch was traced only up to a tangent — the S4 seam); (2) a fragment's
interior sample lands **ON** the other solid within tolerance (a **coincident /
tangent-face** configuration — no robust inside/outside); (3) **coincident /
overlapping** curved faces (no discrete transversal seam to split); (4) a **branch-point
/ self-intersecting** seam (a singular multi-branch crossing); (5) a **freeform** (NURBS
/ Bézier) operand face (S5-a is elementary-only); (6) either operand is not a native
body. When either operand is an OCCT body, each such case SHALL produce EXACTLY the
fallback (OCCT `BRepAlgoAPI`) engine's result; when both operands are native voids OCCT
cannot read, the engine SHALL report an honest error. The change SHALL NOT fake,
stub-out, hand-tune, or partially implement any deferred case; each SHALL be labelled and
verified as a fall-through, never faked. `nearTangentGaps > 0` SHALL be treated as the
honest S4 fallback boundary, not consumed and not an error.

#### Scenario: A near-tangent curved pair declines to OCCT (host + parity)
- GIVEN a curved boolean whose S3 `TraceSet` reports `nearTangentGaps > 0` (e.g. tangent
  cylinders / a sphere tangent to a box face), with the native engine active
  (`cc_set_engine(1)`)
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` AND (with an OCCT operand) the
  result SHALL be identical to invoking the same call with the OCCT engine active
  (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A coincident / branch-point curved pair declines (host)
- GIVEN a curved configuration with coincident / overlapping faces or a branch-point /
  self-intersecting seam (no single transversal seam per face), with the native engine
  active
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` (rather than emit a wrong
  classification) AND the engine SHALL NOT emit a native result for that call

#### Scenario: A freeform or non-native operand falls through (host)
- GIVEN a curved boolean where an operand has a NURBS / Bézier face, OR an operand is not
  a native body, with the native engine active
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` AND fall through to the fallback
  engine for that call, identical to `cc_set_engine(0)`

### Requirement: SSI curved boolean parity with OCCT through the facade (simulator gate)

The SSI-driven curved boolean's fidelity SHALL be reported as a **measured native-vs-OCCT
parity** against `BRepAlgoAPI_{Fuse,Cut,Common}` on the simulator — **volume**,
**surface area**, **watertightness** (closed shell), and **shape validity** (`BRepCheck`)
— on transversal elementary pairs (cylinder∩cylinder including the Steinmetz common,
sphere∩box, cone∩box), rather than asserted to be a perfect result. The harness SHALL be
modelled on the S3 marching sim harness (`scripts/run-sim-native-ssi-marching.sh` +
`tests/sim/native_ssi_marching_parity.mm`), built against the SDK sysroot, linked with
the numsci IOSSIM substrate and the OCCT-for-iOS libs, and run via `xcrun simctl spawn`
on a booted simulator. The count of pairs deferred to OCCT (near-tangent / coincident /
freeform) SHALL be **reported** (the S4 seam), not hidden or padded, and whatever S5-a
cannot compute SHALL fall back to OCCT and be reported with the measured gap.

#### Scenario: native-vs-OCCT curved boolean parity is reported per pair on the simulator
- GIVEN a transversal elementary curved pair built both as native curved solids and as
  OCCT `BRepPrimAPI` solids
- WHEN the native SSI-driven boolean and OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` each compute
  the op on the simulator
- THEN the harness SHALL report the native vs OCCT volume delta, surface-area delta,
  watertight/closed-shell status, and shape validity within tolerance, compared at the
  `cybercad::native::boolean` C++ boundary
- AND no `cc_*` entry point SHALL have been added, and the count of pairs deferred to OCCT
  SHALL be reported, not hidden

#### Scenario: a deferred pair is reported as an OCCT fall-through, not faked
- GIVEN a curved pair S5-a cannot compute (near-tangent / coincident / freeform)
- WHEN parity is measured
- THEN the harness SHALL report it as deferred to OCCT (counted, with the measured gap)
  AND the native path SHALL have emitted no fabricated or hand-tuned result for it
