# native-booleans

## ADDED Requirements

### Requirement: Freeform operand descriptor + recogniseFreeformSolid admission gate (admit one reachable freeform operand or DECLINE)

The native boolean library SHALL provide an OCCT-free, header-only freeform operand
DESCRIPTOR and admission gate (in `src/native/boolean/freeform_operand.h`) — an
**additive sibling** to the analytic `recogniseCurvedSolid` (`ssi_boolean.h`), which
SHALL remain byte-identical. The descriptor SHALL be a data model of a
freeform-faced solid — a `FreeformOperand` carrying, for each boundary face, its role
(`Freeform` for a `Kind::BSpline`/`Kind::Bezier` wall; `AnalyticHalfSpace` for a
`Plane` — or `Cylinder`/`Sphere`/`Cone` — cap/half-space), its WORLD-PLACED
`FaceSurface` (surface + face location folded, as `worldFrame` does), its genuinely
trimmed outer `EDGE_LOOP`, and its orientation-resolved outward normal; plus the
operand's world-space axis-aligned bounding box, the indices of the freeform walls
and of the analytic half-spaces, a closed-2-manifold/watertight flag, and the operand
`Shape` itself. The descriptor SHALL expose EXACTLY what the M2 assembly reads: the
freeform `Face`(s) for the B2 face-split, the analytic half-spaces, the world AABB for
the B3 membership ON-band, and the operand `Shape` for the M0 `SolidMesher`.

`recogniseFreeformSolid(const topology::Shape&)` SHALL return
`std::optional<FreeformOperand>` and SHALL ADMIT one reachable freeform operand: a
non-null `Solid` with a single shell, at least ONE genuinely-trimmed freeform face (a
real pcurve-bounded `EDGE_LOOP` that is NOT the full parametric rectangle and carries
no inner hole loop), every remaining face an admissible analytic cap/half-space, whose
boundary is CLOSED and WATERTIGHT (every edge shared by EXACTLY two faces) and whose
faces round-trip faithfully (each recorded `surface`/kind/trim equals the face's own).
It SHALL return `std::nullopt` — an HONEST DECLINE carrying the measured blocker — for
a non-`Solid`, an open or leaky boundary, a multi-shell operand, a freeform face that
is bare-periodic (a `VERTEX_LOOP` — the analytic/structured paths own it) or holed, a
non-admissible surface kind (e.g. `Torus`), or an operand with NO freeform face (the
analytic `recogniseCurvedSolid` and planar paths own those).

The descriptor and gate SHALL be strictly ADDITIVE: they SHALL NOT modify
`recogniseCurvedSolid` or `classifyPoint` (`ssi_boolean.h`), SHALL NOT modify the
consumed M0/M1/B2/B3 subsystems, SHALL introduce NO `cc_*` ABI surface or POD-layout
change, and SHALL add ZERO OCCT includes under `src/native/**`. No tolerance SHALL be
weakened to admit an operand, and a non-admissible operand SHALL DECLINE (`nullopt`)
rather than be forced through. This first slice targets the SIMPLEST reachable
operand — a single genuinely-trimmed freeform-wall solid with planar caps; richer
operands (multi-freeform-face, holed, freeform↔freeform) are explicitly deferred.

#### Scenario: A single-freeform-wall solid with planar caps is admitted and round-trips its faces/kinds/trims (host, analytic — no OCCT)

- GIVEN a native `Solid` whose boundary is one genuinely-trimmed `Kind::BSpline` wall (a real pcurve-bounded `EDGE_LOOP`) plus planar cap faces, with a KNOWN closed-form volume/area, built on the host with NO OCCT linked
- WHEN `recogniseFreeformSolid` is evaluated on it
- THEN it SHALL return a `FreeformOperand` in which every `OperandFace`'s world-placed `surface`/kind and trimmed outer loop equal the face's own within tolerance, the freeform wall is tagged `Freeform` and the caps `AnalyticHalfSpace`, each outward normal respects the face `Orientation`, the world AABB is tight, and the watertight flag is true — the descriptor faithfully captures the freeform solid

#### Scenario: A non-admissible operand DECLINES to nullopt with a measured blocker (host)

- GIVEN an operand outside the first-slice envelope — a non-`Solid`, an open/leaky boundary (an edge NOT shared by exactly two faces), a multi-shell solid, a bare-periodic (`VERTEX_LOOP`) or holed freeform face, a `Torus` face, or a purely-analytic solid with no freeform face — built on the host with NO OCCT
- WHEN `recogniseFreeformSolid` is evaluated on it
- THEN it SHALL return `std::nullopt` (an honest decline) identifying the measured blocker (the reason the caller logs before falling through to OCCT), and SHALL NOT fabricate a descriptor for an inadmissible operand

#### Scenario: The descriptor exposes exactly the handles the M2 verbs consume (host)

- GIVEN a `FreeformOperand` admitted by `recogniseFreeformSolid`
- WHEN the M2 assembly reads it
- THEN the freeform-wall `Face` it exposes SHALL be directly acceptable to B2 `splitFace`, the operand `Shape` it exposes SHALL be directly meshable by the M0 `SolidMesher::mesh`, and the world AABB it exposes SHALL be directly usable as the B3 `classifyPointInMesh` bbox — the descriptor is the single place the assembly reads faces/kinds/surfaces/trims/bounds/orientation

### Requirement: The freeform operand descriptor is strictly additive and preserves the analytic recognise path byte-identical

Adding the `FreeformOperand` descriptor and `recogniseFreeformSolid` gate SHALL leave
the analytic curved-boolean substrate byte-identical: the analytic
`recogniseCurvedSolid`, `classifyPoint`, and the S5-a curved-solid assembler
(`ssi_boolean.h`) SHALL be UNCHANGED, and the consumed M0 mesher
(`tessellate/face_mesher.h trimmedFreeformMesh`), M1 tracer (`ssi/marching.h WLine`),
B2 face-split (`boolean/face_split.h splitFace`), and B3 membership
(`boolean/freeform_membership.h classifyPointInMesh`) SHALL be consumed WITHOUT
modification. The addition SHALL introduce no `cc_*` ABI change and SHALL keep
`src/native/**` free of OCCT includes. The existing curved-boolean and freeform
substrate test suites SHALL pass with counts unchanged from the pre-change baseline.

#### Scenario: The analytic recogniser and the M2 substrate are byte-identical after the descriptor is added (host + sim)

- GIVEN the analytic `recogniseCurvedSolid`/`classifyPoint` and the M0/M1/B2/B3 subsystems before and after `freeform_operand.h` is added, together with the curved-boolean, face-split, freeform-membership, and SSI-marching suites
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN `recogniseCurvedSolid`/`classifyPoint` and the four consumed subsystems SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

### Requirement: Minimal end-to-end freeform↔analytic boolean assembly with mandatory self-verify, or HONEST DECLINE

As a stretch beyond B1, the native boolean library SHALL ATTEMPT a minimal end-to-end
freeform boolean for the SIMPLEST reachable case — a single freeform-wall operand
(admitted by `recogniseFreeformSolid`) CUT by, or COMMON with, ONE analytic PLANAR
half-space — composed ENTIRELY from the landed M2 verbs consumed unchanged: recognise
the operand[B1], trace the wall∩plane seam[M1 `WLine`], split the freeform wall along
the seam[B2 `splitFace`], classify each resulting fragment IN/OUT at its interior
point against the other operand's M0 boundary mesh[B3 `classifyPointInMesh`], select
the surviving fragments per the op's face-survival rule, and weld them watertight[M0
shared-edge weld]. Before returning, the assembler SHALL run the MANDATORY,
OCCT-free self-verify gate — the welded result SHALL be WATERTIGHT (every edge shared
by exactly two faces) AND its enclosed volume SHALL match the closed-form/set-algebra
value within a scale-relative tolerance — and SHALL DISCARD a result that fails
(returning a NULL `Shape` → OCCT).

The assembler SHALL DECLINE (return a NULL `Shape` → OCCT) whenever any verb declines
— `recogniseFreeformSolid` returns `nullopt`, the M1 seam is not one clean chord, B2
`splitFace` returns a `SplitDecline`, or a fragment's B3 verdict is `On`/`Unknown`
(near the ON band or an ambiguous ray consensus) — and SHALL NEVER emit a partial,
overlapping, leaky, or wrong-volume solid, nor guess a fragment's membership. If the
minimal assembly is NOT robustly reachable, this change SHALL LAND B1 (the descriptor
+ gate) and HONEST-DECLINE the assembly, recording which verb declined and the
measured remaining gap. No tolerance SHALL be weakened to force an assembly; OCCT
SHALL remain the oracle and the fallback.

#### Scenario: A single-freeform-face plate CUT by a planar half-space assembles watertight with the correct volume (host, analytic — no OCCT)

- GIVEN a single-freeform-wall operand admitted by `recogniseFreeformSolid` and one analytic planar half-space whose plane cleanly crosses the freeform wall, on a host build with NO OCCT, such that the cut region's volume is KNOWN in closed form
- WHEN the minimal assembler traces the seam[M1], splits the wall[B2], classifies the fragments[B3], welds the survivors[M0], and runs the mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces) whose enclosed volume equals the closed-form CUT value within the scale-relative tolerance — the four verbs compose into a correct freeform boolean

#### Scenario: A minimal assembly that cannot be completed robustly DECLINES to OCCT, and B1 still lands (host)

- GIVEN a freeform↔analytic case in which a verb declines (no clean single-chord seam, a B2 `SplitDecline`, or a fragment centroid that B3 resolves to `On`/`Unknown`) or the welded result fails the watertight/volume self-verify
- WHEN the minimal assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave B1 (the descriptor + gate) landed and the consumed subsystems byte-identical — the honest decline is a first-class outcome

### Requirement: Minimal freeform↔analytic boolean parity with OCCT BRepAlgoAPI_{Cut,Common} (simulator)

When the minimal freeform↔analytic assembly assembles, it SHALL, on a booted iOS
simulator with OCCT linked, match the OCCT oracle: the native result's enclosed
volume and surface area SHALL match `BRepAlgoAPI_Cut` / `BRepAlgoAPI_Common` (measured
via `BRepGProp`) within a scale-relative tolerance, the native result SHALL be
watertight, and a batch of query points SHALL agree with `BRepClass3d_SolidClassifier`
on the native result with ZERO crisp IN↔OUT disagreements. The engine's mandatory
watertight + volume self-verify SHALL DISCARD any native result that is not watertight
or whose volume does not match the OCCT oracle, falling through to OCCT so that a
wrong/leaky solid is NEVER emitted. OCCT SHALL be referenced ONLY in the simulator
proof harness (`src/engine/occt`); the assembler and its inputs SHALL remain
OCCT-free.

#### Scenario: The native freeform CUT/COMMON matches BRepAlgoAPI within tolerance (sim, parity)

- GIVEN the freeform↔analytic-plane operands built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked, for `op ∈ {Cut, Common}`
- WHEN the native minimal assembler produces a result and it is compared against `BRepAlgoAPI_{Cut,Common}` via `BRepGProp` (volume/area) and against `BRepClass3d_SolidClassifier` on a point batch
- THEN the native volume/area SHALL match the OCCT oracle within the scale-relative tolerance, the native result SHALL be watertight, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume native result is discarded to OCCT (sim)

- GIVEN a native minimal-assembly result that is not watertight or whose volume does not match the OCCT oracle within tolerance
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_{Cut,Common}` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path
