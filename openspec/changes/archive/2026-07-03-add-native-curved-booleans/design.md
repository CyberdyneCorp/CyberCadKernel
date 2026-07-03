# Design — add-native-curved-booleans

Narrow curved slice of Phase 4 #5 (`native-booleans`). Extends the archived planar
BSP-CSG boolean to ONE analytic curved family: axis-aligned box ↔ cylinder (cylinder axis
parallel to a box axis). Clean-room from computational-geometry references + the
`cc_boolean` contract; OCCT (`IntTools_FaceFace`, `GeomInt_IntSS`, `IntAna_QuadQuadGeo`,
`BRepAlgoAPI_*`) as reference ORACLE only.

## 1. Why the axis-aligned box-cylinder case is tractable (and general curved is not)

A boolean over arbitrary curved solids needs the **surface-surface intersection curve** of
two arbitrary surfaces — a general spatial curve with no closed form, plus robust
near-tangent classification. That is the research-grade BOPAlgo wall and is OUT OF SCOPE.

For an axis-aligned box against a cylinder whose axis is parallel to a box axis, every
intersection is CLOSED-FORM. Put the cylinder axis along a world axis (say +Z, radius `r`,
axial span `[z0, z1]`), so its lateral surface is `x² + y² = r²`:

- A box face at **constant z** (⟂ the axis) intersects the cylinder in the **CIRCLE**
  `x² + y² = r²` at that z — a `Circle` edge, and the cylinder is capped there by a
  planar disk bounded by that circle.
- A box face at **constant x or y** (∥ the axis) intersects the lateral surface where
  `x = c` (with `|c| < r`) in the two **LINE rulings** `(c, ±√(r²−c²), z)` — straight
  `Line` edges parallel to the axis.
- Point-in-cylinder is `x² + y² ≤ r²` AND `z0 ≤ z ≤ z1` — a radial + two axial
  half-spaces. Point-in-box is the usual six half-spaces.

No numerical surface-surface solver, no marching, no tangent-curve tracing. The traces are
a circle and straight lines, and the volumes are exact (`π·r²·h`). This is precisely the
case OCCT itself routes to its ANALYTIC `IntAna_QuadQuadGeo` quadric-quadric intersector
rather than the general `IntPatch` walker — confirming (oracle only) that the closed-form
circle/line trace is the right decomposition.

## 2. Domain gate — recognising the case (DECLINE ⇒ NULL ⇒ fall through)

`curved_boolean_solid(a, b, op)` accepts ONLY when, after normalisation:

- Exactly one operand is an **all-planar axis-aligned box** (reuse `isAllPlanar`; verify
  6 faces, axis-aligned normals). Call it `box`.
- The other operand is a **simple cylinder solid**: exactly one `Cylinder`-lateral face +
  two planar `Plane` caps bounded by `Circle` edges (as the native revolve / profile
  builders emit — `src/native/topology` already has `FaceSurface::Kind::Cylinder` and
  `EdgeCurve::Kind::Circle`). Call it `cyl`, with axis `a`, radius `r`, span `[t0, t1]`.
- The cylinder **axis is parallel to a box axis** AND to a world axis (within tolerance).

Any deviation returns NULL:
- a `Sphere` / `Cone` / `BSpline` / `Bezier` face → NULL (no analytic split);
- a cylinder whose axis is NOT parallel to a box axis → NULL (trace would be an ellipse);
- a second cylinder / two curved operands → NULL (surface-surface curve);
- a near-tangent / coincident config (`|c| ≈ r` ruling, cap plane ≈ cylinder end,
  coincident axis) → NULL (robustness wall).

The gate is a pure predicate; it never emits geometry for an unsupported case.

## 3. Analytic split

Split proceeds per solid, keeping curves as TRUE edges (never chord polylines in the
B-rep — chording is a tessellation-time concern bounded by deflection):

**Box faces:**
- A face ⟂ the axis whose plane z = k lies inside `[t0, t1]`: the cylinder punches a
  DISK of radius `r` centred on the axis through it. Split the box face into (outer box
  loop) with a circular hole → the face becomes a `Plane` with an outer wire + one inner
  `Circle` wire (for cut, the disk region is removed; for common it is the kept part).
- A face ∥ the axis (constant x or y) that the cylinder crosses: split along the vertical
  chord segment where the lateral surface meets the plane.

**Cylinder faces:**
- The lateral `Cylinder` face is split by each box face ∥ the axis into angular
  fragments (bounded by axial `Line` rulings) and by each box face ⟂ the axis into axial
  bands (bounded by `Circle` arcs). Each fragment stays a `Cylinder`-surface patch with a
  wire of `Circle` arcs + `Line` rulings (deflection-bounded at tessellation).
- The two planar `Plane` caps are split like the box faces where the box crosses them.
- Where a box face ⟂ the axis cuts the cylinder INSIDE its span, a NEW planar `Plane`
  disk cap (bounded by the trace `Circle`) is created so the surviving shell closes
  (e.g. the flat bottom of a blind boss, or the ends of a through-hole tunnel wall).

## 4. Classification + survival (per op)

Classify each fragment by its interior point against the OTHER solid:
- planar fragment → centroid; cylindrical patch → a point on the mid-surface;
- point-in-box (6 half-spaces), point-in-cylinder (radial + axial).

Survival rules (mirroring the planar slice, curved-aware):
- **fuse** `A ∪ B` (round boss): box fragments OUTSIDE cyl + cyl fragments OUTSIDE box.
- **cut** `A − B` (round hole, box − cyl): box fragments OUTSIDE cyl + cyl lateral
  fragments INSIDE box, REVERSED (the tunnel wall, normal pointing into the hole) + the
  planar cap disks where the cylinder enters/exits (for a blind hole).
- **common** `A ∩ B`: box fragments INSIDE cyl + cyl fragments INSIDE box.

Orient every surviving fragment outward (material side), consistent with the op.

## 5. Curved-seam heal (the watertightness crux)

The planar `assemble.h` welds coincident straight corners and repairs T-junctions on
straight edges. The curved case adds **circle / arc seams** shared by a cap disk and the
lateral patch (and **line-ruling seams** shared by a box face and a lateral patch). These
must weld watertight. The key insight, reused from the tessellator's two-stage
edge/face mesher (`edge_mesher.h` + `face_mesher.h`, `native-tessellation`): a shared
curved edge must be discretized ONCE and both incident faces pinned to that identical
discretization. So:

- The cap-disk face and the lateral patch share the SAME `Circle` edge object (same
  centre / radius / axis / parameter range) → the two-stage mesher already meshes them to
  the identical arc samples and welds them (this is exactly the cylinder cap↔side circle
  weld the tessellation slice already handles for a plain cylinder — reused here).
- A line ruling shared by a box face and a lateral patch is a straight edge → the
  existing straight-edge weld + `BoundaryAnchors` canonical seam points apply unchanged.

So the curved seam heal is mostly a REUSE of existing tessellation machinery, provided
the assembler emits SHARED curved edge objects for coincident circle / line seams. The
assembler's vertex weld is extended to also share a curved edge when two fragments carry
a coincident `Circle` / `Line` (same geometry within tolerance).

## 6. Analytic-volume self-verify (mandatory guard)

`booleanResultVerified` (in `native_engine.cpp`) today checks (a) watertight across the
deflection ladder and (b) set-algebra volume from the operands' native volumes. Both
still apply — the operand volumes are analytic (`π·r²·h` for the cylinder,
`w·d·h` for the box, `π·r²·h_overlap` for the intersection band). The check becomes:

- **cut** (through hole): `Vr ≈ boxVol − π·r²·h_through`.
- **fuse** (boss): `Vr ≈ boxVol + π·r²·h_boss − π·r²·h_overlap`.
- **common**: `Vr ≈ π·r²·h_overlap`.

The tolerance is RELATIVE and sized to the curved-face tessellation deflection (a meshed
cylinder under-measures its volume by the chord-vs-arc gap), NOT fp-exact — the curved
faces are deflection-bounded, unlike the planar slice which is fp-exact. A candidate that
is open, non-manifold, or outside this analytic band is DISCARDED.

## 7. The native-void constraint (honest fall-through, not silent forwarding)

IMPORTANT nuance in the current engine: `boolean_op` does NOT forward a native-native
boolean it cannot do to OCCT, because a native `topology::Shape` void CANNOT be read by
the OCCT unwrap (they are different representations). For two NATIVE operands the planar
path already returns an honest ERROR (not a faked or misread result) when it declines.
The curved slice preserves this exactly:

- If BOTH operands are native and the curved builder declines (sphere / cone / non-aligned
  / cyl-cyl / near-tangent) or the self-verify rejects, the engine returns the SAME honest
  error the planar decline returns (native voids OCCT cannot read → never faked, never
  misread).
- The **true OCCT fall-through** for curved cases (sphere / non-aligned / etc.) is
  exercised through the facade in the PARITY gate by building BOTH operands under the OCCT
  engine (`cc_set_engine(0)`), where OCCT owns both voids and computes the reference
  result. The native engine intercepts NONE of these — that is the fall-through proof.

This is the honest reading required by the task: only axis-aligned box-cylinder cut / fuse
/ common lands native; every other curved case is OCCT (labelled, verified rel~0, never
faked), and a native-native curved case the analytic path cannot do is reported as a clean
error rather than a wrong / leaky solid.

## 8. Complexity

The analytic trace / classification predicates are short (≤ ~10). The curved split driver
(per box face × cylinder) is a systems-band loop (~15–20, flagged) isolated in its own
header, mirroring `splitPolygon`. The seam heal reuses existing machinery. Every new
function documents its band per the project's cognitive-complexity policy.

## 9. Files

- `src/native/boolean/cylinder.h` (NEW) — cylinder-solid recognition, axis / radius /
  span extraction, point-in-cylinder, plane-cylinder circle / line trace.
- `src/native/boolean/curved_split.h` (NEW) — analytic box-cylinder split + fragment
  classification + survival selection (the systems-band driver, flagged).
- `src/native/boolean/assemble.h` (EXTENDED) — share coincident `Circle` / `Line` seam
  edges so the curved cap↔lateral seam welds watertight (reusing the two-stage mesher).
- `src/native/boolean/native_boolean.h` (EXTENDED) — `curved_boolean_solid(a, b, op)`
  entry + domain gate; `boolean_solid` tries planar then curved.
- `src/engine/native/native_engine.cpp` (EXTENDED) — `boolean_op` curved attempt;
  `booleanResultVerified` analytic-volume oracle for the box-cylinder case.
- `tests/test_native_boolean.cpp` / `tests/test_native_engine.cpp` (EXTENDED) — host
  analytic cases + facade cases.
- `tests/sim/native_curved_boolean_parity.mm` (NEW) — sim native-vs-OCCT parity, own
  `main()`, on the `run-sim-suite.sh` SKIP list.
